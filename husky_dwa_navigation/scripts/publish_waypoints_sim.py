#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import math
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry, Path
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import NavSatFix
from pyproj import Proj, transform

class WaypointNavigator:
    """Localization 기반 Waypoint Navigation 노드
    
    - 다중 위치 소스를 통한 강건한 localization
    - UTM 좌표계에서 waypoint navigation 수행
    - SUCCESS 상태 디바운싱으로 중복 처리 방지
    """
    
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=True)
        
        # Waypoints 사전 정의 (Gazebo world 절대 좌표)
        # Gazebo 좌표계: x=전진(북), y=좌우(동서)
        # datum을 받아서 UTM 절대 좌표로 변환 예정
        # self.waypoints_gazebo = [
        #     {"x": 42, "y": 0},
        #     {"x": 44, "y": -45},
        #     {"x": -15, "y": -45},
        #     {"x": -45, "y": -45},
        #     {"x": -67, "y": -45},
        #     {"x": -72, "y": -22},
        #     {"x": -67, "y": 0},
        #     {"x": -45, "y": 0},
        #     {"x": -45, "y": -45},
        #     {"x": -45, "y": -92},
        #     {"x": -41, "y": -98},
        #     {"x": -15, "y": -100},
        # ]
        # GPS 좌표로 직접 정의 (위도, 경도)
        self.waypoints_gps = [
            {"lat": 37.56664006372896, "lon": 126.97800154428735},   # Waypoint 1
            {"lat": 37.566695126757374, "lon": 126.97810334806424}   # Waypoint 2
        ]

        # UTM projection 설정 (한국 - UTM Zone 52N)
        self.utm_proj = Proj(proj='utm', zone=52, ellps='WGS84')
        self.wgs84_proj = Proj(proj='latlong', datum='WGS84')

        # GPS → UTM 변환 (절대 좌표)
        self.waypoints_utm_absolute = self.convert_gps_to_utm()

        rospy.loginfo(f"✅ {len(self.waypoints_utm_absolute)}개 GPS waypoint를 UTM으로 변환 완료")
        for i, wp in enumerate(self.waypoints_utm_absolute):
            rospy.loginfo(f"  WP{i+1}: GPS({self.waypoints_gps[i]['lat']:.6f}, {self.waypoints_gps[i]['lon']:.6f}) → UTM({wp['x']:.2f}, {wp['y']:.2f})")

        # map frame 상대 좌표 (첫 GPS를 datum으로 사용)
        self.waypoints_map = []  # 첫 GPS 수신 후 계산

        # datum 정보 (첫 GPS 위치 = map frame 원점)
        self.datum_utm_x = None
        self.datum_utm_y = None
        self.datum_received = False  # 첫 GPS 대기

        # 상태 변수
        self.current_waypoint_index = 0
        self.is_navigating = False
        
        # ✅ SUCCESS 상태 디바운싱을 위한 변수들
        self.current_goal_sent = False  # 현재 waypoint에 대한 goal 발행 여부
        self.waypoints_published = False  # waypoints 시각화 발행 여부
        self.last_success_time = rospy.Time(0)  # 마지막 SUCCESS 처리 시간
        self.success_debounce_duration = 3.0  # SUCCESS 디바운싱 시간 (3초)
        self.waypoint_reached_threshold = 10.0  # waypoint 도달 판정 거리 (10m)
        
        # ✅ 위치 소스 관리
        self.current_pose_utm = None  # UTM 절대 위치 정보
        self.pose_source = "none"     # 현재 사용 중인 위치 소스
        self.pose_last_received = rospy.Time(0)  # 마지막 위치 정보 수신 시간
        self.pose_timeout = 5.0  # 위치 정보 타임아웃 (5초)
        
        # Publishers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.waypoints_pub = rospy.Publisher('/waypoints', String, queue_size=1)
        self.status_pub = rospy.Publisher('/waypoint_navigator/status', String, queue_size=1)

        # 시각화 Publishers
        self.waypoint_markers_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/waypoint_path', Path, queue_size=1, latch=True)
        
        # ✅ GPS 절대 위치 소스 (UTM 좌표)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_fix_callback)  # GPS 원본 데이터

        # 기타 Subscribers
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.move_base_status_callback)
        
        # ✅ 상태 모니터링용 타이머들
        rospy.Timer(rospy.Duration(2.0), self.status_monitor)
        rospy.Timer(rospy.Duration(1.0), self.pose_health_check)  # 위치 정보 상태 체크
        
        rospy.loginfo("🚀 Waypoint Navigator 시작!")
        rospy.loginfo(f"📍 총 {len(self.waypoints_gps)}개의 GPS waypoints 로드됨")
        rospy.loginfo("✅ GPS 좌표를 UTM으로 변환 완료 - 바로 시작 가능")
        rospy.loginfo("📡 위치 소스: /ublox/fix (GPS 원본 데이터)")
        rospy.loginfo("✅ 오직 move_base SUCCESS 상태에서만 다음 waypoint로 이동")
        rospy.loginfo(f"⏱️  SUCCESS 디바운싱: {self.success_debounce_duration}초")

        # 위치 정보 대기 후 네비게이션 시작
        rospy.Timer(rospy.Duration(5.0), self.delayed_start)
    
    def delayed_start(self, event):
        """위치 정보 안정화 후 네비게이션 시작"""
        if not self.datum_received:
            rospy.logwarn("⚠️  GPS datum을 아직 받지 못했습니다. 첫 GPS 대기 중...")
            return

        if self.current_pose_utm is None:
            rospy.logwarn("⚠️  위치 정보를 아직 받지 못했습니다. 위치 소스 확인 필요...")
            return

        if len(self.waypoints_map) == 0:
            rospy.logerr("❌ map frame waypoint 변환 실패!")
            return

        rospy.loginfo(f"✅ 위치 정보 안정화 완료. 네비게이션 시작!")
        rospy.loginfo(f"   위치 소스: {self.pose_source}")
        rospy.loginfo(f"   map frame waypoints: {len(self.waypoints_map)}개")
        self.start_navigation()
        event.shutdown()  # 타이머 중지
    
    def convert_gps_to_utm(self):
        """GPS 좌표를 UTM 절대 좌표로 변환"""
        utm_waypoints = []

        for wp_gps in self.waypoints_gps:
            # GPS (위도, 경도) → UTM (x, y)
            utm_x, utm_y = transform(self.wgs84_proj, self.utm_proj, wp_gps["lon"], wp_gps["lat"])
            utm_waypoints.append({"x": utm_x, "y": utm_y})

        return utm_waypoints

    def gps_fix_callback(self, msg):
        """GPS 원본 데이터를 UTM으로 변환하여 현재 위치 업데이트"""
        if msg.status.status < 0:
            rospy.logwarn_throttle(5, "⚠️ GPS 신호 불량")
            return

        try:
            # GPS (위도, 경도) → UTM (x, y) 절대 좌표
            utm_x, utm_y = transform(self.wgs84_proj, self.utm_proj, msg.longitude, msg.latitude)

            # 첫 GPS를 datum으로 설정 (map frame 원점)
            if not self.datum_received:
                self.datum_utm_x = utm_x
                self.datum_utm_y = utm_y
                self.datum_received = True

                # waypoint를 map frame 상대 좌표로 변환
                self.waypoints_map = []
                for wp_abs in self.waypoints_utm_absolute:
                    map_x = wp_abs["x"] - self.datum_utm_x
                    map_y = wp_abs["y"] - self.datum_utm_y
                    self.waypoints_map.append({"x": map_x, "y": map_y})

                rospy.loginfo(f"📍 Datum 설정: UTM ({self.datum_utm_x:.2f}, {self.datum_utm_y:.2f})")
                rospy.loginfo("🔄 Waypoint → map frame 상대 좌표 변환:")
                for i, (wp_abs, wp_map) in enumerate(zip(self.waypoints_utm_absolute, self.waypoints_map)):
                    rospy.loginfo(f"  WP{i+1}: UTM({wp_abs['x']:.2f}, {wp_abs['y']:.2f}) → map({wp_map['x']:.2f}, {wp_map['y']:.2f})")

            # 현재 위치를 map frame 상대 좌표로 저장
            map_x = utm_x - self.datum_utm_x
            map_y = utm_y - self.datum_utm_y

            self.current_pose_utm = {
                "x": map_x,
                "y": map_y,
                "z": msg.altitude,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0
            }
            self.pose_source = "GPS /ublox/fix"
            self.pose_last_received = rospy.Time.now()

            rospy.loginfo_throttle(10, f"📍 GPS map 좌표: ({map_x:.2f}, {map_y:.2f})")

        except Exception as e:
            rospy.logwarn(f"❌ GPS → UTM 변환 실패: {e}")
    
    def update_pose_utm(self, pose, source):
        """UTM 위치 정보 업데이트"""
        try:
            self.current_pose_utm = {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "qx": pose.orientation.x,
                "qy": pose.orientation.y,
                "qz": pose.orientation.z,
                "qw": pose.orientation.w
            }
            self.pose_source = source
            self.pose_last_received = rospy.Time.now()
            
            # 처음 위치 정보를 받았을 때 로그
            if self.pose_source != source:
                rospy.loginfo(f"✅ 위치 소스 전환: {source}")
                rospy.loginfo(f"   현재 위치: ({pose.position.x:.2f}, {pose.position.y:.2f})")
                
        except Exception as e:
            rospy.logwarn(f"❌ 위치 정보 업데이트 실패 ({source}): {e}")
    
    def is_pose_stale(self):
        """위치 정보가 오래되었는지 확인"""
        if self.pose_last_received == rospy.Time(0):
            return True
        
        time_diff = (rospy.Time.now() - self.pose_last_received).to_sec()
        return time_diff > self.pose_timeout
    
    def pose_health_check(self, event):
        """위치 정보 상태 체크"""
        if self.current_pose_utm is None:
            rospy.logwarn_throttle(10, "⚠️  위치 정보를 받지 못했습니다!")
            rospy.logwarn_throttle(10, "   확인 사항:")
            rospy.logwarn_throttle(10, "   1. /odometry/filtered 토픽 상태: rostopic echo /odometry/filtered")
            rospy.logwarn_throttle(10, "   2. ekf_localization_node 실행 상태 확인")
        elif self.is_pose_stale():
            rospy.logwarn_throttle(10, f"⚠️  위치 정보가 {self.pose_timeout}초 이상 업데이트되지 않음 (소스: {self.pose_source})")
        else:
            rospy.loginfo_throttle(30, f"✅ 위치 정보 정상 (소스: {self.pose_source})")
    
    def calculate_distance(self, pos1, pos2):
        """좌표간 거리 계산"""
        if pos1 is None or pos2 is None:
            return float('inf')
        
        try:
            distance = math.sqrt((pos1["x"] - pos2["x"])**2 + (pos1["y"] - pos2["y"])**2)
            return distance
        except Exception as e:
            rospy.logwarn(f"❌ 거리 계산 실패: {e}")
            return float('inf')
    
    
    def is_waypoint_reached(self, waypoint):
        """현재 위치에서 waypoint 도달 여부 확인"""
        if self.current_pose_utm is None:
            rospy.logdebug("⚠️  current_pose_utm이 None입니다")
            return False
        
        distance = self.calculate_distance(self.current_pose_utm, waypoint)
        is_reached = distance <= self.waypoint_reached_threshold
        
        rospy.logdebug(f"📏 거리 확인: {distance:.2f}m, 도달 여부: {is_reached}")
        return is_reached
    
    def move_base_status_callback(self, msg):
        """move_base 상태 모니터링 - SUCCESS 디바운싱 적용"""
        if not msg.status_list:
            return
            
        latest_status = msg.status_list[-1]
        current_time = rospy.Time.now()
        
        # ✅ SUCCESS 상태 처리 (디바운싱 적용)
        if latest_status.status == 3 and self.current_goal_sent:
            # 마지막 SUCCESS 처리 후 충분한 시간이 지났는지 확인
            time_since_last_success = (current_time - self.last_success_time).to_sec()
            
            if time_since_last_success < self.success_debounce_duration:
                rospy.loginfo_throttle(5, f"⏳ SUCCESS 디바운싱: {self.success_debounce_duration - time_since_last_success:.1f}초 남음")
                return
            
            # ✅ 위치 정보 유효성 확인
            if self.current_pose_utm is None:
                rospy.logwarn("⚠️  SUCCESS 수신했지만 현재 위치 정보가 없음 - SUCCESS 무시")
                return
            
            # ✅ 추가 검증: 실제로 waypoint 근처에 있는지 확인
            if self.current_waypoint_index < len(self.waypoints_utm):
                current_wp = self.waypoints_map[self.current_waypoint_index]
                
                if not self.is_waypoint_reached(current_wp):
                    distance = self.calculate_distance(self.current_pose_utm, current_wp)
                    rospy.logwarn(f"⚠️  SUCCESS 수신했지만 waypoint에서 {distance:.1f}m 떨어져 있음 (임계값: {self.waypoint_reached_threshold}m)")
                    rospy.logwarn(f"   현재 위치: ({self.current_pose_utm['x']:.2f}, {self.current_pose_utm['y']:.2f})")
                    rospy.logwarn(f"   목표 위치: ({current_wp['x']:.2f}, {current_wp['y']:.2f})")
                    rospy.logwarn("🔄 거리 검증 실패 - SUCCESS 무시")
                    return
            
            # ✅ 모든 검증 통과 - 다음 waypoint로 이동
            rospy.loginfo("🎯 move_base SUCCESS! 거리 검증 완료. 다음 waypoint로 이동...")
            self.last_success_time = current_time  # SUCCESS 처리 시간 기록
            self.move_to_next_waypoint()
        
        # ✅ Goal failed 시에는 아무것도 하지 않음 (재시도 없음, 다음 waypoint 이동 없음)
        elif latest_status.status in [4, 5] and self.current_goal_sent:
            rospy.logwarn("❌ move_base 실패. 대기 중... (다음 waypoint 이동 없음)")
            # 아무것도 하지 않음 - 사용자가 수동으로 처리하거나 다른 방법으로 해결
        
        # ✅ 기타 상태 로깅
        elif latest_status.status == 1 and self.current_goal_sent:
            rospy.loginfo_throttle(10, "🔄 move_base ACTIVE - 목적지로 이동 중...")
        elif latest_status.status == 0 and self.current_goal_sent:
            rospy.loginfo_throttle(10, "⏳ move_base PENDING - 목적지 대기 중...")
    
    def start_navigation(self):
        """Navigation 시작"""
        if len(self.waypoints_utm) == 0:
            rospy.logwarn("❌ UTM Waypoints가 설정되지 않음!")
            return
        
        if self.current_pose_utm is None:
            rospy.logwarn("❌ 위치 정보가 없어서 네비게이션을 시작할 수 없습니다!")
            return
        
        self.is_navigating = True
        self.current_waypoint_index = 0
        rospy.loginfo("🚀 UTM Waypoint Navigation 시작!")
        rospy.loginfo(f"   현재 위치: ({self.current_pose_utm['x']:.2f}, {self.current_pose_utm['y']:.2f})")
        rospy.loginfo(f"   위치 소스: {self.pose_source}")
        self.send_current_waypoint()
    
    def send_current_waypoint(self):
        """현재 waypoint를 move_base goal로 전송 (한번만 발행)"""
        if self.current_waypoint_index >= len(self.waypoints_utm):
            rospy.loginfo("🏁 모든 waypoints 완주!")
            self.is_navigating = False
            return
    
        # ✅ 이미 현재 waypoint에 대한 goal을 발행했다면 skip
        if self.current_goal_sent:
            rospy.loginfo_throttle(10, f"⏳ Waypoint {self.current_waypoint_index + 1} 이미 발행됨. move_base SUCCESS 대기 중...")
            return
    
        current_wp = self.waypoints_map[self.current_waypoint_index]
    
        # ✅ 순수 map 절대좌표로 목표점 생성
        goal = PoseStamped()
        goal.header.frame_id = "map"  # map 절대좌표계 (local UTM origin)
        goal.header.stamp = rospy.Time(0)  # 최신 TF 사용
    
        # ✅ UTM 절대좌표 직접 사용 (변환 없음)
        goal.pose.position.x = float(current_wp["x"])
        goal.pose.position.y = float(current_wp["y"])
        goal.pose.position.z = 0.0
    
        # ✅ 방향은 UTM 좌표계 기준으로 계산
        if self.current_waypoint_index < len(self.waypoints_utm) - 1:
            next_wp = self.waypoints_map[self.current_waypoint_index + 1]
            dx = next_wp["x"] - current_wp["x"]
            dy = next_wp["y"] - current_wp["y"]
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0  # 북향
    
        # ✅ UTM 좌표계 기준 방향 설정
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
    
        # ✅ Goal 발행 (한번만)
        self.goal_pub.publish(goal)
        self.current_goal_sent = True  # 현재 waypoint goal 발행 완료 표시
    
        # ✅ 절대좌표 보장을 위한 로깅
        rospy.loginfo(f"📍 UTM 절대좌표 Goal 전송 (한번만):")
        rospy.loginfo(f"   Waypoint: {self.current_waypoint_index + 1}/{len(self.waypoints_utm)}")
        rospy.loginfo(f"   Frame: {goal.header.frame_id}")
        rospy.loginfo(f"   Position: ({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})")
        rospy.loginfo(f"   Orientation: yaw={math.degrees(yaw):.1f}°")
        rospy.loginfo(f"   현재 위치: ({self.current_pose_utm['x']:.2f}, {self.current_pose_utm['y']:.2f})")
        rospy.loginfo(f"   ✅ move_base SUCCESS 상태에서만 다음 waypoint로 이동")
        rospy.loginfo(f"   🛡️  SUCCESS 디바운싱: {self.success_debounce_duration}초")
        rospy.loginfo(f"   📏 도달 임계값: {self.waypoint_reached_threshold}m")
    
        # ✅ 상태 발행 (절대좌표 정보 포함)
        status_msg = {
            "current_waypoint": self.current_waypoint_index + 1,
            "total_waypoints": len(self.waypoints_utm),
            "target_utm_absolute": {
                "x": float(current_wp["x"]),
                "y": float(current_wp["y"]),
                "frame": "map"
            },
            "current_pose": {
                "x": self.current_pose_utm["x"] if self.current_pose_utm else None,
                "y": self.current_pose_utm["y"] if self.current_pose_utm else None,
                "source": self.pose_source
            },
            "status": "navigating",
            "goal_sent_once": True,
            "strict_mode": True,  # SUCCESS에서만 다음 waypoint 이동
            "success_debounce_enabled": True,
            "success_debounce_duration": self.success_debounce_duration
        }
        self.status_pub.publish(String(data=json.dumps(status_msg)))
    
    def move_to_next_waypoint(self):
        """다음 waypoint로 이동 - 오직 move_base SUCCESS+거리 검증에서만 호출됨"""
        rospy.loginfo(f"✅ Waypoint {self.current_waypoint_index + 1} 완료!")
        
        self.current_waypoint_index += 1
        self.current_goal_sent = False  # ✅ 새 waypoint를 위해 goal 발행 플래그 리셋
        
        if self.current_waypoint_index >= len(self.waypoints_utm):
            rospy.loginfo("🏁 모든 waypoints 완주!")
            self.is_navigating = False
            
            # 완주 상태 발행
            status_msg = {
                "current_waypoint": len(self.waypoints_utm),
                "total_waypoints": len(self.waypoints_utm),
                "status": "completed"
            }
            self.status_pub.publish(String(data=json.dumps(status_msg)))
        else:
            rospy.loginfo(f"➡️ 다음 waypoint로 이동: {self.current_waypoint_index + 1}/{len(self.waypoints_utm)}")
            # ✅ 다음 waypoint로 이동하기 전 충분한 대기 시간
            rospy.sleep(2.0)  # 시스템이 안정화될 시간 제공
            self.send_current_waypoint()  # ✅ 새 waypoint goal 발행 (한번만)
    
    def status_monitor(self, event):
        """상태 모니터링"""
        if not self.is_navigating:
            return
        
        if self.current_pose_utm is None:
            rospy.logwarn_throttle(10, "⚠️  위치 정보 없음 - 네비게이션 대기 중...")
            return
        
        if self.current_waypoint_index >= len(self.waypoints_utm):
            return
        
        current_wp = self.waypoints_map[self.current_waypoint_index]
        
        # ✅ 단순 상태 모니터링만 (다음 waypoint 이동 없음)
        if self.current_goal_sent:
            pose_distance = self.calculate_distance(self.current_pose_utm, current_wp)
            time_since_last_success = (rospy.Time.now() - self.last_success_time).to_sec()
            
            rospy.loginfo_throttle(10, f"📍 현재 상태:")
            rospy.loginfo_throttle(10, f"   목표: Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints_utm)}")
            rospy.loginfo_throttle(10, f"   거리: {pose_distance:.2f}m (임계값: {self.waypoint_reached_threshold}m)")
            rospy.loginfo_throttle(10, f"   현재 위치: ({self.current_pose_utm['x']:.2f}, {self.current_pose_utm['y']:.2f})")
            rospy.loginfo_throttle(10, f"   목표 위치: ({current_wp['x']:.2f}, {current_wp['y']:.2f})")
            rospy.loginfo_throttle(10, f"   위치 소스: {self.pose_source}")
            rospy.loginfo_throttle(10, f"   대기: move_base SUCCESS 상태")
            rospy.loginfo_throttle(10, f"   디바운싱: 마지막 SUCCESS 후 {time_since_last_success:.1f}초 경과")
    
    def publish_waypoints_visualization(self):
        """Waypoints 시각화를 위한 데이터 발행 (한번만)"""
        if self.waypoints_published:
            return  # ✅ 이미 발행했으면 skip

        if len(self.waypoints_map) == 0:
            rospy.logwarn("⚠️ waypoints_map이 비어있어 시각화를 발행할 수 없습니다")
            return

        waypoints_data = {
            "frame": "map",  # map frame 상대좌표
            "coordinate_type": "map_relative",
            "waypoints": []
        }

        # ✅ map frame 상대좌표를 x, y 형태로 발행
        for i, wp in enumerate(self.waypoints_map):
            waypoints_data["waypoints"].append({
                "index": i,
                "x": float(wp["x"]),  # map frame 상대좌표
                "y": float(wp["y"]),  # map frame 상대좌표
                "completed": False,  # 초기에는 모두 미완료
                "is_current": i == 0  # 첫번째가 현재 목표
            })

        self.waypoints_pub.publish(String(data=json.dumps(waypoints_data)))
        self.waypoints_published = True  # ✅ 발행 완료 표시

        # ✅ RViz 시각화 발행
        self.publish_rviz_visualization()

        # ✅ 디버깅 로그
        rospy.loginfo(f"📍 map frame Waypoints 발행 완료: {len(waypoints_data['waypoints'])}개")
        rospy.loginfo(f"   좌표계: {waypoints_data['frame']} (상대좌표)")
        rospy.loginfo(f"   🎨 RViz 시각화: /waypoint_markers, /waypoint_path")

    def publish_rviz_visualization(self):
        """RViz용 Waypoint 시각화 마커 발행"""
        if len(self.waypoints_map) == 0:
            rospy.logwarn("⚠️ waypoints_map이 비어있어 RViz 시각화를 발행할 수 없습니다")
            return

        marker_array = MarkerArray()
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for i, wp in enumerate(self.waypoints_map):
            # 1. 구 마커 (Waypoint 위치)
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "waypoints"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            sphere.pose.position.x = float(wp["x"])
            sphere.pose.position.y = float(wp["y"])
            sphere.pose.position.z = 0.5  # 지면에서 0.5m 위

            sphere.pose.orientation.w = 1.0

            sphere.scale.x = 1.0  # 1m 크기
            sphere.scale.y = 1.0
            sphere.scale.z = 1.0

            # 색상: 첫 번째는 초록, 마지막은 빨강, 나머지는 파랑
            if i == 0:
                sphere.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # 초록 (시작)
            elif i == len(self.waypoints_map) - 1:
                sphere.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # 빨강 (끝)
            else:
                sphere.color = ColorRGBA(0.0, 0.5, 1.0, 0.8)  # 파랑 (중간)

            marker_array.markers.append(sphere)

            # 2. 텍스트 마커 (번호 표시)
            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = rospy.Time.now()
            text.ns = "waypoint_numbers"
            text.id = i + 1000  # ID 충돌 방지
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = float(wp["x"])
            text.pose.position.y = float(wp["y"])
            text.pose.position.z = 1.5  # 지면에서 1.5m 위

            text.text = f"WP{i+1}"
            text.scale.z = 0.8  # 텍스트 크기

            text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # 흰색

            marker_array.markers.append(text)

            # 3. Path에 추가
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = float(wp["x"])
            pose_stamped.pose.position.y = float(wp["y"])
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0

            path.poses.append(pose_stamped)

        # 4. 경로 연결 라인 (LINE_STRIP)
        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = rospy.Time.now()
        line.ns = "waypoint_path"
        line.id = 2000
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        line.scale.x = 0.2  # 라인 굵기

        line.color = ColorRGBA(1.0, 1.0, 0.0, 0.8)  # 노란색

        for wp in self.waypoints_utm:
            point = Point()
            point.x = float(wp["x"])
            point.y = float(wp["y"])
            point.z = 0.1
            line.points.append(point)

        marker_array.markers.append(line)

        # 발행
        self.waypoint_markers_pub.publish(marker_array)
        self.path_pub.publish(path)

        rospy.loginfo(f"🎨 RViz 시각화 발행 완료:")
        rospy.loginfo(f"   - Waypoint 마커: {len(self.waypoints_utm)}개 (구 + 텍스트)")
        rospy.loginfo(f"   - 경로 라인: 1개")
        rospy.loginfo(f"   - Path 메시지: {len(path.poses)}개 포즈")

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint Navigator 종료")