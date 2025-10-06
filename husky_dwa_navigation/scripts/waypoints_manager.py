#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import utm
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from actionlib_msgs.msg import GoalID

class KakaoNavigationSystem:
    """완전한 카카오 웨이포인트 자율주행 시스템
    
    - 웹에서 받은 카카오 API 웨이포인트를 UTM Local로 변환
    - 순차적으로 각 웨이포인트를 방문
    - move_base와 연동하여 실제 자율주행 수행
    """
    
    def __init__(self):
        rospy.init_node('kakao_navigation_system', anonymous=True)
        
        # 🎯 UTM Local 원점 관리 (path_visualizer.py와 동기화)
        self.utm_origin_absolute = None
        self.utm_zone = None
        self.origin_synced = False
        
        # 📍 웨이포인트 관리
        self.converted_waypoints_local = [] # UTM Local로 변환된 웨이포인트
        self.current_waypoint_index = 0     # 현재 목표 웨이포인트 인덱스
        self.is_navigating = False          # 네비게이션 상태
        self.navigation_started = False     # 네비게이션 시작 여부
        
        # 🎯 목적지 관리
        self.destination_local = None       # 최종 목적지 UTM Local 좌표
        
        # 🔄 상태 관리
        self.current_goal_sent = False
        self.last_success_time = rospy.Time(0)
        self.success_debounce_duration = 5.0    # SUCCESS 디바운스 시간 증가
        self.waypoint_reached_threshold = 10.0   # 웨이포인트 도달 임계값 (5m)
        self.goal_timeout = 60.0              # 목표 도달 타임아웃 (60초)
        self.goal_start_time = None
        
        # 🗺️ 위치 정보 (UTM Local 좌표)
        self.current_pose_local = None
        self.pose_source = "none"
        self.pose_last_received = rospy.Time(0)
        self.pose_timeout = 5.0
        self.current_gps = None
        
        # 📊 네비게이션 통계
        self.total_waypoints = 0
        self.completed_waypoints = 0
        self.failed_waypoints = 0
        
        # Publishers - navigation_manager 전용 토픽 사용
        self.goal_pub = rospy.Publisher('/waypoint_goal', PoseStamped, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)  # Goal 취소용
        self.visualization_pub = rospy.Publisher('/kakao_waypoints_viz', String, queue_size=1)  # 시각화 전용
        self.status_pub = rospy.Publisher('/kakao_navigation/status', String, queue_size=1)
        self.web_status_pub = rospy.Publisher('/kakao_navigation/web_status', String, queue_size=1)
        
        # Subscribers
        # ✅ robot_localization의 최종 결과인 /odometry/filtered를 주 위치 소스로 사용
        rospy.Subscriber("/odometry/filtered/global", Odometry, self.odometry_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.move_base_status_callback)
        
        # 🌐 기존 웨이포인트 토픽 구독 (path_visualizer에서 처리된 것)
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)
        
        # 🔄 path_visualizer와 동기화를 위한 UTM 원점 정보 구독
        rospy.Subscriber("/utm_origin_info", String, self.utm_origin_sync_callback)
        
        # Timers
        rospy.Timer(rospy.Duration(1.0), self.navigation_monitor)
        rospy.Timer(rospy.Duration(2.0), self.status_monitor)
        rospy.Timer(rospy.Duration(5.0), self.publish_web_status)
        
        rospy.loginfo("🚀 카카오 네비게이션 시스템 시작!")
        rospy.loginfo("🌍 UTM Local 좌표계 기반 자율주행")
        rospy.loginfo("📡 기존 /waypoints 토픽에서 웨이포인트 수신 대기")
        rospy.loginfo("🎯 Path Visualizer에서 처리된 웨이포인트로 자율주행!")
        
    def utm_origin_sync_callback(self, msg):
        """path_visualizer.py에서 설정한 UTM 원점 정보와 동기화"""
        try:
            # [DEBUG] 원점 동기화 메시지 수신 로그
            if self.origin_synced:
                rospy.loginfo_throttle(60, "[DEBUG] UTM 원점은 이미 동기화되었습니다. 메시지를 무시합니다.")
                return

            origin_data = json.loads(msg.data)
            rospy.loginfo("[DEBUG] UTM 원점 동기화 메시지 수신: %s", msg.data)
            
            if "utm_origin_absolute" in origin_data and not self.origin_synced:
                utm_origin = origin_data["utm_origin_absolute"]
                
                self.utm_origin_absolute = {
                    "easting": utm_origin["easting"],
                    "northing": utm_origin["northing"],
                    "lat": utm_origin["lat"],
                    "lon": utm_origin["lon"]
                }
                self.utm_zone = origin_data.get("utm_zone", "unknown")
                self.origin_synced = True
                
                # [DEBUG] 동기화된 원점 정보 상세 로그
                rospy.loginfo("="*50)
                rospy.loginfo("🎯 [SUCCESS] UTM Local 원점 동기화 완료!")
                rospy.loginfo(f"   - Zone: {self.utm_zone}")
                rospy.loginfo(f"   - Easting (X): {self.utm_origin_absolute['easting']}")
                rospy.loginfo(f"   - Northing (Y): {self.utm_origin_absolute['northing']}")
                rospy.loginfo(f"   - Lat: {self.utm_origin_absolute['lat']}, Lon: {self.utm_origin_absolute['lon']}")
                rospy.loginfo("="*50)

                # 대기 중인 웨이포인트가 있으면 처리
                if self.converted_waypoints_local:
                    rospy.loginfo("🔄 대기 중인 웨이포인트 처리 시작...")
                    if self.current_pose_local:
                        self.start_navigation()
                        
        except Exception as e:
            rospy.logerr(f"❌ [ERROR] UTM 원점 동기화 실패: {e}")
    
    def waypoints_callback(self, msg):
        """웹에서 GPS 웨이포인트 수신 및 UTM Local 변환"""
        try:
            # [DEBUG] 원본 데이터 로깅
            rospy.loginfo("[DEBUG] /waypoints 토픽 원본 데이터 수신:\n%s", msg.data)

            data = json.loads(msg.data)
            
            # 🚨 새로운 웨이포인트 수신 - 기존 네비게이션 완전 교체
            if self.is_navigating or self.navigation_started:
                rospy.logwarn("🔄 새로운 웨이포인트 수신됨 - 기존 네비게이션 완전 중단 및 교체")
                rospy.logwarn(f"   중단되는 진행: WP{self.current_waypoint_index + 1}/{self.total_waypoints} (완료: {self.completed_waypoints}개)")
                
                # move_base goal 취소 (현재 진행 중인 목표 중단)
                self.cancel_current_goal()
                
                # 모든 네비게이션 상태 완전 초기화
                self.reset_navigation_state()
                
                rospy.loginfo("✅ 기존 네비게이션 완전 중단 완료")
                rospy.loginfo("🔄 새로운 웨이포인트로 교체 시작...")
            
            # 기존 웨이포인트 데이터 완전 삭제
            self.clear_waypoint_data()
            
            # 🚨 Path Visualizer 데이터 필터링
            if "coordinate_type" in data:
                coord_type = data["coordinate_type"]
                if coord_type in ["local_relative_from_kakao", "kakao_navigation_route", "utm_local_waypoints"]:
                    rospy.loginfo_throttle(5, f"⚠️ Path Visualizer 시각화 데이터 감지 ({coord_type}) - 무시")
                    return
            
            # 🚨 frame 필터링
            if "frame" in data and data["frame"] == "map":
                rospy.loginfo_throttle(5, "⚠️ Map 프레임 데이터 감지 - 무시")
                return
            
            rospy.loginfo("📥 웹에서 GPS 웨이포인트 수신됨!")
            
            if "waypoints" in data:
                waypoints_data = data["waypoints"]
                rospy.loginfo(f"📍 총 {len(waypoints_data)}개 웨이포인트")
                
                # 🚨 GPS 좌표 검증 강화 (nested GPS 좌표 지원)
                valid_gps_count = 0
                for wp in waypoints_data:
                    if isinstance(wp, dict):
                        # 최상위 레벨에서 lat, lon 확인
                        if "lat" in wp and "lon" in wp:
                            lat, lon = wp["lat"], wp["lon"]
                            if (isinstance(lat, (int, float)) and isinstance(lon, (int, float)) and
                                abs(lat) > 0.001 and abs(lon) > 0.001 and
                                -90 <= lat <= 90 and -180 <= lon <= 180):
                                valid_gps_count += 1
                        # original_gps 안에 nested된 GPS 좌표 확인
                        elif "original_gps" in wp and isinstance(wp["original_gps"], dict):
                            gps_data = wp["original_gps"]
                            if "lat" in gps_data and "lon" in gps_data:
                                lat, lon = gps_data["lat"], gps_data["lon"]
                                if (isinstance(lat, (int, float)) and isinstance(lon, (int, float)) and
                                    abs(lat) > 0.001 and abs(lon) > 0.001 and
                                    -90 <= lat <= 90 and -180 <= lon <= 180):
                                    valid_gps_count += 1
                            
                rospy.loginfo(f"🔍 GPS 좌표 검증: {valid_gps_count}개 유효한 GPS 웨이포인트 발견 (총 {len(waypoints_data)}개 중)")
                
                # 디버깅: 첫 번째 웨이포인트 구조 출력
                if len(waypoints_data) > 0:
                    first_wp = waypoints_data[0]
                    rospy.loginfo(f"🔍 첫 번째 웨이포인트 구조: {first_wp}")
                
                if valid_gps_count == 0:
                    rospy.logwarn("⚠️ GPS 좌표가 포함된 웨이포인트가 없음 - Path Visualizer 데이터로 판단")
                    return
                
                rospy.loginfo(f"✅ {valid_gps_count}개 유효한 GPS 웨이포인트 발견")
                
                # UTM 원점이 있는지 확인
                if not self.utm_origin_absolute:
                    rospy.logwarn("❌ [CRITICAL] UTM 원점이 설정되지 않음! 웨이포인트 변환 불가. /utm_origin_info 토픽을 확인하세요.")
                    return
                
                self.converted_waypoints_local = []
                
                for i, wp in enumerate(waypoints_data):
                    # 이미 변환된 UTM Local 좌표가 있는지 확인
                    if "x" in wp and "y" in wp and "original_gps" in wp and "lat" in wp["original_gps"] and "lon" in wp["original_gps"]:
                        # gps_server.py에서 보낸 절대 UTM 좌표를 map 프레임 상대 좌표로 변환
                        map_x, map_y = self.gps_to_map_relative(wp["original_gps"]["lat"], wp["original_gps"]["lon"])
                        local_waypoint = {
                            "x": map_x,
                            "y": map_y,
                            "index": i,
                            "original_gps": wp["original_gps"]
                        }
                        self.converted_waypoints_local.append(local_waypoint)

                        # 로깅 (처음 3개와 마지막 3개만)
                        if i < 3 or i >= len(waypoints_data) - 3:
                            rospy.loginfo(f"   WP{i+1}: GPS({wp['original_gps']['lat']:.6f}, {wp['original_gps']['lon']:.6f}) → map({map_x:.1f}, {map_y:.1f})")

                    elif "lat" in wp and "lon" in wp:
                        # 원시 GPS 좌표인 경우 변환
                        map_x, map_y = self.gps_to_map_relative(wp["lat"], wp["lon"])

                        local_waypoint = {
                            "x": map_x,
                            "y": map_y,
                            "index": i,
                            "original_gps": {"lat": wp["lat"], "lon": wp["lon"]}
                        }
                        self.converted_waypoints_local.append(local_waypoint)

                        # 로깅 (처음 3개와 마지막 3개만)
                        if i < 3 or i >= len(waypoints_data) - 3:
                            rospy.loginfo(f"   WP{i+1}: GPS({wp['lat']:.6f}, {wp['lon']:.6f}) → map({map_x:.1f}, {map_y:.1f})")
                
                self.total_waypoints = len(self.converted_waypoints_local)
                
                if len(waypoints_data) > 6:
                    rospy.loginfo(f"   ... (중간 {len(waypoints_data)-6}개 웨이포인트 생략)")
                
                rospy.loginfo(f"✅ {self.total_waypoints}개 웨이포인트 map 프레임 변환 완료!")
                
                # 목적지 정보 추출 및 변환
                if "destination" in data and data["destination"]:
                    dest = data["destination"]
                    # 이미 변환된 목적지 데이터 확인
                    if "lat" in dest and "lon" in dest:
                        # 원시 GPS 좌표인 경우 변환
                        dest_x, dest_y = self.gps_to_map_relative(dest["lat"], dest["lon"])
                        self.destination_local = {
                            "x": dest_x,
                            "y": dest_y,
                            "original_gps": dest
                        }
                        rospy.loginfo(f"   🎯 목적지: GPS({dest['lat']:.6f}, {dest['lon']:.6f}) → map({dest_x:.1f}, {dest_y:.1f})")
                    elif "x" in dest and "y" in dest and "original_gps" in dest:
                        # gps_server에서 변환된 절대좌표를 map 프레임으로 변환
                        dest_x, dest_y = self.gps_to_map_relative(dest["original_gps"]["lat"], dest["original_gps"]["lon"])
                        self.destination_local = {
                            "x": dest_x,
                            "y": dest_y,
                            "original_gps": dest["original_gps"]
                        }
                        rospy.loginfo(f"   🎯 목적지: GPS({dest['original_gps']['lat']:.6f}, {dest['original_gps']['lon']:.6f}) → map({dest_x:.1f}, {dest_y:.1f})")
                    else:
                        self.destination_local = None
                        rospy.loginfo("   🎯 목적지 정보 없음")
                else:
                    self.destination_local = None
                    rospy.loginfo("   🎯 목적지 정보 없음")
                
                # 새로운 웨이포인트 교체 완료 및 네비게이션 시작
                rospy.loginfo("🎉 새로운 웨이포인트 교체 완료!")
                rospy.loginfo(f"   새로운 웨이포인트: {self.total_waypoints}개")
                rospy.loginfo(f"   목적지: {'설정됨' if self.destination_local else '없음'}")
                
                if self.total_waypoints > 0:
                    # 새로운 웨이포인트 시각화 즉시 업데이트
                    self.publish_waypoints_visualization()
                    
                    if self.current_pose_local:
                        rospy.loginfo("🚀 새로운 웨이포인트로 자율주행 시작!")
                        self.start_navigation()
                    else:
                        rospy.loginfo("⏳ 현재 위치 정보 대기 중...")
                else:
                    rospy.logwarn("❌ 유효한 웨이포인트가 없습니다!")
                    # 빈 시각화 데이터 발행 (기존 웨이포인트 시각적 제거)
                    self.publish_empty_visualization()
                    
            else:
                rospy.logwarn("⚠️ 'waypoints' 키가 없는 데이터 수신")
                
        except Exception as e:
            rospy.logerr(f"❌ 웨이포인트 파싱 오류: {e}")
            rospy.logerr(f"📋 수신된 원본 데이터 (일부): {msg.data[:100]}...")
            
    def gps_to_map_relative(self, lat, lon):
        """GPS → map 프레임 상대좌표 변환"""
        # [DEBUG] 좌표 변환 함수 진입 로그
        if not self.utm_origin_absolute:
            rospy.logwarn_throttle(5, "⚠️ [gps_to_map_relative] UTM 원점 없음. 좌표 변환 불가.")
            return 0.0, 0.0

        # GPS를 절대 UTM 좌표로 변환
        easting, northing, _, _ = utm.from_latlon(lat, lon)

        # [DEBUG] 변환 과정의 모든 값 로깅
        rospy.loginfo("[DEBUG] gps_to_map_relative 변환 과정:")
        rospy.loginfo(f"  - Input GPS: (lat: {lat:.6f}, lon: {lon:.6f})")
        rospy.loginfo(f"  - -> Absolute UTM: (easting: {easting:.2f}, northing: {northing:.2f})")
        rospy.loginfo(f"  - Map Origin UTM: (easting: {self.utm_origin_absolute['easting']:.2f}, northing: {self.utm_origin_absolute['northing']:.2f})")

        # map 프레임 원점(절대 UTM) 기준으로 상대 좌표 계산
        map_x = easting - self.utm_origin_absolute["easting"]
        map_y = northing - self.utm_origin_absolute["northing"]

        rospy.loginfo(f"  - -> Relative Map Coords: (x: {map_x:.2f}, y: {map_y:.2f})")

        return map_x, map_y
            
    def publish_waypoints_visualization(self):
        """변환된 UTM 절대좌표 웨이포인트 시각화 발행"""
        if not self.converted_waypoints_local:
            return

        waypoints_data = {
            "frame": "map",  # UTM 절대좌표계
            "coordinate_type": "kakao_navigation_route",
            "waypoints": [],
            "destination": self.destination_local,
            "total_waypoints": len(self.converted_waypoints_local),
            "current_waypoint": self.current_waypoint_index
        }
        
        for i, wp in enumerate(self.converted_waypoints_local):
            waypoint_item = {
                "index": i,
                "x": float(wp["x"]),
                "y": float(wp["y"]),
                "original_gps": wp.get("original_gps", {}),
                "completed": bool(i < self.current_waypoint_index),  # 명시적 bool() 변환
                "is_current": bool(i == self.current_waypoint_index),  # 명시적 bool() 변환
            }
            
            # 목적지 확인 로직 수정
            if self.destination_local:
                is_destination = (abs(float(wp["x"]) - float(self.destination_local["x"])) < 1.0 and 
                                abs(float(wp["y"]) - float(self.destination_local["y"])) < 1.0)
                waypoint_item["is_destination"] = bool(is_destination)  # 명시적 bool() 변환
            else:
                waypoint_item["is_destination"] = False
                
            waypoints_data["waypoints"].append(waypoint_item)
        
        self.visualization_pub.publish(String(data=json.dumps(waypoints_data)))
        rospy.loginfo(f"📍 카카오 네비게이션 경로 시각화 발행: {len(waypoints_data['waypoints'])}개")
        
    def start_navigation(self):
        """카카오 웨이포인트 자율주행 시작"""
        if not self.converted_waypoints_local:
            rospy.logwarn("❌ 변환된 웨이포인트가 없음!")
            return
            
        if self.current_pose_local is None:
            rospy.logwarn("❌ 현재 위치 정보 없음!")
            return
            
        # 네비게이션 상태 완전 초기화
        rospy.loginfo("🔄 네비게이션 상태 초기화 중...")
        self.is_navigating = True
        self.navigation_started = True 
        self.current_waypoint_index = 0  # 첫 번째 웨이포인트부터 시작
        self.current_goal_sent = False
        self.completed_waypoints = 0
        self.failed_waypoints = 0
        self.goal_start_time = None 
        self.last_success_time = rospy.Time(0)
        
        rospy.loginfo(f"✅ 상태 초기화 완료: WP인덱스={self.current_waypoint_index}, 완료={self.completed_waypoints}, 실패={self.failed_waypoints}")
        
        rospy.loginfo("🚀 순차적 웨이포인트 자율주행 시작!")
        rospy.loginfo(f"   현재 위치: UTM Local ({self.current_pose_local['x']:.2f}, {self.current_pose_local['y']:.2f})")
        rospy.loginfo(f"   총 웨이포인트: {len(self.converted_waypoints_local)}개")
        rospy.loginfo(f"   좌표계: UTM Local")
        rospy.loginfo(f"   🎯 두 번째 웨이포인트부터 navigation_manager로 순차 전송")
        rospy.loginfo(f"   🔧 navigation_manager가 장애물 회피 및 최적화 담당")
        
        # 첫 번째 웨이포인트로 이동 시작
        self.send_current_waypoint()
        
    def send_current_waypoint(self):
        """현재 웨이포인트를 navigation_manager로 순차 전송"""
        if self.current_waypoint_index >= len(self.converted_waypoints_local):
            rospy.loginfo("🏁 모든 카카오 웨이포인트 완주!")
            self.complete_navigation()
            return
            
        if self.current_goal_sent:
            rospy.loginfo_throttle(10, f"⏳ WP{self.current_waypoint_index + 1} 목표 이미 전송됨. 결과 대기 중...")
            return
            
        current_wp = self.converted_waypoints_local[self.current_waypoint_index]
        
        # UTM Local 좌표로 목표점 생성
        goal = PoseStamped()
        goal.header.frame_id = "map"  # UTM 절대좌표계
        goal.header.stamp = rospy.Time.now()
        
        goal.pose.position.x = float(current_wp["x"])
        goal.pose.position.y = float(current_wp["y"])
        goal.pose.position.z = 0.0
        
        # 방향 계산 (다음 웨이포인트 방향 또는 목적지 방향)
        if self.current_waypoint_index < len(self.converted_waypoints_local) - 1:
            next_wp = self.converted_waypoints_local[self.current_waypoint_index + 1]
            dx = next_wp["x"] - current_wp["x"]
            dy = next_wp["y"] - current_wp["y"]
            yaw = math.atan2(dy, dx)
        elif self.destination_local:
            # 마지막 웨이포인트에서는 목적지 방향
            dx = self.destination_local["x"] - current_wp["x"]
            dy = self.destination_local["y"] - current_wp["y"]
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0
            
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Goal을 navigation_manager로 전송 (단일 목표로 처리)
        self.goal_pub.publish(goal)
        self.current_goal_sent = True
        self.goal_start_time = rospy.Time.now()

        # [DEBUG] 발행되는 최종 목표 데이터 상세 로깅
        rospy.loginfo("="*50)
        rospy.loginfo("📤 [DEBUG] /waypoint_goal 토픽으로 최종 목표 발행")
        rospy.loginfo(f"  - Header: frame_id='{goal.header.frame_id}', stamp={goal.header.stamp.to_sec()}")
        rospy.loginfo(f"  - Position: (x={goal.pose.position.x}, y={goal.pose.position.y}, z={goal.pose.position.z})")
        rospy.loginfo(f"  - Orientation: (x={goal.pose.orientation.x}, y={goal.pose.orientation.y}, z={goal.pose.orientation.z}, w={goal.pose.orientation.w})")
        rospy.loginfo("="*50)
        
        # 상세 로깅
        original_gps = current_wp["original_gps"]
        distance = self.calculate_distance(self.current_pose_local, current_wp)
        
        rospy.loginfo(f"📍 웨이포인트를 navigation_manager로 전송:")
        rospy.loginfo(f"   진행: {self.current_waypoint_index + 1}/{len(self.converted_waypoints_local)}")
        rospy.loginfo(f"   GPS: ({original_gps['lat']:.6f}, {original_gps['lon']:.6f})")
        rospy.loginfo(f"   목표: Local({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})")
        rospy.loginfo(f"   거리: {distance:.1f}m")
        rospy.loginfo(f"   방향: {math.degrees(yaw):.1f}°")
        rospy.loginfo(f"   → navigation_manager가 장애물 회피 및 최적화 담당")
        
    def move_to_next_waypoint(self):
        """순차적으로 다음 웨이포인트 전송"""
        self.completed_waypoints += 1
        progress = int((self.completed_waypoints / self.total_waypoints) * 100)
        
        rospy.loginfo(f"✅ WP{self.current_waypoint_index + 1} 완료! (진행률: {progress}%)")

        self.current_waypoint_index += 1
        self.current_goal_sent = False
        
        # 웨이포인트 시각화 업데이트
        self.publish_waypoints_visualization()
        
        if self.current_waypoint_index >= len(self.converted_waypoints_local):
            # 마지막 웨이포인트면 목적지로 이동
            if self.destination_local:
                rospy.loginfo("🎯 최종 목적지로 순차 전송...")
                self.send_destination_goal()
            else:
                rospy.loginfo("🏁 모든 웨이포인트 순차 완주!")
                self.complete_navigation()
        else:
            rospy.loginfo(f"🔄 순차 진행: {self.current_waypoint_index + 1}/{len(self.converted_waypoints_local)}")
            rospy.sleep(0.5)  # navigation_manager 안정화 대기
            self.send_current_waypoint()  # 다음 웨이포인트를 navigation_manager로 순차 전송
    
    def send_destination_goal(self):
        """최종 목적지를 navigation_manager로 전송"""
        if not self.destination_local:
            self.complete_navigation()
            return
            
        goal = PoseStamped()
        goal.header.frame_id = "map"  # UTM 절대좌표계
        goal.header.stamp = rospy.Time.now()
        
        goal.pose.position.x = float(self.destination_local["x"])
        goal.pose.position.y = float(self.destination_local["y"])
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0  # 목적지에서는 방향 신경쓰지 않음
        
        # 최종 목적지를 navigation_manager로 전송
        self.goal_pub.publish(goal)
        self.current_goal_sent = True
        self.goal_start_time = rospy.Time.now()
        
        distance = self.calculate_distance(self.current_pose_local, self.destination_local)
        dest_gps = self.destination_local["original_gps"]
        
        rospy.loginfo(f"🎯 최종 목적지를 navigation_manager로 전송:")
        rospy.loginfo(f"   GPS: ({dest_gps['lat']:.6f}, {dest_gps['lon']:.6f})")
        rospy.loginfo(f"   목표: Local({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})")
        rospy.loginfo(f"   거리: {distance:.1f}m")
        rospy.loginfo(f"   → navigation_manager가 최종 목적지 최적화 담당")
    
    def complete_navigation(self):
        """순차 네비게이션 완료 및 상태 초기화"""
        rospy.loginfo("🎉 순차적 웨이포인트 자율주행 완료!")
        
        # 상태 초기화 (순서 중요!)
        self.is_navigating = False
        self.navigation_started = False
        self.current_goal_sent = False
        self.goal_start_time = None
        
        success_rate = (self.completed_waypoints / self.total_waypoints * 100) if self.total_waypoints > 0 else 0
        
        rospy.loginfo(f"   총 웨이포인트: {self.total_waypoints}개 (순차 처리)")
        rospy.loginfo(f"   완료: {self.completed_waypoints}개")
        rospy.loginfo(f"   실패: {self.failed_waypoints}개")
        rospy.loginfo(f"   성공률: {success_rate:.1f}%")
        rospy.loginfo(f"   최종 웨이포인트 인덱스: {self.current_waypoint_index}")
        rospy.loginfo(f"   🎯 navigation_manager를 통한 장애물 회피 완료")
        
        # 완료 상태 발행
        status = {
            "status": "completed_sequential",
            "total_waypoints": self.total_waypoints,
            "completed_waypoints": self.completed_waypoints,
            "failed_waypoints": self.failed_waypoints,
            "success_rate": success_rate,
            "final_waypoint_index": self.current_waypoint_index,
            "navigation_method": "sequential_via_navigation_manager"
        }
        self.status_pub.publish(String(data=json.dumps(status)))
        
        rospy.loginfo("🔄 순차 네비게이션 상태 초기화 완료 - 새로운 웨이포인트 수신 대기")
    
    def cancel_current_goal(self):
        """현재 move_base goal 취소"""
        try:
            # 모든 목표 취소 (빈 GoalID로 전체 취소)
            cancel_msg = GoalID()
            self.goal_cancel_pub.publish(cancel_msg)
            rospy.loginfo("📤 move_base goal 취소 신호 전송")
            rospy.sleep(0.5)  # 취소 처리 대기
        except Exception as e:
            rospy.logwarn(f"⚠️ Goal 취소 실패: {e}")
    
    def reset_navigation_state(self):
        """네비게이션 상태 완전 초기화"""
        rospy.loginfo("🔄 네비게이션 상태 완전 초기화 중...")
        
        # 모든 상태 변수 초기화
        self.is_navigating = False
        self.navigation_started = False
        self.current_goal_sent = False
        self.goal_start_time = None
        self.last_success_time = rospy.Time(0)
        
        # 진행 상태 초기화
        self.current_waypoint_index = 0
        self.completed_waypoints = 0
        self.failed_waypoints = 0
        self.total_waypoints = 0
        
        rospy.loginfo("✅ 네비게이션 상태 초기화 완료")
    
    def clear_waypoint_data(self):
        """기존 웨이포인트 데이터 완전 삭제"""
        rospy.loginfo("🗑️ 기존 웨이포인트 데이터 삭제 중...")
        
        # 웨이포인트 관련 데이터 삭제
        old_count = len(self.converted_waypoints_local) if hasattr(self, 'converted_waypoints_local') else 0
        self.converted_waypoints_local = []
        self.destination_local = None
        
        if old_count > 0:
            rospy.loginfo(f"🗑️ 기존 {old_count}개 웨이포인트 삭제 완료")
        
        rospy.loginfo("✅ 웨이포인트 데이터 삭제 완료 - 새로운 데이터 수신 준비")
    
    def publish_empty_visualization(self):
        """빈 웨이포인트 시각화 발행 (기존 시각화 제거용)"""
        empty_data = {
            "frame": "map",  # UTM 절대좌표계
            "coordinate_type": "kakao_navigation_route",
            "waypoints": [],
            "destination": None,
            "total_waypoints": 0,
            "current_waypoint": 0,
            "status": "cleared"
        }
        
        self.visualization_pub.publish(String(data=json.dumps(empty_data)))
        rospy.loginfo("🗑️ 기존 웨이포인트 시각화 제거 완료")
    
    def odometry_callback(self, msg):
        """위치 정보 콜백 (robot_localization의 최종 결과)"""
        self.update_pose_local(msg.pose.pose, "/odometry/filtered")

    def update_pose_local(self, pose, source):
        """UTM Local 위치 정보 업데이트"""
        try:
            self.current_pose_local = {
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
            
            # [DEBUG] 위치 정보 업데이트 로그
            rospy.loginfo_throttle(5, f"📍 [DEBUG] 로봇 위치 업데이트 (소스: {source}): "
                                      f"x={pose.position.x:.2f}, y={pose.position.y:.2f}")

            # 웨이포인트가 있고 위치가 안정화되면 네비게이션 시작
            if (self.converted_waypoints_local and not self.navigation_started and 
                self.pose_source != "none"):
                rospy.loginfo("✅ 위치 정보 안정화 → 카카오 네비게이션 시작!")
                self.start_navigation()
                
        except Exception as e:
            rospy.logwarn(f"❌ 위치 정보 업데이트 실패 ({source}): {e}")
    
    def is_pose_stale(self):
        """위치 정보가 오래되었는지 확인"""
        if self.pose_last_received == rospy.Time(0):
            return True
        time_diff = (rospy.Time.now() - self.pose_last_received).to_sec()
        return time_diff > self.pose_timeout
    
    def gps_callback(self, msg):
        """GPS 데이터 업데이트 (참고용)"""
        if msg.status.status >= 0:
            self.current_gps = {
                "lat": msg.latitude,
                "lon": msg.longitude,
                "alt": msg.altitude
            }
    
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
        """현재 위치에서 웨이포인트 도달 여부 확인"""
        if self.current_pose_local is None:
            return False
        distance = self.calculate_distance(self.current_pose_local, waypoint)
        return distance <= self.waypoint_reached_threshold
    
    def move_base_status_callback(self, msg):
        """move_base 상태 모니터링"""
        if not msg.status_list or not self.current_goal_sent:
            return
            
        latest_status = msg.status_list[-1]
        current_time = rospy.Time.now()
        
        if latest_status.status == GoalStatus.SUCCEEDED:  # SUCCESS (3)
            time_since_last_success = (current_time - self.last_success_time).to_sec()
            
            # 디바운스 체크 (중복 SUCCESS 방지)
            if time_since_last_success < self.success_debounce_duration:
                rospy.loginfo_throttle(10, f"⏳ SUCCESS 디바운스 중 ({time_since_last_success:.1f}s < {self.success_debounce_duration}s)")
                return
                
            if self.current_pose_local is None:
                rospy.logwarn("⚠️ SUCCESS 수신했지만 현재 위치 정보가 없음")
                return
            
            # 네비게이션 상태 검증
            if not self.is_navigating:
                rospy.logwarn("⚠️ SUCCESS 수신했지만 네비게이션이 비활성 상태")
                return

            # 현재 웨이포인트 인덱스 유효성 검증
            rospy.loginfo(f"🔍 SUCCESS 검증: 현재 WP{self.current_waypoint_index + 1}/{len(self.converted_waypoints_local)}")
            
            # 거리 검증 (현재 웨이포인트 또는 목적지)
            if self.current_waypoint_index < len(self.converted_waypoints_local):
                target = self.converted_waypoints_local[self.current_waypoint_index]
                target_type = f"웨이포인트 {self.current_waypoint_index + 1}"
            elif self.destination_local:
                target = self.destination_local
                target_type = "최종 목적지"
            else:
                rospy.loginfo("🎯 모든 웨이포인트 완료!")
                self.complete_navigation()
                return

            # ✅ move_base의 SUCCEEDED 상태를 신뢰하고 추가 거리 검증은 생략
            rospy.loginfo(f"🎯 {target_type} 도달 SUCCESS! (move_base 판단)")
            self.last_success_time = current_time
            
            # 목적지 도달 확인
            if (self.current_waypoint_index >= len(self.converted_waypoints_local) - 1 and 
                self.destination_local):
                rospy.loginfo("🏁 최종 목적지 도달!")
                self.complete_navigation()
            else:
                # 순차적으로 다음 웨이포인트 전송
                rospy.loginfo(f"✅ WP{self.current_waypoint_index + 1} 완료 → 다음 웨이포인트로 순차 이동")
                self.move_to_next_waypoint()
                
        elif latest_status.status in [GoalStatus.ABORTED, GoalStatus.REJECTED]:  # FAILED (4, 5)
            self.failed_waypoints += 1
            rospy.logwarn(f"❌ move_base 실패! (상태: {latest_status.status})")
            rospy.logwarn(f"   실패한 웨이포인트: {self.current_waypoint_index + 1}")
            
            # 실패 처리: 다음 웨이포인트로 스킵하거나 재시도
            if self.failed_waypoints < 3:  # 3번까지는 다음 웨이포인트로 스킵
                rospy.loginfo("⏭️ 다음 웨이포인트로 스킵...")
                self.move_to_next_waypoint()
            else:
                rospy.logerr("❌ 연속 실패 3회 → 네비게이션 중단")
                self.complete_navigation()
    
    def navigation_monitor(self, _):
        """네비게이션 모니터링 (타임아웃 체크)"""
        if not self.is_navigating or not self.current_goal_sent:
            return
            
        if self.goal_start_time is None:
            return
            
        # 목표 도달 타임아웃 체크
        elapsed_time = (rospy.Time.now() - self.goal_start_time).to_sec()
        if elapsed_time > self.goal_timeout:
            rospy.logwarn(f"⏰ 목표 도달 타임아웃! ({elapsed_time:.1f}s)")
            rospy.logwarn(f"   현재 웨이포인트: {self.current_waypoint_index + 1}")
            
            # 타임아웃된 웨이포인트 스킵
            self.failed_waypoints += 1
            if self.failed_waypoints < 3:
                rospy.loginfo("⏭️ 타임아웃으로 인한 스킵...")
                self.move_to_next_waypoint()
            else:
                rospy.logerr("❌ 연속 타임아웃 → 네비게이션 중단")
                self.complete_navigation()
    
    def status_monitor(self, _):
        """상태 모니터링 및 로깅"""
        if not self.is_navigating or not self.converted_waypoints_local:
            return
            
        if self.current_pose_local is None:
            rospy.logwarn_throttle(10, "⚠️ 위치 정보 없음 - 네비게이션 대기 중...")
            return
            
        # 현재 목표와의 거리 계산
        if self.current_waypoint_index < len(self.converted_waypoints_local):
            current_target = self.converted_waypoints_local[self.current_waypoint_index]
            target_type = "웨이포인트"
        elif self.destination_local:
            current_target = self.destination_local
            target_type = "목적지"
        else:
            return
            
        if self.current_goal_sent:
            distance = self.calculate_distance(self.current_pose_local, current_target)
            progress = int((self.completed_waypoints / self.total_waypoints) * 100) if self.total_waypoints > 0 else 0
            
            rospy.loginfo_throttle(10, 
                f"🚗 카카오 자율주행 진행 상황:\n"
                f"   진행률: {progress}% ({self.completed_waypoints}/{self.total_waypoints})\n"
                f"   현재 목표: {target_type} ({distance:.1f}m)\n"
                f"   현재 위치: Local({self.current_pose_local['x']:.1f}, {self.current_pose_local['y']:.1f})\n"
                f"   목표 위치: Local({current_target['x']:.1f}, {current_target['y']:.1f})")
            
            # 목표까지 너무 가까우면 도달 판정
            if distance <= self.waypoint_reached_threshold:
                rospy.loginfo(f"📍 {target_type} 근접 도달! (거리: {distance:.1f}m)")
    
    def publish_web_status(self, _):
        """웹 인터페이스용 상태 정보 발행"""
        status = {
            "navigation_active": self.is_navigating,
            "utm_origin_synced": self.origin_synced,
            "current_pose_available": self.current_pose_local is not None,
            "pose_source": self.pose_source,
            "total_waypoints": self.total_waypoints,
            "completed_waypoints": self.completed_waypoints,
            "current_waypoint_index": self.current_waypoint_index,
            "failed_waypoints": self.failed_waypoints
        }
        
        if self.total_waypoints > 0:
            status["progress_percentage"] = int((self.completed_waypoints / self.total_waypoints) * 100)
        else:
            status["progress_percentage"] = 0
            
        if self.current_pose_local:
            status["current_position"] = {
                "x": self.current_pose_local["x"],
                "y": self.current_pose_local["y"]
            }
            
        self.web_status_pub.publish(String(data=json.dumps(status)))

if __name__ == '__main__':
    try:
        navigator = KakaoNavigationSystem()
        rospy.loginfo("🎉 카카오 네비게이션 시스템 실행 중...")
        rospy.loginfo("🌐 웹에서 지도를 클릭하면 자동으로 그 지점으로 이동합니다!")
        rospy.loginfo("📍 웨이포인트를 순차적으로 방문하여 목적지까지 자율주행!")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("카카오 네비게이션 시스템 종료")