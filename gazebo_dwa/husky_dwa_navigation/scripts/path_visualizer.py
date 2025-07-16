#!/usr/bin/env python3

import rospy
import json
import utm
import math
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import numpy as np

class UTMLocalHeadingCorrection:
    """UTM Local 기반 점진적 Heading 보정 Localizer - RViz 호환성 개선"""
    
    def __init__(self):
        rospy.set_param('/use_sim_time', False)
        rospy.init_node('utm_local_heading_correction', anonymous=True)

        # 🎯 UTM Local 좌표계 설정 (RViz 호환)
        self.utm_origin_absolute = None      # 절대 UTM 원점 (GPS 첫 수신 시 설정)
        self.utm_zone = None                 # UTM 존
        self.first_gps_received = False      # 첫 GPS 수신 여부
        self.last_good_gps = None
        
        # FasterLIO 기준점
        self.fasterlio_origin = None
        self.current_body_pose = None        # 현재 FasterLIO body pose
        
        # 🔥 점진적 Heading 보정 시스템
        self.correction_system = {
            "heading_correction": 0.0,         # 현재 적용 중인 heading 보정
            "initial_alignment_done": False,   # 초기 정렬 완료 여부
            "last_correction_time": 0.0,       # 마지막 보정 시간
        }
        
        # 궤적 기록 (모두 UTM Local 상대좌표)
        self.fasterlio_trajectory_local = []   # FasterLIO → UTM Local 변환
        self.gps_trajectory_local = []         # GPS → UTM Local
        self.corrected_trajectory_local = []   # Heading 보정된 FasterLIO
        self.latest_waypoints = None
        
        # 🎯 현재 위치 (UTM Local 좌표)
        self.current_pose_local = None
        self.pose_covariance = np.eye(6) * 0.1
        
        # 거리 추적
        self.total_distance = 0.0
        self.last_position = None

        # ✅ UTM Local 좌표 Publishers
        self.pose_pub = rospy.Publisher("/robot_pose", PoseWithCovarianceStamped, queue_size=1)
        self.odom_pub = rospy.Publisher("/fused_odom", Odometry, queue_size=1)
        
        # ✅ UTM Local 좌표 시각화 Publishers
        self.fasterlio_path_pub = rospy.Publisher("/fasterlio_path", Marker, queue_size=10)
        self.gps_path_pub = rospy.Publisher("/gps_path", Marker, queue_size=10)
        self.corrected_path_pub = rospy.Publisher("/corrected_path", Marker, queue_size=10)
        self.uncertainty_pub = rospy.Publisher("/pose_uncertainty", Marker, queue_size=10)
        self.waypoints_pub = rospy.Publisher("/global_waypoints", MarkerArray, queue_size=10)

        # GPS 원점 정보 발행 (절대 좌표 정보 제공)
        self.gps_data_pub = rospy.Publisher("/gps_data", String, queue_size=10)
        self.utm_origin_pub = rospy.Publisher("/utm_origin_info", String, queue_size=10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribers
        rospy.Subscriber("/Odometry", Odometry, self.fasterlio_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)
        
        # Timers
        rospy.Timer(rospy.Duration(0.1), self.publish_current_pose)
        rospy.Timer(rospy.Duration(0.5), self.publish_visualization)
        rospy.Timer(rospy.Duration(0.1), self.broadcast_dynamic_tf)
        rospy.Timer(rospy.Duration(1.0), self.publish_origin_info)
        rospy.Timer(rospy.Duration(2.0), self.check_gradual_heading_correction)
        
        rospy.loginfo("🚀 UTM Local 기반 Heading 보정 Localizer 시작! (RViz 호환)")
        rospy.loginfo("📍 GPS_FIRST 전략: 첫 실시간 GPS 수신 시 UTM 원점 설정")
        rospy.loginfo("🌍 모든 좌표계 UTM Local 상대좌표로 통일 (RViz 최적화)!")
        rospy.loginfo("🔄 동적 TF 발행으로 실시간 움직임 반영!")
        rospy.loginfo("🔄 점진적 Heading 보정 활성화 (2초마다 체크, 5초마다 실행)!")

    def setup_utm_origin_from_gps(self, lat, lon):
        """🎯 GPS 원점 설정 - UTM Local 좌표계"""
        rospy.loginfo(f"🔄 UTM Local 원점 설정 시도:")
        rospy.loginfo(f"   - GPS 좌표: ({lat:.6f}, {lon:.6f})")
        
        if not self.first_gps_received and self.fasterlio_origin is not None:
            # UTM 변환
            if abs(lat) < 0.01 and abs(lon) < 0.01:
                rospy.loginfo("🎮 시뮬레이션 GPS 감지")
                easting = lat * 111320.0
                northing = lon * 111320.0
                zone_num, zone_letter = 52, 'S'
            else:
                rospy.loginfo("🌍 실제 GPS 좌표 처리")
                easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
            
            rospy.loginfo(f"   - 절대 UTM: ({easting:.1f}, {northing:.1f}) Zone:{zone_num}{zone_letter}")
            
            # FasterLIO 현재 위치 고려한 동기화
            if self.current_body_pose:
                # FasterLIO 원점 기준 상대 위치
                fasterlio_rel_x = self.current_body_pose["x"] - self.fasterlio_origin["x"]
                fasterlio_rel_y = self.current_body_pose["y"] - self.fasterlio_origin["y"]
                
                # UTM Local 원점 = 현재 로봇 위치를 (0,0)으로 설정
                # GPS 위치에서 FasterLIO 상대 위치를 빼서 원점 계산
                local_origin_easting = easting - fasterlio_rel_x
                local_origin_northing = northing - fasterlio_rel_y
                
                rospy.loginfo(f"🔄 UTM Local 원점 동기화:")
                rospy.loginfo(f"   - FasterLIO 상대위치: ({fasterlio_rel_x:.2f}, {fasterlio_rel_y:.2f})")
                rospy.loginfo(f"   - 로봇 현재 GPS: ({easting:.1f}, {northing:.1f})")
                rospy.loginfo(f"   - UTM Local 원점: ({local_origin_easting:.1f}, {local_origin_northing:.1f})")
                
                self.utm_origin_absolute = {
                    "easting": local_origin_easting,
                    "northing": local_origin_northing,
                    "lat": lat,
                    "lon": lon,
                    "robot_current_gps": {"lat": lat, "lon": lon, "easting": easting, "northing": northing}
                }
                
                rospy.loginfo(f"✅ UTM Local 좌표계 설정 완료!")
                rospy.loginfo(f"   - 로봇 현재 위치가 UTM Local (0, 0)이 됩니다")
            else:
                # FasterLIO 위치 정보가 없으면 GPS 위치를 원점으로 설정
                self.utm_origin_absolute = {
                    "easting": easting,
                    "northing": northing,
                    "lat": lat,
                    "lon": lon,
                    "robot_current_gps": {"lat": lat, "lon": lon, "easting": easting, "northing": northing}
                }
            
            self.utm_zone = f"{zone_num}{zone_letter}"
            self.first_gps_received = True
            
            rospy.loginfo(f"   UTM Local 원점 설정 완료!")
            rospy.loginfo(f"   GPS: ({lat:.6f}, {lon:.6f})")
            rospy.loginfo(f"   절대 UTM 원점: ({self.utm_origin_absolute['easting']:.1f}, {self.utm_origin_absolute['northing']:.1f})")
            rospy.loginfo(f"   Zone: {self.utm_zone}")
            rospy.loginfo(f"    현재 로봇 위치 = UTM Local (0, 0)")
            
            return True
        else:
            rospy.logwarn(f"❌ UTM 원점 설정 조건 미충족")
            return False
    
    def gps_to_utm_local(self, lat, lon):
        """GPS → UTM Local 변환 (상대좌표)"""
        if not self.utm_origin_absolute:
            return 0.0, 0.0, None
            
        if abs(lat) < 0.01 and abs(lon) < 0.01:
            # 시뮬레이션 GPS 처리
            easting = lat * 111320.0
            northing = lon * 111320.0
            zone = self.utm_zone
        else:
            easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
            zone = f"{zone_num}{zone_letter}"
        
        # UTM Local 상대좌표 계산
        local_x = easting - self.utm_origin_absolute["easting"]
        local_y = northing - self.utm_origin_absolute["northing"]
        
        return local_x, local_y, zone
    
    def fasterlio_to_utm_local(self, fasterlio_x, fasterlio_y):
        """FasterLIO 좌표를 UTM Local 상대좌표로 변환"""
        if not self.utm_origin_absolute or not self.fasterlio_origin:
            return fasterlio_x, fasterlio_y
        
        # 1단계: FasterLIO 원점 기준 상대좌표
        rel_x = fasterlio_x - self.fasterlio_origin["x"]
        rel_y = fasterlio_y - self.fasterlio_origin["y"]
        
        # 2단계: Heading 보정 적용
        if self.correction_system["initial_alignment_done"]:
            corrected_x, corrected_y = self.rotate_point_around_origin(
                rel_x, rel_y, self.correction_system["heading_correction"]
            )
        else:
            corrected_x, corrected_y = rel_x, rel_y
        
        # 3단계: UTM Local 좌표로 변환 (상대좌표이므로 그대로 사용)
        return corrected_x, corrected_y
    
    def rotate_point_around_origin(self, x, y, angle, origin_x=0.0, origin_y=0.0):
        """원점 기준으로 점 회전"""
        rel_x = x - origin_x
        rel_y = y - origin_y
        
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        rotated_x = rel_x * cos_a - rel_y * sin_a
        rotated_y = rel_x * sin_a + rel_y * cos_a
        
        final_x = rotated_x + origin_x
        final_y = rotated_y + origin_y
        
        return final_x, final_y
    
    def calculate_trajectory_heading(self, trajectory, min_distance=2.0):
        """궤적에서 heading 계산"""
        if len(trajectory) < 2:
            return None
        
        max_distance = 0
        best_heading = None
        
        for i in range(len(trajectory)):
            for j in range(i + 1, len(trajectory)):
                p1 = trajectory[i]
                p2 = trajectory[j]
                
                if "x" not in p1 or "y" not in p1 or "x" not in p2 or "y" not in p2:
                    continue
                
                distance = math.sqrt((p2["x"] - p1["x"])**2 + (p2["y"] - p1["y"])**2)
                
                if distance > max_distance and distance >= min_distance:
                    max_distance = distance
                    best_heading = math.atan2(p2["y"] - p1["y"], p2["x"] - p1["x"])
        
        if best_heading is not None:
            rospy.loginfo(f"✅ Heading 계산: {math.degrees(best_heading):.1f}도 (거리: {max_distance:.1f}m)")
        
        return best_heading
    
    def perform_initial_heading_alignment(self):
        """🎯 초기 Heading 정렬"""
        if len(self.fasterlio_trajectory_local) < 2 or len(self.gps_trajectory_local) < 2:
            rospy.logwarn("❌ 초기 Heading 정렬용 궤적 데이터 부족")
            return False
        
        # 방향 계산
        fasterlio_heading = self.calculate_trajectory_heading(self.fasterlio_trajectory_local, 0.5)
        gps_heading = self.calculate_trajectory_heading(self.gps_trajectory_local, 0.5)
        
        if fasterlio_heading is None or gps_heading is None:
            rospy.logwarn("❌ Heading 계산 실패")
            return False
        
        # 회전각 계산 및 정규화
        angle_diff = gps_heading - fasterlio_heading
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 보정 설정
        self.correction_system["heading_correction"] = angle_diff
        self.correction_system["initial_alignment_done"] = True
        
        rospy.loginfo(f"🎯 초기 Heading 정렬 완료!")
        rospy.loginfo(f"   FasterLIO: {math.degrees(fasterlio_heading):.1f}도")
        rospy.loginfo(f"   GPS: {math.degrees(gps_heading):.1f}도")
        rospy.loginfo(f"   보정: {math.degrees(angle_diff):.1f}도")
        
        self.recalculate_all_trajectories()
        return True
    
    def perform_gradual_heading_correction(self):
        """🔄 점진적 Heading 보정"""
        if len(self.corrected_trajectory_local) < 3 or len(self.gps_trajectory_local) < 3:
            rospy.loginfo(f"⏳ 점진적 보정용 데이터 부족 (FasterLIO: {len(self.corrected_trajectory_local)}, GPS: {len(self.gps_trajectory_local)})")
            return False
        
        # 최신 3개 포인트 사용 (더 반응성 높임)
        corrected_recent = self.corrected_trajectory_local[-3:]
        gps_recent = self.gps_trajectory_local[-3:]
        
        corrected_start = corrected_recent[0]
        corrected_end = corrected_recent[-1]
        gps_start = gps_recent[0]
        gps_end = gps_recent[-1]
        
        # 방향 계산
        corrected_dx = corrected_end["x"] - corrected_start["x"]
        corrected_dy = corrected_end["y"] - corrected_start["y"]
        corrected_distance = math.sqrt(corrected_dx**2 + corrected_dy**2)
        
        gps_dx = gps_end["x"] - gps_start["x"]
        gps_dy = gps_end["y"] - gps_start["y"]
        gps_distance = math.sqrt(gps_dx**2 + gps_dy**2)
        
        if corrected_distance < 0.3 or gps_distance < 0.3:
            rospy.loginfo( f"⏳ 보정 거리 부족 (FasterLIO: {corrected_distance:.1f}m, GPS: {gps_distance:.1f}m)")
            return False
        
        corrected_heading = math.atan2(corrected_dy, corrected_dx)
        gps_heading = math.atan2(gps_dy, gps_dx)
        
        angle_diff = gps_heading - corrected_heading
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        if abs(angle_diff) < math.radians(3.0):
            rospy.loginfo( f"✅ Heading 정렬 양호 (차이: {math.degrees(angle_diff):.1f}도)")
            return False
        
        # 점진적 보정 (20%씩 적용하여 더 빠른 수렴)
        additional_correction = angle_diff * 0.08
        old_correction = self.correction_system["heading_correction"]
        self.correction_system["heading_correction"] += additional_correction
        
        rospy.loginfo(f"🔄 점진적 Heading 보정 적용:")
        rospy.loginfo(f"   FasterLIO 궤적: {corrected_distance:.1f}m, GPS 궤적: {gps_distance:.1f}m")
        rospy.loginfo(f"   전체 각도 차이: {math.degrees(angle_diff):.1f}도")
        rospy.loginfo(f"   보정 강도: {math.degrees(additional_correction):.1f}도 (20% 적용)")
        rospy.loginfo(f"   총 보정: {math.degrees(old_correction):.1f}° → {math.degrees(self.correction_system['heading_correction']):.1f}°")
        
        self.recalculate_all_trajectories()
        return True
    
    def check_gradual_heading_correction(self, event):
        """점진적 Heading 보정 체크 (2초마다)"""
        if not self.correction_system["initial_alignment_done"]:
            rospy.loginfo_throttle(10, "⏳ 초기 정렬이 완료되지 않음 - 점진적 보정 대기 중")
            return
        
        current_time = rospy.Time.now().to_sec()
        time_since_last = current_time - self.correction_system.get("last_correction_time", 0)
        
        rospy.loginfo_throttle(15, f"🔄 점진적 보정 체크: 마지막 보정 후 {time_since_last:.1f}초 경과")
        
        if time_since_last > 5.0:
            rospy.loginfo("🔄 점진적 Heading 보정 시도 중...")
            if self.perform_gradual_heading_correction():
                self.correction_system["last_correction_time"] = current_time
                rospy.loginfo("✅ 점진적 Heading 보정 성공!")
            else:
                rospy.loginfo("ℹ️ 점진적 Heading 보정 조건 미충족")
    
    def recalculate_all_trajectories(self):
        """전체 FasterLIO 궤적을 UTM Local로 재계산"""
        if not self.fasterlio_trajectory_local:
            return
        
        self.corrected_trajectory_local = []
        
        for fasterlio_point in self.fasterlio_trajectory_local:
            # 원본 FasterLIO 좌표 복원 (UTM Local에서 FasterLIO로)
            if not self.fasterlio_origin:
                continue
                
            rel_x = fasterlio_point["x"]
            rel_y = fasterlio_point["y"]
            original_x = rel_x + self.fasterlio_origin["x"]
            original_y = rel_y + self.fasterlio_origin["y"]
            
            # 다시 보정 적용하여 UTM Local로 변환
            corrected_local_x, corrected_local_y = self.fasterlio_to_utm_local(original_x, original_y)
            
            corrected_point = fasterlio_point.copy()
            corrected_point["x"] = corrected_local_x
            corrected_point["y"] = corrected_local_y
            
            self.corrected_trajectory_local.append(corrected_point)
        
        rospy.loginfo(f"✅ {len(self.corrected_trajectory_local)}개 포인트 재계산 완료")
    
    def update_distance(self, new_position):
        """이동 거리 업데이트"""
        if self.last_position is not None:
            dx = new_position["x"] - self.last_position["x"]
            dy = new_position["y"] - self.last_position["y"]
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance += distance
        
        self.last_position = new_position.copy()
    
    def distance_check_local(self, pose1, pose2, threshold):
        """UTM Local 좌표 거리 체크"""
        dx = pose1["x"] - pose2["x"]
        dy = pose1["y"] - pose2["y"]
        return math.sqrt(dx*dx + dy*dy) > threshold
    
    def fasterlio_callback(self, msg):
        """🎯 FasterLIO 메인 콜백"""
        timestamp = msg.header.stamp.to_sec()
        
        # FasterLIO 원시 pose 저장
        self.current_body_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "qx": msg.pose.pose.orientation.x,
            "qy": msg.pose.pose.orientation.y,
            "qz": msg.pose.pose.orientation.z,
            "qw": msg.pose.pose.orientation.w,
            "timestamp": timestamp
        }
        
        # 첫 번째 포즈면 기준점 설정
        if self.fasterlio_origin is None:
            self.fasterlio_origin = self.current_body_pose.copy()
            rospy.loginfo("🎯 FasterLIO 기준점 설정 완료")
        
        # UTM Local 변환 (보정 없이)
        if self.utm_origin_absolute:
            rel_x = self.current_body_pose["x"] - self.fasterlio_origin["x"]
            rel_y = self.current_body_pose["y"] - self.fasterlio_origin["y"]
            
            local_point = {
                "x": rel_x,
                "y": rel_y,
                "z": self.current_body_pose["z"],
                "qx": self.current_body_pose["qx"],
                "qy": self.current_body_pose["qy"],
                "qz": self.current_body_pose["qz"],
                "qw": self.current_body_pose["qw"],
                "timestamp": timestamp
            }
            
            # 궤적 기록
            if not self.fasterlio_trajectory_local or self.distance_check_local(local_point, self.fasterlio_trajectory_local[-1], 0.2):
                self.fasterlio_trajectory_local.append(local_point.copy())
        
        # Heading 보정 적용하여 UTM Local 변환
        corrected_local_x, corrected_local_y = self.fasterlio_to_utm_local(
            self.current_body_pose["x"], self.current_body_pose["y"]
        )
        
        # Orientation 보정
        corrected_qx, corrected_qy, corrected_qz, corrected_qw = self.apply_heading_correction_to_orientation(
            self.current_body_pose["qx"], self.current_body_pose["qy"],
            self.current_body_pose["qz"], self.current_body_pose["qw"]
        )
        
        # 현재 위치 업데이트 (UTM Local 좌표)
        self.current_pose_local = {
            "x": corrected_local_x,
            "y": corrected_local_y,
            "z": self.current_body_pose["z"],
            "qx": corrected_qx,
            "qy": corrected_qy,
            "qz": corrected_qz,
            "qw": corrected_qw,
            "timestamp": timestamp
        }
        
        # 거리 및 궤적 업데이트
        self.update_distance(self.current_pose_local)
        
        if self.utm_origin_absolute:
            if not self.corrected_trajectory_local or self.distance_check_local(self.current_pose_local, self.corrected_trajectory_local[-1], 0.2):
                self.corrected_trajectory_local.append(self.current_pose_local.copy())
        
        # 초기 정렬 체크 (조건 완화: 0.5m 이동 또는 3개 이상 궤적 데이터)
        if (not self.correction_system["initial_alignment_done"] and 
            (self.total_distance >= 0.5 or 
             (len(self.fasterlio_trajectory_local) >= 3 and len(self.gps_trajectory_local) >= 2))):
            rospy.loginfo(f"📏 총 이동거리 {self.total_distance:.1f}m, FasterLIO {len(self.fasterlio_trajectory_local)}개, GPS {len(self.gps_trajectory_local)}개 → 초기 Heading 정렬 수행")
            self.perform_initial_heading_alignment()
        
        # 불확실성 업데이트
        uncertainty = 2.0 if self.correction_system["initial_alignment_done"] else 10.0
        self.pose_covariance[0,0] = uncertainty
        self.pose_covariance[1,1] = uncertainty
        
        rospy.loginfo_throttle(2, f"🎯 UTM Local 위치: ({corrected_local_x:.1f}, {corrected_local_y:.1f}), "
                                 f"누적거리: {self.total_distance:.1f}m")

    def apply_heading_correction_to_orientation(self, qx, qy, qz, qw):
        """Orientation에 heading 보정 적용"""
        if not self.correction_system["initial_alignment_done"]:
            return qx, qy, qz, qw
        
        # Quaternion → Euler (tf2 호환)
        def euler_from_quaternion(quat):
            x, y, z, w = quat
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll = np.arctan2(t0, t1)
            t2 = +2.0 * (w * y - z * x)
            t2 = np.clip(t2, -1.0, 1.0)
            pitch = np.arcsin(t2)
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw = np.arctan2(t3, t4)
            return roll, pitch, yaw
        
        roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
        
        # Yaw 보정
        corrected_yaw = yaw + self.correction_system["heading_correction"]
        
        # 정규화
        while corrected_yaw > math.pi:
            corrected_yaw -= 2 * math.pi
        while corrected_yaw < -math.pi:
            corrected_yaw += 2 * math.pi
        
        # Euler → Quaternion (tf2 호환)
        def quaternion_from_euler(roll, pitch, yaw):
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy
            return [x, y, z, w]
        
        corrected_quat = quaternion_from_euler(roll, pitch, corrected_yaw)
        
        return corrected_quat[0], corrected_quat[1], corrected_quat[2], corrected_quat[3]
    
    def gps_callback(self, msg):
        """GPS 콜백"""
        rospy.loginfo_throttle(10, f"📡 GPS 메시지 수신: status={msg.status.status}, lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")
        
        if msg.status.status >= 0:
            # FasterLIO 준비 대기
            if not self.first_gps_received:
                if self.fasterlio_origin is None:
                    rospy.loginfo_throttle(2, "⏳ FasterLIO 대기 중... GPS 원점 설정 보류")
                    return
                else:
                    success = self.setup_utm_origin_from_gps(msg.latitude, msg.longitude)
                    if success:
                        rospy.loginfo(f"🎯 GPS 기반 UTM Local 원점 설정 성공!")
            
            timestamp = msg.header.stamp.to_sec()
            gps_local_x, gps_local_y, zone = self.gps_to_utm_local(msg.latitude, msg.longitude)
            
            self.last_good_gps = {
                "x": gps_local_x,
                "y": gps_local_y,
                "timestamp": timestamp,
                "lat": msg.latitude,
                "lon": msg.longitude,
                "utm_zone": zone
            }
            
            # GPS 궤적 기록 (더 자주 기록하도록 임계값 낮춤)
            if not self.gps_trajectory_local or self.distance_check_local(self.last_good_gps, self.gps_trajectory_local[-1], 0.3):
                self.gps_trajectory_local.append(self.last_good_gps.copy())
                rospy.loginfo_throttle(5, f"📡 GPS Local 궤적 업데이트: ({gps_local_x:.1f}, {gps_local_y:.1f}) | 총 {len(self.gps_trajectory_local)}개")
                
                # GPS 궤적이 충분하면 즉시 초기 정렬 시도
                if (not self.correction_system["initial_alignment_done"] and 
                    len(self.gps_trajectory_local) >= 2 and len(self.fasterlio_trajectory_local) >= 2):
                    rospy.loginfo(f"📡 GPS 궤적 충분 → 즉시 초기 Heading 정렬 시도")
                    self.perform_initial_heading_alignment()
        else:
            rospy.logwarn_throttle(10, f"❌ GPS 신호 무효: status={msg.status.status}")
    
    def waypoints_callback(self, msg):
        """Waypoints 수신 및 파싱 - UTM Local 좌표계로 변환"""
        try:
            data = json.loads(msg.data)
            rospy.loginfo("📥 새로운 waypoints 수신됨")
        
            if "waypoints" in data:
                self.latest_waypoints = []
                
                # 받은 waypoints를 UTM Local 좌표계로 변환
                for i, wp in enumerate(data["waypoints"]):
                    local_wp = wp.copy()
                    
                    if "lat" in wp and "lon" in wp:
                        # GPS 좌표를 UTM Local로 변환
                        local_x, local_y, _ = self.gps_to_utm_local(wp["lat"], wp["lon"])
                        local_wp["x"] = local_x
                        local_wp["y"] = local_y
                        rospy.loginfo(f"   - WP{i+1}: GPS({wp['lat']:.6f}, {wp['lon']:.6f}) → Local({local_x:.1f}, {local_y:.1f})")
                    elif "x" in wp and "y" in wp:
                        # 이미 좌표계가 있는 경우 (절대 UTM일 수도 있음)
                        if self.utm_origin_absolute and abs(wp["x"]) > 1000:
                            # 절대 UTM 좌표로 보이면 Local로 변환
                            local_x = wp["x"] - self.utm_origin_absolute["easting"]
                            local_y = wp["y"] - self.utm_origin_absolute["northing"]
                            local_wp["x"] = local_x
                            local_wp["y"] = local_y
                            rospy.loginfo(f"   - WP{i+1}: UTM({wp['x']:.1f}, {wp['y']:.1f}) → Local({local_x:.1f}, {local_y:.1f})")
                        else:
                            # 이미 Local 좌표로 보임
                            rospy.loginfo(f"   - WP{i+1}: Local({wp['x']:.1f}, {wp['y']:.1f})")
                    
                    self.latest_waypoints.append(local_wp)
                
                rospy.loginfo(f"   - 총 {len(self.latest_waypoints)}개 waypoints (UTM Local 변환 완료)")
                
                # 즉시 시각화 업데이트
                self.visualize_waypoints()
            
            else:
                rospy.logwarn("⚠️ waypoints 키가 없는 데이터 수신")
                self.latest_waypoints = None
            
        except Exception as e:
            rospy.logerr(f"❌ Waypoints 파싱 오류: {e}")
            self.latest_waypoints = None
    
    def broadcast_dynamic_tf(self, event):
        """🔥 완전한 TF tree 구성 - 모든 프레임 연결"""
        if self.current_pose_local is None:
            return
        
        current_time = rospy.Time.now()
        transforms = []
        
        # ===== 메인 좌표계 체인 =====
        
        # ✅ 1. map -> utm_local (RViz 호환성)
        map_to_utm = TransformStamped()
        map_to_utm.header.stamp = current_time
        map_to_utm.header.frame_id = "map"
        map_to_utm.child_frame_id = "utm_local"
        map_to_utm.transform.rotation.w = 1.0  # 단위 변환
        transforms.append(map_to_utm)
        
        # ✅ 2. utm_local -> odom (FasterLIO 메인 연결)
        utm_to_odom = TransformStamped()
        utm_to_odom.header.stamp = current_time
        utm_to_odom.header.frame_id = "utm_local"
        utm_to_odom.child_frame_id = "odom"
        
        utm_to_odom.transform.translation.x = self.current_pose_local["x"]
        utm_to_odom.transform.translation.y = self.current_pose_local["y"]
        utm_to_odom.transform.translation.z = self.current_pose_local["z"]
        utm_to_odom.transform.rotation.x = self.current_pose_local["qx"]
        utm_to_odom.transform.rotation.y = self.current_pose_local["qy"]
        utm_to_odom.transform.rotation.z = self.current_pose_local["qz"]
        utm_to_odom.transform.rotation.w = self.current_pose_local["qw"]
        
        transforms.append(utm_to_odom)
        
        # ✅ 3. odom -> base_link (로봇 표준 연결)
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = current_time
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_link"
        odom_to_base.transform.rotation.w = 1.0  # 단위 변환
        transforms.append(odom_to_base)
        
        # ===== FasterLIO 호환성 =====
        # camera_init -> body transform은 FasterLIO에서 직접 발행하므로 여기서는 제거
        
        # ===== 센서 프레임들 =====
        
        # ✅ 5. base_link -> os_sensor (Ouster 센서 마운트)
        base_to_os_sensor = TransformStamped()
        base_to_os_sensor.header.stamp = current_time
        base_to_os_sensor.header.frame_id = "base_link"
        base_to_os_sensor.child_frame_id = "os_sensor"
        base_to_os_sensor.transform.translation.x = 0.0
        base_to_os_sensor.transform.translation.y = 0.0
        base_to_os_sensor.transform.translation.z = 0.3  # 센서 높이
        base_to_os_sensor.transform.rotation.w = 1.0
        transforms.append(base_to_os_sensor)
        
        # ✅ 6. os_sensor -> os1_lidar
        os_sensor_to_lidar = TransformStamped()
        os_sensor_to_lidar.header.stamp = current_time
        os_sensor_to_lidar.header.frame_id = "os_sensor"
        os_sensor_to_lidar.child_frame_id = "os1_lidar"
        os_sensor_to_lidar.transform.rotation.w = 1.0
        transforms.append(os_sensor_to_lidar)
        
        # ✅ 7. os_sensor -> os1_imu
        os_sensor_to_imu = TransformStamped()
        os_sensor_to_imu.header.stamp = current_time
        os_sensor_to_imu.header.frame_id = "os_sensor"
        os_sensor_to_imu.child_frame_id = "os1_imu"
        os_sensor_to_imu.transform.rotation.w = 1.0
        transforms.append(os_sensor_to_imu)
        
        # ===== 휠 프레임들 (robot_state_publisher 보완) =====
        
        # ✅ 8. base_link -> 각 휠들 (Husky 기본 구조)
        wheel_positions = {
            "front_left_wheel_link": [0.256, 0.2854, 0.0],
            "front_right_wheel_link": [0.256, -0.2854, 0.0],
            "rear_left_wheel_link": [-0.256, 0.2854, 0.0],
            "rear_right_wheel_link": [-0.256, -0.2854, 0.0]
        }
        
        for wheel_name, position in wheel_positions.items():
            wheel_tf = TransformStamped()
            wheel_tf.header.stamp = current_time
            wheel_tf.header.frame_id = "base_link"
            wheel_tf.child_frame_id = wheel_name
            wheel_tf.transform.translation.x = position[0]
            wheel_tf.transform.translation.y = position[1]
            wheel_tf.transform.translation.z = position[2]
            wheel_tf.transform.rotation.w = 1.0
            transforms.append(wheel_tf)
        
        # 모든 TF 발행
        self.tf_broadcaster.sendTransform(transforms)
        
        rospy.loginfo_throttle(30, f"""
        📡 완전한 TF Tree 발행 완료:
        map -> utm_local -> body -> base_link
                        -> odom
        base_link -> os_sensor -> [os1_lidar, os1_imu]
                    -> [4개 휠 프레임들]
        (camera_init -> body는 FasterLIO에서 발행)
        """)
    
    def publish_current_pose(self, event):
        """현재 위치 발행 - UTM Local 좌표계"""
        if self.current_pose_local is None:
            return
        
        current_time = rospy.Time.now()
        
        # Pose 발행
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "utm_local"
        
        pose_msg.pose.pose.position.x = self.current_pose_local["x"]
        pose_msg.pose.pose.position.y = self.current_pose_local["y"]
        pose_msg.pose.pose.position.z = self.current_pose_local["z"]
        pose_msg.pose.pose.orientation.x = self.current_pose_local["qx"]
        pose_msg.pose.pose.orientation.y = self.current_pose_local["qy"]
        pose_msg.pose.pose.orientation.z = self.current_pose_local["qz"]
        pose_msg.pose.pose.orientation.w = self.current_pose_local["qw"]
        pose_msg.pose.covariance = self.pose_covariance.flatten().tolist()
        
        self.pose_pub.publish(pose_msg)
        
        # Odom 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "utm_local"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = pose_msg.pose
        
        self.odom_pub.publish(odom_msg)

    def publish_visualization(self, event):
        """시각화 발행"""
        self.visualize_fasterlio_path()
        self.visualize_gps_path()
        self.visualize_corrected_path()
        self.visualize_uncertainty()
        self.visualize_waypoints()
    
    def visualize_fasterlio_path(self):
        """FasterLIO 원본 경로 (회색)"""
        if len(self.fasterlio_trajectory_local) < 2:
            return
        
        marker = self.create_local_path_marker(
            self.fasterlio_trajectory_local, "fasterlio_original", 0,
            (0.5, 0.5, 0.5), 2.0
        )
        self.fasterlio_path_pub.publish(marker)
    
    def visualize_gps_path(self):
        """GPS 경로 (파란색)"""
        if len(self.gps_trajectory_local) < 2:
            return
            
        marker = self.create_local_path_marker(
            self.gps_trajectory_local, "gps_path", 0,
            (0.0, 0.0, 1.0), 3.0
        )
        self.gps_path_pub.publish(marker)
    
    def visualize_corrected_path(self):
        """보정된 FasterLIO 경로 (빨간색)"""
        if len(self.corrected_trajectory_local) < 2:
            return
        marker = self.create_local_path_marker(
            self.corrected_trajectory_local, "corrected_path", 0,
            (1.0, 0.0, 0.0), 3.0
        )
        self.corrected_path_pub.publish(marker)
    
    def visualize_uncertainty(self):
        """현재 위치 불확실성"""
        if self.current_pose_local is None:
            return
        
        uncertainty = math.sqrt(self.pose_covariance[0,0])
        
        marker = Marker()
        marker.header.frame_id = "utm_local"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pose_uncertainty"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.current_pose_local["x"]
        marker.pose.position.y = self.current_pose_local["y"]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = uncertainty * 2.0
        marker.scale.y = uncertainty * 2.0
        marker.scale.z = 0.1
        
        # 정렬 상태에 따른 색상
        if self.correction_system["initial_alignment_done"]:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0  # 녹색
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0  # 노란색
        
        marker.color.a = 0.3
        self.uncertainty_pub.publish(marker)
    
    def visualize_waypoints(self):
        """웨이포인트 시각화 - UTM Local 좌표계"""
        marker_array = MarkerArray()

        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.header.frame_id = "utm_local"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "local_waypoints"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
    
        delete_text = Marker()
        delete_text.header.frame_id = "utm_local"
        delete_text.header.stamp = rospy.Time.now()
        delete_text.ns = "waypoint_numbers"
        delete_text.action = Marker.DELETEALL
        marker_array.markers.append(delete_text)

        if not self.latest_waypoints:
            self.waypoints_pub.publish(marker_array)
            return

        total_waypoints = len(self.latest_waypoints)
        valid_points = []
        valid_waypoints = []
    
        for i, wp in enumerate(self.latest_waypoints):
            if "x" in wp and "y" in wp:
                local_x, local_y = float(wp["x"]), float(wp["y"])
                valid_points.append(Point(x=local_x, y=local_y, z=0))
                valid_waypoints.append((i, local_x, local_y))

        if not valid_waypoints:
            self.waypoints_pub.publish(marker_array)
            return

        # 연결선 마커
        if len(valid_points) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = "utm_local"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "local_waypoints"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 1.0
            line_marker.color.r = 1.0
            line_marker.color.g = 0.5
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.pose.orientation.w = 1.0
            line_marker.lifetime = rospy.Duration(0)
            line_marker.points = valid_points
        
            marker_array.markers.append(line_marker)

        # 개별 웨이포인트 마커들
        for wp_index, (original_index, local_x, local_y) in enumerate(valid_waypoints):
            # 웨이포인트 큐브 마커
            cube = Marker()
            cube.header.frame_id = "utm_local"
            cube.header.stamp = rospy.Time.now()
            cube.ns = "local_waypoints"
            cube.id = wp_index + 1
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = local_x
            cube.pose.position.y = local_y
            cube.pose.position.z = 2.0
            cube.pose.orientation.w = 1.0
            cube.scale.x = 4.0
            cube.scale.y = 4.0
            cube.scale.z = 2.5
        
            # 색상 구분
            if original_index == 0:
                cube.color.r, cube.color.g, cube.color.b = 0.0, 1.0, 0.0  # 시작점 - 녹색
            elif original_index == total_waypoints - 1:
                cube.color.r, cube.color.g, cube.color.b = 1.0, 0.0, 0.0  # 끝점 - 빨간색
            else:
                cube.color.r, cube.color.g, cube.color.b = 1.0, 1.0, 0.0  # 중간점 - 노란색
        
            cube.color.a = 0.9
            cube.lifetime = rospy.Duration(0)
        
            marker_array.markers.append(cube)
        
            # 웨이포인트 번호 텍스트
            text = Marker()
            text.header.frame_id = "utm_local"
            text.header.stamp = rospy.Time.now()
            text.ns = "waypoint_numbers"
            text.id = wp_index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = local_x
            text.pose.position.y = local_y
            text.pose.position.z = 6.0
            text.pose.orientation.w = 1.0
            text.scale.z = 4.0
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
            text.text = f"WP{original_index+1}"
            text.lifetime = rospy.Duration(0)
        
            marker_array.markers.append(text)

        # 마커 발행
        self.waypoints_pub.publish(marker_array)

    def create_local_path_marker(self, trajectory, namespace, marker_id, color, line_width):
        """UTM Local 좌표 경로 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "utm_local"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = line_width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        
        points = []
        for pt in trajectory:
            points.append(Point(x=pt["x"], y=pt["y"], z=pt.get("z", 0)))
        
        marker.points = points
        return marker
    
    def publish_origin_info(self, event):
        """실시간 GPS 데이터 발행 - 웹 마커 업데이트용 (간단 버전)"""
        # ✅ 실시간 GPS 데이터가 있으면 최신 GPS 전송
        if self.last_good_gps:
            gps_data = {
                "latitude": self.last_good_gps["lat"],
                "longitude": self.last_good_gps["lon"]
            }
            self.gps_data_pub.publish(json.dumps(gps_data))
            rospy.loginfo_throttle(5, f"📡 실시간 GPS → 웹: ({gps_data['latitude']:.6f}, {gps_data['longitude']:.6f})")
            
        # ✅ 실시간 GPS가 없으면 UTM 원점만 전송 (fallback)
        elif self.utm_origin_absolute:
            gps_data = {
                "latitude": self.utm_origin_absolute["lat"],
                "longitude": self.utm_origin_absolute["lon"]
            }
            self.gps_data_pub.publish(json.dumps(gps_data))
            rospy.loginfo_throttle(10, f"📡 UTM 원점 → 웹: ({gps_data['latitude']:.6f}, {gps_data['longitude']:.6f})")
        
        # UTM 원점 정보도 발행 (기존 기능 유지)
        if self.utm_origin_absolute:
            origin_info = {
                "utm_zone": self.utm_zone,
                "utm_origin_absolute": {
                    "easting": self.utm_origin_absolute["easting"],
                    "northing": self.utm_origin_absolute["northing"],
                    "lat": self.utm_origin_absolute["lat"],
                    "lon": self.utm_origin_absolute["lon"]
                },
                "robot_current_gps": self.utm_origin_absolute.get("robot_current_gps", {}),
                "coordinate_system": "utm_local",
                "description": "UTM Local coordinate system - Robot's first GPS position becomes (0,0)"
            }
            self.utm_origin_pub.publish(json.dumps(origin_info))

if __name__ == '__main__':
    try:
        localizer = UTMLocalHeadingCorrection()
        rospy.loginfo("🎉 UTM Local 기반 Heading 보정 Localizer 실행 중...")
        rospy.loginfo("🌍 RViz Fixed Frame을 'utm_local'으로 설정하세요!")
        rospy.loginfo("✅ 모든 좌표계가 UTM Local 상대좌표로 통일 (RViz 최적화)!")
        rospy.loginfo("🎯 로봇의 첫 GPS 위치가 (0, 0)이 됩니다!")
        rospy.loginfo("🔄 FasterLIO와 GPS 시작점 자동 동기화!")
        rospy.loginfo("📡 동적 TF로 실시간 움직임 반영!")
        rospy.loginfo("🔄 점진적 Heading 보정으로 지속적 정확도 향상!")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 시스템 종료")
    except Exception as e:
        rospy.logerr(f"❌ 시스템 오류: {e}")