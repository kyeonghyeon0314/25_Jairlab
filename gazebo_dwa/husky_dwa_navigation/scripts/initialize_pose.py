#!/usr/bin/env python3
import rospy
import utm
import json
import math
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from visualization_msgs.msg import Marker
import tf2_ros

class PoseInitializer:
    """FasterLIO-GPS 융합 위치 추정 및 Heading 보정"""
    
    def __init__(self):
        rospy.init_node('initialize_pose_node', anonymous=True)

        # UTM 좌표계 관리
        self.utm_origin_absolute = None
        self.utm_zone = None
        self.origin_synced = False
        
        # FasterLIO 관리
        self.fasterlio_origin = None
        self.current_body_pose = None
        self.last_good_gps = None

        # 궤적 기록 (UTM Local 좌표)
        self.fasterlio_trajectory_local = []
        self.gps_trajectory_local = []
        self.corrected_trajectory_local = []

        # 🔥 점진적 Heading 보정 시스템
        self.correction_system = {
            "heading_correction": 0.0,
            "initial_alignment_done": False,
            "last_correction_time": 0.0,
        }
        
        # 현재 위치 및 불확실성
        self.current_pose_local = None
        self.pose_covariance = np.eye(6) * 0.1
        
        # 거리 추적
        self.total_distance = 0.0
        self.last_position = None

        # Publishers
        self.pose_pub = rospy.Publisher("/robot_pose", PoseWithCovarianceStamped, queue_size=1)
        self.odom_pub = rospy.Publisher("/fused_odom", Odometry, queue_size=1)
        self.uncertainty_pub = rospy.Publisher("/pose_uncertainty", Marker, queue_size=10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribers
        rospy.Subscriber("/utm_origin_info", String, self.utm_origin_callback)
        rospy.Subscriber("/Odometry", Odometry, self.fasterlio_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        
        # Timers
        rospy.Timer(rospy.Duration(0.1), self.publish_current_pose)
        rospy.Timer(rospy.Duration(0.1), self.broadcast_dynamic_tf)
        rospy.Timer(rospy.Duration(0.5), self.publish_uncertainty)
        rospy.Timer(rospy.Duration(2.0), self.check_gradual_heading_correction)

        rospy.loginfo("🚀 PoseInitializer 시작 - FasterLIO-GPS 융합 위치 추정")

    def utm_origin_callback(self, msg):
        """UTM 원점 정보 동기화"""
        if not self.origin_synced:
            try:
                data = json.loads(msg.data)
                self.utm_origin_absolute = data["utm_origin_absolute"]
                self.utm_zone = data["utm_zone"]
                self.origin_synced = True
                rospy.loginfo(f"✅ UTM 원점 동기화 완료: Zone {self.utm_zone}")
            except (json.JSONDecodeError, KeyError) as e:
                rospy.logwarn(f"⚠️ UTM 원점 데이터 파싱 실패: {e}")

    def fasterlio_callback(self, msg):
        """FasterLIO 콜백 - 메인 위치 추정 로직"""
        if not self.origin_synced:
            return

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

        # 궤적 처리 및 보정 수행
        self.process_trajectories()
        self.publish_fused_pose()

    def gps_callback(self, msg):
        """GPS 콜백"""
        if not self.origin_synced or msg.status.status < 0:
            return
        
        timestamp = msg.header.stamp.to_sec()
        gps_local_x, gps_local_y = self.gps_to_utm_local(msg.latitude, msg.longitude)
        
        self.last_good_gps = {
            "x": gps_local_x,
            "y": gps_local_y,
            "timestamp": timestamp,
            "lat": msg.latitude,
            "lon": msg.longitude
        }
        
        # GPS 궤적 기록
        if not self.gps_trajectory_local or self.distance_check_local(self.last_good_gps, self.gps_trajectory_local[-1], 0.3):
            self.gps_trajectory_local.append(self.last_good_gps.copy())
            rospy.loginfo_throttle(5, f"📡 GPS 궤적 업데이트: ({gps_local_x:.1f}, {gps_local_y:.1f})")

    def gps_to_utm_local(self, lat, lon):
        """GPS → UTM Local 변환"""
        if abs(lat) < 0.01 and abs(lon) < 0.01:
            easting = lat * 111320.0
            northing = lon * 111320.0
        else:
            easting, northing, _, _ = utm.from_latlon(lat, lon)
        
        local_x = easting - self.utm_origin_absolute["easting"]
        local_y = northing - self.utm_origin_absolute["northing"]
        return local_x, local_y

    def process_trajectories(self):
        """궤적 처리 및 초기 정렬"""
        if self.current_body_pose is None:
            return

        # FasterLIO → UTM Local 변환 (보정 없이)
        rel_x = self.current_body_pose["x"] - self.fasterlio_origin["x"]
        rel_y = self.current_body_pose["y"] - self.fasterlio_origin["y"]
        
        local_point = {
            "x": rel_x,
            "y": rel_y,
            "z": self.current_body_pose["z"],
            "timestamp": self.current_body_pose["timestamp"]
        }
        
        # 궤적 기록
        if not self.fasterlio_trajectory_local or self.distance_check_local(local_point, self.fasterlio_trajectory_local[-1], 0.2):
            self.fasterlio_trajectory_local.append(local_point.copy())

        # 거리 추적
        self.update_distance(local_point)

        # 초기 정렬 체크 (조건 완화)
        if (not self.correction_system["initial_alignment_done"] and 
            (self.total_distance >= 0.5 or 
             (len(self.fasterlio_trajectory_local) >= 3 and len(self.gps_trajectory_local) >= 2))):
            rospy.loginfo(f"📏 초기 정렬 조건 충족 → Heading 정렬 수행")
            self.perform_initial_heading_alignment()

    def perform_initial_heading_alignment(self):
        """🎯 초기 Heading 정렬"""
        if len(self.fasterlio_trajectory_local) < 2 or len(self.gps_trajectory_local) < 2:
            rospy.logwarn("❌ 초기 Heading 정렬용 궤적 데이터 부족")
            return False

        fasterlio_heading = self.calculate_trajectory_heading(self.fasterlio_trajectory_local)
        gps_heading = self.calculate_trajectory_heading(self.gps_trajectory_local)

        if fasterlio_heading is not None and gps_heading is not None:
            angle_diff = gps_heading - fasterlio_heading
            self.correction_system["heading_correction"] = self.normalize_angle(angle_diff)
            self.correction_system["initial_alignment_done"] = True
            
            rospy.loginfo(f"🎯 초기 Heading 정렬 완료!")
            rospy.loginfo(f"   FasterLIO: {math.degrees(fasterlio_heading):.1f}도")
            rospy.loginfo(f"   GPS: {math.degrees(gps_heading):.1f}도")
            rospy.loginfo(f"   보정: {math.degrees(angle_diff):.1f}도")
            
            # 기존 궤적 재계산
            self.recalculate_all_trajectories()
            return True
        
        return False

    def check_gradual_heading_correction(self, event):
        """🔄 점진적 Heading 보정 체크 (2초마다)"""
        if not self.correction_system["initial_alignment_done"]:
            rospy.loginfo_throttle(10, "⏳ 초기 정렬 미완료 - 점진적 보정 대기 중")
            return
        
        current_time = rospy.Time.now().to_sec()
        time_since_last = current_time - self.correction_system.get("last_correction_time", 0)
        
        if time_since_last > 5.0:  # 5초마다 실행
            rospy.loginfo("🔄 점진적 Heading 보정 시도...")
            if self.perform_gradual_heading_correction():
                self.correction_system["last_correction_time"] = current_time
                rospy.loginfo("✅ 점진적 Heading 보정 성공!")

    def perform_gradual_heading_correction(self):
        """🔄 점진적 Heading 보정"""
        if len(self.corrected_trajectory_local) < 3 or len(self.gps_trajectory_local) < 3:
            return False
        
        # 최신 3개 포인트 사용
        corrected_recent = self.corrected_trajectory_local[-3:]
        gps_recent = self.gps_trajectory_local[-3:]
        
        corrected_start, corrected_end = corrected_recent[0], corrected_recent[-1]
        gps_start, gps_end = gps_recent[0], gps_recent[-1]
        
        # 방향 계산
        corrected_dx = corrected_end["x"] - corrected_start["x"]
        corrected_dy = corrected_end["y"] - corrected_start["y"]
        corrected_distance = math.sqrt(corrected_dx**2 + corrected_dy**2)
        
        gps_dx = gps_end["x"] - gps_start["x"]
        gps_dy = gps_end["y"] - gps_start["y"]
        gps_distance = math.sqrt(gps_dx**2 + gps_dy**2)
        
        if corrected_distance < 0.3 or gps_distance < 0.3:
            return False
        
        corrected_heading = math.atan2(corrected_dy, corrected_dx)
        gps_heading = math.atan2(gps_dy, gps_dx)
        
        angle_diff = self.normalize_angle(gps_heading - corrected_heading)
        
        if abs(angle_diff) < math.radians(3.0):
            return False
        
        # 점진적 보정 (8%씩 적용)
        additional_correction = angle_diff * 0.08
        old_correction = self.correction_system["heading_correction"]
        self.correction_system["heading_correction"] += additional_correction
        
        rospy.loginfo(f"🔄 점진적 보정: {math.degrees(additional_correction):.1f}도 추가")
        rospy.loginfo(f"   총 보정: {math.degrees(old_correction):.1f}° → {math.degrees(self.correction_system['heading_correction']):.1f}°")
        
        self.recalculate_all_trajectories()
        return True

    def recalculate_all_trajectories(self):
        """전체 FasterLIO 궤적을 보정 적용하여 재계산"""
        self.corrected_trajectory_local = []
        
        for fasterlio_point in self.fasterlio_trajectory_local:
            corrected_x, corrected_y = self.apply_heading_correction(fasterlio_point["x"], fasterlio_point["y"])
            
            corrected_point = fasterlio_point.copy()
            corrected_point["x"] = corrected_x
            corrected_point["y"] = corrected_y
            
            self.corrected_trajectory_local.append(corrected_point)

    def apply_heading_correction(self, x, y):
        """좌표에 Heading 보정 적용"""
        if not self.correction_system["initial_alignment_done"]:
            return x, y
        
        angle = self.correction_system["heading_correction"]
        corrected_x = x * math.cos(angle) - y * math.sin(angle)
        corrected_y = x * math.sin(angle) + y * math.cos(angle)
        
        return corrected_x, corrected_y

    def publish_fused_pose(self):
        """융합된 위치 정보 발행"""
        if self.current_body_pose is None:
            return

        # FasterLIO → UTM Local 변환
        rel_x = self.current_body_pose["x"] - self.fasterlio_origin["x"]
        rel_y = self.current_body_pose["y"] - self.fasterlio_origin["y"]

        # Heading 보정 적용
        corrected_x, corrected_y = self.apply_heading_correction(rel_x, rel_y)

        # Orientation 보정
        corrected_qx, corrected_qy, corrected_qz, corrected_qw = self.apply_heading_correction_to_orientation(
            self.current_body_pose["qx"], self.current_body_pose["qy"],
            self.current_body_pose["qz"], self.current_body_pose["qw"]
        )

        # 현재 위치 업데이트
        self.current_pose_local = {
            "x": corrected_x,
            "y": corrected_y,
            "z": self.current_body_pose["z"],
            "qx": corrected_qx,
            "qy": corrected_qy,
            "qz": corrected_qz,
            "qw": corrected_qw,
            "timestamp": self.current_body_pose["timestamp"]
        }

        # 보정된 궤적 기록
        if not self.corrected_trajectory_local or self.distance_check_local(self.current_pose_local, self.corrected_trajectory_local[-1], 0.2):
            self.corrected_trajectory_local.append(self.current_pose_local.copy())

        # 불확실성 업데이트
        uncertainty = 2.0 if self.correction_system["initial_alignment_done"] else 10.0
        self.pose_covariance[0,0] = uncertainty
        self.pose_covariance[1,1] = uncertainty

    def apply_heading_correction_to_orientation(self, qx, qy, qz, qw):
        """Orientation에 heading 보정 적용"""
        if not self.correction_system["initial_alignment_done"]:
            return qx, qy, qz, qw
        
        roll, pitch, yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        corrected_yaw = self.normalize_angle(yaw + self.correction_system["heading_correction"])
        return self.quaternion_from_euler(roll, pitch, corrected_yaw)

    def publish_current_pose(self, event):
        """현재 위치 발행"""
        if self.current_pose_local is None:
            return
        
        current_time = rospy.Time.now()
        
        # PoseWithCovarianceStamped 발행
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
        
        # Odometry 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "utm_local"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = pose_msg.pose
        
        self.odom_pub.publish(odom_msg)

    def broadcast_dynamic_tf(self, event):
        """🔥 완전한 TF tree 구성"""
        if self.current_pose_local is None:
            return
        
        current_time = rospy.Time.now()
        transforms = []
        
        # 1. utm_local → odom (메인 변환)
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
        
        # 2. odom → base_link
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = current_time
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_link"
        odom_to_base.transform.rotation.w = 1.0
        transforms.append(odom_to_base)
        
        # 3. base_link → 센서 프레임들
        # base_link → os_sensor
        base_to_os_sensor = TransformStamped()
        base_to_os_sensor.header.stamp = current_time
        base_to_os_sensor.header.frame_id = "base_link"
        base_to_os_sensor.child_frame_id = "os_sensor"
        base_to_os_sensor.transform.translation.z = 0.3  # 센서 높이
        base_to_os_sensor.transform.rotation.w = 1.0
        transforms.append(base_to_os_sensor)
        
        # os_sensor → os1_lidar
        os_sensor_to_lidar = TransformStamped()
        os_sensor_to_lidar.header.stamp = current_time
        os_sensor_to_lidar.header.frame_id = "os_sensor"
        os_sensor_to_lidar.child_frame_id = "os1_lidar"
        os_sensor_to_lidar.transform.rotation.w = 1.0
        transforms.append(os_sensor_to_lidar)
        
        # os_sensor → os1_imu
        os_sensor_to_imu = TransformStamped()
        os_sensor_to_imu.header.stamp = current_time
        os_sensor_to_imu.header.frame_id = "os_sensor"
        os_sensor_to_imu.child_frame_id = "os1_imu"
        os_sensor_to_imu.transform.rotation.w = 1.0
        transforms.append(os_sensor_to_imu)
        
        # 4. base_link → 휠 프레임들
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

    def publish_uncertainty(self, event):
        """위치 불확실성 시각화"""
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

    # 유틸리티 함수들
    def calculate_trajectory_heading(self, trajectory):
        """궤적에서 heading 계산"""
        if len(trajectory) < 2:
            return None
        
        max_distance = 0
        best_heading = None
        
        for i in range(len(trajectory)):
            for j in range(i + 1, len(trajectory)):
                p1, p2 = trajectory[i], trajectory[j]
                distance = math.sqrt((p2["x"] - p1["x"])**2 + (p2["y"] - p1["y"])**2)
                
                if distance > max_distance and distance >= 1.0:
                    max_distance = distance
                    best_heading = math.atan2(p2["y"] - p1["y"], p2["x"] - p1["x"])
        
        return best_heading

    def update_distance(self, new_position):
        """이동 거리 업데이트"""
        if self.last_position is not None:
            dx = new_position["x"] - self.last_position["x"]
            dy = new_position["y"] - self.last_position["y"]
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance += distance
        
        self.last_position = new_position.copy()

    def distance_check_local(self, p1, p2, threshold):
        """거리 체크"""
        return math.sqrt((p1["x"] - p2["x"])**2 + (p1["y"] - p2["y"])**2) > threshold

    def normalize_angle(self, angle):
        """각도 정규화"""
        while angle > math.pi: 
            angle -= 2 * math.pi
        while angle < -math.pi: 
            angle += 2 * math.pi
        return angle

    def euler_from_quaternion(self, x, y, z, w):
        """Quaternion → Euler 변환"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Euler → Quaternion 변환"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return x, y, z, w

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        pi = PoseInitializer()
        pi.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 PoseInitializer 종료")
    except Exception as e:
        rospy.logerr(f"❌ PoseInitializer 오류: {e}")