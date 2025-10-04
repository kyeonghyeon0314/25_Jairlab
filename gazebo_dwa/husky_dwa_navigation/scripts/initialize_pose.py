#!/usr/bin/env python3
import rospy
import utm
import json
import math
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from visualization_msgs.msg import Marker
import tf2_ros

class PoseInitializer:
    """FasterLIO-GPS 융합 위치 추정 및 Heading 보정"""
    
    def __init__(self):
        rospy.init_node('initialize_pose_node', anonymous=True)

        # 🌟 UTM 절대좌표 원점 관리 (Kakao Map 연동용)
        self.utm_origin_absolute = None  # 실제 UTM 절대 좌표 저장
        self.utm_zone = None
        self.origin_synced = False

        # 웹 인터페이스용 실시간 GPS 데이터
        self.current_gps = None

        # FasterLIO 관리
        self.fasterlio_origin = None
        self.current_body_pose = None
        self.last_good_gps = None

        # 궤적 기록 (UTM Local 좌표)
        self.fasterlio_trajectory_local = []
        self.gps_trajectory_local = []
        self.corrected_trajectory_local = []

        # 🔥 개선된 Heading 보정 시스템 (move_front 패턴 기반)
        # ⚠️ 수동 설정 필요: move_front.py의 파라미터와 일치시켜야 함
        # move_front.py: acceleration_time, constant_speed_time, deceleration_time 확인 후 수정
        self.MOVE_FRONT_TIMING = {
            "acceleration": 3.0,    # 🔧 수동 설정: move_front.py의 acceleration_time과 일치
            "constant": 4.0,        # 🔧 수동 설정: move_front.py의 constant_speed_time과 일치  
            "deceleration": 2.0     # 🔧 수동 설정: move_front.py의 deceleration_time과 일치
        }
        
        self.correction_system = {
            "heading_correction": 0.0,
            "initial_alignment_done": False,
            "move_front_detected": False,
            "move_front_start_time": None,
            "move_front_completed": False,
            "movement_phases": self.MOVE_FRONT_TIMING.copy()  # 위의 설정값 사용
        }
        # ❌ last_correction_time 제거됨 - 점진적 보정 미사용
        
        # 🚀 개선 1: 움직임 감지 시스템 (정지 상태 데이터 무시)
        self.motion_detector = {
            "is_moving": False,
            "last_position": None,
            "stationary_threshold": 0.05,  # 5cm 이하 움직임은 정지로 간주
            "movement_start_time": None
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
        self.utm_origin_pub = rospy.Publisher("/utm_origin_info", String, queue_size=1, latch=True)
        self.gps_data_pub = rospy.Publisher("/gps_data", String, queue_size=10)  # 웹 전송용
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribers
        rospy.Subscriber("/utm_origin_info", String, self.utm_origin_callback)
        rospy.Subscriber("/Odometry", Odometry, self.fasterlio_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist, self.cmd_vel_callback)
        
        # Timers
        rospy.Timer(rospy.Duration(0.1), self.publish_current_pose)
        rospy.Timer(rospy.Duration(0.1), self.broadcast_dynamic_tf)
        rospy.Timer(rospy.Duration(0.5), self.publish_uncertainty)
        rospy.Timer(rospy.Duration(1.0), self.check_move_front_pattern)
        rospy.Timer(rospy.Duration(1.0), self.publish_gps_data)  # 웹용 GPS 데이터
        # ❌ 점진적 보정 타이머 제거됨 - move_front 기반 1회 보정만 사용

        rospy.loginfo("🚀 PoseInitializer 시작 - FasterLIO-GPS 융합 위치 추정")
        rospy.loginfo("   ⏳ UTM 원점 대기 (gps_server에서 설정)")
        rospy.loginfo("   ✅ move_front 패턴 기반 정밀 헤딩 보정")
        rospy.loginfo("   ✅ 웹 인터페이스용 GPS 데이터 발행")
        rospy.loginfo("   ✅ TF 구조: map → odom → base_link")
        rospy.loginfo(f"   🚗 설정된 move_front 타이밍:")
        rospy.loginfo(f"      가속: {self.MOVE_FRONT_TIMING['acceleration']}초")
        rospy.loginfo(f"      등속: {self.MOVE_FRONT_TIMING['constant']}초")
        rospy.loginfo(f"      감속: {self.MOVE_FRONT_TIMING['deceleration']}초")
        rospy.loginfo(f"      총 시간: {sum(self.MOVE_FRONT_TIMING.values())}초")
        rospy.logwarn("⚠️  move_front.py 파라미터 변경 시 위 값들도 수동 수정 필요!")

    def get_move_front_total_time(self):
        """move_front 전체 실행 시간 계산"""
        return sum(self.MOVE_FRONT_TIMING.values())
    
    def get_move_front_phase_boundaries(self):
        """move_front 각 단계의 시간 경계 계산"""
        accel_end = self.MOVE_FRONT_TIMING["acceleration"]
        const_end = accel_end + self.MOVE_FRONT_TIMING["constant"]
        decel_end = const_end + self.MOVE_FRONT_TIMING["deceleration"]
        
        return {
            "acceleration_end": accel_end,
            "constant_end": const_end,
            "deceleration_end": decel_end,
            "total_time": decel_end
        }
    
    def get_current_move_front_phase(self, elapsed_time):
        """현재 경과 시간에 따른 move_front 단계 반환"""
        boundaries = self.get_move_front_phase_boundaries()
        
        if elapsed_time <= boundaries["acceleration_end"]:
            return "acceleration"
        elif elapsed_time <= boundaries["constant_end"]:
            return "constant"
        elif elapsed_time <= boundaries["deceleration_end"]:
            return "deceleration"
        else:
            return "completed"

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
        """FasterLIO 콜백 - 메인 위치 추정 로직 (움직일 때만)"""
        if not self.origin_synced:
            return

        timestamp = msg.header.stamp.to_sec()
        
        # FasterLIO 원시 pose 저장
        current_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "qx": msg.pose.pose.orientation.x,
            "qy": msg.pose.pose.orientation.y,
            "qz": msg.pose.pose.orientation.z,
            "qw": msg.pose.pose.orientation.w,
            "timestamp": timestamp
        }
        
        # 🚀 개선: 움직임 감지 업데이트
        self.update_motion_detection(current_pose)
        
        # 🚀 개선: 움직일 때만 데이터 처리
        if not self.motion_detector["is_moving"]:
            return
        
        self.current_body_pose = current_pose
        
        # 첫 번째 포즈면 기준점 설정 (움직임 시작 후)
        if self.fasterlio_origin is None:
            self.fasterlio_origin = self.current_body_pose.copy()
            rospy.loginfo("🎯 FasterLIO 기준점 설정 완료 (움직임 감지 후)")

        # 궤적 처리 및 보정 수행
        self.process_trajectories()
        self.publish_fused_pose()

    def cmd_vel_callback(self, msg):
        """🚗 move_front 패턴 감지"""
        # 🚀 개선: 헤딩 보정 완료 후 로그 중단
        if self.correction_system["initial_alignment_done"]:
            return
            
        # 직진 움직임 감지 (angular.z가 거의 0이고 linear.x > 0)
        is_forward_motion = (msg.linear.x > 0.1 and abs(msg.angular.z) < 0.05)
        
        if is_forward_motion and not self.correction_system["move_front_detected"]:
            self.correction_system["move_front_detected"] = True
            self.correction_system["move_front_start_time"] = rospy.Time.now()
            rospy.loginfo("🚗 move_front 패턴 감지 시작 - 헤딩 보정 준비")
        
        elif not is_forward_motion and self.correction_system["move_front_detected"]:
            # 직진 움직임 종료
            elapsed = (rospy.Time.now() - self.correction_system["move_front_start_time"]).to_sec()
            total_expected = sum(self.correction_system["movement_phases"].values())
            
            if elapsed >= total_expected * 0.8:  # 80% 이상 완료되었다면
                rospy.loginfo("🚗 move_front 패턴 완료 감지 - 헤딩 보정 대기")
                self.correction_system["move_front_completed"] = True
            
            self.correction_system["move_front_detected"] = False

    def gps_callback(self, msg):
        """GPS 콜백 - 궤적 기록만 (원점 설정은 gps_server 담당)"""
        if msg.status.status < 0:
            return

        # 🌐 웹 인터페이스용 실시간 GPS 데이터 업데이트
        self.current_gps = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "status": msg.status.status
        }

        # UTM 원점 대기
        if not self.origin_synced:
            rospy.loginfo_throttle(10, "⏳ UTM 원점 대기 중 (gps_server에서 설정 필요)")
            return

        # 🚀 움직일 때만 GPS 궤적 처리
        if not self.motion_detector["is_moving"]:
            return

        timestamp = msg.header.stamp.to_sec()
        gps_utm_x, gps_utm_y = self.gps_to_utm_absolute(msg.latitude, msg.longitude)

        # UTM 원점 기준 상대좌표 계산 (내부 처리용)
        gps_local_x = gps_utm_x - self.utm_origin_absolute["easting"]
        gps_local_y = gps_utm_y - self.utm_origin_absolute["northing"]

        self.last_good_gps = {
            "x": gps_local_x,
            "y": gps_local_y,
            "timestamp": timestamp,
            "lat": msg.latitude,
            "lon": msg.longitude,
            "status": msg.status.status  # GPS 품질 정보 저장
        }

        # GPS 궤적 기록
        if not self.gps_trajectory_local or self.distance_check_local(self.last_good_gps, self.gps_trajectory_local[-1], 0.3):
            self.gps_trajectory_local.append(self.last_good_gps.copy())
            rospy.loginfo_throttle(5, f"📡 GPS 궤적 업데이트: UTM({gps_utm_x:.1f}, {gps_utm_y:.1f}) Local({gps_local_x:.1f}, {gps_local_y:.1f}) 품질={msg.status.status}")

    def gps_to_utm_absolute(self, lat, lon):
        """GPS → UTM 절대좌표 변환"""
        if abs(lat) < 0.01 and abs(lon) < 0.01:
            easting = lat * 111320.0
            northing = lon * 111320.0
        else:
            easting, northing, _, _ = utm.from_latlon(lat, lon)

        return easting, northing

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

        # 🚀 move_front 완료 후에만 헤딩 보정 수행 (1회만)
        if (not self.correction_system["initial_alignment_done"] and
            self.correction_system["move_front_completed"]):
            rospy.loginfo("🚗 move_front 완료 감지 → 최종 헤딩 보정 수행")
            self.perform_move_front_final_correction()

        # ❌ 일반 초기 정렬 제거됨 - move_front 패턴만 사용

    # ❌ perform_initial_heading_alignment() 제거됨
    # 오직 move_front 기반 보정만 사용

    def check_move_front_pattern(self, _):
        """🚗 move_front 패턴 모니터링"""
        # 🚀 개선: 헤딩 보정 완료 후 로그 중단
        if self.correction_system["initial_alignment_done"]:
            return
            
        if not self.correction_system["move_front_detected"]:
            return
            
        if self.correction_system["move_front_start_time"] is None:
            return
            
        elapsed = (rospy.Time.now() - self.correction_system["move_front_start_time"]).to_sec()
        total_expected = self.get_move_front_total_time()
        
        # 현재 단계 계산 (새로운 헬퍼 함수 사용)
        current_phase = self.get_current_move_front_phase(elapsed)
        
        phase_names = {
            "acceleration": "가속",
            "constant": "등속", 
            "deceleration": "감속",
            "completed": "완료"
        }
        
        phase_display = phase_names.get(current_phase, "알 수 없음")
        rospy.loginfo_throttle(2, f"🚗 move_front 진행: {phase_display} 단계 ({elapsed:.1f}s/{total_expected:.1f}s)")




    # ❌ 점진적 보정 함수 제거됨 - move_front 기반 1회 보정만 사용

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

    def publish_current_pose(self, _):
        """현재 위치 발행 (UTM 절대좌표)"""
        if self.current_pose_local is None or not self.origin_synced:
            return

        current_time = rospy.Time.now()

        # UTM 절대 좌표 계산
        utm_abs_x = self.utm_origin_absolute["easting"] + self.current_pose_local["x"]
        utm_abs_y = self.utm_origin_absolute["northing"] + self.current_pose_local["y"]

        # PoseWithCovarianceStamped 발행
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "map"  # UTM 절대좌표계

        pose_msg.pose.pose.position.x = utm_abs_x
        pose_msg.pose.pose.position.y = utm_abs_y
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
        odom_msg.header.frame_id = "map"  # UTM 절대좌표계
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = pose_msg.pose

        self.odom_pub.publish(odom_msg)

    def broadcast_dynamic_tf(self, _):
        """🔥 TF tree 구성: odom → base_link → sensors (map→odom은 gps_server가 담당)"""
        if not self.origin_synced:
            rospy.loginfo_throttle(10, "⏳ TF 발행 대기: UTM 원점 미설정")
            return

        if self.current_pose_local is None:
            rospy.loginfo_throttle(10, "⏳ TF 발행 대기: FasterLIO 데이터 미수신")
            return

        current_time = rospy.Time.now()
        transforms = []

        # ❌ map → odom 제거: gps_server에서 Static TF로 발행

        # 1. odom → base_link (Dynamic, 로봇의 UTM 절대 위치)
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = current_time
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_link"
        # UTM 절대 좌표 = 원점 + 보정된 상대 위치
        odom_to_base.transform.translation.x = self.utm_origin_absolute["easting"] + self.current_pose_local["x"]
        odom_to_base.transform.translation.y = self.utm_origin_absolute["northing"] + self.current_pose_local["y"]
        odom_to_base.transform.translation.z = self.current_pose_local["z"]
        odom_to_base.transform.rotation.x = self.current_pose_local["qx"]
        odom_to_base.transform.rotation.y = self.current_pose_local["qy"]
        odom_to_base.transform.rotation.z = self.current_pose_local["qz"]
        odom_to_base.transform.rotation.w = self.current_pose_local["qw"]
        transforms.append(odom_to_base)

        # 2. base_link → 센서 프레임들
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

        # 3. base_link → 휠 프레임들
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

    def publish_uncertainty(self, _):
        """위치 불확실성 시각화"""
        if self.current_pose_local is None or not self.origin_synced:
            return

        uncertainty = math.sqrt(self.pose_covariance[0,0])

        # UTM 절대 좌표 계산
        utm_abs_x = self.utm_origin_absolute["easting"] + self.current_pose_local["x"]
        utm_abs_y = self.utm_origin_absolute["northing"] + self.current_pose_local["y"]

        marker = Marker()
        marker.header.frame_id = "map"  # UTM 절대좌표계
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pose_uncertainty"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = utm_abs_x
        marker.pose.position.y = utm_abs_y
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

    def publish_gps_data(self, _):
        """실시간 GPS 데이터 발행 (웹 인터페이스용)"""
        if self.current_gps:
            self.gps_data_pub.publish(json.dumps(self.current_gps))
            rospy.loginfo_throttle(10, f"📡 실시간 GPS → 웹: ({self.current_gps['latitude']:.6f}, {self.current_gps['longitude']:.6f})")
        elif self.utm_origin_absolute:
            # GPS가 없으면 원점 정보라도 전송
            fallback_gps = {
                "latitude": self.utm_origin_absolute["lat"],
                "longitude": self.utm_origin_absolute["lon"],
                "altitude": 0.0
            }
            self.gps_data_pub.publish(json.dumps(fallback_gps))
            rospy.loginfo_throttle(20, f"📡 Fallback GPS → 웹: ({fallback_gps['latitude']:.6f}, {fallback_gps['longitude']:.6f})")

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

    def update_motion_detection(self, current_pose):
        """🚀 움직임 감지 시스템 - 정지 상태 데이터 무시"""
        if self.motion_detector["last_position"] is None:
            self.motion_detector["last_position"] = current_pose.copy()
            return
        
        # 이전 위치와의 거리 계산
        dx = current_pose["x"] - self.motion_detector["last_position"]["x"]
        dy = current_pose["y"] - self.motion_detector["last_position"]["y"]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 움직임 상태 업데이트
        if distance > self.motion_detector["stationary_threshold"]:
            if not self.motion_detector["is_moving"]:
                self.motion_detector["is_moving"] = True
                self.motion_detector["movement_start_time"] = rospy.Time.now()
                rospy.loginfo(f"🏃 움직임 감지! 거리: {distance:.3f}m")
            
            self.motion_detector["last_position"] = current_pose.copy()
        # else: 정지 상태 유지
    
    def perform_move_front_final_correction(self):
        """🚗 move_front 완료 후 최종 헤딩 보정 (정밀 버전)"""
        if self.correction_system["initial_alignment_done"]:
            rospy.loginfo("✅ 이미 헤딩 보정 완료됨 - move_front 보정 무시")
            return

        # move_front 시작 이후의 궤적 데이터 필터링
        if self.correction_system["move_front_start_time"] is None:
            rospy.logwarn("❌ move_front 시작 시간 누락 - 헤딩 보정 실패")
            rospy.logwarn("   move_front 패턴을 먼저 실행해야 합니다!")
            return

        move_start_time = self.correction_system["move_front_start_time"].to_sec()

        # move_front 시작 이후의 FasterLIO 궤적 필터링
        move_fasterlio = [p for p in self.fasterlio_trajectory_local
                         if p["timestamp"] >= move_start_time]

        # move_front 시작 이후의 GPS 궤적 필터링 (고품질만)
        MIN_GPS_QUALITY = 0  # RTK Float (1) 이상
        move_gps = [p for p in self.gps_trajectory_local
                   if p["timestamp"] >= move_start_time and p.get("status", 0) >= MIN_GPS_QUALITY]

        if len(move_fasterlio) < 10 or len(move_gps) < 10:
            rospy.logwarn(f"❌ move_front 데이터 부족: FLio={len(move_fasterlio)}, GPS(고품질)={len(move_gps)}")
            rospy.logwarn("   더 긴 move_front 패턴이 필요합니다!")
            return

        # 시작점과 끝점으로 전체 방향 계산 (충분한 샘플 확보)
        flio_start, flio_end = move_fasterlio[0], move_fasterlio[-1]
        gps_start, gps_end = move_gps[0], move_gps[-1]

        total_flio_distance = math.sqrt((flio_end["x"] - flio_start["x"])**2 +
                                       (flio_end["y"] - flio_start["y"])**2)
        total_gps_distance = math.sqrt((gps_end["x"] - gps_start["x"])**2 +
                                      (gps_end["y"] - gps_start["y"])**2)

        # 정밀도 향상: 최소 5m 이동 요구
        MIN_CALIBRATION_DISTANCE = 5.0
        if total_flio_distance < MIN_CALIBRATION_DISTANCE or total_gps_distance < MIN_CALIBRATION_DISTANCE:
            rospy.logwarn(f"⚠️ move_front 이동거리 부족 (최소 {MIN_CALIBRATION_DISTANCE}m 필요)")
            rospy.logwarn(f"   현재: FLio={total_flio_distance:.1f}m, GPS={total_gps_distance:.1f}m")
            return

        # GPS 품질 추가 검증
        avg_gps_quality = sum(p.get("status", 0) for p in move_gps) / len(move_gps)
        rospy.loginfo(f"📡 GPS 품질: 평균 상태={avg_gps_quality:.2f}, 샘플 수={len(move_gps)}")

        # 정밀 헤딩 계산
        fasterlio_heading = math.atan2(flio_end["y"] - flio_start["y"],
                                      flio_end["x"] - flio_start["x"])
        gps_heading = math.atan2(gps_end["y"] - gps_start["y"],
                                gps_end["x"] - gps_start["x"])

        angle_diff = self.normalize_angle(gps_heading - fasterlio_heading)
        self.correction_system["heading_correction"] = angle_diff
        self.correction_system["initial_alignment_done"] = True

        rospy.loginfo("🏆 move_front 완료 기반 정밀 헤딩 보정 완료!")
        rospy.loginfo(f"   전체 이동거리: FLio={total_flio_distance:.1f}m, GPS={total_gps_distance:.1f}m")
        rospy.loginfo(f"   사용된 샘플: FLio={len(move_fasterlio)}개, GPS(고품질)={len(move_gps)}개")
        rospy.loginfo(f"   평균 GPS 품질: {avg_gps_quality:.2f}")
        rospy.loginfo(f"   FasterLIO 방향: {math.degrees(fasterlio_heading):.2f}도")
        rospy.loginfo(f"   GPS 방향: {math.degrees(gps_heading):.2f}도")
        rospy.loginfo(f"   최종 보정값: {math.degrees(angle_diff):.2f}도")

        self.recalculate_all_trajectories()

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