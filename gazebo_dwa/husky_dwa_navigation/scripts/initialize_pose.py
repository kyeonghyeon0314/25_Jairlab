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

        # 🌟 개선된 UTM 좌표계 관리 (여러 GPS 기반)
        self.utm_origin_absolute = None
        self.utm_zone = None
        self.origin_synced = False
        
        # 🎯 GPS 신뢰도 기반 원점 설정 시스템
        self.gps_candidates = []  # GPS 후보들 저장
        
        # 🔧 GPS 정확도 설정 (수동 조정 가능)
        # ⚡ 빠른 설정: duration=2.0, samples=20, threshold=0.5 
        # ⚖️ 균형 설정: duration=3.0, samples=40, threshold=0.4
        # 🎯 정밀 설정: duration=5.0, samples=80, threshold=0.3
        self.gps_collection_duration = 3.0  # GPS 수집 시간 (초) - 균형 설정
        self.gps_collection_start_time = None
        self.min_gps_samples = 40           # 최소 GPS 샘플 수 (20Hz * 2초 = 40개)
        self.gps_quality_threshold = 0.4    # GPS 품질 임계값 (미터) - 40cm
        
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
            "last_correction_time": 0.0,
            "move_front_detected": False,
            "move_front_start_time": None,
            "move_front_completed": False,
            "movement_phases": self.MOVE_FRONT_TIMING.copy()  # 위의 설정값 사용
        }
        
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
        self.utm_origin_pub = rospy.Publisher("/utm_origin_info", String, queue_size=1)
        
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

        rospy.loginfo("🚀 PoseInitializer 시작 - 개선된 FasterLIO-GPS 융합 위치 추정")
        rospy.loginfo("   ✅ 신뢰도 기반 GPS 원점 설정")
        rospy.loginfo("   ✅ move_front 패턴 기반 헤딩 보정")
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

    def collect_gps_candidates(self, gps_msg):
        """🌟 GPS 후보 수집 및 신뢰도 평가 (움직일 때만)"""
        if gps_msg.status.status < 0:
            return False
        
        # 🚀 개선: 움직일 때만 GPS 후보 수집
        if not self.motion_detector["is_moving"]:
            return False
            
        if self.gps_collection_start_time is None:
            self.gps_collection_start_time = rospy.Time.now()
            rospy.loginfo("🔄 GPS 후보 수집 시작 (움직임 감지 후 신뢰도 기반 원점 설정)")
        
        # GPS 품질 평가 메트릭
        quality_score = self.evaluate_gps_quality(gps_msg)
        
        candidate = {
            "lat": gps_msg.latitude,
            "lon": gps_msg.longitude,
            "quality": quality_score,
            "hdop": getattr(gps_msg.position_covariance, 'hdop', 1.0) if hasattr(gps_msg, 'position_covariance') else 1.0,
            "timestamp": gps_msg.header.stamp.to_sec(),
            "status": gps_msg.status.status
        }
        
        self.gps_candidates.append(candidate)
        
        # 수집 완료 조건 확인
        elapsed_time = (rospy.Time.now() - self.gps_collection_start_time).to_sec()
        
        if (len(self.gps_candidates) >= self.min_gps_samples and 
            elapsed_time >= self.gps_collection_duration):
            
            return self.finalize_utm_origin()
            
        rospy.loginfo_throttle(1, f"📡 GPS 후보 수집 중: {len(self.gps_candidates)}개 수집됨")
        return False
    
    def evaluate_gps_quality(self, gps_msg):
        """GPS 품질 평가 (높을수록 좋음)"""
        # 기본 점수
        base_score = 1.0
        
        # GPS 상태별 점수 조정
        if gps_msg.status.status == 2:  # RTK Fixed
            base_score = 10.0
        elif gps_msg.status.status == 1:  # RTK Float  
            base_score = 5.0
        elif gps_msg.status.status == 0:  # Standard GPS
            base_score = 2.0
        
        # Position covariance가 있다면 추가 평가
        if hasattr(gps_msg, 'position_covariance') and len(gps_msg.position_covariance) >= 9:
            cov_xx = gps_msg.position_covariance[0]
            cov_yy = gps_msg.position_covariance[4]
            position_uncertainty = math.sqrt(cov_xx + cov_yy)
            
            # 불확실성이 낮을수록 높은 점수
            uncertainty_factor = max(0.1, 1.0 / (1.0 + position_uncertainty))
            base_score *= uncertainty_factor
        
        return base_score
    
    def finalize_utm_origin(self):
        """🎯 최적 GPS 후보 선택 및 UTM 원점 설정"""
        if not self.gps_candidates:
            rospy.logwarn("❌ GPS 후보가 없어 원점 설정 실패")
            return False
        
        # 품질 순으로 정렬
        sorted_candidates = sorted(self.gps_candidates, key=lambda x: x['quality'], reverse=True)
        
        # 상위 후보들의 클러스터링 기반 평균 계산
        best_candidates = self.select_best_cluster(sorted_candidates)
        
        if not best_candidates:
            rospy.logwarn("❌ 신뢰할 만한 GPS 클러스터를 찾을 수 없음")
            return False
        
        # 가중 평균 계산
        total_weight = sum(candidate['quality'] for candidate in best_candidates)
        weighted_lat = sum(c['lat'] * c['quality'] for c in best_candidates) / total_weight
        weighted_lon = sum(c['lon'] * c['quality'] for c in best_candidates) / total_weight
        
        # 표준편차 계산
        lat_std = math.sqrt(sum((c['lat'] - weighted_lat)**2 for c in best_candidates) / len(best_candidates))
        lon_std = math.sqrt(sum((c['lon'] - weighted_lon)**2 for c in best_candidates) / len(best_candidates))
        
        rospy.loginfo("🎯 신뢰도 기반 UTM 원점 설정!")
        rospy.loginfo(f"   📊 분석된 GPS 후보: {len(self.gps_candidates)}개")
        rospy.loginfo(f"   🏆 선택된 클러스터: {len(best_candidates)}개")
        rospy.loginfo(f"   📍 최종 좌표: ({weighted_lat:.8f}, {weighted_lon:.8f})")
        rospy.loginfo(f"   📏 표준편차: lat={lat_std*111320:.2f}m, lon={lon_std*111320:.2f}m")
        
        return self.set_utm_origin(weighted_lat, weighted_lon)
    
    def select_best_cluster(self, sorted_candidates):
        """GPS 후보들을 클러스터링하여 최적 그룹 선택"""
        if not sorted_candidates:
            return []
        
        # 최고 품질 후보를 기준으로 클러스터링
        reference = sorted_candidates[0]
        cluster = [reference]
        
        for candidate in sorted_candidates[1:]:
            # 거리 계산 (미터 단위)
            lat_dist = (candidate['lat'] - reference['lat']) * 111320
            lon_dist = (candidate['lon'] - reference['lon']) * 111320
            distance = math.sqrt(lat_dist**2 + lon_dist**2)
            
            # 임계값 내에 있는 후보만 클러스터에 포함
            if distance <= self.gps_quality_threshold:
                cluster.append(candidate)
        
        # 최소 절반 이상의 후보가 클러스터에 포함되어야 신뢰성 인정
        min_cluster_size = max(3, len(sorted_candidates) // 3)
        
        if len(cluster) >= min_cluster_size:
            return cluster
        else:
            # 클러스터가 너무 작으면 상위 30% 후보만 사용
            top_30_percent = max(3, len(sorted_candidates) * 3 // 10)
            return sorted_candidates[:top_30_percent]

    def set_utm_origin(self, lat, lon):
        """첫 번째 GPS 위치를 UTM 원점으로 설정"""
        try:
            if abs(lat) < 0.01 and abs(lon) < 0.01:
                # 시뮬레이션 GPS 처리
                easting = lat * 111320.0
                northing = lon * 111320.0
                utm_zone = "simulation"
            else:
                easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
                utm_zone = f"{zone_number}{zone_letter}"
            
            self.utm_origin_absolute = {
                "easting": easting,
                "northing": northing,
                "lat": lat,
                "lon": lon
            }
            self.utm_zone = utm_zone
            self.origin_synced = True
            
            # UTM 원점 정보 발행
            origin_data = {
                "utm_origin_absolute": self.utm_origin_absolute,
                "utm_zone": utm_zone,
                "timestamp": rospy.Time.now().to_sec()
            }
            
            self.utm_origin_pub.publish(String(data=json.dumps(origin_data)))
            
            rospy.loginfo(f"🎯 UTM 원점 설정 완료!")
            rospy.loginfo(f"   GPS: ({lat:.6f}, {lon:.6f})")
            rospy.loginfo(f"   UTM: ({easting:.1f}, {northing:.1f})")
            rospy.loginfo(f"   Zone: {utm_zone}")
            rospy.loginfo(f"   로봇 위치 = UTM Local (0, 0)")
            
        except Exception as e:
            rospy.logerr(f"❌ UTM 원점 설정 실패: {e}")

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
        """개선된 GPS 콜백 - 신뢰도 기반 원점 설정 (움직일 때만)"""
        if msg.status.status < 0:
            return
            
        # UTM 원점이 설정되지 않았다면 GPS 후보 수집
        if not self.origin_synced:
            if self.collect_gps_candidates(msg):
                rospy.loginfo("✅ GPS 기반 UTM 원점 설정 완료!")
            return
        
        # 🚀 개선: 움직일 때만 GPS 궤적 처리
        if not self.motion_detector["is_moving"]:
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

        # 🚀 개선: move_front 완료 후에만 헤딩 보정 수행
        if (not self.correction_system["initial_alignment_done"] and 
            self.correction_system["move_front_completed"]):
            rospy.loginfo("🚗 move_front 완료 감지 → 최종 헤딩 보정 수행")
            self.perform_move_front_final_correction()
        
        # 일반적인 초기 정렬 체크 (move_front 없이 움직인 경우)
        elif (not self.correction_system["initial_alignment_done"] and 
              not self.correction_system["move_front_detected"] and
              (self.total_distance >= 2.0 or 
               (len(self.fasterlio_trajectory_local) >= 5 and len(self.gps_trajectory_local) >= 5))):
            rospy.loginfo(f"📏 일반 초기 정렬 조건 충족 → Heading 정렬 수행")
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




    def check_gradual_heading_correction(self, _):
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

    def publish_current_pose(self, _):
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

    def broadcast_dynamic_tf(self, _):
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

    def publish_uncertainty(self, _):
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
        """🚗 move_front 완료 후 최종 헤딩 보정"""
        if self.correction_system["initial_alignment_done"]:
            rospy.loginfo("✅ 이미 헤딩 보정 완료됨 - move_front 보정 무시")
            return
            
        # move_front 시작 이후의 궤적 데이터 필터링
        if self.correction_system["move_front_start_time"] is None:
            rospy.logwarn("⚠️ move_front 시작 시간 누락 - 기본 보정 방법 사용")
            self.perform_initial_heading_alignment()
            return
            
        move_start_time = self.correction_system["move_front_start_time"].to_sec()
        
        # move_front 시작 이후의 FasterLIO 궤적 필터링
        move_fasterlio = [p for p in self.fasterlio_trajectory_local 
                         if p["timestamp"] >= move_start_time]
        
        # move_front 시작 이후의 GPS 궤적 필터링  
        move_gps = [p for p in self.gps_trajectory_local 
                   if p["timestamp"] >= move_start_time]
        
        if len(move_fasterlio) < 5 or len(move_gps) < 5:
            rospy.logwarn(f"⚠️ move_front 데이터 부족: FLio={len(move_fasterlio)}, GPS={len(move_gps)} - 기본 보정 시도")
            self.perform_initial_heading_alignment()
            return
            
        # 시작점과 끝점으로 전체 방향 계산 (충분한 샘플 확보)
        flio_start, flio_end = move_fasterlio[0], move_fasterlio[-1]
        gps_start, gps_end = move_gps[0], move_gps[-1]
        
        total_flio_distance = math.sqrt((flio_end["x"] - flio_start["x"])**2 + 
                                       (flio_end["y"] - flio_start["y"])**2)
        total_gps_distance = math.sqrt((gps_end["x"] - gps_start["x"])**2 + 
                                      (gps_end["y"] - gps_start["y"])**2)
        
        if total_flio_distance < 3.0 or total_gps_distance < 3.0:
            rospy.logwarn(f"⚠️ move_front 이동거리 부족: FLio={total_flio_distance:.1f}m, GPS={total_gps_distance:.1f}m")
            return
        
        # 정밀 헤딩 계산
        fasterlio_heading = math.atan2(flio_end["y"] - flio_start["y"], 
                                      flio_end["x"] - flio_start["x"])
        gps_heading = math.atan2(gps_end["y"] - gps_start["y"], 
                                gps_end["x"] - gps_start["x"])
        
        angle_diff = self.normalize_angle(gps_heading - fasterlio_heading)
        self.correction_system["heading_correction"] = angle_diff
        self.correction_system["initial_alignment_done"] = True
        
        rospy.loginfo("🏆 move_front 완료 기반 최종 헤딩 보정 완료!")
        rospy.loginfo(f"   전체 이동거리: FLio={total_flio_distance:.1f}m, GPS={total_gps_distance:.1f}m")
        rospy.loginfo(f"   사용된 샘플: FLio={len(move_fasterlio)}개, GPS={len(move_gps)}개")
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