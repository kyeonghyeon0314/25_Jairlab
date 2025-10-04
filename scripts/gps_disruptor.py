#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GPS 신호 불안정화 시뮬레이터
실제 GPS 토픽(/ublox/fix)을 구독하여 다양한 불안정 패턴을 적용 후 재발행
"""

import rospy
import random
import numpy as np
import threading
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json

class GPSDisruptor:
    def __init__(self):
        rospy.init_node('gps_disruptor', anonymous=False)

        # 파라미터 로드
        self.disruption_mode = rospy.get_param('~disruption_mode', 'none')  # none, blackout, degradation, drift, multipath, mixed
        self.intensity = rospy.get_param('~intensity', 'medium')  # low, medium, high
        self.enable_disruption = rospy.get_param('~enable', True)

        # 상태 변수
        self.latest_gps = None
        self.gps_lock = threading.Lock()
        self.in_blackout = False
        self.blackout_until = rospy.Time(0)
        self.drift_bias_lat = 0.0
        self.drift_bias_lon = 0.0
        self.last_multipath_time = rospy.Time.now()

        # 통계 변수
        self.stats = {
            "total_received": 0,
            "total_disrupted": 0,
            "blackout_count": 0,
            "drift_applied": 0,
            "multipath_applied": 0
        }

        # 구독자/발행자 설정
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback, queue_size=10)
        self.gps_pub = rospy.Publisher('/ublox/fix', NavSatFix, queue_size=10)
        self.status_pub = rospy.Publisher('/gps_disruptor/status', String, queue_size=1)

        # 상태 발행 타이머
        rospy.Timer(rospy.Duration(5.0), self.publish_status)

        rospy.loginfo("=" * 60)
        rospy.loginfo("GPS Disruptor 시작")
        rospy.loginfo(f"  - 모드: {self.disruption_mode}")
        rospy.loginfo(f"  - 강도: {self.intensity}")
        rospy.loginfo(f"  - 활성화: {self.enable_disruption}")
        rospy.loginfo("=" * 60)

    def gps_callback(self, msg):
        """GPS 메시지 수신 후 불안정화 적용"""
        with self.gps_lock:
            self.latest_gps = msg
            self.stats["total_received"] += 1

        if not self.enable_disruption:
            # 비활성화 상태: 원본 그대로 재발행
            self.gps_pub.publish(msg)
            return

        # 불안정화 모드에 따라 처리
        disrupted_msg = self.apply_disruption(msg)

        if disrupted_msg is not None:
            self.gps_pub.publish(disrupted_msg)

    def apply_disruption(self, msg):
        """불안정화 패턴 적용"""
        current_time = rospy.Time.now()

        if self.disruption_mode == 'none':
            return msg

        elif self.disruption_mode == 'blackout':
            return self.apply_blackout(msg, current_time)

        elif self.disruption_mode == 'degradation':
            return self.apply_degradation(msg)

        elif self.disruption_mode == 'drift':
            return self.apply_drift(msg)

        elif self.disruption_mode == 'multipath':
            return self.apply_multipath(msg, current_time)

        elif self.disruption_mode == 'mixed':
            return self.apply_mixed(msg, current_time)

        else:
            rospy.logwarn_throttle(10, f"알 수 없는 모드: {self.disruption_mode}")
            return msg

    def apply_blackout(self, msg, current_time):
        """신호 끊김 패턴"""
        # 블랙아웃 종료 체크
        if self.in_blackout:
            if current_time < self.blackout_until:
                self.stats["total_disrupted"] += 1
                return None  # 메시지 발행 안 함
            else:
                self.in_blackout = False
                rospy.loginfo("GPS 신호 복구")

        # 새로운 블랙아웃 시작 확률
        if random.random() < self.get_blackout_probability():
            duration = self.get_blackout_duration()
            self.in_blackout = True
            self.blackout_until = current_time + rospy.Duration(duration)
            self.stats["blackout_count"] += 1
            rospy.logwarn(f"GPS 신호 끊김 시작 ({duration:.1f}초)")
            return None

        return msg

    def apply_degradation(self, msg):
        """신호 품질 저하"""
        degraded_msg = NavSatFix()
        degraded_msg.header = msg.header
        degraded_msg.latitude = msg.latitude
        degraded_msg.longitude = msg.longitude
        degraded_msg.altitude = msg.altitude
        degraded_msg.position_covariance_type = msg.position_covariance_type

        # status 저하
        if random.random() < 0.3:  # 30% 확률로 신호 없음
            degraded_msg.status.status = -1
            degraded_msg.status.service = 0
        else:
            degraded_msg.status.status = msg.status.status
            degraded_msg.status.service = msg.status.service

        # covariance 증가 (정확도 저하)
        multiplier = self.get_covariance_multiplier()
        degraded_msg.position_covariance = [
            cov * multiplier for cov in msg.position_covariance
        ]

        self.stats["total_disrupted"] += 1
        return degraded_msg

    def apply_drift(self, msg):
        """위치 드리프트 (천천히 표류)"""
        # 드리프트 바이어스 업데이트 (느린 변화)
        drift_change_rate = 0.00001  # 약 1m/호출
        self.drift_bias_lat += random.gauss(0, drift_change_rate)
        self.drift_bias_lon += random.gauss(0, drift_change_rate)

        # 바이어스가 너무 커지면 리셋
        max_drift = self.get_max_drift()
        if abs(self.drift_bias_lat) > max_drift:
            self.drift_bias_lat = 0.0
        if abs(self.drift_bias_lon) > max_drift:
            self.drift_bias_lon = 0.0

        # 드리프트 적용
        drifted_msg = NavSatFix()
        drifted_msg.header = msg.header
        drifted_msg.status = msg.status
        drifted_msg.latitude = msg.latitude + self.drift_bias_lat
        drifted_msg.longitude = msg.longitude + self.drift_bias_lon
        drifted_msg.altitude = msg.altitude
        drifted_msg.position_covariance = msg.position_covariance
        drifted_msg.position_covariance_type = msg.position_covariance_type

        self.stats["drift_applied"] += 1
        return drifted_msg

    def apply_multipath(self, msg, current_time):
        """멀티패스 간섭 (급격한 위치 점프)"""
        # 멀티패스 발생 간격
        multipath_interval = self.get_multipath_interval()

        if (current_time - self.last_multipath_time).to_sec() > multipath_interval:
            # 급격한 노이즈 추가
            jump_lat = random.gauss(0, 0.0001)  # 약 10m
            jump_lon = random.gauss(0, 0.0001)

            multipath_msg = NavSatFix()
            multipath_msg.header = msg.header
            multipath_msg.status = msg.status
            multipath_msg.latitude = msg.latitude + jump_lat
            multipath_msg.longitude = msg.longitude + jump_lon
            multipath_msg.altitude = msg.altitude
            multipath_msg.position_covariance = msg.position_covariance
            multipath_msg.position_covariance_type = msg.position_covariance_type

            self.last_multipath_time = current_time
            self.stats["multipath_applied"] += 1

            return multipath_msg

        return msg

    def apply_mixed(self, msg, current_time):
        """혼합 모드 (랜덤 조합)"""
        # 랜덤하게 다른 모드 적용
        modes = ['blackout', 'degradation', 'drift', 'multipath']
        weights = [0.2, 0.3, 0.3, 0.2]  # 각 모드 확률

        selected_mode = random.choices(modes, weights=weights)[0]

        if selected_mode == 'blackout':
            return self.apply_blackout(msg, current_time)
        elif selected_mode == 'degradation':
            return self.apply_degradation(msg)
        elif selected_mode == 'drift':
            return self.apply_drift(msg)
        elif selected_mode == 'multipath':
            return self.apply_multipath(msg, current_time)

    # 강도 설정 헬퍼 함수들
    def get_blackout_probability(self):
        """블랙아웃 발생 확률"""
        if self.intensity == 'low':
            return 0.001  # 0.1% (1000번 중 1번)
        elif self.intensity == 'medium':
            return 0.005  # 0.5%
        elif self.intensity == 'high':
            return 0.01   # 1%
        return 0.005

    def get_blackout_duration(self):
        """블랙아웃 지속 시간 (초)"""
        if self.intensity == 'low':
            return random.uniform(1.0, 3.0)
        elif self.intensity == 'medium':
            return random.uniform(3.0, 8.0)
        elif self.intensity == 'high':
            return random.uniform(5.0, 15.0)
        return random.uniform(3.0, 8.0)

    def get_covariance_multiplier(self):
        """Covariance 증가 배수"""
        if self.intensity == 'low':
            return random.uniform(2.0, 3.0)
        elif self.intensity == 'medium':
            return random.uniform(3.0, 7.0)
        elif self.intensity == 'high':
            return random.uniform(5.0, 10.0)
        return random.uniform(3.0, 7.0)

    def get_max_drift(self):
        """최대 드리프트 거리 (도 단위, 약 10m = 0.0001도)"""
        if self.intensity == 'low':
            return 0.0001  # ~10m
        elif self.intensity == 'medium':
            return 0.0002  # ~20m
        elif self.intensity == 'high':
            return 0.0005  # ~50m
        return 0.0002

    def get_multipath_interval(self):
        """멀티패스 발생 간격 (초)"""
        if self.intensity == 'low':
            return random.uniform(2.0, 5.0)
        elif self.intensity == 'medium':
            return random.uniform(1.0, 3.0)
        elif self.intensity == 'high':
            return random.uniform(0.5, 2.0)
        return random.uniform(1.0, 3.0)

    def publish_status(self, event):
        """상태 정보 발행"""
        status_data = {
            "node": "gps_disruptor",
            "mode": self.disruption_mode,
            "intensity": self.intensity,
            "enabled": self.enable_disruption,
            "in_blackout": self.in_blackout,
            "stats": self.stats,
            "disruption_rate": (self.stats["total_disrupted"] / max(1, self.stats["total_received"])) * 100
        }

        self.status_pub.publish(String(data=json.dumps(status_data)))

        # 주기적 로깅
        if self.enable_disruption and self.stats["total_received"] > 0:
            rospy.loginfo_throttle(30,
                f"GPS Disruptor 통계: "
                f"수신={self.stats['total_received']}, "
                f"불안정화={self.stats['total_disrupted']} "
                f"({status_data['disruption_rate']:.1f}%), "
                f"블랙아웃={self.stats['blackout_count']}회"
            )

    def dynamic_reconfigure_callback(self, config, level):
        """동적 파라미터 변경 (선택적)"""
        self.disruption_mode = config.mode
        self.intensity = config.intensity
        self.enable_disruption = config.enable

        rospy.loginfo(f"설정 변경: 모드={self.disruption_mode}, 강도={self.intensity}, 활성={self.enable_disruption}")
        return config

def main():
    try:
        disruptor = GPSDisruptor()

        rospy.loginfo("GPS Disruptor 실행 중...")
        rospy.loginfo("중단하려면 Ctrl+C를 누르세요.")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("GPS Disruptor 종료")
    except Exception as e:
        rospy.logerr(f"GPS Disruptor 오류: {e}")

if __name__ == '__main__':
    main()
