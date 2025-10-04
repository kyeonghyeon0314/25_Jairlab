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
import sys
import select
import termios
import tty
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json

class GPSDisruptor:
    def __init__(self):
        rospy.init_node('gps_disruptor', anonymous=False)

        # 파라미터 로드 (토픽 이름 파라미터 추가)
        self.gps_input_topic = rospy.get_param('~gps_input_topic', '/ublox/fix_input')
        self.gps_output_topic = rospy.get_param('~gps_output_topic', '/ublox/fix')
        self.disruption_mode = rospy.get_param('~disruption_mode', 'normal')  # normal, blackout, jump

        # 상태 변수
        self.latest_gps = None
        self.gps_lock = threading.Lock()
        self.in_blackout = False
        self.blackout_until = rospy.Time(0)
        self.last_jump_time = rospy.Time.now()

        # 통계 변수
        self.stats = {
            "total_received": 0,
            "total_disrupted": 0,
            "jump_applied": 0
        }

        # 구독자/발행자 설정 (파라미터로 토픽 지정)
        self.gps_sub = rospy.Subscriber(self.gps_input_topic, NavSatFix, self.gps_callback, queue_size=10)
        self.gps_pub = rospy.Publisher(self.gps_output_topic, NavSatFix, queue_size=10)
        self.status_pub = rospy.Publisher('/gps_disruptor/status', String, queue_size=1)

        # 상태 발행 타이머
        rospy.Timer(rospy.Duration(5.0), self.publish_status)

        rospy.loginfo("=" * 60)
        rospy.loginfo("GPS Disruptor 시작")
        rospy.loginfo(f"  - 입력 토픽: {self.gps_input_topic}")
        rospy.loginfo(f"  - 출력 토픽: {self.gps_output_topic}")
        rospy.loginfo(f"  - 현재 모드: {self.disruption_mode}")
        rospy.loginfo("=" * 60)
        self.print_help()

    def gps_callback(self, msg):
        """GPS 메시지 수신 후 불안정화 적용"""
        with self.gps_lock:
            self.latest_gps = msg
            self.stats["total_received"] += 1

        # 불안정화 모드에 따라 처리
        disrupted_msg = self.apply_disruption(msg)

        if disrupted_msg is not None:
            self.gps_pub.publish(disrupted_msg)

    def apply_disruption(self, msg):
        """불안정화 패턴 적용"""
        current_time = rospy.Time.now()

        if self.disruption_mode == 'normal':
            # 정상 작동: 원본 그대로 재발행
            return msg

        elif self.disruption_mode == 'blackout':
            return self.apply_blackout(msg, current_time)

        elif self.disruption_mode == 'jump':
            return self.apply_jump(msg, current_time)

        else:
            rospy.logwarn_throttle(10, f"알 수 없는 모드: {self.disruption_mode}")
            return msg

    def apply_blackout(self, msg, current_time):
        """신호 끊김 패턴 - GPS 완전 차단"""
        # 블랙아웃 모드에서는 항상 None 반환 (GPS 차단)
        self.stats["total_disrupted"] += 1
        return None  # 메시지 발행 안 함

    def apply_jump(self, msg, current_time):
        """튐 현상 (급격한 위치 점프) - 지속적으로 점프"""
        # 매 메시지마다 점프 적용 (지속적인 튐 현상)
        # 점프 크기: 0.0001도 ≈ 11m (위도/경도 1도 ≈ 111km)
        jump_magnitude = 0.0001
        # 균등분포 사용 (더 명확한 점프)
        jump_lat = random.uniform(-jump_magnitude, jump_magnitude)
        jump_lon = random.uniform(-jump_magnitude, jump_magnitude)

        jump_msg = NavSatFix()
        jump_msg.header = msg.header
        jump_msg.status = msg.status
        jump_msg.latitude = msg.latitude + jump_lat
        jump_msg.longitude = msg.longitude + jump_lon
        jump_msg.altitude = msg.altitude
        jump_msg.position_covariance = msg.position_covariance
        jump_msg.position_covariance_type = msg.position_covariance_type

        self.stats["jump_applied"] += 1

        # 점프 발생 로그 (5초마다)
        rospy.loginfo_throttle(5, f"GPS 점프 중: lat {msg.latitude:.6f}→{jump_msg.latitude:.6f}, lon {msg.longitude:.6f}→{jump_msg.longitude:.6f}")

        return jump_msg


    def publish_status(self, event):
        """상태 정보 발행"""
        status_data = {
            "node": "gps_disruptor",
            "mode": self.disruption_mode,
            "stats": self.stats,
            "disruption_rate": (self.stats["total_disrupted"] / max(1, self.stats["total_received"])) * 100
        }

        self.status_pub.publish(String(data=json.dumps(status_data)))

        # 주기적 로깅
        if self.disruption_mode != 'normal' and self.stats["total_received"] > 0:
            rospy.loginfo_throttle(30,
                f"GPS Disruptor 통계: "
                f"모드={self.disruption_mode}, "
                f"수신={self.stats['total_received']}, "
                f"불안정화={self.stats['total_disrupted']} "
                f"({status_data['disruption_rate']:.1f}%)"
            )

    def print_help(self):
        """도움말 출력"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("키보드 명령어:")
        rospy.loginfo("  1: 정상 작동 (normal)")
        rospy.loginfo("  2: 신호 끊김 (blackout) - GPS 완전 차단")
        rospy.loginfo("  3: 튐 현상 (jump) - 1초마다 ±5m 점프")
        rospy.loginfo("")
        rospy.loginfo("  s: 현재 상태 출력")
        rospy.loginfo("  r: 통계 초기화")
        rospy.loginfo("  ?: 도움말")
        rospy.loginfo("  q: 종료")
        rospy.loginfo("=" * 60 + "\n")

    def change_mode(self, mode):
        """모드 변경"""
        old_mode = self.disruption_mode
        self.disruption_mode = mode

        # 모드 변경 시 상태 리셋
        self.in_blackout = False
        self.last_jump_time = rospy.Time.now()

        rospy.loginfo(f"[모드 변경] {old_mode} → {self.disruption_mode}")

    def print_current_status(self):
        """현재 상태 출력"""
        mode_name = {
            'normal': '정상 작동',
            'blackout': '신호 끊김 (GPS 완전 차단)',
            'jump': '튐 현상 (1초마다 ±5m)'
        }.get(self.disruption_mode, self.disruption_mode)

        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("현재 상태:")
        rospy.loginfo(f"  - 모드: {mode_name}")
        rospy.loginfo(f"  - 수신: {self.stats['total_received']}개")
        rospy.loginfo(f"  - 불안정화: {self.stats['total_disrupted']}개")
        rospy.loginfo(f"  - 튐 현상: {self.stats['jump_applied']}회")
        rospy.loginfo("=" * 60 + "\n")

    def reset_stats(self):
        """통계 초기화"""
        self.stats = {
            "total_received": 0,
            "total_disrupted": 0,
            "jump_applied": 0
        }
        rospy.loginfo("[통계 초기화 완료]")

def keyboard_input_thread(disruptor):
    """키보드 입력 처리 스레드"""
    # 터미널 설정 백업
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
            # 키 입력 대기 (0.1초 타임아웃)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)

                # 모드 변경
                if key == '1':
                    disruptor.change_mode('normal')
                elif key == '2':
                    disruptor.change_mode('blackout')
                elif key == '3':
                    disruptor.change_mode('jump')

                # 기타 명령
                elif key == 's':
                    disruptor.print_current_status()
                elif key == 'r':
                    disruptor.reset_stats()
                elif key == 'q':
                    rospy.loginfo("종료 명령 수신")
                    rospy.signal_shutdown("사용자 종료")
                    break
                elif key == '?':
                    disruptor.print_help()

    finally:
        # 터미널 설정 복원
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    try:
        disruptor = GPSDisruptor()

        rospy.loginfo("GPS Disruptor 실행 중...")
        rospy.loginfo("키보드 명령어를 사용하여 제어할 수 있습니다.")
        rospy.loginfo("도움말을 보려면 '?' 키를 누르세요.")

        # 키보드 입력 스레드 시작
        kb_thread = threading.Thread(target=keyboard_input_thread, args=(disruptor,), daemon=True)
        kb_thread.start()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("GPS Disruptor 종료")
    except Exception as e:
        rospy.logerr(f"GPS Disruptor 오류: {e}")

if __name__ == '__main__':
    main()
