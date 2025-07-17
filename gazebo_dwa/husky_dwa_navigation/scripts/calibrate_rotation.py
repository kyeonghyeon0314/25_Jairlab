#!/usr/bin/env python3
"""
Husky 바퀴 간격 캘리브레이션 스크립트
360도 제자리 회전을 통해 wheel_separation_multiplier 값을 캘리브레이션합니다.

사용법:
1. 로봇을 평평한 바닥에 놓고 시작 위치를 표시
2. 스크립트 실행: rosrun husky_dwa_navigation calibrate_rotation.py
3. 회전 완료 후 실제 회전 각도를 육안으로 측정
4. 출력된 보정값을 control.yaml에 적용

주의사항:
- 충분한 공간에서 실행
- 장애물이 없는 평평한 바닥
- odom 사용 안함, 단순 cmd_vel 기반 회전
- move_front.py와 같은 부드러운 가속/감속 적용
"""

import rospy
import math
from geometry_msgs.msg import Twist

class RotationCalibrator:
    def __init__(self):
        rospy.init_node('rotation_calibrator', anonymous=True)
        
        # Publisher 설정
        self.cmd_vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1, latch=False)
        
        # Control.yaml 파라미터 기반 설정
        self.max_angular_velocity = 2.0      # control.yaml의 max_velocity (rad/s)
        self.max_angular_acceleration = 2.0  # control.yaml의 max_acceleration (rad/s²)
        
        # 부드러운 회전을 위한 파라미터 (move_front.py 스타일)
        self.acceleration_time = 2.0    # 가속 시간 (초)
        self.constant_speed_time = 5.0  # 등속 회전 시간 (초) - 360도 회전에 필요한 시간
        self.deceleration_time = 2.0    # 감속 시간 (초)
        
        # 현재 사용중인 wheel_separation_multiplier 값 (control.yaml에서)
        self.current_multiplier = 1.875
        
        rospy.loginfo("=== Husky 회전 캘리브레이션 스크립트 ===")
        rospy.loginfo(f"현재 wheel_separation_multiplier: {self.current_multiplier}")
        rospy.loginfo(f"제어 파라미터 - 최대 각속도: {self.max_angular_velocity} rad/s")
        rospy.loginfo(f"제어 파라미터 - 최대 각가속도: {self.max_angular_acceleration} rad/s²")
        rospy.loginfo("360도 제자리 회전을 통해 정확한 값을 측정합니다.")
        rospy.loginfo("로봇 주변에 충분한 공간이 있는지 확인하세요!")
        
        rospy.sleep(2.0)  # 초기화 대기

    def calculate_smooth_angular_velocity(self, elapsed_time):
        """시간에 따른 부드러운 각속도 계산 (move_front.py 스타일)"""
        
        # 단계 1: 가속 구간 (0 ~ acceleration_time)
        if elapsed_time <= self.acceleration_time:
            # 사인 함수를 사용한 부드러운 가속 (0에서 max_angular_velocity까지)
            progress = elapsed_time / self.acceleration_time
            speed_factor = (1 - math.cos(progress * math.pi)) / 2
            return self.max_angular_velocity * speed_factor
        
        # 단계 2: 등속 구간 (acceleration_time ~ acceleration_time + constant_speed_time)
        elif elapsed_time <= self.acceleration_time + self.constant_speed_time:
            return self.max_angular_velocity
        
        # 단계 3: 감속 구간 (나머지 시간)
        else:
            decel_start_time = self.acceleration_time + self.constant_speed_time
            decel_elapsed = elapsed_time - decel_start_time
            
            # 사인 함수를 사용한 부드러운 감속 (max_angular_velocity에서 0까지)
            progress = decel_elapsed / self.deceleration_time
            progress = min(progress, 1.0)  # 1.0을 넘지 않도록 제한
            
            # 역방향 사인 함수로 부드러운 감속
            speed_factor = (1 + math.cos(progress * math.pi)) / 2
            return self.max_angular_velocity * speed_factor
    
    def stop_robot(self):
        """로봇 완전 정지"""
        stop_cmd = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop_cmd)
            rospy.sleep(0.1)
    
    def perform_rotation_test(self):
        """360도 회전 테스트 수행 (부드러운 가속/감속 적용)"""
        rospy.loginfo("\n=== 회전 테스트 시작 ===")
        
        # 사용자 확인
        input("\n로봇의 시작 위치를 표시하고 Enter를 눌러 회전을 시작하세요...")
        
        # 부드러운 회전 수행
        rospy.loginfo("부드러운 360도 시계방향 회전 시작...")
        
        rate = rospy.Rate(20)  # 20Hz로 부드럽게
        start_time = rospy.Time.now()
        
        # 전체 프로세스: 가속 -> 등속 -> 감속
        total_time = self.acceleration_time + self.constant_speed_time + self.deceleration_time
        
        rospy.loginfo(f"가속 시간: {self.acceleration_time}초")
        rospy.loginfo(f"등속 시간: {self.constant_speed_time}초")
        rospy.loginfo(f"감속 시간: {self.deceleration_time}초")
        rospy.loginfo(f"총 회전 시간: {total_time}초")
        
        while not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            
            if elapsed_time > total_time:
                break
            
            # 부드러운 각속도 계산
            current_angular_velocity = self.calculate_smooth_angular_velocity(elapsed_time)
            
            # 시계방향 회전 (음수)
            twist = Twist()
            twist.angular.z = -current_angular_velocity
            self.cmd_vel_pub.publish(twist)
            
            # 진행 상황 출력
            progress_percent = (elapsed_time / total_time) * 100
            if int(progress_percent) % 10 == 0 and int(progress_percent) > 0:
                rospy.loginfo(f"회전 진행: {progress_percent:.0f}% (각속도: {current_angular_velocity:.2f} rad/s)")
            
            rate.sleep()
        
        # 회전 완료 후 정지
        self.stop_robot()
        
        # 최종 결과
        end_time = rospy.Time.now()
        elapsed_time = (end_time - start_time).to_sec()
        
        rospy.loginfo("\n=== 회전 테스트 완료 ===")
        rospy.loginfo(f"총 회전 시간: {elapsed_time:.1f}초")
        rospy.loginfo(f"이론상 360도 회전 완료")
        
        return elapsed_time
    
    def calculate_correction_factor(self, rotation_time):
        """보정 계수 계산"""
        target_degrees = 360.0
        
        rospy.loginfo("\n=== 캘리브레이션 결과 ===")
        
        print("\n실제 회전각을 육안으로 측정해주세요:")
        print("1. 로봇이 원래 방향을 향하고 있나요? (360도)")
        print("2. 조금 모자라거나 더 돌았나요?")
        print("3. 예상 실제 회전각을 입력하세요 (예: 340, 380)")
        
        try:
            actual_degrees = float(input("\n실제 회전각 (도): "))
        except ValueError:
            rospy.logwarn("잘못된 입력입니다. 360도로 가정합니다.")
            actual_degrees = 360.0
        
        # 보정 계수 계산
        # 현재 multiplier × (목표각도 / 실제각도)
        new_multiplier = self.current_multiplier * (target_degrees / actual_degrees)
        
        print(f"\n=== 캘리브레이션 결과 ===")
        print(f"회전 시간: {rotation_time:.1f}초")
        print(f"목표 회전각: {target_degrees:.0f}도")
        print(f"실제 회전각: {actual_degrees:.2f}도")
        print(f"현재 wheel_separation_multiplier: {self.current_multiplier}")
        print(f"권장 wheel_separation_multiplier: {new_multiplier:.3f}")
        
        error_percentage = abs((actual_degrees - target_degrees) / target_degrees) * 100
        print(f"오차율: {error_percentage:.1f}%")
        
        if error_percentage < 1.0:
            print("✅ 현재 설정이 매우 정확합니다!")
        elif error_percentage < 3.0:
            print("✅ 현재 설정이 양호합니다.")
        else:
            print("⚠️  보정이 필요합니다.")
            print(f"\ncontrol.yaml 파일에서 수정:")
            print(f"wheel_separation_multiplier: {new_multiplier:.3f}")
        
        return new_multiplier
    
    def run_calibration(self):
        """전체 캘리브레이션 프로세스 실행"""
        try:
            # 안전을 위한 초기 정지
            self.stop_robot()
            rospy.sleep(1.0)
            
            # 회전 테스트 수행
            rotation_time = self.perform_rotation_test()
            
            # 보정 계수 계산
            self.calculate_correction_factor(rotation_time)
            
            rospy.loginfo("\n캘리브레이션 완료!")
            rospy.loginfo("부드러운 가속/감속이 적용된 회전 테스트였습니다.")
            
        except rospy.ROSInterruptException:
            rospy.loginfo("프로그램이 중단되었습니다.")
        except KeyboardInterrupt:
            rospy.loginfo("사용자에 의해 중단되었습니다.")
        finally:
            # 최종 안전 정지
            self.stop_robot()

def main():
    try:
        calibrator = RotationCalibrator()
        calibrator.run_calibration()
    except rospy.ROSInterruptException:
        rospy.loginfo("노드가 종료되었습니다.")

if __name__ == '__main__':
    main()