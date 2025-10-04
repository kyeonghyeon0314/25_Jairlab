#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

class HuskyController:
    def __init__(self):
        rospy.init_node('husky_movement_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1, latch=False)
        rospy.sleep(1)
        
        # 🚗 Move Front 움직임 파라미터 (수동 설정)
        # ⚠️ 중요: initialize_pose.py의 MOVE_FRONT_TIMING과 일치시켜야 함
        self.max_speed = 1.0             # 최대 속도 (m/s)
        self.min_speed = 0.0             # 최소 속도 (m/s)
        self.acceleration_time = 3.0     # 🔧 가속 시간 (초) - initialize_pose.py "acceleration"과 일치
        self.constant_speed_time = 4.0   # 🔧 등속 시간 (초) - initialize_pose.py "constant"와 일치
        self.deceleration_time = 2.0     # 🔧 감속 시간 (초) - initialize_pose.py "deceleration"과 일치
        
    def move_with_smooth_acceleration(self):
        """부드러운 곡선 가속도로 이동 (가속 -> 등속 -> 감속)"""
        rospy.loginfo("부드러운 가속도로 이동 시작...")
        
        rate = rospy.Rate(20)  # 20Hz로 부드럽게
        start_time = rospy.Time.now()
        
        # 전체 프로세스: 가속 -> 등속 -> 감속
        total_time = self.acceleration_time + self.constant_speed_time + self.deceleration_time
        
        while not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            
            if elapsed_time > total_time:
                break
            
            current_speed = self.calculate_smooth_speed(elapsed_time)
            
            twist = Twist()
            twist.linear.x = current_speed
            self.cmd_vel_pub.publish(twist)
            
            rospy.loginfo_throttle(1, f"시간: {elapsed_time:.2f}s, 속도: {current_speed:.2f} m/s")
            rate.sleep()
        
        # 완전 정지
        self.final_stop()
        rospy.loginfo("부드러운 가속도 이동 완료")
    
    def calculate_smooth_speed(self, elapsed_time):
        """시간에 따른 부드러운 속도 계산"""
        
        # 단계 1: 가속 구간 (0 ~ acceleration_time)
        if elapsed_time <= self.acceleration_time:
            # 사인 함수를 사용한 부드러운 가속 (0에서 max_speed까지)
            progress = elapsed_time / self.acceleration_time
            speed_factor = (1 - math.cos(progress * math.pi)) / 2
            return self.min_speed + (self.max_speed - self.min_speed) * speed_factor
        
        # 단계 2: 등속 구간 (acceleration_time ~ acceleration_time + constant_speed_time)
        elif elapsed_time <= self.acceleration_time + self.constant_speed_time:
            return self.max_speed
        
        # 단계 3: 감속 구간 (나머지 시간)
        else:
            decel_start_time = self.acceleration_time + self.constant_speed_time
            decel_elapsed = elapsed_time - decel_start_time
            
            # 사인 함수를 사용한 부드러운 감속 (max_speed에서 0까지)
            progress = decel_elapsed / self.deceleration_time
            progress = min(progress, 1.0)  # 1.0을 넘지 않도록 제한
            
            # 역방향 사인 함수로 부드러운 감속
            speed_factor = (1 + math.cos(progress * math.pi)) / 2
            return self.max_speed * speed_factor
    
    def final_stop(self):
        """최종 완전 정지"""
        twist = Twist()  # 모든 값이 0
        
        # 확실한 정지를 위해 여러 번 발행
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
    
    def execute_movement(self):
        """부드러운 가속도 움직임 실행"""
        try:
            rospy.loginfo("=== Husky 부드러운 움직임 제어 시작 ===")
            rospy.loginfo(f"🔧 가속 시간: {self.acceleration_time}초")
            rospy.loginfo(f"🔧 등속 시간: {self.constant_speed_time}초") 
            rospy.loginfo(f"🔧 감속 시간: {self.deceleration_time}초")
            rospy.loginfo(f"🔧 총 실행 시간: {self.acceleration_time + self.constant_speed_time + self.deceleration_time}초")
            rospy.loginfo(f"🔧 최대 속도: {self.max_speed} m/s")
            rospy.logwarn("⚠️  이 값들을 변경하면 initialize_pose.py의 MOVE_FRONT_TIMING도 수동 수정 필요!")
            
            # 안전을 위한 초기 정지
            self.final_stop()
            rospy.sleep(0.5)
            
            # 부드러운 가속도로 이동
            self.move_with_smooth_acceleration()
            
            rospy.loginfo("=== 모든 움직임 완료 ===")
            
        except rospy.ROSInterruptException:
            rospy.loginfo("프로그램이 중단되었습니다.")
        except KeyboardInterrupt:
            rospy.loginfo("사용자에 의해 중단되었습니다.")
        finally:
            # 프로그램 종료 시 안전 정지
            rospy.loginfo("안전 정지 수행 중...")
            self.final_stop()

def main():
    try:
        controller = HuskyController()
        controller.execute_movement()
    except rospy.ROSInterruptException:
        rospy.loginfo("노드가 종료되었습니다.")

if __name__ == '__main__':
    main()