#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2

class UnifiedClockPublisher:
    def __init__(self):
        rospy.init_node('unified_clock_publisher')
        self.pub = rospy.Publisher('/clock', Clock, queue_size=10)
        self.rate = rospy.Rate(100)  # 100 Hz
        
        # bag에서 시간 동기화를 위해 센서 데이터 구독
        self.sub_imu = rospy.Subscriber('/ouster/imu', Imu, self.imu_callback)
        self.sub_points = rospy.Subscriber('/ouster/points', PointCloud2, self.points_callback)
        
        self.last_sensor_time = None

    def imu_callback(self, msg):
        # IMU 데이터의 타임스탬프를 /clock으로 발행
        self.publish_clock(msg.header.stamp)

    def points_callback(self, msg):
        # PointCloud2 데이터의 타임스탬프를 /clock으로 발행
        self.publish_clock(msg.header.stamp)

    def publish_clock(self, timestamp):
        # 센서 데이터의 타임스탬프를 /clock으로 발행
        self.last_sensor_time = timestamp
        clock_msg = Clock()
        #clock_msg.clock = self.last_sensor_time
        clock_msg.clock = rospy.Time.now() # 현재 시간 사용
        self.pub.publish(clock_msg)

    def run(self):
        rospy.spin()  # 콜백 기반으로 변경

if __name__ == '__main__':
    try:
        clock_pub = UnifiedClockPublisher()
        clock_pub.run()
    except rospy.ROSInterruptException:
        pass