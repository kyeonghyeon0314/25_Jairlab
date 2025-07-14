#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

class ClockPublisher:
    def __init__(self):
        rospy.init_node('manual_clock_publisher')
        self.pub = rospy.Publisher('/clock', Clock, queue_size=10)
        self.rate = rospy.Rate(100)  # 100 Hz
        
        # bag에서 시간 동기화를 위해 센서 데이터 구독
        self.sub = rospy.Subscriber('/ouster/imu', Imu, self.sensor_callback)
        self.last_sensor_time = None

    def sensor_callback(self, msg):
        # 센서 데이터의 타임스탬프를 /clock으로 발행
        self.last_sensor_time = msg.header.stamp
        clock_msg = Clock()
        #clock_msg.clock = self.last_sensor_time
        clock_msg.clock = rospy.Time.now() # 현재 시간 사용
        self.pub.publish(clock_msg)

    def run(self):
        rospy.spin()  # 콜백 기반으로 변경

if __name__ == '__main__':
    try:
        clock_pub = ClockPublisher()
        clock_pub.run()
    except rospy.ROSInterruptException:
        pass



# kimkh@kimkh:~/JBNU_LIDAR_DATASET_EVAL$ rosbag info 2025-01-11-12-05-15.bag
# path: 2025-01-11-12-05-15.bag
# version: 2.0
# duration: 7:59s (479s)
# start: Jan 11 2025 12:05:16.25 (1736564716.25)
# end: Jan 11 2025 12:13:15.31 (1736565195.31)
# size: 7.0 GB
# messages: 110812
# compression: none [4791/4791 chunks]
# types: diagnostic_msgs/DiagnosticArray [60810da900de1dd6ddd437c3503511da]
# geometry_msgs/Vector3Stamped [7b324c7325e683bf02a9b14b01090ec7]
# sensor_msgs/Imu [6a62c6daae103f4ff57a132d6f95cec2]
# sensor_msgs/NavSatFix [2d3a8cd499b9b4a0249fb98fd05cfa48]
# sensor_msgs/PointCloud2 [1158d486dd51d683ce2f1be655c3c181]
# topics: /diagnostics 4294 msgs : diagnostic_msgs/DiagnosticArray (6 connections)
# /imu/mag 47900 msgs : geometry_msgs/Vector3Stamped
# /ouster/imu 47893 msgs : sensor_msgs/Imu
# /ouster/points 4790 msgs : sensor_msgs/PointCloud2
# /ublox/fix 5935 msgs : sensor_msgs/NavSatFix