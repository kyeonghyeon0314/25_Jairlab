#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Imu

class TimestampSynchronizer:
    def __init__(self):
        rospy.init_node('timestamp_synchronizer')
        
        # PointCloud2 토픽 구독 및 발행
        self.pointcloud_sub = rospy.Subscriber('/new_points', PointCloud2, self.pointcloud_callback)
        self.pointcloud_pub = rospy.Publisher('/ouster/points', PointCloud2, queue_size=10)
        
        # IMU 토픽 구독 및 발행
        self.imu_sub = rospy.Subscriber('/new_imu', Imu, self.imu_callback)
        self.imu_pub = rospy.Publisher('/ouster/imu', Imu, queue_size=10)
        
        rospy.loginfo("Timestamp Synchronizer 시작됨")
        rospy.loginfo("PointCloud 구독: /new_points -> 발행: /ouster/points")
        rospy.loginfo("IMU 구독: /new_imu -> 발행: /ouster/imu")

    def pointcloud_callback(self, msg):
        # 메시지 복사 및 타임스탬프를 현재 시간으로 변경
        synced_msg = PointCloud2()
        synced_msg.header = msg.header
        synced_msg.header.stamp = rospy.Time.now()  # 현재 시간으로 변경
        synced_msg.height = msg.height
        synced_msg.width = msg.width
        synced_msg.fields = msg.fields
        synced_msg.is_bigendian = msg.is_bigendian
        synced_msg.point_step = msg.point_step
        synced_msg.row_step = msg.row_step
        synced_msg.data = msg.data
        synced_msg.is_dense = msg.is_dense
        
        # 동기화된 메시지 발행
        self.pointcloud_pub.publish(synced_msg)
        
        rospy.loginfo_throttle(1.0, f"PointCloud 타임스탬프 동기화: {msg.header.stamp} -> {synced_msg.header.stamp}")
    
    def imu_callback(self, msg):
        # IMU 메시지 복사 및 타임스탬프를 현재 시간으로 변경
        synced_msg = Imu()
        synced_msg.header = msg.header
        synced_msg.header.stamp = rospy.Time.now()  # 현재 시간으로 변경
        synced_msg.orientation = msg.orientation
        synced_msg.orientation_covariance = msg.orientation_covariance
        synced_msg.angular_velocity = msg.angular_velocity
        synced_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        synced_msg.linear_acceleration = msg.linear_acceleration
        synced_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        # 동기화된 IMU 메시지 발행
        self.imu_pub.publish(synced_msg)
        
        rospy.loginfo_throttle(1.0, f"IMU 타임스탬프 동기화: {msg.header.stamp} -> {synced_msg.header.stamp}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        synchronizer = TimestampSynchronizer()
        synchronizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("프로그램이 종료되었습니다.")



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