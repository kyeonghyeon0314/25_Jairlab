#!/usr/bin/env python3
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