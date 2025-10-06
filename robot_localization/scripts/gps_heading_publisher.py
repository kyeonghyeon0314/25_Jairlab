#!/usr/bin/env python3
"""
GPS Heading Publisher
GPS 연속 위치로부터 heading(yaw)을 계산하여 발행하는 노드
"""

import rospy
import math
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion
import tf.transformations as tf_trans

class GPSHeadingPublisher:
    def __init__(self):
        rospy.init_node('gps_heading_publisher')

        # Parameters
        self.min_speed = rospy.get_param('~min_speed', 0.5)  # 최소 속도 (m/s)
        self.position_buffer_size = rospy.get_param('~buffer_size', 5)

        # State
        self.prev_positions = []  # [(lat, lon, timestamp), ...]
        self.current_heading = 0.0

        # Publishers
        self.imu_pub = rospy.Publisher('/gps/heading_imu', Imu, queue_size=10)

        # Subscribers
        self.gps_sub = rospy.Subscribe('/ublox/fix', NavSatFix, self.gps_callback)

        rospy.loginfo("GPS Heading Publisher started")

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """두 GPS 좌표 간의 거리 계산 (미터)"""
        R = 6371000  # Earth radius in meters

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """두 GPS 좌표 간의 bearing 계산 (라디안, 북쪽 기준 시계방향)"""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)

        y = math.sin(dlambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)

        bearing = math.atan2(y, x)

        # Convert from North-clockwise to East-counterclockwise (ENU frame)
        # North = 0, East = 90 → East = 0, North = 90
        enu_heading = math.pi/2 - bearing

        # Normalize to [-pi, pi]
        while enu_heading > math.pi:
            enu_heading -= 2 * math.pi
        while enu_heading < -math.pi:
            enu_heading += 2 * math.pi

        return enu_heading

    def gps_callback(self, msg):
        """GPS 메시지 처리"""
        if msg.status.status == -1:  # NO_FIX
            return

        current_time = rospy.Time.now()
        current_pos = (msg.latitude, msg.longitude, current_time)

        # Add to buffer
        self.prev_positions.append(current_pos)
        if len(self.prev_positions) > self.position_buffer_size:
            self.prev_positions.pop(0)

        # Need at least 2 positions
        if len(self.prev_positions) < 2:
            return

        # Calculate heading from oldest to newest position
        oldest = self.prev_positions[0]
        newest = self.prev_positions[-1]

        # Calculate distance and time
        distance = self.haversine_distance(oldest[0], oldest[1], newest[0], newest[1])
        time_diff = (newest[2] - oldest[2]).to_sec()

        if time_diff < 0.1:  # Avoid division by zero
            return

        speed = distance / time_diff

        # Only update heading if moving fast enough
        if speed < self.min_speed:
            rospy.logdebug(f"Speed too low: {speed:.2f} m/s < {self.min_speed} m/s")
            return

        # Calculate heading
        heading = self.calculate_bearing(oldest[0], oldest[1], newest[0], newest[1])
        self.current_heading = heading

        # Publish as IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "gps"

        # Convert heading to quaternion
        quat = tf_trans.quaternion_from_euler(0, 0, heading)
        imu_msg.orientation = Quaternion(*quat)

        # Set covariance (low for orientation, high for angular velocity)
        imu_msg.orientation_covariance[0] = -1  # Not providing roll
        imu_msg.orientation_covariance[4] = -1  # Not providing pitch
        imu_msg.orientation_covariance[8] = 0.1  # Yaw covariance

        imu_msg.angular_velocity_covariance[0] = -1

        self.imu_pub.publish(imu_msg)

        rospy.loginfo_throttle(2.0, f"GPS Heading: {math.degrees(heading):.1f}°, Speed: {speed:.2f} m/s")

if __name__ == '__main__':
    try:
        node = GPSHeadingPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
