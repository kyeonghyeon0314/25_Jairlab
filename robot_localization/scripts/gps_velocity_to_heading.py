#!/usr/bin/env python3
"""
GPS Velocity to Heading Converter
GPS velocity (TwistWithCovarianceStamped)로부터 heading을 계산하여
IMU orientation 메시지로 변환
"""

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TwistWithCovarianceStamped
import tf.transformations as tf_trans

class GPSVelocityToHeading:
    def __init__(self):
        rospy.init_node('gps_velocity_to_heading')

        # Parameters
        self.min_speed = rospy.get_param('~min_speed', 0.3)  # 최소 속도 (m/s)
        self.heading_covariance = rospy.get_param('~heading_covariance', 0.1)

        # State
        self.current_heading = None

        # Publishers
        self.imu_pub = rospy.Publisher('/gps/heading_imu', Imu, queue_size=10)

        # Subscribers
        rospy.Subscriber('/ublox/fix_velocity', TwistWithCovarianceStamped,
                        self.velocity_callback)

        rospy.loginfo("GPS Velocity to Heading Converter started")
        rospy.loginfo(f"  Min speed threshold: {self.min_speed} m/s")

    def velocity_callback(self, msg):
        """GPS velocity 메시지로부터 heading 계산"""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        # Calculate speed
        speed = math.sqrt(vx**2 + vy**2)

        # Only calculate heading if moving fast enough
        if speed < self.min_speed:
            rospy.logdebug(f"Speed too low: {speed:.2f} m/s < {self.min_speed} m/s")
            return

        # Calculate heading from velocity
        # GPS velocity는 ENU 좌표계 (East-North-Up)
        heading = math.atan2(vy, vx)
        self.current_heading = heading

        # Create IMU message with orientation
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.header.frame_id = "gps"

        # Convert heading to quaternion
        quat = tf_trans.quaternion_from_euler(0, 0, heading)
        imu_msg.orientation = Quaternion(*quat)

        # Set covariance
        # -1 means "not provided" for roll and pitch
        imu_msg.orientation_covariance = [
            -1, 0, 0,
            0, -1, 0,
            0, 0, self.heading_covariance
        ]

        # Angular velocity not provided
        imu_msg.angular_velocity_covariance[0] = -1

        # Publish
        self.imu_pub.publish(imu_msg)

        rospy.loginfo_throttle(2.0,
            f"GPS Heading: {math.degrees(heading):.1f}°, Speed: {speed:.2f} m/s, "
            f"Velocity: ({vx:.3f}, {vy:.3f})")

if __name__ == '__main__':
    try:
        node = GPSVelocityToHeading()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
