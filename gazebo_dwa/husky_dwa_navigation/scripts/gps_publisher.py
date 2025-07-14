#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json

if __name__ == "__main__":
    rospy.init_node("fake_gps_publisher")
    pub = rospy.Publisher("gps_data", String, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        fake_data = {
            "latitude": 35.8550893,
            "longitude": 127.17304149999999 
        }
        pub.publish(json.dumps(fake_data))
        rospy.loginfo(f"Send: {fake_data}")
        rate.sleep()
