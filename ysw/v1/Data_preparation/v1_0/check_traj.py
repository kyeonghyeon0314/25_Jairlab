#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf

def read_trajectory(file_path):
    """Reads the trajectory file and extracts positions and orientations."""
    trajectory = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            timestamp, x, y, z, qx, qy, qz, qw = map(float, parts)
            trajectory.append((timestamp, x, y, z, qx, qy, qz, qw))
    return trajectory

def publish_trajectory(file_path):
    """Publishes the trajectory as a nav_msgs/Path to be visualized in RViz."""
    rospy.init_node('trajectory_publisher', anonymous=True)
    path_pub = rospy.Publisher('/trajectory', Path, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    path = Path()
    path.header.frame_id = "map"  # Change if necessary

    trajectory = read_trajectory(file_path)
    for timestamp, x, y, z, qx, qy, qz, qw in trajectory:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.from_sec(timestamp)
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        
        path.poses.append(pose_stamped)

    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        path_pub.publish(path)
        rate.sleep()

if __name__ == "__main__":
    trajectory_file = "/home/ysw/catkin_ws/src/faster-lio/Log/traj.txt"  # Change this to your actual file path
    publish_trajectory(trajectory_file)

# #!/usr/bin/env python
# import rospy
# import numpy as np
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point, Quaternion
# import tf

# def read_trajectory(file_path):
#     """Reads the trajectory file and extracts positions and orientations."""
#     trajectory = []
#     with open(file_path, 'r') as f:
#         for line in f:
#             if line.startswith('#'):
#                 continue  # Skip header lines
#             parts = line.strip().split()
#             if len(parts) != 8:
#                 continue  # Ensure correct format
#             timestamp, x, y, z, qx, qy, qz, qw = map(float, parts)
#             trajectory.append((timestamp, x, y, z, qx, qy, qz, qw))
#     return trajectory

# def publish_markers(file_path):
#     """Publishes trajectory poses as arrow markers in RViz."""
#     rospy.init_node('trajectory_marker_publisher', anonymous=True)
#     marker_pub = rospy.Publisher('/trajectory_markers', Marker, queue_size=10)
#     rate = rospy.Rate(10)  # Set publishing rate to 10 Hz

#     trajectory = read_trajectory(file_path)
#     marker_id = 0  # Unique ID for each marker

#     while not rospy.is_shutdown():
#         for timestamp, x, y, z, qx, qy, qz, qw in trajectory:
#             marker = Marker()
#             marker.header.frame_id = "map"  # Reference frame for RViz
#             marker.header.stamp = rospy.Time.now()
#             marker.ns = "trajectory"
#             marker.id = marker_id
#             marker.type = Marker.ARROW  # Use arrow marker for orientation
#             marker.action = Marker.ADD
            
#             # Set position
#             marker.pose.position.x = x
#             marker.pose.position.y = y
#             marker.pose.position.z = z
            
#             # Set orientation (quaternion format)
#             marker.pose.orientation = Quaternion(qx, qy, qz, qw)
            
#             # Set marker properties
#             marker.scale.x = 0.2  # Arrow shaft length
#             marker.scale.y = 0.05  # Arrow width
#             marker.scale.z = 0.05  # Arrow height
            
#             marker.color.a = 1.0  # Alpha (visibility)
#             marker.color.r = 1.0  # Red color
#             marker.color.g = 0.0
#             marker.color.b = 0.0
            
#             marker.lifetime = rospy.Duration(0)  # Persistent marker
#             marker_pub.publish(marker)
#             marker_id += 1  # Increment marker ID for uniqueness
        
#         rate.sleep()  # Maintain publishing rate

# if __name__ == "__main__":
#     trajectory_file = "/home/ysw/catkin_ws/src/faster-lio/Log/traj.txt"  # Change this to your actual file path
#     publish_markers(trajectory_file)


