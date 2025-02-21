#!/usr/bin/env python
import rospy
import message_filters
# import numpy as np
from sensor_msgs.msg import PointCloud2

# Load poses from file
def load_poses(file_path):
    """Loads poses from poses_00.txt and stores them with timestamps."""
    poses = []
    with open(file_path, 'r') as f:
        for line in f:
            values = list(map(float, line.strip().split()))
            if len(values) != 13:
                continue  # Ensure correct format
            
            # Store pose as (timestamp, flattened pose matrix)
            poses.append(values)
    return poses

# Save the synchronized poses to a new file
def save_poses(file_path, synchronized_poses):
    """Saves synchronized poses to a new file in the same format."""
    with open(file_path, 'w') as f:
        for pose in synchronized_poses:
            line = ' '.join(map(str, pose))
            f.write(line + '\n')

# Synchronization class
class PoseSynchronizer:
    def __init__(self, poses, output_file):
        self.poses = poses  # Loaded poses (each is a list of 12 values)
        self.output_file = output_file
        self.synchronized_poses = []  # Store new poses matched to point clouds

        rospy.init_node('pose_synchronizer', anonymous=True)

        # Subscriber for PointCloud2
        self.pc_sub = message_filters.Subscriber('/ouster/points', PointCloud2)

        # Approximate time synchronization (adjust slop for accuracy)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.pc_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.callback)

    def callback(self, pc_msg):
        """Synchronizes poses with PointCloud2 messages."""
        # Convert ROS timestamp to float (seconds)
        pc_timestamp = pc_msg.header.stamp.to_sec()

        # Find closest matching pose
        closest_pose = self.find_closest_pose(pc_timestamp)
        self.synchronized_poses.append(closest_pose)

        rospy.loginfo(f"Synchronized {len(self.synchronized_poses)} frames")

    def find_closest_pose(self, timestamp):
        """Find the closest pose based on timestamp."""
        min_diff = float('inf')
        closest_pose = self.poses[0]  # Default to first pose; need for init_
        
        for pose in self.poses:
            pose_timestamp = pose[0]  # First value in each line is the timestamp
            diff = abs(pose_timestamp - timestamp)
            if diff < min_diff:
                min_diff = diff
                closest_pose = pose

        return closest_pose

    def run(self):
        """Runs the synchronizer and waits for shutdown."""
        rospy.spin()
        save_poses(self.output_file, self.synchronized_poses)
        rospy.loginfo(f"New synchronized poses saved to {self.output_file}")

if __name__ == "__main__":
    poses_file = "/path/to/poses_00.txt"  # Change to actual file path
    output_file = "/path/to/new_poses_00.txt"  # Output file

    poses = load_poses(poses_file)
    if not poses:
        rospy.logerr("No valid poses found! Check the input file.")
        exit(1)

    sync = PoseSynchronizer(poses, output_file)
    sync.run()


# cd ~/catkin_ws/src && catkin_create_pkg time_synchronizer rospy cv_bridge tf message_filters sensor_msgs
# cd time_synchronizer/src && touch time_synchronizer.py && chmod +x time_synchronizer.py
# gedit ../package.xml    # fix...
# gedit ../CMakeLists.txt # fix...
# cd ~/catkin_ws && catkin_make --only-pkg-with-deps time_synchronizer

# roscore
# rosrun time_synchronizer time_synchronizer.py
# rosbag play *.bag --clock --topics /ouster/points