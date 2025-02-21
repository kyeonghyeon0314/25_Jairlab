#!/usr/bin/env python
import os
import rospy
import message_filters
import argparse
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_matrix

def process_sequence(input_path):
    """Converts trajectory files to SemanticKITTI pose format (R11 R12 R13 T1 ...)."""
    poses = []
    with open(input_path, 'r') as f_in:
        for line in f_in:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) != 8:
                continue
                
            timestamp, x, y, z = map(float, parts[0:4])
            qx, qy, qz, qw = map(float, parts[4:8])

            # Convert quaternion to 3x3 rotation matrix
            R = quaternion_matrix([qx, qy, qz, qw])[:3, :3]  

            # Store pose with timestamp
            pose_line = [timestamp] + R.flatten().tolist() + [x, y, z]
            poses.append(pose_line)

    return poses

def synchronize_poses_with_pcd(poses, pcd_timestamps):
    """Matches each PointCloud2 frame with a corresponding pose."""
    synchronized_poses = []
    pose_idx = 0

    for pcd_time in pcd_timestamps:
        while pose_idx < len(poses)-1 and poses[pose_idx + 1][0] < pcd_time:
            pose_idx += 1

        if pose_idx < len(poses)-1:
            prev_pose = poses[pose_idx]
            next_pose = poses[pose_idx + 1]
            alpha = (pcd_time - prev_pose[0]) / (next_pose[0] - prev_pose[0] + 1e-6)
            interp_pose = generate_pseudo_pose(prev_pose, next_pose, alpha)
        else:
            interp_pose = poses[-1]  # Repeat last pose if no future poses exist

        synchronized_poses.append(interp_pose[1:])  # Exclude timestamp

    return synchronized_poses

def generate_pseudo_pose(prev_pose, next_pose, alpha):
    """Interpolates between two poses."""
    return [(1 - alpha) * p1 + alpha * p2 for p1, p2 in zip(prev_pose, next_pose)]

class PoseSynchronizer:
    def __init__(self, poses, output_file):
        self.poses = poses  
        self.output_file = output_file
        self.pcd_timestamps = []  
        self.synchronized_poses = []  
        self.received_messages = 0  # Track how many PointCloud2 messages are received

        rospy.init_node('pose_synchronizer', anonymous=True)
        self.pc_sub = message_filters.Subscriber('/ouster/points', PointCloud2)
        # adjustment
        # default: queue_size=10, slop=0.1
        self.ts = message_filters.ApproximateTimeSynchronizer([self.pc_sub], queue_size=10, slop=1)
        self.ts.registerCallback(self.callback)

    def callback(self, pc_msg):
        """Collects PointCloud2 timestamps and ensures messages are received before processing."""
        self.pcd_timestamps.append(pc_msg.header.stamp.to_sec())
        self.received_messages += 1

    def run(self):
        """Waits for enough PointCloud2 messages before processing synchronization."""
        rospy.loginfo("Waiting for PointCloud2 messages...")

        # Wait until at least one message is received
        while self.received_messages == 0 and not rospy.is_shutdown():
            # adjustment
            # default: 0.1
            rospy.sleep(0.0001)  # Prevent busy waiting

        rospy.loginfo(f"Received first PointCloud2 message. Synchronizing poses...")

        # Synchronize after collecting all messages
        rospy.spin()  # Keep listening until ROS shuts down

        rospy.loginfo("Processing collected data...")
        self.synchronized_poses = synchronize_poses_with_pcd(self.poses, self.pcd_timestamps)

        with open(self.output_file, 'w') as f:
            for pose in self.synchronized_poses:
                f.write(" ".join(map(str, pose)) + '\n')

        rospy.loginfo(f"Synchronized poses saved to {self.output_file}")
        rospy.signal_shutdown("Synchronization complete")

def main():
    parser = argparse.ArgumentParser(description="Convert and synchronize trajectory files with LiDAR point clouds")
    parser.add_argument("input_dir", help="Directory containing traj.txt files")
    parser.add_argument("output_dir", help="Output directory for synchronized poses")
    args = parser.parse_args()

    input_file = os.path.join(args.input_dir, f"traj.txt")
    output_dir = os.path.join(args.output_dir, "00")
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "poses.txt")

    if os.path.exists(input_file):
        poses = process_sequence(input_file)
        if poses:
            sync = PoseSynchronizer(poses, output_file)
            sync.run()
            rospy.spin()  # Keep the node alive until `rosbag play` finishes
        print(f"Processed and synchronized one sequence")
    else:
        print(f"Warning: Missing input file for sequence")

if __name__ == "__main__":
    main()

# cd ~/catkin_ws/src
# catkin_create_pkg pose_synchronizer rospy tf message_filters sensor_msgs
# cd pose_synchronizer/src && touch pose_synchronizer.py && chmod +x pose_synchronizer.py
# gedit ../CMakeLists.txt # fix... catkin_package(), catkin_install_python()
# cd ~/catkin_ws && catkin_make --only-pkg-with-deps pose_synchronizer

# roscore
# rosrun pose_synchronizer pose_synchronizer.py /home/ysw/catkin_ws/src/faster-lio/ori_data_by_fasterlio/using_dedicated /home/ysw/catkin_ws/src/faster-lio/data_by_bag_bin/using_dedicated
# rosbag play /home/ysw/source/lidar/2024-03-16-15-07-14.bag --clock --topics /ouster/points
