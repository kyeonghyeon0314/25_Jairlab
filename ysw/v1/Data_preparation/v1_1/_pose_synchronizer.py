#!/usr/bin/env python3
import os
import rospy
import message_filters
import argparse
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_matrix

def process_sequence(input_path):
    """
    Loads the trajectory file (assumed to have lines:
    "#timestamp x y z q_x q_y q_z q_w")
    and converts each quaternion into a 3x3 rotation matrix.
    Returns a list of poses; each pose is a list:
      [timestamp, R11, R12, R13, T1,
                 R21, R22, R23, T2,
                 R31, R32, R33, T3]
    """
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
            # Create a pose line in KITTI format (excluding any calibration transforms)
            pose_line = [timestamp,
                         R[0,0], R[0,1], R[0,2], x,
                         R[1,0], R[1,1], R[1,2], y,
                         R[2,0], R[2,1], R[2,2], z]
            poses.append(pose_line)
    return poses

def interpolate_pose(prev_pose, next_pose, alpha):
    """
    Linearly interpolates (element-wise) between two poses (lists).
    Assumes the first element is timestamp and the remainder are
    the pose values. (Note: For rotation, proper interpolation (slerp)
    is more accurate but here we assume small differences.)
    """
    return [(1 - alpha) * p + alpha * q for p, q in zip(prev_pose, next_pose)]

def synchronize_poses_with_pcd(poses, pcd_timestamps):
    """
    For each pcd timestamp, finds the two trajectory poses that bracket it
    and interpolates between them.
    Returns a list of synchronized poses (with the timestamp removed),
    so that there is one output pose per PointCloud2 message.
    """
    synchronized = []
    num_poses = len(poses)
    pose_idx = 0  # index in the trajectory list
    for pcd_time in pcd_timestamps:
        # Advance so that poses[pose_idx] is the last pose with timestamp <= pcd_time.
        while pose_idx < num_poses - 1 and poses[pose_idx + 1][0] <= pcd_time:
            pose_idx += 1
        if pose_idx == num_poses - 1:
            # No future pose available – use the last pose.
            interp_pose = poses[-1]
        else:
            prev_pose = poses[pose_idx]
            next_pose = poses[pose_idx + 1]
            t_prev, t_next = prev_pose[0], next_pose[0]
            # Compute interpolation factor (avoid division by zero)
            alpha = (pcd_time - t_prev) / (t_next - t_prev + 1e-6)
            interp_pose = interpolate_pose(prev_pose, next_pose, alpha)
        # Exclude the timestamp from the synchronized output
        synchronized.append(interp_pose[1:])
    return synchronized

class PoseSynchronizer:
    def __init__(self, poses, output_file):
        self.poses = poses
        self.output_file = output_file
        self.pcd_timestamps = []
        self.received_messages = 0

        rospy.init_node('pose_synchronizer', anonymous=True)
        # Subscribe to the /ouster/points topic.
        self.pc_sub = message_filters.Subscriber('/ouster/points', PointCloud2)
        # Use ApproximateTimeSynchronizer with a generous queue/slop,
        # so that every point cloud gets its timestamp recorded.
        self.ts = message_filters.ApproximateTimeSynchronizer([self.pc_sub],
                                                             queue_size=50,
                                                             slop=0.2)
        self.ts.registerCallback(self.callback)

    def callback(self, pc_msg):
        self.pcd_timestamps.append(pc_msg.header.stamp.to_sec())
        self.received_messages += 1

    def run(self):
        rospy.loginfo("Waiting for PointCloud2 messages...")
        # Wait for the first message to arrive.
        while self.received_messages == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Received {} PointCloud2 messages.".format(self.received_messages))
        # Optionally, wait a short period to collect a full set if needed.
        rospy.sleep(1.0)
        # Synchronize the trajectory poses with the collected pcd timestamps.
        synchronized_poses = synchronize_poses_with_pcd(self.poses, self.pcd_timestamps)
        # Ensure we have exactly one pose per point cloud (if not, the scheme below fills extra lines).
        if len(synchronized_poses) < len(self.pcd_timestamps):
            # This case should not happen if every PC message triggers callback – but to be safe:
            diff = len(self.pcd_timestamps) - len(synchronized_poses)
            for _ in range(diff):
                synchronized_poses.append(self.poses[-1][1:])

        # Write the synchronized poses to the output file.
        with open(self.output_file, 'w') as f:
            for pose in synchronized_poses:
                f.write(" ".join(map(str, pose)) + "\n")
        rospy.loginfo("Synchronized poses saved to {}".format(self.output_file))
        rospy.signal_shutdown("Synchronization complete")

def main():
    parser = argparse.ArgumentParser(
        description="Synchronize trajectory file poses with LiDAR (PointCloud2) timestamps")
    parser.add_argument("input_dir", help="Directory containing traj.txt file")
    parser.add_argument("output_dir", help="Output directory for synchronized poses")
    args = parser.parse_args()

    input_file = os.path.join(args.input_dir, "traj.txt")
    output_seq_dir = os.path.join(args.output_dir, "00")
    os.makedirs(output_seq_dir, exist_ok=True)
    output_file = os.path.join(output_seq_dir, "poses.txt")
    
    if os.path.exists(input_file):
        poses = process_sequence(input_file)
        if poses:
            synchronizer = PoseSynchronizer(poses, output_file)
            synchronizer.run()
            rospy.spin()
        else:
            rospy.logerr("No valid poses found in trajectory file.")
    else:
        rospy.logerr("Trajectory file {} does not exist.".format(input_file))

if __name__ == "__main__":
    main()
