import rosbag
import os
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
import argparse

def process_pointcloud(msg):
    """
    Convert PointCloud2 message into a numpy array of [x, y, z, intensity].
    """
    points = []
    for point in read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
        points.append([point[0], point[1], point[2], point[3]])  # x, y, z, intensity
    return np.array(points, dtype=np.float32)

def save_pointcloud(bin_file, points):
    """
    Save point cloud data as a .bin file.
    """
    points.tofile(bin_file)

def main():
    parser = argparse.ArgumentParser(description="Convert ROS bag files to SemanticKITTI format.")
    parser.add_argument("--sequence", type=str, required=True, help="Sequence ID (e.g., '00', '01').")
    parser.add_argument("--bag", type=str, required=True, help="Path to the ROS bag file.")
    parser.add_argument("--output_dir", type=str, default="dataset/sequences", help="Base output directory.")

    args = parser.parse_args()

    sequence_id = args.sequence
    bag_file = args.bag
    output_dir = os.path.join(args.output_dir, sequence_id)

    velodyne_dir = os.path.join(output_dir, "velodyne")
    os.makedirs(velodyne_dir, exist_ok=True)

    print(f"Processing bag file: {bag_file}")
    
    bag = rosbag.Bag(bag_file)
    idx = 0

    start_time = None

    for topic, msg, t in bag.read_messages(topics="/ouster/points"):  # Replace with your PointCloud2 topic name
        # if isinstance(msg, PointCloud2):
        if idx >= 0:
            points = process_pointcloud(msg)
            bin_file = os.path.join(velodyne_dir, f"{idx:06d}.bin")
            save_pointcloud(bin_file, points)
            print(f"Saved: {bin_file}")

            idx += 1

        # # Save timestamps
        # times_file = os.path.join(output_dir, "times.txt")
        # with open(times_file, "a") as f:
        #     f.write(f"{t.to_sec()}\n")

        # calib.txt: extrinsic between os1-32 & imu <not sure>

    bag.close()
    print(f"Finished processing. {idx} files saved in {velodyne_dir}.")

if __name__ == "__main__":
    main()

# understanding imu data(not sure)
    # x: up or down
    # y: right or left
    # z: forward


# python3 prepare_dataset_v0.py --sequence 00 --bag /home/ysw/source/lidar/2024-03-16-15-07-14.bag --output_dir ./dataset/sequences/
