import os
import sys
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import open3d as o3d

def create_pointcloud2(points, frame_id="map"):
    """Create a sensor_msgs/PointCloud2 message from an array of points."""
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    header = rospy.Header(frame_id=frame_id)
    return point_cloud2.create_cloud(header, fields, points)

def save_accumulated_pcd(points, output_file):
    """Save accumulated points to a .pcd file."""
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(output_file, point_cloud)
    rospy.loginfo(f"Saved accumulated point cloud to: {output_file}")

def publish_and_accumulate(bin_dir, publisher, output_pcd_file):
    """Publish and accumulate all .bin files in the directory as PointCloud2."""
    bin_files = sorted([os.path.join(bin_dir, f) for f in os.listdir(bin_dir) if f.endswith('.bin')])
    if not bin_files:
        rospy.logerr(f"No .bin files found in directory: {bin_dir}")
        return

    rospy.loginfo(f"Publishing and accumulating {len(bin_files)} .bin files from {bin_dir}")
    accumulated_points = []

    for file in bin_files:
        rospy.loginfo(f"Processing: {file}")
        points = np.fromfile(file, dtype=np.float32).reshape(-1, 4)[:, :3]  # Extract XYZ
        accumulated_points.append(points)

        # Publish for RViz visualization
        cloud_msg = create_pointcloud2(points)
        cloud_msg.header.stamp = rospy.Time.now()
        publisher.publish(cloud_msg)
        rospy.sleep(0.5)  # Delay to allow RViz visualization

    # Save accumulated points to .pcd
    all_points = np.vstack(accumulated_points)
    save_accumulated_pcd(all_points, output_pcd_file)

if __name__ == "__main__":
    rospy.init_node('bin_to_pointcloud2', anonymous=True)
    
    # Check for CLI arguments
    if len(sys.argv) != 2:
        print("Usage: python bin_to_pointcloud2.py <path_to_bin_files>")
        sys.exit(1)

    bin_dir = sys.argv[1]
    if not os.path.isdir(bin_dir):
        print(f"Error: {bin_dir} is not a valid directory.")
        sys.exit(1)

    output_pcd_file = os.path.join(bin_dir, "accumulated_pointcloud.pcd")
    pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)

    try:
        publish_and_accumulate(bin_dir, pub, output_pcd_file)
    except rospy.ROSInterruptException:
        print("ROS node interrupted. Exiting.")

# roscore
# rviz
# python3 check_pcd.py ./dataset/sequences/00/velodyne/
