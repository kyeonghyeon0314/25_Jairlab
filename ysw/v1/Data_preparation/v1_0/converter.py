#!/usr/bin/env python3     # need this line?
# .pcd -> .bin & traj.txt -> poses.txt converter
import os
import numpy as np
import argparse
from glob import glob
# from pypcd import pypcd  # pip install pypcd
import open3d as o3d
from tf.transformations import quaternion_matrix    # use library

def process_trajectory(traj_file, output_pose_file):
    """
    Read a trajectory file (with lines like:
    "#timestamp x y z q_x q_y q_z q_w")
    and write a poses.txt file where each line is formatted as:
    "R11 R12 R13 T1 R21 R22 R23 T2 R31 R32 R33 T3".
    This function uses each quaternion (order: qx, qy, qz, qw) to compute a rotation matrix.
    """
    with open(traj_file, 'r') as f_in, open(output_pose_file, 'w') as f_out:
        for line in f_in:
            # Skip comments
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            # Extract translation (x, y, z) and quaternion components (qx, qy, qz, qw)
            x, y, z = parts[1:4]
            qx, qy, qz, qw = parts[4:8]
            x, y, z = float(x), float(y), float(z)
            q = [float(qx), float(qy), float(qz), float(qw)]
            R = quaternion_matrix(q)
            # Format the KITTI pose: 3x3 rotation matrix followed by translation vector.
            pose_line = f"{R[0,0]} {R[0,1]} {R[0,2]} {x} " \
                        f"{R[1,0]} {R[1,1]} {R[1,2]} {y} " \
                        f"{R[2,0]} {R[2,1]} {R[2,2]} {z}"
            f_out.write(pose_line + "\n")

def convert_pcd_to_bin(pcd_file, bin_file):
    """
    Load a .pcd file, extract its 'x', 'y', 'z' and 'intensity' columns,
    and save the data in KITTI .bin format: each point is 4 float32 values.
    If 'intensity' is missing, zeros are used.
    """
    pc = pypcd.PointCloud.from_path(pcd_file)
    data = pc.pc_data
    # Convert x,y,z arrays to float32
    x = data['x'].astype(np.float32)
    y = data['y'].astype(np.float32)
    z = data['z'].astype(np.float32)
    # Intensity is used if available; otherwise fill with zeros.
    if 'intensity' in data.dtype.names:
        intensity = data['intensity'].astype(np.float32)
    else:
        intensity = np.zeros_like(x, dtype=np.float32)
    # Combine the points into an (N, 4) array
    points = np.column_stack((x, y, z, intensity))
    # Write the binary data directly in float32 representation
    points.tofile(bin_file)

def convert_pcd2bin(pcd_path, bin_path):
    # Load the PCD file
    pcd = o3d.io.read_point_cloud(pcd_path)
    
    # Convert to NumPy array (N, 3)
    points = np.asarray(pcd.points, dtype=np.float32)
    
    # Check if intensity exists (SemanticKITTI format requires it)
    if pcd.has_colors():
        intensity = np.mean(np.asarray(pcd.colors, dtype=np.float32), axis=1, keepdims=True)  # Use color intensity
    else:
        intensity = np.zeros((points.shape[0], 1), dtype=np.float32)  # Default to zero if no intensity
    
    # Stack into (N, 4) -> (x, y, z, intensity)
    points = np.hstack((points, intensity))
    
    # Save as binary file
    points.tofile(bin_path)
    print(f"Saved {bin_path} with {points.shape[0]} points.")

def process_sequence(seq_id, input_dir, output_dir):
    """
    Process one sequence given its two-digit id.
    Assumes that:
      - The trajectory file is at: input_dir/traj_{seq_id}.txt
      - All .pcd files for that sequence are in the folder: input_dir/{seq_id}/
    The output is written to:
      output_dir/sequences/{seq_id}/poses.txt for poses and
      output_dir/sequences/{seq_id}/ouster/*.bin for bin files.
    """
    # Trajectory file
    traj_file = os.path.join(input_dir, f"traj_{seq_id}.txt")
    # PCD files folder for the sequence
    pcd_folder = os.path.join(input_dir, seq_id)
    
    # Create output directories: sequences/{seq_id}/ouster/
    output_seq_dir = os.path.join(output_dir, "sequences", seq_id)
    ouster_dir = os.path.join(output_seq_dir, "ouster")
    os.makedirs(ouster_dir, exist_ok=True)
    
    # Process the trajectory file to create poses.txt.
    output_pose_file = os.path.join(output_seq_dir, "poses.txt")
    if os.path.exists(traj_file):
        process_trajectory(traj_file, output_pose_file)
        print(f"Processed trajectory for sequence {seq_id}")
    else:
        print(f"Warning: Missing trajectory file for sequence {seq_id}")
    
    # Find and sort .pcd files (e.g. scans_1.pcd, scans_2.pcd, ...).
    pcd_files = sorted(glob(os.path.join(pcd_folder, "*.pcd")))
    for idx, pcd_file in enumerate(pcd_files):
        # Create file name with six digits (e.g., 000000.bin).
        bin_filename = f"{idx:06d}.bin"
        bin_file_path = os.path.join(ouster_dir, bin_filename)
        # convert_pcd_to_bin(pcd_file, bin_file_path)
        convert_pcd2bin(pcd_file, bin_file_path)
    print(f"Converted {len(pcd_files)} PCD files to BIN for sequence {seq_id}")

def main():
    parser = argparse.ArgumentParser(
        description="Convert PCD point clouds and trajectory files to KITTI format."
    )
    parser.add_argument(
        "input_dir",
        help="Input directory containing trajectory files (traj_XX.txt) and sequence folders (e.g., '00', '01', ...)"
    )
    parser.add_argument(
        "output_dir",
        help="Output directory where sequences folder will be created (with poses.txt and ouster/*.bin files)"
    )
    parser.add_argument(
        "--num_seq",
        type=int,
        default=11,
        help="Number of sequences to process (default is 11)."
    )
    args = parser.parse_args()
    
    # Loop over the sequence IDs (formatted as two-digit strings)
    for seq in range(args.num_seq):
        seq_str = f"{seq:02d}"
        process_sequence(seq_str, args.input_dir, args.output_dir)

if __name__ == "__main__":
    main()

# python3 converter.py /home/ysw/catkin_ws/src/faster-lio/ori_data_by_fasterlio /home/ysw/catkin_ws/src/faster-lio/data_by_bag_bin
