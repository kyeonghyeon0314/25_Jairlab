import os
import argparse
from tf.transformations import quaternion_matrix

def process_sequence(input_path, output_path):
    with open(input_path, 'r') as f_in, open(output_path, 'w') as f_out:
        for line in f_in:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) != 8:
                continue
                
            x, y, z = parts[1:4]
            qx, qy, qz, qw = parts[4:8]
            
            # Convert quaternion to rotation matrix
            R = quaternion_matrix([qx, qy, qz, qw])
            
            # Format as KITTI pose line
            pose_line = f"{R[0,0]} {R[0,1]} {R[0,2]} {x} " \
                        f"{R[1,0]} {R[1,1]} {R[1,2]} {y} " \
                        f"{R[2,0]} {R[2,1]} {R[2,2]} {z}"
            f_out.write(pose_line + '\n')

def main():
    parser = argparse.ArgumentParser(description='Convert trajectory files to KITTI poses')
    parser.add_argument('input_dir', help='Directory containing traj_XX.txt files')
    parser.add_argument('output_dir', help='Output directory for sequences')
    args = parser.parse_args()

    for seq in range(11):
        seq_str = f"{seq:02d}"
        input_file = os.path.join(args.input_dir, f"traj_{seq_str}.txt")
        output_dir = os.path.join(args.output_dir, "sequences", seq_str)
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, "poses.txt")
        
        if os.path.exists(input_file):
            process_sequence(input_file, output_file)
            print(f"Processed sequence {seq_str}")
        else:
            print(f"Warning: Missing input file for sequence {seq_str}")

if __name__ == "__main__":
    main()

# python traj_to_poses.py ~/catkin_ws/src/faster-lio/Log/traj .