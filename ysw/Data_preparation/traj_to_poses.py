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
            
            # Format as SemanticKITTI pose line
            pose_line = f"{R[0,0]} {R[0,1]} {R[0,2]} {x} " \
                        f"{R[1,0]} {R[1,1]} {R[1,2]} {y} " \
                        f"{R[2,0]} {R[2,1]} {R[2,2]} {z}"
            f_out.write(pose_line + '\n')

def main():
    parser = argparse.ArgumentParser(description='Convert trajectory files to KITTI poses')
    parser.add_argument('input_dir', help='Directory containing traj_XX.txt files')
    parser.add_argument('output_dir', help='Output directory for sequences')
    args = parser.parse_args()

    for seq in range(20):
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

# python traj_to_poses.py ~/catkin_ws/src/faster-lio/Log .

# Troubleshooting(unstable pose at starting point)
        # 1. change line1~? of poses.txt to "1 0 0 0 0 1 0 0 0 0 1 0"(adjustable)
                # seq01(1~10), 
        # 2. insert "1 0 0 0 0 1 0 0 0 0 1 0" atop poses.txt if needed(fit with number of bin files)
# i.e. seq01 bag: bin_num(from pure bag): 9803     VS      poses.txt' line_num: 9795
# poses.txt would be like..
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 1 0 0 0 0 1 0 0 0 0 1 0
# 0.9999521509632338 0.009656937324509002 -0.0015618404247895994 0.148824297361743 -0.00965703061295838 0.9999533684302349 -5.2199303973866693e-05 -0.028313803528974 0.0015612635083120103 6.727954708220075e-05 0.9999987789641146 -0.003908984862445
# ...