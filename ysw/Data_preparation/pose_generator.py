"""
Usage:
python pose_generator.py start_index num_interpolated input_file output_file

Example:
python pose_generator.py 1 5 \
/home/ysw/source/semanticKitti/custom/semantickitti/sequences/test/11/poses.txt \
./poses.txt

This script reads a poses.txt file where each line is formatted as:
R11 R12 R13 T1 R21 R22 R23 T2 R31 R32 R33 T3

It then inserts “num_interpolated” poses between the given pair:
(line start_index) and (line start_index+1)

The rotations are smoothly interpolated using SLERP and the translations linearly.
"""

import sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

def parse_pose_line(line):
    """
    Given a line of 12 values (as a string) in the format:
    R11 R12 R13 T1 R21 R22 R23 T2 R31 R32 R33 T3
    returns a tuple (R, T) where
    R is a 3x3 NumPy array (rotation matrix) and
    T is a 3-element NumPy vector (translation).
    """
    parts = line.strip().split()
    if len(parts) != 12:
        raise ValueError("Each pose line must have 12 float values.")
    values = [float(x) for x in parts]
    # Assemble the rotation matrix from the appropriate values.
    # Format: row1: (R11,R12,R13), row2: (R21,R22,R23), row3: (R31,R32,R33)
    R = np.array([
            [values[0], values[1], values[2]],
            [values[4], values[5], values[6]],
            [values[8], values[9], values[10]]])
    # Translations: T1, T2, T3 are at indices 3, 7, 11
    T = np.array([values[3], values[7], values[11]])
    return R, T

def pose_to_line(R, T):
    """
    Given a rotation matrix R (3x3) and translation vector T (3,),
    returns a string in the format:
    "R11 R12 R13 T1 R21 R22 R23 T2 R31 R32 R33 T3"
    with 8 decimal places for each number.
    """
    # We follow the order: row1 (R[0, :], then T), row2, row3.
    values = []
    for i in range(3):
        # Append row of the rotation matrix.
        values.extend(R[i, :].tolist())
        # Then append the ith element of the translation.
        values.append(T[i])
    return " ".join(f"{val:.8f}" for val in values)

def main():
    if len(sys.argv) != 5:
        print("Usage: python pose_generator.py start_index num_interpolated input_file output_file")
        sys.exit(1)

    try:
        # start_index is assumed to be 1-based. We want to insert new poses between
        # the pose at start_index and the one at start_index+1.
        start_index = int(sys.argv[1])
        num_interp = int(sys.argv[2])
    except ValueError:
        print("start_index and num_interpolated must be integers.")
        sys.exit(1)

    input_file = sys.argv[3]
    output_file = sys.argv[4]

    # Read all the lines from the input file.
    with open(input_file, 'r') as f:
        lines = f.readlines()

    # Convert the 1-based start_index to a 0-based index.
    start_idx = start_index - 1

    # We need to have at least two lines to interpolate between.
    if start_idx < 0 or start_idx + 1 >= len(lines):
        print("Error: start_index is out of range or there is no following line to interpolate to.")
        sys.exit(1)

    # Parse the two poses (the pair between which we insert interpolated ones)
    R1, T1 = parse_pose_line(lines[start_idx])
    R2, T2 = parse_pose_line(lines[start_idx + 1])

    # Prepare times: t=0 for the first pose and t=1 for the second.
    key_times = [0,1]
    # Create a Rotation object for each of the two poses.
    rots = Rotation.from_matrix(np.array([R1, R2]))     # pip install --upgrade scipy

    # Set up a SLERP interpolator
    slerp = Slerp(key_times, rots)

    # Create equally spaced interpolation times (include endpoints if needed)
    # Here we want only the N in-between poses, so exclude endpoints.
    interp_times = np.linspace(0, 1, num=num_interp + 2)[1:-1]

    # Get interpolated rotations
    interp_rots = slerp(interp_times)
    # For translations, perform linear interpolation:
    interp_translations = [(1 - t) * T1 + t * T2 for t in interp_times]

    # Format the newly computed interpolated poses into lines.
    interp_lines = []
    for rot_matrix, trans_vector in zip(interp_rots.as_matrix(), interp_translations):
        line = pose_to_line(rot_matrix, trans_vector)
        interp_lines.append(line + "\n")

    # Create the new file contents:
    # -  Keep all poses up to and including the start_index-th pose.
    # -  Then add the interpolated poses.
    # -  Then add the remainder of the file (starting from the original start_index+1).
    new_lines = []
    new_lines.extend(lines[:start_idx + 1])
    new_lines.extend(interp_lines)
    new_lines.extend(lines[start_idx + 1:])

    # Write the new file to the given output path.
    with open(output_file, 'w') as f:
        f.writelines(new_lines)

    print(f"Successfully inserted {num_interp} interpolated poses between line {start_index} and line {start_index + 1} into:")
    print(f"  {output_file}")

if __name__ == "__main__":
    main()