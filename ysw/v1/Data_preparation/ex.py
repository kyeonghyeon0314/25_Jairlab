# ref
# https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
# https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
# https://github.com/MichaelGrupp/evo/issues/153
# https://github.com/MichaelGrupp/evo/blob/master/evo/core/transformations.py#L1258

import numpy
import math

_EPS = numpy.finfo(float).eps * 4.0
print(_EPS) # 8.881784197001252e-16

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> numpy.allclose(M, numpy.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True

    """
    q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
    n = numpy.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = numpy.outer(q, q)
    return numpy.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0]],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0]],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]]])

import numpy as np

def quaternion_to_rotation_matrix(q_x, q_y, q_z, q_w):
    """
    Convert a quaternion (q_x, q_y, q_z, q_w) into a 3x3 rotation matrix.
    """
    R = np.array([
        [1 - 2 * (q_y**2 + q_z**2), 2 * (q_x * q_y - q_z * q_w), 2 * (q_x * q_z + q_y * q_w)],
        [2 * (q_x * q_y + q_z * q_w), 1 - 2 * (q_x**2 + q_z**2), 2 * (q_y * q_z - q_x * q_w)],
        [2 * (q_x * q_z - q_y * q_w), 2 * (q_y * q_z + q_x * q_w), 1 - 2 * (q_x**2 + q_y**2)]
    ])
    return R

def v3(quaternion):
    """Convert quaternion to 3x3 rotation matrix"""
    q = np.array(quaternion, dtype=np.float64)
    # n = np.dot(q, q)
    n = q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2

    print('np.dot VS manual')
    print('np.dot: ', np.dot(q,q))
    print('mul manually: ', n)
    # if n < _EPS:
    #     return np.identity(3)
    if n == 0:
        return np.identity(3)
    
    s = 2 / n  # Scale factor
    
    print('math.sqrt precision check')
    print('math.sqrt()**2: ',math.sqrt(s)**2)
    print(s)
    
    return  np.array([
        [1 - s*(q[1]**2 + q[2]**2), s * (q[0] * q[1] - q[2] * q[3]), s * (q[0] * q[2] + q[1] * q[3])],
        [s * (q[0] * q[1] + q[2] * q[3]), 1 - s * (q[0]**2 + q[2]**2), s * (q[1] * q[2] - q[0] * q[3])],
        [s * (q[0] * q[2] - q[1] * q[3]), s * (q[1] * q[2] + q[0] * q[3]), 1 - s * (q[0]**2 + q[1]**2)]
    ])
test = [1, 0, 0, 0]
test_q = numpy.array(test, dtype=numpy.float64, copy=True)
test_n = numpy.dot(test_q, test_q)  # test_n: 1
test_q *= math.sqrt(2.0 / test_n)
print(test_q)   # [1.41421356 0.         0.         0.        ] ????
test_q = numpy.outer(test_q, test_q)
print(test_q)
# [[2. 0. 0. 0.]
#  [0. 0. 0. 0.]
#  [0. 0. 0. 0.]
#  [0. 0. 0. 0.]]

# 100th line of traj.txt created by faster-LIO with first bag file; q_x, q_y, q_z, q_w
# change order to [q_w, q_x, q_y, q_z]
quat = [0.999390925644008, -0.008357784211309, -0.004638522929725, -0.033562021520099]
rot = quaternion_matrix(quat)
print('\n'*3, rot)
    # [[ 0.99770415  0.0671607  -0.00871039]
    #  [-0.06700562  0.99760748  0.01701674]
    #  [ 0.0098324  -0.01639403  0.99981726]]

q = [0, 0, 0, 1]
print('\n', quaternion_matrix(q))

# Example usage:
q_x, q_y, q_z, q_w = 0.0, 0.0, 0.0, 1.0  # Identity quaternion
rotation_matrix = quaternion_to_rotation_matrix(q_x, q_y, q_z, q_w)
print(rotation_matrix)

q_1 = [1,2,3]
q_2 = [4,5,6]
print(np.outer(q_1, q_2))

# check : https://www.andre-gaschler.com/rotationconverter/

########################### DEBUGGING #####################

eps_limit_check = 0.000000000000001
if(eps_limit_check < _EPS):
    print("AAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHH")
print('\n'*5)
print("calculation check for seq00")
print("seq00",": first pose: ")
x, y, z, qx, qy, qz, qw = 0.001703091881709, -0.002019366481713, 0.001319611142677, -0.000079097287848, 0.000012570071147, -0.000012058879051, 0.999999996720098
print(x,y,z,qx,qy,qz,qw)

quat = [qx, qy, qz, qw]
print(v3(quat))
