/*
    <Pipeline>
(active bag file) --> [faster-LIO]  --(/Odometry) --> [robot]

    <To-do>
0. [Data preparation](kitti format)
- save poses.txt of seq00 ~ seq9?
        kitti format: 12 elements
        - example for 1st line of seq00
        1 9.31323e-10 -3.27418e-11 0 -9.31323e-10 1 -4.65661e-10 7.45058e-09 1.09139e-11 -9.31323e-10 1 0
    
    * Quaternion to Rotation Transfomation
    * use state_point_ in lasermapping.cc:Line805 --> directly return rot_mat from euler_angle...
    * https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
    * ************************ BACK TO quat2rot ********************
    * < writing rot_mat to poses.txt may disturb odometry >
    
    *           [sematicKITTI poses.txt format]
        * x-axis: right
        * y-axis: downward
        * z-axis: forward
    *


- calib.txt: 1 0 0 0 0 1 0 0 0 0 1 0

cf)
poses = inverse(calib) * pose * calib
4*4 matrix = (4*4)*(4*4)*(4*4)

1. record bag file with moving robot(faster-LIO must be on to read active bag file & publish pose)

2. add robot node.
robot node subscribes /Odometry topic that faster-LIO publishes.
--> to help path-planning

    <ref>
AMCL, /tf: https://www.reddit.com/r/ROS/comments/oreiz0/getting_the_current_position_of_the_robot/
parse_poses: https://github.com/PRBonn/semantic-kitti-api/issues/78
quat2rot: https://github.com/MichaelGrupp/evo/issues/153
quat2rot wiki: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
calculator: https://www.andre-gaschler.com/rotationconverter/
visualier tool: https://eater.net/quaternions/video/doublecover
    <Issues>
1. AMCL: noetic built-in? don't know whether it's faster than LIO or not yet.
2. simply using TF topic?; don't need LIO algo(need transformation among frames)
    /tf vs faster-LIO vs DLIO ??? compare odometry

    <Questions>
1. can i use lidar with AMCL?
2. /tf vs /Odometry(faster-LIO)

*/



/*
    Command scenario(not sure)

$ roscore
$ rosbag record /ouster/points /ouster/imu -O test.bag
$ roslaunch faster_lio mapping_ouster32.launch
$ rosbag play test.bag.active --clock --topics /ouster/points /ouster/imu

*/