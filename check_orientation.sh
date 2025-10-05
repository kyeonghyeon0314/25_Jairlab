#!/bin/bash
# IMU와 Odometry의 방향 확인 스크립트

echo "=== Checking IMU orientation ==="
rostopic echo /ouster/imu -n 1 | grep -A4 "orientation:"

echo -e "\n=== Checking Odometry orientation (local) ==="
rostopic echo /odometry/filtered -n 1 | grep -A4 "orientation:"

echo -e "\n=== Checking Odometry orientation (global) ==="
rostopic echo /odometry/filtered/global -n 1 | grep -A4 "orientation:"

echo -e "\n=== Checking GPS odometry ==="
rostopic echo /odometry/gps -n 1 | grep -A4 "orientation:"

echo -e "\n=== Convert quaternion to yaw ==="
echo "Use: rosrun tf tf_echo odom base_link"
echo "Or check navsat_transform logs for 'Transform heading factor'"
