cmake_minimum_required(VERSION 3.0.2)
project(husky_dwa_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base
  dwa_local_planner
  pcl_ros
  nodelet
  pointcloud_to_laserscan
  costmap_2d
  dynamic_reconfigure
  geographic_msgs
  geodesy
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    tf
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    move_base
    dwa_local_planner
    pcl_ros
    nodelet
    pointcloud_to_laserscan
    costmap_2d
    dynamic_reconfigure
    geographic_msgs
    geodesy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python 3 스크립트 설치 설정
catkin_install_python(
  PROGRAMS 
    scripts/navigation_manager.py
    scripts/path_visualizer.py
    scripts/gps_server.py
    scripts/initialize_pose.py
    scripts/make_frame.py
    scripts/move_front.py
    scripts/sync2.py
    scripts/waypoints_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 설정 파일 및 런치 파일 설치
install(
  DIRECTORY 
    launch 
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Python 스크립트에 실행 권한 부여
install(
  PROGRAMS
    scripts/navigation_manager.py
    scripts/path_visualizer.py
    scripts/gps_server.py
    scripts/initialize_pose.py
    scripts/make_frame.py
    scripts/move_front.py
    scripts/sync2.py
    scripts/waypoints_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)