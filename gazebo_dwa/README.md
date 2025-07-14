# 개요
gazebo환경에 husky와 velodyne 라이다 추가하고 dwa를 통한 장애물 회피 기동
- faster-lio와 gps를 통해 utm 좌표에 localizaiton 수행

##### 영상 설명
- Bule : Gps
- Red : faster-lio + gps
- Gray : faster-lio

ouster/imu를 활용하다 보니, 초기 heading이 맞지 않는 모습을 보입니다.

<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="docs/video/final_goal.gif" alt="drawing" width="100%"/>
    <p style="text-align: center;">시연영상</p>
  </div>
</div>
<!--
<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="docs/images/rqt.png" width="100%">
    <p style="text-align: center;">rqt_graph</p>
  </div>
</div>
<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="docs/images/laserScan.png" width="100%">
    <p style="text-align: center;">laserScan</p>
  </div>
</div>
<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="docs/video/move.gif" width="100%">
    <p style="text-align: center;">동적 장애물</p>
  </div>
</div> -->
<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="docs/images/rqt_graph_final.png" width="100%">
    <p style="text-align: center;">rqt_graph</p>
  </div>
</div>

# 주요 적용 사항
## main branch 참조 [[Link](https://github.com/kyeonghyeon0314/gazebo_dwa)]
- husky에 HDL-32E 추가
- husky_velodyne.launch 수행내용
- husky_dwa_navigation
- NavigationManager

#### 기술적 부분
faster-lio에서 map을 생성하는 부분을 없애고 x-y평면으로 정상여하여 pose를 추출하게 변형한다. [[팀원2](https://github.com/Cascio99)분이 제작]


# TODO
```
mkdir -p ~/husky_ws/src
cd husky_ws
catkin_make

cd src
git clone https://github.com/kyeonghyeon0314/gazebo_dwa.git -b gps_localization

# 필수 의존성 설치
sudo apt-get install ros-noetic-gazebo-ros \
                     ros-noetic-roscpp \
                     ros-noetic-rospy \
                     ros-noetic-sensor-msgs \
                     ros-noetic-std-msgs \
                     ros-noetic-geometry-msgs \
                     ros-noetic-nav-msgs \
                     ros-noetic-tf \
                     ros-noetic-tf2-ros \
                     ros-noetic-tf2-geometry-msgs \
                     ros-noetic-velodyne-description \
                     ros-noetic-velodyne-gazebo-plugins \
                     ros-noetic-husky-description \
                     ros-noetic-husky-gazebo \
                     ros-noetic-husky-control \
                     ros-noetic-dwa-local-planner \
                     ros-noetic-global-planner \
                     ros-noetic-move-base \
                     ros-noetic-pcl-ros \
                     ros-noetic-nodelet \
                     ros-noetic-pointcloud-to-laserscan \
                     ros-noetic-costmap-2d \
                     ros-noetic-dynamic-reconfigure \
                     ros-noetic-urdf \
                     ros-noetic-xacro
pip install utm

cd ..
catkin_make

# 시뮬레이션 및 로봇 스폰
roslaunch husky_dwa_navigation husky_gazebo_spawn.launch

# dwa및 localization
roslaunch husky_dwa_navigation husky_control_nav_localization.launch

# utm 프레임 및 시각화
cd ~/husky_ws/src/gazebo_dwa/husky_dwa_navigation/scripts
python3 path_visualizer.py

# waypoints 발행
cd ~/husky_ws/src/gazebo_dwa/husky_dwa_navigation/scripts
python3 publish_waypoints_sim.py
```
