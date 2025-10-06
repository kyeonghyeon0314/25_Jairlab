# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 코드베이스 개요

이 저장소는 전북대학교에서 개발한 **맵리스(Map-less) 자율주행 로봇 시스템**입니다. Husky 로봇 플랫폼 기반으로 고정밀 GPS와 LiDAR를 활용한 자율주행 시스템을 구현하며, 사전 맵 생성 없이 실시간 내비게이션이 가능합니다.

### 주요 구성 요소

- **faster-lio**: 개량된 Faster-LIO SLAM 시스템 (맵 생성 기능 제거, x-y 평면 정사영)
- **gazebo_dwa**: Gazebo 시뮬레이션 환경과 DWA 로컬 플래너
- **husky**: Clearpath Husky 로봇 공식 패키지
- **husky_robot**: 실제 하드웨어용 패키지
- **ouster-ros**: Ouster LiDAR 드라이버
- **ublox_f9p**: u-blox F9P GPS 수신기 드라이버

## 빌드 및 실행 명령어

### 워크스페이스 빌드
```bash
cd ~/husky_ws
catkin_make
source devel/setup.bash
```

### 필수 의존성 설치
```bash
sudo apt-get install ros-noetic-nmea-msgs \
                     ros-noetic-gazebo-ros \
                     ros-noetic-dwa-local-planner \
                     ros-noetic-move-base \
                     ros-noetic-pcl-ros \
                     ros-noetic-pointcloud-to-laserscan \
                     ros-noetic-costmap-2d \
                     ros-noetic-husky-description \
                     ros-noetic-husky-gazebo \
                     ros-noetic-husky-control

pip install utm
```

### 시뮬레이션 실행 (권장 순서)
```bash
# 1. Ouster LiDAR 실행
roslaunch husky_dwa_navigation ouster_topics_only.launch

# 2. GPS 실행 (실제 하드웨어용)
roslaunch ublox_gps ublox_device.launch

# 3. 통합 내비게이션 시스템 (프레임 설정, waypoints, global_path, gps+Odom)
roslaunch husky_dwa_navigation integrated_navigation.launch

# 4. 메인 제어 시스템 (faster-lio, DWA, 로봇 스폰)
roslaunch husky_dwa_navigation husky_control_nav_localization.launch
```

### Gazebo 시뮬레이션용
```bash
# 시뮬레이션 환경 스폰
roslaunch husky_dwa_navigation husky_gazebo_spawn.launch

# DWA 및 localization
roslaunch husky_dwa_navigation husky_control_nav_localization.launch

# 시각화 및 waypoints
cd ~/husky_ws/src/gazebo_dwa/husky_dwa_navigation/scripts
python3 path_visualizer.py
python3 publish_waypoints_sim.py
```

### 실제 로봇 운영용
```bash
# 초기 Heading 보정을 위한 직선 주행
python3 ~/husky_ws/src/gazebo_dwa/husky_dwa_navigation/scripts/move_front.py

# 백 파일 재생시 토픽 리매핑
rosrun husky_dwa_navigation sync2.py
rosbag play <*.bag> /ouster/points:=/new_points /ouster/imu:=/new_imu
```

## 아키텍처 구조

### 핵심 시스템 흐름
1. **센서 데이터 수집**: Ouster LiDAR + u-blox F9P GPS
2. **Localization**: Faster-LIO (개량) + GPS 융합
3. **Global Planning**: 웹 인터페이스를 통한 GPS 기반 waypoint 생성
4. **Local Planning**: DWA 알고리즘 기반 장애물 회피
5. **Control**: Twist 명령 발행 및 로봇 제어

### 주요 노드 구조 (integrated_navigation.launch)
- `make_frame_node`: UTM 로컬 좌표계 설정 및 GPS-FasterLIO 동기화
- `initialize_pose_node`: FasterLIO-GPS 융합 위치 추정 및 Heading 보정  
- `path_visualizer_node`: 경로 및 웨이포인트 시각화
- `gps_server_node`: 웹 인터페이스 및 GPS 서버 (포트: 8000, 웹소켓: 8765, 8766)

### 데이터 처리 파이프라인 (husky_control_nav_localization.launch)
1. **센서 전처리**: VoxelGrid 필터 (leaf_size: 0.10) + Z축 필터링 (-0.85~2m)
2. **포인트클라우드 변환**: PointCloud2 → LaserScan 변환
3. **내비게이션**: Move Base + DWA Local Planner
4. **시각화**: RViz (faster_lio 설정 파일 사용)

## 프레임 구조
- `base_link`: 로봇 베이스 프레임
- `ouster`: LiDAR 센서 프레임  
- `gps`: GPS 안테나 프레임
- `utm`: UTM 좌표계 글로벌 프레임
- `map`: Faster-LIO 맵 프레임

## 개발 시 주의사항

### Faster-LIO 수정사항
- 맵 생성 알고리즘 제거됨
- Hash map capacity를 100,000으로 축소
- z축 정사영으로 pose 추출
- GPS와의 융합을 위한 프레임 변환 로직 포함

### 실제 운영 시
- 초기 heading 보정을 위해 직선 주행 필요
- Ouster IMU 사용으로 인한 초기 heading 오차 존재
- GPS RTK 고정 해상도 필요 (정확도 향상)

### 시뮬레이션 vs 실제
- 시뮬레이션: `use_sim_time: true`
- 실제 하드웨어: `use_sim_time: false`
- 백 파일 재생 시 토픽 리매핑 필요

## 웹 인터페이스
- 메인 포트: 8000 (GPS 좌표 선택 UI)
- 웹소켓 포트: 8765, 8766 (실시간 데이터 통신)
- 카카오 내비 API 활용한 경로 계획

## 성능 특성
- DWA 파라미터: costmap 기반 동적 윈도우 접근법
- 센서 융합: GPS + LiDAR 기반 robust localization  
- 실시간 처리: 10Hz 제어 주기
- 안전성: 다층 costmap 기반 충돌 회피

## 스크립트 역할 정의

### 🌐 웹 인터페이스 및 데이터 수집
- **`gps_server.py`**: 웹 인터페이스(포트 8000, 웹소켓 8765/8766)를 제공하고, 사용자가 선택한 GPS 좌표를 `/waypoints` 토픽으로 발행한다.

### 🎯 자율주행 실행 엔진
- **`waypoints_manager.py`** (구 waypoints_generator.py): `/waypoints` 토픽을 구독하여 GPS 좌표를 UTM Local로 변환하고, navigation_manager와 연동하여 순차적 자율주행을 실행한다. 네비게이션 상태 관리, 웨이포인트 시각화(`/kakao_waypoints_viz`), 진행률 모니터링을 담당한다. **첫 번째 웨이포인트는 건너뛰고 두 번째부터 처리하여 현재 위치 혼선을 방지**한다.

### 🗺️ 좌표계 및 위치 관리
- **`make_frame.py`**: UTM 로컬 좌표계를 설정하고 GPS-FasterLIO 동기화를 담당한다. 전체 시스템의 프레임 변환 기준점을 제공한다.

- **`initialize_pose.py`**: `/ublox/fix` GPS와 `/Odometry`(Faster-LIO)를 융합하여 `/fused_odom`을 발행한다. **첫 번째 GPS 신호로 UTM 원점을 설정하고 `/utm_origin_info` 토픽으로 발행**한다. 초기 위치 설정 및 move_front.py 실행 감지를 통한 heading 보정 기능을 포함한다.

### 🚗 로봇 제어
- **`move_front.py`**: 초기 heading 보정을 위해 로봇을 직진 이동시킨다. initialize_pose.py가 이를 감지하여 방향 보정을 수행한다.

### 📊 시각화 및 모니터링
- **`path_visualizer.py`**: GPS 경로(`/ublox/fix`)와 보정된 경로(`/fused_odom`)를 시각화한다. initialize_pose.py에서 설정된 UTM 원점 정보(`/utm_origin_info`)를 수신하여 동기화하며, 웨이포인트 시각화 기능을 포함한다.

### 🎮 내비게이션 관리
- **`navigation_manager.py`** (구 navigation_manager_node.py): DWA 내비게이션 시스템 관리 및 costmap 경계 모니터링을 담당한다. waypoints_manager로부터 `/waypoint_goal` 토픽을 수신하여 단일 목표 최적화 및 장애물 회피를 처리한다.

### ⏰ 데이터 동기화
- **`sync2.py`**: 백 파일 재생 시 타임스탬프 동기화를 처리한다. `/ouster/points:=/new_points` 등의 토픽 리매핑과 함께 사용된다.

## Localization 아키텍처

### 센서 데이터 플로우
```
u-blox GPS → /ublox/fix → initialize_pose.py → UTM 원점 설정 → /utm_origin_info
                                 ↓
Ouster LiDAR → Faster-LIO → /Odometry → initialize_pose.py
                                 ↓
GPS + Faster-LIO 융합 → /fused_odom (보정됨)
                                 ↓
waypoints_manager.py (주 위치 소스) + path_visualizer.py (시각화)
                                 ↓
navigation_manager.py (목표 최적화 및 장애물 회피)
                                 ↓
move_base (DWA 로컬 플래너)
```

### Navigation 토픽 흐름
```
waypoints_manager.py → /waypoint_goal → navigation_manager.py
                                           ↓
navigation_manager.py → /move_base_simple/goal → move_base
```

### Faster-LIO 설정 위치
- **실행**: `husky_control_nav_localization.launch:78` → `faster-lio/launch/mapping_ouster32.launch`
- **설정**: `faster-lio/config/ouster32.yaml` (SLAM 파라미터)
- **출력**: `/Odometry` 토픽 (UTM Local 좌표)

### Navigation Manager 특징
- costmap 경계 외부 목표에 대한 중간 목표 생성
- 경로 계획 실패 시 자동 목표 위치 조정 (측면 이동)
- move_base 상태 모니터링 및 오류 복구