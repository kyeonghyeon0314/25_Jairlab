# 전북대학교 자율주행 로봇 시스템

이 저장소는 전북대학교 자율주행 연구실에서 개발한 혁신적인 **맵리스(Map-less) 자율주행 시스템**입니다. 기존 SLAM 기반 시스템과 달리, **고정밀 GPS/RTK 위치 추정**과 **실시간 라이다 센싱**을 결합하여 사전 맵 생성 없이도 안전하고 정확한 자율주행을 구현합니다. 본 시스템은 도심 환경에서의 실용적 자율주행을 목표로 하며, 동적 환경 변화에 강인한 적응형 내비게이션 알고리즘을 특징으로 합니다.


- 해당 저장소는 gazebo 시뮬레이션 상의 완성 본입니다.  
**[gazebo_dwa](https://github.com/kyeonghyeon0314/gazebo_dwa/tree/gps_localization)**


## 시스템 요구사항
- Ubuntu 20.04 LTS
- ROS Noetic


## 🎯 시스템 개요

본 시스템은 다음과 같은 주요 기능을 제공합니다:

- **전역 경로 계획**: 웹 기반 GPS 좌표 수집 및 ROS 토픽 발행
- **의미론적 분할**: FRNet 기반 LaserMix 반지도 학습을 통한 주행가능 영역 분류
- **지역 경로 계획**: DWA(Dynamic Window Approach)를 통한 장애물 회피
- **Localization**: Faster-LIO를 통한 경량 라이다-관성 오도메트리
- **위치 추정**: GPS와 Ouster 라이다 센서 통합

## 🚀 사용법
실행 순서
```bash
roslaunch husky_dwa_navigation ouster_topics_only.launch               # Ouster 실행
roslaunch ublox_gps ublox_device.launch                                # GPS 실행
# 설치 필요
sudo apt-get install ros-noetic-nmea-msgs


roslaunch husky_dwa_navigation integrated_navigation.launch            # 프레임 설정 , waypoints, global_path, gps+Odom
roslaunch husky_dwa_navigation husky_control_nav_localization.launch   # faster-lio, DWA, 로봇 스폰 등등
python3 move_front.py                                                  # 직선주행으로 초기 Heading 맞추기 
```

## 🔧 핵심 구성요소

### 1. 전역 경로 계획 (Global Path Planning)
카카오 내비 API를 활용한 웹 기반 GPS 좌표 수집 시스템

**작성자의 저장소**: 
[global_path_planner](https://github.com/kdh044/global_path)

**주요 스크립트**:
- `gps_server.py`: 목적지 선택을 위한 웹 UI 서버
- `gps_publisher.py`: GPS 좌표 발행자
- `waypoints_generator.py`: 웨이포인트 생성 및 발행

### 2. 의미론적 분할 (Semantic Segmentation) (아직 추가하지 못함 , 개별 진행)
LaserMix 기반 반지도 학습을 통한 포인트 클라우드 분류

**주요 참고 자료**:
- [LaserMix](https://github.com/ldkong1205/LaserMix)
- [FRNet](https://github.com/Xiangxu-0103/FRNet)

**작성자의 저장소**:
- [FRNet_ROS : tomark](https://github.com/t0mark/FRNet_ROS)
- [FRNet-LaserMix : kyeonghyeon0314](https://github.com/kyeonghyeon0314/FRNet-LaserMix)

**개선 사항**:
- 클래스 수를 20개에서 5개로 축소 (road, car, sidewalk, other-vehicle, unlabeled)
- IoU 점수 향상: 67.45% → 69.32%, 61.93% → 69.32%
- 최종 mIoU 점수: 87.67% (원논문 대비 13% 향상)


### 3. 지역 경로 계획 (Local Path Planning)
DWA 알고리즘을 이용한 장애물 회피 및 지역 경로 계획

**작성자의 저장소**: 
- [gazebo_dwa : kyeonghyeon0314](https://github.com/kyeonghyeon0314/gazebo_dwa/tree/main)
- [local_path_planner : kdh044](https://github.com/kdh044/Jbnu-Final/tree/main)


### 4. Localization (Faster-LIO)
Faster-LIO의 Iterated Error State Kalman Filter를 그대로 사용
**변경사항** :
- Hash map capacity를 100,000d으로 축소
- z축으로 정사영
- map 생성 알고리즘 삭제