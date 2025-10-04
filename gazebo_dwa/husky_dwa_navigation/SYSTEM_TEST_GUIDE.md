# 🚀 Husky DWA Navigation 시스템 테스트 가이드

## 📋 시스템 개요

완전히 통합된 자율주행 시스템:
- **Hardware**: Husky UGV + GPS + IMU + LiDAR
- **Localization**: Dual EKF (Local + Global) with GPS fusion
- **Navigation**: DWA Local Planner + Move Base
- **Interface**: Web-based waypoint management

---

## 🔧 1. 시스템 실행

### **단일 Launch 파일로 전체 시스템 실행**

```bash
roslaunch husky_dwa_navigation integrated_navigation.launch
```

### **실행되는 노드들**:

#### **1. Hardware & Control**
- `husky_node`: Husky 하드웨어 드라이버
- `base_controller_spawner`: Wheel controller
- `robot_state_publisher`: Robot TF 발행
- `twist_mux`: 조이스틱 우선순위 제어

#### **2. Localization** (핵심!)
- `navsat_transform`: GPS → UTM 변환
- `ekf_local`: Local EKF (odom 프레임, Wheel + IMU)
- `ekf_global`: Global EKF (map 프레임, Local EKF + GPS)

#### **3. Sensor Processing**
- `voxel_grid`: PointCloud downsampling
- `pointcloud_to_laserscan`: LaserScan 변환

#### **4. Navigation**
- `move_base`: DWA navigation
- `navigation_manager`: Dynamic planner switching

#### **5. Web Interface**
- `path_visualizer_node`: 경로 시각화
- `gps_server_node`: 웹 서버 (http://localhost:8000)
- `waypoint_manager_node`: 자율주행 관리

---

## 🔍 2. 시스템 진단

### **진단 스크립트 실행**

```bash
cd /home/kimkh/Jongseol/25_Jairlab/gazebo_dwa/husky_dwa_navigation/scripts
python3 diagnose_localization.py
```

### **정상 출력 예시**

```
🔧 노드 실행 상태 검사
✅ Local EKF (ekf_local): 실행 중
✅ Global EKF (ekf_global): 실행 중
✅ GPS → UTM 변환 (navsat_transform): 실행 중

📡 센서 토픽 검사
✅ /ublox/fix: 데이터 수신 중
✅ /ouster/imu: 데이터 수신 중
✅ /husky_velocity_controller/odom: 데이터 수신 중
✅ /odometry/filtered/local: 데이터 수신 중
✅ /gps/fix/odometry: 데이터 수신 중
✅ /odometry/filtered/global: 데이터 수신 중

🔍 TF 트리 구조 검사
✅ map → odom TF 존재
✅ odom → base_link TF 존재

🎯 근본 원인 분석
✅ 모든 시스템 정상!
```

---

## 📊 3. 주요 토픽 확인

### **Localization 토픽**

```bash
# GPS 입력
rostopic echo /ublox/fix

# IMU 입력
rostopic echo /ouster/imu

# Wheel Odometry 입력
rostopic echo /husky_velocity_controller/odom

# Local EKF 출력 (odom 프레임)
rostopic echo /odometry/filtered/local

# navsat_transform 출력 (GPS → map 프레임)
rostopic echo /gps/fix/odometry

# Global EKF 출력 (map 프레임) - Navigation에서 사용
rostopic echo /odometry/filtered/global
```

### **Navigation 토픽**

```bash
# Move base goal
rostopic echo /move_base/goal

# Move base status
rostopic echo /move_base/status

# Waypoint goal (from waypoints_manager)
rostopic echo /waypoint_goal
```

---

## 🗺️ 4. TF 트리 확인

### **TF 트리 구조** (정상 상태)

```
map (Global EKF가 발행)
 └─ odom (Global EKF가 발행: map → odom)
     └─ base_link (Local EKF가 발행: odom → base_link)
         └─ ... (robot links)
```

### **TF 확인 명령어**

```bash
# TF 트리 전체 확인
rosrun tf view_frames
evince frames.pdf

# 특정 TF 확인
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map base_link
```

---

## 🌐 5. 웹 인터페이스 사용

### **접속**
```
http://localhost:8000
```

### **기능**
1. **실시간 GPS 위치 표시**
2. **Kakao Map에서 waypoints 설정**
3. **자율주행 시작/중지**
4. **경로 시각화**

### **Waypoint 추가 방법**
1. 웹 브라우저에서 http://localhost:8000 접속
2. Kakao Map에서 목표 지점 클릭
3. Waypoints 생성
4. "Start Navigation" 버튼 클릭

---

## ⚠️ 6. 문제 해결

### **문제 1: EKF 노드가 실행되지 않음**

**증상**:
```
❌ Local EKF (ekf_local): 실행되지 않음
❌ Global EKF (ekf_global): 실행되지 않음
```

**원인**:
- `husky_node` 실패 (하드웨어 미연결)
- Config 파일 경로 오류

**해결**:
```bash
# 노드 로그 확인
rosnode list
rosnode info ekf_local

# Launch 파일 재실행
roslaunch husky_dwa_navigation integrated_navigation.launch
```

---

### **문제 2: map 프레임이 존재하지 않음**

**증상**:
```
❌ map → odom TF 타임아웃
[move_base] canTransform: target_frame map does not exist
```

**원인**: Global EKF가 작동하지 않음

**해결**:
```bash
# 1. GPS 데이터 확인
rostopic hz /ublox/fix

# 2. Local EKF 출력 확인
rostopic hz /odometry/filtered/local

# 3. GPS odometry 확인
rostopic hz /gps/fix/odometry

# 4. Global EKF 로그 확인
rosnode info ekf_global
```

---

### **문제 3: Wheel Odometry 없음**

**증상**:
```
❌ /husky_velocity_controller/odom: 데이터 없음 (타임아웃)
```

**원인**: Husky 하드웨어 드라이버 실패

**해결**:
```bash
# 1. Husky 연결 확인
ls /dev/ttyUSB*

# 2. husky_node 로그 확인
rosnode info husky_node

# 3. 시뮬레이션 테스트 (하드웨어 없이)
# integrated_navigation.launch에서 husky_node 주석 처리
# 대신 시뮬레이션 odometry 발행
```

---

## ✅ 7. 정상 작동 체크리스트

### **Localization**
- [ ] GPS 데이터 수신 중 (`/ublox/fix`)
- [ ] IMU 데이터 수신 중 (`/ouster/imu`)
- [ ] Wheel Odometry 수신 중 (`/husky_velocity_controller/odom`)
- [ ] Local EKF 출력 (`/odometry/filtered/local`)
- [ ] GPS Odometry 출력 (`/gps/fix/odometry`)
- [ ] Global EKF 출력 (`/odometry/filtered/global`)
- [ ] `map → odom` TF 발행 중
- [ ] `odom → base_link` TF 발행 중

### **Navigation**
- [ ] `/scan` 토픽 수신 중 (LaserScan)
- [ ] Move base 실행 중
- [ ] Costmap 생성 중 (RViz에서 확인)
- [ ] Goal 발행 가능

### **Web Interface**
- [ ] http://localhost:8000 접속 가능
- [ ] GPS 위치 표시됨
- [ ] Waypoints 설정 가능
- [ ] 자율주행 시작 가능

---

## 📈 8. 성능 모니터링

### **Localization 정확도**

```bash
# Global EKF 공분산 확인 (낮을수록 정확)
rostopic echo /odometry/filtered/global | grep -A 36 covariance
```

### **CPU/메모리 사용률**

```bash
# 전체 노드 리소스 사용률
rosnode list | xargs -I {} rosnode info {} | grep -E "Node|Pid"
top -p $(pidof -d',' ekf_local ekf_global navsat_transform)
```

### **토픽 주파수 확인**

```bash
# 센서 주파수
rostopic hz /ublox/fix          # 목표: ~10 Hz
rostopic hz /ouster/imu         # 목표: 100 Hz
rostopic hz /husky_velocity_controller/odom  # 목표: 50 Hz

# EKF 출력 주파수
rostopic hz /odometry/filtered/local   # 목표: 50 Hz
rostopic hz /odometry/filtered/global  # 목표: 30 Hz
```

---

## 🎯 9. 좌표계 구조 (중요!)

### **Localization Pipeline**

```
┌─────────────────────────────────────┐
│ 센서 입력                           │
├─────────────────────────────────────┤
│ /ublox/fix        (GPS)             │
│ /ouster/imu       (IMU)             │
│ /husky_.../odom   (Wheel)           │
└──────────┬──────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│ navsat_transform                    │
│ - GPS → UTM 변환                    │
│ - 출력: /gps/fix/odometry           │
└──────────┬──────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│ Local EKF (odom 프레임)             │
│ - 입력: Wheel + IMU                 │
│ - 출력: /odometry/filtered/local    │
│ - TF: odom → base_link              │
└──────────┬──────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│ Global EKF (map 프레임)             │
│ - 입력: Local EKF + GPS Odometry    │
│ - 출력: /odometry/filtered/global   │
│ - TF: map → odom                    │
└──────────┬──────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│ Move Base (Navigation)              │
│ - Localization: /odometry/.../global│
│ - Costmap: map 프레임               │
└─────────────────────────────────────┘
```

---

## 📞 10. 추가 리소스

### **관련 파일**

- **Launch**: `integrated_navigation.launch`
- **Config**:
  - `ekf_local.yaml`
  - `ekf_global.yaml`
  - `*_costmap_params.yaml`
- **Scripts**:
  - `diagnose_localization.py` (진단 도구)
  - `gps_server.py` (웹 서버)
  - `waypoints_manager.py` (자율주행)

### **로그 위치**

```bash
# ROS 로그
~/.ros/log/latest/

# 특정 노드 로그 확인
roscd
cd log/latest/
grep -r "error" .
```

---

**문서 버전**: 1.0
**최종 업데이트**: 2025-10-05
**작성자**: Claude Code Assistant
