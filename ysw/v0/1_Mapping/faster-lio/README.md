## [faster-lio](https://github.com/gaoxiang12/faster-lio/tree/main)
- [LeGO_LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM): 9 dof 사용, loop-clousre 있어야 맵핑 잘함
- faster-lio는 6 dof 사용; rosbag play시 imu(/ouster/imu) 토픽 활용

## build_setting
[link](https://airlab-jbnu.notion.site/Faster-LIO-fbea2fdcf87f4994b64a49db2afb7538) 참고
1. glog error
```
sudo apt-get install libgoogle-glog-dev
```
2. [WARNING] livox_ros_driver is not found in workspace: 추후 확인할 예정

## Mapping
- $ rosbag play *.bag --clock --topics /ouster/points /ouster/imu
- LeGO_LOAM보다 받는 메시지가 첫 bag 파일 기준으로 100배 정도나 큼; 용량은 440 MB(fasterlio.png, vi_cmp.png)
- 11번째 맵 만들 때 많이 뒤틀림(distortion.mp4); loop 아직 안 생겨서 발생?
### Duration: 1180 / 3293 (자연대 본관 쪽에서 중도 가기 위해 꺾는 부분)
* 일단 마지막 11번째 맵핑 시도: 절반도 안 된 시점에서 중단: scan.pcd가 2 GB...

##### rviz lagging fix
- Terminator 우클릭 -> Launch using Dedicated Graphic Cards
- nvidia-smi -l 10: /opt/ros/noetic/lib/rviz/rviz 활성화

- $ rosbag play 11.bag --clock --topics /ouster/points /ouster/imu /utm_deltas

lego_loam; "rosbag play .. --topics /ouster/points ``/ouster/imu``" 돌려보기