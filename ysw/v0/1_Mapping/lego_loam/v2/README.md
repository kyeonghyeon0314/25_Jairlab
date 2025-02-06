# faster-lio 시도해봤다가 1180/3293 지점 왜곡과 큰 용량으로 인해 다시 돌아옴.
- "ouster1-32" 환경에 맞게 수정하기

## References
- [ouster version](https://airlab-jbnu.notion.site/Faster-LIO-fbea2fdcf87f4994b64a49db2afb7538)
- [#134](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/13); uncomment line159 in imageProjection.cpp
- [OS1-32 spec](https://data.ouster.io/downloads/datasheets/datasheet-rev7-v3p1-os1.pdf)
#### utility.h::ang_bottom: Field of View/Vertical; vary by version, but line65...
- [About groundScanInd](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/169)
- [#185](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/185); 11번째 bag 맵핑 잘 안 되면 추후에 useClosedRing=false로?
- [About loop-closure(#234)](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/234); loopClosureEnableFlag: false >> true?
- [ROS docs](https://docs.ros.org/en/melodic/api/ouster_driver/html/point__os1_8h_source.html#l00018)
- [#266](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/266)
- [#163](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/163)

    cd ~/catkin_ws && catkin_make
    roslaunch lego_loam run.launch
    rosbag play [bag_name].bag --clock    // 맵핑할 때 /ouster/imu도 사용
- 일단 01, 09, 11번째 pcd부터 맵핑
* 11번째 pcd는 cornerMap, surfaceMap, trajectory, finalMap 모두 저장

# 수정한 부분
- utility.h; imuTopic 지정 & os1-32 설정
- imageProjection.cpp; uncomment line166

===========================================================


## todo: 맵핑 잘 안될 경우
1. usedClosedRing, loopClosureEnableFlag 값 바꿔서 실행해보기
2. extern const float historyKeyframeFitnessScore = 0.1;
3. ang_bottom 값 변경: 16.6+0.1 > 21.2+0.1
4. PointXYZIRPYT 구조체 변경?
...
10. ouster1-32에 맞게 featureAssociation.cpp 알고리즘 수정

## 1st_try(finalCloud_01.pcd, finalCloud09.pcd, rosgraph_01.png)
- distortion_01.png(1000s쯤부터 이상해짐) 

## 2nd_try: useClosedRing = false
- rosbag play .. -s 1000; 괜찮아 보였으나(play_1000.png), 중도 건물이 더 찌그러져서(distortion_00002.png) 서로 다른 길이 겹침(distortion_03.png)

## 3rd_try: useClosedRing = true로 되돌리고 historyKeyframeFitnessScore 값 줄인(0.3->0.1) 상태로 imu 안 씀(rosbag play .. --topics /ouster/points)
- 중도 더 찌그러짐(distortion_04.png)

## 4th_try: historyKeyframeFitnessScore = 0.3으로 되돌림
    $ rosbag play .. --topics /ouster/points
- 여전히 이상함(distortion_05.png)
- 6 dof imu 쓰면서 용량 적게 pcd 저장하는 방식(ALOAM, fast-lio2 ...) 찾아보면서[fast-lio2](https://github.com/engcang/SLAM-application) feature-extraction algorithm 수정하는 방향 모색하도록 결정

## 5th_try: useClosedRing, ang_bottom 값 변경
- useClosedRing = false, ang_bottom = 21.2+0.1, rosbag play [11th].bag --clock
- distortion_06.png
- useClosedRing, ang_bottom : set to default

## 10th_try: Feature-Extraction algorithm 수정
| Velodyne VLP-16 | Ouster1-32 |
| ------ | ------ |
| 9 dof | 6 dof |


