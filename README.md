# ilegal_parking_scooter_dectecion
## Reference
- [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
- 미정
## Contents
- mapping
- Localization
- Semantic Segmantation

# mapping([LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM))
맵핑한 후 pcd 파일로 변환하여 저장

## 진행과정
### /laser_cloud_surround Topic을 활용한 pcd 저장
```
roslaunch lego_loam run.launch
rosbag play lidar_5.bag --clock --topic /ouster/points
rosbag record -O mapping_data.bag /laser_cloud_surround
rosrun pcl_ros bag_to_pcd mapping_data.bag /laser_cloud_surround  # 마지막으로 저장되는 pcd 파일이 전체 map
```
### 코드 수정을 통한 pcd 저장(mapOptmization.cpp 의 public에 추가)
```
def mapSave(){

}
```
## 맵 정보
- **mapping_01.pcd** : 6호관 7호관 사이
- **mapping_02.pcd** : 제2도서관 연구단지? 쪽 길
- **mapping_03.pcd** : 농대 쪽 큰길 -> 중도 정문쪽
- **mapping_04.pcd** : 건지광장
- **mapping_05.pcd** : 7호관 -> 2호관 -> 4호관 -> 3호관 -> 공대입구 -> 내리막
- **mapping_06.pcd** : 자연대 본관과 3호관
- **mapping_07.pcd** : 공대 입구-> 내리막 -> 진수당 한바퀴
- **mapping_08.pcd** : 진수당 주차장 -> 법대 오르막 오른뒤 한바퀴-> 본부별관 앞 주차장 순회
- **mapping_09.pcd** : 법대,글로벌 인재관 사이 오르막 오르고난 후 장소 -> 제2도서관 (loop closer 없음)
- **mapping_10.pcd** : 경상대 2호관 에서 법대내리막
- **mapping_11.pcd** : 공대 공장동 앞 -> 7호관 - > 6호관-> 제2도서관-> 연구실 단지 -> 후생관쪽 문-> 농대길->중도 정문 -> 후생관 -> 경상대 2호관-> 인문대 -> 실크로드 센터 -> 공대 오르막

<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_01.png" width="70%">
    <p style="text-align: center;">mapping_01</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_02.png" width="70%">
    <p style="text-align: center;">mapping_02</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_03.png" width="70%">
    <p style="text-align: center;">mapping_03</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_04.png" width="70%">
    <p style="text-align: center;">mapping_04</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_05.png" width="70%">
    <p style="text-align: center;">mapping_05</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_06.png" width="70%">
    <p style="text-align: center;">mapping_06</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_07.png" width="70%">
    <p style="text-align: center;">mapping_07</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_08.png" width="70%">
    <p style="text-align: center;">mapping_08</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_09.png" width="70%">
    <p style="text-align: center;">mapping_09</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_10.png" width="70%">
    <p style="text-align: center;">mapping_10</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/map/mapping_11.png" width="70%">
    <p style="text-align: center;">mapping_11</p>
  </div>
</div>

