# CONTENTS
- Global Path Planning[[Link](#global-path-planning)]
- Semantic Segmentation[[Link](#semantic-segmentation)]
- Local Path Planning[[Link](#local-path-planning)]


# Global Path Planning
## references
[아무거나]()
- Kakao API 활용 waypoints 생성

# Semantic Segmentation
## references
- [LaserMix](https://github.com/ldkong1205/LaserMix)
- [FRNet](https://github.com/Xiangxu-0103/FRNet)
## 적용 형식
- LaserMix의 teacher-student network 반지도학습 프레임을 이용
- teacher network에 선행학습된 checkpoints 적용
- 클래수 MMdetection 기준의 20개에서 5개로 변경 (road, car, sidewalk, other-vehicle, unlabeled)
<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="/img/semantic_segmentation/lasermix.png" width="40%">
    <p style="text-align: center;">수정된 LaserMix</p>
</div>

### 개선 사항
- 아직 학습중

### 코드
[FRNet-LaserMix](https://github.com/kyeonghyeon0314/FRNet-LaserMix)


  
# Local Path Planning
## Local Path Planner[[RL-DWA](https://github.com/BlackTea12/RL-DWA)]
## Localization



# 아직 지우지 마세요 나중에 활용 예정
<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="/img/semantic_segmentation/root_directory.png" width="70%">
    <p style="text-align: center;">최상위 디렉토리</p>
  </div>
  <div style="margin-bottom: 10px;">
    <img src="/img/semantic_segmentation/save_directory.png" width="70%">
    <p style="text-align: center;">tmp</p>
  </div>
</div>


## 맵 정보
- [**mapping_01.pcd**](/img/map/mapping_01.png) : 6호관 7호관 사이
- [**mapping_02.pcd**](/img/map/mapping_02.png) : 제2도서관 연구단지? 쪽 길
- [**mapping_03.pcd**](/img/map/mapping_03.png) : 농대 쪽 큰길 -> 중도 정문쪽
- [**mapping_04.pcd**](/img/map/mapping_04.png) : 건지광장
- [**mapping_05.pcd**](/img/map/mapping_05.png) : 7호관 -> 2호관 -> 4호관 -> 3호관 -> 공대입구 -> 내리막
- [**mapping_06.pcd**](/img/map/mapping_06.png) : 자연대 본관과 3호관
- [**mapping_07.pcd**](/img/map/mapping_07.png) : 공대 입구-> 내리막 -> 진수당 한바퀴
- [**mapping_08.pcd**](/img/map/mapping_08.png) : 진수당 주차장 -> 법대 오르막 오른뒤 한바퀴-> 본부별관 앞 주차장 순회
- [**mapping_09.pcd**](/img/map/mapping_09.png) : 법대,글로벌 인재관 사이 오르막 오르고난 후 장소 -> 제2도서관 (loop closer 없음)
- [**mapping_10.pcd**](/img/map/mapping_10.png) : 경상대 2호관 에서 법대내리막
- [**mapping_11.pcd**](/img/map/mapping_11.png) : 공대 공장동 앞 -> 7호관 - > 6호관-> 제2도서관-> 연구실 단지 -> 후생관쪽 문-> 농대길->중도 정문 -> 후생관 -> 경상대 2호관-> 인문대 -> 실크로드 센터 -> 공대 오르막
