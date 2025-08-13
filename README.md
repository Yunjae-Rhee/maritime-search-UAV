**폴더 구성**

- 초기 세팅  
  - `route0.csv`  
    : 처음에 Mission Planner로 만든 기준 경로(.csv)  
  - `identifier.py`  
    : 영상 & 학습 데이터 → 객체 탐지 → `prob{n}.csv` 생성  
  - `route_update.py` 
    : `route{n}.wp` + `prob{n}.csv` → 새로운 경로 생성 → 기체에 업데이트 & `route{n}.wp` 덮어쓰기  
  - `training_data/`  
    : 학습용 이미지·레이블 등 원본 데이터 저장소  
  - `visualize.py`  
    : 확률 구간 및 경로를 지도 위에 시각화  



- 실행하며 추가되는 파일(로그)  
  - `prob1.csv`, `prob2.csv`, …  
  - `route0.wp`, `route1.wp`, …  
