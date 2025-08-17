## 📂 폴더 구성

my-project/  
├─ **run_mission.py**  
│  └─ 비행 시작, 확률 데이터(`prob`) 감시, `route_update` 실행  
├─ **route_update.py**  
│  └─ 확률 데이터(`prob`) → 폴리곤/경로 생성, 드론 업로드, 로그 저장  
├─ **identifier.py**  
│  └─ 영상 객체탐지 → `probN.csv` 생성  
├─ **visualize.py**  
│  └─ 대시보드 표시 (현재 위치 + 폴리곤 + 경로 + 확률점 지도)  
└─ **log/**  
  ├─ **route0.wp**    # 초기 경로 (최소 1개 필요)  
  ├─ **prob1.csv**   # (예시) 확률 데이터  
  ├─ **polygon0.txt** # (있으면 좋음, 없어도 됨)  
  └─ (실행 중 자동 생성) `route1.wp`, `polygon1.txt`, `live_state.json`, `track.csv`, ...  
