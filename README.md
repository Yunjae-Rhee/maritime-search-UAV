**폴더 구성**

your-project/
├─ run_mission.py          # 비행 시작 + prob 감시 + route_update 실행
├─ route_update.py         # prob → 폴리곤/경로 생성 + 드론에 업로드 + 로그저장
├─ identifier.py           # 영상 객체탐지 → probN.csv 생성
├─ telemetry_stream.py     # 드론 현재 위치를 log/live_state.json, log/track.csv로 기록
├─ visualize.py            # 대시보드 (현재위치 + 폴리곤 + 경로 + 확률점 지도)
├─ requirements.txt        # (선택) 의존성 목록
└─ log/
   ├─ route0.wp            # 초기 경로 (최소 1개 필요)
   ├─ prob1.csv            # (예시) 확률 데이터
   ├─ polygon0.txt         # (있으면 좋음, 없어도 됨)
   └─ (실행 중 자동 생성) route1.wp, polygon1.txt, live_state.json, track.csv, ...