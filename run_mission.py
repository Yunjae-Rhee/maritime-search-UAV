# run_mission.py — Single-process Single-connection mission runner
import os, sys, time, signal, re, json, threading, subprocess
from pathlib import Path
from datetime import datetime, timezone
from pymavlink import mavutil

# ================== 설정 ==================
LOG_DIR  = Path.cwd() / "log"
PY       = sys.executable
ROUTE_UPDATE = Path.cwd() / "route_update.py"  # 계산 전용으로 호출: --no-upload
IDENTIFIER   = Path.cwd() / "identifier.py"    # (선택) prob 생성기 — MAVLink 미사용이면 OK

CONN_STR = "udp:127.0.0.1:14550"  # 권장: UDP 포워딩 (시리얼 충돌 회피)
BAUD     = 115200                 # 시리얼을 쓸 때만 의미
REL_ALT  = 10.0                   # 자동이륙 목표 상대고도(m)

POLL_SEC = 2.0            # prob 감시 주기
NO_UPDATE_EXIT_CODE = 99  # route_update가 '업데이트 없음'으로 종료할 때의 코드
# =========================================

# --------- 파일/인덱스 유틸 ---------
def latest_prob_index() -> int:
    mx = -1
    for p in LOG_DIR.glob("prob*.csv"):
        m = re.match(r"prob(\d+)\.csv$", p.name)
        if m:
            mx = max(mx, int(m.group(1)))
    return mx

def latest_route_file() -> Path | None:
    mx, latest = -1, None
    for p in LOG_DIR.glob("route*.wp"):
        m = re.match(r"route(\d+)\.wp$", p.name)
        if m:
            n = int(m.group(1))
            if n > mx:
                mx, latest = n, p
    return latest

def parse_wp_latlonalt_list(wp_path: Path):
    pts = []
    with open(wp_path, "r", encoding="utf-8") as f:
        header = f.readline().strip()
        if header != "QGC WPL 110":
            raise ValueError("Unsupported .wp format (need QGC WPL 110)")
        for line in f:
            parts = line.strip().split("\t")
            if len(parts) < 12:
                continue
            lat = float(parts[8]); lon = float(parts[9]); alt = float(parts[10])
            pts.append((lat, lon, alt))
    return pts[1:]  # 0번(홈) 제외

# --------- MAVLink 동작 ---------
def mav_connect():
    m = mavutil.mavlink_connection(CONN_STR, baud=BAUD)
    m.wait_heartbeat()
    print(f"[run] MAVLink connected: sys={m.target_system} comp={m.target_component}")
    return m

def upload_mission_from_points(m, latlonalt_list):
    ts, tc = m.target_system, m.target_component
    m.mav.mission_clear_all_send(ts, tc)
    total = len(latlonalt_list)
    m.mav.mission_count_send(ts, tc, total)
    sent = 0
    while sent < total:
        req = m.recv_match(type=["MISSION_REQUEST_INT","MISSION_REQUEST"], blocking=True, timeout=5)
        if req is None:
            raise TimeoutError("MISSION_REQUEST timeout")
        seq = req.seq
        lat, lon, alt = latlonalt_list[seq]
        m.mav.mission_item_int_send(
            ts, tc, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0,0,0,0,
            int(lat*1e7), int(lon*1e7), float(alt)
        )
        sent += 1
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    print(f"[run] Mission upload ACK: {getattr(ack, 'type', 'None')}")

def set_mode(m, mode_name: str):
    try:
        m.set_mode_apm(mode_name)
        print(f"[run] Mode -> {mode_name}")
    except Exception as e:
        print(f"[run] Mode set failed: {e}")

def arm_and_takeoff(m, rel_alt: float):
    ts, tc = m.target_system, m.target_component
    # ARM
    m.mav.command_long_send(ts, tc,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1,0,0,0,0,0,0)
    print("[run] ARM requested")
    # GUIDED
    set_mode(m, "GUIDED")
    # TAKEOFF
    m.mav.command_long_send(ts, tc,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            0, 0,0,0,0, 0,0, float(rel_alt))
    print(f"[run] Takeoff to {rel_alt} m requested")

# --------- 텔레메트리 로거(동일 연결 사용) ---------
def telemetry_logger(m: mavutil.mavfile, stop_evt: threading.Event):
    LOG_DIR.mkdir(exist_ok=True)
    state_path = LOG_DIR / "live_state.json"
    track_path = LOG_DIR / "track.csv"
    if not track_path.exists():
        track_path.write_text("timestamp,lat,lon,alt\n", encoding="utf-8")

    while not stop_evt.is_set():
        try:
            msg = m.recv_match(type=["GLOBAL_POSITION_INT","GPS_RAW_INT"], blocking=False)
            if not msg:
                time.sleep(0.1); continue
            now = datetime.now(timezone.utc).isoformat()
            if msg.get_type() == "GLOBAL_POSITION_INT":
                lat = msg.lat/1e7; lon = msg.lon/1e7; alt = msg.relative_alt/1000.0
            elif msg.get_type() == "GPS_RAW_INT":
                lat = msg.lat/1e7; lon = msg.lon/1e7; alt = ""
            else:
                continue
            # 현재 상태(덮어쓰기)
            state = {"timestamp": now, "lat": lat, "lon": lon, "alt": alt if alt != "" else None}
            state_path.write_text(json.dumps(state), encoding="utf-8")
            # 궤적(추가)
            with open(track_path, "a", encoding="utf-8") as f:
                f.write(f"{now},{lat},{lon},{alt}\n")
        except Exception:
            # 로깅은 중단 없이 계속
            time.sleep(0.1)

# --------- 기본 체크 ---------
def ensure_basics():
    LOG_DIR.mkdir(exist_ok=True)
    if not ROUTE_UPDATE.exists():
        print("[run] route_update.py가 없습니다.", file=sys.stderr); sys.exit(1)
    r = latest_route_file()
    if not r:
        print("[run] log/에 route*.wp가 없습니다. (route0.wp를 두거나 MP로 업로드하세요.)",
              file=sys.stderr); sys.exit(1)

# --------- 메인 ---------
def main():
    ensure_basics()
    print("[run] 시작")

    # 0) 최신 route 업로드 + 이륙 + AUTO
    route_path = latest_route_file()
    print(f"[run] 초기 업로드 대상: {route_path.name}")
    latlonalt = parse_wp_latlonalt_list(route_path)
    if not latlonalt:
        print("[run] 미션 포인트가 없습니다.", file=sys.stderr); sys.exit(1)

    m = mav_connect()
    upload_mission_from_points(m, latlonalt)
    arm_and_takeoff(m, REL_ALT)
    set_mode(m, "AUTO")  # 미션 시작

    # 텔레메트리 로거 시작(동일 연결)
    stop_evt = threading.Event()
    tlog = threading.Thread(target=telemetry_logger, args=(m, stop_evt), daemon=True)
    tlog.start()
    print("[run] telemetry logger started")

    # (선택) identifier 자동 실행 — MAVLink를 사용하지 않는다면 포트 충돌 없음
    ident_proc = None
    if IDENTIFIER.exists():
        print("[run] identifier.py 실행")
        ident_proc = subprocess.Popen([PY, str(IDENTIFIER)], cwd=str(Path.cwd()))
    else:
        print("[run] identifier.py 없음 → 미사용")

    # 새 prob 감시 → 계산만 수행 → 같은 연결로 업로드
    last_seen = latest_prob_index()
    print(f"[run] prob 감시 시작 인덱스: {last_seen}")

    # 종료 신호 핸들러
    stop = {"flag": False}
    def handle_sig(signum, frame):
        stop["flag"] = True
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, handle_sig)

    try:
        while not stop["flag"]:
            time.sleep(POLL_SEC)
            cur = latest_prob_index()
            if cur > last_seen:
                print(f"[run] 새 확률 파일 감지: prob{cur}.csv → route_update (no-upload) 실행")
                # 계산만 수행 (업로드는 run_mission이 담당)
                ret = subprocess.run([PY, str(ROUTE_UPDATE), "--no-upload"],
                                     cwd=str(Path.cwd()),
                                     capture_output=True, text=True)
                if ret.stdout:
                    print("[route_update.py 출력]\n" + ret.stdout, end="")
                if ret.stderr:
                    print("[route_update.py 에러]\n" + ret.stderr, file=sys.stderr, end="")
                code = ret.returncode
                if code == NO_UPDATE_EXIT_CODE:
                    print("[run] route_update: 업데이트 없음 → run_mission 종료")
                    break
                elif code != 0:
                    print(f"[run] route_update.py 실패 (exit={code})", file=sys.stderr)
                else:
                    # 방금 저장된 최신 route 업로드
                    new_route = latest_route_file()
                    if new_route:
                        try:
                            new_pts = parse_wp_latlonalt_list(new_route)
                            if new_pts:
                                print(f"[run] 새 미션 업로드: {new_route.name} (pts={len(new_pts)})")
                                upload_mission_from_points(m, new_pts)
                                # 안전하게 AUTO 유지
                                set_mode(m, "AUTO")
                            else:
                                print("[run] 새 미션 포인트가 비어 있음", file=sys.stderr)
                        except Exception as e:
                            print(f"[run] 새 미션 업로드 실패: {e}", file=sys.stderr)
                    else:
                        print("[run] 최신 route 파일을 찾지 못함", file=sys.stderr)
                last_seen = cur
    finally:
        # 종료 정리
        if ident_proc and ident_proc.poll() is None:
            print("[run] identifier 종료 요청")
            ident_proc.terminate()
            try:
                ident_proc.wait(3)
            except Exception:
                ident_proc.kill()
        stop_evt.set()
        tlog.join(timeout=2.0)
        print("[run] 종료")

if __name__ == "__main__":
    main()