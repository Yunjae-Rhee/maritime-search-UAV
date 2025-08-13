# run_mission.py — 실행과 동시에 비행 시작 + prob 감시/업데이트
import os, sys, time, subprocess, signal, re
from pathlib import Path
from pymavlink import mavutil

LOG_DIR = Path.cwd() / "log"
PY = sys.executable
IDENTIFIER = Path.cwd() / "identifier.py"
ROUTE_UPDATE = Path.cwd() / "route_update.py"

POLL_SEC = 2.0
NO_UPDATE_EXIT_CODE = 99   # route_update.py에서 '업데이트 없음' 신호
# ===== 환경 맞게 수정 =====
CONN_STR = "COM7"          # 예) 'COM7' (Windows USB), '/dev/ttyACM0' (Linux), 'udp:127.0.0.1:14550'
BAUD     = 115200          # 텔레메트리면 보통 57600
REL_ALT  = 10.0            # 이륙 목표 상대고도(m)

# ---------- 유틸 ----------
def latest_prob_index():
    max_n = -1
    for p in LOG_DIR.glob("prob*.csv"):
        m = re.match(r"prob(\d+)\.csv$", p.name)
        if m:
            max_n = max(max_n, int(m.group(1)))
    return max_n

def latest_route_file():
    max_n, latest = -1, None
    for p in LOG_DIR.glob("route*.wp"):
        m = re.match(r"route(\d+)\.wp$", p.name)
        if m:
            n = int(m.group(1))
            if n > max_n:
                max_n, latest = n, p
    return latest

def parse_wp_latlonalt_list(wp_path: Path):
    pts = []
    with open(wp_path, "r", encoding="utf-8") as f:
        header = f.readline().strip()
        if header != "QGC WPL 110":
            raise ValueError("Unsupported .wp format")
        for line in f:
            parts = line.strip().split("\t")
            if len(parts) < 12:  # idx,cur,frame,cmd,p1..p4,x,y,z,autoc
                continue
            lat = float(parts[8]); lon = float(parts[9]); alt = float(parts[10])
            pts.append((lat, lon, alt))
    # 첫 줄은 홈 → 이후가 실제 웨이포인트
    return pts[1:]

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
    print(f"[run] Mission upload ACK: {getattr(ack,'type','None')}")

def set_mode(m, mode_name):
    try:
        m.set_mode_apm(mode_name)
        print(f"[run] Mode -> {mode_name}")
    except Exception as e:
        print(f"[run] Mode set failed: {e}")

def arm_and_takeoff(m, rel_alt):
    ts, tc = m.target_system, m.target_component
    # ARM
    m.mav.command_long_send(ts, tc, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1,0,0,0,0,0,0)
    print("[run] ARM requested")
    # GUIDED
    set_mode(m, "GUIDED")
    # TAKEOFF
    m.mav.command_long_send(ts, tc, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            0, 0,0,0,0, 0,0, float(rel_alt))
    print(f"[run] Takeoff to {rel_alt} m (relative) requested")

def ensure_basics():
    LOG_DIR.mkdir(exist_ok=True)
    if not ROUTE_UPDATE.exists():
        print("[run] route_update.py가 없습니다.", file=sys.stderr)
        sys.exit(1)
    r = latest_route_file()
    if not r:
        print("[run] log/에 route*.wp가 없습니다. (초기 미션은 MP로 올리거나 route0.wp를 만들어 두세요.)",
              file=sys.stderr)
        sys.exit(1)

# ---------- 메인 ----------
def main():
    ensure_basics()
    print("[run] 시작")

    # 0) 시작과 동시에 최신 route 업로드 + 이륙 + AUTO
    route_path = latest_route_file()
    print(f"[run] 초기 업로드 대상: {route_path.name}")
    latlonalt = parse_wp_latlonalt_list(route_path)
    if not latlonalt:
        print("[run] 미션 포인트가 없습니다.", file=sys.stderr)
        sys.exit(1)

    m = mav_connect()
    upload_mission_from_points(m, latlonalt)
    arm_and_takeoff(m, REL_ALT)
    set_mode(m, "AUTO")  # 미션 시작

    # 1) identifier 실행(있으면)
    ident_proc = None
    if IDENTIFIER.exists():
        print("[run] identifier.py 실행")
        ident_proc = subprocess.Popen([PY, str(IDENTIFIER)], cwd=str(Path.cwd()))
    else:
        print("[run] identifier.py 없음 → 미사용")

    # 2) 새 prob 감시 → route_update 실행
    last_seen = latest_prob_index()
    print(f"[run] 감시 시작 인덱스: {last_seen}")

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
                print(f"[run] 새 확률 파일 감지: prob{cur}.csv → route_update 실행")
                ret = subprocess.run([PY, str(ROUTE_UPDATE)],
                                     cwd=str(Path.cwd()),
                                     capture_output=True, text=True)
                if ret.stdout:
                    print("[route_update.py 출력]\n" + ret.stdout, end="")
                if ret.stderr:
                    print("[route_update.py 에러]\n" + ret.stderr, file=sys.stderr, end="")
                code = ret.returncode
                if code == NO_UPDATE_EXIT_CODE:
                    print("[run] route_update가 업데이트 안 함 → run_mission 종료")
                    break
                elif code != 0:
                    print(f"[run] route_update.py 실패 (exit={code})", file=sys.stderr)
                else:
                    # 안전하게 AUTO 유지 (이미 비행 중이면 그대로 진행)
                    try:
                        set_mode(m, "AUTO")
                    except Exception:
                        pass
                last_seen = cur
    finally:
        if ident_proc and ident_proc.poll() is None:
            ident_proc.terminate()
            try:
                ident_proc.wait(3)
            except Exception:
                ident_proc.kill()
        print("[run] 종료")

if __name__ == "__main__":
    main()