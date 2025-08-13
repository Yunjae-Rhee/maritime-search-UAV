# telemetry_stream.py
import json, time
from pathlib import Path
from pymavlink import mavutil

CONN_STR="COM7"   # 환경에 맞게
BAUD=115200       # 텔레: 보통 57600
LOG = Path("log"); LOG.mkdir(exist_ok=True)
state_path = LOG/"live_state.json"
track_path = LOG/"track.csv"
if not track_path.exists():
    track_path.write_text("t,lat,lon,alt\n", encoding="utf-8")

m = mavutil.mavlink_connection(CONN_STR, baud=BAUD)
m.wait_heartbeat()
print("[telemetry] connected")

while True:
    msg = m.recv_match(type=["GLOBAL_POSITION_INT","GPS_RAW_INT"], blocking=True, timeout=1)
    t = time.time()
    if not msg: 
        continue
    try:
        if msg.get_type()=="GLOBAL_POSITION_INT":
            lat = msg.lat/1e7; lon = msg.lon/1e7; alt = msg.relative_alt/1000.0
        else:  # GPS_RAW_INT
            lat = msg.lat/1e7; lon = msg.lon/1e7; alt = None
        state_path.write_text(json.dumps({"t":t,"lat":lat,"lon":lon,"alt":alt}), encoding="utf-8")
        with open(track_path,"a",encoding="utf-8") as f:
            f.write(f"{t},{lat},{lon},{'' if alt is None else alt}\n")
    except Exception:
        pass
    time.sleep(0.2)