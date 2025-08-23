# visualize.py (dashboard.py 보강본)
# - 현재 위치 + 탐색 폴리곤 + 최신 경로 + 확률 점 + 비행 궤적 표시
# - log/ 아래 파일 변화를 1초마다 반영

import json, re
from pathlib import Path
import pandas as pd
from dash import Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go

# --- 상단 경로 고정 (visualize.py가 "코드" 폴더에 있을 때) ---
from pathlib import Path
ROOT = Path(__file__).resolve().parent     # 프로젝트 루트
LOG  = ROOT / "log"                               # 기본 로그 디렉토리 사용
# 만약 log_sample을 쓰고 있다면: LOG = ROOT / "log_sample"

print("[BOOT] ROOT =", ROOT)
print("[BOOT] LOG  =", LOG)

def latest(pattern: str):  # ex) r"route(\d+)\.wp"
    mx, pth = -1, None
    for f in LOG.glob("*"):
        m = re.match(pattern, f.name)
        if m:
            n = int(m.group(1))
            if n > mx:
                mx, pth = n, f
    return pth

def load_route():
    f = latest(r"route(\d+)\.wp")
    if not f:
        return [], None
    try:
        with open(f, "r", encoding="utf-8") as g:
            lines = g.readlines()
        if len(lines) < 2:
            return [], None
        # 홈
        parts = lines[1].strip().split("\t")
        if len(parts) < 11:
            return [], None
        home = (float(parts[8]), float(parts[9]))
        # 웨이포인트
        wpts = []
        for line in lines[2:]:
            parts = line.strip().split("\t")
            if len(parts) >= 11:
                wpts.append((float(parts[8]), float(parts[9])))
        return wpts, home
    except Exception:
        return [], None

def load_polygon():
    f = latest(r"polygon(\d+)\.txt")
    if not f:
        return []
    coords = []
    try:
        with open(f, "r", encoding="utf-8") as g:
            for line in g:
                if line.startswith("Polygon exterior"):
                    continue
                line = line.strip()
                if not line:
                    continue
                try:
                    lon, lat = map(float, line.split(","))
                    coords.append((lat, lon))  # (lat, lon)로 저장
                except Exception:
                    continue
        return coords
    except Exception:
        return []

def load_prob(p_h=0.3, p_s=0.4):
    f = latest(r"prob(\d+)\.csv")
    if not f:
        return pd.DataFrame(columns=["lat","lon","p_human","p_ship"])
    try:
        df = pd.read_csv(f)
        if not {"lat","lon","p_human","p_ship"}.issubset(df.columns):
            return pd.DataFrame(columns=["lat","lon","p_human","p_ship"])
        return df  # ← 여기서 조건 제거
    except Exception:
        return pd.DataFrame(columns=["lat","lon","p_human","p_ship"])

def load_state():
    f = LOG / "live_state.json"
    if not f.exists():
        return {}
    try:
        return json.loads(f.read_text(encoding="utf-8"))
    except Exception:
        return {}

def load_track():
    f = LOG / "track.csv"
    if not f.exists():
        return pd.DataFrame(columns=["timestamp", "lat", "lon", "alt"])
    try:
        df = pd.read_csv(f)
        # 최소 컬럼 체크
        if not {"lat", "lon"}.issubset(df.columns):
            return pd.DataFrame(columns=["timestamp", "lat", "lon", "alt"])
        return df
    except Exception:
        return pd.DataFrame(columns=["timestamp", "lat", "lon", "alt"])

app = Dash(__name__)
app.layout = html.Div([
    html.H3("해상 조난사고 수색 UAV",style={
        "fontFamily": "'Noto Sans KR', 'Malgun Gothic', sans-serif",
        "fontWeight": "bold",
        "fontSize": "28px",
        "color": "#333"
    }),

    # 지도
    dcc.Graph(id="map", config={"scrollZoom": True}, style={"height": "70vh", "marginBottom": "12px"}),

    # 확률 선택창 (버튼 위로 이동)
    html.Div([
        html.Label("사람 발견 확률 ≥", style={"fontFamily": "'Noto Sans', 'Malgun Gothic', sans-serif","fontWeight": "bold"}),
        dcc.Slider(id="p_h", min=0, max=1, step=0.05, value=0.3,
                   tooltip={"always_visible": True}),
        html.Label("선박 발견 확률 ≥",  style={"fontFamily": "'Noto Sans', 'Malgun Gothic', sans-serif","fontWeight": "bold"}),
        dcc.Slider(id="p_s", min=0, max=1, step=0.05, value=0.4,
                   tooltip={"always_visible": True}),
        html.Span(id="last_updated", style={"marginLeft":"10px", "color":"#555"})
    ], style={"maxWidth": "600px", "margin": "0 auto"}),

    # 큰 버튼
    html.Div([
        html.Button(
            "Apply & Refresh",
            id="refresh",
            n_clicks=0,
            style={
                "width": "100%",
                "padding": "16px 24px",
                "fontSize": "20px",
                "fontWeight": "600",
                "borderRadius": "12px",
                "backgroundColor": "#2563eb",  # 파랑
                "color": "white",
                "border": "none",
                "cursor": "pointer",
                "boxShadow": "0 4px 12px rgba(0,0,0,0.12)"
            }
        )
    ], style={"maxWidth": "600px", "margin": "12px auto 0"})
])
@app.callback(
    Output("map", "figure"),
    Input("refresh", "n_clicks"),   # 버튼 클릭만 Input
    State("p_h", "value"),          # 슬라이더 값은 State로
    State("p_s", "value")
)

def update(_, p_h, p_s):
    # 데이터 로드
    wpts, home = load_route()
    poly = load_polygon()
    dfp = load_prob(p_h, p_s)
    state = load_state()
    df_track = load_track()

    fig = go.Figure()

    '''    # 비행 궤적 (lines)
    if not df_track.empty and {"lat", "lon"}.issubset(df_track.columns):
        fig.add_trace(go.Scattermapbox(
            lat=df_track["lat"], lon=df_track["lon"],
            mode="lines", name="Track",
                line={"color": "rgba(0,0,255,1)", "width": 3}
        ))
    '''
    # 경로 (markers+lines)
    if wpts:
        fig.add_trace(go.Scattermapbox(
            lat=[p[0] for p in wpts], lon=[p[1] for p in wpts],
            mode="markers+lines", name="Route", marker={"size": 6},
            line={"color": "rgba(0,0,255,0.3)", "width": 3}
        ))

    # 폴리곤 칠
    if len(poly) >= 3:
        plat, plon = zip(*poly+[poly[0]])  # 닫힌 경로
        fig.add_trace(go.Scattermapbox(
            lat=plat,
            lon=plon,
            mode="lines",
            name="Polygon",
            fill="toself",  # 내부 채우기
            fillcolor="rgba(255, 0, 0, 0.2)",  # 빨강, 20% 투명
            line={"width": 0, "color": "red"}
        ))

    # 확률 통과 점 (markers)

    # 사람만 (빨강)
    human_df = dfp[(dfp["p_human"] >= p_h) & (dfp["p_ship"] < p_s)]
    fig.add_trace(go.Scattermapbox(
        lat=human_df["lat"], lon=human_df["lon"], mode="markers",
        name="Human", marker={"size": 8, "color": "red"}
    ))

    # 배만 (파랑)
    ship_df = dfp[(dfp["p_ship"] >= p_s) & (dfp["p_human"] < p_h)]
    fig.add_trace(go.Scattermapbox(
        lat=ship_df["lat"], lon=ship_df["lon"], mode="markers",
        name="Ship", marker={"size": 8, "color": "blue"}
    ))

    # 둘 다 (보라)
    both_df = dfp[(dfp["p_ship"] >= p_s) & (dfp["p_human"] >= p_h)]
    fig.add_trace(go.Scattermapbox(
        lat=both_df["lat"], lon=both_df["lon"], mode="markers",
        name="Both", marker={"size": 8, "color": "purple"}
    ))

    # 현재 위치 (marker)
    center = None
    if ("lat" in state) and ("lon" in state):
        try:
            lat_now = float(state["lat"])
            lon_now = float(state["lon"])
            fig.add_trace(go.Scattermapbox(
                lat=[lat_now],
                lon=[lon_now],
                mode="markers+text",
                name="UAV",
                text=["UAV"],
                textposition="top center",
                marker={"size": 14, "color": "orange"}
            ))
            center = [lat_now, lon_now]
        except Exception as e:
            print("[STATE] invalid lat/lon in live_state.json:", e)
    elif wpts:
        center = [wpts[0][0], wpts[0][1]]
    else:
        center = [37.5665, 126.9780]  # 최후의 보루; 서울시청

    fig.update_layout(
        mapbox={
            "style": "carto-positron",   # ← 밝은 회색톤
            "center": {"lat": center[0], "lon": center[1]},
            "zoom": 15
        },
        margin={"l":0,"r":0,"t":0,"b":0},
        legend={"orientation": "h", "yanchor": "bottom", "y": 0.01}
    )
    return fig

if __name__ == "__main__":
    # 외부에서: python visualize.py 실행 후 브라우저에서 http://localhost:8050 접속
    app.run(debug=False, host="0.0.0.0", port=8050)