# dashboard.py
import json, re
from pathlib import Path
import pandas as pd
from dash import Dash, dcc, html, Input, Output
import plotly.graph_objects as go

LOG = Path("log")

def latest(pattern):  # ex) r"route(\d+)\.wp"
    mx, pth = -1, None
    for f in LOG.glob("*"):
        m = re.match(pattern, f.name)
        if m:
            n = int(m.group(1)); 
            if n > mx: mx, pth = n, f
    return pth

def load_route():
    f = latest(r"route(\d+)\.wp")
    if not f: return [], None
    with open(f, "r", encoding="utf-8") as g:
        lines = g.readlines()
    # 홈
    parts = lines[1].strip().split("\t")
    home = (float(parts[8]), float(parts[9]))
    # 웨이포인트
    wpts = []
    for line in lines[2:]:
        parts = line.strip().split("\t")
        if len(parts) >= 12:
            wpts.append((float(parts[8]), float(parts[9])))
    return wpts, home

def load_polygon():
    f = latest(r"polygon(\d+)\.txt")
    if not f: return []
    coords=[]
    with open(f,"r",encoding="utf-8") as g:
        for line in g:
            if line.startswith("Polygon exterior"): continue
            line=line.strip()
            if not line: continue
            lon,lat = map(float, line.split(","))
            coords.append((lat,lon))
    return coords

def load_prob(p_h=0.3, p_s=0.4):
    f = latest(r"prob(\d+)\.csv")
    if not f: return pd.DataFrame(columns=["lat","lon","p_human","p_ship"])
    df = pd.read_csv(f)
    return df[(df.p_human>=p_h) & (df.p_ship>=p_s)][["lat","lon","p_human","p_ship"]]

def load_state():
    f = LOG/"live_state.json"
    if not f.exists(): return {}
    try:
        return json.loads(f.read_text(encoding="utf-8"))
    except:
        return {}

app = Dash(__name__)
app.layout = html.Div([
    html.H3("Drone Live Map"),
    html.Div([
        html.Label("p_human ≥"),
        dcc.Slider(id="p_h", min=0, max=1, step=0.05, value=0.3, tooltip={"always_visible":True}),
        html.Label("p_ship ≥"),
        dcc.Slider(id="p_s", min=0, max=1, step=0.05, value=0.4, tooltip={"always_visible":True}),
    ], style={"maxWidth":"600px"}),
    dcc.Graph(id="map"),
    dcc.Interval(id="tick", interval=1000, n_intervals=0),  # 1초마다 갱신
])

@app.callback(Output("map","figure"), [Input("tick","n_intervals"), Input("p_h","value"), Input("p_s","value")])
def update(_, p_h, p_s):
    # 데이터 로드
    wpts, home = load_route()
    poly = load_polygon()
    dfp = load_prob(p_h, p_s)
    state = load_state()

    fig = go.Figure()

    # 경로
    if wpts:
        fig.add_trace(go.Scattermapbox(
            lat=[p[0] for p in wpts], lon=[p[1] for p in wpts],
            mode="markers+lines", name="Route", marker={"size":6}
        ))
    # 폴리곤(외곽)
    if len(poly) >= 3:
        lat,lon = zip(*poly+[poly[0]])
        fig.add_trace(go.Scattermapbox(lat=lat, lon=lon, mode="lines", name="Polygon"))
    # 확률 통과 점
    if not dfp.empty:
        fig.add_trace(go.Scattermapbox(
            lat=dfp["lat"], lon=dfp["lon"], mode="markers",
            name=f"Prob pts (≥{p_h:.2f}, ≥{p_s:.2f})", marker={"size":8}
        ))
    # 현재 위치(드론)
    if state.get("lat") and state.get("lon"):
        fig.add_trace(go.Scattermapbox(
            lat=[state["lat"]], lon=[state["lon"]],
            mode="markers", name="Drone", marker={"size":14}
        ))
        center = [state["lat"], state["lon"]]
    else:
        # fallback: 홈 또는 (0,0)
        center = [wpts[0][0], wpts[0][1]] if wpts else [37.5665, 126.9780]

    fig.update_layout(
        mapbox_style="open-street-map",
        mapbox_zoom=16,
        mapbox_center={"lat":center[0], "lon":center[1]},
        margin={"l":0,"r":0,"t":0,"b":0}
    )
    return fig

if __name__ == "__main__":
    app.run_server(debug=False, host="0.0.0.0", port=8050)