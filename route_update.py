# route_update.py — prob → polygon → lawnmower path → route(N+1).wp 저장 (업로드 없음)
import sys, os, re, argparse
from pathlib import Path
import pandas as pd
import numpy as np
from shapely.geometry import MultiPoint, Polygon, LineString
from shapely.ops import polygonize, unary_union
from pyproj import Transformer

LOG_DIR = Path.cwd() / "log"

# ----------------- 유틸 -----------------
def find_latest_index(pattern: str) -> int:
    """pattern 예: r'^route(\d+)\.wp$'"""
    mx = -1
    for fname in os.listdir(LOG_DIR):
        m = re.match(pattern, fname)
        if m:
            mx = max(mx, int(m.group(1)))
    return mx

def find_latest_prob_csv() -> Path | None:
    mx = -1; latest = None
    for fname in os.listdir(LOG_DIR):
        m = re.match(r'^prob(\d+)\.csv$', fname)
        if m:
            n = int(m.group(1))
            if n > mx:
                mx = n; latest = fname
    return (LOG_DIR / latest) if latest else None

def load_home(wp_path: Path):
    with open(wp_path, "r", encoding="utf-8") as f:
        hdr = f.readline().strip()
        if hdr != "QGC WPL 110":
            raise ValueError(".wp version must be QGC WPL 110")
        parts = f.readline().strip().split('\t')
        lat, lon, alt = map(float, parts[8:11])
    return lat, lon, alt

def load_existing_polygon(txt_path: Path) -> Polygon | None:
    if not txt_path.exists():
        return None
    coords = []
    with open(txt_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if (not line) or line.startswith("Polygon exterior"):
                continue
            try:
                lon, lat = map(float, line.split(","))
                coords.append((lon, lat))
            except Exception:
                continue
    if len(coords) >= 3:
        return Polygon(coords)
    return None

def save_polygon_txt(path: Path, poly: Polygon):
    with open(path, "w", encoding="utf-8") as f:
        f.write("Polygon exterior vertices (lon, lat):\n")
        for lon, lat in poly.exterior.coords:
            f.write(f"{lon}, {lat}\n")

def save_wp(out_path: Path, home_lat: float, home_lon: float, rel_alt: float, coords_latlon: list[tuple[float,float]]):
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("QGC WPL 110\n")
        # 홈 행 (frame=0, cmd=16은 관례적으로 두지만 실제 사용은 웨이포인트들)
        f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{home_lat}\t{home_lon}\t{rel_alt}\t1\n")
        # 웨이포인트 (MAV_FRAME_GLOBAL_RELATIVE_ALT = 3, MAV_CMD_NAV_WAYPOINT = 16)
        for i, (lat, lon) in enumerate(coords_latlon, start=1):
            f.write(f"{i}\t0\t3\t16\t0\t0\t0\t0\t{lat}\t{lon}\t{rel_alt}\t1\n")

# ----------------- 기하 -----------------
def concave_hull(points_ll, alpha_ll):
    """아주 단순한 concave 근사(거리 임계로 간선 구성). points_ll: [[lat,lon], ...]"""
    pts = np.asarray(points_ll, dtype=float)
    edges = []
    for i, p in enumerate(pts):
        for q in pts[i+1:]:
            if np.linalg.norm(p - q) < alpha_ll:
                # shapely polygonize는 (x,y)=(lon,lat) 순서를 기대
                edges.append([(p[1], p[0]), (q[1], q[0])])
    if not edges:
        # fallback: convex hull
        mp = MultiPoint([(lon, lat) for lat, lon in pts])
        return mp.convex_hull
    mls = unary_union(edges)
    polys = list(polygonize(mls))
    if not polys:
        mp = MultiPoint([(lon, lat) for lat, lon in pts])
        return mp.convex_hull
    return max(polys, key=lambda p: p.area)

def build_polygon(points_latlon: np.ndarray, concave=False, alpha=0.001) -> Polygon:
    """points_latlon: Nx2 with columns [lat, lon]"""
    if len(points_latlon) < 3:
        return None
    if concave:
        return concave_hull(points_latlon, alpha)
    # convex
    mp = MultiPoint([(lon, lat) for lat, lon in points_latlon])
    return mp.convex_hull

def lawnmower(poly_xy: Polygon, spacing_m: float) -> list[tuple[float,float]]:
    """poly_xy: 로컬 좌표계(미터) 폴리곤, 반환: [(x,y), ...]"""
    minx, miny, maxx, maxy = poly_xy.bounds
    y = miny
    flip = False
    path_xy = []
    while y <= maxy:
        seg = LineString([(minx, y), (maxx, y)]).intersection(poly_xy)
        if not seg.is_empty:
            if hasattr(seg, "geoms"):
                seg = max(seg, key=lambda s: s.length)
            coords = list(seg.coords)
            if flip: coords.reverse()
            path_xy.extend(coords)
        y += spacing_m
        flip = not flip
    return path_xy

# ----------------- 메인 로직 -----------------
def main():
    ap = argparse.ArgumentParser(description="prob → polygon → grid path → route(N+1).wp")
    ap.add_argument("--p-human", type=float, default=0.3, help="p_human 임계값")
    ap.add_argument("--p-ship",  type=float, default=0.4, help="p_ship 임계값")
    ap.add_argument("--spacing", type=float, default=40.0, help="라운드모어 간격(m)")
    ap.add_argument("--margin",  type=float, default=0.0, help="폴리곤 외곽 버퍼(m)")
    ap.add_argument("--concave", action="store_true", help="concave hull 사용")
    ap.add_argument("--alpha",   type=float, default=0.001, help="concave 간선 임계(위경도)")
    ap.add_argument("--min-area", type=float, default=10.0, help="최소 탐색 면적(m^2)")
    ap.add_argument("--overlap-th", type=float, default=0.9, help="겹침 비율 임계 (이전 폴리곤 대비)")
    ap.add_argument("--rel-alt", type=float, default=10.0, help="웨이포인트 상대고도(m)")
    ap.add_argument("--no-upload", action="store_true", default=True,
                    help="계산/파일저장만 하고 업로드는 하지 않음(단일 연결 구조 권장)")
    args = ap.parse_args()

    LOG_DIR.mkdir(exist_ok=True)

    # 최신 인덱스/파일
    idx_route = find_latest_index(r'^route(\d+)\.wp$')
    if idx_route < 0:
        print("[route_update] log/에 route*.wp가 없습니다. (route0.wp 필요)", file=sys.stderr)
        sys.exit(1)

    old_wp   = LOG_DIR / f"route{idx_route}.wp"
    txt_old  = LOG_DIR / f"polygon{idx_route}.txt"
    csv_prob = find_latest_prob_csv()
    if not csv_prob:
        print("[route_update] prob*.csv가 없습니다.", file=sys.stderr)
        sys.exit(1)

    # 출력 경로
    out_wp  = LOG_DIR / f"route{idx_route + 1}.wp"
    out_txt = LOG_DIR / f"polygon{idx_route + 1}.txt"

    # 홈 로드
    home_lat, home_lon, _home_alt = load_home(old_wp)

    # 이전 폴리곤(있으면)
    exist_poly_geo = load_existing_polygon(txt_old)

    # 확률 파일 로드 및 필터
    df = pd.read_csv(csv_prob)
    if not {"lat","lon","p_human","p_ship"}.issubset(df.columns):
        print("[route_update] prob csv에 필요한 컬럼(lat,lon,p_human,p_ship)이 없습니다.", file=sys.stderr)
        sys.exit(2)

    pts = df[(df.p_human >= args.p_human) & (df.p_ship >= args.p_ship)][["lat","lon"]].to_numpy()
    if len(pts) < 3:
        print(f"[route_update] 필터 통과 점이 3개 미만({len(pts)}). 업데이트 없음.")
        sys.exit(99)

    # 폴리곤 생성 (+ margin)
    poly_geo = build_polygon(pts, concave=args.concave, alpha=args.alpha)
    if poly_geo is None or poly_geo.is_empty:
        print("[route_update] 폴리곤 생성 실패. 업데이트 없음.")
        sys.exit(99)
    if args.margin > 0:
        # 대략 1도 ≈ 111km 가정으로 버퍼(m→deg) 근사
        poly_geo = poly_geo.buffer(args.margin / 111_000.0)

    # 겹침 비율 계산 (이전 폴리곤 면적 대비)
    if exist_poly_geo is not None and not exist_poly_geo.is_empty:
        inter_area_deg2 = poly_geo.intersection(exist_poly_geo).area
        base_area_deg2  = exist_poly_geo.area
        overlap_ratio = (inter_area_deg2 / base_area_deg2) if base_area_deg2 > 0 else 0.0
    else:
        overlap_ratio = 0.0

    # 로컬 투영 (Tmerc 중심=home)으로 면적/경로 생성
    proj = Transformer.from_crs("epsg:4326",
                                f"+proj=tmerc +lat_0={home_lat} +lon_0={home_lon}",
                                always_xy=True)
    poly_xy = Polygon([proj.transform(lon, lat) for lon, lat in poly_geo.exterior.coords])
    area_m2 = poly_xy.area

    print(f"[route_update] 영역 면적: {area_m2:.1f} m^2, 겹침: {overlap_ratio:.1%}")

    # 종료 조건
    if args.min_area and area_m2 <= args.min_area:
        print("[route_update] 탐색 구간이 너무 작음 → 업데이트 없음")
        sys.exit(99)
    if overlap_ratio >= args.overlap_th:
        print("[route_update] 이전 구역과 겹침 과다 → 업데이트 없음")
        sys.exit(99)

    # 라운드모어 경로(로컬) → 위경도로 역변환
    grid_xy = lawnmower(poly_xy, spacing_m=args.spacing)
    if not grid_xy:
        print("[route_update] 라운드모어 경로 생성 실패 → 업데이트 없음")
        sys.exit(99)

    inv = Transformer.from_crs(proj.target_crs, "epsg:4326", always_xy=True)
    grid_geo_latlon = [(inv.transform(x, y)[1], inv.transform(x, y)[0]) for (x, y) in grid_xy]  # (lat,lon)

    # 저장
    save_wp(out_wp, home_lat, home_lon, args.rel_alt, grid_geo_latlon)
    save_polygon_txt(out_txt, poly_geo)

    print(f"[route_update] 저장 완료: {out_wp.name} (WPs={len(grid_geo_latlon)}), {out_txt.name}")
    # 업로드는 단일 연결(run_mission)이 담당하므로 여기서는 끝.
    sys.exit(0)

if __name__ == "__main__":
    main()