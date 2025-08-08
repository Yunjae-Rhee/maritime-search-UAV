import argparse
import sys
import pandas as pd
import numpy as np
from shapely.geometry import MultiPoint, Polygon, LineString
from shapely.ops import polygonize, unary_union
from pyproj import Transformer
import os, re

def find_latest_and_next_route_wp(directory: str):

    max_n = -1
    latest_file = None
    # Iterate through directory
    for fname in os.listdir(directory):
        m = re.match(r'^route(\d+)\.wp$', fname)
        if m:
            n = int(m.group(1))
            if n > max_n:
                max_n = n
                latest_file = fname
    # Determine next filename
    next_index = max_n + 1 if max_n >= 0 else 0
    next_file = f'route{next_index}.wp'
    return latest_file, next_file


def find_latest_prob_csv(directory: str):

    max_n = -1
    latest_file = None
    # Iterate through directory
    for fname in os.listdir(directory):
        m = re.match(r'^prob(\d+)\.csv$', fname)
        if m:
            n = int(m.group(1))
            if n > max_n:
                max_n = n
                latest_file = fname
    return latest_file

def load_home(wp_path):
    with open(wp_path) as f:
        hdr = f.readline().strip()
        if hdr != "QGC WPL 110":
            raise ValueError(".wp version must be QGC WPL 110")
        lat, lon, alt = map(float, f.readline().split('\t')[8:11])
    return lat, lon, alt

def load_coords(wp_path):
    coords = []
    with open(wp_path, "r") as f:
        f.readline()  # header
        for line in f:
            parts = line.strip().split("\t")
            if len(parts) >= 11:
                lat, lon = float(parts[8]), float(parts[9])
                coords.append((lat, lon))
    return coords[1:]

def save_wp(out, home, alt, coords):
    with open(out, "w") as f:
        f.write("QGC WPL 110\n")
        f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{home[0]}\t{home[1]}\t{alt}\n")
        for i, (lat, lon) in enumerate(coords, 1):
            f.write(f"{i}\t0\t3\t16\t0\t0\t0\t0\t{lat}\t{lon}\t{alt}\n")
    print(f"Saved '{out}' with {len(coords)} waypoints")

def concave_hull(points, alpha):
    edges = []
    for i, p in enumerate(points):
        for j, q in enumerate(points[i+1:], start=i+1):
            if np.linalg.norm(p - q) < alpha:
                edges.append([(p[1], p[0]), (q[1], q[0])])
    mls = unary_union(edges)
    polys = polygonize(mls)
    return max(polys, key=lambda p: p.area)

def build_polygon(pts, concave=False, alpha=0.001):
    mp = MultiPoint([(lon, lat) for lat, lon in pts])
    return concave_hull(pts, alpha) if concave else mp.convex_hull

def lawnmower(poly_xy, spacing):
    minx, miny, maxx, maxy = poly_xy.bounds
    y, flip, path = miny, False, []
    while y <= maxy:
        seg = LineString([(minx, y), (maxx, y)]).intersection(poly_xy)
        if not seg.is_empty:
            if hasattr(seg, 'geoms'):
                seg = max(seg, key=lambda s: s.length)
            coords = list(seg.coords)
            if flip:
                coords.reverse()
            path.extend(coords)
        y += spacing
        flip = not flip
    return path

# Example usage in notebook
old_wp, output_wp = find_latest_and_next_route_wp(os.getcwd())
csv_file = find_latest_prob_csv(os.getcwd())
p1, p2 = 0.3, 0.4
spacing = 40 #Z-자(라운드모어) 그리드 경로의 격자 간격을 미터 단위로 지정합니다.
margin = 0 #필터된 점들을 둘러싼 다각형을 buffer(margin) 으로 확장할 때 사용할 버퍼 폭(미터)입니다.
concave_flag = False #Concave Hull 알고리즘에서 점 간선을 만들기 위한 거리 임계치(단위: 위·경도)입니다.
alpha = 0.001
min_area = 10 #다각형(폴리곤) 면적이 이 값 이하로 작아지면 “완료”로 간주하고 그리드 생성을 중단하는 기준 면적(㎡)입니다.
OVERLAP_THRESHOLD = 0.9



# Load home
home_lat, home_lon, home_alt = load_home(old_wp)
existing_coords = load_coords(old_wp)

exist_poly = None
if len(existing_coords) >= 3:
    exist_poly = build_polygon(existing_coords, concave_flag, alpha)


# Load and filter points
df = pd.read_csv(csv_file)
pts = df[(df.p_human >= p1) & (df.p_ship >= p2)][["lat", "lon"]].to_numpy()

# Build polygon
poly_geo = build_polygon(pts, concave_flag, alpha)
if margin:
    poly_geo = poly_geo.buffer(margin / 111000)

if exist_poly is not None:
    inter_area   = poly_geo.intersection(exist_poly).area
    base_area    = poly_geo.area
    overlap_ratio = inter_area / base_area if base_area > 0 else 0.0
#    print(f"Polygon overlap: {overlap_ratio:.1%}")
else:
    overlap_ratio = 0.0


# Project to ENU
proj = Transformer.from_crs("epsg:4326", f"+proj=tmerc +lat_0={home_lat} +lon_0={home_lon}", always_xy=True)
poly_xy = Polygon([proj.transform(lon, lat) for lon, lat in poly_geo.exterior.coords])
area_m2 = poly_xy.area
print(f"Area: {area_m2:.1f} m²")

if min_area and area_m2 <= min_area:
    print("탐색 구간이 작아 작업을 종료합니다.")
elif overlap_ratio >= OVERLAP_THRESHOLD:
    print(f"탐색 구간이 이전 경로와 {overlap_ratio:.1%} 겹칩니다. 작업을 종료합니다.")
else:
    # 그리드 생성 이후 비교
    grid_xy = lawnmower(poly_xy, spacing)
    inv = Transformer.from_crs(proj.target_crs, "epsg:4326", always_xy=True)
    grid_geo = [inv.transform(x, y)[::-1] for x, y in grid_xy]
    save_wp(output_wp, (home_lat, home_lon), home_alt, grid_geo)