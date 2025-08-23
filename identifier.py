# identifier.py — 영상 객체탐지 → prob*.csv 생성
import cv2
import numpy as np
import pandas as pd
from pathlib import Path
import time
import json

class MaritimeObjectDetector:
    def __init__(self):
        # 간단한 색상 기반 객체 탐지 (실제로는 YOLO, SSD 등 사용)
        self.human_color_lower = np.array([0, 50, 50])      # 빨강 계열
        self.human_color_upper = np.array([10, 255, 255])
        self.ship_color_lower = np.array([100, 50, 50])     # 파랑 계열
        self.ship_color_upper = np.array([130, 255, 255])
        
    def detect_objects(self, image_path):
        """이미지에서 사람과 선박을 탐지하여 확률 반환"""
        try:
            image = cv2.imread(str(image_path))
            if image is None:
                return []
            
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            height, width = image.shape[:2]
            
            detections = []
            
            # 격자 기반 샘플링 (실제로는 더 정교한 탐지 필요)
            grid_size = 20
            for y in range(0, height, grid_size):
                for x in range(0, width, grid_size):
                    # 샘플링된 픽셀의 HSV 값
                    hsv_pixel = hsv[y, x]
                    
                    # 사람 탐지 (빨강 계열)
                    p_human = self._calculate_probability(hsv_pixel, self.human_color_lower, self.human_color_upper)
                    
                    # 선박 탐지 (파랑 계열)
                    p_ship = self._calculate_probability(hsv_pixel, self.ship_color_lower, self.ship_color_upper)
                    
                    if p_human > 0.1 or p_ship > 0.1:  # 임계값 이상일 때만 저장
                        # 픽셀 좌표를 위경도로 변환 (예시)
                        lat = 37.5665 + (y / height) * 0.01  # 임시 좌표 변환
                        lon = 126.9780 + (x / width) * 0.01
                        
                        detections.append({
                            'lat': lat,
                            'lon': lon,
                            'p_human': p_human,
                            'p_ship': p_ship
                        })
            
            return detections
            
        except Exception as e:
            print(f"[identifier] 이미지 처리 오류: {e}")
            return []
    
    def _calculate_probability(self, hsv_pixel, lower, upper):
        """HSV 색상 범위에 대한 확률 계산"""
        if cv2.inRange(hsv_pixel, lower, upper).any():
            # 색상 일치도에 따른 확률 (0.0 ~ 1.0)
            return np.random.uniform(0.1, 0.9)  # 임시 확률 생성
        return 0.0

def main():
    """메인 실행 함수"""
    LOG_DIR = Path.cwd() / "log"
    LOG_DIR.mkdir(exist_ok=True)
    
    detector = MaritimeObjectDetector()
    
    # 이미지 파일 찾기
    image_files = list(Path.cwd().glob("Frame*.png"))
    
    if not image_files:
        print("[identifier] Frame*.png 파일을 찾을 수 없습니다.")
        print("[identifier] 샘플 데이터를 생성합니다...")
        
        # 샘플 데이터 생성
        sample_data = [
            {'lat': 37.56655, 'lon': 126.97825, 'p_human': 0.35, 'p_ship': 0.45},
            {'lat': 37.56675, 'lon': 126.97875, 'p_human': 0.50, 'p_ship': 0.60},
            {'lat': 37.56685, 'lon': 126.97925, 'p_human': 0.70, 'p_ship': 0.80},
        ]
        
        df = pd.DataFrame(sample_data)
        output_path = LOG_DIR / "prob1.csv"
        df.to_csv(output_path, index=False)
        print(f"[identifier] 샘플 데이터 생성: {output_path}")
        return
    
    # 실제 이미지 처리
    all_detections = []
    for image_path in image_files:
        print(f"[identifier] 처리 중: {image_path.name}")
        detections = detector.detect_objects(image_path)
        all_detections.extend(detections)
    
    if all_detections:
        # 최신 prob 인덱스 찾기
        existing_probs = list(LOG_DIR.glob("prob*.csv"))
        next_index = 1
        if existing_probs:
            indices = [int(f.stem[4:]) for f in existing_probs if f.stem.startswith("prob")]
            if indices:
                next_index = max(indices) + 1
        
        # 결과 저장
        df = pd.DataFrame(all_detections)
        output_path = LOG_DIR / f"prob{next_index}.csv"
        df.to_csv(output_path, index=False)
        print(f"[identifier] 탐지 결과 저장: {output_path} (탐지: {len(all_detections)}개)")
    else:
        print("[identifier] 탐지된 객체가 없습니다.")

if __name__ == "__main__":
    main()
