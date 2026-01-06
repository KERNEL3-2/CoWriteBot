import numpy as np
import matplotlib.pyplot as plt
import math, cv2
from PIL import Image, ImageDraw, ImageFont
from skimage.morphology import skeletonize
import os

# (선택) pip install rdp
# RDP 알고리즘이 없으면 간단한 자체 구현 함수를 사용합니다.
try:
    from rdp import rdp
except ImportError:
    def rdp(points, epsilon):
        # 간단한 다운샘플링으로 대체 (RDP가 훨씬 품질이 좋습니다)
        return points[::int(max(1, 1/epsilon))]

class ImageToPath:
    def __init__(self):
        self.__image_path = None

    def __image_to_skeleton_paths(self):
        strokes = self.__extract_raw_strokes()
        result = self.__merge_broken_strokes(strokes, connect_dist=15.0)
        return result

    def __extract_raw_strokes(self):
        if self.__image_path is None:
            return []

        img = cv2.imread(self.__image_path, cv2.IMREAD_GRAYSCALE)
        img = cv2.resize(img, (1280, 720))
        # 2. 노이즈 제거 (가우시안 블러 - 선택사항이지만 권장)
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        # 3. Canny Edge Detection
        # 100, 200은 임계값(Threshold)으로, 숫자가 낮을수록 더 많은 선을 찾습니다.
        edges = cv2.Canny(blurred, 100, 200)
        binary_array = np.array(edges) >= 128
        
        skeleton = skeletonize(binary_array)
        y_indices, x_indices = np.where(skeleton)
        points = set(zip(x_indices, y_indices))
        
        if not points: return []

        raw_strokes = []
        
        while points:
            start_point = next(iter(points))
            current_stroke = [start_point]
            points.remove(start_point)
            
            while True:
                last_point = current_stroke[-1]
                # 8방향 탐색
                candidates = []
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0: continue
                        neighbor = (last_point[0] + dx, last_point[1] + dy)
                        if neighbor in points:
                            candidates.append(neighbor)
                
                if candidates:
                    # 후보 중 가장 가까운 점 선택
                    next_point = min(candidates, key=lambda p: math.dist(last_point, p))
                    current_stroke.append(next_point)
                    points.remove(next_point)
                else:
                    # 더 이상 연결된 점이 없으면 종료
                    break
            
            # 너무 짧은 노이즈 제거 (기준 완화)
            if len(current_stroke) > 5:
                raw_strokes.append(current_stroke)
                
        return raw_strokes

    def __merge_broken_strokes(self, strokes, connect_dist=10.0):
        """
        [핵심 기능] 가까운 끝점끼리 연결하여 끊어진 획을 하나로 합침
        """
        if not strokes: return []
        
        # 획들을 계속 순회하며 합칠 수 있는지 확인
        merged = True
        while merged:
            merged = False
            used_indices = set()
            
            for i in range(len(strokes)):
                if i in used_indices: continue
                
                base_stroke = list(strokes[i])
                best_match_idx = -1
                best_match_type = None # 'tail_to_head', 'tail_to_tail', etc.
                min_d = connect_dist
                
                tail = base_stroke[-1]
                head = base_stroke[0]

                for j in range(len(strokes)):
                    if i == j or j in used_indices: continue
                    
                    target_stroke = strokes[j]
                    t_head = target_stroke[0]
                    t_tail = target_stroke[-1]
                    
                    # 4가지 경우의 수 거리 계산 (시작-시작, 시작-끝, 끝-시작, 끝-끝)
                    d_tail_head = math.dist(tail, t_head)
                    d_tail_tail = math.dist(tail, t_tail)
                    d_head_tail = math.dist(head, t_tail)
                    d_head_head = math.dist(head, t_head)

                    # 가장 가까운 연결 찾기
                    if d_tail_head < min_d:
                        min_d = d_tail_head
                        best_match_idx = j
                        best_match_type = 'tail_to_head'
                    elif d_tail_tail < min_d:
                        min_d = d_tail_tail
                        best_match_idx = j
                        best_match_type = 'tail_to_tail'
                    elif d_head_tail < min_d:
                        min_d = d_head_tail
                        best_match_idx = j
                        best_match_type = 'head_to_tail'
                    elif d_head_head < min_d:
                        min_d = d_head_head
                        best_match_idx = j
                        best_match_type = 'head_to_head'
                
                if best_match_idx != -1:
                    target = strokes[best_match_idx]
                    if best_match_type == 'tail_to_head':
                        base_stroke.extend(target)
                    elif best_match_type == 'tail_to_tail':
                        base_stroke.extend(target[::-1]) # 뒤집어서 붙임
                    elif best_match_type == 'head_to_tail':
                        base_stroke = target + base_stroke
                    elif best_match_type == 'head_to_head':
                        base_stroke = target[::-1] + base_stroke
                    
                    strokes[i] = base_stroke # 합쳐진 것으로 업데이트
                    used_indices.add(best_match_idx) # 합쳐진 대상은 다음에 건너뜀
                    merged = True # 변화 발생, 반복 다시 수행
            
            # 병합되지 않은 획들 정리
            final_list = []
            for i in range(len(strokes)):
                if i not in used_indices:
                    final_list.append(strokes[i])
            strokes = final_list

        return strokes

    def convert_pixels_to_robot_mm(self, strokes, scale=0.1):
        """
        좌표 변환 및 RDP 알고리즘을 통한 포인트 최적화
        """
        robot_paths = []
        for stroke in strokes:
            # 1. 픽셀 -> 로봇 좌표 변환
            # Y축 반전 및 스케일링
            converted_stroke = []
            for x, y in stroke:
                rx = x * scale
                ry = -y * scale
                converted_stroke.append([rx, ry]) # RDP는 리스트 형태 선호
            
            # 2. RDP 알고리즘 적용 (단순 다운샘플링보다 곡선과 모서리를 잘 살림)
            # epsilon값이 클수록 점이 더 많이 줄어들고 직선화됨 (0.5 ~ 2.0 권장)
            # numpy array로 변환하여 rdp에 전달
            if len(converted_stroke) > 2:
                new_stroke = [(round(p[0], 2), round(p[1], 2)) for p in converted_stroke[::5]]
                # try:
                #     simplified = rdp(np.array(converted_stroke), epsilon=0.5)
                #     # 다시 튜플 리스트로 변환
                #     new_stroke = [(round(p[0], 2), round(p[1], 2)) for p in simplified]
                # except:
                #      # rdp 라이브러리가 없거나 에러시 기존 방식(step=3 정도로 완화)
                #     new_stroke = [(round(p[0], 2), round(p[1], 2)) for p in converted_stroke[::3]]
            else:
                new_stroke = [(round(p[0], 2), round(p[1], 2)) for p in converted_stroke]

            if len(new_stroke) > 1:
                robot_paths.append(new_stroke)
                
        return robot_paths

    # (나머지 get_min_xy, visualize_robot_path 등은 기존 유지)
    def get_min_xy(self, strokes):
        if not strokes: return (0,0)
        x_min = float("inf")
        y_min = float("inf")
        for stroke in strokes:
            x_vals, y_vals = zip(*stroke)
            x_min = min(x_min, *x_vals)
            y_min = min(y_min, *y_vals)
        return (x_min, y_min)
    
    def image_to_path(self, image_path) -> list:
        self.__image_path = image_path
        pixel_strokes = self.__image_to_skeleton_paths()
        strokes = self.convert_pixels_to_robot_mm(pixel_strokes)
        (offset_x, offset_y) = self.get_min_xy(strokes)
        result = []
        for stroke in strokes:
            result.append(list(map(lambda x: (round(x[0] - offset_x, 2), round(x[1] - offset_y, 2)), stroke)))
        
        return result
    
    def visualize_robot_path(self, strokes):
        if not strokes:
            print("시각화할 데이터가 없습니다.")
            return

        plt.figure(figsize=(10, 6))
        # 색상맵 생성
        colors = plt.cm.rainbow(np.linspace(0, 1, len(strokes)))
        
        for i, (stroke, color) in enumerate(zip(strokes, colors)):
            x_vals, y_vals = zip(*stroke)
            # 시작점과 끝점 표시로 연결 상태 확인
            plt.plot(x_vals, y_vals, marker='.', markersize=2, linestyle='-', color=color, label=f'Stroke {i+1}')
            # plt.plot(x_vals[0], y_vals[0], 'o', color=color, markersize=5) # 시작점
        
        plt.title(f"Optimized Path (Strokes: {len(strokes)})")
        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.axis('equal')
        plt.grid(True)
        plt.show()

# 실행부
if __name__ == '__main__':
    # 폰트 경로는 본인 환경에 맞게 수정
    # itp = ImageToPath()
    
    image_path = 'cowritebot/resource/cat.png'
    # print(f"'{image_path}' 경로 생성 중...")
    # print(os.getcwd())
    
    # # 획 추출 및 병합
    # final_paths = itp.image_to_path(image_path)

    # # 시각화
    # itp.visualize_robot_path(final_paths)

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    _, binary = cv2.threshold(img, 0, 127, cv2.THRESH_BINARY)

    # 뼈대 추출을 위한 빈 이미지 생성
    skel = np.zeros(binary.shape, np.uint8)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

    while True:
        # 침식 및 팽창 연산 반복
        eroded = cv2.erode(binary, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(binary, temp)
        skel = cv2.bitwise_or(skel, temp)
        binary = eroded.copy()

        # 모든 픽셀이 처리되면 종료
        if cv2.countNonZero(binary) == 0:
            break

    cv2.imshow('Skeleton', skel)
    cv2.waitKey(0)