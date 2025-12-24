import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont
from skimage.morphology import skeletonize

class TextToPath:
    def __init__(self, text_size = 1000, font_path = "/usr/share/fonts/truetype/nanum/NanumBrush.ttf"):
        self.text_size = text_size
        self.font_path = font_path
        self.font = None
        try:
            self.font = ImageFont.truetype(font_path, text_size)
        except:
            print(f"폰트를 찾을 수 없습니다: {font_path}")
        
    def text_to_skeleton_paths(self, text):
        """
        텍스트를 이미지로 렌더링 후 뼈대(Skeleton)를 추출하여 정렬된 경로(Path)로 반환
        """
        if not self.font:
            raise Exception(f"폰트를 찾을 수 없습니다: {self.font_path}")
        # 1. 텍스트를 고해상도 이미지(비트맵)로 그리기
        # 여백을 충분히 줍니다.
        image_width = self.text_size * len(text) + 20
        image_height = self.text_size + 20
        
        img = Image.new('L', (image_width, image_height), 0) # 검은 배경
        draw = ImageDraw.Draw(img)

        # 텍스트 그리기 (흰색)
        draw.text((10, 10), text, font=self.font, fill=255)
        
        # Numpy 배열로 변환 (0 or 1)
        data = np.array(img) > 128
        
        # 2. 뼈대 추출 (Skeletonize) - 핵심 단계
        # 글자의 두께를 1픽셀로 만듭니다.
        skeleton = skeletonize(data)
        
        # 3. 픽셀 좌표 추출 (y, x) -> (x, y)
        # y좌표는 이미지 기준이므로 위에서 아래로 증가합니다. 나중에 반전 필요.
        y_indices, x_indices = np.where(skeleton)
        points = list(zip(x_indices, y_indices))
        
        if not points:
            return []

        # 4. 점들을 선(Stroke)으로 연결 (Nearest Neighbor Algorithm)
        # 흩뿌려진 점들을 가까운 순서대로 이어야 로봇이 움직일 수 있습니다.
        sorted_strokes = []
        
        # 방문하지 않은 점들을 집합으로 관리
        remaining_points = set(points)
        
        while remaining_points:
            # 새로운 획 시작 (임의의 점 선택 - 보통 가장 왼쪽 위나 남은 점 중 하나)
            # 여기서는 남은 점 중 하나를 꺼냄
            start_point = next(iter(remaining_points))
            current_stroke = [start_point]
            remaining_points.remove(start_point)
            
            while True:
                last_point = current_stroke[-1]
                
                # 현재 점 주변(반경 2픽셀 내)에서 연결된 점 찾기
                # 단순화를 위해 가장 가까운 점을 찾습니다.
                candidates = []
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0: continue
                        neighbor = (last_point[0] + dx, last_point[1] + dy)
                        if neighbor in remaining_points:
                            candidates.append(neighbor)
                
                if candidates:
                    # 후보 중 하나 선택 (여기선 그냥 첫번째)
                    next_point = candidates[0]
                    current_stroke.append(next_point)
                    remaining_points.remove(next_point)
                else:
                    # 더 이상 연결된 점이 없으면 획 종료
                    break
                    
            # 노이즈 제거 (너무 짧은 획은 버림)
            if len(current_stroke) > 10:
                sorted_strokes.append(current_stroke)

        return sorted_strokes

    def convert_pixels_to_robot_mm(self, strokes, scale=0.07, downsample_step = 10):
        """
        픽셀 좌표를 로봇 mm 좌표로 변환 및 Y축 반전
        """
        robot_paths = []
        for stroke in strokes:
            new_stroke = []
            for x, y in stroke[::downsample_step]:
                # 이미지 좌표계(Y down) -> 로봇 좌표계(Y up) 변환
                rx = x * scale
                ry = -y * scale 
                new_stroke.append((round(rx, 2), round(ry, 2)))
            if len(new_stroke) > 1:
                robot_paths.append(new_stroke)
        return robot_paths

    def get_min_xy(self, strokes):
        x_min = float("inf")
        y_min = float("inf")
        for stroke in strokes:
            x_vals, y_vals = zip(*stroke)
            x_min = min(x_min, *x_vals)
            y_min = min(y_min, *y_vals)
        return (x_min, y_min)
    
    def text_to_path(self, text) -> list:
        pixel_strokes = self.text_to_skeleton_paths(text)
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
        for i, stroke in enumerate(strokes):
            x_vals, y_vals = zip(*stroke)
            plt.plot(x_vals, y_vals, marker='.', markersize=4, linestyle='-', label=f'Stroke {i+1}')
        
        plt.title("Skeletonized Single Line Path")
        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.axis('equal')
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    ttp = TextToPath()
    text = "가나다"
    print(f"'{text}' 뼈대 추출 중...")
    
    # 1. 뼈대 픽셀 추출
    pixel_strokes = ttp.text_to_skeleton_paths(text)
    
    # 2. 로봇 좌표로 변환 (스케일 조절)
    robot_strokes = ttp.convert_pixels_to_robot_mm(pixel_strokes)
    # 3. 결과 확인
    ttp.visualize_robot_path(robot_strokes)