import pyrealsense2 as rs
import numpy as np
import cv2
import rclpy, time
from cowritebot_interfaces.srv import GetPenPosition
from rclpy.node import Node
from object_detection.realsense import ImgNode
from cv_bridge import CvBridge

class FindMarker(Node):
    def __init__(self):
        super().__init__('find_marker_node')
        self.get_logger().info("initializing...")
        self.img_node = ImgNode()
        self.bridge = CvBridge()
        self.intrinsics = None
        while self.intrinsics is None:
            time.sleep(0.5)
            self.intrinsics = self._wait_for_valid_data(
                self.img_node.get_camera_intrinsic, "camera intrinsics"
            )

        self.create_service(
            GetPenPosition,
            'get_pen_position',
            self.callback_get_pen_position
        )
        self.get_logger().info("find_marker_node initialized.")
    
    def callback_get_pen_position(self, request, response):
        pen_location_info = self.get_pen_position()
        if pen_location_info:
            self.get_logger().info(f'{pen_location_info}')
            response.success = True
            response.center = pen_location_info['center']
            response.cap_end_3d = pen_location_info['cap_end_3d']
            response.tip_end_3d = pen_location_info['tip_end_3d']
            response.angle = pen_location_info['angle']
            response.area = pen_location_info['area']
        else:
            response.success = False

        return response

    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            self.get_logger().info(f"Retry getting {description}.")
            rclpy.spin_once(self.img_node, timeout_sec=0.1)
            data = getter()
        return data
    
    def get_frames(self, duration=1.0):
        """가장 최신의 Color, Depth 프레임 쌍을 가져옵니다."""
        for _ in range(20):
            rclpy.spin_once(self.img_node, timeout_sec=0.05)
            color_msg = self.img_node.get_color_frame()
            depth_msg = self.img_node.get_depth_frame()
            
            if color_msg is not None and depth_msg is not None:
                return color_msg, depth_msg
            
            time.sleep(0.01)
            
        return None, None

    def get_hsv_values(self):
        # 파란색 HSV 범위 (모나미 보드마카)
        h_min = 100
        h_max = 130
        s_min = 100
        s_max = 255
        v_min = 50
        v_max = 255
        min_area = 100
        return (np.array([h_min, s_min, v_min]), 
                np.array([h_max, s_max, v_max]),
                max(min_area, 10))  # 최소 10
    
    def get_3d_point(self, x, y, depth_array):
        """
        x, y: 이미지상의 픽셀 좌표 (pixel)
        depth: 해당 픽셀의 깊이 값 (m 또는 mm)
        fx, fy: 초점 거리 (camera_info의 K[0], K[4])
        ppx, ppy: 주점 (camera_info의 K[2], K[5])

        Joint 좌표계 [0, 0, 90, 0, 90, 0]의 posx: [373.000, 0.000, 405.000, 149.678, 180.000, 59.678]
        """
        
        # 0. 깊이 값이 0이면 (인식 불가) 계산 불가
        depth_mm = depth_array[y, x]
        if depth_mm <= 0:
            return None

        # 1. 픽셀 좌표를 카메라 중심 기준(Optical Center)으로 이동
        # ppx, ppy를 빼서 이미지 중앙을 (0,0)으로 만듭니다.
        x_normalized = (x - self.intrinsics['ppx']) / self.intrinsics['fx']
        y_normalized = (y - self.intrinsics['ppy']) / self.intrinsics['fy']
        
        # 2. 정규 좌표에 깊이(Z)를 곱해 실제 3D 좌표 계산
        X = x_normalized * depth_mm
        Y = y_normalized * depth_mm
        Z = depth_mm
        
        return np.array([X, Y, Z])
    
    def transform_to_robot_base(self, point_cam):
        """
        point_cam: [X, Y, Z] (카메라 기준 3D 좌표)
        H_base2cam: 4x4 변환 행렬 (Calibration 결과물)
        """
        if point_cam is None:
            return None

        # 1. 3D 좌표를 동차 좌표(Homogeneous Coordinates)로 변환 [X, Y, Z, 1]
        point_homog = np.append(point_cam, 1.0)

        # 캘리브레이션으로 행렬
        H_base2cam = np.array([
            [0, -1, 0, 0.3],  # 카메라가 로봇 베이스에서 X방향으로 0.3m 떨어짐
            [1, 0, 0, 0.0], # Y축은 일치한다고 가정
            [0, 0, 1, 1.0],  # 카메라 높이가 책상으로부터 1m 떨어짐
            [0, 0, 0, 1]
        ])
        
        # 2. 행렬 곱셈 수행: P_base = H_base2cam * P_cam
        point_base_homog = H_base2cam @ point_homog
        
        # 3. 다시 [X, Y, Z]로 반환
        return point_base_homog[:3]

    def detect_pen(self, color_image, depth_frame, hsv_lower, hsv_upper, min_area):
        # [시각화] 원본 이미지를 복사하여 그림 그릴 준비
        vis_img = color_image.copy()

        # BGR → HSV 변환
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # [시각화 함수] 화면 출력 및 키 입력 대기 (함수 내부 헬퍼)
        def show_result():
            cv2.imshow("Pen Detection", vis_img)
            cv2.waitKey(1)

        if not contours:
            show_result() # 검출 실패해도 화면 갱신
            return None
        
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < min_area:
            show_result() # 너무 작으면 무시하고 화면 갱신
            return None
        
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        
        # [시각화] 펜의 외곽선(Box) 그리기 (초록색)
        cv2.drawContours(vis_img, [box], 0, (0, 255, 0), 2)

        center, size, angle = rect
        width, height = size
        
        if width > height:
            length = width
            angle_rad = np.radians(angle)
        else:
            length = height
            angle_rad = np.radians(angle + 90)
        
        # 펜의 방향 벡터 (2D)
        direction_2d = np.array([np.cos(angle_rad), np.sin(angle_rad)])
        
        # 펜의 양 끝점 계산
        half_length = length / 2
        end1 = (int(center[0] + direction_2d[0] * half_length),
                int(center[1] + direction_2d[1] * half_length))
        end2 = (int(center[0] - direction_2d[0] * half_length),
                int(center[1] - direction_2d[1] * half_length))
        
        # 뚜껑 쪽 (위쪽, y가 작은 쪽) 결정
        # 손으로 뚜껑을 잡고 있으니 펜촉이 아래로 향함
        if end1[1] < end2[1]:
            cap_end = end1  # 뚜껑 (위)
            tip_end = end2  # 펜촉 (아래)
        else:
            cap_end = end2
            tip_end = end1
        
        # [시각화] 뚜껑(빨강), 펜촉(파랑), 중심선(노랑) 그리기
        cv2.circle(vis_img, cap_end, 5, (0, 0, 255), -1)   # Red: Cap
        cv2.circle(vis_img, tip_end, 5, (255, 0, 0), -1)   # Blue: Tip
        cv2.line(vis_img, cap_end, tip_end, (0, 255, 255), 2) # Yellow Line

        # 3D 좌표 계산
        cap_3d = self.transform_to_robot_base(self.get_3d_point(cap_end[0], cap_end[1], depth_frame))
        tip_3d = self.transform_to_robot_base(self.get_3d_point(tip_end[0], tip_end[1], depth_frame))
        
        if cap_3d is None or tip_3d is None:
            return None
        
        center = list(map(float, center))
        cap_end = list(map(float, cap_end))
        tip_end = list(map(float, tip_end))
        cap_3d = list(map(lambda x: round(x, 2), cap_3d))
        tip_3d = list(map(lambda x: round(x, 2), tip_3d))
        
        result = {
            'center': center,
            'cap_end_3d': cap_3d,
            'tip_end_3d': tip_3d,
            'angle': angle,
            'area': area
        }
        
        show_result()
        return result
    
    def get_pen_position(self):
        for _ in range(5000):
            color_frame, depth_frame = self.get_frames()
            
            if color_frame is None or depth_frame is None:
                time.sleep(0.1)
                continue
        
            hsv_lower, hsv_upper, min_area = self.get_hsv_values()
            
            # 여기서 detect_pen 내부의 cv2.imshow가 호출됩니다.
            result = self.detect_pen(color_frame, depth_frame, hsv_lower, hsv_upper, min_area)
            
            if result:
                # 펜을 찾았어도 화면을 계속 보고 싶다면 여기서 바로 리턴하지 않고
                # 잠시 대기하거나, 혹은 찾자마자 리턴합니다.
                # (현재 로직은 찾으면 바로 리턴하여 서비스 응답을 보냅니다)
                return result
        
        return None


def main(args=None):
    rclpy.init()
    node = FindMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()