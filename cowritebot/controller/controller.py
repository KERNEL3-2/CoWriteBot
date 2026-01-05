import sys, os
import argparse
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from cowritebot_interfaces.srv import GetPenPosition
from .text_to_path import TextToPath
import time
import matplotlib.pyplot as plt

# Gerber 지원 (선택적)
try:
    from .gerber_to_path import GerberToPath, DummyGerberToPath, GERBONARA_AVAILABLE
    GERBER_SUPPORT = True
except ImportError:
    GERBER_SUPPORT = False
    GERBONARA_AVAILABLE = False

package_path = get_package_share_directory('cowritebot')
ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
VELOCITY, ACC = 50, 20
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
dsr_node = rclpy.create_node("controller_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, wait, get_current_posx, movel, movesx, posx, task_compliance_ctrl, \
                        set_desired_force, check_force_condition, release_force, release_compliance_ctrl, \
                        DR_TOOL, DR_AXIS_Z, DR_MV_MOD_REL
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()


init_posx = [420, -5, 170, 0, 180, 0]
dy, dx = 4, 4
LIMIT_OF_CH_IN_SENTENCE = 5
NUM_OF_GRID_DOT = 5

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller_node')

        self.open_gripper_client = self.create_client(Trigger, f'/{ROBOT_ID}/gripper/open')
        self.close_gripper_client = self.create_client(Trigger, f'/{ROBOT_ID}/gripper/close')
        self.req = Trigger.Request()
        self._client = self.create_client(GetPenPosition, 'get_pen_position')
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.pen_req = GetPenPosition.Request()
        self.down_position_z = 0
        self.ttp = TextToPath()

        self.init_robot()
    
    def find_pen(self):
        # 비동기적으로 요청 전송 (결과를 기다리는 Future 객체 반환)
        future = self._client.call_async(self.pen_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) # 결과가 올 때까지 대기
        self.get_logger().info("펜 위치 요청.")
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"성공: {response}")
            else:
                self.get_logger().error(f"실패: {response}")
        else:
            # 서비스 호출 자체에 실패한 경우 (서버가 죽었거나 타임아웃 등)
            self.get_logger().error("서비스 호출 실패")
    
    def init_robot(self):
        # 준비 자세
        JReady = [0, 0, 90, 0, 90, -90]
        self.get_logger().info("준비 자세로 이동합니다.")
        movej(JReady, vel=VELOCITY, acc=ACC)        
        self.get_logger().info("글씨 쓸 준비 완료")
    
    def open_gripper(self):
        result = self.open_gripper_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, result)
        return result.result()

    def close_gripper(self):
        result = self.close_gripper_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, result)
        return result.result()
    
    def grisp_pen(self):
        self.get_logger().info("펜 잡기")
        self.open_gripper()
        wait(3)
        self.close_gripper()
        wait(1)

    def movelBeforeWrite(self, posx_array):
        movel(posx(posx_array), vel=50, acc=20)

    ## 순응제어와 힘제어를 시작하고 유지
    def pendown(self):
        task_compliance_ctrl([300, 300, 300, 0, 0, 0])
        fd = [0, 0, -3, 0, 0, 0]
        fctrl_dir = [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_TOOL)
        while not check_force_condition(DR_AXIS_Z, max=3):
            pass
    
        release_force()  # 힘제어 해제
        release_compliance_ctrl() # 모든 작업 완료 후 순응제어 해제
        wait(0.3)  # 안정화

        self.down_position_z = get_current_posx()[0][2] + 0.6

    def penup(self):
        movel(posx(0, 0, 5, 0, 0, 0), mod=DR_MV_MOD_REL, vel=80,acc=30)

    def cvtIndexToPosx(self, char_sequence, num_index):
        global init_posx
        result = init_posx.copy()

        result[0] += (NUM_OF_GRID_DOT * (char_sequence % LIMIT_OF_CH_IN_SENTENCE) + (num_index % NUM_OF_GRID_DOT)) * dx
        result[1] -= (NUM_OF_GRID_DOT * (char_sequence // LIMIT_OF_CH_IN_SENTENCE) + (num_index // NUM_OF_GRID_DOT)) * dy
        return result

    def cvtCharToArray(self, char):
        db = {}
        db['A'] = [[20, 10, 2, 14, 24], [10, 14]]
        db['B'] = [[0, 20], [0, 3, 9, 13, 10], [13, 19, 23, 20]]
        db['C'] = [[4, 1, 5, 15, 21, 24]]
        db['D'] = [[0, 20], [0, 2, 14, 22, 20]]
        db['E'] = [[4, 0, 20, 24], [10, 14]]
        db['F'] = [[4, 0, 20], [10, 14]]
        db['G'] = [[4, 1, 5, 15, 21, 23, 19, 14, 12]]
        db['H'] = [[0, 10, 20], [4, 14, 24], [10, 12, 14]]
        db['I'] = [[1, 3], [2, 12, 22], [21, 23]]
        db['J'] = [[2, 4], [3, 13, 18, 22, 21, 15]]
        db['K'] = [[0, 10, 20], [4, 10, 24]]
        db['L'] = [[0, 20, 24]]
        db['M'] = [[20, 10, 0, 12, 4, 14, 24]]
        db['N'] = [[20, 10, 0, 24, 14, 4]]
        db['O'] = [[2, 1, 10, 21, 23, 14, 3, 2]]
        db['P'] = [[0, 10, 20],[0, 3, 9, 13, 10]]
        db['Q'] = [[2, 1, 10, 21, 23, 14, 3, 2], [12, 24]]
        db['R'] = [[0, 10, 20],[0, 3, 9, 13, 10], [12, 24]]
        db['S'] = [[9, 3, 1, 5, 10, 11, 13, 19, 23, 21, 15]]
        db['T'] = [[0, 2, 4], [2, 12, 22]]
        db['U'] = [[0, 15, 21, 23, 19, 4]]
        db['V'] = [[0, 22, 4]]
        db['W'] = [[0, 21, 12, 23, 4]]
        db['X'] = [[0, 12, 24], [4, 12, 20]]
        db['Y'] = [[0, 12, 4], [12, 22]]
        db['Z'] = [[0, 4, 20, 24]]

        if not char.isalpha():
            raise Exception('{} is not alphabet.'.format(char))

        return db[char.upper()]

    def cvtCharToPosxArray(self, char_sequence, char):
        result = []
        for arr in self.cvtCharToArray(char):
            result.append(list(map(lambda x: self.cvtIndexToPosx(char_sequence, x), arr)))
        return result

    def setZ(self, posx_array):
        result = []
        for pos in posx_array:
            tmp = list(pos)
            tmp[2] = self.down_position_z
            result.append(posx(tmp))
        return result
    
    def typeSentence(self, sentence):
        # 메인 루프
        self.get_logger().info("글씨 쓸 위치로 이동")
        movel(posx(init_posx), vel=VELOCITY, acc=ACC)

        for i, char in enumerate(sentence):
            if char == ' ':
                continue
            
            posx_arrays = self.cvtCharToPosxArray(i, char)
            lp = len(posx_arrays)
            
            self.get_logger().info(f"현재 {char} 쓸 위치로 이동")
            self.movelBeforeWrite(posx_arrays[0][0])
            self.get_logger().info(f"현재 {char} 쓸 위치로 이동완료")
            
            for j, arr in enumerate(posx_arrays):
                self.get_logger().info(f"pendown")
                self.pendown()  # 힘제어 시작 및 유지
                movesx(self.setZ(arr), vel=80, acc=30)
                self.get_logger().info(f"penup")
                self.penup()  # 힘제어만 해제, 순응제어는 유지

                if j + 1 < lp:
                    self.movelBeforeWrite(posx_arrays[j + 1][0])
    
    def strokeToPosxList(self, stroke):
        result = []
        xs = []
        ys = []
        global init_posx
        for (x, y) in stroke:
            tmp = init_posx.copy()
            tmp[0] += x
            tmp[1] += y
            xs.append(tmp[0] + x)
            ys.append(tmp[1] + y)
            result.append(posx(tmp))
        
        return result, xs, ys
    
    def typeSentenceHangul(self, sentence):
        # 메인 루프
        self.get_logger().info("글씨 쓸 위치로 이동")
        movel(posx(init_posx), vel=VELOCITY, acc=ACC)
        
        _strokes = self.ttp.text_to_path(sentence)
        lp = len(_strokes)

        strokes = []
        for stroke in _strokes:
            arr, *_ = self.strokeToPosxList(stroke)
            strokes.append(arr)

        for stroke in strokes:
            self.movelBeforeWrite(stroke[0])
            self.get_logger().info(f"pendown")
            self.pendown()
            movesx(self.setZ(stroke), vel=80, acc=30)
            self.get_logger().info(f"penup")
            self.penup()
    
    def visualize_robot_path(self, sentence):
        strokes = self.ttp.text_to_path(sentence)
        if not strokes:
            print("시각화할 데이터가 없습니다.")
            return

        plt.figure(figsize=(10, 6))

        for stroke in strokes:
            _, xs, ys = self.strokeToPosxList(stroke)
            plt.plot(xs, ys, marker='.', markersize=4, linestyle='-', label=f'Stroke ')

        plt.title("Skeletonized Single Line Path")
        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.axis('equal')
        plt.grid(True)
        plt.show()

    def drawGerberPath(self, gerber_path: str, scale: float = 1.0):
        """
        Gerber 파일에서 경로를 추출하여 그리기

        Args:
            gerber_path: Gerber 파일 경로 (.gbr, .gtl 등)
            scale: 스케일 팩터
        """
        if not GERBER_SUPPORT:
            self.get_logger().error("Gerber 지원이 활성화되지 않았습니다.")
            return

        self.get_logger().info(f"Gerber 파일 로드: {gerber_path}")

        if GERBONARA_AVAILABLE:
            gtp = GerberToPath(scale=scale)
            strokes = gtp.gerber_to_path(gerber_path)
            strokes = gtp.normalize_to_origin(strokes)
        else:
            self.get_logger().warn("gerbonara 미설치 - 샘플 데이터 사용")
            gtp = DummyGerberToPath(scale=scale)
            strokes = gtp.generate_sample_pcb_traces()

        if not strokes:
            self.get_logger().error("경로를 추출할 수 없습니다.")
            return

        self.get_logger().info(f"추출된 경로: {len(strokes)}개")

        # 글씨 쓸 위치로 이동
        self.get_logger().info("작업 위치로 이동")
        movel(posx(init_posx), vel=VELOCITY, acc=ACC)

        # 각 stroke 그리기
        for i, stroke in enumerate(strokes):
            if len(stroke) < 2:
                continue

            # stroke를 로봇 좌표로 변환
            robot_stroke = []
            for (x, y) in stroke:
                tmp = init_posx.copy()
                tmp[0] += x
                tmp[1] += y
                robot_stroke.append(posx(tmp))

            self.get_logger().info(f"Stroke {i+1}/{len(strokes)} 그리기 ({len(stroke)} points)")
            self.movelBeforeWrite(robot_stroke[0])
            self.pendown()
            movesx(self.setZ(robot_stroke), vel=80, acc=30)
            self.penup()

        self.get_logger().info("Gerber 경로 그리기 완료!")

    def drawDrillPoints(self, excellon_path: str, scale: float = 1.0):
        """
        Excellon 드릴 파일에서 포인트 추출하여 점 찍기 (납땜 포인트)

        Args:
            excellon_path: Excellon 파일 경로 (.drl, .xln 등)
            scale: 스케일 팩터
        """
        if not GERBER_SUPPORT:
            self.get_logger().error("Gerber 지원이 활성화되지 않았습니다.")
            return

        self.get_logger().info(f"Excellon 파일 로드: {excellon_path}")

        if GERBONARA_AVAILABLE:
            gtp = GerberToPath(scale=scale)
            points = gtp.excellon_to_points(excellon_path)
        else:
            self.get_logger().warn("gerbonara 미설치 - 샘플 데이터 사용")
            gtp = DummyGerberToPath(scale=scale)
            points = gtp.generate_sample_drill_points()

        if not points:
            self.get_logger().error("포인트를 추출할 수 없습니다.")
            return

        self.get_logger().info(f"추출된 드릴 포인트: {len(points)}개")

        # 작업 위치로 이동
        self.get_logger().info("작업 위치로 이동")
        movel(posx(init_posx), vel=VELOCITY, acc=ACC)

        # 각 포인트에서 점 찍기
        for i, (x, y) in enumerate(points):
            tmp = init_posx.copy()
            tmp[0] += x
            tmp[1] += y

            self.get_logger().info(f"Point {i+1}/{len(points)}: ({x:.1f}, {y:.1f})")
            self.movelBeforeWrite(tmp)
            self.pendown()
            wait(0.3)  # 잠시 대기 (점 찍기)
            self.penup()

        self.get_logger().info("드릴 포인트 작업 완료!")

    def visualize_gerber_path(self, gerber_path: str, scale: float = 1.0):
        """Gerber 경로 시각화"""
        if not GERBER_SUPPORT:
            print("Gerber 지원이 활성화되지 않았습니다.")
            return

        if GERBONARA_AVAILABLE:
            gtp = GerberToPath(scale=scale)
            strokes = gtp.gerber_to_path(gerber_path)
            strokes = gtp.normalize_to_origin(strokes)
        else:
            print("gerbonara 미설치 - 샘플 데이터 사용")
            gtp = DummyGerberToPath(scale=scale)
            strokes = gtp.generate_sample_pcb_traces()

        if hasattr(gtp, 'visualize'):
            gtp.visualize(strokes, title=f"Gerber: {gerber_path}")
        else:
            # DummyGerberToPath용 시각화
            plt.figure(figsize=(10, 8))
            for stroke in strokes:
                x_vals = [p[0] for p in stroke]
                y_vals = [p[1] for p in stroke]
                plt.plot(x_vals, y_vals, 'b-', linewidth=2)
            plt.title(f"Gerber Path (Sample)")
            plt.xlabel("X (mm)")
            plt.ylabel("Y (mm)")
            plt.axis('equal')
            plt.grid(True, alpha=0.3)
            plt.show()

def main(args=None):
    # 명령줄 인자 파싱
    parser = argparse.ArgumentParser(description='CoWriteBot Controller')
    parser.add_argument('--sentence', '-s', type=str, default=None,
                       help='쓸 문장 (한글 또는 영어)')
    parser.add_argument('--gerber', '-g', type=str, default=None,
                       help='Gerber 파일 경로 (.gbr, .gtl 등)')
    parser.add_argument('--drill', '-d', type=str, default=None,
                       help='Excellon 드릴 파일 경로 (.drl, .xln 등)')
    parser.add_argument('--scale', type=float, default=1.0,
                       help='Gerber/드릴 스케일 팩터 (기본: 1.0)')
    parser.add_argument('--skip-grasp', action='store_true',
                       help='펜 잡기 스킵 (이미 잡은 상태)')
    parser.add_argument('--visualize', '-v', action='store_true',
                       help='경로 시각화만 하고 종료')
    parser.add_argument('--sample-pcb', action='store_true',
                       help='샘플 PCB 패턴 그리기 (테스트용)')

    # ROS2 args와 분리
    parsed_args, remaining = parser.parse_known_args()

    node = BaseController()
    try:
        # 시각화 모드
        if parsed_args.visualize:
            if parsed_args.sentence:
                node.visualize_robot_path(parsed_args.sentence)
            elif parsed_args.gerber:
                node.visualize_gerber_path(parsed_args.gerber, parsed_args.scale)
            elif parsed_args.sample_pcb:
                node.visualize_gerber_path(None, parsed_args.scale)  # 샘플 데이터
            return

        # 펜 잡기 (skip-grasp 옵션이 없으면)
        if not parsed_args.skip_grasp:
            node.grisp_pen()

        # Gerber 모드
        if parsed_args.gerber:
            node.get_logger().info(f"Gerber 파일 그리기: {parsed_args.gerber}")
            node.drawGerberPath(parsed_args.gerber, parsed_args.scale)
        # 드릴 모드
        elif parsed_args.drill:
            node.get_logger().info(f"드릴 포인트 그리기: {parsed_args.drill}")
            node.drawDrillPoints(parsed_args.drill, parsed_args.scale)
        # 샘플 PCB 모드
        elif parsed_args.sample_pcb:
            node.get_logger().info("샘플 PCB 패턴 그리기")
            node.drawGerberPath(None, parsed_args.scale)  # gerbonara 없으면 샘플 데이터 사용
        # 문장 모드
        elif parsed_args.sentence:
            node.get_logger().info(f"'{parsed_args.sentence}' 쓰기 시작")
            node.typeSentenceHangul(parsed_args.sentence)
            node.get_logger().info("쓰기 완료!")
        else:
            # 문장이 없으면 인터랙티브 모드
            while rclpy.ok():
                sentence = input('문자를 입력하세요 (종료: q): ')
                if sentence.lower() == 'q':
                    break
                if sentence.strip():
                    node.typeSentenceHangul(sentence)
    except KeyboardInterrupt:
        pass
    finally:
        release_force()
        release_compliance_ctrl()
        node.destroy_node()
        rclpy.shutdown()


def main_launcher(sentence: str, skip_grasp: bool = False):
    """런처에서 호출용 함수

    Args:
        sentence: 쓸 문장
        skip_grasp: True면 펜 잡기 스킵

    Returns:
        bool: 성공 여부
    """
    node = BaseController()
    try:
        if not skip_grasp:
            node.grisp_pen()

        if sentence:
            node.get_logger().info(f"'{sentence}' 쓰기 시작")
            node.typeSentenceHangul(sentence)
            node.get_logger().info("쓰기 완료!")
        return True
    except Exception as e:
        node.get_logger().error(f"오류 발생: {e}")
        return False
    finally:
        release_force()
        release_compliance_ctrl()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()