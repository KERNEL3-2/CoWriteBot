import sys, os
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from cowritebot_interfaces.srv import GetPenPosition
import time

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

        # self.get_logger().info("펜 잡기")
        # self.open_gripper()
        # wait(3)
        # self.close_gripper()

        # wait(1)
        # self.get_logger().info("글씨 쓸 준비 완료")
    
    def open_gripper(self):
        result = self.open_gripper_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, result)
        return result.result()

    def close_gripper(self):
        result = self.close_gripper_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, result)
        return result.result()
    
    def grisp_pen(self):
        pass

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

def main(args=None):
    node = BaseController()
    try:
        # node.grisp_pen()
        # while rclpy.ok():
        #     sentence = input('문자를 입력하세요.')
            # node.typeSentence(sentence)
        while True:
            node.find_pen()
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        release_force()
        release_compliance_ctrl()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()