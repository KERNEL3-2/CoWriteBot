import sys
import os
import subprocess
import warnings
import threading

# 경고 메시지 숨기기
warnings.filterwarnings("ignore", category=UserWarning)
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

# Qt 한글 입력 (ibus) 지원
os.environ['QT_IM_MODULE'] = 'ibus'

from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QTextEdit, QPushButton, QLabel, QFileDialog,
                             QStackedWidget, QRadioButton, QMessageBox, QTabWidget,
                             QGroupBox, QLineEdit, QSizePolicy)
from PyQt6.QtGui import QPixmap, QDragEnterEvent, QDropEvent, QPalette, QColor
from PyQt6.QtCore import Qt, pyqtSignal, QThread, QProcess
import rclpy
from text_to_path import TextToPath
from gerber_to_path import GerberToPath
from rclpy.node import Node
from cowritebot_interfaces.srv import UserInput
from visualize_gerber import visualize_gerber

# 모듈 경로 설정
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# 채팅 위젯 임포트
try:
    from chat_widget import ChatWidget
    CHAT_AVAILABLE = True
except ImportError as e:
    CHAT_AVAILABLE = False
    print(f"[Warning] chat_widget 모듈을 찾을 수 없습니다: {e}")

# voice_processing 임포트
try:
    from voice_processing.command_parser import RobotCommand
    VOICE_AVAILABLE = True
except ImportError as e:
    VOICE_AVAILABLE = False
    print(f"[Warning] voice_processing 모듈을 찾을 수 없습니다: {e}")


# --- [이전과 동일한 커스텀 파일 박스 클래스] ---
class FileUploadBox(QLabel):

    file_selected = pyqtSignal(bool) # 클래스 변수 : 모든 객체가 값을 공유

    def __init__(self, parent=None):
        super().__init__(parent)
        self.file_path = None # 멤버 변수 : 객체별로 값이 다 다름
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setText("\n이곳을 클릭하거나\n.gbr 파일을 드래그하여 업로드하세요.\n")
        self.set_default_style()
        self.setMinimumSize(380, 150)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setAcceptDrops(True)
        self.setWordWrap(True)

    def set_default_style(self):
        self.setStyleSheet("border: 2px dashed #aaa; background-color: #f0f0f0; border-radius: 10px; color: #555;")

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            fname, _ = QFileDialog.getOpenFileName(self, '.gbr 파일 선택', '../samples', 'Gerber Files (*.gbr)')
            if fname: self.handle_file(fname)

    def dragEnterEvent(self, event: QDragEnterEvent):
        if event.mimeData().hasUrls():
            event.accept()
            self.setStyleSheet("border: 2px solid #3daee9; background-color: #e0f7fa; border-radius: 10px;")
        else:
            event.ignore()

    def dropEvent(self, event: QDropEvent):
        files = [u.toLocalFile() for u in event.mimeData().urls()]
        if files:
            self.handle_file(files[0])

    def handle_file(self, file_path):
        """파일 종류에 따라 화면 표시를 다르게 처리"""
        self.file_path = file_path
        ext = os.path.splitext(file_path)[1].lower()
        file_name = os.path.basename(file_path)

        # 파일 파일인 경우 미리보기
        if ext in ['.png', '.jpg', '.jpeg', '.bmp', '.gif']:
            pixmap = QPixmap(file_path)
            scaled = pixmap.scaled(self.width()-20, self.height()-20,
                                   Qt.AspectRatioMode.KeepAspectRatio,
                                   Qt.TransformationMode.SmoothTransformation)
            self.setPixmap(scaled)
            self.setStyleSheet("border: 1px solid #ccc; background-color: white; border-radius: 10px;")
        else:
            # 일반 파일인 경우 아이콘 모양과 파일명 표시
            self.setPixmap(QPixmap()) # 기존 파일 제거
            self.setText(f"파일이 선택되었습니다:\n{file_name}")
            self.setStyleSheet("border: 1px solid #3daee9; background-color: #ffffff; border-radius: 10px; color: #333; font-weight: bold;")

        self.file_selected.emit(True)


# --- 로봇 프로세스 워커 ---
class RobotProcessWorker(QThread):
    """로봇 명령 실행 백그라운드 워커"""
    finished = pyqtSignal(bool, str)  # (성공 여부, 메시지)
    process_started = pyqtSignal(object)  # 프로세스 객체 전달
    output = pyqtSignal(str)  # 실시간 출력

    def __init__(self, command: str, args: list = None, parent=None):
        super().__init__(parent)
        self.command = command
        self.args = args or []
        self.process = None

    def run(self):
        try:
            HOME = os.path.expanduser("~")

            if self.command == "bringup_virtual":
                # 가상 로봇 연결 (에뮬레이터)
                cmd = "source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash && ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual"
                self._run_persistent(cmd)
                return

            elif self.command == "bringup_real":
                # 실제 로봇 연결
                # args[0]: host IP
                host = self.args[0] if self.args else "192.168.137.100"
                cmd = f"source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash && ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:={host} rviz:=false"
                self._run_persistent(cmd)
                return

            elif self.command == "controller":
                # controller 실행
                sentence = self.args[0] if self.args else ""
                cmd = f"source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash && cd ~/CoWriteBot && ros2 run cowritebot controller --sentence \"{sentence}\""

            else:
                self.finished.emit(False, f"알 수 없는 명령: {self.command}")
                return

            result = subprocess.run(
                ["bash", "-c", cmd],
                capture_output=True,
                text=True,
                timeout=300
            )

            if result.returncode == 0:
                self.finished.emit(True, "완료")
            else:
                self.finished.emit(False, result.stderr or "실행 실패")

        except subprocess.TimeoutExpired:
            self.finished.emit(False, "시간 초과 (5분)")
        except Exception as e:
            self.finished.emit(False, str(e))

    def _run_persistent(self, cmd):
        """지속 실행 프로세스 (bringup용) - 로그 모니터링"""
        import signal
        import time
        import select

        # 준비 완료 감지 키워드 (gripper_service_node가 마지막에 시작됨)
        ready_keywords = [
            "Gripper Service Ready",
            "gripper_service_node",
        ]
        # 실패 감지 키워드
        fail_keywords = [
            "Connect Failed",
            "Timeout",
            "Error",
        ]

        try:
            self.process = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid
            )
            self.process_started.emit(self.process)

            timeout = 60  # 최대 60초 대기
            start_time = time.time()
            log_buffer = []
            connection_failed = False

            while time.time() - start_time < timeout:
                if self.process.poll() is not None:
                    # 프로세스가 종료됨
                    remaining = self.process.stdout.read() if self.process.stdout else ""
                    self.finished.emit(False, "프로세스 종료됨")
                    return

                # 로그 읽기 (non-blocking)
                try:
                    line = self.process.stdout.readline()
                    if line:
                        log_buffer.append(line.strip())
                        self.output.emit(line.strip())

                        # 준비 완료 감지
                        for keyword in ready_keywords:
                            if keyword in line:
                                self.finished.emit(True, "로봇 시스템 준비 완료!")
                                return

                        # 연결 실패 감지
                        if "Connect Failed" in line or "connect timed out" in line:
                            connection_failed = True

                except:
                    pass

                time.sleep(0.1)

            # 타임아웃
            if self.process.poll() is None:
                if connection_failed:
                    self.finished.emit(False, "로봇 연결 실패 - IP 또는 네트워크 확인")
                else:
                    self.finished.emit(True, "로봇 시스템 시작됨 (타임아웃)")
            else:
                self.finished.emit(False, "초기화 실패")

        except Exception as e:
            self.finished.emit(False, str(e))


# --- [메인 애플리케이션 클래스] ---
class MainUI(QWidget):
    # 조인트 값 업데이트 시그널
    joint_values_updated = pyqtSignal(list)  # positions_deg 리스트
    joint_update_failed = pyqtSignal(str)  # 에러 메시지
    # 컨트롤러 실행 완료 시그널
    controller_finished = pyqtSignal(bool, str)  # (성공 여부, 메시지)
    # 서비스 호출 결과 시그널
    service_call_finished = pyqtSignal(bool, str, str)  # (성공 여부, action_name, 메시지)
    # 기존 로봇 연결 감지 시그널
    existing_robot_detected = pyqtSignal(bool)  # 연결 여부

    def __init__(self):
        super().__init__()
        # 시그널 연결
        self.joint_values_updated.connect(self._update_joint_inputs)
        self.joint_update_failed.connect(self._on_joint_update_failed)
        self.controller_finished.connect(self._on_controller_finished)
        self.service_call_finished.connect(self._on_service_call_finished)
        self.existing_robot_detected.connect(self._on_existing_robot_detected)
        self.node = MainController()
        self.ttp = TextToPath()
        self.gtp = GerberToPath()
        self.current_gerber_path = None  # 현재 로드된 Gerber 파일 경로
        self.robot_worker = None  # 로봇 프로세스 워커
        self.robot_connected = False  # 로봇 연결 상태
        self.pending_robot_connect = False  # 로봇 연결 대기 상태 (IP 입력 대기)
        self.robot_process = None  # 로봇 bringup 프로세스
        self.initUI()
        # UI 시작 시 기존 로봇 연결 확인
        self.check_existing_robot_connection()

    def closeEvent(self, event):
        """UI 종료 시 로봇 프로세스 종료"""
        if self.robot_process and self.robot_process.poll() is None:
            import signal
            try:
                # 프로세스 그룹 전체 종료
                os.killpg(os.getpgid(self.robot_process.pid), signal.SIGTERM)
                self.robot_process.wait(timeout=5)
                print("[INFO] 로봇 프로세스 종료됨")
            except Exception as e:
                print(f"[WARN] 프로세스 종료 중 오류: {e}")
                try:
                    os.killpg(os.getpgid(self.robot_process.pid), signal.SIGKILL)
                except:
                    pass
        event.accept()

    def initUI(self):
        self.setWindowTitle('CoWriteBot')
        self.setMinimumSize(750, 800)
        self.resize(750, 1000)

        main_layout = QVBoxLayout()

        # 탭 위젯 생성
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #ccc;
                border-radius: 5px;
            }
            QTabBar::tab {
                background: #f0f0f0;
                border: 1px solid #ccc;
                padding: 10px 20px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background: white;
                border-bottom-color: white;
            }
        """)

        # --- 탭 1: 직접 입력 (기존 UI) ---
        self.direct_input_tab = QWidget()
        self.setup_direct_input_tab()
        self.tabs.addTab(self.direct_input_tab, "직접 입력")

        # --- 탭 2: 음성/채팅 ---
        if CHAT_AVAILABLE:
            self.chat_widget = ChatWidget()
            self.chat_widget.command_received.connect(self.execute_voice_command)
            self.tabs.addTab(self.chat_widget, "음성/채팅")
        else:
            placeholder = QLabel("음성/채팅 기능을 사용할 수 없습니다.\n필요한 모듈을 설치해주세요.")
            placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.tabs.addTab(placeholder, "음성/채팅")

        # --- 탭 3: 로봇 제어 ---
        self.robot_control_tab = QWidget()
        self.setup_robot_control_tab()
        self.tabs.addTab(self.robot_control_tab, "로봇 제어")

        main_layout.addWidget(self.tabs)
        self.setLayout(main_layout)

    def setup_robot_control_tab(self):
        """로봇 제어 탭 설정"""
        layout = QVBoxLayout()

        # 상태 표시
        self.robot_status_label = QLabel("로봇 상태: 연결 안됨")
        self.robot_status_label.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                background-color: #ffcccc;
                border-radius: 5px;
            }
        """)
        layout.addWidget(self.robot_status_label)

        # 로봇 연결 그룹
        connect_group = QGroupBox("로봇 연결")
        connect_layout = QVBoxLayout()

        # 가상/실제 모드 선택
        mode_layout = QHBoxLayout()
        self.btn_virtual_mode = QRadioButton("가상 (에뮬레이터)")
        self.btn_real_mode = QRadioButton("실제 로봇")
        self.btn_virtual_mode.setChecked(True)
        mode_layout.addWidget(self.btn_virtual_mode)
        mode_layout.addWidget(self.btn_real_mode)
        connect_layout.addLayout(mode_layout)

        # IP 입력 필드 (실제 로봇 모드용)
        ip_layout = QHBoxLayout()
        ip_label = QLabel("로봇 IP:")
        self.robot_ip_input = QLineEdit()
        self.robot_ip_input.setPlaceholderText("예: 192.168.137.100")
        self.robot_ip_input.setStyleSheet("""
            QLineEdit {
                padding: 8px;
                border: 1px solid #ccc;
                border-radius: 5px;
                font-size: 13px;
            }
            QLineEdit:focus {
                border-color: #2196F3;
            }
        """)
        ip_layout.addWidget(ip_label)
        ip_layout.addWidget(self.robot_ip_input)
        connect_layout.addLayout(ip_layout)

        # 모드 변경 시 IP 입력 필드 활성화/비활성화
        self.btn_virtual_mode.toggled.connect(self.on_robot_mode_changed)
        self.btn_real_mode.toggled.connect(self.on_robot_mode_changed)
        self.robot_ip_input.setEnabled(False)  # 초기에는 가상 모드이므로 비활성화

        self.btn_bringup = QPushButton("1. 로봇 시스템 시작")
        self.btn_bringup.setMinimumHeight(50)
        self.btn_bringup.setStyleSheet("""
            QPushButton {
                font-size: 14px;
                background-color: #2196F3;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #1976D2; }
            QPushButton:disabled { background-color: #ccc; }
        """)
        self.btn_bringup.clicked.connect(self.run_bringup)
        connect_layout.addWidget(self.btn_bringup)

        # 로봇 연결 해제 버튼
        self.btn_disconnect = QPushButton("로봇 연결 해제")
        self.btn_disconnect.setMinimumHeight(50)
        self.btn_disconnect.setStyleSheet("""
            QPushButton {
                font-size: 14px;
                background-color: #FF9800;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #F57C00; }
            QPushButton:disabled { background-color: #ccc; }
        """)
        self.btn_disconnect.clicked.connect(self.disconnect_robot)
        self.btn_disconnect.setEnabled(False)
        connect_layout.addWidget(self.btn_disconnect)

        connect_group.setLayout(connect_layout)
        layout.addWidget(connect_group)

        # 그리퍼 제어 그룹
        gripper_group = QGroupBox("그리퍼 제어")
        gripper_layout = QHBoxLayout()

        self.btn_gripper_open = QPushButton("그리퍼 열기")
        self.btn_gripper_open.setMinimumHeight(40)
        self.btn_gripper_open.setStyleSheet("""
            QPushButton {
                font-size: 13px;
                background-color: #9C27B0;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #7B1FA2; }
        """)
        self.btn_gripper_open.clicked.connect(self.gripper_open)
        gripper_layout.addWidget(self.btn_gripper_open)

        self.btn_gripper_close = QPushButton("그리퍼 닫기")
        self.btn_gripper_close.setMinimumHeight(40)
        self.btn_gripper_close.setStyleSheet("""
            QPushButton {
                font-size: 13px;
                background-color: #673AB7;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #512DA8; }
        """)
        self.btn_gripper_close.clicked.connect(self.gripper_close)
        gripper_layout.addWidget(self.btn_gripper_close)

        gripper_group.setLayout(gripper_layout)
        layout.addWidget(gripper_group)

        # 조인트 제어 그룹
        joint_group = QGroupBox("조인트 제어")
        joint_layout = QVBoxLayout()

        # 조인트 입력 필드들
        joint_input_layout = QHBoxLayout()
        self.joint_inputs = []
        for i in range(6):
            joint_field = QLineEdit()
            joint_field.setPlaceholderText(f"J{i+1}")
            joint_field.setText("0.0")
            joint_field.setMaximumWidth(60)
            joint_field.setStyleSheet("padding: 5px; font-size: 12px;")
            self.joint_inputs.append(joint_field)
            joint_input_layout.addWidget(QLabel(f"J{i+1}:"))
            joint_input_layout.addWidget(joint_field)
        joint_layout.addLayout(joint_input_layout)

        # 버튼 레이아웃
        joint_btn_layout = QHBoxLayout()

        # 현재값 읽기 버튼
        self.btn_read_joint = QPushButton("현재값 읽기")
        self.btn_read_joint.setMinimumHeight(35)
        self.btn_read_joint.setStyleSheet("""
            QPushButton {
                font-size: 12px;
                background-color: #607D8B;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #455A64; }
        """)
        self.btn_read_joint.clicked.connect(self.fetch_current_joint_values)
        joint_btn_layout.addWidget(self.btn_read_joint)

        # 이동 버튼
        self.btn_move_joint = QPushButton("조인트 이동")
        self.btn_move_joint.setMinimumHeight(35)
        self.btn_move_joint.setStyleSheet("""
            QPushButton {
                font-size: 12px;
                background-color: #009688;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #00796B; }
        """)
        self.btn_move_joint.clicked.connect(self.move_joint)
        joint_btn_layout.addWidget(self.btn_move_joint)

        joint_layout.addLayout(joint_btn_layout)

        joint_group.setLayout(joint_layout)
        layout.addWidget(joint_group)

        # 긴급 정지 그룹
        emergency_group = QGroupBox("긴급 제어")
        emergency_layout = QVBoxLayout()

        self.btn_emergency_stop = QPushButton("긴급 정지")
        self.btn_emergency_stop.setMinimumHeight(60)
        self.btn_emergency_stop.setStyleSheet("""
            QPushButton {
                font-size: 18px;
                font-weight: bold;
                background-color: #f44336;
                color: white;
                border-radius: 5px;
                border: 3px solid #b71c1c;
            }
            QPushButton:hover { background-color: #d32f2f; }
            QPushButton:pressed { background-color: #b71c1c; }
        """)
        self.btn_emergency_stop.clicked.connect(self.emergency_stop)
        emergency_layout.addWidget(self.btn_emergency_stop)

        emergency_group.setLayout(emergency_layout)
        layout.addWidget(emergency_group)

        # 빠른 실행 그룹
        quick_group = QGroupBox("빠른 실행")
        quick_layout = QVBoxLayout()

        self.btn_full_sequence = QPushButton("전체 시퀀스 실행 (접근 → 잡기 → 쓰기)")
        self.btn_full_sequence.setMinimumHeight(50)
        self.btn_full_sequence.setStyleSheet("""
            QPushButton {
                font-size: 14px;
                background-color: #FF9800;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #F57C00; }
            QPushButton:disabled { background-color: #ccc; }
        """)
        self.btn_full_sequence.clicked.connect(self.run_full_sequence)
        quick_layout.addWidget(self.btn_full_sequence)

        quick_group.setLayout(quick_layout)
        layout.addWidget(quick_group)

        # 로그 출력 (유동적 크기)
        self.robot_log = QTextEdit()
        self.robot_log.setReadOnly(True)
        self.robot_log.setMinimumHeight(100)
        self.robot_log.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #00ff00;
                font-family: monospace;
                font-size: 12px;
            }
        """)
        # 텍스트 추가 시 자동 스크롤
        self.robot_log.textChanged.connect(lambda: self.robot_log.verticalScrollBar().setValue(
            self.robot_log.verticalScrollBar().maximum()))
        layout.addWidget(QLabel("로그:"))
        layout.addWidget(self.robot_log, 1)  # stretch factor 1로 로그창이 늘어남

        self.robot_control_tab.setLayout(layout)

    def on_robot_mode_changed(self):
        """로봇 모드 변경 시 IP 입력 필드 활성화/비활성화"""
        is_real = self.btn_real_mode.isChecked()
        self.robot_ip_input.setEnabled(is_real)
        if is_real:
            self.robot_ip_input.setText("110.120.1.52")
        else:
            self.robot_ip_input.clear()

    def run_bringup(self, ip_address: str = None):
        """로봇 시스템 시작 (bringup)"""
        self.set_robot_buttons_enabled(False)

        if self.btn_virtual_mode.isChecked():
            mode = "virtual"
            self.robot_log.append("[INFO] 가상 로봇 (에뮬레이터) 시작...")
            self.robot_worker = RobotProcessWorker("bringup_virtual")
        else:
            mode = "real"
            # IP 주소 결정: 파라미터 > 입력 필드 > 기본값
            host_ip = ip_address or self.robot_ip_input.text().strip() or "192.168.137.100"
            self.robot_log.append(f"[INFO] 실제 로봇 연결 시작... (IP: {host_ip})")
            self.robot_worker = RobotProcessWorker("bringup_real", [host_ip])

        self.update_robot_status(f"{mode} 모드 시작 중...", "#fff3cd")
        self.robot_worker.process_started.connect(self.on_robot_process_started)
        self.robot_worker.output.connect(self.on_robot_log)
        self.robot_worker.finished.connect(self.on_bringup_finished)
        self.robot_worker.start()

    def on_robot_log(self, log_line: str):
        """로봇 프로세스 로그 출력"""
        # 중요 로그만 UI에 표시
        if any(kw in log_line for kw in ["INFO", "WARN", "ERROR", "Started", "Ready", "Failed", "Timeout"]):
            short_log = log_line[-100:] if len(log_line) > 100 else log_line
            self.robot_log.append(f"[ROS] {short_log}")

    def on_robot_process_started(self, process):
        """로봇 프로세스가 시작되면 저장"""
        self.robot_process = process
        self.robot_log.append(f"[INFO] 프로세스 시작됨 (PID: {process.pid})")
        self.btn_disconnect.setEnabled(True)

    def gripper_open(self):
        """그리퍼 열기"""
        self.robot_log.append("[INFO] 그리퍼 열기...")
        cmd = "source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash && ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger"
        self._run_service_call(cmd, "그리퍼 열기")

    def gripper_close(self):
        """그리퍼 닫기"""
        self.robot_log.append("[INFO] 그리퍼 닫기...")
        cmd = "source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash && ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger"
        self._run_service_call(cmd, "그리퍼 닫기")

    def move_joint(self):
        """조인트 이동"""
        try:
            # 조인트 값 읽기
            joint_values = []
            for i, field in enumerate(self.joint_inputs):
                val = float(field.text().strip())
                joint_values.append(val)

            pos_str = ", ".join([str(v) for v in joint_values])
            self.robot_log.append(f"[INFO] 조인트 이동: [{pos_str}]")

            cmd = f'source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash && ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{{pos: [{pos_str}], vel: 30.0, acc: 30.0}}"'
            self._run_service_call(cmd, "조인트 이동")

        except ValueError as e:
            self.robot_log.append(f"[ERROR] 잘못된 입력값: {e}")
            QMessageBox.warning(self, "입력 오류", "조인트 값은 숫자로 입력해주세요.")

    def fetch_current_joint_values(self):
        """현재 조인트 값 가져오기"""
        import math

        # 즉시 피드백
        self.robot_log.append("[INFO] 조인트 값 읽는 중...")

        def run_in_thread():
            try:
                cmd = "source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash && ros2 topic echo /dsr01/joint_states sensor_msgs/msg/JointState --once 2>/dev/null"
                result = subprocess.run(
                    ["bash", "-c", cmd],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode == 0:
                    output = result.stdout
                    lines = output.split('\n')

                    # name과 position 파싱
                    names = []
                    positions = []
                    current_section = None

                    for line in lines:
                        stripped = line.strip()
                        if stripped.startswith('name:'):
                            current_section = 'name'
                            continue
                        elif stripped.startswith('position:'):
                            current_section = 'position'
                            continue
                        elif stripped.startswith('velocity:') or stripped.startswith('effort:'):
                            current_section = None
                            continue

                        if current_section == 'name' and stripped.startswith('- '):
                            names.append(stripped[2:].strip())
                        elif current_section == 'position' and stripped.startswith('- '):
                            try:
                                val = float(stripped[2:].strip())
                                positions.append(val)
                            except ValueError:
                                pass

                    # joint_1 ~ joint_6 순서로 재정렬
                    if len(names) >= 6 and len(positions) >= 6:
                        joint_dict = dict(zip(names, positions))
                        ordered_positions = []
                        for i in range(1, 7):
                            joint_name = f"joint_{i}"
                            if joint_name in joint_dict:
                                ordered_positions.append(joint_dict[joint_name])
                            else:
                                ordered_positions.append(0.0)

                        positions_deg = [math.degrees(p) for p in ordered_positions]
                        self.joint_values_updated.emit(positions_deg)
                    else:
                        self.joint_update_failed.emit(f"파싱 실패 (names={len(names)}, pos={len(positions)})")
                else:
                    self.joint_update_failed.emit(f"읽기 실패 (rc={result.returncode})")
            except subprocess.TimeoutExpired:
                self.joint_update_failed.emit("타임아웃")
            except Exception as e:
                self.joint_update_failed.emit(str(e)[:100])

        thread = threading.Thread(target=run_in_thread, daemon=True)
        thread.start()

    def _on_joint_update_failed(self, msg):
        """조인트 값 업데이트 실패 처리"""
        self.robot_log.append(f"[WARN] 조인트 값 읽기 실패: {msg}")

    def _on_controller_finished(self, success, message):
        """컨트롤러 실행 완료 처리"""
        self.btn_execute.setEnabled(True)
        self.btn_execute.setText("실행")
        if success:
            self.robot_log.append(f"[OK] {message}")
        else:
            self.robot_log.append(f"[ERROR] {message}")

    def _update_joint_inputs(self, positions_deg):
        """조인트 입력 필드에 현재값 설정 (메인 스레드에서 호출)"""
        for i, val in enumerate(positions_deg):
            if i < len(self.joint_inputs):
                self.joint_inputs[i].setText(f"{val:.2f}")
        self.robot_log.append(f"[OK] 현재 조인트 값 로드 완료")

    def _run_service_call(self, cmd: str, action_name: str):
        """ROS2 서비스 호출 실행"""
        def run_in_thread():
            try:
                result = subprocess.run(
                    ["bash", "-c", cmd],
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                if result.returncode == 0:
                    self.service_call_finished.emit(True, action_name, "완료")
                else:
                    err_msg = result.stderr[:200] if result.stderr else "실패"
                    self.service_call_finished.emit(False, action_name, err_msg)
            except subprocess.TimeoutExpired:
                self.service_call_finished.emit(False, action_name, "타임아웃")
            except Exception as e:
                self.service_call_finished.emit(False, action_name, str(e)[:100])

        thread = threading.Thread(target=run_in_thread, daemon=True)
        thread.start()

    def _on_service_call_finished(self, success, action_name, message):
        """서비스 호출 완료 처리"""
        if success:
            self.robot_log.append(f"[OK] {action_name} {message}")
        else:
            self.robot_log.append(f"[ERROR] {action_name} 실패: {message}")

    def check_existing_robot_connection(self):
        """기존 로봇 연결 확인 (UI 시작 시 호출)"""
        def check_in_thread():
            try:
                # joint_states 토픽이 발행되고 있는지 확인
                cmd = "source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash && timeout 2 ros2 topic echo /dsr01/joint_states sensor_msgs/msg/JointState --once 2>/dev/null"
                result = subprocess.run(
                    ["bash", "-c", cmd],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                # 토픽 데이터가 있으면 로봇이 연결된 상태
                self.existing_robot_detected.emit(result.returncode == 0 and "position:" in result.stdout)
            except:
                self.existing_robot_detected.emit(False)

        thread = threading.Thread(target=check_in_thread, daemon=True)
        thread.start()

    def _on_existing_robot_detected(self, connected):
        """기존 로봇 연결 감지 결과 처리"""
        if connected:
            self.robot_connected = True
            self.robot_log.append("[INFO] 기존 로봇 연결 감지됨")
            self.update_robot_status("로봇 연결됨 (기존)", "#d4edda")
            self.btn_disconnect.setEnabled(True)
            # 현재 조인트 값도 가져오기
            self.fetch_current_joint_values()

    def disconnect_robot(self):
        """로봇 연결 해제"""
        self.robot_log.append("[INFO] 로봇 연결 해제 중...")

        if self.robot_process and self.robot_process.poll() is None:
            # UI에서 시작한 프로세스인 경우
            import signal
            try:
                os.killpg(os.getpgid(self.robot_process.pid), signal.SIGTERM)
                self.robot_process.wait(timeout=5)
                self.robot_log.append("[OK] 로봇 연결 해제 완료")
            except Exception as e:
                self.robot_log.append(f"[WARN] 정상 종료 실패, 강제 종료: {e}")
                try:
                    os.killpg(os.getpgid(self.robot_process.pid), signal.SIGKILL)
                except:
                    pass
            self.robot_process = None
        elif self.robot_connected:
            # 기존 연결 (다른 곳에서 시작된 프로세스)인 경우 pkill로 종료
            try:
                subprocess.run(["pkill", "-9", "-f", "bringup.launch"], timeout=5)
                subprocess.run(["pkill", "-9", "-f", "ros2_control_node"], timeout=5)
                subprocess.run(["pkill", "-9", "-f", "gripper"], timeout=5)
                subprocess.run(["pkill", "-9", "-f", "robot_state_publisher"], timeout=5)
                self.robot_log.append("[OK] 기존 로봇 프로세스 종료 완료")
            except Exception as e:
                self.robot_log.append(f"[WARN] 프로세스 종료 중 오류: {e}")
        else:
            self.robot_log.append("[INFO] 연결된 로봇이 없습니다.")
            return

        self.robot_connected = False
        self.btn_disconnect.setEnabled(False)
        self.update_robot_status("연결 해제됨", "#ffcccc")

    def emergency_stop(self):
        """긴급 정지 - 모든 로봇 관련 프로세스 강제 종료"""
        import signal
        self.robot_log.append("[EMERGENCY] 긴급 정지 실행!")
        self.update_robot_status("긴급 정지!", "#f44336")

        # 1. 현재 프로세스 종료
        if self.robot_process and self.robot_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.robot_process.pid), signal.SIGKILL)
            except:
                pass
            self.robot_process = None

        # 2. 모든 관련 프로세스 강제 종료
        try:
            subprocess.run(["pkill", "-9", "-f", "ros2.*e0509"], capture_output=True)
            subprocess.run(["pkill", "-9", "-f", "dsr01"], capture_output=True)
            subprocess.run(["pkill", "-9", "-f", "controller_manager"], capture_output=True)
        except:
            pass

        self.robot_connected = False
        self.btn_disconnect.setEnabled(False)
        self.robot_log.append("[EMERGENCY] 모든 로봇 프로세스 종료됨")

    def on_bringup_finished(self, success, message):
        """로봇 시스템 시작 완료"""
        self.set_robot_buttons_enabled(True)
        if success:
            self.robot_connected = True
            mode = "가상" if self.btn_virtual_mode.isChecked() else "실제"
            self.robot_log.append(f"[OK] {mode} 로봇 시스템 시작 완료")
            self.update_robot_status("로봇 시스템 준비됨", "#d4edda")
            # 현재 조인트 값 가져오기
            self.fetch_current_joint_values()
        else:
            self.robot_connected = False
            self.robot_log.append(f"[ERROR] 로봇 시스템 시작 실패: {message}")
            self.update_robot_status("시스템 시작 실패", "#f8d7da")

    def run_full_sequence(self):
        """전체 시퀀스 실행"""
        text = self.input_text.toPlainText().strip() if hasattr(self, 'input_text') else ""
        if not text:
            QMessageBox.warning(self, "경고", "직접 입력 탭에서 쓸 문장을 먼저 입력해주세요.")
            return

        self.set_robot_buttons_enabled(False)
        self.robot_log.append(f"[INFO] 전체 시퀀스 시작: '{text}'")
        self.update_robot_status("전체 시퀀스 실행 중...", "#fff3cd")

        # bringup → grip → write 순차 실행
        self.pending_sentence = text
        if self.btn_virtual_mode.isChecked():
            self.robot_worker = RobotProcessWorker("bringup_virtual")
        else:
            self.robot_worker = RobotProcessWorker("bringup_real")
        self.robot_worker.finished.connect(self.on_sequence_bringup_done)
        self.robot_worker.start()

    def on_sequence_bringup_done(self, success, message):
        """시퀀스: bringup 완료 후 controller 실행"""
        if success:
            self.robot_log.append("[OK] 로봇 시스템 준비 완료, 글씨 쓰기 시작...")
            self.robot_worker = RobotProcessWorker("controller", [self.pending_sentence])
            self.robot_worker.finished.connect(self.on_sequence_complete)
            self.robot_worker.start()
        else:
            self.set_robot_buttons_enabled(True)
            self.robot_log.append(f"[ERROR] 시퀀스 실패: {message}")
            self.update_robot_status("시퀀스 실패", "#f8d7da")

    def on_sequence_complete(self, success, message):
        """시퀀스 완료"""
        self.set_robot_buttons_enabled(True)
        if success:
            self.robot_log.append("[OK] 전체 시퀀스 완료!")
            self.update_robot_status("작업 완료", "#d4edda")
        else:
            self.robot_log.append(f"[ERROR] 시퀀스 실패: {message}")
            self.update_robot_status("작업 실패", "#f8d7da")

    def set_robot_buttons_enabled(self, enabled):
        """로봇 제어 버튼 활성화/비활성화"""
        self.btn_bringup.setEnabled(enabled)
        self.btn_full_sequence.setEnabled(enabled)

    def update_robot_status(self, text, color):
        """로봇 상태 업데이트"""
        self.robot_status_label.setText(f"로봇 상태: {text}")
        self.robot_status_label.setStyleSheet(f"""
            QLabel {{
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                background-color: {color};
                border-radius: 5px;
            }}
        """)

    def setup_direct_input_tab(self):
        """직접 입력 탭 설정 (기존 UI)"""
        layout = QVBoxLayout()

        # 1. 스택 위젯 (모드에 따라 내용이 바뀜)
        self.stack = QStackedWidget()

        # --- 페이지 1: 텍스트 입력 ---
        self.page_text = QWidget()
        text_layout = QVBoxLayout()

        # 텍스트 입력 창 (유동적 크기)
        self.input_text = QTextEdit()
        self.input_text.setPlaceholderText("여기에 내용을 입력하세요...")
        self.input_text.setMinimumSize(380, 150)
        self.input_text.setStyleSheet("""
            QTextEdit {
                border: 1px solid #ccc;
                border-radius: 10px;
                padding: 10px;
                font-size: 14px;
            }
        """)
        self.input_text.textChanged.connect(self.update_button_state)

        text_layout.addWidget(self.input_text, 1)  # stretch factor 1
        self.page_text.setLayout(text_layout)

        # --- 페이지 2: 파일 업로드 ---
        self.page_image = QWidget()
        image_layout = QVBoxLayout()
        self.upload_box = FileUploadBox()
        self.upload_box.file_selected.connect(self.update_button_state)
        image_layout.addWidget(self.upload_box, 1)  # stretch factor 1
        self.page_image.setLayout(image_layout)

        # 스택에 페이지 추가
        self.stack.addWidget(self.page_text)
        self.stack.addWidget(self.page_image)
        layout.addWidget(self.stack, 1)  # stretch factor 1로 스택이 늘어남

        # 2. 상단 모드 선택 (토글 형태의 라디오 버튼)
        mode_layout = QHBoxLayout()
        self.btn_text_mode = QRadioButton("텍스트 입력")
        self.btn_image_mode = QRadioButton("파일 업로드")
        self.btn_text_mode.setChecked(True) # 기본값: 텍스트

        # 버튼 스타일링 (선택 시 강조)
        mode_style = "QRadioButton::indicator { width: 0px; height: 0px; } " \
                     "QRadioButton { padding: 10px; border: 1px solid #ccc; border-radius: 5px; background: #f9f9f9; } " \
                     "QRadioButton:checked { background: #3daee9; color: white; border: 1px solid #3daee9; font-weight: bold; }"
        self.btn_text_mode.setStyleSheet(mode_style)
        self.btn_image_mode.setStyleSheet(mode_style)

        mode_layout.addWidget(self.btn_text_mode)
        mode_layout.addWidget(self.btn_image_mode)
        layout.addLayout(mode_layout)

        # 3. 하단 공통 [미리보기] 버튼
        self.btn_preview = QPushButton("미리보기")
        self.btn_preview.setMinimumHeight(50)
        self.btn_preview.setStyleSheet("""
            QPushButton {
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton[state="active"] {
                background-color: blue;
                color: white;
            }
            QPushButton[state="inactive"] {
                background-color: gray;
                color: black;
            }
            QPushButton:hover { background-color: #27ae60; }
            QPushButton:pressed { background-color: #1e8449; }
        """)
        self.btn_preview.clicked.connect(self.preview)
        layout.addWidget(self.btn_preview)

        # 4. 하단 공통 [실행] 버튼
        self.btn_execute = QPushButton("실 행")
        self.btn_execute.setMinimumHeight(50)
        self.btn_execute.setStyleSheet("""
            QPushButton {
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton[state="active"] {
                background-color: #2ecc71;
                color: white;
            }
            QPushButton[state="inactive"] {
                background-color: gray;
                color: black;
            }
            QPushButton:hover { background-color: #27ae60; }
            QPushButton:pressed { background-color: #1e8449; }
        """)
        self.btn_execute.clicked.connect(self.run_process)
        layout.addWidget(self.btn_execute)

        # 이벤트 연결: 모드 버튼 클릭 시 페이지 전환
        self.update_button_state(False)
        self.btn_text_mode.toggled.connect(self.switch_mode)
        self.btn_image_mode.toggled.connect(self.switch_mode)

        self.direct_input_tab.setLayout(layout)

    def update_button_state(self, flag = None):
        if flag is None:
            flag = self.input_text.toPlainText() != '' \
                if self.btn_text_mode.isChecked() \
                    else self.upload_box.file_path is not None

        self.btn_preview.setEnabled(flag)
        self.btn_execute.setEnabled(flag)
        self.btn_preview.setProperty("state", "active" if flag else "inactive")
        self.btn_execute.setProperty("state", "active" if flag else "inactive")

        # refresh
        self.btn_preview.style().unpolish(self.btn_preview)
        self.btn_preview.style().polish(self.btn_preview)
        self.btn_execute.style().unpolish(self.btn_execute)
        self.btn_execute.style().polish(self.btn_execute)

    def switch_mode(self):
        if self.btn_text_mode.isChecked():
            self.stack.setCurrentIndex(0) # 텍스트 페이지
        else:
            self.stack.setCurrentIndex(1) # 파일 페이지
        self.update_button_state()

    def _is_korean(self, char):
        """문자가 한글인지 확인"""
        if char:
            code = ord(char)
            return (0xAC00 <= code <= 0xD7A3) or (0x3131 <= code <= 0x3163)
        return False

    def _complete_korean_input(self):
        """한글 조합 완료 처리"""
        current_text = self.input_text.toPlainText()
        if current_text and self._is_korean(current_text[-1]):
            # 임시 공백 추가 후 제거하여 한글 조합 완료
            cursor = self.input_text.textCursor()
            cursor.movePosition(cursor.MoveOperation.End)
            cursor.insertText(" ")
            current_text = self.input_text.toPlainText().rstrip()
            self.input_text.setPlainText(current_text)

    def run_process(self):
        """[실행] 버튼 클릭 시 동작 로직"""
        # 1. 로봇 연결 상태 확인
        if not self.robot_connected:
            QMessageBox.warning(self, "경고", "로봇이 연결되지 않았습니다.\n'로봇 제어' 탭에서 먼저 로봇을 연결해주세요.")
            return

        # 2. 입력 내용 확인
        if self.btn_text_mode.isChecked():
            # 한글 조합 완료 처리
            self._complete_korean_input()
            # 텍스트 모드
            content = self.input_text.toPlainText().strip()
            if not content:
                QMessageBox.warning(self, "경고", "텍스트를 입력해주세요!")
                return
            # controller 명령어 생성
            cmd = f'source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash && ros2 run cowritebot controller --sentence "{content}" --skip-grasp'
            self.robot_log.append(f"[INFO] 텍스트 쓰기 시작: '{content}'")
        else:
            # 파일 모드 (Gerber)
            file_path = self.upload_box.file_path
            if not file_path:
                QMessageBox.warning(self, "경고", "파일을 먼저 업로드해주세요!")
                return
            self.current_gerber_path = file_path
            cmd = f'source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash && ros2 run cowritebot controller --gerber "{file_path}" --skip-grasp'
            self.robot_log.append(f"[INFO] Gerber 그리기 시작: {file_path}")

        # 3. 실행 버튼 비활성화
        self.btn_execute.setEnabled(False)
        self.btn_execute.setText("실행 중...")

        # 4. 백그라운드에서 실행
        def run_controller():
            try:
                result = subprocess.run(
                    ["bash", "-c", cmd],
                    capture_output=True,
                    text=True,
                    timeout=600  # 10분 타임아웃
                )
                if result.returncode == 0:
                    self.controller_finished.emit(True, "작업 완료")
                else:
                    self.controller_finished.emit(False, result.stderr[:200] if result.stderr else "실행 실패")
            except subprocess.TimeoutExpired:
                self.controller_finished.emit(False, "타임아웃 (10분 초과)")
            except Exception as e:
                self.controller_finished.emit(False, str(e)[:200])

        thread = threading.Thread(target=run_controller, daemon=True)
        thread.start()

    def preview(self):
        if self.btn_text_mode.isChecked():
            # 텍스트 모드일 때
            content = self.input_text.toPlainText()
            if not content:
                QMessageBox.warning(self, "경고", "텍스트를 입력해주세요!")
            else:
                self.ttp.visualize_robot_path(self.ttp.text_to_path(content))
        else:
            file_path = self.upload_box.file_path
            if not file_path:
                QMessageBox.warning(self, "경고", "파일를 먼저 업로드해주세요!")
            else:
                visualize_gerber(filepath=file_path)

    def _extract_ip(self, text: str) -> str:
        """텍스트에서 IP 주소 추출"""
        import re
        # IP 주소 패턴 (xxx.xxx.xxx.xxx)
        ip_pattern = r'\b(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})\b'
        match = re.search(ip_pattern, text)
        if match:
            ip = match.group(1)
            # 유효한 IP인지 간단히 검증
            parts = ip.split('.')
            if all(0 <= int(p) <= 255 for p in parts):
                return ip
        return ""

    def execute_voice_command(self, command):
        """음성/채팅 명령 실행"""
        if not VOICE_AVAILABLE:
            return

        cmd_type = command.command
        params = command.parameters

        # IP 입력 대기 상태인 경우 처리
        if self.pending_robot_connect:
            # 응답 텍스트에서 IP 추출 시도
            response_text = command.original_text or ""
            ip_pattern = self._extract_ip(response_text)
            if ip_pattern:
                self.pending_robot_connect = False
                self.robot_ip_input.setText(ip_pattern)
                if CHAT_AVAILABLE:
                    self.chat_widget.display_system_message(f"실제 로봇 시스템을 시작합니다... (IP: {ip_pattern})")
                self.btn_real_mode.setChecked(True)  # 실제 모드로 전환
                self.run_bringup(ip_pattern)
                return
            elif cmd_type == RobotCommand.UNKNOWN:
                # 명령이 아닌 일반 텍스트가 IP일 수 있음
                ip_pattern = self._extract_ip(response_text)
                if not ip_pattern:
                    if CHAT_AVAILABLE:
                        self.chat_widget.display_system_message("올바른 IP 주소 형식이 아닙니다. 다시 입력해주세요. (예: 192.168.137.100)")
                    return

        if cmd_type == RobotCommand.WRITE_TEXT:
            text = params.get("text", "")
            if text:
                self.node.send_request(True, text)

        elif cmd_type == RobotCommand.START_SOLDERING:
            # 파일 경로가 있으면 사용, 없으면 마지막 로드된 Gerber 사용
            file_path = params.get("file_path") or self.current_gerber_path
            if file_path:
                self.node.send_request(False, file_path)
            else:
                if CHAT_AVAILABLE:
                    self.chat_widget.display_system_message("Gerber 파일을 먼저 업로드해주세요.")

        elif cmd_type == RobotCommand.LOAD_GERBER:
            file_path = params.get("file_path", "")
            if file_path and os.path.exists(file_path):
                self.current_gerber_path = file_path
                if CHAT_AVAILABLE:
                    self.chat_widget.display_system_message(f"Gerber 파일 로드됨: {file_path}")

        elif cmd_type == RobotCommand.GO_HOME:
            # 홈 위치로 이동 (추후 구현)
            if CHAT_AVAILABLE:
                self.chat_widget.display_system_message("홈 위치로 이동합니다.")

        elif cmd_type == RobotCommand.STOP:
            # 중지 (추후 구현)
            if CHAT_AVAILABLE:
                self.chat_widget.display_system_message("작업을 중지합니다.")

        elif cmd_type == RobotCommand.GET_STATUS:
            # 상태 확인
            status = "현재 대기 중입니다."
            if self.robot_connected:
                status = "로봇 연결됨, 준비 완료"
            if self.current_gerber_path:
                status += f"\n로드된 파일: {os.path.basename(self.current_gerber_path)}"
            if CHAT_AVAILABLE:
                self.chat_widget.display_system_message(status)

        elif cmd_type == RobotCommand.CONNECT_ROBOT:
            # 로봇 시스템 시작 (bringup)
            ip_from_params = params.get("ip") or params.get("host") or params.get("ip_address")

            if self.btn_virtual_mode.isChecked():
                # 가상 모드는 IP 필요 없음
                if CHAT_AVAILABLE:
                    self.chat_widget.display_system_message("가상 로봇 시스템을 시작합니다...")
                self.run_bringup()
            elif ip_from_params:
                # IP가 파라미터로 제공됨
                if CHAT_AVAILABLE:
                    self.chat_widget.display_system_message(f"실제 로봇 시스템을 시작합니다... (IP: {ip_from_params})")
                self.robot_ip_input.setText(ip_from_params)
                self.run_bringup(ip_from_params)
            else:
                # IP 입력 요청
                self.pending_robot_connect = True
                if CHAT_AVAILABLE:
                    self.chat_widget.display_system_message("로봇 IP 주소를 입력해주세요. (예: 192.168.137.100)")

        elif cmd_type == RobotCommand.GRIP_PEN:
            # 그리퍼 열기 (펜 잡기 대신)
            if CHAT_AVAILABLE:
                self.chat_widget.display_system_message("그리퍼를 엽니다...")
            self.gripper_open()

        elif cmd_type == RobotCommand.RUN_SEQUENCE:
            # 전체 시퀀스
            text = params.get("text", "")
            if not text:
                text = self.input_text.toPlainText().strip() if hasattr(self, 'input_text') else ""
            if text:
                self.pending_sentence = text
                if CHAT_AVAILABLE:
                    self.chat_widget.display_system_message(f"전체 시퀀스 시작: '{text}'")
                self.run_full_sequence()
            else:
                if CHAT_AVAILABLE:
                    self.chat_widget.display_system_message("쓸 문장을 먼저 입력해주세요.")


class MainController(Node):
    def __init__(self):
        super().__init__('main_controller_node')
        self.is_processing = False
        self.user_input_req = UserInput.Request()
        self._client = self.create_client(UserInput, 'get_user_input')

    def send_request(self, is_text, contents):
        self.is_processing = True
        self.user_input_req.is_text = is_text
        self.user_input_req.contents = contents
        result = self._client.call_async(self.user_input_req)
        rclpy.spin_until_future_complete(self, result)
        self.is_processing = False
        return result.result()


def main(args=None):
    rclpy.init()
    app = QApplication(sys.argv)
    window = MainUI()
    try:
        window.show()
        sys.exit(app.exec())
    except KeyboardInterrupt:
        pass
    finally:
        window.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
