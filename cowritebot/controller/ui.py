import sys
import os
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QFileDialog, QStackedWidget, 
                             QRadioButton, QMessageBox, QTextEdit, QProgressBar)
from PyQt6.QtGui import QPixmap, QDragEnterEvent, QDropEvent, QColor
from PyQt6.QtCore import Qt, pyqtSignal, QThread
import rclpy
from text_to_path import TextToPath
from gerber_to_path import GerberToPath
from rclpy.node import Node
from rclpy.action import ActionClient
from cowritebot_interfaces.action import UserInput
from visualize_gerber import visualize_gerber

UNEXPECTED_ERROR = -1
FAILED = 0
SUCCEEDED = 1
NEXT_STEP = 2

# --- [로딩 오버레이 위젯] ---
class LoadingOverlay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False) # 마우스 클릭 차단
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground) # 배경 투명 허용
        self.hide() # 기본적으로 숨김

        # 전체 레이아웃 (중앙 정렬)
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # 로딩 서클 (원형 느낌을 내기 위해 스타일링된 ProgressBar 사용)
        self.progress = QProgressBar()
        self.progress.setFixedSize(200, 20)
        self.progress.setRange(0, 0) # ★ 핵심: 최솟값=최댓값=0이면 무한 로딩 애니메이션 작동
        self.progress.setTextVisible(False)
        self.progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #3daee9;
                border-radius: 10px;
                background-color: white;
            }
            QProgressBar::chunk {
                background-color: #3daee9;
                border-radius: 8px;
            }
        """)
        
        # 로딩 텍스트
        self.label = QLabel("요청 처리 중...")
        self.label.setStyleSheet("color: white; font-weight: bold; font-size: 14px; margin-top: 10px;")
        
        layout.addWidget(self.progress)
        layout.addWidget(self.label)
        self.setLayout(layout)

    def paintEvent(self, event):
        # 반투명 검은 배경 그리기
        from PyQt6.QtGui import QPainter
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(0, 0, 0, 150)) # 마지막 인자 150이 투명도 (0~255)

# --- [ROS 요청을 처리할 워커 쓰레드] ---
class ServiceWorker(Node, QThread):
    finished_signal = pyqtSignal(int, str) # 결과를 메인으로 보내는 신호
    progress_signal = pyqtSignal(float)

    def __init__(self, is_text, contents, skip_grasp, scale = 1.0):
        Node.__init__(self, 'request_user_input_node')
        QThread.__init__(self)

        self._action_client = ActionClient(self, UserInput, 'get_user_input')
        self._request_info = {
            'is_text': is_text,
            'contents': contents,
            'skip_grasp': skip_grasp,
            'scale': scale
        }
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        progress = round(feedback.progress, 3)
        self.progress_signal.emit(progress)

    def run(self):
        goal_msg = UserInput.Goal()
        goal_msg.is_text = self._request_info['is_text']
        goal_msg.contents = self._request_info['contents']
        goal_msg.skip_grasp = self._request_info['skip_grasp']
        goal_msg.scale = self._request_info['scale']
        
        # 1. 서버 대기
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.finished_signal.emit(UNEXPECTED_ERROR, "Action Server is not available")
            return

        # 2. 목표 전송 (비동기)
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self.finished_signal.emit(NEXT_STEP, None)
        # 3. 목표가 수락될 때까지 Spin (★ 핵심: 여기서 콜백 처리가 일어남)
        while rclpy.ok() and not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.finished_signal.emit(UNEXPECTED_ERROR, 'Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # 4. 결과 대기 (비동기)
        get_result_future = goal_handle.get_result_async()

        # 5. 결과가 나올 때까지 Spin (★ 핵심: 피드백 콜백도 이 루프 덕분에 실행됨)
        while rclpy.ok() and not get_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        # 6. 최종 결과 처리
        result = get_result_future.result().result
        self.get_logger().info(f'Action Finished. Success: {result.is_success}')

        self.finished_signal.emit(SUCCEEDED if result.is_success else FAILED, f'Action Finished. Success: {result.is_success}')

# --- [이전과 동일한 커스텀 파일 박스 클래스] ---
class FileUploadBox(QLabel):

    file_selected = pyqtSignal(bool) # 클래스 변수 : 모든 객체가 값을 공유

    def __init__(self, parent=None):
        super().__init__(parent)
        self.file_path = None # 멤버 변수 : 객체별로 값이 다 다름
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setText("\n이곳을 클릭하거나\n.gbr 파일을 드래그하여 업로드하세요.\n")
        self.set_default_style()
        self.setFixedHeight(200)
        self.setAcceptDrops(True)
        self.setWordWrap(True)

    def set_default_style(self):
        self.setStyleSheet("border: 2px dashed #aaa; background-color: #f0f0f0; border-radius: 10px; color: #555;")

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            fname, _ = QFileDialog.getOpenFileName(self, '.gbr 파일 선택', './cowritebot/samples', 'Gerber Files (*.gbr)')
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
            self.setText(f"📄\n\n파일이 선택되었습니다:\n{file_name}")
            self.setStyleSheet("border: 1px solid #3daee9; background-color: #ffffff; border-radius: 10px; color: #333; font-weight: bold;")
        
        self.file_selected.emit(True)

# --- [새로 추가된 제어 화면 클래스] ---
class ControlPage(QWidget):
    # 메인 화면으로 돌아가기 위한 시그널 (필요시 사용)
    go_back_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # 상태 표시 라벨
        self.lbl_status = QLabel("로봇이 동작 중입니다...")
        self.lbl_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_status.setStyleSheet("font-size: 18px; font-weight: bold; margin-bottom: 20px;")
        layout.addWidget(self.lbl_status)

        self.lbl_progress = QLabel(f'진행률 : 0%')
        self.lbl_progress.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_progress.setStyleSheet("font-size: 18px; font-weight: bold; margin-bottom: 20px;")
        layout.addWidget(self.lbl_progress)

        # 버튼 3개 생성
        self.btn1 = QPushButton("일시정지")
        self.btn2 = QPushButton("중지 (초기화면)")
        self.btn3 = QPushButton("긴급 정지")

        # 버튼 스타일 공통 적용
        btn_style = """
            QPushButton {
                font-size: 16px; font-weight: bold; border-radius: 10px; padding: 15px;
                background-color: #3498db; color: white;
            }
            QPushButton:hover { background-color: #2980b9; }
            QPushButton:pressed { background-color: #1abc9c; }
        """
        self.btn1.setStyleSheet(btn_style)
        self.btn2.setStyleSheet(btn_style)
        self.btn3.setStyleSheet(btn_style.replace("#3498db", "#e74c3c").replace("#2980b9", "#c0392b")) # 빨간색

        # 버튼 기능 연결 (예시)
        self.btn1.clicked.connect(lambda: print("버튼 1 클릭됨"))
        self.btn2.clicked.connect(self.go_back) # 초기화면으로
        self.btn3.clicked.connect(lambda: print("버튼 3 클릭됨"))

        layout.addWidget(self.btn1)
        layout.addWidget(self.btn2)
        layout.addWidget(self.btn3)
        
        self.setLayout(layout)

    def go_back(self):
        # 초기 화면으로 돌아가기 위해 시그널 발생
        self.go_back_signal.emit()
    
    def set_progress(self, proc):
        self.lbl_progress.setText(f'진행률 : {proc}%')

# --- [메인 애플리케이션 클래스] ---
class MainUI(QWidget):
    def __init__(self):
        super().__init__()
        self.ttp = TextToPath()
        self.gtp = GerberToPath()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('CowriteBot Controller')
        self.setFixedSize(420, 420) # 높이를 조금 늘림

        # ★ 1. 전체 레이아웃 (Root Stack 사용)
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.root_stack = QStackedWidget()

        # --- [화면 1] 입력 페이지 구성 (기존 UI) ---
        self.input_page_widget = QWidget()
        input_page_layout = QVBoxLayout()

        # (1-1) 모드 선택 버튼
        mode_layout = QHBoxLayout()
        self.btn_text_mode = QRadioButton("텍스트 입력")
        self.btn_image_mode = QRadioButton("파일 업로드")
        self.btn_text_mode.setChecked(True)
        
        mode_style = "QRadioButton::indicator { width: 0px; height: 0px; } " \
                     "QRadioButton { padding: 10px; border: 1px solid #ccc; border-radius: 5px; background: #f9f9f9; } " \
                     "QRadioButton:checked { background: #3daee9; color: white; border: 1px solid #3daee9; font-weight: bold; }"
        self.btn_text_mode.setStyleSheet(mode_style)
        self.btn_image_mode.setStyleSheet(mode_style)
        mode_layout.addWidget(self.btn_text_mode)
        mode_layout.addWidget(self.btn_image_mode)
        input_page_layout.addLayout(mode_layout)

        # (1-2) 입력 스택 (텍스트/파일)
        self.input_stack = QStackedWidget()
        
        # 텍스트 입력 페이지
        page_text = QWidget()
        text_layout = QVBoxLayout()
        self.input_text = QTextEdit()
        self.input_text.setPlaceholderText("여기에 내용을 입력하세요...")
        self.input_text.setFixedHeight(200)     # <-- 변경된 코드
        self.input_text.setStyleSheet("""
            QTextEdit {
                border: 1px solid #ccc;
                border-radius: 10px;
                padding: 10px;
                font-size: 14px;
                background-color: white;
            }
        """)
        self.input_text.textChanged.connect(self.update_button_state)
        text_layout.addWidget(self.input_text)
        page_text.setLayout(text_layout)
        
        # 파일 업로드 페이지
        page_image = QWidget()
        image_layout = QVBoxLayout()
        self.upload_box = FileUploadBox()
        self.upload_box.file_selected.connect(self.update_button_state)
        image_layout.addWidget(self.upload_box)
        page_image.setLayout(image_layout)

        self.input_stack.addWidget(page_text)
        self.input_stack.addWidget(page_image)
        input_page_layout.addWidget(self.input_stack)

        # (1-3) 하단 버튼 (미리보기, 실행)
        self.btn_preview = QPushButton("미리보기")
        self.btn_execute = QPushButton("실 행")
        
        btn_base_style = """
            QPushButton { font-size: 16px; font-weight: bold; border-radius: 5px; min-height: 50px; }
            QPushButton[state="active"] { color: white; }
            QPushButton[state="inactive"] { background-color: gray; color: black; }
        """
        self.btn_preview.setStyleSheet(btn_base_style + """
            QPushButton[state="active"] { background-color: blue; } 
            QPushButton:hover { background-color: #0056b3; }
        """)
        self.btn_execute.setStyleSheet(btn_base_style + """
            QPushButton[state="active"] { background-color: #2ecc71; } 
            QPushButton:hover { background-color: #27ae60; }
        """)

        self.btn_preview.clicked.connect(self.preview)
        self.btn_execute.clicked.connect(self.run_process)

        input_page_layout.addWidget(self.btn_preview)
        input_page_layout.addWidget(self.btn_execute)
        
        self.input_page_widget.setLayout(input_page_layout)

        # --- [화면 2] 제어 페이지 구성 (새로 추가됨) ---
        self.control_page_widget = ControlPage()
        # [중지] 버튼 등을 눌렀을 때 다시 입력 화면으로 돌아오도록 연결
        self.control_page_widget.go_back_signal.connect(self.show_input_page)


        # --- Root Stack에 두 화면 추가 ---
        self.root_stack.addWidget(self.input_page_widget)   # index 0
        self.root_stack.addWidget(self.control_page_widget) # index 1

        self.main_layout.addWidget(self.root_stack)

        # 이벤트 연결 및 초기화
        self.update_button_state(False)
        self.btn_text_mode.toggled.connect(self.switch_input_mode)
        self.btn_image_mode.toggled.connect(self.switch_input_mode)

        # ★ 4. 로딩 오버레이 생성 (마지막에 생성해야 맨 위에 뜸)
        self.loading_overlay = LoadingOverlay(self)
    
    def resizeEvent(self, event):
        self.loading_overlay.resize(self.size())
        super().resizeEvent(event)
    
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

    def switch_input_mode(self):
        if self.btn_text_mode.isChecked():
            self.input_stack.setCurrentIndex(0) # 텍스트 페이지
        else:
            self.input_stack.setCurrentIndex(1) # 파일 페이지
        self.update_button_state()

    def show_control_page(self):
        """제어 화면(버튼 3개)으로 전환"""
        self.root_stack.setCurrentIndex(1)

    def show_input_page(self):
        """입력 화면(초기 화면)으로 복귀"""
        self.root_stack.setCurrentIndex(0)
    
    def run_process(self):
        """[실행] 버튼 클릭 시 동작 로직"""
        
        # 1. 데이터 준비
        is_text = self.btn_text_mode.isChecked()
        contents = self.input_text.toPlainText() if is_text else self.upload_box.file_path

        if not contents:
            QMessageBox.warning(self, "경고", "내용을 입력해주세요.")
            return

        # 2. 로딩 화면 켜기 & 버튼 비활성화
        self.loading_overlay.show()
        self.update_button_state(False)

        # 3. 워커 쓰레드 시작 (ROS 요청)
        self.worker = ServiceWorker(is_text, contents, False, 3.0)
        
        # 쓰레드가 끝났을 때 실행될 함수 연결
        self.worker.finished_signal.connect(self.on_processing)
        self.worker.progress_signal.connect(self.set_progress)
        self.set_progress(0)
        self.worker.start()
    
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
    
    def on_processing(self, result_code, result_msg):
        self.loading_overlay.hide()
        self.update_button_state(True)

        # 2. 결과 처리
        if result_code == UNEXPECTED_ERROR:
            QMessageBox.critical(self, "에러", "서비스 연결에 실패했습니다.")
            return
        
        elif result_code == FAILED:
            QMessageBox.critical(self, "실패", result_msg)
        elif result_code == SUCCEEDED:
            QMessageBox.information(self, "성공", result_msg)
            self.show_input_page()
        else:
            self.show_control_page()
    
    def set_progress(self, progress):
        self.control_page_widget.set_progress(progress * 100)
    
    def destroy_node(self):
        self.worker.destroy_node()

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
        window.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()