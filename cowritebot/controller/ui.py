import sys
import os
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QTextEdit, QPushButton, QLabel, QFileDialog, 
                             QStackedWidget, QRadioButton, QMessageBox)
from PyQt6.QtGui import QPixmap, QDragEnterEvent, QDropEvent, QPalette, QColor
from PyQt6.QtCore import Qt, pyqtSignal
import rclpy
from text_to_path import TextToPath
from gerber_to_path import GerberToPath
from rclpy.node import Node
from cowritebot_interfaces.srv import UserInput
from visualize_gerber import visualize_gerber

# --- [이전과 동일한 커스텀 파일 박스 클래스] ---
class FileUploadBox(QLabel):

    file_selected = pyqtSignal(bool) # 클래스 변수 : 모든 객체가 값을 공유

    def __init__(self, parent=None):
        super().__init__(parent)
        self.file_path = None # 멤버 변수 : 객체별로 값이 다 다름
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setText("\n이곳을 클릭하거나\n.gbr 파일을 드래그하여 업로드하세요.\n")
        self.set_default_style()
        self.setFixedSize(380, 200)
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
            self.setText(f"📄\n\n파일이 선택되었습니다:\n{file_name}")
            self.setStyleSheet("border: 1px solid #3daee9; background-color: #ffffff; border-radius: 10px; color: #333; font-weight: bold;")
        
        self.file_selected.emit(True)

# --- [메인 애플리케이션 클래스] ---
class MainUI(QWidget):
    def __init__(self):
        super().__init__()
        self.node = MainController()
        self.ttp = TextToPath()
        self.gtp = GerberToPath()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('모드 선택')
        self.setFixedSize(420, 400)
        
        main_layout = QVBoxLayout()

        # 1. 스택 위젯 (모드에 따라 내용이 바뀜)
        self.stack = QStackedWidget()

        # --- 페이지 1: 텍스트 입력 ---
        self.page_text = QWidget()
        text_layout = QVBoxLayout()

        # 텍스트 입력 창
        self.input_text = QTextEdit()
        self.input_text.setPlaceholderText("여기에 내용을 입력하세요...")
        self.input_text.setFixedSize(380, 200) # 파일 업로드 박스와 동일한 크기로 설정
        self.input_text.setStyleSheet("""
            QTextEdit {
                border: 1px solid #ccc;
                border-radius: 10px;
                padding: 10px;
                font-size: 14px;
            }
        """)
        self.input_text.textChanged.connect(self.update_button_state)
        
        text_layout.addWidget(self.input_text)
        self.page_text.setLayout(text_layout)

        # --- 페이지 2: 파일 업로드 ---
        self.page_image = QWidget()
        image_layout = QVBoxLayout()
        self.upload_box = FileUploadBox()
        self.upload_box.file_selected.connect(self.update_button_state)
        image_layout.addWidget(self.upload_box)
        self.page_image.setLayout(image_layout)

        # 스택에 페이지 추가
        self.stack.addWidget(self.page_text)
        self.stack.addWidget(self.page_image)
        main_layout.addWidget(self.stack)

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
        main_layout.addLayout(mode_layout)

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
        main_layout.addWidget(self.btn_preview)

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
        main_layout.addWidget(self.btn_execute)

        # 이벤트 연결: 모드 버튼 클릭 시 페이지 전환
        self.update_button_state(False)
        self.btn_text_mode.toggled.connect(self.switch_mode)
        self.btn_image_mode.toggled.connect(self.switch_mode)

        self.setLayout(main_layout)
    
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
    
    def run_process(self):
        """[실행] 버튼 클릭 시 동작 로직"""
        if self.btn_text_mode.isChecked():
            # 텍스트 모드일 때
            content = self.input_text.toPlainText()
            if not content:
                QMessageBox.warning(self, "경고", "텍스트를 입력해주세요!")
            else:
                self.node.send_request(True, content)
        else:
            # 파일 모드일 때
            file_path = self.upload_box.file_path
            if not file_path:
                QMessageBox.warning(self, "경고", "파일를 먼저 업로드해주세요!")
            else:
                self.node.send_request(False, file_path)
    
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