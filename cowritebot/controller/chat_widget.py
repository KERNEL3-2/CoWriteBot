"""
PyQt6 채팅/음성 인터페이스 위젯

채팅창과 음성 입력/출력 기능을 제공합니다.
"""

import sys
import os

# voice_processing 모듈 경로 추가
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QTextEdit, QLineEdit, QPushButton, QLabel, QFrame
)
from PyQt6.QtCore import Qt, pyqtSignal, QThread
from PyQt6.QtGui import QTextCursor, QColor

from voice_processing.llm import LLM
from voice_processing.stt import SpeechRecognizer
from voice_processing.tts import TextToSpeechEngine
from voice_processing.command_parser import CommandParser, ParsedCommand, RobotCommand
from voice_processing.chat_manager import ChatManager


class VoiceWorker(QThread):
    """음성 인식 백그라운드 워커"""
    text_recognized = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    listening_started = pyqtSignal()
    listening_stopped = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.stt = None
        self.should_stop = False

    def run(self):
        """음성 인식 실행"""
        try:
            self.stt = SpeechRecognizer()
            self.listening_started.emit()

            result = self.stt.listen_once(timeout=10.0)
            if result and not self.should_stop:
                self.text_recognized.emit(result)

        except Exception as e:
            self.error_occurred.emit(str(e))
        finally:
            self.listening_stopped.emit()

    def stop(self):
        """음성 인식 중지"""
        self.should_stop = True
        if self.stt:
            self.stt.stop_listening()


class LLMWorker(QThread):
    """LLM 호출 백그라운드 워커"""
    response_ready = pyqtSignal(dict)
    error_occurred = pyqtSignal(str)

    def __init__(self, llm: LLM, user_input: str, parent=None):
        super().__init__(parent)
        self.llm = llm
        self.user_input = user_input

    def run(self):
        """LLM 호출 실행"""
        try:
            result = self.llm.parse_command(self.user_input)
            self.response_ready.emit(result)
        except Exception as e:
            self.error_occurred.emit(str(e))


class ChatWidget(QWidget):
    """채팅/음성 인터페이스 위젯"""
    command_received = pyqtSignal(object)  # ParsedCommand 전달

    def __init__(self, parent=None):
        super().__init__(parent)

        # 컴포넌트 초기화
        self.llm = LLM()
        self.tts = TextToSpeechEngine()
        self.command_parser = CommandParser()
        self.chat_manager = ChatManager()

        # 워커 스레드
        self.voice_worker = None
        self.llm_worker = None

        # TTS 활성화 여부
        self.tts_enabled = True

        self.initUI()

    def initUI(self):
        """UI 초기화"""
        layout = QVBoxLayout()
        layout.setSpacing(10)

        # 1. 채팅 히스토리 영역
        self.chat_display = QTextEdit()
        self.chat_display.setReadOnly(True)
        self.chat_display.setStyleSheet("""
            QTextEdit {
                background-color: #f5f5f5;
                border: 1px solid #ddd;
                border-radius: 8px;
                padding: 10px;
                font-size: 14px;
            }
        """)
        self.chat_display.setMinimumHeight(150)
        layout.addWidget(self.chat_display, 1)  # stretch factor 1로 채팅창이 늘어남

        # 2. 상태 표시줄
        self.status_label = QLabel("대기 중...")
        self.status_label.setStyleSheet("""
            QLabel {
                color: #666;
                font-size: 12px;
                padding: 5px;
            }
        """)
        layout.addWidget(self.status_label)

        # 3. 입력 영역
        input_layout = QHBoxLayout()

        # 텍스트 입력
        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText("메시지를 입력하세요...")
        self.input_field.setStyleSheet("""
            QLineEdit {
                border: 1px solid #ccc;
                border-radius: 5px;
                padding: 10px;
                font-size: 14px;
            }
            QLineEdit:focus {
                border-color: #3daee9;
            }
        """)
        self.input_field.returnPressed.connect(self.send_message)
        input_layout.addWidget(self.input_field)

        # 전송 버튼
        self.send_button = QPushButton("전송")
        self.send_button.setStyleSheet("""
            QPushButton {
                background-color: #3daee9;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px 20px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2d9ed9;
            }
            QPushButton:pressed {
                background-color: #1d8ec9;
            }
        """)
        self.send_button.clicked.connect(self.send_message)
        input_layout.addWidget(self.send_button)

        layout.addLayout(input_layout)

        # 4. 버튼 영역
        button_layout = QHBoxLayout()

        # 마이크 버튼
        self.mic_button = QPushButton("마이크")
        self.mic_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px 20px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton[listening="true"] {
                background-color: #f44336;
            }
        """)
        self.mic_button.clicked.connect(self.toggle_voice_input)
        button_layout.addWidget(self.mic_button)

        # TTS 토글 버튼
        self.tts_button = QPushButton("음성 출력: ON")
        self.tts_button.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px 20px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7B1FA2;
            }
            QPushButton[enabled="false"] {
                background-color: #ccc;
            }
        """)
        self.tts_button.clicked.connect(self.toggle_tts)
        button_layout.addWidget(self.tts_button)

        # 초기화 버튼
        self.clear_button = QPushButton("대화 초기화")
        self.clear_button.setStyleSheet("""
            QPushButton {
                background-color: #757575;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px 20px;
            }
            QPushButton:hover {
                background-color: #616161;
            }
        """)
        self.clear_button.clicked.connect(self.clear_chat)
        button_layout.addWidget(self.clear_button)

        layout.addLayout(button_layout)

        self.setLayout(layout)

        # 초기 메시지
        self.display_system_message("안녕하세요! CoWriteBot 어시스턴트입니다. 무엇을 도와드릴까요?")

    def _is_korean(self, char):
        """문자가 한글인지 확인"""
        if char:
            code = ord(char)
            # 한글 유니코드 범위: 가-힣 (0xAC00-0xD7A3), ㄱ-ㅎ (0x3131-0x314E), ㅏ-ㅣ (0x314F-0x3163)
            return (0xAC00 <= code <= 0xD7A3) or (0x3131 <= code <= 0x3163)
        return False

    def send_message(self):
        """메시지 전송"""
        # 한글 조합 완료를 위해 임시 문자 추가 후 제거
        current_text = self.input_field.text()
        if current_text and self._is_korean(current_text[-1]):
            self.input_field.setText(current_text + " ")
            current_text = self.input_field.text().rstrip()
            self.input_field.setText(current_text)

        text = self.input_field.text().strip()
        if not text:
            return

        self.input_field.clear()
        self.display_user_message(text)
        self.chat_manager.add_user_message(text)

        self.set_status("처리 중...")
        self.set_input_enabled(False)

        # LLM 워커 시작
        self.llm_worker = LLMWorker(self.llm, text)
        self.llm_worker.response_ready.connect(self.on_llm_response)
        self.llm_worker.error_occurred.connect(self.on_llm_error)
        self.llm_worker.start()

    def on_llm_response(self, result: dict):
        """LLM 응답 처리"""
        response = result.get("response", "")
        self.display_assistant_message(response)

        # 명령 파싱
        parsed = self.command_parser.parse(result, "")

        # 대화 히스토리에 추가
        self.chat_manager.add_assistant_message(response, parsed if parsed.is_valid() else None)

        # 명령인 경우 시그널 발생
        if parsed.is_valid():
            self.display_command_info(parsed)
            self.command_received.emit(parsed)

        # TTS 출력
        if self.tts_enabled and response:
            self.tts.speak_async(response)

        self.set_status("대기 중...")
        self.set_input_enabled(True)

    def on_llm_error(self, error: str):
        """LLM 오류 처리"""
        self.display_system_message(f"오류: {error}")
        self.set_status("오류 발생")
        self.set_input_enabled(True)

    def toggle_voice_input(self):
        """음성 입력 토글"""
        if self.voice_worker and self.voice_worker.isRunning():
            self.voice_worker.stop()
            self.mic_button.setText("마이크")
            self.mic_button.setProperty("listening", "false")
            self.mic_button.style().unpolish(self.mic_button)
            self.mic_button.style().polish(self.mic_button)
        else:
            self.start_voice_input()

    def start_voice_input(self):
        """음성 입력 시작"""
        self.mic_button.setText("듣는 중...")
        self.mic_button.setProperty("listening", "true")
        self.mic_button.style().unpolish(self.mic_button)
        self.mic_button.style().polish(self.mic_button)

        self.set_status("음성 입력 대기 중...")

        self.voice_worker = VoiceWorker()
        self.voice_worker.text_recognized.connect(self.on_voice_recognized)
        self.voice_worker.error_occurred.connect(self.on_voice_error)
        self.voice_worker.listening_stopped.connect(self.on_voice_stopped)
        self.voice_worker.start()

    def on_voice_recognized(self, text: str):
        """음성 인식 완료"""
        self.input_field.setText(text)
        self.send_message()

    def on_voice_error(self, error: str):
        """음성 인식 오류"""
        self.display_system_message(f"음성 인식 오류: {error}")
        self.set_status("음성 인식 실패")

    def on_voice_stopped(self):
        """음성 인식 종료"""
        self.mic_button.setText("마이크")
        self.mic_button.setProperty("listening", "false")
        self.mic_button.style().unpolish(self.mic_button)
        self.mic_button.style().polish(self.mic_button)
        self.set_status("대기 중...")

    def toggle_tts(self):
        """TTS 토글"""
        self.tts_enabled = not self.tts_enabled
        if self.tts_enabled:
            self.tts_button.setText("음성 출력: ON")
        else:
            self.tts_button.setText("음성 출력: OFF")
            self.tts.stop()

    def clear_chat(self):
        """대화 초기화"""
        self.chat_display.clear()
        self.chat_manager.clear_history()
        self.llm.clear_history()
        self.display_system_message("대화가 초기화되었습니다.")

    def display_user_message(self, text: str):
        """사용자 메시지 표시"""
        self._append_message("사용자", text, "#e3f2fd", "#1976D2")

    def display_assistant_message(self, text: str):
        """어시스턴트 메시지 표시"""
        self._append_message("CoWriteBot", text, "#f5f5f5", "#333")

    def display_system_message(self, text: str):
        """시스템 메시지 표시"""
        self._append_message("시스템", text, "#fff3e0", "#e65100")

    def display_command_info(self, command: ParsedCommand):
        """명령 정보 표시"""
        cmd_name = command.command.name
        params = command.parameters
        info = f"[명령 인식] {cmd_name}"
        if params:
            info += f" | 파라미터: {params}"
        self._append_message("시스템", info, "#e8f5e9", "#2e7d32")

    def _append_message(self, sender: str, text: str, bg_color: str, text_color: str):
        """메시지 추가"""
        html = f"""
        <div style="background-color: {bg_color}; padding: 8px; margin: 4px 0; border-radius: 5px;">
            <b style="color: {text_color};">{sender}:</b>
            <span style="color: #333;">{text}</span>
        </div>
        """
        self.chat_display.append(html)
        self.chat_display.moveCursor(QTextCursor.MoveOperation.End)

    def set_status(self, text: str):
        """상태 표시"""
        self.status_label.setText(text)

    def set_input_enabled(self, enabled: bool):
        """입력 활성화/비활성화"""
        self.input_field.setEnabled(enabled)
        self.send_button.setEnabled(enabled)


# 독립 실행 테스트
if __name__ == "__main__":
    from PyQt6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    widget = ChatWidget()
    widget.setWindowTitle("CoWriteBot 채팅")
    widget.resize(500, 600)
    widget.show()
    sys.exit(app.exec())