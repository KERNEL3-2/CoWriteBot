"""
Google Speech Recognition 기반 음성 인식 모듈

마이크 입력을 텍스트로 변환합니다.
"""

import speech_recognition as sr
from typing import Callable, Optional
import threading


class SpeechRecognizer:
    """Google Speech Recognition을 사용한 음성 인식"""

    def __init__(self, language: str = "ko-KR"):
        """
        Args:
            language: 인식 언어 (기본: 한국어)
        """
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.language = language
        self.is_listening = False
        self._listen_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # 마이크 노이즈 조정
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)

    def listen_once(self, timeout: float = 5.0) -> Optional[str]:
        """
        한 번 듣고 텍스트 반환

        Args:
            timeout: 대기 시간 (초)

        Returns:
            인식된 텍스트 또는 None
        """
        try:
            with self.microphone as source:
                print("듣는 중...")
                audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=10)

            text = self.recognizer.recognize_google(audio, language=self.language)
            return text

        except sr.WaitTimeoutError:
            print("음성 입력 시간 초과")
            return None
        except sr.UnknownValueError:
            print("음성을 인식할 수 없습니다")
            return None
        except sr.RequestError as e:
            print(f"Google STT 서비스 오류: {e}")
            return None

    def start_listening(self, callback: Callable[[str], None],
                       error_callback: Optional[Callable[[str], None]] = None):
        """
        백그라운드에서 계속 듣기 시작

        Args:
            callback: 텍스트 인식 시 호출될 함수
            error_callback: 에러 발생 시 호출될 함수
        """
        if self.is_listening:
            return

        self._stop_event.clear()
        self.is_listening = True

        def listen_loop():
            while not self._stop_event.is_set():
                try:
                    with self.microphone as source:
                        audio = self.recognizer.listen(
                            source,
                            timeout=3.0,
                            phrase_time_limit=10
                        )

                    text = self.recognizer.recognize_google(audio, language=self.language)
                    if text and callback:
                        callback(text)

                except sr.WaitTimeoutError:
                    continue
                except sr.UnknownValueError:
                    continue
                except sr.RequestError as e:
                    if error_callback:
                        error_callback(f"STT 서비스 오류: {e}")
                    break
                except Exception as e:
                    if error_callback:
                        error_callback(f"예상치 못한 오류: {e}")
                    break

            self.is_listening = False

        self._listen_thread = threading.Thread(target=listen_loop, daemon=True)
        self._listen_thread.start()

    def stop_listening(self):
        """듣기 중지"""
        self._stop_event.set()
        self.is_listening = False
        if self._listen_thread:
            self._listen_thread.join(timeout=2.0)
            self._listen_thread = None


# 테스트
if __name__ == "__main__":
    recognizer = SpeechRecognizer()

    print("마이크에 말씀해주세요...")
    result = recognizer.listen_once(timeout=5.0)

    if result:
        print(f"인식된 텍스트: {result}")
    else:
        print("인식 실패")
