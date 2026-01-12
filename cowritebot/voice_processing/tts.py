"""
gTTS 기반 음성 합성 모듈

텍스트를 음성으로 변환하여 재생합니다.
"""

import os
import tempfile
import threading
from typing import Optional, Callable

from gtts import gTTS
import pygame

class TextToSpeechEngine:
    """gTTS를 사용한 음성 합성"""

    def __init__(self, language: str = "ko"):
        """
        Args:
            language: 음성 언어 (기본: 한국어)
        """
        self.language = language
        self.is_speaking = False
        self._speak_thread: Optional[threading.Thread] = None

        # pygame 믹서 초기화
        pygame.mixer.init()

    def speak(self, text: str, blocking: bool = True):
        """
        텍스트를 음성으로 출력

        Args:
            text: 출력할 텍스트
            blocking: True면 재생 완료까지 대기
        """
        if not text.strip():
            return

        if blocking:
            self._speak_sync(text)
        else:
            self._speak_thread = threading.Thread(
                target=self._speak_sync,
                args=(text,),
                daemon=True
            )
            self._speak_thread.start()

    def _speak_sync(self, text: str):
        """동기 방식으로 음성 출력"""
        self.is_speaking = True
        temp_file = None

        try:
            # gTTS로 음성 생성
            tts = gTTS(text=text, lang=self.language)

            # 임시 파일에 저장
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as f:
                temp_file = f.name
                tts.save(temp_file)

            # pygame으로 재생
            pygame.mixer.music.load(temp_file)
            pygame.mixer.music.play()

            # 재생 완료 대기
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)

        except Exception as e:
            print(f"TTS 오류: {e}")
        finally:
            self.is_speaking = False
            # 임시 파일 삭제
            if temp_file and os.path.exists(temp_file):
                try:
                    pygame.mixer.music.unload()
                    os.remove(temp_file)
                except:
                    pass

    def speak_async(self, text: str, callback: Optional[Callable[[], None]] = None):
        """
        비동기로 음성 출력

        Args:
            text: 출력할 텍스트
            callback: 재생 완료 후 호출될 함수
        """
        def speak_with_callback():
            self._speak_sync(text)
            if callback:
                callback()

        self._speak_thread = threading.Thread(target=speak_with_callback, daemon=True)
        self._speak_thread.start()

    def stop(self):
        """재생 중지"""
        if pygame.mixer.music.get_busy():
            pygame.mixer.music.stop()
        self.is_speaking = False

    def wait(self):
        """현재 재생 완료까지 대기"""
        if self._speak_thread and self._speak_thread.is_alive():
            self._speak_thread.join()

    def cleanup(self):
        """리소스 정리"""
        self.stop()
        pygame.mixer.quit()


# 테스트
if __name__ == "__main__":
    tts = TextToSpeechEngine()

    print("TTS 테스트 시작...")
    tts.speak("안녕하세요. 코라이트봇입니다.")
    print("재생 완료")

    tts.cleanup()
