"""
대화 관리 모듈

대화 히스토리와 컨텍스트를 관리합니다.
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional, List, Dict, Any
from .command_parser import ParsedCommand


@dataclass
class ChatMessage:
    """채팅 메시지"""
    role: str                   # "user" | "assistant" | "system"
    content: str                # 메시지 내용
    timestamp: datetime = field(default_factory=datetime.now)
    command: Optional[ParsedCommand] = None  # 명령인 경우

    def to_dict(self) -> Dict[str, Any]:
        """딕셔너리로 변환"""
        return {
            "role": self.role,
            "content": self.content,
            "timestamp": self.timestamp.isoformat(),
            "command": self.command.to_dict() if self.command else None
        }

    def is_command(self) -> bool:
        """명령 메시지인지 확인"""
        return self.command is not None and self.command.is_valid()


class ChatManager:
    """대화 히스토리 관리"""

    def __init__(self, max_history: int = 50):
        """
        Args:
            max_history: 최대 히스토리 개수
        """
        self.history: List[ChatMessage] = []
        self.max_history = max_history

    def add_user_message(self, content: str) -> ChatMessage:
        """사용자 메시지 추가"""
        message = ChatMessage(role="user", content=content)
        self._add_message(message)
        return message

    def add_assistant_message(self, content: str,
                              command: Optional[ParsedCommand] = None) -> ChatMessage:
        """어시스턴트 메시지 추가"""
        message = ChatMessage(role="assistant", content=content, command=command)
        self._add_message(message)
        return message

    def add_system_message(self, content: str) -> ChatMessage:
        """시스템 메시지 추가"""
        message = ChatMessage(role="system", content=content)
        self._add_message(message)
        return message

    def _add_message(self, message: ChatMessage):
        """메시지 추가 (내부용)"""
        self.history.append(message)

        # 최대 개수 초과 시 오래된 메시지 제거
        while len(self.history) > self.max_history:
            self.history.pop(0)

    def get_history(self, limit: Optional[int] = None) -> List[ChatMessage]:
        """
        히스토리 반환

        Args:
            limit: 반환할 최대 개수 (None이면 전체)

        Returns:
            메시지 리스트
        """
        if limit is None:
            return list(self.history)
        return list(self.history[-limit:])

    def get_context_for_llm(self, limit: int = 20) -> List[Dict[str, str]]:
        """
        LLM 호출용 컨텍스트 반환

        Args:
            limit: 포함할 메시지 개수

        Returns:
            [{"role": "...", "content": "..."}] 형태
        """
        messages = self.get_history(limit)
        return [
            {"role": msg.role, "content": msg.content}
            for msg in messages
            if msg.role in ("user", "assistant")
        ]

    def get_last_command(self) -> Optional[ParsedCommand]:
        """마지막 명령 반환"""
        for message in reversed(self.history):
            if message.is_command():
                return message.command
        return None

    def clear_history(self):
        """히스토리 초기화"""
        self.history.clear()

    def get_summary(self) -> Dict[str, Any]:
        """대화 요약 정보"""
        command_count = sum(1 for msg in self.history if msg.is_command())
        return {
            "total_messages": len(self.history),
            "user_messages": sum(1 for msg in self.history if msg.role == "user"),
            "assistant_messages": sum(1 for msg in self.history if msg.role == "assistant"),
            "command_count": command_count,
            "last_activity": self.history[-1].timestamp.isoformat() if self.history else None
        }


# 테스트
if __name__ == "__main__":
    from .command_parser import RobotCommand

    manager = ChatManager()

    # 대화 추가
    manager.add_user_message("안녕하세요")
    manager.add_assistant_message("안녕하세요! 무엇을 도와드릴까요?")

    manager.add_user_message("안녕 써줘")
    cmd = ParsedCommand(
        command=RobotCommand.WRITE_TEXT,
        parameters={"text": "안녕"},
        response_text="안녕을 쓰겠습니다."
    )
    manager.add_assistant_message("안녕을 쓰겠습니다.", command=cmd)

    # 출력
    print("=== 대화 히스토리 ===")
    for msg in manager.get_history():
        print(f"[{msg.role}] {msg.content}")
        if msg.is_command():
            print(f"  -> 명령: {msg.command.command.name}")

    print(f"\n=== 요약 ===")
    print(manager.get_summary())

    print(f"\n=== 마지막 명령 ===")
    last_cmd = manager.get_last_command()
    if last_cmd:
        print(last_cmd.to_dict())
