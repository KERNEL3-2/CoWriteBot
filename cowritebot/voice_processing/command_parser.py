"""
로봇 명령 파싱 모듈

LLM 응답을 로봇 실행 가능한 명령으로 변환합니다.
"""

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Dict, Any


class RobotCommand(Enum):
    """지원되는 로봇 명령"""
    WRITE_TEXT = auto()        # 텍스트 쓰기
    START_SOLDERING = auto()   # 납땜 시작
    LOAD_GERBER = auto()       # Gerber 파일 로드
    STOP = auto()              # 중지
    GO_HOME = auto()           # 홈 위치로 이동
    GET_STATUS = auto()        # 상태 확인
    GRIP_PEN = auto()          # 펜 잡기
    RELEASE_PEN = auto()       # 펜 놓기
    # CONNECT_ROBOT = auto()     # 로봇 연결 (sim2real)
    RUN_SEQUENCE = auto()      # 전체 시퀀스 실행
    UNKNOWN = auto()           # 인식 불가


@dataclass
class ParsedCommand:
    """파싱된 로봇 명령"""
    command: RobotCommand
    parameters: Dict[str, Any] = field(default_factory=dict)
    confidence: float = 1.0
    original_text: str = ""
    response_text: str = ""

    def is_valid(self) -> bool:
        """명령이 유효한지 확인"""
        return self.command != RobotCommand.UNKNOWN

    def to_dict(self) -> Dict[str, Any]:
        """딕셔너리로 변환"""
        return {
            "command": self.command.name,
            "parameters": self.parameters,
            "confidence": self.confidence,
            "original_text": self.original_text,
            "response_text": self.response_text
        }


class CommandParser:
    """LLM 응답을 로봇 명령으로 변환"""

    # 명령 문자열 → RobotCommand 매핑
    COMMAND_MAP = {
        "WRITE_TEXT": RobotCommand.WRITE_TEXT,
        "START_SOLDERING": RobotCommand.START_SOLDERING,
        "LOAD_GERBER": RobotCommand.LOAD_GERBER,
        "STOP": RobotCommand.STOP,
        "GO_HOME": RobotCommand.GO_HOME,
        "GET_STATUS": RobotCommand.GET_STATUS,
        "GRIP_PEN": RobotCommand.GRIP_PEN,
        "RELEASE_PEN": RobotCommand.RELEASE_PEN,
        # "CONNECT_ROBOT": RobotCommand.CONNECT_ROBOT,
        "RUN_SEQUENCE": RobotCommand.RUN_SEQUENCE,
    }

    def parse(self, llm_result: Dict[str, Any], original_text: str = "") -> ParsedCommand:
        """
        LLM 결과를 ParsedCommand로 변환

        Args:
            llm_result: LLM.parse_command()의 반환값
            original_text: 원본 사용자 입력

        Returns:
            ParsedCommand 객체
        """
        is_command = llm_result.get("is_command", False)
        command_str = llm_result.get("command")
        parameters = llm_result.get("parameters", {})
        response = llm_result.get("response", "")

        if not is_command or not command_str:
            return ParsedCommand(
                command=RobotCommand.UNKNOWN,
                parameters={},
                confidence=0.0,
                original_text=original_text,
                response_text=response
            )

        # 명령 문자열을 RobotCommand로 변환
        command = self.COMMAND_MAP.get(command_str.upper(), RobotCommand.UNKNOWN)

        # 파라미터 정규화
        normalized_params = self._normalize_parameters(command, parameters)

        return ParsedCommand(
            command=command,
            parameters=normalized_params,
            confidence=1.0 if command != RobotCommand.UNKNOWN else 0.0,
            original_text=original_text,
            response_text=response
        )

    def _normalize_parameters(self, command: RobotCommand, params: Dict[str, Any]) -> Dict[str, Any]:
        """명령별 파라미터 정규화"""
        normalized = {}

        if command == RobotCommand.WRITE_TEXT:
            # 텍스트 추출
            text = params.get("text") or params.get("content") or params.get("message", "")
            normalized["text"] = str(text).strip()
            # 스케일 추출
            if "scale" in params:
                normalized["scale"] = float(params["scale"])

        elif command == RobotCommand.LOAD_GERBER:
            # 파일 경로 추출
            path = params.get("file_path") or params.get("path") or params.get("file", "")
            normalized["file_path"] = str(path).strip()

        elif command == RobotCommand.START_SOLDERING:
            # 옵션: Gerber 경로, 스케일 등
            if "file_path" in params:
                normalized["file_path"] = str(params["file_path"]).strip()
            if "scale" in params:
                normalized["scale"] = float(params["scale"])

        # 그 외 명령은 파라미터 그대로 전달
        else:
            normalized = params

        return normalized


# 테스트
if __name__ == "__main__":
    parser = CommandParser()

    # 테스트 케이스
    test_cases = [
        {
            "is_command": True,
            "command": "WRITE_TEXT",
            "parameters": {"text": "안녕하세요"},
            "response": "안녕하세요를 쓰겠습니다."
        },
        {
            "is_command": True,
            "command": "START_SOLDERING",
            "parameters": {},
            "response": "납땜을 시작합니다."
        },
        {
            "is_command": False,
            "command": None,
            "parameters": {},
            "response": "오늘 날씨가 좋네요."
        }
    ]

    for llm_result in test_cases:
        parsed = parser.parse(llm_result, "테스트 입력")
        print(f"\n입력: {llm_result}")
        print(f"결과: {parsed.to_dict()}")
        print(f"유효: {parsed.is_valid()}")
