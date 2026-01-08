"""
LLM 기반 대화 및 명령 파싱 모듈

ChatAnthropic을 사용하여 자연어 명령을 로봇 동작으로 변환합니다.
"""

import os
import json
from typing import Optional, List, Dict, Any
from dotenv import load_dotenv
from langchain_anthropic import ChatAnthropic

try:
    from ament_index_python.packages import get_package_share_directory
    package_path = get_package_share_directory("cowritebot")
except:
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# .env 파일 로드 (여러 경로 시도)
_current_file_dir = os.path.dirname(os.path.abspath(__file__))
_possible_env_paths = [
    os.path.join(_current_file_dir, '..', 'resource', '.env'),  # voice_processing에서 상대경로
    os.path.join(os.getcwd(), 'cowritebot/resource/.env'),  # CoWriteBot 루트에서 실행
    os.path.join(package_path, 'resource/.env'),  # ROS2 패키지 경로
]
for _env_path in _possible_env_paths:
    if os.path.exists(_env_path):
        load_dotenv(dotenv_path=_env_path)
        break
anthropic_api_key = os.getenv("ANTHROPIC_API_KEY")


SYSTEM_PROMPT = """당신은 CoWriteBot 로봇의 AI 어시스턴트입니다.
사용자의 자연어 명령을 이해하고 적절한 로봇 동작으로 변환합니다.

## 지원 기능
1. **텍스트 쓰기** (WRITE_TEXT): 한글/영어 문자를 로봇 팔로 씁니다
   - 예: "안녕하세요 써줘", "Hello 적어줘"

2. **납땜 시작** (START_SOLDERING): PCB 납땜 작업을 수행합니다
   - 예: "납땜 시작", "솔더링 해줘"

3. **Gerber 파일 로드** (LOAD_GERBER): PCB 패턴을 로드합니다
   - 예: "이 파일로 작업해", "Gerber 열어"

4. **중지** (STOP): 현재 작업을 중지합니다
   - 예: "멈춰", "정지", "스톱"

5. **홈 위치** (GO_HOME): 로봇을 초기 위치로 이동합니다
   - 예: "홈으로", "초기 위치로"

6. **상태 확인** (GET_STATUS): 현재 로봇 상태를 확인합니다
   - 예: "상태 확인", "지금 뭐해?"

7. **로봇 연결** (CONNECT_ROBOT): 로봇 시스템을 시작합니다 (bringup)
   - 예: "로봇 연결해줘", "로봇 시작해줘", "시스템 켜줘"
   - IP 주소가 포함된 경우: "192.168.137.100으로 연결해줘"
   - 파라미터: {"ip": "IP주소"} (IP 주소가 명시된 경우)

8. **펜 잡기** (GRIP_PEN): 로봇이 펜을 잡습니다
   - 예: "펜 잡아", "펜 집어"

9. **전체 시퀀스** (RUN_SEQUENCE): 접근 → 펜잡기 → 글씨쓰기 전체 실행
   - 예: "전체 실행해줘", "처음부터 해줘"

## 응답 형식
명령을 인식하면 다음 JSON 형식으로 응답하세요:
```json
{
    "is_command": true,
    "command": "COMMAND_TYPE",
    "parameters": {"key": "value"},
    "response": "사용자에게 보여줄 응답"
}
```

일반 대화인 경우:
```json
{
    "is_command": false,
    "command": null,
    "parameters": {},
    "response": "대화 응답"
}
```

항상 친절하고 자연스러운 한국어로 응답하세요."""


class LLM:
    """LLM 기반 대화 및 명령 파싱"""

    def __init__(self):
        self.llm = ChatAnthropic(
            model="claude-haiku-4-5-20251001",
            temperature=0.3,
            api_key=anthropic_api_key
        )
        self.conversation_history: List[Dict[str, str]] = []
        self.max_history = 20

    def _build_messages(self, user_input: str) -> List[Dict[str, str]]:
        """LLM 호출용 메시지 구성"""
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT}
        ]

        # 최근 대화 히스토리 추가
        messages.extend(self.conversation_history[-self.max_history:])

        # 현재 사용자 입력 추가
        messages.append({"role": "user", "content": user_input})

        return messages

    def parse_command(self, user_input: str) -> Dict[str, Any]:
        """
        사용자 입력을 파싱하여 명령 또는 대화 응답 반환

        Args:
            user_input: 사용자 입력 텍스트

        Returns:
            {
                "is_command": bool,
                "command": str or None,
                "parameters": dict,
                "response": str
            }
        """
        messages = self._build_messages(user_input)

        try:
            response = self.llm.invoke(messages)
            content = response.content

            # JSON 파싱 시도
            try:
                # JSON 블록 추출
                if "```json" in content:
                    json_str = content.split("```json")[1].split("```")[0].strip()
                elif "```" in content:
                    json_str = content.split("```")[1].split("```")[0].strip()
                else:
                    json_str = content.strip()

                result = json.loads(json_str)

                # 대화 히스토리에 추가
                self.conversation_history.append({"role": "user", "content": user_input})
                self.conversation_history.append({"role": "assistant", "content": result.get("response", "")})

                return result

            except json.JSONDecodeError:
                # JSON 파싱 실패 시 일반 대화로 처리
                self.conversation_history.append({"role": "user", "content": user_input})
                self.conversation_history.append({"role": "assistant", "content": content})

                return {
                    "is_command": False,
                    "command": None,
                    "parameters": {},
                    "response": content
                }

        except Exception as e:
            return {
                "is_command": False,
                "command": None,
                "parameters": {},
                "response": f"죄송합니다. 오류가 발생했습니다: {str(e)}"
            }

    def chat(self, user_input: str) -> str:
        """
        일반 대화 응답

        Args:
            user_input: 사용자 입력

        Returns:
            응답 텍스트
        """
        result = self.parse_command(user_input)
        return result.get("response", "")

    def clear_history(self):
        """대화 히스토리 초기화"""
        self.conversation_history.clear()

    # 이전 버전 호환용 메서드
    def set_messages(self, msg: str):
        """이전 버전 호환용"""
        self.messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": msg},
        ]

    def response(self) -> Optional[str]:
        """이전 버전 호환용"""
        if not hasattr(self, 'messages') or not self.messages:
            return None
        res = self.llm.invoke(self.messages)
        print(res.content)
        return res.content


# 테스트
if __name__ == "__main__":
    llm = LLM()

    test_inputs = [
        "안녕하세요 써줘",
        "납땜 시작해줘",
        "지금 뭐하고 있어?",
        "멈춰",
        "오늘 날씨 어때?"
    ]

    for user_input in test_inputs:
        print(f"\n사용자: {user_input}")
        result = llm.parse_command(user_input)
        print(f"명령 여부: {result['is_command']}")
        if result['is_command']:
            print(f"명령: {result['command']}")
            print(f"파라미터: {result['parameters']}")
        print(f"응답: {result['response']}")
