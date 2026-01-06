import os
from dotenv import load_dotenv
from langchain_anthropic import ChatAnthropic
from ament_index_python.packages import get_package_share_directory

package_path = get_package_share_directory("cowritebot")
is_load = load_dotenv(dotenv_path=os.path.join(os.getcwd(), 'cowritebot/resource/.env'))
# is_laod = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
anthropic_api_key = os.getenv("ANTHROPIC_API_KEY")
############ GetKeyword Node ############
class LLM:
    def __init__(self):
        # 변경: ChatOpenAI → ChatAnthropic
        self.llm = ChatAnthropic(
            model="claude-haiku-4-5-20251001",
            temperature=0.5,
            api_key=anthropic_api_key
        )

    def set_messages(self, msg):
        self.messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": msg},
        ]
    
    def response(self):
        if not self.messages:
            return
        res = self.llm.invoke(self.messages)
        print(res.content)

if __name__ == "__main__":
    get_keyword = LLM()
    get_keyword.set_messages('LLM은 어떤 원리로 작동하나요? 100자 이내로 설명해주세요.')
    get_keyword.response()