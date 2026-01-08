# CoWriteBot

Doosan E0509 + RH-P12-RN-A 그리퍼를 이용한 글씨 쓰기 Sim2Real 프로젝트

## 개요

Isaac Lab 기반 강화학습으로 로봇이 펜을 적절한 자세로 잡는 방법을 학습하고, 이를 실제 로봇에 적용하는 Sim2Real 파이프라인입니다. GUI와 LLM 기반 음성/채팅 인터페이스를 통해 로봇을 제어할 수 있습니다.

## 관련 레포지토리

| 레포지토리 | 설명 |
|-----------|------|
| **[e0509_gripper_description](https://github.com/KERNEL3-2/e0509_gripper_description)** | ROS2 로봇 패키지 (URDF, launch, 그리퍼 제어) |
| **[CoWriteBotRL](https://github.com/KERNEL3-2/CoWriteBotRL)** | Isaac Lab 기반 강화학습 환경 및 학습 스크립트 |
| **[sim2real](https://github.com/KERNEL3-2/sim2real)** | 강화학습을 통한 로봇 제어 파이프라인 |

---

## 시스템 아키텍처

### 전체 구조
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              CoWriteBot UI                                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │  직접 입력   │  │  로봇 제어   │  │   납땜      │  │  채팅/음성   │        │
│  │  (텍스트/   │  │  (연결/조인트/│  │  (Gerber   │  │  (LLM 기반  │        │
│  │   Gerber)   │  │   그리퍼)    │  │   시각화)   │  │   명령)     │        │
│  └──────┬──────┘  └──────┬──────┘  └─────────────┘  └──────┬──────┘        │
└─────────┼────────────────┼──────────────────────────────────┼───────────────┘
          │                │                                  │
          ▼                ▼                                  ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            ROS2 Communication                                │
│                                                                              │
│  Topics:                          Services:                                  │
│  • /dsr01/joint_states           • /dsr01/gripper/open                      │
│  • /dsr01/state                  • /dsr01/gripper/close                     │
│                                  • /dsr01/motion/move_joint                 │
│                                  • /dsr01/system/robotiq_gripper            │
└─────────────────────────────────────────────────────────────────────────────┘
          │                │                                  │
          ▼                ▼                                  ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Doosan Robot (E0509)                                 │
│                    + RH-P12-RN-A Gripper                                     │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 음성/채팅 인터페이스 구조
```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│  사용자   │ -> │  UI 채팅  │ -> │   STT    │ -> │   LLM    │ -> │  명령    │
│  (음성/  │    │  /음성    │    │ (Google) │    │ (Claude) │    │  파싱    │
│  텍스트) │    │  입력     │    │          │    │          │    │          │
└──────────┘    └──────────┘    └──────────┘    └──────────┘    └────┬─────┘
                                                                      │
                     ┌────────────────────────────────────────────────┘
                     ▼
              ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
              │  Controller  │ -> │    ROS2      │ -> │    로봇      │
              │  (명령 실행)  │    │   Service    │    │   동작      │
              └──────────────┘    └──────────────┘    └──────────────┘
                     │
                     ▼
              ┌──────────────┐    ┌──────────────┐
              │     TTS      │ -> │   음성 출력   │
              │   (gTTS)     │    │   (피드백)    │
              └──────────────┘    └──────────────┘
```

---

## ROS2 통신 구조

### Topics

| Topic | Type | 설명 |
|-------|------|------|
| `/dsr01/joint_states` | `sensor_msgs/JointState` | 현재 조인트 상태 (position, velocity) |
| `/dsr01/state` | `dsr_msgs2/RobotState` | 로봇 전체 상태 |

### Services

| Service | Type | 설명 |
|---------|------|------|
| `/dsr01/gripper/open` | `std_srvs/Trigger` | 그리퍼 열기 |
| `/dsr01/gripper/close` | `std_srvs/Trigger` | 그리퍼 닫기 |
| `/dsr01/motion/move_joint` | `dsr_msgs2/MoveJoint` | 조인트 이동 |
| `/dsr01/system/robotiq_gripper` | - | 그리퍼 세부 제어 |
| `get_pen_position` | `cowritebot_interfaces/GetPenPosition` | 펜 위치 감지 (카메라) |

### 조인트 순서
```
Joint States Topic 순서:  joint_1, joint_2, joint_4, joint_5, joint_3, joint_6
UI 표시 순서:            joint_1, joint_2, joint_3, joint_4, joint_5, joint_6
```
> 주의: ROS2 토픽에서 조인트 순서가 1,2,4,5,3,6으로 반환됨. UI에서 자동 재정렬 처리.

---

## GUI 사용법

### 실행
```bash
cd ~/CoWriteBot/cowritebot/controller
python3 ui.py
```

### 탭 구성

#### 1. 직접 입력 탭
- **텍스트 모드**: 한글/영어 입력 → 로봇이 글씨 작성
- **파일 모드**: Gerber 파일(.gbr, .gtl) 업로드 → PCB 패턴 그리기
- **미리보기**: 경로 시각화
- **실행**: 로봇 동작 시작

#### 2. 로봇 제어 탭
- **로봇 연결**: 시뮬레이션/실제 로봇 선택 및 연결
- **조인트 제어**: 6축 조인트 값 입력 및 이동
- **그리퍼 제어**: 열기/닫기
- **현재값 읽기**: 로봇의 현재 조인트 상태 읽기

#### 3. 납땜 탭
- Gerber 파일 시각화
- Trace/Pad 경로 확인

#### 4. 채팅 탭
- 자연어로 로봇 제어 (LLM 기반)
- 음성 입력 지원 (마이크 버튼)
- TTS 피드백 (스피커 토글)

### 지원 음성/채팅 명령

| 명령 | 예시 | 동작 |
|------|------|------|
| WRITE_TEXT | "안녕 써줘", "Hello 적어줘" | 텍스트 작성 |
| START_SOLDERING | "납땜 시작", "솔더링 해줘" | PCB 납땜 |
| LOAD_GERBER | "이 파일로 작업해" | Gerber 로드 |
| STOP | "멈춰", "정지" | 작업 중지 |
| GO_HOME | "홈으로", "초기 위치로" | 홈 위치 이동 |
| GET_STATUS | "상태 확인", "지금 뭐해?" | 상태 조회 |
| CONNECT_ROBOT | "로봇 연결해줘" | 로봇 연결 |
| OPEN_GRIPPER | "그리퍼 열어" | 그리퍼 열기 |
| CLOSE_GRIPPER | "그리퍼 닫아" | 그리퍼 닫기 |
| MOVE_JOINT | "조인트 1을 30도로" | 조인트 이동 |

---

## 설치

### 1. 로봇 패키지 설치 (ROS2 워크스페이스)
```bash
mkdir -p ~/doosan_ws/src
cd ~/doosan_ws/src

# Doosan 드라이버 (포크 버전 - Flange Serial 지원)
git clone -b humble https://github.com/KERNEL3-2/doosan-robot2.git

# 그리퍼 패키지
git clone https://github.com/ROBOTIS-GIT/RH-P12-RN-A.git

# 로봇 description 패키지
git clone https://github.com/KERNEL3-2/e0509_gripper_description.git

# 글씨 쓰기 패키지
git clone https://github.com/KERNEL3-2/CoWriteBot.git

# 빌드
cd ~/doosan_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 2. Python 의존성 설치
```bash
# 기본 의존성
pip install numpy scipy matplotlib pillow scikit-image

# 음성/채팅 기능
pip install SpeechRecognition gTTS pygame pyaudio langchain-anthropic

# 시스템 패키지 (Ubuntu)
sudo apt install portaudio19-dev python3-pyaudio ffmpeg
```

### 3. API 키 설정
```bash
# cowritebot/resource/.env 파일 생성
echo "ANTHROPIC_API_KEY=your-api-key-here" > ~/CoWriteBot/cowritebot/resource/.env
```

### 4. 환경 설정 (~/.bashrc)
```bash
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
```

---

## 실행 방법

### 방법 1: GUI 사용 (권장)

```bash
# 터미널 1: 로봇 Bringup
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<robot_ip>

# 터미널 2: GUI 실행
cd ~/CoWriteBot/cowritebot/controller
python3 ui.py
```

GUI에서:
1. "로봇 제어" 탭 → 실제 로봇 선택 → IP 입력 → 연결
2. "직접 입력" 탭 → 텍스트 입력 → 실행

### 방법 2: CLI 사용

```bash
# 터미널 1: 로봇 Bringup
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<robot_ip>

# 터미널 2: Controller 직접 실행
ros2 run cowritebot controller --sentence "안녕하세요" --skip-grasp
```

### 방법 3: 통합 런처 사용

```bash
# 터미널 1: 로봇 Bringup
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<robot_ip>

# 터미널 2: 런처 실행 (펜 접근 → 펜 잡기 → 글씨 쓰기 자동 진행)
~/CoWriteBot/cowritebot/pen_write_launcher.py -s "안녕하세요"
```

---

## Controller 옵션

```bash
ros2 run cowritebot controller [OPTIONS]

OPTIONS:
  --sentence, -s TEXT    쓸 문장 (한글 또는 영어)
  --gerber, -g PATH      Gerber 파일 경로 (.gbr, .gtl 등)
  --drill, -d PATH       Excellon 드릴 파일 경로
  --scale FLOAT          Gerber/드릴 스케일 팩터 (기본: 1.0)
  --skip-grasp           펜 잡기 스킵 (이미 잡은 상태)
  --visualize, -v        경로 시각화만 하고 종료
  --sample-pcb           샘플 PCB 패턴 그리기 (테스트용)
```

### 사용 예시
```bash
# 텍스트 쓰기 (펜 이미 잡은 상태)
ros2 run cowritebot controller --sentence "Hello" --skip-grasp

# Gerber 파일 그리기
ros2 run cowritebot controller --gerber ./board.gtl --skip-grasp

# 경로 미리보기만
ros2 run cowritebot controller --sentence "테스트" --visualize
```

---

## 파일 구조

```
CoWriteBot/
├── cowritebot/
│   ├── controller/
│   │   ├── ui.py                 # 메인 GUI
│   │   ├── controller.py         # 로봇 제어 노드
│   │   ├── chat_widget.py        # 채팅 UI 위젯
│   │   ├── text_to_path.py       # 텍스트 → 경로 변환
│   │   ├── gerber_to_path.py     # Gerber → 경로 변환
│   │   └── visualize_gerber.py   # Gerber 시각화
│   ├── voice_processing/
│   │   ├── llm.py                # LLM 기반 명령 파싱
│   │   ├── stt.py                # 음성 → 텍스트 (STT)
│   │   ├── tts.py                # 텍스트 → 음성 (TTS)
│   │   ├── command_parser.py     # 명령어 파싱
│   │   └── chat_manager.py       # 대화 히스토리 관리
│   ├── object_detection/
│   │   └── find_marker.py        # 펜 위치 감지 서비스
│   └── resource/
│       └── .env                  # API 키 설정
├── cowritebot_interfaces/
│   └── srv/
│       ├── GetPenPosition.srv    # 펜 위치 서비스 정의
│       └── UserInput.srv         # 사용자 입력 서비스 정의
└── README.md
```

---

## 트러블슈팅

### 1. 그리퍼 명령 타임아웃
```
message='그리퍼 명령 타임아웃'
```
- **원인**: DART 플랫폼과 ROS2 동시 사용 시 제어권 충돌
- **해결**: DART 플랫폼 완전히 종료 후 재시도

### 2. movesx 포인트 수 초과
```
AssertionError: The 'pos_cnt' field must be an integer in [-128, 127]
```
- **원인**: 한 획의 포인트가 127개 초과
- **해결**: `movesx_chunked()` 사용 (120개씩 분할 실행)

### 3. 한글 입력 안됨
- **해결**: 한글 입력 후 화살표 키를 눌러 조합 완료

### 4. get_pen_position 서비스 대기
```
service not available, waiting again...
```
- **원인**: 펜 감지 서비스 미실행
- **해결**: `--skip-grasp` 옵션 사용 (펜 이미 잡은 상태에서)

### 5. API 키 오류
```
ValidationError: api_key Input should be a valid string
```
- **해결**: `cowritebot/resource/.env` 파일에 `ANTHROPIC_API_KEY` 설정

---

## 하드웨어

- **로봇**: Doosan E0509
- **그리퍼**: Robotis RH-P12-RN-A
- **카메라**: Intel RealSense D455F (Eye-to-Hand 구성)

## 의존성

### 실제 로봇
- ROS2 Humble
- Doosan Robot SDK (dsr_control2)
- RH-P12-RN-A 그리퍼 패키지

### 시뮬레이션 (CoWriteBotRL)
- Isaac Sim 4.5+
- Isaac Lab
- RSL-RL

### 펜 감지
- YOLOv8 Segmentation
- Intel RealSense SDK

## License

Apache-2.0
