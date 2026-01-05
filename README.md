# CoWriteBot

# Sim2Real

Doosan E0509 + RH-P12-RN-A 그리퍼를 이용한 글씨 쓰기 Sim2Real 프로젝트

## 개요

Isaac Lab 기반 강화학습으로 로봇이 펜을 적절한 자세로 잡는 방법을 학습하고, 이를 실제 로봇에 적용하는 Sim2Real 파이프라인입니다.

## 관련 레포지토리

| 레포지토리 | 설명 |
|-----------|------|
| **[e0509_gripper_description](https://github.com/KERNEL3-2/e0509_gripper_description)** | ROS2 로봇 패키지 (URDF, launch, 그리퍼 제어) |
| **[CoWriteBotRL](https://github.com/KERNEL3-2/CoWriteBotRL)** | Isaac Lab 기반 강화학습 환경 및 학습 스크립트 |
| **[sim2real](https://github.com/KERNEL3-2/sim2real)** | 강화학습을 통한 로봇 제어 파이프라인 | 


## 의존성

### 시뮬레이션 (CoWriteBotRL)
- Isaac Sim 4.5+
- Isaac Lab
- RSL-RL

### 실제 로봇 (e0509_gripper_description)
- ROS2 Humble
- Doosan Robot SDK (dsr_control2)
- RH-P12-RN-A 그리퍼 패키지

### 펜 감지
- YOLOv8 Segmentation
- Intel RealSense SDK

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

### 2. Sim2Real 레포 클론
```bash
cd ~
git clone https://github.com/KERNEL3-2/sim2real.git
```

### 3. Python 의존성 설치
```bash
cd ~/sim2real
pip install torch numpy scipy roboticstoolbox-python ultralytics pyrealsense2 opencv-python
```

### 4. (선택) 강화학습 환경 설치
시뮬레이션 학습을 하려면 CoWriteBotRL도 클론:
```bash
cd ~
git clone https://github.com/KERNEL3-2/CoWriteBotRL.git
```

### 5. 환경 설정 (~/.bashrc)
```bash
# ROS2
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash

# Python 경로 (sim2real 모듈 import용)
export PYTHONPATH=$PYTHONPATH:~/sim2real
```

## 빠른 시작

### 1. 시뮬레이션 학습 (CoWriteBotRL)
```bash
source ~/isaacsim_env/bin/activate
cd ~/CoWriteBotRL
python pen_grasp_rl/scripts/train_v7.py --headless --num_envs <num_envs> --max_iterations <num_iter>
```

### 2. 학습된 정책 테스트 (시뮬레이션)
```bash
python pen_grasp_rl/scripts/play_v7.py --checkpoint <model_path>
```

### 3. 실제 로봇 실행

#### 방법 A: 통합 런처 사용 (권장)
```bash
# 터미널 1: 로봇 bringup
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<robot_ip>

# 터미널 2: 런처 실행 (펜 접근 → 펜 잡기 → 글씨 쓰기 자동 진행)
~/CoWriteBot/pen_write_launcher.py -s "안녕하세요"
```

#### 방법 B: 단계별 수동 실행
```bash
# 터미널 1: 로봇 bringup
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<robot_ip>

# 터미널 2: Sim2Real 실행 (펜 위치로 접근)
cd ~/sim2real/sim2real
python3 run_sim2real.py --checkpoint <model_path>

# 터미널 3: 글씨 쓰기 실행 (펜 잡기 + 글씨 쓰기)
source ~/CoWriteBot/install/setup.bash
ros2 run cowritebot controller --sentence "안녕하세요"
```

## 실행 흐름

```
┌─────────────────────────────────────────────────────────────┐
│                    pen_write_launcher.py                     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  [1] sim2real (run_sim2real.py --auto-start --auto-exit)    │
│                                                              │
│   • YOLO로 펜 캡 위치/방향 감지                              │
│   • RL Policy가 TCP 델타 계산                                │
│   • Differential IK로 관절 각도 변환                         │
│   • 로봇 이동 → 펜 캡 2cm 이내 도달 시 자동 종료             │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  [2] controller (ros2 run cowritebot controller)            │
│                                                              │
│   • grisp_pen(): 그리퍼 열기 → 닫기 (펜 잡기)               │
│   • TextToPath: 텍스트 → 로봇 경로 변환                      │
│   • pendown(): 힘제어로 펜 내리기                            │
│   • movesx(): 경로 따라 이동 (글씨 쓰기)                     │
│   • penup(): 펜 올리기 → 완료                                │
└─────────────────────────────────────────────────────────────┘
```

## 런처 옵션

```bash
# 기본 사용
~/CoWriteBot/pen_write_launcher.py -s "문장"

# 수동 시작 (sim2real에서 'g' 키로 시작)
~/CoWriteBot/pen_write_launcher.py -s "문장" --manual-start

# sim2real 스킵 (이미 펜 근처에 있을 때)
~/CoWriteBot/pen_write_launcher.py -s "문장" --skip-approach

# 펜 잡기 스킵 (이미 펜을 잡고 있을 때)
~/CoWriteBot/pen_write_launcher.py -s "문장" --skip-grasp

# 테스트 (실제 실행 없이 명령만 확인)
~/CoWriteBot/pen_write_launcher.py -s "문장" --dry-run
```




## 하드웨어

- **로봇**: Doosan E0509
- **그리퍼**: Robotis RH-P12-RN-A
- **카메라**: Intel RealSense D455F (Eye-to-Hand 구성)

## License

Apache-2.0
