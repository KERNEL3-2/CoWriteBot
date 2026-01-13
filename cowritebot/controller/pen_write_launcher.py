#!/usr/bin/env python3
"""
Pen Write Launcher - sim2real 펜 잡기 + CoWriteBot 글씨 쓰기 통합 런처

사용법:
    python pen_write_launcher.py --sentence "안녕하세요"
    python pen_write_launcher.py -s "Hello" --skip-approach  # 접근 스킵 (이미 펜 근처)
    python pen_write_launcher.py -s "테스트" --dry-run       # 테스트 모드

흐름:
    1. sim2real 실행 (RL policy로 펜 접근 + 그리퍼로 잡기)
    2. 목표 도달 시 자동 그리퍼 닫기 + 종료
    3. controller 실행 (글씨 쓰기)
"""

import argparse
import subprocess
import sys
import os
import time

# 경로 설정
HOME = os.path.expanduser("~")
SIM2REAL_PATH = os.path.join(HOME, "sim2real/sim2real/run_sim2real_unified.py")
CALIBRATION_PATH = os.path.join(HOME, "sim2real/sim2real/config/calibration_eye_to_hand.npz")
COWRITEBOT_PATH = os.path.join(HOME, "CoWriteBot")

# ROS2 환경 설정 명령
ROS2_SETUP = """
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
"""


def run_sim2real(checkpoint: str = None, timeout: float = 300.0) -> bool:
    """
    sim2real 실행 (펜 접근 + 그리퍼 잡기)

    Args:
        checkpoint: 모델 경로 (필수)
        timeout: 최대 실행 시간

    Returns:
        bool: 성공 여부
    """
    print("=" * 60)
    print("[1/2] sim2real 실행 - 펜 접근 + 잡기")
    print("=" * 60)

    cmd = [
        "python3", SIM2REAL_PATH,
        "--mode", "ik",
        "--calibration", CALIBRATION_PATH,
    ]

    if checkpoint:
        cmd.extend(["--checkpoint", checkpoint])
    else:
        # 기본 체크포인트 경로
        default_checkpoint = os.path.join(HOME, "e0509_osc_7/model_4999.pt")
        cmd.extend(["--checkpoint", default_checkpoint])

    print(f"  명령: {' '.join(cmd)}")
    print()
    print("  ※ 'g' 키를 눌러 Policy 실행 시작")
    print("  ※ 펜 도달 시 자동으로 그리퍼 닫기 + 종료")
    print()

    try:
        result = subprocess.run(
            cmd,
            timeout=timeout,
            cwd=os.path.dirname(SIM2REAL_PATH),
        )
        success = result.returncode == 0
        print()
        print(f"  sim2real 종료 (코드: {result.returncode}, 성공: {success})")
        return success

    except subprocess.TimeoutExpired:
        print(f"\n  [TIMEOUT] sim2real이 {timeout}초 내에 완료되지 않음")
        return False
    except KeyboardInterrupt:
        print("\n  [중단] 사용자에 의해 중단됨")
        return False
    except Exception as e:
        print(f"\n  [ERROR] sim2real 실행 실패: {e}")
        return False


def run_controller(sentence: str) -> bool:
    """
    controller 실행 (글씨 쓰기)

    sim2real에서 이미 펜을 잡았으므로 글씨 쓰기만 실행

    Args:
        sentence: 쓸 문장

    Returns:
        bool: 성공 여부
    """
    print()
    print("=" * 60)
    print("[2/2] controller 실행 - 글씨 쓰기")
    print("=" * 60)

    # ros2 run 명령으로 실행 (펜 잡기 스킵 - sim2real에서 이미 잡음)
    bash_cmd = f"""
{ROS2_SETUP}
cd {COWRITEBOT_PATH}
ros2 run cowritebot controller --sentence "{sentence}" --skip-grasp
"""

    print(f"  문장: '{sentence}'")
    print()

    try:
        result = subprocess.run(
            ["bash", "-c", bash_cmd],
            timeout=300,  # 5분
        )
        success = result.returncode == 0
        print()
        print(f"  controller 종료 (코드: {result.returncode}, 성공: {success})")
        return success

    except subprocess.TimeoutExpired:
        print("\n  [TIMEOUT] controller가 5분 내에 완료되지 않음")
        return False
    except KeyboardInterrupt:
        print("\n  [중단] 사용자에 의해 중단됨")
        return False
    except Exception as e:
        print(f"\n  [ERROR] controller 실행 실패: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Pen Write Launcher - sim2real 펜 잡기 + CoWriteBot 글씨 쓰기 통합 런처',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예제:
  python pen_write_launcher.py -s "안녕하세요"
  python pen_write_launcher.py -s "Hello" --skip-approach  # 이미 펜을 잡고 있을 때
  python pen_write_launcher.py -s "테스트" --dry-run
        """
    )
    parser.add_argument('--sentence', '-s', type=str, required=True,
                       help='쓸 문장 (한글 또는 영어)')
    parser.add_argument('--skip-approach', action='store_true',
                       help='sim2real 스킵 (이미 펜을 잡고 있을 때)')
    parser.add_argument('--checkpoint', type=str, default=None,
                       help='sim2real 모델 경로 (기본: ~/e0509_osc_7/model_4999.pt)')
    parser.add_argument('--timeout', type=float, default=300.0,
                       help='sim2real 최대 실행 시간 (초, 기본: 300)')
    parser.add_argument('--dry-run', action='store_true',
                       help='실제 실행 없이 명령만 출력')

    args = parser.parse_args()

    print()
    print("=" * 60)
    print("  Pen Write Launcher")
    print("=" * 60)
    print(f"  문장: '{args.sentence}'")
    print(f"  펜 잡기 (sim2real): {'스킵' if args.skip_approach else '실행'}")
    print("=" * 60)

    if args.dry_run:
        print("\n[DRY RUN] 실제 실행하지 않음")
        print(f"\n1. sim2real 명령:")
        print(f"   python3 {SIM2REAL_PATH} --mode ik --calibration {CALIBRATION_PATH} --checkpoint <model>")
        print(f"\n2. controller 명령:")
        print(f"   ros2 run cowritebot controller --sentence \"{args.sentence}\" --skip-grasp")
        return 0

    # 확인
    print("\n계속하려면 Enter, 취소하려면 Ctrl+C...")
    try:
        input()
    except KeyboardInterrupt:
        print("\n취소됨")
        return 1

    start_time = time.time()

    # Step 1: sim2real (펜 접근 + 잡기)
    if not args.skip_approach:
        success = run_sim2real(
            checkpoint=args.checkpoint,
            timeout=args.timeout,
        )
        if not success:
            print("\n[FAILED] sim2real 펜 잡기 실패")
            return 1

        # 잠시 대기 (로봇 안정화)
        print("\n  로봇 안정화 대기 (1초)...")
        time.sleep(1)

    # Step 2: controller (글씨 쓰기)
    success = run_controller(sentence=args.sentence)

    elapsed = time.time() - start_time

    print()
    print("=" * 60)
    if success:
        print(f"  완료! (총 {elapsed:.1f}초)")
    else:
        print(f"  실패 (총 {elapsed:.1f}초)")
    print("=" * 60)

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
