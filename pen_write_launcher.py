#!/usr/bin/env python3
"""
Pen Write Launcher - sim2real 펜 잡기 + CoWriteBot 글씨 쓰기 통합 런처

사용법:
    python pen_write_launcher.py --sentence "안녕하세요"
    python pen_write_launcher.py -s "Hello" --skip-approach  # 접근 스킵 (이미 펜 근처)
    python pen_write_launcher.py -s "테스트" --dry-run       # 테스트 모드

흐름:
    1. sim2real 실행 (RL policy로 펜 위치로 접근)
    2. 접근 완료 시 자동 종료
    3. controller 실행 (그리퍼로 펜 잡기 + 글씨 쓰기)
"""

import argparse
import subprocess
import sys
import os
import time

# 경로 설정
HOME = os.path.expanduser("~")
SIM2REAL_PATH = os.path.join(HOME, "sim2real/sim2real/run_sim2real.py")
COWRITEBOT_PATH = os.path.join(HOME, "CoWriteBot")

# ROS2 환경 설정 명령
ROS2_SETUP = """
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
"""


def run_sim2real(auto_start: bool = True, auto_exit: bool = True,
                 checkpoint: str = None, timeout: float = 120.0) -> bool:
    """
    sim2real 실행 (펜 위치로 접근)

    Args:
        auto_start: 펜 감지 시 자동 시작
        auto_exit: 목표 도달 시 자동 종료
        checkpoint: 모델 경로 (기본값 사용 시 None)
        timeout: 최대 실행 시간

    Returns:
        bool: 성공 여부
    """
    print("=" * 60)
    print("[1/2] sim2real 실행 - 펜 위치로 접근")
    print("=" * 60)

    cmd = ["python3", SIM2REAL_PATH]

    if auto_start:
        cmd.append("--auto-start")
    if auto_exit:
        cmd.append("--auto-exit")
    if checkpoint:
        cmd.extend(["--checkpoint", checkpoint])

    print(f"  명령: {' '.join(cmd)}")
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


def run_controller(sentence: str, skip_grasp: bool = False) -> bool:
    """
    controller 실행 (펜 잡기 + 글씨 쓰기)

    Args:
        sentence: 쓸 문장
        skip_grasp: 펜 잡기 스킵

    Returns:
        bool: 성공 여부
    """
    print()
    print("=" * 60)
    print("[2/2] controller 실행 - 펜 잡기 + 글씨 쓰기")
    print("=" * 60)

    # ros2 run 명령으로 실행
    bash_cmd = f"""
{ROS2_SETUP}
cd {COWRITEBOT_PATH}
ros2 run cowritebot controller --sentence "{sentence}" {"--skip-grasp" if skip_grasp else ""}
"""

    print(f"  문장: '{sentence}'")
    print(f"  펜 잡기: {'스킵' if skip_grasp else '실행'}")
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
        description='Pen Write Launcher - sim2real + CoWriteBot 통합 런처',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예제:
  python pen_write_launcher.py -s "안녕하세요"
  python pen_write_launcher.py -s "Hello" --skip-approach
  python pen_write_launcher.py -s "테스트" --dry-run
        """
    )
    parser.add_argument('--sentence', '-s', type=str, required=True,
                       help='쓸 문장 (한글 또는 영어)')
    parser.add_argument('--skip-approach', action='store_true',
                       help='sim2real 접근 단계 스킵 (이미 펜 근처에 있을 때)')
    parser.add_argument('--skip-grasp', action='store_true',
                       help='펜 잡기 스킵 (이미 펜을 잡고 있을 때)')
    parser.add_argument('--checkpoint', type=str, default=None,
                       help='sim2real 모델 경로 (기본: ~/ikv7/model_99999.pt)')
    parser.add_argument('--timeout', type=float, default=120.0,
                       help='sim2real 최대 실행 시간 (초, 기본: 120)')
    parser.add_argument('--dry-run', action='store_true',
                       help='실제 실행 없이 명령만 출력')
    parser.add_argument('--manual-start', action='store_true',
                       help="sim2real에서 'g' 키로 수동 시작 (기본: 자동 시작)")

    args = parser.parse_args()

    print()
    print("=" * 60)
    print("  Pen Write Launcher")
    print("=" * 60)
    print(f"  문장: '{args.sentence}'")
    print(f"  sim2real 접근: {'스킵' if args.skip_approach else '실행'}")
    print(f"  펜 잡기: {'스킵' if args.skip_grasp else '실행'}")
    print(f"  시작 모드: {'수동 (g 키)' if args.manual_start else '자동'}")
    print("=" * 60)

    if args.dry_run:
        print("\n[DRY RUN] 실제 실행하지 않음")
        print(f"\n1. sim2real 명령:")
        print(f"   python3 {SIM2REAL_PATH} --auto-start --auto-exit")
        print(f"\n2. controller 명령:")
        print(f"   ros2 run cowritebot controller --sentence \"{args.sentence}\"")
        return 0

    # 확인
    print("\n계속하려면 Enter, 취소하려면 Ctrl+C...")
    try:
        input()
    except KeyboardInterrupt:
        print("\n취소됨")
        return 1

    start_time = time.time()

    # Step 1: sim2real (접근)
    if not args.skip_approach:
        success = run_sim2real(
            auto_start=not args.manual_start,
            auto_exit=True,
            checkpoint=args.checkpoint,
            timeout=args.timeout,
        )
        if not success:
            print("\n[FAILED] sim2real 접근 실패")
            return 1

        # 잠시 대기 (로봇 안정화)
        print("\n  로봇 안정화 대기 (2초)...")
        time.sleep(2)

    # Step 2: controller (펜 잡기 + 글씨 쓰기)
    success = run_controller(
        sentence=args.sentence,
        skip_grasp=args.skip_grasp,
    )

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
