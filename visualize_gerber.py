#!/usr/bin/env python3
"""
Gerber 파일 변환 시각화 도구

사용법:
    ./visualize_gerber.py                     # 샘플 PCB 패턴
    ./visualize_gerber.py path/to/file.gbr    # 실제 Gerber 파일
    ./visualize_gerber.py --sample            # 샘플 PCB 패턴
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib.pyplot as plt
import numpy as np

def visualize_gerber(filepath=None):
    """Gerber 파일 변환 과정 시각화"""

    from cowritebot.controller.gerber_to_path import (
        GerberToPath, DummyGerberToPath, GERBONARA_AVAILABLE
    )

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    if filepath and os.path.exists(filepath) and GERBONARA_AVAILABLE:
        # 실제 Gerber 파일
        from gerbonara import GerberFile
        gerber = GerberFile.open(filepath)

        # 1. 원본 Gerber
        ax1 = axes[0]
        for obj in gerber.objects:
            if hasattr(obj, 'x1') and hasattr(obj, 'x2'):
                x1, y1 = float(obj.x1), float(obj.y1)
                x2, y2 = float(obj.x2), float(obj.y2)
                ax1.plot([x1, x2], [y1, y2], 'b-', linewidth=2)
                ax1.plot(x1, y1, 'go', markersize=3)

        ax1.set_title(f"1. Original Gerber: {os.path.basename(filepath)}")

        # 2. 변환된 경로
        gtp = GerberToPath(scale=1.0)
        strokes = gtp.gerber_to_path(filepath)
        strokes = gtp.normalize_to_origin(strokes)
        title2 = "2. Robot Path (converted)"

    else:
        # 샘플 PCB 패턴
        print("샘플 PCB 패턴 사용 (QFP IC)")
        gtp = DummyGerberToPath()
        strokes = gtp.generate_sample_pcb_traces()

        # 1. 원본 (개별 선분으로 표시)
        ax1 = axes[0]
        for stroke in strokes:
            if len(stroke) >= 2:
                # 각 선분을 개별로 표시
                for i in range(len(stroke) - 1):
                    ax1.plot([stroke[i][0], stroke[i+1][0]],
                            [stroke[i][1], stroke[i+1][1]], 'b-', linewidth=2)
                ax1.plot(stroke[0][0], stroke[0][1], 'go', markersize=3)

        ax1.set_title("1. Original PCB Pattern (QFP IC)")
        title2 = "2. Robot Path (optimized)"

    ax1.set_xlabel("X (mm)")
    ax1.set_ylabel("Y (mm)")
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)

    # 2. 변환된 로봇 경로
    ax2 = axes[1]
    colors = plt.cm.rainbow(np.linspace(0, 1, max(len(strokes), 1)))

    for i, stroke in enumerate(strokes):
        if len(stroke) < 2:
            continue
        x = [p[0] for p in stroke]
        y = [p[1] for p in stroke]
        ax2.plot(x, y, '-', color=colors[i], linewidth=2)
        ax2.plot(x[0], y[0], 'go', markersize=6)   # 시작점 (녹색)
        ax2.plot(x[-1], y[-1], 'ro', markersize=6) # 끝점 (빨강)

        # 이동 순서 표시
        mid_idx = len(x) // 2
        ax2.annotate(f'{i+1}', (x[mid_idx], y[mid_idx]), fontsize=10,
                    color=colors[i], fontweight='bold')

    ax2.set_title(title2)
    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Y (mm)")
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)

    # 범례
    ax2.plot([], [], 'go', markersize=8, label='Start')
    ax2.plot([], [], 'ro', markersize=8, label='End')
    ax2.legend(loc='upper right')

    plt.tight_layout()

    print(f"\n=== 변환 결과 ===")
    print(f"총 stroke 수: {len(strokes)}")
    print(f"(펜 올림/내림 {len(strokes)}회)")

    plt.show()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] != '--sample':
        visualize_gerber(sys.argv[1])
    else:
        visualize_gerber(None)
