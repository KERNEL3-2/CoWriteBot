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
import argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib.pyplot as plt
import numpy as np

def visualize_gerber(filepath=None, separate_pads=False, output=None):
    """Gerber 파일 변환 과정 시각화"""

    from .gerber_to_path import (
        GerberToPath, DummyGerberToPath, GERBONARA_AVAILABLE
    )

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    pads = []

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
        if separate_pads:
            strokes, pads = gtp.gerber_to_path(filepath, separate_pads=True)
        else:
            strokes = gtp.gerber_to_path(filepath)

        # strokes 정규화
        if strokes:
            min_x, min_y, _, _ = gtp.get_bounds(strokes)
            strokes = [[(p[0] - min_x, p[1] - min_y) for p in stroke] for stroke in strokes]
            # pads도 같은 offset으로 정규화 (pads는 (x, y) 튜플 리스트)
            if pads:
                pads = [(p[0] - min_x, p[1] - min_y) for p in pads]

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
        ax2.plot(x[0], y[0], 'g^', markersize=8)   # 시작점 (녹색 삼각형)
        ax2.plot(x[-1], y[-1], 'bs', markersize=6) # 끝점 (파란색 사각형)

        # 이동 순서 표시
        mid_idx = len(x) // 2
        ax2.annotate(f'{i+1}', (x[mid_idx], y[mid_idx]), fontsize=10,
                    color=colors[i], fontweight='bold')

    # Pads 표시 (빨간 원)
    if pads:
        for pad in pads:
            px, py = pad[0], pad[1]
            ax2.plot(px, py, 'ro', markersize=10, alpha=0.7)

    ax2.set_title(title2)
    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Y (mm)")
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)

    # 범례
    ax2.plot([], [], 'g^', markersize=8, label='Start')
    ax2.plot([], [], 'bs', markersize=6, label='End')
    if pads:
        ax2.plot([], [], 'ro', markersize=10, alpha=0.7, label=f'Pads ({len(pads)})')
    ax2.legend(loc='upper right')

    plt.tight_layout()

    print(f"\n=== 변환 결과 ===")
    print(f"Traces: {len(strokes)}")
    if pads:
        print(f"Pads: {len(pads)}")
    print(f"(총 펜 올림/내림 {len(strokes) + len(pads)}회)")

    if output:
        plt.savefig(output, dpi=150, bbox_inches='tight')
        print(f"저장됨: {output}")
    else:
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Gerber 파일 변환 시각화')
    parser.add_argument('--gerber', '-g', type=str, help='Gerber 파일 경로')
    parser.add_argument('--separate-pads', '-s', action='store_true', help='Pad와 Trace 분리')
    parser.add_argument('--output', '-o', type=str, help='출력 이미지 경로')
    parser.add_argument('--sample', action='store_true', help='샘플 PCB 패턴 사용')
    args = parser.parse_args()

    filepath = args.gerber if args.gerber and not args.sample else None
    visualize_gerber(filepath, separate_pads=args.separate_pads, output=args.output)
