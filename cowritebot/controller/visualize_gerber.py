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

    from gerber_to_path import (
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
        from gerbonara.graphic_objects import Flash
        flash_count = 0
        for obj in gerber.objects:
            if hasattr(obj, 'x1') and hasattr(obj, 'x2'):
                # Line 객체
                x1, y1 = float(obj.x1), float(obj.y1)
                x2, y2 = float(obj.x2), float(obj.y2)
                ax1.plot([x1, x2], [y1, y2], 'b-', linewidth=2)
                ax1.plot(x1, y1, 'go', markersize=3)
            elif isinstance(obj, Flash):
                # Flash 객체 (패드)
                x, y = float(obj.x), float(obj.y)
                ax1.plot(x, y, 'm+', markersize=8, markeredgewidth=2)
                flash_count += 1

        ax1.set_title(f"1. Original Gerber: {os.path.basename(filepath)} (Flash: {flash_count})")

        # 2. 변환된 경로
        gtp = GerberToPath(scale=1.0)
        if separate_pads:
            strokes, pad_strokes = gtp.gerber_to_path(filepath, separate_pads=True)
        else:
            strokes = gtp.gerber_to_path(filepath)
            pad_strokes = []

        # strokes 정규화
        all_strokes = strokes + pad_strokes
        if all_strokes:
            min_x, min_y, _, _ = gtp.get_bounds(all_strokes)
            strokes = [[(p[0] - min_x, p[1] - min_y) for p in stroke] for stroke in strokes]
            # pad_strokes도 같은 offset으로 정규화
            if pad_strokes:
                pad_strokes = [[(p[0] - min_x, p[1] - min_y) for p in stroke] for stroke in pad_strokes]

        title2 = "2. Robot Path (converted)"

    else:
        # 샘플 PCB 패턴
        print("샘플 PCB 패턴 사용 (QFP IC)")
        gtp = DummyGerberToPath()
        strokes = gtp.generate_sample_pcb_traces()
        pad_strokes = []

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
    # 구분이 잘 되는 색상 리스트 (인접 색상이 확실히 다름)
    distinct_colors = [
        '#1f77b4',  # 파랑
        '#ff7f0e',  # 주황
        '#2ca02c',  # 초록
        '#9467bd',  # 보라
        '#17becf',  # 청록
        '#e377c2',  # 분홍
        '#7f7f7f',  # 회색
        '#bcbd22',  # 올리브
        '#8c564b',  # 갈색
        '#d62728',  # 빨강 (마지막에 배치)
    ]

    # 시작점 위치 기록 (겹침 감지용)
    start_points = {}
    for i, stroke in enumerate(strokes):
        if len(stroke) >= 2:
            key = (round(stroke[0][0], 1), round(stroke[0][1], 1))
            if key not in start_points:
                start_points[key] = []
            start_points[key].append(i)

    for i, stroke in enumerate(strokes):
        if len(stroke) < 2:
            continue
        x = [p[0] for p in stroke]
        y = [p[1] for p in stroke]
        color = distinct_colors[i % len(distinct_colors)]
        line_styles = ['-', '--', '-.', ':']
        ax2.plot(x, y, line_styles[i % len(line_styles)], color=color, linewidth=2.5, alpha=0.8)
        ax2.plot(x[-1], y[-1], 'ko', markersize=4) # 끝점 (검은 점)

        # 시작점 겹침 처리 - 겹치면 다른 방향으로 오프셋
        key = (round(x[0], 1), round(y[0], 1))
        overlap_idx = start_points[key].index(i)
        offsets = [(0, 12), (12, 0), (0, -12), (-12, 0)]  # 위, 오른쪽, 아래, 왼쪽
        offset = offsets[overlap_idx % len(offsets)]

        # 시작점에 순서 번호 표시
        ax2.annotate(f'{i+1}', (x[0], y[0]), fontsize=8,
                    color=color, fontweight='bold',
                    ha='center', va='center',
                    xytext=offset, textcoords='offset points',
                    bbox=dict(boxstyle='circle,pad=0.2', fc='white', ec=color, lw=1.5, alpha=0.95))

    # Pad strokes 표시 (마젠타)
    if pad_strokes:
        for stroke in pad_strokes:
            if len(stroke) >= 2:
                x = [p[0] for p in stroke]
                y = [p[1] for p in stroke]
                ax2.plot(x, y, 'm-', linewidth=2, alpha=0.8)

    ax2.set_title(title2)
    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Y (mm)")
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)

    # 범례
    ax2.plot([], [], 'ko', markersize=4, label='End')
    num_pads = len(pad_strokes) // 2  # 십자 하나당 2개의 stroke
    if pad_strokes:
        ax2.plot([], [], 'm-', linewidth=2, alpha=0.8, label=f'Pads ({num_pads})')
    ax2.legend(loc='upper right')

    plt.tight_layout()

    print(f"\n=== 변환 결과 ===")
    print(f"Traces: {len(strokes)}")
    if pad_strokes:
        print(f"Pads: {num_pads} (십자 stroke: {len(pad_strokes)})")
    print(f"(총 stroke: {len(strokes) + len(pad_strokes)})")

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
