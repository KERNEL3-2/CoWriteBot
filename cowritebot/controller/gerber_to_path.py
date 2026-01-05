"""
Gerber/Excellon 파일을 로봇 경로로 변환하는 모듈

PCB 납땜 시나리오:
- Gerber 파일: 구리 트레이스 경로 추출
- Excellon 파일: 드릴 홀(납땜 포인트) 위치 추출

사용법:
    gtp = GerberToPath()

    # Gerber 파일에서 트레이스 경로 추출
    strokes = gtp.gerber_to_path("board.gtl")

    # Excellon 파일에서 드릴 포인트 추출
    points = gtp.excellon_to_points("board.drl")

의존성:
    pip install gerbonara
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional
import os

# gerbonara import 시도
try:
    from gerbonara import GerberFile
    from gerbonara.excellon import ExcellonFile
    from gerbonara.graphic_objects import Line, Arc, Flash
    GERBONARA_AVAILABLE = True
except ImportError:
    GERBONARA_AVAILABLE = False
    print("[Warning] gerbonara not installed. Run: pip install gerbonara")


class GerberToPath:
    """Gerber/Excellon 파일을 로봇 경로로 변환"""

    def __init__(self, scale: float = 1.0, offset_x: float = 0.0, offset_y: float = 0.0):
        """
        Args:
            scale: 스케일 팩터 (Gerber 단위 → mm)
            offset_x: X 오프셋 (mm)
            offset_y: Y 오프셋 (mm)
        """
        self.scale = scale
        self.offset_x = offset_x
        self.offset_y = offset_y

        if not GERBONARA_AVAILABLE:
            raise ImportError("gerbonara 라이브러리가 필요합니다. pip install gerbonara")

    def gerber_to_path(self, filepath: str,
                        pad_threshold: float = 1.5,
                        separate_pads: bool = False) -> List[List[Tuple[float, float]]]:
        """
        Gerber 파일에서 트레이스 경로 추출

        Args:
            filepath: Gerber 파일 경로 (.gbr, .gtl, .gbl 등)
            pad_threshold: 패드로 인식할 최대 선분 길이 (mm)
            separate_pads: True면 패드와 트레이스 분리 반환

        Returns:
            strokes: [[(x1, y1), (x2, y2), ...], ...] 형태의 경로 리스트
            separate_pads=True인 경우: (traces, pads) 튜플 반환
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"파일을 찾을 수 없습니다: {filepath}")

        gerber = GerberFile.open(filepath)
        traces = []  # 트레이스 (긴 선분)
        pads = []    # 패드 (짧은 선분)

        for obj in gerber.objects:
            if isinstance(obj, Line):
                x1 = float(obj.x1) * self.scale + self.offset_x
                y1 = float(obj.y1) * self.scale + self.offset_y
                x2 = float(obj.x2) * self.scale + self.offset_x
                y2 = float(obj.y2) * self.scale + self.offset_y

                # 선분 길이로 패드/트레이스 구분
                length = self._distance((x1, y1), (x2, y2))
                if length < pad_threshold:
                    # 패드: 중심점만 저장
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                    pads.append((cx, cy))
                else:
                    traces.append([(x1, y1), (x2, y2)])

            elif isinstance(obj, Arc):
                arc_points = self._arc_to_points(obj)
                if arc_points:
                    traces.append(arc_points)

        # 패드 중복 제거 (같은 위치의 십자 패드 등)
        pads = self._deduplicate_points(pads, tolerance=0.5)

        # 트레이스 병합
        merged_traces = self._merge_connected_strokes(traces)

        if separate_pads:
            return merged_traces, pads
        else:
            return merged_traces

    def _deduplicate_points(self, points: List[Tuple[float, float]],
                           tolerance: float = 0.5) -> List[Tuple[float, float]]:
        """중복 점 제거"""
        if not points:
            return []

        unique = [points[0]]
        for p in points[1:]:
            is_dup = False
            for u in unique:
                if self._distance(p, u) < tolerance:
                    is_dup = True
                    break
            if not is_dup:
                unique.append(p)
        return unique

    def excellon_to_points(self, filepath: str) -> List[Tuple[float, float]]:
        """
        Excellon 드릴 파일에서 홀 위치 추출

        Args:
            filepath: Excellon 파일 경로 (.drl, .xln 등)

        Returns:
            points: [(x1, y1), (x2, y2), ...] 형태의 포인트 리스트
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"파일을 찾을 수 없습니다: {filepath}")

        excellon = ExcellonFile.open(filepath)
        points = []

        for obj in excellon.objects:
            x = float(obj.x) * self.scale + self.offset_x
            y = float(obj.y) * self.scale + self.offset_y
            points.append((x, y))

        # 경로 최적화 (TSP - 가장 가까운 점 순서로 정렬)
        if len(points) > 1:
            points = self._optimize_point_order(points)

        return points

    def excellon_to_path(self, filepath: str, connect: bool = False) -> List[List[Tuple[float, float]]]:
        """
        Excellon 드릴 파일을 경로로 변환

        Args:
            filepath: Excellon 파일 경로
            connect: True면 모든 점을 하나의 경로로 연결

        Returns:
            strokes: 경로 리스트
        """
        points = self.excellon_to_points(filepath)

        if connect and len(points) > 1:
            # 모든 점을 하나의 경로로 연결
            return [points]
        else:
            # 각 점을 개별 "점" 경로로 (짧은 선분)
            strokes = []
            for x, y in points:
                # 점을 작은 십자 모양으로 표현
                size = 0.5  # mm
                strokes.append([(x - size, y), (x + size, y)])
                strokes.append([(x, y - size), (x, y + size)])
            return strokes

    def _arc_to_points(self, arc, num_segments: int = 20) -> List[Tuple[float, float]]:
        """Arc 객체를 점들의 리스트로 변환"""
        try:
            # gerbonara Arc 객체에서 좌표 추출
            cx = float(arc.cx) if hasattr(arc, 'cx') else (float(arc.x1) + float(arc.x2)) / 2
            cy = float(arc.cy) if hasattr(arc, 'cy') else (float(arc.y1) + float(arc.y2)) / 2

            x1, y1 = float(arc.x1), float(arc.y1)
            x2, y2 = float(arc.x2), float(arc.y2)

            # 시작/끝 각도 계산
            start_angle = np.arctan2(y1 - cy, x1 - cx)
            end_angle = np.arctan2(y2 - cy, x2 - cx)
            radius = np.sqrt((x1 - cx)**2 + (y1 - cy)**2)

            # 각도 범위 생성
            if hasattr(arc, 'clockwise') and arc.clockwise:
                if end_angle > start_angle:
                    end_angle -= 2 * np.pi
            else:
                if end_angle < start_angle:
                    end_angle += 2 * np.pi

            angles = np.linspace(start_angle, end_angle, num_segments)

            points = []
            for angle in angles:
                x = cx + radius * np.cos(angle)
                y = cy + radius * np.sin(angle)
                x = x * self.scale + self.offset_x
                y = y * self.scale + self.offset_y
                points.append((x, y))

            return points
        except Exception as e:
            print(f"[Warning] Arc 변환 실패: {e}")
            return []

    def _merge_connected_strokes(self, strokes: List[List[Tuple[float, float]]],
                                  tolerance: float = 0.1) -> List[List[Tuple[float, float]]]:
        """연결된 선분들을 하나의 경로로 병합"""
        if not strokes:
            return []

        merged = []
        used = [False] * len(strokes)

        for i, stroke in enumerate(strokes):
            if used[i]:
                continue

            current = list(stroke)
            used[i] = True
            changed = True

            while changed:
                changed = False
                for j, other in enumerate(strokes):
                    if used[j]:
                        continue

                    # 현재 경로의 끝점과 다른 경로의 시작점이 가까운지 확인
                    if self._distance(current[-1], other[0]) < tolerance:
                        current.extend(other[1:])
                        used[j] = True
                        changed = True
                    elif self._distance(current[-1], other[-1]) < tolerance:
                        current.extend(list(reversed(other))[1:])
                        used[j] = True
                        changed = True
                    elif self._distance(current[0], other[-1]) < tolerance:
                        current = list(other[:-1]) + current
                        used[j] = True
                        changed = True
                    elif self._distance(current[0], other[0]) < tolerance:
                        current = list(reversed(other))[:-1] + current
                        used[j] = True
                        changed = True

            if len(current) > 1:
                merged.append(current)

        # 경로 방향 정규화 (왼쪽 하단에서 시작하도록)
        normalized = self._normalize_stroke_direction(merged)

        return normalized

    def _normalize_stroke_direction(self, strokes: List[List[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:
        """경로 방향 정규화 - 시작점이 끝점보다 왼쪽 또는 아래에 오도록"""
        result = []
        for stroke in strokes:
            if len(stroke) < 2:
                result.append(stroke)
                continue

            start = stroke[0]
            end = stroke[-1]

            # 시작점과 끝점 비교 (왼쪽 하단 우선)
            # x가 더 작거나, x가 같으면 y가 더 작은 쪽이 시작점
            if (start[0] > end[0]) or (start[0] == end[0] and start[1] > end[1]):
                result.append(list(reversed(stroke)))
            else:
                result.append(stroke)

        return result

    def _distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """두 점 사이의 거리"""
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def _optimize_point_order(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        점들의 순서를 최적화 (Nearest Neighbor TSP)
        로봇이 이동하는 총 거리를 최소화
        """
        if len(points) <= 2:
            return points

        remaining = list(points)
        ordered = [remaining.pop(0)]

        while remaining:
            last = ordered[-1]
            # 가장 가까운 점 찾기
            nearest_idx = min(range(len(remaining)),
                            key=lambda i: self._distance(last, remaining[i]))
            ordered.append(remaining.pop(nearest_idx))

        return ordered

    def get_bounds(self, strokes: List[List[Tuple[float, float]]]) -> Tuple[float, float, float, float]:
        """경로의 경계 반환 (min_x, min_y, max_x, max_y)"""
        if not strokes:
            return (0, 0, 0, 0)

        all_points = [p for stroke in strokes for p in stroke]
        xs = [p[0] for p in all_points]
        ys = [p[1] for p in all_points]

        return (min(xs), min(ys), max(xs), max(ys))

    def normalize_to_origin(self, strokes: List[List[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:
        """경로를 원점 기준으로 정규화"""
        min_x, min_y, _, _ = self.get_bounds(strokes)

        normalized = []
        for stroke in strokes:
            normalized.append([(x - min_x, y - min_y) for x, y in stroke])

        return normalized

    def visualize(self, strokes: List[List[Tuple[float, float]]],
                  title: str = "Gerber Path", show: bool = True):
        """경로 시각화"""
        if not strokes:
            print("시각화할 데이터가 없습니다.")
            return

        plt.figure(figsize=(10, 8))
        colors = plt.cm.rainbow(np.linspace(0, 1, len(strokes)))

        for i, (stroke, color) in enumerate(zip(strokes, colors)):
            if len(stroke) < 2:
                continue
            x_vals = [p[0] for p in stroke]
            y_vals = [p[1] for p in stroke]
            plt.plot(x_vals, y_vals, '-', color=color, linewidth=1.5)
            plt.plot(x_vals[0], y_vals[0], 'o', color=color, markersize=4)  # 시작점

        plt.title(f"{title} (Strokes: {len(strokes)})")
        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.axis('equal')
        plt.grid(True, alpha=0.3)

        if show:
            plt.show()

        return plt.gcf()


# 테스트용 더미 데이터 생성 (gerbonara 없이 테스트)
class DummyGerberToPath:
    """gerbonara 없이 테스트용 더미 데이터 생성"""

    def __init__(self, scale: float = 1.0, offset_x: float = 0.0, offset_y: float = 0.0):
        self.scale = scale
        self.offset_x = offset_x
        self.offset_y = offset_y

    def generate_sample_pcb_traces(self) -> List[List[Tuple[float, float]]]:
        """샘플 PCB 트레이스 생성 (IC 핀 패턴)"""
        strokes = []

        # QFP IC 패키지 시뮬레이션 (4면에 핀)
        ic_size = 20  # mm
        pin_pitch = 2  # mm
        pin_length = 3  # mm
        num_pins_per_side = 8

        # 상단 핀들
        for i in range(num_pins_per_side):
            x = -ic_size/2 + pin_pitch * (i + 0.5)
            y1 = ic_size/2
            y2 = ic_size/2 + pin_length
            strokes.append([(x, y1), (x, y2)])

        # 하단 핀들
        for i in range(num_pins_per_side):
            x = -ic_size/2 + pin_pitch * (i + 0.5)
            y1 = -ic_size/2
            y2 = -ic_size/2 - pin_length
            strokes.append([(x, y1), (x, y2)])

        # 좌측 핀들
        for i in range(num_pins_per_side):
            y = -ic_size/2 + pin_pitch * (i + 0.5)
            x1 = -ic_size/2
            x2 = -ic_size/2 - pin_length
            strokes.append([(x1, y), (x2, y)])

        # 우측 핀들
        for i in range(num_pins_per_side):
            y = -ic_size/2 + pin_pitch * (i + 0.5)
            x1 = ic_size/2
            x2 = ic_size/2 + pin_length
            strokes.append([(x1, y), (x2, y)])

        # 트레이스 연결선 추가 (몇 개 핀 연결)
        # 상단 첫번째 핀 → 우측 첫번째 핀
        strokes.append([
            (-ic_size/2 + pin_pitch * 0.5, ic_size/2 + pin_length),
            (-ic_size/2 + pin_pitch * 0.5, ic_size/2 + pin_length + 5),
            (ic_size/2 + pin_length + 5, ic_size/2 + pin_length + 5),
            (ic_size/2 + pin_length + 5, -ic_size/2 + pin_pitch * 0.5),
            (ic_size/2 + pin_length, -ic_size/2 + pin_pitch * 0.5),
        ])

        return strokes

    def generate_sample_drill_points(self) -> List[Tuple[float, float]]:
        """샘플 드릴 포인트 생성"""
        points = []

        # 4x4 그리드 패턴
        for i in range(4):
            for j in range(4):
                x = i * 5 - 7.5
                y = j * 5 - 7.5
                points.append((x, y))

        return points


# 실행 테스트
if __name__ == '__main__':
    if GERBONARA_AVAILABLE:
        gtp = GerberToPath(scale=1.0)
        print("GerberToPath 사용 가능")
    else:
        print("gerbonara 미설치 - DummyGerberToPath 사용")
        gtp = DummyGerberToPath()

    # 샘플 데이터 시각화
    if isinstance(gtp, DummyGerberToPath):
        strokes = gtp.generate_sample_pcb_traces()
        print(f"생성된 트레이스: {len(strokes)}개")

        plt.figure(figsize=(10, 8))
        for stroke in strokes:
            x_vals = [p[0] for p in stroke]
            y_vals = [p[1] for p in stroke]
            plt.plot(x_vals, y_vals, 'b-', linewidth=2)
            plt.plot(x_vals[0], y_vals[0], 'go', markersize=4)

        plt.title("Sample PCB Traces (QFP IC Pattern)")
        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.show()
