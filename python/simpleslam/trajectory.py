"""轨迹读写：TUM 与 KITTI，与 C++ 侧同一口径。

位姿在这里是 ``(平移, 四元数)``，四元数按 Hamilton 约定存成 ``(w, x, y, z)``
（与全框架的公约一致；TUM 文件里的列序是 xyzw，读写时各转一次）。
不引入 numpy：这个模块要能在任何装了 Python 的机器上直接用。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Iterator, Sequence

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # (w, x, y, z)
Mat3 = tuple[Vec3, Vec3, Vec3]


class TrajectoryError(ValueError):
    """轨迹文件格式不对，或时间不是严格递增。"""


# ── 位姿的最小代数 ──


def quat_normalize(q: Quat) -> Quat:
    norm = math.sqrt(sum(c * c for c in q))
    if norm < 1e-12:
        raise TrajectoryError("四元数为零")
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def quat_multiply(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conjugate(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    rotated = quat_multiply(quat_multiply(q, (0.0, *v)), quat_conjugate(q))
    return (rotated[1], rotated[2], rotated[3])


def quat_to_matrix(q: Quat) -> Mat3:
    w, x, y, z = quat_normalize(q)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def matrix_to_quat(m: Mat3) -> Quat:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return quat_normalize(
            (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
        )
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return quat_normalize(
            ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
        )
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return quat_normalize(
            ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
        )
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return quat_normalize(
        ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
    )


@dataclass(frozen=True)
class Pose:
    translation: Vec3 = (0.0, 0.0, 0.0)
    rotation: Quat = (1.0, 0.0, 0.0, 0.0)

    def compose(self, other: "Pose") -> "Pose":
        t = quat_rotate(self.rotation, other.translation)
        return Pose(
            (
                self.translation[0] + t[0],
                self.translation[1] + t[1],
                self.translation[2] + t[2],
            ),
            quat_normalize(quat_multiply(self.rotation, other.rotation)),
        )

    def inverse(self) -> "Pose":
        inv = quat_conjugate(self.rotation)
        t = quat_rotate(inv, self.translation)
        return Pose((-t[0], -t[1], -t[2]), inv)

    def scaled(self, scale: float) -> "Pose":
        return Pose(
            (self.translation[0] * scale, self.translation[1] * scale,
             self.translation[2] * scale),
            self.rotation,
        )

    @property
    def matrix(self) -> Mat3:
        return quat_to_matrix(self.rotation)

    def angle_deg(self) -> float:
        """本位姿旋转部分的转角（度）。"""
        w = min(1.0, abs(quat_normalize(self.rotation)[0]))
        return math.degrees(2.0 * math.acos(w))


@dataclass
class Sample:
    time: float  # 秒
    pose: Pose


@dataclass
class Trajectory:
    """按时间严格递增排列的位姿序列。"""

    samples: list[Sample] = field(default_factory=list)
    name: str = ""

    def push(self, time: float, pose: Pose) -> None:
        if self.samples and time <= self.samples[-1].time:
            raise TrajectoryError("轨迹样本的时间必须严格递增")
        self.samples.append(Sample(time, pose))

    def __len__(self) -> int:
        return len(self.samples)

    def __iter__(self) -> Iterator[Sample]:
        return iter(self.samples)

    def __getitem__(self, index: int) -> Sample:
        return self.samples[index]

    @property
    def times(self) -> list[float]:
        return [sample.time for sample in self.samples]

    def path_length(self) -> float:
        total = 0.0
        for a, b in zip(self.samples, self.samples[1:]):
            total += math.dist(a.pose.translation, b.pose.translation)
        return total

    def nearest(self, time: float, tolerance: float) -> int | None:
        """最近邻样本的下标；超出容差返回 None。"""
        if not self.samples:
            return None
        best = min(range(len(self.samples)), key=lambda i: abs(self.samples[i].time - time))
        if abs(self.samples[best].time - time) > tolerance:
            return None
        return best

    def transformed_by(self, transform: Pose) -> "Trajectory":
        out = Trajectory(name=self.name)
        for sample in self.samples:
            out.samples.append(Sample(sample.time, transform.compose(sample.pose)))
        return out

    def scaled(self, scale: float) -> "Trajectory":
        out = Trajectory(name=self.name)
        for sample in self.samples:
            out.samples.append(Sample(sample.time, sample.pose.scaled(scale)))
        return out


# ── 文件格式 ──


def _numbers(line: str) -> list[float]:
    return [float(token) for token in line.split()]


def _iter_data_lines(lines: Iterable[str]) -> Iterator[tuple[int, str]]:
    for number, raw in enumerate(lines, start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        yield number, line


def read_tum(path: str) -> Trajectory:
    trajectory = Trajectory(name=path)
    with open(path, "r", encoding="utf-8") as handle:
        for number, line in _iter_data_lines(handle):
            values = _numbers(line)
            if len(values) < 8:
                raise TrajectoryError(f"{path}:{number} TUM 每行需要 8 个数")
            t = values[0]
            translation = (values[1], values[2], values[3])
            rotation = quat_normalize((values[7], values[4], values[5], values[6]))
            trajectory.push(t, Pose(translation, rotation))
    return trajectory


def write_tum(path: str, trajectory: Trajectory, precision: int = 9) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("# timestamp tx ty tz qx qy qz qw\n")
        for sample in trajectory:
            w, x, y, z = quat_normalize(sample.pose.rotation)
            tx, ty, tz = sample.pose.translation
            handle.write(
                " ".join(
                    f"{value:.{precision}f}"
                    for value in (sample.time, tx, ty, tz, x, y, z, w)
                )
                + "\n"
            )


def read_kitti(path: str, period_s: float = 0.1, times_path: str | None = None) -> Trajectory:
    times: list[float] = []
    if times_path:
        with open(times_path, "r", encoding="utf-8") as handle:
            times = [_numbers(line)[0] for _, line in _iter_data_lines(handle)]

    trajectory = Trajectory(name=path)
    with open(path, "r", encoding="utf-8") as handle:
        for index, (number, line) in enumerate(_iter_data_lines(handle)):
            values = _numbers(line)
            if len(values) < 12:
                raise TrajectoryError(f"{path}:{number} KITTI 每行需要 12 个数")
            matrix = (
                (values[0], values[1], values[2]),
                (values[4], values[5], values[6]),
                (values[8], values[9], values[10]),
            )
            translation = (values[3], values[7], values[11])
            if times:
                if index >= len(times):
                    raise TrajectoryError("KITTI 时刻文件比位姿少")
                t = times[index]
            else:
                t = index * period_s
            trajectory.push(t, Pose(translation, matrix_to_quat(matrix)))
    return trajectory


def write_kitti(path: str, trajectory: Trajectory, precision: int = 9) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        for sample in trajectory:
            m = sample.pose.matrix
            t = sample.pose.translation
            values: list[float] = []
            for row in range(3):
                values.extend([m[row][0], m[row][1], m[row][2], t[row]])
            handle.write(" ".join(f"{v:.{precision}f}" for v in values) + "\n")


def detect_format(path: str) -> str:
    with open(path, "r", encoding="utf-8") as handle:
        for _, line in _iter_data_lines(handle):
            count = len(line.split())
            if count >= 12:
                return "kitti"
            if count >= 8:
                return "tum"
            raise TrajectoryError(f"无法判定轨迹格式: {path}")
    raise TrajectoryError(f"轨迹文件是空的: {path}")


def read_trajectory(path: str, fmt: str | None = None, **kwargs) -> Trajectory:
    fmt = fmt or detect_format(path)
    if fmt == "tum":
        return read_tum(path)
    if fmt == "kitti":
        return read_kitti(path, **kwargs)
    raise TrajectoryError(f"未知轨迹格式 {fmt}")


def write_trajectory(path: str, trajectory: Trajectory, fmt: str = "tum", **kwargs) -> None:
    if fmt == "tum":
        write_tum(path, trajectory, **kwargs)
    elif fmt == "kitti":
        write_kitti(path, trajectory, **kwargs)
    else:
        raise TrajectoryError(f"未知轨迹格式 {fmt}")


def associate(
    estimate: Trajectory, reference: Trajectory, tolerance: float = 0.02
) -> list[tuple[int, int]]:
    """时间关联：对估计的每个样本找参考里最近且不超过容差的那个。"""
    pairs: list[tuple[int, int]] = []
    for index, sample in enumerate(estimate):
        match = reference.nearest(sample.time, tolerance)
        if match is not None:
            pairs.append((index, match))
    return pairs


def poses_from(trajectory: Trajectory, indices: Sequence[int]) -> list[Pose]:
    return [trajectory[i].pose for i in indices]
