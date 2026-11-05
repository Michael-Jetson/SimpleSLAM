"""ATE / RPE：与 C++ 侧同一口径的纯标准库实现。

为什么这里也要有一份：写算法时想在 notebook 里看一眼、画张图、试几个对齐方式，
不该为此去装 numpy 或跑子进程。但**卡门禁用 C++ 那一份**（``simpleslam-eval``）——
两份实现的数值由 ``python/tests/test_cpp_parity.py`` 比对，允许重复的前提是能自动比。

两条纪律与 C++ 侧一致：
1. 对齐方式是结果的一部分，所以它在返回值里，不只在调用参数里；
2. 平移与旋转分开统计，不加权成一个标量。

**支持范围比 C++ 窄，而且是刻意的**：这里只有最近邻关联，没有插值关联，也不接收
评测目标窗口（没有 ``ate.target_coverage``）。协议为 ``interpolate`` 的报告不由这里复算，
严格评测与门禁一律走 C++。补一份暂时没人用的第二实现，只会多一份要维护、要比对的代码；
真需要时再加，并同时补上对照测试。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

from simpleslam.trajectory import (
    Mat3,
    Pose,
    Trajectory,
    Vec3,
    associate,
    matrix_to_quat,
)


@dataclass
class Statistics:
    count: int = 0
    rmse: float = 0.0
    mean: float = 0.0
    median: float = 0.0
    stddev: float = 0.0
    min: float = 0.0
    max: float = 0.0


def statistics(values: Sequence[float]) -> Statistics:
    stats = Statistics(count=len(values))
    if not values:
        return stats
    ordered = sorted(values)
    stats.min = ordered[0]
    stats.max = ordered[-1]
    middle = len(ordered) // 2
    stats.median = (
        ordered[middle]
        if len(ordered) % 2 == 1
        else 0.5 * (ordered[middle - 1] + ordered[middle])
    )
    stats.mean = sum(ordered) / len(ordered)
    stats.rmse = math.sqrt(sum(v * v for v in ordered) / len(ordered))
    stats.stddev = math.sqrt(
        sum((v - stats.mean) ** 2 for v in ordered) / len(ordered)
    )
    return stats


# ── 3×3 线性代数（够 Umeyama 用，不多写） ──


def _mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )  # type: ignore[return-value]


def _mat_transpose(m: Mat3) -> Mat3:
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def _mat_det(m: Mat3) -> float:
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def _mat_vec(m: Mat3, v: Vec3) -> Vec3:
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))  # type: ignore


def _jacobi_eigen(a: Mat3, iterations: int = 64) -> tuple[list[float], Mat3]:
    """对称 3×3 的循环 Jacobi 特征分解。返回 (特征值, 特征向量按列)。"""
    m = [list(row) for row in a]
    v = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    for _ in range(iterations):
        off = sum(m[i][j] ** 2 for i in range(3) for j in range(3) if i != j)
        if off < 1e-30:
            break
        for p in range(2):
            for q in range(p + 1, 3):
                if abs(m[p][q]) < 1e-300:
                    continue
                theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q])
                t = math.copysign(1.0, theta) / (abs(theta) + math.sqrt(theta * theta + 1.0))
                c = 1.0 / math.sqrt(t * t + 1.0)
                s = t * c
                for k in range(3):
                    mkp = m[k][p]
                    mkq = m[k][q]
                    m[k][p] = c * mkp - s * mkq
                    m[k][q] = s * mkp + c * mkq
                for k in range(3):
                    mpk = m[p][k]
                    mqk = m[q][k]
                    m[p][k] = c * mpk - s * mqk
                    m[q][k] = s * mpk + c * mqk
                for k in range(3):
                    vkp = v[k][p]
                    vkq = v[k][q]
                    v[k][p] = c * vkp - s * vkq
                    v[k][q] = s * vkp + c * vkq
    values = [m[i][i] for i in range(3)]
    order = sorted(range(3), key=lambda i: values[i], reverse=True)
    eigenvalues = [values[i] for i in order]
    eigenvectors = tuple(tuple(v[row][i] for i in order) for row in range(3))
    return eigenvalues, eigenvectors  # type: ignore[return-value]


def _complete_orthonormal(columns: list) -> None:
    """把 None 的列补成与已有列正交的单位向量。"""
    for index in range(3):
        if columns[index] is not None:
            continue
        for seed in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            w = list(seed)
            for other in columns:
                if other is None:
                    continue
                projection = sum(w[i] * other[i] for i in range(3))
                for i in range(3):
                    w[i] -= projection * other[i]
            norm = math.sqrt(sum(c * c for c in w))
            if norm > 1e-6:
                columns[index] = tuple(c / norm for c in w)
                break
        if columns[index] is None:  # 三列全退化：只可能是点集为单点
            columns[index] = (1.0, 0.0, 0.0) if index == 0 else (
                (0.0, 1.0, 0.0) if index == 1 else (0.0, 0.0, 1.0)
            )


@dataclass
class Alignment:
    kind: str = "none"
    transform: Pose = field(default_factory=Pose)
    scale: float = 1.0


def umeyama(source: Sequence[Vec3], target: Sequence[Vec3], with_scale: bool) -> Alignment:
    """Umeyama 最小二乘对齐：target ≈ scale · R · source + t。"""
    n = len(source)
    if n == 0 or n != len(target):
        raise ValueError("Umeyama 需要等长且非空的点集")

    mean_s = tuple(sum(p[i] for p in source) / n for i in range(3))
    mean_t = tuple(sum(p[i] for p in target) / n for i in range(3))

    cov = [[0.0] * 3 for _ in range(3)]
    variance = 0.0
    for s, t in zip(source, target):
        ds = tuple(s[i] - mean_s[i] for i in range(3))
        dt = tuple(t[i] - mean_t[i] for i in range(3))
        variance += sum(c * c for c in ds)
        for i in range(3):
            for j in range(3):
                cov[i][j] += dt[i] * ds[j]
    variance /= n
    h: Mat3 = tuple(tuple(cov[i][j] / n for j in range(3)) for i in range(3))  # type: ignore

    # SVD via eigen(HᵀH)：3×3 足够，且不引入依赖
    eigenvalues, vmat = _jacobi_eigen(_mat_mul(_mat_transpose(h), h))
    singular = [math.sqrt(max(value, 0.0)) for value in eigenvalues]

    columns: list[Vec3 | None] = []
    for index in range(3):
        v_col = tuple(vmat[row][index] for row in range(3))
        if singular[index] > 1e-12:
            hv = _mat_vec(h, v_col)  # type: ignore[arg-type]
            columns.append(tuple(c / singular[index] for c in hv))  # type: ignore
        else:
            columns.append(None)
    # 退化方向（点集共线或共面）用 Gram-Schmidt 补成正交基。
    # 这种情形下旋转本来就不唯一，任取一组正交基都给出同样小的残差；
    # 但 U 必须是正交阵，否则 R 根本不是旋转，结果会整个失真。
    _complete_orthonormal(columns)
    umat: Mat3 = tuple(  # type: ignore[assignment]
        tuple(columns[c][r] for c in range(3)) for r in range(3)  # type: ignore[index]
    )

    d = 1.0 if _mat_det(umat) * _mat_det(vmat) > 0.0 else -1.0
    smat: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, d))
    rotation = _mat_mul(_mat_mul(umat, smat), _mat_transpose(vmat))

    scale = 1.0
    if with_scale and variance > 1e-12:
        scale = (singular[0] + singular[1] + d * singular[2]) / variance

    rotated = _mat_vec(rotation, mean_s)  # type: ignore[arg-type]
    translation = tuple(mean_t[i] - scale * rotated[i] for i in range(3))
    return Alignment(
        kind="sim3" if with_scale else "se3",
        transform=Pose(translation, matrix_to_quat(rotation)),  # type: ignore[arg-type]
        scale=scale,
    )


def compute_alignment(
    estimate: Trajectory,
    reference: Trajectory,
    pairs: Sequence[tuple[int, int]],
    kind: str,
) -> Alignment:
    if kind == "none" or not pairs:
        return Alignment(kind="none")
    if kind == "first_pose":
        i, j = pairs[0]
        return Alignment(
            kind="first_pose",
            transform=reference[j].pose.compose(estimate[i].pose.inverse()),
        )
    if len(pairs) < 3:
        return compute_alignment(estimate, reference, pairs, "first_pose")
    source = [estimate[i].pose.translation for i, _ in pairs]
    target = [reference[j].pose.translation for _, j in pairs]
    return umeyama(source, target, with_scale=(kind == "sim3"))


@dataclass
class AteResult:
    translation: Statistics = field(default_factory=Statistics)
    rotation_deg: Statistics = field(default_factory=Statistics)
    alignment: Alignment = field(default_factory=Alignment)
    pairs: int = 0
    estimate_samples: int = 0
    reference_samples: int = 0
    #: 被关联到的**不同**参考样本数。关联是最近邻、允许多对一，所以它不等于 pairs。
    matched_reference_samples: int = 0
    #: 关联覆盖区间内，相邻两个被关联参考样本之间的最大间隔（秒）。
    max_reference_gap: float = 0.0

    covered_span: tuple[float, float] = (0.0, 0.0)
    reference_span: tuple[float, float] = (0.0, 0.0)

    @property
    def coverage(self) -> float:
        """关联率：已输出的估计里有多少能关联到参考。不是任务完成率。"""
        return 0.0 if not self.estimate_samples else self.pairs / self.estimate_samples

    @property
    def reference_coverage(self) -> float:
        """参考侧覆盖率，值域 [0, 1]。

        分子必须是**不同**的参考样本数：最近邻允许多个估计落到同一个参考样本上，
        用 pairs 作分子会算出大于 1 的覆盖率——11 个估计挤在 2 个参考样本上会得到 5.5。
        """
        if not self.reference_samples:
            return 0.0
        return self.matched_reference_samples / self.reference_samples

    @property
    def span_ratio(self) -> float:
        """关联覆盖的时间跨度占参考跨度的比例。提前退出的算法在这里原形毕露。"""
        total = self.reference_span[1] - self.reference_span[0]
        if total <= 0.0:
            return 0.0
        return (self.covered_span[1] - self.covered_span[0]) / total


def absolute_trajectory_error(
    estimate: Trajectory,
    reference: Trajectory,
    alignment: str = "se3",
    tolerance: float = 0.02,
) -> AteResult:
    result = AteResult(
        estimate_samples=len(estimate), reference_samples=len(reference)
    )
    pairs = associate(estimate, reference, tolerance)
    result.pairs = len(pairs)
    if len(reference):
        result.reference_span = (reference[0].time, reference[len(reference) - 1].time)
    if pairs:
        result.covered_span = (reference[pairs[0][1]].time, reference[pairs[-1][1]].time)
        previous = pairs[0][1]
        result.matched_reference_samples = 1
        for _, current in pairs[1:]:
            if current == previous:
                continue
            result.matched_reference_samples += 1
            gap = reference[current].time - reference[previous].time
            result.max_reference_gap = max(result.max_reference_gap, gap)
            previous = current
    result.alignment = compute_alignment(estimate, reference, pairs, alignment)
    if not pairs:
        return result

    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    for i, j in pairs:
        pose = estimate[i].pose
        if result.alignment.scale != 1.0:
            pose = pose.scaled(result.alignment.scale)
        aligned = result.alignment.transform.compose(pose)
        delta = reference[j].pose.inverse().compose(aligned)
        translation_errors.append(math.sqrt(sum(c * c for c in delta.translation)))
        rotation_errors.append(delta.angle_deg())

    result.translation = statistics(translation_errors)
    result.rotation_deg = statistics(rotation_errors)
    return result


@dataclass
class RpeResult:
    translation: Statistics = field(default_factory=Statistics)
    rotation_deg: Statistics = field(default_factory=Statistics)
    pairs: int = 0
    delta_kind: str = "time"
    normalized: bool = False


def relative_pose_error(
    estimate: Trajectory,
    reference: Trajectory,
    delta_time: float = 1.0,
    delta_distance: float | None = None,
    alignment: str = "none",
    tolerance: float = 0.02,
    normalize: bool | None = None,
) -> RpeResult:
    kind = "distance" if delta_distance is not None else "time"
    if normalize is None:
        normalize = kind == "distance"
    result = RpeResult(delta_kind=kind, normalized=normalize)

    pairs = associate(estimate, reference, tolerance)
    if len(pairs) < 2:
        return result
    align = compute_alignment(estimate, reference, pairs, alignment)

    def aligned_pose(index: int) -> Pose:
        pose = estimate[index].pose
        if align.scale != 1.0:
            pose = pose.scaled(align.scale)
        return pose if align.kind == "none" else align.transform.compose(pose)

    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    # 按距离取段时，走过的路沿参考轨迹自身的采样累积：估计稀疏时，相邻两对之间的参考点
    # 不能跳过——只数被关联上的那几个点，量到的是弦长，不是走过的路（与 C++ 同口径）
    cumulative = [0.0]
    for k in range(1, len(reference)):
        cumulative.append(
            cumulative[-1]
            + math.dist(reference[k - 1].pose.translation, reference[k].pose.translation)
        )
    start = 0
    while start + 1 < len(pairs):
        end = start
        travelled = 0.0
        if kind == "time":
            while (
                end + 1 < len(pairs)
                and reference[pairs[end][1]].time - reference[pairs[start][1]].time < delta_time
            ):
                end += 1
            travelled = reference[pairs[end][1]].time - reference[pairs[start][1]].time
            if end == start or travelled < delta_time:
                break
        else:
            origin = cumulative[pairs[start][1]]
            while end + 1 < len(pairs) and travelled < float(delta_distance):
                end += 1
                travelled = cumulative[pairs[end][1]] - origin
            if end == start or travelled < float(delta_distance):
                break

        est_delta = aligned_pose(pairs[start][0]).inverse().compose(aligned_pose(pairs[end][0]))
        ref_delta = (
            reference[pairs[start][1]].pose.inverse().compose(reference[pairs[end][1]].pose)
        )
        error = ref_delta.inverse().compose(est_delta)

        translation = math.sqrt(sum(c * c for c in error.translation))
        rotation = error.angle_deg()
        if normalize:
            divisor = max(travelled, 1e-9)
            translation /= divisor
            rotation /= divisor
        translation_errors.append(translation)
        rotation_errors.append(rotation)
        start = end

    result.pairs = len(translation_errors)
    result.translation = statistics(translation_errors)
    result.rotation_deg = statistics(rotation_errors)
    return result


def flatten(ate: AteResult, rpe: RpeResult) -> dict[str, float]:
    """与 C++ 的 eval::flatten 同名同键，便于两边比对与共用基线文件。"""
    return {
        "ate.translation.rmse": ate.translation.rmse,
        "ate.translation.mean": ate.translation.mean,
        "ate.translation.median": ate.translation.median,
        "ate.translation.max": ate.translation.max,
        "ate.rotation_deg.rmse": ate.rotation_deg.rmse,
        "ate.rotation_deg.mean": ate.rotation_deg.mean,
        "ate.pairs": float(ate.pairs),
        "ate.coverage": ate.coverage,
        "ate.reference_coverage": ate.reference_coverage,
        "ate.matched_reference_samples": float(ate.matched_reference_samples),
        "ate.span_ratio": ate.span_ratio,
        "ate.max_reference_gap_s": ate.max_reference_gap,
        "ate.scale": ate.alignment.scale,
        "rpe.translation.rmse": rpe.translation.rmse,
        "rpe.translation.mean": rpe.translation.mean,
        "rpe.rotation_deg.rmse": rpe.rotation_deg.rmse,
        "rpe.rotation_deg.mean": rpe.rotation_deg.mean,
        "rpe.pairs": float(rpe.pairs),
    }
