"""Python 侧指标：对齐方式改变结论、退化点集不产生垃圾旋转、RPE 只看局部。"""

import math
import unittest

from simpleslam.metrics import (
    absolute_trajectory_error,
    relative_pose_error,
    statistics,
    umeyama,
)
from simpleslam.trajectory import Pose, Trajectory


def curved(count: int, transform=None, scale: float = 1.0) -> Trajectory:
    trajectory = Trajectory()
    for i in range(count):
        theta = i * 0.2
        pose = Pose(
            (5.0 * math.cos(theta) * scale, 5.0 * math.sin(theta) * scale, 0.1 * i * scale),
            (math.cos(theta / 2), 0.0, 0.0, math.sin(theta / 2)),
        )
        trajectory.push(i * 0.1, transform.compose(pose) if transform else pose)
    return trajectory


class MetricsTest(unittest.TestCase):
    def test_identical_trajectories_have_zero_error(self):
        truth = curved(20)
        result = absolute_trajectory_error(truth, truth, "none")
        self.assertEqual(result.pairs, 20)
        self.assertAlmostEqual(result.coverage, 1.0)
        self.assertAlmostEqual(result.translation.rmse, 0.0, places=12)

    def test_alignment_changes_the_conclusion(self):
        truth = curved(20)
        offset = Pose((3.0, -1.0, 0.5), (math.cos(0.15), 0.0, 0.0, math.sin(0.15)))
        estimate = curved(20, transform=offset)

        raw = absolute_trajectory_error(estimate, truth, "none")
        self.assertGreater(raw.translation.rmse, 1.0)

        aligned = absolute_trajectory_error(estimate, truth, "se3")
        self.assertEqual(aligned.alignment.kind, "se3")
        self.assertAlmostEqual(aligned.translation.rmse, 0.0, places=9)
        self.assertAlmostEqual(aligned.rotation_deg.rmse, 0.0, places=8)

    def test_sim3_recovers_scale(self):
        truth = curved(20)
        estimate = curved(20, scale=2.0)
        result = absolute_trajectory_error(estimate, truth, "sim3")
        self.assertAlmostEqual(result.alignment.scale, 0.5, places=9)
        self.assertAlmostEqual(result.translation.rmse, 0.0, places=8)

    def test_collinear_points_still_give_a_valid_rotation(self):
        # 共线点集的旋转不唯一；但求出的 U 必须是正交阵，否则 R 根本不是旋转，
        # 平移残差会整个失真。这里只断言平移能被对齐掉。
        source = [(i * 0.5, 0.0, 0.0) for i in range(10)]
        target = [(i * 0.5 * math.cos(0.3), i * 0.5 * math.sin(0.3), 0.0) for i in range(10)]
        alignment = umeyama(source, target, with_scale=False)
        for s, t in zip(source, target):
            mapped = alignment.transform.compose(Pose(s)).translation
            self.assertAlmostEqual(math.dist(mapped, t), 0.0, places=9)

    def test_rpe_ignores_global_offset(self):
        truth = curved(40)
        offset = Pose((100.0, -50.0, 7.0), (math.cos(0.55), 0.0, 0.0, math.sin(0.55)))
        estimate = curved(40, transform=offset)
        result = relative_pose_error(estimate, truth, delta_time=0.5)
        self.assertGreater(result.pairs, 0)
        self.assertAlmostEqual(result.translation.rmse, 0.0, places=9)

    def test_time_mismatch_lowers_coverage(self):
        truth = curved(10)
        shifted = Trajectory()
        for sample in truth:
            shifted.push(sample.time + 0.06, sample.pose)
        result = absolute_trajectory_error(shifted, truth, "none", tolerance=0.01)
        self.assertEqual(result.pairs, 0)
        self.assertAlmostEqual(result.coverage, 0.0)

    def test_statistics(self):
        empty = statistics([])
        self.assertEqual(empty.count, 0)
        stats = statistics([1.0, 2.0, 3.0, 4.0])
        self.assertAlmostEqual(stats.mean, 2.5)
        self.assertAlmostEqual(stats.median, 2.5)
        self.assertAlmostEqual(stats.rmse, math.sqrt(30.0 / 4.0))


if __name__ == "__main__":
    unittest.main()
