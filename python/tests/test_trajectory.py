"""Python 侧轨迹：格式往返、时间递增、四元数与矩阵互转。"""

import math
import tempfile
import unittest
from pathlib import Path

from simpleslam.trajectory import (
    Pose,
    Trajectory,
    TrajectoryError,
    detect_format,
    matrix_to_quat,
    quat_to_matrix,
    read_trajectory,
    write_trajectory,
)


def spiral(count: int) -> Trajectory:
    trajectory = Trajectory()
    for i in range(count):
        theta = i * 0.2
        trajectory.push(
            i * 0.1,
            Pose(
                (5.0 * math.cos(theta), 5.0 * math.sin(theta), 0.1 * i),
                (math.cos(theta / 2), 0.0, 0.0, math.sin(theta / 2)),
            ),
        )
    return trajectory


class TrajectoryTest(unittest.TestCase):
    def test_time_must_increase(self):
        trajectory = Trajectory()
        trajectory.push(1.0, Pose())
        with self.assertRaises(TrajectoryError):
            trajectory.push(1.0, Pose())

    def test_quaternion_matrix_roundtrip(self):
        for angle in (0.0, 0.3, 1.9, 3.0):
            q = (math.cos(angle / 2), 0.0, math.sin(angle / 2), 0.0)
            back = matrix_to_quat(quat_to_matrix(q))
            # 四元数与其相反数表示同一旋转
            dot = abs(sum(a * b for a, b in zip(q, back)))
            self.assertAlmostEqual(dot, 1.0, places=12)

    def test_tum_roundtrip(self):
        original = spiral(10)
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "traj.tum")
            write_trajectory(path, original, "tum")
            self.assertEqual(detect_format(path), "tum")
            loaded = read_trajectory(path)
            self.assertEqual(len(loaded), len(original))
            for a, b in zip(original, loaded):
                self.assertAlmostEqual(a.time, b.time, places=6)
                self.assertAlmostEqual(
                    math.dist(a.pose.translation, b.pose.translation), 0.0, places=7
                )

    def test_kitti_roundtrip(self):
        original = spiral(8)
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "traj.kitti")
            write_trajectory(path, original, "kitti")
            self.assertEqual(detect_format(path), "kitti")
            loaded = read_trajectory(path, "kitti", period_s=0.1)
            self.assertEqual(len(loaded), len(original))
            for a, b in zip(original, loaded):
                self.assertAlmostEqual(
                    math.dist(a.pose.translation, b.pose.translation), 0.0, places=7
                )

    def test_pose_algebra(self):
        a = Pose((1.0, 2.0, 3.0), (math.cos(0.4), 0.0, 0.0, math.sin(0.4)))
        identity = a.compose(a.inverse())
        self.assertAlmostEqual(math.dist(identity.translation, (0.0, 0.0, 0.0)), 0.0, places=12)
        self.assertAlmostEqual(abs(identity.rotation[0]), 1.0, places=12)

    def test_path_length(self):
        trajectory = Trajectory()
        for i in range(11):
            trajectory.push(i * 0.1, Pose((i * 0.5, 0.0, 0.0)))
        self.assertAlmostEqual(trajectory.path_length(), 5.0, places=9)


if __name__ == "__main__":
    unittest.main()
