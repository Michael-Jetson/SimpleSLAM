"""Python 与 C++ 两份实现的逐字节 / 逐数字比对。

配置的规范化导出与哈希、轨迹指标，在两边各有一份实现。重复是刻意的
（Python 侧不该为了算个哈希去起子进程），但只在能自动比对的前提下成立——
这个文件就是那个前提。两边一旦漂移，这里立刻红。

需要先构建工具：
    cmake --build <build> --target simpleslam_config_tool simpleslam_eval_tool
然后
    SIMPLESLAM_BUILD_DIR=<build> python3 -m unittest discover -s python/tests
找不到工具时整组跳过（而不是假装通过）。
"""

import math
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from simpleslam.config import Config
from simpleslam.metrics import absolute_trajectory_error, relative_pose_error
from simpleslam.schema import Schema
from simpleslam.trajectory import Pose, Trajectory, write_trajectory


def find_tool(name: str, env: str) -> str | None:
    from_env = os.environ.get(env)
    if from_env and Path(from_env).exists():
        return from_env
    build_dir = os.environ.get("SIMPLESLAM_BUILD_DIR")
    if build_dir:
        candidate = Path(build_dir) / "tools" / name
        if candidate.exists():
            return str(candidate)
    return shutil.which(name)


CONFIG_TOOL = find_tool("simpleslam-config", "SIMPLESLAM_CONFIG_TOOL")
EVAL_TOOL = find_tool("simpleslam-eval", "SIMPLESLAM_EVAL_TOOL")


def run(tool: str, *args: str) -> str:
    result = subprocess.run([tool, *args], capture_output=True, text=True, check=True)
    return result.stdout


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


@unittest.skipIf(CONFIG_TOOL is None, "没有 simpleslam-config，跳过配置比对")
class ConfigParityTest(unittest.TestCase):
    def setUp(self) -> None:
        self.schema = Schema.from_json(run(CONFIG_TOOL, "schema"))

    def test_schema_is_not_empty(self):
        self.assertGreater(len(self.schema), 20)
        self.assertIn("odometry.max_iterations", self.schema)

    def test_default_dump_matches_byte_for_byte(self):
        self.assertEqual(Config().canonical(self.schema), run(CONFIG_TOOL, "default"))

    def test_canonical_and_hash_match(self):
        cases = [
            Config(),
            Config(name="demo"),
            Config(name="demo", seed=17, **{"odometry.max_iterations": 40}),
            Config(**{
                "packetizer.imu_required": False,
                "packetizer.post_support_s": 0.02,
                "backend.loop_score_threshold": 0.75,
                "odometry.voxel_size_m": 1.0 / 3.0,
            }),
            Config(**{"evaluation.output": "/tmp/est.tum", "runtime.worker_threads": 4}),
        ]
        with tempfile.TemporaryDirectory() as directory:
            for index, config in enumerate(cases):
                path = Path(directory) / f"config_{index}.yaml"
                path.write_text(config.to_yaml(self.schema), encoding="utf-8")
                self.assertEqual(
                    config.canonical(self.schema),
                    run(CONFIG_TOOL, "dump", str(path)),
                    f"第 {index} 个配置的规范化导出不一致",
                )
                self.assertEqual(
                    config.hash(self.schema),
                    run(CONFIG_TOOL, "hash", str(path)).strip(),
                    f"第 {index} 个配置的哈希不一致",
                )


@unittest.skipIf(EVAL_TOOL is None, "没有 simpleslam-eval，跳过指标比对")
class MetricsParityTest(unittest.TestCase):
    def parse(self, output: str) -> dict[str, float]:
        """从报告 YAML 里取指标。用 --report 拿结构化输出，不解析人读表格。"""
        metrics: dict[str, float] = {}
        inside = False
        for line in output.splitlines():
            if line.startswith("metrics:"):
                inside = True
                continue
            if inside:
                if not line.startswith("  "):
                    break
                key, _, value = line.strip().partition(":")
                metrics[key] = float(value)
        return metrics

    def run_case(self, estimate: Trajectory, reference: Trajectory, align: str,
                 expect_exit: int = 0, rpe_distance: float | None = None):
        with tempfile.TemporaryDirectory() as directory:
            est_path = str(Path(directory) / "est.tum")
            ref_path = str(Path(directory) / "ref.tum")
            report_path = str(Path(directory) / "report.yaml")
            # 写文件时精度有限，所以两边读的是同一份文件，比的是同一组输入
            write_trajectory(est_path, estimate, "tum", precision=12)
            write_trajectory(ref_path, reference, "tum", precision=12)
            rpe_args = (["--rpe-distance", str(rpe_distance)] if rpe_distance is not None
                        else ["--rpe-delta", "0.5"])
            completed = subprocess.run(
                [
                    EVAL_TOOL, est_path, ref_path,
                    "--align", align,
                    "--tolerance", "0.02",
                    *rpe_args,
                    "--report", report_path,
                ],
                capture_output=True, text=True, check=False,
            )
            self.assertEqual(completed.returncode, expect_exit, completed.stderr)
            cpp = self.parse(Path(report_path).read_text(encoding="utf-8"))

            from simpleslam.trajectory import read_trajectory

            est = read_trajectory(est_path, "tum")
            ref = read_trajectory(ref_path, "tum")
            ate = absolute_trajectory_error(est, ref, align, 0.02)
            if rpe_distance is not None:
                rpe = relative_pose_error(est, ref, delta_distance=rpe_distance, tolerance=0.02)
            else:
                rpe = relative_pose_error(est, ref, delta_time=0.5, tolerance=0.02)
            from simpleslam.metrics import flatten

            python = flatten(ate, rpe)
        return cpp, python

    def assertMetricsClose(self, cpp, python, keys):
        for key in keys:
            self.assertIn(key, cpp, f"C++ 报告里没有 {key}")
            self.assertAlmostEqual(
                cpp[key], python[key], places=6, msg=f"{key} 两边不一致"
            )

    def test_no_alignment(self):
        truth = curved(30)
        offset = Pose((1.0, -0.5, 0.25), (math.cos(0.1), 0.0, 0.0, math.sin(0.1)))
        cpp, python = self.run_case(curved(30, transform=offset), truth, "none")
        self.assertMetricsClose(
            cpp, python,
            ["ate.translation.rmse", "ate.translation.mean", "ate.translation.max",
             "ate.rotation_deg.rmse", "ate.pairs", "ate.coverage",
             "rpe.translation.rmse", "rpe.rotation_deg.rmse", "rpe.pairs"],
        )

    def test_se3_alignment(self):
        truth = curved(30)
        offset = Pose((3.0, -1.0, 0.5), (math.cos(0.15), 0.0, 0.0, math.sin(0.15)))
        cpp, python = self.run_case(curved(30, transform=offset), truth, "se3")
        self.assertMetricsClose(
            cpp, python,
            ["ate.translation.rmse", "ate.rotation_deg.rmse", "ate.scale", "ate.pairs"],
        )

    def test_sim3_alignment(self):
        truth = curved(30)
        cpp, python = self.run_case(curved(30, scale=1.7), truth, "sim3")
        self.assertMetricsClose(
            cpp, python, ["ate.translation.rmse", "ate.rotation_deg.rmse", "ate.scale"]
        )

    def test_metric_sets_are_identical(self):
        # 两边支持的指标集合本身也要一致：只比挑出来的几个键，新增的指标漂移了也看不见
        truth = curved(30)
        cpp, python = self.run_case(curved(30), truth, "se3")
        self.assertEqual(set(cpp), set(python))
        self.assertMetricsClose(cpp, python, sorted(cpp))

    def test_many_to_one_coverage(self):
        # 11 个估计挤在 2 个参考样本上：覆盖率按不同参考样本计，不超过 1
        estimate = Trajectory()
        for i in range(11):
            estimate.push(i * 0.001, Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
        reference = Trajectory()
        reference.push(0.0, Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
        reference.push(0.010, Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
        cpp, python = self.run_case(estimate, reference, "none")
        self.assertAlmostEqual(python["ate.reference_coverage"], 1.0)
        self.assertEqual(python["ate.matched_reference_samples"], 2.0)
        self.assertMetricsClose(
            cpp, python,
            ["ate.reference_coverage", "ate.matched_reference_samples", "ate.pairs",
             "ate.max_reference_gap_s"],
        )

    def test_zero_association_is_not_a_result(self):
        # 一对都没关联上：两边都不能把它显示成零误差，C++ 门禁判无效（退出码 4）
        estimate = curved(10)
        reference = Trajectory()
        for i in range(10):
            reference.push(100.0 + i, Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
        cpp, python = self.run_case(estimate, reference, "none", expect_exit=4)
        self.assertEqual(cpp["ate.pairs"], 0.0)
        self.assertEqual(python["ate.pairs"], 0.0)

    def test_rpe_distance_follows_the_reference_path(self):
        # 稀疏估计配密集真值：走过的路沿真值自身的采样量（20 米），不是两次估计之间的弦长
        identity = (1.0, 0.0, 0.0, 0.0)
        reference = Trajectory()
        reference.push(0.0, Pose((0.0, 0.0, 0.0), identity))
        reference.push(1.0, Pose((0.0, 10.0, 0.0), identity))
        reference.push(2.0, Pose((10.0, 10.0, 0.0), identity))
        estimate = Trajectory()
        estimate.push(0.0, Pose((0.0, 0.0, 0.0), identity))
        estimate.push(2.0, Pose((11.0, 10.0, 0.0), identity))
        cpp, python = self.run_case(estimate, reference, "none", rpe_distance=10.0)
        self.assertAlmostEqual(python["rpe.translation.rmse"], 1.0 / 20.0)
        self.assertMetricsClose(cpp, python, ["rpe.translation.rmse", "rpe.pairs"])

    def test_short_trajectory_has_no_rpe(self):
        # 不足一个 RPE 间隔：两边都给出零段，而不是凑出一个数
        truth = curved(4)
        cpp, python = self.run_case(curved(4), truth, "none")
        self.assertEqual(cpp["rpe.pairs"], 0.0)
        self.assertEqual(python["rpe.pairs"], 0.0)


if __name__ == "__main__":
    unittest.main()
