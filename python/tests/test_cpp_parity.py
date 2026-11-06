"""Python 与 C++ 两份实现的逐字节 / 逐数字比对。

任务配置的规范化导出与哈希（含插件节点的还原）、轨迹指标，在两边各有一份实现。
重复是刻意的（扫参时 Python 侧不该为每个变体起一次子进程），但只在能自动比对的
前提下成立——这个文件就是那个前提。两边一旦漂移，这里立刻红。

配置比对要一个登记了里程计的入口：内置目录里没有里程计（算法由用户写），
simpleslam-config 还原不了任何完整的任务。示例入口 simpleslam-lio-skeleton 登记了
my_lio，就用它。需要先构建：
    cmake --build <build> --target simpleslam_lio_skeleton simpleslam_eval_tool
然后
    SIMPLESLAM_BUILD_DIR=<build> python3 -m unittest discover -s python/tests
找不到工具时整组跳过（而不是假装通过）。
"""

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from simpleslam.config import Config, ConfigError
from simpleslam.metrics import absolute_trajectory_error, flatten, relative_pose_error
from simpleslam.schema import Schema
from simpleslam.trajectory import Pose, Trajectory, write_trajectory

REPO = Path(__file__).resolve().parents[2]
LIO = "system.modules.odometry.plugin"
SOURCE = "data.source"


def find_tool(name: str, env: str, subdir: str = "tools") -> str | None:
    from_env = os.environ.get(env)
    if from_env and Path(from_env).exists():
        return from_env
    build_dir = os.environ.get("SIMPLESLAM_BUILD_DIR")
    if build_dir:
        candidate = Path(build_dir) / subdir / name
        if candidate.exists():
            return str(candidate)
    return shutil.which(name)


TASK_TOOL = find_tool("simpleslam-lio-skeleton", "SIMPLESLAM_TASK_TOOL", "examples")
EVAL_TOOL = find_tool("simpleslam-eval", "SIMPLESLAM_EVAL_TOOL")


def run(tool: str, *args: str) -> str:
    result = subprocess.run([tool, *args], capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise AssertionError(f"{tool} {' '.join(args)} 退出码 {result.returncode}: {result.stderr}")
    return result.stdout


def run_python(*args: str) -> str:
    """跑 python -m simpleslam 或仓库里的脚本，包从源码树里取。"""
    env = {**os.environ, "PYTHONPATH": str(REPO / "python")}
    result = subprocess.run([sys.executable, *args], capture_output=True, text=True,
                            check=False, env=env)
    if result.returncode != 0:
        raise AssertionError(f"python {' '.join(args)} 退出码 {result.returncode}: {result.stderr}")
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


@unittest.skipIf(TASK_TOOL is None,
                 "没有登记了里程计的入口（simpleslam-lio-skeleton），跳过配置比对")
class ConfigParityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.schema = Schema.from_json(run(TASK_TOOL, "schema"))

    @staticmethod
    def base(**sections) -> Config:
        config = Config({LIO: {"type": "my_lio"},
                         SOURCE: {"type": "kitti_odometry", "path": "/nonexistent/kitti"}})
        return config.update(**sections)

    def assertSameAsCpp(self, config: Config, label: str) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.yaml"
            path.write_text(config.to_yaml(self.schema), encoding="utf-8")
            self.assertEqual(config.canonical(self.schema), run(TASK_TOOL, "dump", str(path)),
                             f"{label}：规范化导出不一致")
            check = run(TASK_TOOL, "check", str(path))
            self.assertIn(f"任务哈希: {config.hash(self.schema)}\n", check, f"{label}：任务哈希")
            self.assertIn(f"系统哈希: {config.system_hash(self.schema)}\n", check,
                          f"{label}：系统哈希")

    def test_schema_lists_fields_slots_and_plugins(self):
        self.assertGreater(len(self.schema), 30)
        self.assertIn("system.modules.graph_optimizer.odometry_translation_weight", self.schema)
        self.assertEqual(self.schema.slot_interface(LIO), "OdometryBase")
        self.assertEqual(self.schema.slot_interface(SOURCE), "DatasetReader")
        self.assertIn("my_lio", self.schema.implementations("OdometryBase"))
        self.assertEqual(self.schema.implementations("DatasetReader"),
                         ["kitti_odometry", "kitti_raw", "replay"])

    def test_canonical_and_hashes_match(self):
        nasty = 'a # b: "c" \\ d\n\te'
        tuned = self.base()
        tuned[LIO].update(keyframe_distance_m=1 / 3, keyframe_rotation_deg=20)
        tuned[SOURCE].update(sequence="00", skip_scans=5, groundtruth_frame="camera")
        raw = self.base(data={"calibration": nasty, "sequences": ["00", "true", "1.5"]})
        raw[SOURCE] = {"type": "kitti_raw", "path": nasty, "repair_scan_timing": True}
        cases = {
            "最小任务": self.base(),
            "任务节": self.base(task={"name": "demo", "seed": 17, "mode": "online"}),
            "系统节": self.base(system={
                "pipeline": "mapping",
                "runtime": {"spin_budget": 1024, "online_max_in_flight": 0},
                "packetizer": {"source_policy": "config", "imu_required": False,
                               "post_support_s": 0.02},
                "modules": {
                    "place_recognizer": {"plugin": {"type": "none"}},
                    "graph_optimizer": {"plugin": {"type": "none"},
                                        "odometry_translation_weight": 1 / 3,
                                        "odometry_rotation_weight": 7},
                },
            }),
            "插件参数": tuned,
            "难缠的字符串": raw,
            "评测与产出": self.base(
                task={"name": nasty},
                evaluation={"evaluators": {"ate": {"type": "ate", "alignment": "sim3",
                                                   "association": "interpolate"},
                                           "rpe_1s": {"type": "rpe", "delta_s": 1.5}},
                            "baselines": "b.yaml", "allow_incomplete": True},
                outputs={"dir": "/tmp/elsewhere", "record": True, "kitti_trajectories": True},
            ),
        }
        # 评测函数整张表写与一项一项写是同一张表，后写的覆盖先写的
        by_entry = self.base()
        by_entry["evaluation.evaluators"] = {"ate": {"type": "ate"}, "z9": {"type": "ate"}}
        by_entry["evaluation.evaluators.ate"] = {"type": "ate", "alignment": "none"}
        by_entry["evaluation.evaluators.rpe"] = {"type": "rpe"}
        cases["评测函数按项写"] = by_entry
        for label, config in cases.items():
            self.assertSameAsCpp(config, label)
        resolved = by_entry.resolved(self.schema)["evaluation.evaluators"]
        self.assertEqual(sorted(resolved), ["ate", "rpe", "z9"])
        self.assertEqual(resolved["ate"]["alignment"], "none")

    def test_evaluator_names_follow_cpp_rules(self):
        for name in ("ate.v2", "ate-2", ""):
            config = self.base(evaluation={"evaluators": {name: {"type": "ate"}}})
            with self.assertRaises(ConfigError, msg=name):
                config.canonical(self.schema)
        with self.assertRaises(ConfigError):
            self.base(evaluation={"evaluators": {"ate": {"alignment": "se3"}}}).canonical(
                self.schema)  # 没写 type

    def test_outputs_do_not_change_the_task_hash(self):
        # 产出放在哪里、怎么评测都不影响运行：任务哈希不变
        a = self.base()
        b = self.base(outputs={"dir": "/tmp/elsewhere"},
                      evaluation={"evaluators": {"ate": {"type": "ate", "alignment": "sim3"}}})
        self.assertEqual(a.hash(self.schema), b.hash(self.schema))
        with tempfile.TemporaryDirectory() as directory:
            hashes = []
            for index, config in enumerate((a, b)):
                path = Path(directory) / f"task_{index}.yaml"
                path.write_text(config.to_yaml(self.schema), encoding="utf-8")
                hashes.append(run(TASK_TOOL, "hash", str(path)).strip())
        self.assertEqual(hashes, [a.hash(self.schema)] * 2)

    def test_repo_tasks_load_through_cpp(self):
        for name in ("kitti_odometry.yaml", "kitti_raw.yaml"):
            path = REPO / "configs" / "tasks" / name
            loaded = Config.load(path, TASK_TOOL, self.schema)
            self.assertEqual(loaded.canonical(self.schema), run(TASK_TOOL, "dump", str(path)), name)
            self.assertEqual(loaded.hash(self.schema), run(TASK_TOOL, "hash", str(path)).strip())

    def test_system_file_and_overrides_go_through_cpp(self):
        config = Config(task={"name": "py"}, system=str(REPO / "configs/systems/my_lio.yaml"),
                        data={"source": {"type": "kitti_odometry", "path": "/nonexistent"}})
        config.override(f"{LIO}.keyframe_distance_m", 2.5)
        config.override(f"{SOURCE}.sequence", "07")
        complete = config.through_cpp(tool=TASK_TOOL, schema=self.schema)
        self.assertEqual(complete[LIO]["keyframe_distance_m"], 2.5)
        self.assertEqual(complete[SOURCE]["sequence"], "07")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.yaml"
            path.write_text(config.to_yaml(self.schema), encoding="utf-8")
            self.assertEqual(complete.hash(self.schema), run(TASK_TOOL, "hash", str(path)).strip())

    def test_repo_python_example(self):
        example = str(REPO / "configs" / "tasks" / "example.py")
        with tempfile.TemporaryDirectory() as directory:
            rendered = Path(directory) / "nested" / "example.yaml"
            rendered.parent.mkdir()
            run_python("-m", "simpleslam", "--tool", TASK_TOOL, "config", "render", example,
                       "-o", str(rendered))
            # 渲染出的文件挪了位置，引用的系统文件仍然找得到
            cpp_hash = run(TASK_TOOL, "hash", str(rendered)).strip()
            check = run_python("-m", "simpleslam", "--tool", TASK_TOOL, "config", "check", example)
            self.assertIn(f"任务哈希: {cpp_hash}\n", check)

            # 扫参的变体：每个都能加载，改的是系统，系统哈希各不相同
            listed = run_python(example, str(Path(directory) / "sweep")).split()
            self.assertEqual(len(listed), 3)
            system_hashes = set()
            for path in listed:
                check = run(TASK_TOOL, "check", path)
                system_hashes.add(check.split("系统哈希: ")[1].strip())
            self.assertEqual(len(system_hashes), 3)


def metrics_of(result: dict) -> dict[str, float]:
    """结论 result.json 里的指标：全名是 ``<评测函数的名字>.<量>``，样本数是 ``<名字>.samples``，
    与门禁用的指标同名。"""
    metrics: dict[str, float] = {}
    for name, evaluation in result["evaluations"].items():
        metrics[f"{name}.samples"] = float(evaluation["samples"])
        for scalar, entry in evaluation["scalars"].items():
            metrics[f"{name}.{scalar}"] = float(entry["value"])
    return metrics


@unittest.skipIf(EVAL_TOOL is None, "没有 simpleslam-eval，跳过指标比对")
class MetricsParityTest(unittest.TestCase):
    def run_case(self, estimate: Trajectory, reference: Trajectory, align: str,
                 expect_exit: int = 0, rpe_distance: float | None = None):
        with tempfile.TemporaryDirectory() as directory:
            est_path = str(Path(directory) / "est.tum")
            ref_path = str(Path(directory) / "ref.tum")
            out = Path(directory) / "evaluated"
            # 写文件时精度有限，所以两边读的是同一份文件，比的是同一组输入
            write_trajectory(est_path, estimate, "tum", precision=12)
            write_trajectory(ref_path, reference, "tum", precision=12)
            rpe_args = (["--rpe-distance", str(rpe_distance)] if rpe_distance is not None
                        else ["--rpe-delta", "0.5"])
            completed = subprocess.run(
                [
                    EVAL_TOOL, est_path, ref_path,
                    "--out", str(out),
                    "--align", align,
                    "--tolerance", "0.02",
                    *rpe_args,
                ],
                capture_output=True, text=True, check=False,
            )
            self.assertEqual(completed.returncode, expect_exit, completed.stderr)
            # 用结论里的结构化数据，不解析给人看的输出
            cpp = metrics_of(json.loads((out / "result.json").read_text(encoding="utf-8")))

            from simpleslam.trajectory import read_trajectory

            est = read_trajectory(est_path, "tum")
            ref = read_trajectory(ref_path, "tum")
            ate = absolute_trajectory_error(est, ref, align, 0.02)
            if rpe_distance is not None:
                rpe = relative_pose_error(est, ref, delta_distance=rpe_distance, tolerance=0.02)
            else:
                rpe = relative_pose_error(est, ref, delta_time=0.5, tolerance=0.02)
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
             "ate.rotation.rmse", "ate.samples", "ate.coverage",
             "rpe.translation.rmse", "rpe.rotation.rmse", "rpe.samples"],
        )

    def test_se3_alignment(self):
        truth = curved(30)
        offset = Pose((3.0, -1.0, 0.5), (math.cos(0.15), 0.0, 0.0, math.sin(0.15)))
        cpp, python = self.run_case(curved(30, transform=offset), truth, "se3")
        self.assertMetricsClose(
            cpp, python,
            ["ate.translation.rmse", "ate.rotation.rmse", "ate.scale", "ate.samples"],
        )

    def test_sim3_alignment(self):
        truth = curved(30)
        cpp, python = self.run_case(curved(30, scale=1.7), truth, "sim3")
        self.assertMetricsClose(
            cpp, python, ["ate.translation.rmse", "ate.rotation.rmse", "ate.scale"]
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
            ["ate.reference_coverage", "ate.matched_reference_samples", "ate.samples",
             "ate.max_reference_gap"],
        )

    def test_zero_association_is_not_a_result(self):
        # 一对都没关联上：两边都不能把它显示成零误差，C++ 门禁判无效（退出码 4）
        estimate = curved(10)
        reference = Trajectory()
        for i in range(10):
            reference.push(100.0 + i, Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
        cpp, python = self.run_case(estimate, reference, "none", expect_exit=4)
        self.assertEqual(cpp["ate.samples"], 0.0)
        self.assertEqual(python["ate.samples"], 0.0)

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
        self.assertMetricsClose(cpp, python, ["rpe.translation.rmse", "rpe.samples"])

    def test_short_trajectory_has_no_rpe(self):
        # 不足一个 RPE 间隔：两边都给出零段，而不是凑出一个数
        truth = curved(4)
        cpp, python = self.run_case(curved(4), truth, "none")
        self.assertEqual(cpp["rpe.samples"], 0.0)
        self.assertEqual(python["rpe.samples"], 0.0)


if __name__ == "__main__":
    unittest.main()
