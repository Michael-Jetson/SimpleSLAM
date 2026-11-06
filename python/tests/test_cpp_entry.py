"""Python 写的任务经 C++ 入口加载与导出：渲染出的 YAML C++ 读得懂，读到的就是 Python 写的。

还原插件节点、补默认值、读系统文件、应用覆盖项、换算旧写法都只在 C++ 一处，Python 不留
第二份。这里核对两边接得上：schema 从入口导出，任务经入口的 ``dump`` 展开。

要一个登记了里程计的入口：内置目录里没有里程计（算法由用户写），示例入口
simpleslam-lio-skeleton 登记了 my_lio，就用它。需要先构建：
    cmake --build <build> --target simpleslam_lio_skeleton
然后
    SIMPLESLAM_BUILD_DIR=<build> python3 -m unittest discover -s python/tests
找不到入口时整组跳过（而不是假装通过）。
"""

import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any

from simpleslam.config import Config, ConfigError
from simpleslam.schema import Schema

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


def lookup(config: Config, path: str) -> Any:
    """C++ 读到的那一项。插件节点表整张是一个值，表里的一项（``<表>.<名字>``）到表里取。"""
    if path in config:
        return config[path]
    head, _, name = path.rpartition(".")
    table = config.get(head)
    return table.get(name) if isinstance(table, dict) else None


def contained(written: Any, loaded: Any) -> bool:
    """Python 写下的每一项在 C++ 读到的里面都在、值相同；C++ 补上的默认值不管。"""
    if isinstance(written, dict):
        return isinstance(loaded, dict) and all(
            key in loaded and contained(value, loaded[key]) for key, value in written.items())
    if isinstance(written, float) or isinstance(loaded, float):
        return isinstance(loaded, (int, float)) and float(written) == float(loaded)
    return written == loaded


@unittest.skipIf(TASK_TOOL is None,
                 "没有登记了里程计的入口（simpleslam-lio-skeleton），跳过")
class ThroughCppTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.schema = Schema.from_json(run(TASK_TOOL, "schema"))

    @staticmethod
    def base(**sections) -> Config:
        config = Config({LIO: {"type": "my_lio"},
                         SOURCE: {"type": "kitti_odometry", "path": "/nonexistent/kitti"}})
        return config.update(**sections)

    def assertLoadsAsWritten(self, config: Config, label: str) -> None:
        complete = config.through_cpp(tool=TASK_TOOL, schema=self.schema)
        for path, value in config.as_dict().items():
            loaded = lookup(complete, path)
            self.assertTrue(contained(value, loaded),
                            f"{label}：{path} 写的是 {value!r}，C++ 读到 {loaded!r}")

    def test_schema_lists_fields_slots_and_plugins(self):
        self.assertGreater(len(self.schema), 20)
        self.assertIn("system.modules.graph_optimizer.odometry_translation_weight", self.schema)
        self.assertEqual(self.schema.slots[LIO], "OdometryBase")
        self.assertEqual(self.schema.slots[SOURCE], "DatasetReader")
        self.assertIn("my_lio", self.schema.implementations("OdometryBase"))
        self.assertEqual(self.schema.implementations("DatasetReader"),
                         ["kitti_odometry", "kitti_raw"])
        self.assertIn("task.seed", self.schema.legacy)  # 去掉的键也在换算表里，Python 认得

    def test_rendered_tasks_load_as_written(self):
        nasty = 'a # b: "c" \\ d\n\te'
        tuned = self.base()
        tuned[LIO].update(keyframe_distance_m=1 / 3, keyframe_rotation_deg=20)
        tuned[SOURCE].update(sequence="00", skip_scans=5, groundtruth_frame="camera")
        raw = self.base(data={"calibration": nasty, "sequences": ["00", "true", "1.5"]})
        raw[SOURCE] = {"type": "kitti_raw", "path": nasty, "repair_scan_timing": True}
        cases = {
            "最小任务": self.base(),
            "任务节": self.base(task={"name": "demo", "mode": "online"}),
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
                                           "rpe_1s": {"type": "rpe", "delta_s": 1.5}}},
                outputs={"dir": "/tmp/elsewhere", "kitti_trajectories": True},
            ),
        }
        for label, config in cases.items():
            self.assertLoadsAsWritten(config, label)
        # 评测函数整张表写与一项一项写是同一张表，后写的覆盖先写的
        by_entry = self.base()
        by_entry["evaluation.evaluators"] = {"ate": {"type": "ate"}, "z9": {"type": "ate"}}
        by_entry["evaluation.evaluators.ate"] = {"type": "ate", "alignment": "none"}
        by_entry["evaluation.evaluators.rpe"] = {"type": "rpe"}
        evaluators = by_entry.through_cpp(tool=TASK_TOOL,
                                          schema=self.schema)["evaluation.evaluators"]
        self.assertEqual(sorted(evaluators), ["ate", "rpe", "z9"])
        self.assertEqual(evaluators["ate"]["alignment"], "none")

    def test_old_spelling_goes_through_cpp_to_the_same_task(self):
        # 旧写法交给 C++ 换算：与新写法得到同一份完整的任务
        old = self.base(task={"name": "py", "purpose": "experiment"},
                        evaluation={"alignment": "sim3", "rpe_delta_s": 1.0})
        new = self.base(task={"name": "py"}, evaluation={"evaluators": {
            "ate": {"type": "ate", "alignment": "sim3"}, "rpe": {"type": "rpe", "delta_s": 1.0}}})
        self.assertEqual(old.legacy_paths(self.schema),
                         ["evaluation.alignment", "evaluation.rpe_delta_s", "task.purpose"])
        self.assertEqual(old.dump(tool=TASK_TOOL, schema=self.schema),
                         new.dump(tool=TASK_TOOL, schema=self.schema))

    def test_evaluator_names_and_types_are_checked(self):
        for name in ("ate.v2", "ate-2", ""):
            config = self.base(evaluation={"evaluators": {name: {"type": "ate"}}})
            with self.assertRaises(ConfigError, msg=name):
                config.validate(self.schema)  # 名字进指标名与文件名：Python 先查
        with self.assertRaises(ConfigError):  # 没写 type：C++ 加载时报
            self.base(evaluation={"evaluators": {"ate": {"alignment": "se3"}}}).through_cpp(
                tool=TASK_TOOL, schema=self.schema)

    def test_repo_tasks_load_through_cpp(self):
        for name in ("kitti_odometry.yaml", "kitti_raw.yaml"):
            path = REPO / "configs" / "tasks" / name
            loaded = Config.load(path, TASK_TOOL, self.schema)
            # 读回来的就是完整的任务：再交给 C++ 导出一遍，逐字节不变
            self.assertEqual(loaded.dump(tool=TASK_TOOL, schema=self.schema),
                             run(TASK_TOOL, "dump", str(path)), name)

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
            self.assertEqual(config.dump(tool=TASK_TOOL, schema=self.schema),
                             run(TASK_TOOL, "dump", str(path)))

    def test_repo_python_example(self):
        example = str(REPO / "configs" / "tasks" / "example.py")
        with tempfile.TemporaryDirectory() as directory:
            rendered = Path(directory) / "nested" / "example.yaml"
            rendered.parent.mkdir()
            run_python("-m", "simpleslam", "config", "render", example, "-o", str(rendered),
                       "--entry", TASK_TOOL)
            # 渲染出的文件挪了位置，引用的系统文件仍然找得到；完整的任务由 C++ 导出
            cpp_dump = run(TASK_TOOL, "dump", str(rendered))
            full = run_python("-m", "simpleslam", "config", "render", example, "--full",
                              "--entry", TASK_TOOL)
            self.assertEqual(full, cpp_dump)
            check = run_python("-m", "simpleslam", "config", "check", example,
                               "--entry", TASK_TOOL)
            self.assertIn("任务有效", check)
            # 旧的 --tool 写法照样能用
            old = run_python("-m", "simpleslam", "--tool", TASK_TOOL, "config", "check", example)
            self.assertIn("任务有效", old)

            # 扫参的变体：每个都能加载，改的是系统，展开后的系统各不相同
            listed = run_python(example, str(Path(directory) / "sweep")).split()
            self.assertEqual(len(listed), 3)
            systems = {run(TASK_TOOL, "dump", path).split("\nsystem:\n")[1].split("\ndata:")[0]
                       for path in listed}
            self.assertEqual(len(systems), 3)


if __name__ == "__main__":
    unittest.main()
