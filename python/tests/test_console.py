"""操作台：命令行归一化、构建目录里找入口、任务记录的摘要与对比表、扫参变体、
端到端跑任务与单独评测。

不构建：要 C++ 入口的用例用 SIMPLESLAM_BUILD_DIR 下已经构建好的示例入口（--entry <路径>），
找不到就跳过。
"""

import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from simpleslam import console
from simpleslam.__main__ import _docker_argv, main, normalize
from simpleslam.config import Config, ConfigError
from simpleslam.results import RESULT_SCHEMA, ResultError, csv_table, describe, rows, table
from simpleslam.results import worst_exit_code
from simpleslam.workspace import BuildError, Workspace

REPO = Path(__file__).resolve().parents[2]


def find_entry() -> str | None:
    build_dir = os.environ.get("SIMPLESLAM_BUILD_DIR")
    if not build_dir:
        return None
    path = Path(build_dir) / "examples" / "simpleslam-lio-skeleton"
    return str(path) if path.exists() else None


ENTRY = find_entry()


def write_result(run_dir: Path, data: dict) -> Path:
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "result.json").write_text(json.dumps({"schema": RESULT_SCHEMA, **data}),
                                         encoding="utf-8")
    return run_dir


def scalar(value: float | None, unit: str = "m", label: str = "") -> dict:
    return {"value": value, "unit": unit, "label": label}


def run_result(task: str, evaluations: dict, status: str = "pass", exit_code: int = 0) -> dict:
    return {"kind": "run", "task": task, "mode": "offline", "software_version": "3.0",
            "task_hash": "a" * 16, "system_hash": "b" * 16, "complete": True,
            "completeness": "complete", "completeness_source": "本次运行", "declaration": "",
            "status": status, "detail": "", "exit_code": exit_code, "trajectories": {},
            "evaluations": evaluations, "gate": None}


class NormalizeTest(unittest.TestCase):
    def test_task_file_means_run(self):
        self.assertEqual(normalize(["configs/tasks/x.yaml", "a=1"]),
                         ["run", "configs/tasks/x.yaml", "a=1"])
        self.assertEqual(normalize(["--entry", "e", "t.py"]), ["run", "--entry", "e", "t.py"])

    def test_explicit_commands_are_kept(self):
        self.assertEqual(normalize(["show", "x.yaml"]), ["show", "x.yaml"])
        self.assertEqual(normalize(["demo"]), ["demo"])
        self.assertEqual(normalize(["--help"]), ["--help"])

    def test_docker_flags_stay_on_the_host(self):
        argv = ["run", "t.yaml", "--docker", "--mount", "/data", "--mount=/more", "a=1"]
        self.assertEqual(_docker_argv(argv), ["run", "t.yaml", "a=1"])

    def test_overrides_go_to_the_entry_and_unknown_flags_are_errors(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                main(["run", "t.yaml", "--bogus"])


class WorkspaceTest(unittest.TestCase):
    def test_artifact_comes_from_the_file_api(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Workspace(Path(directory))
            reply = workspace.build_dir / ".cmake" / "api" / "v1" / "reply"
            reply.mkdir(parents=True)
            (reply / "index-1.json").write_text(json.dumps(
                {"reply": {"codemodel-v2": {"jsonFile": "codemodel.json"}}}))
            (reply / "codemodel.json").write_text(json.dumps({"configurations": [{"targets": [
                {"name": "my_entry", "jsonFile": "t1.json"},
                {"name": "lib", "jsonFile": "t2.json"}]}]}))
            (reply / "t1.json").write_text(json.dumps(
                {"type": "EXECUTABLE", "artifacts": [{"path": "apps/my-entry"}]}))
            (reply / "t2.json").write_text(json.dumps({"type": "STATIC_LIBRARY"}))
            self.assertEqual(workspace.artifact("my_entry"),
                             (workspace.build_dir / "apps" / "my-entry").resolve())
            with self.assertRaisesRegex(BuildError, "有的是 my_entry"):
                workspace.artifact("missing")

    def test_existing_executable_is_used_as_is(self):
        workspace = Workspace(REPO)
        self.assertEqual(workspace.entry(sys.executable, build=False),
                         Path(sys.executable).resolve())

    def test_build_root_can_move_out_of_the_tree(self):
        with tempfile.TemporaryDirectory() as directory:
            old = os.environ.get("SIMPLESLAM_BUILD_ROOT")
            os.environ["SIMPLESLAM_BUILD_ROOT"] = directory
            try:
                self.assertEqual(Workspace(REPO, "Debug").build_dir, Path(directory) / "debug")
            finally:
                if old is None:
                    del os.environ["SIMPLESLAM_BUILD_ROOT"]
                else:
                    os.environ["SIMPLESLAM_BUILD_ROOT"] = old


class ResultsTest(unittest.TestCase):
    def test_rows_table_and_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            single = write_result(Path(directory) / "a", run_result("kitti-00", {"ate": {
                "type": "ate", "samples": 4, "notes": ["关联（nearest）：4 对，估计 4 个"],
                "scalars": {"translation.rmse": scalar(1.5, "m", "ATE_t"),
                            "rotation.rmse": scalar(0.25, "deg", "ATE_r"),
                            "translation.max": scalar(None)}}}))  # null：没有结果
            suite = write_result(Path(directory) / "b", {
                "kind": "sequences", "task": "suite", "mode": "online",
                "software_version": "3.0", "task_hash": "c" * 16,
                "system_hash": "d" * 16, "exit_code": 1,
                "sequences": [
                    {"name": "00", "dir": "x", "log": "y", "exit_code": 0, "status": "pass",
                     "complete": True, "problem": "",
                     "scalars": {"ate.translation.rmse": scalar(2.0, "m", "ATE_t")}},
                    {"name": "05", "dir": "x", "log": "y", "exit_code": 1, "status": "error",
                     "complete": None, "problem": "目录不存在", "scalars": {}}],
                "mean": {}})
            items = rows(single) + rows(suite)
            self.assertEqual([row.name for row in items], ["kitti-00", "suite/00", "suite/05"])
            self.assertNotIn("ate.translation.max", items[0].scalars)  # null 不当零
            # 表头取评测函数标了短名的那几项，Python 里不抄列名
            text = table(items)
            self.assertIn("ATE_t", text)
            self.assertIn("1.5000", text)
            self.assertIn("bbbbbbbb", text)  # 系统哈希的前八位
            lines = csv_table(items).splitlines()
            self.assertEqual(lines[0], "name,status,exit_code,complete,ate.translation.rmse,"
                                       "ate.rotation.rmse,system_hash")
            self.assertEqual(lines[1], "kitti-00,pass,0,true,1.5,0.25," + "b" * 16)
            self.assertEqual(lines[3], "suite/05,error,1,,,," + "d" * 16)  # 没有结果就留空
            # 按全名挑列
            picked = csv_table(items, ["ate.rotation.rmse"]).splitlines()
            self.assertEqual(picked[0], "name,status,exit_code,complete,ate.rotation.rmse,"
                                        "system_hash")
            self.assertIn("ate.rotation.rmse", table(items, metrics=["ate.rotation.rmse"]))
            self.assertIn("suite/05: 目录不存在", describe(suite))
            self.assertIn("评测说明:\n  ate: 关联（nearest）：4 对", describe(single))

    def test_same_label_is_qualified_by_the_evaluator(self):
        with tempfile.TemporaryDirectory() as directory:
            both = write_result(Path(directory) / "a", run_result("kitti-00", {
                "ate": {"samples": 4, "scalars": {"translation.rmse": scalar(1.5, "m", "ATE_t")}},
                "ate_sim3": {"samples": 4,
                             "scalars": {"translation.rmse": scalar(1.2, "m", "ATE_t")}}}))
            header = table(rows(both)).splitlines()[0]
            self.assertIn("ate:ATE_t", header)
            self.assertIn("ate_sim3:ATE_t", header)

    def test_results_written_by_v2_are_read(self):
        # 读旧写新：v2 写的第 1 版在内存里换成第 2 版的样子，能和 v3 的结果放进一张表
        common = {"mode": "offline", "purpose": "experiment", "software_version": "2.4",
                  "task_hash": "a" * 16, "system_hash": "b" * 16}
        with tempfile.TemporaryDirectory() as directory:
            old = Path(directory) / "old"
            old.mkdir()
            (old / "result.json").write_text(json.dumps({
                "schema": 1, "kind": "run", "task": "kitti-00", "complete": True, "exit_code": 0,
                "status": "pass", "detail": "", "summary": {"ate.translation.rmse": 1.5},
                "metrics": {"ate.translation.rmse": 1.5, "ate.rotation_deg.rmse": 0.25,
                            "ate.pairs": 30, "ate.max_reference_gap_s": 0.1,
                            "rpe.translation.rmse": 0.0, "rpe.pairs": 0}, **common}))
            [row] = rows(old)
            self.assertEqual(row.scalars["ate.rotation.rmse"].label, "ATE_r")
            self.assertEqual(row.scalars["ate.rotation.rmse"].unit, "deg")
            self.assertEqual(row.scalars["ate.max_reference_gap"].unit, "s")
            self.assertNotIn("rpe.translation.rmse", row.scalars)  # 一对都没关联上：不给数
            self.assertEqual(row.metrics["ate.translation.rmse"], 1.5)  # v2 的属性名留作别名
            self.assertIn("ATE_r", table(rows(old)))
            suite = Path(directory) / "suite"
            suite.mkdir()
            (suite / "result.json").write_text(json.dumps({
                "schema": 1, "kind": "sequences", "task": "s", "exit_code": 0,
                "sequences": [{"name": "00", "archive": "x", "log": "y", "exit_code": 0,
                               "status": "pass", "complete": True, "problem": "",
                               "metrics": {"ate.translation.rmse": 2.0, "ate.pairs": 10}}],
                "mean": {"ate.translation.rmse": 2.0}, **common}))
            [item] = rows(suite)
            self.assertEqual(item.scalars["ate.translation.rmse"].label, "ATE_t")
            self.assertIn("2.0000", describe(suite).splitlines()[-1])  # 平均那一行

    def test_missing_or_foreign_results_are_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ResultError, "没有 result.json"):
                rows(Path(directory))
            run_dir = Path(directory) / "new"
            run_dir.mkdir()
            (run_dir / "result.json").write_text(json.dumps({"schema": RESULT_SCHEMA + 1}))
            with self.assertRaisesRegex(ResultError, "格式版本"):
                rows(run_dir)

    def test_worst_exit_code(self):
        self.assertEqual(worst_exit_code([0, 3, 5]), 5)
        self.assertEqual(worst_exit_code([3, 4, 0]), 4)
        self.assertEqual(worst_exit_code([4, 2]), 1)  # 用法错误、被信号终止都算出错
        self.assertEqual(worst_exit_code([]), 0)


class SweepTest(unittest.TestCase):
    def write(self, directory: Path, body: str) -> Path:
        path = directory / "task.py"
        path.write_text("from simpleslam import Config\n" + body, encoding="utf-8")
        return path

    def test_variants_need_unique_directory_names(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            with self.assertRaisesRegex(ConfigError, "没有 sweep"):
                console.load_python_sweep(self.write(base, "config = Config()\n"))
            duplicate = ("def sweep():\n"
                         "    return [Config(task={'name': 'a'}), Config(task={'name': 'a'})]\n")
            with self.assertRaisesRegex(ConfigError, "重名"):
                console.load_python_sweep(self.write(base, duplicate))
            unnamed = "def sweep():\n    return [Config()]\n"
            with self.assertRaisesRegex(ConfigError, "task.name"):
                console.load_python_sweep(self.write(base, unnamed))
            relative = ("def sweep():\n    return [Config(task={'name': 'a'}, "
                        "system='../systems/x.yaml')]\n")
            [variant] = console.load_python_sweep(self.write(base, relative))
            self.assertEqual(variant["system"], str((base / "../systems/x.yaml").resolve()))


@unittest.skipIf(ENTRY is None, "没有构建好的示例入口（SIMPLESLAM_BUILD_DIR），跳过端到端用例")
class EndToEndTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.directory = tempfile.TemporaryDirectory()
        cls.base = Path(cls.directory.name)
        subprocess.run([sys.executable, str(REPO / "scripts" / "make_kitti_fixture.py"),
                        "--out", str(cls.base / "fx"), "--frames", "8"],
                       check=True, capture_output=True)
        cls.sequence = cls.base / "fx" / "odometry" / "sequences" / "00"

    @classmethod
    def tearDownClass(cls) -> None:
        cls.directory.cleanup()

    def quiet(self, argv: list[str]) -> tuple[int, str]:
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
            code = main(argv)
        return code, out.getvalue()

    def test_run_evaluate_show_and_compare(self):
        run_dir = self.base / "run"
        code, _ = self.quiet([str(REPO / "configs" / "tasks" / "kitti_odometry.yaml"),
                              f"data.source.path={self.sequence}", f"outputs.dir={run_dir}",
                              "--entry", ENTRY])
        self.assertEqual(code, 0)
        code, text = self.quiet(["show", str(run_dir)])
        self.assertEqual(code, 0)
        self.assertIn("kitti-00", text)
        self.assertIn("pass", text)
        self.assertIn("ATE_t", text)

        # 单独评测交给入口：加一个评测函数重评，结论随之更新
        code, _ = self.quiet(["evaluate", str(run_dir), "evaluation.evaluators.ate_sim3.type=ate",
                              "evaluation.evaluators.ate_sim3.alignment=sim3", "--entry", ENTRY])
        self.assertEqual(code, 0)
        code, text = self.quiet(["show", str(run_dir)])
        self.assertIn("ate_sim3:ATE_t", text)

        code, text = self.quiet(["compare", str(run_dir), str(run_dir), "--csv",
                                 str(self.base / "cmp.csv"),
                                 "--metrics", "ate.translation.rmse,ate_sim3.scale"])
        self.assertEqual(code, 0)
        lines = (self.base / "cmp.csv").read_text().splitlines()
        self.assertEqual(len(lines), 3)
        self.assertEqual(lines[0], "name,status,exit_code,complete,ate.translation.rmse,"
                                   "ate_sim3.scale,system_hash")

    def test_sweep_runs_every_variant(self):
        task = self.base / "sweep_task.py"
        task.write_text(
            "from simpleslam import Config\n"
            f"SYSTEM = {str(REPO / 'configs/systems/my_lio.yaml')!r}\n"
            f"SEQUENCE = {str(self.sequence)!r}\n"
            "config = Config(task={'name': 'fx'}, system=SYSTEM,\n"
            "    data={'source': {'type': 'kitti_odometry', 'path': SEQUENCE}})\n"
            "def sweep():\n"
            "    for d in (0.5, 2.0):\n"
            "        v = config.copy()\n"
            "        v['task.name'] = f'fx-kf{d:g}'\n"
            "        v.override('system.modules.odometry.plugin.keyframe_distance_m', d)\n"
            "        yield v\n", encoding="utf-8")
        code, text = self.quiet(["sweep", str(task), "--out", str(self.base / "sweep"),
                                 "--jobs", "2", "--entry", ENTRY])
        self.assertEqual(code, 0)
        self.assertIn("fx-kf0.5", text)
        hashes = {rows(self.base / "sweep" / name)[0].system_hash
                  for name in ("fx-kf0.5", "fx-kf2")}
        self.assertEqual(len(hashes), 2)  # 改的是系统，系统哈希各不相同

    def test_impls_lists_the_catalogs(self):
        code, text = self.quiet(["impls", "--entry", ENTRY])
        self.assertEqual(code, 0)
        self.assertIn("my_lio", text)
        self.assertIn("kitti_odometry", text)


if __name__ == "__main__":
    unittest.main()
