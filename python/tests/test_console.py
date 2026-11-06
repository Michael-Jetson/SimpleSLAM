"""操作台：命令行归一化、构建目录里找入口、档案摘要与对比表、扫参变体、端到端跑任务。

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


def write_result(archive: Path, data: dict) -> Path:
    archive.mkdir(parents=True, exist_ok=True)
    (archive / "result.json").write_text(json.dumps({"schema": RESULT_SCHEMA, **data}),
                                         encoding="utf-8")
    return archive


def run_result(task: str, status: str = "pass", exit_code: int = 0, **summary) -> dict:
    return {"kind": "run", "task": task, "mode": "offline", "purpose": "experiment",
            "software_version": "2.3", "task_hash": "a" * 16, "system_hash": "b" * 16,
            "complete": True, "exit_code": exit_code, "status": status, "detail": "",
            "summary": summary, "metrics": summary}


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
            single = write_result(Path(directory) / "a", run_result(
                "kitti-00", **{"ate.translation.rmse": 1.5, "ate.rotation_deg.rmse": 0.25}))
            suite = write_result(Path(directory) / "b", {
                "kind": "sequences", "task": "suite", "mode": "online",
                "purpose": "experiment", "software_version": "2.3", "task_hash": "c" * 16,
                "system_hash": "d" * 16, "exit_code": 1,
                "sequences": [
                    {"name": "00", "archive": "x", "log": "y", "exit_code": 0, "status": "pass",
                     "complete": True, "problem": "", "metrics": {"ate.translation.rmse": 2.0}},
                    {"name": "05", "archive": "x", "log": "y", "exit_code": 1, "status": "error",
                     "complete": None, "problem": "目录不存在", "metrics": {}}],
                "mean": {}})
            items = rows(single) + rows(suite)
            self.assertEqual([row.name for row in items], ["kitti-00", "suite/00", "suite/05"])
            text = table(items)
            self.assertIn("ATE_t", text)
            self.assertIn("1.5000", text)
            self.assertIn("bbbbbbbb", text)  # 系统哈希的前八位
            lines = csv_table(items).splitlines()
            self.assertEqual(lines[0], "name,status,exit_code,complete,ate.translation.rmse,"
                                       "ate.rotation_deg.rmse,system_hash")
            self.assertEqual(lines[1], "kitti-00,pass,0,true,1.5,0.25," + "b" * 16)
            self.assertEqual(lines[3], "suite/05,error,1,,,," + "d" * 16)  # 没有结果就留空
            self.assertIn("suite/05: 目录不存在", describe(suite))

    def test_missing_or_foreign_results_are_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ResultError, "没有 result.json"):
                rows(Path(directory))
            archive = Path(directory) / "new"
            archive.mkdir()
            (archive / "result.json").write_text(json.dumps({"schema": RESULT_SCHEMA + 1}))
            with self.assertRaisesRegex(ResultError, "格式版本"):
                rows(archive)

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

    def test_run_show_and_compare(self):
        archive = self.base / "run"
        code, _ = self.quiet([str(REPO / "configs" / "tasks" / "kitti_odometry.yaml"),
                              f"data.source.path={self.sequence}", f"outputs.dir={archive}",
                              "--entry", ENTRY])
        self.assertEqual(code, 0)
        code, text = self.quiet(["show", str(archive)])
        self.assertEqual(code, 0)
        self.assertIn("kitti-00", text)
        self.assertIn("pass", text)
        code, text = self.quiet(["compare", str(archive), str(archive), "--csv",
                                 str(self.base / "cmp.csv")])
        self.assertEqual(code, 0)
        self.assertEqual(len((self.base / "cmp.csv").read_text().splitlines()), 3)

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
