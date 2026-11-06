"""评测外部轨迹走真实入口：调用构建出来的 simpleslam-eval，核对写出的结论与退出码。

只测库函数不够：库的结果正确，不代表入口把它原样接上了。这里直接跑可执行文件：
只评测、不判通过与否，结论（``result.json``）里写着评测函数的参数与结果，完整性是 null；
没测出东西时如实写无结果，不写成零误差；认不出的选项给出用法。
"""

import json
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


def find_tool(name: str) -> str | None:
    build_dir = os.environ.get("SIMPLESLAM_BUILD_DIR")
    if build_dir:
        candidate = Path(build_dir) / "tools" / name
        if candidate.exists():
            return str(candidate)
    return shutil.which(name)


EVAL_TOOL = find_tool("simpleslam-eval")


@unittest.skipIf(EVAL_TOOL is None, "没有 simpleslam-eval，跳过")
class EvalCliTest(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.out = self.root / "evaluated"
        self.estimate = self.root / "est.tum"
        self.same = self.root / "same.tum"
        self.far = self.root / "far.tum"
        rows = [f"{t:.1f} {t:.1f} 0 0 0 0 0 1" for t in (1.0, 2.0, 3.0, 4.0)]
        self.estimate.write_text("\n".join(rows) + "\n", encoding="utf-8")
        self.same.write_text("\n".join(rows) + "\n", encoding="utf-8")
        far = [f"{t + 100:.1f} 0 0 0 0 0 0 1" for t in (1.0, 2.0, 3.0, 4.0)]
        self.far.write_text("\n".join(far) + "\n", encoding="utf-8")

    def tearDown(self) -> None:
        self.directory.cleanup()

    def run_tool(self, *args: str) -> tuple[int, dict | None, str]:
        """跑一次，返回退出码、写出的结论（没写出时为 None）与标准错误。"""
        result = self.out / "result.json"
        if result.exists():
            result.unlink()
        completed = subprocess.run(
            [EVAL_TOOL, *args, "--out", str(self.out), "--overwrite"],
            capture_output=True, text=True, check=False,
        )
        data = json.loads(result.read_text(encoding="utf-8")) if result.exists() else None
        return completed.returncode, data, completed.stderr

    def test_writes_the_same_kind_of_result(self):
        code, data, _ = self.run_tool(str(self.estimate), str(self.same), "--dataset", "kitti/00")
        self.assertEqual(code, 0)
        self.assertEqual(data["task"], "kitti/00")
        self.assertIsNone(data["complete"])  # 一条轨迹文件说不出产生它的运行是否完整
        self.assertEqual(data["exit_code"], 0)
        self.assertEqual(data["evaluations"]["ate"]["status"], "ok")
        self.assertEqual(data["evaluations"]["ate"]["samples"], 4)

    def test_without_out_it_prints_and_keeps_no_record(self):
        completed = subprocess.run([EVAL_TOOL, str(self.estimate), str(self.same)],
                                   capture_output=True, text=True, check=False)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("评测 ate（ate）: 4 个样本", completed.stdout)
        self.assertNotIn("结论:", completed.stdout)

    def test_zero_association_is_no_result_not_zero_error(self):
        code, data, _ = self.run_tool(str(self.estimate), str(self.far))
        self.assertEqual(code, 0)  # 评测照常完成，只是没测出东西
        self.assertEqual(data["evaluations"]["ate"]["status"], "no_result")
        self.assertEqual(data["evaluations"]["ate"]["scalars"], {})  # 无结果不写成零误差

    def test_unknown_option_prints_usage(self):
        code, data, stderr = self.run_tool(str(self.estimate), str(self.same), "--nope")
        self.assertEqual(code, 2)
        self.assertIsNone(data)
        self.assertIn("用法: simpleslam-eval", stderr)

    def test_non_finite_or_out_of_range_numbers_are_rejected(self):
        # nan、负的间隔、带单位的数：不能原样写进评测函数的参数
        for args in (["--interpolate", "nan"], ["--rpe-delta", "-1"], ["--tolerance", "0.1s"],
                     ["--window", "5", "1"]):
            code, data, _ = self.run_tool(str(self.estimate), str(self.same), *args)
            self.assertEqual(code, 2, args)
            self.assertIsNone(data, args)

    def test_params_record_what_was_actually_used(self):
        code, data, _ = self.run_tool(
            str(self.estimate), str(self.same),
            "--align", "first_pose", "--tolerance", "0.03", "--rpe-delta", "0.1",
        )
        self.assertEqual(code, 0)
        # 评测函数还原后的全部参数随结论保存
        ate = data["evaluations"]["ate"]
        self.assertEqual(ate["type"], "ate")
        self.assertEqual(ate["params"]["alignment"], "first_pose")
        self.assertEqual(ate["params"]["association"], "nearest")
        self.assertAlmostEqual(ate["params"]["tolerance_s"], 0.03)
        rpe = data["evaluations"]["rpe"]
        self.assertAlmostEqual(rpe["params"]["delta_s"], 0.1)
        self.assertIn("按时间取段，每段 0.1 秒", rpe["notes"])


if __name__ == "__main__":
    unittest.main()
