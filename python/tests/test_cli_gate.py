"""评测门禁走真实入口：调用构建出来的 simpleslam-eval，核对退出码与结论内容一致。

只测库函数不够：库的判定正确，不代表入口把它原样接上了。这里直接跑可执行文件，
看它在零关联、缺数据集、缺基线条目时写进结论（``result.json``）的门禁与退出码是不是同一个；
以及独立评测时"运行完整性未知"如何处理：没有基线照常测量并注明，有基线不可比较，
运行结论必须列着这条轨迹的内容摘要，人工声明原样留在结论里。
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


@unittest.skipIf(EVAL_TOOL is None, "没有 simpleslam-eval，跳过入口门禁测试")
class EvalCliGateTest(unittest.TestCase):
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
        self.baselines = self.root / "baselines.yaml"
        self.baselines.write_text(
            "baselines:\n  - dataset: kitti/00\n    bounds:\n      ate.translation.rmse: 1.0\n",
            encoding="utf-8",
        )

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

    def produced_by_run(self, complete: bool) -> tuple[Path, Path]:
        """一条"某次运行产出的"估计轨迹，和那次运行的结论：结论里列着这条轨迹的内容摘要，
        摘要由入口自己算出来。"""
        code, data, _ = self.run_tool(str(self.estimate), str(self.same))
        self.assertEqual(code, 0)
        produced = self.root / "produced.tum"
        shutil.copyfile(self.out / "trajectory" / "odometry.tum", produced)
        run_result = self.root / "run" / "result.json"
        run_result.parent.mkdir()
        run_result.write_text(json.dumps({
            "schema": 2, "kind": "run", "complete": complete,
            "trajectories": {"odometry": {"file": "trajectory/odometry.tum",
                                          "digest": data["trajectories"]["odometry"]["digest"]}},
        }), encoding="utf-8")
        return produced, run_result

    def test_without_out_it_prints_and_keeps_no_record(self):
        # v2 的速查用法：只打印结论，不报任务记录的位置
        completed = subprocess.run([EVAL_TOOL, str(self.estimate), str(self.same)],
                                   capture_output=True, text=True, check=False)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("评测 ate（ate）: 4 个样本", completed.stdout)
        self.assertNotIn("结论:", completed.stdout)

    def test_report_flag_is_gone_and_says_what_to_use(self):
        code, data, stderr = self.run_tool(str(self.estimate), str(self.same),
                                           "--report", str(self.root / "report.yaml"))
        self.assertEqual(code, 2)
        self.assertIsNone(data)
        self.assertIn("--out", stderr)

    def test_v2_run_report_is_evidence_too(self):
        produced, run_result = self.produced_by_run(complete=True)
        recorded = json.loads(run_result.read_text(encoding="utf-8"))
        digest = recorded["trajectories"]["odometry"]["digest"]
        report = self.root / "report.yaml"
        report.write_text(f"dataset: kitti/00\nrun_completeness: complete\n"
                          f"trajectory_digest: {digest}\n", encoding="utf-8")
        code, data, _ = self.run_tool(
            str(produced), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
            "--run-report", str(report),
        )
        self.assertEqual(code, 0)
        self.assertTrue(data["complete"])
        self.assertEqual(data["completeness_source"], str(report))

    def test_zero_association_without_baseline_is_invalid(self):
        code, data, _ = self.run_tool(str(self.estimate), str(self.far))
        self.assertEqual(code, 4)
        self.assertEqual(data["gate"]["outcome"], "invalid")
        self.assertEqual(data["evaluations"]["ate"]["status"], "no_result")
        self.assertEqual(data["evaluations"]["ate"]["scalars"], {})  # 无结果不写成零误差

    def test_missing_dataset_entry_is_incomparable_in_both_places(self):
        code, data, _ = self.run_tool(
            str(self.estimate), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/05",
        )
        self.assertEqual(code, 5)
        self.assertEqual(data["gate"]["outcome"], "incomparable")
        self.assertEqual(data["status"], "incomparable")

    def test_baseline_without_dataset_is_incomparable(self):
        code, data, _ = self.run_tool(
            str(self.estimate), str(self.same), "--baselines", str(self.baselines)
        )
        self.assertEqual(code, 5)
        self.assertEqual(data["gate"]["outcome"], "incomparable")

    def test_unknown_completeness_with_baseline_is_incomparable(self):
        # 一条轨迹文件说不出产生它的运行有没有跑完：不能拿去和全程基线比高低
        code, data, stderr = self.run_tool(
            str(self.estimate), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
        )
        self.assertEqual(code, 5)
        self.assertEqual(data["gate"]["outcome"], "incomparable")
        self.assertIsNone(data["complete"])
        self.assertEqual(data["completeness"], "unknown")
        self.assertIn("--run-report", stderr)  # 告诉人怎么补上证据

    def test_unknown_completeness_without_baseline_is_measured_but_marked(self):
        code, data, _ = self.run_tool(str(self.estimate), str(self.same))
        self.assertEqual(code, 0)
        self.assertEqual(data["gate"]["outcome"], "pass")
        self.assertEqual(data["completeness"], "unknown")
        self.assertIn("未知", data["gate"]["detail"])
        # 测量照常：标量进结论，逐样本进表格
        ate = data["evaluations"]["ate"]
        self.assertEqual(ate["samples"], 4)
        self.assertAlmostEqual(ate["scalars"]["translation.rmse"]["value"], 0.0)
        self.assertEqual(ate["scalars"]["translation.rmse"]["label"], "ATE_t")
        self.assertTrue((self.out / ate["tables"]["errors"]["file"]).is_file())

    def test_declared_completeness_is_recorded(self):
        code, data, _ = self.run_tool(
            str(self.estimate), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
            "--assume-complete", "测试夹具：四帧全程已知",
        )
        self.assertEqual(code, 0)
        self.assertEqual(data["gate"]["outcome"], "pass")
        self.assertEqual(data["completeness"], "declared_complete")
        self.assertEqual(data["declaration"], "测试夹具：四帧全程已知")

    def test_run_report_must_describe_this_trajectory(self):
        produced, run_result = self.produced_by_run(complete=True)
        code, data, _ = self.run_tool(
            str(produced), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
            "--run-report", str(run_result),
        )
        self.assertEqual(code, 0)
        self.assertTrue(data["complete"])
        self.assertEqual(data["completeness_source"], str(run_result))

        # 换一条轨迹，那份结论就不再为它作证
        produced.write_text("1.0 1.0 0 0 0 0 0 1\n2.0 2.5 0 0 0 0 0 1\n", encoding="utf-8")
        code, _, stderr = self.run_tool(
            str(produced), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
            "--run-report", str(run_result),
        )
        self.assertEqual(code, 1)
        self.assertIn("内容摘要不符", stderr)

    def test_incomplete_run_report_is_invalid(self):
        produced, run_result = self.produced_by_run(complete=False)
        code, data, _ = self.run_tool(
            str(produced), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
            "--run-report", str(run_result),
        )
        self.assertEqual(code, 4)
        self.assertEqual(data["gate"]["outcome"], "invalid")
        self.assertFalse(data["complete"])

    def test_non_finite_or_out_of_range_numbers_are_rejected(self):
        # nan、负的间隔、带单位的数：不能原样进入协议
        for args in (["--interpolate", "nan"], ["--rpe-delta", "-1"], ["--tolerance", "0.1s"],
                     ["--window", "5", "1"]):
            code, data, _ = self.run_tool(str(self.estimate), str(self.same), *args)
            self.assertEqual(code, 2, args)
            self.assertIsNone(data, args)

    def test_evidence_and_declaration_are_exclusive(self):
        produced, run_result = self.produced_by_run(complete=True)
        code, _, _ = self.run_tool(
            str(produced), str(self.same),
            "--run-report", str(run_result), "--assume-complete", "两样都给",
        )
        self.assertEqual(code, 2)
        code, _, _ = self.run_tool(str(self.estimate), str(self.same), "--assume-complete", "")
        self.assertEqual(code, 2)

    def test_protocol_records_what_was_actually_used(self):
        code, data, _ = self.run_tool(
            str(self.estimate), str(self.same),
            "--align", "first_pose", "--tolerance", "0.03", "--rpe-delta", "0.1",
        )
        self.assertEqual(code, 0)
        # 协议是评测函数还原后的全部参数，随结论保存
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
