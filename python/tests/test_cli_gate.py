"""评测门禁走真实入口：调用构建出来的 simpleslam-eval，核对退出码与报告内容一致。

只测库函数不够：库的判定正确，不代表入口把它原样接上了。这里直接跑可执行文件，
看它在零关联、缺数据集、缺基线条目时写进报告的结论与退出码是不是同一个；
以及独立评测时"运行完整性未知"如何处理：没有基线照常测量并注明，有基线不可比较，
运行报告必须绑定这条轨迹的内容，人工声明原样留在报告里。
"""

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


def outcome_of(report: Path) -> str:
    for line in report.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("outcome:"):
            return stripped.split(":", 1)[1].strip()
    return ""


def field_of(report: Path, key: str) -> str:
    for line in report.read_text(encoding="utf-8").splitlines():
        if line.startswith(key + ":"):
            return line.split(":", 1)[1].strip().strip('"')
    return ""


def protocol_of(report: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    inside = False
    for line in report.read_text(encoding="utf-8").splitlines():
        if line.startswith("protocol:"):
            inside = True
            continue
        if inside:
            if not line.startswith("  "):
                break
            key, _, value = line.strip().partition(":")
            values[key] = value.strip().strip('"')
    return values


@unittest.skipIf(EVAL_TOOL is None, "没有 simpleslam-eval，跳过入口门禁测试")
class EvalCliGateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
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

    def run_tool(self, *args: str) -> tuple[int, Path]:
        report = self.root / "report.yaml"
        if report.exists():
            report.unlink()
        completed = subprocess.run(
            [EVAL_TOOL, *args, "--report", str(report)],
            capture_output=True, text=True, check=False,
        )
        return completed.returncode, report

    def run_report_for(self, completeness: str) -> Path:
        """写一份绑定了当前估计轨迹内容的运行报告。摘要由入口自己算出来。"""
        code, report = self.run_tool(str(self.estimate), str(self.same))
        self.assertEqual(code, 0)
        digest = field_of(report, "trajectory_digest")
        self.assertTrue(digest)
        run_report = self.root / f"run_{completeness}.yaml"
        run_report.write_text(
            f"run_completeness: {completeness}\n"
            f"trajectory_file: {self.estimate}\n"
            f"trajectory_digest: {digest}\n",
            encoding="utf-8",
        )
        return run_report

    def test_zero_association_without_baseline_is_invalid(self):
        code, report = self.run_tool(str(self.estimate), str(self.far))
        self.assertEqual(code, 4)
        self.assertEqual(outcome_of(report), "invalid")

    def test_missing_dataset_entry_is_incomparable_in_both_places(self):
        code, report = self.run_tool(
            str(self.estimate), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/05",
        )
        self.assertEqual(code, 5)
        self.assertEqual(outcome_of(report), "incomparable")

    def test_baseline_without_dataset_is_incomparable(self):
        code, report = self.run_tool(
            str(self.estimate), str(self.same), "--baselines", str(self.baselines)
        )
        self.assertEqual(code, 5)
        self.assertEqual(outcome_of(report), "incomparable")

    def test_unknown_completeness_with_baseline_is_incomparable(self):
        # 一条轨迹文件说不出产生它的运行有没有跑完：不能拿去和全程基线比高低
        code, report = self.run_tool(
            str(self.estimate), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
        )
        self.assertEqual(code, 5)
        self.assertEqual(outcome_of(report), "incomparable")
        self.assertEqual(field_of(report, "run_completeness"), "unknown")

    def test_unknown_completeness_without_baseline_is_measured_but_marked(self):
        code, report = self.run_tool(str(self.estimate), str(self.same))
        self.assertEqual(code, 0)
        self.assertEqual(outcome_of(report), "pass")
        self.assertEqual(field_of(report, "run_completeness"), "unknown")
        self.assertIn("未知", report.read_text(encoding="utf-8"))

    def test_declared_completeness_is_recorded(self):
        code, report = self.run_tool(
            str(self.estimate), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
            "--assume-complete", "测试夹具：四帧全程已知",
        )
        self.assertEqual(code, 0)
        self.assertEqual(outcome_of(report), "pass")
        self.assertEqual(field_of(report, "run_completeness"), "declared_complete")
        self.assertEqual(field_of(report, "completeness_declaration"), "测试夹具：四帧全程已知")

    def test_run_report_must_describe_this_trajectory(self):
        run_report = self.run_report_for("complete")
        code, report = self.run_tool(
            str(self.estimate), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
            "--run-report", str(run_report),
        )
        self.assertEqual(code, 0)
        self.assertEqual(field_of(report, "run_completeness"), "complete")
        self.assertEqual(field_of(report, "completeness_source"), str(run_report))

        # 换一条轨迹，运行报告就不再为它作证
        self.estimate.write_text("1.0 1.0 0 0 0 0 0 1\n2.0 2.5 0 0 0 0 0 1\n", encoding="utf-8")
        code, _ = self.run_tool(
            str(self.estimate), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
            "--run-report", str(run_report),
        )
        self.assertEqual(code, 1)

    def test_incomplete_run_report_is_invalid(self):
        run_report = self.run_report_for("incomplete")
        code, report = self.run_tool(
            str(self.estimate), str(self.same),
            "--baselines", str(self.baselines), "--dataset", "kitti/00",
            "--run-report", str(run_report),
        )
        self.assertEqual(code, 4)
        self.assertEqual(outcome_of(report), "invalid")

    def test_non_finite_or_out_of_range_numbers_are_rejected(self):
        # nan、负的间隔、带单位的数：不能原样进入协议
        for args in (["--interpolate", "nan"], ["--rpe-delta", "-1"], ["--tolerance", "0.1s"],
                     ["--window", "5", "1"]):
            code, _ = self.run_tool(str(self.estimate), str(self.same), *args)
            self.assertEqual(code, 2, args)

    def test_evidence_and_declaration_are_exclusive(self):
        run_report = self.run_report_for("complete")
        code, _ = self.run_tool(
            str(self.estimate), str(self.same),
            "--run-report", str(run_report), "--assume-complete", "两样都给",
        )
        self.assertEqual(code, 2)
        code, _ = self.run_tool(str(self.estimate), str(self.same), "--assume-complete", "")
        self.assertEqual(code, 2)

    def test_protocol_records_what_was_actually_used(self):
        code, report = self.run_tool(
            str(self.estimate), str(self.same),
            "--align", "first_pose", "--tolerance", "0.03", "--rpe-delta", "0.1",
        )
        self.assertEqual(code, 0)
        protocol = protocol_of(report)
        self.assertEqual(protocol["alignment"], "first_pose")
        self.assertEqual(protocol["association"], "nearest")
        self.assertAlmostEqual(float(protocol["association_tolerance_s"]), 0.03)
        self.assertAlmostEqual(float(protocol["rpe_delta"]), 0.1)
        self.assertEqual(protocol["rpe_delta_kind"], "time")


if __name__ == "__main__":
    unittest.main()
