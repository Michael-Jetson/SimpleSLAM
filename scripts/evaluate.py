#!/usr/bin/env python3
"""SimpleSLAM 轨迹评测脚本 -- 调用 evo 工具计算 ATE/RPE

用法:
    python scripts/evaluate.py ref.txt est.txt --format tum --metric both --plot

依赖:
    pip install evo
"""

import argparse
import subprocess
import sys


def run_metric(tool: str, ref: str, est: str, fmt: str, plot: bool) -> int:
    cmd = [tool, fmt, ref, est, "--verbose", "--align"]
    if plot:
        cmd.append("--plot")
    print(f"\n{'=' * 60}")
    print(f"  {' '.join(cmd)}")
    print(f"{'=' * 60}\n")
    return subprocess.call(cmd)


def main() -> None:
    parser = argparse.ArgumentParser(description="SimpleSLAM 轨迹评测")
    parser.add_argument("ref", help="参考轨迹文件（ground truth）")
    parser.add_argument("est", help="估计轨迹文件")
    parser.add_argument("--format", choices=["tum", "kitti"], default="tum",
                        help="轨迹格式（默认 tum）")
    parser.add_argument("--metric", choices=["ate", "rpe", "both"], default="both",
                        help="评测指标（默认 both）")
    parser.add_argument("--plot", action="store_true", help="显示绘图")
    args = parser.parse_args()

    failed = False

    if args.metric in ("ate", "both"):
        ret = run_metric("evo_ape", args.ref, args.est, args.format, args.plot)
        if ret != 0:
            failed = True

    if args.metric in ("rpe", "both"):
        ret = run_metric("evo_rpe", args.ref, args.est, args.format, args.plot)
        if ret != 0:
            failed = True

    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
