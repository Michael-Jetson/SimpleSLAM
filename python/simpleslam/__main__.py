"""``python -m simpleslam``：配置渲染与校验、轨迹格式转换、精度速查。

配置可以写成 ``.py``：文件里构造一个名为 ``config`` 的
:class:`~simpleslam.config.Config`（或提供 ``build()`` 返回它），
``render`` 把它变成 C++ 能加载的 YAML。

精度的权威实现是 ``simpleslam-eval``（退出码能让流水线失败）；
这里的 ``eval`` 子命令是给交互式查看用的。
"""

from __future__ import annotations

import argparse
import runpy
import sys
from pathlib import Path

from simpleslam.config import Config, ConfigError
from simpleslam.metrics import absolute_trajectory_error, relative_pose_error
from simpleslam.schema import Schema, SchemaError
from simpleslam.trajectory import read_trajectory, write_trajectory


def _load_schema(tool: str | None, required: bool) -> Schema | None:
    try:
        return Schema.load(tool)
    except SchemaError as exc:
        if required:
            raise
        print(f"提示: 没有 schema，跳过校验（{exc}）", file=sys.stderr)
        return None


def _load_config(path: str, schema: Schema | None) -> Config:
    source = Path(path)
    if source.suffix == ".py":
        namespace = runpy.run_path(str(source))
        config = namespace.get("config")
        if config is None and callable(namespace.get("build")):
            config = namespace["build"]()
        if not isinstance(config, Config):
            raise ConfigError(f"{path} 里没有名为 config 的 Config，也没有 build() 返回它")
    else:
        config = Config.from_yaml(source.read_text(encoding="utf-8"), schema)
    if schema is not None:
        config.validate(schema)
    return config


def _config_command(args: argparse.Namespace) -> int:
    if args.action == "schema":
        schema = _load_schema(args.tool, required=True)
        assert schema is not None
        for field in schema:
            flag = "可调" if field.tunable else "固定"
            print(f"{field.path:44} {field.type:7} {flag}  默认={field.default:<12} {field.doc}")
        return 0

    schema = _load_schema(args.tool, required=args.action in ("hash", "full"))
    config = _load_config(args.input, schema)

    if args.action == "check":
        print(f"配置有效: {args.input}（{len(config)} 项显式设置）")
        if schema is not None:
            print(f"哈希: {config.hash(schema)}")
        return 0
    if args.action == "hash":
        assert schema is not None
        print(config.hash(schema))
        return 0

    text = config.canonical(schema) if (args.full and schema) else config.to_yaml(schema)
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
        print(f"已写出 {args.output}", file=sys.stderr)
    else:
        sys.stdout.write(text)
    return 0


def _eval_command(args: argparse.Namespace) -> int:
    estimate = read_trajectory(args.estimate, args.format)
    reference = read_trajectory(args.reference, args.format)
    ate = absolute_trajectory_error(estimate, reference, args.align, args.tolerance)
    rpe = relative_pose_error(
        estimate,
        reference,
        delta_time=args.rpe_delta,
        delta_distance=args.rpe_distance,
        tolerance=args.tolerance,
    )
    print(f"估计: {args.estimate} ({len(estimate)} 帧)")
    print(f"真值: {args.reference} ({len(reference)} 帧)")
    print(
        f"对齐: {ate.alignment.kind}  尺度: {ate.alignment.scale:.6f}  "
        f"关联: {ate.pairs} 对 ({ate.coverage * 100:.1f}%)"
    )
    rows = [("ATE", "平移", ate.translation, "m"), ("ATE", "旋转", ate.rotation_deg, "deg")]
    rows += [
        ("RPE", "平移", rpe.translation, "m/m" if rpe.normalized else "m"),
        ("RPE", "旋转", rpe.rotation_deg, "deg/m" if rpe.normalized else "deg"),
    ]
    for family, what, stats, unit in rows:
        pairs = ate.pairs if family == "ATE" else rpe.pairs
        if pairs == 0:
            # 没测出来就不打印数字：零误差与"无结果"不能长得一样
            print(f"{family} {what:6}  无结果（没有可比较的样本）")
            continue
        print(
            f"{family} {what:6} rmse={stats.rmse:12.6f} mean={stats.mean:12.6f} "
            f"median={stats.median:12.6f} max={stats.max:12.6f} [{unit}] n={stats.count}"
        )
    if ate.pairs == 0:
        # 与 C++ 的门禁一致：一对都没关联上是"这次没测出东西"，退出码 4
        print("评测结论: invalid 没有任何有效关联", file=sys.stderr)
        return 4
    return 0


def _convert_command(args: argparse.Namespace) -> int:
    trajectory = read_trajectory(args.input, args.format)
    write_trajectory(args.output, trajectory, args.to)
    print(f"{len(trajectory)} 帧 → {args.output}（{args.to}）", file=sys.stderr)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m simpleslam")
    parser.add_argument("--tool", help="simpleslam-config 的路径（默认从 PATH 找）")
    subparsers = parser.add_subparsers(dest="command", required=True)

    config_parser = subparsers.add_parser("config", help="配置渲染与校验")
    config_parser.add_argument(
        "action", choices=["schema", "render", "check", "hash"], help="要做什么"
    )
    config_parser.add_argument("input", nargs="?", help="配置文件（.py 或 .yaml）")
    config_parser.add_argument("-o", "--output", help="输出文件（默认打到标准输出）")
    config_parser.add_argument(
        "--full", action="store_true", help="输出补齐默认值的完整配置"
    )
    config_parser.set_defaults(func=_config_command)

    eval_parser = subparsers.add_parser("eval", help="轨迹精度速查")
    eval_parser.add_argument("estimate")
    eval_parser.add_argument("reference")
    eval_parser.add_argument("--format", choices=["tum", "kitti"], default=None)
    eval_parser.add_argument(
        "--align", choices=["none", "first_pose", "se3", "sim3"], default="se3"
    )
    eval_parser.add_argument("--tolerance", type=float, default=0.02)
    eval_parser.add_argument("--rpe-delta", type=float, default=1.0, dest="rpe_delta")
    eval_parser.add_argument(
        "--rpe-distance", type=float, default=None, dest="rpe_distance"
    )
    eval_parser.set_defaults(func=_eval_command)

    convert_parser = subparsers.add_parser("convert", help="轨迹格式转换")
    convert_parser.add_argument("input")
    convert_parser.add_argument("output")
    convert_parser.add_argument("--format", choices=["tum", "kitti"], default=None)
    convert_parser.add_argument("--to", choices=["tum", "kitti"], required=True)
    convert_parser.set_defaults(func=_convert_command)

    args = parser.parse_args(argv)
    if args.command == "config" and args.action != "schema" and not args.input:
        parser.error("config " + args.action + " 需要一个配置文件")
    try:
        return int(args.func(args))
    except (ConfigError, SchemaError, OSError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
