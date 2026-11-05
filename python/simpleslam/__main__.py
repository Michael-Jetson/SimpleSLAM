"""``python -m simpleslam``：任务渲染与校验、轨迹格式转换、精度速查。

任务可以写成 ``.py``：文件里构造一个名为 ``config`` 的
:class:`~simpleslam.config.Config`（或提供 ``build()`` 返回它），
``render`` 把它变成 C++ 能加载的 YAML。``.py`` 里 ``system`` 的相对路径
相对这个文件所在的目录，和 YAML 任务相对任务文件一样。

schema 与插件的字段清单来自 ``--tool`` 指定的入口：用户写的算法要用登记了它的入口
（例如 ``simpleslam-lio-skeleton``），``simpleslam-config`` 只有内置实现。

精度的权威实现是 ``simpleslam-eval``（退出码能让流水线失败）；
这里的 ``eval`` 子命令是给交互式查看用的。
"""

from __future__ import annotations

import argparse
import os
import runpy
import sys
from pathlib import Path

from simpleslam.config import SYSTEM_FILE, Config, ConfigError
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


def _load_config(path: str, schema: Schema | None, tool: str | None) -> Config:
    """``.py`` 任务原样取出（引用的系统文件换成绝对路径）；YAML 任务交给 C++ 读。"""
    source = Path(path)
    if source.suffix != ".py":
        return Config.load(source, tool, schema)
    namespace = runpy.run_path(str(source))
    config = namespace.get("config")
    if config is None and callable(namespace.get("build")):
        config = namespace["build"]()
    if not isinstance(config, Config):
        raise ConfigError(f"{path} 里没有名为 config 的 Config，也没有 build() 返回它")
    system_file = config.get(SYSTEM_FILE)
    if isinstance(system_file, str):
        config = config.copy()
        config[SYSTEM_FILE] = str((source.resolve().parent / system_file).resolve())
    if schema is not None:
        config.validate(schema)
    return config


def _complete(config: Config, schema: Schema, tool: str | None) -> Config:
    """完整、自足的任务：自足的 Python 任务直接用，引用了文件或带覆盖项的交给 C++ 展开。"""
    return config if config.self_contained else config.through_cpp(tool=tool, schema=schema)


def _print_schema(schema: Schema) -> None:
    for field in schema:
        flag = "可调" if field.tunable else "固定"
        print(f"{field.path:52} {field.type:7} {flag}  默认={field.default:<12} {field.doc}")
    if schema.slots:
        print("\n插件节点可选的实现：")
        for path, interface in sorted(schema.slots.items()):
            names = "、".join(schema.implementations(interface)) or "（还没有任何实现）"
            print(f"  {path}（{interface}）: {names}")


def _config_command(args: argparse.Namespace) -> int:
    if args.action == "schema":
        schema = _load_schema(args.tool, required=True)
        assert schema is not None
        _print_schema(schema)
        return 0

    # 只有不补默认值的渲染可以没有 schema；其余都要按实现的字段清单还原
    needs_schema = args.action != "render" or args.full or not args.input.endswith(".py")
    schema = _load_schema(args.tool, required=needs_schema)
    config = _load_config(args.input, schema, args.tool)

    if args.action in ("check", "hash"):
        assert schema is not None
        complete = _complete(config, schema, args.tool)
        if args.action == "hash":
            print(complete.hash(schema))
            return 0
        print(f"任务有效: {args.input}（{len(config)} 项显式设置）")
        print(f"任务哈希: {complete.hash(schema)}")
        print(f"系统哈希: {complete.system_hash(schema)}")
        return 0

    if args.full:
        assert schema is not None
        text = _complete(config, schema, args.tool).canonical(schema)
    else:
        if args.output and isinstance(config.get(SYSTEM_FILE), str):
            # 渲染出的文件挪了位置：系统文件改写成相对它的路径
            config = config.copy()
            target = Path(args.output).resolve().parent
            config[SYSTEM_FILE] = os.path.relpath(config[SYSTEM_FILE], target)
        text = config.to_yaml(schema)
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
    parser.add_argument(
        "--tool",
        help="导出 schema、加载任务的入口（登记了你的算法的可执行文件；"
        "默认 SIMPLESLAM_CONFIG_TOOL，其次 PATH 里的 simpleslam-config）",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    config_parser = subparsers.add_parser("config", help="任务渲染与校验")
    config_parser.add_argument(
        "action", choices=["schema", "render", "check", "hash"], help="要做什么"
    )
    config_parser.add_argument("input", nargs="?", help="任务文件（.py 或 .yaml）")
    config_parser.add_argument("-o", "--output", help="输出文件（默认打到标准输出）")
    config_parser.add_argument(
        "--full", action="store_true", help="输出补齐默认值、还原插件节点的完整任务"
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
        parser.error("config " + args.action + " 需要一个任务文件")
    try:
        return int(args.func(args))
    except (ConfigError, SchemaError, OSError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
