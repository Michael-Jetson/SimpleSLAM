"""``python -m simpleslam``（根目录的 ``./run.py`` 是同一个东西）：SimpleSLAM 的操作台。

    ./run.py demo                               合成夹具，跑通整条链路
    ./run.py <任务.yaml|任务.py> [路径=值 …]      构建（按需）后跑一个任务，等同 run
    ./run.py evaluate <任务记录> [evaluation.…=值 …]
                                                单独评测一份任务记录：补评，或换评测函数重评
    ./run.py sweep <任务.py> [路径=值 …]          跑 sweep() 生成的一组变体，最后汇总
    ./run.py show <任务记录> [--metrics …]        一份任务记录的摘要
    ./run.py compare <任务记录> … [--metrics …] [--csv]
                                                几份任务记录的对比表
    ./run.py impls                              入口里登记了哪些实现、各有哪些参数
    ./run.py config | eval | convert …          任务渲染与校验、轨迹精度速查、轨迹格式转换

C++ 入口是加载、校验、执行与评测判定的唯一实现，操作台只编排与展示。入口默认是示例
``simpleslam_lio_skeleton``，用户写的算法用登记了它的入口（``--entry <目标名或路径>``，或环境
变量 ``SIMPLESLAM_ENTRY``）。默认在本机构建 Release，``--debug`` 换成 Debug，``--docker`` 在开发
镜像里做同样的事。

任务可以写成 ``.py``：文件里构造一个名为 ``config`` 的 :class:`~simpleslam.config.Config`
（或提供 ``build()`` 返回它）；``.py`` 里 ``system`` 的相对路径相对这个文件所在的目录。

精度的权威实现是 C++（退出码能让流水线失败）：``evaluate`` 交给入口，写结论、过门禁；
``eval`` 子命令是给交互式查看用的，只打印，不写结论。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from simpleslam import console
from simpleslam.config import SYSTEM_FILE, Config, ConfigError
from simpleslam.metrics import absolute_trajectory_error, relative_pose_error
from simpleslam.results import ResultError
from simpleslam.schema import Schema, SchemaError
from simpleslam.trajectory import read_trajectory, write_trajectory
from simpleslam.workspace import BuildError, Workspace, in_docker, repo_root, run_in_docker

COMMANDS = ("run", "evaluate", "demo", "sweep", "show", "compare", "impls", "config", "eval",
            "convert")
# 覆盖项里指向数据的路径：--docker 时把它们所在的目录挂进容器
_DATA_PATHS = ("data.source.path=", "data.calibration=", "evaluation.groundtruth=")


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
    config = console.load_python_task(source)
    if schema is not None:
        config.validate(schema)
    return config


def _complete(config: Config, schema: Schema, tool: str | None) -> Config:
    """完整、自足的任务：自足的 Python 任务直接用；引用了文件、带覆盖项或用了 v2 写法的
    交给 C++ 展开。"""
    if config.self_contained and not config.legacy_paths(schema):
        return config
    return config.through_cpp(tool=tool, schema=schema)


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


def _workspace(args: argparse.Namespace) -> Workspace:
    return Workspace(repo_root(), "Debug" if args.debug else "Release")


def _options(args: argparse.Namespace) -> console.Options:
    return console.Options(entry=args.entry, build=not args.no_build,
                           jobs=getattr(args, "jobs", 1),
                           overwrite=getattr(args, "overwrite", False))


def _run_command(args: argparse.Namespace) -> int:
    return console.run_task(_workspace(args), Path(args.task), args.overrides, _options(args))


def _evaluate_command(args: argparse.Namespace) -> int:
    return console.evaluate(_workspace(args), Path(args.run_dir), args.overrides, _options(args))


def _metrics(args: argparse.Namespace) -> list[str] | None:
    """``--metrics a.b.c,d.e.f``：按全名挑列。"""
    if not args.metrics:
        return None
    return [item.strip() for item in args.metrics.split(",") if item.strip()]


def _demo_command(args: argparse.Namespace) -> int:
    return console.demo(_workspace(args), _options(args))


def _sweep_command(args: argparse.Namespace) -> int:
    out = Path(args.out) if args.out else None
    return console.sweep(_workspace(args), Path(args.task), args.overrides, _options(args), out)


def _show_command(args: argparse.Namespace) -> int:
    print(console.show(Path(args.run_dir), _metrics(args)), end="")
    return 0


def _compare_command(args: argparse.Namespace) -> int:
    text = console.compare([Path(item) for item in args.run_dirs], as_csv=args.csv is not None,
                           metrics=_metrics(args))
    if args.csv:
        Path(args.csv).write_text(text, encoding="utf-8")
        print(f"已写出 {args.csv}", file=sys.stderr)
    else:
        print(text, end="")
    return 0


def _impls_command(args: argparse.Namespace) -> int:
    print(console.impls(_workspace(args), _options(args)), end="")
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m simpleslam",
                                     description="SimpleSLAM 的操作台：构建、跑任务、看结果")
    parser.add_argument(
        "--tool",
        help="config 子命令用的入口（可执行文件；默认 SIMPLESLAM_CONFIG_TOOL，其次 PATH 里的 "
        "simpleslam-config）",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    build = argparse.ArgumentParser(add_help=False)
    build.add_argument("--entry", help="入口：CMake 目标名或可执行文件路径（默认示例入口）")
    build.add_argument("--debug", action="store_true", help="用 Debug 构建（默认 Release）")
    build.add_argument("--no-build", action="store_true", help="不构建，直接用构建目录里已有的入口")
    build.add_argument("--docker", action="store_true", help="在开发镜像里构建与运行")
    build.add_argument("--mount", action="append", default=[],
                       help="--docker 时只读挂进容器的数据目录，可以写多次")

    run = subparsers.add_parser("run", parents=[build], help="构建（按需）后跑一个任务")
    run.add_argument("task", help="任务文件（.yaml 或 .py）")
    run.add_argument("--jobs", type=int, default=1, help="多序列时同时跑几个序列")
    run.add_argument("--overwrite", action="store_true", help="任务记录目录已有内容时照样写")
    run.set_defaults(func=_run_command, overrides=[])

    evaluate = subparsers.add_parser(
        "evaluate", parents=[build],
        help="单独评测一份任务记录（入口的 evaluate：写结论、过门禁；覆盖项只改评测一节）")
    evaluate.add_argument("run_dir", metavar="任务记录")
    evaluate.set_defaults(func=_evaluate_command, overrides=[])

    demo = subparsers.add_parser("demo", parents=[build], help="合成夹具，跑通整条链路")
    demo.set_defaults(func=_demo_command)

    sweep = subparsers.add_parser("sweep", parents=[build], help="跑 .py 任务里 sweep() 的变体")
    sweep.add_argument("task", help="定义了 sweep() 的 .py 任务")
    sweep.add_argument("--jobs", type=int, default=1, help="同时跑几个变体")
    sweep.add_argument("--overwrite", action="store_true",
                       help="变体的任务记录目录已有内容时照样写")
    sweep.add_argument("--out", help="变体任务记录的上层目录（默认 runs/<文件名>）")
    sweep.set_defaults(func=_sweep_command, overrides=[])

    metrics = argparse.ArgumentParser(add_help=False)
    metrics.add_argument("--metrics", metavar="全名,…",
                         help="按全名挑列，如 ate.translation.rmse,rpe.translation.rmse"
                         "（默认取评测函数标了短名的那几项）")

    show = subparsers.add_parser("show", parents=[metrics], help="一份任务记录的摘要")
    show.add_argument("run_dir", metavar="任务记录")
    show.set_defaults(func=_show_command)

    compare = subparsers.add_parser("compare", parents=[metrics], help="几份任务记录的对比表")
    compare.add_argument("run_dirs", metavar="任务记录", nargs="+")
    compare.add_argument("--csv", metavar="文件", help="把对比表写成 CSV")
    compare.set_defaults(func=_compare_command)

    impls = subparsers.add_parser("impls", parents=[build], help="入口里登记的实现与参数")
    impls.set_defaults(func=_impls_command)

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

    eval_parser = subparsers.add_parser("eval", help="轨迹精度速查（只打印，不写结论）")
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
    return parser


def normalize(argv: list[str]) -> list[str]:
    """没写子命令而给了任务文件时，等同 ``run``：``./run.py 任务.yaml`` 就是一键运行。
    ``run`` 插在最前面，任务文件前后的选项（``--entry`` 等）都归它。"""
    if any(arg in COMMANDS for arg in argv):
        return argv
    if any(not arg.startswith("-") and arg.endswith((".yaml", ".yml", ".py")) for arg in argv):
        return ["run", *argv]
    return argv


def _docker_argv(argv: list[str]) -> list[str]:
    """容器里执行的命令行：去掉 --docker 与 --mount，它们只对宿主有意义。"""
    out: list[str] = []
    skip = False
    for arg in argv:
        if skip:
            skip = False
        elif arg == "--docker" or arg.startswith("--mount="):
            continue
        elif arg == "--mount":
            skip = True
        else:
            out.append(arg)
    return out


def _data_mounts(args: argparse.Namespace) -> list[Path]:
    mounts = [Path(item) for item in args.mount]
    for item in getattr(args, "overrides", []):
        for prefix in _DATA_PATHS:
            if item.startswith(prefix):
                path = Path(item[len(prefix):].strip('"'))
                if path.exists():
                    mounts.append(path if path.is_dir() else path.parent)
    return mounts


def main(argv: list[str] | None = None) -> int:
    argv = normalize(list(sys.argv[1:] if argv is None else argv))
    parser = _parser()
    args, extra = parser.parse_known_args(argv)
    # 跑任务的命令里，形如 路径=值 的参数是覆盖项，原样交给 C++ 入口
    if args.command in ("run", "sweep", "evaluate"):
        args.overrides = [item for item in extra if "=" in item and not item.startswith("-")]
        extra = [item for item in extra if item not in args.overrides]
    if extra:
        parser.error(f"认不出的参数: {' '.join(extra)}")
    if args.command == "config" and args.action != "schema" and not args.input:
        parser.error("config " + args.action + " 需要一个任务文件")
    try:
        if getattr(args, "docker", False) and not in_docker():
            return run_in_docker(repo_root(), _docker_argv(argv), _data_mounts(args))
        return int(args.func(args))
    except (BuildError, ConfigError, ResultError, SchemaError, OSError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
