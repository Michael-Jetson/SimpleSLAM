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
    ./run.py config | eval …                    任务渲染与校验、评测外部轨迹

C++ 入口是加载、校验、执行与评测的唯一实现，操作台只编排与展示。入口默认是示例
``simpleslam_lio_skeleton``，用户写的算法用登记了它的入口（``--entry <目标名或路径>``，或环境
变量 ``SIMPLESLAM_ENTRY``）。默认在本机构建 Release，``--debug`` 换成 Debug，``--docker`` 在开发
镜像里做同样的事。

任务可以写成 ``.py``：文件里构造一个名为 ``config`` 的 :class:`~simpleslam.config.Config`
（或提供 ``build()`` 返回它）；``.py`` 里 ``system`` 的相对路径相对这个文件所在的目录。

评测都在 C++：``evaluate`` 交给入口，补评一份任务记录；``eval`` 交给 ``simpleslam-eval``，
评测别的系统跑出来的轨迹，后面的选项原样交给它。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from simpleslam import console
from simpleslam.config import SYSTEM_FILE, Config, ConfigError, dump_file
from simpleslam.results import ResultError
from simpleslam.schema import Schema, SchemaError
from simpleslam.workspace import BuildError, Workspace, in_docker, repo_root, run_in_docker

COMMANDS = ("run", "evaluate", "demo", "sweep", "show", "compare", "impls", "config", "eval")
# 覆盖项里指向数据的路径：--docker 时把它们所在的目录挂进容器
_DATA_PATHS = ("data.source.path=", "data.calibration=", "evaluation.groundtruth=")


def _config_entry(args: argparse.Namespace) -> str:
    """config 子命令用的入口：同别的子命令，``--entry`` 或示例入口，按需构建。"""
    return str(_workspace(args).entry(args.entry, build=not args.no_build))


def _load_config(path: str, schema: Schema, tool: str) -> Config:
    """``.py`` 任务原样取出（引用的系统文件换成绝对路径）；YAML 任务交给 C++ 读。"""
    source = Path(path)
    if source.suffix != ".py":
        return Config.load(source, tool, schema)
    return console.load_python_task(source).validate(schema)


def _print_schema(schema: Schema) -> None:
    for field in schema:
        print(f"{field.path:52} {field.type:7} 默认={field.default:<12} {field.doc}")
    if schema.slots:
        print("\n插件节点可选的实现：")
        for path, interface in sorted(schema.slots.items()):
            names = "、".join(schema.implementations(interface)) or "（还没有任何实现）"
            print(f"  {path}（{interface}）: {names}")


def _config_command(args: argparse.Namespace) -> int:
    tool = _config_entry(args)
    schema = Schema.from_tool(tool)
    if args.action == "schema":
        _print_schema(schema)
        return 0

    config = _load_config(args.input, schema, tool)
    if args.action == "check":
        if args.input.endswith(".py"):
            config.through_cpp(tool=tool, schema=schema)  # C++ 读一遍：写错了在这里报
        print(f"任务有效: {args.input}")
        return 0

    if args.full:
        # 完整的任务由 C++ 导出：系统展开、覆盖项应用、插件节点还原、默认值补齐
        text = config.dump(tool=tool, schema=schema) if args.input.endswith(".py") \
            else dump_file(args.input, tool)
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
    return console.evaluate_trajectories(_workspace(args), args.estimate, args.reference,
                                         args.forwarded, _options(args))



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
        help="单独评测一份任务记录（入口的 evaluate：写结论；覆盖项只改评测一节）")
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

    config_parser = subparsers.add_parser("config", parents=[build], help="任务渲染与校验")
    config_parser.add_argument(
        "action", choices=["schema", "render", "check"], help="要做什么"
    )
    config_parser.add_argument("input", nargs="?", help="任务文件（.py 或 .yaml）")
    config_parser.add_argument("-o", "--output", help="输出文件（默认打到标准输出）")
    config_parser.add_argument(
        "--full", action="store_true", help="输出 C++ 展开后的完整任务（补齐默认值、还原插件节点）"
    )
    config_parser.set_defaults(func=_config_command)

    eval_parser = subparsers.add_parser(
        "eval", parents=[build],
        help="评测外部轨迹（simpleslam-eval；后面的选项原样交给它，如 --out、--align sim3）")
    eval_parser.add_argument("estimate", help="估计轨迹")
    eval_parser.add_argument("reference", help="真值轨迹")
    eval_parser.add_argument("forwarded", nargs=argparse.REMAINDER, metavar="选项",
                             help="原样交给 simpleslam-eval 的选项，写在两条轨迹后面")
    eval_parser.set_defaults(func=_eval_command)

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
