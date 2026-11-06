"""操作台：构建、跑任务、批量跑、看结果。

C++ 入口是加载、校验、执行与评测判定的唯一实现，这里只编排与展示：

* ``run``：按需构建入口，YAML 任务直接交给它，``.py`` 任务先渲染；
* ``evaluate``：单独评测一份任务记录，交给入口的 ``evaluate``（补评、换评测函数重评）；
* ``demo``：合成一份最小 KITTI 夹具，用示例入口跑通整条链路；
* ``sweep``：跑 ``.py`` 任务里 ``sweep()`` 生成的一组变体，每个变体一份任务记录，最后汇总；
* ``show``、``compare``：读任务记录的 ``result.json``，给出摘要与对比表；
* ``impls``：入口里登记了哪些实现、各有哪些参数。
"""

from __future__ import annotations

import runpy
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from simpleslam.config import SYSTEM_FILE, Config, ConfigError, assignment
from simpleslam.results import (ResultError, Row, csv_table, describe, rows, table,
                                worst_exit_code)
from simpleslam.schema import Schema
from simpleslam.workspace import DEFAULT_ENTRY, EVAL_TOOL, Workspace


def anchored(config: Config, base: Path) -> Config:
    """``.py`` 任务里 ``system`` 的相对路径相对这个文件所在的目录：换成绝对路径。"""
    system_file = config.get(SYSTEM_FILE)
    if not isinstance(system_file, str):
        return config
    result = config.copy()
    result[SYSTEM_FILE] = str((base / system_file).resolve())
    return result


def load_python_task(path: Path) -> Config:
    """``.py`` 任务：文件里名为 ``config`` 的 Config，或 ``build()`` 返回的 Config。"""
    namespace = runpy.run_path(str(path))
    config = namespace.get("config")
    if config is None and callable(namespace.get("build")):
        config = namespace["build"]()
    if not isinstance(config, Config):
        raise ConfigError(f"{path} 里没有名为 config 的 Config，也没有 build() 返回它")
    return anchored(config, path.resolve().parent)


def load_python_sweep(path: Path) -> list[Config]:
    """``.py`` 任务里 ``sweep()`` 给出的一组变体。每个变体要有自己的 ``task.name``：
    它是任务记录的目录名。"""
    namespace = runpy.run_path(str(path))
    make = namespace.get("sweep")
    if not callable(make):
        raise ConfigError(f"{path} 里没有 sweep()")
    variants = list(make())
    if not variants or not all(isinstance(item, Config) for item in variants):
        raise ConfigError(f"{path} 的 sweep() 要给出一组 Config")
    names = [item.get("task.name") for item in variants]
    for name in names:
        if not isinstance(name, str) or not name or "/" in name or name in (".", ".."):
            raise ConfigError(f"sweep() 的每个变体都要写 task.name，而且能当目录名，得到 {name!r}")
    if len(set(names)) != len(names):
        raise ConfigError(f"sweep() 里有重名的变体：{names}")
    return [anchored(item, path.resolve().parent) for item in variants]


def render(config: Config, schema: Schema, target: Path) -> Path:
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(config.to_yaml(schema), encoding="utf-8")
    return target


@dataclass
class Options:
    """跑任务的公共选项。"""

    entry: str | None = None
    build: bool = True
    jobs: int = 1
    overwrite: bool = False


def _flags(options: Options, *, jobs: bool) -> list[str]:
    flags = ["--overwrite"] if options.overwrite else []
    if jobs and options.jobs > 1:
        flags.append(f"--jobs={options.jobs}")
    return flags


def run_task(workspace: Workspace, task: Path, overrides: list[str], options: Options) -> int:
    entry = workspace.entry(options.entry, build=options.build)
    source: list[str] = []
    if task.suffix == ".py":
        # 渲染之前的 .py 也存进任务记录（source/）：人写的是它，不是渲染出的 YAML
        source = [f"--source={task.resolve()}"]
        schema = Schema.from_tool(str(entry))
        task = render(load_python_task(task), schema,
                      workspace.build_dir / "rendered" / f"{task.stem}.yaml")
    argv = [str(entry), str(task), *overrides, *_flags(options, jobs=True), *source]
    return subprocess.run(argv).returncode


def evaluate(workspace: Workspace, run_dir: Path, overrides: list[str], options: Options) -> int:
    """单独评测一份任务记录：入口的 ``evaluate`` 读任务记录、写结论，覆盖项只能改评测一节。
    要用登记了这些评测函数的入口。"""
    entry = workspace.entry(options.entry, build=options.build)
    return subprocess.run([str(entry), "evaluate", str(run_dir), *overrides]).returncode


def evaluate_trajectories(workspace: Workspace, estimate: str, reference: str,
                          flags: list[str], options: Options) -> int:
    """评测别的系统跑出来的轨迹：交给 ``simpleslam-eval``，选项原样转交。"""
    tool = workspace.entry(EVAL_TOOL, build=options.build)
    return subprocess.run([str(tool), estimate, reference, *flags]).returncode


def demo(workspace: Workspace, options: Options) -> int:
    entry = workspace.entry(options.entry or DEFAULT_ENTRY, build=options.build)
    fixture = workspace.build_dir / "demo" / "fixture"
    sequence = fixture / "odometry" / "sequences" / "00"
    if not sequence.is_dir():
        script = workspace.root / "scripts" / "make_kitti_fixture.py"
        subprocess.run([sys.executable, str(script), "--out", str(fixture), "--frames", "40"],
                       check=True)
    run_dir = workspace.root / "runs" / "demo"
    task = workspace.root / "configs" / "tasks" / "kitti_odometry.yaml"
    argv = [str(entry), str(task), assignment("data.source.path", str(sequence)),
            assignment("outputs.dir", str(run_dir)), "--overwrite"]
    code = subprocess.run(argv).returncode
    if (run_dir / "result.json").is_file():
        print(describe(run_dir))
    return code


def _run_parallel(commands: list[tuple[str, list[str], Path]], jobs: int) -> dict[str, int]:
    """至多 jobs 个子进程同时跑，各自的输出写进日志。"""
    pending = list(commands)
    running: dict[subprocess.Popen, tuple[str, object]] = {}
    codes: dict[str, int] = {}
    while pending or running:
        while pending and len(running) < max(1, jobs):
            name, argv, log = pending.pop(0)
            log.parent.mkdir(parents=True, exist_ok=True)
            handle = open(log, "w", encoding="utf-8")  # noqa: SIM115：子进程结束才关
            process = subprocess.Popen(argv, stdout=handle, stderr=subprocess.STDOUT)
            running[process] = (name, handle)
            print(f"{name} 开始，日志 {log}", flush=True)
        done = next((process for process in running if process.poll() is not None), None)
        if done is None:
            time.sleep(0.05)
            continue
        name, handle = running.pop(done)
        handle.close()
        codes[name] = done.returncode
        print(f"{name} 结束，退出码 {done.returncode}", flush=True)
    return codes


def sweep(workspace: Workspace, task: Path, overrides: list[str], options: Options,
          out: Path | None = None) -> int:
    entry = workspace.entry(options.entry, build=options.build)
    schema = Schema.from_tool(str(entry))
    variants = load_python_sweep(task)
    base = out if out is not None else workspace.root / "runs" / task.stem
    commands = []
    for variant in variants:
        name = variant["task.name"]
        rendered = render(variant, schema,
                          workspace.build_dir / "rendered" / task.stem / f"{name}.yaml")
        # 扫参决定任务记录放在哪：一个变体一份，都在 <base> 下
        argv = [str(entry), str(rendered), *overrides,
                assignment("outputs.dir", str(base / name)), *_flags(options, jobs=False),
                f"--source={task.resolve()}"]
        commands.append((name, argv, base / "logs" / f"{name}.log"))
    codes = _run_parallel(commands, options.jobs)
    items = []
    for name, _, log in commands:
        try:
            items.extend(rows(base / name))
        except ResultError:
            items.append(Row(name=name, exit_code=codes[name], complete=None,
                             problem=f"见日志 {log}"))
    print(table(items))
    return worst_exit_code(list(codes.values()))


def show(run_dir: Path, metrics: list[str] | None = None) -> str:
    return describe(run_dir, metrics)


def compare(run_dirs: list[Path], *, as_csv: bool = False,
            metrics: list[str] | None = None) -> str:
    """几份任务记录的对比表；多序列的每个序列一行。列默认取评测函数标了短名的那几项，
    ``metrics`` 给出全名时按给的挑。``as_csv`` 时给出 CSV。"""
    items: list[Row] = []
    for run_dir in run_dirs:
        try:
            items.extend(rows(run_dir))
        except ResultError as exc:
            items.append(Row(name=str(run_dir), exit_code=1, complete=None, problem=str(exc)))
    return csv_table(items, metrics) if as_csv else table(items, metrics=metrics)


def impls(workspace: Workspace, options: Options) -> str:
    entry = workspace.entry(options.entry, build=options.build)
    schema = Schema.from_tool(str(entry))
    used_by: dict[str, list[str]] = {}
    for path, interface in schema.slots.items():
        used_by.setdefault(interface, []).append(path)
    lines = [f"入口: {entry}"]
    for interface in schema.interfaces():
        where = "、".join(sorted(used_by.get(interface, []))) or "作为子槽"
        lines.append(f"\n{interface}（{where}）")
        for type_name in schema.implementations(interface):
            lines.append(f"  {type_name}")
            plugin = schema.plugin(interface, type_name)
            for field in plugin.fields if plugin else ():
                default = field.default if field.default != "" else '""'
                lines.append(f"    {field.name:28} {field.kind:14} 默认={default:<12} {field.doc}")
    return "\n".join(lines) + "\n"
