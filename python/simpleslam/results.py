"""任务记录的结论：读 C++ 写的 ``result.json``（格式版本 2），整理成表。

Python 不解析任务记录里的 YAML，也不重新判定任何东西：结论、哪些量进汇总表（评测函数
标了短名的那几项）、什么时候算没有结果，都是 C++ 写进 ``result.json`` 的。这里只负责
显示与对比；逐样本的数据在任务记录的 ``evaluation/`` 里，画图直接读那些 CSV。
"""

from __future__ import annotations

import csv
import io
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

# 读得懂的 result.json 格式版本（C++ 的 run_dir::kResultSchema）
RESULT_SCHEMA = 2

# 退出码的严重程度：出错 > 无效 > 不可比较 > 失败 > 通过，与 C++ 的多序列汇总同一口径
_SEVERITY = {0: 0, 3: 1, 5: 2, 4: 3}


class ResultError(RuntimeError):
    """任务记录里没有可读的结论。"""


@dataclass(frozen=True)
class Scalar:
    """一个标量结果：值、单位、进汇总表的短名（空表示不进）。"""

    value: float
    unit: str = ""
    label: str = ""


@dataclass
class Row:
    """表里的一行：一次运行，或多序列里的一个序列。"""

    name: str
    status: str
    exit_code: int
    complete: bool | None
    #: 全名（``<评测函数的名字>.<量>``）→ 标量。没有结果的量不在这里，不当零
    scalars: dict[str, Scalar] = field(default_factory=dict)
    system_hash: str = ""
    problem: str = ""


def read_result(run_dir: Path) -> dict:
    path = Path(run_dir) / "result.json"
    if not path.is_file():
        raise ResultError(f"{run_dir} 里没有 result.json：运行没有走到最后，或者这不是任务记录")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ResultError(f"{path} 读不懂: {exc}") from exc
    if data.get("schema") != RESULT_SCHEMA:
        raise ResultError(f"{path} 的格式版本是 {data.get('schema')}，"
                          f"这里只读得懂 {RESULT_SCHEMA}：换用写出它的那个版本的操作台")
    return data


def _scalars(entries: dict, prefix: str = "") -> dict[str, Scalar]:
    """``{名字: {value, unit, label}}`` 读成标量。C++ 把非有限值写成 null：跳过，不当零。"""
    out: dict[str, Scalar] = {}
    for name, entry in entries.items():
        value = entry.get("value")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        if math.isfinite(value):
            out[prefix + name] = Scalar(float(value), entry.get("unit", ""), entry.get("label", ""))
    return out


def run_scalars(data: dict) -> dict[str, Scalar]:
    """单次运行的结论里各评测函数的标量，全名是 ``<评测函数的名字>.<量>``。
    与 C++ 读子任务记录（``run_dir::readResult``）同一口径。"""
    out: dict[str, Scalar] = {}
    for name, evaluation in data.get("evaluations", {}).items():
        out.update(_scalars(evaluation.get("scalars", {}), f"{name}."))
    return out


def rows(run_dir: Path) -> list[Row]:
    """一份任务记录的行：单次运行一行，多序列每个序列一行。"""
    data = read_result(run_dir)
    if data["kind"] == "sequences":
        return [
            Row(name=f"{data['task']}/{item['name']}", status=item["status"],
                exit_code=item["exit_code"], complete=item["complete"],
                scalars=_scalars(item["scalars"]), system_hash=data["system_hash"],
                problem=item.get("problem", ""))
            for item in data["sequences"]
        ]
    return [Row(name=data["task"], status=data["status"], exit_code=data["exit_code"],
                complete=data["complete"], scalars=run_scalars(data),
                system_hash=data["system_hash"], problem=data.get("detail", ""))]


def worst_exit_code(codes: list[int]) -> int:
    worst = 0
    for code in codes:
        rank = _SEVERITY.get(code, 4)
        if rank > _SEVERITY.get(worst, 4):
            worst = 1 if rank == 4 else code
    return worst


def columns(items: list[Row], metrics: list[str] | None = None) -> list[tuple[str, str]]:
    """表的列：（全名，表头）。给了 metrics 就按给的全名；否则取评测函数标了短名的那几项，
    按第一次出现的先后，与 C++ 的多序列汇总表同一口径。几项用了同一个短名（比如两个 ate
    评测函数）时，表头前面加上评测函数的名字，免得两列看着一样。"""
    if metrics:
        return [(name, name) for name in metrics]
    labels: dict[str, str] = {}
    for row in items:
        for name, scalar in row.scalars.items():
            if scalar.label and name not in labels:
                labels[name] = scalar.label
    taken = list(labels.values())
    return [(name, label if taken.count(label) == 1 else f"{name.split('.', 1)[0]}:{label}")
            for name, label in labels.items()]


def _width(text: str) -> int:
    """终端里的显示宽度：汉字占两列。"""
    return sum(2 if ord(char) >= 0x1100 else 1 for char in text)


def _pad(text: str, width: int) -> str:
    return text + " " * max(1, width - _width(text))


def _cell(scalars: dict[str, Scalar], name: str) -> str:
    return f"{scalars[name].value:.4f}" if name in scalars else "-"


def table(items: list[Row], mean: dict[str, Scalar] | None = None,
          metrics: list[str] | None = None) -> str:
    """对比表：名称、结论、退出码、完整与否、各列的量、系统哈希。没有值的格子写 -。"""
    chosen = columns(items, metrics)
    name_width = max([_width("名称")] + [_width(row.name) for row in items]) + 2
    lines = [_pad("名称", name_width) + _pad("结论", 14) + _pad("退出码", 8) + _pad("完整", 6)
             + "".join(_pad(header, 12) for _, header in chosen) + "系统哈希"]
    for row in items:
        complete = "-" if row.complete is None else ("是" if row.complete else "否")
        cells = "".join(_pad(_cell(row.scalars, name), 12) for name, _ in chosen)
        lines.append(_pad(row.name, name_width) + _pad(row.status, 14)
                     + _pad(str(row.exit_code), 8) + _pad(complete, 6) + cells
                     + row.system_hash[:8])
    if mean is not None:
        cells = "".join(_pad(_cell(mean, name), 12) for name, _ in chosen)
        lines.append(_pad("平均（每个序列都有结果才算）", name_width + 28) + cells)
    return "\n".join(lines) + "\n"


def csv_table(items: list[Row], metrics: list[str] | None = None) -> str:
    """对比表的 CSV：一行一次运行或一个序列。列名用量的全名，数值写成往返无损的形式，
    没有结果的格子留空，不写零。"""
    names = [name for name, _ in columns(items, metrics)]
    buffer = io.StringIO()
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(["name", "status", "exit_code", "complete", *names, "system_hash"])
    for row in items:
        complete = "" if row.complete is None else ("true" if row.complete else "false")
        values = [repr(row.scalars[name].value) if name in row.scalars else "" for name in names]
        writer.writerow([row.name, row.status, row.exit_code, complete, *values, row.system_hash])
    return buffer.getvalue()


def describe(run_dir: Path, metrics: list[str] | None = None) -> str:
    """一份任务记录的摘要：身份、结论、汇总表、评测的说明；出错的序列写明原因。"""
    data = read_result(run_dir)
    head = (f"任务记录: {run_dir}\n任务: {data['task']}  运行方式: {data['mode']}  "
            f"版本: {data['software_version']}\n"
            f"任务哈希: {data['task_hash']}  系统哈希: {data['system_hash']}  "
            f"退出码: {data['exit_code']}\n")
    items = rows(run_dir)
    mean = _scalars(data["mean"]) if data["kind"] == "sequences" else None
    text = head + table(items, mean, metrics)
    # 读结论的人要知道的前提：实际用的对齐方式、换算过的坐标系等
    notes = [f"  {name}: {note}" for name, evaluation in data.get("evaluations", {}).items()
             for note in evaluation.get("notes", [])]
    if notes:
        text += "评测说明:\n" + "\n".join(notes) + "\n"
    problems = "".join(f"  {row.name}: {row.problem}\n" for row in items
                       if row.problem and row.status in ("error", "invalid", "fail"))
    return text + (f"问题:\n{problems}" if problems else "")
