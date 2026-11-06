"""任务档案的摘要：读 C++ 执行器写的 ``result.json``，整理成表。

Python 不解析档案里的 YAML，也不重新判定任何东西：结论、汇总表取哪几项、什么时候算没有
结果，都是 C++ 写进 ``result.json`` 的。这里只负责显示与对比。
"""

from __future__ import annotations

import csv
import io
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

# 汇总表的列：与 C++ 的 archive::kSummaryColumns 相同的表头。只是显示用的名字，不是规则
HEADERS = {
    "ate.translation.rmse": "ATE_t",
    "ate.rotation_deg.rmse": "ATE_r",
    "rpe.translation.rmse": "RPE_t",
    "rpe.rotation_deg.rmse": "RPE_r",
}

# 读得懂的 result.json 格式版本（C++ 的 archive::kResultSchema）
RESULT_SCHEMA = 1

# 退出码的严重程度：出错 > 无效 > 不可比较 > 失败 > 通过，与 C++ 的多序列汇总同一口径
_SEVERITY = {0: 0, 3: 1, 5: 2, 4: 3}


class ResultError(RuntimeError):
    """档案里没有可读的摘要。"""


@dataclass
class Row:
    """表里的一行：一次运行，或多序列里的一个序列。"""

    name: str
    status: str
    exit_code: int
    complete: bool | None
    metrics: dict[str, float] = field(default_factory=dict)
    system_hash: str = ""
    problem: str = ""


def read_result(archive: Path) -> dict:
    path = Path(archive) / "result.json"
    if not path.is_file():
        raise ResultError(f"{archive} 里没有 result.json：运行没有走到最后，或者这不是任务档案")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ResultError(f"{path} 读不懂: {exc}") from exc
    if data.get("schema") != RESULT_SCHEMA:
        raise ResultError(f"{path} 的格式版本是 {data.get('schema')}，"
                          f"这里只读得懂 {RESULT_SCHEMA}：换用写出它的那个版本的操作台")
    return data


def rows(archive: Path) -> list[Row]:
    """一个档案的行：单次运行一行，多序列每个序列一行。"""
    data = read_result(archive)
    if data["kind"] == "sequences":
        return [
            Row(name=f"{data['task']}/{item['name']}", status=item["status"],
                exit_code=item["exit_code"], complete=item["complete"],
                metrics=_numbers(item["metrics"]), system_hash=data["system_hash"],
                problem=item.get("problem", ""))
            for item in data["sequences"]
        ]
    return [Row(name=data["task"], status=data["status"], exit_code=data["exit_code"],
                complete=data["complete"], metrics=_numbers(data["summary"]),
                system_hash=data["system_hash"], problem=data.get("detail", ""))]


def _numbers(metrics: dict) -> dict[str, float]:
    # C++ 把非有限值写成 null：显示成"没有结果"，不当零
    return {key: value for key, value in metrics.items()
            if isinstance(value, (int, float)) and math.isfinite(value)}


def worst_exit_code(codes: list[int]) -> int:
    worst = 0
    for code in codes:
        rank = _SEVERITY.get(code, 4)
        if rank > _SEVERITY.get(worst, 4):
            worst = 1 if rank == 4 else code
    return worst


def _width(text: str) -> int:
    """终端里的显示宽度：汉字占两列。"""
    return sum(2 if ord(char) >= 0x1100 else 1 for char in text)


def _pad(text: str, width: int) -> str:
    return text + " " * max(1, width - _width(text))


def table(items: list[Row], mean: dict[str, float] | None = None) -> str:
    """对比表：名称、结论、退出码、完整与否、汇总指标、系统哈希。没有值的格子写 -。"""
    columns = [key for key in HEADERS if any(key in row.metrics for row in items)] or list(HEADERS)
    name_width = max([_width("名称")] + [_width(row.name) for row in items]) + 2
    lines = [_pad("名称", name_width) + _pad("结论", 14) + _pad("退出码", 8) + _pad("完整", 6)
             + "".join(_pad(HEADERS[key], 12) for key in columns) + "系统哈希"]
    for row in items:
        complete = "-" if row.complete is None else ("是" if row.complete else "否")
        cells = "".join(_pad(_cell(row.metrics, key), 12) for key in columns)
        lines.append(_pad(row.name, name_width) + _pad(row.status, 14)
                     + _pad(str(row.exit_code), 8) + _pad(complete, 6) + cells
                     + row.system_hash[:8])
    if mean is not None:
        cells = "".join(_pad(_cell(mean, key), 12) for key in columns)
        lines.append(_pad("平均（每个序列都有结果才算）", name_width + 28) + cells)
    return "\n".join(lines) + "\n"


def csv_table(items: list[Row]) -> str:
    """对比表的 CSV：一行一次运行或一个序列。列名用指标的全名，数值写成往返无损的形式，
    没有结果的格子留空，不写零。"""
    columns = [key for key in HEADERS if any(key in row.metrics for row in items)] or list(HEADERS)
    buffer = io.StringIO()
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(["name", "status", "exit_code", "complete", *columns, "system_hash"])
    for row in items:
        complete = "" if row.complete is None else ("true" if row.complete else "false")
        values = [repr(row.metrics[key]) if key in row.metrics else "" for key in columns]
        writer.writerow([row.name, row.status, row.exit_code, complete, *values, row.system_hash])
    return buffer.getvalue()


def _cell(metrics: dict[str, float], key: str) -> str:
    return f"{metrics[key]:.4f}" if key in metrics else "-"


def describe(archive: Path) -> str:
    """一个档案的摘要：身份、结论、汇总表；出错的序列写明原因。"""
    data = read_result(archive)
    head = (f"档案: {archive}\n任务: {data['task']}  运行方式: {data['mode']}  "
            f"目的: {data['purpose']}  版本: {data['software_version']}\n"
            f"任务哈希: {data['task_hash']}  系统哈希: {data['system_hash']}  "
            f"退出码: {data['exit_code']}\n")
    items = rows(archive)
    mean = _numbers(data["mean"]) if data["kind"] == "sequences" else None
    problems = "".join(f"  {row.name}: {row.problem}\n" for row in items
                       if row.problem and row.status in ("error", "invalid", "fail"))
    return head + table(items, mean) + (f"问题:\n{problems}" if problems else "")
