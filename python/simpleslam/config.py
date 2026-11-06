"""任务配置：用 Python 写任务，渲染成 C++ 能加载的 YAML。

一个任务是"用哪个 SLAM 系统、在线还是离线、处理哪份数据、评测不评测、怎么评测"。
``Config`` 对应一份任务文件，键是点分路径，和 YAML 一一对应::

    from simpleslam import Config

    config = Config(
        task={"name": "kitti-00"},
        system="../systems/my_lio.yaml",  # 引用系统文件；也可以按节写出整个系统
        data={"source": {"type": "kitti_odometry", "path": "/data/kitti/odometry/sequences/00"}},
    )
    config.override("system.modules.odometry.plugin.keyframe_distance_m", 2.0)

带 ``type`` 的映射是插件节点（选一个实现，其余键是它的参数）。评测函数是一张带名字的
插件节点表（``evaluation.evaluators``），整张写或按 ``evaluation.evaluators.<名字>``
一项一项写都行。系统写在文件里时，
改系统参数只能用覆盖项，和命令行同一写法。渲染：
``python -m simpleslam config render task.py -o task.yaml``。

Python 只写显式设置过的项，按 C++ 导出的 schema 核对键名与类型，免得扫参时几个子进程
一起报同一个拼写错误。读系统文件、应用覆盖项、还原插件节点、补默认值都在 C++ 一处：
要完整的任务，交给入口的 ``dump``（:meth:`Config.dump`、:meth:`Config.load`、
:meth:`Config.through_cpp`）。
"""

from __future__ import annotations

import copy
import math
import os
import re
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Iterator, Mapping

from simpleslam.schema import Field, Schema, find_tool

# 任务文件里不是配置字段的两个键：引用的系统文件、覆盖项。C++ 把它们记作出处
SYSTEM_FILE = "system"
OVERRIDES = "overrides"

_INT64_MIN = -(2**63)
_INT64_MAX = 2**63 - 1


class ConfigError(ValueError):
    """配置项不存在、类型不符，或 YAML 结构不对。"""


# ── 值的文本：与 C++ 的 quoteYaml / canonicalText 逐字符一致，规范化导出依赖它们 ──


def _quote_yaml(text: str) -> str:
    """字符串一律加引号转义。不加引号是不可逆的：``name: "a # b"`` 往返后只剩 ``a``。"""
    out = ['"']
    for char in text:
        if char == '"':
            out.append('\\"')
        elif char == "\\":
            out.append("\\\\")
        elif char == "\n":
            out.append("\\n")
        elif char == "\r":
            out.append("\\r")
        elif char == "\t":
            out.append("\\t")
        else:
            out.append(char)
    out.append('"')
    return "".join(out)


def _format_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return "%.17g" % value
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_quote_yaml(str(item)) for item in value) + "]"
    return _quote_yaml(str(value))


def _flow_value(value: Any) -> str:
    """覆盖项里的值：标量与列表同上，插件节点写成行内映射。"""
    if isinstance(value, Mapping):
        keys = sorted(value, key=lambda key: (key != "type", key))
        return "{" + ", ".join(f"{key}: {_flow_value(value[key])}" for key in keys) + "}"
    return _format_value(value)


def assignment(path: str, value: Any) -> str:
    """一条覆盖项：``路径=值``，值按 YAML 的写法给出（字符串加引号转义）。
    任务文件里的 ``overrides`` 与命令行用的都是这种写法。"""
    return f"{path}={_flow_value(value)}"


# ── 类型检查：与 C++ 同一口径，不做隐式转换；实数字段接受整数（按实数存） ──


def _require_int64(value: int, path: str) -> int:
    if not _INT64_MIN <= value <= _INT64_MAX:
        raise ConfigError(f"{path}：{value} 超出 64 位整数的范围")
    return value


_ENTRY_NAME = re.compile(r"[A-Za-z0-9_]+")


def _check_entry_name(table: str, name: str) -> None:
    """表里的名字只收字母、数字与 _，与 C++ 的 validateConfig 一致。"""
    if not _ENTRY_NAME.fullmatch(name):
        raise ConfigError(f"{table} 里的名字 {name} 只能用字母、数字与 _："
                          "它是指标名的第一段，也是表格的文件名")


def _coerce_kind(kind: str, value: Any, path: str) -> Any:
    """配置字段的值（bool / int / double / string / list / slot / slots）。"""
    if kind == "bool":
        if not isinstance(value, bool):
            raise ConfigError(f"{path} 需要 bool，得到 {value!r}")
        return value
    if kind == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{path} 需要 int，得到 {value!r}")
        return _require_int64(value, path)
    if kind == "double":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"{path} 需要实数，得到 {value!r}")
        if not math.isfinite(value):
            raise ConfigError(f"{path} 必须是有限实数")
        return float(value)
    if kind == "list":
        if not isinstance(value, (list, tuple)) or not all(isinstance(v, str) for v in value):
            raise ConfigError(f"{path} 需要字符串列表，得到 {value!r}")
        return list(value)
    if kind == "slot":
        if not isinstance(value, Mapping):
            raise ConfigError(f"{path} 是插件节点，要写成映射，至少写出 type")
        return _copy_plugin(value, path)
    if kind == "slots":
        if not isinstance(value, Mapping):
            raise ConfigError(f"{path} 是带名字的插件节点表，要写成 {{名字: 插件节点}}")
        out: dict[str, Any] = {}
        for name, node in value.items():
            _check_entry_name(path, name)
            out[name] = _coerce_kind("slot", node, f"{path}.{name}")
        return out
    if not isinstance(value, str):
        raise ConfigError(f"{path} 需要字符串，得到 {value!r}")
    return value


def _copy_plugin(node: Mapping[str, Any], path: str) -> dict[str, Any]:
    """插件节点的深拷贝与结构检查：type 是字符串，参数是标量，子节点是映射。"""
    out: dict[str, Any] = {}
    for key, value in node.items():
        sub = f"{path}.{key}"
        if key == "type":
            if not isinstance(value, str):
                raise ConfigError(f"{sub} 要写成实现的类型名")
            out[key] = value
        elif isinstance(value, Mapping):
            out[key] = _copy_plugin(value, sub)
        elif isinstance(value, (bool, int, float, str)):
            out[key] = value
        else:
            raise ConfigError(f"{sub} 插件参数只能是标量或子节点，得到 {value!r}")
    return out


# ── 渲染：嵌套、按字段顺序、节与节之间空一行 ──


def _emit_plugin(lines: list[str], node: Mapping[str, Any], indent: int) -> None:
    """type 在前，参数与子节点各按键名排序。"""
    pad = " " * indent
    lines.append(f"{pad}type: {_quote_yaml(node.get('type', ''))}")
    params = sorted(k for k, v in node.items() if k != "type" and not isinstance(v, Mapping))
    children = sorted(k for k, v in node.items() if isinstance(v, Mapping))
    for key in params:
        lines.append(f"{pad}{key}: {_format_value(node[key])}")
    for key in children:
        lines.append(f"{pad}{key}:")
        _emit_plugin(lines, node[key], indent + 2)


def _emit(values: Mapping[str, Any], fields: list[Field]) -> str:
    lines: list[str] = []
    open_parts: list[str] = []
    for field in fields:
        if field.path not in values:
            continue
        parts = field.path.split(".")
        key, parents = parts[-1], parts[:-1]
        common = 0
        while common < min(len(open_parts), len(parents)) and open_parts[common] == parents[common]:
            common += 1
        del open_parts[common:]
        if common == 0 and lines:
            lines.append("")  # 新的顶层节（或顶层键）前空一行
        for index in range(common, len(parents)):
            lines.append("  " * index + parents[index] + ":")
            open_parts.append(parents[index])
        pad = "  " * len(parents)
        value = values[field.path]
        if field.type == "slot":
            lines.append(f"{pad}{key}:")
            _emit_plugin(lines, value, len(pad) + 2)
        elif field.type == "slots":
            # 空表写成 {}；否则按名字排序，每项一个插件节点（与 C++ 的 dump 一致）
            lines.append(f"{pad}{key}:" + ("" if value else " {}"))
            for name in sorted(value):
                lines.append(f"{pad}  {name}:")
                _emit_plugin(lines, value[name], len(pad) + 4)
        else:
            lines.append(f"{pad}{key}: {_format_value(value)}")
    return "\n".join(lines) + "\n" if lines else ""



def _flatten(prefix: str, values: Mapping[str, Any], out: dict[str, Any]) -> None:
    """按节写的映射展开成点分路径。带 type 的映射是插件节点，整体作为一个值。"""
    for key, value in values.items():
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(value, Mapping) and "type" not in value:
            _flatten(path, value, out)
        else:
            out[path] = value


def _slot_containing(path: str, schema: Schema) -> str | None:
    """路径落在哪个插件字段里面（不含插件字段本身）。"""
    for field in schema:
        if field.type in ("slot", "slots") and path.startswith(field.path + "."):
            return field.path
    return None


def _table_of(path: str, schema: Schema) -> str | None:
    """路径是不是插件节点表里的一项（``<表>.<名字>``），是就返回表的路径。"""
    table, dot, _ = path.rpartition(".")
    if dot and table in schema and schema.field(table).type == "slots":
        return table
    return None


def _merged_tables(values: Mapping[str, Any], schema: Schema) -> dict[str, Any]:
    """插件节点表的单项（``<表>.<名字>``）并进整张表：先整张表，再逐项，后写的覆盖先写的，
    与 C++ 应用覆盖项的顺序一致。按节写的 ``evaluation={"evaluators": {...}}`` 展开后
    就是单项，所以两种写法得到同一张表。"""
    out: dict[str, Any] = {}
    for path, value in values.items():
        table = _table_of(path, schema) if path not in schema else None
        if table is None:
            out[path] = value
    for path, value in values.items():
        table = _table_of(path, schema) if path not in schema else None
        if table is not None:
            entries = dict(out.get(table) or {})
            entries[path[len(table) + 1:]] = value
            out[table] = entries
    return out


class Config:
    """一份任务文件。只保存显式给出的项；默认值由 schema 提供。"""

    def __init__(self, values: Mapping[str, Any] | None = None, **sections: Any) -> None:
        self._values: dict[str, Any] = {}
        self.update(values, **sections)

    # ── 读写 ──

    def __setitem__(self, path: str, value: Any) -> None:
        if not path:
            raise ConfigError("配置项路径不能为空")
        # 深拷贝：插件节点与列表是可变的，变体之间不能共用同一个对象
        self._values[path] = copy.deepcopy(value)

    def __getitem__(self, path: str) -> Any:
        """返回存着的那个值：插件节点可以就地改，``config[槽]["参数"] = …``。"""
        return self._values[path]

    def __delitem__(self, path: str) -> None:
        del self._values[path]

    def __contains__(self, path: object) -> bool:
        return path in self._values

    def __iter__(self) -> Iterator[str]:
        return iter(self._values)

    def __len__(self) -> int:
        return len(self._values)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Config) and self._values == other._values

    def __repr__(self) -> str:
        items = ", ".join(f"{k!r}: {v!r}" for k, v in sorted(self._values.items()))
        return f"Config({{{items}}})"

    def get(self, path: str, default: Any = None) -> Any:
        return self._values.get(path, default)

    def update(self, values: Mapping[str, Any] | None = None, **sections: Any) -> "Config":
        """按点分路径或按节更新，两种写法可以混用：

        ``update({"system.runtime.spin_budget": 40})`` 与
        ``update(system={"runtime": {"spin_budget": 40}})`` 等价。
        """
        flat: dict[str, Any] = {}
        _flatten("", values or {}, flat)
        _flatten("", sections, flat)
        for path, value in flat.items():
            self[path] = value
        return self

    def override(self, path: str, value: Any) -> "Config":
        """追加一条覆盖项，写法与命令行相同（``路径=值``）。

        系统写在文件里时，改系统参数只能这样写：``system`` 下的键会被整份系统文件替换。
        插件内部的参数也可以：``system.modules.odometry.plugin.keyframe_distance_m``。
        """
        overrides = list(self._values.get(OVERRIDES, []))
        overrides.append(assignment(path, value))
        self._values[OVERRIDES] = overrides
        return self

    def copy(self) -> "Config":
        """深拷贝：变体改自己的插件节点，不会改到基底。"""
        clone = Config()
        clone._values = copy.deepcopy(self._values)
        return clone

    def as_dict(self) -> dict[str, Any]:
        return copy.deepcopy(self._values)

    # ── 校验 ──

    def validate(self, schema: Schema) -> "Config":
        """未知项与类型不符在这里报出来，不等到 C++ 侧才发现。
        插件节点的参数对不对、取值是否有意义（枚举、范围），由 C++ 在加载时查，
        Python 不另写一份。"""
        system_file = self._values.get(SYSTEM_FILE)
        if system_file is not None and (not isinstance(system_file, str) or not system_file):
            raise ConfigError("system 要写成系统文件的路径（或按节写出 system.* 各项）")
        overrides = self._values.get(OVERRIDES, [])
        if not isinstance(overrides, list) or not all(isinstance(o, str) for o in overrides):
            raise ConfigError("overrides 要写成 路径=值 的字符串列表")
        for item in overrides:
            path, equals, _ = item.partition("=")
            if not equals or not path:
                raise ConfigError(f"覆盖项要写成 路径=值，得到 {item}")
            if path not in schema and _slot_containing(path, schema) is None:
                raise ConfigError(f"覆盖项里的未知配置项 {path}")
        for path, value in self._values.items():
            if path in (SYSTEM_FILE, OVERRIDES):
                continue
            if system_file is not None and path.startswith("system."):
                raise ConfigError(f"{path}：system 引用了系统文件 {system_file}，"
                                  "改系统参数要写成覆盖项（config.override）")
            table = _table_of(path, schema) if path not in schema else None
            if table is not None:  # 表里的一项：名字合规，值是插件节点
                _check_entry_name(table, path[len(table) + 1:])
                _coerce_kind("slot", value, path)
                continue
            if path not in schema:
                slot = _slot_containing(path, schema)
                if slot is not None:
                    raise ConfigError(f"{path} 在插件节点 {slot} 里面：整体写出这个节点"
                                      "（带 type），或用 config.override")
                raise ConfigError(f"未知配置项 {path}")
            _coerce_kind(schema.field(path).type, value, path)
        return self

    # ── 渲染 ──

    def to_yaml(self, schema: Schema | None = None) -> str:
        """渲染成任务 YAML，只写显式设置过的项：任务文件因此只说"我改了什么"，而不是把
        几十个默认值抄一遍。完整的任务由 C++ 导出（:meth:`dump`）。没有 schema 时不校验，
        按路径排序输出。
        """
        order = self._render_order(schema)
        values = self._values if schema is None else _merged_tables(self._values, schema)
        return _emit(values, order)

    def _render_order(self, schema: Schema | None) -> list[Field]:
        def pseudo(path: str, value: Any) -> Field:
            kind = "slot" if isinstance(value, Mapping) else "string"
            return Field(path, kind, "", "", False)

        if schema is None:
            return [pseudo(path, value) for path, value in sorted(self._values.items())]
        self.validate(schema)
        order: list[Field] = []
        for field in schema:
            # 引用的系统文件放在 system 节本来的位置上
            if field.path.startswith("system.") and SYSTEM_FILE in self._values:
                if not order or order[-1].path != SYSTEM_FILE:
                    order.append(pseudo(SYSTEM_FILE, ""))
                continue
            order.append(field)
        if OVERRIDES in self._values:
            order.append(pseudo(OVERRIDES, ""))
        return order

    # ── 从 YAML 读回来 ──

    @classmethod
    def from_yaml(cls, text: str, schema: Schema) -> "Config":
        """解析任务 YAML 的受限子集：两格缩进的块映射、双引号或不加引号的标量、
        行内列表、空映射 ``{}``、``#`` 注释。

        C++ 的 dump 与 :meth:`to_yaml` 都在这个子集里。
        别的写法交给 C++：见 :meth:`load`。插件节点从哪一层开始只有 schema 知道，
        所以这里要求 schema。
        """
        if schema is None:
            raise ConfigError("读任务 YAML 需要 schema：插件节点从哪一层开始只有 schema 知道")
        slot_paths = {field.path for field in schema if field.type == "slot"}
        table_paths = {field.path for field in schema if field.type == "slots"}
        config = cls()

        def walk(node: Mapping[str, Any], prefix: str) -> None:
            for key, value in node.items():
                path = f"{prefix}.{key}" if prefix else key
                if path == SYSTEM_FILE and not isinstance(value, Mapping):
                    config[path] = _unquote_scalar(value)
                elif path == OVERRIDES:
                    config[path] = _parse_list(value, path)
                elif path in slot_paths:
                    if not isinstance(value, Mapping):
                        raise ConfigError(f"{path} 是插件节点，要写成映射")
                    config[path] = _plugin_values(value)
                elif path in table_paths:
                    if not isinstance(value, Mapping) or not all(
                            isinstance(node, Mapping) for node in value.values()):
                        raise ConfigError(f"{path} 是带名字的插件节点表，每项要写成映射")
                    config[path] = {name: _plugin_values(node) for name, node in value.items()}
                elif isinstance(value, Mapping) and path not in schema:
                    walk(value, path)
                else:
                    config[path] = _typed_value(value, path, schema)

        walk(_parse_block_yaml(text), "")
        config.validate(schema)
        return config

    @classmethod
    def load(cls, path: str | os.PathLike[str], tool: str | None = None,
             schema: Schema | None = None) -> "Config":
        """任意写法的任务文件：交给 C++ 加载（读系统文件、应用覆盖项、还原插件节点），
        导出完整的任务再读回来。得到的是完整、自足的任务。

        ``tool`` 是登记了所需实现的入口，见 :func:`simpleslam.schema.find_tool`。
        """
        binary = find_tool(tool)
        if schema is None:
            schema = Schema.from_tool(binary)
        return cls.from_yaml(_dump(binary, Path(path)), schema)

    def dump(self, base_dir: str | os.PathLike[str] = ".", tool: str | None = None,
             schema: Schema | None = None) -> str:
        """C++ 导出的完整任务：系统展开、覆盖项应用、插件节点还原、默认值补齐。
        要知道两份任务有什么不同，就比它。

        ``base_dir`` 是 ``system`` 里相对路径的基准，通常是任务 ``.py`` 所在的目录。
        """
        rendered = self.copy()
        system_file = rendered.get(SYSTEM_FILE)
        if isinstance(system_file, str):
            rendered[SYSTEM_FILE] = os.path.abspath(os.path.join(base_dir, system_file))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.yaml"
            path.write_text(rendered.to_yaml(schema), encoding="utf-8")
            return _dump(find_tool(tool), path)

    def through_cpp(self, base_dir: str | os.PathLike[str] = ".", tool: str | None = None,
                    schema: Schema | None = None) -> "Config":
        """让 C++ 读系统文件、应用覆盖项、还原插件节点，得到完整、自足的任务。"""
        binary = find_tool(tool)
        if schema is None:
            schema = Schema.from_tool(binary)
        return Config.from_yaml(self.dump(base_dir, binary, schema), schema)


def dump_file(path: str | os.PathLike[str], tool: str | None = None) -> str:
    """任务文件经 C++ 展开后的完整任务（``<入口> dump``）。"""
    return _dump(find_tool(tool), Path(path))


def _dump(binary: str, path: Path) -> str:
    """``<入口> dump <任务>``：C++ 读这份任务，导出完整的任务。"""
    try:
        result = subprocess.run([binary, "dump", str(path)], capture_output=True, text=True,
                                check=True)
    except subprocess.CalledProcessError as exc:
        raise ConfigError(f"{binary} dump {path} 失败: {exc.stderr.strip()}") from exc
    except OSError as exc:
        raise ConfigError(f"调用 {binary} 失败: {exc}") from exc
    return result.stdout


# ── 受限 YAML 子集的解析 ──


def _strip_comment(line: str) -> str:
    """去掉行尾注释：``#`` 在行首或空白之后、且不在引号里，才是注释。

    ``name: "a # b"`` 被朴素地按 ``#`` 切一刀就只剩 ``name: "a ``，往返因此丢内容。
    """
    in_quotes = False
    index = 0
    while index < len(line):
        char = line[index]
        if in_quotes:
            if char == "\\":
                index += 2
                continue
            if char == '"':
                in_quotes = False
        elif char == '"':
            in_quotes = True
        elif char == "#" and (index == 0 or line[index - 1] in " \t"):
            return line[:index]
        index += 1
    return line


def _unquote_yaml(text: str) -> str:
    """_quote_yaml 的逆。只认它会产生的那几种转义。"""
    body = text[1:-1]
    out: list[str] = []
    index = 0
    while index < len(body):
        char = body[index]
        if char != "\\":
            out.append(char)
            index += 1
            continue
        index += 1
        if index >= len(body):
            break
        escaped = body[index]
        out.append({"n": "\n", "r": "\r", "t": "\t"}.get(escaped, escaped))
        index += 1
    return "".join(out)


def _is_quoted(text: str) -> bool:
    return len(text) >= 2 and text.startswith('"') and text.endswith('"')


def _unquote_scalar(text: str) -> str:
    return _unquote_yaml(text) if _is_quoted(text) else text


def _parse_block_yaml(text: str) -> dict[str, Any]:
    """两格一层的块映射。值是原样的标量文本、行内列表的文本，或空映射。"""
    root: dict[str, Any] = {}
    stack: list[tuple[int, dict[str, Any]]] = [(-1, root)]
    for number, raw in enumerate(text.splitlines(), start=1):
        line = _strip_comment(raw).rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        if indent % 2 != 0:
            raise ConfigError(f"第 {number} 行：缩进要是两格的整数倍")
        stripped = line.strip()
        if stripped.startswith("- "):
            raise ConfigError(f"第 {number} 行：块序列不在受限子集里，写成行内列表，"
                              "或交给 Config.load")
        key, sep, value = stripped.partition(":")
        if not sep:
            raise ConfigError(f"第 {number} 行：不是 键: 值")
        key = key.strip()
        value = value.strip()
        while stack[-1][0] >= indent:
            stack.pop()
        parent = stack[-1][1]
        if key in parent:
            raise ConfigError(f"第 {number} 行：键 {key} 重复")
        if not value:
            child: dict[str, Any] = {}
            parent[key] = child
            stack.append((indent, child))
        elif value == "{}":
            parent[key] = {}  # 空映射：规范化导出里的空插件节点表
        elif value.startswith("{"):
            raise ConfigError(f"第 {number} 行：行内映射不在受限子集里，交给 Config.load")
        elif value.startswith("'"):
            raise ConfigError(f"第 {number} 行：单引号字符串不在受限子集里，改用双引号")
        else:
            parent[key] = value
    return root


_INT_TEXT = re.compile(r"-?[0-9]+")
_REAL_TEXT = re.compile(r"-?(?:[0-9]+\.?[0-9]*|\.[0-9]+)(?:[eE][+-]?[0-9]+)?")


def _scalar(text: str) -> Any:
    """插件参数的标量，与 C++ 的 scalarValue 一致：带引号是字符串，true/false 是布尔，
    能完整读成 64 位整数的是整数，能完整读成实数的是实数，其余是字符串。"""
    if _is_quoted(text) or not text:
        return _unquote_scalar(text)
    if text in ("true", "false"):
        return text == "true"
    body = text[1:] if text.startswith("+") else text
    if _INT_TEXT.fullmatch(body):
        integer = int(body)
        if _INT64_MIN <= integer <= _INT64_MAX:
            return integer  # 超出范围时 C++ 的 from_chars 失败，接着按实数读
    if _REAL_TEXT.fullmatch(body):
        real = float(body)
        mantissa = re.split("[eE]", body)[0]
        # from_chars 对上溢、下溢报 result_out_of_range，C++ 因此把它当字符串
        if not math.isinf(real) and (real != 0.0 or not re.search("[1-9]", mantissa)):
            return real
    return text


def _plugin_values(node: Mapping[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in node.items():
        if isinstance(value, Mapping):
            out[key] = _plugin_values(value)
        elif key == "type":
            out[key] = _unquote_scalar(value)  # 类型名总是字符串，与 C++ 一致
        else:
            out[key] = _scalar(value)
    return out


def _parse_list(text: Any, path: str) -> list[str]:
    """行内列表：元素是双引号字符串或不加引号的文本。"""
    if not isinstance(text, str) or not (text.startswith("[") and text.endswith("]")):
        raise ConfigError(f"{path} 需要写成行内列表 [a, b]，得到 {text!r}")
    body = text[1:-1]
    items: list[str] = []
    index = 0
    while True:
        while index < len(body) and body[index] in " \t":
            index += 1
        if index >= len(body):
            return items
        if body[index] == '"':
            end = index + 1
            while end < len(body) and body[end] != '"':
                end += 2 if body[end] == "\\" else 1
            if end >= len(body):
                raise ConfigError(f"{path}：列表里的引号没有闭合")
            items.append(_unquote_yaml(body[index:end + 1]))
            index = end + 1
        else:
            end = body.find(",", index)
            end = len(body) if end < 0 else end
            items.append(body[index:end].strip())
            index = end
        while index < len(body) and body[index] in " \t":
            index += 1
        if index < len(body):
            if body[index] != ",":
                raise ConfigError(f"{path}：列表元素之间要用逗号分隔")
            index += 1


def _typed_value(text: Any, path: str, schema: Schema) -> Any:
    """按 schema 声明的类型解析。带引号的只能是字符串，与 C++ 一致。"""
    if path not in schema:
        slot = _slot_containing(path, schema)
        if slot is not None:
            raise ConfigError(f"{path} 在插件节点 {slot} 里面")
        raise ConfigError(f"未知配置项 {path}")
    field = schema.field(path)
    if isinstance(text, Mapping):
        raise ConfigError(f"{path} 不是一个节")
    if field.type == "list":
        return _parse_list(text, path)
    if _is_quoted(text):
        if field.type != "string":
            raise ConfigError(f"{path} 需要 {field.type}，得到带引号的字符串")
        return _unquote_yaml(text)
    try:
        return field.parse(text)
    except ValueError as exc:
        raise ConfigError(str(exc)) from exc
