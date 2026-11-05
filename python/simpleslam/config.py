"""任务配置：用 Python 写任务，渲染成 C++ 能加载的 YAML。

一个任务是"用哪个 SLAM 系统、在线还是离线、处理哪份数据、做实验还是只出结果"。
``Config`` 对应一份任务文件，键是点分路径，和 YAML 一一对应::

    from simpleslam import Config

    config = Config(
        task={"name": "kitti-00"},
        system="../systems/my_lio.yaml",  # 引用系统文件；也可以按节写出整个系统
        data={"source": {"type": "kitti_odometry", "path": "/data/kitti/odometry/sequences/00"}},
    )
    config.override("system.modules.odometry.plugin.keyframe_distance_m", 2.0)

带 ``type`` 的映射是插件节点（选一个实现，其余键是它的参数）。系统写在文件里时，
改系统参数只能用覆盖项，和命令行同一写法。渲染：
``python -m simpleslam config render task.py -o task.yaml``。

规范化导出（:meth:`Config.canonical`）与哈希在 C++ 与 Python 各有一份实现，插件节点
按 schema 里各实现的字段清单还原，与 C++ 的目录同一口径。重复是刻意的——扫参时
Python 侧不该为每个变体起一次子进程——但它只在有自动比对的前提下成立：
``python/tests/test_cpp_parity.py`` 逐字节比两边的输出。读系统文件、应用覆盖项
这两件事 Python 不做，交给 C++（:meth:`Config.load`、:meth:`Config.through_cpp`）。
"""

from __future__ import annotations

import copy
import math
import os
import re
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Callable, Iterator, Mapping

from simpleslam.schema import Field, Schema, SchemaError, find_tool, round_to_float32

# 任务文件里不是配置字段的两个键：引用的系统文件、覆盖项。C++ 把它们记作出处，不进哈希
SYSTEM_FILE = "system"
OVERRIDES = "overrides"

_INT64_MIN = -(2**63)
_INT64_MAX = 2**63 - 1


class ConfigError(ValueError):
    """配置项不存在、类型不符，或 YAML 结构不对。"""


# ── 值的文本：与 C++ 的 quoteYaml / canonicalText 逐字符一致，哈希依赖它们 ──


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


def _join(names: list[str]) -> str:
    return "、".join(names)


def _kind_of(value: Any) -> str:
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "double"
    if isinstance(value, Mapping):
        return "插件节点"
    if isinstance(value, (list, tuple)):
        return "列表"
    return "string"


# ── 类型检查：与 C++ 同一口径，不做隐式转换；实数字段接受整数（按实数存） ──


def _require_int64(value: int, path: str) -> int:
    if not _INT64_MIN <= value <= _INT64_MAX:
        raise ConfigError(f"{path}：{value} 超出 64 位整数的范围")
    return value


def _coerce_kind(kind: str, value: Any, path: str) -> Any:
    """配置字段的值（bool / int / double / string / list / slot）。"""
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
    if not isinstance(value, str):
        raise ConfigError(f"{path} 需要字符串，得到 {value!r}")
    return value


def _coerce_param(kind: str, value: Any, path: str) -> Any:
    """插件参数（bool / int / float / double / string），与 C++ 的 detail::assign 一致。"""
    mismatch = ConfigError(f"{path}：应为 {kind}，实际是 {_kind_of(value)}")
    if kind == "bool":
        if not isinstance(value, bool):
            raise mismatch
        return value
    if kind == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise mismatch
        return _require_int64(value, path)
    if kind in ("float", "double"):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise mismatch
        real = float(value)  # YAML 里的 100 与 100.0 是同一个量
        if not math.isfinite(real):
            raise ConfigError(f"{path}：不是有限数")
        if kind == "float":
            try:
                real = round_to_float32(real, path)
            except ValueError as exc:
                raise ConfigError(str(exc)) from exc
        return real
    if not isinstance(value, str):
        raise mismatch
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


# ── 插件节点的还原：与 C++ 的 Catalog::resolve 同一口径 ──


def _resolve_plugin(node: Mapping[str, Any], interface: str, schema: Schema,
                    path: str) -> dict[str, Any]:
    """按实现的字段清单还原：没写的参数补默认值，没写的子槽用默认节点，多余的键是错误。"""
    type_name = node.get("type", "")
    names = schema.implementations(interface)
    if not type_name:
        raise ConfigError(f"{path}.type：没有写 type。这里要求 {interface}，"
                          f"可选的实现有 {_join(names)}")
    plugin = schema.plugin(interface, type_name)
    if plugin is None:
        message = f"{path}.type = {type_name}：这里要求 {interface}"
        others = schema.interfaces_providing(type_name)
        if others:
            message += f"，{type_name} 是 {_join(others)} 的实现"
        message += "，目录里还没有任何实现" if not names else f"；满足的实现有 {_join(names)}"
        raise ConfigError(message)

    out: dict[str, Any] = {"type": type_name}
    known: list[str] = []
    for field in plugin.fields:
        known.append(field.name)
        sub = f"{path}.{field.name}"
        given = field.name in node
        value = node.get(field.name)
        child_interface = field.slot_interface
        if child_interface is not None:
            if not given:
                if field.default_spec is None:
                    raise ConfigError(f"{sub}：缺少这个槽，要用 type 选一个实现")
                out[field.name] = field.default_value()  # 已是 C++ 还原过的完整节点
            elif not isinstance(value, Mapping):
                raise ConfigError(f"{sub}：这是一个槽，要写成带 type 的映射")
            else:
                out[field.name] = _resolve_plugin(value, child_interface, schema, sub)
        elif not given:
            out[field.name] = field.default_value()
        elif isinstance(value, Mapping):
            raise ConfigError(f"{sub}：这是一个参数，不是槽")
        else:
            out[field.name] = _coerce_param(field.kind, value, sub)

    unknown = sorted(key for key in node if key != "type" and key not in known)
    if unknown:
        message = f"{path}.{unknown[0]}：{type_name} 没有这个参数"
        message += "，它不接受任何参数" if not known else f"，可用的有 {_join(known)}"
        raise ConfigError(message)
    return out


# ── 导出：嵌套、按字段顺序、节与节之间空一行（与 C++ 的 dump 一致） ──


def _emit_plugin(lines: list[str], node: Mapping[str, Any], indent: int) -> None:
    """type 在前，参数与子节点各按键名排序（与 C++ 的 emitPlugin 一致）。"""
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
        else:
            lines.append(f"{pad}{key}: {_format_value(value)}")
    return "\n".join(lines) + "\n" if lines else ""


def _fnv1a64(text: str) -> str:
    """与 C++ 的 digestHex 相同：FNV-1a 64，十六进制。"""
    digest = 0xCBF29CE484222325
    for byte in text.encode("utf-8"):
        digest ^= byte
        digest = (digest * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return "%016x" % digest


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
        if field.type == "slot" and path.startswith(field.path + "."):
            return field.path
    return None


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
        overrides.append(f"{path}={_flow_value(value)}")
        self._values[OVERRIDES] = overrides
        return self

    def copy(self) -> "Config":
        """深拷贝：变体改自己的插件节点，不会改到基底。"""
        clone = Config()
        clone._values = copy.deepcopy(self._values)
        return clone

    def as_dict(self) -> dict[str, Any]:
        return copy.deepcopy(self._values)

    @property
    def self_contained(self) -> bool:
        """不引用系统文件、没有覆盖项：Python 自己就能还原、导出与哈希。"""
        return SYSTEM_FILE not in self._values and not self._values.get(OVERRIDES)

    # ── 校验与还原 ──

    def validate(self, schema: Schema) -> "Config":
        """未知项与类型不符在这里报出来，不等到 C++ 侧才发现。
        插件节点要还原才知道参数对不对，见 :meth:`resolved`；取值是否有意义（枚举、
        范围）由 C++ 在加载时查，Python 不另写一份。"""
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
            if path not in schema:
                slot = _slot_containing(path, schema)
                if slot is not None:
                    raise ConfigError(f"{path} 在插件节点 {slot} 里面：整体写出这个节点"
                                      "（带 type），或用 config.override")
                raise ConfigError(f"未知配置项 {path}")
            _coerce_kind(schema.field(path).type, value, path)
        return self

    def resolved(self, schema: Schema) -> dict[str, Any]:
        """补齐默认值之后的完整取值，按 schema 的字段顺序；插件节点按各实现的字段清单还原。"""
        if not self.self_contained:
            raise ConfigError(
                "这份任务引用了系统文件或带覆盖项：读文件、应用覆盖项交给 C++，"
                "先用 through_cpp() 得到完整的任务"
            )
        self.validate(schema)
        out: dict[str, Any] = {}
        for field in schema:
            given = field.path in self._values
            value = self._values[field.path] if given else field.default_value()
            value = _coerce_kind(field.type, value, field.path)
            if field.type == "slot":
                try:
                    interface = schema.slot_interface(field.path)
                except SchemaError as exc:
                    raise ConfigError(str(exc)) from exc
                value = _resolve_plugin(value, interface, schema, field.path)
            out[field.path] = value
        return out

    def _canonical(self, schema: Schema, keep: Callable[[str], bool]) -> str:
        resolved = self.resolved(schema)
        return _emit(resolved, [field for field in schema if keep(field.path)])

    def canonical(self, schema: Schema) -> str:
        """规范化导出。与 C++ 的 dumpConfig 逐字节一致（有比对测试）。"""
        return self._canonical(schema, lambda path: True)

    def hash(self, schema: Schema) -> str:
        """任务哈希，不含 outputs：产出放在哪里不影响结果。与 C++ 的 configHash 相同。"""
        return _fnv1a64(self._canonical(schema, lambda path: not path.startswith("outputs.")))

    def system_hash(self, schema: Schema) -> str:
        """系统哈希：只覆盖 system 子树，标识算法的配置。与 C++ 的 systemHash 相同。"""
        return _fnv1a64(self._canonical(schema, lambda path: path.startswith("system.")))

    # ── 渲染 ──

    def to_yaml(self, schema: Schema | None = None, *, full: bool = False) -> str:
        """渲染成任务 YAML。

        默认只写显式设置过的项——任务文件因此只说"我改了什么"，而不是把几十个
        默认值抄一遍。``full=True`` 时输出规范化的完整任务（插件节点已还原）。
        没有 schema 时不校验，按路径排序输出。
        """
        if full:
            if schema is None:
                raise ConfigError("输出完整任务需要 schema")
            return self.canonical(schema)
        return _emit(self._values, self._render_order(schema))

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
        行内列表、``#`` 注释。

        规范化导出（:meth:`canonical` 与 C++ 的 dump）和 :meth:`to_yaml` 都在这个子集里。
        别的写法交给 C++：见 :meth:`load`。插件节点从哪一层开始只有 schema 知道，
        所以这里要求 schema。
        """
        if schema is None:
            raise ConfigError("读任务 YAML 需要 schema：插件节点从哪一层开始只有 schema 知道")
        slot_paths = {field.path for field in schema if field.type == "slot"}
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
        规范化导出后再读回来。得到的是完整、自足的任务。

        ``tool`` 是登记了所需实现的入口，见 :func:`simpleslam.schema.find_tool`。
        """
        binary = find_tool(tool)
        if schema is None:
            schema = Schema.from_tool(binary)
        try:
            dumped = subprocess.run([binary, "dump", str(path)], capture_output=True,
                                    text=True, check=True).stdout
        except subprocess.CalledProcessError as exc:
            raise ConfigError(f"{binary} dump {path} 失败: {exc.stderr.strip()}") from exc
        except OSError as exc:
            raise ConfigError(f"调用 {binary} 失败: {exc}") from exc
        return cls.from_yaml(dumped, schema)

    def through_cpp(self, base_dir: str | os.PathLike[str] = ".", tool: str | None = None,
                    schema: Schema | None = None) -> "Config":
        """让 C++ 读系统文件、应用覆盖项、还原插件节点，得到完整、自足的任务。

        ``base_dir`` 是 ``system`` 里相对路径的基准，通常是任务 ``.py`` 所在的目录。
        """
        rendered = self.copy()
        system_file = rendered.get(SYSTEM_FILE)
        if isinstance(system_file, str):
            rendered[SYSTEM_FILE] = os.path.abspath(os.path.join(base_dir, system_file))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.yaml"
            path.write_text(rendered.to_yaml(schema), encoding="utf-8")
            return Config.load(path, tool, schema)


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
    """两格一层的块映射。值是原样的标量文本，或行内列表的文本。"""
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
