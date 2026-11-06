"""配置 schema：从 C++ 那份唯一的字段清单读出来，不在这里重写一遍。

字段清单之外还有两样：插件字段要求的接口（``slots``），以及各目录里每个实现的
字段清单（``plugins``）。哪个可执行文件导出 schema，目录里就是它登记的实现：
用户写的算法要用用户自己的入口导出（``SIMPLESLAM_CONFIG_TOOL`` 指向它）。

没有缓存副本。缓存会漂移，而漂移了的 schema 比没有 schema 更糟——
它会让校验通过，然后在 C++ 侧报未知键。
"""

from __future__ import annotations

import copy
import json
import os
import shutil
import struct
import subprocess
from dataclasses import dataclass
from typing import Any, Iterable, Iterator


class SchemaError(RuntimeError):
    """找不到 schema 来源，或 schema 内容不可用。"""


@dataclass(frozen=True)
class Field:
    path: str
    type: str  # bool / int / double / string / list / slot / slots（带名字的插件节点表）
    default: str
    doc: str
    tunable: bool

    def parse(self, text: str):
        """按本字段的类型解析一个字符串值。类型不符抛 ValueError。"""
        return parse_kind(self.type, text, self.path)

    def default_value(self):
        if self.type == "list":
            return json.loads(self.default) if self.default else []
        if self.type == "slot":
            return {"type": self.default}
        if self.type == "slots":
            return {}  # 默认是空表：没列就没有
        return self.parse(self.default) if self.type != "string" else self.default


def find_tool(tool: str | None = None) -> str:
    """导出 schema、加载任务的那个入口：显式给出的，或 SIMPLESLAM_CONFIG_TOOL，或 PATH 里的
    simpleslam-config。后者只有内置实现，用户写的算法要用登记了它的入口。"""
    binary = tool or os.environ.get("SIMPLESLAM_CONFIG_TOOL") or shutil.which("simpleslam-config")
    if not binary:
        raise SchemaError(
            "找不到 simpleslam-config。先构建它（cmake --build <build> "
            "--target simpleslam_config_tool），或用 SIMPLESLAM_CONFIG_TOOL "
            "指向可执行文件（用户写的算法用用户自己的入口）。"
        )
    return binary


def round_to_float32(value: float, path: str) -> float:
    """C++ 的单精度字段在赋值时就舍入了，还原后的节点存的是舍入后的值。"""
    try:
        return struct.unpack("f", struct.pack("f", value))[0]
    except OverflowError as exc:
        raise ValueError(f"{path}：{value!r} 超出这个字段的取值范围") from exc


def parse_kind(kind: str, text: str, path: str):
    """按类型解析默认值文本。口径与 C++ 导出的默认值文本一致。"""
    if kind == "bool":
        if text in ("true", "True", "yes", "on"):
            return True
        if text in ("false", "False", "no", "off"):
            return False
        raise ValueError(f"{path} 需要 true/false，得到 {text!r}")
    if kind == "int":
        return int(text)
    if kind == "double":
        return float(text)
    if kind == "float":
        return round_to_float32(float(text), path)
    return text


def plugin_node(spec: dict[str, Any]) -> dict[str, Any]:
    """schema JSON 里的插件节点（type / params / children 三段）换成配置里的写法：
    一个映射，``type`` 加参数，子节点是嵌套的映射。"""
    node: dict[str, Any] = {"type": spec["type"]}
    node.update(spec.get("params", {}))
    for key, child in spec.get("children", {}).items():
        node[key] = plugin_node(child)
    return node


@dataclass(frozen=True)
class PluginField:
    name: str
    kind: str  # bool / int / float / double / string / slot:<接口名>
    default: str
    doc: str
    # 子槽的完整默认节点（类型加补齐的参数），没有默认实现时为 None。
    # 默认实现可以带非默认参数，只凭 default 里的类型名补不对。
    default_spec: dict[str, Any] | None = None

    @property
    def slot_interface(self) -> str | None:
        return self.kind[len("slot:"):] if self.kind.startswith("slot:") else None

    def default_value(self):
        if self.slot_interface is not None:
            return copy.deepcopy(self.default_spec)
        if self.kind == "string":
            return self.default
        return parse_kind(self.kind, self.default, self.name)


@dataclass(frozen=True)
class PluginSchema:
    interface: str
    type: str
    fields: tuple[PluginField, ...]


class Schema:
    """配置字段清单，加上插件字段要求的接口与各实现的字段清单。"""

    def __init__(
        self,
        fields: Iterable[Field],
        slots: dict[str, str] | None = None,
        plugins: Iterable[PluginSchema] = (),
    ) -> None:
        self._fields = list(fields)
        self._by_path = {field.path: field for field in self._fields}
        self._slots = dict(slots or {})
        self._plugins = {(plugin.interface, plugin.type): plugin for plugin in plugins}

    # ── 构造 ──

    @classmethod
    def from_json(cls, text: str) -> "Schema":
        try:
            payload = json.loads(text)
        except json.JSONDecodeError as exc:
            raise SchemaError(f"schema 不是合法 JSON: {exc}") from exc
        try:
            fields = [
                Field(
                    path=item["path"],
                    type=item["type"],
                    default=item["default"],
                    doc=item.get("doc", ""),
                    tunable=bool(item.get("tunable", False)),
                )
                for item in payload["fields"]
            ]
            plugins = [
                PluginSchema(
                    interface=item["interface"],
                    type=item["type"],
                    fields=tuple(
                        PluginField(
                            name=field["name"],
                            kind=field["kind"],
                            default=field["default"],
                            doc=field.get("doc", ""),
                            default_spec=(plugin_node(field["default_spec"])
                                          if "default_spec" in field else None),
                        )
                        for field in item["fields"]
                    ),
                )
                for item in payload.get("plugins", [])
            ]
            slots = dict(payload.get("slots", {}))
        except (KeyError, TypeError) as exc:
            raise SchemaError(f"schema 结构不对: {exc}") from exc
        if not fields:
            raise SchemaError("schema 里一个字段也没有")
        return cls(fields, slots, plugins)

    @classmethod
    def from_tool(cls, tool: str | None = None) -> "Schema":
        """调用 ``<tool> schema`` 取得字段清单。默认是 simpleslam-config（只有内置实现）。"""
        binary = find_tool(tool)
        try:
            result = subprocess.run(
                [binary, "schema"], capture_output=True, text=True, check=True
            )
        except (OSError, subprocess.CalledProcessError) as exc:
            raise SchemaError(f"调用 {binary} schema 失败: {exc}") from exc
        return cls.from_json(result.stdout)

    @classmethod
    def load(cls, tool: str | None = None) -> "Schema":
        return cls.from_tool(tool)

    # ── 查询 ──

    def __iter__(self) -> Iterator[Field]:
        return iter(self._fields)

    def __len__(self) -> int:
        return len(self._fields)

    def __contains__(self, path: object) -> bool:
        return path in self._by_path

    def field(self, path: str) -> Field:
        try:
            return self._by_path[path]
        except KeyError as exc:
            raise SchemaError(f"未知配置项 {path}") from exc

    @property
    def paths(self) -> list[str]:
        return [field.path for field in self._fields]

    def tunable_paths(self) -> list[str]:
        return [field.path for field in self._fields if field.tunable]

    def slot_interface(self, path: str) -> str:
        """插件字段要求的接口名。schema 没给对照表时报错：还原插件节点离不开它。"""
        try:
            return self._slots[path]
        except KeyError as exc:
            raise SchemaError(f"schema 里没有插件字段 {path} 要求的接口") from exc

    def implementations(self, interface: str) -> list[str]:
        return sorted(t for (i, t) in self._plugins if i == interface)

    def interfaces(self) -> list[str]:
        """目录里出现过的全部接口。"""
        return sorted({i for (i, _) in self._plugins})

    def interfaces_providing(self, type_name: str) -> list[str]:
        """哪些接口的目录里有这个类型名。选错接口时给出提示用。"""
        return sorted({i for (i, t) in self._plugins if t == type_name})

    @property
    def slots(self) -> dict[str, str]:
        """插件字段到它要求的接口名。"""
        return dict(self._slots)

    def plugin(self, interface: str, type_name: str) -> PluginSchema | None:
        return self._plugins.get((interface, type_name))
