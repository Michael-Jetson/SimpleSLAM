"""配置 schema：从 C++ 那份唯一的字段清单读出来，不在这里重写一遍。

字段清单之外还有几样：插件字段要求的接口（``slots``），各目录里每个实现的字段清单与
它用到的参数（``plugins``），内置模块用到的参数（``modules``），改过写法的配置项
（``renamed``）。哪个入口导出 schema，目录里就是它登记的实现：用户写的算法要用
用户自己的入口导出。

没有缓存副本。缓存会漂移，而漂移了的 schema 比没有 schema 更糟——
它会让校验通过，然后在 C++ 侧报未知键。
"""

from __future__ import annotations

import json
import os
import subprocess
from dataclasses import dataclass
from typing import Any, Iterable, Iterator, Mapping


class SchemaError(RuntimeError):
    """找不到 schema 来源，或 schema 内容不可用。"""


@dataclass(frozen=True)
class Field:
    path: str
    # bool / int / double / string / list / slot / slots（带名字的插件节点表）/
    # params（参数的值：名字 → 标量，名字由选中的组件声明）
    type: str
    default: str
    doc: str

    def parse(self, text: str):
        """按本字段的类型解析一个字符串值。类型不符抛 ValueError。"""
        return parse_kind(self.type, text, self.path)


def find_tool(tool: str | None = None) -> str:
    """导出 schema、加载任务的那个入口（可执行文件）：显式给出的，或环境变量
    SIMPLESLAM_CONFIG_TOOL。操作台的命令默认用示例入口，按需构建。"""
    binary = tool or os.environ.get("SIMPLESLAM_CONFIG_TOOL")
    if not binary:
        raise SchemaError(
            "没有给入口：用 tool= 指向登记了所需实现的入口（可执行文件），"
            "或设环境变量 SIMPLESLAM_CONFIG_TOOL"
        )
    return binary


def parse_kind(kind: str, text: str, path: str):
    """按配置字段的类型解析文本。口径与 C++ 导出的默认值文本一致。"""
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
    return text


@dataclass(frozen=True)
class PluginField:
    name: str
    kind: str  # bool / int / float / double / string / slot:<接口名>
    default: str
    doc: str


@dataclass(frozen=True)
class ParamSpec:
    """一个参数的声明：名字、类型、默认值、说明、范围、运行中改不改、写不写。"""
    name: str
    type: str  # bool / int / double / string
    default: Any
    doc: str
    min: float | None
    max: float | None
    runtime: bool
    writes: bool


@dataclass(frozen=True)
class PluginSchema:
    interface: str
    type: str
    fields: tuple[PluginField, ...]
    params: tuple[ParamSpec, ...] = ()


def _param_specs(items: Iterable[Mapping[str, Any]]) -> tuple[ParamSpec, ...]:
    return tuple(
        ParamSpec(name=item["name"], type=item["type"], default=item["default"],
                  doc=item.get("doc", ""), min=item.get("min"), max=item.get("max"),
                  runtime=bool(item.get("runtime", False)), writes=bool(item.get("writes", False)))
        for item in items
    )


class Schema:
    """配置字段清单，加上插件字段要求的接口与各实现的字段清单。"""

    def __init__(
        self,
        fields: Iterable[Field],
        slots: dict[str, str] | None = None,
        plugins: Iterable[PluginSchema] = (),
        renamed: dict[str, str] | None = None,
        modules: dict[str, tuple[ParamSpec, ...]] | None = None,
    ) -> None:
        self._fields = list(fields)
        self._by_path = {field.path: field for field in self._fields}
        self._slots = dict(slots or {})
        self._plugins = {(plugin.interface, plugin.type): plugin for plugin in plugins}
        self._renamed = dict(renamed or {})
        self._modules = dict(modules or {})

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
                        )
                        for field in item["fields"]
                    ),
                    params=_param_specs(item.get("params", [])),
                )
                for item in payload.get("plugins", [])
            ]
            slots = dict(payload.get("slots", {}))
            renamed = dict(payload.get("renamed", {}))
            modules = {item["module"]: _param_specs(item.get("params", []))
                       for item in payload.get("modules", [])}
        except (KeyError, TypeError) as exc:
            raise SchemaError(f"schema 结构不对: {exc}") from exc
        if not fields:
            raise SchemaError("schema 里一个字段也没有")
        return cls(fields, slots, plugins, renamed, modules)

    @classmethod
    def from_tool(cls, tool: str | None = None) -> "Schema":
        """调用 ``<tool> schema`` 取得字段清单。"""
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

    def implementations(self, interface: str) -> list[str]:
        return sorted(t for (i, t) in self._plugins if i == interface)

    def interfaces(self) -> list[str]:
        """目录里出现过的全部接口。"""
        return sorted({i for (i, _) in self._plugins})

    @property
    def renamed(self) -> dict[str, str]:
        """改过写法的配置项：旧写法 → 新写法。旧写法照样认，原样交给 C++，由它翻译并提示。"""
        return dict(self._renamed)

    @property
    def module_params(self) -> dict[str, tuple[ParamSpec, ...]]:
        """内置模块自己用到的参数，按模块名。"""
        return dict(self._modules)

    @property
    def slots(self) -> dict[str, str]:
        """插件字段到它要求的接口名。"""
        return dict(self._slots)

    def plugin(self, interface: str, type_name: str) -> PluginSchema | None:
        return self._plugins.get((interface, type_name))
