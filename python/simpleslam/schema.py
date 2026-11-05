"""配置 schema：从 C++ 那份唯一的字段清单读出来，不在这里重写一遍。

没有缓存副本。缓存会漂移，而漂移了的 schema 比没有 schema 更糟——
它会让校验通过，然后在 C++ 侧报未知键。
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from dataclasses import dataclass
from typing import Iterable, Iterator


class SchemaError(RuntimeError):
    """找不到 schema 来源，或 schema 内容不可用。"""


@dataclass(frozen=True)
class Field:
    path: str
    type: str
    default: str
    doc: str
    tunable: bool

    def parse(self, text: str):
        """按本字段的类型解析一个字符串值。类型不符抛 ValueError。"""
        if self.type == "bool":
            if text in ("true", "True", "yes", "on"):
                return True
            if text in ("false", "False", "no", "off"):
                return False
            raise ValueError(f"{self.path} 需要 true/false，得到 {text!r}")
        if self.type == "int":
            return int(text)
        if self.type == "double":
            return float(text)
        return text

    def default_value(self):
        return self.parse(self.default) if self.type != "string" else self.default


class Schema:
    """配置字段清单。"""

    def __init__(self, fields: Iterable[Field]) -> None:
        self._fields = list(fields)
        self._by_path = {field.path: field for field in self._fields}

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
        except (KeyError, TypeError) as exc:
            raise SchemaError(f"schema 结构不对: {exc}") from exc
        if not fields:
            raise SchemaError("schema 里一个字段也没有")
        return cls(fields)

    @classmethod
    def from_tool(cls, tool: str | None = None) -> "Schema":
        """调用 simpleslam-config schema 取得字段清单。"""
        binary = tool or os.environ.get("SIMPLESLAM_CONFIG_TOOL") or shutil.which(
            "simpleslam-config"
        )
        if not binary:
            raise SchemaError(
                "找不到 simpleslam-config。先构建它（cmake --build <build> "
                "--target simpleslam_config_tool），或用 SIMPLESLAM_CONFIG_TOOL "
                "指向可执行文件。"
            )
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
