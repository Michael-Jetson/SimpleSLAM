"""配置对象：用 Python 写配置，渲染成 C++ 能加载的 YAML。

用法（一个 ``.py`` 配置文件长这样）::

    from simpleslam import Config

    config = Config(name="kitti-00")
    config["runtime.offline"] = True
    config["odometry.algorithm"] = "kiss"
    config["odometry.max_iterations"] = 40

    # 也可以按节写
    config.update(packetizer={"imu_required": False, "post_support_s": 0.02})

然后 ``python -m simpleslam config render cfg.py -o cfg.yaml``。

规范化导出（:meth:`Config.canonical`）与哈希在 C++ 与 Python 各有一份实现。
重复是刻意的——Python 侧不该为了算个哈希去调子进程——但它只在有自动比对的
前提下成立：``python/tests/test_cpp_parity.py`` 逐字节比两边的输出。
"""

from __future__ import annotations

from typing import Any, Iterator, Mapping

from simpleslam.schema import Schema


class ConfigError(ValueError):
    """配置项不存在、类型不符，或 YAML 结构不对。"""


def _quote_yaml(text: str) -> str:
    """字符串一律加引号转义，与 C++ 的 quoteYaml 逐字符一致。

    不加引号是不可逆的：``name: "a # b"`` 会在往返中变成 ``a``。
    """
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
    """与 C++ 的 canonicalText 逐字符一致。哈希依赖它，所以不能随手改。"""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return "%.17g" % value
    return _quote_yaml(str(value))


def _coerce(field, value: Any) -> Any:
    """把 Python 值收敛到字段声明的类型。不做隐式转换，与 C++ 侧同一口径。"""
    if field.type == "bool":
        if not isinstance(value, bool):
            raise ConfigError(f"{field.path} 需要 bool，得到 {value!r}")
        return value
    if field.type == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{field.path} 需要 int，得到 {value!r}")
        return value
    if field.type == "double":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"{field.path} 需要实数，得到 {value!r}")
        return float(value)
    if not isinstance(value, str):
        raise ConfigError(f"{field.path} 需要字符串，得到 {value!r}")
    return value


class Config:
    """一份配置。只保存显式给出的项；默认值由 schema 提供。"""

    def __init__(self, **values: Any) -> None:
        self._values: dict[str, Any] = {}
        for key, value in values.items():
            self[key] = value

    # ── 读写单项 ──

    def __setitem__(self, path: str, value: Any) -> None:
        if not path:
            raise ConfigError("配置项路径不能为空")
        self._values[path] = value

    def __getitem__(self, path: str) -> Any:
        return self._values[path]

    def __contains__(self, path: object) -> bool:
        return path in self._values

    def __iter__(self) -> Iterator[str]:
        return iter(self._values)

    def __len__(self) -> int:
        return len(self._values)

    def __repr__(self) -> str:
        items = ", ".join(f"{k}={v!r}" for k, v in sorted(self._values.items()))
        return f"Config({items})"

    def get(self, path: str, default: Any = None) -> Any:
        return self._values.get(path, default)

    def update(self, other: Mapping[str, Any] | None = None, **sections: Any) -> "Config":
        """按点分路径或按节更新。

        ``update({"odometry.max_iterations": 40})`` 与
        ``update(odometry={"max_iterations": 40})`` 等价。
        """
        if other:
            for path, value in other.items():
                self[path] = value
        for section, values in sections.items():
            if isinstance(values, Mapping):
                for key, value in values.items():
                    self[f"{section}.{key}"] = value
            else:
                self[section] = values
        return self

    def as_dict(self) -> dict[str, Any]:
        return dict(self._values)

    # ── 与 schema 打交道 ──

    def validate(self, schema: Schema) -> "Config":
        """未知项与类型不符都在这里报出来，不等到 C++ 侧才发现。"""
        for path, value in self._values.items():
            if path not in schema:
                raise ConfigError(f"未知配置项 {path}")
            _coerce(schema.field(path), value)
        return self

    def resolved(self, schema: Schema) -> dict[str, Any]:
        """补齐默认值之后的完整取值，按 schema 的字段顺序。"""
        out: dict[str, Any] = {}
        for field in schema:
            if field.path in self._values:
                out[field.path] = _coerce(field, self._values[field.path])
            else:
                out[field.path] = field.default_value()
        for path in self._values:
            if path not in schema:
                raise ConfigError(f"未知配置项 {path}")
        return out

    def canonical(self, schema: Schema) -> str:
        """规范化导出。与 C++ 的 dumpConfig 逐字节一致（有比对测试）。"""
        resolved = self.resolved(schema)
        lines: list[str] = []
        current_section = ""
        for field in schema:
            parts = field.path.split(".")
            value = _format_value(resolved[field.path])
            if len(parts) == 1:
                if current_section:
                    lines.append("")
                    current_section = ""
                lines.append(f"{parts[0]}: {value}")
                continue
            if parts[0] != current_section:
                lines.append("")
                lines.append(f"{parts[0]}:")
                current_section = parts[0]
            lines.append(f"  {parts[1]}: {value}")
        return "\n".join(lines) + "\n"

    def hash(self, schema: Schema) -> str:
        """配置哈希（FNV-1a 64，十六进制）。与 C++ 的 configHash 相同。"""
        digest = 0xCBF29CE484222325
        for byte in self.canonical(schema).encode("utf-8"):
            digest ^= byte
            digest = (digest * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
        return "%016x" % digest

    def to_yaml(self, schema: Schema | None = None, *, full: bool = False) -> str:
        """渲染成 YAML。

        默认只写显式设置过的项——配置文件因此只说"我改了什么"，
        而不是把几十个默认值抄一遍。``full=True`` 或给了 schema 且要求完整时，
        输出规范化的全量配置。
        """
        if full:
            if schema is None:
                raise ConfigError("输出全量配置需要 schema")
            return self.canonical(schema)
        if schema is not None:
            self.validate(schema)

        sections: dict[str, list[tuple[str, Any]]] = {}
        toplevel: list[tuple[str, Any]] = []
        order = schema.paths if schema is not None else sorted(self._values)
        seen: set[str] = set()
        for path in order:
            if path not in self._values:
                continue
            seen.add(path)
            parts = path.split(".")
            if len(parts) == 1:
                toplevel.append((parts[0], self._values[path]))
            else:
                sections.setdefault(parts[0], []).append((parts[1], self._values[path]))
        for path in sorted(self._values):
            if path in seen:
                continue
            parts = path.split(".")
            if len(parts) == 1:
                toplevel.append((parts[0], self._values[path]))
            else:
                sections.setdefault(parts[0], []).append((parts[1], self._values[path]))

        lines = [f"{key}: {_format_value(value)}" for key, value in toplevel]
        for section, entries in sections.items():
            if lines:
                lines.append("")
            lines.append(f"{section}:")
            lines.extend(f"  {key}: {_format_value(value)}" for key, value in entries)
        return "\n".join(lines) + "\n" if lines else ""

    # ── 从 YAML 读回来 ──

    @classmethod
    def from_yaml(cls, text: str, schema: Schema | None = None) -> "Config":
        """解析配置 YAML 的受限子集：两层映射、标量取值、# 注释。

        刻意不引入 YAML 库：配置的权威解析在 C++，这里只需要够用来做
        渲染与比对。遇到子集之外的写法就报错，不猜。
        """
        config = cls()
        section = ""
        for number, raw in enumerate(text.splitlines(), start=1):
            line = _strip_comment(raw).rstrip()
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip(" "))
            if indent not in (0, 2):
                raise ConfigError(f"第 {number} 行：只支持两层配置（缩进 0 或 2）")
            stripped = line.strip()
            if ":" not in stripped:
                raise ConfigError(f"第 {number} 行：不是 键: 值")
            key, _, value = stripped.partition(":")
            key = key.strip()
            value = value.strip()
            if indent == 0 and not value:
                section = key
                continue
            path = key if indent == 0 else f"{section}.{key}"
            if indent == 2 and not section:
                raise ConfigError(f"第 {number} 行：缩进项没有所属的节")
            config[path] = _parse_scalar(value, path, schema)
        if schema is not None:
            config.validate(schema)
        return config


def _strip_comment(line: str) -> str:
    """去掉行尾注释，但不碰引号里的 ``#``。

    `name: "a # b"` 被朴素地按 ``#`` 切一刀就只剩 `name: "a `，
    往返因此丢内容——C++ 侧走 yaml-cpp 不会有这个问题，Python 侧的受限解析器
    必须自己处理。
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
        elif char == "#":
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


def _parse_scalar(text: str, path: str, schema: Schema | None):
    """标量解析。有 schema 就按声明的类型解析，没有就按字面量推断。"""
    if schema is not None and path in schema:
        field = schema.field(path)
        if text.startswith('"') and text.endswith('"') and len(text) >= 2:
            if field.type != "string":
                raise ConfigError(f"{path} 需要{field.type}，得到带引号的字符串")
            return _unquote_yaml(text)
        try:
            return field.parse(text)
        except ValueError as exc:
            raise ConfigError(str(exc)) from exc
    if text.startswith('"') and text.endswith('"') and len(text) >= 2:
        return _unquote_yaml(text)
    if text in ("true", "false"):
        return text == "true"
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text
