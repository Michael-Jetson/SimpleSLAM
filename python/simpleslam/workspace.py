"""构建目录与入口：一键运行要先有一个构建好的入口程序。

入口是登记了算法实现的 C++ 可执行文件，示例是 ``simpleslam_lio_skeleton``，用户写的算法用
自己的入口（``--entry`` 或环境变量 ``SIMPLESLAM_ENTRY``）。这里只做三件事：

* 第一次用时配置构建目录：只构建新栈、不构建测试，默认 Release；
* 之后每次只增量构建要用的那个目标；
* 按 CMake 的 File API 找到目标的可执行文件，不猜输出路径。

构建规则全在 CMake 里，这里一条也不写。``docker/deps/manif`` 存在时自动用它（预下载的依赖，
见 ``.gitignore``），本机没有装 manif 也能配置。``--docker`` 时在开发镜像里重新执行同一条命令。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

DEFAULT_ENTRY = "simpleslam_lio_skeleton"
DOCKER_IMAGE = "simpleslam-dev"
_IN_DOCKER = "SIMPLESLAM_IN_DOCKER"


class BuildError(RuntimeError):
    """配置或构建失败，或者构建目录里找不到要的目标。"""


def repo_root() -> Path:
    """仓库根目录：本包在 ``<根>/python/simpleslam/`` 下。"""
    root = Path(__file__).resolve().parents[2]
    if not (root / "CMakeLists.txt").is_file():
        raise BuildError(f"{root} 不像 SimpleSLAM 的源码树：操作台要在源码树里用")
    return root


@dataclass(frozen=True)
class Workspace:
    root: Path
    build_type: str = "Release"

    @property
    def build_dir(self) -> Path:
        """``<构建根>/<类型>``。构建根默认是源码树下的 build/，环境变量 SIMPLESLAM_BUILD_ROOT
        可以换到别处（源码树在同步盘里时，构建产物不该跟着同步）。"""
        base = Path(os.environ.get("SIMPLESLAM_BUILD_ROOT") or self.root / "build")
        # 容器里的构建目录与本机分开：工具链与依赖不同，混用同一个 CMake 缓存会出错
        prefix = "docker-" if os.environ.get(_IN_DOCKER) else ""
        return base / f"{prefix}{self.build_type.lower()}"

    def entry(self, name_or_path: str | None = None, *, build: bool = True) -> Path:
        """入口的可执行文件。给的是可执行文件的路径就直接用；否则当作 CMake 目标名，按需构建。"""
        candidate = name_or_path or os.environ.get("SIMPLESLAM_ENTRY") or DEFAULT_ENTRY
        path = Path(candidate)
        if path.is_file() and os.access(path, os.X_OK):
            return path.resolve()
        if build:
            self.build(candidate)
        return self.artifact(candidate)

    def configure(self) -> None:
        query = self.build_dir / ".cmake" / "api" / "v1" / "query" / "codemodel-v2"
        query.parent.mkdir(parents=True, exist_ok=True)
        query.touch()
        args = ["cmake", "-S", str(self.root), "-B", str(self.build_dir),
                f"-DCMAKE_BUILD_TYPE={self.build_type}", "-DSIMPLESLAM_NEW_STACK_ONLY=ON",
                "-DBUILD_TESTING=OFF"]
        manif = self.root / "docker" / "deps" / "manif"
        if (manif / "include").is_dir():
            args.append(f"-DSIMPLESLAM_MANIF_INCLUDE_DIR={manif / 'include'}")
            # 单引号留给编译命令的 shell：路径里可能有空格
            args.append(f"-DCMAKE_CXX_FLAGS=-I'{manif / 'external' / 'tl'}'")
        print(f"配置构建目录 {self.build_dir}（{self.build_type}）", file=sys.stderr)
        if subprocess.run(args).returncode != 0:
            raise BuildError("配置失败（日志见上）：依赖不全时运行 scripts/install_deps.sh，"
                             "或加 --docker 在开发镜像里构建")

    def build(self, target: str) -> None:
        if not (self.build_dir / "CMakeCache.txt").is_file() or not self._reply_index():
            self.configure()
        print(f"构建 {target}（{self.build_type}）", file=sys.stderr)
        jobs = str(os.cpu_count() or 2)
        args = ["cmake", "--build", str(self.build_dir), "--target", target, "-j", jobs]
        if subprocess.run(args).returncode != 0:
            raise BuildError(f"构建 {target} 失败（日志见上）")

    def artifact(self, target: str) -> Path:
        """目标的可执行文件，取自 CMake File API 的 codemodel。"""
        reply = self.build_dir / ".cmake" / "api" / "v1" / "reply"
        index = self._reply_index()
        if index is None:
            raise BuildError(f"{self.build_dir} 里没有 CMake 的目标清单：先构建一次")
        codemodel = _read_json(reply / index["reply"]["codemodel-v2"]["jsonFile"])
        executables: list[str] = []
        for configuration in codemodel["configurations"]:
            for item in configuration["targets"]:
                detail = _read_json(reply / item["jsonFile"])
                if detail.get("type") != "EXECUTABLE":
                    continue
                if item["name"] == target:
                    return (self.build_dir / detail["artifacts"][0]["path"]).resolve()
                executables.append(item["name"])
        known = "、".join(sorted(executables))
        raise BuildError(f"构建目录里没有可执行目标 {target}；有的是 {known}")

    def _reply_index(self) -> dict | None:
        reply = self.build_dir / ".cmake" / "api" / "v1" / "reply"
        indexes = sorted(reply.glob("index-*.json")) if reply.is_dir() else []
        return _read_json(indexes[-1]) if indexes else None


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def in_docker() -> bool:
    return bool(os.environ.get(_IN_DOCKER))


def run_in_docker(root: Path, argv: list[str], mounts: list[Path]) -> int:
    """在开发镜像里执行同一条命令。源码树按原路径挂进去，数据目录只读挂进去，路径都不用改。"""
    probe = subprocess.run(["docker", "image", "inspect", DOCKER_IMAGE], capture_output=True)
    if probe.returncode != 0:
        print(f"构建开发镜像 {DOCKER_IMAGE}", file=sys.stderr)
        if subprocess.run(["docker", "build", "-t", DOCKER_IMAGE, str(root)]).returncode != 0:
            raise BuildError("构建开发镜像失败（日志见上）")
    cwd = Path.cwd().resolve()
    volumes = {root.resolve(): "rw"}
    if not cwd.is_relative_to(root.resolve()):
        volumes[cwd] = "rw"
    for mount in mounts:
        volumes.setdefault(mount.resolve(), "ro")
    args = ["docker", "run", "--rm", "--user", f"{os.getuid()}:{os.getgid()}",
            "-e", f"{_IN_DOCKER}=1", "-w", str(cwd)]
    for path, mode in volumes.items():
        args += ["-v", f"{path}:{path}:{mode}"]
    args += [DOCKER_IMAGE, "python3", str(root / "run.py"), *argv]
    return subprocess.run(args).returncode
