"""SimpleSLAM 的 Python 侧工具：配置、轨迹、精度指标。

三件事，和 C++ 侧的分工是明确的：

* **配置**——真源在 C++。这里不重写 schema，而是读
  ``simpleslam-config schema`` 的输出；配置可以用 Python 写（一个 ``.py``
  文件构造 :class:`~simpleslam.config.Config`），渲染成 YAML 交给 C++ 加载。
  规范化导出与哈希在两边各有一份实现，由 ``python/tests/test_cpp_parity.py``
  逐字节比对——重复只在有自动比对的前提下才被允许。
* **轨迹**——TUM / KITTI 的读写，与 C++ 侧同一口径。
* **指标**——ATE / RPE，纯标准库实现（没有 numpy 依赖）。
  需要卡门禁时用 ``simpleslam-eval``：那一份是权威实现，退出码能让流水线失败。
"""

from simpleslam.config import Config, ConfigError
from simpleslam.schema import Schema, SchemaError
from simpleslam.trajectory import Trajectory, read_trajectory, write_trajectory

__all__ = [
    "Config",
    "ConfigError",
    "Schema",
    "SchemaError",
    "Trajectory",
    "read_trajectory",
    "write_trajectory",
]
