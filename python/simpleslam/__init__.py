"""SimpleSLAM 的 Python 侧：在外围调用与启动 C++，不进它的运行。

* **操作台**——``./run.py``（``python -m simpleslam``）：构建入口、跑任务、补评、扫参、
  看结果。加载、校验、执行、评测都在 C++ 入口里，这里只编排与展示。
* **任务配置**——用 Python 写任务（一个 ``.py`` 文件构造 :class:`~simpleslam.config.Config`），
  渲染成 YAML 交给 C++ 加载。字段清单读入口导出的 schema（``<入口> schema``），不在这里
  重写；完整的任务由 C++ 导出（``<入口> dump``）。
* **轨迹**——TUM / KITTI 的读写与格式转换。
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
