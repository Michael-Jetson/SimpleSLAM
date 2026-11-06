#!/usr/bin/env python3
"""SimpleSLAM 的一键入口：构建、跑任务、看结果。等同于在 python/ 下执行 python3 -m simpleslam。

    ./run.py demo                       合成夹具，跑通整条链路（不需要下载数据集）
    ./run.py configs/tasks/kitti_odometry.yaml data.source.path=/data/kitti/odometry/sequences/00
    ./run.py show runs/kitti-00         看一个档案
    ./run.py --help                     全部子命令
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "python"))

from simpleslam.__main__ import main  # noqa: E402  先把包的路径加上

raise SystemExit(main())
