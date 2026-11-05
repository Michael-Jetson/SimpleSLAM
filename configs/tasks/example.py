"""用 Python 写的任务：系统用 ../systems/my_lio.yaml，离线跑 KITTI raw 的一段采集并评测。

渲染成 YAML（--tool 是登记了算法的入口，你的算法用你自己的入口）::

    python -m simpleslam --tool <build>/examples/simpleslam-lio-skeleton \\
        config render configs/tasks/example.py -o example.yaml

扫参的变体渲染进一个目录：``python3 configs/tasks/example.py <目录>``。

用 Python 写任务的理由是"配置本身需要计算"：按传感器频率推参数、一次生成一组
扫参任务。不需要计算时，写 YAML 更直白——两种都支持，不强迫统一。
"""

from pathlib import Path

from simpleslam import Config

OXTS_HZ = 10.0  # KITTI raw 同步版的真值频率

# 变体可能渲染到别的目录，所以系统文件按本文件的位置写成绝对路径
SYSTEM = Path(__file__).resolve().parent.parent / "systems" / "my_lio.yaml"

config = Config(
    task={"name": "kitti-raw-0027", "purpose": "experiment", "mode": "offline"},
    system=str(SYSTEM),
    data={
        "source": {
            "type": "kitti_raw",
            "path": "/data/kitti/raw/2011_09_26/2011_09_26_drive_0027_sync",
        }
    },
    # 估计在扫描终点、真值在 OXTS 时刻：按插值关联，相邻真值最多隔一个半周期
    evaluation={
        "format": "tum",
        "alignment": "se3",
        "association": "interpolate",
        "max_interpolation_gap_s": 1.5 / OXTS_HZ,
    },
)


def sweep(distances=(0.5, 1.0, 2.0)):
    """扫参：同一份基底生成一组变体，每个变体是一个任务、一个档案目录。

    系统写在文件里，改系统参数用覆盖项，写法与命令行相同。
    """
    for distance in distances:
        variant = config.copy()
        variant["task.name"] = f"kitti-raw-0027-kf{distance:g}"
        variant.override("system.modules.odometry.plugin.keyframe_distance_m", distance)
        yield variant


if __name__ == "__main__":
    import sys

    target = Path(sys.argv[1])
    target.mkdir(parents=True, exist_ok=True)
    for variant in sweep():
        path = target / f"{variant['task.name']}.yaml"
        path.write_text(variant.to_yaml(), encoding="utf-8")
        print(path)
