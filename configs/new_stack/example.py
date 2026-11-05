"""用 Python 写的等价配置。

渲染：python -m simpleslam config render configs/new_stack/example.py -o out.yaml

用 Python 写配置的理由是"配置本身需要计算"：按数据集算支撑区间、
按传感器频率推关键帧间隔、一次生成一组扫参配置。不需要计算时，
写 YAML 更直白——两种都支持，不强迫统一。
"""

from simpleslam import Config

SCAN_HZ = 10.0
IMU_HZ = 200.0

config = Config(name="example")
config.update(
    runtime={"offline": True},
    # 支撑区间取两个 IMU 周期：够做终点插值，也够在线估时间偏置
    packetizer={"imu_required": True, "post_support_s": 2.0 / IMU_HZ},
    odometry={
        "algorithm": "constant_velocity",
        "keyframe_distance_m": 1.0,
        "keyframe_interval_s": 10.0 / SCAN_HZ,
        "max_iterations": 30,
    },
    backend={"place_recognizer": "none", "graph_optimizer": "none"},
    evaluation={
        "output": "/tmp/simpleslam_estimate.tum",
        "format": "tum",
        "alignment": "se3",
    },
)


def sweep(values=(20, 30, 40)):
    """扫参：同一份基底配置生成一组变体。这是 .py 配置真正的用处。"""
    for iterations in values:
        variant = Config(**config.as_dict())
        variant["name"] = f"example-iter{iterations}"
        variant["odometry.max_iterations"] = iterations
        yield variant
