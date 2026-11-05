#!/usr/bin/env python3
"""生成一份最小的 KITTI 格式夹具，用来冒烟测试整条链路。

这**不是**数据集——它只有几十帧、每帧几百个点，跑出来的精度没有意义。
它的用处是：不下载 20GB 也能验证"读取 → 打包 → 里程计 → 真源 → 评测 → 导出"
这条链路是通的，以及自己写的读取器改动没有把格式弄坏。

用法:
    python3 scripts/make_kitti_fixture.py --out /tmp/kitti_fixture --frames 40
    build/examples/simpleslam-lio-skeleton configs/tasks/kitti_odometry.yaml \\
        data.source.path=/tmp/kitti_fixture/odometry/sequences/00 outputs.dir=/tmp/kitti_run
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path


def pose_at(index: int, step: float, turn_rad: float) -> tuple[list[float], list[float]]:
    """沿一段圆弧行驶：返回 (3x3 行主序旋转, 平移)。"""
    yaw = turn_rad * index
    if abs(turn_rad) < 1e-9:
        x, y = step * index, 0.0
    else:
        radius = step / turn_rad
        x = radius * math.sin(yaw)
        y = radius * (1.0 - math.cos(yaw))
    c, s = math.cos(yaw), math.sin(yaw)
    return [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0], [x, y, 0.0]


def write_scan(path: Path, rings: int, per_ring: int) -> None:
    """一圈圈的点。方位角铺满 360°，读取器才有东西反推逐点时间。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as handle:
        for ring in range(rings):
            pitch = math.radians(-24.0 + 26.0 * ring / max(rings - 1, 1))
            for i in range(per_ring):
                azimuth = 2.0 * math.pi * i / per_ring
                distance = 12.0 + 3.0 * math.sin(3.0 * azimuth)
                x = distance * math.cos(pitch) * math.cos(azimuth)
                y = distance * math.cos(pitch) * math.sin(azimuth)
                z = distance * math.sin(pitch)
                handle.write(struct.pack("<ffff", x, y, z, 0.5))


def row_major(rotation: list[float], translation: list[float]) -> str:
    values = []
    for row in range(3):
        values.extend(rotation[row * 3: row * 3 + 3])
        values.append(translation[row])
    return " ".join(f"{v:.12f}" for v in values)


def make_odometry(root: Path, frames: int, rings: int, per_ring: int, step: float,
                  turn: float) -> Path:
    sequence = root / "odometry" / "sequences" / "00"
    times = []
    poses = []
    for i in range(frames):
        write_scan(sequence / "velodyne" / f"{i:06d}.bin", rings, per_ring)
        times.append(f"{0.1 * i:.6f}")
        rotation, translation = pose_at(i, step, turn)
        poses.append(row_major(rotation, translation))
    (sequence / "times.txt").write_text("\n".join(times) + "\n")
    # Tr = T_cam0_velo：相机在激光前上方，绕 x 转 -90° 再绕 z 转 -90°（近似 KITTI 的关系）
    (sequence / "calib.txt").write_text(
        "P0: 1 0 0 0 0 1 0 0 0 0 1 0\n"
        "Tr: " + row_major([0, -1, 0, 0, 0, -1, 1, 0, 0], [0.0, -0.08, 0.27]) + "\n"
    )
    poses_dir = root / "odometry" / "poses"
    poses_dir.mkdir(parents=True, exist_ok=True)
    # poses/XX.txt 是相机系的；读取器会用 Tr 换到 body 系
    cam_poses = []
    for i in range(frames):
        rotation, translation = pose_at(i, step, turn)
        cam_poses.append(row_major(rotation, translation))
    (poses_dir / "00.txt").write_text("\n".join(cam_poses) + "\n")
    return sequence


def stamp(seconds: float) -> str:
    whole = int(seconds)
    nanos = int(round((seconds - whole) * 1e9))
    minutes, sec = divmod(whole, 60)
    hours, minutes = divmod(minutes, 60)
    return f"2011-09-26 {13 + hours:02d}:{minutes:02d}:{sec:02d}.{nanos:09d}"


def make_raw(root: Path, frames: int, rings: int, per_ring: int, imu_hz: float) -> Path:
    drive = root / "raw" / "2011_09_26" / "2011_09_26_drive_0001_sync"
    velodyne = drive / "velodyne_points"
    starts, mids, ends = [], [], []
    for i in range(frames):
        write_scan(velodyne / "data" / f"{i:010d}.bin", rings, per_ring)
        starts.append(stamp(10.0 + 0.1 * i))
        mids.append(stamp(10.05 + 0.1 * i))
        ends.append(stamp(10.1 + 0.1 * i))
    (velodyne / "timestamps_start.txt").write_text("\n".join(starts) + "\n")
    (velodyne / "timestamps.txt").write_text("\n".join(mids) + "\n")
    (velodyne / "timestamps_end.txt").write_text("\n".join(ends) + "\n")

    oxts = drive / "oxts"
    (oxts / "data").mkdir(parents=True, exist_ok=True)
    # 采样要盖住第一帧起点之前与最后一帧终点之后各一段，否则首末包缺插值括点。
    # 真实 `_sync` 目录里 oxts 是按同步帧取的约 10 Hz 导航状态，不是 100 Hz 惯性流；
    # 默认按 10 Hz 生成，`--imu-hz` 用来造高频变体做对照。
    span = frames * 0.1 + 0.4
    count = max(2, int(round(span * imu_hz)))
    lines = []
    for i in range(count):
        t = 10.0 - 0.2 + i / imu_hz
        lines.append(stamp(t))
        lat = 49.0 + 1.0e-6 * i
        # oxts 一行的字段序（KITTI 原始 README）：
        #   0..2   lat lon alt        3..5   roll pitch yaw
        #   6..10  vn ve vf vl vu     11..13 ax ay az   （车体系比力）
        #   14..16 af al au           17..19 wx wy wz   （车体系角速度）
        #   20..22 wf wl wu           23..25 posacc velacc navstat
        #   26..29 numsats posmode velmode orimode
        row = [lat, 8.4, 110.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 5.0, 0.0, 0.0,
               0.1, 0.0, 9.81,          # ax ay az
               0.1, 0.0, 9.81,          # af al au（此处 roll=pitch=0，两组相同）
               0.0, 0.0, 0.02,          # wx wy wz
               0.0, 0.0, 0.02,          # wf wl wu
               0.05, 0.05, 4, 8, 1, 1, 1]
        (oxts / "data" / f"{i:010d}.txt").write_text(
            " ".join(f"{v:.9f}" if isinstance(v, float) else str(v) for v in row) + "\n")
    (oxts / "timestamps.txt").write_text("\n".join(lines) + "\n")

    date = drive.parent
    (date / "calib_imu_to_velo.txt").write_text(
        "calib_time: 25-May-2012\nR: 1 0 0 0 1 0 0 0 1\nT: -0.81 0.32 -0.80\n")
    (date / "calib_velo_to_cam.txt").write_text(
        "calib_time: 15-Mar-2012\nR: 0 -1 0 0 0 -1 1 0 0\nT: 0 -0.08 -0.27\n")
    return drive


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="/tmp/kitti_fixture", help="输出目录")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--rings", type=int, default=16)
    parser.add_argument("--per-ring", type=int, default=64)
    parser.add_argument("--step", type=float, default=0.5, help="每帧前进（米）")
    parser.add_argument("--turn", type=float, default=0.02, help="每帧转角（弧度）")
    parser.add_argument("--imu-hz", type=float, default=10.0,
                        help="oxts 采样率。真实 _sync 目录是按同步帧取的约 10 Hz 导航状态；"
                             "要造 100 Hz 惯性流的对照夹具就显式传 100")
    args = parser.parse_args()

    root = Path(args.out)
    sequence = make_odometry(root, args.frames, args.rings, args.per_ring, args.step, args.turn)
    drive = make_raw(root, args.frames, args.rings, args.per_ring, args.imu_hz)
    print(f"odometry: {sequence}")
    print(f"raw:      {drive}")
    print("这是合成夹具，不是数据集——精度数字只说明链路通了。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
