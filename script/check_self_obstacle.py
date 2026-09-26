#!/usr/bin/env python3
# =============================================================
# 查"车自己脚下/身上是不是被判成障碍"
#
# 背景:
#   障碍来自 intensity_voxel_layer 观测到的 terrain_map / terrain_map_ext 点云,
#   再经 inflation_layer 膨胀。如果车自己的云台/枪管/轮子被雷达看到, 点云里就会
#   出现"贴在车身上"的点 -> 那些格子变成致命障碍(254) -> 车所在格变成内切(253)
#   -> 控制器抽样路径点撞到 >=253 直接抛 Collision detected(车不动),
#      Theta* 也会报 "Either of the start or goal pose are an obstacle!"。
#
# 这个脚本把"离车 1.5 m 以内的点"按 车体坐标系(base_link: x 前, y 左, z 上)
# 列出来, 一眼就能看出是自身体(近、在某个固定方向、通常 z 在 0.5 m 以上)
# 还是真障碍(远一点、在地面附近)。
#
# 用法:  source install/setup.bash && python3 script/check_self_obstacle.py
#        可选 --range 1.5   --topics /terrain_map_ext,/terrain_map,/cloud_registered
# 只订阅, 不发指令。
# =============================================================
import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


def read_xyz(msg: PointCloud2):
    """把 PointCloud2 的 x/y/z 字段读成 Nx3 float 数组(不依赖 sensor_msgs_py 版本)"""
    fields = {f.name: f for f in msg.fields}
    if not {"x", "y", "z"} <= set(fields):
        return None
    n = msg.width * msg.height
    if n == 0:
        return None
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    out = np.empty((n, 3), dtype=np.float32)
    for i, name in enumerate(("x", "y", "z")):
        f = fields[name]
        # 逐点按 offset/point_step 取 4 字节 little-endian float
        idx = (np.arange(n) * msg.point_step + f.offset)
        if idx.max() + 4 > buf.size:
            return None
        out[:, i] = buf[idx[:, None] + np.arange(4)].view(np.float32).ravel()
    return out


class SelfCheck(Node):
    def __init__(self, topics, rng):
        super().__init__("self_obstacle_check")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rng = rng
        self.clouds = {}
        self.scans = {}
        for t in topics:
            self.create_subscription(PointCloud2, t,
                                     lambda m, t=t: self.clouds.__setitem__(t, m), 5)
        self.create_subscription(LaserScan, "/obstacle_scan",
                                 lambda m: self.scans.__setitem__("obstacle_scan", m), 5)

    def base_T(self, frame, stamp):
        try:
            tr = self.tf_buffer.lookup_transform("base_link", frame, Time(),
                                                 Duration(seconds=0.5))
        except Exception:  # noqa: BLE001
            try:
                tr = self.tf_buffer.lookup_transform("base_link", frame, Time())
            except Exception as e:  # noqa: BLE001
                return None, str(e)
        t, q = tr.transform.translation, tr.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return (R, np.array([t.x, t.y, t.z])), None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--range", type=float, default=1.5, help="只看离车这么近的点(米)")
    ap.add_argument("--topics", type=str,
                    default="/terrain_map_ext,/terrain_map",
                    help="要检查的点云话题, 逗号分隔")
    ap.add_argument("--wait", type=float, default=8.0)
    args = ap.parse_args()

    rclpy.init()
    node = SelfCheck([t for t in args.topics.split(",") if t.strip()], args.range)
    t0 = node.get_clock().now().nanoseconds / 1e9
    while rclpy.ok() and node.get_clock().now().nanoseconds / 1e9 - t0 < args.wait:
        rclpy.spin_once(node, timeout_sec=0.2)

    print("=" * 74)
    print(f" 离车 {args.range} m 以内的点云 (base_link: +x 前 / +y 左 / +z 上)")
    print("=" * 74)
    for topic, msg in node.clouds.items():
        pts = read_xyz(msg)
        if pts is None or len(pts) == 0:
            print(f"\n--- {topic}: 没数据/读不出 xyz ---")
            continue
        tr, err = node.base_T(msg.header.frame_id, msg.header.stamp)
        if tr is None:
            print(f"\n--- {topic}: TF base_link<-{msg.header.frame_id} 失败: {err}")
            continue
        R, t = tr
        p = (R @ pts.T).T + t
        finite = np.isfinite(p).all(axis=1)
        p = p[finite]
        d = np.linalg.norm(p, axis=1)
        near = p[d <= args.range]
        print(f"\n--- {topic} (frame={msg.header.frame_id}, {len(p)} 点有效)")
        print(f"    1.5m 内: {len(near)} 点", end="")
        if len(near):
            dd = np.linalg.norm(near, axis=1)
            print(f"   最近 {dd.min():.3f} m   高度 z: "
                  f"{near[:,2].min():+.2f} ~ {near[:,2].max():+.2f} m")
            order = np.argsort(dd)[:12]
            print("    最近的点 (x前, y左, z上, 距离):")
            for i in order:
                x, y, z = near[i]
                print(f"      x={x:+.3f} y={y:+.3f} z={z:+.3f}  d={math.hypot(x, y, z):.3f}"
                      f"   方位={math.degrees(math.atan2(y, x)):+7.1f}deg")
            # 按方位分桶, 看是不是集中在某个方向(自身体)
            ang = np.degrees(np.arctan2(near[:, 1], near[:, 0]))
            hist, edges = np.histogram(ang, bins=12, range=(-180, 180))
            print("    方位分布(每30度): " + " ".join(
                f"{int(edges[i])}:{hist[i]}" for i in range(len(hist))))
        else:
            print()

    sc = node.scans.get("obstacle_scan")
    if sc is not None:
        r = np.asarray(sc.ranges, dtype=float)
        ok = np.isfinite(r)
        if ok.any():
            i = int(np.argmin(np.where(ok, r, np.inf)))
            a = math.degrees(sc.angle_min + i * sc.angle_increment)
            print(f"\n--- /obstacle_scan: 最近回波 {r[i]:.3f} m @ {a:+.1f} deg"
                  f" (range_min={sc.range_min:.2f}, range_max={sc.range_max:.2f})")
            near = ok & (r < 1.0)
            print(f"    <1.0 m 的回波数: {int(near.sum())}")
    print("\n" + "=" * 74)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
