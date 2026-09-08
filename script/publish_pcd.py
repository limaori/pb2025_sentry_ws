#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""发布 PCD 点云到 ROS2 话题, 用于在 RViz 中可视化先验点云地图。

用法:
  python3 script/publish_pcd.py [pcd路径] [frame_id] [话题名]
  # 默认: src/pb2025_sentry_nav/point_lio/PCD/scans.pcd, frame=map, topic=prior_map

示例(配合 start_nav_reloc.sh 的导航 RViz, 与实时点云对比):
  python3 script/publish_pcd.py \
    src/pb2025_sentry_nav/point_lio/PCD/scans.pcd map prior_map \
    --ros-args -r __ns:=/red_standard_robot1
  再在 RViz(Fixed Frame=map) Add -> PointCloud2, Topic 选 /red_standard_robot1/prior_map。
  RViz 会保留最近一帧, 无需一直发(也可以加 -p repub_rate:=0.2 慢速重发)。

支持二进制/ASCII PCD, 读取 x y z 字段(其余忽略)。
"""
import sys
import io

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


def read_xyz_pcd(path, max_points=None):
    """解析 PCD(x y z 字段), 返回 Nx3 numpy 数组(可选抽样)。"""
    with open(path, "rb") as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.strip().startswith(b"DATA"):
                # 其余为点云数据, 在 with 内一次性读出
                body = f.read()
                break
    fields, sizes, data_type = [], [], None
    npoints = 0
    for line in header.decode("ascii", "replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "FIELDS":
            fields = parts[1:]
        elif parts[0] == "SIZE":
            sizes = [int(v) for v in parts[1:]]
        elif parts[0] in ("POINTS", "WIDTH"):
            npoints = int(parts[1])
        elif parts[0] == "DATA":
            data_type = parts[1].strip()

    if "x" not in fields or "y" not in fields or "z" not in fields:
        raise RuntimeError("PCD 缺少 x/y/z 字段: %s" % fields)
    xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")

    if data_type == "binary":
        raw = np.frombuffer(body, dtype=np.float32)
        ncols = len(fields)
        rows = raw.reshape(-1, ncols)[:npoints]
    else:  # ascii
        rows = np.loadtxt(io.BytesIO(body), dtype=np.float32)
        rows = rows[:npoints]

    xyz = rows[:, [xi, yi, zi]]

    # 大文件抽样, 避免一次发布 168MB 消息
    if max_points is not None and xyz.shape[0] > max_points:
        idx = np.linspace(0, xyz.shape[0] - 1, max_points).astype(np.int64)
        xyz = xyz[idx]
    return xyz


def main():
    pcd = sys.argv[1] if len(sys.argv) > 1 else \
        "/home/srm/pb2025_sentry_ws/src/pb2025_sentry_nav/point_lio/PCD/scans.pcd"
    frame = "map"
    topic = "prior_map"
    if len(sys.argv) >= 3:
        frame = sys.argv[2]
    if len(sys.argv) >= 4:
        topic = sys.argv[3]

    print("读取 PCD: %s" % pcd)
    xyz = read_xyz_pcd(pcd, max_points=1000000)
    print("点数: %d" % len(xyz))

    rclpy.init()
    node = rclpy.create_node("pcd_publisher")
    node.declare_parameter("frame_id", frame)
    node.declare_parameter("repub_rate", 1.0)  # Hz; RViz 保留最近一帧, 0.2 即可
    f = node.get_parameter("frame_id").value
    rate = node.get_parameter("repub_rate").value
    node.declare_parameter("fix_mount", True)
    fix_mount = node.get_parameter("fix_mount").value

    # 先验 PCD 保存在 Point-LIO 的 camera_init(=初始雷达系, 含安装 roll30°/yaw90°),
    # 默认用 TF(base_footprint->front_mid360, 即安装位姿)把它修正到世界系,
    # 使 RViz 里的先验地图与 2D 地图方向一致。
    if fix_mount:
        try:
            import tf2_ros
            buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(buf, node)
            t = None
            for _ in range(40):  # 等静态TF到达
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    t = buf.lookup_transform("base_footprint", "front_mid360", rclpy.time.Time())
                    break
                except Exception:
                    continue
            if t is not None:
                q = t.transform.rotation
                x, y, z, w = q.x, q.y, q.z, q.w
                R = np.array([
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ])
                tv = np.array([
                    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
                ])
                xyz = xyz @ R.T + tv
                print("已应用安装位姿修正 (TF, t=%s)" % tv.round(3))
            else:
                # 兜底: mid360 硬编码安装位姿 pose="0.16 0.0 0.18 roll=30° yaw=90°"
                print("[警告] 未等到 TF, 使用硬编码安装位姿修正。")
                roll, yaw = np.deg2rad(30.0), np.deg2rad(90.0)
                Rx = np.array([[1, 0, 0],
                               [0, np.cos(roll), -np.sin(roll)],
                               [0, np.sin(roll), np.cos(roll)]])
                Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                               [np.sin(yaw), np.cos(yaw), 0],
                               [0, 0, 1]])
                R = Rz @ Rx
                tv = np.array([0.16, 0.0, 0.18])
                xyz = xyz @ R.T + tv
        except Exception as e:
            print("[警告] 修正失败(%s), 输出原始点云。" % e)

    # 用 transient_local 让晚订阅的 RViz 也能立即拿到
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    pub = node.create_publisher(PointCloud2, topic, qos)

    # 直接用 numpy 构造 PointCloud2, 5M 点也很快
    from std_msgs.msg import Header
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = f
    msg.height = 1
    msg.width = len(xyz)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(xyz)
    msg.is_dense = True
    msg.data = xyz.astype(np.float32).tobytes()
    msg.header.stamp = node.get_clock().now().to_msg()

    print("发布到话题 %s (frame=%s, %.1fHz), Ctrl+C 退出..." % (topic, f, rate))
    period = 1.0 / rate if rate > 0 else 0.0
    while rclpy.ok():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        if period > 0:
            rclpy.spin_once(node, timeout_sec=period)
        else:
            break


if __name__ == "__main__":
    main()
