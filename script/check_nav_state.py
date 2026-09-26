#!/usr/bin/env python3
# =============================================================
# 导航现场体检: 一次性把"车为什么不去/不动"需要的数据抓齐
#
# 查什么:
#   1) 车在哪 (map->base_link), 用固定坐标系 map
#   2) 面板上现在的航点 (/waypoints MarkerArray) 与车的距离、朝向
#   3) 全局代价地图的代价分布, 以及 车/航点 所在格的代价
#      -> 254 致命障碍 / 253 内切膨胀 / 255 未知, 这三个都会让
#         Theta* 报 "Either of the start or goal pose are an obstacle!"
#   4) 当前全局路径 /plan 上每个采样点在 局部代价地图 里的代价
#      -> 复现控制器 isCollisionDetected(): 代价 >=253 就抛
#         "Collision detected in the trajectory", 车直接不动
#   5) 车周围 2m x 2m 的代价地图 ASCII 图, 看墙/膨胀在哪一侧
#
# 用法:
#   source install/setup.bash
#   python3 script/check_nav_state.py                 # 默认
#   python3 script/check_nav_state.py --radius 2.5    # ASCII 图范围(米)
#
# 注意: 只订阅, 不发任何指令, 不影响正在跑的导航。
# =============================================================
import argparse
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

# 注意: OccupancyGrid 话题里发布的是"发布值", 不是 costmap 内部的 0..255:
#   内部 254(LETHAL)  -> 发布 100
#   内部 253(INSCRIBED) -> 发布 99
#   内部 255(NO_INFORMATION) -> 发布 -1
#   其余内部代价 n -> 发布 n*100/255
# 控制器插件用的是内部值(判据 >=253), 所以这里判"内切及以上"用发布值 >= 99。
COST_NAME = {0: "FREE", 99: "INSCRIBED(253)", 100: "LETHAL(254)"}


def cost_name(c):
    if c is None:
        return ""
    if c < 0:
        return "UNKNOWN(255)"
    return COST_NAME.get(c, f"cost{c}(内部≈{int(round(c*255/100))})")


def is_obstacle(c):
    """控制器/Theta* 眼里的障碍: 内部代价 >= 253, 即发布值 >= 99"""
    return c is not None and c >= 99


def latched_qos():
    return QoSProfile(depth=1,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


class Diag(Node):
    def __init__(self):
        super().__init__("nav_state_diag")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.global_cost = None
        self.local_cost = None
        self.static_map = None
        self.waypoints = None
        self.plan = None
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._g, latched_qos())
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 self._l, latched_qos())
        self.create_subscription(OccupancyGrid, "/map", self._m, latched_qos())
        self.create_subscription(MarkerArray, "/waypoints", self._w, 10)
        self.create_subscription(Path, "/plan", self._p, 10)

    def _g(self, m):
        self.global_cost = m

    def _l(self, m):
        self.local_cost = m

    def _m(self, m):
        self.static_map = m

    def _w(self, m):
        self.waypoints = m

    def _p(self, m):
        self.plan = m

    # ---------- 工具 ----------
    def robot_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), Duration(seconds=0.5))
        except Exception as e:  # noqa: BLE001
            print(f"[!] 取 map->base_link 失败: {e}")
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    @staticmethod
    def cost_at(grid, x, y):
        """返回 (代价, 行列) 或 (None, None)"""
        info = grid.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if col < 0 or row < 0 or col >= info.width or row >= info.height:
            return None, (col, row)
        return grid.data[row * info.width + col], (col, row)

    def ascii_map(self, grid, cx, cy, half):
        info = grid.info
        res = info.resolution
        n = int(half / res)
        c0 = int((cx - info.origin.position.x) / res)
        r0 = int((cy - info.origin.position.y) / res)
        rows = []
        for dr in range(n, -n - 1, -1):          # 上=+y
            line = []
            for dc in range(-n, n + 1):
                c, r = c0 + dc, r0 + dr
                if c < 0 or r < 0 or c >= info.width or r >= info.height:
                    line.append(" ")
                    continue
                v = int(grid.data[r * info.width + c])
                if dc == 0 and dr == 0:
                    line.append("R")             # 车所在格
                elif v < 0:
                    line.append("?")             # 未知(255)
                elif v == 100:
                    line.append("#")             # 致命障碍(254)
                elif v == 99:
                    line.append("o")             # 内切膨胀(253) -> 控制器判碰撞
                elif v == 0:
                    line.append(".")             # 可走
                elif v >= 90:
                    line.append("+")             # 高代价(接近内切)
                else:
                    line.append("-")             # 低代价(膨胀衰减区)
            rows.append("".join(line))
        return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=1.2,
                    help="ASCII 图半径(米), 默认 1.2")
    ap.add_argument("--wait", type=float, default=12.0,
                    help="等待话题的时间(秒)")
    ap.add_argument("--points", type=str, default="",
                    help='额外要查的点, 形如 "0.33,-0.39 1.00,-1.78"')
    args = ap.parse_args()

    rclpy.init()
    node = Diag()
    t0 = node.get_clock().now().nanoseconds / 1e9
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        got = all(v is not None for v in
                  (node.global_cost, node.local_cost, node.static_map))
        if got and node.get_clock().now().nanoseconds / 1e9 - t0 > 2.0:
            break
        if node.get_clock().now().nanoseconds / 1e9 - t0 > args.wait:
            break

    print("=" * 74)
    print(" 导航现场体检")
    print("=" * 74)

    pose = node.robot_pose()
    if pose is None:
        print("[!] 取不到车的位置, 后面的判断会缺参照")
    else:
        x, y, yaw = pose
        print(f"车位姿(map): x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+7.1f} deg")

    wps = []
    if node.waypoints is not None:
        for mk in node.waypoints.markers:
            if mk.type == 2 and mk.action != 2:      # SPHERE = 航点位置
                wps.append((mk.pose.position.x, mk.pose.position.y))
        print(f"面板航点数(/waypoints 里的红球): {len(wps)}")
    else:
        print("[/waypoints] 还没收到(面板里没设航点?)")

    for name, grid in (("静态地图 /map", node.static_map),
                       ("全局代价 /global_costmap/costmap", node.global_cost),
                       ("局部代价 /local_costmap/costmap", node.local_cost)):
        if grid is None:
            print(f"\n--- {name}: 没收到 ---")
            continue
        data = np.asarray(grid.data, dtype=np.int16)
        vals, cnt = np.unique(data, return_counts=True)
        top = ", ".join(f"{int(v)}:{int(c)}({cost_name(int(v))})"
                        for v, c in sorted(zip(vals, cnt), key=lambda z: -z[1])[:6])
        print(f"\n--- {name}: {grid.info.width}x{grid.info.height} @"
              f"{grid.info.resolution:.3f}m  原点({grid.info.origin.position.x:.2f},"
              f"{grid.info.origin.position.y:.2f})")
        print(f"    代价分布 top: {top}")
        def _cnt(v):
            return int(cnt[list(vals).index(v)]) if v in vals else 0
        n_obs = sum(_cnt(v) for v in vals if v >= 99)
        print(f"    致命(100)={_cnt(100)}  内切(99)={_cnt(99)}  未知(-1)={_cnt(-1)}"
              f"  障碍类合计(>=99)={n_obs}")

    # ---- 车到最近障碍/内切格的距离: 决定控制器会不会直接拒绝动 ----
    if node.global_cost is not None and pose is not None:
        g = node.global_cost
        info, res = g.info, g.info.resolution
        data = np.asarray(g.data, dtype=np.int16).reshape(info.height, info.width)
        col = int((pose[0] - info.origin.position.x) / res)
        row = int((pose[1] - info.origin.position.y) / res)
        R = 60                                   # 3 m 窗口
        r0, r1 = max(0, row-R), min(info.height, row+R+1)
        c0, c1 = max(0, col-R), min(info.width, col+R+1)
        win = data[r0:r1, c0:c1]
        yy, xx = np.mgrid[r0:r1, c0:c1]
        dist = np.hypot(xx-col, yy-row) * res
        for lab, m in (("致命障碍(100)", win == 100), ("内切膨胀(>=99)", win >= 99)):
            if m.any():
                d = dist[m].min()
                j = np.argmin(np.where(m, dist, 1e9))
                print(f"    {lab}: 最近 {d:.2f} m (在格({xx.ravel()[j]},{yy.ravel()[j]}),"
                      f" map({info.origin.position.x + xx.ravel()[j]*res:+.2f},"
                      f"{info.origin.position.y + yy.ravel()[j]*res:+.2f}))")
            else:
                print(f"    {lab}: 3 m 内没有")
        if win[row-r0, col-c0] >= 99:
            print("    [!] 车自己所在的格已是 内切/致命: 控制器抽样的路径点必然含"
                  " >=253 的格 -> 直接抛 Collision detected, 车一步都动不了")

    # 车 / 航点 在全局代价地图上的代价 = Theta* 判定用的东西
    if node.global_cost is not None:
        g = node.global_cost
        print("\n--- Theta* 起点/终点判定 (全局代价地图) ---")
        if pose:
            c, rc = node.cost_at(g, pose[0], pose[1])
            print(f"    起点(车) 格{rc} 代价={c} {cost_name(c) if c is not None else ''}"
                  + ("   <== 致命/内切, 规划直接抛 "
                     "'Either of the start or goal pose are an obstacle!'"
                     if is_obstacle(c) else ""))
        for i, (wx, wy) in enumerate(wps, 1):
            c, rc = node.cost_at(g, wx, wy)
            d = math.hypot(wx - pose[0], wy - pose[1]) if pose else float("nan")
            bad = "   <== 规划会拒绝这个航点" if is_obstacle(c) else ""
            print(f"    wp_{i} ({wx:+.2f},{wy:+.2f}) 距车{d:5.2f}m "
                  f"格{rc} 代价={c} {cost_name(c) if c is not None else ''}{bad}")

    # 额外指定的点(比如日志里规划失败的那些目标点)
    if args.points.strip() and node.global_cost is not None:
        print("\n--- 指定点体检 (静态地图 / 全局代价 / 局部代价) ---")
        for tok in args.points.replace(";", " ").split():
            try:
                px, py = (float(v) for v in tok.split(","))
            except ValueError:
                print(f"    跳过无法解析的 '{tok}'")
                continue
            cells = []
            for lab, gr in (("map", node.static_map), ("全局", node.global_cost),
                            ("局部", node.local_cost)):
                if gr is None:
                    cells.append(f"{lab}=?")
                    continue
                c, rc = node.cost_at(gr, px, py)
                cells.append(f"{lab}:{c}{'' if c is None else '(' + cost_name(c) + ')'}")
            d = math.hypot(px - pose[0], py - pose[1]) if pose else float("nan")
            print(f"    ({px:+.2f},{py:+.2f}) 距车{d:5.2f}m  " + "  ".join(cells))

    # 复现控制器的碰撞检查
    if node.plan is not None and node.local_cost is not None and node.plan.poses:
        lc = node.local_cost
        print(f"\n--- 控制器碰撞检查复现 (/plan {len(node.plan.poses)} 点, "
              f"按代码抽 10 点, 判据 代价>=253) ---")
        n = len(node.plan.poses)
        bad = []
        for i in range(10):
            idx = min(i * n // 10, n - 1)
            p = node.plan.poses[idx].pose.position
            c, rc = node.cost_at(lc, p.x, p.y)
            tag = ""
            if c is None:
                tag = "(局部地图外, 代码里算无碰撞)"
            elif is_obstacle(c):
                tag = "<== 判定为碰撞!"
                bad.append(i)
            print(f"    #{i:2d} ({p.x:+.2f},{p.y:+.2f}) 局部代价={c} "
                  f"{cost_name(c) if c is not None else ''} {tag}")
        print(f"    => {'会抛 Collision detected, 车不动' if bad else '可通过, 会正常发速度'}")
    else:
        print("\n--- 没有 /plan 或 /local_costmap/costmap, 跳过碰撞检查复现 ---")

    if node.global_cost is not None and pose is not None:
        print(f"\n--- 车周围 {args.radius*2:.1f}m x {args.radius*2:.1f}m "
              f"全局代价图 (上=+y, 右=+x, R=车) ---")
        print("     图例: . 可走   - 低代价   + 高代价   o 膨胀(>=253)   "
              "# 致命障碍   ? 未知")
        for line in node.ascii_map(node.global_cost, pose[0], pose[1], args.radius):
            print("     " + line)

    print("\n" + "=" * 74)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
