#!/usr/bin/env bash
# =============================================================
# 从 rosbag 离线跑 Point-LIO, 生成先验点云地图 PCD
#
# 用法（脚本没有可执行位, 用 bash 调用）:
#   BAG=maps/srm_raw_01 OUT=maps/srm_site_01.pcd bash script/build_pcd_from_bag.sh
#   或直接给两个位置参数:
#   bash script/build_pcd_from_bag.sh maps/srm_raw_01 maps/srm_site_01.pcd
#
# 可选环境变量:
#   RATE=1.0              bag 回放倍速（建议 1.0; 太快会丢 IMU 影响建图质量）
#   START_OFFSET=0        从 bag 第几秒开始回放
#   LIO_WAIT_SEC=300      等 Point-LIO 落盘的秒数上限
#   KEEP_NEW_AS_SCANS=0   置 1 则把新点云留在 point_lio/PCD/scans.pcd（会覆盖旧地图）
#   EXTRA_LAUNCH_ARGS=""  追加给 launch 的参数, 例如 lidar_rpy:="0 0 0"
#
# 原理:
#   1) 复用实车建图同款 srm_slam_launch.py, 但:
#        start_lidar:=false  不启雷达驱动, 用 bag 里的 /livox/lidar + /livox/imu
#        save_pcd:=true      累积点云, 退出时落盘
#        use_rviz:=false     无界面
#      gravity / 外参 / 滤波器参数都与实车建图时完全一致, 保证和 slam_toolbox 的
#      栅格图同源同坐标系（点云在 Point-LIO 的 camera_init 即 odom 系下）。
#   2) ros2 bag play 按原始时间戳回放;
#   3) 回放结束后对 point_lio 进程直接发 SIGINT。它主循环是 while (rclcpp::ok()),
#      收到 SIGINT 会跳出循环并把累积点云写成
#      src/pb2025_sentry_nav/point_lio/PCD/scans.pcd
#      （路径由编译期 ROOT_DIR 宏决定, 改不了位置）。
#   4) 把结果拷到 OUT, 并还原被覆盖的旧 scans.pcd。
#
# 注意:
#   - 不要用 kill -9, 那样不会落盘。
#   - 落盘前 memory 里要装下整张图, 大场景请确保内存充足。
# =============================================================

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${PB2025_WS_DIR:-$WS_DIR}"

BAG="${BAG:-${1:-$WS_DIR/maps/srm_raw_01}}"
OUT="${OUT:-${2:-}}"
RATE="${RATE:-1.0}"
START_OFFSET="${START_OFFSET:-0}"
LIO_WAIT_SEC="${LIO_WAIT_SEC:-300}"
KEEP_NEW_AS_SCANS="${KEEP_NEW_AS_SCANS:-0}"
EXTRA_LAUNCH_ARGS="${EXTRA_LAUNCH_ARGS:-}"

PCD_DIR="$WS_DIR/src/pb2025_sentry_nav/point_lio/PCD"
RAW_PCD="$PCD_DIR/scans.pcd"
BACKUP_PCD="$PCD_DIR/scans.pcd.bak"
RUN_DIR="$WS_DIR/.cache/pcd_build"
LAUNCH_LOG="$RUN_DIR/launch.log"

BAG="$(readlink -f "$BAG")"
if [ ! -d "$BAG" ]; then
  echo "[错误] 找不到 rosbag 目录: $BAG" >&2
  exit 1
fi

if [ -z "$OUT" ]; then
  OUT="$WS_DIR/maps/$(basename "$BAG").pcd"
fi
mkdir -p "$(dirname "$OUT")"
OUT="$(readlink -f "$(dirname "$OUT")")/$(basename "$OUT")"

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到 $WS_DIR/install/setup.bash, 请先编译工作空间。" >&2
  exit 1
fi

# 只读环境（例如沙箱）里 $HOME 不可写, ROS 日志会写失败并直接崩掉, 换到工作空间内
if [ ! -w "${HOME:-/nonexistent}" ]; then
  export ROS_HOME="${ROS_HOME:-$WS_DIR/.cache/ros_home}"
  export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROS_HOME/log}"
fi

# ROS 的 setup.bash 会引用未绑定的变量, 必须先关掉 nounset 再 source
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"
set -u

mkdir -p "$RUN_DIR"

LAUNCH_PID=""
cleanup() {
  local status=$?
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[清理] 关闭建图相关节点..."
    kill -INT -"$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
      sleep 0.5
    done
    kill -TERM -"$LAUNCH_PID" 2>/dev/null || true
    sleep 1
    kill -KILL -"$LAUNCH_PID" 2>/dev/null || true
  fi
  # 还原旧的先验地图, 避免影响仿真重定位流程
  if [ -f "$BACKUP_PCD" ] && [ "$KEEP_NEW_AS_SCANS" != "1" ]; then
    mv -f "$BACKUP_PCD" "$RAW_PCD"
    echo "[清理] 已还原原有 $RAW_PCD"
  fi
  return $status
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# 先备份 Point-LIO 固定输出路径上的旧地图（新结果会覆盖它）
if [ -f "$RAW_PCD" ]; then
  echo "[提示] 备份已有 $RAW_PCD -> $BACKUP_PCD"
  mv -f "$RAW_PCD" "$BACKUP_PCD"
fi

echo "[1/4] 启动 Point-LIO（回放模式, save_pcd:=true）"
echo "      日志: $LAUNCH_LOG"
# setsid 让 launch 自成进程组, 方便最后整组清理
# shellcheck disable=SC2086
setsid ros2 launch pb2025_nav_bringup srm_slam_launch.py \
  start_lidar:=false save_pcd:=true use_rviz:=false ${EXTRA_LAUNCH_ARGS} \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

echo "[2/4] 等待 Point-LIO 就绪..."
ready=0
for _ in $(seq 1 120); do
  if pgrep -f pointlio_mapping >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 1
done
if [ "$ready" != "1" ]; then
  echo "[错误] Point-LIO 未启动, 请查看日志: $LAUNCH_LOG" >&2
  tail -n 20 "$LAUNCH_LOG" >&2 || true
  exit 1
fi
sleep 3 # 给订阅和参数一点就绪时间

echo "[3/4] 回放 rosbag: $BAG"
echo "      倍速: $RATE   起始偏移: ${START_OFFSET}s   时长: 见 bag 信息"
ros2 bag play "$BAG" --rate "$RATE" --start-offset "$START_OFFSET"

sleep 2
LIO_PID="$(pgrep -f pointlio_mapping | head -n 1 || true)"
if [ -z "$LIO_PID" ]; then
  echo "[错误] 找不到 point_lio 进程, 无法触发落盘。" >&2
  exit 1
fi

echo "[4/4] 回放结束, 通知 Point-LIO 落盘（SIGINT $LIO_PID）..."
kill -INT "$LIO_PID"
for _ in $(seq 1 "$LIO_WAIT_SEC"); do
  if ! kill -0 "$LIO_PID" 2>/dev/null; then break; fi
  sleep 1
done
if kill -0 "$LIO_PID" 2>/dev/null; then
  echo "[警告] Point-LIO 尚未退出, 落盘可能不完整。" >&2
fi

if [ ! -s "$RAW_PCD" ]; then
  echo "[错误] 没有生成点云文件: $RAW_PCD" >&2
  echo "[错误] 请检查日志: $LAUNCH_LOG" >&2
  exit 1
fi

cp -f "$RAW_PCD" "$OUT"
POINTS="$(sed -n 's/^POINTS[[:space:]]*//p' "$OUT" | head -n 1)"
SIZE="$(du -h "$OUT" | cut -f1)"

echo
echo "[完成] 先验点云地图: $OUT"
echo "       点数: ${POINTS:-未知}   大小: $SIZE"
if [ "$KEEP_NEW_AS_SCANS" = "1" ]; then
  echo "       新的点云已保留在 $RAW_PCD（旧地图仍在 $BACKUP_PCD）"
else
  echo "       仿真用的 $RAW_PCD 已还原; 如需让它指向新地图见 start_nav_reloc.sh 的 PRIOR_PCD_FILE"
fi
