#!/usr/bin/env bash
# =============================================================
# 一键启动仿真流程：Gazebo 仿真 + RViz 导航 + 键鼠控制
#
# 用法:
#   ./script/start_oneclick.sh
#   或
#   PB2025_WS_DIR=/path/to/ws ./script/start_oneclick.sh
#
# 说明:
#   - 默认打开 3 个独立终端窗口（沿用方案里的“第一个/第二个/第三个终端”）。
#   - 若想换成同一窗口里的 3 个标签页, 把下方的 OPEN_MODE 改为 "tab" 即可。
#   - 每个命令都会先 source 工作空间的 install/setup.bash。
# =============================================================

set -euo pipefail

# 工作目录 = 脚本所在目录的上一级（即 ROS2 工作空间根目录）
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 允许通过环境变量覆盖
WS_DIR="${PB2025_WS_DIR:-$WS_DIR}"

# 窗口 / 标签页 模式: "window" 每个命令一个独立窗口, "tab" 同一窗口多个标签页
OPEN_MODE="${OPEN_MODE:-window}"

# 终端程序（可改成 konsole / xfce4-terminal 等）
TERMINAL="${TERMINAL:-gnome-terminal}"

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到工作空间的 install/setup.bash, 请确认路径: $WS_DIR" >&2
  exit 1
fi

# 打开一个终端执行指定命令
open_term() {
  local title="$1"
  local cmd="$2"
  # 在终端内先 cd 到工作空间, source 环境, 再执行命令; 结束后保留 shell 便于查看输出
  local full_cmd="cd '$WS_DIR' && source install/setup.bash && ${cmd}; exec bash"

  if [ "$OPEN_MODE" = "tab" ]; then
    "$TERMINAL" --tab --title="$title" -- bash -c "$full_cmd"
  else
    "$TERMINAL" --title="$title" -- bash -c "$full_cmd"
  fi
}

echo "[1/3] 启动 Gazebo 仿真..."
open_term "Gazebo 仿真" "ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"

echo "[2/3] 启动 RViz 导航..."
open_term "RViz 导航" "ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=True use_sim_time:=True use_rviz:=True"

echo "[3/3] 启动键鼠控制..."
open_term "键鼠控制" "ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.5 -p w:=0.5"

echo "已在 ${OPEN_MODE} 模式下启动 3 个终端: Gazebo / RViz / 键鼠控制"
