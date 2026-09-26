#!/usr/bin/env bash
# =============================================================
# 一键抓取"导航急冲 / 定位跳变"现场数据
#
# 用途:
#   复现"车到某个地方突然很快地往前窜"时, 把判断所需的一切一次性抓齐:
#     1) rosbag: 速度链(cmd_vel_controller / cmd_vel_nav2_result / cmd_vel_chassis)、
#        代价地图、全局/局部路径、前视点、TF、里程计、动作(action)状态、/rosout
#     2) 实时监视 /cmd_vel_chassis: 一旦 |v| > 阈值(默认 0.6 m/s, 高于控制器
#        v_linear_max 0.5) 就自动往 marks.txt 写一条带时间戳的记录
#     3) 手动标记: 录制中按回车打一条标记(可带文字), 输入 q 回车提前停止
#     4) 停止后自动收拢: ros2 bag info、节点/话题清单、nav2 各节点 param dump、
#        当前加载的地图信息、以及本次运行新产生的 ~/.ros/log 日志文件
#
# 用法:
#   bash ./script/capture_nav_lurch.sh                     # 导航已另起, 只负责抓
#   bash ./script/capture_nav_lurch.sh --with-nav          # 顺带用 start_real_nav.sh 起导航
#   bash ./script/capture_nav_lurch.sh -n                  # 只解析打印, 不录制
#   bash ./script/capture_nav_lurch.sh -d 1800             # 最长录 30 分钟(默认 20 分钟)
#   bash ./script/capture_nav_lurch.sh --with-terrain      # 额外录 terrain_map(_ext) 点云(体积大)
#   bash ./script/capture_nav_lurch.sh -o ~/bags/test1     # 指定输出目录
#
# 常用可选项:
#   -o, --out DIR        输出目录 (默认 ~/bags/lurch_<月日_时分秒>)
#   -d, --duration SEC   最长录制时长, 到点自动停 (默认 1200)
#   --max-size-mb N      bag 超过 N MB 自动停, 防止把磁盘写满 (默认 3000)
#   --thresh V           /cmd_vel_chassis 自动标记阈值, m/s (默认 0.6)
#   --no-watch           关闭 cmd_vel 实时监视(不自动打标记)
#   --with-nav           先调用 script/start_real_nav.sh 起导航, 等 Nav2 起来再录
#   --nav-args "..."     --with-nav 时传给 start_real_nav.sh 的参数
#                        (默认 "--lio -m xjl0914 --map-to-odom 0.0 0.0 0.0")
#   --with-terrain       连 terrain_map / terrain_map_ext 一起录(点云, 可能几 MB/s)
#   --no-qos-overrides   不使用 QoS 覆盖文件(默认使用, 否则 /map、/tf_static、
#                        两个 costmap 是 transient_local, 会订不到)
#   --force              话题匹配少于 5 个时也硬录(默认直接报错退出)
#   -n, --check          只解析并打印将要执行的内容
#   -h, --help           显示帮助
#
# 录制中(前台)交互:
#   回车              -> 打一条标记(事后对齐"车窜的那一下")
#   <任意文字>回车    -> 打一条带说明的标记
#   q 回车            -> 停止录制并收拢数据(Ctrl+C 同样可以)
#
# 抓完之后把整个输出目录(或它的路径)给分析方即可, 关键文件:
#   bag/             rosbag2 数据
#   bag_info.txt     ros2 bag info
#   marks.txt        自动/手动时间戳标记(epoch 秒, 与 bag、~/.ros/log 同一时间轴)
#   meta.txt         本次命令、时间、话题订阅数、地图信息
#   params/          behavior_server / controller_server / fake_vel_transform 等 param dump
#   logs/            本次运行新产生的 ~/.ros/log 日志
#
# 注意:
#   - 本脚本只读现场、只写自己的输出目录, 不改动工作空间里任何代码或配置。
#   - 复现 1 m/s 窜车有风险: 场地清空、遥控器放手边。
#   - 不要同时录 /livox/lidar (4 分钟就 ~1 GB)。需要点云请用 --with-terrain 短录。
# =============================================================

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${PB2025_WS_DIR:-$WS_DIR}"

# ---- 默认参数 --------------------------------------------------------------
STAMP="$(date +%m%d_%H%M%S)"
OUT_DIR="${HOME}/bags/lurch_${STAMP}"
DURATION=1200
MAX_SIZE_MB=3000
THRESH="0.6"
USE_WATCH=1
WITH_NAV=0
NAV_ARGS="--lio -m xjl0914 --map-to-odom 0.0 0.0 0.0"
WITH_TERRAIN=0
USE_QOS=1
DRY_RUN=0
FORCE=0

# ---- 想要的话题(不存在的会自动跳过) ----------------------------------------
WANT_TOPICS=(
  # 时间轴 / 日志
  /tf
  /tf_static
  /rosout
  # 定位与位姿
  /odometry
  /map
  # 速度链"三件套"(核心证据)
  /cmd_vel_controller        # 控制器输出, 应 <= v_linear_max
  /cmd_vel_nav2_result       # velocity_smoother 输出 + 恢复行为输出 混在这里
  /cmd_vel_chassis           # 底盘真正收到的
  # 路径 / 前视点
  /plan
  /local_plan
  /lookahead_point
  /received_global_plan
  # 代价地图(规划/控制的实际输入)
  /local_costmap/costmap
  /global_costmap/costmap
  /local_costmap/published_footprint
  /global_costmap/published_footprint
  # 动作状态: 谁在开车、什么时候开的
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/feedback
  /navigate_through_poses/_action/status
  /follow_path/_action/status
  /follow_path/_action/feedback
  /compute_path_to_pose/_action/status
  /backup/_action/status
  /backup/_action/feedback
  /spin/_action/status
  /drive_on_heading/_action/status
  /follow_waypoints/_action/status
  # 可选点云(体积大, --with-terrain 打开)
  /terrain_map
  /terrain_map_ext
)
TERRAIN_TOPICS=(/terrain_map /terrain_map_ext)

# transient_local 的话题必须显式声明 QoS, 否则录不到
QOS_TOPICS=(/tf_static /map /local_costmap/costmap /global_costmap/costmap)

q() { printf '%q' "$1"; }

usage() {
  awk 'NR > 1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

# ---- 参数解析 --------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    -o | --out) OUT_DIR="${2:?--out 需要目录}"; shift 2 ;;
    -d | --duration) DURATION="${2:?--duration 需要秒数}"; shift 2 ;;
    --max-size-mb) MAX_SIZE_MB="${2:?--max-size-mb 需要数值}"; shift 2 ;;
    --thresh) THRESH="${2:?--thresh 需要数值}"; shift 2 ;;
    --no-watch) USE_WATCH=0; shift ;;
    --with-nav) WITH_NAV=1; shift ;;
    --nav-args) NAV_ARGS="${2:?--nav-args 需要参数串}"; shift 2 ;;
    --with-terrain) WITH_TERRAIN=1; shift ;;
    --no-qos-overrides) USE_QOS=0; shift ;;
    --force) FORCE=1; shift ;;
    -n | --check) DRY_RUN=1; shift ;;
    -h | --help) usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 --help 查看用法)" >&2; exit 1 ;;
  esac
done

OUT_DIR="${OUT_DIR/#\~/$HOME}"
BAG_DIR="$OUT_DIR/bag"

# ---- 环境 ------------------------------------------------------------------
# 注意: ROS 的 setup.bash 不是 set -u / set -e 安全的(会引用未定义变量), 先把开关关掉
set +eu
if [ -f "$WS_DIR/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS_DIR/install/setup.bash"
elif [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
set -eu
if ! command -v ros2 >/dev/null 2>&1; then
  echo "[错误] 找不到 ros2, 请先 source /opt/ros/humble/setup.bash" >&2
  exit 1
fi

START_EPOCH="$(date +%s)"
START_ISO="$(date -Ins)"

# ---- 需要的话题: 与现场实际存在的话题取交集 --------------------------------
echo "========================================================="
echo " 抓取导航现场数据"
echo "   输出目录  : $OUT_DIR"
echo "   起始时间  : $START_ISO (epoch $START_EPOCH)"
echo "   最长时长  : ${DURATION}s    体积上限: ${MAX_SIZE_MB}MB"
echo "   自动标记  : $([ "$USE_WATCH" = 1 ] && echo "开 (|v| > ${THRESH} m/s)" || echo 关)"
echo "========================================================="
echo

# ---- 现场进程自检: 是否已有导航栈在跑 --------------------------------------
NAV_PROCS="$(ps -u "$(id -u)" -o pid,etime,cmd 2>/dev/null | grep -E \
  '[b]ringup_launch\.py|[c]omponent_container_isolated|[s]tandard_robot_pp_ros2_node|[l]ivox_ros_driver2_node|[p]ointlio_mapping' || true)"
NAV_PROC_N="$(printf '%s\n' "$NAV_PROCS" | grep -c . || true)"
if [ "${NAV_PROC_N:-0}" -gt 0 ]; then
  echo "[0/6] 检测到已有 $NAV_PROC_N 个导航/底盘/雷达进程在运行:"
  printf '%s\n' "$NAV_PROCS" | cut -c1-160 | sed 's/^/      /'
  echo "      (正常情况: 就是你现在要抓的那一套。但同一套栈只能有一份,"
  echo "       否则两个 fake_vel_transform 会同时往 /cmd_vel_chassis 发速度)"
  if [ "$WITH_NAV" = 1 ]; then
    echo "      [警告] 你传了 --with-nav 但栈已在运行: start_real_nav.sh 会跳过已在跑的步骤," >&2
    echo "      [警告] 可能出现新旧混杂, 建议先清干净再启动:" >&2
    echo "      [警告]   pkill -TERM -f 'ros2 launch pb2025_nav_bringup'" >&2
    echo "      [警告]   pkill -TERM -f 'component_container_isolated'" >&2
    echo "      [警告]   pkill -TERM -f 'standard_robot_pp_ros2.launch.py'" >&2
    echo "      [警告]   pkill -TERM -f 'livox_ros_driver2_node'; pkill -TERM -f 'rviz2'" >&2
  fi
  echo
fi

# ---- 起导航(可选): 必须在"扫描话题"之前 ------------------------------------
# 顺序很关键: 先起栈, 再扫话题。反过来的话 --with-nav 会扫到一个还没起栈的环境,
# 结果只录到 /rosout —— 这是 19:05 那次实测踩出来的坑。
if [ "$WITH_NAV" = 1 ]; then
  echo "[1/6] 先启动导航: start_real_nav.sh $NAV_ARGS"
  ( cd "$WS_DIR" && bash ./script/start_real_nav.sh $NAV_ARGS ) || true
  echo "      等待 Nav2 起来 (最长等 150s)..."
  ok=0
  for _ in $(seq 1 150); do
    tl="$(ros2 topic list --include-hidden-topics 2>/dev/null || true)"
    if printf '%s\n' "$tl" | grep -Fxq /cmd_vel_chassis \
      && printf '%s\n' "$tl" | grep -Fxq /cmd_vel_controller \
      && printf '%s\n' "$tl" | grep -Fxq /local_costmap/costmap \
      && printf '%s\n' "$tl" | grep -Fxq /map; then
      ok=1; break
    fi
    sleep 1
  done
  if [ "$ok" = 1 ]; then
    echo "      Nav2 已就绪, 再等 8s 让 costmap / 定位稳定..."
    sleep 8
  else
    echo "      [错误] 等了 150s 也没等到 Nav2 的关键话题(看导航终端是不是起失败了)。" >&2
    echo "      [错误] 强行录只会得到一个只有 /rosout 的废包, 已退出。" >&2
    exit 1
  fi
  echo
else
  echo "[1/6] 使用已在运行的导航栈 (未传 --with-nav)"
  echo
fi

echo "[2/6] 扫描现有话题..."
if ! RAW_TOPICS="$(ros2 topic list --include-hidden-topics 2>/dev/null)"; then
  echo "[警告] ros2 topic list 失败(DDS/daemon 问题?), 稍后重试一次" >&2
  sleep 2
  RAW_TOPICS="$(ros2 topic list --include-hidden-topics 2>/dev/null || true)"
fi

has_topic() { printf '%s\n' "$RAW_TOPICS" | grep -Fxq "$1"; }

TOPICS=()
MISSING=()
for t in "${WANT_TOPICS[@]}"; do
  if [ "$WITH_TERRAIN" = 0 ]; then
    skip=0
    for tt in "${TERRAIN_TOPICS[@]}"; do [ "$t" = "$tt" ] && skip=1; done
    [ "$skip" = 1 ] && continue
  fi
  if has_topic "$t"; then TOPICS+=("$t"); else MISSING+=("$t"); fi
done

if [ "${#TOPICS[@]}" -eq 0 ] || { [ "${#TOPICS[@]}" -lt 5 ] && [ "$FORCE" != 1 ]; }; then
  echo "[错误] 只匹配到 ${#TOPICS[@]} 个想要的话题——导航栈没起来 / 话题名不对。" >&2
  echo "[错误] 这样录出来的包基本没用。请确认导航已启动并 source 了同一个工作空间," >&2
  echo "[错误] 或用 --with-nav 让本脚本自己起栈; 确实要硬录请加 --force。" >&2
  printf '[错误] 当前能看到的候选: %s\n' "${TOPICS[*]:-<空>}" >&2
  exit 1
fi
echo "      将录制 ${#TOPICS[@]} 个话题"
if [ "${#MISSING[@]}" -gt 0 ]; then
  printf '      跳过(当前不存在): %s\n' "${MISSING[*]}"
fi

# 关键话题的发布者数量自检(能提前发现"节点没起/串口没插"这类问题)
echo
echo "[3/6] 关键话题发布者自检..."
KEY_TOPICS=(/cmd_vel_chassis /cmd_vel_controller /cmd_vel_nav2_result /local_costmap/costmap /global_costmap/costmap /odometry /tf)
for t in "${KEY_TOPICS[@]}"; do
  if has_topic "$t"; then
    n="$(ros2 topic info "$t" 2>/dev/null | awk '/Publisher count/{print $3}' || true)"
    printf '      %-28s publisher=%s%s\n' "$t" "${n:-?}" "$([ "${n:-0}" = "0" ] && echo '   <== 没有发布者!' || true)"
  else
    printf '      %-28s (不存在)\n' "$t"
  fi
done
if ! has_topic /cmd_vel_chassis; then
  echo "      [警告] /cmd_vel_chassis 不存在: 导航栈(+fake_vel_transform)没起来, 车不会动" >&2
fi
if [ -e /dev/ttyACM0 ]; then
  printf '      %s\n' "$(ls -l /dev/ttyACM0)"
  [ -w /dev/ttyACM0 ] || echo "      [警告] 当前用户对 /dev/ttyACM0 没有写权限: 加 sudo chmod 666 /dev/ttyACM0, 否则车不会动" >&2
else
  echo "      [警告] /dev/ttyACM0 不存在: 底盘串口没插, 车不会动" >&2
fi

# ---- QoS 覆盖文件 ----------------------------------------------------------
QOS_FILE="$OUT_DIR/qos_nav.yaml"
qos_yaml_body() {
  local t
  for t in "${QOS_TOPICS[@]}"; do
    cat <<EOF
${t}:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 100
EOF
  done
}

# ---- 组装命令 --------------------------------------------------------------
REC_BASE=(ros2 bag record -o "$BAG_DIR" --include-hidden-topics)
REC_CMD=("${REC_BASE[@]}")
[ "$USE_QOS" = 1 ] && REC_CMD+=(--qos-profile-overrides-path "$QOS_FILE")
REC_CMD+=("${TOPICS[@]}")

echo
echo "[4/6] 录制命令:"
printf '      %s\n' "${REC_CMD[*]}"

if [ "$DRY_RUN" = 1 ]; then
  echo
  echo "[--check] 只解析, 未启动。输出目录将是: $OUT_DIR"
  exit 0
fi

# ---- 准备输出目录 ----------------------------------------------------------
mkdir -p "$OUT_DIR" "$OUT_DIR/params" "$OUT_DIR/logs"
if [ "$USE_QOS" = 1 ]; then
  qos_yaml_body > "$QOS_FILE"
  echo "      已写 QoS 覆盖: $QOS_FILE"
fi

# ---- 启动 cmd_vel 实时监视(自动打标记) -------------------------------------
WATCH_PID=""
if [ "$USE_WATCH" = 1 ]; then
  cat > "$OUT_DIR/watch_cmdvel.py" <<'PYEOF'
# 监视 /cmd_vel_chassis: 超过阈值就打印一行带时间戳的记录(供 marks.txt 使用)
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

THRESH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6
TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/cmd_vel_chassis"


class Watcher(Node):
    def __init__(self) -> None:
        super().__init__("lurch_watcher")
        self.create_subscription(Twist, TOPIC, self.cb, 10)
        self.last = 0.0
        print(f"# watch {TOPIC} thresh={THRESH}", flush=True)

    def cb(self, msg: Twist) -> None:
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        mag = (vx * vx + vy * vy) ** 0.5
        now = time.time()
        if mag > THRESH and now - self.last > 1.0:
            self.last = now
            print(
                f"AUTO {now:.3f} {TOPIC} vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} |v|={mag:.3f}",
                flush=True,
            )


rclpy.init()
try:
    rclpy.spin(Watcher())
except (KeyboardInterrupt, ExternalShutdownException):
    pass          # 被主脚本 kill 时属于正常退出, 不要把 traceback 写进 marks.txt
finally:
    try:
        rclpy.try_shutdown()
    except Exception:
        pass
PYEOF
  # 监视进程直接 append 到 marks.txt; 主循环负责把新增的 AUTO 行回显到终端
  python3 -u "$OUT_DIR/watch_cmdvel.py" "$THRESH" /cmd_vel_chassis \
    >> "$OUT_DIR/marks.txt" 2>> "$OUT_DIR/watcher.log" &
  WATCH_PID=$!
  MARK_LINES=0
  echo "      已启动 /cmd_vel_chassis 监视 (pid $WATCH_PID), 超阈值的时刻会自动写进 marks.txt"
fi

echo
echo "[5/6] 开始录制..."
REC_PID=""
REC_LOG="$OUT_DIR/record.log"
start_recorder() {
  "${REC_CMD[@]}" > "$REC_LOG" 2>&1 &
  REC_PID=$!
}

start_recorder
sleep 3
if ! kill -0 "$REC_PID" 2>/dev/null; then
  # 常见原因: 该版本 rosbag2 不支持 --qos-profile-overrides-path
  if [ "$USE_QOS" = 1 ]; then
    echo "[警告] 录制进程立刻退出, 尝试不用 QoS 覆盖再起一次 (详见 $REC_LOG)" >&2
    tail -n 5 "$REC_LOG" >&2 || true
    rm -rf "$BAG_DIR"          # 清掉第一次尝试留下的空目录, 免得第二次起不来
    REC_CMD=(ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}")
    start_recorder
    sleep 3
  fi
  if ! kill -0 "$REC_PID" 2>/dev/null; then
    echo "[错误] 录制起不来, 看 $REC_LOG" >&2
    tail -n 20 "$REC_LOG" >&2 || true
    exit 1
  fi
fi
echo "      rosbag2 已开始 (pid $REC_PID)"
echo
echo "---------------------------------------------------------"
echo " 现在去复现: 先跑几个正常点, 再到出问题的那个点附近点目标。"
echo "   回车        -> 打一条标记(建议在车窜的瞬间按)"
echo "   文字+回车   -> 打带说明的标记(例如: 窜了/撞了/定位跳了)"
echo "   q 回车      -> 停止录制并收拢数据   (Ctrl+C 一样可以)"
echo "---------------------------------------------------------"

FINISHED=0
finish() {
  [ "$FINISHED" = 1 ] && return 0
  FINISHED=1

  echo
  echo "停止录制中(等 rosbag2 落盘...)"
  if [ -n "$REC_PID" ] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -INT "$REC_PID" 2>/dev/null || true
    for _ in $(seq 1 40); do
      kill -0 "$REC_PID" 2>/dev/null || break
      sleep 0.5
    done
    kill -0 "$REC_PID" 2>/dev/null && kill -TERM "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi
  if [ -n "${WATCH_PID:-}" ]; then
    kill "$WATCH_PID" 2>/dev/null || true
    wait "$WATCH_PID" 2>/dev/null || true
  fi

  STOP_EPOCH="$(date +%s)"
  printf '\nMARK %s 脚本停止录制\n' "$(date +%s.%N)" >> "$OUT_DIR/marks.txt" 2>/dev/null || true

  echo
  echo "[6/6] 收拢数据..."
  {
    echo "start_epoch = $START_EPOCH"
    echo "start_iso   = $START_ISO"
    echo "stop_epoch  = $STOP_EPOCH"
    echo "stop_iso    = $(date -Ins)"
    echo "hostname    = $(hostname)"
    echo "duration_s  = $((STOP_EPOCH - START_EPOCH))"
    echo "ws_dir      = $WS_DIR"
    echo "out_dir     = $OUT_DIR"
    echo "record_cmd  = ${REC_CMD[*]}"
    echo "nav_args    = $NAV_ARGS   (WITH_NAV=$WITH_NAV)"
    echo
    echo "--- 现场进程(录制开始时) ---"
    printf '%s\n' "$NAV_PROCS"
    echo
    echo "--- ros2 node list ---"
    ros2 node list 2>&1 || true
    echo
    echo "--- ros2 topic list -t (含隐藏话题) ---"
    ros2 topic list -t --include-hidden-topics 2>&1 || true
  } > "$OUT_DIR/meta.txt" 2>&1

  # 当前加载的地图(确认跑的是哪张图、origin/resolution 对不对)
  {
    echo "--- map_server yaml_filename ---"
    ros2 param get /map_server yaml_filename 2>&1 || true
    echo
    echo "--- /map info ---"
    timeout 10 ros2 topic echo /map --field info --once 2>&1 || true
  } > "$OUT_DIR/map_info.txt" 2>&1

  # nav2 运行时参数
  for n in behavior_server controller_server planner_server velocity_smoother \
           bt_navigator fake_vel_transform local_costmap global_costmap map_server; do
    err="$OUT_DIR/params/${n}.dump.err"
    ros2 param dump "/$n" > "$OUT_DIR/params/${n}.yaml" 2>"$err" || true
    if [ -s "$err" ]; then
      { echo "param dump 失败: $n"; cat "$err"; } >> "$OUT_DIR/params/_errors.txt"
    fi
    rm -f "$err"
    [ -s "$OUT_DIR/params/${n}.yaml" ] || rm -f "$OUT_DIR/params/${n}.yaml"   # 空文件没意义
  done

  # 本次运行新产生的 ROS 日志
  if [ -d "$HOME/.ros/log" ]; then
    find "$HOME/.ros/log" -maxdepth 1 -type f -name '*.log' \
      -newermt "@$((START_EPOCH - 60))" -exec cp -t "$OUT_DIR/logs" {} + 2>/dev/null || true
    while IFS= read -r d; do
      [ -f "$d/launch.log" ] || continue
      cp "$d/launch.log" "$OUT_DIR/logs/$(basename "$d")__launch.log" 2>/dev/null || true
    done < <(find "$HOME/.ros/log" -maxdepth 1 -mindepth 1 -type d \
      -newermt "@$((START_EPOCH - 60))" 2>/dev/null || true)
  fi

  # bag 信息与体积
  if [ -d "$BAG_DIR" ]; then
    ros2 bag info "$BAG_DIR" > "$OUT_DIR/bag_info.txt" 2>&1 || true
  else
    echo "没有找到 bag 目录: $BAG_DIR" > "$OUT_DIR/bag_info.txt"
  fi
  du -sh "$BAG_DIR" > "$OUT_DIR/size.txt" 2>/dev/null || true
  du -sh "$OUT_DIR" >> "$OUT_DIR/size.txt" 2>/dev/null || true

  echo
  echo "========================================================="
  echo " 抓取完成: $OUT_DIR"
  [ -f "$OUT_DIR/size.txt" ] && sed 's/^/   /' "$OUT_DIR/size.txt"
  if [ -f "$OUT_DIR/bag_info.txt" ]; then
    n_topics="$(grep -c 'Topic:' "$OUT_DIR/bag_info.txt" 2>/dev/null || echo 0)"
    if [ "${n_topics:-0}" -lt 5 ]; then
      echo " [警告] bag 里只有 ${n_topics} 个话题, 这份抓取基本没用, 建议重录!"
    else
      echo " bag 话题数: $n_topics"
    fi
  fi
  echo " 标记文件 : $OUT_DIR/marks.txt"
  grep -c '^' "$OUT_DIR/marks.txt" 2>/dev/null | sed 's/^/   标记行数: /' || true
  grep '^AUTO' "$OUT_DIR/marks.txt" 2>/dev/null | sed 's/^/   自动标记: /' || true
  echo
  echo " 发给分析方: 目录路径 $OUT_DIR (或整个目录/压缩包)"
  echo " 想自己先看一眼:"
  echo "   grep -n AUTO $OUT_DIR/marks.txt"
  echo "   grep -n 'backing up\\|Collision detected\\|Failed to make progress\\|obstacle' $OUT_DIR/logs/*.log"
  echo "========================================================="
}

# 任何退出路径(正常结束/中断/管道被关/终端关掉)都走 finish 收尾, 避免录包器变成野进程
trap 'finish' EXIT
trap 'echo; echo "[中断] 收到信号, 收尾..."; exit 130' INT TERM HUP PIPE

# ---- 主循环: 等时长/体积上限/用户按键 --------------------------------------
MARK_LINES=0        # 已回显过的 marks.txt 行数
HEARTBEAT=0
while true; do
  line=""
  if read -r -t 1 line; then
    # 读到了: 用户按了回车(空行)或输入了文字
    case "$line" in
      q | Q | quit | QUIT | stop | s) echo "[用户] 停止录制"; break ;;
      *)
        printf 'MANUAL %s %s\n' "$(date +%s.%N)" "${line:-(无说明)}" >> "$OUT_DIR/marks.txt"
        echo "  [标记] ${line:-(无说明)}"
        ;;
    esac
  else
    # 超时(交互终端) 或 stdin 已 EOF(管道): 后者补个 sleep, 避免忙等
    [ -t 0 ] || sleep 1
  fi

  # 把监视进程新写进来的 AUTO 行回显出来
  if [ -f "$OUT_DIR/marks.txt" ]; then
    total="$(grep -c '^' "$OUT_DIR/marks.txt" 2>/dev/null || echo 0)"
    if [ "${total:-0}" -gt "$MARK_LINES" ]; then
      sed -n "$((MARK_LINES + 1)),\$p" "$OUT_DIR/marks.txt" | grep '^AUTO' | sed 's/^/  [!] /' || true
      MARK_LINES="$total"
    fi
  fi

  # 录制进程死了就退出
  if ! kill -0 "$REC_PID" 2>/dev/null; then
    echo "[警告] rosbag2 已退出(见 $REC_LOG)"
    break
  fi

  now="$(date +%s)"
  elapsed=$((now - START_EPOCH))
  if [ "$elapsed" -ge "$DURATION" ]; then
    echo "[到达最长录制时长 ${DURATION}s, 自动停止]"
    break
  fi
  size_mb="$(du -sm "$BAG_DIR" 2>/dev/null | awk '{print $1}' || echo 0)"
  if [ "${size_mb:-0}" -ge "$MAX_SIZE_MB" ]; then
    echo "[bag 已达 ${size_mb}MB (上限 ${MAX_SIZE_MB}MB), 自动停止]"
    break
  fi
  if [ "$elapsed" -ge "$HEARTBEAT" ]; then
    HEARTBEAT=$((elapsed + 60))
    printf '  ... 已录 %ss, bag %sMB, 标记 %s 条\n' "$elapsed" "${size_mb:-0}" \
      "$(grep -c '^' "$OUT_DIR/marks.txt" 2>/dev/null || echo 0)"
  fi
done

finish
