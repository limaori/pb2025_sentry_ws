#!/usr/bin/env bash
# =============================================================
# 一键启动实车导航: Livox 雷达驱动 + Nav2 导航栈 (+ RViz / 手柄)
#
# 用法:
#   ./script/start_real_nav.sh                            # 默认地图 srm_site_01, 不启 SLAM, 不启重定位
#   ./script/start_real_nav.sh --reloc                    # 先验 PCD 重定位 (small_gicp)
#   ./script/start_real_nav.sh --slam                     # SLAM 建图 (slam_toolbox)
#   ./script/start_real_nav.sh -m srm_site_01 --reloc     # 显式指定地图名
#   ./script/start_real_nav.sh --map /abs/m.yaml --prior-pcd /abs/m.pcd --reloc
#   ./script/start_real_nav.sh --list-maps                # 列出可选地图
#   ./script/start_real_nav.sh --check                    # 只解析并打印将要执行的命令
#
# 三种定位方式(互斥), 区别只在 map->odom 由谁发布:
#   --slam    : slam_toolbox 建图, 由 SLAM 发布 map->odom, 不需要先验地图
#   --reloc   : small_gicp 与先验 PCD 做 GICP 配准, 发布 map->odom;
#               需要 --prior-pcd, odom->base 由 point_lio 提供
#   (默认)    : map_server 加载先验栅格图 + 静态 map->odom, 需要外部里程计
#
# 常用可选项:
#   -m, --map NAME|PATH        地图名或路径 (默认 srm_site_01)
#       --prior-pcd PATH       重定位先验点云, 默认 pcd/reality/<map>.pcd
#       --slam / --reloc       见上, 两者互斥
#       --map-to-odom X Y YAW  静态定位模式下 map->odom 初始位姿 (默认 0 0 0)
#       --params-file PATH     nav2 参数文件 (默认 config/reality/srm_nav2_params.yaml)
#       --lidar-xyz "X Y Z"    雷达在 base_link 下的安装位置 (默认 0.15 -0.15 0.22)
#       --lidar-rpy "R P Y"    雷达安装角, 弧度 (默认 -0.06981317007977318 0 -1.5707963267948966)
#       --namespace NS         命名空间 (默认空 = 根命名空间)
#       --no-robot-state-pub   不启动 robot_state_publisher
#                              (默认启动: 默认参数文件要用 base_link / livox_frame /
#                               livox_imu / livox_scan 这些 frame, 由 SRM 模型提供;
#                               若车体模块已自行发布 URDF/TF, 请加此开关避免重复)
#       --no-rviz              不启动 RViz (默认启动)
#       --joy                  启动手柄遥控 (默认关闭)
#       --no-composition       不使用组合节点 (use_composition:=False)
#   -n, --check                只解析并打印将要执行的命令, 不真正启动
#   -h, --help                 显示帮助
#
# 环境变量:
#   OPEN_MODE=tab|window       终端标签页 / 独立窗口 (默认 tab)
#   TERMINAL=gnome-terminal    终端程序
#   PB2025_WS_DIR=<ws>         覆盖工作空间路径
#
# 说明:
#   - 导航栈通过 bringup_launch.py 组装, 因此重定位开关真正可用
#     (rm_navigation_reality_launch.py 未透传 use_pcd_localization, 传了也无效)。
#   - 车体模型与参数默认用 SRM 版本:
#       模型: srm_robot_state_publisher_launch.py  (base_link / livox_frame / livox_imu / livox_scan)
#       参数: config/reality/srm_nav2_params.yaml  (pb2025 导航结构 + SRM 实车参数)
#     想回到上游 pb2025 那套(base_footprint / gimbal_yaw / front_mid360)时,
#     加 --params-file .../config/reality/nav2_params.yaml 并换成上游模型即可;
#     两套 frame 不能混用, 参数文件与模型必须配套。
#   - 每个命令都会先 source 工作空间的 install/setup.bash。
# =============================================================

set -euo pipefail

# 工作目录 = 脚本所在目录的上一级（即 ROS2 工作空间根目录）
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${PB2025_WS_DIR:-$WS_DIR}"

# 包源码目录 / 安装后的 share 目录 (本工作空间是 symlink 安装, 两者内容等价)
SRC_SHARE_DIR="$WS_DIR/src/pb2025_sentry_nav/pb2025_nav_bringup"
INSTALL_SHARE_DIR="$WS_DIR/install/pb2025_nav_bringup/share/pb2025_nav_bringup"

# 窗口 / 标签页 模式: "tab" 同一窗口多个标签页(默认), "window" 每个命令一个独立窗口
OPEN_MODE="${OPEN_MODE:-tab}"
# 终端程序（可改成 konsole / xfce4-terminal 等）
TERMINAL="${TERMINAL:-gnome-terminal}"

# ---- 默认参数（允许用环境变量覆盖）----------------------------------------
MAP_NAME="${MAP_NAME:-${MAP:-srm_site_01}}"
PRIOR_PCD_FILE="${PRIOR_PCD_FILE:-}"
PARAMS_FILE="${PARAMS_FILE:-}"
NAMESPACE="${NAMESPACE:-}"
START_SLAM="${START_SLAM:-0}"
START_RELOC="${START_RELOC:-0}"
USE_RVIZ="${USE_RVIZ:-1}"
USE_ROBOT_STATE_PUB="${USE_ROBOT_STATE_PUB:-1}"
USE_JOY="${USE_JOY:-0}"
USE_COMPOSITION="${USE_COMPOSITION:-1}"
MAP_TO_ODOM_X="${MAP_TO_ODOM_X:-0.0}"
MAP_TO_ODOM_Y="${MAP_TO_ODOM_Y:-0.0}"
MAP_TO_ODOM_YAW="${MAP_TO_ODOM_YAW:-0.0}"
DRY_RUN="${DRY_RUN:-0}"
# SRM 实车雷达安装位姿（与 srm_slam_launch.py 的默认值一致）
LIDAR_XYZ="${LIDAR_XYZ:-0.15 -0.15 0.22}"
LIDAR_RPY="${LIDAR_RPY:--0.06981317007977318 0.0 -1.5707963267948966}"

# ---- 路径解析 -------------------------------------------------------------
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到工作空间的 install/setup.bash, 请确认路径: $WS_DIR" >&2
  exit 1
fi

# 优先用安装后的 share 目录（launch 运行时 get_package_share_directory 返回的也是它），
# 找不到再退回源码目录。
pick_dir() {
  local d
  for d in "$@"; do
    if [ -d "$d" ]; then
      printf '%s' "$d"
      return 0
    fi
  done
  printf '%s' "$1"
}

PACKAGE_DIR="$(pick_dir "$INSTALL_SHARE_DIR" "$SRC_SHARE_DIR")"
MAP_DIR="$PACKAGE_DIR/map/reality"
PCD_DIR="$PACKAGE_DIR/pcd/reality"
PACKAGE_SRC_DIR="$SRC_SHARE_DIR"

if [ -z "$PARAMS_FILE" ]; then
  PARAMS_FILE="$PACKAGE_DIR/config/reality/srm_nav2_params.yaml"
fi

RVIZ_CONFIG="$PACKAGE_DIR/rviz/nav2_default_view.rviz"

# 输入校验用的布尔判断
is_true() {
  case "${1,,}" in
    1 | true | yes | on) return 0 ;;
    *) return 1 ;;
  esac
}

# shell 安全引用
q() { printf '%q' "$1"; }

usage() {
  # 打印文件开头的连续注释块（跳过 shebang）
  awk 'NR > 1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

# 列出可选地图: pcd/reality 与工作空间 maps/ 下的 *.yaml
list_maps() {
  echo "可选地图（名称去掉 .yaml 后作为 --map 的值）:"
  echo
  local f name pcd_ok seen="" found=0
  for f in "$SRC_SHARE_DIR"/map/reality/*.yaml "$WS_DIR"/maps/*.yaml; do
    [ -f "$f" ] || continue
    name="$(basename "$f" .yaml)"
    # 同名地图可能同时存在于两处, 只列一次
    case " $seen " in *" $name "*) continue ;; esac
    seen="$seen $name"
    pcd_ok="无"
    if [ -f "$SRC_SHARE_DIR/pcd/reality/$name.pcd" ] || [ -f "$WS_DIR/maps/$name.pcd" ] \
      || [ -f "$WS_DIR/maps/${name}_mapframe.pcd" ]; then
      pcd_ok="有"
    fi
    printf '  %-24s 先验PCD: %s\n' "$name" "$pcd_ok"
    printf '  %-24s   栅格图: %s\n' "" "$f"
    found=1
  done
  if [ "$found" = "0" ]; then
    echo "  (未找到任何 .yaml 地图)"
  fi
  echo
  echo "地图目录  : $SRC_SHARE_DIR/map/reality"
  echo "先验PCD目录: $SRC_SHARE_DIR/pcd/reality"
  echo "注意: --reloc 需要先验 PCD, 且该 PCD 必须已经在地图坐标系下。"
}

# 解析地图 yaml: 支持直接给路径, 或给名称后按几个约定目录查找
resolve_map_yaml() {
  local name="$1" cand
  for cand in "$name" "$WS_DIR/$name"; do
    if [ -f "$cand" ]; then
      readlink -f "$cand"
      return 0
    fi
  done
  for cand in \
    "$MAP_DIR/$name.yaml" \
    "$SRC_SHARE_DIR/map/reality/$name.yaml" \
    "$WS_DIR/maps/$name.yaml" \
    "$PACKAGE_DIR/map/simulation/$name.yaml"; do
    if [ -f "$cand" ]; then
      readlink -f "$cand"
      return 0
    fi
  done
  return 1
}

# 解析先验 PCD: 注意 _mapframe 版本优先于 Point-LIO camera_init 系的原始点云,
# 因为 GICP 重定位要求先验点云与先验栅格图在同一坐标系。
resolve_prior_pcd() {
  local name="$1" cand
  for cand in "$name" "$WS_DIR/$name"; do
    if [ -f "$cand" ]; then
      readlink -f "$cand"
      return 0
    fi
  done
  for cand in \
    "$PCD_DIR/$name.pcd" \
    "$SRC_SHARE_DIR/pcd/reality/$name.pcd" \
    "$WS_DIR/maps/${name}_mapframe.pcd" \
    "$WS_DIR/maps/$name.pcd"; do
    if [ -f "$cand" ]; then
      readlink -f "$cand"
      return 0
    fi
  done
  return 1
}

# ---- 参数解析 -------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    -m | --map) MAP_NAME="${2:?--map 需要参数}"; shift 2 ;;
    --prior-pcd) PRIOR_PCD_FILE="${2:?--prior-pcd 需要参数}"; shift 2 ;;
    --params-file) PARAMS_FILE="${2:?--params-file 需要参数}"; shift 2 ;;
    --lidar-xyz) LIDAR_XYZ="${2:?--lidar-xyz 需要 "X Y Z"}"; shift 2 ;;
    --lidar-rpy) LIDAR_RPY="${2:?--lidar-rpy 需要 "R P Y"}"; shift 2 ;;
    --namespace) NAMESPACE="${2:?--namespace 需要参数}"; shift 2 ;;
    --map-to-odom)
      MAP_TO_ODOM_X="${2:?--map-to-odom 需要 X Y YAW}"
      MAP_TO_ODOM_Y="${3:?--map-to-odom 需要 X Y YAW}"
      MAP_TO_ODOM_YAW="${4:?--map-to-odom 需要 X Y YAW}"
      shift 4
      ;;
    --slam) START_SLAM=1; shift ;;
    --no-slam) START_SLAM=0; shift ;;
    --reloc | --relocalization) START_RELOC=1; shift ;;
    --no-reloc) START_RELOC=0; shift ;;
    --rviz) USE_RVIZ=1; shift ;;
    --no-rviz) USE_RVIZ=0; shift ;;
    --robot-state-pub) USE_ROBOT_STATE_PUB=1; shift ;;
    --no-robot-state-pub) USE_ROBOT_STATE_PUB=0; shift ;;
    --joy) USE_JOY=1; shift ;;
    --no-joy) USE_JOY=0; shift ;;
    --composition) USE_COMPOSITION=1; shift ;;
    --no-composition) USE_COMPOSITION=0; shift ;;
    -n | --check) DRY_RUN=1; shift ;;
    --list-maps) list_maps; exit 0 ;;
    -h | --help) usage; exit 0 ;;
    *)
      echo "[错误] 未知参数: $1" >&2
      echo "        用 --help 查看用法。" >&2
      exit 1
      ;;
  esac
done

# ---- 参数校验 -------------------------------------------------------------
if is_true "$START_SLAM" && is_true "$START_RELOC"; then
  echo "[错误] SLAM 与重定位互斥: SLAM 由 slam_toolbox 发布 map->odom," >&2
  echo "[错误] 重定位由 small_gicp 发布 map->odom, 同时开启会导致 TF 冲突。" >&2
  exit 1
fi

SLAM_ARG=$([ "$START_SLAM" = "1" ] && echo True || echo False)
RELOC_ARG=$([ "$START_RELOC" = "1" ] && echo True || echo False)
COMPOSITION_ARG=$([ "$USE_COMPOSITION" = "1" ] && echo True || echo False)

if [ ! -f "$PARAMS_FILE" ]; then
  echo "[错误] nav2 参数文件不存在: $PARAMS_FILE" >&2
  echo "[错误] 用 --params-file 指定, 或确认工作空间已编译。" >&2
  exit 1
fi

# 参数文件与车体模型必须配套: 模型提供什么 frame, 参数文件就用什么 frame。
# 默认的 SRM 模型发 base_link/livox_*; 上游 nav2_params.yaml 要的是
# base_footprint/gimbal_yaw/front_mid360, 混用会满屏 TF 报错且重定位卡死。
if [ "$USE_ROBOT_STATE_PUB" = "1" ] && ! grep -qE "livox_imu|base_link" "$PARAMS_FILE"; then
  cat >&2 <<EOF
[错误] 参数文件与车体模型不配套:
[错误]   参数文件 $PARAMS_FILE
[错误]   里面没有 base_link / livox_imu, 看起来是上游 pb2025 那份
[错误]   (要用 base_footprint / gimbal_yaw / front_mid360), 而本脚本启动的
[错误]   SRM 模型只发 base_link / livox_frame / livox_imu / livox_scan。
[错误] 请二选一:
[错误]   1) 用 SRM 参数: 不加 --params-file (默认 config/reality/srm_nav2_params.yaml)
[错误]   2) 用上游模型: 本脚本暂不支持, 请改用 rm_navigation_reality_launch.py
EOF
  exit 1
fi
# 地图: SLAM 模式下 bringup_launch 仍要求该参数存在, 但不加载栅格图
MAP_YAML=""
if MAP_YAML="$(resolve_map_yaml "$MAP_NAME")"; then
  :
else
  if [ "$START_SLAM" = "1" ]; then
    # 建图模式不需要先验栅格图, 给个占位路径即可
    MAP_YAML="$MAP_DIR/$MAP_NAME.yaml"
    echo "[提示] SLAM 模式不加载先验栅格图, 未找到 $MAP_NAME.yaml 属正常。" >&2
  else
    echo "[错误] 未找到地图: $MAP_NAME" >&2
    case "$MAP_NAME" in
      */*) : ;;
      *)
        echo "[错误] 已查找: $MAP_DIR/$MAP_NAME.yaml" >&2
        echo "[错误]           $WS_DIR/maps/$MAP_NAME.yaml" >&2
        ;;
    esac
    echo "[错误] 用 --map <名称|路径> 指定, 或 --list-maps 查看可选地图。" >&2
    echo "[错误] 若确实要边建图边导航, 请改用 --slam。" >&2
    exit 1
  fi
fi

# 先验点云: 仅重定位需要
PRIOR_PCD=""
if [ "$START_RELOC" = "1" ]; then
  if [ -z "$PRIOR_PCD_FILE" ]; then
    PRIOR_PCD_FILE="$MAP_NAME"
  fi
  if PRIOR_PCD="$(resolve_prior_pcd "$PRIOR_PCD_FILE")"; then
    :
  else
    echo "[错误] 重定位需要先验点云, 但未找到: $PRIOR_PCD_FILE" >&2
    # 只有当入参是"地图名"时才列出按约定目录拼出的候选路径
    case "$PRIOR_PCD_FILE" in
      */*) : ;;
      *)
        echo "[错误] 已查找: $PCD_DIR/$PRIOR_PCD_FILE.pcd" >&2
        echo "[错误]           $WS_DIR/maps/${PRIOR_PCD_FILE}_mapframe.pcd" >&2
        echo "[错误]           $WS_DIR/maps/$PRIOR_PCD_FILE.pcd" >&2
        ;;
    esac
    echo "[错误] 用 --prior-pcd <路径> 指定。" >&2
    echo "[错误] 注意: PCD 必须在地图坐标系下, Point-LIO camera_init 系的原始点云不能直接用。" >&2
    exit 1
  fi
  # 先验 PCD 坐标系自检: mapframe 版本头部有坐标系标注, 原始点云通常没有
  if ! head -c 1024 "$PRIOR_PCD" | grep -aqiE "map frame|map_frame|地图坐标系"; then
    echo "[警告] $PRIOR_PCD" >&2
    echo "[警告] 头部未标注坐标系; 若它是 Point-LIO camera_init 系的原始点云," >&2
    echo "[警告] GICP 重定位初值会错。请先转到与栅格图一致的地图坐标系。" >&2
  fi
else
  # 非重定位模式: 该参数仅供 launch 声明, 传占位路径即可
  PRIOR_PCD="${PRIOR_PCD_FILE:-$PCD_DIR/$MAP_NAME.pcd}"
fi

# ---- 组装各条启动命令 -----------------------------------------------------
NS_ARG=""
[ -n "$NAMESPACE" ] && NS_ARG="namespace:=$NAMESPACE"

# 1) Livox 雷达驱动: bringup_launch 不负责起驱动, 必须单独启动。
#    参数取自 nav2_params.yaml 的 livox_ros_driver2 段 (含 user_config_path)。
DRIVER_CMD="ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args --params-file $(q "$PARAMS_FILE")"
if [ -n "$NAMESPACE" ]; then
  DRIVER_CMD="$DRIVER_CMD -r __ns:=$(q "$NAMESPACE")"
fi

# 2) robot_state_publisher: 默认启动，用 SRM 模型（srm_robot_state_publisher_launch.py，
#    与 srm_slam_launch.py 同一套构建逻辑）。默认参数文件里的 base_link / livox_frame /
#    livox_imu / livox_scan 全部由它提供; 缺了这些 frame，重定位节点会在构造时一直等 TF
#    而卡住整个 launch。车体模块若已自己发 URDF/TF 请用 --no-robot-state-pub 关闭，
#    否则会出现两个重复的 TF 发布者。
#    注意: extrinsic_T 与 lidar_xyz/lidar_rpy 必须来自同一份标定 —— 模型里 livox_imu 的
#    偏移取自参数文件的 point_lio.mapping.extrinsic_T，安装位姿取自这里。
RSP_CMD="ros2 launch pb2025_nav_bringup srm_robot_state_publisher_launch.py $NS_ARG use_sim_time:=False"
RSP_CMD="$RSP_CMD params_file:=$(q "$PARAMS_FILE")"
RSP_CMD="$RSP_CMD lidar_xyz:=$(q "$LIDAR_XYZ")"
RSP_CMD="$RSP_CMD lidar_rpy:=$(q "$LIDAR_RPY")"

# 3) 导航栈
NAV_CMD="ros2 launch pb2025_nav_bringup bringup_launch.py"
NAV_CMD="$NAV_CMD $NS_ARG"
NAV_CMD="$NAV_CMD slam:=$SLAM_ARG"
NAV_CMD="$NAV_CMD use_pcd_localization:=$RELOC_ARG"
NAV_CMD="$NAV_CMD map:=$(q "$MAP_YAML")"
NAV_CMD="$NAV_CMD prior_pcd_file:=$(q "$PRIOR_PCD")"
NAV_CMD="$NAV_CMD params_file:=$(q "$PARAMS_FILE")"
NAV_CMD="$NAV_CMD use_sim_time:=False"
NAV_CMD="$NAV_CMD autostart:=true"
NAV_CMD="$NAV_CMD use_composition:=$COMPOSITION_ARG"
NAV_CMD="$NAV_CMD use_respawn:=false"
NAV_CMD="$NAV_CMD map_to_odom_x:=$MAP_TO_ODOM_X"
NAV_CMD="$NAV_CMD map_to_odom_y:=$MAP_TO_ODOM_Y"
NAV_CMD="$NAV_CMD map_to_odom_yaw:=$MAP_TO_ODOM_YAW"

# 4) RViz
RVIZ_CMD="ros2 launch pb2025_nav_bringup rviz_launch.py $NS_ARG use_sim_time:=False rviz_config:=$(q "$RVIZ_CONFIG")"

# 5) 手柄遥控
JOY_CMD="ros2 launch pb2025_nav_bringup joy_teleop_launch.py $NS_ARG use_sim_time:=False joy_config_file:=$(q "$PARAMS_FILE")"

# ---- 打印配置摘要 ---------------------------------------------------------
if [ "$START_SLAM" = "1" ]; then
  MODE_DESC="SLAM 建图 (slam_toolbox 发布 map->odom)"
elif [ "$START_RELOC" = "1" ]; then
  MODE_DESC="先验 PCD 重定位 (small_gicp 发布 map->odom)"
else
  MODE_DESC="静态地图定位 (静态 map->odom)"
fi

echo "==================== 实车导航启动配置 ===================="
echo "  工作空间    : $WS_DIR"
echo "  定位方式    : $MODE_DESC"
if [ "$START_SLAM" = "1" ]; then
  echo "  地图 (yaml) : $MAP_YAML  (SLAM 模式未加载)"
else
  echo "  地图 (yaml) : $MAP_YAML"
fi
if [ "$START_RELOC" = "1" ]; then
  echo "  先验 PCD    : $PRIOR_PCD"
else
  echo "  先验 PCD    : $PRIOR_PCD  (未使用)"
fi
echo "  参数文件    : $PARAMS_FILE"
if [ "$USE_ROBOT_STATE_PUB" = "1" ]; then
  echo "  车体模型    : srm_robot_state_publisher_launch.py (SRM)"
  echo "  雷达安装    : xyz=[$LIDAR_XYZ]  rpy=[$LIDAR_RPY]"
else
  echo "  车体模型    : 不启动 (假定其他模块已发布 URDF/TF)"
fi
echo "  命名空间    : ${NAMESPACE:-<根命名空间>}"
echo "  slam        : $SLAM_ARG    use_pcd_localization: $RELOC_ARG"
echo "  组合节点    : $COMPOSITION_ARG"
if [ "$START_RELOC" != "1" ] && [ "$START_SLAM" != "1" ]; then
  echo "  map->odom   : x=$MAP_TO_ODOM_X y=$MAP_TO_ODOM_Y yaw=$MAP_TO_ODOM_YAW"
fi
echo "========================================================="

# ---- 启动 ---------------------------------------------------------------
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

# 检查命令是否已经在运行。
# 使用带方括号的正则，避免 pgrep 把自身的匹配命令算进去。
process_running() {
  pgrep -u "$(id -u)" -f "$1" >/dev/null 2>&1
}

# 收集启动步骤, 便于统一编号与 dry-run
STEP_TITLES=()
STEP_PATTERNS=()
STEP_CMDS=()

add_step() {
  STEP_TITLES+=("$1")
  STEP_PATTERNS+=("$2")
  STEP_CMDS+=("$3")
}

add_step "Livox 雷达驱动" '[l]ivox_ros_driver2_node' "$DRIVER_CMD"
if [ "$USE_ROBOT_STATE_PUB" = "1" ]; then
  add_step "SRM 车体模型 (robot_state_publisher)" '[s]rm_robot_state_publisher_launch\.py' "$RSP_CMD"
fi
add_step "导航栈 (Nav2)" '[b]ringup_launch\.py' "$NAV_CMD"
if [ "$USE_RVIZ" = "1" ]; then
  add_step "RViz" '[r]viz_launch\.py' "$RVIZ_CMD"
fi
if [ "$USE_JOY" = "1" ]; then
  add_step "手柄遥控" '[j]oy_teleop_launch\.py' "$JOY_CMD"
fi

TOTAL=${#STEP_TITLES[@]}

if [ "$DRY_RUN" = "1" ]; then
  echo
  echo "[--check] 仅解析, 不启动。将要执行的命令:"
  for i in "${!STEP_TITLES[@]}"; do
    printf '\n[%d/%d] %s\n  %s\n' "$((i + 1))" "$TOTAL" "${STEP_TITLES[$i]}" "${STEP_CMDS[$i]}"
  done
  echo
  echo "解析完成，未启动任何进程。"
  exit 0
fi

for i in "${!STEP_TITLES[@]}"; do
  title="${STEP_TITLES[$i]}"
  pattern="${STEP_PATTERNS[$i]}"
  cmd="${STEP_CMDS[$i]}"
  printf '[%d/%d] 启动 %s...\n' "$((i + 1))" "$TOTAL" "$title"

  if process_running "$pattern"; then
    echo "[跳过] 已检测到正在运行的进程: $title"
    continue
  fi

  open_term "$title" "$cmd"
done

echo "启动流程处理完成（终端模式: ${OPEN_MODE}）。"
