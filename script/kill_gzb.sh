#!/usr/bin/env bash

set -u

WAIT_SECONDS="${WAIT_SECONDS:-5}"

if ! [[ "$WAIT_SECONDS" =~ ^[0-9]+$ ]]; then
  echo "[错误] WAIT_SECONDS 必须是非负整数。" >&2
  exit 2
fi

# 覆盖 Gazebo Classic、Ignition/Gazebo Sim，以及本项目启动仿真的 ros2 launch。
PATTERN='(^|/)(gzserver|gzclient|gazebo)([[:space:]]|$)|[i]gn[[:space:]]+gazebo|[g]z[[:space:]]+sim|[r]os2[[:space:]]+launch[[:space:]]+rmu_gazebo_simulator[[:space:]]+bringup_sim\.launch\.py'

find_gazebo_pids() {
  pgrep -f "$PATTERN" 2>/dev/null || true
}

mapfile -t pids < <(find_gazebo_pids)

if ((${#pids[@]} == 0)); then
  echo "未发现正在运行的 Gazebo 仿真进程。"
  exit 0
fi

echo "发现以下 Gazebo 仿真进程："
ps -o pid=,stat=,cmd= -p "$(IFS=,; echo "${pids[*]}")"

echo "正在请求进程退出（SIGINT）..."
kill -INT "${pids[@]}" 2>/dev/null || true

for ((second = 0; second < WAIT_SECONDS; second++)); do
  mapfile -t remaining < <(find_gazebo_pids)
  ((${#remaining[@]} == 0)) && break
  sleep 1
done

mapfile -t remaining < <(find_gazebo_pids)
if ((${#remaining[@]} > 0)); then
  echo "等待 ${WAIT_SECONDS} 秒后仍有残留，正在强制结束：${remaining[*]}"
  kill -KILL "${remaining[@]}" 2>/dev/null || true
fi

sleep 0.2
mapfile -t remaining < <(find_gazebo_pids)
if ((${#remaining[@]} > 0)); then
  echo "[错误] 以下进程未能结束：${remaining[*]}" >&2
  exit 1
fi

echo "Gazebo 仿真进程已结束。"
