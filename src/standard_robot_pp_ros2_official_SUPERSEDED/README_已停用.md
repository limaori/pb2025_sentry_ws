# 已停用（官方版串口节点）

本目录是 pb2025 上游的 `standard_robot_pp_ros2`。**已停用，不参与编译**（同目录的
`COLCON_IGNORE` 让 colcon 跳过它）。

停用原因：它的串口报文协议和 SRM 实车 C 板不一致，两者不可能同时使用：

| 项目 | 本目录（官方版，已停用） | 现在生效的版本 |
|---|---|---|
| 帧起始字节 | 0x5A | **0xA5** |
| 帧头结构 | 起始 + 长度(1字节) + id + CRC8 = 4 字节 | **起始 + 长度(2字节) + seq + CRC8 = 5 字节** |
| 速度报文编号 | 0x01 | **0x0302** |
| 速度报文内容 | vx,vy,wz + 底盘姿态 + 云台 + 射击 + 自瞄 + 时间戳 | **vx,vy,wz + is_recovering** |

现在生效的是 `src/standard_robot_pp_ros2/`（从 `~/srm_auto_sentry` 移植的 SRM 实车版本）。

保留本目录仅为可回溯，需要回到官方协议时：删除本目录的 `COLCON_IGNORE`，并把
`src/standard_robot_pp_ros2/` 移开即可。
