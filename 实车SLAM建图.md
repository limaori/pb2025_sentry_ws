# SRM 实车 MID360 建图

本入口使用当前工作空间的 **MID360 → Point-LIO → slam_toolbox**，生成有二维回环修正的 `/map`。适合先在平整场地遥控建图；底盘遥控由你原来的遥控器/下位机负责。入口仅启动建图节点，不启动串口、导航控制器、决策或 Gazebo。

## 1. 已接入的机器人资料

原工程根目录默认是 `/home/srm/srm_auto_sentry`，以下相对路径均在 `src/pb_rmsimulation/src/` 下：

| 项目 | 原文件 / 采用值 |
|---|---|
| 实际使用的 URDF 模板 | `rm_nav_bringup/urdf/sentry_robot_cylinder.xacro`，与旧 `bringup_real.launch.py` 一致 |
| 雷达模型 | `rm_simulation/pb_rm_simulation/meshes/mid360.stl` |
| 安装平移 | `measurement_params_real.yaml` 的 `0.15 -0.15 0.22` 米 |
| 安装旋转 | 旧 `config/reality/MID360_config.json` 的 roll=-4°、pitch=0°、yaw=-90° |
| 主机有线 IP | `192.168.1.50/24` |
| 雷达 IP | `192.168.1.184` |
| LiDAR → IMU 外参 | `extrinsic_T=[-0.011,-0.02329,0.04412]` 米，`extrinsic_R=I` |

旧实车 launch 使用的是 cylinder 模板，不能只看 `sentry_robot_real.xacro` 的默认高度 0.49 m。新入口运行时直接读取旧模板与 mesh，因此只需要 source 当前工作空间，无需编译或 source 旧工作空间。旧工程文件需保留；换电脑时同时复制该目录，或通过 `source_workspace:=/新路径` 指定。

安装角按旧驱动补偿值作为物理安装角迁移，仍须在实车上验证方向和地面水平。新入口把这组角放进 URDF 的 `base_link → livox_frame`，驱动 JSON 外参保持全零，避免对点云/IMU 再旋转一次。驱动不会替你发布机器人安装 TF。

Point-LIO 输出的是 IMU 位姿，入口补充 `livox_imu` 坐标系并据此计算底盘位姿；`extrinsic_T` 不能直接拿来当雷达相对车体的位置。重力向量根据安装角自动设置，使 Point-LIO 世界系与后续安装变换一致。

## 2. 编译

已有当前工作空间的编译结果时，执行：

```bash
cd /home/srm/pb2025_sentry_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install --packages-select pb2025_nav_bringup
```

首次在新电脑部署时，先按项目 README 安装依赖，并编译建图使用的包：

```bash
colcon build --symlink-install --packages-select \
  livox_ros_driver2 point_lio loam_interface sensor_scan_generation \
  pointcloud_to_laserscan pb2025_nav_bringup
```

系统还需具备 `slam_toolbox`、`nav2_map_server`、`robot_state_publisher`、`xacro`、`rviz2`。不要在同一个终端叠加两个工程的 `install/setup.bash`，两个工程含有同名但不同版本的驱动和 LIO 包。

## 3. 接线与网络

雷达上电并接到工控机有线网口。检查：

```bash
ip -br address
```

在本机检查时，有线接口为 `enp114s0`，状态 DOWN；Wi-Fi 为 `192.168.1.61`，并没有配置雷达接收地址 `192.168.1.50`。连好网线后给有线口设置静态地址。若已有 `mid360` 连接配置，直接启用；否则创建：

```bash
sudo nmcli con add type ethernet ifname enp114s0 con-name mid360 \
  ipv4.method manual ipv4.addresses 192.168.1.50/24 \
  ipv4.never-default yes ipv6.method disabled
sudo nmcli con up mid360
ping -I enp114s0 -c 3 192.168.1.184
ip route get 192.168.1.184
```

路由应走有线口。Wi-Fi 也在 `192.168.1.0/24` 时需要检查路由选择。雷达实际 IP 不同时，修改 `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/mid360_user_config.json` 的 `lidar_configs[].ip`；主机地址变化时同步修改四个非空 `host_net_info.*_ip`。

## 4. 开始建图

退出原来的仿真/建图启动程序和单独启动的 Livox 驱动，再运行：

```bash
cd /home/srm/pb2025_sentry_ws
bash script/start_real_slam.sh
```

等价命令：

```bash
source /opt/ros/humble/setup.bash
source /home/srm/pb2025_sentry_ws/install/local_setup.bash
ros2 launch pb2025_nav_bringup srm_slam_launch.py
```

雷达与车体先静止数秒，等待 IMU 初始化以及 RViz 出现地图，再缓慢遥控遍历场地，最后回到起点附近形成回环。先验证静止时墙面不漂、前进时模型沿车头方向移动，再扩大建图范围。

安装发生变化时，使用实际测量值，长度单位米、角度单位弧度：

```bash
bash script/start_real_slam.sh \
  lidar_xyz:="0.15 -0.15 0.22" \
  lidar_rpy:="-0.06981317007977318 0 -1.5707963267948966"
```

无显示器时加 `use_rviz:=False`。原 `script/start_slam.sh` 是 Gazebo 仿真入口，实车用新脚本。

## 5. 验证数据与 TF

新终端 source 当前工作空间后，逐项检查：

```bash
ros2 topic type /livox/lidar
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /registered_scan
ros2 topic hz /scan
ros2 topic info /map
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

`/livox/lidar` 必须是 `livox_ros_driver2/msg/CustomMsg`，约 10 Hz；IMU 约 200 Hz。驱动也发布供观察的 `/livox/lidar/pointcloud`，不能用它替换本配置的 LIO 输入。

TF 主链为 `map → odom → base_link → livox_frame → livox_imu`，`base_link → livox_scan` 是投影二维扫描使用的水平坐标系。`map → odom` 由 slam_toolbox 发布，`odom → base_link` 由 sensor_scan_generation 发布，固定关节由 robot_state_publisher 发布。不要另开静态 `map → odom` 或发布同名底盘 TF 的旧程序。

`srm_slam.yaml` 配置算法和话题；当前扫描选取距底盘原点高度 0.10～1.00 m 的点，并保留原始反射强度 0～255。高度在 launch 的点云转换节点中设置。这是平地建图的起始配置；坡道、多层场景需要重新评估地面分割和高度筛选。现有 sensor_scan_generation 会把底盘 TF 压到平面，不能当成完整六自由度定位。

## 6. 保存与回放

**保持建图程序运行**，新终端执行，地图名请每次换一个以保留历史结果：

```bash
source /opt/ros/humble/setup.bash
source /home/srm/pb2025_sentry_ws/install/local_setup.bash
mkdir -p /home/srm/pb2025_sentry_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/srm/pb2025_sentry_ws/maps/srm_site_01 \
  --ros-args -p save_map_timeout:=10.0
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/srm/pb2025_sentry_ws/maps/srm_site_01'}"
```

前者保存导航用 `.pgm + .yaml`；后者保存 `.posegraph + .data`，便于继续建图/使用 slam_toolbox 定位。本入口使用根命名空间，保存时不要加仿真的 `/red_standard_robot1`。

若还要三维点云，启动时加 `save_pcd:=True`。结束时 Ctrl+C 正常退出，Point-LIO 将点云写入编译时源码目录的 `src/pb2025_sentry_nav/point_lio/PCD/scans.pcd`。它会在内存累计点云并覆盖同名文件，保存后及时另存。**该 PCD 属于 Point-LIO 的 `camera_init` 系，未经过二维回环优化，不能假设与保存的栅格地图天然对齐并直接用于 small_gicp 重定位。** 后续导航接入还需统一车体 frame、定位方式、地图坐标和底盘速度接口。

建议首次实测同步录制原始数据，新终端执行：

```bash
ros2 bag record -o /home/srm/pb2025_sentry_ws/maps/srm_raw_01 \
  /livox/lidar /livox/imu
```

这份由新入口录制的数据可以在无雷达时重建：

```bash
bash script/start_real_slam.sh start_lidar:=False use_sim_time:=True
```

另一终端 source 当前工作空间并执行：

```bash
ros2 bag play /home/srm/pb2025_sentry_ws/maps/srm_raw_01 --clock
```

旧驱动若已旋转过点云/IMU，旧 bag 不能直接假定是同一组原始坐标，需要先核对录制时的外参与消息格式。

## 7. 当前验证范围

已编译 `pb2025_nav_bringup`，以独立 ROS domain 启动无硬件链路，使用合成 MID360 CustomMsg 和 IMU 验证了 Point-LIO、底盘 TF、二维扫描、slam_toolbox 及地图/位姿图保存。生成测试地图为 121×121、分辨率 0.05 m，静止底盘 XY 偏差小于 1 mm，雷达 TF 高度为 0.22 m。RViz 已启动且未报 mesh 加载错误。测试进程已退出。

本机 RViz 显示地图时出现 `active samplers with a different type refer to the same texture image unit` 的 OpenGL 着色器报错，软件渲染也复现。`/map` 生成与保存已独立验证成功，但 RViz 地图显示仍需检查本机图形环境；可先加 `use_rviz:=False` 运行建图和保存，之后在图形环境正常的电脑查看。

这些是离线链路检查，不代表实车标定、实际雷达时间戳、动态运动精度或场地回环已验收。雷达当前未接入，需要按第 3～5 节完成实车验证。
