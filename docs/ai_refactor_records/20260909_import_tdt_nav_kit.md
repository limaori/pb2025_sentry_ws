# 导入 TDT 导航算法改造记录

## 用户意图

将 `/home/srm/tdt-nav-kit` 中的开源导航算法放入本 ROS 2 工作区。

## 范围

新增独立 ROS 2 包 `tdt_nav_kit`，提供原始前端搜索和可选轨迹优化库。

## 不在范围内

不替换当前 Nav2 的 Theta* 规划器，不修改已有话题（topic）、坐标系（frame）、参数和 launch 文件。

## 探查发现

### 已检查文件

检查了工作区的 Nav2 包、`pb_nav2_plugins`、bringup 参数，以及源项目的 YAstar、KinodynamicAstar、MinimumSnap、SfcSquare 和构建脚本。

### 生效的逻辑路径

当前工作区的全局规划器仍由 `pb2025_nav_bringup` 配置为 Theta*。

### 数据流

新包只提供可链接 C++ 库和头文件，不自动创建 ROS 节点或改变现有导航数据流。

### 风险说明

工作区当前未发现可用的 `OsqpEigen` CMake 包，因此轨迹优化库采用可选构建；YAstar 前端不依赖 OSQP。

### 建议的修改边界

新增独立包和安装规则，保留上游算法源码接口。

## 修改内容

### 变更文件

- `src/pb2025_sentry_nav/tdt_nav_kit/`
- `docs/ai_refactor_records/20260909_import_tdt_nav_kit.md`

### 关键变更

复制上游算法源码和头文件，建立 ROS 2 `ament_cmake` 包，使用 C++20；检测到 `OsqpEigen` 时构建 MinimumSnap，否则只构建前端库。

### 保持的行为

上游类接口和算法实现未改写；现有 Nav2 配置和运行链路不变。

### 有意调整的行为

新增包的构建产物按依赖拆为前端库和可选轨迹库。

## 审查复核

### 已执行检查

- [x] 关键路径检查
- [x] 新增文件和构建规则检查
- [x] 依赖可用性检查
- [x] `colcon build --packages-select tdt_nav_kit`
- [x] 真实 `rmuc_2025_tunnel` 地图运行验证：`(0.12, 6.01)` → `(5.17, 6.01)`，109 个路径点，约 6.31 m
- [x] Gazebo Fortress 仿真启动检查：机器人、`/clock`、地图和 Nav2 生命周期节点均成功启动

### 发现的问题

工作区环境没有 `OsqpEigen`，因此无法在当前环境验证轨迹库编译；YAstar 前端已通过 CMake、`colcon` 和真实仿真地图运行验证。新增 demo 节点只发布路径用于 RViz 检查，不接管 Nav2 控制链路。

### 最终结果

通过（静态接入完成；完整轨迹库构建需先安装 OsqpEigen）。
