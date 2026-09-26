# 局部规划器 TEB（项目版说明）

本文讲两件事：

1. **TEB 本身是什么、怎么算的**（原理部分，与具体参数无关，长期有效）。
2. **本项目里 TEB 是怎么接进来的、现在配成什么样、哪里会踩坑**（工程部分）。

> 快照说明：文中引用的**参数数值**是 2026-09-26 20:52 从仓库里读出来的（当时实车/仿真配置正在从"非全向"改成"全向"，文件在被实时编辑）。
> 参数**含义与调参方向**是稳定的；具体数值请以文件为准：
> - 实车：`src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/srm_nav2_params.yaml`（`controller_server.FollowPath`）
> - 仿真对照：`src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params_teb.yaml`
> - 上游参考：`src/teb_local_planner/params/teb_params.yaml`
>
> 姊妹文档：`局部规划器相关(ai).md`（讲当前 OmniPidPursuit 控制器与几种方案对比）。本文可以看成它的 TEB 展开版。

---

## 0. TL;DR

- TEB = **Timed Elastic Band**（带时间的弹性带）。它不是"在局部窗口里搜一条新路"，而是**把全局路径当成一根橡皮筋，用图优化把它拉成一条满足底盘运动学、避障、且时间最短的可行轨迹**。
- 优化的变量有两组：**每个中间位姿 `s_i = (x, y, θ)`** 和 **相邻位姿之间的时间间隔 `ΔT_i`**。前者决定"走哪"，后者决定"多快"。
- 约束都是**软约束**（代价项），不是硬约束：速度/加速度/避障/朝向对齐都写成"违反量越界就付代价"，最后交给 g2o 的 Levenberg-Marquardt 求解。
- TEB 相对纯跟踪控制器（本项目原来的 OmniPidPursuit）最大的增量是**同伦类（多拓扑）规划**：它会生成左绕/右绕等若干条本质不同（不等价）的候选轨迹，各自优化一遍，再选代价最小的。这是它能主动绕障的原因。
- 代价：**每个控制周期都要跑一次（甚至多次）非线性优化**，CPU 开销远大于跟踪类控制器。所以本项目把 `controller_frequency` 从 20 Hz 降到了 5 Hz。
- 本项目用的是**社区 ROS2/Nav2 移植版**（`src/teb_local_planner`，v0.9.1 的 Nav2 化版本，含若干 fork 增量），不是官方 ROS1 包；插件已注册可用，配置已切到 TEB。
- 现在实车/仿真都改成了**全向模式**（`max_vel_y > 0` + `weight_kinematics_nh = 0`）：TEB 只做平移、不控底盘朝向。这和麦克纳姆底盘 + 原来的 OmniPidPursuit（`enable_rotation: false`）思路一致。

---

## 1. TEB 是什么

### 1.1 出处与定位

- 作者 Christoph Rösmann（TU Dortmund），原始论文见 §14 参考资料；ROS1 时代是 `teb_local_planner`，接到 `move_base` 的 `base_local_planner` 接口上。
- 在 Nav2 里它的角色是 **Controller（控制器插件）**，插件类 `teb_local_planner::TebLocalPlannerROS`，基类 `nav2_core::Controller`（`src/teb_local_planner/teb_local_planner_plugin.xml`）。
- 也就是说：**全局规划器（本项目是 Theta*）给出粗路径，TEB 负责把这根粗路径变成"能开、不撞、尽量快"的局部轨迹并输出 `cmd_vel`。**

### 1.2 为什么叫"带时间的弹性带"

- **弹性带（Elastic Band）**：把一串路径点看成橡皮筋上的节点。橡皮筋被障碍物"顶开"（避障），被起点终点"拉直"（贴路径），被内部张力"拉平"（平滑）。
- **带时间（Timed）**：每个节点额外带一个到下一个节点的时间间隔 `ΔT_i`。于是"轨迹"不只是几何形状，还隐含了速度/加速度信息：`v ≈ Δs/ΔT`，`a ≈ Δ(Δs/ΔT)/ΔT`。橡皮筋因此还知道"这里该慢、那里可以快"。

### 1.3 和"局部搜索类"规划器的区别

| 思路 | 代表 | 干什么 |
| --- | --- | --- |
| 采样 + 前向仿真 + 打分 | DWB | 在速度空间撒一堆 `(vx, vy, ω)`，模拟一小段，打分选最好的 |
| 随机采样控制序列 | MPPI | 撒大量控制序列，按代价加权（软最优） |
| 跟踪已有路径 | RPP / 本项目 OmniPidPursuit | 选前视点，PID/几何跟踪，不生成新几何 |
| **连续优化形变** | **TEB** | **把全局路径当初始解，连续优化位姿+时间，同时满足避障与运动学** |

一句话：DWB/MPPI 是"**撒点选优**"，TEB 是"**解一个带约束的优化问题**"。所以 TEB 的轨迹通常更平滑、更"懂时间"，代价是算力与调参复杂度。

### 1.4 数学表述（够用版）

优化变量（N 个位姿 + N-1 个时间间隔）：

```text
Q = { s_1, s_2, ..., s_N ,  ΔT_1, ..., ΔT_{N-1} }
s_i = (x_i, y_i, θ_i)
```

目标函数（所有代价项加权求和，g2o 用 χ² 形式累加）：

```text
min  Σ_k  w_k · ‖ e_k(Q) ‖²_{Ω_k}
```

其中每个 `e_k` 是一种"违反量"（不是硬约束）：

- 离障碍太近 → 违反量 = `min_obstacle_dist - d_i`（正数才罚）
- 速度超上限 → 违反量 = `|v_i| - v_max`
- 加速度超上限、朝向与运动方向不对齐、离 via-point 太远、总时间太长……

**求解器**：g2o 的 `SparseOptimizer` + Levenberg-Marquardt（本项目实测用的是解析/数值混合雅可比，见 §11.6）。

---

## 2. 一个控制周期里发生了什么（源码走读）

入口是 `TebLocalPlannerROS::computeVelocityCommands()`：
`src/teb_local_planner/src/teb_local_planner_ros.cpp:249`

```text
ControllerServer 以 controller_frequency 调用
        │
        ├─(1) pruneGlobalPlan               ros.cpp:286   砍掉机器人身后已经走过的全局路径点
        ├─(2) transformGlobalPlan           ros.cpp:292   转进局部代价地图坐标系 + 截断视野
        ├─(3) updateViaPointsContainer      ros.cpp:301   按 0.3 m 间隔抽 via-point（软引导点）
        ├─(4) configureBackupModes          ros.cpp:305   缩视野/振荡恢复（见 §6）
        ├─(5) estimateLocalGoalOrientation  ros.cpp:321   global_plan_overwrite_orientation=true 时自己算目标朝向
        ├─(6) 组装障碍物                     ros.cpp:341   costmap_converter 优先，否则逐格 LETHAL
        ├─(7) planner_->plan()              opt.cpp:298   热启动 or 重初始化 → 优化
        │        └─ optimizeTEB(inner=5, outer=4)         外层循环 + 自适应权重
        ├─(8) hasDiverged?                  ros.cpp:374   发散 → 清零 + 抛异常
        ├─(9) isTrajectoryFeasible?         ros.cpp:401   用真实 footprint 撞代价地图检查（只查前 2 个位姿）
        ├─(10) getVelocityCommand           ros.cpp:419   从轨迹里取速度（前视 look_ahead_poses 个点）
        ├─(11) saturateVelocity             ros.cpp:432   软约束兜底：硬截断到 max_vel_*
        └─ 返回 TwistStamped（→ velocity_smoother → 底盘）
```

### 2.1 步骤 (2)：视野其实被代价地图卡住

`transformGlobalPlan`（`ros.cpp:713`）里真正的截断阈值是：

```text
dist_threshold = max(costmap 宽, 高) / 2 × 0.85
```

本项目局部代价地图是 **5 m × 5 m**（`srm_nav2_params.yaml` 的 `local_costmap.width/height`），于是：

```text
2.5 × 0.85 = 2.125 m
```

也就是说虽然配了 `max_global_plan_lookahead_dist: 3.0`，**实际 TEB 每周期只优化机器人前方约 2.1 m 的路径**（这也符合 TEB"局部规划器"的定位）。想让 TEB 看得更远，要同时放大局部代价地图，而不是只改这个参数。

### 2.2 步骤 (7)：热启动与重初始化

`TebOptimalPlanner::plan()`（`src/teb_local_planner/src/optimal_planner.cpp:298`）：

- **热启动**：如果新目标与上一次轨迹末端距离 < `force_reinit_new_goal_dist`（默认 1 m）且角度差 < `force_reinit_new_goal_angular`（默认 π/2），就在旧轨迹上"剪掉走过的、接上新目标"（`opt.cpp:311`），保留上一周期的解 → 更快更稳。
- **重初始化**：否则清空弹性带，用全局路径重新铺点（`opt.cpp:315`）。全局重规划后第一次通常会走这条路。

**这一步是 TEB 抖动/跳变的主要来源**：目标点每次移动都可能触发"新同伦类"，轨迹整体跳一下。

### 2.3 离散化与自动增删点

- 初始铺点间隔按 `dt_ref`（0.3 s）与 `max_vel_x` 估算，之后每个外层迭代调用 `TimedElasticBand::autoResize(dt_ref, dt_hysteresis, min_samples, max_samples)`（`src/timed_elastic_band.cpp:230`）：
  - 某段 `ΔT` 比 `dt_ref + dt_hysteresis` 大 → **中间插一个点**；
  - 比 `dt_ref - dt_hysteresis` 小 → **删掉一个点**。
- 所以轨迹点数不是固定的：**跑得快时点会变多**（每 0.3 s 一个点），`max_samples: 500` 是上限。
- `ΔT` 在优化里**没有硬下界**（`vertex_timediff.h` 的 `oplusImpl` 只是加法），防止 `ΔT → 0` 靠的是 `autoResize` 拉回 + `1/ΔT` 型代价（速度/加速度项）自然爆炸形成的"软墙"。

### 2.4 建图：一次优化里到底加了哪些边

`TebOptimalPlanner::buildGraph()`：`src/teb_local_planner/src/optimal_planner.cpp:331`

| 顺序 | 调用 | 条件 |
| --- | --- | --- |
| 1 | `AddTEBVertices` | 位姿顶点 + 时间差顶点 |
| 2 | `AddEdgesObstacles` / `...Legacy` | `weight_obstacle != 0`（默认新式关联） |
| 3 | `AddEdgesPredictedObstacles` | `use_predicted_obstacles` 且跟踪器存在（本项目**关闭**） |
| 3' | `AddEdgesDynamicObstacles` | 否则若 `include_dynamic_obstacles`（本项目**开启**） |
| 4 | `AddEdgesViaPoints` | `weight_viapoint != 0` 且有 via-point |
| 5 | `AddEdgesVelocity` | 速度上限（`weight_max_vel_*`） |
| 6 | `AddEdgesAcceleration` | 加速度上限（`weight_acc_lim_*`） |
| 7 | `AddEdgesTimeOptimal` | 时间最优（`weight_optimaltime`） |
| 8 | `AddEdgesShortestPath` | 路径最短（`weight_shortest_path`，本项目为 0 → 不加） |
| 9 | `AddEdgesKinematicsDiffDrive` / `Carlike` | `min_turning_radius==0` 走差速分支；两个权重都是 0 时**直接不加边** |
| 10 | `AddEdgesPreferRotDir` | 仅在检测到振荡、指定偏好转向时 |
| 11 | `AddEdgesVelocityObstacleRatio` | `weight_velocity_obstacle_ratio > 0`（本项目**关闭**） |

> 关键：**权重为 0 = 这条约束根本不存在**（不是"权重很小"）。这是很多"改了没用"的原因。

### 2.5 优化：内外层循环 + 自适应权重

`optimizeTEB(no_inner_iterations=5, no_outer_iterations=4)`：

```text
for outer in 1..no_outer_iterations(4):
    autoResize()                      # 增删轨迹点
    buildGraph(weight_multiplier)     # 建图（每个外层重建一次）
    optimizeGraph(no_inner_iterations)# g2o LM 迭代 5 次
    weight_multiplier *= weight_adapt_factor   # 默认 2.0
```

- `weight_adapt_factor` 的作用：**反复失败时把避障权重越调越大**（100 → 200 → 400 …），比一开始就给个巨大权重数值条件更好。
- 副作用：一个控制周期内优化不收敛时，该周期的避障权重会被放大到很夸张，轨迹可能被"顶"得很远——这类跳变通常伴随日志里的失败信息。
- 注意一个不对称：`AddEdgesDynamicObstacles()` 在 `opt.cpp:353` **没有传 `weight_multiplier`**（用默认 1.0），所以**动态障碍代价不参与自适应放大**，静态障碍会。

### 2.6 取速度：为什么输出的是"前视几个点"的速度

`TebOptimalPlanner::getVelocityCommand()`：`src/teb_local_planner/src/optimal_planner.cpp:1180`

```text
look_ahead_poses 个 ΔT 累加到 ≥ dt_ref × look_ahead_poses
extractVelocity(Pose(0), Pose(look_ahead), dt)  → (vx, vy, ω)
```

`extractVelocity`（`opt.cpp:1142`）里有一个**决定全向/非全向的分支**：

```cpp
if (cfg_->robot.max_vel_y == 0)  // 非全向
{ vx = sign(Δs·朝向) × |Δs| / dt;  vy = 0; }
else                             // 全向
{ (vx, vy) = 把 Δs 旋转到机器人本体系 / dt; }
```

**结论：`max_vel_y` 是"这车是不是全向"的总开关**。它是 0，无论其它参数怎么配，TEB 输出 `vy` 永远是 0（本项目改造前就是这样，项目注释里也写了）。

### 2.7 可行性检查：安全网，但网眼比想象的粗

`isTrajectoryFeasible`：`src/teb_local_planner/src/optimal_planner.cpp:1295`

1. 只检查**前 `feasibility_check_no_poses` 个位姿**（本项目 **2**；若 `feasibility_check_lookahead_distance > 0` 还会按距离进一步收窄）；
2. 位姿之间若旋转超过 `min_resolution_collision_check_angular` 或平移超过内切半径，会**插值补点**再查（防止两个点分别合法、中间撞上）；
3. 用的是 **DWB 的 `ObstacleFootprintCritic::scorePose` + 真实 footprint**（本项目是半径 0.33 m 的圆）——和优化里用的 `min_obstacle_dist` 是两套判据；
4. **开启多拓扑时**（`homotopy_class_planner.cpp:699`）逻辑是：取当前最优候选检查，不可行就**把它扔掉换下一个候选**；如果不可行的恰好是上一周期那条（`last_best_teb_`），直接返回失败（避免在两条轨迹间来回跳）。

失败后果（`ros.cpp:402`）：**输出全 0 速度 + 清空规划器 + 抛 `PlannerException`**。这一周期没有任何速度输出（不是减速，是直接放弃）。

`feasibility_check_no_poses: 2` 在 0.3 s 一个点的离散下，大约只覆盖前方 0.6 s / 不到 1 m 的轨迹。**跑快时觉得"贴着障碍过去"或者偶尔擦碰，把它加到 5~10 是最直接的缓解手段**（代价是每周期多几次 footprint 碰撞检查）。

---

## 3. 代价项（g2o edges）一览

误差函数都写成"违反量"，g2o 的 χ² 会把它们平方，所以**最终代价对违反量是二次增长**。

| 边 | 顶点 | 误差（核心项） | 权重参数 |
| --- | --- | --- | --- |
| `EdgeObstacle` | pose | `penaltyBoundFromBelow(dist, min_obstacle_dist, ε)` | `weight_obstacle` |
| `EdgeInflatedObstacle` | pose | 上面一项 + `penaltyBoundToInterval(dist, 0, inflation_dist, ε)` | `weight_obstacle` / `weight_inflation` |
| `EdgeDynamicObstacle` | pose+ΔT | 与前向预测位置的余量（`min_obstacle_dist` / `dynamic_obstacle_inflation_dist`） | `weight_dynamic_obstacle(_inflation)` |
| `EdgeViaPoint` | pose | `‖s_i − via_point‖` | `weight_viapoint` |
| `EdgeVelocity`(非全向) | pose,pose,ΔT | `penaltyBoundToInterval(v, −v_max_back, v_max, ε)` + 同式(ω, ω_max, ε) | `weight_max_vel_x` / `_theta` |
| `EdgeVelocityHolonomic` | pose,pose,ΔT | 上式 + `penaltyBoundToInterval(vy, max_vel_y, 0)`（**ε=0**） | `weight_max_vel_x` / `_y` / `_theta` |
| `EdgeAcceleration`(及 Start/Goal 变体) | 3×pose,2×ΔT | `penaltyBoundToInterval(a_x, acc_lim_x, ε)` + 同式(a_θ)；Start/Goal 变体用真实起末速度 | `weight_acc_lim_x` / `_theta` |
| `EdgeAccelerationHolonomic` | 同上 | 多一维 `penaltyBoundToInterval(a_y, acc_lim_y, ε)` | `weight_acc_lim_x` / `_y` / `_theta` |
| `EdgeKinematicsDiffDrive` | pose,pose | (a) 运动方向垂直朝向的分量 (b) 后退分量 | `weight_kinematics_nh` / `_forward_drive` |
| `EdgeKinematicsCarlike` | pose,pose | 转弯半径下界 | `weight_kinematics_turning_radius` |
| `EdgeTimeOptimal` | ΔT | `ΔT_i`（越小越快） | `weight_optimaltime` |
| `EdgeShortestPath` | pose,pose,ΔT | `‖s_{i+1} − s_i‖`（点间欧氏距离） | `weight_shortest_path` |
| `EdgePreferRotDir` | pose,pose,ΔT | `penaltyBoundFromBelow(±Δθ, 0, 0)`（只有振荡恢复时加） | `weight_prefer_rotdir` |
| `EdgeVelocityObstacleRatio` | pose,pose,ΔT | 离障碍越近、允许速度越低 | `weight_velocity_obstacle_ratio` |

### 3.1 惩罚函数长什么样

`src/teb_local_planner/include/teb_local_planner/g2o_types/penalties.h`：

```text
penaltyBoundFromBelow(var, a, ε)：
    var ≥ a + ε  → 0
    否则          → a + ε − var          （线性，斜率 1）

penaltyBoundToInterval(var, a, b, ε)：
    a + ε ≤ var ≤ b − ε → 0
    var < a + ε         → a + ε − var
    var > b − ε         → var − (b − ε)
```

要点：

- 是**分段线性**（不是分段二次），二次性只来自 g2o 的 `χ² = Σ eᵀΩe`。越界距离翻倍 → 代价翻 4 倍，所以"稍微越界"可以接受、"严重越界"会被狠狠推回来。
- `penalty_epsilon`（本项目 **0.1**，源码默认 0.05）是**安全裕量**：它把可行域向内收缩，`dist ≥ min_obstacle_dist + 0.1` 才完全零代价。**把它调大 = 实际更保守**，即使你没动 `min_obstacle_dist`。
- 多个约束故意用 `ε = 0`（如非完整前向约束、转弯半径），避免把弹性带从起点顶开。

---

## 4. 多拓扑（同伦类）规划 —— 绕障能力的真正来源

开关：`enable_homotopy_class_planning`（本项目 **True**），实现在 `src/teb_local_planner/src/homotopy_class_planner.cpp`。

一个周期的流程：

```text
HomotopyClassPlanner::plan()                        hcp.cpp:113
  ├─ renewAndAnalyzeOldTebs()                       hcp.cpp:220   旧候选：剪枝、剔除绕远的（delete_detours_backwards）
  ├─ exploreEquivalenceClassesAndInitTebs()          hcp.cpp:343
  │     └─ graph_search_->createGraph(...)           hcp.cpp:361
  │           · 在起点-终点之间的矩形区域内随机采样建路网（人数 = roadmap_graph_samples，范围 = roadmap_graph_area_width / …）
  │           · 或 simple_exploration=true 时用"每个障碍左侧/右侧"的确定性策略
  ├─ addAndInitNewTeb()  ← 对每条"新拓扑"的候选路径各建一条弹性带
  ├─ optimizeAllTEBs()                               hcp.cpp:472   各候选分别跑 TEB 优化（可多线程）
  └─ selectBestTeb()                                 hcp.cpp:576
        · 代价 = 障碍代价(×selection_obst_cost_scale) + via-point 代价 + 时间代价
        · 与上次最优比较：新代价 < 旧代价 × selection_cost_hysteresis 才切换（滞回，防抖）
        · selection_prefer_initial_plan：给"和全局路径同拓扑"的候选打折
```

**同类判定**：用 **H 特征签名（H-signature）** 描述一条轨迹与各障碍的缠绕关系，签名相近 = 同拓扑 = 等价类（`h_signature_prescaler` / `h_signature_threshold`）。`max_number_classes`（本项目 4）限制同时优化几条。

实践含义：

- **绕障能力主要来自这里**：同一个障碍，左绕和右绕是两个不同拓扑，都会被生成并优化，最后按代价选。这就是"TEB 会自己绕，而跟踪类控制器只会停"的根本区别。
- **代价也主要来自这里**：每个候选都是一次完整的 TEB 优化。`max_number_classes`、`enable_multithreading`、`roadmap_graph_samples` 是 CPU 的三个旋钮。
- **滞回参数是"抖动"的解药**：`selection_cost_hysteresis: 5.0` 表示新轨迹代价要**小于旧轨迹的 1/5** 才允许切换（数值越大越"懒得换"）。
- `switching_blocking_period`（本项目 0）可以强制"切换后有 N 秒不许再切"，是治疗反复横跳的另一个开关。

---

## 5. 障碍物是怎么进来的

两条互斥路径（`ros.cpp:345`）：

1. **配了 `costmap_converter_plugin`（本项目就是这条）** → 用 `costmap_converter` 把代价地图栅格聚成**点 / 圆 / 线段 / 多边形**，走到 `updateObstacleContainerWithCostmapConverter`（`ros.cpp:501`）。
2. 没配 converter → `updateObstacleContainerWithCostmap`（`ros.cpp:471`）**逐格**把 `LETHAL_OBSTACLE` 变成点。

要点：

- converter 只取 **`cost >= LETHAL_OBSTACLE`** 的格（`src/costmap_converter/src/costmap_to_polygons.cpp:226`）→ **膨胀层（inflation）不算障碍**。所以：
  - `min_obstacle_dist` 是"**车体外缘 → 真实障碍**"的净余量，和代价地图的 `inflation_radius` **叠加**；
  - 本项目：`robot_radius 0.33 + min_obstacle_dist 0.27 = 0.60 m 中心净空`，而全局规划器只按 `robot_radius 0.33` 规划 → **TEB 明显更保守**，窄于约 1.2 m 的通道会被它拒掉（项目注释也指出了这点，要钻窄道就把 `min_obstacle_dist` 降到 0.10~0.15）。
- **多边形的建模比点更准**：墙上一个点障碍和一个多边形障碍，TEB 给的绕行形状完全不同。`CostmapToPolygonsDBSMCCH` 是凸包版本，够用且便宜。
- **动态障碍**：converter 会带 `velocities`，`include_dynamic_obstacles: True` 时会用**匀速模型**前推障碍位置（`EdgeDynamicObstacle`）。本项目 `odom_topic: odometry` 就是喂给 converter 估速度用的。
- 本 fork 额外提供 **Kalman 跟踪 + 加速度预测**（`use_predicted_obstacles`，默认 **false**，本项目未开）；开了就用 `EdgePredictedObstacle` 取代匀速模型。注意它与 `include_dynamic_obstacles` 是**二选一**（`opt.cpp:350-353`）。
- 还可以通过话题 `~/obstacles` 直接塞自定义障碍（`updateObstacleContainerWithCustomObstacles`，`ros.cpp:559`），适合把视觉/雷达检出的动态目标喂进来。

---

## 6. TEB 自带的恢复机制

这些是**规划器内部**的恢复（在 `configureBackupModes()`，`ros.cpp:989`），和 Nav2 的恢复行为（Spin / BackUpFreeSpace / DriveOnHeading / Wait）是两层东西。

| 机制 | 参数 | 行为 |
| --- | --- | --- |
| 缩短视野 | `shrink_horizon_backup`（True）、`shrink_horizon_min_duration`（10 s） | 出现不可行轨迹后，把局部目标**砍到一半**（连续失败 10 次再砍一半），并保持至少 10 s。**先活下来再说**：视野短了更容易找到可行解，避免直接抛异常停车 |
| 振荡恢复 | `oscillation_recovery`（True）、`oscillation_v_eps` / `oscillation_omega_eps`（0.1）、`oscillation_*_duration`（10 s） | 检测"平均速度/角速度都很小但一直在动" = 左右横跳，锁定当前转向偏好（`EdgePreferRotDir`）持续 10 s |
| 发散检测 | `divergence_detection_enable`（源码默认 **false**，本项目未开） | 用马氏距离判断优化发散，开了会多算 Hessian 统计量（更贵） |

失败在 Nav2 侧的表现（`ros.cpp:360/374/402/419` 四处都会抛 `PlannerException`）：

```text
抛异常 → ControllerServer 记一次失败 → 超过 failure_tolerance(0.3s) → 触发恢复行为
                                     → 或 progress_checker 判定卡住(0.5 m / 10 s) → 全局重规划
```

---

## 7. 本项目接线情况

### 7.1 包与插件

| 项 | 值 |
| --- | --- |
| 源码 | `src/teb_local_planner`（另有消息包 `src/teb_msgs`） |
| 版本 | `0.9.1`（`package.xml`），ROS2/Nav2 移植 |
| 插件类 | `teb_local_planner::TebLocalPlannerROS`（基类 `nav2_core::Controller`） |
| 依赖 | `g2o`、`nav2_core`、`nav2_costmap_2d`、`costmap_converter`、`dwb_critics`、`teb_msgs` |
| 注册 | `install/teb_local_planner/share/ament_index/resource_index/nav2_core__pluginlib__plugin/teb_local_planner` → `share/teb_local_planner/teb_local_planner_plugin.xml`（pluginlib 可发现，**已构建**） |

### 7.2 三份配置，谁在用哪份

| 文件 | 局部规划器 | 谁加载它 |
| --- | --- | --- |
| `config/reality/srm_nav2_params.yaml` | **TEB** | `script/start_real_nav.sh`（默认参数文件）→ `bringup_launch.py` |
| `config/simulation/nav2_params_teb.yaml` | **TEB** | 手动用 `params_file:=...` 指定 |
| `config/simulation/nav2_params.yaml` | OmniPidPursuit | `rm_navigation_simulation_launch.py` 等**默认**用的还是这份 |

也就是说：**实车已经在跑 TEB；仿真默认还是 OmniPid，要比对 TEB 必须显式换参数文件。**

### 7.3 怎么跑 / 怎么确认生效

```bash
# 实车（默认就是 TEB 配置）
bash script/start_real_nav.sh --lio

# 仿真里跑 TEB 版参数
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 slam:=False use_pcd_localization:=False use_sim_time:=True \
  params_file:=$HOME/pb2025_sentry_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params_teb.yaml

# 确认加载的是 TEB 还是 OmniPid
ros2 param get /controller_server FollowPath.plugin

# 确认控制频率是否真的跑得到（TEB 常见"跟不上"）
ros2 topic hz /cmd_vel            # 或实车 /cmd_vel_controller
```

日志里出现 `Costmap conversion plugin ... loaded.` 才说明 converter 生效（否则障碍物走的是逐格点路径）。
出现 `Control loop missed its desired rate` 说明这一周期 TEB 没算完，需要降 `controller_frequency` 或降 `max_number_classes`。

### 7.4 数据流

```text
Theta*(全局)  ──global plan──►  TEB(Controller 插件)
                                   │  需要：odom（map→base 位姿）、局部 costmap（障碍）、TF
                                   ▼
                              cmd_vel（TwistStamped）
                                   ▼
                          velocity_smoother（实车：max_velocity [0.5, 0.5, 1.0]）
                                   ▼
                                底盘串口
```

注意：**TEB 输出的速度还会被 `velocity_smoother` 削一次**。实车 `max_vel_x: 1.0` 并不代表真的跑 1.0 m/s，实际被 `velocity_smoother` 的 0.5 m/s 卡住（配置注释里已说明）。调 TEB 的速度上限时，**两份配置要一起看**。

---

## 8. 参数详解（按本项目配置顺序）

### 8.1 轨迹离散

| 参数 | 本项目 | 含义 / 调整方向 |
| --- | --- | --- |
| `teb_autosize` | 1.0 | 自动增删轨迹点。**保持开启** |
| `dt_ref` | 0.3 | 期望时间分辨率（秒/点）。≈ 控制器周期的 1~2 倍比较合理；调小=更精细更贵，调大=点更少更粗糙、轨迹更"折" |
| `dt_hysteresis` | 0.1 | autosize 的滞回带宽，避免频繁增删点（一般取 dt_ref 的 10%~30%） |
| `min_samples` | 未配（默认 3） | 最少点数 |
| `max_samples` | 500 | 点数上限。**轨迹被拉得又长又快时点数会飙升**（点 ≈ 总时长/dt_ref），是"CPU 忽高忽低"的常见原因 |
| `global_plan_overwrite_orientation` | True | 用全局路径几何自己算局部目标朝向。**本项目必须 True**：Theta* 不保证路径点朝向，中间点全是 `yaw=0`，用 False 会让车"莫名其妙拧着走" |
| `allow_init_with_backwards_motion` | False | 是否允许初始化成倒车。**没装后向传感器就别开** |
| `global_plan_viapoint_sep` | 0.3 | 沿全局路径每 0.3 m 取一个 via-point，把 TEB 拉回路径附近（软约束）。调大=更自由（易抄近道/绕远），调小=更贴路径（可能更僵） |
| `max_global_plan_lookahead_dist` | 3.0 | 优化视野上限（实际被局部代价地图压到 ~2.1 m，见 §2.1） |
| `global_plan_prune_distance` | 1.0 | 机器人身后多远之外的路径点剪掉 |
| `exact_arc_length` | False | 用精确弧长算速度/加速度（更准更慢）。车小、dt 小的时候 False 足够 |
| `feasibility_check_no_poses` | **2** | 每个周期用真实 footprint 检查前几个位姿。**安全网眼**，建议 5~10 |
| `feasibility_check_lookahead_distance` | 未配（默认 -1） | 按距离限制检查范围（-1 = 只按点数） |
| `control_look_ahead_poses` | 未配（默认 **1**） | 取速度时跳过几个点。调大 = 更平滑但更"迟"、切弯更早；抖的时候可以试 2~3 |
| `min_resolution_collision_check_angular` | 未配（默认 π） | 碰撞检查的角度插值阈值（π 基本等于不插值） |
| `publish_feedback` | False | 发布 `/teb_feedback`（完整轨迹+障碍+各候选）。**排障时临时开一下很值** |

### 8.2 机器人运动学（全向 vs 非全向）

| 参数 | 本项目 | 含义 |
| --- | --- | --- |
| `max_vel_x` | 1.0（实车）/ 1.0（仿真） | 前进速度上限（实车还会被 velocity_smoother 削到 0.5） |
| `max_vel_x_backwards` | 未配（默认 0.2） | 倒车速度上限 |
| `max_vel_y` | **0.5**（实车）/ 1.0（仿真） | **全向总开关**：0 = 禁用横移（非全向），>0 = 允许横移 |
| `max_vel_theta` | 1.0 | 角速度上限 |
| `acc_lim_x` / `acc_lim_y` / `acc_lim_theta` | 2.5 / 1.0 / 3.2（实车） | 加速度上限。**`acc_lim_y` 也必须 > 0 才走全向加速度边** |
| `footprint_model` | circular, r=0.33（实车）/ 0.2（仿真） | 车体模型。**必须与代价地图的 `robot_radius` 一致**，否则"以为过得去"或"过分保守" |
| `weight_kinematics_nh` | **0.0** | 非完整约束权重。1000 = 强迫朝向对齐运动方向（会输出 `angular.z` 转底盘）；**0 = 完全不约束朝向**（纯平移，麦克纳姆常用） |
| `weight_kinematics_forward_drive` | **0.0** | 惩罚倒车。两个都为 0 时整条运动学边都不加（`opt.cpp:963`） |
| `weight_kinematics_turning_radius` | 1.0 | 只对 `min_turning_radius > 0` 的车型生效，本项目未设 `min_turning_radius`（默认 0）→ **这条边不会加**，数值无实际作用 |
| `free_goal_vel` | False | 到达目标时速度是否可非零（一般 False，要求刹停） |

### 8.3 障碍物

| 参数 | 本项目 | 含义 / 调整方向 |
| --- | --- | --- |
| `min_obstacle_dist` | 0.27 | **车体外缘到障碍的最小净余量**（与膨胀层叠加）。调大更保守；钻窄道降到 0.10~0.15 |
| `inflation_dist` | 0.6 | 软缓冲区：> `min_obstacle_dist` 时才生效，区间内付小额代价（`weight_inflation`）。**它比 min_obstacle_dist 大 → 走 `EdgeInflatedObstacle`**，所以轨迹会"先躲远一点再贴过去" |
| `include_dynamic_obstacles` | True | 用匀速模型预测动障碍 |
| `dynamic_obstacle_inflation_dist` | 0.6 | 动障碍的软缓冲区 |
| `include_costmap_obstacles` | True | ⚠️ **配了 `costmap_converter_plugin` 后这个开关不起作用**（代码只在"逐格取点"那条分支里读它，`ros.cpp:474` 的调用点仅在无 converter 时进入） |
| `costmap_obstacles_behind_robot_dist` | 1.0 | 车身侧后方多远之内的障碍还算进来（只在逐格分支生效） |
| `obstacle_poses_affected` | 15 | 一个障碍影响它最近轨迹点周围多少个点（"影响范围"，影响避障的"宽度"和平滑度） |
| `costmap_converter_plugin` | `CostmapToPolygonsDBSMCCH` | 栅格→多边形（凸包）。可换 `...LinesRANSAC` 等 |
| `costmap_converter_spin_thread` | True | converter 用独立线程（True 更稳） |
| `costmap_converter_rate` | 5 | ⚠️ 见 §11.2：这个名字在本 fork 里**启动时不读**，写多少都用代码默认 5 |

### 8.4 优化权重（最需要理解的一组）

| 参数 | 本项目 | 作用与调参方向 |
| --- | --- | --- |
| `no_inner_iterations` | 5 | 每次建图后 g2o 迭代次数。**CPU 主要旋钮** |
| `no_outer_iterations` | 4 | 外层循环次数（每次 re-autosize + 重建图 + 自适应加权） |
| `penalty_epsilon` | 0.1 | 约束内缩量（见 §3.1）。调大 = 更保守 |
| `obstacle_cost_exponent` | 4.0 | 障碍代价的非线性指数：越界越深罚得越狠（=1 为线性）。**调大 = 更"绝不贴近"** |
| `weight_max_vel_x/_y/_theta` | 0.5 / 2.0 / 0.5 | 速度上限软约束。调大=更严格守速度上限（但会挤压其它目标） |
| `weight_acc_lim_x/_y/_theta` | 0.5 / 1.0 / 10.5 | 加速度上限软约束。`_theta` 偏大是刻意的：限制转向"生硬" |
| `weight_optimaltime` | 1.0 | 时间最优。**调大 = 更急/更快**（也更容易顶到速度上限） |
| `weight_shortest_path` | 0.0 | 路径最短项。**0 = 不加这条边**（想惩罚绕远可给个小值，如 1.0） |
| `weight_obstacle` | 100.0 | 避障权重（会被 `weight_adapt_factor` 逐轮放大） |
| `weight_inflation` | 0.2 | 软缓冲区代价（应远小于 `weight_obstacle`） |
| `weight_viapoint` | 50.0 | 贴全局路径的权重。**调大 = 更贴路径（但绕障意愿变弱）** |
| `weight_dynamic_obstacle(_inflation)` | 10.0 / 0.2 | 动态障碍 |
| `weight_adapt_factor` | 2.0 | 每外层 ×2 放大障碍权重（见 §2.5） |

> **平衡关系**：`weight_viapoint` ↑ 与 `weight_obstacle` ↓ 都会让车"更贴原路径、更不愿意绕"；反过来就容易绕远甚至画圈。调这两个比值基本等于调"听话 vs 灵活"。

### 8.5 多拓扑

| 参数 | 本项目 | 作用 |
| --- | --- | --- |
| `enable_homotopy_class_planning` | True | **绕障能力开关**；关掉后 TEB 退化成"在全局路径附近做形变"，几乎不会主动绕 |
| `enable_multithreading` | True | 多候选并行优化（多核才有意义） |
| `max_number_classes` | 4 | 同时保留/优化的拓扑数。**CPU 与绕障丰富度的权衡** |
| `roadmap_graph_samples` | 见 §11.1 | 建路网的采样点数（YAML 里写的名字不对） |
| `roadmap_graph_area_width` | 5.0 | 采样矩形区域的宽度（米），越大越敢往侧面绕 |
| `roadmap_graph_area_length_scale` | 1.0 | 区域长度相对起终点距离的缩放 |
| `h_signature_prescaler` | 0.5 | H 签名缩放（障碍多时防止签名重叠） |
| `h_signature_threshold` | 0.1 | 判定"同拓扑"的阈值 |
| `obstacle_heading_threshold` | 0.45 | 只有"朝向与目标方向一致度"达标的障碍才用于生成绕行候选 |
| `selection_cost_hysteresis` | 5.0 | **防抖滞回**：新轨迹要比旧的好 5 倍才切换 |
| `selection_prefer_initial_plan` | 1.0 | 给"和全局路径同拓扑"的候选打折（1.0 = 不打折） |
| `selection_obst_cost_scale` | 1.0 | 选优时障碍代价的额外缩放（源码默认 100，本项目改成 1） |
| `selection_alternative_time_cost` | True | 选优时用总时长做时间代价（更符合"哪条更快"） |
| `switching_blocking_period` | 0.0 | 切换后锁定期（0 = 不锁）；**反复横跳可以试着给 1~3 s** |
| `viapoints_all_candidates` | True | 所有候选都连 via-point（否则只有初始拓扑连） |
| `delete_detours_backwards` | True | 丢弃"绕得比最优还久 3 倍"的候选（`max_ratio_detours_duration_best_duration`） |
| `visualize_hc_graph` | False | 可视化路网图（排障用） |

### 8.6 恢复

见 §6 的表格（本项目 `shrink_horizon_backup`、`oscillation_recovery` 都开着，`divergence_detection_enable` 没开）。

---

## 9. 调参 playbook（症状 → 先动哪个）

| 症状 | 优先检查/调整 |
| --- | --- |
| **原地打转 / 轨迹"打结"** | ① 朝向约束是否与坐标系矛盾：`weight_kinematics_nh=0`（全向车/合成坐标系必须为 0）；② 仿真里 `local_costmap.robot_base_frame` 是 `gimbal_yaw_fake` 这种"合成系"时，**不能去控朝向**；③ `oscillation_recovery` 是否开着 |
| **左右横跳 / 反复换道** | `selection_cost_hysteresis` ↑（如 10~20）、`switching_blocking_period` 给 1~3 s、`weight_viapoint` ↑ |
| **贴障碍太近 / 偶尔擦碰** | `min_obstacle_dist` ↑、`feasibility_check_no_poses` ↑（2 → 5~10）、`obstacle_cost_exponent` ↑、`weight_obstacle` ↑ |
| **窄道过不去 / 绕大圈** | `min_obstacle_dist` ↓（0.27 → 0.10~0.15）、代价地图 `inflation_radius` ↓、`weight_viapoint` ↑、必要时关 `enable_homotopy_class_planning` 对比 |
| **频繁 `trajectory is not feasible` 后停车** | `shrink_horizon_backup` 保持 True、`max_samples` ↑（点太多被截断）、检查 footprint 半径与 costmap `robot_radius` 是否一致、膨胀半径是否把通道堵死 |
| **`Control loop missed its desired rate`** | `controller_frequency` ↓（本项目已从 20 → 5）、`max_number_classes` ↓、`max_samples` ↓、`no_outer_iterations` ↓、`enable_multithreading` 确认 True |
| **跑起来太慢/太急** | `weight_optimaltime`（↑更急）、`max_vel_*`、`weight_max_vel_*`、实车别忘 `velocity_smoother.max_velocity` |
| **切弯太早/太晚、跟踪不贴线** | `control_look_ahead_poses`（1 → 2~3）、`dt_ref`、`global_plan_viapoint_sep`、`weight_viapoint` |
| **目标点附近来回蹭** | `xy_goal_tolerance`（goal_checker，本项目 0.15）、`free_goal_vel`、`weight_optimaltime` |
| **算力忽高忽低** | 点数是自适应的 → 看 `max_samples`；速度越快轨迹点越多 |

---

## 10. 全向（麦克纳姆）改造检查清单

要让 TEB 真正输出并利用 `vy`，**下面每一处都要满足**，漏一个就会"看起来改了但没效果"：

1. **TEB 侧**：`max_vel_y > 0`（`extractVelocity` 才走全向分支，`opt.cpp:1154`）
2. **TEB 侧**：`acc_lim_y > 0`（否则加速度边走非全向分支，`opt.cpp:823`）
3. **TEB 侧**：`weight_max_vel_y > 0`、`weight_acc_lim_y > 0`（权重为 0 = 不加边）
4. **TEB 侧**：`weight_kinematics_nh = 0` **且** `weight_kinematics_forward_drive = 0`（不加朝向对齐边，`opt.cpp:963`）——注意这是"不控朝向"，**不是**"支持横移"的必要条件，但全向车通常就是要它
5. **Nav2 侧**：`min_y_velocity_threshold` 必须**降到 ~0.05**（作用在**里程计反馈的 vy** 上，低于阈值直接置 0；差速车模板默认 0.5 会把横向速度反馈全部吃掉）
6. **代价地图侧**：`robot_base_frame` 必须是**真实底盘系**。仿真里用的是 `gimbal_yaw_fake`（`fake_vel_transform` 用自身里程计偏航反算的合成系），拿它当控制目标会永远对不齐 → 角速度顶死、原地打转
7. **下游侧**：`velocity_smoother.max_velocity` 的 y 分量要够大（实车 `[0.5, 0.5, 1.0]`），否则 TEB 算的 vy 会被削掉
8. **下游侧**：底盘固件/串口协议要真的支持横移速度（`vy` 字段被消费）

> 前 7 条在仓库里都已按全向配置改过（写作时的状态）；第 8 条属于实车验证项。

---

## 11. 代码里值得知道的坑（本项目 fork 实测）

### 11.1 `roadmap_graph_no_samples` 这个键是无效的

- 两份 nav2 参数文件里都写了 `roadmap_graph_no_samples: 15`。
- 本 fork 在**启动时**声明/读取的名字是 **`roadmap_graph_samples`**（`src/teb_local_planner/src/teb_config.cpp:151`、`:286`），而 `roadmap_graph_no_samples` 只出现在**运行时动态改参回调**里（`teb_config.cpp:678`）。
- 后果：YAML 里这个键在启动时不会被读取（默认值恰好也是 15，所以看不出问题）。**要改采样点数，必须写 `roadmap_graph_samples`。**
- 验证方法：`ros2 param list /controller_server | grep roadmap` 看实际存在的名字。

### 11.2 `costmap_converter_rate` 启动时也不读

`costmap_converter_rate` 在本 fork 里既没有 `declare_parameter`，也没有 `get_parameter`（只有动态回调里有，`teb_config.cpp:666`），启动时用的是构造默认值 **5**（`teb_config.h:324`）。YAML 里写 5 恰好一致；**改成别的值不会生效**。

### 11.3 `include_costmap_obstacles` 被 converter 顶掉了

配置加载顺序上，只要 `costmap_converter_plugin` 非空就走 converter 分支（`ros.cpp:122-137`、`ros.cpp:345`），逐格取点的分支（含 `include_costmap_obstacles` 判断）根本不进入。**这个键在本项目里是无效果的**。

### 11.4 可行性与"不断换候选"的微妙之处

`homotopy_class_planner.cpp:699` 的可行性检查会**依次扔掉不可行的候选**直到找到一个可行的。如果所有候选都不可行，且最优的那条与上一周期相同，它会直接返回失败（防抖）。表现是：**明明是静止的障碍，车却在不同周期里选择不同绕法**——这属于设计行为，不是 bug；用 `selection_cost_hysteresis` / `switching_blocking_period` 抑制。

### 11.5 软约束 + 硬截断

优化出来的 `vx/vy/ω` 只保证"代价上不鼓励越界"，最后 `saturateVelocity`（`ros.cpp:432`）会**硬截断**到 `max_vel_*`。所以偶尔看到输出贴在上限是正常的；但若长期顶在限幅上，说明权重太小或目标本身不可达。

### 11.6 雅可比：一半解析、一半数值

- `USE_ANALYTIC_JACOBI` 在 `include/teb_local_planner/teb_config.h:52` **是定义的**。
- 但 **障碍、速度、加速度**三条边的解析雅可比被内层 `#if 0` 关掉了（`edge_obstacle.h:108`、`edge_velocity.h:121`、`edge_acceleration.h:152`）→ 走 g2o 数值雅可比（每个变量都要扰动求导，**这是最贵的一块**）。
- **运动学边（差速/车型）和时间最优边**保留了 `#if 1` 的解析雅可比。
- 实践含义：想省 CPU，优先减少**障碍物关联数量**（`obstacle_poses_affected`、障碍多边形/点的个数），而不是改迭代次数。

### 11.7 「发散检查」在本项目里其实是个空操作

`hasDiverged()` 的第一行就是：

```cpp
if (!cfg_->recovery.divergence_detection_enable) return false;   // opt.cpp:1071
```

而 `divergence_detection_enable` 源码默认 **false**、本项目**没有配置** → `ros.cpp:374` 那个"发散就清零"的分支**永远不会触发**。

后果：**轨迹"打结"、原地转圈这类病态解不会被判为发散**，会一路输出下去——本项目在仿真里遇到的"弹性带塌缩成 262 个点的结"就属于这种情况（`max_samples` 被打满通常同时出现）。
要真的用上这个保护，得同时给：

```yaml
divergence_detection_enable: true
divergence_detection_max_chi_squared: 10.0   # 默认值
```

代价是每个周期多算 Hessian 批量统计量（更慢）。排查病态轨迹更划算的做法还是看可视化 `/local_plan`、`/teb_poses`，别只看 `cmd_vel`。

### 11.8 `weight_prefer_rotdir` 与时间代价

`EdgePreferRotDir` 只在振荡恢复激活时加入（`opt.cpp:1006` 起），且源码注释明确提醒它可能让开环/闭环表现不一致。**不要长期开着当"偏好左转"用。**

---

## 12. 观测与排障速查

RViz 里直接看（都属于 controller_server 的相对话题）：

| 话题 | 内容 |
| --- | --- |
| `/local_plan` | **最终选中的优化轨迹**（最该看的） |
| `/teb_poses` | 轨迹位姿数组 |
| `/teb_markers` | 障碍物、via-point、不可行位姿（namespace `InfeasibleRobotPoses`）、时间轴可视化等全部标记 |
| `/teb_feedback` | 完整诊断：**所有候选轨迹** + 选中的索引 + 活跃障碍（需要 `publish_feedback: true`） |

常用命令：

```bash
ros2 param get /controller_server FollowPath.plugin          # 确认是 TEB
ros2 param get /controller_server FollowPath.max_vel_y       # 确认真的是全向
ros2 param get /local_costmap/local_costmap inflation_radius # 与 min_obstacle_dist 一起看
ros2 topic hz /local_plan                                    # 实际规划频率
ros2 topic hz /cmd_vel_controller                            # 实际控制输出频率（实车 /cmd_vel）
```

> 用了命名空间（仿真里是 `/red_standard_robot1`）时，节点和话题名都要带上，例如
> `ros2 param get /red_standard_robot1/controller_server FollowPath.plugin`。

日志关键字（按"出问题先搜这个"排序）：

| 关键字 | 含义 |
| --- | --- |
| `trajectory is not feasible` | 前几个位姿撞了 → 本周期输出 0 速度 |
| `not able to obtain a local plan` | 优化失败（常伴随视野被堵死） |
| `the trajectory has diverged` | 优化发散，已清空规划器（**本项目未开发散检测，见 §11.7，实际不会出现**） |
| `Activating reduced horizon backup mode` | 视野被砍半，说明刚经历过失败 |
| `possible oscillation ... detected` | 检测到横跳，锁定转向偏好 10 s |
| `Control loop missed its desired rate` | TEB 算不过来（Nav2 侧） |

---

## 13. 与其他方案对比 & 本项目建议

| 方案 | 基本思想 | 绕障 | 全向 | 平滑 | 算力/调参 |
| --- | --- | --- | --- | --- | --- |
| OmniPidPursuit（本项目原控制器） | 跟踪路径 + PID + 曲率限速 | 弱（只会停/重规划） | 好 | 好 | 低/低 |
| DWB | 速度采样 + 前向模拟 + 打分 | 中 | 好 | 中 | 中/较高 |
| **TEB** | **位姿+时间的图优化，多拓扑** | **强** | **支持（需按 §10 配全）** | 好 | **高/高** |
| MPPI | 大量控制序列采样，代价加权 | 强 | 好 | 很好 | 高/中高 |
| MPC | 显式动力学 + 在线优化 | 强 | 好 | 很好 | 很高/很高 |

对本项目（麦克纳姆哨兵、静态场地 + 偶发动态目标）：

- **TEB 的价值**：遇到动态/临时障碍时能主动绕，而不是"停—恢复—重规划"；轨迹带时间信息，速度规划比纯 PID 更合理。
- **TEB 的代价**：控制频率掉到 5 Hz；参数多、相互耦合；行为对 `max_vel_y`/`weight_kinematics_nh`/代价地图坐标系这些"全局开关"非常敏感（§10、§11）。
- **落地顺序建议**：
  1. 仿真按 `nav2_params_teb.yaml` 跑通静态场地的绕桩（用 `/local_plan` 确认轨迹几何正常、不打结）；
  2. 打开 `publish_feedback` 看多拓扑候选是否合理（该绕的时候有没有生成绕行候选）；
  3. 实车先把速度压到 `velocity_smoother` 的 0.5 m/s 跑通，再逐步放开；
  4. 若最终目标是"更平滑、更省调参"，`局部规划器相关(ai).md` 里给的优先级（MPPI > DWB > TEB）仍值得作为备选路线评估。

---

## 14. 参考资料

- 源码（本项目自带）：`src/teb_local_planner`、`src/teb_msgs`、`src/costmap_converter`
- 论文：C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann, T. Bertram, *Trajectory modification considering dynamic constraints of autonomous robots*（TEB 原始论文）；以及 *Integrated online trajectory planning and optimization in distinctive topologies*（同伦类规划）
- Nav2 官方文档：Controller Server、Costmap 2D、`nav2_mppi_controller`（对比用）；`min_*_velocity_threshold` 的含义见 Controller Server 参数表
- 项目内：`局部规划器相关(ai).md`（方案对比）、`调试日志.md`（TEB 上手记录/膨胀半径调整记录）
- 上游参考参数：`src/teb_local_planner/params/teb_params.yaml`（本项目的实车/仿真 TEB 配置就是从它改出来的，差异见 git 历史）

---

## 附：写作时（2026-09-26）的关键数值快照

实车 `config/reality/srm_nav2_params.yaml`：

```text
controller_frequency: 5.0        min_y_velocity_threshold: 0.05
max_vel_x: 1.0  max_vel_y: 0.5  max_vel_theta: 1.0
acc_lim_x: 2.5  acc_lim_y: 1.0  acc_lim_theta: 3.2
footprint: circular r=0.33       min_obstacle_dist: 0.27  inflation_dist: 0.6
dt_ref: 0.3  dt_hysteresis: 0.1  max_samples: 500  feasibility_check_no_poses: 2
penalty_epsilon: 0.1  obstacle_cost_exponent: 4.0
weight_obstacle: 100  weight_viapoint: 50  weight_optimaltime: 1.0
weight_kinematics_nh: 0.0  weight_kinematics_forward_drive: 0.0
enable_homotopy_class_planning: True  max_number_classes: 4  selection_cost_hysteresis: 5.0
velocity_smoother.max_velocity: [0.5, 0.5, 1.0]
```

> 这些数值正在被持续调整（尤其速度与权重）。**参数含义看 §8，数值看文件。**
