# tdt_nav_kit

ROS 2 package containing the reusable algorithms from `tdt-nav-kit`:

- `tdt_nav_frontend`: `YAstar` and `KinodynamicAstar` grid search.
- `tdt_nav_trajectory`: `MinimumSnap` and `SfcSquare` trajectory optimization, built when `OsqpEigen` is available.

Headers are installed under `tdt_nav_kit/YAstar` and `tdt_nav_kit/MinimumSnapOsqp`.
The package does not replace the workspace's current Nav2 planner configuration.
OpenCV is used when available for the image-based helper APIs; YAstar retains a non-OpenCV SDF fallback.

## Simulation validation

Build and source the workspace first:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select tdt_nav_kit
source install/setup.bash
```

Start Gazebo, the existing navigation stack, and then the validation node in three terminals:

```bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py use_referee:=False
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py
ros2 launch tdt_nav_kit tdt_nav_kit_demo.launch.py namespace:=red_standard_robot1
```

In RViz, set an initial pose with `2D Pose Estimate`. Publish a goal to the demo topic (the default Nav2 RViz Goal tool sends an action goal instead):

```bash
ros2 topic pub --once /red_standard_robot1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 5.17, y: 6.01}, orientation: {w: 1.0}}}"
```

The node consumes `initialpose` and `goal_pose`, runs YAstar on the namespaced `map`, and publishes `tdt_nav_kit/path`. Add an RViz `Path` display for `/red_standard_robot1/tdt_nav_kit/path` to compare the result with Nav2's current planner. This validation node publishes a path for inspection; it does not replace Nav2's Theta* plugin or command the robot.

The original upstream source is MIT licensed. OSQP, OsqpEigen and Eigen retain
their respective upstream licenses; see `THIRD_PARTY_LICENCE`.
