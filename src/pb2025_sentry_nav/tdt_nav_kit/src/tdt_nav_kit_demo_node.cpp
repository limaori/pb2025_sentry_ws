#include "tdt_nav_kit/YAstar/yastar.hpp"

#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class TdtNavKitDemoNode : public rclcpp::Node {
public:
  TdtNavKitDemoNode() : Node("tdt_nav_kit_demo") {
    map_topic_ = declare_parameter("map_topic", "map");
    start_topic_ = declare_parameter("start_topic", "initialpose");
    goal_topic_ = declare_parameter("goal_topic", "goal_pose");
    path_topic_ = declare_parameter("path_topic", "tdt_nav_kit/path");
    plan_period_ = declare_parameter("plan_period", 1.0);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_ = *message;
        has_map_ = true;
      });
    start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      start_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        start_.header = message->header;
        start_.pose = message->pose.pose;
        has_start_ = true;
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_ = *message;
        has_goal_ = true;
      });
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(10));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(plan_period_)),
      [this]() { plan(); });

    RCLCPP_INFO(get_logger(), "Waiting for map on '%s', start on '%s', goal on '%s'",
      map_topic_.c_str(), start_topic_.c_str(), goal_topic_.c_str());
  }

private:
  void plan() {
    nav_msgs::msg::OccupancyGrid map;
    geometry_msgs::msg::PoseStamped start;
    geometry_msgs::msg::PoseStamped goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_map_ || !has_start_ || !has_goal_) return;
      map = map_;
      start = start_;
      goal = goal_;
    }
    if (map.info.width == 0 || map.info.height == 0 ||
        map.data.size() != static_cast<size_t>(map.info.width) * map.info.height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring invalid occupancy grid");
      return;
    }
    if (std::abs(map.info.origin.orientation.z) > 1e-4 ||
        std::abs(map.info.origin.orientation.w - 1.0) > 1e-4) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Map origin rotation is unsupported by this demo; expected zero yaw");
    }

    std::vector<int8_t> planner_map(map.data.size(), 100);
    for (size_t index = 0; index < map.data.size(); ++index) {
      planner_map[index] = map.data[index] == 0 ? 0 : 100;
    }
    YAstar planner;
    planner.setMap(static_cast<int>(map.info.width), static_cast<int>(map.info.height),
      map.info.resolution, static_cast<float>(map.info.origin.position.x),
      static_cast<float>(map.info.origin.position.y), planner_map);
    planner.initCostMap();
    const auto points = planner.search(
      Eigen::Vector2f(static_cast<float>(start.pose.position.x), static_cast<float>(start.pose.position.y)),
      Eigen::Vector2f(static_cast<float>(goal.pose.position.x), static_cast<float>(goal.pose.position.y)));

    nav_msgs::msg::Path path;
    path.header = map.header;
    path.header.stamp = now();
    for (size_t index = 0; index < points.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[index].x();
      pose.pose.position.y = points[index].y();
      const auto &next = points[index + 1 < points.size() ? index + 1 : index];
      const double yaw = std::atan2(next.y() - points[index].y(), next.x() - points[index].x());
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    if (points.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "YAstar returned no path");
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "YAstar path: %zu points, %.2f m",
        points.size(), YAstar::getLength(points));
    }
  }

  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid map_;
  geometry_msgs::msg::PoseStamped start_;
  geometry_msgs::msg::PoseStamped goal_;
  bool has_map_{false};
  bool has_start_{false};
  bool has_goal_{false};
  std::string map_topic_;
  std::string start_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  double plan_period_{1.0};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TdtNavKitDemoNode>());
  rclcpp::shutdown();
  return 0;
}
