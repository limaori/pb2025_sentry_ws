#!/usr/bin/env python3

# Copyright 2026 Shiyu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SimulationGroundTruthOdometry(Node):
    def __init__(self):
        super().__init__("simulation_ground_truth_odometry")
        self.initial_pose = None
        self.zero_pose_since = None
        self.zero_pose_grace_ns = 2_000_000_000
        self.last_publish_time = None
        self.publish_period_ns = 20_000_000  # 50 Hz is sufficient for Nav2.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, "odometry", 10)
        self.create_subscription(
            Odometry, "chassis_odometry_gt", self.odometry_callback, 10
        )

    def odometry_callback(self, message):
        pose = message.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        if self.initial_pose is None:
            # Gazebo can publish one zero-valued odometry sample while the
            # robot entity is still being spawned. Do not use that placeholder
            # as the odometry origin, or the real spawn pose will appear
            # outside the Nav2 map.
            is_zero_pose = (
                abs(pose.position.x) < 1e-6
                and abs(pose.position.y) < 1e-6
                and abs(pose.position.z) < 1e-6
                and abs(yaw) < 1e-6
            )
            if is_zero_pose:
                # A paused Gazebo world can repeatedly publish the placeholder
                # pose while /clock is still zero.  Never initialize from it:
                # the first real pose after unpausing may be the configured
                # spawn pose, far away from the world origin.
                if self.get_clock().now().nanoseconds == 0:
                    self.zero_pose_since = None
                    return
                now_ns = time.monotonic_ns()
                if self.zero_pose_since is None:
                    self.zero_pose_since = now_ns
                # Some Gazebo versions emit zero odometry until the model's
                # odometry plugin has settled. Give a real pose priority, but
                # do not leave Nav2 without an odom TF forever when zero is
                # the intended relative-odometry origin.
                if now_ns - self.zero_pose_since < self.zero_pose_grace_ns:
                    return
            self.initial_pose = (pose.position.x, pose.position.y, yaw)
            self.get_logger().info("Ground-truth odometry origin initialized")

        # Use the node's ROS clock for every derived message.  Gazebo bridge
        # messages can carry a stale source timestamp after /clock starts;
        # forwarding it makes Nav2 request transforms from the distant past.
        stamp = self.get_clock().now()
        publish_time_ns = time.monotonic_ns()
        if (
            self.last_publish_time is not None
            and publish_time_ns - self.last_publish_time < self.publish_period_ns
        ):
            return
        self.last_publish_time = publish_time_ns

        initial_x, initial_y, initial_yaw = self.initial_pose
        world_dx = pose.position.x - initial_x
        world_dy = pose.position.y - initial_y
        cos_yaw = math.cos(initial_yaw)
        sin_yaw = math.sin(initial_yaw)
        x = cos_yaw * world_dx + sin_yaw * world_dy
        y = -sin_yaw * world_dx + cos_yaw * world_dy
        relative_yaw = yaw - initial_yaw

        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(relative_yaw / 2.0)
        transform.transform.rotation.w = math.cos(relative_yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header = transform.header
        odometry.child_frame_id = transform.child_frame_id
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation = transform.transform.rotation
        odometry.twist = message.twist
        self.odom_publisher.publish(odometry)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationGroundTruthOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
