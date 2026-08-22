#!/usr/bin/env python3

import math

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
            self.initial_pose = (pose.position.x, pose.position.y, yaw)
            self.get_logger().info("Ground-truth odometry origin initialized")

        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if (
            self.last_publish_time is not None
            and stamp_ns - self.last_publish_time < self.publish_period_ns
        ):
            return
        self.last_publish_time = stamp_ns

        initial_x, initial_y, initial_yaw = self.initial_pose
        world_dx = pose.position.x - initial_x
        world_dy = pose.position.y - initial_y
        cos_yaw = math.cos(initial_yaw)
        sin_yaw = math.sin(initial_yaw)
        x = cos_yaw * world_dx + sin_yaw * world_dy
        y = -sin_yaw * world_dx + cos_yaw * world_dy
        relative_yaw = yaw - initial_yaw

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "chassis"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(relative_yaw / 2.0)
        transform.transform.rotation.w = math.cos(relative_yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header = transform.header
        odometry.child_frame_id = "gimbal_yaw"
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
