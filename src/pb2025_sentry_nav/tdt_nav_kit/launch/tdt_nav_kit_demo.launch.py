from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("map_topic", default_value="map"),
        DeclareLaunchArgument("start_topic", default_value="initialpose"),
        DeclareLaunchArgument("goal_topic", default_value="goal_pose"),
        Node(
            package="tdt_nav_kit",
            executable="tdt_nav_kit_demo",
            namespace=namespace,
            output="screen",
            parameters=[{
                "map_topic": LaunchConfiguration("map_topic"),
                "start_topic": LaunchConfiguration("start_topic"),
                "goal_topic": LaunchConfiguration("goal_topic"),
            }],
        ),
    ])
