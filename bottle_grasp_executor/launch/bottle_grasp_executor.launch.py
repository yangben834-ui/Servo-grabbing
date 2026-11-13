"""启动 bottle_grasp_executor 抓取执行节点。

用法:
  # dry_run (默认): 只打印目标位姿, 不驱动机械臂
  ros2 launch bottle_grasp_executor bottle_grasp_executor.launch.py

  # 真机执行 (需 h3001_bridge 与感知节点已在运行)
  ros2 launch bottle_grasp_executor bottle_grasp_executor.launch.py dry_run:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("bottle_grasp_executor"),
        "config",
        "bottle_grasp_executor.yaml",
    )

    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="true",
        description="true: 只打印目标不发送机械臂命令; false: 真机执行",
    )

    executor_node = Node(
        package="bottle_grasp_executor",
        executable="bottle_grasp_executor",
        name="bottle_grasp_executor",
        output="screen",
        parameters=[config, {"dry_run": LaunchConfiguration("dry_run")}],
    )

    return LaunchDescription([dry_run_arg, executor_node])
