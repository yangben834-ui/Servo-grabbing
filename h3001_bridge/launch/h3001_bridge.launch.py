"""启动 h3001_bridge 桥接节点 (H3001 CAN ↔ ROS 2)。

用法:
  # 默认参数
  ros2 launch h3001_bridge h3001_bridge.launch.py

  # 临时调低运动速度 (首次调试建议)
  ros2 launch h3001_bridge h3001_bridge.launch.py speed_percent:=5

  # 只读反馈、不使能电机
  ros2 launch h3001_bridge h3001_bridge.launch.py auto_enable:=false

前置条件:
  1. 厂商 web server (h3001_web_server_linux.py) 必须先停掉, 否则两者会
     抢同一条 can0 总线: pkill -f h3001_web_server_linux.py
  2. CAN 适配器已配置好: sudo ip link set can0 up type can bitrate 1000000
  3. SDK 能被导入: 节点 import 的是 sdk_linux (2026-09-16 起, 旧版是 h3001_sdk)。
     若报 "无法导入 sdk_linux", 用环境变量指到 SDK 目录, 例如
     export H3001_SDK_PATH=/home/qin/zdl_fz/src/SDK_linux
     该变量的优先级高于所有硬编码候选路径。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("h3001_bridge"),
        "config",
        "h3001_bridge.yaml",
    )

    speed_percent_arg = DeclareLaunchArgument(
        "speed_percent",
        default_value="10",
        description=(
            "MCU 运动速度百分比 (0~100)。**本 launch 的默认值会覆盖 "
            "config/h3001_bridge.yaml 里的同名参数** (launch 的参数字典排在 yaml "
            "之后), 所以只改 yaml 是没用的, 要改就改这里或从命令行给。"
            "2026-09-16 由 1 改成 10: 1 比本文件最初注释建议的 5~10 还低一个数量级, "
            "下降时每段 1cm 要走很久, 到位判定反而更容易误判。"
        ),
    )
    auto_enable_arg = DeclareLaunchArgument(
        "auto_enable",
        default_value="true",
        description="true: 启动时使能六轴电机; false: 只读反馈",
    )
    can_channel_arg = DeclareLaunchArgument(
        "can_channel",
        default_value="can0",
        description="SocketCAN 接口名",
    )
    invert_rx_ry_arg = DeclareLaunchArgument(
        "invert_rx_ry",
        default_value="false",
        description=(
            "固件 RPY 约定修正: 对 rx/ry 取反 (临时方案, 当前关闭)。"
            "注意本 launch 的默认值会覆盖 yaml, 改默认值必须改这里"
        ),
    )

    bridge_node = Node(
        package="h3001_bridge",
        executable="h3001_bridge",
        name="h3001_bridge",
        output="screen",
        parameters=[
            config,
            {
                # 显式声明类型, 否则 LaunchConfiguration 会以字符串下发,
                # 与节点里 declare_parameter 推断出的 int/bool 类型冲突。
                "speed_percent": ParameterValue(
                    LaunchConfiguration("speed_percent"), value_type=int
                ),
                "auto_enable": ParameterValue(
                    LaunchConfiguration("auto_enable"), value_type=bool
                ),
                "can_channel": ParameterValue(
                    LaunchConfiguration("can_channel"), value_type=str
                ),
                "invert_rx_ry": ParameterValue(
                    LaunchConfiguration("invert_rx_ry"), value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            speed_percent_arg,
            auto_enable_arg,
            can_channel_arg,
            invert_rx_ry_arg,
            bridge_node,
        ]
    )
