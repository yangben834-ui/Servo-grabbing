from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("bottle_grasp_perception")
    default_config = PathJoinSubstitution(
        [package_share, "config", "grasp_pose.yaml"]
    )
    default_model = PathJoinSubstitution(
        [package_share, "models", "best_train_multi"]
    )
    arguments = [
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("model_path", default_value=default_model),
        DeclareLaunchArgument("model_input_size", default_value="640"),
        DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/depth/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/camera/depth/camera_info"
        ),
    ]
    node = Node(
        package="bottle_grasp_perception",
        executable="grasp_pose_node",
        name="bottle_grasp_perception",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {
                "model_path": LaunchConfiguration("model_path"),
                "model_input_size": LaunchConfiguration("model_input_size"),
                "color_topic": LaunchConfiguration("color_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            },
        ],
    )
    return LaunchDescription(arguments + [node])
