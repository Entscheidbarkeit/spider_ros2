from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="/home/atvpi/transformer_ros2_workspace/src/spider_mcu/config/mcu.yaml",
        description="Path to the spider_mcu parameter file",
    )

    spider_mcu_node = Node(
        package="spider_mcu",
        executable="spider_mcu_node",
        name="spider_mcu_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        spider_mcu_node,
    ])