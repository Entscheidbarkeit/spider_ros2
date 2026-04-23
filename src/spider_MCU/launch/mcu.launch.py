from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="/home/atvpi/transformer_ros2_workspace/src/spider_MCU/config/mcu.yaml",
        description="Path to the spider_MCU parameter file",
    )

    spider_mcu_node = Node(
        package="spider_MCU",
        executable="spider_MCU_node",
        name="spider_mcu_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        spider_mcu_node,
    ])