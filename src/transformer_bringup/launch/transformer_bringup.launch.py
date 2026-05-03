"""Transformer bringup launch.

Starts core nodes and gates optional peripherals behind launch arguments.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate the full bringup LaunchDescription with optional sensors."""
    servos_pkg = Path(get_package_share_directory('transformer_hw_servos'))
    actuators_pkg = Path(get_package_share_directory('transformer_hw_actuators'))
    controller_pkg = Path(get_package_share_directory('transformer_controller'))
    mcu_pkg = Path(get_package_share_directory('spider_mcu'))

    servos_launch = servos_pkg / 'launch' / 'servos.launch.py'
    actuators_launch = actuators_pkg / 'launch' / 'actuator.launch.py'
    controller_launch = controller_pkg / 'launch' / 'transformer_controller.launch.py'
    mcu_launch = mcu_pkg / 'launch' / 'mcu.launch.py'
    # Using direct node instantiation instead of rs_launch to ensure params YAML is applied at configure time
    # realsense_launch = realsense_pkg / 'launch' / 'rs_launch.py'

    """ Optional device readiness gate to improve boot reliability on SBCs """
    wait_for_devices_arg = DeclareLaunchArgument(
        'wait_for_devices', default_value='true',
        description='Block launch until core device files (gpiochip, pwmchip, serial) appear or timeout')
    device_wait_timeout_arg = DeclareLaunchArgument(
        'device_wait_timeout', default_value='20',
        description='Max seconds to wait for device files before proceeding')
    wait_for_devices = LaunchConfiguration('wait_for_devices')
    device_wait_timeout = LaunchConfiguration('device_wait_timeout')

    def _wait_for_paths(context):
        import os
        import time
        # Resolve launch-time values
        should_wait = str(wait_for_devices.perform(context)).lower() == 'true'
        timeout_s = float(device_wait_timeout.perform(context) or '20')
        serial_port = LaunchConfiguration('wheeltec_serial_port').perform(context)
        # Known device paths based on default configs
        required = [
            '/dev/gpiochip4',            # default gpiochip for direction lines
            '/sys/class/pwm/pwmchip0',   # default pwmchip
        ]
        # Serial is optional unless IMU is enabled; still safe to wait briefly
        if serial_port:
            required.append(serial_port)

        if not should_wait:
            print('[bringup] Device wait disabled; continuing without probing')
            return []

        deadline = time.time() + max(0.0, timeout_s)
        pending = set(required)
        last_report = 0.0
        while pending and time.time() < deadline:
            found = {p for p in pending if os.path.exists(p)}
            pending -= found
            now = time.time()
            if now - last_report > 2.0:
                print('[bringup] Waiting for devices: {} (timeout {}s)'.format(sorted(pending), int(deadline-now)))
                last_report = now
            time.sleep(0.25)

        if pending:
            print('[bringup] Proceeding despite missing devices: {}'.format(sorted(pending)))
        else:
            print('[bringup] All required devices are present')
        return []

    device_probe = OpaqueFunction(function=_wait_for_paths)

    """ Servos """
    servos_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(servos_launch))
    )

    """ Actuators """
    actuators_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(actuators_launch))
    )

    """ MCU """
    mcu_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(mcu_launch))
    ) 

    """ Transformer Controller (delayed start) """
    controller_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(controller_launch))
    )


    controller_start_delay_arg = DeclareLaunchArgument(
        'controller_start_delay',
        # Increase default delay to allow PWM/GPIO exports and backend initialisation on slower SBCs
        default_value='6.0',
        description='Seconds to wait before starting the transformer controller so hardware nodes are ready')
    controller_start_delay = LaunchConfiguration('controller_start_delay')
    controller_start = TimerAction(period=controller_start_delay, actions=[controller_include])

    """ MSP Bridge """
    start_msp_arg = DeclareLaunchArgument(
        'start_msp', default_value='true', description='Start the MSP bridge node')
    start_msp = LaunchConfiguration('start_msp')
    msp_pkg_share = Path(get_package_share_directory('transformer_msp_bridge'))
    msp_params = [str(msp_pkg_share / 'config' / 'msp_bridge.yaml')]
    msp_node = Node(
        package='transformer_msp_bridge',
        executable='transformer_msp_bridge_node',
        name='transformer_msp_bridge',
        output='screen',
        parameters=msp_params,
        condition=IfCondition(start_msp)
    )

    """ RC Mode Switch """
    start_rc_switch_arg = DeclareLaunchArgument(
        'start_rc_switch', default_value='true', description='Start the RC mode switch node')
    start_rc_switch = LaunchConfiguration('start_rc_switch')
    rc_switch_pkg = Path(get_package_share_directory('transformer_rc_mode_switch'))
    rc_switch_launch = rc_switch_pkg / 'launch' / 'rc_mode_switch.launch.py'
    rc_switch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(rc_switch_launch)),
        condition=IfCondition(start_rc_switch))
    rc_switch_start_delay_arg = DeclareLaunchArgument(
        'rc_switch_start_delay',
        # Give controller extra time to finish initialization before RC client starts
        default_value='8.0',
        description='Seconds to wait before starting the RC mode switch so the controller is available')
    rc_switch_start_delay = LaunchConfiguration('rc_switch_start_delay')
    rc_switch_start = TimerAction(period=rc_switch_start_delay, actions=[rc_switch_include])

    """ RealSense D4xx Camera """
    start_realsense_arg = DeclareLaunchArgument(
        'start_realsense', default_value='false', description='Start Intel RealSense D4xx camera node')
    start_realsense = LaunchConfiguration('start_realsense')
    bringup_pkg = Path(get_package_share_directory('transformer_bringup'))
    realsense_config = bringup_pkg / 'config' / 'realsense.yaml'
    camera_namespace_arg = DeclareLaunchArgument(
        'camera_namespace', default_value='transformer', description='Namespace for the RealSense camera')
    camera_name_arg = DeclareLaunchArgument(
        'camera_name', default_value='D435I', description='Logical camera name')
    camera_namespace = LaunchConfiguration('camera_namespace')
    camera_name = LaunchConfiguration('camera_name')
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=camera_name,
        namespace=camera_namespace,
        output='screen',
        parameters=[str(realsense_config)],
        # Suppress noisy WARNs about stream re-enable at startup for the camera node
        arguments=['--ros-args', '--log-level', 'transformer.D435I:=error'],
        condition=IfCondition(start_realsense),
    )
    # Use native camera_info topics published by the driver; avoid relays to prevent QoS/timestamp mismatches

    """ Wheeltec N100 IMU """
    start_wheeltec_imu_arg = DeclareLaunchArgument(
        'start_wheeltec_imu', default_value='false', description='Start Wheeltec N100 IMU node')
    start_wheeltec_imu = LaunchConfiguration('start_wheeltec_imu')
    wheeltec_serial_port_arg = DeclareLaunchArgument(
        'wheeltec_serial_port', default_value='/dev/ttyUSB0',
        description='Serial port for Wheeltec N100 IMU')
    wheeltec_serial_baud_arg = DeclareLaunchArgument(
        'wheeltec_serial_baud', default_value='921600',
        description='Baud rate for Wheeltec N100 IMU')
    wheeltec_serial_port = LaunchConfiguration('wheeltec_serial_port')
    wheeltec_serial_baud = LaunchConfiguration('wheeltec_serial_baud')
    wheeltec_imu_node = Node(
        package='wheeltec_n100_imu',
        executable='imu_node',
        name='wheeltec_n100_imu',
        output='screen',
        parameters=[{
            'serial_port': wheeltec_serial_port,
                'serial_baud': wheeltec_serial_baud,
        }],
        condition=IfCondition(start_wheeltec_imu)
    )

    """ Robot Description Publisher """
    start_description_arg = DeclareLaunchArgument(
        'start_description', default_value='true', description='Start robot_state_publisher with transformer_description URDF')
    start_description = LaunchConfiguration('start_description')
    desc_share = Path(get_package_share_directory('transformer_description'))
    xacro_file = desc_share / 'urdf' / 'transformer.urdf.xacro'
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': ParameterValue(Command(['xacro ', str(xacro_file)]), value_type=str),
        }],
        condition=IfCondition(start_description)
    )

    return LaunchDescription([
        wait_for_devices_arg,
        device_wait_timeout_arg,
        start_msp_arg,
        start_rc_switch_arg,
        start_realsense_arg,
        start_description_arg,
        controller_start_delay_arg,
        rc_switch_start_delay_arg,
        camera_namespace_arg,
        camera_name_arg,
        start_wheeltec_imu_arg,
        wheeltec_serial_port_arg,
        wheeltec_serial_baud_arg,
        device_probe,
        servos_include,
        actuators_include,
        mcu_include,
        controller_start,
        msp_node,
        rc_switch_start,
        robot_state_publisher,
        realsense_node,
        wheeltec_imu_node,
    ])
