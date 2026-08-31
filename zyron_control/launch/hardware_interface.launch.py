import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_control_pkg = get_package_share_directory("zyron_control")
    serial_port = LaunchConfiguration("serial_port")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("zyron_description"),
                    "urdf",
                    "zyron.urdf.xacro",
                ),
                " is_sim:=False",
                " serial_port:=",
                serial_port,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        remappings=[
            ("/zyron_controller/cmd_vel", "/zyron_controller/cmd_vel_stamped"),
            ("/zyron_imu_broadcaster/imu", "/zyron/imu"),
        ],
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            os.path.join(robot_control_pkg, "config", "zyron_controllers.yaml"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zyron_imu_broadcaster", "--controller-manager", "/controller_manager"],
    )

    zyron_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zyron_controller", "--controller-manager", "/controller_manager"],
    )

    twist_relay_node = Node(
        package="zyron_control",
        executable="twist_to_twist_stamped.py",
        name="twist_to_twist_stamped",
        output="screen",
    )

    local_localization_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_control_pkg, "launch", "local_localization_ekf.launch.py")
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/mcu",
                description="Serial port connected to the Zyron ESP32",
            ),
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            imu_broadcaster_spawner,
            zyron_controller_spawner,
            local_localization_ekf,
            twist_relay_node,
        ]
    )
