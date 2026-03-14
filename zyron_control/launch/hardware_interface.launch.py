import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

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
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            os.path.join(
                get_package_share_directory("zyron_control"),
                "config",
                "zyron_controllers.yaml",
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    zyron_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zyron_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        zyron_controller_spawner,
    ])
