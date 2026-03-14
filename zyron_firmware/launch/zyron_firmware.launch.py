import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():



    serial_driver = LifecycleNode(
        package="zyron_firmware",
        executable="zyron_serial_driver",
        name="zyron_serial_driver",
        namespace="",
        parameters=[{os.path.join(
                get_package_share_directory("zyron_firmware"),
                "config",
                "zyron_driver_config.yaml",
            )

        }],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_serial",
        parameters=[
            {"autostart": True},
            {"node_names": ["zyron_serial_driver"]},
        ],
    )


    return LaunchDescription([
        serial_driver,
        lifecycle_manager,
    ])
