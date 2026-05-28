import os
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription

def generate_launch_description():
    robot_control_pkg = get_package_share_directory('zyron_control')
    robot_firmware_pkg = get_package_share_directory('zyron_firmware')

    zyron_firmware_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_firmware_pkg, 'launch', 'zyron_firmware.launch.py')))

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
        remappings=[('/zyron_controller/cmd_vel','/zyron_controller/cmd_vel_stamped')],
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

    twist_relay_node = Node(
        package='zyron_control',
        executable='twist_to_twist_stamped.py',
        name='twist_to_twist_stamped',
        output='screen',
    )

    local_localization_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_control_pkg, 'launch', 'local_localization_ekf.launch.py')))


    return LaunchDescription([
        zyron_firmware_driver,
        robot_state_publisher,
        controller_manager,
        # joint_state_broadcaster_spawner,
        zyron_controller_spawner,
        local_localization_ekf,
        twist_relay_node
    ])
