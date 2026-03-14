from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    use_sim_time_env = os.getenv('USE_SIM_TIME', 'false').lower() == 'true'

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("zyron_control"), "config", "local_ekf.yaml"),
                    {'use_sim_time': False}],
    )

    return LaunchDescription([
        robot_localization,
    ])