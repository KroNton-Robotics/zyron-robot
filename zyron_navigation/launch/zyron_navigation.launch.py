from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    #call other launches inside the main launch
    zyron_amcl_launch = os.path.join(get_package_share_directory('zyron_localization'),'launch','zyron_localization.launch.py')
    zyron_nav2_launch = os.path.join(get_package_share_directory('zyron_navigation'),'launch','zyron_nav2.launch.py')
    
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            zyron_amcl_launch
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            zyron_nav2_launch
        )
    )


    return LaunchDescription([   
        amcl_launch, 
        nav2_launch
    ])