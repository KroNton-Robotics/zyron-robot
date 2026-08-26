from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Paths to configuration files
    planner_yaml = os.path.join(get_package_share_directory('zyron_navigation'), 'config', 'zyron_planner_server.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('zyron_navigation'), 'config', 'zyron_bt_navigator.yaml')
    controller_yaml = os.path.join(get_package_share_directory('zyron_navigation'), 'config', 'zyron_controller.yaml')
    bt_xml_path = os.path.join(  get_package_share_directory('zyron_navigation'), 
                               'behavior_trees', 'nav_to_pose_w_replanning_and_recovery.xml')
    use_sim_time=True

    # Planner Server Node
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml,{"use_sim_time": use_sim_time},]
        
        )
    
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml,{"use_sim_time": use_sim_time}],
        remappings=[('/odom','/odom'),('/cmd_vel','/cmd_vel')])  
      
    bt_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml,
            {"use_sim_time": use_sim_time},
            {'default_nav_to_pose_bt_xml': bt_xml_path},
            # {'default_nav_through_poses_bt_xml': bt_xml_path}
            ]
        )
    
    # Behavior Server (Needed for the <Wait> action)
    behaviors_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Lifecycle Manager Node
    lifecycle_mange_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'autostart': True},{"use_sim_time": use_sim_time},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'bt_navigator',
                                    'behavior_server'
                                    ]}])
    


    return LaunchDescription([   
        planner_node,
        controller_node,
        bt_node,
        behaviors_node,
        lifecycle_mange_node,
    ])