import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():

    # livox_ros_driver2
    start_livox_ros_driver2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'launch', 'msg_MID360_launch.py')
        )
    )
    delayed_star_livox_ros_driver2 = TimerAction(
        period=1.0,
        actions=[start_livox_ros_driver2]
    )

    # fast_lio
    start_fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
        )
    )
    delayed_start_fast_lio = TimerAction(
        period=4.0,
        actions=[start_fast_lio]
    )

    # local_planner
    start_local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vehicle_simulator'), 'launch', 'system_real_robot.launch.py')
        )
    )
    delayed_start_local_planner = TimerAction(
        period=6.0,
        actions=[start_local_planner]
    )

    # global_planner
    start_map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mapping'), 'launch', 'bringup_launch.py')
        )
    )
    a_star_node = Node(
        package='a_star',
        executable='a_star_node',
        name='a_star',
        output='both',
    )
    path2waypoint_node = Node(
        package='path2waypoint',
        executable='path2waypoint',
        name='path2waypoint',
        output='both',
    )
    delayed_global_planner = TimerAction(
        period=1.0,
        actions=[a_star_node, path2waypoint_node]
    )
    delayed_start_map_server = TimerAction(
        period=10.0,
        actions=[start_map_server]
    )

    # rviz2
    rviz_config_file = os.path.join(get_package_share_directory('rm_nav_bringup'), 'rviz', 'rm_nav.rviz')
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    delayed_start_rviz = TimerAction(
        period=12.0,
        actions=[start_rviz]
    )

    # static_tf
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        output='screen'
    )
    delayed_static_tf = TimerAction(
        period=1.0,
        actions=[static_tf]
    )


    ld = LaunchDescription()

    # Add the actions to the LaunchDescription
    ld.add_action(delayed_global_planner)
    ld.add_action(delayed_star_livox_ros_driver2)
    ld.add_action(delayed_start_fast_lio)
    ld.add_action(delayed_start_local_planner)
    ld.add_action(delayed_start_map_server)
    ld.add_action(delayed_static_tf)
    ld.add_action(delayed_start_rviz)

    return ld
