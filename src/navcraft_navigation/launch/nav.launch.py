from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('navcraft_navigation')
    params = PathJoinSubstitution([share, 'config', 'nav2_basic.yaml'])
    footprint = PathJoinSubstitution([share, 'config', 'robot_footprint.yaml'])
    sim_time = LaunchConfiguration('use_sim_time')
    nodes = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py'])),
            launch_arguments={
                'map': PathJoinSubstitution([share, 'maps', 'indoor_room.yaml']),
                'params_file': params, 'use_sim_time': sim_time,
                'use_composition': 'False', 'autostart': 'true',
            }.items()),
    ]
    for package, executable in [('nav2_controller', nodes[0]), ('nav2_planner', nodes[1]),
                                ('nav2_behaviors', nodes[2]), ('nav2_bt_navigator', nodes[3])]:
        actions.append(Node(package=package, executable=executable, name=executable,
                            parameters=[params, footprint, {'use_sim_time': sim_time}],
                            remappings=[('cmd_vel', '/nav2_cmd_vel')], output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_manager_navigation', parameters=[{
                            'use_sim_time': sim_time, 'autostart': True, 'node_names': nodes}], output='screen'))
    return LaunchDescription(actions)
