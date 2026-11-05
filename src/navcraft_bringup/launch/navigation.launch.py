from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def include(package, filename, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])), launch_arguments=args.items())
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        include('navcraft_simulation', 'sim.launch.py', {'gui': LaunchConfiguration('gui')}),
        include('navcraft_navigation', 'nav.launch.py', {'use_sim_time': 'true'}),
        include('navcraft_navigation', 'map_manager.launch.py', {}),
        Node(package='navcraft_mission', executable='navigate_sequence_server_node',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='navcraft_safety', executable='cmd_vel_safety_gate_node', output='screen'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', PathJoinSubstitution([FindPackageShare('navcraft_simulation'), 'rviz', 'navigation.rviz'])],
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
