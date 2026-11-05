from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    clock = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_clear', default_value='false'),
        Node(package='navcraft_diagnostics', executable='system_monitor_node', parameters=[clock]),
        Node(package='navcraft_diagnostics', executable='fault_supervisor_node',
             parameters=[clock, {'auto_clear': LaunchConfiguration('auto_clear')}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('navcraft_diagnostics'), 'launch', 'diagnostics.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
    ])
