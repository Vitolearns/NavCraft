"""Select exactly one simulation control chain: navigation, slam, or tracking."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, filename, args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare(package), 'launch', filename])), launch_arguments=args.items())


def start(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode == 'navigation':
        return [include('navcraft_bringup', 'navigation.launch.py', {
            'gui': LaunchConfiguration('gui'), 'use_rviz': LaunchConfiguration('use_rviz')})]
    if mode not in ('slam', 'tracking'):
        raise ValueError('mode must be navigation, slam, or tracking')
    actions = [include('navcraft_simulation', 'sim.launch.py', {'gui': LaunchConfiguration('gui')})]
    if mode == 'tracking':
        controller = LaunchConfiguration('controller').perform(context)
        if controller not in ('pure_pursuit', 'stanley'):
            raise ValueError('controller must be pure_pursuit or stanley')
        actions += [include('navcraft_bringup', 'tracking.launch.py', {
            'controller': controller, 'use_mock_chassis': 'false', 'use_sim_time': 'true',
            'output_csv': LaunchConfiguration('output_csv'),
            'output_json': LaunchConfiguration('output_json')})]
    else:
        actions += [include('navcraft_teleop', 'cmd_vel_stack.launch.py', {
            'use_sim_time': 'true', 'default_source': 'teleop'}),
            include('navcraft_navigation', 'slam.launch.py', {
                'use_sim_time': 'true', 'slam_mode': LaunchConfiguration('slam_mode')})]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='navigation'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('slam_mode', default_value='slam_toolbox'),
        DeclareLaunchArgument('controller', default_value='pure_pursuit'),
        DeclareLaunchArgument('output_csv', default_value='/tmp/navcraft_tracking.csv'),
        DeclareLaunchArgument('output_json', default_value='/tmp/navcraft_tracking.json'),
        OpaqueFunction(function=start),
    ])
