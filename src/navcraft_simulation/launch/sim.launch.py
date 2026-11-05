import os
import tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    sim = get_package_share_directory('navcraft_simulation')
    description = get_package_share_directory('navcraft_description')
    geometry = os.path.join(description, 'config', 'robot_geometry.yaml')
    robot = xacro.process_file(os.path.join(description, 'urdf', 'robot.urdf.xacro'),
                               mappings={'geometry_file': geometry}).toxml()
    world = xacro.process_file(os.path.join(sim, 'worlds', 'indoor_room.sdf.xacro'), mappings={
        'geometry_file': geometry,
        'simulation_physics_file': os.path.join(sim, 'config', 'simulation_physics.yaml'),
        'simulation_media_directory': os.path.join(sim, 'media'),
    }).toxml()
    fd, path = tempfile.mkstemp(prefix='robot_single_', suffix='.sdf')
    with os.fdopen(fd, 'w') as stream:
        stream.write(world)

    def cleanup(_):
        if os.path.exists(path):
            os.unlink(path)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        ExecuteProcess(cmd=['gz', 'sim', '-r', path], condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', path], condition=UnlessCondition(LaunchConfiguration('gui')), output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/mobile_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/sim/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/sim/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/sim/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}]),
        Node(package='navcraft_sensors', executable='imu_filter_node',
             parameters=[{'input_topic': '/sim/imu', 'output_topic': '/imu/data', 'frame_id': 'imu_link', 'use_sim_time': True}]),
        Node(package='navcraft_sensors', executable='laser_filter_node',
             parameters=[{'input_topic': '/sim/scan', 'output_topic': '/scan', 'use_sim_time': True}]),
        Node(package='navcraft_simulation', executable='gazebo_odom_tf_node', parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('gui', default_value='true'), OpaqueFunction(function=start)])
