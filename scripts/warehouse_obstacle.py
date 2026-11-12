#!/usr/bin/env python3
"""Move a real collision obstacle in the current demo's Gazebo partition."""
import argparse
import os
import subprocess


def set_obstacle_pose(x, y, z):
    env = dict(os.environ)
    env.setdefault('GZ_PARTITION', 'navcraft-' + env.get('ROS_DOMAIN_ID', '73'))
    response = subprocess.run([
        'gz', 'service', '-s', '/world/indoor_room/set_pose',
        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req',
        f'name: "safety_obstacle_block", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}',
    ], env=env, text=True, capture_output=True, timeout=10)
    if response.returncode or 'data: true' not in response.stdout:
        raise RuntimeError('Gazebo obstacle move failed: ' + response.stdout + response.stderr)


def move_obstacle(block):
    # Fixed position ahead of home -> receiving; never teleport the robot.
    position = (-1.5, 0.0, 0.375) if block else (0.0, 0.0, -2.0)
    set_obstacle_pose(*position)
    print('Obstacle ' + ('placed at (-1.5, 0)' if block else 'cleared'), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['block', 'clear'])
    args = parser.parse_args()
    move_obstacle(args.action == 'block')


if __name__ == '__main__':
    main()
