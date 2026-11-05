#!/usr/bin/env python3
"""Run smoke or warehouse waypoints. Use demo.sh cancel to stop."""
import argparse
import rclpy
from warehouse_route import ROUTES
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from navcraft_interfaces.action import NavigateSequence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', choices=ROUTES, default='smoke')
    args = parser.parse_args()
    rclpy.init()
    node = Node('sequence_demo_client')
    client = ActionClient(node, NavigateSequence, '/navigate_sequence')
    if not client.wait_for_server(timeout_sec=20):
        raise RuntimeError('Sequence executor not available')
    goal = NavigateSequence.Goal()
    for x, y in ROUTES[args.route]:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.w = 1.0
        goal.goals.append(p)
    sent = client.send_goal_async(goal, feedback_callback=lambda m: node.get_logger().info(
        f'{m.feedback.state}: waypoint {m.feedback.current_goal_index}/{m.feedback.total_goals}', throttle_duration_sec=1))
    rclpy.spin_until_future_complete(node, sent)
    handle = sent.result()
    if not handle.accepted:
        raise RuntimeError('Goal rejected: busy, invalid goal, or FAULT')
    done = handle.get_result_async()
    rclpy.spin_until_future_complete(node, done)
    result = done.result()
    print(result)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result.result.success else 1


if __name__ == '__main__':
    raise SystemExit(main())
