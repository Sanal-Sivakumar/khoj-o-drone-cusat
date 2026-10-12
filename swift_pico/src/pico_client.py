#!/usr/bin/env python3

'''
Khoj-o-Drone (KD) — Task 2B: Waypoint Navigation Action Client
Team ID: KD_5844

Fetches waypoints from 'waypoints' service (GetWaypoints.srv),
and sequentially sends them as goals to 'waypoint_navigation' action server (NavToWaypoint.action).
Ensures non-blocking asynchronous execution and full 4-waypoint completion.
'''

import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints


class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')
        self.callback_group = ReentrantCallbackGroup()

        self.goals = []
        self.goal_index = 0
        self.mission_complete = False

        # Action client for 'waypoint_navigation'
        self.action_client = ActionClient(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            callback_group=self.callback_group
        )

        # Service client for 'waypoints'
        self.cli = self.create_client(
            GetWaypoints,
            'waypoints',
            callback_group=self.callback_group
        )

        self.get_logger().info('WayPointClient initialized. Waiting for waypoints service...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service waypoints not available yet, waiting...')

        # Request waypoints from service
        self.req = GetWaypoints.Request()
        self.req.get_waypoints = True

    def send_request(self):
        return self.cli.call_async(self.req)

    def receive_goals(self):
        self.get_logger().info('Requesting waypoints from service...')
        future = self.send_request()
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if not response or not response.waypoints.poses:
            self.get_logger().error('Failed to receive waypoints from service!')
            return False

        self.get_logger().info(f'Received {len(response.waypoints.poses)} waypoints from service.')
        for i, pose in enumerate(response.waypoints.poses):
            wp = [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
            self.goals.append(wp)
            name = response.marker_ids[i] if i < len(response.marker_ids) else f'wp{i+1}'
            self.get_logger().info(f'  {name}: [{wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}]')

        # Wait for action server before sending the first goal
        self.get_logger().info('Waiting for waypoint_navigation action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server ready! Starting mission at Waypoint 1...')

        # Dispatch the first goal
        self.send_goal(self.goals[0])
        return True

    def send_goal(self, waypoint):
        goal_msg = NavToWaypoint.Goal()
        goal_msg.waypoint.position.x = float(waypoint[0])
        goal_msg.waypoint.position.y = float(waypoint[1])
        goal_msg.waypoint.position.z = float(waypoint[2])
        goal_msg.hover = True

        self.get_logger().info(f'>>> Dispatching Goal {self.goal_index + 1}/{len(self.goals)}: [{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}]')

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal {self.goal_index + 1} was rejected by server!')
            return

        self.get_logger().info(f'Goal {self.goal_index + 1} accepted by server. Navigating...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        settle_time = result.hov_time
        self.get_logger().info(f'Goal {self.goal_index + 1} completed! Result settle time: {settle_time}s')

        self.goal_index += 1

        if self.goal_index < len(self.goals):
            # Send next goal without blocking
            self.send_goal(self.goals[self.goal_index])
        else:
            self.mission_complete = True
            self.get_logger().info('='*60)
            self.get_logger().info('All waypoints have been reached successfully')
            self.get_logger().info('='*60)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        x = fb.current_waypoint.pose.position.x
        y = fb.current_waypoint.pose.position.y
        z = fb.current_waypoint.pose.position.z
        t = fb.current_waypoint.header.stamp.sec
        self.get_logger().info(f'Feedback: Pos=[{x:5.2f}, {y:5.2f}, {z:5.2f}] | Hold={t}s', throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WayPointClient()
    success = waypoint_client.receive_goals()

    if not success:
        waypoint_client.destroy_node()
        rclpy.shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(waypoint_client)

    try:
        executor.spin()
    except KeyboardInterrupt:
        waypoint_client.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
