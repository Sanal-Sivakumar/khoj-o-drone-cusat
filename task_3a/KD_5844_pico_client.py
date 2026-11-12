#!/usr/bin/env python3

'''
Khoj-o-Drone (KD) — Task 3A: Mission Action Client (pico_client)
Team ID: KD_5844

Runs the whole mission:
  CRITICAL  get the critical route ('waypoints'), start the 55 s clock as the first goal is
            sent (that is what makes the drone lift off), send the goals one by one.
  SWITCH    when the last critical Save completes OR the clock reaches 55 s, whichever is
            first: cancel the active goal, wait for the cancel to finish, send the drone's
            position from the last feedback to 'stable_route'.
  STABLE    send the stable route goal by goal. Its last goal is F6, where the server
            hovers for 1 s and lands.

Route points -> goals (marker_ids from the service say what each point is):
    survivor  -> Visit goal (Z = 13, hover) followed by Save goal (Z = 14, hover)
    "VIA"     -> one fly-past goal (Z = 13, hover = False)
    "F6"      -> one hover goal at Z = 13; the server lands after it
'''

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints, GetStableRoute


CRITICAL_WINDOW = 55.0     # seconds from take-off. KEEP AT 55 FOR THE SUBMISSION RUN.
VISIT_Z = 13.0
SAVE_Z = 14.0
VIA_NAME = 'VIA'
HOME_NAME = 'F6'
CANCEL_WAIT = 2.0          # give up waiting for a cancelled goal's result after this long

CRITICAL, SWITCHING, STABLE, DONE = 'CRITICAL', 'SWITCHING', 'STABLE', 'DONE'


def route_to_goals(response):
    '''Service response -> list of goal dicts in flight order.'''
    goals = []
    for i, pose in enumerate(response.waypoints.poses):
        name = response.marker_ids[i] if i < len(response.marker_ids) else VIA_NAME
        x, y = float(pose.position.x), float(pose.position.y)
        if name == VIA_NAME:
            goals.append(dict(name='via', xyz=(x, y, VISIT_Z), hover=False, save=False))
        elif name == HOME_NAME:
            goals.append(dict(name=f'{name} hover+land', xyz=(x, y, VISIT_Z), hover=True, save=False))
        else:
            goals.append(dict(name=f'{name} Visit', xyz=(x, y, VISIT_Z), hover=True, save=False))
            goals.append(dict(name=f'{name} Save', xyz=(x, y, SAVE_Z), hover=True, save=True))
    return goals


class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.RLock()

        self.phase = CRITICAL
        self.goals = []
        self.goal_index = 0
        self.send_id = 0              # identifies the goal currently in flight
        self.goal_handle = None       # handle of the active goal, once accepted
        self.cancel_pending = False   # Switch fired before the active goal was accepted
        self.cancel_started = None
        self.t0 = None                # mission clock (monotonic), set at the first goal
        self.switch_time = None
        self.last_position = None     # from the latest action feedback
        self.saves = {CRITICAL: 0, STABLE: 0}

        self.action_client = ActionClient(
            self, NavToWaypoint, 'waypoint_navigation', callback_group=self.callback_group)
        self.cli = self.create_client(GetWaypoints, 'waypoints', callback_group=self.callback_group)
        self.stable_cli = self.create_client(GetStableRoute, 'stable_route', callback_group=self.callback_group)

        self.get_logger().info('WayPointClient initialized. Waiting for the waypoint services...')
        while not (self.cli.wait_for_service(timeout_sec=1.0) and
                   self.stable_cli.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Services waypoints / stable_route not available yet, waiting...')

        self.clock_timer = None

    # ------------------------------------------------------------------ start-up
    def elapsed(self):
        return 0.0 if self.t0 is None else time.monotonic() - self.t0

    def receive_goals(self):
        self.get_logger().info('Requesting the critical route...')
        req = GetWaypoints.Request()
        req.get_waypoints = True
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response or not response.waypoints.poses:
            self.get_logger().error('Failed to receive the critical route from the service!')
            return False

        self.goals = route_to_goals(response)
        self.get_logger().info(f'Critical route: {list(response.marker_ids)} -> {len(self.goals)} goals')

        # The wait for the action server must NOT eat into the 55 s, so the clock starts after it.
        self.get_logger().info('Waiting for waypoint_navigation action server...')
        self.action_client.wait_for_server()

        with self.lock:
            # t = 0: the first goal is what makes the drone lift off, so starting the clock
            # here is never later than take-off.
            self.t0 = time.monotonic()
            self.clock_timer = self.create_timer(0.02, self.check_clock, callback_group=self.callback_group)
            self.get_logger().info(f'Action server ready. Clock started, critical window {CRITICAL_WINDOW:.0f} s.')
            self.send_goal()
        return True

    # ------------------------------------------------------------------ goals
    def send_goal(self):
        '''Sends self.goals[self.goal_index]. Call with the lock held.'''
        goal = self.goals[self.goal_index]
        self.send_id += 1
        send_id = self.send_id
        self.goal_handle = None

        msg = NavToWaypoint.Goal()
        msg.waypoint.position.x, msg.waypoint.position.y, msg.waypoint.position.z = goal['xyz']
        msg.hover = goal['hover']
        self.get_logger().info(
            f'[t={self.elapsed():5.1f}s {self.phase}] goal {self.goal_index + 1}/{len(self.goals)}: {goal["name"]} '
            f'[{goal["xyz"][0]:.2f}, {goal["xyz"][1]:.2f}, {goal["xyz"][2]:.2f}]')

        future = self.action_client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(lambda f, i=send_id: self.goal_response_callback(f, i))

    def goal_response_callback(self, future, send_id):
        handle = future.result()
        with self.lock:
            if send_id != self.send_id:
                return
            if not handle.accepted:
                self.get_logger().error('Goal rejected by the server!')
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(lambda f, i=send_id: self.get_result_callback(f, i))
            if self.cancel_pending:            # the Switch fired while this goal was being accepted
                self.cancel_pending = False
                handle.cancel_goal_async()

    def feedback_callback(self, feedback_msg):
        p = feedback_msg.feedback.current_waypoint.pose.position
        self.last_position = (float(p.x), float(p.y), float(p.z))
        hold = feedback_msg.feedback.current_waypoint.header.stamp.sec
        self.get_logger().info(
            f'[t={self.elapsed():5.1f}s] pos=[{p.x:5.2f}, {p.y:5.2f}, {p.z:5.2f}] hold={hold}s',
            throttle_duration_sec=1.0)

    def get_result_callback(self, future, send_id):
        status = future.result().status
        with self.lock:
            if send_id != self.send_id:
                return                              # result of a goal we no longer care about
            self.goal_handle = None

            if self.phase == SWITCHING:             # the cancel has finished (or the goal just ended)
                self.request_stable_route()
                return
            if self.phase == DONE:
                return
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f'Goal ended with status {status}; sending it again.')
                self.send_goal()
                return

            goal = self.goals[self.goal_index]
            if goal['save']:
                self.saves[self.phase] += 1
                self.get_logger().info(f'[t={self.elapsed():5.1f}s] SAVED {goal["name"].split()[0]}')
            self.goal_index += 1

            if self.goal_index < len(self.goals):
                self.send_goal()
            elif self.phase == CRITICAL:
                self.trigger_switch('all critical survivors saved')
            else:
                self.phase = DONE
                self.get_logger().info('=' * 60)
                self.get_logger().info(
                    f'Mission complete: landed at t = {self.elapsed():.1f} s | critical saved '
                    f'{self.saves[CRITICAL]}, stable saved {self.saves[STABLE]}')
                self.get_logger().info('=' * 60)

    # ------------------------------------------------------------------ the Switch
    def check_clock(self):
        with self.lock:
            if self.phase == CRITICAL and self.elapsed() >= CRITICAL_WINDOW:
                self.trigger_switch(f'{CRITICAL_WINDOW:.0f} s critical window is up')
            elif (self.phase == SWITCHING and self.cancel_started is not None and
                  time.monotonic() - self.cancel_started > CANCEL_WAIT):
                self.get_logger().warn('Cancel did not finish in time; carrying on.')
                self.send_id += 1                   # ignore whatever that goal still reports
                self.goal_handle = None
                self.request_stable_route()

    def trigger_switch(self, reason):
        '''Call with the lock held. Ends the critical phase exactly once.'''
        if self.phase != CRITICAL:
            return
        self.phase = SWITCHING
        self.switch_time = self.elapsed()
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'THE SWITCH at t = {self.switch_time:.2f} s ({reason}). Critical saved: {self.saves[CRITICAL]}')
        self.get_logger().info('=' * 60)

        goal_in_flight = self.goal_index < len(self.goals)
        if not goal_in_flight:
            self.request_stable_route()             # nothing to cancel
            return
        self.cancel_started = time.monotonic()
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling the active goal...')
            self.goal_handle.cancel_goal_async()    # get_result_callback continues once it is done
        else:
            self.cancel_pending = True              # goal sent but not yet accepted

    def request_stable_route(self):
        '''Call with the lock held, once no goal is active.'''
        self.cancel_started = None
        self.phase = STABLE
        req = GetStableRoute.Request()
        if self.last_position is not None:
            req.position.x, req.position.y, req.position.z = self.last_position
        elif self.goals:                            # no feedback at all: best guess is the last target
            req.position.x, req.position.y, req.position.z = self.goals[min(self.goal_index, len(self.goals) - 1)]['xyz']
        self.get_logger().info(
            f'Requesting the stable route from [{req.position.x:.2f}, {req.position.y:.2f}, {req.position.z:.2f}]')
        self.stable_cli.call_async(req).add_done_callback(self.stable_route_callback)

    def stable_route_callback(self, future):
        response = future.result()
        with self.lock:
            if not response or not response.waypoints.poses:
                self.get_logger().error('Stable route service returned nothing!')
                return
            self.goals = route_to_goals(response)
            self.goal_index = 0
            self.get_logger().info(f'Stable route: {list(response.marker_ids)} -> {len(self.goals)} goals')
            self.send_goal()


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
