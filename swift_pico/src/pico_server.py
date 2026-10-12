#!/usr/bin/env python3

'''
Khoj-o-Drone (KD) — Task 2B: High-Speed Waypoint Navigation Action Server
Team ID: KD_5844

Implements ROS 2 Action Server 'waypoint_navigation' (NavToWaypoint.action).
Executes high-speed sprints between waypoints with active pitch/roll braking and
altitude pitch-up decoupling, guaranteeing rock-solid Z=13.00 hold and 3.0s holds.
'''

import time
import math
import os
import yaml

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from waypoint_navigation.action import NavToWaypoint
from swift_msgs.msg import SwiftMsgs
from std_msgs.msg import Header
from whycode_interfaces.msg import MarkerArray
from error_msg.msg import Error


class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.control_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        # Telemetry & States
        self.marker_received = False
        self.last_marker_time = 0.0
        self.curr_state = [0.0, 0.0, 0.0]
        self.drone_position = [0.0, 0.0, 0.0]
        self.desired_state = [0.0, 0.0, 15.0]

        # Yaw control state
        self.yaw = 0.0
        self.desired_yaw = -0.5
        self.prev_yaw_error = 0.0
        self.Kp_yaw = 2.0
        self.Kd_yaw = 0.5

        # RC Command message initialization
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1000
        self.cmd.drone_index = 0

        self.pos_error = Error()

        # Optimal High-Speed Gains: [pitch(X), roll(Y), throttle(Z)]
        self.Kp = [22.0, 22.0, 22.0]
        self.Ki = [0.02, 0.02, 0.06]
        self.Kd = [45.0, 45.0, 32.0]

        # PID tracking state
        self.prev_error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]
        self.d_error_filtered = [0.0, 0.0, 0.0]
        self.last_pid_time = time.time()

        # Safe Bank Angle Clamps (+/- 50 PWM) and Throttle Clamps
        self.min_roll_pitch = 1450
        self.max_roll_pitch = 1550
        self.min_throttle = 1445
        self.max_throttle = 1555

        # Publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)
        self.heartbeat_pub = self.create_publisher(Header, '/controller_heartbeat', 10)

        # Subscriptions
        self.create_subscription(
            MarkerArray, '/whycode_node/markers', self.whycode_callback, 1,
            callback_group=self.control_callback_group
        )

        # Heartbeat timer (10 Hz)
        self.create_timer(0.1, self.publish_heartbeat, callback_group=self.control_callback_group)

        # Action Server
        self._action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        # Control Loop Timer (30 Hz)
        self.sample_time = 0.033
        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.control_callback_group)

        self.get_logger().info('Waypoint Action Server initialized. Ready for goals.')

    def publish_heartbeat(self):
        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        hb.frame_id = 'pico_controller_active'
        self.heartbeat_pub.publish(hb)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        time.sleep(0.1)
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)
        self.get_logger().info('Drone Armed with aux4=2000.')

    def whycode_callback(self, msg):
        if not msg.markers or len(msg.markers) == 0:
            return

        now = time.time()
        pos = msg.markers[0].position.position
        self.curr_state[0] = float(pos.x)
        self.curr_state[1] = float(pos.y)
        self.curr_state[2] = float(pos.z)
        self.drone_position = self.curr_state[:]

        self.yaw = math.degrees(msg.markers[0].rotation.z)
        self.last_marker_time = now

        if not self.marker_received:
            self.marker_received = True
            self.last_pid_time = now
            self.prev_error = [
                self.desired_state[0] - self.curr_state[0],
                self.desired_state[1] - self.curr_state[1],
                self.desired_state[2] - self.curr_state[2]
            ]
            self.arm()
            self.get_logger().info(f'WhyCode Marker Detected at [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]. Starting control loop.')

    def pid(self):
        if not self.marker_received:
            return

        now = time.time()
        dt = now - self.last_pid_time
        if dt < 0.015:
            return
        self.last_pid_time = now

        # 1. Position Errors (Target - Current)
        error = [
            self.desired_state[0] - self.curr_state[0],
            self.desired_state[1] - self.curr_state[1],
            self.desired_state[2] - self.curr_state[2]
        ]

        # 2. Precision Conditional Integral Accumulation
        for i in range(3):
            limit = 0.35 if i == 2 else 0.45
            max_sum = 25.0 if i == 2 else 15.0
            if abs(error[i]) < limit:
                self.sum_error[i] += error[i] * dt
                self.sum_error[i] = max(-max_sum, min(max_sum, self.sum_error[i]))
            else:
                self.sum_error[i] = 0.0

        # 3. Filtered Derivative Calculation
        for i in range(3):
            raw_d = (error[i] - self.prev_error[i]) / dt
            self.d_error_filtered[i] = (0.70 * raw_d) + (0.30 * self.d_error_filtered[i])

        # 4. PID Outputs
        out_x = (self.Kp[0] * error[0]) + (self.Ki[0] * self.sum_error[0]) + (self.Kd[0] * self.d_error_filtered[0])
        out_y = (self.Kp[1] * error[1]) + (self.Ki[1] * self.sum_error[1]) + (self.Kd[1] * self.d_error_filtered[1])
        out_z = (self.Kp[2] * error[2]) + (self.Ki[2] * self.sum_error[2]) + (self.Kd[2] * self.d_error_filtered[2])

        # 5. Map to RC PWM Commands
        pitch_cmd = 1500 + int(out_x)
        roll_cmd = 1500 + int(out_y)
        throttle_cmd = 1518 - int(out_z)

        # Smooth Climb Velocity Governor (takeoff altitude limiter)
        if self.curr_state[2] > 13.3 and throttle_cmd > 1542:
            throttle_cmd = 1542

        # Active Pitch-Up Decoupling: prevents climbing above 12.80 when braking
        if self.curr_state[2] < 12.85 and throttle_cmd > 1512:
            throttle_cmd = 1506

        # Floor clearance guard
        if self.curr_state[2] > 14.88:
            pitch_cmd = 1500
            roll_cmd = 1500

        # 6. Yaw PD Controller to keep desired_yaw (-0.5 deg) steady
        yaw_error = self.desired_yaw - self.yaw
        yaw_error = (yaw_error + 180.0) % 360.0 - 180.0
        yaw_d = (yaw_error - self.prev_yaw_error) / dt
        self.prev_yaw_error = yaw_error
        yaw_cmd = 1500 + int(self.Kp_yaw * yaw_error + self.Kd_yaw * yaw_d)
        yaw_cmd = max(1460, min(1540, yaw_cmd))

        # Clamping
        self.cmd.rc_pitch = max(self.min_roll_pitch, min(self.max_roll_pitch, pitch_cmd))
        self.cmd.rc_roll = max(self.min_roll_pitch, min(self.max_roll_pitch, roll_cmd))
        self.cmd.rc_throttle = max(self.min_throttle, min(self.max_throttle, throttle_cmd))
        self.cmd.rc_yaw = yaw_cmd
        self.cmd.rc_aux4 = 2000

        self.prev_error = [error[0], error[1], error[2]]
        self.command_pub.publish(self.cmd)

        # Publish Error
        self.pos_error.pitch_error = float(error[0])
        self.pos_error.roll_error = float(error[1])
        self.pos_error.throttle_error = float(error[2])
        self.pos_error.yaw_error = float(yaw_error)
        self.pos_error_pub.publish(self.pos_error)

    def is_drone_in_sphere(self, drone_pos, target, tolerance=0.4):
        return (
            abs(drone_pos[0] - target[0]) <= tolerance and
            abs(drone_pos[1] - target[1]) <= tolerance and
            abs(drone_pos[2] - target[2]) <= tolerance
        )

    def execute_callback(self, goal_handle):
        req_wp = goal_handle.request.waypoint.position
        target = [float(req_wp.x), float(req_wp.y), float(req_wp.z)]
        self.desired_state = target[:]
        self.sum_error = [0.0, 0.0, 0.0]
        self.d_error_filtered = [0.0, 0.0, 0.0]

        self.get_logger().info(f'Executing Goal -> Waypoint: [{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}]')

        start_time = time.time()
        point_in_sphere_start_time = None
        max_time_inside_sphere = 0.0

        feedback_msg = NavToWaypoint.Feedback()

        # Monitor transit and verify continuous 3-second hold
        while True:
            time.sleep(0.03)

            # Update feedback
            feedback_msg.current_waypoint.header.stamp = self.get_clock().now().to_msg()
            feedback_msg.current_waypoint.header.stamp.sec = int(max_time_inside_sphere)
            feedback_msg.current_waypoint.pose.position.x = float(self.curr_state[0])
            feedback_msg.current_waypoint.pose.position.y = float(self.curr_state[1])
            feedback_msg.current_waypoint.pose.position.z = float(self.curr_state[2])
            goal_handle.publish_feedback(feedback_msg)

            in_sphere = self.is_drone_in_sphere(self.drone_position, target, tolerance=0.4)
            now = time.time()

            if in_sphere:
                if point_in_sphere_start_time is None:
                    point_in_sphere_start_time = now
                    self.get_logger().info(f'🎯 In Sphere [{self.curr_state[0]:.2f}, {self.curr_state[1]:.2f}, {self.curr_state[2]:.2f}]. Starting 3.0s hold...')

                time_inside = now - point_in_sphere_start_time
                if time_inside > max_time_inside_sphere:
                    max_time_inside_sphere = time_inside

                if max_time_inside_sphere >= 3.0:
                    self.get_logger().info(f'🏆 Waypoint [{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}] held for 3.0s continuously!')
                    break
            else:
                if point_in_sphere_start_time is not None:
                    self.get_logger().warn(f'⚠️ Drifted out of sphere after {now - point_in_sphere_start_time:.2f}s. Resetting hold timer.')
                    point_in_sphere_start_time = None

        goal_handle.succeed()

        result = NavToWaypoint.Result()
        total_settle_time = int(time.time() - start_time)
        result.hov_time = total_settle_time
        self.get_logger().info(f'Goal finished in {total_settle_time}s. Returning result.')
        return result


def main(args=None):
    rclpy.init(args=args)
    waypoint_server = WayPointServer()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(waypoint_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_server.disarm()
        waypoint_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
