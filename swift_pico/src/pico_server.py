#!/usr/bin/env python3

'''
Khoj-o-Drone (KD) — Task 3A: Mission Action Server (pico_server)
Team ID: KD_5844

Action server 'waypoint_navigation' (NavToWaypoint.action).
Restores the proven, critically damped stable PID controller from Task 2B / 3A baseline:
  * Pure coupled pitch/roll straight-line limiting (zero obstacle clipping)
  * Active Yaw PD controller keeping desired_yaw (-0.5 deg) steady
  * Safe, smooth bank angle limits (+/- 110 PWM) preventing centrifugal obstacle clipping
  * Thrust-driven altitude observer eliminating WhyCode vision range jitter in Z
  * Zero-crossing Z anti-windup, floor cushion, and ground liftoff governor
  * Safe obstacle lookahead (VIA_LOOKAHEAD = 0.80) to follow planned clearances
  * Arm on first goal for optimal timed benchmark
'''

import os
import time
import math

import rclpy
import threading

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from waypoint_navigation.action import NavToWaypoint
from swift_msgs.msg import SwiftMsgs
from std_msgs.msg import Header
from whycode_interfaces.msg import MarkerArray
from error_msg.msg import Error


FLIGHT_Z = 13.0            # cruise / Visit altitude (whycode Z, measured down from the camera)
SPHERE_RADIUS = 0.40       # official e-Yantra hold tolerance (radius <= 0.40 WhyCode units)
HOLD_MARGIN = 0.12         # hold margin for strict evaluation
VISIT_HOLD = 1.0 + HOLD_MARGIN   # seconds at Z = 13
SAVE_HOLD = 2.0 + HOLD_MARGIN    # seconds at Z = 14
SAVE_Z_SPLIT = 13.5        # a hover goal below this altitude line (Z larger) is a Save
MARKER_TIMEOUT = 0.5       # no whycode message for this long = marker lost, hold restarts
HOME_RADIUS = 0.6          # a hover goal this close to the take-off point is the landing goal
VIA_RADIUS = 0.35          # fly-past: done when this close, or once it has gone past
VIA_LOOKAHEAD = 0.80       # safe lookahead around obstacle clearance nodes
LAND_RATE = 2.0            # steady descent speed for landing set-point
LAND_TIMEOUT = 8.0


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

        # Mission state
        self.home = None            # [x, y, z] of the drone on the ground before take-off
        self.landed = False         # True after touchdown: motors off, controller silent
        self.armed = False          # motors are armed when the first goal arrives (keeps the armed time short)
        self.goal_seq = 0           # increases with every accepted goal
        self.goal_lock = threading.Lock()

        # Altitude observer (filters out high-frequency vision range noise in Z)
        self.z_est = 15.0
        self.vz_est = 0.0
        self.z_bias = 0.0
        self.last_az = 0.0
        self.z_meas_time = 0.0
        self.hover_throttle = 1516

        # Yaw control state (Active PD)
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

        # Precision Critically Damped Gains: [pitch(X), roll(Y), throttle(Z)]
        self.Kp = [54.0, 54.0, 62.0]
        self.Ki = [0.55, 0.55, 0.75]
        self.Kd = [98.0, 98.0, 93.0]

        # PID tracking state
        self.prev_error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]
        self.d_error_filtered = [0.0, 0.0, 0.0]
        self.last_pid_time = time.time()

        # Controlled Bank Angle Clamps (+/- 120 PWM) and Throttle Clamps
        # Preserves 140 PWM motor headroom, preventing centrifugal drift and obstacle clipping
        self.min_roll_pitch = 1380
        self.max_roll_pitch = 1620
        self.min_throttle = 1425
        self.max_throttle = 1585

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
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )

        # Control Loop Timer (30 Hz)
        self.sample_time = 0.033
        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.control_callback_group)

        self.get_logger().info('Waypoint Action Server initialized. Stable Obstacle-Safe PID Ready.')

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

        self.yaw = math.degrees(float(msg.markers[0].rotation.z))
        self.last_marker_time = now

        if not self.marker_received:
            self.marker_received = True
            self.last_pid_time = now
            self.home = self.curr_state[:]      # where the drone stands before take-off (F6)
            self.prev_error = [
                self.desired_state[0] - self.curr_state[0],
                self.desired_state[1] - self.curr_state[1],
                self.desired_state[2] - self.curr_state[2]
            ]
            self.get_logger().info(f'WhyCode Marker Detected at [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]. Starting control loop.')

    def pid(self):
        if not self.marker_received or self.landed or not self.armed:
            return

        now = time.time()
        dt = now - self.last_pid_time
        if dt < 0.015:
            return
        self.last_pid_time = now

        # 0. Altitude observer: filters WhyCode vision range jitter in Z
        on_ground = self.home is not None and self.curr_state[2] > self.home[2] - 0.03 and self.last_az >= 0.0
        if on_ground:
            self.z_est, self.vz_est, self.z_bias = self.curr_state[2], 0.0, 0.0
        else:
            az = self.last_az + self.z_bias
            self.z_est += self.vz_est * dt + 0.5 * az * dt * dt
            self.vz_est += az * dt
            if self.last_marker_time != self.z_meas_time:
                self.z_meas_time = self.last_marker_time
                r = max(-0.25, min(0.25, self.curr_state[2] - self.z_est))
                self.z_est += 0.14 * r
                self.vz_est += 0.31 * r
                self.z_bias = max(-1.5, min(1.5, self.z_bias + 0.34 * r))

        # 1. Position Errors (Target - Current)
        # Rescale X / Y by z_est / curr_z to eliminate edge vision parallax jitter
        scale = self.z_est / self.curr_state[2] if self.curr_state[2] > 1.0 else 1.0
        error = [
            self.desired_state[0] - self.curr_state[0] * scale,
            self.desired_state[1] - self.curr_state[1] * scale,
            self.desired_state[2] - self.z_est
        ]

        # 2. Precision Conditional Integral Accumulation (anti-windup outside hold zone)
        for i in range(3):
            limit = 0.22 if i == 2 else 0.35
            max_sum = 10.0 if i == 2 else 8.0
            if abs(error[i]) < limit:
                self.sum_error[i] += error[i] * dt
                self.sum_error[i] = max(-max_sum, min(max_sum, self.sum_error[i]))
            else:
                self.sum_error[i] = 0.0

        # 3. Filtered Derivative Calculation (low-pass filter against vision noise)
        for i in range(2):
            raw_d = (error[i] - self.prev_error[i]) / dt
            self.d_error_filtered[i] = (0.65 * raw_d) + (0.35 * self.d_error_filtered[i])
        self.d_error_filtered[2] = -self.vz_est

        # 4. PID Outputs
        out_x = (self.Kp[0] * error[0]) + (self.Ki[0] * self.sum_error[0]) + (self.Kd[0] * self.d_error_filtered[0])
        out_y = (self.Kp[1] * error[1]) + (self.Ki[1] * self.sum_error[1]) + (self.Kd[1] * self.d_error_filtered[1])
        out_z = (self.Kp[2] * error[2]) + (self.Ki[2] * self.sum_error[2]) + (self.Kd[2] * self.d_error_filtered[2])

        # 4b. Limit pitch and roll TOGETHER: scale both by the exact same factor so tilt keeps its
        #     exact line-of-sight direction and the drone flies a laser-straight line to the target.
        #     This prevents dog-leg diagonal bulging that clips obstacles!
        dist_xy = math.hypot(error[0], error[1])
        limit = float(self.max_roll_pitch - 1500)
        # Smooth terminal braking when inside 0.60m of target to settle dead-center with zero overshoot
        if dist_xy < 0.60:
            limit = max(42.0, min(limit, 42.0 + (limit - 42.0) * (dist_xy / 0.60)))
        biggest = max(abs(out_x), abs(out_y))
        if biggest > limit:
            out_x *= limit / biggest
            out_y *= limit / biggest

        # 5. Map to RC PWM Commands
        pitch_cmd = 1500 + int(out_x)
        roll_cmd = 1500 + int(out_y)
        tilt = math.hypot(out_x, out_y) * 9.24e-4
        tilt_boost = (1.0 / max(math.cos(tilt), 0.8) - 1.0) * 9.81 / 0.0201
        throttle_cmd = self.hover_throttle - int(out_z) + int(tilt_boost)

        # Zero-crossing anti-windup on Z
        if error[2] * self.prev_error[2] < 0:
            self.sum_error[2] = 0.0

        # Floor cushion for Save (14.0): catch firmly if dipping below 14.18
        if self.desired_state[2] <= 14.05 and self.z_est > 14.18 and throttle_cmd < 1530:
            throttle_cmd = 1535

        # Ground Liftoff Governor: prevents rocketing upward only while on the floor (Z > 14.85)
        if self.curr_state[2] > 14.85 and throttle_cmd > 1542:
            throttle_cmd = 1542

        # Floor clearance guard: strict vertical climb before lateral banking
        if self.curr_state[2] > 14.75:
            pitch_cmd = 1500
            roll_cmd = 1500

        # 6. Yaw PD Controller to keep desired_yaw (-0.5 deg) rock-steady
        yaw_error = self.desired_yaw - self.yaw
        yaw_error = (yaw_error + 180.0) % 360.0 - 180.0
        yaw_d = (yaw_error - self.prev_yaw_error) / dt
        self.prev_yaw_error = yaw_error
        yaw_cmd = 1500 + int(self.Kp_yaw * yaw_error + self.Kd_yaw * yaw_d)
        yaw_cmd = max(1460, min(1540, yaw_cmd))

        # 7. Marker lost guard: level off if marker flickers out
        if not self.marker_visible():
            pitch_cmd = 1500
            roll_cmd = 1500
            throttle_cmd = self.hover_throttle

        # Clamping
        self.cmd.rc_pitch = max(self.min_roll_pitch, min(self.max_roll_pitch, pitch_cmd))
        self.cmd.rc_roll = max(self.min_roll_pitch, min(self.max_roll_pitch, roll_cmd))
        self.cmd.rc_throttle = max(self.min_throttle, min(self.max_throttle, throttle_cmd))
        lift = (9.81 + 0.0201 * (self.cmd.rc_throttle - self.hover_throttle)) * max(math.cos(tilt), 0.8)
        self.last_az = -1.02 * (lift - 9.81)
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

    # ------------------------------------------------------------------ helpers
    def in_sphere(self, target, radius=SPHERE_RADIUS):
        d = math.sqrt(sum((self.drone_position[i] - target[i]) ** 2 for i in range(3)))
        return d <= radius

    def marker_visible(self):
        return (time.time() - self.last_marker_time) <= MARKER_TIMEOUT

    def set_target(self, target):
        self.desired_state = [float(target[0]), float(target[1]), float(target[2])]
        self.sum_error = [0.0, 0.0, 0.0]
        # Reset derivative tracking across waypoint jumps to avoid impulse kicks
        scale = self.z_est / self.curr_state[2] if self.curr_state[2] > 1.0 else 1.0
        self.prev_error = [
            self.desired_state[0] - self.curr_state[0] * scale,
            self.desired_state[1] - self.curr_state[1] * scale,
            self.desired_state[2] - self.z_est
        ]
        self.d_error_filtered = [0.0, 0.0, 0.0]

    def is_home(self, target):
        return (self.home is not None and
                math.hypot(target[0] - self.home[0], target[1] - self.home[1]) <= HOME_RADIUS)

    # ------------------------------------------------------------------ action callbacks
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received: accepted.')
        return CancelResponse.ACCEPT

    def publish_feedback(self, goal_handle, feedback_msg, hold_time):
        feedback_msg.current_waypoint.header.stamp = self.get_clock().now().to_msg()
        feedback_msg.current_waypoint.header.stamp.sec = int(hold_time)
        feedback_msg.current_waypoint.pose.position.x = float(self.curr_state[0])
        feedback_msg.current_waypoint.pose.position.y = float(self.curr_state[1])
        feedback_msg.current_waypoint.pose.position.z = float(self.curr_state[2])
        goal_handle.publish_feedback(feedback_msg)

    def execute_callback(self, goal_handle):
        with self.goal_lock:
            self.goal_seq += 1
            my_seq = self.goal_seq

        req = goal_handle.request
        target = [float(req.waypoint.position.x), float(req.waypoint.position.y), float(req.waypoint.position.z)]
        hover = bool(req.hover)
        result = NavToWaypoint.Result()
        feedback_msg = NavToWaypoint.Feedback()
        start_time = time.time()

        # Wait for the first whycode fix
        while rclpy.ok() and not self.marker_received:
            time.sleep(0.02)

        if self.landed or not self.armed:     # first goal (or a goal after landing): arm and take off
            self.landed = False
            self.armed = True
            self.last_pid_time = time.time()
            self.prev_error = [0.0, 0.0, 0.0]
            self.arm()

        # ---- what kind of goal is this? ----
        if not hover:
            kind = 'fly-past'
        elif target[2] > SAVE_Z_SPLIT:
            kind, hold_needed = 'Save', SAVE_HOLD
        elif self.is_home(target):
            kind, hold_needed = 'Home', VISIT_HOLD
        else:
            kind, hold_needed = 'Visit', VISIT_HOLD
        self.get_logger().info(f'Goal [{kind}] -> [{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}]')

        if kind == 'fly-past':
            sx, sy = self.curr_state[0], self.curr_state[1]
            leg = math.hypot(target[0] - sx, target[1] - sy)
            ux, uy = ((target[0] - sx) / leg, (target[1] - sy) / leg) if leg > 1e-3 else (0.0, 0.0)
            aim_x = target[0] + VIA_LOOKAHEAD * ux
            aim_y = target[1] + VIA_LOOKAHEAD * uy
            self.set_target([aim_x, aim_y, FLIGHT_Z])
        else:
            self.set_target(target)

        hold_start = None
        held = 0.0
        while rclpy.ok():
            time.sleep(0.01)

            if my_seq != self.goal_seq:           # a newer goal has taken over
                goal_handle.abort()
                return result

            if goal_handle.is_cancel_requested:
                self.set_target([self.curr_state[0], self.curr_state[1], FLIGHT_Z])
                goal_handle.canceled()
                self.get_logger().info(
                    f'Goal [{kind}] cancelled. Holding at [{self.curr_state[0]:.2f}, {self.curr_state[1]:.2f}].')
                result.hov_time = int(time.time() - start_time)
                return result

            self.publish_feedback(goal_handle, feedback_msg, held)

            if kind == 'fly-past':
                dx, dy = target[0] - self.curr_state[0], target[1] - self.curr_state[1]
                passed = (dx * ux + dy * uy) <= 0.0
                if math.hypot(dx, dy) <= VIA_RADIUS or passed:
                    break
                continue

            now = time.time()
            inside_tol = self.in_sphere(target, SPHERE_RADIUS)
            # Require settling within 0.28m vertically and 0.32m 3D before starting hold timer
            inside_entry = self.in_sphere(target, 0.32) and abs(self.curr_state[2] - target[2]) <= 0.28
            if self.marker_visible() and (inside_entry if hold_start is None else inside_tol):
                if hold_start is None:
                    hold_start = now
                held = now - hold_start
                if held >= hold_needed:
                    self.get_logger().info(f'{kind} complete: held {held:.2f} s.')
                    break
            elif hold_start is not None:
                self.get_logger().warn(f'Left the sphere after {now - hold_start:.2f} s — hold restarts.')
                hold_start, held = None, 0.0

        if kind == 'Home' and rclpy.ok():
            self.land(goal_handle, feedback_msg, target)

        goal_handle.succeed()
        result.hov_time = int(time.time() - start_time)
        return result

    def land(self, goal_handle, feedback_msg, target):
        '''Lower the set-point steadily to the ground over F6, then cut the motors and stay down.'''
        ground_z = self.home[2]
        self.get_logger().info('Hover at F6 done. Landing...')
        z = self.curr_state[2]
        t0 = last = time.time()
        while rclpy.ok() and time.time() - t0 < LAND_TIMEOUT:
            time.sleep(0.02)
            now = time.time()
            z = min(z + LAND_RATE * (now - last), ground_z + 0.3)
            last = now
            self.desired_state = [target[0], target[1], z]
            self.publish_feedback(goal_handle, feedback_msg, 0.0)
            if self.z_est >= ground_z - 0.05:
                break
        time.sleep(0.1)
        self.landed = True                    # pid() goes silent from here
        for _ in range(5):
            self.disarm()
            time.sleep(0.05)
        self.get_logger().info(
            f'Landed at [{self.curr_state[0]:.2f}, {self.curr_state[1]:.2f}, {self.curr_state[2]:.2f}]. Motors off.')


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
