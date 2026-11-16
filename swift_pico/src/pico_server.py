#!/usr/bin/env python3

'''
Khoj-o-Drone (KD) — Task 3A: Mission Action Server (pico_server)
Team ID: KD_5844

Action server 'waypoint_navigation' (NavToWaypoint.action). The flight controller is the
Task 2B PID. What each goal means is read from the goal itself:

    hover = False                  -> fly-past point: no stop, done as soon as the drone passes it
    hover = True,  Z = 13          -> Visit: hold 1 s inside the 0.4 sphere
    hover = True,  Z = 14          -> Save:  hold 2 s inside the 0.4 sphere
    hover = True,  Z = 13, over the take-off point (F6)
                                   -> hold 1 s, then descend, touch down and disarm

New in Task 3:
  * cancel support — a cancelled goal stops and the drone holds where it is until the next goal
  * landing at F6
  * pitch / roll are limited together (same direction, scaled) so the drone flies STRAIGHT
    legs — the planner's obstacle clearance assumes straight legs
'''

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
VISIT_HOLD = 1.0           # seconds at Z = 13
SAVE_HOLD = 2.0            # seconds at Z = 14
SAVE_Z_SPLIT = 13.5        # a hover goal below this altitude line (Z larger) is a Save
MARKER_TIMEOUT = 0.5       # no whycode message for this long = marker lost, hold restarts
HOME_RADIUS = 0.6          # a hover goal this close to the take-off point is the landing goal
VIA_RADIUS = 0.35          # fly-past: done when this close, or once it has gone past
VIA_LOOKAHEAD = 0.80       # aim beyond the via node so the drone corners at full cruise speed
LAND_RATE = 2.8            # descent speed of the landing set-point, whycode units per second
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
        self.goal_seq = 0           # increases with every accepted goal
        self.goal_lock = threading.Lock()

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

        # Optimal Critically Damped Gains: [pitch(X), roll(Y), throttle(Z)]
        self.Kp = [30.0, 30.0, 34.0]
        self.Ki = [0.03, 0.03, 0.06]
        self.Kd = [60.0, 60.0, 38.0]

        # PID tracking state
        self.prev_error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]
        self.d_error_filtered = [0.0, 0.0, 0.0]
        self.last_pid_time = time.time()

        # Controlled High-Speed Bank Angle Clamps (+/- 75 PWM) and Throttle Clamps
        self.min_roll_pitch = 1425
        self.max_roll_pitch = 1575
        self.min_throttle = 1445
        self.max_throttle = 1565

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
            self.home = self.curr_state[:]      # where the drone stands before take-off (F6)
            self.prev_error = [
                self.desired_state[0] - self.curr_state[0],
                self.desired_state[1] - self.curr_state[1],
                self.desired_state[2] - self.curr_state[2]
            ]
            self.arm()
            self.get_logger().info(f'WhyCode Marker Detected at [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]. Starting control loop.')

    def pid(self):
        if not self.marker_received or self.landed:
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
            limit = 0.45
            max_sum = 25.0 if i == 2 else 15.0
            if abs(error[i]) < limit:
                self.sum_error[i] += error[i] * dt
                self.sum_error[i] = max(-max_sum, min(max_sum, self.sum_error[i]))
            else:
                self.sum_error[i] = 0.0

        # 3. Filtered Derivative Calculation (low-pass filter against WhyCode noise)
        for i in range(3):
            raw_d = (error[i] - self.prev_error[i]) / dt
            self.d_error_filtered[i] = (0.65 * raw_d) + (0.35 * self.d_error_filtered[i])

        # 4. PID Outputs
        out_x = (self.Kp[0] * error[0]) + (self.Ki[0] * self.sum_error[0]) + (self.Kd[0] * self.d_error_filtered[0])
        out_y = (self.Kp[1] * error[1]) + (self.Ki[1] * self.sum_error[1]) + (self.Kd[1] * self.d_error_filtered[1])
        out_z = (self.Kp[2] * error[2]) + (self.Ki[2] * self.sum_error[2]) + (self.Kd[2] * self.d_error_filtered[2])

        # 4b. Limit pitch and roll TOGETHER: scale both by the same factor so the tilt keeps its
        #     direction and the drone flies a straight line to the target. (Clamping each axis
        #     on its own makes a 45-degree dash followed by a one-axis run — a dog-leg that can
        #     clip an obstacle the planner had routed around.)
        limit = float(self.max_roll_pitch - 1500)
        biggest = max(abs(out_x), abs(out_y))
        if biggest > limit:
            out_x *= limit / biggest
            out_y *= limit / biggest

        # 5. Map to RC PWM Commands
        pitch_cmd = 1500 + int(out_x)
        roll_cmd = 1500 + int(out_y)
        throttle_cmd = 1518 - int(out_z)

        # Zero-crossing anti-windup on Z
        if error[2] * self.prev_error[2] < 0:
            self.sum_error[2] = 0.0

        # Floor cushion for Save (14.0): catch firmly if dipping below 14.25
        if self.desired_state[2] <= 14.05 and self.curr_state[2] > 14.25 and throttle_cmd < 1525:
            throttle_cmd = 1530

        # Ground Liftoff Governor: prevents rocketing upward only while on the floor (Z > 14.85)
        if self.curr_state[2] > 14.85 and throttle_cmd > 1542:
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

    # ------------------------------------------------------------------ helpers
    def in_sphere(self, target, radius=SPHERE_RADIUS):
        d = math.sqrt(sum((self.drone_position[i] - target[i]) ** 2 for i in range(3)))
        return d <= radius

    def marker_visible(self):
        return (time.time() - self.last_marker_time) <= MARKER_TIMEOUT

    def set_target(self, target):
        self.desired_state = [float(target[0]), float(target[1]), float(target[2])]
        self.sum_error = [0.0, 0.0, 0.0]

    def is_home(self, target):
        return (self.home is not None and
                math.hypot(target[0] - self.home[0], target[1] - self.home[1]) <= HOME_RADIUS)

    # ------------------------------------------------------------------ action callbacks
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # ROS 2 action servers reject cancels unless told otherwise.
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

        # Wait for the first whycode fix (the server may get a goal the instant it comes up).
        while rclpy.ok() and not self.marker_received:
            time.sleep(0.02)

        if self.landed:                       # a goal after landing: take off again
            self.landed = False
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
        out_start = None
        held = 0.0
        while rclpy.ok():
            time.sleep(0.02)

            if my_seq != self.goal_seq:           # a newer goal has taken over
                goal_handle.abort()
                return result

            if goal_handle.is_cancel_requested:
                # Stop this goal and hold right here (at cruise altitude) until the next one.
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
            if self.marker_visible() and self.in_sphere(target):
                out_start = None
                if hold_start is None:
                    hold_start = now
                held = now - hold_start
                if held >= hold_needed:
                    self.get_logger().info(f'{kind} complete: held {held:.2f} s.')
                    break
            elif hold_start is not None:
                if out_start is None:
                    out_start = now
                elif now - out_start > 0.30:
                    self.get_logger().warn(f'Left the sphere after {now - hold_start:.2f} s — hold restarts.')
                    hold_start, held = None, 0.0
                    out_start = None

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
            z = min(z + LAND_RATE * (now - last), ground_z + 0.3)   # ends just below the floor: no bounce
            last = now
            self.desired_state = [target[0], target[1], z]
            self.publish_feedback(goal_handle, feedback_msg, 0.0)
            if self.curr_state[2] >= ground_z - 0.15:               # touched down
                break
        time.sleep(0.2)
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
