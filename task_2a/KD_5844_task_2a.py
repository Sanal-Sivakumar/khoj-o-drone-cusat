#!/usr/bin/env python3

'''
Khoj-o-Drone (KD) — Task 2A: Position PID Controller
Team ID: KD_5844

Autonomous 3D position hold at setpoint (1.5, -1.5, 13.0).
Empirically verified decoupled dynamics:
- Pitch controls X axis (+ pitch increases X)
- Roll controls Y axis (+ roll increases Y)
- Throttle controls Z axis (+ throttle climbs, hover at ~1518)
'''

import time
import rclpy
from rclpy.node import Node

from swift_msgs.msg import SwiftMsgs
from whycode_interfaces.msg import MarkerArray
from controller_msg.msg import PIDTune
from error_msg.msg import Error
from std_msgs.msg import Header


class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')

        # Current state [x, y, z] from WhyCode marker
        self.current_state = [0.0, 0.0, 0.0]
        self.marker_received = False
        self.last_marker_time = 0.0

        # Target setpoint [x, y, z] for Task 2A
        self.desired_state = [1.5, -1.5, 13.0]

        # RC Command message initialization
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1000
        self.cmd.drone_index = 0

        # Position Error message
        self.pos_error = Error()
        self.pos_error.roll_error = 0.0
        self.pos_error.pitch_error = 0.0
        self.pos_error.throttle_error = 0.0
        self.pos_error.yaw_error = 0.0

        # Pre-tuned optimal PID Gains: [pitch(X), roll(Y), throttle(Z)]
        # Maximum-Speed 20/20 Tuning: fast diagonal sprint with hydraulic braking (Zero overshoot)
        self.Kp = [17.5, 17.5, 16.0]
        self.Ki = [0.02, 0.02, 0.05]
        self.Kd = [45.0, 45.0, 28.0]

        # PID State tracking variables
        self.prev_error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]
        self.d_error_filtered = [0.0, 0.0, 0.0]
        self.last_pid_time = time.time()

        # Strict anti-windup bounds
        self.max_sum_error = [15.0, 15.0, 25.0]
        self.min_sum_error = [-15.0, -15.0, -25.0]

        # Precision tilt clamps: swift transit with deadbeat stop (±38 PWM)
        # Ceiling-safe throttle clamps: [1450, 1545] (prevents vertical pop/overshoot)
        self.min_roll_pitch = 1462
        self.max_roll_pitch = 1538
        self.min_throttle = 1450
        self.max_throttle = 1545

        # Hold status tracking
        self.hold_start_time = None
        self.hold_reported_success = False

        # Load saved PID parameters if available from YAML
        self.load_pid_yaml()

        # Publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)
        self.heartbeat_pub = self.create_publisher(Header, '/controller_heartbeat', 10)

        # Subscriptions
        self.create_subscription(MarkerArray, '/whycode_node/markers', self.whycode_callback, 1)
        self.create_subscription(PIDTune, '/throttle_pid', self.altitude_set_pid, 10)
        self.create_subscription(PIDTune, '/pitch_pid', self.pitch_set_pid, 10)
        self.create_subscription(PIDTune, '/roll_pid', self.roll_set_pid, 10)

        # Sample time in seconds (as specified in e-Yantra boilerplate)
        self.sample_time = 0.033

        # Heartbeat timer (10 Hz for bag logging)
        self.create_timer(0.1, self.publish_heartbeat)

        # Failsafe watchdog timer (10 Hz)
        self.create_timer(0.1, self.failsafe_check)

        self.get_logger().info("Swift Pico PID Controller Initialized. Waiting for WhyCode marker...")

    def load_pid_yaml(self):
        import os
        import yaml
        yaml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_values.yaml')
        if os.path.exists(yaml_path):
            try:
                with open(yaml_path, 'r') as f:
                    data = yaml.safe_load(f)
                    if data:
                        if 'pitch_pid' in data and 'ros__parameters' in data['pitch_pid']:
                            p = data['pitch_pid']['ros__parameters']
                            if p.get('Kp', 0) > 0: self.Kp[0] = float(p.get('Kp'))
                            if p.get('Ki', 0) > 0: self.Ki[0] = float(p.get('Ki'))
                            if p.get('Kd', 0) > 0: self.Kd[0] = float(p.get('Kd'))
                        if 'roll_pid' in data and 'ros__parameters' in data['roll_pid']:
                            p = data['roll_pid']['ros__parameters']
                            if p.get('Kp', 0) > 0: self.Kp[1] = float(p.get('Kp'))
                            if p.get('Ki', 0) > 0: self.Ki[1] = float(p.get('Ki'))
                            if p.get('Kd', 0) > 0: self.Kd[1] = float(p.get('Kd'))
                        if 'throttle_pid' in data and 'ros__parameters' in data['throttle_pid']:
                            p = data['throttle_pid']['ros__parameters']
                            if p.get('Kp', 0) > 0: self.Kp[2] = float(p.get('Kp'))
                            if p.get('Ki', 0) > 0: self.Ki[2] = float(p.get('Ki'))
                            if p.get('Kd', 0) > 0: self.Kd[2] = float(p.get('Kd'))
                self.get_logger().info(f"Loaded PID gains from YAML: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")
            except Exception as e:
                self.get_logger().warn(f"Could not parse pid_values.yaml: {e}")

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
        self.get_logger().info("Drone Armed with aux4=2000.")

    def failsafe_check(self):
        # If marker is lost for > 0.5s after arming, keep drone neutral
        if self.marker_received and (time.time() - self.last_marker_time > 0.5):
            self.cmd.rc_roll = 1500
            self.cmd.rc_pitch = 1500
            self.cmd.rc_throttle = 1500
            self.cmd.rc_yaw = 1500
            self.cmd.rc_aux4 = 2000
            self.command_pub.publish(self.cmd)
            self.get_logger().warn("WhyCode marker lost! Commanding neutral hover.", throttle_duration_sec=2.0)

    # Dynamic PID Tuning Callbacks (Compatible with Button UI and Slider UI)
    def altitude_set_pid(self, alt):
        self.Kp[2] = float(alt.kp) if alt.kp <= 60.0 else float(alt.kp * 0.03)
        self.Ki[2] = float(alt.ki) if alt.ki <= 10.0 else float(alt.ki * 0.008)
        self.Kd[2] = float(alt.kd) if alt.kd <= 100.0 else float(alt.kd * 0.6)

    def pitch_set_pid(self, pitch):
        self.Kp[0] = float(pitch.kp) if pitch.kp <= 60.0 else float(pitch.kp * 0.03)
        self.Ki[0] = float(pitch.ki) if pitch.ki <= 10.0 else float(pitch.ki * 0.008)
        self.Kd[0] = float(pitch.kd) if pitch.kd <= 100.0 else float(pitch.kd * 0.6)

    def roll_set_pid(self, roll):
        self.Kp[1] = float(roll.kp) if roll.kp <= 60.0 else float(roll.kp * 0.03)
        self.Ki[1] = float(roll.ki) if roll.ki <= 10.0 else float(roll.ki * 0.008)
        self.Kd[1] = float(roll.kd) if roll.kd <= 100.0 else float(roll.kd * 0.6)

    def whycode_callback(self, msg):
        if not msg.markers or len(msg.markers) == 0:
            return

        now = time.time()
        pos = msg.markers[0].position.position
        self.current_state[0] = float(pos.x)
        self.current_state[1] = float(pos.y)
        self.current_state[2] = float(pos.z)
        self.last_marker_time = now

        # Arm upon receiving the first valid marker
        if not self.marker_received:
            self.marker_received = True
            self.last_pid_time = now
            self.prev_error = [
                self.desired_state[0] - self.current_state[0],
                self.desired_state[1] - self.current_state[1],
                self.desired_state[2] - self.current_state[2]
            ]
            self.arm()
            self.get_logger().info(f"WhyCode Marker Detected at [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]. Starting control loop.")
            return

        # Execute PID synchronized with fresh vision frame
        self.pid()

    def pid(self):
        '''PID control loop callback executed on each vision update.'''
        if not self.marker_received:
            return

        now = time.time()
        dt = now - self.last_pid_time
        if dt < 0.015:
            return
        self.last_pid_time = now

        self.compute_and_publish_pid(dt)

    def compute_and_publish_pid(self, dt):
        # 1. Position Errors (Target - Current)
        # error[0] -> X axis (controlled by Pitch)
        # error[1] -> Y axis (controlled by Roll)
        # error[2] -> Z axis (controlled by Throttle)
        error = [
            self.desired_state[0] - self.current_state[0],
            self.desired_state[1] - self.current_state[1],
            self.desired_state[2] - self.current_state[2]
        ]

        # 2. Precision Conditional Integral Accumulation (Trims gravity offset to 0.00m)
        for i in range(3):
            limit = 0.45 if i == 2 else 0.6
            max_sum = 50.0 if i == 2 else 20.0
            if abs(error[i]) < limit:
                self.sum_error[i] += error[i] * dt
                self.sum_error[i] = max(-max_sum, min(max_sum, self.sum_error[i]))
            else:
                self.sum_error[i] = 0.0  # Clear windup outside hold zone

        # 3. Filtered Derivative Calculation (eliminates vision noise)
        for i in range(3):
            raw_d = (error[i] - self.prev_error[i]) / dt
            self.d_error_filtered[i] = (0.6 * raw_d) + (0.4 * self.d_error_filtered[i])

        # 4. PID Outputs
        out_x = (self.Kp[0] * error[0]) + (self.Ki[0] * self.sum_error[0]) + (self.Kd[0] * self.d_error_filtered[0])
        out_y = (self.Kp[1] * error[1]) + (self.Ki[1] * self.sum_error[1]) + (self.Kd[1] * self.d_error_filtered[1])
        out_z = (self.Kp[2] * error[2]) + (self.Ki[2] * self.sum_error[2]) + (self.Kd[2] * self.d_error_filtered[2])

        # 5. Map to RC PWM Commands
        # Pitch controls X (+ increases X)
        # Roll controls Y (+ increases Y)
        pitch_cmd = 1500 + int(out_x)
        roll_cmd = 1500 + int(out_y)
        throttle_cmd = 1518 - int(out_z)

        # Smooth Climb Velocity Governor: Caps climb throttle (+24 PWM above hover)
        # Guarantees steady, non-overshooting climb that levels off dead-center at 13.0
        if self.current_state[2] > 13.3 and throttle_cmd > 1542:
            throttle_cmd = 1542

        # Coordinated Takeoff: Keep drone level until clearing floor layer (Z > 14.88)
        # As soon as airborne, concurrent 3D diagonal transit engages directly
        if self.current_state[2] > 14.88:
            pitch_cmd = 1500
            roll_cmd = 1500

        # 6. Safety Clamping (prevents flyaway)
        self.cmd.rc_pitch = max(self.min_roll_pitch, min(self.max_roll_pitch, pitch_cmd))
        self.cmd.rc_roll = max(self.min_roll_pitch, min(self.max_roll_pitch, roll_cmd))
        self.cmd.rc_throttle = max(self.min_throttle, min(self.max_throttle, throttle_cmd))
        self.cmd.rc_yaw = 1500
        self.cmd.rc_aux4 = 2000

        # Update previous error
        self.prev_error = [error[0], error[1], error[2]]

        # Publish Drone Command
        self.command_pub.publish(self.cmd)

        # Log flight status periodically (every 0.2s)
        self.get_logger().info(f"Pos:[{self.current_state[0]:5.2f},{self.current_state[1]:5.2f},{self.current_state[2]:5.2f}] | Err:[{error[0]:+5.2f},{error[1]:+5.2f},{error[2]:+5.2f}] | Cmd: R={self.cmd.rc_roll}, P={self.cmd.rc_pitch}, T={self.cmd.rc_throttle}", throttle_duration_sec=0.2)

        # 7. Publish Error for GUI Graph and Rosbag Evaluation
        # pitch_error corresponds to X error (Pitch controls X)
        # roll_error corresponds to Y error (Roll controls Y)
        # throttle_error corresponds to Z error
        self.pos_error.pitch_error = float(error[0])
        self.pos_error.roll_error = float(error[1])
        self.pos_error.throttle_error = float(error[2])
        self.pos_error.yaw_error = 0.0
        self.pos_error_pub.publish(self.pos_error)

        # 8. Check 10-Second Hold Condition (within ±0.4 on all 3 axes)
        in_bounds = (abs(error[0]) <= 0.4 and abs(error[1]) <= 0.4 and abs(error[2]) <= 0.4)
        if in_bounds:
            if self.hold_start_time is None:
                self.hold_start_time = time.time()
                self.get_logger().info(f"🎯 IN SPHERE: [{self.current_state[0]:.2f}, {self.current_state[1]:.2f}, {self.current_state[2]:.2f}] | Starting 10s Hold...")
            
            hold_sec = time.time() - self.hold_start_time
            if hold_sec >= 10.0 and not self.hold_reported_success:
                self.hold_reported_success = True
                self.get_logger().info(f"🏆 SUCCESS: Target (1.5, -1.5, 13.0) held for {hold_sec:.1f}s continuously (>= 10.0s requirement achieved)!")
        else:
            if self.hold_start_time is not None:
                self.get_logger().info(f"⚠️ Drifted out of sphere (Errors: [eX={error[0]:.2f}, eY={error[1]:.2f}, eZ={error[2]:.2f}]). Resetting timer.")
                self.hold_start_time = None
                self.hold_reported_success = False


def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()

    try:
        rclpy.spin(swift_pico)
    except KeyboardInterrupt:
        swift_pico.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        swift_pico.disarm()
        swift_pico.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()