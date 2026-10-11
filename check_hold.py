#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from error_msg.msg import Error

TOL = 0.4      # allowed error on every axis
HOLD = 10.0    # seconds it must stay inside


class Check(Node):
    def __init__(self):
        super().__init__('check_hold')
        self.create_subscription(Error, '/pos_error', self.cb, 10)
        self.t0 = None          # time of first message = "start"
        self.in_since = None    # when the current in-band streak began
        self.last_print = 0.0
        self.done = False

    def cb(self, m):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        errs = (m.pitch_error, m.roll_error, m.throttle_error)
        ok = all(abs(e) <= TOL for e in errs)

        if ok:
            if self.in_since is None:
                self.in_since = t
            held = t - self.in_since
        else:
            self.in_since = None
            held = 0.0

        if t - self.last_print >= 1.0:
            self.last_print = t
            print(f"t={t:5.1f}s  pitch={errs[0]:+.2f} roll={errs[1]:+.2f} "
                  f"thr={errs[2]:+.2f}  held={held:4.1f}s")

        if held >= HOLD and not self.done:
            self.done = True
            end = self.in_since + HOLD
            print("\n=== RESULT ===")
            print(f"Entered the +/-{TOL} band at t={self.in_since:.1f}s")
            print(f"10 s hold finished at t={end:.1f}s")
            print("PASS (inside 60 s)" if end <= 60 else "FAIL (over 60 s)")
            print("FULL MARKS (hold done by 15 s)" if end <= 15
                  else "Not full marks (hold finished after 15 s)")


def main():
    rclpy.init()
    node = Check()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
