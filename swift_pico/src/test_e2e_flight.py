#!/usr/bin/env python3

'''
Clean End-to-End Automated Test Runner for Swift Pico Task 2A.
Ensures single-instance execution, captures full telemetry, and terminates cleanly.
'''

import os
import sys
import time
import subprocess
import signal

def cleanup_all():
    print("[TEST] Cleaning up any lingering processes...")
    subprocess.run(["pkill", "-9", "-f", "mujoco_bridge"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "whycode_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "image_view"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "rotors_swift_interface"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "roll_pitch_yawrate_thrust_controller"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "pico_controller"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def main():
    cleanup_all()

    env = os.environ.copy()
    
    print("\n[TEST] 1. Launching MuJoCo Simulation...")
    sim_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && ros2 launch swift_pico swift_pico_simulation.launch.py"
    ]
    sim_proc = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)

    # Wait 4 seconds for simulation and WhyCode to initialize
    print("[TEST] 2. Waiting 4 seconds for simulation bridge and WhyCode camera...")
    time.sleep(4.0)

    print("[TEST] 3. Launching pico_controller_PID.py...")
    ctrl_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && python3 /root/pico_ws/src/swift_pico/src/pico_controller_PID.py"
    ]
    ctrl_proc = subprocess.Popen(ctrl_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

    start_time = time.time()
    test_duration = 28.0
    print(f"[TEST] 4. Monitoring flight for {test_duration} seconds:\n" + "="*70)

    try:
        while time.time() - start_time < test_duration:
            line = ctrl_proc.stdout.readline()
            if line:
                print(line.rstrip())
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[TEST] Interrupted by user.")
    finally:
        print("\n" + "="*70 + "\n[TEST] 5. Test finished. Shutting down all processes cleanly...")
        try:
            os.killpg(os.getpgid(ctrl_proc.pid), signal.SIGTERM)
        except Exception:
            pass
        try:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
        except Exception:
            pass
        time.sleep(1.0)
        cleanup_all()
        print("[TEST] Complete. Container is 100% clean.\n")

if __name__ == '__main__':
    main()
