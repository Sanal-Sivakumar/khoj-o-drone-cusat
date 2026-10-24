#!/usr/bin/env python3
"""
Automated End-to-End Test Runner for Task 2C Autonomous Waypoint Navigation.
Launches Simulation -> waypoint_service -> verifies detection -> pico_client -> pico_server
Monitors flight across all 4 critical survivors against the 60.0s benchmark (100.0s max).
"""

import os
import sys
import time
import subprocess
import signal
import threading
from queue import Queue


def cleanup_all():
    print("[TEST 2C] Cleaning up any lingering processes...")
    for p in ['mujoco_bridge', 'whycode_node', 'image_view', 'rotors_swift_interface',
              'roll_pitch_yawrate_thrust_controller', 'pico_controller',
              'waypoint_service', 'pico_client', 'pico_server', 'record']:
        subprocess.run(['pkill', '-9', '-f', p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def stream_reader(pipe, name, queue):
    for line in iter(pipe.readline, ''):
        if line:
            queue.put((name, line.rstrip()))
    pipe.close()


def main():
    cleanup_all()

    env = os.environ.copy()
    env['DISPLAY'] = ':0'
    env['PYTHONUNBUFFERED'] = '1'

    print("\n[TEST 2C] 1. Launching MuJoCo Simulation...")
    sim_cmd = [
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && ros2 launch swift_pico swift_pico_simulation.launch.py'
    ]
    sim_proc = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
    time.sleep(4.0)

    print("[TEST 2C] 2. Launching Autonomous waypoint_service...")
    srv_cmd = [
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && python3 -u /root/pico_ws/src/swift_pico/src/waypoint_service.py'
    ]
    srv_proc = subprocess.Popen(srv_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

    log_queue = Queue()
    threading.Thread(target=stream_reader, args=(srv_proc.stdout, 'SERVICE', log_queue), daemon=True).start()

    # Wait up to 10s for waypoint_service to detect survivors
    print("[TEST 2C] 3. Waiting for survivor detection and path planning...")
    detection_done = False
    for _ in range(100):
        while not log_queue.empty():
            source, line = log_queue.get_nowait()
            print(f"[{source}] {line}")
            if any(k in line for k in ["Route from ", "Critical survivors detected", "Destroyed /image_raw subscription", "Shortest TSP Route Planned"]):
                detection_done = True
        if detection_done:
            break
        time.sleep(0.1)

    print("\n[TEST 2C] 4. Launching pico_client...")
    cli_cmd = [
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && python3 -u /root/pico_ws/src/swift_pico/src/pico_client.py'
    ]
    cli_proc = subprocess.Popen(cli_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
    threading.Thread(target=stream_reader, args=(cli_proc.stdout, 'CLIENT', log_queue), daemon=True).start()
    time.sleep(1.0)

    print("[TEST 2C] 5. Starting rosbag recording (task_2c)...")
    subprocess.run(['rm', '-rf', '/root/pico_ws/task_2c'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    bag_cmd = [
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && cd /root/pico_ws && ros2 bag record -o task_2c /whycode_node/markers /controller_heartbeat'
    ]
    bag_proc = subprocess.Popen(bag_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(1.0)

    print("[TEST 2C] 6. Launching pico_server (Kicking off flight)...")
    pico_cmd = [
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && python3 -u /root/pico_ws/src/swift_pico/src/pico_server.py'
    ]
    pico_proc = subprocess.Popen(pico_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
    threading.Thread(target=stream_reader, args=(pico_proc.stdout, 'SERVER', log_queue), daemon=True).start()

    print("[TEST 2C] 7. Monitoring mission progress (100.0s runway, 60.0s full marks target)...")
    print("=" * 75)
    start_time = time.time()
    max_duration = 100.0
    completed = False

    try:
        while time.time() - start_time < max_duration:
            while not log_queue.empty():
                source, line = log_queue.get_nowait()
                elapsed = time.time() - start_time

                if any(k in line for k in ['Dispatching Goal', 'Goal accepted', 'In Sphere', 'continuously', 'completed', 'All waypoints have been reached', 'WhyCode Marker Detected']):
                    print(f"[{elapsed:5.1f}s] [{source:7s}] {line}")

                if "All waypoints have been reached successfully" in line:
                    completed = True
                    mission_time = elapsed
                    print("\n" + "=" * 75)
                    print(f"🏆 MISSION ACCOMPLISHED! ALL 4 WAYPOINTS COMPLETED IN {mission_time:.2f}s!")
                    if mission_time <= 60.0:
                        print(f"⭐️ TIME SCORE: FULL MARKS! ({mission_time:.2f}s <= 60.0s Target Benchmark)")
                    else:
                        print(f"⚠️ TIME SCORE: {mission_time:.2f}s (Official Target: <= 60.0s, Hard Cap: 100.0s)")
                    print("=" * 75 + "\n")
                    time.sleep(2.0)
                    break

            if completed:
                break
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("[TEST 2C] Interrupted by user.")
    finally:
        print("[TEST 2C] 8. Cleaning up all processes...")
        try:
            if 'bag_proc' in locals() and bag_proc:
                os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
                time.sleep(1.0)
        except Exception:
            pass
        for proc in [pico_proc, cli_proc, srv_proc, sim_proc]:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1.0)
        cleanup_all()
        print("[TEST 2C] All processes cleaned up successfully.")


if __name__ == '__main__':
    main()

