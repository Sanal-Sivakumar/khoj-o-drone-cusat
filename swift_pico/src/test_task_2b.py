#!/usr/bin/env python3

'''
Automated End-to-End Test Runner for Task 2B Waypoint Navigation.
Launches Simulation -> waypoint_service -> pico_client -> pico_server
Uses threaded line readers to prevent any stdout pipe blocking.
Runs with full 100-second runway and benchmarks against the 40-second target.
'''

import os
import sys
import time
import subprocess
import signal
import threading
from queue import Queue

def cleanup_all():
    print('[TEST] Cleaning up any lingering processes...')
    for p in ['mujoco_bridge', 'whycode_node', 'image_view', 'rotors_swift_interface',
              'roll_pitch_yawrate_thrust_controller', 'pico_controller',
              'waypoint_service', 'pico_client', 'pico_server']:
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

    print('\n[TEST 2B] 1. Launching MuJoCo Simulation...')
    sim_cmd = [
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && ros2 launch swift_pico swift_pico_simulation.launch.py'
    ]
    sim_proc = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
    time.sleep(4.0)

    print('[TEST 2B] 2. Launching waypoint_service...')
    srv_cmd = [
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && python3 -u /root/pico_ws/install/swift_pico/lib/swift_pico/waypoint_service.py'
    ]
    srv_proc = subprocess.Popen(srv_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
    time.sleep(1.0)

    print('[TEST 2B] 3. Launching pico_client...')
    cli_cmd = [
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && python3 -u /root/pico_ws/install/swift_pico/lib/swift_pico/pico_client.py'
    ]
    cli_proc = subprocess.Popen(cli_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
    time.sleep(1.0)

    print('[TEST 2B] 4. Launching pico_server (Kicking off flight)...')
    pico_cmd = [
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && python3 -u /root/pico_ws/install/swift_pico/lib/swift_pico/pico_server.py'
    ]
    pico_proc = subprocess.Popen(pico_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

    log_queue = Queue()
    threading.Thread(target=stream_reader, args=(pico_proc.stdout, 'SERVER', log_queue), daemon=True).start()
    threading.Thread(target=stream_reader, args=(cli_proc.stdout, 'CLIENT', log_queue), daemon=True).start()

    print('[TEST 2B] 5. Monitoring mission progress (100.0s runway)...')
    print('='*75)
    start_time = time.time()
    max_duration = 100.0
    completed = False

    try:
        while time.time() - start_time < max_duration:
            while not log_queue.empty():
                source, line = log_queue.get_nowait()
                elapsed = time.time() - start_time

                # Filter and display high-value telemetry
                if any(k in line for k in ['Dispatching Goal', 'Goal accepted', 'In Sphere', 'continuously', 'completed', 'All waypoints have been reached', 'WhyCode Marker Detected']):
                    print(f'[{elapsed:5.1f}s] [{source:6s}] {line}')

                if 'All waypoints have been reached successfully' in line:
                    completed = True
                    mission_time = elapsed
                    print('\n' + '='*75)
                    print(f'🏆 MISSION ACCOMPLISHED! ALL 4 WAYPOINTS COMPLETED IN {mission_time:.2f}s!')
                    if mission_time <= 40.0:
                        print(f'⭐️ TIME SCORE: FULL MARKS! ({mission_time:.2f}s <= 40.0s Target Benchmark)')
                    else:
                        print(f'⚠️ TIME SCORE: {mission_time:.2f}s (Official Target: <= 40.0s, Hard Cap: 100.0s)')
                    print('='*75 + '\n')
                    time.sleep(2.0)
                    break

            if completed:
                break
            time.sleep(0.05)

    except KeyboardInterrupt:
        print('[TEST 2B] Interrupted by user.')
    finally:
        print('[TEST 2B] 6. Cleaning up all processes...')
        for proc in [pico_proc, cli_proc, srv_proc, sim_proc]:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1.0)
        cleanup_all()
        print('[TEST 2B] All processes cleaned up successfully.')

if __name__ == '__main__':
    main()
