#!/usr/bin/env python3

'''
Automated Rosbag Recorder and Packager for Task 2A (KD_5844).
Records /whycode_node/markers, /pos_error, and /controller_heartbeat with FastDDS and QoS overrides,
verifies 10s continuous hold, and generates KD_5844_task_2a.zip.
'''

import os
import sys
import time
import shutil
import subprocess
import signal
import zipfile

TEAM_ID = "5844"
TASK_NAME = "task_2a"
SRC_DIR = "/root/pico_ws/src"
BAG_DIR = os.path.join(SRC_DIR, TASK_NAME)
ZIP_NAME = f"KD_{TEAM_ID}_{TASK_NAME}.zip"
ZIP_PATH = os.path.join(SRC_DIR, ZIP_NAME)
FASTDDS_PATH = "/root/pico_ws/src/swift_pico/config/fastdds_profile.xml"
QOS_PATH = "/root/pico_ws/src/swift_pico/config/qos_override.yaml"

def cleanup_all():
    print("[RECORDER] Cleaning up background processes...")
    subprocess.run(["pkill", "-9", "-f", "mujoco_bridge"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "whycode_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "image_view"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "rotors_swift_interface"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "roll_pitch_yawrate_thrust_controller"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "pico_controller"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-2", "-f", "ros2 bag record"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def main():
    cleanup_all()

    if os.path.exists(BAG_DIR):
        shutil.rmtree(BAG_DIR)
    if os.path.exists(ZIP_PATH):
        os.remove(ZIP_PATH)

    env = os.environ.copy()
    env['FASTRTPS_DEFAULT_PROFILES_FILE'] = FASTDDS_PATH
    env['RMW_FASTRTPS_PUBLICATION_MODE'] = 'ASYNCHRONOUS'

    print("\n[RECORDER] 1. Launching MuJoCo Simulation...")
    sim_cmd = [
        "bash", "-c",
        f"export FASTRTPS_DEFAULT_PROFILES_FILE={FASTDDS_PATH} && export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS && source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && ros2 launch swift_pico swift_pico_simulation.launch.py"
    ]
    sim_proc = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)

    print("[RECORDER] 2. Waiting 4 seconds for simulation to stabilize...")
    time.sleep(4.0)

    print(f"[RECORDER] 3. Starting ROS 2 Bag Recording in {BAG_DIR}...")
    bag_cmd = [
        "bash", "-c",
        f"export FASTRTPS_DEFAULT_PROFILES_FILE={FASTDDS_PATH} && export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS && source /opt/ros/humble/setup.bash && cd {SRC_DIR} && ros2 bag record -o {TASK_NAME} --qos-profile-overrides-path {QOS_PATH} /whycode_node/markers /pos_error /controller_heartbeat"
    ]
    bag_proc = subprocess.Popen(bag_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
    time.sleep(2.0)

    print("[RECORDER] 4. Launching pico_controller_PID.py...")
    ctrl_cmd = [
        "bash", "-c",
        f"export FASTRTPS_DEFAULT_PROFILES_FILE={FASTDDS_PATH} && export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS && source /opt/ros/humble/setup.bash && source /root/pico_ws/install/setup.bash && python3 /root/pico_ws/src/swift_pico/src/pico_controller_PID.py"
    ]
    ctrl_proc = subprocess.Popen(ctrl_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

    start_time = time.time()
    hold_achieved = False
    max_duration = 35.0

    print("[RECORDER] 5. Monitoring flight and recording bag:\n" + "="*70)
    try:
        while time.time() - start_time < max_duration:
            line = ctrl_proc.stdout.readline()
            if line:
                print(line.rstrip())
                if "SUCCESS: Target" in line:
                    hold_achieved = True
                    print("\n[RECORDER] >>> 10-Second Hold Achieved! Allowing 3 extra seconds before stopping... <<<\n")
                    time.sleep(3.0)
                    break
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[RECORDER] Interrupted.")
    finally:
        print("\n" + "="*70 + "\n[RECORDER] 6. Gracefully stopping controller, recorder, and simulation...")
        try:
            os.killpg(os.getpgid(ctrl_proc.pid), signal.SIGTERM)
        except Exception:
            pass
        time.sleep(1.0)
        # Send SIGINT specifically to ros2 process group so it flushes metadata.yaml
        try:
            os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
        except Exception:
            pass
        time.sleep(3.0)
        try:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
        except Exception:
            pass
        time.sleep(1.0)
        cleanup_all()

    if not hold_achieved:
        print("[RECORDER] ❌ Error: 10s hold was not achieved within timeframe.")
        return 1

    metadata_file = os.path.join(BAG_DIR, "metadata.yaml")
    if not os.path.exists(metadata_file):
        print("[RECORDER] Metadata missing, running ros2 bag reindex...")
        subprocess.run(["bash", "-c", f"source /opt/ros/humble/setup.bash && ros2 bag reindex {BAG_DIR}"], check=False)

    db3_file = None
    for f in os.listdir(BAG_DIR):
        if f.endswith(".db3"):
            db3_file = os.path.join(BAG_DIR, f)
            break

    if not db3_file or not os.path.exists(metadata_file):
        print(f"[RECORDER] ❌ Error: Bag files not found in {BAG_DIR}.")
        return 1

    print(f"\n[RECORDER] 7. Packaging {ZIP_NAME} with direct-root structure...")
    with zipfile.ZipFile(ZIP_PATH, 'w', zipfile.ZIP_DEFLATED) as zipf:
        zipf.write(metadata_file, arcname="metadata.yaml")
        zipf.write(db3_file, arcname=os.path.basename(db3_file))

    print(f"[RECORDER] Archive created: {ZIP_PATH}")
    print("[RECORDER] Archive contents:")
    with zipfile.ZipFile(ZIP_PATH, 'r') as zipf:
        zipf.printdir()

    print("\n[RECORDER] Bag Information:")
    subprocess.run(["bash", "-c", f"source /opt/ros/humble/setup.bash && ros2 bag info {BAG_DIR}"], check=False)

    print("\n[RECORDER] ✅ Task 2A Rosbag Recording and Packaging COMPLETE!")
    return 0

if __name__ == '__main__':
    sys.exit(main())
