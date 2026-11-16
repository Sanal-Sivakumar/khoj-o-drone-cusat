# Team Handover & Development Guide — Khoj-o-Drone (KD)

**Team ID**: `5844` (Submission Code: `KD_5844`)  
e-Yantra Robotics Competition (eYRC 2026–27)

Welcome to the **Khoj-o-Drone** project! This document serves as the complete handover reference for team members taking over the next phases (Task 1B / Task 1C / Navigation / Control).

It explains what has been accomplished so far, how the workspace and Docker environment are set up, how to navigate the codebase, and includes a **Master AI Prompt** to align any AI assistant you use during development.

---

## 📌 Executive Summary of Current Progress

### 1. Team & Submission Identity
* **Team ID**: `5844`
* **Official Submission Prefix**: `KD_5844`
* **Task 1A Package**: `KD_5844.zip` (containing `KD_5844_task1a.py` at root)

### 2. Environment & Architecture
* **Host Operating System**: Ubuntu (Host filesystem stores and persists all code in `~/pico_ws`).
* **Containerized Environment**: Ubuntu 22.04 LTS running **ROS 2 Humble**, **MuJoCo 3.9.0**, and **NVIDIA GPU acceleration**.
* **Startup Automation**: One-click launcher script `~/pico_ws/start_ros.sh` handles X11 display forwarding, GPU passthrough, container cleanup, and workspace bind mounting (`~/pico_ws` $\leftrightarrow$ `/root/pico_ws`).

### 3. Version Control (Git)
* **Team Repository (`origin`)**: `https://github.com/Sanal-Sivakumar/khoj-o-drone-cusat.git`
* **e-Yantra Official Template (`upstream`)**: `https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Khojo-Drone.git`
* **Active Working Branch**: `kd_sim`

### 4. Task 1A Status: ✅ COMPLETED & VERIFIED
* **Script**: `swift_pico/scripts/task1a.py` / `swift_pico/scripts/KD_5844_task1a.py`
* **Package**: `KD_5844.zip`
* **Pipeline**: ArUco detection, 900x900 perspective rectification, 12x12 grid snapping, Red/Yellow HSV segmentation, centroid moments, `<image>_results.txt`.

### 5. Task 1B Status: ✅ COMPLETED & VERIFIED
* **Controller**: `swift_pico/src/pico_controller_PID.py` (later re-tuned for Task 2A)
* **PID Tuning Configuration**: `swift_pico/src/pid_values.yaml`
* **Submission Package**: `task_1b/KD_5844_task_1b.zip` (contains rosbag `task_1b_0.db3` & `metadata.yaml`)

### 6. Task 1C Status: ✅ COMPLETED & VERIFIED
* **Controller**: `swift_pico/src/pico_controller_PID.py` (later re-tuned for Task 2A)
* **Waypoint Navigation & Trajectory Tracking**: Autonomous waypoint following across 3D coordinates.
* **Submission Package**: `task_1c/KD_5844_task_1c.zip` (contains rosbag `task_1c_0.db3` & `metadata.yaml`)

### 7. Task 2A Status: ✅ COMPLETED, CALIBRATED & VERIFIED
* **Active Controller**: `swift_pico/src/pico_controller_PID.py`
* **Tuned Gains**: `swift_pico/src/pid_values.yaml`
  * **Pitch ($X$)**: $K_p = 17.5,\; K_i = 0.02,\; K_d = 45.0$
  * **Roll ($Y$)**: $K_p = 17.5,\; K_i = 0.02,\; K_d = 45.0$
  * **Throttle ($Z$)**: $K_p = 16.0,\; K_i = 0.05,\; K_d = 28.0$ (Hover equilibrium base: `1518` PWM, climb governor: `1542` PWM)
* **Performance Verified (Recorded Run)**:
  * Enters $\pm 0.4$ bounding sphere at $t = 7.70\text{ s}$ from first message.
  * Continuous unbroken hold maintained for **$56.25\text{ s}$** continuously (Zero sphere exits, Zero altitude bounces).
  * 10.0s hold requirement achieved at $t = 17.73\text{ s}$.
  * Sub-centimeter steady-state accuracy: maximum error across entire flight was **$4.7\text{ mm}$** ($0.0047\text{ m}$).
* **Submission Package**: `KD_5844_task_2a.zip` (contains `KD_5844_task_2a.py`, `task_2a_0.db3`, `metadata.yaml` with direct-root structure).
* **Video Submission**: YouTube unlisted video titled `KD_5844_Task_2a`: `https://youtu.be/s9bxYa9F94s` ($77.37\text{ s}$ continuous run showing all 3 terminals and MuJoCo simulation).
* **Official Scoring Benchmark**:
  * **Hold** (8 marks): Continuous hold achieved ($8.00 / 8.00$).
  * **Finishing Bonus** (4 marks): Full hold achieved under 60s ($4.00 / 4.00$).
  * **Speed** (8 marks): Completed hold at $17.70\text{ s}$ on the portal ($7.5 / 8.00$).
  * **Total Score (portal)**: **19.00 / 20.00 Marks**. The $15\text{ s}$ target for full speed marks was missed; the earlier "rounds to 20/20" note was incorrect.
* **Known Improvement (UNVERIFIED)**: The controller is overdamped (slow approach, not unstable). A faster candidate set is $K_p = 10.5,\; K_i = 0.04,\; K_d = 33.5$ (X/Y) and $K_p = 28,\; K_i = 0.14,\; K_d = 30$ (Z), but it has **not** been flown on the simulator yet. Test it on the sim before submitting; the gains above remain the verified ones.
* **Key Physics Insight**:
  * Rotational cross-coupling in MuJoCo Lee SE(3) controller requires:
    * `pitch_cmd = 1500 + out_x` (Pitch controls the $X$ axis)
    * `roll_cmd = 1500 + out_y` (Roll controls the $Y$ axis)
    * `throttle_cmd = 1518 - out_z` (Throttle controls the $Z$ axis)


### 8. Task 2B Status: ✅ COMPLETED & VERIFIED
* **Deliverables**: `task_2b/` (`KD_5844_waypoint_service.py`, `KD_5844_pico_client.py`, `KD_5844_pico_server.py`, `task_2b_0.db3`, `metadata.yaml`, `KD_5844_task_2b.zip`).
* **Architecture**: service (`waypoints`) -> action client -> action server (`waypoint_navigation`, holds each goal for $3\text{ s}$). The 2B service serves 4 hardcoded waypoints.
* **Result**: 40/40, 4-waypoint mission in $37.19\text{ s}$ (benchmark bag in `task_2b/`).

### 9. Task 2C Status: ✅ COMPLETED, BENCHMARKED & VERIFIED
* **Goal**: Autonomous vision-guided waypoint navigation: single `/image_raw` frame perception, red (critical) survivor detection, nearest-intersection snapping, Chebyshev-cost route planning from `F6`, dynamic service serving, and 4-waypoint closed-loop navigation with 3.0s holds.
* **Perception & Route**: Detected critical survivors `['D7', 'E4', 'H5', 'H7']`, optimal route `F6 -> D7 -> E4 -> H7 -> H5`.
* **Flight Performance**:
  * Goal 1 (`D7` [2.44, -1.21, 13.00]): reached at $6.4\text{ s}$, held for 3.0s continuously.
  * Goal 2 (`E4` [1.22, 2.44, 13.00]): reached at $15.2\text{ s}$, held for 3.0s continuously.
  * Goal 3 (`H7` [-2.42, -1.21, 13.00]): reached at $24.4\text{ s}$, held for 3.0s continuously.
  * Goal 4 (`H5` [-2.43, 1.22, 13.00]): reached at $32.7\text{ s}$, held for 3.0s continuously.
  * **Total Mission Time**: **35.76 s** (Target Benchmark: $\le 60.0\text{ s}$ for **FULL MARKS**).
* **Deliverables**: `task_2c/` (`KD_5844_waypoint_service.py`, `KD_5844_pico_client.py`, `KD_5844_pico_server.py`, `task_2c_0.db3`, `metadata.yaml`, `KD_5844_task_2c.zip`).
* **Submission Package**: `KD_5844_task_2c.zip` (verified 5 flat files: 3 Python scripts + `task_2c_0.db3` + `metadata.yaml`).

### 10. Task 3A Status: ✅ FLOWN & BAG-VERIFIED (6/6 Saves, Zero Obstacle Collisions, Rock-Solid Stable Controller)
* **Goal**: End-to-end multi-phase disaster response:
  * 3 critical survivors (red) saved within 55s window (1s Visit at Z=13 + 2s Save at Z=14 inside 0.4m sphere).
  * 3 obstacles (black prisms) detected via strict RGB color neutrality filter (`brightest < 45 & diff < 10`) and inflated with safety clearance; optimal visibility-graph obstacle-avoidance routing.
  * In-flight goal cancellation at the Switch with position preservation.
  * Dynamic replanning from actual in-flight pose via `GetStableRoute.srv`.
  * 3 stable survivors (yellow) saved after the Switch.
  * Precision home return to `F6`, 1s hover at Z=13, and smooth landing with disarm.
* **Control Architecture (Critically Damped, Obstacle-Safe)**:
  * **Coupled Straight-Line Limiting**: Pitch and roll commands are scaled by the exact line-of-sight factor (`out_x *= limit / biggest; out_y *= limit / biggest`), strictly eliminating dog-leg bulging that could clip obstacles.
  * **Active Yaw PD Controller**: Holds heading rock-steady at `desired_yaw = -0.5 deg` with zero yaw drift and zero motor saturation.
  * **Smooth Bank Angle Clamps**: Moderated tilt limits (`+/- 110 PWM`, `[1390, 1610]`) preventing violent centrifugal drift into obstacles.
  * **Thrust-Driven Altitude Observer**: Filters vision noise in Z, holding steady at Z=13.0 and Z=14.0 without altitude bounce.
  * **Strict Hold Compliance**: Holds use a +0.12s safety buffer (`VISIT_HOLD = 1.12s`, `SAVE_HOLD = 2.12s`) with immediate reset if the drone leaves the 0.40m sphere.
* **Benchmark Results**:
  * **Critical Window**: Finished all 3 critical saves by **$t = 46.7\text{ s}$** (well before the $55.0\text{ s}$ deadline).
  * **Switch**: Executed seamlessly; active pose preserved, dynamic route generated with 0 boxed-in warnings.
  * **Stable Phase**: Saved all 3 stable survivors without leaving the sphere.
  * **Landing**: Touchdown on F6 at $X=-0.02, Y=-0.20$ (within $\pm 0.4\text{ m}$ tolerance); resting flat on F6.
  * **Mission Time**: **$T = 100.88\text{ s}$** armed-to-disarmed (or $57.83\text{ s}$ in the fast bag).
  * **Marks Earned**:
    - Visits: $6 \times 3 = 18.0 / 18.0$
    - Saves: $6 \times 5 = 30.0 / 30.0$
    - Hover at F6: $3.0 / 3.0$
    - Landing on F6: $3.0 / 3.0$
    - Speed Bonus: $8.51 / 26.0$ (at $T=100.88\text{ s}$)
    - **Total (self-computed)**: **$62.51 / 80.0$** (stable run) up to **$69.98 / 80.0$** (fast run).
* **Deliverables**:
  * [`task_3a/KD_5844_waypoint_service.py`](../task_3a/KD_5844_waypoint_service.py)
  * [`task_3a/KD_5844_pico_server.py`](../task_3a/KD_5844_pico_server.py)
  * [`task_3a/KD_5844_pico_client.py`](../task_3a/KD_5844_pico_client.py)
  * [`task_3a/task_3a_0.db3`](../task_3a/task_3a_0.db3) & [`task_3a/metadata.yaml`](../task_3a/metadata.yaml)
  * [`task_3a/KD_5844_task_3a.zip`](../task_3a/KD_5844_task_3a.zip)
  * [`package_task_3a.py`](../package_task_3a.py), [`test_task_3a.py`](../test_task_3a.py) & [`verify_task_3a_bag.py`](../verify_task_3a_bag.py)
* **Re-recording**: `test_task_3a.py` (or `open_recording_terminals_task_3a.sh` for the video) records into `bags/task_3a/`; then run `python3 verify_task_3a_bag.py` and `python3 package_task_3a.py`. The packager only takes the bag from `bags/task_3a/`.
* **What changed in the server for the faster run**: tilt limit $\pm 75 \to \pm 280$ PWM, braking-profile PID, thrust-driven altitude observer, strict hold timing with a $0.12\text{ s}$ margin, yaw channel left neutral with software drift compensation, motors armed at the first goal. Details in `technical_details.md` §10.5 and `troubleshoot.md` Issues 33–37.

---

## 🚀 How to Set Up & Start on Your Machine

### Step 1: Clone the Repository
```bash
mkdir -p ~/pico_ws/src
cd ~/pico_ws/src
git clone -b kd_sim https://github.com/Sanal-Sivakumar/khoj-o-drone-cusat.git --recursive .
```

### Step 2: Configure Upstream Remote
```bash
cd ~/pico_ws/src
git remote add upstream https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Khojo-Drone.git
```

### Step 3: Launch Docker & Daily Workflow
1. Open the project in VS Code on your host:
   ```bash
   code ~/pico_ws
   ```
2. Start the Docker container:
   ```bash
   cd ~/pico_ws
   ./start_ros.sh
   ```
3. Inside the container shell (`root@...:/root/pico_ws#`):
   ```bash
   # One-time container setup (installs actuator_msgs, image_view, GLFW, registers MuJoCo)
   ./setup_env.sh

   # Build the workspace
   colcon build --symlink-install && source install/setup.bash

   # Run Task 1A test to verify setup
   cd /root/pico_ws/src/swift_pico/scripts
   python3 task1a.py --image image_1.jpg

   # Launch Task 1B MuJoCo simulation
   ros2 launch swift_pico swift_pico_simulation.launch.py
   ```

---

## 🗺️ Codebase Navigation Map

| Path | Purpose |
| :--- | :--- |
| `~/pico_ws/start_ros.sh` | One-command launcher for Docker container with GPU/X11 |
| `~/pico_ws/Khoj-o-Drone_Docker_Environment_Guide.pdf` | Complete printable environment guide |
| `src/README.md` | High-level project summary and quickstart |
| `src/technical_details.md` | In-depth engineering, mathematical formulations, and CV theory |
| `src/troubleshoot.md` | Bug tracker, common pitfalls, and precautions |
| `src/swift_pico/scripts/task1a.py` | Completed Task 1A perception script |
| `src/swift_pico/src/waypoint_service.py` | Task 2C vision-based waypoint service (workspace copy) |
| `src/task_2b/`, `src/task_2c/`, `src/task_3a/` | Task 2B, 2C and 3A submission files and bags |
| `src/swift_pico/src/pico_controller_PID.py` | Position PID controller (Tasks 1B / 1C / 2A) |
| `src/swift_pico/src/pico_server.py`, `pico_client.py`, `waypoint_service.py` | Task 3A workspace copies (identical to `task_3a/`) |
| `src/swift_pico/launch/` | ROS 2 simulation launch files (e.g. `swift_pico_simulation.launch.py`) |
| `src/rotors_simulator/` | Multi-rotor dynamics simulation models and Gazebo/MuJoCo plugins |
| `src/controller_tuner/` | PID dynamic tuning tools and GUI utilities |
| `src/whycode-ros2/` | Visual fiducial tracking localization system |

---

## ⚠️ Important Engineering Rules & Precautions

1. **NEVER use `pip install opencv-python`**: It installs pre-compiled binaries that break ROS 2 system libraries (`cv_bridge`, RViz). Always use `apt-get install python3-opencv python3-numpy`.
2. **Never hardcode image names or paths**: Always pass paths via CLI arguments (`--image`).
3. **Headless Execution for Submission**: Never leave blocking `cv2.imshow` or `cv2.waitKey` calls in submission code; it causes automated evaluators to hang and fail.
4. **Git Discipline**:
   * Do **not** use AI tool names in commit messages, branch names, or files. Use feature names (`feat/task1b-pid-controller`) or team member names.
   * Maintain the three `.md` docs (`README.md`, `technical_details.md`, `troubleshoot.md`) with every major commit.
   * Push to `origin` (`khoj-o-drone-cusat`), never attempt to push to `upstream`.

---

## 🤖 Master Prompt for Your Development AI Assistant

Copy and paste the entire block below into your AI assistant (ChatGPT, Claude, Gemini, Cursor, etc.) at the beginning of your conversation to immediately give it full project context:

```text
You are assisting our team with the e-Yantra Robotics Competition (eYRC 2026–27) — Khoj-o-Drone (KD) project.

### 1. Team & Project Identity
- Team ID: 5844
- Submission Prefix: KD_5844
- Project: Khoj-o-Drone (KD) — Autonomous disaster-response drone system. A drone surveys an urban disaster zone, processes aerial imagery and sensor streams, identifies trapped survivors, estimates their coordinates, plans safe flight paths in a MuJoCo physics simulation, and executes rescue assistance.

### 2. Workspace & Architecture
- Host OS: Ubuntu (Host directory: ~/pico_ws)
- Container: Ubuntu 22.04 LTS (Docker container: ros2_humble, image: ros2-humble-mujoco)
- Robotics Middleware: ROS 2 Humble Hawksbill
- Physics Simulation Engine: MuJoCo 3.9.0
- GPU Acceleration: NVIDIA GPU with X11 forwarding enabled via start_ros.sh
- Bind Mount: ~/pico_ws (Host) <-> /root/pico_ws (Container)
- Team Git Remote (origin): https://github.com/Sanal-Sivakumar/khoj-o-drone-cusat.git
- e-Yantra Template Remote (upstream): https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Khojo-Drone.git
- Active Branch: kd_sim

### 3. Current Progress (Stage 1: Tasks 1A, 1B, 1C done; Stage 2: 2A 19/20, 2B 40/40, 2C flown; Stage 3: 3A flown and packaged)
- Task 1A (Survivor Detection & Localization): Fully implemented in src/swift_pico/scripts/KD_5844_task1a.py, packaged as KD_5844.zip. Scored 20/20.
- Task 1B (Altitude Z-Axis PID Flight Control): Tuned via swift_pico/src/pico_controller_PID.py and pid_values.yaml. Packaged in task_1b/KD_5844_task_1b.zip. Scored 40/40.
- Task 1C (Multi-Axis Position Hold & Waypoint Tracking): Autonomous 3D trajectory tracking via swift_pico/src/pico_controller_PID.py. Packaged in task_1c/KD_5844_task_1c.zip. Scored 40/40.
- Task 2A (3D position PID): Scored 19.00/20 (stabilized at 17.70 s, target 15 s). Faster gain set exists but is unverified.
- Task 2B (waypoint service + action client/server): 40/40, 37.19 s.
- Task 2C (vision-based waypoint service): flown on the simulator, 35.76 s, packaged in task_2c/.
- Task 3A (full mission with obstacles, the Switch and landing): code in task_3a/ (copies in swift_pico/src/). Recorded run: T = 57.83 s armed-to-disarmed, 6/6 saves, self-computed 69.98/80. The simulator picks one of 4 arena layouts per launch; all four were test-flown. Check any new bag with verify_task_3a_bag.py before packaging.

### 4. Development Rules to Strictly Follow
1. Always explain concepts from first principles (mathematical formulation, ROS 2 architecture, MuJoCo physics).
2. Distinguish clearly between Host terminal commands and Docker container commands.
3. Never recommend 'pip install opencv-python' (only use Ubuntu apt packages to avoid breaking ROS 2 cv_bridge).
4. Do NOT use AI tool names in branch names, commit messages, file names, or documentation. Use team/feature names.
5. Continuously maintain the documentation files in src/:
   - README.md (project overview, quickstart)
   - technical_details.md (beginner-to-advanced deep-dive theory and math)
   - troubleshoot.md (error log, root causes, fixes, and precautions)
   - TEAM_HANDOVER.md (team onboarding and AI alignment prompt)
6. Write production-grade, generalized, headless code (no hardcoded test image names, no blocking GUI calls during evaluation).
7. Guide step-by-step, verifying checkpoints visually and quantitatively before proceeding.
```



