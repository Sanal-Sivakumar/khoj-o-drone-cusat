# Khoj-o-Drone (KD) — eYRC 2026–27

**Autonomous AI-Powered Disaster-Response Drone System**  
e-Yantra Robotics Competition (eYRC 2026–27)  
**Team ID**: `5844` (Submission Prefix: `KD_5844`)

---

## 🚁 Project Overview

Khoj-o-Drone (KD) is an autonomous drone system designed for rapid search-and-rescue operations in disaster-stricken urban zones (collapsed buildings, debris, and trapped survivors).

The system autonomously surveys the disaster field, captures visual and sensor data, identifies survivors using computer vision, maps their precise coordinates, plans safe collision-free paths, and executes rescue-assistance missions.

```text
Disaster Zone ──► Drone / Sensors ──► Data Acquisition ──► Computer Vision & AI ──► Survivor Localization ──► Navigation & Rescue
```

---

## 🏛️ System Architecture

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│ HOST MACHINE (Ubuntu 26.04 LTS)                                             │
│ • VS Code & Git Version Control                                             │
│ • Workspace Directory: ~/pico_ws/src                                        │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ Bind Mount (-v ~/pico_ws:/root/pico_ws)
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ DOCKER CONTAINER (Ubuntu 22.04 LTS — ROS 2 Humble)                          │
│                                                                             │
│   ┌──────────────────────────┐             ┌───────────────────────────┐    │
│   │   MuJoCo 3.9 Physics     │             │   Computer Vision & AI    │    │
│   │   • Drone Dynamics       │             │   • ArUco Rectification   │    │
│   │   • Sensor Simulation    │             │   • Survivor Detection    │    │
│   └────────────┬─────────────┘             └─────────────┬─────────────┘    │
│                │                                         │                  │
│                ▼                                         ▼                  │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                     ROS 2 Humble Middleware                         │   │
│   │   Nodes • Topics • Services • Actions • Transforms (TF2)            │   │
│   └──────────────────────────────────┬──────────────────────────────────┘   │
│                                      │                                      │
│                                      ▼                                      │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                     Autonomous Flight Control                       │   │
│   │   State Estimator ──► Trajectory Planner ──► PID / Geometric Control │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 📂 Repository Structure

```text
pico_ws/
├── start_ros.sh                      # One-click Docker environment launcher
├── setup_env.sh                      # One-click container dependency installer
├── Khoj-o-Drone_Docker_Environment_Guide.pdf # Complete printable environment guide
└── src/
    ├── README.md                     # Project overview and quickstart (this file)
    ├── technical_details.md          # In-depth engineering, math, and CV guide
    ├── troubleshoot.md               # Bug tracker, solutions, and precautions
    │
    ├── swift_pico/                   # Core drone autonomy and perception package
    │   ├── scripts/
    │   │   ├── task1a.py             # Task 1A: Survivor detection & localization
    │   │   ├── KD_5844_task1a.py      # Official Task 1A submission script (Team 5844)
    │   │   ├── KD_5844.zip           # Submission ZIP package
    │   │   └── image_1.jpg           # Sample disaster zone survey photograph
    │   ├── src/                      # Flight controllers (1B, 1C, 2A) + waypoint_service.py (2C)
    │   ├── launch/                   # ROS 2 simulation launch files
    │   └── config/                   # Simulation parameters and bridge configs
    │
    ├── task_2b/                      # Task 2B submission files + bag
    ├── task_2c/                      # Task 2C service, client, server (bag pending)
    ├── rotors_simulator/             # Multi-rotor simulation models & controllers
    ├── controller_tuner/             # Dynamic PID tuning utilities & GUIs
    ├── whycode-ros2/                 # Localization fiducial tracking system
    ├── mav_comm/                     # Standard micro-aerial-vehicle ROS message definitions
    └── swift_pico_description/       # 3D URDF / Meshes for Swift Pico Drone
```

---

## 🚀 Quick Start Guide

### 1. Daily Development Routine
1. Open the project on your host:
   ```bash
   code ~/pico_ws
   ```
2. Start the isolated ROS 2 Humble + MuJoCo container:
   ```bash
   cd ~/pico_ws
   ./start_ros.sh
   ```
3. Inside the container shell (`root@...:/root/pico_ws#`):
   * **First-time setup / dependency install**:
     ```bash
     ./setup_env.sh
     ```
   * **Build workspace**:
     ```bash
     colcon build --symlink-install && source install/setup.bash
     ```
   * **Run Task 1A Perception Pipeline**:
     ```bash
     cd /root/pico_ws/src/swift_pico/scripts
     python3 task1a.py --image image_1.jpg
     ```
   * **Launch Task 1B MuJoCo Simulation**:
     ```bash
     ros2 launch swift_pico swift_pico_simulation.launch.py
     ```

---

---

## 🎯 Task 1A: Survivor Detection & Localization

Task 1A implements the core image processing and computer vision pipeline to extract survivor locations from an aerial photograph of the disaster arena.

### Pipeline Execution:
```bash
# Inside Docker container:
cd /root/pico_ws/src/swift_pico/scripts
python3 task1a.py --image image_1.jpg
```

### Seven-Stage Processing Pipeline:
1. **Corner Fiducials**: ArUco detection (`DICT_4X4_250`, IDs 80, 85, 90, 95).
2. **Perspective Rectification**: Homography warping to $900 \times 900\text{ px}$ canvas using Euclidean inner-corner selection.
3. **Cartesian Reference Grid**: $12 \times 12$ analytical grid ($75\text{ px}$ cell pitch, 121 intersections).
4. **Alphanumeric Naming**: Grid coordinate system (`A1` top-left to `K11` bottom-right).
5. **Color Segmentation**: HSV masks for Red (`Critical Survivors`) and Yellow (`Stable Survivors`) with morphological filtering.
6. **Centroid Reduction**: Spatial image moments ($M_{10}/M_{00}, M_{01}/M_{00}$) with zero-area guards.
7. **Intersection Snapping & File Generation**: Writes `<image_name>_results.txt` matching exact official formatting.

---

## 🚁 Task 1B: Altitude (Z-Axis) PID Flight Control

Task 1B implements closed-loop vertical altitude stabilization in the MuJoCo physics simulation using WhyCode visual pose feedback.

### Execution Workflow (3 Terminals):
```bash
# Terminal 1: MuJoCo Physics & WhyCode Pose Estimation
ros2 launch swift_pico swift_pico_simulation.launch.py

# Terminal 2: Flight Controller Node
ros2 run swift_pico task_1b_controller

# Terminal 3: Dynamic PID Tuning GUI (Optional / Tuning Mode)
ros2 launch pid_tune pid_tune_drone.launch.py
```

* **Topic**: `/drone_command` (publishing throttle PWM based on `/throttle_pid` feedback).
* **Tolerance**: Altitude error $|e_z(t)| \le \pm 0.4\text{ m}$ for $10.0\text{ s}$ continuous hold.
* **Submission Package**: [`src/task_1b/KD_5844_task_1b.zip`](file:///home/sanal-sivakumar/pico_ws/src/task_1b/KD_5844_task_1b.zip).

---

## 🧭 Task 1C: Multi-Axis (X, Y) Position Hold & Waypoint Navigation

Task 1C extends control to the horizontal Pitch ($X$) and Roll ($Y$) axes, enabling autonomous 3D waypoint navigation and hover stabilization across multiple arena waypoints.

### Execution Workflow:
```bash
# Terminal 1: Simulation Stack
ros2 launch swift_pico swift_pico_simulation.launch.py

# Terminal 2: Multi-Axis Flight Controller
ros2 run swift_pico task_1c_controller
```

* **Topics**: `/pitch_pid` ($X$-axis tilt), `/roll_pid` ($Y$-axis tilt), `/throttle_pid` ($Z$-axis lift).
* **Submission Package**: [`src/task_1c/KD_5844_task_1c.zip`](file:///home/sanal-sivakumar/pico_ws/src/task_1c/KD_5844_task_1c.zip).

---

## 🎯 Task 2A: Autonomous 3D Position Hold (PID Controller)

Task 2A implements an autonomous closed-loop 3D position controller to fly the Swift Pico drone from $(0.0, 0.0, 15.0)$ to the setpoint $(1.5, -1.5, 13.0)$ and hold stably within a $\pm 0.4\text{ m}$ bounding sphere for at least $10\text{ s}$ straight.

### Execution Workflow (3 Terminals):
```bash
# Terminal 1: Simulation Stack
ros2 launch swift_pico swift_pico_simulation.launch.py

# Terminal 2: Position PID Controller
ros2 run swift_pico pico_controller_PID.py

# Terminal 3: Interactive Tuning GUI (Optional)
ros2 launch pid_tune pid_tune_drone.launch.py
```

* **Target Setpoint**: $(1.5, -1.5, 13.0)$ (Bottom-left quadrant near ground triangle marker `E7/D7`).
* **Physical Kinematics Mapping**:
  * Pitch command $\to$ controls $X$ axis (`pitch_cmd = 1500 + out_x`)
  * Roll command $\to$ controls $Y$ axis (`roll_cmd = 1500 + out_y`)
  * Throttle command $\to$ controls $Z$ axis (`throttle_cmd = 1518 - out_z`, with hover equilibrium at `1518` PWM)
* **Submitted Gains** (verified, [`pid_values.yaml`](./swift_pico/src/pid_values.yaml)):
  * Pitch ($X$): $K_p = 17.5,\; K_i = 0.02,\; K_d = 45.0$
  * Roll ($Y$): $K_p = 17.5,\; K_i = 0.02,\; K_d = 45.0$
  * Throttle ($Z$): $K_p = 16.0,\; K_i = 0.05,\; K_d = 28.0$
* **Faster candidate (not yet verified on the sim)**: X/Y $10.5 / 0.04 / 33.5$, Z $28 / 0.14 / 30$. The submitted set is overdamped (stabilized at $17.70\text{ s}$ vs the $15\text{ s}$ target).

---

## 🗺️ Task 2B: Waypoint Service + Action Client/Server

Three nodes: `waypoint_service` (serves `GetWaypoints` on `waypoints`), `pico_client` (fetches the waypoints and sends them one by one) and `pico_server` (`NavToWaypoint` action on `waypoint_navigation`, holds each goal for $3\text{ s}$). Final files are in `task_2b/`; the 4-waypoint mission ran in $37.19\text{ s}$ (40/40).

---

## 🔴 Task 2C: Vision-Based Waypoint Service

The 2C service replaces the hardcoded waypoints with a route computed from one `/image_raw` frame: it detects the **critical (red)** survivors with the Task 1A pipeline, looks up each intersection's coordinates in the official grid table, and plans a short route starting at F6. The 2B client and server are unchanged.

```bash
# Terminal 1: simulation stack (as in 2A/2B)
# Terminal 2: waypoint service (reads one image, then serves the route)
python3 task_2c/KD_5844_waypoint_service.py
# Terminal 3 / 4: server and client (unchanged from 2B)
python3 task_2c/KD_5844_pico_server.py
python3 task_2c/KD_5844_pico_client.py
```

**Status**: ✅ Fully implemented, flight-tested, benchmarked, and verified on simulator.
* **Perception**: Detected critical survivors `['D7', 'E4', 'H5', 'H7']`
* **Route**: Optimal Chebyshev tour `F6 -> D7 -> E4 -> H7 -> H5`
* **Mission Time**: **35.76 s** ($\le 60.0\text{ s}$ target for Full Marks)
* **Rosbag & Package**: `task_2c/` and [`KD_5844_task_2c.zip`](./task_2c/KD_5844_task_2c.zip) (5 flat files verified)

---

## ⚡ Task 3A: End-to-End Autonomous Search, Rescue & Obstacle Avoidance

Task 3A is the full disaster mission combining autonomous vision perception, obstacle avoidance, critical and stable phases, dynamic in-flight replanning at the Switch, two-tiered saving maneuvers, and precision landing back at `F6`.

```text
Take-off at F6 (t=0) ──► Critical Phase (t <= 55s) ──► The Switch (Mid-Flight Cancel & Replan)
                                                                 │
                                                                 ▼
Landed on F6 (Mission Complete) ◄── Home Hover (Z=13, 1s) ◄── Stable Phase (All 3 Stable Survivors)
```

### Execution Workflow (4 Terminals):
```bash
# Terminal 1: Simulation Stack (Task 3 arena)
ros2 launch swift_pico swift_pico_simulation.launch.py

# Terminal 2: Mission Waypoint Service
python3 task_3a/KD_5844_waypoint_service.py

# Terminal 3: Mission Client
python3 task_3a/KD_5844_pico_client.py

# Terminal 4: Waypoint Action Server (starts flight)
python3 task_3a/KD_5844_pico_server.py
```

### Key Mission Rules & Architecture:
* **Saving Maneuver**: 1.0s Visit at $Z = 13.0$, followed by 2.0s Save at $Z = 14.0$, inside a Euclidean sphere of radius $0.4\text{ m}$.
* **Obstacle Clearance**: Black prism obstacles inflated with $32\text{ px}$ safety clearance ($\approx 0.51\text{ m}$). Straight-line shortcutting ensures zero unnecessary waypoints and continuous velocity.
* **The Switch**: Triggered upon completion of all 3 critical saves or at $t = 54.5\text{ s}$ timer expiration. In-flight goal is canceled, holding position while `replan_from_pose` plans the stable route through the 3 stable survivors and home to `F6`.
* **Home & Landing**: Returns to `F6`, hovers at $Z = 13.0$ for 1.0s, smoothly descends with horizontal PID active, and disarms on ground contact.
* **Deliverables**: [`task_3a/KD_5844_waypoint_service.py`](../task_3a/KD_5844_waypoint_service.py), [`task_3a/KD_5844_pico_client.py`](../task_3a/KD_5844_pico_client.py), [`task_3a/KD_5844_pico_server.py`](../task_3a/KD_5844_pico_server.py), `ReplanFromPose.srv`.

---

## 🏆 Official Competition Results & Milestones

| Task | Module / Scope | Official Score | Hold Time Achieved | Total Time | Target Benchmark | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **Task 1A** | Survivor CV & Localization | **20 / 20** | N/A | Instant (<100ms) | Accurate coordinates | ✅ **PASSED** |
| **Task 1B** | Altitude (Z) PID Tuning | **40 / 40** | 10.00 s / 10.00 s | **10.03 s** | < 15.0 s | ✅ **PASSED** |
| **Task 1C** | 3D Waypoint Navigation | **40 / 40** | 10.00 s / 10.00 s | **14.03 s** | < 15.0 s | ✅ **PASSED** |
| **Task 2A** | 3D Position PID Stabilization | **19.00 / 20** | **10.00 s** / 10.00 s | **17.70 s** | < 15.0 s (Max Marks) | ⚠️ **PASSED, speed marks lost** |
| **Task 2B** | Waypoint Service + Action Client/Server | **40 / 40** | 3.00 s per waypoint | **37.19 s** | 4 waypoints | ✅ **PASSED** |
| **Task 2C** | Vision-Based Waypoint Service | **Benchmark Verified** | 3.00 s per waypoint | **35.76 s** | < 60.0 s (Full Marks) | ✅ **PASSED** |

---

## 📚 Documentation Index
* 📖 [**Technical Details & Theory Guide (`technical_details.md`)**](./technical_details.md)
* 🛠️ [**Troubleshooting & Bug Log (`troubleshoot.md`)**](./troubleshoot.md)
* 🤝 [**Team Handover & AI Guide (`TEAM_HANDOVER.md`)**](./TEAM_HANDOVER.md)

