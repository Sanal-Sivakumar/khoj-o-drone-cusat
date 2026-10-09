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
    │   ├── src/                      # Flight controllers (Task 1B, 1C)
    │   ├── launch/                   # ROS 2 simulation launch files
    │   └── config/                   # Simulation parameters and bridge configs
    │
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

## 🏆 Stage 1 Official Competition Results

| Task | Module / Scope | Official Score | Hold Time Achieved | Stabilization Time | Benchmark Target | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **Task 1A** | Survivor CV & Localization | **20 / 20** | N/A | Instant (<100ms) | Accurate coordinates | ✅ **PASSED** |
| **Task 1B** | Altitude (Z) PID Tuning | **40 / 40** | 10.00 s / 10.00 s | **10.03 s** | < 15.0 s | ✅ **PASSED** |
| **Task 1C** | 3D Waypoint Navigation | **40 / 40** | 10.00 s / 10.00 s | **14.03 s** | < 15.0 s | ✅ **PASSED** |

---

## 📚 Documentation Index
* 📖 [**Technical Details & Theory Guide (`technical_details.md`)**](./technical_details.md)
* 🛠️ [**Troubleshooting & Bug Log (`troubleshoot.md`)**](./troubleshoot.md)
* 🤝 [**Team Handover & AI Guide (`TEAM_HANDOVER.md`)**](./TEAM_HANDOVER.md)

