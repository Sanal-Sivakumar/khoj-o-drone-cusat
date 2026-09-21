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
   * **Build workspace**:
     ```bash
     colcon build && source install/setup.bash
     ```
   * **Run Task 1A Perception Pipeline**:
     ```bash
     cd /root/pico_ws/src/swift_pico/scripts
     python3 task1a.py --image image_1.jpg
     ```

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

## 📚 Documentation Index
* 📖 [**Technical Details & Theory Guide (`technical_details.md`)**](./technical_details.md)
* 🛠️ [**Troubleshooting & Bug Log (`troubleshoot.md`)**](./troubleshoot.md)
* 🤝 [**Team Handover & AI Guide (`TEAM_HANDOVER.md`)**](./TEAM_HANDOVER.md)


