# Khoj-o-Drone (KD) — eYRC 2026–27

**Autonomous AI-Powered Disaster-Response Drone System**  
e-Yantra Robotics Competition (eYRC 2026–27)

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

## 📚 Documentation Index
* 📖 [**Technical Details & Theory Guide (`technical_details.md`)**](./technical_details.md)
* 🛠️ [**Troubleshooting & Bug Log (`troubleshoot.md`)**](./troubleshoot.md)
