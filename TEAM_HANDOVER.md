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
* **Script**: `swift_pico/scripts/task1a.py`
* **Pipeline Implemented**:
  1. ArUco Marker Detection (`DICT_4X4_250`, IDs `80`, `85`, `90`, `95`).
  2. Homography Perspective Rectification to a normalized $900 \times 900\text{ px}$ top-down canvas using Euclidean inner-corner selection.
  3. $12 \times 12$ analytical reference grid ($75.0\text{ px}$ cell size, 121 intersections).
  4. Alphanumeric coordinate system (`A1` top-left to `K11` bottom-right).
  5. HSV color segmentation for `Critical Survivors` (Red Triangles) and `Stable Survivors` (Yellow Circles) with morphological noise cleanup.
  6. Spatial image moment centroid reduction ($M_{10}/M_{00}, M_{01}/M_{00}$) with division-by-zero protection.
  7. Nearest intersection snapping with boundary safety guards.
  8. Output file generation (`<image_name>_results.txt`) matching exact competition specifications character-for-character.

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
   # Build the workspace
   colcon build && source install/setup.bash

   # Run Task 1A test to verify setup
   cd /root/pico_ws/src/swift_pico/scripts
   python3 task1a.py --image image_1.jpg
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
| `src/swift_pico/src/task_1b_controller/` | Task 1B flight controller package (PID / Waypoint navigation) |
| `src/swift_pico/src/task_1c_controller/` | Task 1C advanced autonomy controller |
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

### 3. Current Progress (Task 1A Completed & Packaged)
- Task 1A (Survivor Detection & Localization) is fully implemented in src/swift_pico/scripts/task1a.py and official submission script KD_5844_task1a.py.
- Packaged as KD_5844.zip (containing only KD_5844_task1a.py at root).
- Implemented with cross-version OpenCV compatibility (supports OpenCV 4.5.4 DetectorParameters_create as well as OpenCV 4.7+ / 5.x DetectorParameters).
- It detects ArUco corner markers (DICT_4X4_250: IDs 80, 85, 90, 95 or generic quadrant-sorted markers), performs 900x900 perspective rectification, generates a 12x12 grid (121 named intersections from A1 to K11), extracts Red (Critical) and Yellow (Stable) survivors via HSV segmentation and spatial image moments, snaps them to nearest intersections, and writes <image_name>_results.txt.

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


