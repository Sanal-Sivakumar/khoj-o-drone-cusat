# Technical Documentation — Khoj-o-Drone (KD)

Welcome to the comprehensive technical documentation for **Khoj-o-Drone**. This document explains all architectural layers, mathematical foundations, computer vision pipelines, and ROS 2 communication patterns in clear first principles so anyone can understand how and why every component works.

---

## Table of Contents
1. [Core Architectural Concept](#1-core-architectural-concept)
2. [Host vs. Docker Virtualization Layer](#2-host-vs-docker-virtualization-layer)
3. [Task 1A: Computer Vision & Perspective Rectification](#3-task-1a-computer-vision--perspective-rectification)
   - [Step 1: ArUco Marker Detection](#step-1-aruco-marker-detection)
   - [Step 2: Perspective Transform & Homography Matrix](#step-2-perspective-transform--homography-matrix)
   - [Coordinate Frame Transformation](#coordinate-frame-transformation)
4. [ROS 2 Middleware Layer](#4-ros-2-middleware-layer)
5. [MuJoCo Physics Simulation Engine](#5-mujoco-physics-simulation-engine)

---

## 1. Core Architectural Concept

Autonomous robotic search-and-rescue requires solving three fundamental robotics problems:
1. **Perception**: What does the world look like? Where are the survivors, obstacles, and reference landmarks?
2. **Localization & Mapping**: Where is the drone in world coordinates, and where are the detected targets relative to the environment?
3. **Planning & Control**: How should the drone fly to inspect targets safely and deliver assistance?

---

## 2. Host vs. Docker Virtualization Layer

### Why use Docker?
* **Host Machine**: Ubuntu 26.04 LTS (Modern Linux kernel, latest drivers).
* **Target Robotics Environment**: ROS 2 Humble Hawksbill (designed and guaranteed stable on Ubuntu 22.04 LTS Jammy).

Instead of dual-booting or degrading the host operating system, Docker runs an isolated **Ubuntu 22.04 userspace container** using the host's Linux kernel and NVIDIA GPU drivers.

### The Bind Mount Mechanism
A **Bind Mount** (`-v /home/.../pico_ws:/root/pico_ws`) directly maps a folder on the host hard drive into the container file tree:
* **Host Side**: Open VS Code, edit Python scripts, manage Git branches.
* **Container Side**: Run `colcon build`, execute ROS 2 nodes, launch MuJoCo GPU simulations.
* **Result**: Zero loss of data when containers start, exit, or restart.

---

## 3. Task 1A: Computer Vision & Perspective Rectification

In disaster response, aerial cameras rarely capture an arena from a perfect $90^\circ$ perpendicular (nadir) angle. Drone roll, pitch, and altitude variations cause **perspective distortion** (a square playing field appears as a skewed quadrilateral).

```text
Skewed Camera View (Photograph)                Rectified Top-Down Map (900 x 900)
       [80] ────── [85]                               [80] ────────── [85]
       /             \                                 │                │
      /  Disaster     \          Perspective           │    Disaster    │
     /     Zone        \        Transformation         │      Zone      │
    /                   \     ─────────────────►       │                │
   [95] ─────────────── [90]                           [95] ────────── [90]
```

---

### Step 1: ArUco Marker Detection

#### What are ArUco Markers?
ArUco markers are synthetic square fiducial markers. Each marker contains:
1. A wide black outer border for fast edge and contour extraction.
2. An inner binary matrix encoding a unique numeric ID using error-correcting codes.

#### Dictionary: `DICT_4X4_250`
* `4x4`: The inner binary grid resolution is $4 \times 4$ bits.
* `250`: The dictionary contains 250 distinct IDs.
* **Required Arena Markers**: `80` (Top-Left), `85` (Top-Right), `90` (Bottom-Right), `95` (Bottom-Left).

#### Detection Mechanism:
1. **Adaptive Thresholding**: Converts the color image to binary to separate black borders from the bright background.
2. **Contour Extraction & Polygon Approximation**: Extracts closed 4-vertex quadrilateral contours.
3. **Canonical Extraction**: Re-samples the inner $4 \times 4$ grid into a canonical binary matrix and checks against the dictionary to decode the marker ID.

---

### Step 2: Perspective Transform & Homography Matrix

To transform the skewed arena into a square $900 \times 900$ pixel coordinate frame:

#### 1. The 16-to-4 Corner Selection Algorithm
Each of the 4 markers contains 4 corners (16 total candidate points). The playing field boundary corresponds to the **inner corner** of each marker (the corner pointing towards the center of the arena).

To make this robust against any drone rotation or distortion:
1. Compute the centroid (approximate center) of the arena by averaging marker centers:
   $$\bar{C} = \frac{1}{4} \sum_{i \in \{80, 85, 90, 95\}} \text{Center}(M_i)$$
2. For each marker $M_i$, choose the vertex $P_k$ with the minimum Euclidean distance to $\bar{C}$:
   $$P_{\text{inner}}(M_i) = \arg\min_{P \in M_i} \| P - \bar{C} \|_2$$

#### 2. Homography Matrix ($H$)
A projective transform relates 2D points in the camera frame $\mathbf{x} = [x, y, 1]^T$ to points in the rectified world map $\mathbf{x}' = [x', y', 1]^T$ via a $3 \times 3$ Homography matrix $H$:

$$\mathbf{x}' \sim H \mathbf{x} = \begin{bmatrix} h_{11} & h_{12} & h_{13} \\ h_{21} & h_{22} & h_{23} \\ h_{31} & h_{32} & h_{33} \end{bmatrix} \begin{bmatrix} x \\ y \\ 1 \end{bmatrix}$$

Using 4 corresponding point pairs:
* Point 0: Inner Corner of Marker 80 $\rightarrow (0, 0)$
* Point 1: Inner Corner of Marker 85 $\rightarrow (900, 0)$
* Point 2: Inner Corner of Marker 90 $\rightarrow (900, 900)$
* Point 3: Inner Corner of Marker 95 $\rightarrow (0, 900)$

OpenCV solves the linear system using Singular Value Decomposition (`cv2.getPerspectiveTransform`) and applies bilinear interpolation (`cv2.warpPerspective`) to generate the final $900 \times 900$ image.

---

### Step 3: Grid Coordinate System Formulation

Once the arena is warped into the canonical $900 \times 900$ pixel canvas, the physical arena is mapped into a discrete $12 \times 12$ Cartesian grid.

#### 1. Mathematical Dimensions
* **Canvas Resolution**: $900 \times 900$ pixels.
* **Grid Resolution**: $12 \times 12$ square cells (144 total cells).
* **Cell Pitch ($S$)**:
  $$S = \frac{900}{12} = 75.0\text{ pixels / cell}$$
* **Grid Lines**: 11 interior horizontal lines and 11 interior vertical lines ($11 \times 11 = 121$ line intersections).
* **Line Coordinates**:
  $$x_j = j \cdot 75\text{ px}, \quad y_i = i \cdot 75\text{ px} \quad \text{for } i, j \in \{0, 1, \dots, 12\}$$

#### 2. Coordinate Transformations: Pixel $\leftrightarrow$ Grid Cell
* **Forward (Pixel $(x, y) \to$ Grid Cell $(r, c)$)**:
  $$r = \left\lfloor \frac{y}{75} \right\rfloor, \quad c = \left\lfloor \frac{x}{75} \right\rfloor \quad \text{where } r, c \in [0, 11]$$
* **Inverse (Grid Cell $(r, c) \to$ Cell Center $(x_{\text{center}}, y_{\text{center}})$)**:
  $$x_{\text{center}} = (c + 0.5) \times 75, \quad y_{\text{center}} = (r + 0.5) \times 75$$

#### 3. Analytical Grid vs. Hough Line Detection
* **Analytical Grid (Chosen Method)**: Leverages the homography-rectified $900 \times 900$ geometry. $O(1)$ computation, zero false detections, and immune to line occlusions (trees, black building cubes, survivor markers).
* **Visual Verification**: Overlaying computed lines at $k \times 75$ px directly tests the sub-pixel precision of the Step 2 perspective transform.

---

### Step 4: Intersection Coordinate System & Naming Convention

Survivors and disaster landmarks in Khoj-o-Drone stand **on the intersections of grid lines**, rather than inside grid cells.

#### 1. Naming Standard
Every intersection location is uniquely defined by:
* **Column Letter**: `A` to `K` (corresponding to column line indices $j = 1 \dots 11$, from left to right).
* **Row Number**: `1` to `11` (corresponding to row line indices $i = 1 \dots 11$, from top to bottom).
* **Format**: Pure alphanumeric concatenation with **no spaces, separators, or zero-padding** (e.g. `A1`, `C2`, `K11`).
* **Bounds**:
  * Top-Left intersection: `A1` at $(75, 75)\text{ px}$
  * Bottom-Right intersection: `K11` at $(825, 825)\text{ px}$
  * Total Intersection Count: $11 \times 11 = 121$

#### 2. Intersection Engine (`ArenaCoordinateSystem`)
* **Continuous Pixel-to-Intersection Mapping**:
  Given any pixel coordinate $(x, y)$ from a detected survivor centroid:
  $$j = \text{clip}\left(\text{round}\left(\frac{x}{75}\right), 1, 11\right), \quad i = \text{clip}\left(\text{round}\left(\frac{y}{75}\right), 1, 11\right)$$
  $$\text{Name} = \text{chr}(65 + j - 1) + \text{str}(i)$$
* **Euclidean Quantization Error**:
  $$e = \sqrt{(x - j \times 75)^2 + (y - i \times 75)^2}$$
  Allows evaluating if a detected centroid is closely aligned with a physical line intersection ($e \le 15\text{ px}$).

---

## 4. ROS 2 Middleware Layer

ROS 2 (Robot Operating System 2) serves as the computational nervous system:
* **Nodes**: Modular single-purpose processes (e.g., `perception_node`, `controller_node`, `whycode_node`).
* **Topics**: Asynchronous publisher/subscriber data buses (e.g., `/camera/image_raw`, `/drone/odometry`).
* **Services**: Synchronous request/response interactions (e.g., `/arm_drone`, `/set_mode`).
* **TF2 (Transforms)**: Keeps track of coordinate frames over time (`world` $\to$ `drone_base_link` $\to$ `camera_link`).

---

## 5. MuJoCo Physics Simulation Engine

**MuJoCo** (Multi-Joint dynamics with Contact) simulates:
* **Continuous Multi-Body Dynamics**: Quadrotor inertia, motor thrust, aerodynamic drag, gravity.
* **Contacts & Collisions**: Rigid surface interactions with debris and buildings.
* **Sensors**: Simulated IMU (accelerometer/gyroscope), downward rangefinder, and RGB camera streams mapped directly to ROS 2 topics.


