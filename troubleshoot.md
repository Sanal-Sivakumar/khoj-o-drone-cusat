# Troubleshooting & Knowledge Base — Khoj-o-Drone (KD)

This document catalogs every error, environment conflict, and bug encountered during development, along with root-cause analyses, exact fixes, and engineering precautions.

---

## 📋 Quick Diagnostic Index

| Issue / Error Message | Root Cause | Solution Reference |
| :--- | :--- | :--- |
| `ModuleNotFoundError: No module named 'cv2'` | Command executed on Host terminal instead of inside Docker | [Issue 1](#issue-1-modulenotfounderror-no-module-named-cv2) |
| `cv_bridge` or ROS 2 GUI crashes with Qt errors | `pip install opencv-python` polluted system libraries | [Issue 2](#issue-2-why-never-use-pip-install-opencv-python) |
| `Error response from daemon: Conflict. The container name "ros2_humble" is already in use` | Old stopped container instance still registered with Docker daemon | [Issue 3](#issue-3-docker-container-name-conflict) |
| `Error: Cannot open display: :0` / `cv2.imshow` crash | Host X11 display server rejecting client connections from container | [Issue 4](#issue-4-x11-gui-window-access-denied) |
| `AttributeError: module 'cv2.aruco' has no attribute 'detectMarkers'` | OpenCV ArUco API breaking changes between OpenCV 4.6 and 4.7+ / 5.x | [Issue 5](#issue-5-opencv-aruco-api-version-changes) |
| Warped image is rotated, flipped, or inverted | Incorrect ordering of Source vs. Destination points in Homography | [Issue 6](#issue-6-perspective-transform-point-ordering-mismatch) |
| Permission denied when editing files on Host | Root-owned files created by Docker build processes | [Issue 7](#issue-7-handling-root-permissions-on-mounted-workspace) |
| Automated Evaluator hanging or timeout | Interactive `cv2.imshow` / `cv2.waitKey` in submission code | [Issue 8](#issue-8-automated-evaluator-hanging--timeout) |
| Output file format discrepancies | Formatting, spacing, or newline mismatch | [Issue 9](#issue-9-output-file-format-discrepancies) |
| `AttributeError: module 'cv2.aruco' has no attribute 'DetectorParameters'` | Evaluator uses Ubuntu 22.04 system OpenCV 4.5.4 (`DetectorParameters_create`) | [Issue 10](#issue-10-attributeerror-module-cv2aruco-has-no-attribute-detectorparameters) |

---

### Issue 1: `ModuleNotFoundError: No module named 'cv2'`

#### Symptom:
```text
Traceback (most recent call last):
  File "task1a.py", line 4, in <module>
    import cv2
ModuleNotFoundError: No module named 'cv2'
```

#### Cause:
The command was run in the **Host terminal** (`sanal-sivakumar@...`) where OpenCV is not installed, instead of the **Docker container** (`root@...`).

#### Fix:
Always start or enter the Docker container before executing competition scripts:
```bash
# From host terminal:
cd ~/pico_ws
./start_ros.sh

# Inside container:
cd /root/pico_ws/src/swift_pico/scripts
python3 task1a.py --image image_1.jpg
```

---

### Issue 2: Why NEVER use `pip install opencv-python`

#### Symptom:
After installing OpenCV via `pip`, ROS 2 packages using `cv_bridge`, RViz, or `image_view` crash with `symbol lookup error` or `Qt platform plugin could not be initialized`.

#### Cause:
The PyPI wheel (`opencv-python`) bundles its own closed-source copies of `libQtCore`, `libQtGui`, and `ffmpeg`. These conflict with the Ubuntu system shared libraries required by ROS 2.

#### Precaution / Rule:
**Never use `pip install opencv-python` in robotics environments.**  
Always use Ubuntu's official system package:
```bash
sudo apt update && sudo apt install -y python3-opencv python3-numpy
```

---

### Issue 3: Docker Container Name Conflict

#### Symptom:
```text
docker: Error response from daemon: Conflict. The container name "/ros2_humble" is already in use by container "...".
```

#### Cause:
A previous container exited but was not removed.

#### Fix:
Force remove the old container before starting:
```bash
docker rm -f ros2_humble
```
*(Our `start_ros.sh` script automatically runs this cleanup on every launch).*

---

### Issue 4: X11 GUI Window Access Denied

#### Symptom:
```text
QStandardPaths: XDG_RUNTIME_DIR not set
cv2.error: OpenCV(5.0.0): Cannot connect to X server :0
```

#### Cause:
The host's X11 server blocks foreign connections from Docker containers for security reasons.

#### Fix:
Allow local root connections on the host:
```bash
xhost +local:root
```

---

### Issue 5: OpenCV ArUco API Version Changes

#### Symptom:
Code examples found online using `cv2.aruco.detectMarkers()` fail or throw deprecation warnings in OpenCV 4.7+ / 5.0.

#### Cause:
OpenCV modernized the ArUco module in version 4.7.0, moving from standalone functions to the `ArucoDetector` class.

#### Cross-Version Safe Implementation:
```python
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()

if hasattr(cv2.aruco, "ArucoDetector"):
    # Modern OpenCV 4.7+ / 5.x
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(image)
else:
    # Legacy OpenCV <= 4.6
    corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
```

---

### Issue 6: Perspective Transform Point Ordering Mismatch

#### Symptom:
The straightened image from Step 2 is upside down, mirrored, or severely skewed.

#### Cause:
The source quadrilateral vertices were passed in a different geometric order than the destination rectangle $[(0,0), (900,0), (900,900), (0,900)]$.

#### Fix:
Strictly enforce the corner mapping:
* **Point 0**: Inner corner of Marker **80** $\rightarrow (0, 0)$ [Top-Left]
* **Point 1**: Inner corner of Marker **85** $\rightarrow (900, 0)$ [Top-Right]
* **Point 2**: Inner corner of Marker **90** $\rightarrow (900, 900)$ [Bottom-Right]
* **Point 3**: Inner corner of Marker **95** $\rightarrow (0, 900)$ [Bottom-Left]

---

### Issue 7: Handling Root Permissions on Mounted Workspace

#### Symptom:
Files created inside Docker show lock icons in the host file manager or cannot be edited without `sudo`.

#### Cause:
Docker containers run as `root` by default, creating files with UID `0`.

#### Fix:
Reset ownership of all files in `~/pico_ws` to your host user:
```bash
sudo chown -R $USER:$USER ~/pico_ws
```

---

### Issue 8: Automated Evaluator Hanging / Timeout

#### Symptom:
Automated evaluation scripts (`eyantra-autoeval` or server runners) report timeout or freeze indefinitely when testing `task1a.py`.

#### Cause:
Interactive GUI calls (`cv2.imshow`, `cv2.waitKey(0)`) block standard I/O execution waiting for an interactive desktop keypress event that never arrives in automated headless testing.

#### Fix:
Ensure all `cv2.imshow` and `cv2.waitKey` calls are guarded behind a development-only flag (e.g., `--display`) and default to headless execution in production.

---

* Use `', '.join(survivor_list)` to guarantee exactly one comma and one space between names.

---

### Issue 10: `AttributeError: module 'cv2.aruco' has no attribute 'DetectorParameters'`

#### Symptom:
When submitting `KD_5844.zip` to the automated e-Yantra evaluation portal, the evaluator fails with:
```text
AttributeError: module 'cv2.aruco' has no attribute 'DetectorParameters'. Did you mean: 'DetectorParameters_create'?
```

#### Cause:
The local developer environment / container had OpenCV 4.7+ or 5.x installed, where `cv2.aruco.DetectorParameters()` is standard. However, the e-Yantra remote evaluation server runs on Ubuntu 22.04 LTS with standard system OpenCV 4.5.4 / 4.6, which uses `cv2.aruco.DetectorParameters_create()`.

#### Fix:
Use multi-version fallback guards for both parameter instantiation and dictionary lookup:
```python
# 1. Dictionary lookup fallback
if hasattr(cv2.aruco, "getPredefinedDictionary"):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
else:
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

# 2. Parameter creation fallback
if hasattr(cv2.aruco, "DetectorParameters_create"):
    parameters = cv2.aruco.DetectorParameters_create()
elif hasattr(cv2.aruco, "DetectorParameters"):
    parameters = cv2.aruco.DetectorParameters()
else:
    parameters = None

# 3. Detection method fallback
if hasattr(cv2.aruco, "ArucoDetector"):
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(image)
else:
    corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
```



