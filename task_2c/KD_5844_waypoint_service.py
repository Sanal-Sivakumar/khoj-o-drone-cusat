#!/usr/bin/env python3

'''
Khoj-o-Drone (KD) — Vision-Guided Waypoint Service
Team ID: KD_5844

waypoint_service builds its own waypoint list instead of using typed-in coordinates:
  1. Subscribe ONCE to /image_raw (latched frame: RELIABLE + TRANSIENT_LOCAL), then drop the subscription.
  2. Task 1A pipeline: find the ArUco corners, rectify the arena, segment red (critical) survivors,
     snap each one to its nearest grid intersection.
  3. Look the intersections up in grid_whycode_coordinates.csv (Z is always 13).
  4. Plan the visiting order from the arena centre F6 (exhaustive search, cost = flight-time proxy).
  5. Store the route, and answer GetWaypoints only once the route exists.

pico_client / pico_server are unchanged from Task 2B.
'''

import csv
import itertools
import json
import math
import os
import re
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from waypoint_navigation.srv import GetWaypoints


WAYPOINT_Z = 13.0          # every waypoint is flown at Z = 13
START_NAME = 'F6'          # drone always starts at the arena centre
IMAGE_TOPIC = '/image_raw'
REQUEST_WAIT_SEC = 90.0    # how long a request may wait for the route to be planned
DEBUG_IMAGE_PATH = '/tmp/kd_5844_route_debug.png'

NAME_RE = re.compile(r'^[A-K](?:[1-9]|1[01])$')


# ----------------------------------------------------------------------------
# Grid intersection -> whycode (x, y) lookup
# ----------------------------------------------------------------------------
def _grid_file_candidates():
    here = os.path.dirname(os.path.abspath(__file__))
    homes = [os.path.expanduser('~'), '/root']
    paths = [os.environ.get('KD_GRID_FILE', ''),
             os.path.join(here, 'grid_whycode_coordinates.csv')]
    for h in homes:
        paths.append(os.path.join(h, 'pico_ws/src/swift_pico/scripts/grid_whycode_coordinates.csv'))
    paths.append(os.path.join(here, 'grid_whycode_coordinates.json'))
    for h in homes:
        paths.append(os.path.join(h, 'pico_ws/src/swift_pico/src/grid_whycode_coordinates.json'))
    return [p for p in paths if p]


def _load_grid_csv(path):
    with open(path, newline='') as f:
        rows = [[c.strip() for c in r] for r in csv.reader(f) if r]

    # Header-driven columns if there is a header with x / y columns
    x_idx = y_idx = None
    for r in rows[:3]:
        low = [c.lower() for c in r]
        xs = [i for i, c in enumerate(low) if c == 'x' or c.startswith('x_') or c.startswith('x (')]
        ys = [i for i, c in enumerate(low) if c == 'y' or c.startswith('y_') or c.startswith('y (')]
        if xs and ys:
            x_idx, y_idx = xs[0], ys[0]
            break

    table = {}
    for r in rows:
        name_idx = next((i for i, c in enumerate(r) if NAME_RE.match(c.upper())), None)
        if name_idx is None:
            continue
        name = r[name_idx].upper()
        x = y = None
        if x_idx is not None and max(x_idx, y_idx) < len(r):
            try:
                x, y = float(r[x_idx]), float(r[y_idx])
            except ValueError:
                x = y = None
        if x is None:
            nums = []
            for c in r[name_idx + 1:] + r[:name_idx]:
                try:
                    nums.append(float(c))
                except ValueError:
                    pass
            if len(nums) < 2:
                continue
            x, y = nums[0], nums[1]
        table[name] = (x, y)
    return table


def _load_grid_json(path):
    with open(path) as f:
        data = json.load(f)
    return {k.upper(): (float(v['x']), float(v['y'])) for k, v in data.items()}


def load_grid_table():
    '''Returns ({name: (x, y)}, path_used). Raises if no usable table is found.'''
    for path in _grid_file_candidates():
        if not os.path.isfile(path):
            continue
        try:
            table = _load_grid_json(path) if path.endswith('.json') else _load_grid_csv(path)
        except Exception:
            continue
        if START_NAME in table and len(table) >= 100:
            return table, path
    raise FileNotFoundError('grid_whycode_coordinates (csv/json) not found or unreadable')


# ----------------------------------------------------------------------------
# Task 1A pipeline (arena rectification + critical survivor detection)
# ----------------------------------------------------------------------------
class ArenaCoordinateSystem:
    '''Maps pixels on the rectified 900x900 canvas to the 11x11 grid intersections (A1..K11).'''

    def __init__(self, canvas_size=900, num_cells=12):
        self.canvas_size = canvas_size
        self.num_cells = num_cells
        self.cell_size = canvas_size / num_cells
        self.num_intersections = num_cells - 1
        self.name_to_px = {}
        self.idx_to_name = {}
        for col in range(1, num_cells):
            for row in range(1, num_cells):
                name = f'{chr(ord("A") + col - 1)}{row}'
                self.name_to_px[name] = (col * self.cell_size, row * self.cell_size)
                self.idx_to_name[(col, row)] = name

    def nearest_intersection(self, x, y):
        col = int(np.clip(np.round(x / self.cell_size), 1, self.num_intersections))
        row = int(np.clip(np.round(y / self.cell_size), 1, self.num_intersections))
        return self.idx_to_name[(col, row)]


def detect_corner_markers(image):
    '''ArUco DICT_4X4_250 detection that works across OpenCV 3.x ... 5.x. None if < 4 markers.'''
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    else:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        parameters = cv2.aruco.DetectorParameters_create()
    elif hasattr(cv2.aruco, 'DetectorParameters'):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = None

    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

    if ids is None or len(ids) < 4:
        return None
    return corners, ids


def straighten_arena(image, corners, ids, output_size=900):
    '''Perspective-warps the arena (inner corners of the 4 corner markers) to a square canvas.'''
    marker_dict = {int(mid): c[0] for mid, c in zip(ids.flatten(), corners)}
    centers = {mid: np.mean(pts, axis=0) for mid, pts in marker_dict.items()}
    arena_center = np.mean(list(centers.values()), axis=0)

    def inner_corner(pts):
        return pts[np.argmin([np.linalg.norm(p - arena_center) for p in pts])]

    h, w = image.shape[:2]
    if {80, 85, 90, 95}.issubset(marker_dict.keys()):
        tl, tr, br, bl = (inner_corner(marker_dict[i]) for i in (80, 85, 90, 95))
    else:
        def closest(px, py):
            return min(centers, key=lambda m: (centers[m][0] - px) ** 2 + (centers[m][1] - py) ** 2)
        tl = inner_corner(marker_dict[closest(0, 0)])
        tr = inner_corner(marker_dict[closest(w, 0)])
        br = inner_corner(marker_dict[closest(w, h)])
        bl = inner_corner(marker_dict[closest(0, h)])

    src = np.array([tl, tr, br, bl], dtype=np.float32)
    dst = np.array([[0, 0], [output_size, 0], [output_size, output_size], [0, output_size]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(image, matrix, (output_size, output_size))


def find_critical_contours(warped, min_area=200):
    '''Critical survivors are the RED shapes (HSV segmentation + morphology + area filter).'''
    hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])),
        cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255])))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel), cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) >= min_area]


def contour_centroid(contour):
    m = cv2.moments(contour)
    if m['m00'] > 0:
        return m['m10'] / m['m00'], m['m01'] / m['m00']
    cx, cy = np.mean(contour[:, 0, :], axis=0)
    return float(cx), float(cy)


def detect_critical_survivors(image_bgr, coord_system):
    '''Returns (sorted list of intersection names, rectified canvas) or (None, None) on failure.'''
    found = detect_corner_markers(image_bgr)
    if found is None:
        return None, None
    corners, ids = found
    canvas = straighten_arena(image_bgr, corners, ids, coord_system.canvas_size)
    names = []
    for contour in find_critical_contours(canvas):
        cx, cy = contour_centroid(contour)
        name = coord_system.nearest_intersection(cx, cy)
        if name not in names:          # two blobs snapping to one intersection = one stop
            names.append(name)
    return sorted(names), canvas


# ----------------------------------------------------------------------------
# Path planning
# ----------------------------------------------------------------------------
def leg_cost(a, b):
    '''
    Flight-time proxy for one leg. pico_server clamps pitch and roll independently, so the drone
    covers x and y at the same capped rate: leg time follows the LARGER axis difference
    (Chebyshev), not the straight-line length. Measured on our Task 2B bag:
    3.61 m -> 5.9 s for two legs of different Euclidean length, 4.82 m -> 6.5 s.
    '''
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def plan_route(start_xy, stops):
    '''
    stops: {name: (x, y)}. Returns the visiting order (list of names) minimising total leg cost
    from start_xy. Exhaustive search (24 orders for 4 survivors); nearest-neighbour beyond 8.
    '''
    names = list(stops)
    if len(names) <= 1:
        return names

    def path_cost(order):
        cur, total, euclid = start_xy, 0.0, 0.0
        for n in order:
            total += leg_cost(cur, stops[n])
            euclid += math.hypot(stops[n][0] - cur[0], stops[n][1] - cur[1])
            cur = stops[n]
        return round(total, 6), euclid        # tie-break on straight-line length

    if len(names) <= 8:
        return list(min(itertools.permutations(names), key=path_cost))

    order, cur, left = [], start_xy, set(names)
    while left:
        nxt = min(left, key=lambda n: (leg_cost(cur, stops[n]), n))
        order.append(nxt)
        left.remove(nxt)
        cur = stops[nxt]
    return order


def draw_route_debug(canvas, coord_system, order):
    '''Checkpoint picture: route drawn intersection-to-intersection on the rectified arena.'''
    img = canvas.copy()
    pts = [coord_system.name_to_px[START_NAME]] + [coord_system.name_to_px[n] for n in order]
    for a, b in zip(pts[:-1], pts[1:]):
        cv2.line(img, tuple(int(v) for v in a), tuple(int(v) for v in b), (0, 140, 255), 3)
    cv2.circle(img, tuple(int(v) for v in pts[0]), 8, (255, 0, 0), -1)
    cv2.putText(img, START_NAME, (int(pts[0][0]) + 10, int(pts[0][1]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    for i, n in enumerate(order, 1):
        p = tuple(int(v) for v in coord_system.name_to_px[n])
        cv2.circle(img, p, 22, (0, 255, 0), 3)
        cv2.putText(img, f'{i}:{n}', (p[0] + 25, p[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return img


# ----------------------------------------------------------------------------
# ROS 2 helpers / node
# ----------------------------------------------------------------------------
def image_msg_to_bgr(msg):
    '''sensor_msgs/Image -> BGR numpy array (no cv_bridge, so no NumPy ABI surprises).'''
    enc = msg.encoding.lower()
    channels = {'mono8': 1, '8uc1': 1, 'rgb8': 3, 'bgr8': 3, '8uc3': 3, 'rgba8': 4, 'bgra8': 4}.get(enc)
    if channels is None:
        raise ValueError(f'unsupported image encoding: {msg.encoding}')
    row = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    img = row[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if enc in ('mono8', '8uc1'):
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if enc == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if enc == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return np.ascontiguousarray(img)


class WayPoints(Node):

    def __init__(self):
        super().__init__('waypoints_service')
        self.cb_group = ReentrantCallbackGroup()

        self.grid, grid_path = load_grid_table()
        self.coord_system = ArenaCoordinateSystem()
        self.get_logger().info(f'Loaded {len(self.grid)} grid intersections from {grid_path}')

        self.waypoints = []        # [[x, y, z], ...] in visiting order — filled by the planner
        self.marker_ids = []       # intersection names, same order
        self.route_ready = threading.Event()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.image_sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, qos, callback_group=self.cb_group)
        self.srv = self.create_service(
            GetWaypoints, 'waypoints', self.waypoint_callback, callback_group=self.cb_group)
        self.get_logger().info(f'Waiting for the one {IMAGE_TOPIC} frame...')

    def image_callback(self, msg):
        if self.route_ready.is_set():
            return
        try:
            image = image_msg_to_bgr(msg)
            names, canvas = detect_critical_survivors(image, self.coord_system)
        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')
            return
        if names is None:
            self.get_logger().error('Fewer than 4 ArUco corner markers found in the frame.')
            return
        if not names:
            self.get_logger().error('No critical survivors detected in the frame.')
            return

        missing = [n for n in names if n not in self.grid]
        if missing:
            self.get_logger().error(f'Intersections missing from grid table: {missing}')
            return

        self.get_logger().info(f'Critical survivors detected: {names}')
        stops = {n: self.grid[n] for n in names}
        order = plan_route(self.grid[START_NAME], stops)

        self.waypoints = [[stops[n][0], stops[n][1], WAYPOINT_Z] for n in order]
        self.marker_ids = list(order)
        for n, wp in zip(order, self.waypoints):
            self.get_logger().info(f'  {n}: [{wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}]')
        self.get_logger().info(f'Route from {START_NAME}: {" -> ".join([START_NAME] + order)}')

        try:
            cv2.imwrite(DEBUG_IMAGE_PATH, draw_route_debug(canvas, self.coord_system, order))
            self.get_logger().info(f'Route picture saved to {DEBUG_IMAGE_PATH}')
        except Exception:
            pass

        self.route_ready.set()
        self.destroy_subscription(self.image_sub)     # one frame is all we need

    def waypoint_callback(self, request, response):
        if not request.get_waypoints:
            self.get_logger().warn('Incoming request with get_waypoints=False: request rejected.')
            return response

        if not self.route_ready.wait(timeout=REQUEST_WAIT_SEC):
            self.get_logger().error('Request timed out: no route has been planned yet.')
            return response

        response.waypoints.poses = [Pose() for _ in self.waypoints]
        for pose, wp in zip(response.waypoints.poses, self.waypoints):
            pose.position.x = float(wp[0])
            pose.position.y = float(wp[1])
            pose.position.z = float(wp[2])
        response.marker_ids = list(self.marker_ids)
        self.get_logger().info(f'Incoming request for Waypoints: served {len(self.waypoints)} waypoints.')
        return response


def main():
    rclpy.init()
    waypoints = WayPoints()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(waypoints)

    try:
        executor.spin()
    except KeyboardInterrupt:
        waypoints.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoints.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
