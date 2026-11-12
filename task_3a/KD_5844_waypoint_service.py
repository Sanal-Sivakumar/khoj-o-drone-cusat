#!/usr/bin/env python3

'''
Khoj-o-Drone (KD) — Task 3A: Vision-Guided Mission Planner (waypoint_service)
Team ID: KD_5844

Everything is planned from ONE /image_raw frame — nothing about the arena is typed in:
  1. Task 1A pipeline: ArUco corners -> rectified 900x900 arena -> red (critical) and
     yellow (stable) survivors snapped to grid intersections, plus the black obstacles.
  2. Each obstacle is grown by a clearance margin into an OFF-LIMITS area.
  3. Planner: visibility graph around the off-limits areas + Dijkstra for every leg,
     then an exhaustive search over the visiting order.
  4. Service 'waypoints'    (GetWaypoints)   -> critical route from F6.
     Service 'stable_route' (GetStableRoute) -> stable route from the position in the
                                                request, through all stable survivors,
                                                ending at F6.

Route format (both services): one pose per route point, all at Z = 13.
marker_ids[i] tells the client what pose i is:
    grid name ("B10") -> survivor: the client does Visit (Z=13, 1 s) then Save (Z=14, 2 s)
    "VIA"             -> fly-past point around an obstacle, no stop
    "F6"              -> home: hover 1 s, then the server lands
'''

import csv
import heapq
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
from waypoint_navigation.srv import GetWaypoints, GetStableRoute


WAYPOINT_Z = 13.0          # every route point is flown at Z = 13
START_NAME = 'F6'          # take-off and landing intersection
VIA_NAME = 'VIA'
IMAGE_TOPIC = '/image_raw'
REQUEST_WAIT_SEC = 90.0    # how long a request may wait for the frame to be processed
DEBUG_IMAGE_PATH = '/tmp/kd_5844_task3_plan.png'
DEBUG_STABLE_PATH = '/tmp/kd_5844_task3_stable.png'

# ---- Off-limits tuning (whycode units; 1 unit is about 1 m, one grid cell is about 1.2) ----
CLEARANCE = 0.70           # off-limits = obstacle grown by this much (drone half-span + drift)
HARD_CLEARANCE = 0.40      # never closer than this, even when leaving a hold point
NODE_EXTRA = 0.25          # corner points sit this much further out than the off-limits edge
VIA_PENALTY = 0.40         # cost of one extra turn, in the same units as leg_cost
ORDER_SLACK = 1.0          # critical orders within this cost of the best are compared on total mission cost
ARENA_EDGE = 0.45          # in grid cells: how close to the arena border a corner point may be

NAME_RE = re.compile(r'^[A-K](?:[1-9]|1[01])$')


# ----------------------------------------------------------------------------
# Grid intersection -> whycode (x, y) lookup
# ----------------------------------------------------------------------------
def _grid_file_candidates():
    here = os.path.dirname(os.path.abspath(__file__))
    homes = [os.path.expanduser('~'), '/root']
    paths = [os.environ.get('KD_GRID_FILE', ''),
             os.path.join(here, 'grid_whycode_coordinates.csv'),
             os.path.join(here, 'grid_whycode_coordinates.json')]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('swift_pico')
        paths.append(os.path.join(share, 'scripts', 'grid_whycode_coordinates.json'))
        paths.append(os.path.join(share, 'scripts', 'grid_whycode_coordinates.csv'))
    except Exception:
        pass
    for h in homes:
        paths.append(os.path.join(h, 'pico_ws/src/swift_pico/scripts/grid_whycode_coordinates.csv'))
        paths.append(os.path.join(h, 'pico_ws/src/swift_pico/src/grid_whycode_coordinates.json'))
    return [p for p in paths if p]


def _load_grid_csv(path):
    with open(path, newline='') as f:
        rows = [[c.strip() for c in r] for r in csv.reader(f) if r]

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
# Task 1A pipeline: rectify the arena, find survivors and obstacles
# ----------------------------------------------------------------------------
class ArenaCoordinateSystem:
    '''
    Rectified 900x900 canvas <-> 11x11 grid intersections (A1..K11) <-> whycode (x, y).
    The pixel <-> whycode mapping is fitted from the grid table, so corner points that are
    not on an intersection can be converted too.
    '''

    def __init__(self, grid, canvas_size=900, num_cells=12):
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

        names = [n for n in self.name_to_px if n in grid]
        px = np.array([self.name_to_px[n] for n in names], dtype=np.float32)
        wc = np.array([grid[n] for n in names], dtype=np.float32)
        self._px_to_wc, _ = cv2.findHomography(px, wc)
        self._wc_to_px = np.linalg.inv(self._px_to_wc)
        # whycode units per pixel, measured from the table itself
        d_wc = np.linalg.norm(np.array(grid['A1']) - np.array(grid['K11']))
        d_px = np.linalg.norm(np.array(self.name_to_px['A1']) - np.array(self.name_to_px['K11']))
        self.px_per_unit = float(d_px / d_wc)

    @staticmethod
    def _apply(h, pt):
        v = h @ np.array([pt[0], pt[1], 1.0])
        return float(v[0] / v[2]), float(v[1] / v[2])

    def px_to_whycode(self, pt):
        return self._apply(self._px_to_wc, pt)

    def whycode_to_px(self, pt):
        return self._apply(self._wc_to_px, pt)

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


def _clean(mask, k=5):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    return cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel), cv2.MORPH_OPEN, kernel)


def contour_centroid(contour):
    m = cv2.moments(contour)
    if m['m00'] > 0:
        return m['m10'] / m['m00'], m['m01'] / m['m00']
    cx, cy = np.mean(contour[:, 0, :], axis=0)
    return float(cx), float(cy)


def find_survivors(canvas, coord, min_area=200):
    '''Returns (critical names, stable names): red and yellow shapes snapped to intersections.'''
    hsv = cv2.cvtColor(canvas, cv2.COLOR_BGR2HSV)
    red = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])),
        cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255])))
    yellow = cv2.inRange(hsv, np.array([18, 100, 100]), np.array([35, 255, 255]))

    out = []
    for mask in (red, yellow):
        names = []
        contours, _ = cv2.findContours(_clean(mask), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) < min_area:
                continue
            name = coord.nearest_intersection(*contour_centroid(c))
            if name != START_NAME and name not in names:
                names.append(name)
        out.append(sorted(names))
    return out[0], out[1]


def find_obstacle_mask(canvas, coord):
    '''
    Obstacles are the big black blobs. Returns a 0/255 mask of them on the canvas.
    Rejected: small dark specks (tree shadows), and anything sitting on F6 — that is the
    drone itself (dark body + whycode ring) waiting to take off.
    '''
    # Black = every channel low AND no colour cast (dark tree shadows are green, not black).
    px = canvas.astype(np.int16)
    brightest, dullest = px.max(axis=2), px.min(axis=2)
    dark = ((brightest < 55) & (brightest - dullest < 25)).astype(np.uint8) * 255
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_area = 0.20 * coord.cell_size ** 2            # a fifth of a grid cell
    fx, fy = coord.name_to_px[START_NAME]
    mask = np.zeros(canvas.shape[:2], dtype=np.uint8)
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        hull_area = cv2.contourArea(cv2.convexHull(c))
        if hull_area <= 0 or area / hull_area < 0.75:  # obstacles are solid shapes
            continue
        cx, cy = contour_centroid(c)
        if math.hypot(cx - fx, cy - fy) < 0.7 * coord.cell_size:
            continue                                   # the drone on F6
        cv2.drawContours(mask, [c], -1, 255, -1)
    return mask


# ----------------------------------------------------------------------------
# Path planning around the off-limits areas
# ----------------------------------------------------------------------------
def leg_cost(a, b):
    '''
    Flight-time proxy for one straight leg (whycode units). pico_server limits the larger of
    the pitch / roll commands, so leg time follows the LARGER axis difference (Chebyshev).
    A pinch of straight-line length breaks ties.
    '''
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + 0.02 * math.hypot(dx, dy)


def _dilate(mask, radius_px):
    r = max(1, int(round(radius_px)))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    return cv2.dilate(mask, kernel)


class ObstaclePlanner:
    '''
    Works on the rectified canvas (pixels) so everything can be drawn and checked by eye.
      soft  = obstacles grown by `clearance`       -> the OFF-LIMITS area
      hard  = obstacles grown by `hard_clearance`  -> never entered, no exceptions
      nodes = corner points just outside the off-limits areas (visibility-graph vertices)
    A leg may only touch `soft` right at a survivor / the current drone position that already
    lies inside it (so the drone can get out), never in the middle of a leg.
    '''

    def __init__(self, obstacle_mask, coord, clearance=CLEARANCE):
        self.coord = coord
        self.clearance = clearance
        ppu = coord.px_per_unit
        self.size = obstacle_mask.shape[0]
        self.obstacles = obstacle_mask
        self.soft = _dilate(obstacle_mask, clearance * ppu)
        self.hard = _dilate(obstacle_mask, min(HARD_CLEARANCE, clearance) * ppu)
        self.escape_px = 1.6 * clearance * ppu
        self.lo = ARENA_EDGE * coord.cell_size
        self.hi = self.size - self.lo

        self.nodes = []
        grown = _dilate(obstacle_mask, (clearance + NODE_EXTRA) * ppu)
        contours, _ = cv2.findContours(grown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            poly = cv2.approxPolyDP(c, 0.08 * NODE_EXTRA * ppu + 3.0, True)
            for p in poly[:, 0, :]:
                x, y = float(p[0]), float(p[1])
                if not (self.lo <= x <= self.hi and self.lo <= y <= self.hi):
                    continue
                if self.soft[int(round(y)), int(round(x))]:
                    continue                          # inside another obstacle's off-limits area
                self.nodes.append((x, y))
        self.node_wc = [coord.px_to_whycode(p) for p in self.nodes]

        n = len(self.nodes)
        self.node_edges = [[] for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                if self.segment_free(self.nodes[i], self.nodes[j]):
                    c = leg_cost(self.node_wc[i], self.node_wc[j]) + VIA_PENALTY
                    self.node_edges[i].append((j, c))
                    self.node_edges[j].append((i, c))

    def clamp(self, p):
        return (float(np.clip(p[0], 2, self.size - 3)), float(np.clip(p[1], 2, self.size - 3)))

    def segment_free(self, a, b):
        '''True if the straight leg a -> b (pixels) stays clear of the off-limits areas.'''
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(length / 2.0) + 1)
        xs = np.clip(np.round(np.linspace(a[0], b[0], n)).astype(int), 0, self.size - 1)
        ys = np.clip(np.round(np.linspace(a[1], b[1], n)).astype(int), 0, self.size - 1)
        if self.hard[ys, xs].any():
            return False
        hit = self.soft[ys, xs] > 0
        if not hit.any():
            return True
        # Allowed only as a short run touching an end point that is itself inside the area.
        max_run = int(self.escape_px / max(length / (n - 1), 1e-6))
        lead = 0
        while lead < n and hit[lead]:
            lead += 1
        trail = 0
        while trail < n - lead and hit[n - 1 - trail]:
            trail += 1
        if lead > max_run or trail > max_run:
            return False
        return not hit[lead:n - trail].any()

    def shortest(self, a_px, b_px):
        '''
        Cheapest obstacle-free path a -> b. Returns (cost, [corner points in px between a and b])
        or (inf, None) if there is none.
        '''
        a_wc, b_wc = self.coord.px_to_whycode(a_px), self.coord.px_to_whycode(b_px)
        if self.segment_free(a_px, b_px):
            return leg_cost(a_wc, b_wc), []

        n = len(self.nodes)
        to_goal = [leg_cost(self.node_wc[i], b_wc) if self.segment_free(self.nodes[i], b_px) else None
                   for i in range(n)]
        dist = [math.inf] * n
        prev = [-1] * n
        heap = []
        for i in range(n):
            if self.segment_free(a_px, self.nodes[i]):
                dist[i] = leg_cost(a_wc, self.node_wc[i]) + VIA_PENALTY
                heapq.heappush(heap, (dist[i], i))
        best, best_end = math.inf, -1
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist[u] or d >= best:
                continue
            if to_goal[u] is not None and d + to_goal[u] < best:
                best, best_end = d + to_goal[u], u
            for v, c in self.node_edges[u]:
                if d + c < dist[v]:
                    dist[v], prev[v] = d + c, u
                    heapq.heappush(heap, (dist[v], v))
        if best_end < 0:
            return math.inf, None
        chain = []
        u = best_end
        while u >= 0:
            chain.append(self.nodes[u])
            u = prev[u]
        return best, chain[::-1]


class MissionPlanner:
    '''Visiting order + obstacle-free legs for both phases. All stops are grid intersections.'''

    def __init__(self, coord, grid, obstacle_mask, critical, stable, log=None):
        self.coord = coord
        self.grid = grid
        self.critical = list(critical)
        self.stable = list(stable)
        self.log = log or (lambda m: None)
        self.px = {n: coord.name_to_px[n] for n in set(critical) | set(stable) | {START_NAME}}

        # If the full clearance leaves some survivor unreachable, tighten it step by step.
        for clearance in (CLEARANCE, 0.55, HARD_CLEARANCE):
            self.planner = ObstaclePlanner(obstacle_mask, coord, clearance)
            self._planners = {clearance: self.planner}
            self._legs = {}
            names = list(self.px)
            if all(self.leg(a, b)[1] is not None for a in names for b in names if a != b):
                break
            self.log(f'Clearance {clearance:.2f} blocks a leg, trying a smaller one.')

    def leg(self, a, b):
        '''(cost, corner points) between two named stops, cached.'''
        if (a, b) not in self._legs:
            cost, vias = self.planner.shortest(self.px[a], self.px[b])
            self._legs[(a, b)] = (cost, vias)
            self._legs[(b, a)] = (cost, vias[::-1] if vias is not None else None)
        return self._legs[(a, b)]

    def _safe_leg(self, a, b):
        cost, vias = self.leg(a, b)
        if vias is None:        # cannot happen with 3 obstacles; fly direct rather than give up
            self.log(f'No clear path {a} -> {b}; using the direct leg.')
            return leg_cost(self.grid[a], self.grid[b]) + 50.0, []
        return cost, vias

    def _best_stable(self, first_leg):
        '''first_leg(name) -> (cost, vias) from the start position. Returns (cost, order).'''
        if not self.stable:
            return first_leg(START_NAME)[0], []
        best = (math.inf, [])
        for order in itertools.permutations(self.stable):
            total = first_leg(order[0])[0]
            for a, b in zip(order, order[1:] + (START_NAME,)):
                total += self._safe_leg(a, b)[0]
            if total < best[0]:
                best = (total, list(order))
        return best

    def plan_critical(self):
        '''Returns (route, order). route = [(name, (x, y) whycode, (px, py))], F6 not included.'''
        if not self.critical:
            return [], []
        options = []
        for order in itertools.permutations(self.critical):
            cost = sum(self._safe_leg(a, b)[0] for a, b in zip((START_NAME,) + order, order))
            after = self._best_stable(lambda n, last=order[-1]:
                                      self._safe_leg(last, n) if n != last else (0.0, []))[0]
            options.append((cost, cost + after, order))
        fastest = min(o[0] for o in options)
        # Critical survivors are on a clock: only orders close to the fastest are allowed,
        # and among those the one that leaves the shortest stable phase + trip home wins.
        _, _, order = min((o for o in options if o[0] <= fastest + ORDER_SLACK),
                          key=lambda o: (o[1], o[0]))
        route = []
        for a, b in zip((START_NAME,) + order, order):
            route += self._expand(self._safe_leg(a, b)[1], b)
        return route, list(order)

    def plan_stable(self, start_xy):
        '''Stable route from a whycode position, through every stable survivor, ending at F6.'''
        start_px = self.planner.clamp(self.coord.whycode_to_px(start_xy))
        targets = self.stable or [START_NAME]

        # The drone can be cancelled anywhere, even squeezed between an obstacle and the arena
        # edge. If the normal clearance gives no way out, plan the first leg with a tighter one.
        first = {}
        for clearance in (self.planner.clearance, 0.55, HARD_CLEARANCE, 0.25):
            if clearance > self.planner.clearance:
                continue
            if clearance not in self._planners:
                self._planners[clearance] = ObstaclePlanner(self.planner.obstacles, self.coord, clearance)
            first = {n: self._planners[clearance].shortest(start_px, self.px[n]) for n in targets}
            if all(v[1] is not None for v in first.values()):
                break
            self.log(f'Switch position boxed in at clearance {clearance:.2f}, trying a smaller one.')
        for n, (cost, vias) in list(first.items()):
            if vias is None:
                self.log(f'No clear path from the Switch position to {n}; using the direct leg.')
                first[n] = (leg_cost(start_xy, self.grid[n]) + 50.0, [])

        def first_leg(name):
            return first[name]

        _, order = self._best_stable(first_leg)
        stops = order + [START_NAME]
        route = self._expand(first_leg(stops[0])[1], stops[0])
        for a, b in zip(stops, stops[1:]):
            route += self._expand(self._safe_leg(a, b)[1], b)
        return route, order, start_px

    def _expand(self, vias, stop):
        pts = [(VIA_NAME, self.coord.px_to_whycode(p), p) for p in vias]
        pts.append((stop, tuple(self.grid[stop]), self.px[stop]))
        return pts


def draw_plan(canvas, coord, planner, critical, stable, start_px, route, colour):
    '''Checkpoint picture: survivors, obstacles, off-limits areas and one route.'''
    img = canvas.copy()
    tint = img.copy()
    tint[planner.soft > 0] = (0, 0, 255)
    img = cv2.addWeighted(tint, 0.25, img, 0.75, 0)
    for mask, col, thick in ((planner.soft, (0, 0, 255), 1), (planner.obstacles, (0, 0, 255), 3)):
        cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cs, -1, col, thick)
    for names, col, tag in ((critical, (0, 0, 255), 'critical'), (stable, (0, 220, 255), 'stable')):
        for n in names:
            p = tuple(int(v) for v in coord.name_to_px[n])
            cv2.circle(img, p, 30, col, 3)
            cv2.putText(img, f'{tag} {n}', (p[0] - 45, p[1] + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    f6 = tuple(int(v) for v in coord.name_to_px[START_NAME])
    cv2.circle(img, f6, 30, (255, 0, 0), 3)
    cv2.putText(img, START_NAME, (f6[0] + 34, f6[1] + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    cur = tuple(int(v) for v in start_px)
    step = 0
    for name, _, px in route:
        nxt = tuple(int(v) for v in px)
        cv2.arrowedLine(img, cur, nxt, colour, 3, tipLength=0.06)
        if name == VIA_NAME:
            cv2.circle(img, nxt, 6, colour, -1)
        else:
            step += 1
            cv2.putText(img, str(step), (nxt[0] + 10, nxt[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 3)
        cur = nxt
    return img


def analyse_frame(image_bgr, grid, log=None):
    '''
    One camera frame -> everything the mission needs.
    Returns dict(coord, canvas, critical, stable, mission) or raises ValueError with the reason.
    '''
    found = detect_corner_markers(image_bgr)
    if found is None:
        raise ValueError('fewer than 4 ArUco corner markers found in the frame')
    coord = ArenaCoordinateSystem(grid)
    canvas = straighten_arena(image_bgr, found[0], found[1], coord.canvas_size)
    critical, stable = find_survivors(canvas, coord)
    if not critical and not stable:
        raise ValueError('no survivors detected in the frame')
    missing = [n for n in critical + stable if n not in grid]
    if missing:
        raise ValueError(f'intersections missing from the grid table: {missing}')
    mask = find_obstacle_mask(canvas, coord)
    mission = MissionPlanner(coord, grid, mask, critical, stable, log)
    return dict(coord=coord, canvas=canvas, critical=critical, stable=stable, mission=mission)


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


def fill_route(response, route):
    response.waypoints.poses = []
    for _, (x, y), _ in route:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = WAYPOINT_Z
        response.waypoints.poses.append(pose)
    response.marker_ids = [name for name, _, _ in route]
    return response


def describe(route):
    return ' -> '.join(n if n != VIA_NAME else 'via' for n, _, _ in route)


class WayPoints(Node):

    def __init__(self):
        super().__init__('waypoints_service')
        self.cb_group = ReentrantCallbackGroup()

        self.grid, grid_path = load_grid_table()
        self.get_logger().info(f'Loaded {len(self.grid)} grid intersections from {grid_path}')

        self.scene = None          # result of analyse_frame
        self.critical_route = []   # [(name, (x, y), (px, py))]
        self.route_ready = threading.Event()
        self.plan_lock = threading.Lock()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.image_sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, qos, callback_group=self.cb_group)
        self.srv = self.create_service(
            GetWaypoints, 'waypoints', self.waypoint_callback, callback_group=self.cb_group)
        self.stable_srv = self.create_service(
            GetStableRoute, 'stable_route', self.stable_callback, callback_group=self.cb_group)
        self.get_logger().info(f'Waiting for the one {IMAGE_TOPIC} frame...')

    def image_callback(self, msg):
        with self.plan_lock:
            if self.route_ready.is_set():
                return
            try:
                scene = analyse_frame(image_msg_to_bgr(msg), self.grid, self.get_logger().warn)
                route, order = scene['mission'].plan_critical()
            except Exception as e:
                self.get_logger().error(f'Frame not usable yet: {e}')
                return

            log = self.get_logger().info
            log(f'Critical survivors: {scene["critical"]}')
            log(f'Stable survivors:   {scene["stable"]}')
            n_obs = len(cv2.findContours(scene['mission'].planner.obstacles,
                                         cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0])
            log(f'Obstacles: {n_obs}  (off-limits clearance {scene["mission"].planner.clearance:.2f})')
            if len(scene['critical']) != 3 or len(scene['stable']) != 3 or n_obs != 3:
                self.get_logger().warn('Expected 3 critical, 3 stable and 3 obstacles — check the debug picture.')
            log(f'Critical route: {START_NAME} -> {describe(route)}')
            for name, (x, y), _ in route:
                log(f'  {name:>4}: [{x:6.2f}, {y:6.2f}, {WAYPOINT_Z:.2f}]')

            try:
                coord, mission = scene['coord'], scene['mission']
                img = draw_plan(scene['canvas'], coord, mission.planner, scene['critical'],
                                scene['stable'], coord.name_to_px[START_NAME], route, (0, 140, 255))
                cv2.imwrite(DEBUG_IMAGE_PATH, img)
                log(f'Plan picture saved to {DEBUG_IMAGE_PATH}')
            except Exception:
                pass

            self.scene = scene
            self.critical_route = route
            self.route_ready.set()
        self.destroy_subscription(self.image_sub)     # one frame is all we need

    def waypoint_callback(self, request, response):
        if not request.get_waypoints:
            self.get_logger().warn('Incoming request with get_waypoints=False: request rejected.')
            return response
        if not self.route_ready.wait(timeout=REQUEST_WAIT_SEC):
            self.get_logger().error('Request timed out: no route has been planned yet.')
            return response
        fill_route(response, self.critical_route)
        self.get_logger().info(f'Critical route served: {len(self.critical_route)} waypoints.')
        return response

    def stable_callback(self, request, response):
        if not self.route_ready.wait(timeout=REQUEST_WAIT_SEC):
            self.get_logger().error('Stable request timed out: the frame has not been processed.')
            return response
        start = (float(request.position.x), float(request.position.y))
        scene = self.scene
        with self.plan_lock:
            route, order, start_px = scene['mission'].plan_stable(start)
        fill_route(response, route)
        self.get_logger().info(
            f'Stable route from [{start[0]:.2f}, {start[1]:.2f}]: {describe(route)}')
        try:
            img = draw_plan(scene['canvas'], scene['coord'], scene['mission'].planner, scene['critical'],
                            scene['stable'], start_px, route, (255, 200, 0))
            cv2.imwrite(DEBUG_STABLE_PATH, img)
        except Exception:
            pass
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
