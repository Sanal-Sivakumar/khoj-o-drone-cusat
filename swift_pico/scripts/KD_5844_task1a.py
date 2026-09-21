#!/usr/bin/env python3
"""
Khoj-o-Drone (KD) — Task 1A: Survivor Detection and Localization
e-Yantra Robotics Competition (eYRC 2026–27)

Team ID: KD_5844
Authors: Sanal Sivakumar & Team
"""

import argparse
import os
import sys
import cv2
import numpy as np


class ArenaCoordinateSystem:
    """
    Coordinate transformation engine for the 12x12 Khoj-o-Drone arena.
    Handles mathematical pixel-to-intersection mappings for all 121 intersections (A1 to K11).
    """
    def __init__(self, canvas_size=900, num_cells=12):
        self.canvas_size = canvas_size
        self.num_cells = num_cells
        self.cell_size = canvas_size / num_cells  # 75.0 px per cell
        self.num_intersections = num_cells - 1    # 11 interior lines (A-K, 1-11)

        self.name_to_coord = {}
        self.coord_to_name = {}

        for col_idx in range(1, num_cells):  # 1 to 11 -> A to K
            col_letter = chr(ord('A') + col_idx - 1)
            x_px = col_idx * self.cell_size

            for row_idx in range(1, num_cells):  # 1 to 11 -> 1 to 11
                name = f"{col_letter}{row_idx}"
                y_px = row_idx * self.cell_size

                self.name_to_coord[name] = (x_px, y_px)
                self.coord_to_name[(col_idx, row_idx)] = name

    def pixel_to_nearest_intersection(self, x, y):
        """
        Maps continuous (x, y) pixel location on the rectified canvas
        to the nearest named arena intersection with boundary safety clamping.
        """
        col_idx = int(np.clip(np.round(x / self.cell_size), 1, self.num_intersections))
        row_idx = int(np.clip(np.round(y / self.cell_size), 1, self.num_intersections))

        name = self.coord_to_name[(col_idx, row_idx)]
        center_x, center_y = self.name_to_coord[name]
        euclidean_dist = np.hypot(x - center_x, y - center_y)

        return name, (center_x, center_y), euclidean_dist


def detect_corner_markers(image):
    """
    Detects ArUco markers (DICT_4X4_250) compatible across ALL OpenCV versions
    (OpenCV 3.x, 4.2, 4.5.x, 4.6.x, 4.7+, 5.x).
    """
    # 1. Get dictionary (backward compatible)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    else:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

    # 2. Get detector parameters (backward compatible)
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        # OpenCV <= 4.6 (e.g. OpenCV 4.5.4 on Ubuntu 22.04)
        parameters = cv2.aruco.DetectorParameters_create()
    elif hasattr(cv2.aruco, "DetectorParameters"):
        # OpenCV >= 4.7 / 5.x
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = None

    # 3. Detect markers
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

    if ids is None or len(ids) < 4:
        print(f"[ERROR] Expected at least 4 ArUco markers, but found: {0 if ids is None else len(ids)}", file=sys.stderr)
        sys.exit(1)

    detected_ids = [int(x) for x in ids.flatten()]
    return corners, ids, detected_ids


def straighten_arena(image, corners, ids, output_size=900):
    """
    Performs perspective transform using the inner corners of the 4 corner markers
    to produce a square top-down 900x900 canvas of the playing field.
    Works for any marker IDs (image_1, image_2, etc.) using geometric quadrant sorting.
    """
    marker_dict = {int(mid): corner[0] for mid, corner in zip(ids.flatten(), corners)}
    marker_centers = {mid: np.mean(pts, axis=0) for mid, pts in marker_dict.items()}

    # Compute arena center from all detected markers
    all_centers = list(marker_centers.values())
    arena_center = np.mean(all_centers, axis=0)

    def get_inner_corner(pts, center):
        dists = [np.linalg.norm(pt - center) for pt in pts]
        return pts[np.argmin(dists)]

    # If standard IDs 80, 85, 90, 95 are present, use them
    if {80, 85, 90, 95}.issubset(set(marker_dict.keys())):
        tl = get_inner_corner(marker_dict[80], arena_center)
        tr = get_inner_corner(marker_dict[85], arena_center)
        br = get_inner_corner(marker_dict[90], arena_center)
        bl = get_inner_corner(marker_dict[95], arena_center)
    else:
        # Generic geometric quadrant sorting for arbitrary marker IDs (e.g. image_2.jpg)
        tl_id = min(marker_centers.keys(), key=lambda m: (marker_centers[m][0] - 0)**2 + (marker_centers[m][1] - 0)**2)
        tr_id = min(marker_centers.keys(), key=lambda m: (marker_centers[m][0] - image.shape[1])**2 + (marker_centers[m][1] - 0)**2)
        br_id = min(marker_centers.keys(), key=lambda m: (marker_centers[m][0] - image.shape[1])**2 + (marker_centers[m][1] - image.shape[0])**2)
        bl_id = min(marker_centers.keys(), key=lambda m: (marker_centers[m][0] - 0)**2 + (marker_centers[m][1] - image.shape[0])**2)

        tl = get_inner_corner(marker_dict[tl_id], arena_center)
        tr = get_inner_corner(marker_dict[tr_id], arena_center)
        br = get_inner_corner(marker_dict[br_id], arena_center)
        bl = get_inner_corner(marker_dict[bl_id], arena_center)

    src_pts = np.array([tl, tr, br, bl], dtype=np.float32)
    dst_pts = np.array([
        [0, 0],
        [output_size, 0],
        [output_size, output_size],
        [0, output_size]
    ], dtype=np.float32)

    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped_arena = cv2.warpPerspective(image, matrix, (output_size, output_size))

    return warped_arena


def find_survivor_contours(warped_image, min_area=200):
    """
    Isolates red and yellow survivor regions using HSV color segmentation,
    morphological cleanup, and contour extraction.
    """
    hsv = cv2.cvtColor(warped_image, cv2.COLOR_BGR2HSV)

    # 1. Red color mask (wraparound: [0, 10] and [170, 180])
    lower_red1, upper_red1 = np.array([0, 100, 100]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([170, 100, 100]), np.array([180, 255, 255])
    mask_red = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    # 2. Yellow color mask ([18, 100, 100] to [35, 255, 255])
    lower_yellow, upper_yellow = np.array([18, 100, 100]), np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 3. Morphological filtering
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    clean_red = cv2.morphologyEx(cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel), cv2.MORPH_OPEN, kernel)
    clean_yellow = cv2.morphologyEx(cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel), cv2.MORPH_OPEN, kernel)

    # 4. Extract and filter contours
    raw_red, _ = cv2.findContours(clean_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_contours = [c for c in raw_red if cv2.contourArea(c) >= min_area]

    raw_yellow, _ = cv2.findContours(clean_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    yellow_contours = [c for c in raw_yellow if cv2.contourArea(c) >= min_area]

    return red_contours, yellow_contours


def extract_centroid(contour):
    """
    Reduces shape contour to a single centroid point using spatial image moments.
    """
    M = cv2.moments(contour)
    if M["m00"] > 0:
        cx = float(M["m10"] / M["m00"])
        cy = float(M["m01"] / M["m00"])
    else:
        cx, cy = np.mean(contour[:, 0, :], axis=0)
    return (cx, cy)


def write_results_file(image_path, detected_ids, critical_survivors, stable_survivors):
    """
    Writes the exact formatted results to <image_name>_results.txt in the same directory.
    """
    output_path = os.path.splitext(image_path)[0] + "_results.txt"

    lines = [
        f"Detected marker IDs: {detected_ids}",
        "",
        f"Critical Survivors: {', '.join(critical_survivors)}",
        f"Stable Survivors: {', '.join(stable_survivors)}"
    ]
    content = "\n".join(lines) + "\n"

    with open(output_path, "w") as f:
        f.write(content)

    return output_path


def main():
    parser = argparse.ArgumentParser(description="Khoj-o-Drone Task 1A Pipeline — Team KD_5844")
    parser.add_argument(
        "--image",
        type=str,
        required=True,
        help="Path to input disaster image",
    )
    args = parser.parse_args()

    # 1. Fail loudly if image cannot be read
    if not os.path.exists(args.image):
        print(f"[ERROR] Image file does not exist at: {args.image}", file=sys.stderr)
        sys.exit(1)

    image = cv2.imread(args.image)
    if image is None:
        print(f"[ERROR] Could not read image at: {args.image}", file=sys.stderr)
        sys.exit(1)

    # Step 1: Detect corner markers
    corners, ids, detected_ids = detect_corner_markers(image)

    # Step 2: Straighten arena to 900x900
    warped_arena = straighten_arena(image, corners, ids, output_size=900)

    # Step 3 & 4: Coordinate system
    coord_system = ArenaCoordinateSystem(canvas_size=900, num_cells=12)

    # Step 5: Find survivor contours
    red_contours, yellow_contours = find_survivor_contours(warped_arena, min_area=200)

    # Step 6 & 7: Extract centroids and snap to nearest intersections
    critical_survivors = []
    for c in red_contours:
        cx, cy = extract_centroid(c)
        name, _, _ = coord_system.pixel_to_nearest_intersection(cx, cy)
        critical_survivors.append(name)

    stable_survivors = []
    for c in yellow_contours:
        cx, cy = extract_centroid(c)
        name, _, _ = coord_system.pixel_to_nearest_intersection(cx, cy)
        stable_survivors.append(name)

    # Sort for deterministic output
    critical_survivors.sort()
    stable_survivors.sort()

    # Write output file
    results_path = write_results_file(args.image, detected_ids, critical_survivors, stable_survivors)
    print(f"[SUCCESS] Pipeline executed successfully. Results written to: {results_path}")


if __name__ == "__main__":
    main()
