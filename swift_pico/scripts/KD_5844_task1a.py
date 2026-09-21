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
    Detects ArUco markers (DICT_4X4_250) and validates that IDs 80, 85, 90, 95 exist.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

    if ids is None or len(ids) == 0:
        print("[ERROR] No ArUco markers detected in the image.", file=sys.stderr)
        sys.exit(1)

    detected_ids = [int(x) for x in ids.flatten()]
    required_ids = {80, 85, 90, 95}
    missing_ids = required_ids - set(detected_ids)
    if missing_ids:
        print(f"[ERROR] Missing required marker IDs: {missing_ids}.", file=sys.stderr)
        sys.exit(1)

    return corners, ids, detected_ids


def straighten_arena(image, corners, ids, output_size=900):
    """
    Performs perspective transform using the inner corners of markers 80, 85, 90, 95
    to produce a square top-down 900x900 canvas of the playing field.
    """
    marker_dict = {int(mid): corner[0] for mid, corner in zip(ids.flatten(), corners)}
    centers = [np.mean(marker_dict[mid], axis=0) for mid in [80, 85, 90, 95]]
    arena_center = np.mean(centers, axis=0)

    def get_inner_corner(pts, center):
        dists = [np.linalg.norm(pt - center) for pt in pts]
        return pts[np.argmin(dists)]

    tl = get_inner_corner(marker_dict[80], arena_center)  # Marker 80 -> Top-Left
    tr = get_inner_corner(marker_dict[85], arena_center)  # Marker 85 -> Top-Right
    br = get_inner_corner(marker_dict[90], arena_center)  # Marker 90 -> Bottom-Right
    bl = get_inner_corner(marker_dict[95], arena_center)  # Marker 95 -> Bottom-Left

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
