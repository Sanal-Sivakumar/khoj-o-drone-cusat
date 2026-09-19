#!/usr/bin/env python3
import argparse
import sys
import cv2
import numpy as np


class ArenaCoordinateSystem:
    """
    Generalized coordinate transformation engine for the 12x12 Khoj-o-Drone arena.
    Handles mathematical pixel-to-intersection and intersection-to-pixel mappings
    for all 121 intersections (A1 to K11).
    """
    def __init__(self, canvas_size=900, num_cells=12):
        self.canvas_size = canvas_size
        self.num_cells = num_cells
        self.cell_size = canvas_size / num_cells  # 75.0 px per cell
        self.num_intersections = num_cells - 1    # 11 interior lines (A-K, 1-11)

        # Build bidirectional lookup table for all 121 intersections
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
        """Maps any (x, y) pixel location to the nearest named intersection."""
        col_idx = int(np.clip(np.round(x / self.cell_size), 1, self.num_intersections))
        row_idx = int(np.clip(np.round(y / self.cell_size), 1, self.num_intersections))

        name = self.coord_to_name[(col_idx, row_idx)]
        center_x, center_y = self.name_to_coord[name]
        euclidean_dist = np.hypot(x - center_x, y - center_y)

        return name, (center_x, center_y), euclidean_dist

    def intersection_to_pixel(self, name):
        """Returns the exact (x, y) center pixel of a given intersection name."""
        if name not in self.name_to_coord:
            raise ValueError(f"Invalid intersection name '{name}'. Must be between A1 and K11.")
        return self.name_to_coord[name]

    def draw_checkpoint_overlay(self, image):
        """
        Step 4 Checkpoint Visualizer:
        Draws the grid lines and prints the name next to EVERY single one of the 121 intersections.
        """
        overlay = image.copy()

        # 1. Draw subtle grid lines in green
        for i in range(self.num_cells + 1):
            pos = int(round(i * self.cell_size))
            cv2.line(overlay, (pos, 0), (pos, self.canvas_size), (0, 180, 0), 1)
            cv2.line(overlay, (0, pos), (self.canvas_size, pos), (0, 180, 0), 1)

        # 2. Draw red dot and text name next to EVERY one of the 121 intersections
        for name, (x, y) in self.name_to_coord.items():
            ix, iy = int(round(x)), int(round(y))

            # Red intersection marker
            cv2.circle(overlay, (ix, iy), 3, (0, 0, 255), -1)

            # Name label next to each intersection
            cv2.putText(
                overlay,
                name,
                (ix + 4, iy - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.28,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )

        return overlay


def detect_corner_markers(image):
    """Detects ArUco markers (DICT_4X4_250) and validates IDs 80, 85, 90, 95."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

    if ids is None or len(ids) == 0:
        print("[ERROR] No ArUco markers detected in the image.")
        sys.exit(1)

    detected_ids = ids.flatten().tolist()
    required_ids = {80, 85, 90, 95}
    missing_ids = required_ids - set(detected_ids)
    if missing_ids:
        print(f"[ERROR] Missing required marker IDs: {missing_ids}. Cannot continue!")
        sys.exit(1)

    return corners, ids


def straighten_arena(image, corners, ids, output_size=900):
    """Performs perspective transform using the inner corners of markers 80, 85, 90, 95."""
    marker_dict = {mid: corner[0] for mid, corner in zip(ids.flatten(), corners)}
    centers = [np.mean(marker_dict[mid], axis=0) for mid in [80, 85, 90, 95]]
    arena_center = np.mean(centers, axis=0)

    def get_inner_corner(pts, center):
        dists = [np.linalg.norm(pt - center) for pt in pts]
        return pts[np.argmin(dists)]

    tl = get_inner_corner(marker_dict[80], arena_center)
    tr = get_inner_corner(marker_dict[85], arena_center)
    br = get_inner_corner(marker_dict[90], arena_center)
    bl = get_inner_corner(marker_dict[95], arena_center)

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


def main():
    parser = argparse.ArgumentParser(description="Khoj-o-Drone Task 1A Pipeline")
    parser.add_argument(
        "--image",
        type=str,
        default="image_1.jpg",
        help="Path to input disaster image",
    )
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        print(f"[ERROR] Could not read image at: {args.image}")
        sys.exit(1)

    # Step 1: Detect corner markers
    corners, ids = detect_corner_markers(image)

    # Step 2: Straighten arena to 900x900
    warped_arena = straighten_arena(image, corners, ids, output_size=900)

    # Step 3 & 4: Coordinate system with all 121 named intersections
    coord_system = ArenaCoordinateSystem(canvas_size=900, num_cells=12)
    debug_overlay = coord_system.draw_checkpoint_overlay(warped_arena)

    print(f"[CHECKPOINT 4] All {len(coord_system.name_to_coord)} intersections generated:")
    for row in range(1, 12):
        row_names = [f"{chr(ord('A') + c - 1)}{row}" for c in range(1, 12)]
        print("  ".join(f"{name:>3}" for name in row_names))

    # Visual Checkpoint Window
    cv2.imshow("Step 4 Checkpoint - 121 Named Intersections", debug_overlay)
    print("\nEyeball the board: A1 top-left, K11 bottom-right, letters advancing rightwards.")
    print("Press any key on the image window to continue...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()