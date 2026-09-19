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


def find_survivor_contours(warped_image, min_area=200):
    """
    Step 5: Isolates red and yellow survivor regions using HSV color segmentation,
    morphological cleanup, and contour extraction.
    """
    # 1. Convert to HSV color space
    hsv = cv2.cvtColor(warped_image, cv2.COLOR_BGR2HSV)

    # 2. Red color mask (wraparound: [0, 10] and [170, 180])
    lower_red1, upper_red1 = np.array([0, 100, 100]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([170, 100, 100]), np.array([180, 255, 255])
    mask_red = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    # 3. Yellow color mask ([18, 100, 100] to [35, 255, 255])
    lower_yellow, upper_yellow = np.array([18, 100, 100]), np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 4. Morphological filtering (Close to heal grid lines, Open to remove speckle noise)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    clean_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    clean_red = cv2.morphologyEx(clean_red, cv2.MORPH_OPEN, kernel)

    clean_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
    clean_yellow = cv2.morphologyEx(clean_yellow, cv2.MORPH_OPEN, kernel)

    # 5. Extract and area-filter contours
    raw_red_contours, _ = cv2.findContours(clean_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_contours = [c for c in raw_red_contours if cv2.contourArea(c) >= min_area]

    raw_yellow_contours, _ = cv2.findContours(clean_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    yellow_contours = [c for c in raw_yellow_contours if cv2.contourArea(c) >= min_area]

    return red_contours, yellow_contours


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

    # Step 3 & 4: Coordinate system
    coord_system = ArenaCoordinateSystem(canvas_size=900, num_cells=12)

    # Step 5: Find survivors
    red_contours, yellow_contours = find_survivor_contours(warped_arena, min_area=200)

    total_survivors = len(red_contours) + len(yellow_contours)
    print(f"[CHECKPOINT 5] Detection Results:")
    print(f"  • Red Survivors:    {len(red_contours)}")
    print(f"  • Yellow Survivors: {len(yellow_contours)}")
    print(f"  • Total Count:      {total_survivors}")

    # Checkpoint Visualizer: Draw contrasting outlines
    debug_image = warped_arena.copy()
    # Draw Red survivor outlines in Bright Cyan (BGR: 255, 255, 0)
    cv2.drawContours(debug_image, red_contours, -1, (255, 255, 0), 2)
    # Draw Yellow survivor outlines in Bright Magenta (BGR: 255, 0, 255)
    cv2.drawContours(debug_image, yellow_contours, -1, (255, 0, 255), 2)

    cv2.imshow("Step 5 Checkpoint - Survivor Contours", debug_image)
    print("\nEyeball check: Do the drawn outlines match the real survivors with zero noise?")
    print("Press any key to close the window...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()