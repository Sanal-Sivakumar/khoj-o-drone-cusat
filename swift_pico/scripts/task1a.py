#!/usr/bin/env python3
import argparse
import sys
import cv2
import numpy as np


def detect_corner_markers(image):
    """Detects ArUco markers (DICT_4X4_250) and validates IDs 80, 85, 90, 95."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            image, aruco_dict, parameters=parameters
        )

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


def straighten_arena(image, corners, ids):
    """
    Performs perspective transform using the inner corners of markers 80, 85, 90, 95
    to produce a square top-down 900x900 image of the playing field.
    """
    # 1. Map each marker ID to its 4 corners
    marker_dict = {}
    for marker_id, corner in zip(ids.flatten(), corners):
        marker_dict[marker_id] = corner[0]

    # 2. Compute the approximate arena center from all marker centers
    centers = [np.mean(marker_dict[mid], axis=0) for mid in [80, 85, 90, 95]]
    arena_center = np.mean(centers, axis=0)

    # 3. Helper to find the corner closest to the arena center (the inner corner)
    def get_inner_corner(pts, center):
        dists = [np.linalg.norm(pt - center) for pt in pts]
        return pts[np.argmin(dists)]

    # 4. Extract ordered inner corners
    tl = get_inner_corner(marker_dict[80], arena_center)  # Marker 80 -> Top-Left
    tr = get_inner_corner(marker_dict[85], arena_center)  # Marker 85 -> Top-Right
    br = get_inner_corner(marker_dict[90], arena_center)  # Marker 90 -> Bottom-Right
    bl = get_inner_corner(marker_dict[95], arena_center)  # Marker 95 -> Bottom-Left

    # 5. Define source and destination points
    src_pts = np.array([tl, tr, br, bl], dtype=np.float32)
    dst_pts = np.array([
        [0, 0],         # Top-Left
        [900, 0],       # Top-Right
        [900, 900],     # Bottom-Right
        [0, 900]        # Bottom-Left
    ], dtype=np.float32)

    # 6. Compute Homography Matrix and Warp
    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped_arena = cv2.warpPerspective(image, matrix, (900, 900))

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

    # Step 2: Straighten the arena to 900x900
    warped_arena = straighten_arena(image, corners, ids)
    print(f"[CHECKPOINT 2 PASSED] Warped arena shape: {warped_arena.shape}")

    # Visual Checkpoint: Display the warped playing field
    cv2.imshow("Step 2 - Warped Arena (900x900)", warped_arena)
    print("Displaying Step 2 result. Press any key on the image window to continue...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()