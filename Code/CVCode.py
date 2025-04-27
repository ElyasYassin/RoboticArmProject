import cv2
import numpy as np
import cv2.aruco as aruco

# Define IDs
corner_ids = [1, 2, 3, 4]  # Workspace corners
object_ids = [5, 6, 7, 8]  # Multiple objects now

# Real-world positions of the 4 corners in cm
workspace_corners = np.array([
    [81, 0],    # ID 1: bottom-right
    [81, 51],   # ID 2: top-right
    [0, 51],    # ID 3: top-left
    [0, 0]      # ID 4: bottom-left
], dtype=np.float32)

# Camera calibration
K = np.array([
    [3.120468069119191e+03, 0, 1.489565452469227e+03],
    [0, 3.126550067715977e+03, 2.024657223966020e+03],
    [0, 0, 1]
])
dist = np.array([0.202498358552109, -0.301802564296554, 0, 0])  # [k1, k2, p1, p2]

# ArUco dictionary and parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
parameters = aruco.DetectorParameters()

# Start video capture
cap = cv2.VideoCapture(1)

# Smoothing filter for object positions
prev_positions = {obj_id: np.array([0.0, 0.0]) for obj_id in object_ids}

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Undistort the frame
    undistorted_frame = cv2.undistort(frame, K, dist)

    # Detect markers
    corners, ids, _ = aruco.detectMarkers(undistorted_frame, aruco_dict, parameters=parameters)

    if ids is not None:
        ids = ids.flatten()
        marker_positions = {}

        for i, marker_id in enumerate(ids):
            # Get center of each marker
            c = corners[i][0]
            center = c.mean(axis=0)
            marker_positions[marker_id] = center

        # If all 4 corners are detected
        if all(id in marker_positions for id in corner_ids):
            image_points = np.array([marker_positions[i] for i in corner_ids], dtype=np.float32)

            # Compute homography
            H, status = cv2.findHomography(image_points, workspace_corners, cv2.RANSAC)

            # Loop through each object
            for obj_id in object_ids:
                if obj_id in marker_positions:
                    obj_pixel = np.array([[marker_positions[obj_id]]], dtype=np.float32)
                    obj_world = cv2.perspectiveTransform(obj_pixel, H)
                    x_cm, y_cm = obj_world[0][0]

                    # Apply smoothing
                    smoothed_position = 0.8 * prev_positions[obj_id] + 0.2 * np.array([x_cm, y_cm])
                    prev_positions[obj_id] = smoothed_position

                    print(f"Object {obj_id} detected at workspace position: ({smoothed_position[0]:.2f} cm, {smoothed_position[1]:.2f} cm)")

                    # Draw on the frame
                    cv2.circle(undistorted_frame, tuple(marker_positions[obj_id].astype(int)), 5, (0, 255, 0), -1)
                    cv2.putText(undistorted_frame, f"ID {obj_id}: ({smoothed_position[0]:.1f}, {smoothed_position[1]:.1f}) cm",
                                tuple(marker_positions[obj_id].astype(int) + np.array([10, -10])),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Draw and color-code the corners
        for marker_id, position in marker_positions.items():
            if marker_id == 1:
                cv2.drawMarker(undistorted_frame, tuple(position.astype(int)), (0, 0, 255), cv2.MARKER_CROSS)
            elif marker_id == 2:
                cv2.drawMarker(undistorted_frame, tuple(position.astype(int)), (0, 255, 0), cv2.MARKER_STAR)
            elif marker_id == 3:
                cv2.drawMarker(undistorted_frame, tuple(position.astype(int)), (255, 0, 0), cv2.MARKER_TILTED_CROSS)
            elif marker_id == 4:
                cv2.drawMarker(undistorted_frame, tuple(position.astype(int)), (0, 255, 255), cv2.MARKER_DIAMOND)

    # Draw all detected markers
    aruco.drawDetectedMarkers(undistorted_frame, corners, ids)
    cv2.imshow("Aruco Workspace Tracker", undistorted_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()