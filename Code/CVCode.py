import cv2
import numpy as np
import cv2.aruco as aruco

# Define IDs
corner_ids = [1, 2, 3, 4]  # TL, TR, BR, BL
object_id = 5
bin_coords = (40, 30)  # Example bin position (x, y) in cm

# Real-world positions of the 4 corners in cm
workspace_corners = np.array([
    [0, 0],       # ID 1: bottom-left
    [0, 38],      # ID 2: top-left
    [51, 38],     # ID 3: top-right
    [51, 0]       # ID 4: bottom-right
], dtype=np.float32)

# Camera calibration (adjust this according to your actual calibration)
K = np.array([[3.120468069119191e+03, 0, 1.489565452469227e+03], [0, 3.126550067715977e+03, 2.024657223966020e+03], [0, 0, 1]])  # Example camera matrix
dist = np.array([0.1, -0.1, 0, 0])  # Example distortion coefficients

# ArUco dictionary and parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
parameters = aruco.DetectorParameters()  # Corrected line

# Start video capture
cap = cv2.VideoCapture(1)

# Smoothing filter for object position
prev_position = np.array([0, 0])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Undistort the image
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

            # Compute homography from image → real-world workspace using RANSAC for robustness
            H, status = cv2.findHomography(image_points, workspace_corners, cv2.RANSAC)

            # If object marker is detected
            if object_id in marker_positions:
                obj_pixel = np.array([[marker_positions[object_id]]], dtype=np.float32)
                obj_world = cv2.perspectiveTransform(obj_pixel, H)
                x_cm, y_cm = obj_world[0][0]

                # Apply smoothing filter to the detected object position
                smoothed_position = 0.8 * prev_position + 0.2 * np.array([x_cm, y_cm])
                prev_position = smoothed_position

                print(f"Object detected at workspace position: ({smoothed_position[0]:.2f} cm, {smoothed_position[1]:.2f} cm)")

                # Draw overlay on the image
                cv2.circle(undistorted_frame, tuple(marker_positions[object_id].astype(int)), 5, (0, 255, 0), -1)
                cv2.putText(undistorted_frame, f"({smoothed_position[0]:.1f}, {smoothed_position[1]:.1f}) cm", 
                            tuple(marker_positions[object_id].astype(int) + np.array([10, -10])), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Draw a line to the bin location
                cv2.line(undistorted_frame, tuple(marker_positions[object_id].astype(int)), (bin_coords[0], bin_coords[1]), (0, 255, 0), 2)
                cv2.putText(undistorted_frame, "Path to Bin", (bin_coords[0] + 10, bin_coords[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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

    # Draw markers on the frame
    aruco.drawDetectedMarkers(undistorted_frame, corners, ids)
    cv2.imshow("Aruco Workspace Tracker", undistorted_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()