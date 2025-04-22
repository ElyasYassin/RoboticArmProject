import cv2
import numpy as np
import cv2.aruco as aruco

# Define IDs
corner_ids = [1, 2, 3, 4]  # TL, TR, BR, BL
object_id = 5

# Real-world positions of the 4 corners in cm
workspace_corners = np.array([
    [0, 0],       # ID 1: Top-left
    [30, 0],      # ID 2: Top-right
    [30, 20],     # ID 3: Bottom-right
    [0, 20]       # ID 4: Bottom-left
], dtype=np.float32)

# ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
parameters = aruco.DetectorParameters()

# Start video capture
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect markers
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

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
            # Compute homography from image → real-world workspace
            H, _ = cv2.findHomography(image_points, workspace_corners)

            # If object marker is detected
            if object_id in marker_positions:
                obj_pixel = np.array([[marker_positions[object_id]]], dtype=np.float32)
                obj_world = cv2.perspectiveTransform(obj_pixel, H)
                x_cm, y_cm = obj_world[0][0]

                print(f"Object detected at workspace position: ({x_cm:.2f} cm, {y_cm:.2f} cm)")

                # Draw overlay
                cv2.circle(frame, tuple(marker_positions[object_id].astype(int)), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"({x_cm:.1f}, {y_cm:.1f}) cm", 
                            tuple(marker_positions[object_id].astype(int) + np.array([10, -10])), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Draw markers
    aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("Aruco Workspace Tracker", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()