import cv2
import numpy as np
import cv2.aruco as aruco
import threading
import time

# --- Global store for detected objects ---
detected_objects = {}

# --- Mapping object IDs to colors ---
id_to_color = {
    5: "purple",
    6: "purple",
    7: "purple",
    8: "green"
}

# --- Thread control flag ---
cv_thread_running = True

# --- DEBUG flag ---
DEBUG = True  # Set to True to print detailed info

def cv_thread():
    global detected_objects

    # Setup ArUco parameters
    corner_ids = [1, 2, 3, 4]  # Workspace corners
    object_ids = [5, 6, 7, 8]  # Object markers

    workspace_corners = np.array([
        [81, 0],    # ID 1: bottom-right
        [81, 51],   # ID 2: top-right
        [0, 51],    # ID 3: top-left
        [0, 0]      # ID 4: bottom-left
    ], dtype=np.float32)

    K = np.array([
        [3.120468069119191e+03, 0, 1.489565452469227e+03],
        [0, 3.126550067715977e+03, 2.024657223966020e+03],
        [0, 0, 1]
    ])
    dist = np.array([0.202498358552109, -0.301802564296554, 0, 0])

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    parameters = aruco.DetectorParameters()

    cap = cv2.VideoCapture(1)

    prev_positions = {obj_id: np.array([0.0, 0.0]) for obj_id in object_ids}

    while cv_thread_running:
        ret, frame = cap.read()
        if not ret:
            continue

        undistorted_frame = cv2.undistort(frame, K, dist)
        corners, ids, _ = aruco.detectMarkers(undistorted_frame, aruco_dict, parameters=parameters)

        if ids is not None:
            ids = ids.flatten()
            marker_positions = {}

            if DEBUG:
                print(f"[DEBUG] Detected marker IDs: {ids}")

            for i, marker_id in enumerate(ids):
                c = corners[i][0]
                center = c.mean(axis=0)
                marker_positions[marker_id] = center

                if DEBUG:
                    print(f"[DEBUG] Marker {marker_id} pixel center: {center}")

            if all(id in marker_positions for id in corner_ids):
                image_points = np.array([marker_positions[i] for i in corner_ids], dtype=np.float32)
                H, status = cv2.findHomography(image_points, workspace_corners, cv2.RANSAC)

                for obj_id in object_ids:
                    if obj_id in marker_positions:
                        obj_pixel = np.array([[marker_positions[obj_id]]], dtype=np.float32)
                        obj_world = cv2.perspectiveTransform(obj_pixel, H)
                        x_cm, y_cm = obj_world[0][0]

                        smoothed_position = 0.8 * prev_positions[obj_id] + 0.2 * np.array([x_cm, y_cm])
                        prev_positions[obj_id] = smoothed_position

                        color = id_to_color.get(obj_id, "unknown")
                        detected_objects[color] = (smoothed_position[0], smoothed_position[1])

                        if DEBUG:
                            print(f"[DEBUG] Object ID {obj_id} ({color}): World position X={x_cm:.2f} cm, Y={y_cm:.2f} cm")
                            print(f"[DEBUG] Smoothed Position -> X={smoothed_position[0]:.2f} cm, Y={smoothed_position[1]:.2f} cm")

        # Slow down thread loop to save CPU
        time.sleep(0.2)

    cap.release()
    cv2.destroyAllWindows()

def start_cv_thread():
    global cv_thread_running
    cv_thread_running = True
    t = threading.Thread(target=cv_thread)
    t.daemon = True
    t.start()

def stop_cv_thread():
    global cv_thread_running
    cv_thread_running = False

if __name__ == "__main__":
    start_cv_thread()

    while True:
        time.sleep(1)
        print(detected_objects)