#pip install opencv-contrib-python

## This is the code for the ArUco marker detection
import cv2
import cv2.aruco as aruco

# Initialize camera
cap = cv2.VideoCapture(0)

# Load ArUco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        for i, marker_id in enumerate(ids):
            print(f"Marker {marker_id}: {corners[i]}")
    
    # Display frame
    aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

## This is the code for the Color based tracking

import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# Define color ranges (adjust for your object)
lower_bound = np.array([30, 150, 50])  # Example: Green
upper_bound = np.array([90, 255, 255])

while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Find contours of detected object
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        print(f"Object Position: ({x}, {y})")

    cv2.imshow("Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()