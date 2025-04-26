import cv2
import numpy as np
import cv2.aruco as aruco
import serial
import time
import math
import pyttsx3  # Text-to-speech library
from InverseKinematics import inverse_kinematics_3d, forward_kinematics_3d  # From your InverseKinematics.py

# Initialize the text-to-speech engine
engine = pyttsx3.init()

# Set up serial communication with Arduino
ser = serial.Serial('COM3', 9600)
time.sleep(2)  # Allow time for the Arduino to reset

# Define IDs for the ArUco markers
corner_ids = [1, 2, 3, 4]  # TL, TR, BR, BL
object_id = 5
bin_id = 6
bin_coords = (40, 30)  # Example bin position in cm

# Define workspace corners (for homography calculation)
workspace_corners = np.array([
    [0, 0],       # ID 1: bottom-left
    [0, 38],      # ID 2: top-left
    [51, 38],     # ID 3: top-right
    [51, 0]       # ID 4: bottom-right
], dtype=np.float32)

# Camera calibration (adjust according to your calibration)
K = np.array([[3.120468069119191e+03, 0, 1.489565452469227e+03], [0, 3.126550067715977e+03, 2.024657223966020e+03], [0, 0, 1]])  # Example camera matrix
dist = np.array([0.1, -0.1, 0, 0])  # Example distortion coefficients

# Start video capture
cap = cv2.VideoCapture(1)

# Object position smoothing
prev_position = np.array([0, 0])

# Create a function to speak and log actions
def speak_and_log(message):
    print(message)  # Log the message to the console
    with open("robot_log.txt", "a") as log_file:
        log_file.write(message + "\n")  # Append to the log file
    engine.say(message)  # Speak the message
    engine.runAndWait()  # Wait for the speech to finish

def send_angles_to_arduino(theta1, theta2, theta3):
    # Retry mechanism for sending angles to Arduino
    retry_count = 3
    while retry_count > 0:
        try:
            data = f"{theta1},{theta2},{theta3}\n"
            ser.write(data.encode())
            time.sleep(1)  # Wait for movement to complete
            print(f"Sent angles to Arduino: {theta1}, {theta2}, {theta3}")
            return True
        except Exception as e:
            print(f"Error sending data to Arduino: {e}")
            retry_count -= 1
            speak_and_log(f"Error in sending data to Arduino, retrying ({retry_count} attempts left)...")
            time.sleep(2)  # Wait before retrying
    return False  # Failed after retries

def handle_aruco_detection(marker_positions):
    if object_id not in marker_positions or bin_id not in marker_positions:
        speak_and_log("Error: Could not detect object or bin markers.")
        return False
    return True

def perform_ik_and_move(target_pos, retries=3):
    # Retry mechanism for inverse kinematics
    while retries > 0:
        try:
            base, shoulder, elbow = inverse_kinematics_3d(*target_pos)
            speak_and_log(f"Calculated angles: Base: {base}, Shoulder: {shoulder}, Elbow: {elbow}")
            # Send to Arduino
            if send_angles_to_arduino(base, shoulder, elbow):
                return True
            else:
                speak_and_log("Failed to send angles to Arduino. Retrying...")
                retries -= 1
                time.sleep(2)
        except ValueError as e:
            print(f"IK Error: {e}")
            speak_and_log(f"IK Error: {e}. Retrying...")
            retries -= 1
            time.sleep(2)
    return False  # Failed after retries

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame from camera.")
        speak_and_log("Error: Could not read frame from camera.")
        break

    # Undistort the image
    undistorted_frame = cv2.undistort(frame, K, dist)

    # Detect ArUco markers
    corners, ids, _ = aruco.detectMarkers(undistorted_frame, aruco.DICT_5X5_100, parameters=aruco.DetectorParameters())

    if ids is not None:
        ids = ids.flatten()
        marker_positions = {}

        for i, marker_id in enumerate(ids):
            # Get center of each marker
            c = corners[i][0]
            center = c.mean(axis=0)
            marker_positions[marker_id] = center

        # Handle marker detection errors
        if not handle_aruco_detection(marker_positions):
            continue

        # Detect object and bin positions
        if object_id in marker_positions:
            object_position = marker_positions[object_id]
            speak_and_log(f"Object detected at workspace position: ({object_position[0]:.2f} cm, {object_position[1]:.2f} cm)")

            # Compute inverse kinematics for object positioning
            target_pos = (object_position[0], object_position[1], 0)  # Z is 0 as it's on the table
            if not perform_ik_and_move(target_pos):
                speak_and_log("Failed to move to object after multiple attempts.")
                continue  # Skip to next loop iteration if move fails

            # Close gripper
            ser.write("G:180\n".encode())  # Close the gripper (Assuming 180 is full close)
            time.sleep(1)  # Wait for the gripper to close
            speak_and_log("Grabbed the object.")

        if bin_id in marker_positions:
            bin_position = marker_positions[bin_id]
            speak_and_log(f"Bin detected at workspace position: ({bin_position[0]:.2f} cm, {bin_position[1]:.2f} cm)")

            # Compute inverse kinematics for bin positioning
            target_pos_bin = (bin_position[0], bin_position[1], 0)  # Z is 0 as well for the bin
            if not perform_ik_and_move(target_pos_bin):
                speak_and_log("Failed to move to bin after multiple attempts.")
                continue  # Skip to next loop iteration if move fails

            # Open gripper
            ser.write("G:0\n".encode())  # Open the gripper
            time.sleep(1)  # Wait for the gripper to open
            speak_and_log("Placed the object in the bin.")

    # Draw markers on the frame for visualization
    aruco.drawDetectedMarkers(undistorted_frame, corners, ids)
    cv2.imshow("Aruco Workspace Tracker", undistorted_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Close the serial connection
ser.close()