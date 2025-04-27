from ArduinoComms import send_serial_command 
from ForwardKinematics import forward_kinematics
import time
import math
import numpy as np

# Initialize current joint angles
base_angle = 90
shoulder_angle = 90
elbow_angle = 90

# Target position (from your ArUco detection or pre-known)
target_x = 20
target_y = 10
target_z = 5

# PID parameters
Kp_base = 0.2
Ki_base = 0.00
Kd_base = 0.1

Kp_shoulder = 0.5
Ki_shoulder = 0.00
Kd_shoulder = 0.2

Kp_elbow = 0.5
Ki_elbow = 0.00
Kd_elbow = 0.2

# Initialize error history
prev_error_base = 0
integral_base = 0

prev_error_shoulder = 0
integral_shoulder = 0

prev_error_elbow = 0
integral_elbow = 0

# Loop settings
time_step = 0.05  

while True:
    start_time = time.time()

    # this gets the current position
    current_x, current_y, current_z = forward_kinematics(base_angle, shoulder_angle, elbow_angle)

    # --- 2. COMPUTE: Find error to target ---
    error_x = target_x - current_x
    error_y = target_y - current_y
    error_z = target_z - current_z
    total_error = (error_x**2 + error_y**2 + error_z**2)**0.5

    if total_error < 0.5:
        print("Target reached!")
        break

    # Calculate base error as angular difference
    desired_base_angle = math.degrees(math.atan2(error_y, error_x))
    error_base = desired_base_angle - base_angle

    # PID for base
    integral_base += error_base * time_step
    derivative_base = (error_base - prev_error_base) / time_step
    base_correction = (Kp_base * error_base) + (Ki_base * integral_base) + (Kd_base * derivative_base)
    prev_error_base = error_base

    # PID for shoulder (vertical control)
    error_shoulder = error_z
    integral_shoulder += error_shoulder * time_step
    derivative_shoulder = (error_shoulder - prev_error_shoulder) / time_step
    shoulder_correction = (Kp_shoulder * error_shoulder) + (Ki_shoulder * integral_shoulder) + (Kd_shoulder * derivative_shoulder)
    prev_error_shoulder = error_shoulder

    # PID for elbow (extension control)
    error_elbow = -error_x  # Elbow flexes to adjust forward reach
    integral_elbow += error_elbow * time_step
    derivative_elbow = (error_elbow - prev_error_elbow) / time_step
    elbow_correction = (Kp_elbow * error_elbow) + (Ki_elbow * integral_elbow) + (Kd_elbow * derivative_elbow)
    prev_error_elbow = error_elbow

    # --- 3. UPDATE ANGLES ---
    base_angle += np.clip(base_correction, -5, 5)
    shoulder_angle += np.clip(shoulder_correction, -5, 5)
    elbow_angle += np.clip(elbow_correction, -5, 5)

    # --- 4. Clamp angles into valid ranges ---
    base_angle = np.clip(base_angle, 0, 180)
    shoulder_angle = np.clip(shoulder_angle, 50, 155)
    elbow_angle = np.clip(elbow_angle, 90, 140)

    # --- 5. ACTUATE: Send new angles to robot ---
    send_serial_command(base_angle, shoulder_angle, elbow_angle)

    print(f"Sent to robot: Base={base_angle:.2f}, Shoulder={shoulder_angle:.2f}, Elbow={elbow_angle:.2f} | Total Error={total_error:.2f}")

    # --- 6. Wait for next control cycle ---
    elapsed = time.time() - start_time
    sleep_time = max(0, time_step - elapsed)
    time.sleep(sleep_time)