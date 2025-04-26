import math
import numpy as np

# Arm link lengths
L1 = 21.0  # Upper arm length in cm
L2 = 18.2 + 7  # Forearm length in cm

# Normalize angles for servo (0-180 range)
def normalize_angle(angle):
    return max(0, min(180, angle))

# Inverse kinematics with base height included
def inverse_kinematics_3d(x, y, z):
    # Calculate base angle (rotation of the base to align with the object)
    base_rad = math.atan2(x, z)  # Base rotation to reach the target in 3D space
    base_deg = math.degrees(-base_rad)  # Convert radians to degrees and invert direction
    base_servo = 90 + base_deg  # Adjust base angle for servo (centered at 90 degrees)

    # Planar distance calculation (ignoring height for now)
    planar_dist = math.sqrt(x**2 + z**2)
    rel_y = y  # Y-coordinate in the table plane
    D = math.sqrt(planar_dist**2 + rel_y**2)

    # Check if the target is out of reach
    if D > (L1 + L2) or D < abs(L1 - L2):
        raise ValueError("Target is out of reach")

    # Calculate the angle for the elbow joint (theta2)
    cos_theta2 = (planar_dist**2 + rel_y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_theta2)  # Elbow angle in radians
    elbow_deg = 90 + math.degrees(theta2)  # Convert to degrees and adjust for servo range

    # Calculate the shoulder angle (theta1)
    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(rel_y, planar_dist) - math.atan2(k2, k1)  # Shoulder angle in radians
    shoulder_deg = 90 + math.degrees(theta1)  # Convert to degrees and adjust for servo range

    # Invert the shoulder and elbow directions (as per your setup)
    shoulder_deg = 180 - normalize_angle(shoulder_deg)  # Reverse direction of shoulder angle
    elbow_deg = normalize_angle(elbow_deg)  # Elbow angle is positive (no inversion needed)

    # Normalize the base angle for servo
    base_servo = normalize_angle(base_servo)  # Ensure base angle is within the servo range (0-180)

    return base_servo, shoulder_deg, elbow_deg

# Helper function to calculate relative position
def relative_position(object_pos, base_pos):
    x_rel = object_pos[0] - base_pos[0]
    y_rel = object_pos[1] - base_pos[1]
    z_rel = object_pos[2] - base_pos[2]
    return (x_rel, y_rel, z_rel)

# Example positions
object_position = (15, 5, 15)  # (x, y, z) in cm, y=0 because it’s on the table
roboticarm_position = (0, 0, 0)

# Calculate relative position of the object to the robotic arm base
target_pos = relative_position(object_position, roboticarm_position)
print(f"Relative target position: {target_pos}")

# Compute angles and pose
try:
    base, shoulder, elbow = inverse_kinematics_3d(*target_pos)
    print(f"Base angle: {base}°, Shoulder angle: {shoulder}°, Elbow angle: {elbow}°")
except ValueError as e:
    print(f"IK Error: {e}")