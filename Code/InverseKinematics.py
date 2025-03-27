## This is the code for the Inverse Kinematics

import numpy as np

# Define arm link lengths
L1, L2 = 10, 10  # in cm

# Target position (from OpenCV tracking)
x_target, y_target = 15, 5  # Example values

# Compute inverse kinematics
cos_theta2 = (x_target**2 + y_target**2 - L1**2 - L2**2) / (2 * L1 * L2)
theta2 = np.arccos(cos_theta2)

k1 = L1 + L2 * np.cos(theta2)
k2 = L2 * np.sin(theta2)
theta1 = np.arctan2(y_target, x_target) - np.arctan2(k2, k1)

# Convert to degrees
theta1 = np.degrees(theta1)
theta2 = np.degrees(theta2)

print(f"Servo Angles: θ1 = {theta1:.2f}, θ2 = {theta2:.2f}")