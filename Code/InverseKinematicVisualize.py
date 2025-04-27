import math
import matplotlib.pyplot as plt
import numpy as np
from ForwardKinematics import forward_kinematics

def draw_robot_arm(base_angle_deg, shoulder_angle_deg, elbow_angle_deg, target_xyz=None):
    # Arm link lengths
    L1 = 20.955  # Upper arm
    L2 = 22.225  # Forearm

    # Corrected angles (your model correction)
    shoulder_corrected = shoulder_angle_deg - 120
    elbow_corrected = 140 - elbow_angle_deg

    # Convert to radians
    base = math.radians(base_angle_deg)
    shoulder = math.radians(shoulder_corrected)
    elbow = math.radians(elbow_corrected)

    # Shoulder joint (relative to base)
    x1 = L1 * math.cos(shoulder)
    z1 = L1 * math.sin(shoulder)

    # Elbow joint (relative to shoulder)
    x2 = x1 + L2 * math.cos(shoulder + elbow)
    z2 = z1 + L2 * math.sin(shoulder + elbow)

    # Rotate everything by base rotation (around Z axis)
    x1_final = x1 * math.cos(base)
    y1_final = x1 * math.sin(base)

    x2_final = x2 * math.cos(base)
    y2_final = x2 * math.sin(base)

    # Points
    points_x = [0, x1_final, x2_final]
    points_y = [0, y1_final, y2_final]
    points_z = [0, z1, z2]

    # Plot
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(points_x, points_y, points_z, 'o-', linewidth=5, markersize=8, label='Robot Arm')
    ax.scatter([x2_final], [y2_final], [z2], color='green', label='Gripper', s=100)

    # Label Link 1 (Upper Arm)
    mid_x1 = (0 + x1_final) / 2
    mid_y1 = (0 + y1_final) / 2
    mid_z1 = (0 + z1) / 2
    ax.text(mid_x1, mid_y1, mid_z1, "Link 1", color='red', fontsize=10)

    # Label Link 2 (Forearm)
    mid_x2 = (x1_final + x2_final) / 2
    mid_y2 = (y1_final + y2_final) / 2
    mid_z2 = (z1 + z2) / 2
    ax.text(mid_x2, mid_y2, mid_z2, "Link 2", color='blue', fontsize=10)

    if target_xyz:
        ax.scatter([target_xyz[0]], [target_xyz[1]], [target_xyz[2]], color='red', label='Target', s=100)

    ax.set_xlim(-60, 60)
    ax.set_ylim(-60, 60)
    ax.set_zlim(-10, 60)
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.set_title('Robot Arm Reaching Toward Target')
    ax.grid(True)
    ax.legend()
    plt.show()

bx = 47
by = 1
bz = 12.7

base_angle = 57.53
shoulder_angle = 82.86
elbow_angle = 181
target = (68 - bx , 34 - by, 5 - bz)

draw_robot_arm(base_angle, shoulder_angle, elbow_angle, target_xyz=target)