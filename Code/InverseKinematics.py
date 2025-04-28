import math
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# --- Robot Parameters ---
BASE_X = 47  # cm
BASE_Y = -3   # cm
BASE_Z = 10  # cm
BASE_HEIGHT = BASE_Z
L1 = 21  # Upper arm length (cm)
L2 = 22  # Forearm length (cm)

# --- Base Rotation (Yaw) ---
def solve_base_rotation(x_base, y_base, x_target, y_target):
    dx = x_target - x_base
    dy = y_target - y_base
    theta_base_rad = math.atan2(dy, dx)
    theta_base_deg = math.degrees(theta_base_rad)
    base_servo_angle = theta_base_deg
    return base_servo_angle

# --- Corrected Shoulder and Elbow Solver ---
def solve_shoulder_elbow(x_base, y_base, z_base, x_target, y_target, z_target):
    dx = x_target - x_base
    dy = y_target - y_base
    dz = z_target - z_base

    r = math.sqrt(dx**2 + dy**2)
    d = math.sqrt(r**2 + dz**2)

    if d > (L1 + L2):
        raise ValueError(f"Target is out of reach. Distance {d:.2f} cm > Max reach {L1 + L2} cm.")

    cos_elbow = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_rad = math.acos(cos_elbow)
    elbow_angle_deg = math.degrees(elbow_rad)

    phi = math.atan2(dz, r)
    cos_shoulder = (d**2 + L1**2 - L2**2) / (2 * d * L1)
    cos_shoulder = max(-1.0, min(1.0, cos_shoulder))
    shoulder_offset_rad = math.acos(cos_shoulder)

    shoulder_rad = phi + shoulder_offset_rad
    shoulder_angle_deg = math.degrees(shoulder_rad)

    shoulder_angle_deg = max(0, min(90, shoulder_angle_deg))
    elbow_angle_deg = max(0, min(90, elbow_angle_deg))

    return shoulder_angle_deg, elbow_angle_deg

# --- Full Inverse Kinematics Solver ---
def inverse_kinematics(x_target, y_target, z_target):
    base_servo = solve_base_rotation(BASE_X, BASE_Y, x_target, y_target)
    shoulder_servo, elbow_servo = solve_shoulder_elbow(BASE_X, BASE_Y, BASE_Z, x_target, y_target, z_target)
    return base_servo, shoulder_servo, elbow_servo

# --- Forward Kinematics ---
def forward_kinematics(base_angle, shoulder_angle, elbow_angle):
    base_rad = math.radians(base_angle)
    shoulder_rad = math.radians(shoulder_angle)
    elbow_rad = math.radians(elbow_angle)

    forearm_rad = shoulder_rad + elbow_rad - math.radians(90)

    x_upper = L1 * math.cos(shoulder_rad) * math.cos(base_rad)
    y_upper = L1 * math.cos(shoulder_rad) * math.sin(base_rad)
    z_upper = BASE_HEIGHT + L1 * math.sin(shoulder_rad)

    x_gripper = x_upper + L2 * math.cos(forearm_rad) * math.cos(base_rad)
    y_gripper = y_upper + L2 * math.cos(forearm_rad) * math.sin(base_rad)
    z_gripper = z_upper + L2 * math.sin(forearm_rad)

    return x_gripper, y_gripper, z_gripper

# --- Position Error ---
def position_error(x_target, y_target, z_target, x_current, y_current, z_current):
    return math.sqrt((x_target - x_current)**2 + (y_target - y_current)**2 + (z_target - z_current)**2)

# --- Cost Function for Optimization ---
def cost_function(angles, x_target, y_target, z_target):
    base_angle, shoulder_angle, elbow_angle = angles
    x_fk, y_fk, z_fk = forward_kinematics(base_angle, shoulder_angle, elbow_angle)
    x_fk_world = x_fk + BASE_X
    y_fk_world = y_fk + BASE_Y
    return position_error(x_target, y_target, z_target, x_fk_world, y_fk_world, z_fk)

# --- Optimization Solver ---
def optimize_ik(base_angle, shoulder_angle, elbow_angle, x_target, y_target, z_target):
    initial_guess = [base_angle, shoulder_angle, elbow_angle]
    result = minimize(cost_function, initial_guess, args=(x_target, y_target, z_target), method='Nelder-Mead',
                      options={'xatol': 1e-2, 'fatol': 1e-2, 'maxiter': 100})
    return result.x

# --- Plot Functions ---

def plot_xy_plane(x_target, y_target):
    plt.figure(figsize=(10, 8))
    plt.title("Base Rotation - XY Plane", fontsize=16)
    plt.plot([BASE_X, x_target], [BASE_Y, y_target], 'r-o', label="Base to Target", markersize=8)
    plt.xlabel("X (cm)", fontsize=14)
    plt.ylabel("Y (cm)", fontsize=14)
    plt.axhline(0, color='k', linestyle='--')
    plt.axvline(0, color='k', linestyle='--')
    plt.legend(fontsize=12)
    plt.grid(True)
    plt.gca().set_aspect('equal')
    plt.show()

def plot_yz_plane(shoulder_angle, elbow_angle):
    plt.figure(figsize=(10, 8))
    plt.title("Shoulder and Elbow Motion - YZ Plane", fontsize=16)

    plt.plot([0], [BASE_HEIGHT], 'ko', markersize=8)

    shoulder_rad = math.radians(shoulder_angle)
    elbow_rad = math.radians(elbow_angle)

    y_upper = L1 * math.cos(shoulder_rad)
    z_upper = BASE_HEIGHT + L1 * math.sin(shoulder_rad)
    plt.plot([0, y_upper], [BASE_HEIGHT, z_upper], 'b-o', label="Upper Arm", markersize=8)

    forearm_rad = shoulder_rad + elbow_rad - math.radians(90)

    y_forearm = y_upper + L2 * math.cos(forearm_rad)
    z_forearm = z_upper + L2 * math.sin(forearm_rad)
    plt.plot([y_upper, y_forearm], [z_upper, z_forearm], 'g-o', label="Forearm", markersize=8)

    plt.xlabel("Y (cm)", fontsize=14)
    plt.ylabel("Z (cm)", fontsize=14)
    plt.axhline(0, color='k', linestyle='--')
    plt.axvline(0, color='k', linestyle='--')
    plt.legend()
    plt.grid(True)
    plt.gca().set_aspect('equal')
    plt.show()

# --- Main Execution ---

if __name__ == "__main__":
    x_target = 51
    y_target = 35
    z_target = 25

    try:
        # Solve IK
        base_angle, shoulder_angle, elbow_angle = inverse_kinematics(x_target, y_target, z_target)

        print(f"Initial IK Angles:")
        print(f"Base: {base_angle:.2f}°")
        print(f"Shoulder: {shoulder_angle:.2f}°")
        print(f"Elbow: {elbow_angle:.2f}°")

        # Optimize using FK
        base_angle, shoulder_angle, elbow_angle = optimize_ik(base_angle, shoulder_angle, elbow_angle,
                                                              x_target, y_target, z_target)

        print(f"\nCorrected IK Angles:")
        print(f"Base: {base_angle:.2f}°")
        print(f"Shoulder: {shoulder_angle:.2f}°")
        print(f"Elbow: {elbow_angle:.2f}°")

        # Final FK
        x_fk, y_fk, z_fk = forward_kinematics(base_angle, shoulder_angle, elbow_angle)
        print(f"\nCorrected FK Gripper Position:")
        print(f"X: {x_fk + BASE_X:.2f} cm")
        print(f"Y: {y_fk + BASE_Y:.2f} cm")
        print(f"Z: {z_fk:.2f} cm")

        # Plot
        plot_xy_plane(x_target, y_target)
        plot_yz_plane(shoulder_angle, elbow_angle)

    except ValueError as e:
        print(f"Error: {e}")