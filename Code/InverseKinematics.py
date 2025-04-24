import math

# Arm link lengths (in cm)
L1 = 21.0
L2 = 18.2  + 7

# Servo safe limits
BASE_MIN = 0
BASE_MAX = 180

SHOULDER_MIN = 50
SHOULDER_MAX = 155

ELBOW_MIN = 0
ELBOW_MAX = 180

def clamp(angle, min_val, max_val):
    return max(min_val, min(angle, max_val))

def inverse_kinematics_3d(x, y, z):
    # --- 1. Base rotation (XZ plane) ---
    base_rad = math.atan2(x, z)
    base_deg = math.degrees(base_rad)
    base_servo = clamp(90 + base_deg, BASE_MIN, BASE_MAX)

    # --- 2. Project to arm plane (YZ) ---
    planar_dist = math.sqrt(x**2 + z**2)
    target_y = y
    D = math.sqrt(planar_dist**2 + target_y**2)

    # --- 3. Check reachability ---
    if D > (L1 + L2) or D < abs(L1 - L2):
        raise ValueError("Target is out of reach")

    # --- 4. Elbow angle (θ2) ---
    cos_theta2 = (planar_dist**2 + target_y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_theta2)
    elbow_deg = clamp(90 + math.degrees(theta2), ELBOW_MIN, ELBOW_MAX)

    # --- 5. Shoulder angle (θ1) ---
    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(target_y, planar_dist) - math.atan2(k2, k1)
    shoulder_deg = clamp(90 + math.degrees(theta1), SHOULDER_MIN, SHOULDER_MAX)

    return {
        "base": round(base_servo, 2),
        "shoulder": round(shoulder_deg, 2),
        "elbow": round(elbow_deg, 2)
    }

    
try:
    result = inverse_kinematics_3d(x=10, y=15, z=20)
    print(f"Base: {result['base']}°, Shoulder: {result['shoulder']}°, Elbow: {result['elbow']}°")
except ValueError as e:
    print("Error:", e)