import math
import csv

# Example FK function (you'll replace this with your actual robot's FK)
def forward_kinematics(base_angle_deg, shoulder_angle_deg, elbow_angle_deg):
    # Example arm link lengths (cm)
    L1 = 20.955  # Upper arm
    L2 = 22.225  # Forearm
    
    # Correct angles based on physical arm setup
    shoulder_corrected = shoulder_angle_deg - 120
    elbow_corrected = 140 - elbow_angle_deg

    # Convert to radians
    base = math.radians(base_angle_deg)
    shoulder = math.radians(shoulder_corrected)
    elbow = math.radians(elbow_corrected)
    

    # Planar 2D FK first (shoulder + elbow contribution)
    x = (L1 * math.cos(shoulder)) + (L2 * math.cos(shoulder + elbow))
    z = (L1 * math.sin(shoulder)) + (L2 * math.sin(shoulder + elbow))

    # Base rotation adds the y component (spinning around vertical axis)
    x_final = x * math.cos(base)
    y_final = x * math.sin(base)
    z_final = z  # Height stays the same

    print(x_final, y_final, z_final)
    return (x_final, y_final, z_final)


if __name__ == "__main__":
    print("Getting forward kinematics")
    
    # List of test poses: (base, shoulder, elbow)
    test_poses = [
        (90, 50, 40),
        (60, 90, 45),
        (120, 50, 60),
        (90, 90, 0),
        (0, 50, 45),
    ]

    # List to hold results
    results = []

    print("Starting Calibration Logger...\n")

    # Go through each pose
    for i, (base_angle, shoulder_angle, elbow_angle) in enumerate(test_poses):
        print(f"Test {i+1}: Move robot to Base={base_angle}°, Shoulder={shoulder_angle}°, Elbow={elbow_angle}°")
        
        # Compute expected position with FK
        x_expected, y_expected, z_expected = forward_kinematics(base_angle, shoulder_angle, elbow_angle)
        print(f"Predicted Position (FK): X={x_expected:.2f}cm, Y={y_expected:.2f}cm, Z={z_expected:.2f}cm")
        
        Xb = 47
        Yb = 1
        Zb = 12.7
        
        
        # Wait for user to measure and input the real-world position
        x_real = float(input("Enter measured X (cm): ")) - Xb
        y_real = float(input("Enter measured Y (cm): ")) - Yb
        z_real = float(input("Enter measured Z (cm): ")) - Zb
        
        # Calculate offsets
        error_x = x_real - x_expected
        error_y = y_real - y_expected
        error_z = z_real - z_expected
        total_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)

        print(f"Error: ΔX={error_x:.2f}cm, ΔY={error_y:.2f}cm, ΔZ={error_z:.2f}cm, Total Error={total_error:.2f}cm\n")

        # Save the result
        results.append({
            'Test #': i+1,
            'Base Angle (deg)': base_angle,
            'Shoulder Angle (deg)': shoulder_angle,
            'Elbow Angle (deg)': elbow_angle,
            'X Predicted': x_expected,
            'Y Predicted': y_expected,
            'Z Predicted': z_expected,
            'X Real': x_real,
            'Y Real': y_real,
            'Z Real': z_real,
            'Error X': error_x,
            'Error Y': error_y,
            'Error Z': error_z,
            'Total Error': total_error
        })

    # Save results to CSV
    csv_filename = "fk_vs_real_calibration_results_Aruco_Code_Calibration.csv"
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=results[0].keys())
        writer.writeheader()
        for data in results:
            writer.writerow(data)

    print(f"\nCalibration data saved to {csv_filename}!")