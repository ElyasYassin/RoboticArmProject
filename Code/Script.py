from InverseKinematics import inverse_kinematics, forward_kinematics, optimize_ik
from ArduinoComms import send_serial_command
from VoiceCommand import speak
from CVCode import start_cv_thread, detected_objects
from ColorDetection import detect_color

import time

# Define bin coordinates for each color
bin_coordinates = {
    "green": (30, 10, 25),   
    "purple": (10, 30, 25)
}

def main_loop():
    # Start background CV thread to update detected objects
    start_cv_thread()
    time.sleep(3)  # Let thread warm up

    while True:
        speak("Waiting for object placement on gripper.")

        # Auto-detect color at gripper
        detected_color = detect_color(camera_index=1, max_attempts=10, visualize=True)

        if detected_color in bin_coordinates:
            speak(f"{detected_color.capitalize()} object detected on gripper. Preparing to move to bin.")

            # Move directly to bin corresponding to detected color
            bin_x, bin_y, bin_z = bin_coordinates[detected_color]

            # Solve IK for bin placement
            base_angle, shoulder_angle, elbow_angle = inverse_kinematics(bin_x, bin_y, bin_z)
            base_angle, shoulder_angle, elbow_angle = optimize_ik(base_angle, shoulder_angle, elbow_angle, bin_x, bin_y, bin_z)

            # Send movement to Arduino
            send_serial_command(base_angle, shoulder_angle, elbow_angle)
            speak(f"{detected_color.capitalize()} object placed in bin. Returning to standby.")
            time.sleep(2)

        else:
            speak("No valid object detected on gripper. Waiting...")
            time.sleep(2)
            
            
main_loop()