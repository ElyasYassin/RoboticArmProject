from InverseKinematics import inverse_kinematics, forward_kinematics, optimize_ik
from ArduinoComms import send_serial_command
from VoiceCommand import speak
from CVCode import start_cv_thread, stop_cv_thread, detected_objects
from ColorDetection import detect_color

import time

# Define bin coordinates for each color
bin_coordinates = {
    "green": None,   
    "purple": None
}

def main_loop():

    
    speak("Robotic arm system online and ready.")

    while True:
        speak("Waiting for object placement on gripper.")

        # Auto-detect color at gripper
        detected_color = detect_color(camera_index=1, max_attempts=10, visualize=True)
        time.sleep(2)
        
        if detected_color not in bin_coordinates:
            speak(f"Unknown object color: {detected_color}. Waiting...")
            time.sleep(2)
            continue

        speak(f"Trying to locate {detected_color.capitalize()} bin.")

        start_cv_thread()
        print(bin_coordinates["green"])
        print(bin_coordinates["purple"])
        
        time.sleep(3)  
        
        wait_time = 0
        max_wait = 10  
        found_bin = False
        
        while wait_time < max_wait:
            if detected_color in detected_objects:
                bin_coordinates[detected_color] = detected_objects[detected_color]
                found_bin = True
                break
            time.sleep(0.2)
            wait_time += 0.2
        
        
        stop_cv_thread()
        
        if not found_bin:
            speak(f"Could not locate {detected_color} bin. Waiting...")
            continue
        
        bin_x, bin_y = bin_coordinates[detected_color]
        bin_z = 25  
        
        speak(f"{detected_color.capitalize()} bin located. Moving object.")

        
        
        try:

            # Solve IK for bin placement
            base_angle, shoulder_angle, elbow_angle = inverse_kinematics(bin_x, bin_y, bin_z)
            base_angle, shoulder_angle, elbow_angle = optimize_ik(base_angle, shoulder_angle, elbow_angle, bin_x, bin_y, bin_z)

            print(base_angle, shoulder_angle, elbow_angle)
            
            # Send movement to Arduino
            
            speak(f"Moving towards the {detected_color.capitalize()} bin.")

            send_serial_command(90, 90, 90, 0)
            time.sleep(4)
            send_serial_command(base_angle, shoulder_angle, 180 - elbow_angle, 0)
            time.sleep(8)
            send_serial_command(base_angle, shoulder_angle, 180 - elbow_angle, 180)
            time.sleep(9)
            send_serial_command(90, 90, 90, 80)
            time.sleep(3)
             
            speak(f"{detected_color.capitalize()} object placed in bin. Returning to standby.")
            time.sleep(2)

        except Exception as e:
            print(f"Movement error: {e}")
            speak("Error during movement. Returning to standby.")
            time.sleep(2)
            
main_loop()
