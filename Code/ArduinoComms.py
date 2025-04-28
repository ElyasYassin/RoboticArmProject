import serial
import time

# COM3 is the serial connection
ser = serial.Serial('COM3', 9600)
time.sleep(2)  

def send_serial_command(base_angle, shoulder_angle, elbow_angle, gripper_angle=115):
    """
    Sends a command to the Arduino to set servo positions.
    """
    # The angles that get sent
    cmd = f"A:{base_angle:.2f}; B:{shoulder_angle:.2f}; C:{elbow_angle:.2f}; G:{gripper_angle}\n"
    ser.write(cmd.encode())
    print(f"Sent: {cmd.strip()}")

def close_serial():
    ser.close()
    
    
if __name__ == "__main__":
    time.sleep(2)
    send_serial_command(90, 90, 90, 0)
    time.sleep(4)
    send_serial_command(110.12, 61.68, 160.8)
    time.sleep(5)
    send_serial_command(110.12, 61.68, 160.8, 180)
    time.sleep(3)
    send_serial_command(90, 90, 90, 180)

    
    