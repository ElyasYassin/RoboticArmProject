import serial
import time

# Open serial port once at the start
ser = serial.Serial('COM3', 9600)
time.sleep(2)  # Let Arduino reset after serial open

def send_serial_command(base_angle, shoulder_angle, elbow_angle, gripper_angle=115):
    """
    Sends a command to the Arduino to set servo positions.
    """
    # Build the command string exactly like your Arduino expects
    cmd = f"A:{base_angle:.2f}; B:{shoulder_angle:.2f}; C:{elbow_angle:.2f}; G:{gripper_angle}\n"
    ser.write(cmd.encode())
    print(f"Sent: {cmd.strip()}")

# Don't forget to close serial connection when finished!
def close_serial():
    ser.close()
    
    
if __name__ == "__main__":
    send_serial_command(117.35, 49.65, 169.65)
    time.sleep(5)
    send_serial_command(117.35, 49.65, 169.65, 0)
    time.sleep(5)
    send_serial_command(90, 90, 90, 0)
    time.sleep(5)
    send_serial_command(90, 90, 90, 180)

    
    