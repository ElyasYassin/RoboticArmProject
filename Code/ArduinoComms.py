import serial
import time

ser = serial.Serial('COM3', 9600)
time.sleep(2)  

# Base: A, lower: B, upper: C, gripper G
"""
const int BASE_MIN     = 60,  BASE_MAX     = 120;
const int LOWER_MIN    = 50,  LOWER_MAX    = 155;
const int UPPER_MIN    = 0,  UPPER_MAX    = 180;
const int GRIPPER_MIN  = 55,   GRIPPER_MAX  = 180;
"""

commands = [
    "A:90; B:50;C:130; G:180\n", 
    "A:90; B:50;C:130;G:0\n", 
     "A:130; B:90;C:90; G:0\n", 
     "A:130; B:50;C:130; G:0\n", 
    "A:130; B:50;C:130; G:180\n", 
    "A:90; B:90;C:90; G:180\n", 

]

# Send each command with a delay between them
for cmd in commands:
    ser.write(cmd.encode())
    print(f"Sent: {cmd.strip()}")
    time.sleep(3)  # Wait for movement to complete

# Close the serial connection
ser.close()