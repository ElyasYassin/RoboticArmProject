import serial
import time

ser = serial.Serial('COM3', 9600)
time.sleep(2)  

# Base: A, upper: B, lower: C, gripper G


commands = [
     "B:65;C:55;G:0\n",   # Initial position
    "B:75;C:65;G:180\n",   
    "B:85;C:75;G:0\n",   
    "B:95;C:85;G:0\n",   
    "B:105;C:95;G:180\n",   

]

# Send each command with a delay between them
for cmd in commands:
    ser.write(cmd.encode())
    print(f"Sent: {cmd.strip()}")
    time.sleep(1)  # Wait for movement to complete

# Close the serial connection
ser.close()