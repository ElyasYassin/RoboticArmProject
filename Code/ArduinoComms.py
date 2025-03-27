## This is the code for sending data to the Arduino
import serial
import time

# Open Serial Port (Adjust to match your Arduino)
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Wait for connection

# Send angles to Arduino
theta1, theta2 = 45, 30  # Example values
command = f"{int(theta1)} {int(theta2)}\n"
arduino.write(command.encode())

# Close serial connection
arduino.close()