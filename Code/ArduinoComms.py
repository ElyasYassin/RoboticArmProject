import serial
import time

ser = serial.Serial('COM3', 9600)
time.sleep(2)  

# Base: A, upper: B, lower: C, gripper G

command = "A:90;B:180;C:0;G:0\n"
ser.write(command.encode())
ser.close()