import cv2
import serial
import time
ser = serial.Serial("/dev/ttyACM0", 9600)
print("Connection");

while True:
    i = raw_input('Enter')
    ser.write(i+'\n')
    print(i)
ser.write(str(0)+" "+str(0)+'\n')
