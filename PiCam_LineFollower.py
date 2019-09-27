import time
import cv2
import numpy as np
import serial
from picamera.array import PiRGBArray
from picamera import PiCamera
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
resol = [320, 240]
camera.resolution = (resol[0], resol[1])
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(resol[0], resol[1]))
centre_x = int(resol[0]/2)
centre_y = int(resol[1]/2)
 
# allow the camera to warmup
time.sleep(0.1)
fov = 40
frameRate = 0
rightMaxSpeed = 100
leftMaxSpeed = 100
rightBaseSpeed = 60
leftBaseSpeed = 60
Kp = 0.2
Kd = 0
startTime = time.time()
c = 0
lastError = 0
ser = serial.Serial("/dev/ttyACM0", 9600)
print("connection Est");
def stopAll():
    ser.write(str(0)+" "+str(0)+'\n')
def move(error):
    global lastError #as lastError = error is used in next lines which will create a new local variable
    prop = Kp * error
    derv = Kd * (error - lastError)
    lastError = error
    motorSpeed = int(prop + derv)
    rightMotorSpeed = rightBaseSpeed - motorSpeed
    leftMotorSpeed = leftBaseSpeed + motorSpeed
    if (rightMotorSpeed > rightMaxSpeed):
        rightMotorSpeed = rightMaxSpeed
    if (leftMotorSpeed > leftMaxSpeed):
        leftMotorSpeed = leftMaxSpeed
    if (rightMotorSpeed < 0):
        rightMotorSpeed = 0
    if (leftMotorSpeed < 0):
        leftMotorSpeed = 0
    print(leftMotorSpeed, rightMotorSpeed, frameRate, prop)
    ser.write(str(leftMotorSpeed)+" "+str(rightMotorSpeed)+'\n')

def imchange(orgFrame):
   # cv2.imshow('orgFrame', orgFrame)
    frame = cv2.inRange(orgFrame, (0, 0, 0), (80, 80, 80))
   # cv2.imshow('masked', frame)
    kernel = np.ones((3, 3), np.uint8)
    frame = cv2.erode(frame, kernel, iterations=5)
    frame = cv2.dilate(frame, kernel, iterations=9)
   # cv2.imshow('eroded', frame)
    contours, hierarchy = cv2.findContours(frame.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(orgFrame, contours, -1, (0, 255, 0), 3)
    #cv2.imshow('contours', orgFrame)
    def checkForContours(contours):
        for contour in contours:
            for point in contour:
                if (point[0][1] <= centre_y + fov and point[0][1] >= centre_y - fov):
                    return True, contour
        return False, 0
    if len(contours)>0:
        cv2.rectangle(orgFrame, (0, centre_y - fov), (centre_x * 2, centre_y + fov), (200, 200, 0), 2)
        cv2.line(orgFrame, (centre_x, 0), (centre_x, centre_y*2), (0, 255, 255), 1)
        flag, contour = checkForContours(contours)
        if(flag==True):
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(orgFrame, (x, y), (x+w, y+h), (0, 0, 255), 4)

            # cv2.line(orgFrame, (x+int(w/2), 220), (x+int(w/2), 260), (255, 0, 0), 3)
            cv2.circle(orgFrame, (x + int(w / 2), centre_y), 5, (0, 255, 255), 10)
            
            error = (x+int(w/2)) - centre_x
            # cv2.putText(orgFrame, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(orgFrame, str(error), (centre_x, int(centre_y/4)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            move(error)

        #else:
            #stopAll()
        cv2.imshow("final", orgFrame)
    return frame


 

 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    modifiedFrame = imchange(image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stopAll()
        break
    c += 1
    if(time.time()-startTime>=1):
        startTime = time.time()
        frameRate = c
        c=0

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    
