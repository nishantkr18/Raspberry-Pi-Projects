import time
import cv2
from math import floor
import numpy as np

c = 0
cap = cv2.VideoCapture(0)
print (cap.get(3),cap.get(4))
cap.set(3, 600)
cap.set(4, 720)
print (cap.get(3),cap.get(4))
startTime = time.time()
while True:
    ret, capturedFrame = cap.read()
    cv2.imshow('OrgImg', capturedFrame)
    # modifiedFrame = imchange(capturedFrame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    c += 1
    if(time.time()-startTime>=1):
        startTime = time.time()
        print(c, (time.time()))
        c=0
