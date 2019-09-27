import RPi.GPIO as GPIO  
from time import sleep     # this lets us have a time delay (see line 15)  
GPIO.setmode(GPIO.BOARD)     # set up BCM GPIO numbering  
GPIO.setup(3, GPIO.IN)
GPIO.setup(8, GPIO.IN)
GPIO.setup(5, GPIO.OUT)  
GPIO.setup(7, GPIO.OUT)
GPIO.setup(10, GPIO.OUT)        
p = GPIO.PWM(10, 50)
p.start(0)

GPIO.output(5, False)
GPIO.output(7, True)

p.ChangeDutyCycle(7.5)    # Changes the pulse width to 12 (so moves the servo)
sleep(2)


def doStuff():
    if GPIO.input(8): # if port 25 == 1  
        print "Object Detected- No Moisture, Sorry" 
        p.ChangeDutyCycle(2.5)     # Changes the pulse width to 3 (so moves the servo)
        sleep(4)  
        p.ChangeDutyCycle(7.5)    # Changes the pulse width to 12 (so moves the servo)
        sleep(2)
    else:  
        print "Object Detected- Moisture detected"
        p.ChangeDutyCycle(12.5)     # Changes the pulse width to 3 (so moves the servo)
        sleep(4)  
        p.ChangeDutyCycle(7.5)    # Changes the pulse width to 12 (so moves the servo)
        sleep(2)
        
        
        

try:  
    while True:            # this will carry on until you hit CTRL+C  
        if GPIO.input(3): # if port 25 == 1  
            doStuff()
            sleep(0.5) 
        else:  
            print "No Object Detected"  
            sleep(0.5)    
  
finally:                   # this block will run no matter how the try block exits  
    GPIO.cleanup()         # clean up after yourself  
