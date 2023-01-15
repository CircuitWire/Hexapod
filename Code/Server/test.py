import time
import os
from Led import *
from Control import *

print(os.getcwd())
led=Led()

def test_Led():
    try:
        #Red wipe
        print ("\nRed wipe")
        led.colorWipe(led.strip, Color(255, 0, 0)) 
        time.sleep(1)
        
        
        #Green wipe
        print ("\nGreen wipe")
        led.colorWipe(led.strip, Color(0, 255, 0)) 
        time.sleep(1)
        
        
        #Blue wipe
        print ("\nBlue wipe")
        led.colorWipe(led.strip, Color(0, 0, 255)) 
        time.sleep(1)
        
        
        #White wipe
        print ("\nWhite wipe")
        led.colorWipe(led.strip, Color(255, 255, 255)) 
        time.sleep(1)
        
        led.colorWipe(led.strip, Color(0, 0, 0))   #turn off the light
        print ("\nEnd of program")
    except KeyboardInterrupt:
        led.colorWipe(led.strip, Color(0, 0, 0))   #turn off the light
        print ("\nEnd of program")

def test_rgb_Led():
    try:
        print ("\Going up in R")
        for i in range(255):
            led.colorWipe(led.strip, Color(i, 0, 0)) 
            time.sleep(.05)
        
        for i in range(255):                
            led.colorWipe(led.strip, Color(255-i, i, 0)) 
            time.sleep(0.05)
        
        for i in range(255):
            led.colorWipe(led.strip, Color(0, 255-i, i)) 
            time.sleep(0.05)
        
        led.colorWipe(led.strip, Color(0, 0, 0))   #turn off the light
        print ("\nEnd of program")
    except KeyboardInterrupt:
        led.colorWipe(led.strip, Color(0, 0, 0))   #turn off the light
        print ("\nEnd of program")

def david_san_test_forward():
    # Looking at the Canvas Hexapod documentation to see ID's of the legs
    # would prob be wrong; but just coding something out to start
    ARM_MOVEMENT = 10
    LEG_MOVEMENT = 50
    def _forward1():
        # Lift arm up?
        for i in range(ARM_MOVEMENT):
            servo.setServoAngle(23,90+i)
            servo.setServoAngle(11,90+i)
            servo.setServoAngle(17,90+i)
            time.sleep(0.005)
        # turn 3 legs forward?
        for i in range(LEG_MOVEMENT):
            servo.setServoAngle(22, 90+i)
            servo.setServoAngle(12, 90+i)
            servo.setServoAngle(18, 90+i)
            time.sleep(0.005)
        # put arms down?
        for i in range(ARM_MOVEMENT):
            servo.setServoAngle(23,(90+ARM_MOVEMENT)-i)
            servo.setServoAngle(11,(90+ARM_MOVEMENT)-i)
            servo.setServoAngle(17,(90+ARM_MOVEMENT)-i)
            time.sleep(0.005)
        
    def _forward2():
        # Lift other arms up?
        for i in range(ARM_MOVEMENT):
            servo.setServoAngle(8,90+i)
            servo.setServoAngle(20,90+i)
            servo.setServoAngle(14,90+i)
            time.sleep(0.005)
        # turn other 3 legs forward?
        for i in range(LEG_MOVEMENT):
            servo.setServoAngle(9, 90+i)
            servo.setServoAngle(19, 90+i)
            servo.setServoAngle(15, 90+i)
            time.sleep(0.005)
        # put other arms down?
        for i in range(ARM_MOVEMENT):
            servo.setServoAngle(8,(90+ARM_MOVEMENT)-i)
            servo.setServoAngle(20,(90+ARM_MOVEMENT)-i)
            servo.setServoAngle(14,(90+ARM_MOVEMENT)-i)
            time.sleep(0.005)
        
    _forward1()
    _forward2()
    arms = [19, 9, 15]
    #for arm in arms:
    #    servo.setServoAngle(arm, 90)
    print ("\nEnd of program")   

from Ultrasonic import *
ultrasonic=Ultrasonic()                
def test_Ultrasonic():
    try:
        while True:
            data=ultrasonic.getDistance()   #Get the value
            print ("Obstacle distance is "+str(data)+"CM")
            time.sleep(1)
    except KeyboardInterrupt:
        print ("\nEnd of program")

from Servo import *
servo=Servo()
#for i in range(90):
     #  servo.setServoAngle(,i) 
     #  time.sleep(0.005)
def test_Servo():
    try:
        #for inner motor the higher the angle it goes back, 
        #the lowerthe angle is towards the head

        #for middle motor 14, the lower the angle 10 points leg down
        #for middle motor 14, the higher the angle 90 points leg up

        #for outermost motor 13, the higher the angle 90 points leg inward/down
        #for outermost motor 13, the lower the angle 10 points leg outward/up
        for i in range(20):
            servo.setServoAngle(13,10+i)
            servo.setServoAngle(13,10+i)        
        
        c=Control()
        time.sleep(2)
        c.relax(True)
    except KeyboardInterrupt:
        print ("\nEnd of program")
        
        
from ADC import *
adc=ADC()
def test_Adc():
    try:
        while True:
            Power=adc.batteryPower()
            print ("The battery voltage is "+str(Power)+'\n')
            time.sleep(1)
    except KeyboardInterrupt:
        print ("\nEnd of program")

from Buzzer import *
buzzer=Buzzer()
def test_Buzzer():
    try:
        buzzer.run('1')
        time.sleep(1)
        print ("1S")
        time.sleep(1)
        print ("2S")
        time.sleep(1)
        print ("3S")
        buzzer.run('0')
        print ("\nEnd of program")
    except KeyboardInterrupt:
        buzzer.run('0')
        print ("\nEnd of program")
        
import threading        
from Control import *
# Main program logic follows:
def aa():
    while True:
        test_Led()
        #Power=adc.batteryPower()
        #print ("The battery voltage is "+str(Power)+'\n')
        data=ultrasonic.getDistance()   #Get the value
        print ("Obstacle distance is "+str(data)+"CM")
def bb():
    while True:
        for i in range(30,150,1):
            servo.setServoAngle(1,i)
            time.sleep(0.05)
        for i in range(150,30,-1):
            servo.setServoAngle(1,i)
            time.sleep(0.05)
        for i in range(90,150,1):
            servo.setServoAngle(0,i)
            time.sleep(0.05)
        for i in range(150,90,-1):
            servo.setServoAngle(0,i)
            time.sleep(0.05)
if __name__ == '__main__':
    print ('Program is starting ... ')
    import sys
    if len(sys.argv)<2:
        print ("Parameter error: Please assign the device")
        exit() 
    if sys.argv[1] == 'Led':
        test_Led()
    elif sys.argv[1] == 'Ultrasonic':
        test_Ultrasonic()
    elif sys.argv[1] == 'Servo': 
        test_Servo()               
    elif sys.argv[1] == 'ADC':   
        test_Adc()  
    elif sys.argv[1] == 'Buzzer':   
        test_Buzzer()
    elif sys.argv[1] == 'DTS':
        david_san_test_forward()
        
        
        
        
