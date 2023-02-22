from Ultrasonic import *
from Control import *
import myCode
from Servo import *


c=Control() 		#Creating object 'control' of 'Control' class.
ultrasonic=Ultrasonic() #Creating object 'ultrasonic' of 'Ultrasonic' class. 
# servo = Servo()
# servo.setServoAngle(0,9*10)

try:
    while True:
        #           inches, gait, left/right, go forward/backward, speed, move at aan angle
        # myCode.move(1, 2, 0, 12, 11, 0)
        
        data = ultrasonic.getDistance()
        print(str(data))
            
        # check distance
        if (data < 20):
            # move back
            myCode.move(2, 2, 0, -12, 11, 0)
            
            # stop
            # myCode.move(0, 2, 0, 0, 9, 0)
            time.sleep(1)	          
          
            # measure
            data = ultrasonic.getDistance()
            print(str(data))
            
            # move right
            myCode.move(1, 2, 12, 0, 9, 0)
            time.sleep(3)	
            distanceRight=ultrasonic.getDistance()
            print("dist right: ", str(distanceRight))
            
        elif (data >= 20):
            myCode.move(1, 2, 0,  12, 11, 0)
                
            
            
except KeyboardInterrupt:
        myCode.relax()
        print("\nEnd of program")
            

