from Ultrasonic import *
from Control import *
import myCode

c=Control() 		#Creating object 'control' of 'Control' class.
ultrasonic=Ultrasonic() #Creating object 'ultrasonic' of 'Ultrasonic' class.               

try:
    while True:
        data = ultrasonic.getDistance()
        print(str(data))
	
        if(data <= 20):
          
          #how to make it stop
          myCode.stop()
          print("sleep for a bit")
          time.sleep(3)	
          
          myCode.rotateCW(30)
          time.sleep(3)	
          distanceRight=ultrasonic.getDistance()
          print("dist right: ", str(distanceRight))
          myCode.rotateCCW(60)
          time.sleep(3)	
          distanceLeft=ultrasonic.getDistance()
          print("dist left: ", str(distanceLeft))
          myCode.rotateCW(30)
          
          myCode.back()
          if(distanceRight >= distanceLeft):
            myCode.rotateCW(30)
          else:
            myCode.rotateCCW(30)
	
        myCode.forward()
except KeyboardInterrupt:
        myCode.relax()
        print("\nEnd of program")
