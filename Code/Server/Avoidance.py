from Ultrasonic import *
from Control import *
import myCode

c=Control() 		#Creating object 'control' of 'Control' class.
ultrasonic=Ultrasonic() #Creating object 'ultrasonic' of 'Ultrasonic' class.               


ry:
    while True:
        data = ultrasonic.getDistance()
        print(str(data))
	
        if(data <= 20):
          
          
          time.sleep(1)	          
          
          #back        inches, gait, x, y, delay, angle
          myCode.move(3, 2, 0, -12, 11, 0)

          time.sleep(1)	
          #right
          myCode.attitude(0,0,-15)
          time.sleep(3)	
          distanceRight=ultrasonic.getDistance()
          print("dist right: ", str(distanceRight))
          
          
          #left
          myCode.attitude(0,0,15)
          time.sleep(3)	
          distanceLeft=ultrasonic.getDistance()
          print("dist left: ", str(distanceLeft))
                    
          
          
          if(distanceRight >= distanceLeft):
            #crab walk right
            myCode.move(3, 2, 12, 0, 11, 0)
          else:
            #crab walk left
            myCode.move(3, 2, -12, 0, 11, 0)
      
        # forward        
        myCode.move(3, 2, 0, 12, 11, 0)
except KeyboardInterrupt:
        myCode.relax()
        print("\nEnd of program")
