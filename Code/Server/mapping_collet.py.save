from Ultrasonic import *
import myCode
from Servo import *
import csv

#set up, initialize
ultrasonic=Ultrasonic()
f = open('data.csv', 'w')
writer = csv.writer(f)
servo = Servo()
    
#values
counter = 1 # timer
tilt = 5    # vertical

#Tilting
for i in range(6,14,2):
    tilt += 5    #modulus 360?
                       #50-180
    servo.setServoAngle(0,i*10)
    time.sleep(0.4)
    
    pan = 0       #horizontal
    
    #Panning
    for j in range (15,-16,-1):
        pan += 5    
        counter += 1
        myCode.attitude(0,0,j)
        data = [(counter+1), pan, tilt, ultrasonic.getDistance()]
        writer.writerow(data)
        time.sleep(0.1)
#servo.setServoAngle(0,90)

    
#print to data.csv
f.close()

