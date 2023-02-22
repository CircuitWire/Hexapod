
#to save battery, relax hex after moving

from Control import *
from Servo import *
from IMU import *
import time
import math
from Ultrasonic import *
import serial
import RPi.GPIO as GPIO

c=Control()
servo=Servo()

#data array:
#data=['CMD_MOVE', '1', '0', '25', '10', '0']
   #Move command:'CMD_MOVE'
   #Gait Mode: "1"       
     #Note: "2" is one leg at a time, "1" is three legs
   #Moving direction: x='0',y='25'
   #Delay:'10'
   #Action Mode : '0'   Angleless turn 

#========================= Fix Head to neutral position =====================
def fix_head():
    servo.setServoAngle(0,100)
    c.relax(True)

#========================== Movement for other Classes  ================
def move(inches, gait, x, y, delay, angle):
    
    gait = str(gait)
    x = str(x)
    y = str(y)    
    delay = str(delay)
    angle = str(angle)
    
    for i in range(inches):
        data=['CMD_MOVE', gait, x, y, delay, angle] 
        c.run(data)    
    #c.relax(True)

#========================== Movement  =====================================
def movement_without_param():
    gait = input("\nEnter Gait Mode ('1' is three legs, '2' is one leg, anything 3+ is Redundancy/Incline")
    x = input("Enter x-value: ")
    y = input("Enter y-value: ")
    delay = input("Enter delay-value (Ex: '1' is slow, '10' is fast): ")
    angle = input("Enter angle turn (or rotate if x,y were 0,0): ")
    for i in range(3):
        data=['CMD_MOVE', gait, x, y, delay, angle] 
        c.run(data)    
    #c.relax(True)

#========================== Forward  =====================================
#data=['CMD_MOVE', '2', '0', '12', '5', '0']
#                   |    |     |
#                   |    |     |
#      stable gait: 2,   x,    y = (0,12) perfect inch

def forward(x):
    for i in range(x):
        data=['CMD_MOVE', '2', '0', '12', '11', '0']
        c.run(data)
    #c.relax(True)

#============================ Left  =====================================

def left(x):
  for i in range(x):
    data=['CMD_MOVE', '1', '-12', '0', '10', '0']  
    c.run(data)
    #c.relax(True)
    
#========================== Right  =====================================

def right(x):
  for i in range(x):
    data=['CMD_MOVE', '2', '12', '0', '12', '0']  
    c.run(data)
    #c.relax(True)
    
#========================== Backward  ====================================

def back():
  for i in range(2):
    data=['CMD_MOVE', '1', '0', '-12', '10', '0']  
    c.run(data)
    #c.relax(True)
    
#========================= Servo =========================================
def test_Servo():
    print("   \\              /")
    print("    18    ||    13")
    print("     \\    ||    /")
    print("      17  ||  14")
    print("       \\ head /")
    print("        16  15 ")
    print("        ======")
    print("21-20-19|    |12-11-10")
    print("        ======")
    print("       22    9 ")
    print("       /      \\")
    print("     23        8")
    print("     /          \\")
    print("   27            31")
    print("   /              \\")
    channel = int(input("Enter a channel for Single Servo: "))

    
    try:
        #for inner motor the higher the angle it goes back, 
        #the lowerthe angle is towards the head

        #for middle motor 14, the lower the angle 10 points leg down
        #for middle motor 14, the higher the angle 90 points leg up

        #for outermost motor 13, the higher the angle 90 points leg inward/down
        #for outermost motor 13, the lower the angle 10 points leg outward/up
        for i in range(20):
            servo.setServoAngle(channel,10+i)
            servo.setServoAngle(channel,10+i)
        
        c=Control()
        time.sleep(1)
        c.relax(True)
    except KeyboardInterrupt:
        c.relax(True)
        print ("\nEnd of program")
        
#===========================   IMU    =================================
def test_imu():
    s=IMU()
    time1=time.time()
    while True:
        try:    
            time.sleep(0.5)
            roll,pitch,yaw=s.imuUpdate()
            print(roll,pitch,yaw)
        except Exception as e:
            print(e)
            os.system("i2cdetect -y 1")
            break

#================   IMU Gyro and Accelerameter    =======================
def test_imu_gyro():
    s=IMU()
    time1=time.time()
    while True:
        try:    
            time.sleep(0.5)
            accel_data,gyro_data=s.average_filter()
            print(gyro_data)
        except Exception as e:
            print(e)
            os.system("i2cdetect -y 1")
            break
            
#==================== Push Up(use -12, 12) ==============================
def pushup():
               #x,y,z
	c.posittion(0,0,5)      #push up
	c.posittion(0,0,26)      #push up
	time.sleep(0.5)
	c.posittion(0,0,0)      #go down
	c.posittion(0,0,-26)      #go down
	time.sleep(1)
	c.posittion(0,0,0)      #push up

	time.sleep(1)
	c.relax(True)

#======================= Figure 8  =====================================
#revisit method to change size of figure eight in method parameter
def figure8():
  for i in range(5):
      data=['CMD_MOVE', '1', '0', '25', '10', '10']  		#figure 8 clock wise first, change signs of last 10 (action mode) to change direction
      c.run(data)
  for i in range(10):
      data=['CMD_MOVE', '1', '0', '25', '10', '-10'] 
      c.run(data)   
  for i in range(5):
      data=['CMD_MOVE', '1', '0', '25', '10', '10'] 
      c.run(data)
      c.relax(True)
  
#======================= rotate CW Method with sys.argv[2] =========================
      
def rotateCW(rotation):
    rotation = int(rotation)
    #when rotation divided by 30 = 1, rotate 1/4 of a circle cw
    print("num/30", int(rotation/30))
    for i in range(int(rotation/30)):
      data=['CMD_MOVE', '1', '0', '0', '10', '10']  # rotate 90 degs clockwise
      c.run(data) 
    #if 40
       
    #c.relax(True)

#======================= rotate CCW Method with sys.argv[2] ===============

def rotateCCW(rotation):
    rotation = int(rotation)
    
    for i in range(int(rotation/30)):
          data=['CMD_MOVE', '1', '0', '0', '7', '-10'] 
          c.run(data)
    #c.relax(True)

#======================= Circle CW Method ===============================

def circleCW(rotation = 0):
  #if user does not parse desired rotation as a parameter it will prompt them in the method
  if(rotation == 0):
    rotation = input("Enter 90, 180, 270, or 360 for rotation CW: ")
  #when rotation = 1, rotate 1/4 of a circle cw
  if(rotation == 90):
    print("90 degr")
    for i in range(3):
      data=['CMD_MOVE', '1', '0', '25', '10', '10']  # rotate 90 degs clockwise
      c.run(data)   
  #when rotation = 2, rotate 1/2 of a circle cw
  elif(rotation == 180):
    for i in range(6):
      data=['CMD_MOVE', '1', '0', '25', '10', '10']  # rotate 180 degs clockwise
      c.run(data)  
  #when rotation = 1, rotate 3/4 of a circle cw
  elif(rotation == 270):
    for i in range(9):
      data=['CMD_MOVE', '1', '0', '25', '10', '10']  # rotate 270 degs clockwise
      c.run(data) 
  #when rotation = 1, rotate full circle cw
  elif(rotation == 360):
    for i in range(12):
      data=['CMD_MOVE', '1', '0', '25', '10', '10']  # rotate 360 degs clockwise
      c.run(data)
  c.relax(True)

#======================= Circle CCW Method ==============================

def circleCCW(rotation):
    rotation = int(rotation)
    #when rotation = 1, rotate 1/4 of a circle cw
    if(rotation == 90):
        for i in range(3):
          data=['CMD_MOVE', '1', '0', '25', '10', '-10']  # rotate 90 degs counter clockwise
          c.run(data)  
    #when rotation = 2, rotate 1/2 of a circle cw
    elif(rotation == 180):
        for i in range(6):
          data=['CMD_MOVE', '1', '0', '25', '10', '-10']  # rotate 180 degs counter clockwise
          c.run(data)        
    #when rotation = 1, rotate 3/4 of a circle cw
    elif(rotation == 270):
        for i in range(9):
          data=['CMD_MOVE', '1', '0', '25', '10', '-10']  # rotate 270 degs counter clockwise
          c.run(data) 
    #when rotation = 1, rotate full circle cw
    elif(rotation == 260):
        for i in range(12):
          data=['CMD_MOVE', '1', '0', '25', '10', '-10']  # rotate 360 degs counter clockwise
          c.run(data)
    c.relax(True)

#========================= Relax Method ===============================
def relax():
  c.relax(True)

#========================= Attitude/Climbing Method ===============================
def attitude(x,y,z):
    #climbing
                #attitude      x   y  z
    #c.order =['CMD_ATTITUDE','10','5','0','','']
    c.set_order(x,y,z)
    c.check_condition()
    #time.sleep(3)
    
    #forward()
    #time.sleep(3)
    #c.relax(True)

#========================= look right ===============================    
def lookright():
    attitude(0,0,15)
    time.sleep(3)
    attitude(0,0,0)
    c.relax(True)
    
#========================= look left ===============================
def lookleft():
    attitude(0,0,-15)
    time.sleep(3)
    attitude(0,0,0)
    c.relax(True)
    
    
    
#========================= LiDar Readings ==============================

def listenArduinoLiDar():
    ser=serial.Serial("/dev/ttyACM0",115200)  #change ACM number as found from ls /dev/tty/ACM*
    ser.baudrate=115200

    #GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.OUT)
    distance_Inches = 0.0
    for i in range(3):
        print('printing for loop')
        read_ser=ser.readline()
        print(read_ser)
        distance = str(read_ser).split("'")
        print((distance[1]).split("\x5c")[0])
        try:
            distance_Inches += float(str((distance[1]).split("\x5c")[0]))
        except:
            print("invalid reading")
            
    return distance_Inches/3

    
#========================= Receive Coor ===============================

def recieveCoordinates(x, y):
  	#setup
    ultrasonic=Ultrasonic()
    xCoorDis = x
    yCoorDis = y
    degreeRad = math.atan(y/x)
    try:
      	#go towards sent coordinates until within 2 inches from coordinate
        while xCoorDis >= 2 and yCoorDis >= 2:
            data=listenArduinoLiDar()   #Get the value
            print ("Obstacle distance is "+str(data)+"CM")
            #constantly get ultrasonic values to see if object is in front
            #if object detected is within 16 cm, trigger object detected func
            if(data <= 20):
                objectDetected(degreeRad, xCoorDis, yCoorDis)
            #if not, keep going forward and decrement your coordinate distance
            #as you go forward
            else:
                forward(1)
                xCoorDis = xCoorDis - (math.cos(degreeRad))
                yCoorDis = yCoorDis - (math.sin(degreeRad))
    except KeyboardInterrupt:
        c.relax(True)
        print("\nEnd of program")
    c.relax(True)
      
def objectDetected(degreeRad, xCoorDis, yCoorDis):
    ultrasonic=Ultrasonic()
    #leftDisReading will be a collection of readings to the left
    #of our hex and vice versa for rightDisReading
    leftDisReading=0
    rightDisReading=0
    try:
      	#we do range -3 to -16 because that turns our hexapod towards the left
        #the lower the num, the more we shift left
        for i in range(-3, -16, -2):
            attitude(0,0,i)
            time.sleep(0.1)
            #increment leftDisReading reading by the range of detected value
            leftDisReading+=listenArduinoLiDar()
        print("dist left: ", str(leftDisReading))
        #we do range 3 to 16 because that turns our hexapod towards the right
        #the lower the num, the more we shift right
        for i in range(3, 16, 2):
          attitude(0,0,i)
          time.sleep(0.1)
          #increment rightDisReading reading by the range of detected value
          rightDisReading+=listenArduinoLiDar()
        print("dist right: ", str(rightDisReading))
        objectInFront = True
        goingAroundObject = True
        #compare collection of left values to collection
        #of right values. Whichever number is higher means
        #there is more space for the hex to go through
        if(rightDisReading >= leftDisReading):
          	#we want to keep track of the number of times we go right
            #in order to adjust back to our neutral orginial path
            clicksRight=0
            while(objectInFront):
                clicksRight+=1
                right(4)
                objectInFront=listenArduinoLiDar()
                time.sleep(0.2)
                attitude(0,0,-15)
                objectInFrontLookingLeft=listenArduinoLiDar()
                time.sleep(0.2)
                if(objectInFront >= 20 and objectInFrontLookingLeft >= 50):
                    objectInFront = False
            #this while loop will run once we establish that the hexapod has
            #enough room to go forward
            while(goingAroundObject):
                forward(5)
                xCoorDis = xCoorDis - 5*(math.cos(degreeRad))
                yCoorDis = yCoorDis - 5*(math.sin(degreeRad))
                time.sleep(0.2)
                attitude(0,0,-15)
                time.sleep(0.2)
                objectDis=listenArduinoLiDar()
                time.sleep(0.2)
                if(objectDis >= 50):
                    forward(5)
                    goingAroundObject = False
            #for loop that adjusts the hex back to neutral position
            #based clicksRight
            for i in range(0, clicksRight, 1):
                left(4)
        else:
          	#we want to keep track of the number of times we go left
            #in order to adjust back to our neutral orginial path
            clicksLeft=0
            while(objectInFront):
                clicksLeft+=1
                left(6)
                objectInFront=listenArduinoLiDar()
                time.sleep(0.2)
                attitude(0,0,15)
                objectInFrontLookingRight=listenArduinoLiDar()
                time.sleep(0.2)
                if(objectInFront >= 20 and objectInFrontLookingRight >= 50):
                    objectInFront = False
            #this while loop will run once we establish that the hexapod has
            #enough room to go forward
            while(goingAroundObject):
                rotateCW(10)
                forward(5)
                xCoorDis = xCoorDis - 5*(math.cos(degreeRad))
                yCoorDis = yCoorDis - 5*(math.sin(degreeRad))
                time.sleep(2)
                attitude(0,0,15)
                time.sleep(1)
                objectDis=listenArduinoLiDar()
                time.sleep(1)
                if(objectDis >= 50):
                    forward(5)
                    goingAroundObject = False
            #for loop that adjusts the hex back to neutral position
            #based clicksLeft
            for i in range(0, clicksLeft, 1):
                right(4)
    except KeyboardInterrupt:
        c.relax(True)
        print("\nEnd of program")
  
    

   
  
    

   

# def recieveCoordinates(x, y):
    # ultrasonic=Ultrasonic()
    # xCoorDis = x
    # yCoorDis = y
    # degreeRad = math.atan(y/x)
    # try:
        # while xCoorDis >= 2 and yCoorDis >= 2:
            # data=ultrasonic.getDistance()   #Get the value
            # print ("Obstacle distance is "+str(data)+"CM")
            # if(data <= 16):
                # objectDetected(degreeRad, xCoorDis, yCoorDis)
            # else:
                # forward(1)
                # xCoorDis = xCoorDis - (math.cos(degreeRad))
                # yCoorDis = yCoorDis - (math.sin(degreeRad))
    # except KeyboardInterrupt:
        # c.relax(True)
        # print("\nEnd of program")
    # #c.relax(True)
      
# def objectDetected(degreeRad, xCoorDis, yCoorDis):
    # ultrasonic=Ultrasonic()
    # leftDisReading=0
    # rightDisReading=0
    # try:
        # for i in range(-3, -16, -3):
            # attitude(0,0,i)
            # time.sleep(0.3)
            # leftDisReading+=ultrasonic.getDistance()
        # print("dist left: ", str(leftDisReading))
        # time.sleep(1.0)
        # for i in range(3, 16, 3):
          # attitude(0,0,i)
          # time.sleep(0.3)
          # rightDisReading+=ultrasonic.getDistance()
        # print("dist right: ", str(rightDisReading))
        # time.sleep(1.0)
        # objectInFront = True
        # goingAroundObject = True
        # if(rightDisReading >= leftDisReading):
            # clicksRight=0
            # while(objectInFront is True):
                # clicksRight+=1
                # right(4)
                # time.sleep(1)
                # objectInFront=ultrasonic.getDistance()
                # if(objectInFront >= 20):
                    # objectInFront = False
            # while(goingAroundObject is True):
                # forward(5)
                # xCoorDis = xCoorDis - 5*(math.cos(degreeRad))
                # yCoorDis = yCoorDis - 5*(math.sin(degreeRad))
                # time.sleep(1)
                # attitude(0,0,-5)
                # time.sleep(1)
                # objectDis=ultrasonic.getDistance()
                # time.sleep(1)
                # if(objectDis >= 30):
                    # forward(5)
                    # goingAroundObject = False
            # for i in range(0, clicksRight, 1):
                # left(4)
        # else:
            # clicksLeft=0
            # while(objectInFront is True):
                # clicksLeft+=1
                # left(4)
                # time.sleep(2)
                # objectInFront=ultrasonic.getDistance()
                # if(objectInFront >= 20):
                    # objectInFront = False
            # while(goingAroundObject is True):
                # forward(5)
                # xCoorDis = xCoorDis - 5*(math.cos(degreeRad))
                # yCoorDis = yCoorDis - 5*(math.sin(degreeRad))
                # time.sleep(2)
                # attitude(0,0,5)
                # time.sleep(1)
                # objectDis=ultrasonic.getDistance()
                # time.sleep(1)
                # if(objectDis >= 30):
                    # forward(5)
                    # goingAroundObject = False
            # for i in range(0, clicksLeft, 1):
                # right(4)
    # except KeyboardInterrupt:
        # c.relax(True)
        # print("\nEnd of program")
  
#======================== Inputs for terminal =========================
if __name__ == '__main__':
    print ('Program is starting ... ')
    import sys
    if len(sys.argv)<2:
        print ("Parameter error: Please assign the device")
        exit()
    if sys.argv[1] == 'Head' or sys.argv[1] == 'head':
        fix_head()
    elif sys.argv[1] == 'Move' or sys.argv[1] == 'move':
        move('3', '1', '0', '12', '10', '0')
    elif sys.argv[1] == 'Demo' or sys.argv[1] == 'demo':
        movement_without_param()
    elif sys.argv[1] == 'Foward' or sys.argv[1] == 'forward':
        forward(65)
        #This is where the x variable for for loop is
    elif sys.argv[1] == 'Left' or sys.argv[1] == 'left':
        left(3)
    elif sys.argv[1] == 'Right' or sys.argv[1] == 'right':
        right(3)
    elif sys.argv[1] == 'Back' or sys.argv[1] == 'back':
        back()
    elif sys.argv[1] == 'Servo' or sys.argv[1] == 'servo':
        test_Servo()
    elif sys.argv[1] == 'IMU' or sys.argv[1] == 'imu':
        test_imu()
    elif sys.argv[1] == 'Gyro' or sys.argv[1] == 'gyro':
        test_imu_gyro()
    elif sys.argv[1] == 'Pushup' or sys.argv[1] == 'pushup':
        pushup()
    elif sys.argv[1] == 'Figure8' or sys.argv[1] == 'figure8':
        figure8()
    elif len(sys.argv) == 3 and (sys.argv[1] == 'RotateCW' or sys.argv[1] == 'rotateCW'):           
        rotateCW(sys.argv[2])    # sudo python myCode.py rotateCW 90
    elif len(sys.argv) == 3 and (sys.argv[1] == 'RotateCCW' or sys.argv[1] == 'rotateCCW'):   
        rotateCCW(sys.argv[2])  # sudo python myCode.py rotateCCW 90
    elif sys.argv[1] == 'CircleCW' or sys.argv[1] == 'circleCW':   
        circleCW()        # sudo python myCode.py CircleCW 
    elif sys.argv[1] == 'CircleCCW' or sys.argv[1] == 'circleCCW':   
        circleCCW()
    elif sys.argv[1] == 'Relax' or sys.argv[1] == 'relax':   
        relax()
    elif sys.argv[1] == 'Att' or sys.argv[1] == 'att':
        attitude(0,0,15)
    elif sys.argv[1] == 'LookRight' or sys.argv[1] == 'lookright':   
        lookright()
    elif sys.argv[1] == 'LookLeft' or sys.argv[1] == 'lookleft':   
        lookleft()
    elif sys.argv[1] == 'coor' or sys.argv[1] == 'Coor':
        recieveCoordinates(10, 10)




