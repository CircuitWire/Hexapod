#this myCode.py is only for TERMINAL INPUTS

#make sure to use relax at the end or else servos get stiff
#Import everything in the control module, 
#including functions, classes, variables, and more.
from Control import *

#Creating object 'control' of 'Control' class.
c=Control()

#example:
        # 0     ,   1,   2,   3,    4,    5
#data=['CMD_MOVE', '1', '0', '25', '10', '0']
#Move command:'CMD_MOVE'
#Gait Mode: "1"        Note: "2" is one by one, "1" is three by three # 3rd gait 
#Moving direction: x='0',y='25'
#Delay:'10'
#Action Mode : '0'   Angleless turn 

#=========== Moving Head (Head Servo Heating up, Not Consistent) ======
#this method did not work as intended for some reason
#from Servo import *
#servo = Servo()
#servo.setServoAngle(1, -100)
#servo.setServoAngle(0, -100)
#c.relax(True)

#========================== Movement  =====================================

def movement():
    gait = input("\nEnter Gait Mode ('1' is three legs, '2' is one leg, '3' is in development: ")
    x = input("Enter x-value: ")
    y = input("Enter y-value: ")
    delay = input("Enter delay-value (Ex: '1' is slow, '10' is fast): ")
    angle = input("Enter angle turn (or rotate if x,y were 0,0): ")
    for i in range(3):
        data=['CMD_MOVE', gait, x, y, delay, angle] 
        c.run(data)    
    c.relax(True)


#========================== Forward  =====================================

def forward():
  for i in range(3):
    data=['CMD_MOVE', '3', '0', '10', '3', '0']  #testing out gait mode "3"
    c.run(data)

#def forward(rangeVal, gait, x, y, delay, actionMode):
  #for i in range(rangeVal):
    #data=['CMD_MOVE', gait, x, y, delay, '0']  #"1" move three by 3 fwd
    #c.run(data)

#============================ Left  =====================================

def left():
  for i in range(3):
    data=['CMD_MOVE', '2', '-15', '0', '10', '0']  
    c.run(data)
    c.relax(True)
    
#========================== Right  =====================================

def right():
  for i in range(3):
    data=['CMD_MOVE', '2', '15', '0', '10', '0']  
    c.run(data)
    c.relax(True)
    
#========================== Backward  ====================================

def back():
  for i in range(3):
    data=['CMD_MOVE', '2', '0', '-15', '10', '0']  
    c.run(data)
    c.relax(True)
    
#========================= Servo ======================================
from Servo import *
servo=Servo()
#for i in range(90):
     #  servo.setServoAngle(,i) 
     #  time.sleep(0.005)
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
        print ("\nEnd of program")
        
#===========================   IMU    =================================
from IMU import *
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

#==================== Push Up(use -12, 12) ==============================
import time
def pushup():
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

#======================= noParam Rotate CW Method ===============================
      
def noParamRotateCW():
  #if user does not parse desired rotation as a parameter it will prompt them in the method
  rotation = int(input("Enter 90, 180, 270, or 360 for rotation CW: "))
  
  #when rotation = 1, rotate 1/4 of a circle cw
  if(rotation == 90):
    for i in range(3):
      data=['CMD_MOVE', '1', '0', '0', '10', '10']  # rotate 90 degs clockwise
      c.run(data)  
  #when rotation = 2, rotate 1/2 of a circle cw
  elif(rotation == 180):
    for i in range(6):
      data=['CMD_MOVE', '1', '0', '0', '10', '10']  # rotate 180 degs clockwise
      c.run(data) 
  #when rotation = 1, rotate 3/4 of a circle cw
  elif(rotation == 270):
    for i in range(9):
      data=['CMD_MOVE', '1', '0', '0', '10', '10']  # rotate 270 degs clockwise
      c.run(data) 
  #when rotation = 1, rotate full circle cw
  elif(rotation == 360):
    for i in range(12):
      data=['CMD_MOVE', '1', '0', '0', '10', '10']  # rotate 360 degs clockwise
      c.run(data)
  c.relax(True)
  
  
#======================= rotate CW Method with sys.argv[2] =========================
      
def rotateCW(rotation):
    rotation = int(rotation)
    #when rotation = 1, rotate 1/4 of a circle cw
    if(rotation == 90):
        for i in range(3):
          data=['CMD_MOVE', '1', '0', '0', '10', '10']  # rotate 90 degs clockwise
          c.run(data)  
    #when rotation = 2, rotate 1/2 of a circle cw
    elif(rotation == 180):
        for i in range(6):
          data=['CMD_MOVE', '1', '0', '0', '10', '10']  # rotate 180 degs clockwise
          c.run(data) 
    #when rotation = 1, rotate 3/4 of a circle cw
    elif(rotation == 270):
        for i in range(9):
          data=['CMD_MOVE', '1', '0', '0', '10', '10']  # rotate 270 degs clockwise
          c.run(data) 
    #when rotation = 1, rotate full circle cw
    elif(rotation == 360):
        for i in range(12):
          data=['CMD_MOVE', '1', '0', '0', '10', '10']  # rotate 360 degs clockwise
          c.run(data)
    c.relax(True)

#======================= noParam Rotate CCW Method ===========================
  
def noParamRotateCCW():
  #if user does not parse desired rotation as a parameter it will prompt them in the method
  rotation = int(input("Enter 90, 180, 270, or 360 for rotation CCW: "))
  
  #when rotation = 1, rotate 1/4 of a circle cw
  if(rotation == 90):
    for i in range(3):
      data=['CMD_MOVE', '1', '0', '0', '10', '-10']  # rotate 90 degs counter clockwise
      c.run(data)   
  #when rotation = 2, rotate 1/2 of a circle cw
  elif(rotation == 180):
    for i in range(6):
      data=['CMD_MOVE', '1', '0', '0', '10', '-10']  # rotate 180 degs counter clockwise
      c.run(data)  
  #when rotation = 1, rotate 3/4 of a circle cw
  elif(rotation == 270):
    for i in range(9):
      data=['CMD_MOVE', '1', '0', '0', '10', '-10']  # rotate 270 degs counter clockwise
      c.run(data) 
  #when rotation = 1, rotate full circle cw
  elif(rotation == 360):
    for i in range(12):
      data=['CMD_MOVE', '1', '0', '0', '10', '-10']  # rotate 360 degs counter clockwise
      c.run(data)
  c.relax(True)

#======================= rotate CCW Method with sys.argv[2] ===============

def rotateCCW(rotation):
    rotation = int(rotation)
    #when rotation = 1, rotate 1/4 of a circle cw
    if(rotation == 90):
        for i in range(3):
          data=['CMD_MOVE', '1', '0', '0', '10', '-10']  # rotate 90 degs counter clockwise
          c.run(data)   
    #when rotation = 2, rotate 1/2 of a circle cw
    elif(rotation == 180):
        for i in range(6):
          data=['CMD_MOVE', '1', '0', '0', '10', '-10']  # rotate 180 degs counter clockwise
          c.run(data)  
    #when rotation = 1, rotate 3/4 of a circle cw
    elif(rotation == 270):
        for i in range(9):
          data=['CMD_MOVE', '1', '0', '0', '10', '-10']  # rotate 270 degs counter clockwise
          c.run(data) 
    #when rotation = 1, rotate full circle cw
    elif(rotation == 360):
        for i in range(12):
          data=['CMD_MOVE', '1', '0', '0', '10', '-10']  # rotate 360 degs counter clockwise
          c.run(data)
    c.relax(True)

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


#=================== test individual methods ==========================
#data=['CMD_MOVE', '1', '0', '25', '10', '0']
#exit()

#======================== Inputs for terminal =========================
if __name__ == '__main__':
    print ('Program is starting ... ')
    import sys
    if len(sys.argv)<2:
        print ("Parameter error: Please assign the device")
        exit()
    if sys.argv[1] == 'Move' or sys.argv[1] == 'move':
        movement()
    elif sys.argv[1] == 'Foward' or sys.argv[1] == 'forward':
        forward()
    elif sys.argv[1] == 'Left' or sys.argv[1] == 'left':
        left()
    elif sys.argv[1] == 'Right' or sys.argv[1] == 'right':
        right()
    elif sys.argv[1] == 'Backward' or sys.argv[1] == 'backward':
        back()
    elif sys.argv[1] == 'Servo' or sys.argv[1] == 'servo':
        test_Servo()
    elif sys.argv[1] == 'IMU' or sys.argv[1] == 'imu':
        test_imu()
    elif sys.argv[1] == 'Pushup' or sys.argv[1] == 'pushup':
        pushup()
    elif sys.argv[1] == 'Figure8' or sys.argv[1] == 'figure8':
        figure8()
    elif sys.argv[1] == 'RotateCW' or sys.argv[1] == 'rotateCW':
        if len(sys.argv) == 2:
            print("without")
            noParamRotateCW()        # sudo python myCode.py rotateCW             
        elif len(sys.argv) == 3:
            rotateCW(sys.argv[2])    # sudo python myCode.py rotateCW 90
        else:
            print("Try again")
    elif sys.argv[1] == 'RotateCCW' or sys.argv[1] == 'rotateCCW':   
        if len(sys.argv) == 2:
            noParamRotateCCW()      # sudo python myCode.py rotateCCW  
        elif len(sys.argv) == 3:
            rotateCCW(sys.argv[2])  # sudo python myCode.py rotateCCW 90
        else:
            print("Try again")        
    elif sys.argv[1] == 'CircleCW' or sys.argv[1] == 'circleCW':   
        circleCW()        # sudo python myCode.py CircleCW 
    elif sys.argv[1] == 'CircleCCW' or sys.argv[1] == 'circleCCW':   
        circleCCW()
    elif sys.argv[1] == 'Relax' or sys.argv[1] == 'relax':   
        relax()






