#This myCodeClass.py is for OTHER PYTHON FILES, NOT TERMINAL INPUTS

#make sure to use relax at the end or else servos get stiff
#Import everything in the control module, 
#including functions, classes, variables, and more.
from Control import *
from Servo import *
from IMU import *
import time

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
class myCodeClass:
  def __init__(self):
        n=0 #not important

  #========================== Forward  =====================================

  def forward(self):
    for i in range(3):
      data=['CMD_MOVE', '2', '0', '10', '10', '0']  #testing out gait mode "3"
      c.run(data)
      
  #========================== Forward with Parameter ===========================

  def forward(self, distance):
    for i in range(3):
      data=['CMD_MOVE', '2', '0', distance, '10', '0']  #testing out gait mode "3"
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

  def right(self):
    for i in range(3):
      data=['CMD_MOVE', '2', '15', '0', '10', '0']  
      c.run(data)
      c.relax(True)
      
  #========================== Backward  ====================================

  def back(self):
    for i in range(3):
      data=['CMD_MOVE', '2', '0', '-15', '10', '0']  
      c.run(data)
      c.relax(True)
      
  #========================= Servo ======================================
  #for i in range(90):
       #  servo.setServoAngle(,i) 
       #  time.sleep(0.005)
  def test_Servo(self, channel):
      servo=Servo()
      print("   \\              /")
      print("    18    ||    13")
      print("     \\    ||    /")
      print("      17  ||  14")
      print("       \\ head /")
      print("        16  15 ")
      print("        ======")
      print("21-19-20|    |12-11-10")
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
  def test_imu(self):
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
  def pushup(self):
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
  def figure8(self):
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
        
  def rotateCW(self, rotation):
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

  def circleCCW(self, rotation):
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
  def relax(self):
    c.relax(True)


#======================== Inputs for terminal =========================
if __name__ == '__main__':
    pass
