from Ultrasonic import *
import myCode
from Servo import *
import csv
from IMU import *
from Control import *

c=Control()

def gather_3d_map():
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

#========================= attitude Method ===============================
def gather_imu_data():
    
  	
    #initialize imu():
    s=IMU()
    
    #set up writer to write to csv
    f = open('data/imu_data.csv', 'w')
    writer = csv.writer(f)
   
    while True:
        try:    
            time.sleep(0.1)
            accel_data, gyro_data = s.average_filter()
            roll, pitch, yaw = s.imuUpdate() 
            
            startTime=time.time() 
                      
            print('accel_data', accel_data)
            print('gyro_data', gyro_data)
            print('roll, pitch, yaw', roll, pitch, yaw)
            
            endTime=time.time()
            diff=endTime-startTime
            tme=diff
            data = [tme,accel_data['x'], accel_data['y'], accel_data['z'], 
                    gyro_data['x'], gyro_data['y'], gyro_data['z'], 
                    roll, pitch, yaw]
            writer.writerow(data)
            time.sleep(0.1)
        except KeyboardInterrupt:
            f.close()
            c.relax(True)
            print("\nKeyboardInterrupt")     
            break
        except Exception as e:
            print(e)
            c.relax(True)
            os.system("i2cdetect -y 1")
            break
      	
    #print to data.csv
    f.close()

#========================= accel Method ===============================
def gather_accel_data():
    #initialize imu():
    s=IMU()
    
    #set up writer to write to csv
    f = open('data/accel_data.csv', 'w')
    writer = csv.writer(f)
    
    
    while True:
        try:    
            time.sleep(0.1)
            accel_data, gyro_data = s.average_filter()  
            data = [accel_data['x'], accel_data['y'], accel_data['z']]
            writer.writerow(data)
            time.sleep(0.1)
        except KeyboardInterrupt:
            #print to data.csv
            f.close()
            print("\nKeyboardInterrupt")     
            break
        except Exception as e:
            print(e)
            os.system("i2cdetect -y 1")
            break
      	
    #print to data.csv
    f.close()
  	  	
#========================= gyro Method ===============================
def gather_gyro_data():
    #initialize imu():
    s=IMU()
    
    #set up writer to write to csv
    f = open('data/gyro_data.csv', 'w')
    writer = csv.writer(f)
    
    
    while True:
        try:    
            time.sleep(0.1)
            accel_data, gyro_data = s.average_filter()                        
            print('gyro_data', gyro_data)
            data = [gyro_data['x'], gyro_data['y'], gyro_data['z']]
            writer.writerow(data)            
            time.sleep(0.1)
        except KeyboardInterrupt:
            #print to data.csv
            f.close()
            print("\nKeyboardInterrupt")     
            break
        except Exception as e:
            print(e)
            os.system("i2cdetect -y 1")
            break
      	
    #print to data.csv
    f.close()
    
#========================= rpy Method ===============================
def gather_rpy_data():
    #initialize imu():
    s=IMU()
    
    #set up writer to write to csv
    f = open('data/rpy_data.csv', 'w')
    writer = csv.writer(f)
    
    
    while True:
        try:    
            time.sleep(0.1)
            roll, pitch, yaw = s.imuUpdate()            
            data = [roll, pitch, yaw]
            print('roll, pitch, yaw', roll, pitch, yaw)
            writer.writerow(data)
            time.sleep(0.1)
        except KeyboardInterrupt:
            #print to data.csv
            f.close()
            print("\nKeyboardInterrupt")          
            break
        except Exception as e:
            print(e)
            os.system("i2cdetect -y 1")
            break
            
#========================= Lidar Method ===============================
def gather_lidar_data():
    
    #no setup, myCode.listenArduinoLiDar() has the set up
    
    #set up writer to write to csv
    f = open('data/lidar_data.csv', 'w')
    writer = csv.writer(f)
    
    
    while True:
        try:    
            time.sleep(0.1)
            data = myCode.listenArduinoLiDar()
            writer.writerow(data)
            time.sleep(0.1)
        except KeyboardInterrupt:
            #print to data.csv
            f.close()
            print("\nKeyboardInterrupt")     
            break
        except Exception as e:
            print(e)
            os.system("i2cdetect -y 1")
            break
      	
    #print to data.csv
    f.close()
    
    
#======================== Inputs for terminal =========================
if __name__ == '__main__':    
    import sys
    if len(sys.argv)<2:
        print ("Parameter error: Please assign the device")
        exit() 
    if sys.argv[1] == 'IMU' or sys.argv[1] == 'imu': 
        gather_imu_data() 
    elif sys.argv[1] == '3D' or sys.argv[1] == '3d': 
        gather_3d_map()
    elif sys.argv[1] == 'Accel' or sys.argv[1] == 'accel': 
        gather_accel_data()     
    elif sys.argv[1] == 'Gyro' or sys.argv[1] == 'gyro': 
        gather_gyro_data()             
    elif sys.argv[1] == 'RPY' or sys.argv[1] == 'rpy': 		#RPY = roll, pitch, yaw
        gather_rpy_data()
    elif sys.argv[1] == 'Lidar' or sys.argv[1] == 'LiDar' or sys.argv[1] == 'lidar': 
        gather_lidar_data()
