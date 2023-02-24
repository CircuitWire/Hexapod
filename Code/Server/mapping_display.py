#display
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import Axes3D 

def mapping3d():
            
    df2 = pd.read_csv("data.csv")
    # Dataset is now stored in a Pandas Dataframe

    df2.head()

    D = df2.to_numpy()
    t = D[:,0]
    pan = D[:,1]
    tilt = D[:,2]
    r_us = D[:,3] + 10
    #r_tof = D[:,4] + 7

    #plt.plot(t, r_us)
    #plt.show()

    #cartesian
    phi = (90-tilt)*np.pi/180
    theta = pan*np.pi/180

    x_us = r_us*np.cos(theta)*np.sin(phi)
    y_us = r_us*np.sin(theta)*np.sin(phi)
    z_us = r_us*np.cos(phi)

    #plt.axes(projection="3d")

    #setup
    plt.style.use('dark_background')
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.scatter(x_us, y_us, z_us,s=2, color='y') #, s=2

    #Labels
    ax.set_ylim3d(-25,25)
    ax.set_xlim3d(-25,25)
    ax.set_zlim3d(0,25)
    # ax.set_xlabel('x', color='w')
    # ax.set_ylabel('y', color='w')
    # ax.set_zlabel('z', color='w')

    #Colors
    plt.gca().patch.set_facecolor('black')
    ax.w_xaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
    ax.w_yaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
    ax.w_zaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))

    # # Remove tick labels
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_zticklabels([])
    # Transparent spines
    ax.w_xaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
    ax.w_yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
    ax.w_zaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
    # No ticks
    ax.set_xticks([]) 
    ax.set_yticks([]) 
    ax.set_zticks([])

    #add white lines
    ax.grid(False)
    x, y, z = np.zeros((3,3))
    u, v, w = np.array([[4000,0,0],[0,4000,0],[0,0,2500]])

    ax.quiver(x,y,z,u,v,w,arrow_length_ratio=0.0,linewidth=0.5,color='w')


    plt.show()
    
    
#data = [accel_data['x'], accel_data['y'], accel_data['z'], 
                  #  gyro_data['x'], gyro_data['y'], gyro_data['z'], 
                  #  roll, pitch, yaw]
    
    
def displayIMU():
    df2 = pd.read_csv("data/imu_data.csv") # Dataset is now stored in a Pandas Dataframe

    #              0           1       2      3     4
    #data = [accel_data, gyro_data, roll, pitch, yaw]
    D = df2.to_numpy()
    
    time_imu = D[:,0]
    initial_time_imu = 0

    linear_acceleration_x = D[:,1] #might be wrong
    linear_acceleration_y = D[:,2]  #might be wrong
    linear_acceleration_z = D[:,3]  #might be wrong

    gyro_x = D[:,4] #angular_velocity
    gyro_y = D[:,5]#angular_velocity
    gyro_z = D[:,6] #angular_velocity
    
    roll = D[:,7]
    pitch = D[:,8]
    yaw = D[:,9]


    #angular velocity
    #ang_vel = plt.figure()
    
    
    plt.subplot(1, 3, 1)
   
    plt.plot(time_imu*1000, gyro_x, color = 'r')
    plt.plot(time_imu*1000, gyro_y, color = 'g')
    plt.plot(time_imu*1000, gyro_z, color = 'b')
    plt.title('Angular Velocity (Gyro_Reading) ')
    plt.ylabel('Velocity (rad/s)')
    plt.xlabel('Time (ms)')
    
   

    #linear acceleration
    #lin_acc = plt.figure()
    
    plt.xlabel('Time (ms)')
    plt.subplot(1, 3, 2)
    plt.plot(time_imu*1000, linear_acceleration_x, color = 'r')
    plt.plot(time_imu*1000, linear_acceleration_y, color = 'g')
    plt.plot(time_imu*1000, linear_acceleration_z, color = 'b')
    plt.title('Linear Acceleration ' )
    plt.ylabel('Linear Acceleration (m/s^2)')
    plt.xlabel('Time (ms)')
    
   

    #rpy
   # rpy = plt.figure()
    
    plt.subplot(1, 3,3)
   
    plt.plot(time_imu*1000, roll, color='g')
    plt.plot(time_imu*1000, pitch, color='b')
    plt.plot(time_imu*1000, yaw, color='r')
    plt.title('Roll, Pitch, Yaw ')
    plt.ylabel('Angle (deg)')
    plt.xlabel('Time (ms)')
    plt.show()
    
    
    
    
if __name__ == '__main__':    
    import sys
    if len(sys.argv)<2:
        print ("Parameter error: Please assign the device")
        exit() 
    if sys.argv[1] == 'IMU' or sys.argv[1] == 'imu': 
        displayIMU()
    elif sys.argv[1] == '3D' or sys.argv[1] == '3d': 
        mapping3d()
    elif sys.argv[1] == 'Accel' or sys.argv[1] == 'accel': 
        gather_accel_data()     
    elif sys.argv[1] == 'Gyro' or sys.argv[1] == 'gyro': 
        gather_gyro_data()             
    elif sys.argv[1] == 'RPY' or sys.argv[1] == 'rpy': 		#RPY = roll, pitch, yaw
        gather_rpy_data()
    elif sys.argv[1] == 'Lidar' or sys.argv[1] == 'LiDar' or sys.argv[1] == 'lidar': 
        gather_lidar_data()
