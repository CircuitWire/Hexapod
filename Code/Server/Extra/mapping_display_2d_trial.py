#display
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import Axes3D 


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
#z_us = r_us*np.cos(phi)

#plt.axes(projection="3d")

#setup
plt.style.use('dark_background')
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.scatter(x_us, y_us, s=2, color='y') #, s=2
# ax.scatter(x_us, y_us, z_us,s=2, color='y') #, s=2

#Labels
ax.set_ylim3d(-25,25)
ax.set_xlim3d(-25,25)
# ax.set_zlim3d(0,25)
# ax.set_xlabel('x', color='w')
# ax.set_ylabel('y', color='w')
# ax.set_zlabel('z', color='w')

#Colors
plt.gca().patch.set_facecolor('black')
ax.w_xaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
ax.w_yaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
# ax.w_zaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))

# # Remove tick labels
ax.set_xticklabels([])
ax.set_yticklabels([])
# ax.set_zticklabels([])
# Transparent spines
ax.w_xaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
ax.w_yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
# ax.w_zaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
# No ticks
ax.set_xticks([]) 
ax.set_yticks([]) 
# ax.set_zticks([])
ax.quiver(x,y,z,u,v,w,arrow_length_ratio=0.0,linewidth=0.5,color='w')
#add white lines
ax.grid(False)
x, y = np.zeros((2,2))
u, v = np.array([[4000, 0], [0, 4000]])
# x, y, z = np.zeros((3,3))
# u, v, w = np.array([[4000,0,0],[0,4000,0],[0,0,2500]])

ax.quiver(x,y,u,v,arrow_length_ratio=0.0,linewidth=0.5,color='w')
# ax.quiver(x,y,z,u,v,w,arrow_length_ratio=0.0,linewidth=0.5,color='w')


plt.show()
