import random                                  
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits import mplot3d
from matplotlib.patches import Circle, PathPatch
from matplotlib.text import TextPath
from matplotlib.transforms import Affine2D
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.animation as animation

# Importing Packages
import numpy as np        
from mpl_toolkits.mplot3d import Axes3D


# Time Array
File_data = np.loadtxt('/home/haotiangu/catkin_ws/src/tcps_image_attack/scripts/result/PSNR_NOISE.txt')
# Position Arrays
column_size = File_data[:, 0]#np.sin(np.pi/5 * t)
t = len(column_size)
x = np.linspace(0,t,t)
y = File_data[:, 0]#np.sin(np.pi/3 * t)

# Setting up Data Set for Animation
dataSet = np.array([x, y])  # Combining our position coordinates

numDataPoints = len(dataSet)
File_data_1 = np.loadtxt('/home/haotiangu/catkin_ws/src/tcps_image_attack/scripts/result/PSNR_NOISE.txt')
# Position Arrays
x_1 = np.linspace(0,t,t)#np.sin(np.pi/5 * t)
print(len(x_1))
y_1 = File_data_1[:, 1]#np.sin(np.pi/3 * t)

# Setting up Data Set for Animation
dataSet_1 = np.array([x_1, y_1])  # Combining our position coordinates


plt.plot(x,y,label='PSNR noisy') 
plt.plot(x_1,y_1,label='PSNR') 

# Add Title

#plt.title("PSNR comparision") 

# Add Axes Labels

plt.xlabel("the batch frames") 
plt.ylabel("PSNR value") 
plt.legend(loc='best')
# Display
plt.show()