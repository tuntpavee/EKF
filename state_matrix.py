import numpy as np
import lidar_localization
#imu
x = 0
y = 0
z = 0

pitch = 0
roll = 0
yaw = 0

vx = 0
vy = 0
vz = 0

imu_state_matrix = np.array([[x,y,z],
                            [pitch,roll,yaw],
                            [vx,vy,vz]])

print(imu_state_matrix)
#2D lidar
lidar = lidar_localization.center_pose()
