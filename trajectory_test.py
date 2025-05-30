# -*- coding: utf-8 -*-
"""
Created on Mon Mar  4 11:43:36 2024

@author: grem6
"""

from trajectory_functions import traj_functions
import numpy as np
from numpy.polynomial import polynomial
import matplotlib.pyplot as plt

pp_coefficients = np.loadtxt(open("trajectory_PolynomialCoefficientMatrix.csv",
                                  "rb"), delimiter=",", skiprows=0)
waypointTimes = np.loadtxt(open("trajectory_WaypointTimes.csv", "rb"),
                           delimiter=",", skiprows=0)

[position_coef_x,
 position_coef_y,
 position_coef_z] = traj_functions.PolyCoefAssigning(pp_coefficients)

velocity_coef_x = traj_functions.PolyderMatrix(position_coef_x)
velocity_coef_y = traj_functions.PolyderMatrix(position_coef_y)
velocity_coef_z = traj_functions.PolyderMatrix(position_coef_z)

acceleration_coef_x = traj_functions.PolyderMatrix(velocity_coef_x)
acceleration_coef_y = traj_functions.PolyderMatrix(velocity_coef_y)
acceleration_coef_z = traj_functions.PolyderMatrix(velocity_coef_z)

jerk_coef_x = traj_functions.PolyderMatrix(acceleration_coef_x)
jerk_coef_y = traj_functions.PolyderMatrix(acceleration_coef_y)
jerk_coef_z = traj_functions.PolyderMatrix(acceleration_coef_z)

step_size = 0.01
t = np.arange(waypointTimes[0], waypointTimes[-1] + step_size, step_size)

t_adjusted = np.zeros(t.size)
segment = np.zeros(t.size, dtype=int)
position_x = np.zeros(t.size)
position_y = np.zeros(t.size)
position_z = np.zeros(t.size)
velocity_x = np.zeros(t.size)
velocity_y = np.zeros(t.size)
velocity_z = np.zeros(t.size)
velocity_norm2D = np.zeros(t.size)
acceleration_x = np.zeros(t.size)
acceleration_y = np.zeros(t.size)
acceleration_z = np.zeros(t.size)
jerk_x = np.zeros(t.size)
jerk_y = np.zeros(t.size)
jerk_z = np.zeros(t.size)
yaw = np.zeros(t.size)
yaw_dot = np.zeros(t.size)
yaw_dot_dot = np.zeros(t.size)


for i in range(t.size):
  [t_adjusted[i],segment[i]] = traj_functions.PolyTimeAdjusted(waypointTimes,t[i])
  
  position_x[i] = polynomial.polyval(t_adjusted[i], position_coef_x[segment[i],:])
  position_y[i] = polynomial.polyval(t_adjusted[i], position_coef_y[segment[i],:])
  position_z[i] = polynomial.polyval(t_adjusted[i], position_coef_z[segment[i],:])
  
  velocity_x[i] = polynomial.polyval(t_adjusted[i], velocity_coef_x[segment[i],:])
  velocity_y[i] = polynomial.polyval(t_adjusted[i], velocity_coef_y[segment[i],:])
  velocity_z[i] = polynomial.polyval(t_adjusted[i], velocity_coef_z[segment[i],:])
  velocity_norm2D[i] = traj_functions.Norm2D(velocity_coef_x[segment[i],:],
                                             velocity_coef_y[segment[i],:],
                                             t_adjusted[i])
  
  acceleration_x[i] = polynomial.polyval(t_adjusted[i], acceleration_coef_x[segment[i],:])
  acceleration_y[i] = polynomial.polyval(t_adjusted[i], acceleration_coef_y[segment[i],:])
  acceleration_z[i] = polynomial.polyval(t_adjusted[i], acceleration_coef_z[segment[i],:])
  
  jerk_x[i] = polynomial.polyval(t_adjusted[i], jerk_coef_x[segment[i],:])
  jerk_y[i] = polynomial.polyval(t_adjusted[i], jerk_coef_y[segment[i],:])
  jerk_z[i] = polynomial.polyval(t_adjusted[i], jerk_coef_z[segment[i],:])
  
  if t_adjusted[i] == 0:
    yaw[i] = 0
    yaw_dot[i] = 0
    yaw_dot_dot[i] = 0   
    
  elif (t_adjusted[i] > 0 and velocity_norm2D[i] < 1e-5):
    yaw[i] = yaw[i-1]
    yaw_dot[i] = 0
    yaw_dot_dot[i] = 0 
  else:
    yaw[i] = traj_functions.YawComputation(velocity_coef_x[segment[i],:],
                                           velocity_coef_y[segment[i],:],
                                           t_adjusted[i])
    yaw_dot[i] = traj_functions.YawDotComputation(velocity_coef_x[segment[i],:],
                                                  velocity_coef_y[segment[i],:],
                                                  acceleration_coef_x[segment[i],:],
                                                  acceleration_coef_y[segment[i],:],
                                                  t_adjusted[i])
    yaw_dot_dot[i] = (traj_functions.
                      YawDotDotComputation(velocity_coef_x[segment[i],:],
                                           velocity_coef_y[segment[i],:],
                                           acceleration_coef_x[segment[i],:],
                                           acceleration_coef_y[segment[i],:],
                                           jerk_coef_x[segment[i],:],
                                           jerk_coef_y[segment[i],:],
                                           t_adjusted[i]))
  

#%% Plotting
# Plotting position in 3D
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(position_x, position_y, position_z, 'blue')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.view_init(20, 45)
ax.set_title('Position')
ax.set_aspect('equal')
ax.invert_zaxis()
ax.invert_yaxis()

# Plotting x-position
fig = plt.figure()
plt.plot(t, position_x)
plt.xlabel('t [s]')
plt.ylabel('x-position [m]')
plt.title('X-Position')

# Plotting x-velocity
fig = plt.figure()
plt.plot(t, velocity_x)
plt.xlabel('t [s]')
plt.ylabel('x-velocity [m/s]')
plt.title('X-velocity')

# Plotting x-acceleration
fig = plt.figure()
plt.plot(t, acceleration_x)
plt.xlabel('t [s]')
plt.ylabel('x-acceleration [m/s^2]')
plt.title('X-acceleration')

# Plotting x-jerk
fig = plt.figure()
plt.plot(t, jerk_x)
plt.xlabel('t [s]')
plt.ylabel('x-jerk [m/s^3]')
plt.title('X-jerk')

# Plotting yaw
fig = plt.figure()
plt.plot(t, yaw)
plt.xlabel('t [s]')
plt.ylabel('yaw [rad]')
plt.title('Yaw')

# Plotting yaw rate
fig = plt.figure()
plt.plot(t, yaw_dot)
plt.xlabel('t [s]')
plt.ylabel('yaw rate [rad/s]')
plt.title('Yaw rate')

# Plotting yaw acceleration
fig = plt.figure()
plt.plot(t, yaw_dot_dot)
plt.xlabel('t [s]')
plt.ylabel('yaw acceleration [rad/s^2]')
plt.title('Yaw acceleration')

  
  
                                                               
                                                               