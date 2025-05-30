# -*- coding: utf-8 -*-
"""
Created on Mon Mar  4 11:54:15 2024

@author: grem6
"""

import math
import numpy as np
from numpy.polynomial import polynomial

class traj_functions:
  def PolyCoefAssigning(poly_coeff_matrix_in):
    "Assigning polynomial coefficients to designated variables"
    
    poly_coeff_matrix_in_size = poly_coeff_matrix_in.shape
    
    poly_coeff_matrix_out_size = [int(poly_coeff_matrix_in_size[0]/3),
                                  int(poly_coeff_matrix_in_size[1])]
    
    poly_coeff_matrix_out_x = np.zeros((poly_coeff_matrix_out_size[0],
                                        poly_coeff_matrix_out_size[1]))
    poly_coeff_matrix_out_y = np.zeros((poly_coeff_matrix_out_size[0],
                                        poly_coeff_matrix_out_size[1]))
    poly_coeff_matrix_out_z = np.zeros((poly_coeff_matrix_out_size[0],
                                        poly_coeff_matrix_out_size[1]))
    
    for i in range(poly_coeff_matrix_out_size[0]):
      poly_coeff_matrix_out_x[i,:] = poly_coeff_matrix_in[3*i + 0, :]
      poly_coeff_matrix_out_y[i,:] = poly_coeff_matrix_in[3*i + 1, :]
      poly_coeff_matrix_out_z[i,:] = poly_coeff_matrix_in[3*i + 2, :]
    
    return [poly_coeff_matrix_out_x,
            poly_coeff_matrix_out_y,
            poly_coeff_matrix_out_z]
  
  def PolyderMatrix(poly_coeff_matrix_in):
    "Derivative of the piecewise polynomial coefficient matrix"
    
    poly_coeff_matrix_in_size = poly_coeff_matrix_in.shape
    
    poly_coeff_matrix_out = np.zeros((poly_coeff_matrix_in_size[0],
                                      poly_coeff_matrix_in_size[1] - 1))
    
    for i in range(poly_coeff_matrix_in_size[0]):
      poly_coeff_matrix_out[i,:] = polynomial.polyder(poly_coeff_matrix_in[i,:])
      
    return poly_coeff_matrix_out
  
  def PolyTimeAdjusted(time_waypoint_vector,t):
    """Adjusts the time to be fed to the various polynomials and identifies the
    segment of the trajectory"""
    
    num_waypoints = time_waypoint_vector.size
    segment = 0
    t_adjusted = 0
    
    for i in range(num_waypoints):
      if (t >= time_waypoint_vector[i] and t <= time_waypoint_vector[i + 1]):
        segment = i
        t_adjusted = t - time_waypoint_vector[segment]
        break
    
    return [t_adjusted, segment]
  
  def Norm2D(poly_coef_x,poly_coef_y,t):
    """Computes the 2D norm from polynomial coefficients of the components of
    the vector"""
    
    norm_value = math.sqrt((polynomial.polyval(t, poly_coef_x))**2 +
                           (polynomial.polyval(t, poly_coef_y))**2)
    return norm_value
  
  def Norm2Dderivative(poly_coef_x,
                       poly_coef_y,
                       poly_coef_x_prime,
                       poly_coef_y_prime,
                       t):
    """Computes the derivative of the 2D norm from polynomial
    coefficients of the components of the vector"""
    
    derivative_value = ((polynomial.polyval(t, poly_coef_x) * 
                         polynomial.polyval(t, poly_coef_x_prime) +
                         polynomial.polyval(t, poly_coef_y) * 
                         polynomial.polyval(t, poly_coef_y_prime)) /
                         traj_functions.Norm2D(poly_coef_x,poly_coef_y,t))
    return derivative_value
    
  def YawComputation(Vx_coef,Vy_coef,t):
    "Yaw computation from trajectory velocity in XY plane"
    
    Vx = polynomial.polyval(t, Vx_coef)
    Vy = polynomial.polyval(t, Vy_coef)

    yaw = math.atan2(Vy, Vx)
    return yaw
  
  def YawDotComputation(Vx_coef,Vy_coef,Ax_coef,Ay_coef,t):
    "Yaw dot computation from trajectory velocity in XY plane"
    
    Ax = polynomial.polyval(t, Ax_coef)
    Ay = polynomial.polyval(t, Ay_coef)
    acceleration_angle = math.atan2(Ay, Ax)
    
    velocity_norm = traj_functions.Norm2D(Vx_coef,Vy_coef,t)
    acceleration_norm = traj_functions.Norm2D(Ax_coef,Ay_coef,t)
    
    yaw = traj_functions.YawComputation(Vx_coef,Vy_coef,t)

    yaw_dot = ((acceleration_norm/velocity_norm) * 
               math.sin(acceleration_angle - yaw))
    return yaw_dot
  
  def YawDotDotComputation(Vx_coef,Vy_coef,
                           Ax_coef,Ay_coef,
                           Jx_coef,Jy_coef,
                           t):
    "Yaw dot dot computation from trajectory velocity in XY plane"
    
    Jx = polynomial.polyval(t, Jx_coef)
    Jy = polynomial.polyval(t, Jy_coef)
    jerk_angle = math.atan2(Jy, Jx)
    
    velocity_norm = traj_functions.Norm2D(Vx_coef,Vy_coef,t)
    jerk_norm = traj_functions.Norm2D(Jx_coef,Jy_coef,t)
    
    velocity_norm_prime = traj_functions.Norm2Dderivative(Vx_coef,
                                                          Vy_coef,
                                                          Ax_coef,
                                                          Ay_coef,
                                                          t)
    
    yaw = traj_functions.YawComputation(Vx_coef,Vy_coef,t)
    yaw_dot = traj_functions.YawDotComputation(Vx_coef,Vy_coef,
                                               Ax_coef,Ay_coef,t)
    
    yaw_dot_dot = (jerk_norm * math.sin(jerk_angle - yaw) - 
                   2*velocity_norm_prime*yaw_dot)/velocity_norm
    return yaw_dot_dot
  
  def SamplingTimeVector(waypointTimes, samplingTime):
    "Create a vector with a defined sampling time to draw the trajectory"
    
    sampling_time_vector = np.arange(waypointTimes[0],
                                     waypointTimes[-1] + samplingTime,
                                     samplingTime)
    return sampling_time_vector
    
    
    
    
    
    
    
    