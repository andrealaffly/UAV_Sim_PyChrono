# =============================================================================
# File containing functions required to run 'drone_bb_test.py' etc.
#
# Author: Mattia Gramuglia
#
# Date: 12/06/2023
#
# =============================================================================

import math
import numpy as np
import pychrono as chrono

# Function that takes as input a ChVectorD element and converts it to a list
def chvector_to_list(v: chrono.ChVectorD) -> list:
    "Function that takes as input a ChVectorD element and converts it to a list"
    return [v.x, v.y, v.z]

# Function that takes as input a ChQuaternionD element and converts it to a list
def chquaternion_to_list(q: chrono.ChQuaternionD) -> list:
    "Function that takes as input a ChQuaternionD element and converts it to a list"
    return [q.e0, q.e1, q.e2, q.e3]

# Function that takes as input a ChCoordsysD element and converts it to a list
def chcoordsys_to_list(c: chrono.ChCoordsysD) -> list:
    "Function that takes as input a ChCoordsysD element and converts it to a list"
    return [chvector_to_list(c.pos), chquaternion_to_list(c.rot)]

# Function that takes as input a ChMatrix33D element and converts it to a list
def chmatrix33_to_list(M: chrono.ChMatrix33D) -> list:
    "Function that takes as input a ChMatrix33D element and converts it to a list"
    MATRIX = [[M[0,0], M[0,1], M[0,2]],
              [M[1,0], M[1,1], M[1,2]],
              [M[2,0], M[2,1], M[2,2]]]
    return MATRIX

# Function that takes a list of lists and transforms it to an array/matrix where elements can be accesed like: matrix[0,0]
def list_to_array(List):
    "Function that takes a list of lists and transforms it to an array/matrix where elements can be accesed like: matrix[0,0]"
    np_array = np.zeros(len(List[0]))
    for i in range(1,len(List) - 1):
          row_data = List[i]   # get row_data as list
          np_array = np.vstack((np_array, np.array(row_data)))
          # np_array = np.asmatrix(np_array)
    return np_array

# Function that takes a quaternion expressed in Global coord and builds the rotation Matrix to go from GLOBAL coord to LOCAL coord
# The output is in the form of an array
# For reference see the book: Baruh, Analytical Dynamics, pag.384, formula 7.7.19"
def rotmat_fromQ_Glob_to_Loc_asarray(q: chrono.ChQuaternionD) -> list:
    "Function that takes a quaternion expressed in Global coord and builds the rotation Matrix to go from GLOBAL coord to LOCAL coord. The output is in the form of an array. For reference see the book: Baruh, Analytical Dynamics, pag.384, formula 7.7.19"
    
    e0 = q.e0
    e1 = q.e1
    e2 = q.e2
    e3 = q.e3
    RotMat = [[(e0**2 + e1**2 - e2**2 - e3**2), 2*(e1*e2 + e0*e3), 2*(e1*e3 - e0*e2)],
              [2*(e1*e2 - e0*e3), (e0**2 - e1**2 + e2**2 - e3**2), 2*(e2*e3 + e0*e1)],
              [2*(e1*e3 + e0*e2), 2*(e2*e3 - e0*e1), (e0**2 - e1**2 - e2**2 + e3**2)]]
    return RotMat

# Function that takes a quaternion expressed in Global coord and builds the rotation Matrix to go from GLOBAL coord to LOCAL coord
# The output is in the form of a ChMatrix33D
# For reference see the book: Baruh, Analytical Dynamics, pag.384, formula 7.7.19"
def rotmat_fromQ_Glob_to_Loc_asChMatrix33(q: chrono.ChQuaternionD) -> chrono.ChMatrix33D:
    "Function that takes a quaternion expressed in Global coord and builds the rotation Matrix to go from GLOBAL coord to LOCAL coord. The output is in the form of a ChMatrix33D. For reference see the book: Baruh, Analytical Dynamics, pag.384, formula 7.7.19"
    e0 = q.e0
    e1 = q.e1
    e2 = q.e2
    e3 = q.e3
    RotMat = [[(e0**2 + e1**2 - e2**2 - e3**2), 2*(e1*e2 + e0*e3), 2*(e1*e3 - e0*e2)],
               [2*(e1*e2 - e0*e3), (e0**2 - e1**2 + e2**2 - e3**2), 2*(e2*e3 + e0*e1)],
               [2*(e1*e3 + e0*e2), 2*(e2*e3 - e0*e1), (e0**2 - e1**2 - e2**2 + e3**2)]]
    
    RotMat_asChMatrix33 = chrono.ChMatrix33D()
    RotMat_asChMatrix33.SetMatr(RotMat)
    return RotMat_asChMatrix33

# Function that takes a quaternion expressed in Local coord and builds the rotation Matrix to go from LOCAL coord to GLOBAL coord
# The output is in the form of a ChMatrix33D
# For reference see the book: Baruh, Analytical Dynamics, pag.384, formula 7.7.19"
def rotmat_fromQ_Loc_to_Glob_asChMatrix33(q: chrono.ChQuaternionD) -> chrono.ChMatrix33D:
    "Function that takes a quaternion expressed in Local coord and builds the rotation Matrix to go from LOCAL coord to GLOBAL coord. The output is in the form of a ChMatrix33D. For reference see the book: Baruh, Analytical Dynamics, pag.384, formula 7.7.19"
    e0 = q.e0
    e1 = q.e1
    e2 = q.e2
    e3 = q.e3
    
    RotMat = [[(e0**2 + e1**2 - e2**2 - e3**2), 2*(e1*e2 - e0*e3), 2*(e1*e3 + e0*e2)],
              [2*(e1*e2 + e0*e3), (e0**2 - e1**2 + e2**2 - e3**2), 2*(e2*e3 - e0*e1)],
              [2*(e1*e3 - e0*e2), 2*(e2*e3 + e0*e1), (e0**2 - e1**2 - e2**2 + e3**2)]]
    
    RotMat_asChMatrix33 = chrono.ChMatrix33D()
    RotMat_asChMatrix33.SetMatr(RotMat)
    return RotMat_asChMatrix33

# Function that takes a quaternion expressed in Global coord and computes the 3-2-1 sequence of Euler angles.
# The output is in the form of a ChVectorD.
# For reference see: 'https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles'
def euler321_fromQ_asChVector(q: chrono.ChQuaternionD) -> chrono.ChVectorD:
    "Function that takes a quaternion expressed in Global coord and computes the 3-2-1 sequence of Euler angles. The output is in the form of a ChVectorD. For reference see: 'https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles'"
    e0 = q.e0
    e1 = q.e1
    e2 = q.e2
    e3 = q.e3
    
    # roll = 0
    # pitch = 0
    # yaw = 0
    # euler_321 = [roll, pitch, yaw]
    
    # Roll (x-axis rotation)
    r1 = 2*(e0*e1 + e2*e3)
    r2 = 1 - 2*(e1**2 + e2**2)
    roll = math.atan2(r1,r2)
    
    # Pitch (y-axis rotation)
    p1 = math.sqrt(1 + 2*(e0*e2 - e1*e3))
    p2 = math.sqrt(1 - 2*(e0*e2 - e1*e3))
    pitch = 2*math.atan2(p1,p2) - math.pi/2
    
    # Yaw (z-axis rotation)
    y1 = 2*(e0*e3 + e1*e2)
    y2 = 1 - 2*(e2**2 + e3**2)
    yaw = math.atan2(y1, y2)
    
    euler_321_ChVector = chrono.ChVectorD()
    euler_321_ChVector.Set(roll, pitch, yaw)
    
    return euler_321_ChVector

# Function that takes as input a ChVector representing a 321 sequence of euler angles (roll, pitch, yaw)
# and gives as output a rotation matrix as a ChMatrix33.
# For reference see "A. L'Afflitto, A Mathematical Perspective on Flight Dynamics and Control, Section 1.4.2"
def rotmat_from_euler321(euler321: chrono.ChVectorD):
    "Function that takes as input a ChVector representing a 321 sequence of euler angles (roll, pitch, yaw) and gives as output a rotation matrix as a ChMatrix33. For reference see A. L'Afflitto, A Mathematical Perspective on Flight Dynamics and Control, Section 1.4.2"
    r = euler321.x # roll
    p = euler321.y # pitch
    y = euler321.z # yaw
    
    # Yaw Rotation Matrix transposed
    psi_l = [[ math.cos(y),  math.sin(y),  0], 
             [-math.sin(y),  math.cos(y),  0],
             [ 0,            0,            1]]
    
    # Pitch Rotation Matrix transposed
    theta_l = [[math.cos(p), 0,  -math.sin(p)],
               [ 0,          1,   0          ],
               [math.sin(p), 0,   math.cos(p)]]
    
    # Roll Rotation Matrix transposed
    phi_l = [[1,  0,             0          ],
             [0,  math.cos(r),   math.sin(r)],
             [0, -math.sin(r),   math.cos(r)]]
    
    psi_t = np.asarray(psi_l)
    theta_t = np.asarray(theta_l)
    phi_t = np.asarray(phi_l)
    
    R = np.matmul(phi_t, np.matmul(theta_t, psi_t))
    
    RotMat = chrono.ChMatrix33D()
   
    RotMat.SetMatr(R.tolist())
    
    return RotMat

# Function that takes in input a ChVector of Euler angles in radians and outputs a ChVector of Euler angles in degrees
def rad2deg(angles_in_rad: chrono.ChVectorD):
    "Function that takes in input a ChVector of Euler angles in radians and outputs a ChVector of Euler angles in degrees"
    
    x_rad = angles_in_rad.x
    y_rad = angles_in_rad.y
    z_rad = angles_in_rad.z
    
    x_deg = x_rad * 180/math.pi
    y_deg = y_rad * 180/math.pi
    z_deg = z_rad * 180/math.pi
    
    angles_in_deg = chrono.ChVectorD()
    angles_in_deg.Set(x_deg, y_deg, z_deg)
    
    return angles_in_deg
    
    
# Function that takes as input a ChMatrix33D representing a rotation matrix and
# gives as output a 321 sequence of euler angles (roll, pitch, yaw) as a ChVectorD.
# For reference see Matlab quaternion codes  
def euler321_from_rotmat(M: chrono.ChMatrix33D) -> chrono.ChVectorD:
    "Function that takes as input a ChMatrix33D representing a rotation matrix and gives as output a 321 sequence of euler angles (roll, pitch, yaw) as a ChVectorD. For reference see Matlab quaternion codes"
    R = np.array([[M[0,0], M[0,1], M[0,2]],
                  [M[1,0], M[1,1], M[1,2]],
                  [M[2,0], M[2,1], M[2,2]]])
    
    euler321 = chrono.ChVectorD()
    
    if ((R[2][0] - 1) < 0.01) and  ((R[2][0] - 1) > -0.01):
        euler321.Set(math.atan2(-R[0][1], R[1][1]), -math.pi/2, 0) # As ROLL, PITCH, YAW
        
    elif ((R[2][0] + 1) < 0.01) and  ((R[2][0] + 1) > -0.01):
        euler321.Set(-math.atan2(-R[0][1], R[1][1]), math.pi/2, 0) # As ROLL, PITCH, YAW
        
    else:
        euler321.Set(math.atan2(R[2][1], R[2][2]), math.atan2(-R[2][0], math.sqrt(1 - R[2][0] ** 2)), math.atan2(R[1][0], R[0][0])) # As ROLL, PITCH, YAW
    
    return euler321 # As ROLL, PITCH, YAW

# Function that takes as input a ChMatrix33D representing a rotation matrix and
# gives as output a 321 sequence of euler angles (roll, pitch, yaw) as a ChVectorD.
# For reference see Matlab quaternion codes "from_quaternion_to_321_angles_new2"  
def euler321_from_rotmat_matlab(M: chrono.ChMatrix33D) -> chrono.ChVectorD:
    "Function that takes as input a ChMatrix33D representing a rotation matrix and gives as output a 321 sequence of euler angles (roll, pitch, yaw) as a ChVectorD. For reference see Matlab quaternion codes"
    R = np.array([[M[0,0], M[0,1], M[0,2]],
                  [M[1,0], M[1,1], M[1,2]],
                  [M[2,0], M[2,1], M[2,2]]])
    
    euler321 = chrono.ChVectorD()
    
    if ((R[0][2] - 1) < 0.01) and  ((R[0][2] - 1) > -0.01):
        euler321.Set(math.atan2(-R[1][0], R[1][1]), -math.pi/2, 0) # As ROLL, PITCH, YAW
        
    elif ((R[0][2] + 1) < 0.01) and  ((R[0][2] + 1) > -0.01):
        euler321.Set(-math.atan2(-R[1][0], R[1][1]), math.pi/2, 0) # As ROLL, PITCH, YAW
        
    else:
        euler321.Set(math.atan2(R[1][2], R[2][2]), math.atan2(-R[0][2], math.sqrt(1 - R[0][2] ** 2)), math.atan2(R[0][1], R[0][0])) # As ROLL, PITCH, YAW
    
    return euler321 # As ROLL, PITCH, YAW

def euler321_from_rotmat_matlab_opposite_direction(M: chrono.ChMatrix33D) -> chrono.ChVectorD:
    "Function that takes as input a ChMatrix33D representing a rotation matrix and gives as output a 321 sequence of euler angles (roll, pitch, yaw) as a ChVectorD. For reference see Matlab quaternion codes"
    R = np.array([[M[0,0], M[0,1], M[0,2]],
                  [M[1,0], M[1,1], M[1,2]],
                  [M[2,0], M[2,1], M[2,2]]])
    
    euler321 = chrono.ChVectorD()
    
    if ((R[0][2] - 1) < 0.01) and  ((R[0][2] - 1) > -0.01):
        euler321.Set(math.atan2(-R[1][0], R[1][1]), -math.pi/2, 0) # As ROLL, PITCH, YAW
        
    elif ((R[0][2] + 1) < 0.01) and  ((R[0][2] + 1) > -0.01):
        euler321.Set(-math.atan2(-R[1][0], R[1][1]), math.pi/2, 0) # As ROLL, PITCH, YAW
        
    else:
        euler321.Set(math.atan2(-R[1][2], -R[2][2]), math.atan2(-R[0][2], -math.sqrt(1 - R[0][2] ** 2)), math.atan2(-R[0][1], -R[0][0])) # As ROLL, PITCH, YAW
    
    return euler321 # As ROLL, PITCH, YAW

def rk4singlestep(fun, dt, t0, y0):
    """
    This function does a single 4th-order Runge-Kutta integration step.

    Parameters
    ----------
    fun : TYPE
        ODE.
    dt : TYPE
        timestep.
    t0 : TYPE
        current initial time.
    y0 : TYPE
        current initial condition.

    Returns
    -------
    yout : TYPE
        result of the ODE.

    """
    
    f1 = fun(t0, y0)
    f2 = fun(t0 + dt /2, y0 + (dt / 2) * f1)
    f3 = fun(t0 + dt /2, y0 + (dt / 2) * f2)
    f4 = fun(t0 + dt, y0 + dt * f3)
    yout = y0 + (dt / 6) * (f1 + 2 * f2 + 2 * f3 + f4)
    return yout

def rk4singlestepWP(fun, dt, t0, y0, params):
    """
    This function does a single 4th-order Runge-Kutta integration step and it allow the passing of additional parameters

    Parameters
    ----------
    fun : TYPE
        ODE.
    dt : TYPE
        timestep.
    t0 : TYPE
        current initial time.
    y0 : TYPE
        current initial condition.
    params : TYPE
          parameters to be passed to fun

    Returns
    -------
    yout : TYPE
        result of the ODE.

    """
    
    f1 = fun(t0, y0, params)
    f2 = fun(t0 + dt /2, y0 + (dt / 2) * f1, params)
    f3 = fun(t0 + dt /2, y0 + (dt / 2) * f2, params)
    f4 = fun(t0 + dt, y0 + dt * f3, params)
    yout = y0 + (dt / 6) * (f1 + 2 * f2 + 2 * f3 + f4)
    return yout


def series_element(s):
    """

    Parameters
    ----------
    s : s-th term in the series used to compute reference resetting events

    Returns
    -------
    element : output of the series

    """
    alpha = 1.001
    # alpha = 2
    element = 1/(s**alpha)
    
    return element

def find_s(s_prev, weighted_e_squared):
    """"
    This function allows to compute s such that \sum s is convergent;
    see the comments after (40) in the paper.
    
    weighted_e_squared = e^T(t_j) P e(t_j)
    """

    alpha = 1.001
    # alpha = 2
    
    s = max(math.ceil(weighted_e_squared**(-1/alpha)), s_prev + 1)
        
    return s





















