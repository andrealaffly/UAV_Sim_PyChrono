#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Sep 10 11:15:48 2023

@author: sanjanadhulla
"""
import os
import csv
import shutil
import drone_bb_simulator
import numpy as np
import math
import time
import datetime
import pytz
PI = math.pi

time_max = 11.24


# If wrapper needs to be run the Wrapper_execution should be True else False
# Wrapper_execution = True
Wrapper_execution = False
# If you want to see the visualization then the flag should be True 
visualization_flag = True
# visualization_flag = False


# ----------------------------------------------------------------
#                     CHOOSE THE CONTROLLER
# ----------------------------------------------------------------

# controller_type = 'PID' # Check mass_total_estimated

# controller_type = 'MRACwithBASELINE'

# controller_type = 'TwoLayerMRACwithBASELINE'

# controller_type = 'RobustMRACwithBASELINE'

controller_type = 'RobustTwoLayerMRACwithBASELINE'

# controller_type = 'HybridMRACwithBASELINE'

# controller_type = 'HybridTwoLayerMRACwithBASELINE'

# controller_type = 'HybridRobustMRACwithBASELINE'

# controller_type = 'HybridRobustTwoLayerMRACwithBASELINE'

# controller_type = 'FunnelMRACwithBASELINE'

# controller_type = 'FunnelTwoLayerMRACwithBASELINE'

# controller_type = 'MRACwithBASELINE_SafetyMechanism'

# ----------------------------------------------------------------
#                     %%%%%%%%%%%%%%%%%%%%%%
# ----------------------------------------------------------------

if Wrapper_execution == True:
    visualization_flag = False
    #----------------------------------------------------------------------------
    # Creating a folder to store all the necessary python files for future reference
    
    # Providing the folder path
    origin = os.getcwd()
    
    # Here the folder name is Wrapper_folder_name, it can be changed
    date = datetime.datetime.now(pytz.timezone('America/New_York'))
    date_folder_name = "_"+str(date.month)  + str(date.day)  + str(date.year)+"_"+ str(date.hour)+"_"+ str(date.minute)
    Wrapper_folder_name = "SafetyMech_" + controller_type
    Wrapper_important_files = Wrapper_folder_name+date_folder_name
    target = os.path.join(os.getcwd(), Wrapper_important_files)
    if not os.path.exists(target):
        os.mkdir(target)
        
    # Fetching the list of all the files
    files = os.listdir(origin)
    
    # Fetching all the files to directory and copying only python files
    for file_name in files:
        if ".py" in file_name:
            origin_with_file = os.path.join(origin, file_name)
            target_with_file = os.path.join(target, file_name)
            shutil.copy(origin_with_file, target_with_file)
    print("Files are copied successfully")
    #-----------------------------------------------------------------------------
    
    
    
    #-----------------------------------------------------------------
    # Defining the control parameters for the wrapper
    
    ball_volume = 4/3 * PI * (0.0508/2)**3
    
    payload_mass_array = np.linspace(0.005, 2.7405, 480) 
    # payload_density_array = payload_mass_array/(2 * ball_volume)
    
    # #my_ball_density = 7850
    payload_density_array =[7850]
    #payload_density_array = np.ones(500)*7850
    control_variables = {"my_ball_density":round(payload_density_array[0])}
    #-----------------------------------------------------------------
    
    
    # Creating a CSV file to store the values where the controller went crazy! This file is stored in the new folder that was created
    csv_file_path_abnormalities = os.path.join(target, "Abnormal_values.csv")
    with open(csv_file_path_abnormalities, mode='w', newline='') as csv_file_wrapper_ab:
        csv_writer = csv.writer(csv_file_wrapper_ab)
        column_headers = control_variables.keys()
        csv_writer.writerow(column_headers)
    
    
    # Iterating through the control parameters
    for var,val in control_variables.items():
        # Naming the folder  after the controller_type and control parameter 
        # folder_name = var + "_" + controller_type
        folder_name = "data_" + controller_type # Modified by Mattia
        for i in range(len(payload_density_array)):
            payload_density_current = round(payload_density_array[i])
            control_variables[var] = payload_density_current
            file_name = "value_" + str(i+1)
            drone_bb_simulator.WrapperMain_function(target, controller_type, control_variables,folder_name, file_name,
                                                           time_max, csv_file_path_abnormalities, Wrapper_execution, visualization_flag)

else:
    ball_density = 7850
    control_variables = {"my_ball_density":ball_density}
    drone_bb_simulator.WrapperMain_function(None, controller_type, 
        control_variables,None, None, time_max, None, Wrapper_execution, visualization_flag)