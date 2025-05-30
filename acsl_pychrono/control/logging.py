import os
import datetime
import numpy as np
from scipy.io import savemat

class Logging:
  @staticmethod
  def saveMatlabWorkspaceLog(log_dict, gains, controller_type):
    """
    Save the log and gains data into a .mat file in a structured directory.

    Args:
      log_dict (dict): Dictionary containing the log data.
      gains (object): Object containing gain parameters as attributes.
      controller_type (str): The name/type of the controller.
    """

    # Get current time
    now = datetime.datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    year = now.strftime("%Y")
    month = now.strftime("%m")
    full_date = now.strftime("%Y%m%d")

    # Construct the directory path
    dir_path = os.path.join("logs", year, month, full_date, controller_type, "workspaces")
    os.makedirs(dir_path, exist_ok=True)  # Create all directories if not present

    # Create a dictionary from instance variables
    gains_dict = {
      key: value for key, value in gains.__dict__.items()
      if isinstance(value, (int, float, np.ndarray, np.matrix))
    }
    # Truncate field names to 31 characters
    gains_dict_shortened = {}
    for key, value in gains_dict.items():
      shortened_key = key[:31]  # truncate to 31 characters
      gains_dict_shortened[shortened_key] = value
    # Convert matrices to arrays for MATLAB compatibility
    for key in gains_dict_shortened:
      if isinstance(gains_dict_shortened[key], np.matrix):
        gains_dict_shortened[key] = np.array(gains_dict_shortened[key])

    # Nest gains inside the log structure
    mat_dict = {
      "log": log_dict,  # Log data
      "gains": gains_dict_shortened  # Nested gains struct
    }

    # Construct the full file path for the log file
    filename_log = f"workspace_log_{timestamp}.mat"
    full_path_log = os.path.join(dir_path, filename_log)

    # Save the .mat file with the nested gains structure
    savemat(full_path_log, mat_dict)