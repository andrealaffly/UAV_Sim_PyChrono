import os
import datetime
import subprocess
import warnings
import numpy as np
from scipy.io import savemat
from dataclasses import is_dataclass
import acsl_pychrono.config.config as Cfg

class Logging:
  @staticmethod
  def getOutputDir(sim_cfg: Cfg.SimulationConfig) -> str:
    controller_type = sim_cfg.mission_config.controller_type
    wrapper_flag = sim_cfg.mission_config.wrapper_flag

    # Get current time
    now = datetime.datetime.now()
    year = now.strftime("%Y")
    month = now.strftime("%m")
    full_date = now.strftime("%Y%m%d")

    # Construct the directory path
    if wrapper_flag and sim_cfg.mission_config.wrapper_batch_dir:
      dir_path = os.path.join(sim_cfg.mission_config.wrapper_batch_dir)
    else:
      dir_path = os.path.join("logs", year, month, full_date, controller_type, "workspaces")
    os.makedirs(dir_path, exist_ok=True)  # Create all directories if not present

    return dir_path
  
  @staticmethod
  def generateUniqueFilename(base_name: str, extension: str, dir_path: str, use_suffix: bool) -> str:
    if not use_suffix:
      # No suffix, allow overwrite
      return os.path.join(dir_path, f"{base_name}.{extension}")
    
    # Suffix to avoid overwrite
    run_id = 1
    while True:
      filename = f"{base_name}-{run_id}.{extension}"
      full_path = os.path.join(dir_path, filename)
      if not os.path.exists(full_path):
        return full_path
      run_id += 1

  @staticmethod
  def extractGainsDict(gains) -> dict:
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
    
    return gains_dict_shortened

  @staticmethod
  def dataclassToDict(obj, truncate_keys: bool = True):
    """
    Recursively convert a dataclass to a nested dictionary,
    handling nested dataclasses, NumPy arrays/matrices,
    truncating field names to 31 characters if specified,
    and filtering out unsupported types.
    """
    if is_dataclass(obj):
      result = {}
      for key, value in obj.__dict__.items():
        # Filter to only include serializable fields
        if isinstance(value, (int, float, str, bool, np.ndarray, np.matrix)) or is_dataclass(value):
          converted_value = Logging.dataclassToDict(value, truncate_keys=truncate_keys)
          key = key[:31] if truncate_keys else key
          if isinstance(converted_value, np.matrix):
            converted_value = np.array(converted_value)
          result[key] = converted_value
      return result
    elif isinstance(obj, dict):
      return {k[:31] if truncate_keys else k: Logging.dataclassToDict(v, truncate_keys=truncate_keys) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
      return [Logging.dataclassToDict(v, truncate_keys=truncate_keys) for v in obj]
    elif isinstance(obj, np.matrix):
      return np.array(obj)
    else:
      return obj
    
  @staticmethod
  def getGitRepoInfo() -> dict:
    def run_git_cmd(args):
      try:
        return subprocess.check_output(['git'] + args, cwd=repo_dir, stderr=subprocess.DEVNULL).decode().strip()
      except subprocess.CalledProcessError:
        return ""
      except FileNotFoundError:
        return ""
      
    def get_github_url(repo_url: str, commit_hash: str) -> str:
      if repo_url.endswith(".git"):
        repo_url = repo_url[:-4]
      return f"{repo_url}/tree/{commit_hash}"
    
    repo_dir = os.getcwd()

    if not os.path.exists(os.path.join(repo_dir, '.git')):
      warnings.warn("⚠️ Git repository info cannot be tracked. No .git directory found.")
      return {
        "repo_path": "",
        "remote_url": "",
        "commit_hash": "",
        "commit_tag": "",
        "branch": "",
        "dirty": 0,
        "commit_url": ""
      }
    
    remote_url = run_git_cmd(['remote', 'get-url', 'origin']) or ""
    commit_hash = run_git_cmd(['rev-parse', 'HEAD']) or ""

    git_info = {
      "repo_path": os.path.abspath(repo_dir),
      "remote_url": remote_url,
      "commit_hash": commit_hash,
      "commit_tag": run_git_cmd(['describe', '--tags', '--exact-match']) or "",
      "branch": run_git_cmd(['rev-parse', '--abbrev-ref', 'HEAD']) or "",
      "dirty": 1 if run_git_cmd(['status', '--porcelain']) else 0,
      "commit_url": get_github_url(remote_url, commit_hash) or ""
    }

    return git_info

  @staticmethod
  def saveMatlabWorkspaceLog(log_dict, gains, sim_cfg: Cfg.SimulationConfig, git_info: dict | None = None):
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    dir_path = Logging.getOutputDir(sim_cfg)
    os.makedirs(dir_path, exist_ok=True)

    base_filename = f"workspace_log_{timestamp}"
    full_path_log = Logging.generateUniqueFilename(
      base_filename,
      "mat",
      dir_path,
      Cfg.MissionConfig.wrapper_flag
    )

    mat_dict = {
      "log": log_dict,
      "gains": Logging.extractGainsDict(gains),
      "sim_cfg": Logging.dataclassToDict(sim_cfg)
    }

    if git_info is not None:
      mat_dict["git_info"] = git_info

    savemat(full_path_log, mat_dict)
    
  @staticmethod
  def dataVectorPadding(data_vector: np.ndarray, desired_length: int) -> np.ndarray:
    current_length = data_vector.shape[0]
    if current_length >= desired_length:
      return data_vector
    
    data_vector = data_vector.reshape(current_length, 1).flatten() # Flatten to 1D
    padded_vector = np.pad(data_vector, (0, desired_length - current_length), 'constant')
    return padded_vector