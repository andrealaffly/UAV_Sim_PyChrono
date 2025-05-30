from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Tuple
import numpy as np

@dataclass
class UserDefinedTrajectoryState:
  position: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
  velocity: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
  acceleration: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
  yaw: float = 0.0
  yaw_dot: float = 0.0
  yaw_dot_dot: float = 0.0

class BaseUserDefinedTrajectory(ABC):
  @abstractmethod
  def computeUserDefinedTrajectory(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    pass

  @abstractmethod
  def computeUserDefinedYaw(self) -> Tuple[float, float, float]:
    pass

  def compute(self, t: float) -> UserDefinedTrajectoryState:
    (pos, vel, acc) = self.computeUserDefinedTrajectory(t)
    (yaw, yaw_dot, yaw_dot_dot) = self.computeUserDefinedYaw()
    return UserDefinedTrajectoryState(
      position=pos,
      velocity=vel,
      acceleration=acc,
      yaw=yaw,
      yaw_dot=yaw_dot,
      yaw_dot_dot=yaw_dot_dot
    )