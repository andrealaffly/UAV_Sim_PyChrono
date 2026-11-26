import sys
import numpy as np
from numpy import linalg as LA


class Utils:
  @staticmethod
  def formatVector(vec, precision=4):
    """Format a vector (NumPy array or list) to a list of strings with given precision."""
    formatted = []
    for v in vec:
      try:
        val = v.item() if hasattr(v, 'item') else v
        formatted.append(f'{val:.{precision}f}')
      except Exception as e:
        formatted.append(str(v))  # fallback for debug purposes
    return formatted

  @staticmethod
  def printLabeledVector(label, vec, precision=4):
    """Print a label followed by a formatted vector."""
    formatted = Utils.formatVector(vec, precision)
    print(f'{label}:', ' '.join(formatted))

  @staticmethod
  def printLabeledScalar(label, val, precision=4):
    """Print a label followed by a single formatted scalar."""
    print(f'{label}: {val:.{precision}f}')

  @staticmethod
  def printLabeledNormAndVector(label, vec, precision=4):
    """Print norm and each component of a vector with a label."""
    normVal = LA.norm(vec)
    components = [v.item() if hasattr(v, 'item') else v for v in vec]
    formatted = Utils.formatVector([normVal] + components, precision)
    print(f'{label} [norm x y z]:', ' '.join(formatted))

  @staticmethod
  def printControllerDebugInfo(controller, time_now, flight_params, print_console_flag):
    """Main function to print all controller debug info."""
    if not print_console_flag or time_now <= flight_params.controller_start_time:
      return

    Utils.printLabeledVector('angular_error', controller.angular_error)
    Utils.printLabeledVector('angular_error_dot', controller.angular_error_dot)
    Utils.printLabeledVector('Thrust T', controller.motor_thrusts)
    Utils.printLabeledScalar('Total thrust', np.sum(controller.motor_thrusts))

    euler_angles_deg = (
      np.rad2deg(controller.odein.roll),
      np.rad2deg(controller.odein.pitch),
      np.rad2deg(controller.odein.yaw),
    )
    Utils.printLabeledVector('Pixhawk Euler 321 angles [deg]', euler_angles_deg)
    Utils.printLabeledScalar('Roll reference [deg]', np.rad2deg(controller.roll_ref))
    Utils.printLabeledScalar('Pitch reference [deg]', np.rad2deg(controller.pitch_ref))
    Utils.printLabeledScalar('Yaw reference [deg]', np.rad2deg(controller.odein.yaw_ref))

    Utils.printLabeledVector('Controllers U1 U2 U3 U4', [controller.u1, controller.u2, controller.u3, controller.u4])
    Utils.printLabeledVector('mu', [controller.mu_x, controller.mu_y, controller.mu_z])

    Utils.printLabeledNormAndVector('mu_PD_baseline_tran', controller.mu_PD_baseline_tran)
    Utils.printLabeledNormAndVector('mu_baseline_tran', controller.mu_baseline_tran)
    Utils.printLabeledNormAndVector('mu_adaptive_tran', controller.mu_adaptive_tran)
    Utils.printLabeledNormAndVector('Moment_baseline_PI', controller.Moment_baseline_PI)
    Utils.printLabeledNormAndVector('Moment_baseline', controller.Moment_baseline)
    Utils.printLabeledNormAndVector('Moment_adaptive', controller.Moment_adaptive)

    Utils.printLabeledVector('Pixhawk GLOBAL Position', controller.odein.translational_position_in_I)
    Utils.printLabeledScalar('Pixhawk GLOBAL Velocity NORM', LA.norm(controller.odein.translational_velocity_in_I))

  @staticmethod
  def printSimulationTimeInline(label1, val1, label2, val2, precision=4):
    """Print two labeled scalars on the same line, overwriting previous output for cleaner console."""
    out_str = f"{label1}: {val1:.{precision}f} s   {label2}: {val2:.{precision}f} s --- "
    # \r = carriage return, \033[K = clear line from cursor to end
    sys.stdout.write('\r\033[K' + out_str)
    sys.stdout.flush()
