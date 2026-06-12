import numpy as np
import pychrono as chrono
from abc import ABC, abstractmethod

class UAV_BASE(ABC):
    def __init__(self, controller_name: str):
        
        # --- Controller Name ---
        self.controller_name = controller_name
        
        # --- Environment parameters ---
        self.air_density = 1.225    # Air density [kg/m^3]
        self.G_acc = 9.80665        # Gravitational acceleration
        
        # --- UAV name ---
        self.name = "" # Get the UAV name from the class name
    
        # --- Number of Propellers ---
        self.number_of_propellers = 0
        
        # --- Aerodynamics ---
        self.surface_area = 0.0
        self.drag_coefficient = 0.0
        
        # --- Motor Parameters ---
        self.maximum_motor_thrust = 0.0
        self.minimum_motor_thrust = 0.0
        
        self.propellers_spin_directions = []
        self.motor_efficiency_matrix = None
        self.motor_efficiency_matrix_after_failure = None

        # --- Propeller dynamics ---
        self.K_omega = 0.0
        self.K_torque = 0.0
        
        # --- Mass --- 
        self.mass_total = 0.0
        # --- Center of Mass ---
        self.center_of_mass_estimated = chrono.ChVector3d(0, 0, 0)
        # --- Inertia matrix of the system (NED orientation) ---
        # (x-front, y-right, z-down), computed at the center of mass
        self.Inertia_mat_pixhawk = np.eye(3) # (Specific defined in each UAV subclass)
        
        ## --- Inverse Mixer Matrix ---
        self.U_mat_inv = np.array([])
        
        # --------- CONTROLLER RELATED PARAMETERS ---------
        # Time after the start of simulation at which the controller is switched ON
        self.controller_start_time = 0.0

        # --- Basic UAV-level estimates ---
        self.mass_total_estimated = 0.0
        self.I_matrix_estimated = []
        self.surface_area_estimated = 0.0
        self.drag_coefficient_estimated = 0.0
        self.air_density_estimated = 1.225

        self.drag_coefficient_matrix_estimated = []

        # --- Derivative Filter Gains ---
        self.A_phi_ref = []
        self.B_phi_ref = []
        self.C_phi_ref = []
        self.D_phi_ref = 0.0

        self.A_theta_ref = []
        self.B_theta_ref = []
        self.C_theta_ref = []
        self.D_theta_ref = 0.0

        # --- Controller Gains Config file references ---
        self.controller_config_filename = {}
    
    def loadFromYAML(self, cfg: dict) -> None:
        """Generic loader for shared parameters."""
        
        # --- Environment parameters ---
        self.air_density = cfg["uav"]["environment"].get("air_density", 1.225)
        self.G_acc = cfg["uav"]["environment"].get("G_acc", 9.80665)
        
        # --- Number of Propellers ---
        self.number_of_propellers = cfg["uav"].get("number_of_propellers", 0)
        
        # --- Aerodynamics ---
        self.surface_area = float(cfg["uav"]["aerodynamics"]["surface_area"])
        self.drag_coefficient = float(cfg["uav"]["aerodynamics"]["drag_coefficient"])
        
        # --- Mass --- 
        self.mass_total_estimated = cfg["uav"]["mass"].get("mass_total_estimated", 0.0)
        
        # --- Center of Mass ---
        self.center_of_mass_estimated = chrono.ChVector3d(*cfg["uav"]["mass"]["center_of_mass_estimated"])
        
        # --- Inertia Matrix ---
        # Inertia matrix of the system: (drone frame + box + propellers), computed at the center of mass
        # Originally expressed in Solidworks coordinate sys (x-front, y-up, z-right)
        # If reference frame transformation is needed, it will be done in each UAV subclass,
        # so that the final inertia matrix is expressed in Pixhawk coordinate sys (x-front, y-right, z-down)
        Lxx = float(cfg["uav"]["mass"]["inertia_estimated"]["Lxx"])
        Lyy = float(cfg["uav"]["mass"]["inertia_estimated"]["Lyy"])
        Lzz = float(cfg["uav"]["mass"]["inertia_estimated"]["Lzz"])
        Lxy = float(cfg["uav"]["mass"]["inertia_estimated"]["Lxy"])
        Lxz = float(cfg["uav"]["mass"]["inertia_estimated"]["Lxz"])
        Lyz = float(cfg["uav"]["mass"]["inertia_estimated"]["Lyz"])
        
        self.I_matrix_estimated = np.array([[Lxx, Lxy, Lxz],
                                            [Lxy, Lyy, Lyz],
                                            [Lxz, Lyz, Lzz]])
        
        # --- Motors ---
        self.maximum_motor_thrust = float(cfg["uav"]["motor"]["max_thrust"]) # Maximum thrust [N] per motor
        self.minimum_motor_thrust = float(cfg["uav"]["motor"]["min_thrust"]) # Minimum thrust [N] per motor
        # --- Propeller dynamics ---
        self.K_omega = float(cfg["uav"]["motor"]["K_omega"])
        self.K_torque = float(cfg["uav"]["motor"]["K_torque"])
        
        # Reorder or define coaxial stacking (OPTIONAL)
        # If there is a custom mapping, you can define it in flight_params.uav.motor_index_map
        if "index_map" in cfg["uav"]["thrust_realization"]:
            self.motor_index_map = cfg["uav"]["thrust_realization"]["index_map"]
            if len(self.motor_index_map) != self.number_of_propellers:
                raise ValueError(f"motor_index_map length ({len(self.motor_index_map)}) does not match number of motors ({self.number_of_propellers})")
            else:
                # Handle the efficiency matrices with new index map
                PermutationMatrix = np.eye(self.number_of_propellers)[self.motor_index_map]
                efficiency_matrix_diag = PermutationMatrix @ cfg["uav"]["thrust_realization"]["efficiency_matrix_diag"]
                motor_efficiency_matrix_after_failure = PermutationMatrix @ cfg["uav"]["thrust_realization"]["efficiency_matrix_diag_after_failure"]
        # Otherwise, assume 1:1 mapping
        else:
            self.motor_index_map = list(range(self.number_of_propellers))
            efficiency_matrix_diag = cfg["uav"]["thrust_realization"]["efficiency_matrix_diag"]
            motor_efficiency_matrix_after_failure = cfg["uav"]["thrust_realization"]["efficiency_matrix_diag_after_failure"]
            
        self.propellers_spin_directions = cfg["uav"]["thrust_realization"]["spin_directions"] # Motors' spin direction
        self.motor_efficiency_matrix = np.matrix(np.diag(efficiency_matrix_diag)) # Motor efficiency coefficients
        self.motor_efficiency_matrix_after_failure = np.matrix(np.diag(motor_efficiency_matrix_after_failure)) # Motor efficiency coefficients after failure
        
        # --------- CONTROLLER RELATED PARAMETERS ---------
        controller_cfg = cfg["controller"]
        # Time after the start of simulation at which the controller is switched ON
        self.controller_start_time = float(controller_cfg.get("controller_start_time", 0.0))

        # --- Derivative Filter Gains ---
        derivative_filter_gains_cfg = controller_cfg.get("derivative_filter_gains", {})
        self.A_phi_ref = np.matrix(derivative_filter_gains_cfg.get("A_phi_ref", np.zeros((1, 1))))
        self.B_phi_ref = np.array(derivative_filter_gains_cfg.get("B_phi_ref", np.zeros((1,))))
        self.C_phi_ref = np.matrix(derivative_filter_gains_cfg.get("C_phi_ref", np.zeros((1, 1))))
        self.D_phi_ref = derivative_filter_gains_cfg.get("D_phi_ref", 0.0)

        self.A_theta_ref = np.matrix(derivative_filter_gains_cfg.get("A_theta_ref", np.zeros((1, 1))))
        self.B_theta_ref = np.array(derivative_filter_gains_cfg.get("B_theta_ref", np.zeros((1,))))
        self.C_theta_ref = np.matrix(derivative_filter_gains_cfg.get("C_theta_ref", np.zeros((1, 1))))
        self.D_theta_ref = derivative_filter_gains_cfg.get("D_theta_ref", 0.0)

        # --- Controller Gains Config file references ---
        gains_cfg = controller_cfg.get("gain_yaml_files", {})
        self.controller_config_filename = gains_cfg.get(self.controller_name, "")
        
    @abstractmethod
    def _load_inertia(self) -> None:
        """
        Each subclass must implement this method to build the necessary parameters of the UAV.
        """
        pass
     
    @abstractmethod
    def _compute_estimated_parameters(self) -> None:
        """
        Each subclass must implement this method to build the necessary parameters of the UAV.
        """
        pass
        
    @abstractmethod
    def _compute_mixer_matrix(self, cfg) -> None:
        """
        Each subclass must implement this method to build the necessary parameters of the UAV.
        """
        pass