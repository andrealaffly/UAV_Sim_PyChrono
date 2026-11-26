import os
import csv
import time
import rclpy
from rclpy.node import Node
from uav_interfaces.srv import ComputeControl
import numpy as np

class FlightStackServiceBridge: 
    """
    Replaces the internal Python controller with a lock-step ROS2 service call
    to the C++ flightstack. The simulation loop calls run() as normal; this
    class sends state, blocks until motor commands arrive, then applies them.
    """
    def __init__(self, node: Node, sim):
        self._node = node
        self.sim = sim  # Hold a reference to the simulation to read states directly
        self._client = node.create_client(ComputeControl, '/sil/compute_control')
        node.get_logger().info('FlightStack Bridge: waiting for /sil/compute_control ...')
        if not self._client.wait_for_service(timeout_sec=90.0):
            raise RuntimeError('FlightStack service not available. Is the C++ container running?')
        node.get_logger().info('FlightStack Bridge: connected.')

        self.req = ComputeControl.Request()
        self.motor_thrusts = np.zeros((self.sim.number_of_propellers, 1))

        # -- Lazy-initialised (sim.number_of_propellers may not exist yet) --
        self._thrusts_ready = False
        self.motor_thrusts = None

        # -- Sub-sampling: the C++ flightstack integrates with time_step_rk4_ = 0.01 s.
        # The sim may run at a finer timestep (e.g. 0.005 s).  We only forward a service
        # call every round(0.01 / sim_timestep) physics steps so the C++ RK4 step matches
        # the real elapsed time between calls. --
        _cpp_dt = 0.01  # must match time_step_rk4_ hardcoded in the C++ controller headers
        _sim_dt  = getattr(getattr(sim, 'mission_config', None), 'timestep', _cpp_dt)
        self._call_stride = max(1, round(_cpp_dt / _sim_dt))  # e.g. 2 for 5 ms sim
        self._step_count  = 0
        node.get_logger().info(
            f'FlightStack Bridge: sim dt={_sim_dt*1000:.1f} ms, '
            f'C++ dt={_cpp_dt*1000:.0f} ms → calling C++ every {self._call_stride} step(s).'
        )

        # .. odein: populated on every run() call from the simulation loop --
        self.odein = None

        # -- PID-internal attributes required by PIDLogger.collectData() --
        # These are computed inside the C++ controller and not returned by
        # the service, so they are logged as zero for SIL runs.
        self.roll_ref = 0.0
        self.pitch_ref = 0.0
        self.angular_position_ref_dot  = np.zeros((3, 1))
        self.angular_position_ref_ddot = np.zeros((3, 1))
        self.angular_position_dot      = np.zeros((3, 1))
        self.mu_x = 0.0
        self.mu_y = 0.0
        self.mu_z = 0.0
        self.u1   = 0.0
        self.u2   = 0.0
        self.u3   = 0.0
        self.u4   = 0.0

        # -- MRAC-internal attributes required by MRACLogger.collectData() --
        # These are computed inside the C++ controller and not returned by
        # the service, so they are logged as zero for SIL runs.
        self.x_ref_tran          = np.zeros((6, 1))
        self.omega_ref           = np.zeros((3, 1))
        self.mu_adaptive_tran    = np.zeros((3, 1))
        self.mu_PD_baseline_tran = np.zeros((3, 1))
        self.Moment_baseline     = np.zeros((3, 1))
        self.Moment_adaptive     = np.zeros((3, 1))
        self.Moment_baseline_PI  = np.zeros((3, 1))
        self.omega_cmd           = np.zeros((3, 1))
        self.omega_cmd_dot       = np.zeros((3, 1))
        self.omega_ref_dot       = np.zeros((3, 1))
        self.r_tran              = np.zeros((3, 1))
        self.r_rot               = np.zeros((3, 1))
        self.K_hat_x_tran   = np.zeros(18)
        self.K_hat_r_tran   = np.zeros(9)
        self.Theta_hat_tran = np.zeros(18)
        self.K_hat_x_rot    = np.zeros(9)
        self.K_hat_r_rot    = np.zeros(9)
        self.Theta_hat_rot  = np.zeros(18)
        self.dead_zone_value_tran = 0.0
        self.dead_zone_value_rot  = 0.0
        self.proj_op_activated_K_hat_x_tran  = False
        self.proj_op_activated_K_hat_r_tran  = False
        self.proj_op_activated_Theta_hat_tran = False
        self.proj_op_activated_K_hat_x_rot   = False
        self.proj_op_activated_K_hat_r_rot   = False
        self.proj_op_activated_Theta_hat_rot = False

        # -- TwoLayerMRAC-internal attributes required by TwoLayerMRACLogger.collectData() --
        # These are computed inside the C++ controller and not returned by
        # the service, so they are logged as zero for SIL runs.
        self.proj_op_activated_K_hat_g_tran = False
        self.proj_op_activated_K_hat_g_rot  = False
        self.K_hat_g_tran = np.zeros(18)
        self.K_hat_g_rot  = np.zeros(9)
        self.mu_adaptive_mrac_tran = np.zeros((3, 1))
        self.mu_adaptive_ebci_tran = np.zeros((3, 1))
        self.Moment_adaptive_mrac  = np.zeros((3, 1))
        self.Moment_adaptive_ebci  = np.zeros((3, 1))


        # -- FunnelTwoLayerMRAC-internal attributes required by FunnelTwoLayerMRACLogger.collectData() --
        # These are computed inside the C++ controller and not returned by
        # the service, so they are logged as zero for SIL runs.
        self.eta_funnel_tran = 0.0
        self.eta_dot_funnel_tran = 0.0
        self.xi_funnel_tran = 0.0

        self.lambda_sat_funnel_tran = 0.0
        self.sigma_nom_funnel_tran = 0.0
        self.sigma_ideal_funnel_tran = 0.0

        self.H_function_funnel_tran = 0.0
        self.Ve_funnel_tran = 0.0
        self.eT_M_e_tran = 0.0
        self.diameter_funnel_tran = 0.0

        self.e_tran_dot = np.zeros((6, 1))

        # -- Diagnostic CSV: per-call record of what sil_interface SENT
        # (raw motor_commands, before applyMotorThrustLimitsAndEfficiency)
        # plus call timing, for comparison against the sim's logged
        # "applied" thrusts (thrust_motors_N_T{i}) and to spot
        # synchronization issues. Overwritten at the start of each run.
        os.makedirs("logs", exist_ok=True)
        self._diag_file = open(os.path.join("logs", "sil_bridge_diagnostics.csv"), "w", newline="")
        self._diag_writer = csv.writer(self._diag_file)
        self._diag_writer.writerow(
            ["time", "dt_sim", "wall_dt"] + [f"T{i+1}_sent" for i in range(8)]
        )
        self._prev_sim_time = None
        self._prev_wall_time = None

    # MOCK RUN METHOD: Catches the exact call from runSimulationLoop()
    def run(self, ode_input=None, *args, **kwargs):
        # -- 0. Store ode_input (already populated by simulation loop) -- 
        # All odein.* attributes (position, velocity, euler angles, yaw_ref,
        # trajectory references) are already set by the simulation loop
        # before run() is called — no manual copying needed.
        if ode_input is None:
            raise RuntimeError("The Bridge expects the ode_input object from the simulation loop!")
        self.odein = ode_input

        # -- 1. Lazy-init motor_thrusts array (must happen before any early return) --
        if not self._thrusts_ready:
            self.motor_thrusts = np.zeros((self.sim.number_of_propellers, 1))
            self._thrusts_ready = True

        # -- Sub-sampling guard: skip C++ call on intermediate physics steps --
        # The C++ time_step_rk4_ = 0.01 s; we only call it every _call_stride steps
        # so the RK4 integrator advances in sync with real simulation time.
        self._step_count += 1
        if self._step_count % self._call_stride != 0:
            return  # hold last motor_thrusts (zeros on first skip, then last computed)

        # -- 2. Direct extraction of states from ode_input ---------------------------
        sim_time = self.odein.time_now

        # Extracting position, linear velocity and angular velocity
        pos = self.odein.translational_position_in_I.flatten()
        vel = self.odein.translational_velocity_in_I.flatten()
        omega = self.odein.angular_velocity.flatten()
        
        # Euler angles (useful for debug logs)
        phi = self.odein.roll
        theta = self.odein.pitch
        psi = self.odein.yaw
        
        # Watch out for the signs

        c1 = np.cos(phi/2)
        s1 = np.sin(phi/2)
        c2 = np.cos(theta/2)
        s2 = np.sin(theta/2)
        c3 = np.cos(psi/2)
        s3 = np.sin(psi/2)
        
        qx = s1 * c2 * c3 - c1 * s2 * s3
        qy = c1 * s2 * c3 + s1 * c2 * s3
        qz = c1 * c2 * s3 - s1 * s2 * c3
        qw = c1 * c2 * c3 + s1 * s2 * s3

        # Scalar last convention for quaternions
        quat = [qx, qy, qz, qw]  

        # -- 4. Packing and sending the ROS 2 service request -- ---------------------
        self.req.time = float(sim_time)
        self.req.px, self.req.py, self.req.pz = float(pos[0]), float(pos[1]), float(pos[2])
        self.req.vx, self.req.vy, self.req.vz = float(vel[0]), float(vel[1]), float(vel[2])
        self.req.qx, self.req.qy, self.req.qz, self.req.qw = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
        self.req.wx, self.req.wy, self.req.wz = float(omega[0]), float(omega[1]), float(omega[2])
        
        '''
         # ---- LOG DI DEBUG PER IL QUATERNIONE ----
        if int(sim_time * 100) % 50 == 0:  # Stampa ogni 0.5 secondi
            print(f"\n[DEBUG SIMULATORE BRIDGE t={sim_time:.2f}s]")
            print(f"  Dati estratti da OdeInput -> Pos: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
            print(f"  Angoli in OdeInput (Roll/Pitch/Yaw) rad: {phi:.4f}, {theta:.4f}, {psi:.4f}")
            print(f"  Quaternione nativo Chrono [qx, qy, qz, qw]: {[f'{x:.4f}' for x in quat]}")
            print(f"  Inviato a ROS2 Req -> qx:{self.req.qx:.4f}, qy:{self.req.qy:.4f}, qz:{self.req.qz:.4f}, qw:{self.req.qw:.4f}")
        # -----------------------------------------
        '''
        future = self._client.call_async(self.req)
        # timeout_sec bounds how long we wait if flightstack crashes mid-call (e.g. an
        # unhandled C++ exception aborts the node) - without it, a dead service leaves
        # this future incomplete forever and hangs the whole run.
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)

        res = future.result()
        if res is None:
            raise RuntimeError(f'Service call failed at t={sim_time:.4f}')

        # -- Diagnostic logging: what sil_interface sent (raw motor_commands,
        # before the sim's own thrust saturation/efficiency step) + timing --
        now_wall = time.perf_counter()
        dt_sim  = 0.0 if self._prev_sim_time  is None else sim_time - self._prev_sim_time
        wall_dt = 0.0 if self._prev_wall_time is None else now_wall - self._prev_wall_time
        self._prev_sim_time  = sim_time
        self._prev_wall_time = now_wall
        self._diag_writer.writerow(
            [sim_time, dt_sim, wall_dt] + [f"{t:.6f}" for t in res.motor_commands]
        )
        self._diag_file.flush()

    
        # ---- C++ Response logging ----
        # if int(sim_time * 100) % 50 == 0 and res is not None:
        #    print(f"  Response from C++ Flightstack: {res.motor_commands[:8]}")
        # -------------------------------------------

        # 3. Update the fake controller attribute with real Newtons from the C++ Flightstack
        for idx, thrust_newton in enumerate(res.motor_commands):
            if idx < self.sim.number_of_propellers:
                # thrust_vector is raw (pre-saturation) Newtons; clamp negatives so
                # PyChrono never sees a downward thrust during instabilities.
                self.motor_thrusts[idx][0] = max(0.0, thrust_newton)

        if int(sim_time * 100) % 50 == 0 and res is not None:
            verbi_newton = [float(self.motor_thrusts[i][0]) for i in range(8)]
            print(f"  Response from C++ Flightstack (NEWTON): {[f'{n:.4f} N' for n in verbi_newton]}")

        # -------------------------------------------

        # Note: We do NOT need to run integrateODEOneStepRK4() here! 
        # The C++ flightstack is handling all the math. We just need the thrust output.