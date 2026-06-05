import math
import pychrono as chrono
import pychrono.irrlicht as irr

class Visualization:
  def __init__(self, sim):
    self.sim = sim
    self.follow_cam_pos = chrono.ChVector3d(0,0,0)

  def setup(self):
    if not self.sim.mission_config.visualization_flag:
      return # Exit early if visualization is disabled
    
    # Create the Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(self.sim.m_sys)
    vis.SetWindowSize(1280,960) # (1024,768), (1536,1152)
    vis.SetWindowTitle(self.sim.uav_name + '-Copter - Controller: ' + self.sim.mission_config.controller_type)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataPath() + 'logo_chrono_alpha.png')
    vis.AddSkyBox()
    vis.AddCamera(
      chrono.ChVector3d(self.sim.m_frame.GetPos().x, 0, self.sim.m_frame.GetPos().z)
      + chrono.ChVector3d(-1.5, 2, 1.5),
      self.sim.m_frame.GetPos()
    )
    vis.AddTypicalLights()
    vis.AddLightWithShadow(
      chrono.ChVector3d(0,5,0),    # point, (3,6,2)
      chrono.ChVector3d(3,2,0),    # aimpoint, (0,0,0)
      5,                          # radius (power), (12)
      1,8,                        # near, far planes, (1,11)
      55                          # angle of FOV (55)
    )                
    vis.BindAll()
    self.sim.vis = vis

  def update(self) -> bool:
    if not self.sim.mission_config.visualization_flag or self.sim.vis is None:
      return True # Continue simulation even if visualization is off
    
    if not self.sim.vis.Run():
      return False # Signal to stop simulation loop
    
    self.sim.vis.BeginScene()

    # Camera switching logic
    mode = getattr(self.sim.mission_config, "camera_mode", "fixed")
    if mode == "default":
      self._update_camera_default()
    if mode == "side":
      self._update_camera_side()
    elif mode == "front":
      self._update_camera_front()
    elif mode == "follow":
      self._update_camera_follow()
    elif mode == "fpv":
      self._update_camera_fpv()
    elif mode == "orbit":
      self._update_camera_orbit()
    elif mode == "follow_smooth":
      self._update_camera_follow_smooth()
    elif mode == "topdown":
      self._update_camera_topdown()
    else:
      pass

    self.sim.vis.Render()
    # Draw coordinate systems
    irr.drawCoordsys(self.sim.vis, self.sim.marker_pixhawk.GetAbsCoordsys(), 0.5)  # Pixhawk NED
    irr.drawCoordsys(self.sim.vis, self.sim.global_coord, 1.0)                  # Global frame
    # Debug, center of motors
    # for i in range(self.sim.number_of_propellers):
    #   frame = self.sim.m_markers[i].GetAbsCoordsys()
    # # global_coord = chrono.ChCoordsysd(
    # #     chrono.ChVector3d(0, 0, 0),       # Position
    # #     chrono.ChQuaterniond(1, 0, 0, 0)  # Orientation (Y-up)
    # # )
    #   irr.drawCoordsys(self.sim.vis, frame, 0.3)  
    self.sim.vis.EndScene()
    return True # Continue simulation
  
  def _update_camera_default(self):
    uav_pos = self.sim.m_frame.GetPos()
    camera_position = chrono.ChVector3d(uav_pos.x, 0, uav_pos.z) + chrono.ChVector3d(-1.5, 2, 1.5)
    camera_target = self.sim.m_frame.GetPos()
    self.sim.vis.SetCameraPosition(camera_position)
    self.sim.vis.SetCameraTarget(camera_target)

  def _update_camera_side(self):
    uav_pos = self.sim.m_frame.GetPos()
    camera_position = uav_pos + chrono.ChVector3d(2, 0.2, 1)
    camera_target = uav_pos
    self.sim.vis.SetCameraPosition(camera_position)
    self.sim.vis.SetCameraTarget(camera_target)

  def _update_camera_front(self):
    uav_pos = self.sim.m_frame.GetPos()
    camera_position = chrono.ChVector3d(uav_pos.x, 0, uav_pos.z) + chrono.ChVector3d(2, 2, -1)
    camera_target = uav_pos
    self.sim.vis.SetCameraPosition(camera_position)
    self.sim.vis.SetCameraTarget(camera_target)

  def _update_camera_follow(self):
    time = self.sim.m_sys.GetChTime()
    if time < 6:
      self._update_camera_default()
    elif 6 <= time < 7:
      uav_pos = self.sim.m_frame.GetPos()
      camera_position = chrono.ChVector3d(uav_pos.x, 0, uav_pos.z) + chrono.ChVector3d(1.5, 2, -1.5)
      camera_target = uav_pos
      self.sim.vis.SetCameraPosition(camera_position)
      self.sim.vis.SetCameraTarget(camera_target)

  def _update_camera_fpv(self):
    # === Choose view mode: "front", "side", or "back" ===
    view_mode = "back"

    uav_pos = self.sim.pixhawk_state.pos_LOC
    uav_rot = self.sim.pixhawk_state.rotmat

    # --- Camera offsets for each view (body frame) ---
    offsets = {
      "front": chrono.ChVector3d( 0.55,  0.00,  -0.10),   # in front of UAV
      "side":  chrono.ChVector3d( 0.25,  0.55,  -0.10),   # to the right of UAV
      "back":  chrono.ChVector3d(-0.55,  0.00,  -0.10),   # behind UAV
    }

    # Fallback if wrong flag
    offset_body = offsets.get(view_mode, offsets[view_mode])
    # Final camera position in world coordinates
    camera_position = uav_rot * (uav_pos + offset_body)
    # Look-at point (slightly above UAV frame origin)
    camera_target = self.sim.m_frame.GetPos() + chrono.ChVector3d(0.0, 0.05, 0.0)

    self.sim.vis.SetCameraPosition(camera_position)
    self.sim.vis.SetCameraTarget(camera_target)

  def _update_camera_orbit(self):
    # Orbit parameters
    R = 2.0                     # radius of the circular orbit
    H = 1.0                     # height of the camera
    speed = 0.8                 # angular speed (rad/s)

    t = self.sim.m_sys.GetChTime()
    angle = speed * t

    uav_pos = self.sim.m_frame.GetPos()

    cam_x = uav_pos.x - R * math.cos(angle)
    cam_y = uav_pos.y + H
    cam_z = uav_pos.z - R * math.sin(angle)

    camera_position = chrono.ChVector3d(cam_x, cam_y, cam_z)
    camera_target = uav_pos
    self.sim.vis.SetCameraPosition(camera_position)
    self.sim.vis.SetCameraTarget(camera_target)

  def _update_camera_follow_smooth(self):
    uav_pos = self.sim.m_frame.GetPos()
    target = uav_pos + chrono.ChVector3d(-1.5, 1.2, 1.5)

    # Low-pass filter for smoothing
    alpha = 0.01
    self.follow_cam_pos = self.follow_cam_pos*(1 - alpha) + target*alpha

    self.sim.vis.SetCameraPosition(self.follow_cam_pos)
    self.sim.vis.SetCameraTarget(uav_pos)

  def _update_camera_topdown(self):
    uav_pos = self.sim.m_frame.GetPos()
    camera_position = uav_pos + chrono.ChVector3d(0, 8, 0)  # high above the drone

    self.sim.vis.SetCameraPosition(camera_position)
    self.sim.vis.SetCameraTarget(uav_pos)
