import pychrono as chrono
import pychrono.irrlicht as irr

class Visualization:
  def __init__(self, sim):
    self.sim = sim

  def setup(self):
    if not self.sim.simulation_config.visualization_flag:
      return # Exit early if visualization is disabled
    
    # Create the Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(self.sim.m_sys)
    vis.SetWindowSize(1280,960) # (1024,768), (1536,1152)
    vis.SetWindowTitle('X8-Copter - Controller: ' + self.sim.simulation_config.controller_type)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
    vis.AddSkyBox()
    # vis.AddCamera(chrono.ChVectorD(2.5, 1.5, 0.5)) # (1,1,1), (2.5,1.5,0.5) FIXED CAMERA
    vis.AddCamera(
      chrono.ChVectorD(self.sim.m_frame.GetPos().x, 0, self.sim.m_frame.GetPos().z)
      + chrono.ChVectorD(-1.5, 2, 1.5),
      self.sim.m_frame.GetPos()
    )
    # vis.AddCamera(chrono.ChVectorD(6,1.5,3), chrono.ChVectorD(3,0,-2)) # (2, 1, 3), (2,0.5,1.5)
    vis.AddTypicalLights()
    vis.AddLightWithShadow(
      chrono.ChVectorD(0,5,0),    # point, (3,6,2)
      chrono.ChVectorD(3,2,0),    # aimpoint, (0,0,0)
      5,                          # radius (power), (12)
      1,8,                        # near, far planes, (1,11)
      55                          # angle of FOV (55)
    )                
    vis.BindAll()
    # self.vis = vis
    self.sim.vis = vis

  def update(self) -> bool:
    if not self.sim.simulation_config.visualization_flag or self.sim.vis is None:
      return True # Continue simulation even if visualization is off
    
    if not self.sim.vis.Run():
      return False # Signal to stop simulation loop
    
    self.sim.vis.BeginScene()

    # Camera switching logic
    mode = getattr(self.sim.simulation_config, "camera_mode", "fixed")
    if mode == "default":
      self._add_camera_default()
    if mode == "side":
      self._add_camera_side()
    elif mode == "front":
      self._add_camera_front()
    elif mode == "follow":
      self._add_camera_follow()
    elif mode == "fpv":
      self._add_camera_fpv()
    else:
      pass

    self.sim.vis.Render()
    # Draw coordinate systems
    irr.drawCoordsys(self.sim.vis, self.sim.marker_pixhawk.GetAbsCoord(), 0.5)  # Pixhawk NED
    irr.drawCoordsys(self.sim.vis, self.sim.global_coord, 1.0)                  # Global frame
    self.sim.vis.EndScene()
    return True # Continue simulation
  
  def _add_camera_default(self):
    self.sim.vis.AddCamera(
      chrono.ChVectorD(self.sim.m_frame.GetPos().x, 0, self.sim.m_frame.GetPos().z)
      + chrono.ChVectorD(-1.5, 2, 1.5),
      self.sim.m_frame.GetPos()
    )

  def _add_camera_side(self):
    self.sim.vis.AddCamera(self.sim.m_frame.GetPos() + chrono.ChVectorD(2, 0.2, 1), self.sim.m_frame.GetPos())

  def _add_camera_front(self):
    self.sim.vis.AddCamera(
      chrono.ChVectorD(self.sim.m_frame.GetPos().x, 0, self.sim.m_frame.GetPos().z)
      + chrono.ChVectorD(2, 2, -1), self.sim.m_frame.GetPos()
    )

  def _add_camera_follow(self):
    time = self.sim.m_sys.GetChTime()
    if time < 6:
      self._add_camera_default()
    elif 6 <= time < 7:
      self.sim.vis.AddCamera(
        chrono.ChVectorD(self.sim.m_frame.GetPos().x, 0, self.sim.m_frame.GetPos().z)
        + chrono.ChVectorD(1.5, 2, -1.5), self.sim.m_frame.GetPos()
      )

  def _add_camera_fpv(self):
    if self.sim.m_sys.GetChTime() > 0.1:
      fpv_pos = self.sim.pixhawk_state.rotmat * (self.sim.pixhawk_state.pos_LOC + chrono.ChVectorD(-0.4, 0, -0.1))
      self.sim.vis.AddCamera(fpv_pos, self.sim.m_frame.GetPos() + chrono.ChVectorD(0, 0.05, 0))