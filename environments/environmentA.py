# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Users\mgramuglia\Desktop\JTARV\Mattia Drone\SW_world\environmentA.SLDASM


import pychrono as chrono 
import builtins 

# some global settings: 
sphereswept_r = 0.001
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.003)
chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.002)

shapes_dir = 'environmentA_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('environmentA-1')
body_1.SetPos(chrono.ChVectorD(-3.7,0,0))
body_1.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1.SetMass(2010)
body_1.SetInertiaXX(chrono.ChVectorD(1168.14720149254,3382.48964552239,4058.23684701493))
body_1.SetInertiaXY(chrono.ChVectorD(-174.915111940299,4.44557490469223e-13,-3.72506596464338e-14))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(7.03917910447761,0.895522388059702,0.3),chrono.ChQuaternionD(1,0,0,0)))
body_1.SetBodyFixed(True)

# Visualization shape 
# body_1_1_shape = chrono.ChObjFileShape() 
# body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
# body_1.AddVisualShape(body_1_1_shape, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.ChQuaternionD(1,0,0,0)))

# Attach a visualization shape
# First load a .obj from disk into a ChTriangleMeshConnected:
mesh_for_visualization1 = chrono.ChTriangleMeshConnected()
mesh_for_visualization1.LoadWavefrontMesh(shapes_dir +'body_1_1.obj')
# Now the  triangle mesh is inserted in a ChTriangleMeshShape visualization asset, 
# and added to the body
visualization_shape1 = chrono.ChTriangleMeshShape()
visualization_shape1.SetMesh(mesh_for_visualization1)
visualization_shape1.SetColor(chrono.ChColor(0.96,0.96,0.86)) # beige
# visualization_shape1.SetTexture(chrono.GetChronoDataFile("textures/light_gray.png"))
body_1.AddVisualShape(visualization_shape1)

# Collision material 
mat_1 = chrono.ChMaterialSurfaceNSC()

# Collision shapes 
body_1.GetCollisionModel().ClearModel()

# Triangle mesh collision shape 
body_1_1_collision_mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(shapes_dir + 'body_1_1_collision.obj', False, True) 
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_1_1_collision_mesh.Transform(chrono.ChVectorD(0, 0, 0), mr) 
body_1.GetCollisionModel().AddTriangleMesh(mat_1, body_1_1_collision_mesh, False, False, chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(chrono.ChQuaternionD(1,0,0,0)), sphereswept_r) 
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



