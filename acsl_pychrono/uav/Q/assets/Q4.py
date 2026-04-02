# PyChrono model automatically generated using Chrono::SolidWorks add-in
# Assembly: D:\Virginia-Tech-PhD\PHD_research\PyChrono\UAV_CAD_Models\Q4\PyChronoAssembly\UAV_Assembly_pychrono.SLDASM


import pychrono as chrono 
import builtins 

# Some global settings 
sphereswept_r = 0.001
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.003)
chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.002)

shapes_dir = 'Q4_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0 = chrono.ChBodyAuxRef()
body_0.SetName('SLDW_GROUND')
body_0.SetFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1 = chrono.ChBodyAuxRef()
body_1.SetName('Propeller-1')
body_1.SetPos(chrono.ChVector3d(0.202459825780886,0.0273700000000008,0.134579864496243))
body_1.SetRot(chrono.ChQuaterniond(0.590897628643018,-0.590897628643022,-0.388381246282639,-0.388381246282639))
# body_1.SetMass(0.00541929141423071)
# body_1.SetInertiaXX(chrono.ChVector3d(9.71849966526597e-06,2.20610914193957e-06,1.1009339485845e-05))
# body_1.SetInertiaXY(chrono.ChVector3d(-3.99108183951827e-06,-7.65514773696266e-07,-1.77168914114701e-06))
body_1.SetMass(1e-12) # Modified by Xavier, the original is in the lines above
body_1.SetInertiaXX(chrono.ChVector3d(1e-12,1e-12,1e-12)) # Modified by Xavier, the original is in the lines above
body_1.SetInertiaXY(chrono.ChVector3d(0,0,0)) # Modified by Xavier, the original is in the lines above
body_1.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.91230608809784e-11,5.61998649220364e-12,0.00128496582935105),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChVisualShapeModelFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_shape.SetColor(chrono.ChColor(1, 0, 0))
body_1.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_1)



# Rigid body part
body_2 = chrono.ChBodyAuxRef()
body_2.SetName('Propeller-2')
body_2.SetPos(chrono.ChVector3d(-0.202459825780885,0.0273700000000008,0.134579864496243))
body_2.SetRot(chrono.ChQuaterniond(-0.494459179313888,0.494459179313891,0.505480088620941,0.505480088620941))
# body_2.SetMass(0.00541929141423071)
# body_2.SetInertiaXX(chrono.ChVector3d(1.14376489761126e-05,4.86959831092927e-07,1.1009339485845e-05))
# body_2.SetInertiaXY(chrono.ChVector3d(2.41534481514844e-07,4.25380990011109e-08,-1.92952994061737e-06))
body_2.SetMass(1e-12) # Modified by Xavier, the original is in the lines above
body_2.SetInertiaXX(chrono.ChVector3d(1e-12,1e-12,1e-12)) # Modified by Xavier, the original is in the lines above
body_2.SetInertiaXY(chrono.ChVector3d(0,0,0)) # Modified by Xavier, the original is in the lines above
body_2.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.91230608809784e-11,5.61998649220364e-12,0.00128496582935105),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
# body_1_1_shape = chrono.ChVisualShapeModelFile() 
# body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_2.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_2)



# Rigid body part
body_3 = chrono.ChBodyAuxRef()
body_3.SetName('UAV_Assembly_part_single-1')
body_3.SetPos(chrono.ChVector3d(0,0,0))
body_3.SetRot(chrono.ChQuaterniond(1,0,0,0))
body_3.SetMass(0.725155689894245)
body_3.SetInertiaXX(chrono.ChVector3d(0.00281476025893015,0.00776849449509212,0.00662126607982517))
body_3.SetInertiaXY(chrono.ChVector3d(-3.01324974056602e-05,-9.38857838027414e-06,-4.4113720829106e-06))
body_3.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(0.00412025498024158,-0.0378957456822676,-0.000167739309237873),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChVisualShapeModelFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_shape.SetColor(chrono.ChColor(0.1, 0.1, 0.1))
body_3.AddVisualShape(body_3_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

# Collision Model

body_3.AddCollisionModel(chrono.ChCollisionModel())

# Collision material 
mat_3 = chrono.ChContactMaterialNSC()

# Collision parameters 
mat_3.SetFriction(0.6)
body_3.GetCollisionModel().SetEnvelope(0.03)
body_3.GetCollisionModel().SetSafeMargin(0.01)
mr = chrono.ChMatrix33d()
mr[0,0]=-1; mr[1,0]=-3.19029604777344E-17; mr[2,0]=-1.27611841910938E-16 
mr[0,1]=-9.25185853854297E-17; mr[1,1]=3.33066907387547E-15; mr[2,1]=1 
mr[0,2]=-3.1902960477734E-17; mr[1,2]=1; mr[2,2]=-3.33066907387547E-15 
collshape = chrono.ChCollisionShapeBox(mat_3,0.435,0.3,0.071)
body_3.GetCollisionModel().AddShape(collshape,chrono.ChFramed(chrono.ChVector3d(-4.05008083264838E-17,-0.00149999999999994,3.49720252756924E-17), mr))
mr = chrono.ChMatrix33d()
mr[0,0]=1; mr[1,0]=4.67265582754696E-17; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-3.3831423014075E-15; mr[2,1]=-1 
mr[0,2]=-4.67265582754696E-17; mr[1,2]=1; mr[2,2]=-3.3831423014075E-15 
collshape = chrono.ChCollisionShapeBox(mat_3,0.297,0.201,0.0718)
body_3.GetCollisionModel().AddShape(collshape,chrono.ChFramed(chrono.ChVector3d(-2.94330590577183E-17,-0.0728999999999999,2.53245462190461E-16), mr))
body_3.EnableCollision(True)

exported_items.append(body_3)



# Rigid body part
body_4 = chrono.ChBodyAuxRef()
body_4.SetName('Propeller-3')
body_4.SetPos(chrono.ChVector3d(-0.202459825780885,0.0273699999999997,-0.134579864496244))
body_4.SetRot(chrono.ChQuaterniond(0.504665691166877,-0.504665691166881,-0.495290359444898,-0.495290359444898))
# body_4.SetMass(0.00541929141423071)
# body_4.SetInertiaXX(chrono.ChVector3d(1.14391202759541e-05,4.85488531251439e-07,1.1009339485845e-05))
# body_4.SetInertiaXY(chrono.ChVector3d(-2.054872108774e-07,-3.6187167449684e-08,-1.92965949600375e-06))
body_4.SetMass(1e-12) # Modified by Xavier, the original is in the lines above
body_4.SetInertiaXX(chrono.ChVector3d(1e-12,1e-12,1e-12)) # Modified by Xavier, the original is in the lines above
body_4.SetInertiaXY(chrono.ChVector3d(0,0,0)) # Modified by Xavier, the original is in the lines above
body_4.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.91230608809784e-11,5.61998649220364e-12,0.00128496582935105),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
# body_1_1_shape = chrono.ChVisualShapeModelFile() 
# body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_4.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_4)



# Rigid body part
body_5 = chrono.ChBodyAuxRef()
body_5.SetName('Propeller-4')
body_5.SetPos(chrono.ChVector3d(0.202459825780886,0.0273699999999997,-0.134579864496244))
body_5.SetRot(chrono.ChQuaterniond(-0.494459179313888,0.494459179313892,0.505480088620941,0.505480088620941))
# body_5.SetMass(0.00541929141423071)
# body_5.SetInertiaXX(chrono.ChVector3d(1.14376489761126e-05,4.86959831092927e-07,1.1009339485845e-05))
# body_5.SetInertiaXY(chrono.ChVector3d(2.41534481514845e-07,4.2538099001111e-08,-1.92952994061737e-06))
body_5.SetMass(1e-12) # Modified by Xavier, the original is in the lines above
body_5.SetInertiaXX(chrono.ChVector3d(1e-12,1e-12,1e-12)) # Modified by Xavier, the original is in the lines above
body_5.SetInertiaXY(chrono.ChVector3d(0,0,0)) # Modified by Xavier, the original is in the lines above
body_5.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.91230608809784e-11,5.61998649220364e-12,0.00128496582935105),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
# body_1_1_shape = chrono.ChVisualShapeModelFile() 
# body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_5.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_5)




# Mate constraint: Concentric1 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_1 , SW name: Propeller-1 ,  SW ref.type:2 (2)
link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(0.202459825780886,0.0338700000000008,0.134579864496244)
dA = chrono.ChVector3d(1.87112097223968e-16,-1,3.22335161018178e-15)
cB = chrono.ChVector3d(0.202459825780885,0.0234432222152577,0.134579864496244)
dB = chrono.ChVector3d(8.91062684370096e-17,1,-3.17373878119849e-15)
link_1.SetFlipped(True)
link_1.Initialize(body_3,body_1,False,cA,cB,dA,dB)
link_1.SetName("Concentric1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(0.202459825780886,0.0338700000000008,0.134579864496244)
cB = chrono.ChVector3d(0.202459825780885,0.0234432222152577,0.134579864496244)
dA = chrono.ChVector3d(1.87112097223968e-16,-1,3.22335161018178e-15)
dB = chrono.ChVector3d(8.91062684370096e-17,1,-3.17373878119849e-15)
link_2.Initialize(body_3,body_1,False,cA,cB,dA,dB)
link_2.SetName("Concentric1")
exported_items.append(link_2)


# Mate constraint: Coincident5 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_1 , SW name: Propeller-1 ,  SW ref.type:2 (2)
link_3 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(0.202459825780886,0.0238700000000008,0.134579864496244)
cB = chrono.ChVector3d(0.202459825780885,0.0238700000000008,0.134579864496244)
dA = chrono.ChVector3d(-5.55111512312578e-17,1,-3.22953809397064e-15)
dB = chrono.ChVector3d(-1.37287919294573e-16,-1,3.10057942247187e-15)
link_3.Initialize(body_3,body_1,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident5")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(0.202459825780886,0.0238700000000008,0.134579864496244)
dA = chrono.ChVector3d(-5.55111512312578e-17,1,-3.22953809397064e-15)
cB = chrono.ChVector3d(0.202459825780885,0.0238700000000008,0.134579864496244)
dB = chrono.ChVector3d(-1.37287919294573e-16,-1,3.10057942247187e-15)
link_4.SetFlipped(True)
link_4.Initialize(body_3,body_1,False,cA,cB,dA,dB)
link_4.SetName("Coincident5")
exported_items.append(link_4)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Propeller-2 ,  SW ref.type:2 (2)
link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.202459825780886,0.0338700000000007,0.134579864496244)
dA = chrono.ChVector3d(1.87112097223968e-16,-1,3.22335161018178e-15)
cB = chrono.ChVector3d(-0.202459825780886,0.0234432222152577,0.134579864496244)
dB = chrono.ChVector3d(1.29421597815656e-16,1,-3.17403978552113e-15)
link_5.SetFlipped(True)
link_5.Initialize(body_3,body_2,False,cA,cB,dA,dB)
link_5.SetName("Concentric2")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.202459825780886,0.0338700000000007,0.134579864496244)
cB = chrono.ChVector3d(-0.202459825780886,0.0234432222152577,0.134579864496244)
dA = chrono.ChVector3d(1.87112097223968e-16,-1,3.22335161018178e-15)
dB = chrono.ChVector3d(1.29421597815656e-16,1,-3.17403978552113e-15)
link_6.Initialize(body_3,body_2,False,cA,cB,dA,dB)
link_6.SetName("Concentric2")
exported_items.append(link_6)


# Mate constraint: Coincident6 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Propeller-2 ,  SW ref.type:2 (2)
link_7 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.202459825780886,0.0238700000000007,0.134579864496244)
cB = chrono.ChVector3d(-0.202459825780886,0.0238700000000008,0.134579864496244)
dA = chrono.ChVector3d(-5.55111512312578e-17,1,-3.22953809397064e-15)
dB = chrono.ChVector3d(-1.42728072110856e-16,-1,3.08745629407801e-15)
link_7.Initialize(body_3,body_2,False,cA,cB,dB)
link_7.SetDistance(0)
link_7.SetName("Coincident6")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.202459825780886,0.0238700000000007,0.134579864496244)
dA = chrono.ChVector3d(-5.55111512312578e-17,1,-3.22953809397064e-15)
cB = chrono.ChVector3d(-0.202459825780886,0.0238700000000008,0.134579864496244)
dB = chrono.ChVector3d(-1.42728072110856e-16,-1,3.08745629407801e-15)
link_8.SetFlipped(True)
link_8.Initialize(body_3,body_2,False,cA,cB,dA,dB)
link_8.SetName("Coincident6")
exported_items.append(link_8)


# Mate constraint: Concentric3 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Propeller-3 ,  SW ref.type:2 (2)
link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.202459825780886,0.0338699999999996,-0.134579864496244)
dA = chrono.ChVector3d(1.87112097223968e-16,-1,3.22335161018178e-15)
cB = chrono.ChVector3d(-0.202459825780886,0.0234432222152565,-0.134579864496244)
dB = chrono.ChVector3d(1.83797555721066e-16,1,-3.1740201249123e-15)
link_9.SetFlipped(True)
link_9.Initialize(body_3,body_4,False,cA,cB,dA,dB)
link_9.SetName("Concentric3")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateGeneric()
link_10.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.202459825780886,0.0338699999999996,-0.134579864496244)
cB = chrono.ChVector3d(-0.202459825780886,0.0234432222152565,-0.134579864496244)
dA = chrono.ChVector3d(1.87112097223968e-16,-1,3.22335161018178e-15)
dB = chrono.ChVector3d(1.83797555721066e-16,1,-3.1740201249123e-15)
link_10.Initialize(body_3,body_4,False,cA,cB,dA,dB)
link_10.SetName("Concentric3")
exported_items.append(link_10)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: Propeller-4 ,  SW ref.type:2 (2)
link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(0.202459825780886,0.0338699999999997,-0.134579864496244)
dA = chrono.ChVector3d(1.87112097223968e-16,-1,3.22335161018178e-15)
cB = chrono.ChVector3d(0.202459825780885,0.0234432222152565,-0.134579864496244)
dB = chrono.ChVector3d(2.29185260896735e-16,1,-3.17376908294713e-15)
link_11.SetFlipped(True)
link_11.Initialize(body_3,body_5,False,cA,cB,dA,dB)
link_11.SetName("Concentric4")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(0.202459825780886,0.0338699999999997,-0.134579864496244)
cB = chrono.ChVector3d(0.202459825780885,0.0234432222152565,-0.134579864496244)
dA = chrono.ChVector3d(1.87112097223968e-16,-1,3.22335161018178e-15)
dB = chrono.ChVector3d(2.29185260896735e-16,1,-3.17376908294713e-15)
link_12.Initialize(body_3,body_5,False,cA,cB,dA,dB)
link_12.SetName("Concentric4")
exported_items.append(link_12)


# Mate constraint: Coincident8 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: Propeller-4 ,  SW ref.type:2 (2)
link_13 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(0.202459825780886,0.0238699999999996,-0.134579864496244)
cB = chrono.ChVector3d(0.202459825780885,0.0238699999999996,-0.134579864496244)
dA = chrono.ChVector3d(-5.55111512312578e-17,1,-3.22953809397064e-15)
dB = chrono.ChVector3d(-2.42491735191934e-16,-1,3.08718559150401e-15)
link_13.Initialize(body_3,body_5,False,cA,cB,dB)
link_13.SetDistance(0)
link_13.SetName("Coincident8")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(0.202459825780886,0.0238699999999996,-0.134579864496244)
dA = chrono.ChVector3d(-5.55111512312578e-17,1,-3.22953809397064e-15)
cB = chrono.ChVector3d(0.202459825780885,0.0238699999999996,-0.134579864496244)
dB = chrono.ChVector3d(-2.42491735191934e-16,-1,3.08718559150401e-15)
link_14.SetFlipped(True)
link_14.Initialize(body_3,body_5,False,cA,cB,dA,dB)
link_14.SetName("Coincident8")
exported_items.append(link_14)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Propeller-3 ,  SW ref.type:2 (2)
link_15 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.202459825780886,0.0238699999999996,-0.134579864496244)
cB = chrono.ChVector3d(-0.202459825780886,0.0238699999999996,-0.134579864496244)
dA = chrono.ChVector3d(-5.55111512312578e-17,1,-3.22953809397064e-15)
dB = chrono.ChVector3d(-2.00623998422913e-16,-1,3.08805132781329e-15)
link_15.Initialize(body_3,body_4,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident9")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.202459825780886,0.0238699999999996,-0.134579864496244)
dA = chrono.ChVector3d(-5.55111512312578e-17,1,-3.22953809397064e-15)
cB = chrono.ChVector3d(-0.202459825780886,0.0238699999999996,-0.134579864496244)
dB = chrono.ChVector3d(-2.00623998422913e-16,-1,3.08805132781329e-15)
link_16.SetFlipped(True)
link_16.Initialize(body_3,body_4,False,cA,cB,dA,dB)
link_16.SetName("Coincident9")
exported_items.append(link_16)


# # Mate constraint: Coincident17 [MateCoincident] type:0 align:0 flip:False
# #   Entity 0: C::E name: body_0 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:4 (4)
# #   Entity 1: C::E name: body_0 , SW name: UAV_Assembly_pychrono ,  SW ref.type:4 (4)
# link_17 = chrono.ChLinkMateDistanceZ()
# cA = chrono.ChVector3d(0,0,0)
# cB = chrono.ChVector3d(0,0,0)
# dA = chrono.ChVector3d(0,0,1)
# dB = chrono.ChVector3d(0,0,1)
# link_17.Initialize(body_3,body_0,False,cA,cB,dB)
# link_17.SetDistance(0)
# link_17.SetName("Coincident17")
# exported_items.append(link_17)

# link_18 = chrono.ChLinkMateParallel()
# cA = chrono.ChVector3d(0,0,0)
# dA = chrono.ChVector3d(0,0,1)
# cB = chrono.ChVector3d(0,0,0)
# dB = chrono.ChVector3d(0,0,1)
# link_18.Initialize(body_3,body_0,False,cA,cB,dA,dB)
# link_18.SetName("Coincident17")
# exported_items.append(link_18)


# # Mate constraint: Coincident18 [MateCoincident] type:0 align:0 flip:False
# #   Entity 0: C::E name: body_0 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:4 (4)
# #   Entity 1: C::E name: body_0 , SW name: UAV_Assembly_pychrono ,  SW ref.type:4 (4)
# link_19 = chrono.ChLinkMateDistanceZ()
# cA = chrono.ChVector3d(0,0,0)
# cB = chrono.ChVector3d(0,0,0)
# dA = chrono.ChVector3d(0,1,0)
# dB = chrono.ChVector3d(0,1,0)
# link_19.Initialize(body_3,body_0,False,cA,cB,dB)
# link_19.SetDistance(0)
# link_19.SetName("Coincident18")
# exported_items.append(link_19)

# link_20 = chrono.ChLinkMateParallel()
# cA = chrono.ChVector3d(0,0,0)
# dA = chrono.ChVector3d(0,1,0)
# cB = chrono.ChVector3d(0,0,0)
# dB = chrono.ChVector3d(0,1,0)
# link_20.Initialize(body_3,body_0,False,cA,cB,dA,dB)
# link_20.SetName("Coincident18")
# exported_items.append(link_20)


# # Mate constraint: Coincident19 [MateCoincident] type:0 align:0 flip:False
# #   Entity 0: C::E name: body_0 , SW name: UAV_Assembly_part_single-1 ,  SW ref.type:4 (4)
# #   Entity 1: C::E name: body_0 , SW name: UAV_Assembly_pychrono ,  SW ref.type:4 (4)
# link_21 = chrono.ChLinkMateDistanceZ()
# cA = chrono.ChVector3d(0,0,0)
# cB = chrono.ChVector3d(0,0,0)
# dA = chrono.ChVector3d(1,0,0)
# dB = chrono.ChVector3d(1,0,0)
# link_21.Initialize(body_3,body_0,False,cA,cB,dB)
# link_21.SetDistance(0)
# link_21.SetName("Coincident19")
# exported_items.append(link_21)

# link_22 = chrono.ChLinkMateParallel()
# cA = chrono.ChVector3d(0,0,0)
# dA = chrono.ChVector3d(1,0,0)
# cB = chrono.ChVector3d(0,0,0)
# dB = chrono.ChVector3d(1,0,0)
# link_22.Initialize(body_3,body_0,False,cA,cB,dA,dB)
# link_22.SetName("Coincident19")
# exported_items.append(link_22)


# Auxiliary marker (coordinate system feature)
marker_0_1 = chrono.ChMarker()
marker_0_1.SetName('Coordinate System1')
body_0.AddMarker(marker_0_1)
marker_0_1.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(0.202459825780886,0.0273700000000008,0.134579864496243),chrono.ChQuaterniond(0.707106781186546,-0.707106781186549,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_2 = chrono.ChMarker()
marker_0_2.SetName('Coordinate System2')
body_0.AddMarker(marker_0_2)
marker_0_2.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(-0.202459825780885,0.0273700000000008,0.134579864496243),chrono.ChQuaterniond(0.707106781186546,-0.707106781186549,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_3 = chrono.ChMarker()
marker_0_3.SetName('Coordinate System3')
body_0.AddMarker(marker_0_3)
marker_0_3.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(-0.202459825780885,0.0273699999999997,-0.134579864496244),chrono.ChQuaterniond(0.707106781186546,-0.707106781186549,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_4 = chrono.ChMarker()
marker_0_4.SetName('Coordinate System4')
body_0.AddMarker(marker_0_4)
marker_0_4.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(0.202459825780886,0.0273699999999997,-0.134579864496244),chrono.ChQuaterniond(0.707106781186546,-0.707106781186549,0,0)))
