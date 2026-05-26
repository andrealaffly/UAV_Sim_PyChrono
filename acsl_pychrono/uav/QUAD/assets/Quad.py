# PyChrono model automatically generated using Chrono::SolidWorks add-in
# Assembly: D:\Virginia-Tech-PhD\PHD_research\PyChrono\UAV_CAD_Models\DuyModel\FinalAssembly.SLDASM


import pychrono as chrono 
import builtins 

# Some global settings 
sphereswept_r = 0.001
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.003)
chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.002)

shapes_dir = 'Quad_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0 = chrono.ChBodyAuxRef()
body_0.SetName('SLDW_GROUND')
body_0.SetFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1 = chrono.ChBodyAuxRef()
body_1.SetName('Frame-1')
body_1.SetPos(chrono.ChVector3d(0,0,0))
body_1.SetRot(chrono.ChQuaterniond(1,0,0,0))
body_1.SetMass(1.04291239130921)
body_1.SetInertiaXX(chrono.ChVector3d(0.0035660210440486,0.00606507369923718,0.00408753970286779))
body_1.SetInertiaXY(chrono.ChVector3d(1.15021059236136e-05,-4.08770048541267e-06,8.64537272994129e-07))
body_1.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-0.00149597132207646,-0.00406068459144218,-2.47411685736527e-05),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChVisualShapeModelFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_shape.SetColor(chrono.ChColor(0.1, 0.1, 0.1))
body_1.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

# Collision Model

body_1.AddCollisionModel(chrono.ChCollisionModel())

# Collision material 
mat_1 = chrono.ChContactMaterialNSC()
mr = chrono.ChMatrix33d()
mr[0,0]=-1; mr[1,0]=3.37857869714127E-15; mr[2,0]=-2.41327049795805E-16 
mr[0,1]=0; mr[1,1]=1.67136794525062E-15; mr[2,1]=-1 
mr[0,2]=-3.37857869714127E-15; mr[1,2]=-1; mr[2,2]=-1.67136794525062E-15 
collshape = chrono.ChCollisionShapeBox(mat_1,0.230024571544001,0.265704036691617,0.116)
body_1.GetCollisionModel().AddShape(collshape,chrono.ChFramed(chrono.ChVector3d(-0.00170591778503511,-0.000999999999999585,2.18203829267814E-06), mr))
body_1.EnableCollision(True)

exported_items.append(body_1)



# Rigid body part
body_2 = chrono.ChBodyAuxRef()
body_2.SetName('Propeller-2')
body_2.SetPos(chrono.ChVector3d(0.0969022647597098,0.0243399999999998,-0.11071519630803))
body_2.SetRot(chrono.ChQuaterniond(0.707106781186548,-0.707106781186547,2.46885013108226e-15,0))
# body_2.SetMass(0.00364120675790278)
# body_2.SetInertiaXX(chrono.ChVector3d(1.42060313992044e-06,2.81765034379699e-06,1.4208475579394e-06))
# body_2.SetInertiaXY(chrono.ChVector3d(-1.1934266053721e-12,-2.69794196175028e-12,1.38379431338018e-13))
body_2.SetMass(1e-12) # Modified by Xavier, the original is in the lines above
body_2.SetInertiaXX(chrono.ChVector3d(1e-12,1e-12,1e-12)) # Modified by Xavier, the original is in the lines above
body_2.SetInertiaXY(chrono.ChVector3d(0,0,0)) # Modified by Xavier, the original is in the lines above
body_2.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-4.58230968281564e-08,-2.31695875227173e-07,-1.27309700361918e-08),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChVisualShapeModelFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_shape.SetColor(chrono.ChColor(1, 0, 0))
body_2.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_2)



# Rigid body part
body_3 = chrono.ChBodyAuxRef()
body_3.SetName('Propeller-4')
body_3.SetPos(chrono.ChVector3d(-0.0969022647597099,0.0243400000000004,0.11071519630803))
body_3.SetRot(chrono.ChQuaterniond(0.707106781186548,-0.707106781186547,2.46885013108227e-15,0))
# body_3.SetMass(0.00364120675790278)
# body_3.SetInertiaXX(chrono.ChVector3d(1.42060313992044e-06,2.81765034379699e-06,1.4208475579394e-06))
# body_3.SetInertiaXY(chrono.ChVector3d(-1.1934266053721e-12,-2.69794196175028e-12,1.38379431338018e-13))
body_3.SetMass(1e-12) # Modified by Xavier, the original is in the lines above
body_3.SetInertiaXX(chrono.ChVector3d(1e-12,1e-12,1e-12)) # Modified by Xavier, the original is in the lines above
body_3.SetInertiaXY(chrono.ChVector3d(0,0,0)) # Modified by Xavier, the original is in the lines above
body_3.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-4.58230968281564e-08,-2.31695875227173e-07,-1.27309700361918e-08),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
# body_2_1_shape = chrono.ChVisualShapeModelFile() 
# body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_3.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_3)



# Rigid body part
body_4 = chrono.ChBodyAuxRef()
body_4.SetName('Propeller-3')
body_4.SetPos(chrono.ChVector3d(0.0969022647597116,0.0243399999999996,0.110715196308029))
body_4.SetRot(chrono.ChQuaterniond(0.707106781186548,-0.707106781186547,2.46885013108226e-15,0))
# body_4.SetMass(0.00364120675790278)
# body_4.SetInertiaXX(chrono.ChVector3d(1.42060313992044e-06,2.81765034379699e-06,1.4208475579394e-06))
# body_4.SetInertiaXY(chrono.ChVector3d(-1.1934266053721e-12,-2.69794196175028e-12,1.38379431338018e-13))
body_4.SetMass(1e-12) # Modified by Xavier, the original is in the lines above
body_4.SetInertiaXX(chrono.ChVector3d(1e-12,1e-12,1e-12)) # Modified by Xavier, the original is in the lines above
body_4.SetInertiaXY(chrono.ChVector3d(0,0,0)) # Modified by Xavier, the original is in the lines above
body_4.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-4.58230968281564e-08,-2.31695875227173e-07,-1.27309700361918e-08),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
# body_2_1_shape = chrono.ChVisualShapeModelFile() 
# body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_4.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_4)



# Rigid body part
body_5 = chrono.ChBodyAuxRef()
body_5.SetName('Propeller-1')
body_5.SetPos(chrono.ChVector3d(-0.0969022647597108,0.0243400000000004,-0.110715196308028))
body_5.SetRot(chrono.ChQuaterniond(0.707106781186548,-0.707106781186547,2.46885013108226e-15,0))
# body_5.SetMass(0.00364120675790278)
# body_5.SetInertiaXX(chrono.ChVector3d(1.42060313992044e-06,2.81765034379699e-06,1.4208475579394e-06))
# body_5.SetInertiaXY(chrono.ChVector3d(-1.1934266053721e-12,-2.69794196175028e-12,1.38379431338018e-13))
body_5.SetMass(1e-12) # Modified by Xavier, the original is in the lines above
body_5.SetInertiaXX(chrono.ChVector3d(1e-12,1e-12,1e-12)) # Modified by Xavier, the original is in the lines above
body_5.SetInertiaXY(chrono.ChVector3d(0,0,0)) # Modified by Xavier, the original is in the lines above
body_5.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-4.58230968281564e-08,-2.31695875227173e-07,-1.27309700361918e-08),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
# body_2_1_shape = chrono.ChVisualShapeModelFile() 
# body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_5.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_5)




# Mate constraint: Coincident1 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: Propeller-1 ,  SW ref.type:2 (2)
link_1 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.0964671178570591,0.0202000000000004,-0.11317703447759)
cB = chrono.ChVector3d(-0.0969022647597109,0.0202000000000004,-0.110715196308028)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-3.2000586021278e-31)
dB = chrono.ChVector3d(-3.49148133884313e-15,-1,3.2000586021278e-31)
link_1.Initialize(body_1,body_5,False,cA,cB,dB)
link_1.SetDistance(0)
link_1.SetName("Coincident1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.0964671178570591,0.0202000000000004,-0.11317703447759)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-3.2000586021278e-31)
cB = chrono.ChVector3d(-0.0969022647597109,0.0202000000000004,-0.110715196308028)
dB = chrono.ChVector3d(-3.49148133884313e-15,-1,3.2000586021278e-31)
link_2.SetFlipped(True)
link_2.Initialize(body_1,body_5,False,cA,cB,dA,dB)
link_2.SetName("Coincident1")
exported_items.append(link_2)


# Mate constraint: Concentric1 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_1 , SW name: Frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: Propeller-1 ,  SW ref.type:2 (2)
link_3 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.0969022647597108,0.0327000000000004,-0.110715196308028)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-3.2000586021278e-31)
cB = chrono.ChVector3d(-0.0969022647597109,0.0201552095811017,-0.110715196308028)
dB = chrono.ChVector3d(3.49148133884313e-15,1,-3.2000586021278e-31)
link_3.Initialize(body_1,body_5,False,cA,cB,dA,dB)
link_3.SetName("Concentric1")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateGeneric()
link_4.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.0969022647597108,0.0327000000000004,-0.110715196308028)
cB = chrono.ChVector3d(-0.0969022647597109,0.0201552095811017,-0.110715196308028)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-3.2000586021278e-31)
dB = chrono.ChVector3d(3.49148133884313e-15,1,-3.2000586021278e-31)
link_4.Initialize(body_1,body_5,False,cA,cB,dA,dB)
link_4.SetName("Concentric1")
exported_items.append(link_4)


# Mate constraint: Coincident2 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Propeller-2 ,  SW ref.type:2 (2)
link_5 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(0.0993641029292718,0.0201999999999998,-0.111150343210682)
cB = chrono.ChVector3d(0.0969022647597098,0.0201999999999998,-0.11071519630803)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.66412315968731e-31)
dB = chrono.ChVector3d(-3.49148133884313e-15,-1,4.66412315968731e-31)
link_5.Initialize(body_1,body_2,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident2")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(0.0993641029292718,0.0201999999999998,-0.111150343210682)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.66412315968731e-31)
cB = chrono.ChVector3d(0.0969022647597098,0.0201999999999998,-0.11071519630803)
dB = chrono.ChVector3d(-3.49148133884313e-15,-1,4.66412315968731e-31)
link_6.SetFlipped(True)
link_6.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_6.SetName("Coincident2")
exported_items.append(link_6)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_1 , SW name: Frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Propeller-2 ,  SW ref.type:2 (2)
link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(0.0969022647597099,0.0326999999999998,-0.11071519630803)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.66412315968731e-31)
cB = chrono.ChVector3d(0.0969022647597098,0.020155209581101,-0.11071519630803)
dB = chrono.ChVector3d(3.49148133884313e-15,1,-4.66412315968731e-31)
link_7.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_7.SetName("Concentric2")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateGeneric()
link_8.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(0.0969022647597099,0.0326999999999998,-0.11071519630803)
cB = chrono.ChVector3d(0.0969022647597098,0.020155209581101,-0.11071519630803)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.66412315968731e-31)
dB = chrono.ChVector3d(3.49148133884313e-15,1,-4.66412315968731e-31)
link_8.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_8.SetName("Concentric2")
exported_items.append(link_8)


# Mate constraint: Coincident3 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Propeller-3 ,  SW ref.type:2 (2)
link_9 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(0.0964671178570597,0.0201999999999996,0.113177034477591)
cB = chrono.ChVector3d(0.0969022647597115,0.0201999999999996,0.110715196308029)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.68855045008241e-31)
dB = chrono.ChVector3d(-3.49148133884313e-15,-1,4.68855045008241e-31)
link_9.Initialize(body_1,body_4,False,cA,cB,dB)
link_9.SetDistance(0)
link_9.SetName("Coincident3")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(0.0964671178570597,0.0201999999999996,0.113177034477591)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.68855045008241e-31)
cB = chrono.ChVector3d(0.0969022647597115,0.0201999999999996,0.110715196308029)
dB = chrono.ChVector3d(-3.49148133884313e-15,-1,4.68855045008241e-31)
link_10.SetFlipped(True)
link_10.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_10.SetName("Coincident3")
exported_items.append(link_10)


# Mate constraint: Concentric3 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_1 , SW name: Frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Propeller-3 ,  SW ref.type:2 (2)
link_11 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(0.0969022647597116,0.0326999999999996,0.110715196308029)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.68855045008241e-31)
cB = chrono.ChVector3d(0.0969022647597115,0.0201552095811008,0.110715196308029)
dB = chrono.ChVector3d(3.49148133884313e-15,1,-4.68855045008241e-31)
link_11.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_11.SetName("Concentric3")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateGeneric()
link_12.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(0.0969022647597116,0.0326999999999996,0.110715196308029)
cB = chrono.ChVector3d(0.0969022647597115,0.0201552095811008,0.110715196308029)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.68855045008241e-31)
dB = chrono.ChVector3d(3.49148133884313e-15,1,-4.68855045008241e-31)
link_12.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_12.SetName("Concentric3")
exported_items.append(link_12)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Propeller-4 ,  SW ref.type:2 (2)
link_13 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.0993641029292719,0.0202000000000005,0.111150343210682)
cB = chrono.ChVector3d(-0.0969022647597099,0.0202000000000004,0.11071519630803)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.21056202404906e-31)
dB = chrono.ChVector3d(-3.49148133884313e-15,-1,4.21056202404906e-31)
link_13.Initialize(body_1,body_3,False,cA,cB,dB)
link_13.SetDistance(0)
link_13.SetName("Coincident4")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.0993641029292719,0.0202000000000005,0.111150343210682)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.21056202404906e-31)
cB = chrono.ChVector3d(-0.0969022647597099,0.0202000000000004,0.11071519630803)
dB = chrono.ChVector3d(-3.49148133884313e-15,-1,4.21056202404906e-31)
link_14.SetFlipped(True)
link_14.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_14.SetName("Coincident4")
exported_items.append(link_14)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_1 , SW name: Frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Propeller-4 ,  SW ref.type:2 (2)
link_15 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.0969022647597099,0.0327000000000004,0.11071519630803)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.21056202404906e-31)
cB = chrono.ChVector3d(-0.0969022647597099,0.0201552095811017,0.11071519630803)
dB = chrono.ChVector3d(3.49148133884313e-15,1,-4.21056202404906e-31)
link_15.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_15.SetName("Concentric4")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateGeneric()
link_16.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.0969022647597099,0.0327000000000004,0.11071519630803)
cB = chrono.ChVector3d(-0.0969022647597099,0.0201552095811017,0.11071519630803)
dA = chrono.ChVector3d(3.49148133884313e-15,1,-4.21056202404906e-31)
dB = chrono.ChVector3d(3.49148133884313e-15,1,-4.21056202404906e-31)
link_16.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_16.SetName("Concentric4")
exported_items.append(link_16)


# Auxiliary marker (coordinate system feature)
marker_0_1 = chrono.ChMarker()
marker_0_1.SetName('Coordinate System1')
body_0.AddMarker(marker_0_1)
marker_0_1.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(-0.0969022647597109,0.0202000000000005,-0.110715196308028),chrono.ChQuaterniond(0.707106781186546,-0.707106781186549,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_2 = chrono.ChMarker()
marker_0_2.SetName('Coordinate System2')
body_0.AddMarker(marker_0_2)
marker_0_2.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(0.0969022647597098,0.0201999999999998,-0.11071519630803),chrono.ChQuaterniond(0.707106781186546,-0.707106781186549,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_3 = chrono.ChMarker()
marker_0_3.SetName('Coordinate System3')
body_0.AddMarker(marker_0_3)
marker_0_3.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(0.0969022647597115,0.0201999999999998,0.110715196308029),chrono.ChQuaterniond(0.707106781186546,-0.707106781186549,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_4 = chrono.ChMarker()
marker_0_4.SetName('Coordinate System4')
body_0.AddMarker(marker_0_4)
marker_0_4.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(-0.0969022647597099,0.0202000000000005,0.11071519630803),chrono.ChQuaterniond(0.707106781186546,-0.707106781186549,0,0)))
