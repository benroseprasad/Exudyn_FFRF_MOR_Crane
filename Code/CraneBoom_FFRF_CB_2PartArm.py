# -*- coding: utf-8 -*-
"""
Created on Thu Dec 25 13:46:48 2025

@author: benrose
"""

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.FEM import FEMinterface, ObjectFFRFreducedOrderInterface

import numpy as np
import ngsolve as ngs
from netgen.occ import OCCGeometry

# -----------------------------------------------------------------------------
#                                                                INITIALIZATION >>>
# -----------------------------------------------------------------------------

SC = exu.SystemContainer()
mbs = SC.AddSystem()

useGraphics = True

# -----------------------------------------------------------------------------
#                                                          IMPORTING STEP FILES >>>
# -----------------------------------------------------------------------------

#>>>>>>> Main Boom
stepFile = r"D:\pHd\2025\023-LUT Dr. Grzegorz\ExudynProjects\CraneBoom\RealBoom.step"

print("Loading geometry from:", stepFile)
geom = OCCGeometry(stepFile)
ngmesh = geom.GenerateMesh(maxh=100)
ngmesh.Scale(0.001)  # Solidworks mm to exudyn m
mesh = ngs.Mesh(ngmesh)
print("NGSolve mesh: elements =", mesh.ne,", nodes =",mesh.nv)


#>>>>>>> Stick Boom
stepFile2 = r"D:\pHd\2025\023-LUT Dr. Grzegorz\ExudynProjects\CraneBoom\boom2.step"

print("Loading geometry from:", stepFile2)
geom2 = OCCGeometry(stepFile2)
ngmesh2 = geom2.GenerateMesh(maxh=100)
ngmesh2.Scale(0.001)      # mm → m
mesh2 = ngs.Mesh(ngmesh2)
print("NGSolve mesh: elements =", mesh2.ne,", nodes =",mesh2.nv)

# -----------------------------------------------------------------------------
#                                                      LINEAR ELASTIC FEM MODEL >>>
# -----------------------------------------------------------------------------

rho = 7850
E = 2.1e11
nu = 0.3
meshOrder = 1

fem_Main = FEMinterface()
[bfM, bfK, fes] = fem_Main.ImportMeshFromNGsolve(mesh,
                          density=rho,
                          youngsModulus=E,
                          poissonsRatio=nu,
                          meshOrder=meshOrder)
print("Main Boom FEM nodes:", fem_Main.NumberOfNodes())

fem_Stick = FEMinterface()
[fem2M, fem2K, fem2fes] = fem_Stick.ImportMeshFromNGsolve(
    mesh2,
    density=rho,
    youngsModulus=E,
    poissonsRatio=nu,
    meshOrder=meshOrder
)
print("Stick Boom FEM nodes:", fem_Stick.NumberOfNodes())

# -----------------------------------------------------------------------------
#                                                   IDENTIFYING IMPORTANT NODES >>>
# -----------------------------------------------------------------------------


pinRadius = 0.015
tol = 0.003

#>>>>>>> Main Boom Hinge (MBH)
p1_MBH = [0,0,-0.05]
p2_MBH = [0,0,0.05]
nodes_HingeMain2Base = fem_Main.GetNodesOnCylinder(p1_MBH,p2_MBH,
                                                  radius=pinRadius,
                                                  tolerance=tol)
weights_HingeMain2Base = fem_Main.GetNodeWeightsFromSurfaceAreas(nodes_HingeMain2Base)
print("Boundary nodes at root =", len(nodes_HingeMain2Base))


#>>>>>>> Main Boom cylinder attachment to base (MBC2base)
p1_MBC2Base = [0.20,-0.07,-0.05]
p2_MBC2Base = [0.20,-0.07, 0.05]
nodes_CylMain2Base = fem_Main.GetNodesOnCylinder(p1_MBC2Base, p2_MBC2Base,
                                                radius=pinRadius,
                                                tolerance=tol)
weights_CylMain2Base = fem_Main.GetNodeWeightsFromSurfaceAreas(nodes_CylMain2Base)
print("Actuator boom interface nodes =", len(nodes_CylMain2Base))


#>>>>>>> Stick Boom Hinge (SBHM) on Main Boom
p1_SBHM = [2.300, 0.165, -0.05]
p2_SBHM = [2.300, 0.165,  0.05]
nodes_HingeMain2Stick = fem_Main.GetNodesOnCylinder(p1_SBHM, p2_SBHM,
                                  radius=pinRadius,
                                  tolerance=tol)
weights_HingeMain2Stick = fem_Main.GetNodeWeightsFromSurfaceAreas(nodes_HingeMain2Stick)
print("Tip attachment interface nodes =", len(nodes_HingeMain2Stick))

#>>>>>>> Stick Boom Hinge (SBHS) on Stick Boom
p1_SBHS = [0.150, -0.020, -0.05]
p2_SBHS = [0.150, -0.020,  0.05]
nodes_HingeStick2Main = fem_Stick.GetNodesOnCylinder(p1_SBHS, p2_SBHS,
                                       radius=pinRadius,
                                       tolerance=tol)
weightsAttach2 = fem_Stick.GetNodeWeightsFromSurfaceAreas(nodes_HingeStick2Main)
print("Boom-2 joint-hole nodes =", len(nodes_HingeStick2Main))

#>>>>>>> Stick Boom Cylinder attachment to Main Boom (SBCM)
p1_SBCM = [1.580, 0.220, -0.05]
p2_SBCM = [1.580, 0.220,  0.05]
nodes_CylMain2Stick = fem_Main.GetNodesOnCylinder(p1_SBCM, p2_SBCM,
                                          radius=pinRadius,
                                          tolerance=tol)
weights_CylMain2Stick = fem_Main.GetNodeWeightsFromSurfaceAreas(nodes_CylMain2Stick)
print("Boom-1 → Boom-2 actuator interface nodes =", len(nodes_CylMain2Stick))

#>>>>>>> Stick Boom Cylinder attachment to Stick Boom (SBCS)
p1_SBCS = [0.0, 0.0, -0.05]
p2_SBCS = [0.0, 0.0,  0.05]
nodes_CylStick2Main = fem_Stick.GetNodesOnCylinder(p1_SBCS, p2_SBCS,
                                           radius=pinRadius,
                                           tolerance=tol)
weights_CylStick2Main = fem_Stick.GetNodeWeightsFromSurfaceAreas(nodes_CylStick2Main)
print("Boom-2 actuator mount nodes =", len(nodes_CylStick2Main))

#>>>>>>> Stick Boom Tip Load (SBTL)
p1_SBTL = [1.900, -0.035, -0.05]
p2_SBTL = [1.900, -0.035,  0.05]
nodes_StickTipLoad = fem_Stick.GetNodesOnCylinder(p1_SBTL, p2_SBTL,
                                    radius=pinRadius,
                                    tolerance=tol)
weights_StickTipLoad = fem_Stick.GetNodeWeightsFromSurfaceAreas(nodes_StickTipLoad)
print("Boom-2 load-hole nodes =", len(nodes_StickTipLoad))


# -----------------------------------------------------------------------------
#                                                       CRAIG BAMPTON REDUCTION >>>
# -----------------------------------------------------------------------------

from exudyn.FEM import HCBstaticModeSelection
nModes = 12

boundaryList_MainBoom = [nodes_HingeMain2Base]

fem_Main.ComputeHurtyCraigBamptonModes(
    boundaryNodesList = boundaryList_MainBoom,
    nEigenModes = nModes,
    useSparseSolver = True,
    computationMode = HCBstaticModeSelection.RBE2
)
print("Main Boom Eigenmodes Computed")

# Check frequencies and compare to ABAQUS
freqsExu = fem_Main.GetEigenFrequenciesHz()  # list of all eigen frequencies in Hz

print("Exudyn HCB frequencies (Hz):")
for i, f in enumerate(freqsExu[:12]):   # first 12 like in Abaqus
    print(f"Mode {i+1:2d}: {f:8.3f} Hz")

boundaryList_StickBoom = [nodes_HingeStick2Main]

fem_Stick.ComputeHurtyCraigBamptonModes(
    boundaryNodesList=boundaryList_StickBoom,
    nEigenModes=nModes,
    useSparseSolver=True,
    computationMode=HCBstaticModeSelection.RBE2
)
print("Stick Boom Eigenmodes Computed")

# -----------------------------------------------------------------------------
#                                                   PREPARING FEM VISUALIZATION >>>
# -----------------------------------------------------------------------------

from exudyn.FEM import KirchhoffMaterial
import exudyn as exu

mat = KirchhoffMaterial(E, nu, rho)
varType = exu.OutputVariableType.StressLocal

print("ComputePostProcessingModes ...")
fem_Main.ComputePostProcessingModesNGsolve(fes, material=mat, outputVariableType=varType
)
print("   ...done")

# -----------------------------------------------------------------------------
#                                                            FFRF BODY CREATION >>>
# -----------------------------------------------------------------------------

#>>>>>>> Main Boom FFRF Object Creation
cms_Main = ObjectFFRFreducedOrderInterface(fem_Main)

def UFmassFFRFreducedOrder(mbs, t, itemIndex, qR, qR_t):
    return cms_Main.UFmassFFRFreducedOrder(exu, mbs, t, qR, qR_t)

def UFforceFFRFreducedOrder(mbs, t, itemIndex, qR, qR_t):
    return cms_Main.UFforceFFRFreducedOrder(exu, mbs, t, qR, qR_t)

objFFRF_Main = cms_Main.AddObjectFFRFreducedOrder(
    mbs,
    positionRef=[0, 0, 0],           # pin center
    initialVelocity=[0, 0, 0],
    initialAngularVelocity=[0, 0, 0],
    gravity=[0, -9.81, 0],           
    color=[0.8, 0.1, 0.1, 1.0]
)

#>>>>>>> Stick Boom FFRF Object Creation
cms_Stick = ObjectFFRFreducedOrderInterface(fem_Stick)

# Finding location of the Stick Boom joint at t=0 for smooth connection
pTipRef      = np.array([2.300, 0.165, 0.0])
pAttach2Loc  = np.array([0.150, -0.020, 0.0])
posRef2      = pTipRef - pAttach2Loc      # → [2.15, 0.185, 0]
print("Boom-2 reference position:", posRef2)

# Rotation of Stick Boom at time t=0 for smooth connection based on initial cylinder length
phi2 = -13.72 * np.pi/180.0      # ≈ -0.239 rad
R2 = RotXYZ2RotationMatrix([0, 0, phi2])   # from exudyn.utilities import *

# Global position of Stick Boom actuator pin (local origin of boom2)
P2x = 2.15902
P2y = 0.22000

objFFRF_Stick = cms_Stick.AddObjectFFRFreducedOrder(
    mbs,
    positionRef=[P2x, P2y, 0.0],
    rotationMatrixRef=R2,           # <<--- key part
    initialVelocity=[0, 0, 0],
    initialAngularVelocity=[0, 0, 0],
    gravity=[0, -9.81, 0],
    color=[0.2, 0.2, 0.8, 1.0]   # different color to see it
)


# -----------------------------------------------------------------------------
#                        ASSIGNING MARKERS
# -----------------------------------------------------------------------------

#>>>>>>>>>>>> Ground Main Boom Joint
oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0]))
mGround = mbs.AddMarker(
    MarkerBodyRigid(
        bodyNumber=oGround,
        localPosition=[0, 0, 0]
    )
)

#>>>>>>>>>>>> Ground Main Cylinder Joint
mGroundCyl = mbs.AddMarker(
    MarkerBodyRigid(
        bodyNumber=oGround,
        localPosition=[0.10, -0.65, 0.0]
    )
)

#>>>>>>>>>>>> Main Boom Hinge Marker
mHingeMain2Base = mbs.AddMarker(
    MarkerSuperElementRigid(
        bodyNumber=objFFRF_Main['oFFRFreducedOrder'],
        meshNodeNumbers=np.array(nodes_HingeMain2Base),
        weightingFactors=weights_HingeMain2Base
    )
)

#>>>>>>>>>>>> Main Boom Cylinder attachment pin Marker
mCylMain2Base = mbs.AddMarker(
    MarkerSuperElementRigid(
        bodyNumber=objFFRF_Main['oFFRFreducedOrder'],
        meshNodeNumbers=np.array(nodes_CylMain2Base),
        weightingFactors=weights_CylMain2Base
    )
)

#>>>>>>>>>>>> Main Boom Hinge to Stick Boom
mHingeMain2Stick = mbs.AddMarker(
    MarkerSuperElementRigid(
        bodyNumber=objFFRF_Main['oFFRFreducedOrder'],
        meshNodeNumbers=np.array(nodes_HingeMain2Stick),
        weightingFactors=weights_HingeMain2Stick
    )
)

#>>>>>>>>>>>> Stick Boom Hinge to Main Boom
mHingeStick2Main = mbs.AddMarker(
    MarkerSuperElementRigid(
        bodyNumber=objFFRF_Stick['oFFRFreducedOrder'],
        meshNodeNumbers=np.array(nodes_HingeStick2Main),
        weightingFactors=weightsAttach2
    )
)

#>>>>>>>>>>>> Main Boom Cylinder attachment pin to Stick Boom
mCylMain2Stick = mbs.AddMarker(
    MarkerSuperElementRigid(
        bodyNumber=objFFRF_Main['oFFRFreducedOrder'],
        meshNodeNumbers=np.array(nodes_CylMain2Stick),
        weightingFactors=weights_CylMain2Stick
    )
)

#>>>>>>>>>>>> Stick Boom Cylinder attachment pin to Main Boom
mCylStick2Main = mbs.AddMarker(
    MarkerSuperElementRigid(
        bodyNumber=objFFRF_Stick['oFFRFreducedOrder'],
        meshNodeNumbers=np.array(nodes_CylStick2Main),
        weightingFactors=weights_CylStick2Main
    )
)

#>>>>>>>>>>>> Stick Boom Cylinder Load point marker
mStickTipLoad = mbs.AddMarker(
    MarkerSuperElementRigid(
        bodyNumber=objFFRF_Stick['oFFRFreducedOrder'],
        meshNodeNumbers=np.array(nodes_StickTipLoad),
        weightingFactors=weights_StickTipLoad
    )
)


# -----------------------------------------------------------------------------
#                                                               CREATING JOINTS >>>
# -----------------------------------------------------------------------------

#>>>>>>>>>>>> Main Boom Hinge
mbs.AddObject(
    GenericJoint(
        markerNumbers=[mGround, mHingeMain2Base],
        constrainedAxes=[1, 1, 1, 1, 1, 0],
        visualization=VGenericJoint(axesRadius=0.1*pinRadius, axesLength=0.1*pinRadius)
    )
)


#>>>>>>>>>>>> Main Cylinder
pBase = np.array([0.10, -0.65, 0.0])
pBoom  = np.array([0.20, -0.07, 0.0])

axisVec = pBoom - pBase
axisVec = axisVec / np.linalg.norm(axisVec)  # normalize

# distance actuator object
oAct = mbs.AddObject(
    ObjectConnectorDistance(
        markerNumbers=[mGroundCyl, mCylMain2Base],
        distance=0.589,      # initial length L0 [m]
        activeConnector=True
    )
)

# --- prescribe actuator length over time ---
L0  = 0.589
L60 = 0.791
T   = 5.0

def UFprescribeActuatorLength(mbs, t):
    if t < T:
        s = t/T
        s = 3*s**2 - 2*s**3   # smooth ramp 0→1
        L = L0 + s*(L60 - L0)
    else:
        L = L60

    mbs.SetObjectParameter(oAct, 'distance', L)
    return True

# register callback
mbs.SetPreStepUserFunction(UFprescribeActuatorLength)

mbs.AddObject(
    GenericJoint(
        markerNumbers=[mHingeMain2Stick, mHingeStick2Main],
        constrainedAxes=[1,1,1,1,1,0],  # Only rotation about Z allowed
        visualization=VGenericJoint(
            axesRadius=0.1*pinRadius,
            axesLength=0.1*pinRadius
        )
    )
)


#>>>>>>>>>>>> Stick Cylinder
Lact = 0.57902   # constant actuator length (m)

oAct12 = mbs.AddObject(
    ObjectConnectorDistance(
        markerNumbers=[mCylMain2Stick, mCylStick2Main],
        distance=Lact,
        activeConnector=True
    )
)

# -----------------------------------------------------------------------------
#                                                                CREATING LOADS >>>
# -----------------------------------------------------------------------------

massLoad = 200.0
g = 9.81
Fy = -massLoad * g   # downward

mbs.AddLoad(
    Force(
        markerNumber=mStickTipLoad,
        loadVector=[0, Fy, 0],
        bodyFixed=False
    )
)

# -----------------------------------------------------------------------------
#                                                              CREATING SENSORS >>>
# -----------------------------------------------------------------------------

sensTip1 = mbs.AddSensor(
    SensorMarker(
        markerNumber=mHingeMain2Stick,
        storeInternal=True,   # <-- REQUIRED for trace
        fileName='TipPosition.txt',
        outputVariableType=exu.OutputVariableType.Position
    )
)

sensTip2 = mbs.AddSensor(
    SensorMarker(
        markerNumber=mStickTipLoad,
        storeInternal=True,
        fileName='Tip2Position.txt',
        outputVariableType=exu.OutputVariableType.Position
    )
)

# -----------------------------------------------------------------------------
#                                                             CREATING ASSEMBLY >>>
# -----------------------------------------------------------------------------

mbs.Assemble()

# -----------------------------------------------------------------------------
#                                                           SIMULATION SETTINGS >>>
# -----------------------------------------------------------------------------

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 6000
simulationSettings.timeIntegration.endTime       = 6.0   # 1 second
simulationSettings.timeIntegration.verboseMode   = 1
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.solutionSettings.writeSolutionToFile = True

# -----------------------------------------------------------------------------
#                                                        VISUALIZATION SETTINGS >>>
# -----------------------------------------------------------------------------

# some basic visualization tuning
SC.visualizationSettings.bodies.deformationScaleFactor = 1.0
SC.visualizationSettings.openGL.showFaceEdges = True
SC.visualizationSettings.openGL.showFaces     = True
SC.visualizationSettings.general.drawWorldBasis = True
SC.visualizationSettings.contour.reduceRange = False
SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.StressLocal
SC.visualizationSettings.contour.outputVariableComponent = 0   # σxx
SC.visualizationSettings.bodies.deformationScaleFactor = 1.0
SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.markers.show = False
SC.visualizationSettings.loads.drawSimplified = True
SC.visualizationSettings.sensors.show = True
SC.visualizationSettings.sensors.traces.showPositionTrace = True
SC.visualizationSettings.sensors.traces.showPast = True
SC.visualizationSettings.sensors.traces.showFuture = False
SC.visualizationSettings.sensors.traces.lineWidth = 2
SC.visualizationSettings.sensors.traces.listOfPositionSensors = [sensTip1, sensTip2]

# -----------------------------------------------------------------------------
#                                                               MAIN SIMULATION >>>
# -----------------------------------------------------------------------------

print("Starting simulation ...")

if useGraphics:
    SC.renderer.Start()
    mbs.SolveDynamic(simulationSettings=simulationSettings)
    SC.renderer.Stop()
else:
    mbs.SolveDynamic(simulationSettings=simulationSettings)

print("Done.")

mbs.SolutionViewer()