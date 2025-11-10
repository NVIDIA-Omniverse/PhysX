# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import typing
import carb
import omni.physx.scripts.utils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physx import get_physx_simulation_interface, get_physx_replicator_interface
from omni.physxtests.testBases.articulationTestBase import ArticulationTestBase
from omni.physxtests import utils
from pxr import Usd, Gf, Sdf, UsdGeom, UsdPhysics, UsdUtils, PhysxSchema, Tf, UsdShade
import math
from omni.physx.bindings._physx import (
    PERF_ENV_API,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ANGULAR,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_LINEAR,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ROTX,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ROTY,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ROTZ,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ANGULAR,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_LINEAR,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ROTX,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ROTY,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ROTZ,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ANGULAR,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_LINEAR,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ROTX,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ROTY,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ROTZ,
)


#joint types supported by test
JOINT_TYPE_PRISMATIC_X = 0
JOINT_TYPE_PRISMATIC_Y = 1
JOINT_TYPE_PRISMATIC_Z = 2
JOINT_TYPE_REVOLUTE_X = 3
JOINT_TYPE_REVOLUTE_Y = 4
JOINT_TYPE_REVOLUTE_Z = 5
JOINT_TYPE_SPHERICAL_ROTX = 6
JOINT_TYPE_SPHERICAL_ROTY = 7
JOINT_TYPE_SPHERICAL_ROTZ = 8
SUPPORTED_JOINT_TYPES = [JOINT_TYPE_PRISMATIC_X, JOINT_TYPE_PRISMATIC_Y, JOINT_TYPE_PRISMATIC_Z, JOINT_TYPE_REVOLUTE_X, JOINT_TYPE_REVOLUTE_Y, JOINT_TYPE_REVOLUTE_Z, JOINT_TYPE_SPHERICAL_ROTX, JOINT_TYPE_SPHERICAL_ROTY, JOINT_TYPE_SPHERICAL_ROTZ]
RAD_TO_DEG = [1.0, 1.0, 1.0, 57.2958, 57.2958, 57.2958, 57.2958, 57.2958, 57.2958, ]
SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP = [UsdPhysics.Tokens.linear, UsdPhysics.Tokens.linear, UsdPhysics.Tokens.linear, UsdPhysics.Tokens.angular, UsdPhysics.Tokens.angular, UsdPhysics.Tokens.angular, UsdPhysics.Tokens.rotX, UsdPhysics.Tokens.rotY, UsdPhysics.Tokens.rotZ]
MAX_ACTUATOR_VELOCITY_TO_TOKEN_MAP = [
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_LINEAR,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_LINEAR,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_LINEAR,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ANGULAR,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ANGULAR,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ANGULAR,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ROTX,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ROTY,
    PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ROTZ,
]

VELOCITY_DEPENDENT_RESISTANCE_TO_TOKEN_MAP = [
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_LINEAR,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_LINEAR,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_LINEAR,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ANGULAR,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ANGULAR,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ANGULAR,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ROTX,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ROTY,
    PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ROTZ,
]

SPEED_EFFORT_GRADIENT_TO_TOKEN_MAP = [
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_LINEAR,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_LINEAR,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_LINEAR,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ANGULAR,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ANGULAR,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ANGULAR,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ROTX,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ROTY,
    PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ROTZ,
]


def _isSupportedJointType(jointType: int) -> bool:
    isSupported = False
    for supportedJointType in SUPPORTED_JOINT_TYPES:
        if jointType == supportedJointType:
            isSupported = True
    return isSupported

#attributes
PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY = 0
PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE = 1
PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT = 2

class PhysxPerfEnvAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def new_stage(self):
        self._clean_up()
        await super().new_stage(attach_stage=False)

        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self._stage, 1.0)

        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.y)

        return self._stage

    def _applyPerfEnvAPI(self, jointPrim: Usd.Prim, jointType: int) -> bool:

        if not _isSupportedJointType(jointType):
            print("_applyPerfEnvAPI called with illegal jointType")
            return false

        instanceToken = SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType]

        jointPrim.ApplyAPI(PERF_ENV_API, instanceToken)

        return True

    def _removePerfEnvAPI(self, jointPrim: Usd.Prim, jointType: int) -> bool:

        if not _isSupportedJointType(jointType):
            print("_removePerfEnvAPI called with illegal jointType")
            return false

        instanceToken = SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType]

        jointPrim.RemoveAPI(PERF_ENV_API, instanceToken)

        return True

    def _hasAppliedPerfAPI(self, jointPrim: Usd.Prim, jointType: int) -> bool:
    
        if not _isSupportedJointType(jointType):
            print("_hasAppliedPerfAPI called with illegal jointType")
            return false
    
        instanceToken = SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType]

        hasAPI = jointPrim.GetPrim().HasAPI(PERF_ENV_API, instanceToken)

        return hasAPI
    
    def _setPerfEnvAttribute(self, jointPrim: Usd.Prim, jointType: int, attrId: int, attrVal: float) -> bool:
    
        if not _isSupportedJointType(jointType):
            print("_setPerfEnvAttribute called with illegal jointType")
            return false
    
        if attrId != PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY and attrId != PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE and attrId != PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT:
            print("_setPerfEnvAttribute: must be maxActuatorVelocity or velocityDependentResistance or speedEffortGradient")
            return False
    
        attrName =  MAX_ACTUATOR_VELOCITY_TO_TOKEN_MAP[jointType]
        if  attrId == PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE:
            attrName =  VELOCITY_DEPENDENT_RESISTANCE_TO_TOKEN_MAP[jointType]
        elif attrId == PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT:
            attrName =  SPEED_EFFORT_GRADIENT_TO_TOKEN_MAP[jointType]
    
        jointPrim.GetAttribute(attrName).Set(attrVal)

        return True
    
    def _getPerfEnvAttribute(self, jointPrim: Usd.Prim, jointType: int, attrId: int) -> tuple[bool, float]:
    
        if not _isSupportedJointType(jointType):
            print("_getPerfEnvAttribute called with illegal jointType")
            return false
    
        if attrId != PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY and attrId != PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE and attrId != PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT:
            print("_getPerfEnvAttribute: must be maxActuatorVelocity or velocityDependentResistance or speedEffortGradient")
            return False
    
        instanceToken = SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType]
    
        attrName =  ":maxActuatorVelocity"
        if attrId == PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE:
            attrName = ":velocityDependentResistance"
        elif attrId == PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT:
            attrName = ":speedEffortGradient"
    
        attrVal = jointPrim.GetAttribute("physxDrivePerformanceEnvelope:" + instanceToken + attrName).Get()

        return (True, attrVal)

    def _scalePositionToDegreesAsNecessary(self, jointType: int, f: float) -> float:
        return f*RAD_TO_DEG[jointType]
    
    def _scaleVelocityToDegreesAsNecessary(self, jointType: int, f: float) -> float:
        return f*RAD_TO_DEG[jointType]

    def _scaleStiffnessToDegreesAsNecessary(self, jointType: int, f: float) -> float:
        return f/RAD_TO_DEG[jointType]

    def _scaleDampingToDegreesAsNecessary(self, jointType: int, f: float) -> float:
        return f/RAD_TO_DEG[jointType];

    def _scaleVelocityDependentResistanceToDegreesAsNecessary(self, jointType: int, f: float) -> float:
        return f/RAD_TO_DEG[jointType]

    def _scaleSpeedEffortGradientToDegreesAsNecessary(self, jointType: int, f: float) -> float:
        return f*RAD_TO_DEG[jointType]

    def _scaleVelocityToRadiansAsNecessary(self, jointType: int, f: float) -> float:
        return f/RAD_TO_DEG[jointType]

    def _get_time_step(self):
        return 1.0 / 100.0

    def _prepare_for_simulation(self):
        stageId = self.attach_stage()

    def _clean_up(self):
        if (self._stage_attached):
            self.detach_stage()

    def _simulate_one_frame(self, elapsedTime=None):
        if (elapsedTime is None):
            timeStep = self._get_time_step()
        else:
            timeStep = elapsedTime

        self.step(1, timeStep)

    def _simulate_one_frame_with_prep(self):
        self._prepare_for_simulation()
        self._simulate_one_frame()

    def _simulate(self, secondsToRun):
        timeStep = self._get_time_step()
        targetIterationCount = math.ceil(secondsToRun / timeStep)
        currentTime = 0.0

        physXSimInterface = get_physx_simulation_interface()

        for i in range(targetIterationCount):
            physXSimInterface.simulate(timeStep, currentTime)
            physXSimInterface.fetch_results()
            currentTime += timeStep

    def _simulate_with_prep(self, secondsToRun):
        self._prepare_for_simulation()
        self._simulate(secondsToRun)

    def _create_scene(self, stage: Usd.Stage, path: str, gravityMagn: float = 9.81) -> UsdPhysics.Scene:
        sceneAPI = UsdPhysics.Scene.Define(stage, path)
        sceneAPI.GetGravityMagnitudeAttr().Set(gravityMagn)

        return sceneAPI

    def _create_scene_with_gravity(self, stage: Usd.Stage, gravityMagn: float = 9.81) -> UsdPhysics.Scene:
        scenePath = str(stage.GetDefaultPrim().GetPath()) + "/Scene"
        return self._create_scene(stage, scenePath, gravityMagn)

    def _create_zero_gravity_scene(self, stage: Usd.Stage) -> UsdPhysics.Scene:
        return self._create_scene_with_gravity(stage, 0.0)

    #######################################################################
    ## create articulation 
    #######################################################################

    def _create_body(
        self,
        stage: Usd.Stage,
        bodyPath: str,
        position: Gf.Vec3f = Gf.Vec3f(0.0, 0.0, 0.0),
        orientation: Gf.Quatf = Gf.Quatf(1.0, 0.0, 0.0, 0.0),
        mass: float = 1.0,
        inertia: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0),
        boxSize: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0),
        ) -> Usd.Prim:
    
        bodyXform = UsdGeom.Xform.Define(stage, bodyPath)
        physicsUtils._add_transformation(bodyXform, position, orientation)
    
        bodyPrim = bodyXform.GetPrim()
    
        lin_velocity = Gf.Vec3f(0.0, 0.0, 0.0)
        ang_velocity = Gf.Vec3f(0.0, 0.0, 0.0)
        bodyAPI = UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        bodyAPI.CreateVelocityAttr().Set(lin_velocity)
        bodyAPI.CreateAngularVelocityAttr().Set(ang_velocity)
    
        physxBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(bodyPrim)
        physxBodyAPI.GetMaxAngularVelocityAttr().Set(1.0e10)
        physxBodyAPI.GetLinearDampingAttr().Set(0.0)
        physxBodyAPI.GetAngularDampingAttr().Set(0.0)
        physxBodyAPI.GetSleepThresholdAttr().Set(0.0)
        physxBodyAPI.GetStabilizationThresholdAttr().Set(0.0)
        physxBodyAPI.GetCfmScaleAttr().Set(0.0)
    
        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
        massAPI.GetMassAttr().Set(mass)
        massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(mass))
    
        physicsUtils.add_box(stage, bodyPath + "/Geom", boxSize)
    
        return bodyPrim

    def _create_fixed_base_articulation(self,
        stage: Usd.Stage,
        path: str,
        position: Gf.Vec3f = Gf.Vec3f(0.0, 0.0, 0.0),
        orientation: Gf.Quatf = Gf.Quatf(1.0, 0.0, 0.0, 0.0),
        boxSize: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0),
        nbPosIters: int = 4, nbVelIters: int = 1,
        ) -> tuple[Usd.Prim, Usd.Prim]:

        # Mass and inertia of root link can be anything because we are making a fixed base articulation.
        rootLink = self._create_body(stage, path, position, orientation, 1.0, Gf.Vec3f(1.0, 1.0, 1.0), boxSize)

        # Fixed base articulation needs a joint to the world.
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, path + "/InboundJoint")
        fixedJoint.GetBody1Rel().AddTarget(path)
        artPrim = fixedJoint.GetPrim()

        # Fixed base articulation has the articulation API applied to the joint to the world.
        UsdPhysics.ArticulationRootAPI.Apply(artPrim)
        physxArtAPI = PhysxSchema.PhysxArticulationAPI.Apply(artPrim)

        physxArtAPI.GetSolverPositionIterationCountAttr().Set(nbPosIters)
        physxArtAPI.GetSolverVelocityIterationCountAttr().Set(nbVelIters)
        physxArtAPI.GetSleepThresholdAttr().Set(0.0)
        physxArtAPI.GetStabilizationThresholdAttr().Set(0.0)
        physxArtAPI.GetEnabledSelfCollisionsAttr().Set(False)

        return (artPrim, rootLink)


    def _create_joint(self, stage: Usd.Stage, path: str,
        jointType: int = JOINT_TYPE_PRISMATIC_X,
        body0Path: str = "", body1Path: str = "",
        localPos0: Gf.Vec3f = Gf.Vec3f(0.0), localRot0: Gf.Quatf = Gf.Quatf(1.0, 0.0, 0.0, 0.0),
        localPos1: Gf.Vec3f = Gf.Vec3f(0.0), localRot1: Gf.Quatf = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        ) -> typing.Union[UsdPhysics.PrismaticJoint, UsdPhysics.RevoluteJoint, UsdPhysics.Joint, None]:

        if jointType == JOINT_TYPE_PRISMATIC_X or jointType == JOINT_TYPE_PRISMATIC_Y or jointType == JOINT_TYPE_PRISMATIC_Z:
            joint = UsdPhysics.PrismaticJoint.Define(stage, path)
            joint.GetAxisAttr().Set(SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType])
        elif jointType == JOINT_TYPE_REVOLUTE_X or jointType == JOINT_TYPE_REVOLUTE_Y or jointType == JOINT_TYPE_REVOLUTE_Z:
            joint = UsdPhysics.RevoluteJoint.Define(stage, path)
            joint.GetAxisAttr().Set(SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType])
        elif jointType == JOINT_TYPE_SPHERICAL_ROTX or jointType == JOINT_TYPE_SPHERICAL_ROTY or jointType == JOINT_TYPE_SPHERICAL_ROTZ:
            joint = UsdPhysics.Joint.Define(stage, path)
            lockedAxes = [UsdPhysics.Tokens.transX, UsdPhysics.Tokens.transY, UsdPhysics.Tokens.transZ]            
            for i in range(3):
                limitAPI = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), lockedAxes[i])
                limitAPI.CreateLowAttr(1.0)
                limitAPI.CreateHighAttr(-1.0)
        else:
            return None

        joint.GetBody0Rel().AddTarget(body0Path)
        joint.GetBody1Rel().AddTarget(body1Path)

        joint.GetLocalPos0Attr().Set(localPos0)
        joint.GetLocalRot0Attr().Set(localRot0)

        joint.GetLocalPos1Attr().Set(localPos1)
        joint.GetLocalRot1Attr().Set(localRot1)

        physxJointAPI = PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
        physxJointAPI.GetJointFrictionAttr().Set(0.0)
        physxJointAPI.GetArmatureAttr().Set(0.0)

        return joint

    # returns (art, rootLink, childLink, childInboundJoint)
    def _create_fixedbase_prismatic_or_revolute_or_spherical(
        self,
        stage: Usd.Stage,
        jointType: int = JOINT_TYPE_PRISMATIC_X,
        nbPosIters: int = 4, nbVelIters: int = 1,        
        rootLinkBoxSize: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0),
        childLinkMass: float = 1.0,
        childLinkInertia: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0),
        childLinkBoxSize: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0)
        ) -> tuple[Usd.Prim, Usd.Prim, Usd.Prim, Usd.Prim]:

        rootPath = str(stage.GetDefaultPrim().GetPath())
        rootLinkPath = rootPath + "/RootLink"
        childLinkPath = rootPath + "/ChildLink"
        childLinkInboundJointPath = childLinkPath + "/InboundJoint"

        # Create the fixed articulation and root link
        rootLinkPosition = Gf.Vec3f(0.0, 0.0, 0.0)
        rootLinkOrientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        (artPrim, rootLinkPrim) = self._create_fixed_base_articulation(
            stage,
            rootLinkPath, rootLinkPosition, rootLinkOrientation, rootLinkBoxSize,
            nbPosIters, nbVelIters)

        # Create the child link
        childLinkPosition = Gf.Vec3f(0.0, 0.0, 0.0)
        childLinkOrientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        childLinkPrim = self._create_body(
            stage,
            childLinkPath, childLinkPosition, childLinkOrientation, childLinkMass, childLinkInertia, childLinkBoxSize)

        # Create a joint as the inbound joint of the child link.
        localPos0 = Gf.Vec3f(0.0, 0.0, 0.0)
        localRot0 = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        localPos1 = Gf.Vec3f(0.0, 0.0, 0.0)
        localRot1 = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        childInboundJoint = self._create_joint(
            stage,
            childLinkInboundJointPath, jointType,
            rootLinkPath,
            childLinkPath,
            localPos0, localRot0, localPos1, localRot1)

        # Finished
        return (artPrim, rootLinkPrim, childLinkPrim, childInboundJoint.GetPrim())

    # returns (art, rootLink, childLink, childInboundJoint, jointDriveAPI, jointStateAPI)
    def _create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
        self,
        stage: Usd.Stage,
        jointType: int,
        nbPosIters: int = 4, nbVelIters: int = 1,        
        rootLinkBoxSize: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0),
        childLinkMass: float = 1.0,
        childLinkInertia: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0),
        childLinkBoxSize: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0),
        ) -> tuple[Usd.Prim, Usd.Prim, Usd.Prim, Usd.Prim, UsdPhysics.DriveAPI, PhysxSchema.JointStateAPI]:
        
        # create a physx scene with gravity = 0
        physicsScene = self._create_zero_gravity_scene(stage)

        # create and configure the articulation, root link, child link, inbound joint of child link.
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim) = self._create_fixedbase_prismatic_or_revolute_or_spherical(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)

        # create the joint state api.
        jointStateAPI = PhysxSchema.JointStateAPI.Apply(childLinkInboundJointPrim, SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType])

        # configure the inbound jont of the child link with drive and performance envelope
        jointDriveAPI = UsdPhysics.DriveAPI.Apply(childLinkInboundJointPrim, SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType])

        return (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI)

    #######################################################################
    ## run tests
    #######################################################################

    def _run_test_apply_performance_envelope_api(self, stage: Usd.Stage, jointType: int) -> None: 

        #  scene params
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        lengthScale = 1.0 / metersPerUnit
        lengthScaleSquared = lengthScale*lengthScale
        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        massScale = 1.0 / kilogramsPerUnit

        # articulation params
        nbPosIters = 4
        nbVelIters = 1

        # Root and link box shapes (for debug viz only)
        rootLinkBoxSize = Gf.Vec3f(0.5*lengthScale, 0.5*lengthScale, 0.5*lengthScale)
        childLinkBoxSize = Gf.Vec3f(1.0*lengthScale, 1.0*lengthScale, 1.0*lengthScale)

        # params of child link
        childLinkMass = 1.0*massScale
        childLinkInertia = Gf.Vec3f(1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared)

        # drive and performance envelope
        driveStiffness = 10.0
        driveDamping = 1.0
        driveTargetPos = 1.0
        driveTargetVel =1.0
        performanceEnvelopeMaxEffort = 1.0
        performanceEnvelopeMaxActuatorVelocity = 1.0
        performanceEnvelopeVelDependentResistance = 1.0
        performanceEnvelopeSpeedEffortGradient = 1.0

        # joint start pos and vel
        jointStartPos = 0.0
        jointStartVel = 0.0

        # create the prismatic articulation and configure it with a drive
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)

        jointStateAPI.GetPositionAttr().Set(jointStartPos)
        jointStateAPI.GetVelocityAttr().Set(jointStartVel)
        jointDriveAPI.GetStiffnessAttr().Set(driveStiffness) 
        jointDriveAPI.GetDampingAttr().Set(driveDamping)
        jointDriveAPI.GetTargetPositionAttr().Set(driveTargetPos)
        jointDriveAPI.GetTargetVelocityAttr().Set(driveTargetVel);


        # Test that we can apply the api
        self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
        hasAPI = self._hasAppliedPerfAPI(childLinkInboundJointPrim, jointType)
        self.assertEqual(hasAPI, True)

        if jointType == JOINT_TYPE_PRISMATIC_X or jointType == JOINT_TYPE_PRISMATIC_Y or jointType == JOINT_TYPE_PRISMATIC_Z:
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_REVOLUTE_X), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTX), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTY), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTZ), False)
        elif jointType == JOINT_TYPE_REVOLUTE_X or jointType == JOINT_TYPE_REVOLUTE_Y or jointType == JOINT_TYPE_REVOLUTE_Z:
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_PRISMATIC_X), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTX), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTY), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTZ), False)
        elif jointType == JOINT_TYPE_SPHERICAL_ROTX:
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_PRISMATIC_X), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_REVOLUTE_X), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTY), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTZ), False)
        elif jointType == JOINT_TYPE_SPHERICAL_ROTY:
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_PRISMATIC_X), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_REVOLUTE_X), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTX), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTZ), False)
        elif jointType == JOINT_TYPE_SPHERICAL_ROTZ:
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_PRISMATIC_X), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_REVOLUTE_X), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTX), False)
            self.assertEqual(self._hasAppliedPerfAPI(childLinkInboundJointPrim, JOINT_TYPE_SPHERICAL_ROTY), False)
            
        # Test we can set and query maxActuatorVelocity
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, 2.5)
        (success, maxActuatorVelocity) = self._getPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY)     
        self.assertEqual(success, True)
        self.assertEqual(maxActuatorVelocity, 2.5)

        # Test we can set and query velocityDependentResistance
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE, 3.5)
        (success, velocityDependentResistance) = self._getPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE)
        self.assertEqual(success, True)
        self.assertEqual(velocityDependentResistance, 3.5)

        # Test we can set and query speedEffortGradient
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT, 4.5)
        (success, speedEffortGradient) = self._getPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT)
        self.assertEqual(success, True)
        self.assertEqual(speedEffortGradient, 4.5)
 
        self._simulate_one_frame_with_prep()

    def _run_test_max_effort_with_no_perf_env(self, stage: Usd.Stage, jointType: int) -> None:
    
        #  scene params
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        lengthScale = 1.0 / metersPerUnit
        lengthScaleSquared = lengthScale*lengthScale
        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        massScale = 1.0 / kilogramsPerUnit
    
        # articulation params
        nbPosIters = 1
        nbVelIters = 0
    
        # Root and link box shapes (for debug viz only)
        rootLinkBoxSize = Gf.Vec3f(0.5*lengthScale, 0.5*lengthScale, 0.5*lengthScale)
        childLinkBoxSize = Gf.Vec3f(1.0*lengthScale, 1.0*lengthScale, 1.0*lengthScale)
    
        # params of child link
        childLinkMass = 1.0*massScale
        childLinkInertia = Gf.Vec3f(1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared)
    
        # drive params
        driveStiffness = 100.0
        driveDamping = 0.0
        driveTargetPos = 1.0 
        driveTargetVel = 0.0
    
        # create the prismatic articulation and configure it with a drive
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)
    
        jointDriveAPI.GetStiffnessAttr().Set(self._scaleStiffnessToDegreesAsNecessary(jointType, driveStiffness)) 
        jointDriveAPI.GetDampingAttr().Set(self._scaleDampingToDegreesAsNecessary(jointType, driveDamping))
        jointDriveAPI.GetTargetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, driveTargetPos))
        jointDriveAPI.GetTargetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, driveTargetVel))

        # Spherical joints do not behave the same as prismatic/revolute joints for reasons unknown.
        # This issue is tracked in nvbug 5199927
        permittedError = [0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.1, 0.1, 0.1]

        # test that the drive force works as expected when below the max force threshold.
        # the force is 100 for 0.01 seconds
        jointStartPos = 0
        jointStartVel = 0
        driveMaxForce = 10000000.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        self._simulate_one_frame_with_prep()
        expectedJointSpeed = jointStartVel + self._get_time_step()*driveStiffness*(driveTargetPos - jointStartPos)/childLinkMass
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case1", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedError[jointType])
    
        # test that the drive force works as expected when abov the max force threshold
        # the clamped force is 1 for 0.01 seconds
        jointStartPos = 0
        jointStartVel = 0
        driveMaxForce = 1
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel + self._get_time_step()*driveMaxForce/childLinkMass
        measuredJointSpeed = jointStateAPI.GetVelocityAttr().Get()
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case2", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.00001)
    
    def _run_test_perf_env_max_actuator_velocity(self, stage: Usd.Stage, jointType: int) -> None:
    
        #  scene params
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        lengthScale = 1.0 / metersPerUnit
        lengthScaleSquared = lengthScale*lengthScale
        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        massScale = 1.0 / kilogramsPerUnit
    
        # articulation params
        nbPosIters = 1
        nbVelIters = 0
    
        # Root and link box shapes (for debug viz only)
        rootLinkBoxSize = Gf.Vec3f(0.5*lengthScale, 0.5*lengthScale, 0.5*lengthScale)
        childLinkBoxSize = Gf.Vec3f(1.0*lengthScale, 1.0*lengthScale, 1.0*lengthScale)
    
        # params of child link
        childLinkMass = 1.0*massScale
        childLinkInertia = Gf.Vec3f(1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared)
    
        # drive params
        driveStiffness = 100.0
        driveDamping = 0.0
        driveTargetPos = 1.0
        driveTargetVel = 0.0
    
        # create the prismatic articulation and configure it with a drive
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)
    
        jointDriveAPI.GetStiffnessAttr().Set(self._scaleStiffnessToDegreesAsNecessary(jointType, driveStiffness))
        jointDriveAPI.GetDampingAttr().Set(self._scaleDampingToDegreesAsNecessary(jointType, driveDamping))
        jointDriveAPI.GetTargetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, driveTargetPos))
        jointDriveAPI.GetTargetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, driveTargetVel))
    
        # Case 1
        # run a singe step with the performance envelope
        # test that the drive force works as expected when the joint velocity is outside the performance envelope.
        # the unclamped drive force is 100 for 0.01 seconds which would on its own create a joint speed: jointStartVel + kp*deltaX*dt = 11
        # but with clamping it should be 0 and the joint velocity should be unchanged
        # make sure the clamping is not due to maxDriveForce by setting it to a huge value
        self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 10.0
        driveMaxForce = 10000000.0
        perfEnvMaxActuatorVelocity = 1.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._simulate_one_frame_with_prep()
        expectedJointSpeed = jointStartVel
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case1", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)
    
        # Case 2
        # run a single step without the performance envelope by removing the api
        # make sure we start outside the performance envelope of the previous step to make sure we really have removed the api properly
        # the unclamped drive force is 100 for 0.01 seconds so the outcome joint speed should be:  jointStartVel + kp*deltaX*dt = 11
        self._removePerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 10.0
        driveMaxForce = 10000000.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel + self._get_time_step()*driveStiffness*(driveTargetPos - jointStartPos)/childLinkMass
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        #print("case2", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        error = abs(expectedJointSpeed - measuredJointSpeed)
        self.assertLess(error, 0.2)
    
        # Case 3
        # now reapply the performance envelope api so that we are outside the performance envelope
        # and the drive force is again clamped to 0
        # this is the same as case 1.
        self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0 
        jointStartVel = 10.0
        driveMaxForce = 10000000.0
        perfEnvMaxActuatorVelocity = 1.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        #print("case3", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        error = abs(expectedJointSpeed - measuredJointSpeed)
        self.assertLess(error, 0.0001)
    
        # Case 4
        # run a single step without the performance envelope by removing the api
        # make sure we start outside the performance envelope of the previous step to make sure we really have removed the api properly
        # the unclamped drive force is 100 for 0.01 seconds so the outcome joint speed should be:  jointStartVel + kp*deltaX*dt = 11
        # This is the same as case 2.
        self._removePerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 10.0
        driveMaxForce = 10000000.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel + self._get_time_step()*driveStiffness*(driveTargetPos - jointStartPos)/childLinkMass
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        #print("case4", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        error = abs(expectedJointSpeed - measuredJointSpeed)
        self.assertLess(error, 0.2)
    
        # Case 5
        # now reapply the performance envelope api so that we are outside the performance envelope
        # and the drive force is again clamped to 0
        # this is the same as case 1.
        self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 10.0
        driveMaxForce = 10000000.0
        perfEnvMaxActuatorVelocity = 1.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        #print("case5", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        error = abs(expectedJointSpeed - measuredJointSpeed)
        self.assertLess(error, 0.0001)
    
        # Case 6
        # With the performance envelope api applied, update the max velocity so that we are inside the envelope.
        # The envelope should not impede the drive.
        jointStartPos = 0.0
        jointStartVel = 10.0
        driveMaxForce = 10000000.0
        perfEnvMaxActuatorVelocity = 10000.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel + self._get_time_step()*driveStiffness*(driveTargetPos - jointStartPos)/childLinkMass
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        #print("case6", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        error = abs(expectedJointSpeed - measuredJointSpeed)
        self.assertLess(error, 0.2)

    def _run_test_perf_env_max_effort(self, stage: Usd.Stage, jointType: int) -> None:
    
        #  scene params
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        lengthScale = 1.0 / metersPerUnit
        lengthScaleSquared = lengthScale*lengthScale
        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        massScale = 1.0 / kilogramsPerUnit
    
        # articulation params
        nbPosIters = 1
        nbVelIters = 0
    
        # Root and link box shapes (for debug viz only)
        rootLinkBoxSize = Gf.Vec3f(0.5*lengthScale, 0.5*lengthScale, 0.5*lengthScale)
        childLinkBoxSize = Gf.Vec3f(1.0*lengthScale, 1.0*lengthScale, 1.0*lengthScale)
    
        # params of child link
        childLinkMass = 1.0*massScale
        childLinkInertia = Gf.Vec3f(1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared)
    
        # drive params
        driveStiffness = 100.0
        driveDamping = 0.0
        driveMaxForce = 1.0
        driveTargetPos = 1.0
        driveTargetVel = 0.0
    
        # create the prismatic articulation and configure it with a drive
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)
    
        jointDriveAPI.GetStiffnessAttr().Set(self._scaleStiffnessToDegreesAsNecessary(jointType, driveStiffness))
        jointDriveAPI.GetDampingAttr().Set(self._scaleDampingToDegreesAsNecessary(jointType, driveDamping))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        jointDriveAPI.GetTargetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, driveTargetPos))
        jointDriveAPI.GetTargetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, driveTargetVel))
    
        # Case 1
        # run a singe step with the performance envelope
        # test that the perf envelope max effort works clamps the drive force
        # the unclamped drive force is 100
        # the clamped force is 1
        # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
        self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 10.0
        perfEnvMaxActuatorVelocity = 1000000.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._simulate_one_frame_with_prep()
        expectedJointSpeed = jointStartVel + driveMaxForce*self._get_time_step()/childLinkMass
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case1", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)
    
        # Case 2
        # run a single step without the performance envelope by removing the api
        # the drive max force should clamp the applied force
        # the unclamped drive force is 100
        # the clamped force is 1
        # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
        # do not set driveMaxForce again to make sure it transfers from perf env max effort to drive max force
        self._removePerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 10.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel + driveMaxForce*self._get_time_step()/childLinkMass
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        #print("case2", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        error = abs(expectedJointSpeed - measuredJointSpeed)
        self.assertLess(error, 0.0001)
    
        # Case 3
        # now reapply the performance envelope api
        # test that the perf envelope max effort works clamps the drive force
        # the unclamped drive force is 100
        # the clamped force is 1
        # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
        # do not set driveMaxForce again to make sure it transfers from drive max force to perf env
        self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 10.0
        perfEnvMaxActuatorVelocity = 1000000.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel + driveMaxForce*self._get_time_step()/childLinkMass
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case3", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)
    
        # Case 4
        # run a single step without the performance envelope by removing the api
        # the drive max force should clamp the applied force
        # the unclamped drive force is 100
        # the clamped force is 1
        # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
        # do not set driveMaxForce again to make sure it transfers from perf env max effort to drive max force
        self._removePerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 10.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel + driveMaxForce*self._get_time_step()/childLinkMass
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        #print("case4", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        error = abs(expectedJointSpeed - measuredJointSpeed)
        self.assertLess(error, 0.0001)
    
        # Case 5
        # now reapply the performance envelope api
        # test that the perf envelope max effort works clamps the drive force
        # the unclamped drive force is 100
        # the clamped force is 1
        # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
        # do not set driveMaxForce again to make sure it transfers from drive max force to perf env
        self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 10.0
        perfEnvMaxActuatorVelocity = 1000000.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel + driveMaxForce*self._get_time_step()/childLinkMass
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case5", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)
    
        # Case 6
        # With the performance envelope api applied, update the max effort so that we are inside the envelope.
        # The envelope should not impede the drive.
        # the unclamped drive force is 100
        jointStartPos = 0.0
        jointStartVel = 10.0
        driveMaxForce = 10000000.0
        perfEnvMaxActuatorVelocity = 1000000.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._simulate_one_frame()
        expectedJointSpeed = jointStartVel + self._get_time_step()*driveStiffness*(driveTargetPos - jointStartPos)/childLinkMass
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        #print("case6", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        error = abs(expectedJointSpeed - measuredJointSpeed)
        self.assertLess(error, 0.2)


    def _run_test_perf_env_velocity_dependent_resistance(self, stage: Usd.Stage, jointType: int) -> None:
    
       #  scene params
       metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
       lengthScale = 1.0 / metersPerUnit
       lengthScaleSquared = lengthScale*lengthScale
       kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
       massScale = 1.0 / kilogramsPerUnit
    
       # articulation params
       nbPosIters = 1
       nbVelIters = 0
    
       # Root and link box shapes (for debug viz only)
       rootLinkBoxSize = Gf.Vec3f(0.5*lengthScale, 0.5*lengthScale, 0.5*lengthScale)
       childLinkBoxSize = Gf.Vec3f(1.0*lengthScale, 1.0*lengthScale, 1.0*lengthScale)
    
       # params of child link
       childLinkMass = 1.0*massScale
       childLinkInertia = Gf.Vec3f(1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared)
    
       # drive params (generate drive force of 100 and maxForce of 100)
       driveStiffness = 100.0
       driveDamping = 0.0
       driveMaxForce = 100.0
       driveTargetPos = 1.0
       driveTargetVel = 0.0
    
       # create the prismatic articulation and configure it with a drive
       (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
           stage,
           jointType,
           nbPosIters, nbVelIters,
           rootLinkBoxSize,
           childLinkMass, childLinkInertia, childLinkBoxSize)
    
       jointDriveAPI.GetStiffnessAttr().Set(self._scaleStiffnessToDegreesAsNecessary(jointType, driveStiffness))
       jointDriveAPI.GetDampingAttr().Set(self._scaleDampingToDegreesAsNecessary(jointType, driveDamping))
       jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
       jointDriveAPI.GetTargetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, driveTargetPos))
       jointDriveAPI.GetTargetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, driveTargetVel))
    
       # Case 1
       # run a singe step with the performance envelope
       # test that the perf envelope velocity dependent resistance works clamps the drive force
       # the unclamped drive force is 100
       # The clamped drive force is 100 - velDepRes * vel = 100 - 5 * 10 = 50
       # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
       self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
       jointStartPos = 0.0
       jointStartVel = 10.0
       perfEnvVelDepRes = 5.0
       perfEnvMaxActuatorVelocity = 1000000.0
       jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
       jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
       self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
       self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE, self._scaleVelocityDependentResistanceToDegreesAsNecessary(jointType, perfEnvVelDepRes))
       self._simulate_one_frame_with_prep()
       expectedJointSpeed = jointStartVel + ((100.0 - 5.0*10.0)/1.0)*0.01
       measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
       error = abs(expectedJointSpeed - measuredJointSpeed)
       #print("case1", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
       self.assertLess(error, 0.1)
    
       # Case 2
       # run a single step without the performance envelope by removing the api
       # the drive max force should clamp the applied force
       # the unclamped drive force is 100
       # the clamped force is 1
       # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
       # do not set driveMaxForce again to make sure it transfers from perf env max effort to drive max force
       self._removePerfEnvAPI(childLinkInboundJointPrim, jointType)
       jointStartPos = 0.0
       jointStartVel = 10.0
       jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
       jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
       self._simulate_one_frame()
       expectedJointSpeed = jointStartVel + ((100.0 - 0.0*10.0)/1.0)*0.01
       measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
       #print("case2", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
       error = abs(expectedJointSpeed - measuredJointSpeed)
       self.assertLess(error, 0.2)
    
       # Case 3
       # now reapply the performance envelope api
       # test that the perf envelope velocity dependent resistance works clamps the drive force
       # the unclamped drive force is 100
       # The clamped drive force is 100 - velDepRes * vel = 100 - 5 * 10 = 50
       # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
       self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
       jointStartPos = 0.0
       jointStartVel = 10.0
       perfEnvVelDepRes = 5.0
       perfEnvMaxActuatorVelocity = 1000000.0
       jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
       jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
       self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
       self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE, self._scaleVelocityDependentResistanceToDegreesAsNecessary(jointType, perfEnvVelDepRes))
       self._simulate_one_frame()
       expectedJointSpeed = jointStartVel + ((100.0 - 5.0*10.0)/1.0)*0.01
       measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
       error = abs(expectedJointSpeed - measuredJointSpeed)
       #print("case3", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
       self.assertLess(error, 0.1)
    
       # Case 4
       # run a single step without the performance envelope by removing the api
       # the drive max force should clamp the applied force
       # the unclamped drive force is 100
       # the clamped force is 1
       # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
       # do not set driveMaxForce again to make sure it transfers from perf env max effort to drive max force
       self._removePerfEnvAPI(childLinkInboundJointPrim, jointType)
       jointStartPos = 0.0
       jointStartVel = 10.0
       jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
       jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
       self._simulate_one_frame()
       expectedJointSpeed = jointStartVel + ((100.0 - 0.0*10.0)/1.0)*0.01
       measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
       #print("case4", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
       error = abs(expectedJointSpeed - measuredJointSpeed)
       self.assertLess(error, 0.2)
    
       # Case 5
       # now reapply the performance envelope api
       # test that the perf envelope velocity dependent resistance works clamps the drive force
       # the unclamped drive force is 100
       # The clamped drive force is 100 - velDepRes * vel = 100 - 5 * 10 = 50
       # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
       self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
       jointStartPos = 0.0
       jointStartVel = 10.0
       perfEnvVelDepRes = 5.0
       perfEnvMaxActuatorVelocity = 1000000.0
       jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
       jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
       self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
       self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE, self._scaleVelocityDependentResistanceToDegreesAsNecessary(jointType, perfEnvVelDepRes))
       self._simulate_one_frame()
       expectedJointSpeed = jointStartVel + ((100.0 - 5.0*10.0)/1.0)*0.01
       measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
       error = abs(expectedJointSpeed - measuredJointSpeed)
       #print("case5", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
       self.assertLess(error, 0.1)
    
       # Case 6
       # With the performance envelope api applied, update the velocity dependent resistance.
       # the unclamped drive force is 100
       # The clamped drive force is 100 - velDepRes * vel = 100 - 8 * 10 = 20
       # make sure the clamping is not due to maxActuatorVelocity by setting it to a huge value
       # The envelope should not impede the drive.
       # the unclamped drive force is 100
       self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
       jointStartPos = 0.0
       jointStartVel = 10.0
       perfEnvVelDepRes = 8.0
       perfEnvMaxActuatorVelocity = 1000000.0
       jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
       jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
       self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
       self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE, self._scaleVelocityDependentResistanceToDegreesAsNecessary(jointType, perfEnvVelDepRes))
       self._simulate_one_frame()
       expectedJointSpeed = jointStartVel + ((100.0 - 8.0*10.0)/1.0)*0.01
       measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
       error = abs(expectedJointSpeed - measuredJointSpeed)
       #print("case6", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
       self.assertLess(error, 0.1)

    def _run_test_perf_env_speed_effort_gradient(self, stage: Usd.Stage, jointType: int) -> None:
    
        #  scene params
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        lengthScale = 1.0 / metersPerUnit
        lengthScaleSquared = lengthScale*lengthScale
        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        massScale = 1.0 / kilogramsPerUnit
    
        # articulation params
        nbPosIters = 1
        nbVelIters = 0
    
        # Root and link box shapes (for debug viz only)
        rootLinkBoxSize = Gf.Vec3f(0.5*lengthScale, 0.5*lengthScale, 0.5*lengthScale)
        childLinkBoxSize = Gf.Vec3f(1.0*lengthScale, 1.0*lengthScale, 1.0*lengthScale)
    
        # params of child link
        childLinkMass = 1.0*massScale
        childLinkInertia = Gf.Vec3f(1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared, 1.0*massScale*lengthScaleSquared)
    
        # drive params (generate drive force of 100 and maxForce of 100)
        driveStiffness = 100.0
        driveDamping = 0.0
        driveMaxForce = 10000000.0
        driveTargetPos = 1.0
        driveTargetVel = 0.0
    
        # create the prismatic articulation and configure it with a drive
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)
    
        jointDriveAPI.GetStiffnessAttr().Set(self._scaleStiffnessToDegreesAsNecessary(jointType, driveStiffness))
        jointDriveAPI.GetDampingAttr().Set(self._scaleDampingToDegreesAsNecessary(jointType, driveDamping))
        jointDriveAPI.GetMaxForceAttr().Set(driveMaxForce)
        jointDriveAPI.GetTargetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, driveTargetPos))
        jointDriveAPI.GetTargetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, driveTargetVel))
    
        # Case 1
        # run a singe step with the performance envelope
        # test that the perf envelope speed effort gradient clamps the drive force
        # the unclamped drive force is 100
        # set vStart = 9, vmax = 10, force = 100, speedEffortGradient=0.05, output vel = 9.16666
        self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 9.0
        speedEffortGradient = 0.05
        perfEnvMaxActuatorVelocity = 10.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT, self._scaleSpeedEffortGradientToDegreesAsNecessary(jointType, speedEffortGradient))
        self._simulate_one_frame_with_prep()
        expectedJointSpeed = 9.166667
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case1", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.01)
    
        # Case 2
        # run a single step without the performance envelope by removing the api
        # the unclamped drive force is 100
        self._removePerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 9.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 10.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        #print("case2", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        error = abs(expectedJointSpeed - measuredJointSpeed)
        self.assertLess(error, 0.2)
    
        # Case 3
        # now reapply the performance envelope api
        # run a singe step with the performance envelope
        # test that the perf envelope speed effort gradient works clamps the drive force
        # the unclamped drive force is 100
        # set vStart = 9, vmax = 10, force = 100, speedEffortGradient=0.05, output vel = 9.16666
        self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 9.0
        speedEffortGradient = 0.05
        perfEnvMaxActuatorVelocity = 10.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT, self._scaleSpeedEffortGradientToDegreesAsNecessary(jointType, speedEffortGradient))
        self._simulate_one_frame()
        expectedJointSpeed = 9.166667
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case3", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.01)
    
    
        # Case 4
        # run a single step without the performance envelope by removing the api
        # the unclamped drive force is 100
        self._removePerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 9.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 10.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        #print("case4", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        error = abs(expectedJointSpeed - measuredJointSpeed)
        self.assertLess(error, 0.2)
    
        # Case 5
        # now reapply the performance envelope api
        # run a singe step with the performance envelope
        # test that the perf envelope speed effort gradient clamps the drive force
        # the unclamped drive force is 100
        # set vStart = 9, vmax = 10, force = 100, speedEffortGradient=0.05, output vel = 9.16666
        self._applyPerfEnvAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 9.0
        speedEffortGradient = 0.05
        perfEnvMaxActuatorVelocity = 10.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, perfEnvMaxActuatorVelocity))
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT, self._scaleSpeedEffortGradientToDegreesAsNecessary(jointType, speedEffortGradient))
        self._simulate_one_frame()
        expectedJointSpeed = 9.166667
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case5", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.01)
    
    
        # Case 6
        # With the performance envelope api applied, update the speed effort gradient
        # test that the perf envelope speed effort gradient clamps the drive force
        # the unclamped drive force is 100
        # set vStart = 9, vmax = 10, force = 100, speedEffortGradient=50.0, output vel = 9.0
        jointStartPos = 0.0
        jointStartVel = 9.0
        speedEffortGradient = 50.0
        perfEnvMaxActuatorVelocity = 10.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setPerfEnvAttribute(childLinkInboundJointPrim, jointType, PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT, self._scaleSpeedEffortGradientToDegreesAsNecessary(jointType, speedEffortGradient))
        self._simulate_one_frame()
        expectedJointSpeed = 9.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case6", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.01)
        
        
    async def _run_test_perf_env_warnings(self, jointType: int):
    
        for i in range(2):
            if i == 0:
                setupAsArticulation = True
            else:
                setupAsArticulation = False
            setupAsArticulation = False
            await self._setup_stage_and_bodies(setupAsArticulation = setupAsArticulation)
            if jointType == JOINT_TYPE_PRISMATIC_X or jointType == JOINT_TYPE_PRISMATIC_Y or jointType == JOINT_TYPE_PRISMATIC_Z:
               type = "prismatic"
               axis =  "linear"
            else:
               type = "revolute"
               axis =  "angular"
    
            self._setup_joint(type, axis = "X")
            mass = 1.0
            dt = 1.0 / 60.0
            targetVelocity = 1.0
    
            damping = mass / dt
    
            massAPI = UsdPhysics.MassAPI(self._dynamic_box)
            massAPI.CreateMassAttr(mass)
            massAPI.CreateDensityAttr(0.0)
    
            self._setup_joint_drive(type = axis, damping = damping, targetVel = targetVelocity)
            self._applyPerfEnvAPI(self._joint.GetPrim(), jointType)
            self._setPerfEnvAttribute(self._joint.GetPrim(), jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, 0.0)

            if setupAsArticulation:
                 with utils.AssertNoMessages(self): 
                     self.step(1, dt)
            else:
    
                message = "Performance envelope is supported only for joints that are part of an articulation. Envelope will be ignored for joint: /World/"+type+"Joint"
                with utils.ExpectMessage(self, message):
                    self.step(1, dt)
                message = "Please ensure that the joint is part of an articulation. Performance envelope will be ignored for joint: /World/"+type+"Joint"
                with utils.ExpectMessage(self, message):
                    self._applyPerfEnvAPI(self._joint.GetPrim(), jointType)
                    self._setPerfEnvAttribute(self._joint.GetPrim(), jointType, PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY, 0.0)
                    self.step(1, dt)
    
    #######################################################################
    ## the tests
    #######################################################################

    async def test_apply_performance_envelope_api_revolute_x(self):
        stage = await self.new_stage()
        self._run_test_apply_performance_envelope_api(stage, JOINT_TYPE_REVOLUTE_X)

    async def test_apply_performance_envelope_api_revolute_y(self):
        stage = await self.new_stage()
        self._run_test_apply_performance_envelope_api(stage, JOINT_TYPE_REVOLUTE_Y)

    async def test_apply_performance_envelope_api_revolute_z(self):
        stage = await self.new_stage()
        self._run_test_apply_performance_envelope_api(stage, JOINT_TYPE_REVOLUTE_Z)

    async def test_apply_performance_envelope_api_prismatic_x(self):
        stage = await self.new_stage()
        self._run_test_apply_performance_envelope_api(stage, JOINT_TYPE_PRISMATIC_X)

    async def test_apply_performance_envelope_api_prismatic_y(self):
        stage = await self.new_stage()
        self._run_test_apply_performance_envelope_api(stage, JOINT_TYPE_PRISMATIC_Y)

    async def test_apply_performance_envelope_api_prismatic_z(self):
        stage = await self.new_stage()
        self._run_test_apply_performance_envelope_api(stage, JOINT_TYPE_PRISMATIC_Z)

    async def test_apply_performance_envelope_api_spherical_rotx(self):
        stage = await self.new_stage()
        self._run_test_apply_performance_envelope_api(stage, JOINT_TYPE_SPHERICAL_ROTX)

    async def test_apply_performance_envelope_api_spherical_roty(self):
        stage = await self.new_stage()
        self._run_test_apply_performance_envelope_api(stage, JOINT_TYPE_SPHERICAL_ROTY)

    async def test_apply_performance_envelope_api_spherical_rotz(self):
        stage = await self.new_stage()
        self._run_test_apply_performance_envelope_api(stage, JOINT_TYPE_SPHERICAL_ROTZ)



    #######################################################################
    
    async def test_max_effort_with_no_perf_env_revolute_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_max_effort_with_no_perf_env(stage, JOINT_TYPE_REVOLUTE_X)
    
    async def test_max_effort_with_no_perf_env_revolute_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_max_effort_with_no_perf_env(stage, JOINT_TYPE_REVOLUTE_Y)
    
    async def test_max_effort_with_no_perf_env_revolute_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_max_effort_with_no_perf_env(stage, JOINT_TYPE_REVOLUTE_Z)
    
    async def test_max_effort_with_no_perf_env_prismatic_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_max_effort_with_no_perf_env(stage, JOINT_TYPE_PRISMATIC_X)
    
    async def test_max_effort_with_no_perf_env_prismatic_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_max_effort_with_no_perf_env(stage, JOINT_TYPE_PRISMATIC_Y)
    
    async def test_max_effort_with_no_perf_env_prismatic_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_max_effort_with_no_perf_env(stage, JOINT_TYPE_PRISMATIC_Z)
    
    async def test_max_effort_with_no_perf_env_spherical_rotx(self) -> None:
        stage = await self.new_stage()
        self._run_test_max_effort_with_no_perf_env(stage, JOINT_TYPE_SPHERICAL_ROTX)
    
    async def test_max_effort_with_no_perf_env_spherical_roty(self) -> None:
        stage = await self.new_stage()
        self._run_test_max_effort_with_no_perf_env(stage, JOINT_TYPE_SPHERICAL_ROTY)
    
    async def test_max_effort_with_no_perf_env_spherical_rotz(self) -> None:
        stage = await self.new_stage()
        self._run_test_max_effort_with_no_perf_env(stage, JOINT_TYPE_SPHERICAL_ROTZ)

    
    ########################################################################
    
    async def test_perf_env_max_actuator_velocity_revolute_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_actuator_velocity(stage, JOINT_TYPE_REVOLUTE_X)
    
    async def test_perf_env_max_actuator_velocity_revolute_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_actuator_velocity(stage, JOINT_TYPE_REVOLUTE_Y)
    
    async def test_perf_env_max_actuator_velocity_revolute_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_actuator_velocity(stage, JOINT_TYPE_REVOLUTE_Z)
    
    async def test_perf_env_max_actuator_velocity_prismatic_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_actuator_velocity(stage, JOINT_TYPE_PRISMATIC_X)
    
    async def test_perf_env_max_actuator_velocity_prismatic_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_actuator_velocity(stage, JOINT_TYPE_PRISMATIC_Y)
    
    async def test_perf_env_max_actuator_velocity_prismatic_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_actuator_velocity(stage, JOINT_TYPE_PRISMATIC_Z)

    async def test_perf_env_max_actuator_velocity_spherical_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_actuator_velocity(stage, JOINT_TYPE_SPHERICAL_ROTX)
    
    async def test_perf_env_max_actuator_velocity_spherical_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_actuator_velocity(stage, JOINT_TYPE_SPHERICAL_ROTY)
    
    async def test_perf_env_max_actuator_velocity_spherical_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_actuator_velocity(stage, JOINT_TYPE_SPHERICAL_ROTZ)


    ########################################################################
    
    async def test_perf_env_max_effort_revolute_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_effort(stage, JOINT_TYPE_REVOLUTE_X)
    
    async def test_perf_env_max_effort_revolute_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_effort(stage, JOINT_TYPE_REVOLUTE_Y)
    
    async def test_perf_env_max_effort_revolute_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_effort(stage, JOINT_TYPE_REVOLUTE_Z)
    
    async def test_perf_env_max_effort_prismatic_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_effort(stage, JOINT_TYPE_PRISMATIC_X)
    
    async def test_perf_env_max_effort_prismatic_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_effort(stage, JOINT_TYPE_PRISMATIC_Y)
    
    async def test_perf_env_max_effort_prismatic_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_effort(stage, JOINT_TYPE_PRISMATIC_Z)

    async def test_perf_env_max_effort_spherical_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_effort(stage, JOINT_TYPE_SPHERICAL_ROTX)
    
    async def test_perf_env_max_effort_spherical_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_effort(stage, JOINT_TYPE_SPHERICAL_ROTY)
    
    async def test_perf_env_max_effort_spherical_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_max_effort(stage, JOINT_TYPE_SPHERICAL_ROTZ)

    
    ########################################################################
    
    async def test_perf_env_velocity_dependent_resistance_prismatic_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_velocity_dependent_resistance(stage, JOINT_TYPE_PRISMATIC_X)
    
    async def test_perf_env_velocity_dependent_resistance_prismatic_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_velocity_dependent_resistance(stage, JOINT_TYPE_PRISMATIC_Y)
    
    async def test_perf_env_velocity_dependent_resistance_prismatic_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_velocity_dependent_resistance(stage, JOINT_TYPE_PRISMATIC_Z)
    
    async def test_perf_env_velocity_dependent_resistance_revolute_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_velocity_dependent_resistance(stage, JOINT_TYPE_REVOLUTE_X)
    
    async def test_perf_env_velocity_dependent_resistance_revolute_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_velocity_dependent_resistance(stage, JOINT_TYPE_REVOLUTE_Y)
    
    async def test_perf_env_velocity_dependent_resistance_revolute_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_velocity_dependent_resistance(stage, JOINT_TYPE_REVOLUTE_Z)

    async def test_perf_env_velocity_dependent_resistance_spherical_x(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_velocity_dependent_resistance(stage, JOINT_TYPE_SPHERICAL_ROTX)
    
    async def test_perf_env_velocity_dependent_resistance_spherical_y(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_velocity_dependent_resistance(stage, JOINT_TYPE_SPHERICAL_ROTY)
    
    async def test_perf_env_velocity_dependent_resistance_spherical_z(self) -> None:
        stage = await self.new_stage()
        self._run_test_perf_env_velocity_dependent_resistance(stage, JOINT_TYPE_SPHERICAL_ROTZ)
    
    
    ########################################################################
    
    async def test_perf_env_speed_effort_gradient_prismatic_x(self):
        stage = await self.new_stage()
        self._run_test_perf_env_speed_effort_gradient(stage, JOINT_TYPE_PRISMATIC_X)
    
    async def test_perf_env_speed_effort_gradient_prismatic_y(self):
        stage = await self.new_stage()
        self._run_test_perf_env_speed_effort_gradient(stage, JOINT_TYPE_PRISMATIC_Y)
    
    async def test_perf_env_speed_effort_gradient_prismatic_z(self):
        stage = await self.new_stage()
        self._run_test_perf_env_speed_effort_gradient(stage, JOINT_TYPE_PRISMATIC_Z)
    
    async def test_perf_env_speed_effort_gradient_revolute_x(self):
        stage = await self.new_stage()
        self._run_test_perf_env_speed_effort_gradient(stage, JOINT_TYPE_REVOLUTE_X)
    
    async def test_perf_env_speed_effort_gradient_revolute_y(self):
        stage = await self.new_stage()
        self._run_test_perf_env_speed_effort_gradient(stage, JOINT_TYPE_REVOLUTE_Y)
    
    async def test_perf_env_speed_effort_gradient_revolute_z(self):
        stage = await self.new_stage()
        self._run_test_perf_env_speed_effort_gradient(stage, JOINT_TYPE_REVOLUTE_Z)

    async def test_perf_env_speed_effort_gradient_spherical_x(self):
        stage = await self.new_stage()
        self._run_test_perf_env_speed_effort_gradient(stage, JOINT_TYPE_SPHERICAL_ROTX)
    
    async def test_perf_env_speed_effort_gradient_spherical_y(self):
        stage = await self.new_stage()
        self._run_test_perf_env_speed_effort_gradient(stage, JOINT_TYPE_SPHERICAL_ROTY)
    
    async def test_perf_env_speed_effort_gradient_spherical_z(self):
        stage = await self.new_stage()
        self._run_test_perf_env_speed_effort_gradient(stage, JOINT_TYPE_SPHERICAL_ROTZ)
    
    
    ########################################################################
    
    async def test_perf_env_revolute_joint_warnings(self):
        await self._run_test_perf_env_warnings(JOINT_TYPE_REVOLUTE_X)
    
    async def test_perf_env_prismatic_joint_warnings(self):
        await self._run_test_perf_env_warnings(JOINT_TYPE_PRISMATIC_X)
