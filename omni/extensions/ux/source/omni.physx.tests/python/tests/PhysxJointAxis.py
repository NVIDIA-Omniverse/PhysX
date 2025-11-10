# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import typing
import carb
import omni.physx.scripts.utils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physxtests.testBases.articulationTestBase import ArticulationTestBase
from omni.physx import get_physx_simulation_interface, get_physx_replicator_interface
from omni.physxtests import utils
from pxr import Usd, Gf, Sdf, UsdGeom, UsdPhysics, UsdUtils, PhysxSchema, Tf, UsdShade
import math
from omni.physx.bindings._physx import (
    JOINT_AXIS_API,
    JOINT_AXIS_ATTR_ARMATURE_ANGULAR,
    JOINT_AXIS_ATTR_ARMATURE_LINEAR,
    JOINT_AXIS_ATTR_ARMATURE_ROTX,
    JOINT_AXIS_ATTR_ARMATURE_ROTY,
    JOINT_AXIS_ATTR_ARMATURE_ROTZ,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ANGULAR,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_LINEAR,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ROTX,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ROTY,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ROTZ,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ANGULAR,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_LINEAR,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ROTX,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ROTY,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ROTZ,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ANGULAR,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_LINEAR,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ROTX,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ROTY,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ROTZ,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ANGULAR,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_LINEAR,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ROTX,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ROTY,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ROTZ,
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
ARMATURE_TO_TOKEN_MAP = [
    JOINT_AXIS_ATTR_ARMATURE_LINEAR,
    JOINT_AXIS_ATTR_ARMATURE_LINEAR,
    JOINT_AXIS_ATTR_ARMATURE_LINEAR,
    JOINT_AXIS_ATTR_ARMATURE_ANGULAR,
    JOINT_AXIS_ATTR_ARMATURE_ANGULAR,
    JOINT_AXIS_ATTR_ARMATURE_ANGULAR,
    JOINT_AXIS_ATTR_ARMATURE_ROTX,
    JOINT_AXIS_ATTR_ARMATURE_ROTY,
    JOINT_AXIS_ATTR_ARMATURE_ROTZ,
]

MAX_JOINT_VELOCITY_TO_TOKEN_MAP = [
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_LINEAR,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_LINEAR,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_LINEAR,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ANGULAR,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ANGULAR,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ANGULAR,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ROTX,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ROTY,
    JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ROTZ,
]
STATIC_FRICTION_EFFORT_TO_TOKEN_MAP = [
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_LINEAR,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_LINEAR,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_LINEAR,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ANGULAR,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ANGULAR,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ANGULAR,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ROTX,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ROTY,
    JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ROTZ,
]

DYNAMIC_FRICTION_EFFORT_TO_TOKEN_MAP = [
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_LINEAR,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_LINEAR,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_LINEAR,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ANGULAR,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ANGULAR,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ANGULAR,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ROTX,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ROTY,
    JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ROTZ,
]

VISCOUS_FRICTION_COEFFICIENT_TO_TOKEN_MAP = [
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_LINEAR,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_LINEAR,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_LINEAR,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ANGULAR,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ANGULAR,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ANGULAR,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ROTX,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ROTY,
    JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ROTZ,
]

def _isSupportedJointType(jointType: int) -> bool:
    isSupported = False
    for supportedJointType in SUPPORTED_JOINT_TYPES:
        if jointType == supportedJointType:
            isSupported = True
    return isSupported

#attributes
JOINT_AXIS_ATTR_MAX_VELOCITY = 0
JOINT_AXIS_ATTR_ARMATURE = 1
JOINT_AXIS_ATTR_STATIC_FRICTION = 2
JOINT_AXIS_ATTR_DYNAMIC_FRICTION = 3
JOINT_AXIS_ATTR_VISCOUS_FRICTION = 4

class PhysxJointAxisAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def new_stage(self):
        self._clean_up()
        await super().new_stage(attach_stage=False)

        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self._stage, 1.0)

        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.y)

        return self._stage


    def _applyJointAxisAPI(self, jointPrim: Usd.Prim, jointType: int) -> bool:

        if not _isSupportedJointType(jointType):
            print("_applyJointAxisAPI called with illegal jointType")
            return false

        instanceToken = SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType]

        jointPrim.ApplyAPI(JOINT_AXIS_API, instanceToken)

        return True

    def _removeJointAxisAPI(self, jointPrim: Usd.Prim, jointType: int) -> bool:

        if not _isSupportedJointType(jointType):
            print("_removeJointAxisAPI called with illegal jointType")
            return false

        instanceToken = SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType]

        jointPrim.RemoveAPI(JOINT_AXIS_API, instanceToken)

        return True

    def _hasAppliedJointAxisAPI(self, jointPrim: Usd.Prim, jointType: int) -> bool:
    
        if not _isSupportedJointType(jointType):
            print("_hasAppliedJointAxisAPI called with illegal jointType")
            return false

        instanceToken = SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType]
    
        hasAPI = jointPrim.GetPrim().HasAPI(JOINT_AXIS_API, instanceToken)

        return hasAPI
    
    def _setJointAxisAttribute(self, jointPrim: Usd.Prim, jointType: int, attrId: int, attrVal: float) -> bool:
    
        if not _isSupportedJointType(jointType):
            print("_setJointAxisAttribute called with illegal jointType")
            return false

        if attrId != JOINT_AXIS_ATTR_MAX_VELOCITY and attrId != JOINT_AXIS_ATTR_ARMATURE and attrId != JOINT_AXIS_ATTR_STATIC_FRICTION and attrId != JOINT_AXIS_ATTR_DYNAMIC_FRICTION and attrId != JOINT_AXIS_ATTR_VISCOUS_FRICTION:
            print("_setJointAxisAttribute: must be valid attribute")
            return False
   
        attrName =  MAX_JOINT_VELOCITY_TO_TOKEN_MAP[jointType]
        if attrId == JOINT_AXIS_ATTR_ARMATURE:
            attrName =  ARMATURE_TO_TOKEN_MAP[jointType]
        elif attrId == JOINT_AXIS_ATTR_STATIC_FRICTION:
            attrName =  STATIC_FRICTION_EFFORT_TO_TOKEN_MAP[jointType]
        if attrId == JOINT_AXIS_ATTR_DYNAMIC_FRICTION:
            attrName =  DYNAMIC_FRICTION_EFFORT_TO_TOKEN_MAP[jointType]
        elif attrId == JOINT_AXIS_ATTR_VISCOUS_FRICTION:
            attrName =  VISCOUS_FRICTION_COEFFICIENT_TO_TOKEN_MAP[jointType]

        jointPrim.GetAttribute(attrName).Set(attrVal)

        return True
    
    def _getJointAxisAttribute(self, jointPrim: Usd.Prim, jointType: int, attrId: int) -> tuple[bool, float]:
    
        if not _isSupportedJointType(jointType):
            print("_getJointAxisAttribute called with illegal jointType")
            return false
    
        if attrId != JOINT_AXIS_ATTR_MAX_VELOCITY and attrId != JOINT_AXIS_ATTR_ARMATURE and attrId != JOINT_AXIS_ATTR_STATIC_FRICTION and attrId != JOINT_AXIS_ATTR_DYNAMIC_FRICTION and attrId != JOINT_AXIS_ATTR_VISCOUS_FRICTION:
            print("_getJointAxisAttribute: must be valid attribute")
            return False
    
        instanceToken = SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType]
    
        attrName =  ":maxJointVelocity"
        if attrId == JOINT_AXIS_ATTR_ARMATURE:
            attrName = ":armature"
        elif attrId == JOINT_AXIS_ATTR_STATIC_FRICTION:
            attrName = ":staticFrictionEffort"
        if attrId == JOINT_AXIS_ATTR_DYNAMIC_FRICTION:
            attrName = ":dynamicFrictionEffort"
        elif attrId == JOINT_AXIS_ATTR_VISCOUS_FRICTION:
            attrName = ":viscousFrictionCoefficient"
    
        attrVal = jointPrim.GetAttribute("physxJointAxis:" + instanceToken + attrName).Get()

        return (True, attrVal)

    def _scalePositionToDegreesAsNecessary(self, jointType: int, f: float) -> float:
        return f*RAD_TO_DEG[jointType]
    
    def _scaleVelocityToDegreesAsNecessary(self, jointType: int, f: float) -> float:
        return f*RAD_TO_DEG[jointType]

    def _scaleStiffnessToDegreesAsNecessary(self, jointType: int, f: float) -> float:
        return f/RAD_TO_DEG[jointType]

    def _scaleDampingToDegreesAsNecessary(self, jointType: int, f: float) -> float:
        return f/RAD_TO_DEG[jointType];

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

        # Create a prismatic joint as the inbound joint of the child link.
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

        # configure the inbound joint of the child link with drive 
        jointDriveAPI = UsdPhysics.DriveAPI.Apply(childLinkInboundJointPrim, SUPPORTED_JOINT_TYPE_TO_TOKEN_MAP[jointType])

        return (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI)

    #######################################################################
    ## run tests
    #######################################################################

    def _run_test_physx_joint_api_max_velocity(self, stage: Usd.Stage, jointType: int) -> None:
    
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

        # create the prismatic articulation and configure it with a drive
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)

        # we don't actually need a drive for this test so just disable it
        driveStiffness = 0.0
        driveDamping = 0.0
        driveTargetPos = 0.0
        driveTargetVel = 0.0
        jointDriveAPI.GetStiffnessAttr().Set(self._scaleStiffnessToDegreesAsNecessary(jointType, driveStiffness))
        jointDriveAPI.GetDampingAttr().Set(self._scaleDampingToDegreesAsNecessary(jointType, driveDamping))
        jointDriveAPI.GetTargetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, driveTargetPos))
        jointDriveAPI.GetTargetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, driveTargetVel))

        # create the PhysxJointAPI 
        physxJointAPI = PhysxSchema.PhysxJointAPI.Apply(childLinkInboundJointPrim)
       
        # Case 1
        # apply both apis and test that PhysxJointAxisAPI takes precedence when it is applied
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 2.5
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_MAX_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, 1.0))
        physxJointAPI.GetMaxJointVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, 2.0))
        self._simulate_one_frame_with_prep()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case1", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)

        # Case 2
        # remove PhysxJointAxisAPI and test that PhysxJointAPI still works as expected.
        self._removeJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 2.5
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 2.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case2", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)

        # Case 3
        # reapply PhysxJointAxisAPI
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 2.5
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_MAX_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, 1.0))
        self._simulate_one_frame()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case3", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)

        # Case 4
        # remove PhysxJointAxisAPI and test that PhysxJointAPI still works as expected.
        self._removeJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 2.5
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 2.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case4", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)

        # Case 5
        # With PhysxJointAxisAPI removed, test that PhysxJointAPI can be modified 
        jointStartPos = 0.0
        jointStartVel = 3.5
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        physxJointAPI.GetMaxJointVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, 3.0))
        self._simulate_one_frame()
        expectedJointSpeed = 3.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case4", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)

        # Case 6
        # reapply PhysxJointAxisAPI
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 2.5
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_MAX_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, 1.0))
        self._simulate_one_frame()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case6", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)

        # Case 7
        # With PhysxJointAxisAPI applied, modify max joint velocity and test it works as expected.
        jointStartPos = 0.0
        jointStartVel = 2.5
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_MAX_VELOCITY, self._scaleVelocityToDegreesAsNecessary(jointType, 0.5))
        self._simulate_one_frame()
        expectedJointSpeed = 0.5
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case7", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.0001)


    def _run_test_physx_joint_api_armature(self, stage: Usd.Stage, jointType: int) -> None:
    
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

        # create the prismatic articulation and configure it with a drive
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)

        # we apply force with a drive of stiffness * dx = 1000
        # 1000/(1 + 9) gives acceleration of 100 and velocity 1, while 1000/(1 + 99) gives acceleration of 10 and velocity 0.1
        driveStiffness = 1000.0
        driveDamping = 0.0
        driveTargetPos = 1.0
        driveTargetVel = 0.0
        jointDriveAPI.GetStiffnessAttr().Set(self._scaleStiffnessToDegreesAsNecessary(jointType, driveStiffness))
        jointDriveAPI.GetDampingAttr().Set(self._scaleDampingToDegreesAsNecessary(jointType, driveDamping))
        jointDriveAPI.GetTargetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, driveTargetPos))
        jointDriveAPI.GetTargetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, driveTargetVel))

        # create the PhysxJointAPI 
        physxJointAPI = PhysxSchema.PhysxJointAPI.Apply(childLinkInboundJointPrim)

        # Spherical joints do not behave the same as prismatic/revolute joints for reasons unknown.
        # This issue is tracked in NVBug 5199927
        permittedErrorLargeArmature = [0.002, 0.002, 0.002, 0.002, 0.002, 0.002, 0.01, 0.01, 0.01]
        permittedErrorSmallArmature = [0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.1, 0.1, 0.1]

        # Case 1
        # apply both apis and test that PhysxJointAxisAPI takes precedence
        # Expected acceleration is (1000/(1 + 9))
        # Expected velocity is (1000/(1 + 9))* 0.01 = 1.0
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_ARMATURE, 9.0)
        physxJointAPI.GetArmatureAttr().Set(99.0)
        self._simulate_one_frame_with_prep()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case1", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedErrorSmallArmature[jointType])

        # Case 2
        # remove PhysxJointAxisAPI and test that PhysxJointAPI still works as expected.
        self._removeJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 0.1
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case2", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        #self.assertLess(error, permittedErrorLargeArmature[jointType])

        # Case 3
        # reapply PhysxJointAxisAPI
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_ARMATURE, 9.0)
        self._simulate_one_frame()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case3", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error,  permittedErrorSmallArmature[jointType])

        # Case 4
        # remove PhysxJointAxisAPI and test that PhysxJointAPI still works as expected.
        self._removeJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 0.1
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case4", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedErrorLargeArmature[jointType])

        # Case 5
        # With PhysxJointAxisAPI removed, test that PhysxJointAPI can be modified 
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        physxJointAPI.GetArmatureAttr().Set(1.0)
        self._simulate_one_frame()
        expectedJointSpeed = 5.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case5", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.5)

        # Case 6
        # reapply PhysxJointAxisAPI
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_ARMATURE, 9.0)
        self._simulate_one_frame()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case6", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedErrorSmallArmature[jointType])

        # Case 7
        # With PhysxJointAxisAPI applied, modify max joint velocity and test it works as expected.
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_ARMATURE, 99.0)
        self._simulate_one_frame()
        expectedJointSpeed = 0.1
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case7", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedErrorLargeArmature[jointType])


    def _run_test_physx_joint_api_static_friction(self, stage: Usd.Stage, jointType: int) -> None:
    
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

        # create the prismatic articulation and configure it with a drive
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)

        # we apply force with a drive of stiffness * dx = 100
        driveStiffness = 100.0
        driveDamping = 0.0
        driveTargetPos = 1.0
        driveTargetVel = 0.0
        jointDriveAPI.GetStiffnessAttr().Set(self._scaleStiffnessToDegreesAsNecessary(jointType, driveStiffness))
        jointDriveAPI.GetDampingAttr().Set(self._scaleDampingToDegreesAsNecessary(jointType, driveDamping))
        jointDriveAPI.GetTargetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, driveTargetPos))
        jointDriveAPI.GetTargetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, driveTargetVel))

        # create the PhysxJointAPI 
        physxJointAPI = PhysxSchema.PhysxJointAPI.Apply(childLinkInboundJointPrim)

        # Spherical joints do not behave the same as prismatic/revolute joints for reasons unknown.
        # This issue is tracked in nvbug 5199927
        permittedErrorSmallFriction = [0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.1, 0.1, 0.1]
       
        # Case 1
        # apply both apis and test that PhysxJointAxisAPI takes precedence
        # drive adds 100 force, static friction removes 100 force
        # Expected velocity is 0 + 100*dt - 100*dt = 0
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_STATIC_FRICTION, 100.0)
        physxJointAPI.GetJointFrictionAttr().Set(0.0)
        self._simulate_one_frame_with_prep()
        expectedJointSpeed = 0.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case1", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.02)

        # Case 2
        # remove PhysxJointAxisAPI and test that PhysxJointAPI still works as expected.
        # drive adds 100 force, static friction removes 0 force
        # expected vel = 0 + 100*dt - 0*dt = 1 
        self._removeJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 1
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case2", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedErrorSmallFriction[jointType])

        # Case 3
        # reapply PhysxJointAxisAPI
        # drive adds 100 force, static friction removes 100 force
        # Expected velocity is 0 + 100*dt - 100*dt = 0
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_STATIC_FRICTION, 100.0)
        self._simulate_one_frame()
        expectedJointSpeed = 0.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case3", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.02)

        # Case 4
        # remove PhysxJointAxisAPI and test that PhysxJointAPI still works as expected.
        # drive adds 100 force, static friction removes 0 force
        # expected vel = 0 + 100*dt - 0*dt = 1 
        self._removeJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case4", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedErrorSmallFriction[jointType])

        # Case 5
        # reapply PhysxJointAxisAPI
        # drive adds 100 force, static friction removes 100 force
        # Expected velocity is 0 + 100*dt - 100*dt = 0
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_STATIC_FRICTION, 100.0)
        self._simulate_one_frame()
        expectedJointSpeed = 0.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case5", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.02)

        # Case 6
        # With PhysxJointAxisAPI applied, modify max joint velocity and test it works as expected.
        # drive adds 100 force, static friction removes 0 force
        # expected vel = 0 + 100*dt - 0*dt = 1 
        jointStartPos = 0.0
        jointStartVel = 0.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_STATIC_FRICTION, 10.0)
        self._simulate_one_frame()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case6", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedErrorSmallFriction[jointType])

    def _run_test_physx_joint_api_dynamic_friction(self, stage: Usd.Stage, jointType: int) -> None:
    
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

        # create the prismatic articulation and configure it with a drive
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)

        # we apply force with a drive of stiffness * dx = 100
        driveStiffness = 100.0
        driveDamping = 0.0
        driveTargetPos = 1.0
        driveTargetVel = 0.0
        jointDriveAPI.GetStiffnessAttr().Set(self._scaleStiffnessToDegreesAsNecessary(jointType, driveStiffness))
        jointDriveAPI.GetDampingAttr().Set(self._scaleDampingToDegreesAsNecessary(jointType, driveDamping))
        jointDriveAPI.GetTargetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, driveTargetPos))
        jointDriveAPI.GetTargetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, driveTargetVel))

        # create the PhysxJointAPI 
        physxJointAPI = PhysxSchema.PhysxJointAPI.Apply(childLinkInboundJointPrim)

        # Spherical joints do not behave the same as prismatic/revolute joints for reasons unknown.
        # This issue is tracked in nvbug 5199927
        permittedError = [0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.1, 0.1, 0.1]
       
        # Case 1
        # apply both apis and test that PhysxJointAxisAPI takes precedence
        # drive adds 100 force, static friction does nothing, dynamic friction removes 100 force.
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 1.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_STATIC_FRICTION, 100.0)
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_DYNAMIC_FRICTION, 100.0)
        physxJointAPI.GetJointFrictionAttr().Set(0.0)
        self._simulate_one_frame_with_prep()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case1", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedError[jointType])

        # Case 2
        # remove PhysxJointAxisAPI and test that PhysxJointAPI still works as expected.
        # drive adds 100 force, static friction does nothing, dynamic friction does nothing.
        self._removeJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 1.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 2.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case2", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedError[jointType])

        # Case 3
        # reapply PhysxJointAxisAPI
        # drive adds 100 force, static friction does nothing, dynamic friction removes 100 force.
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 1.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_STATIC_FRICTION, 100.0)
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_DYNAMIC_FRICTION, 100.0)
        self._simulate_one_frame()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case3", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedError[jointType])

        # Case 4
        # remove PhysxJointAxisAPI and test that PhysxJointAPI still works as expected.
        # drive adds 100 force, static friction does nothing, dynamic friction does nothing.
        self._removeJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 1.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 2.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case4", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedError[jointType])

        # Case 5
        # reapply PhysxJointAxisAPI
        # drive adds 100 force, static friction does nothing, dynamic friction removes 100 force.
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 1.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_STATIC_FRICTION, 100.0)
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_DYNAMIC_FRICTION, 100.0)
        self._simulate_one_frame()
        expectedJointSpeed = 1.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case5", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, permittedError[jointType])

        # Case 6
        # With PhysxJointAxisAPI applied, modify max joint velocity and test it works as expected.
        # drive adds 100 force, static friction does nothing, dynamic friction does nothing.
        jointStartPos = 0.0
        jointStartVel = 1.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_DYNAMIC_FRICTION, 0.0)
        self._simulate_one_frame()
        expectedJointSpeed = 2.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case6", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error,permittedError[jointType])

    def _run_test_physx_joint_api_viscous_friction(self, stage: Usd.Stage, jointType: int) -> None:
    
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

        # create the prismatic articulation and configure it with a drive
        (artPrim, rootLinkPrim, childLinkPrim, childLinkInboundJointPrim, jointDriveAPI, jointStateAPI) = self._create_fixedbase_prismatic_or_revolute_or_spherical_with_drive(
            stage,
            jointType,
            nbPosIters, nbVelIters,
            rootLinkBoxSize,
            childLinkMass, childLinkInertia, childLinkBoxSize)

        # we apply force with a drive of stiffness * dx = 100
        driveStiffness = 100.0
        driveDamping = 0.0
        driveTargetPos = 1.0
        driveTargetVel = 0.0
        jointDriveAPI.GetStiffnessAttr().Set(self._scaleStiffnessToDegreesAsNecessary(jointType, driveStiffness))
        jointDriveAPI.GetDampingAttr().Set(self._scaleDampingToDegreesAsNecessary(jointType, driveDamping))
        jointDriveAPI.GetTargetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, driveTargetPos))
        jointDriveAPI.GetTargetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, driveTargetVel))

        # create the PhysxJointAPI 
        physxJointAPI = PhysxSchema.PhysxJointAPI.Apply(childLinkInboundJointPrim)
       
        # Case 1
        # apply both apis and test that PhysxJointAxisAPI takes precedence
        # Drive force 100, vel after drive force is 10, remove 50*vel*dt = 5, expected vel is 5
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 9.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_STATIC_FRICTION, 100.0)
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_VISCOUS_FRICTION, self._scaleDampingToDegreesAsNecessary(jointType, 50.0))
        physxJointAPI.GetJointFrictionAttr().Set(0.0)
        self._simulate_one_frame_with_prep()
        expectedJointSpeed = 5.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case1", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.1)

        # Case 2
        # remove PhysxJointAxisAPI and test that PhysxJointAPI still works as expected.
        # Drive force 100, vel after drive force is 10, remove 0*vel*dt = 5, expected vel is 10
        self._removeJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 9.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 10.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case2", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.2)

        # Case 3
        # reapply PhysxJointAxisAPI
        # Drive force 100, vel after drive force is 10, remove 50*vel*dt = 5, expected vel is 5
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 9.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_STATIC_FRICTION, 100.0)
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_VISCOUS_FRICTION, self._scaleDampingToDegreesAsNecessary(jointType, 50.0))
        physxJointAPI.GetJointFrictionAttr().Set(0.0)
        self._simulate_one_frame()
        expectedJointSpeed = 5.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case3", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.1)

        # Case 4
        # remove PhysxJointAxisAPI and test that PhysxJointAPI still works as expected.
        # Drive force 100, vel after drive force is 10, remove 0*vel*dt = 5, expected vel is 10
        self._removeJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 9.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._simulate_one_frame()
        expectedJointSpeed = 10.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case4", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.2)

        # Case 5
        # reapply PhysxJointAxisAPI
        # Drive force 100, vel after drive force is 10, remove 50*vel*dt = 5, expected vel is 5
        self._applyJointAxisAPI(childLinkInboundJointPrim, jointType)
        jointStartPos = 0.0
        jointStartVel = 9.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_STATIC_FRICTION, 100.0)
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_VISCOUS_FRICTION, self._scaleDampingToDegreesAsNecessary(jointType, 50.0))
        physxJointAPI.GetJointFrictionAttr().Set(0.0)
        self._simulate_one_frame()
        expectedJointSpeed = 5.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case5", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.1)

        # Case 6
        # With PhysxJointAxisAPI applied, modify viscous friction and test it works as expected.
        # Drive force 100, vel after drive force is 10, remove 80*vel*dt = 8, expected vel is 2
        jointStartPos = 0.0
        jointStartVel = 9.0
        jointStateAPI.GetPositionAttr().Set(self._scalePositionToDegreesAsNecessary(jointType, jointStartPos))
        jointStateAPI.GetVelocityAttr().Set(self._scaleVelocityToDegreesAsNecessary(jointType, jointStartVel))
        self._setJointAxisAttribute(childLinkInboundJointPrim, jointType, JOINT_AXIS_ATTR_VISCOUS_FRICTION, self._scaleDampingToDegreesAsNecessary(jointType, 80.0))
        self._simulate_one_frame()
        expectedJointSpeed = 2.0
        measuredJointSpeed = self._scaleVelocityToRadiansAsNecessary(jointType, jointStateAPI.GetVelocityAttr().Get())
        error = abs(expectedJointSpeed - measuredJointSpeed)
        #print("case6", f"{expectedJointSpeed:.5f}", f"{measuredJointSpeed:.5f}")
        self.assertLess(error, 0.2)

    
    async def _run_test_apply_physx_joint_api_warnings(self, jointType: int,):

        for i in range(2):
            if i == 0:
                setupAsArticulation = True
            else:
                setupAsArticulation = False
            setupAsArticulation = False
            await self._setup_stage_and_bodies(setupAsArticulation = setupAsArticulation)
            if jointType == JOINT_TYPE_PRISMATIC_X or jointType == JOINT_TYPE_PRISMATIC_Y or jointType == JOINT_TYPE_PRISMATIC_Z:
               type = "prismatic"
            else:
               type = "revolute"

            self._setup_joint(type, axis = "X")
            mass = 1.0
            dt = 1.0 / 60.0

            massAPI = UsdPhysics.MassAPI(self._dynamic_box)
            massAPI.CreateMassAttr(mass)
            massAPI.CreateDensityAttr(0.0)

            self._applyJointAxisAPI(self._joint.GetPrim(), jointType)
            self._setJointAxisAttribute(self._joint.GetPrim(), jointType, JOINT_AXIS_ATTR_MAX_VELOCITY, 0.0)  
            
            if setupAsArticulation:
                 with utils.AssertNoMessages(self): 
                     self.step(1, dt)
            else:

                message = "PhysxJointAxisAPI is supported only for joints that are part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: /World/"+type+"Joint"
                with utils.ExpectMessage(self, message):
                    self.step(1, dt)
                message = "Please ensure that the joint is part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: /World/"+type+"Joint"
                with utils.ExpectMessage(self, message):
                    self._applyJointAxisAPI(self._joint.GetPrim(), jointType)
                    self._setJointAxisAttribute(self._joint.GetPrim(), jointType, JOINT_AXIS_ATTR_MAX_VELOCITY, 0.0) 
                    self.step(1, dt)
    #######################################################################
    ## the tests
    #######################################################################

    async def test_apply_physx_joint_api_maxjointvelocity_revolute_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_max_velocity(stage, JOINT_TYPE_REVOLUTE_X)

    async def test_apply_physx_joint_api_maxjointvelocity_revolute_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_max_velocity(stage, JOINT_TYPE_REVOLUTE_Y)

    async def test_apply_physx_joint_api_maxjointvelocity_revolute_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_max_velocity(stage, JOINT_TYPE_REVOLUTE_Z)

    async def test_apply_physx_joint_api_maxjointvelocity_prismatic_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_max_velocity(stage, JOINT_TYPE_PRISMATIC_X)

    async def test_apply_physx_joint_api_maxjointvelocity_prismatic_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_max_velocity(stage, JOINT_TYPE_PRISMATIC_Y)

    async def test_apply_physx_joint_api_maxjointvelocity_prismatic_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_max_velocity(stage, JOINT_TYPE_PRISMATIC_Z)

    async def test_apply_physx_joint_api_maxjointvelocity_spherical_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_max_velocity(stage, JOINT_TYPE_SPHERICAL_ROTX)

    async def test_apply_physx_joint_api_maxjointvelocity_spherical_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_max_velocity(stage, JOINT_TYPE_SPHERICAL_ROTY)

    async def test_apply_physx_joint_api_maxjointvelocity_spherical_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_max_velocity(stage, JOINT_TYPE_SPHERICAL_ROTZ)

    #################################################

    async def test_apply_physx_joint_api_armature_revolute_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_armature(stage, JOINT_TYPE_REVOLUTE_X)
    
    async def test_apply_physx_joint_api_armature_revolute_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_armature(stage, JOINT_TYPE_REVOLUTE_Y)
    
    async def test_apply_physx_joint_api_armature_revolute_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_armature(stage, JOINT_TYPE_REVOLUTE_Z)
    
    async def test_apply_physx_joint_api_armature_prismatic_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_armature(stage, JOINT_TYPE_PRISMATIC_X)
    
    async def test_apply_physx_joint_api_armature_prismatic_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_armature(stage, JOINT_TYPE_PRISMATIC_Y)
    
    async def test_apply_physx_joint_api_armature_prismatic_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_armature(stage, JOINT_TYPE_PRISMATIC_Z)

    async def test_apply_physx_joint_api_armature_spherical_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_armature(stage, JOINT_TYPE_SPHERICAL_ROTX)
    
    async def test_apply_physx_joint_api_armature_spherical_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_armature(stage, JOINT_TYPE_SPHERICAL_ROTY)
    
    async def test_apply_physx_joint_api_armature_spherical_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_armature(stage, JOINT_TYPE_SPHERICAL_ROTZ)

    #################################################

    async def test_apply_physx_joint_api_static_friction_revolute_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_static_friction(stage, JOINT_TYPE_REVOLUTE_X)
    
    async def test_apply_physx_joint_api_static_friction_revolute_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_static_friction(stage, JOINT_TYPE_REVOLUTE_Y)
    
    async def test_apply_physx_joint_api_static_friction_revolute_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_static_friction(stage, JOINT_TYPE_REVOLUTE_Z)
    
    async def test_apply_physx_joint_api_static_friction_prismatic_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_static_friction(stage, JOINT_TYPE_PRISMATIC_X)
    
    async def test_apply_physx_joint_api_static_friction_prismatic_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_static_friction(stage, JOINT_TYPE_PRISMATIC_Y)
    
    async def test_apply_physx_joint_api_static_friction_prismatic_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_static_friction(stage, JOINT_TYPE_PRISMATIC_Z)

    async def test_apply_physx_joint_api_static_friction_spherical_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_static_friction(stage, JOINT_TYPE_SPHERICAL_ROTX)
    
    async def test_apply_physx_joint_api_static_friction_spherical_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_static_friction(stage, JOINT_TYPE_SPHERICAL_ROTY)
    
    async def test_apply_physx_joint_api_static_friction_spherical_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_static_friction(stage, JOINT_TYPE_SPHERICAL_ROTZ)


    #################################################

    async def test_apply_physx_joint_api_dynamic_friction_revolute_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_dynamic_friction(stage, JOINT_TYPE_REVOLUTE_X)
    
    async def test_apply_physx_joint_api_dynamic_friction_revolute_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_dynamic_friction(stage, JOINT_TYPE_REVOLUTE_Y)
    
    async def test_apply_physx_joint_api_dynamic_friction_revolute_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_dynamic_friction(stage, JOINT_TYPE_REVOLUTE_Z)
    
    async def test_apply_physx_joint_api_dynamic_friction_prismatic_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_dynamic_friction(stage, JOINT_TYPE_PRISMATIC_X)
    
    async def test_apply_physx_joint_api_dynamic_friction_prismatic_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_dynamic_friction(stage, JOINT_TYPE_PRISMATIC_Y)
    
    async def test_apply_physx_joint_api_dynamic_friction_prismatic_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_dynamic_friction(stage, JOINT_TYPE_PRISMATIC_Z)

    async def test_apply_physx_joint_api_dynamic_friction_spherical_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_dynamic_friction(stage, JOINT_TYPE_SPHERICAL_ROTX)
    
    async def test_apply_physx_joint_api_dynamic_friction_spherical_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_dynamic_friction(stage, JOINT_TYPE_SPHERICAL_ROTY)
    
    async def test_apply_physx_joint_api_dynamic_friction_spherical_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_dynamic_friction(stage, JOINT_TYPE_SPHERICAL_ROTZ)

    #################################################

    async def test_apply_physx_joint_api_viscous_friction_revolute_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_viscous_friction(stage, JOINT_TYPE_REVOLUTE_X)
    
    async def test_apply_physx_joint_api_viscous_friction_revolute_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_viscous_friction(stage, JOINT_TYPE_REVOLUTE_Y)
    
    async def test_apply_physx_joint_api_viscous_friction_revolute_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_viscous_friction(stage, JOINT_TYPE_REVOLUTE_Z)
    
    async def test_apply_physx_joint_api_viscous_friction_prismatic_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_viscous_friction(stage, JOINT_TYPE_PRISMATIC_X)
    
    async def test_apply_physx_joint_api_viscous_friction_prismatic_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_viscous_friction(stage, JOINT_TYPE_PRISMATIC_Y)
    
    async def test_apply_physx_joint_api_viscous_friction_prismatic_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_viscous_friction(stage, JOINT_TYPE_PRISMATIC_Z)

    async def test_apply_physx_joint_api_viscous_friction_spherical_x(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_viscous_friction(stage, JOINT_TYPE_SPHERICAL_ROTX)
    
    async def test_apply_physx_joint_api_viscous_friction_spherical_y(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_viscous_friction(stage, JOINT_TYPE_SPHERICAL_ROTY)
    
    async def test_apply_physx_joint_api_viscous_friction_spherical_z(self):
        stage = await self.new_stage()
        self._run_test_physx_joint_api_viscous_friction(stage, JOINT_TYPE_SPHERICAL_ROTZ)

    #################################################

    async def test_apply_physx_joint_api_revolute_joint_warnings(self):
        await self._run_test_apply_physx_joint_api_warnings(JOINT_TYPE_REVOLUTE_X)
    
    async def test_apply_physx_joint_api_prismatic_joint_warnings(self):
        await self._run_test_apply_physx_joint_api_warnings(JOINT_TYPE_PRISMATIC_X)
