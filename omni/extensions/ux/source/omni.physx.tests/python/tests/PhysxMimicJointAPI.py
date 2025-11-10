# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import typing
import carb
import omni.physx.scripts.utils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physx import get_physx_simulation_interface, get_physx_replicator_interface
from omni.physxtests import utils
from pxr import Usd, Gf, Sdf, UsdGeom, UsdPhysics, UsdUtils, PhysxSchema
import math
from omni.physx.bindings._physx import (
    MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTX,
    MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTY,
    MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTZ,
    MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTX,
    MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTY,
    MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTZ)


JOINT_TYPE_PRISMATIC = 0
JOINT_TYPE_REVOLUTE = 1
JOINT_TYPE_SPHERICAL = 2

JOINT_AXIS_X = 0
JOINT_AXIS_Y = 1
JOINT_AXIS_Z = 2

JOINT_AXIS_TO_TOKEN_MAP = ["X", "Y", "Z"]


TEST_GEARING = 0
TEST_OFFSET = 1
TEST_REFERENCE_JOINT = 2

TEST_MSG_EXCLUDE_FROM_ARTICULATION_TARGET = 0
TEST_MSG_EXCLUDE_FROM_ARTICULATION_REFERENCE = 1
TEST_MSG_REFERENCE_JOINT_AXIS = 2
TEST_MSG_MULTIPLE_MIMIC_ON_1DOF_JOINT = 3
TEST_MSG_NO_REFERENCE_JOINT = 4
TEST_MSG_MULTIPLE_REFERENCE_JOINTS = 5
TEST_MSG_INVALID_REFERENCE_JOINT_PATH = 6
TEST_MSG_INVALID_REFERENCE_JOINT_PRIM_TYPE = 7
TEST_MSG_SAME_AXIS = 8
TEST_MSG_WRAPPED_JOINT_LIMIT_TARGET = 9
TEST_MSG_WRAPPED_JOINT_LIMIT_REFERENCE = 10
TEST_MSG_WRONG_JOINT_TYPE = 11
TEST_MSG_NOT_ALL_LINEAR_DOFS_LOCKED = 12
TEST_MSG_REFERENCE_AXIS_LOCKED = 13
TEST_MSG_DIFFERENT_ARTICULATIONS = 14

FLT_MAX = 3.402823e+38


class PhysxMimicJointAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    async def new_stage(self):
        self._clean_up()
        await super().new_stage(attach_stage=False)

        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self._stage, 1.0)

        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.y)

        return self._stage

    def _get_time_step(self):
        return 1.0 / 60.0

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

    def _get_quat_axis_angle_degree(self, quat: Gf.Quatf, axis: int) -> float:
        rotAngleRadians = 2.0 * math.asin(quat.GetImaginary()[axis])
        rotAngleDegree = rotAngleRadians * 180.0 / math.pi
        return rotAngleDegree

    def _get_joint_orient(self, jointPrim: Usd.Prim, orient0: Gf.Quatf, orient1: Gf.Quatf) -> Gf.Quatf:

        jointAPI = UsdPhysics.Joint(jointPrim)

        localOrient0 = jointAPI.GetLocalRot0Attr().Get()
        localOrient1 = jointAPI.GetLocalRot1Attr().Get()

        jointToWorld0 = (orient0 * localOrient0).GetNormalized()
        jointToWorld1 = (orient1 * localOrient1).GetNormalized()

        joint1to0 = (jointToWorld0.GetInverse() * jointToWorld1).GetNormalized()  # joint orientation in joint0 frame

        return joint1to0

    def _get_joint_pos(self, jointPrim: Usd.Prim, pos0: Gf.Vec3d, pos1: Gf.Vec3d) -> Gf.Vec3f:

        # note: assumes all orientations being identity

        jointAPI = UsdPhysics.Joint(jointPrim)

        localPos0 = jointAPI.GetLocalPos0Attr().Get()
        localPos1 = jointAPI.GetLocalPos1Attr().Get()

        jointToWorld0 = Gf.Vec3f(pos0) + localPos0
        jointToWorld1 = Gf.Vec3f(pos1) + localPos1

        joint1to0 = (-jointToWorld0 + jointToWorld1)  # joint position in joint0 frame

        return joint1to0

    def _get_translate(self, prim: Usd.Prim) -> Gf.Vec3f:
        return prim.GetAttribute("xformOp:translate").Get()

    def _set_translate(self, prim: Usd.Prim, pos: Gf.Vec3f) -> None:
        prim.GetAttribute("xformOp:translate").Set(pos)

    def _get_orient(self, prim: Usd.Prim) -> Gf.Quatf:
        return prim.GetAttribute("xformOp:orient").Get()

    def _set_orient(self, prim: Usd.Prim, orient: Gf.Quatf) -> None:
        prim.GetAttribute("xformOp:orient").Set(orient)

    def _get_lin_velocity(self, prim: Usd.Prim) -> Gf.Vec3f:
        bodyAPI = UsdPhysics.RigidBodyAPI(prim)
        return bodyAPI.GetVelocityAttr().Get()

    def _get_ang_velocity(self, prim: Usd.Prim) -> Gf.Vec3f:
        bodyAPI = UsdPhysics.RigidBodyAPI(prim)
        return bodyAPI.GetAngularVelocityAttr().Get()

    def _get_joint_angle(self, stage: Usd.Stage, jointPrim: Usd.Prim, axis: int) -> float:
        jointAPI = UsdPhysics.Joint(jointPrim)

        targets = jointAPI.GetBody0Rel().GetTargets()
        if (len(targets) > 0):
            prim0 = stage.GetPrimAtPath(targets[0])
            orient0 = self._get_orient(prim0)
        else:
            orient0 = jointAPI.GetLocalRot0Attr().Get()

        targets = jointAPI.GetBody1Rel().GetTargets()
        if (len(targets) > 0):
            prim1 = stage.GetPrimAtPath(targets[0])
            orient1 = self._get_orient(prim1)
        else:
            orient1 = jointAPI.GetLocalRot1Attr().Get()

        jointOrient = self._get_joint_orient(jointPrim, orient0, orient1);

        return self._get_quat_axis_angle_degree(jointOrient, axis)

    def _create_scene(self, stage: Usd.Stage, path: str, gravityMagn: float = 9.81) -> UsdPhysics.Scene:
        sceneAPI = UsdPhysics.Scene.Define(stage, path)
        sceneAPI.GetGravityMagnitudeAttr().Set(gravityMagn)

        return sceneAPI

    def _create_scene_with_gravity(self, stage: Usd.Stage, gravityMagn: float = 9.81) -> UsdPhysics.Scene:
        scenePath = str(stage.GetDefaultPrim().GetPath()) + "/Scene"
        return self._create_scene(stage, scenePath, gravityMagn)

    def _create_zero_gravity_scene(self, stage: Usd.Stage) -> UsdPhysics.Scene:
        return self._create_scene_with_gravity(stage, 0.0)

    def _create_body(
        self,
        stage: Usd.Stage,
        path: str,
        size: typing.Union[Gf.Vec3f, None] = Gf.Vec3f(1.0),
        position: Gf.Vec3f = Gf.Vec3f(0.0),
        orientation: Gf.Quatf = Gf.Quatf(1.0, 0.0, 0.0, 0.0),
        mass: float = 1.0,
        lin_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
        ang_velocity: Gf.Vec3f = Gf.Vec3f(0.0),
        ) -> Usd.Prim:

        bodyXform = UsdGeom.Xform.Define(stage, path)
        physicsUtils._add_transformation(bodyXform, position, orientation)

        bodyPrim = bodyXform.GetPrim()

        bodyAPI = UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        bodyAPI.CreateVelocityAttr().Set(lin_velocity)
        bodyAPI.CreateAngularVelocityAttr().Set(ang_velocity)

        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
        massAPI.GetMassAttr().Set(mass)
        massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(mass))

        physxBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(bodyPrim)
        physxBodyAPI.GetMaxAngularVelocityAttr().Set(1.0e10)
        physxBodyAPI.GetLinearDampingAttr().Set(0.0)
        physxBodyAPI.GetAngularDampingAttr().Set(0.0)
        physxBodyAPI.GetSleepThresholdAttr().Set(0.0)
        physxBodyAPI.GetStabilizationThresholdAttr().Set(0.0)
        physxBodyAPI.GetCfmScaleAttr().Set(0.0)

        if (size is not None):
            physicsUtils.add_box(stage, path + "/Geom", size)

        return bodyPrim

    def _create_articulation(self, stage: Usd.Stage, path: str, useFixBase: bool = True,
        nbPosIters: int = 4, nbVelIters: int = 1, position: Gf.Vec3f = Gf.Vec3f(0.0)) -> tuple[Usd.Prim, Usd.Prim]:

        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        lengthScale = 1.0 / metersPerUnit

        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        massScale = 1.0 / kilogramsPerUnit

        rootLink = self._create_body(stage, path, Gf.Vec3f(lengthScale * 0.001), position, mass = massScale * 1.0)

        if useFixBase:
            fixedJoint = UsdPhysics.FixedJoint.Define(stage, path + "/InboundJoint")
            fixedJoint.GetBody1Rel().AddTarget(path)
            artPrim = fixedJoint.GetPrim()
        else:
            artPrim = rootLink

        UsdPhysics.ArticulationRootAPI.Apply(artPrim)

        physxArtAPI = PhysxSchema.PhysxArticulationAPI.Apply(artPrim)
        physxArtAPI.GetSolverPositionIterationCountAttr().Set(nbPosIters)
        physxArtAPI.GetSolverVelocityIterationCountAttr().Set(nbVelIters)
        physxArtAPI.GetSleepThresholdAttr().Set(0.0)
        physxArtAPI.GetStabilizationThresholdAttr().Set(0.0)
        physxArtAPI.GetEnabledSelfCollisionsAttr().Set(False)

        return (artPrim, rootLink)

    def _create_joint(self, stage: Usd.Stage, path: str,
        jointType: int, axis: int,
        body0Path: str, body1Path: str,
        localPos0: Gf.Vec3f = Gf.Vec3f(0.0), localRot0: Gf.Quatf = Gf.Quatf(1.0, 0.0, 0.0, 0.0),
        localPos1: Gf.Vec3f = Gf.Vec3f(0.0), localRot1: Gf.Quatf = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        ) -> typing.Union[UsdPhysics.PrismaticJoint, UsdPhysics.RevoluteJoint, None]:

        if (jointType == JOINT_TYPE_PRISMATIC):
            joint = UsdPhysics.PrismaticJoint.Define(stage, path)
            joint.GetAxisAttr().Set(JOINT_AXIS_TO_TOKEN_MAP[axis])

        elif (jointType == JOINT_TYPE_REVOLUTE):
            joint = UsdPhysics.RevoluteJoint.Define(stage, path)
            joint.GetAxisAttr().Set(JOINT_AXIS_TO_TOKEN_MAP[axis])

            joint.GetLowerLimitAttr().Set(-1.0e10)
            joint.GetUpperLimitAttr().Set(1.0e10)

        elif (jointType == JOINT_TYPE_SPHERICAL):
            joint = UsdPhysics.Joint.Define(stage, path)

            jointPrim = joint.GetPrim()

            limit = UsdPhysics.LimitAPI.Apply(jointPrim, UsdPhysics.Tokens.transX)
            limit.GetLowAttr().Set(1.0)
            limit.GetHighAttr().Set(0.0)

            limit = UsdPhysics.LimitAPI.Apply(jointPrim, UsdPhysics.Tokens.transY)
            limit.GetLowAttr().Set(1.0)
            limit.GetHighAttr().Set(0.0)

            limit = UsdPhysics.LimitAPI.Apply(jointPrim, UsdPhysics.Tokens.transZ)
            limit.GetLowAttr().Set(1.0)
            limit.GetHighAttr().Set(0.0)

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

    def _create_simple_prismatic_setup(self, stage: Usd.Stage, useFixBase: bool = True,
        axis: int = JOINT_AXIS_X, nbPosIters: int = 4, nbVelIters: int = 1
        ) -> tuple[Usd.Prim, Usd.Prim, Usd.Prim, Usd.Prim, Usd.Prim, Usd.Prim]:

        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        lengthScale = 1.0 / metersPerUnit

        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        massScale = 1.0 / kilogramsPerUnit

        rootPath = str(stage.GetDefaultPrim().GetPath())
        artRootPath = rootPath + "/RootLink"

        linkBoxHalfExtent = Gf.Vec3f(lengthScale * 0.1)

        linkAPos = Gf.Vec3f(lengthScale * -1.0, 0.0, 0.0)
        linkBPos = Gf.Vec3f(lengthScale * 1.0, 0.0, 0.0)

        (artPrim, rootLink) = self._create_articulation(stage, artRootPath, useFixBase, nbPosIters, nbVelIters)

        rootMass = 2.0 * massScale
        massAPI = UsdPhysics.MassAPI(rootLink)
        massAPI.GetMassAttr().Set(rootMass)
        massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(rootMass))

        linkMass = 1.0 * massScale

        linkAPath = rootPath + "/LinkA"
        linkA = self._create_body(stage, linkAPath, 2.0 * linkBoxHalfExtent, linkAPos, mass = linkMass)

        linkAJointPath = linkAPath + "/InboundJoint"
        linkAJoint = self._create_joint(stage, linkAJointPath, JOINT_TYPE_PRISMATIC, axis, artRootPath, linkAPath,
            localPos0 = linkAPos)

        linkBPath = rootPath + "/LinkB"
        linkB = self._create_body(stage, linkBPath, 2.0 * linkBoxHalfExtent, linkBPos, mass = linkMass)

        linkBJointPath = linkBPath + "/InboundJoint"
        linkBJoint = self._create_joint(stage, linkBJointPath, JOINT_TYPE_PRISMATIC, axis, artRootPath, linkBPath,
            localPos0 = linkBPos)

        return (artPrim, rootLink, linkA, linkAJoint.GetPrim(), linkB, linkBJoint.GetPrim())

    def _create_simple_revolute_setup(self, stage: Usd.Stage, useFixBase: bool = True,
        axis: int = JOINT_AXIS_X, nbPosIters: int = 4, nbVelIters: int = 1
        ) -> tuple[Usd.Prim, Usd.Prim, Usd.Prim, Usd.Prim, Usd.Prim, Usd.Prim]:

        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        lengthScale = 1.0 / metersPerUnit

        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        massScale = 1.0 / kilogramsPerUnit

        rootPath = str(stage.GetDefaultPrim().GetPath())
        artRootPath = rootPath + "/RootLink"

        linkBoxHalfExtent = Gf.Vec3f(lengthScale * 0.1)

        linkAPos = Gf.Vec3f(lengthScale * -1.0, 0.0, 0.0)
        linkBPos = Gf.Vec3f(lengthScale * 1.0, 0.0, 0.0)

        startAngle = (30.0 * math.pi) / 180.0
        startAngleHalf = 0.5 * startAngle

        quatDirPart = Gf.Vec3f(0.0)

        quatDirPart[axis] = math.sin(startAngleHalf)
        linkAOrient = Gf.Quatf(math.cos(startAngleHalf), quatDirPart[0], quatDirPart[1], quatDirPart[2])

        quatDirPart[axis] = math.sin(-startAngleHalf)
        linkBOrient = Gf.Quatf(math.cos(-startAngleHalf), quatDirPart[0], quatDirPart[1], quatDirPart[2])

        (artPrim, rootLink) = self._create_articulation(stage, artRootPath, useFixBase, nbPosIters, nbVelIters)

        rootMass = 2.0 * massScale
        massAPI = UsdPhysics.MassAPI(rootLink)
        massAPI.GetMassAttr().Set(rootMass)
        massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(rootMass))

        linkMass = 1.0 * massScale

        linkAPath = rootPath + "/LinkA"
        linkA = self._create_body(stage, linkAPath, 2.0 * linkBoxHalfExtent, linkAPos, linkAOrient, mass = linkMass)

        linkAJointPath = linkAPath + "/InboundJoint"
        linkAJoint = self._create_joint(stage, linkAJointPath, JOINT_TYPE_REVOLUTE, axis, artRootPath, linkAPath,
            localPos0 = linkAPos, localRot0 = linkAOrient)

        linkBPath = rootPath + "/LinkB"
        linkB = self._create_body(stage, linkBPath, 2.0 * linkBoxHalfExtent, linkBPos, linkBOrient, mass = linkMass)

        linkBJointPath = linkBPath + "/InboundJoint"
        linkBJoint = self._create_joint(stage, linkBJointPath, JOINT_TYPE_REVOLUTE, axis, artRootPath, linkBPath,
            localPos0 = linkBPos, localRot0 = linkBOrient)

        return (artPrim, rootLink, linkA, linkAJoint.GetPrim(), linkB, linkBJoint.GetPrim())

    def _create_simple_spherical_setup(self, stage: Usd.Stage, useFixBase: bool = True,
        nbPosIters: int = 4, nbVelIters: int = 1
        ) -> tuple[Usd.Prim, Usd.Prim, Usd.Prim, Usd.Prim]:

        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        lengthScale = 1.0 / metersPerUnit

        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        massScale = 1.0 / kilogramsPerUnit

        rootPath = str(stage.GetDefaultPrim().GetPath())
        artRootPath = rootPath + "/RootLink"

        linkBoxHalfExtent = Gf.Vec3f(lengthScale * 0.1, lengthScale * 0.5, lengthScale * 0.1)

        linkPos = Gf.Vec3f(0.0, lengthScale * -0.5, 0.0)

        (artPrim, rootLink) = self._create_articulation(stage, artRootPath, useFixBase, nbPosIters, nbVelIters)

        rootMass = 2.0 * massScale
        massAPI = UsdPhysics.MassAPI(rootLink)
        massAPI.GetMassAttr().Set(rootMass)
        massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(rootMass))

        linkMass = 1.0 * massScale

        linkPath = rootPath + "/Link"
        link = self._create_body(stage, linkPath, 2.0 * linkBoxHalfExtent, linkPos, mass = linkMass)

        jointPath = linkPath + "/InboundJoint"
        joint = self._create_joint(stage, jointPath, JOINT_TYPE_SPHERICAL, -1, artRootPath, linkPath,
            localPos1 = -linkPos)

        jointPrim = joint.GetPrim()

        limit = UsdPhysics.LimitAPI.Apply(jointPrim, UsdPhysics.Tokens.rotY)
        limit.GetLowAttr().Set(1.0)
        limit.GetHighAttr().Set(0.0)

        return (artPrim, rootLink, link, jointPrim)


    #
    # Ensure mimic joint connects the position of two prismatic joints as expected
    #
    async def test_prismatic_simple(self):
        axisList = [JOINT_AXIS_X, JOINT_AXIS_Y, JOINT_AXIS_Z]

        for axis in axisList:
            stage = await self.new_stage()

            nbPosIters = 16
            nbVelIters = 1
            useFixBase = True

            gearing = 1.0
            offset = 0.0

            targetPosition = -0.1

            self._create_zero_gravity_scene(stage)

            (artPrim, rootLink, linkA, linkAJoint, linkB, linkBJoint) = self._create_simple_prismatic_setup(
                stage, useFixBase, axis, nbPosIters, nbVelIters)

            nonsenseDofMap = [UsdPhysics.Tokens.rotX, UsdPhysics.Tokens.rotY, UsdPhysics.Tokens.rotZ]
            # when using a prismatic or revolute joint, the degree of freedom attribute should be ignored.
            # The test will use a map that makes no sense (rot for prismatic) to check that it does not
            # matter in this scenario.

            mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(linkBJoint, nonsenseDofMap[axis])
            mimicJointAPI.GetReferenceJointRel().AddTarget(linkAJoint.GetPath())
            mimicJointAPI.GetReferenceJointAxisAttr().Set(nonsenseDofMap[axis])
            mimicJointAPI.GetGearingAttr().Set(gearing)
            mimicJointAPI.GetOffsetAttr().Set(offset)

            driveAPI = UsdPhysics.DriveAPI.Apply(linkAJoint, UsdPhysics.Tokens.linear)
            driveAPI.GetTargetPositionAttr().Set(targetPosition)
            driveAPI.GetDampingAttr().Set(0.0)
            driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

            self._simulate_one_frame_with_prep()

            rootLinkPos = self._get_translate(rootLink)
            linkAPos = self._get_translate(linkA)
            linkBPos = self._get_translate(linkB)

            linkAVel = self._get_lin_velocity(linkA)
            linkBVel = self._get_lin_velocity(linkB)

            posErrTolerance = 0.001
            velErrTolerance = 0.001

            linkAJointPos = self._get_joint_pos(linkAJoint, rootLinkPos, linkAPos)[axis]

            self.assertAlmostEqual(linkAJointPos, targetPosition, delta=posErrTolerance)

            self.assertAlmostEqual(linkAPos[axis], -linkBPos[axis], delta=posErrTolerance)
            self.assertAlmostEqual(linkAVel[axis], -linkBVel[axis], delta=velErrTolerance)


    #
    # Ensure mimic joint connects the position of two revolute joints as expected
    #
    async def test_revolute_simple(self):
        axisList = [JOINT_AXIS_X, JOINT_AXIS_Y, JOINT_AXIS_Z]

        for axis in axisList:
            stage = await self.new_stage()

            nbPosIters = 16
            nbVelIters = 1
            useFixBase = True

            gearing = 1.0
            offset = 0.0

            targetAngleDegree = 15.0

            self._create_zero_gravity_scene(stage)

            (artPrim, rootLink, linkA, linkAJoint, linkB, linkBJoint) = self._create_simple_revolute_setup(
                stage, useFixBase, axis, nbPosIters, nbVelIters)

            nonsenseDofMap = [UsdPhysics.Tokens.rotX, UsdPhysics.Tokens.rotY, UsdPhysics.Tokens.rotZ]
            # when using a prismatic or revolute joint, the degree of freedom attribute should be ignored.
            # The test will use a map that makes no sense (rot for prismatic) to check that it does not
            # matter in this scenario.

            mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(linkBJoint, nonsenseDofMap[axis])
            mimicJointAPI.GetReferenceJointRel().AddTarget(linkAJoint.GetPath())
            mimicJointAPI.GetReferenceJointAxisAttr().Set(nonsenseDofMap[axis])
            mimicJointAPI.GetGearingAttr().Set(gearing)
            mimicJointAPI.GetOffsetAttr().Set(offset)

            driveAPI = UsdPhysics.DriveAPI.Apply(linkAJoint, UsdPhysics.Tokens.angular)
            driveAPI.GetTargetPositionAttr().Set(targetAngleDegree)
            driveAPI.GetDampingAttr().Set(0.0)
            driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

            self._simulate_one_frame_with_prep()

            rootLinkOrient = self._get_orient(rootLink)
            linkAOrient = self._get_orient(linkA)
            linkBOrient = self._get_orient(linkB)

            linkAVel = self._get_ang_velocity(linkA)
            linkBVel = self._get_ang_velocity(linkB)

            linkAJoint1to0 = self._get_joint_orient(linkAJoint, rootLinkOrient, linkAOrient)

            linkAJointAngleDegree = self._get_quat_axis_angle_degree(linkAJoint1to0, axis)
            linkAAngleDegree = self._get_quat_axis_angle_degree(linkAOrient, axis)
            linkBAngleDegree = self._get_quat_axis_angle_degree(linkBOrient, axis)

            posErrTolerance = 0.05
            velErrTolerance = 0.001

            self.assertAlmostEqual(linkAJointAngleDegree, targetAngleDegree, delta=posErrTolerance)

            self.assertAlmostEqual(linkAAngleDegree, -linkBAngleDegree, delta=posErrTolerance)
            self.assertAlmostEqual(linkAVel[axis], -linkBVel[axis], delta=velErrTolerance)


    #
    # Ensure mimic joint works for a generic joint that has all linear degrees of freedom
    # locked. This test also makes sure that it works to have the same joint be target
    # and reference at the same time (if target and reference axis are different).
    # The angle of one joint axis is controlled by a position drive, a mimic joint tries
    # to mirror that angle along another axis of the same joint.
    #
    async def test_spherical_simple(self):
        stage = await self.new_stage()

        nbPosIters = 16
        nbVelIters = 1
        useFixBase = True

        gearing = 1.0
        offset = 0.0

        targetAngleDegree = 15.0

        self._create_zero_gravity_scene(stage)

        (artPrim, rootLink, link, joint) = self._create_simple_spherical_setup(
            stage, useFixBase, nbPosIters, nbVelIters)

        mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(joint, UsdPhysics.Tokens.rotX)
        mimicJointAPI.GetReferenceJointRel().AddTarget(joint.GetPath())
        mimicJointAPI.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotZ)
        mimicJointAPI.GetGearingAttr().Set(gearing)
        mimicJointAPI.GetOffsetAttr().Set(offset)

        driveAPI = UsdPhysics.DriveAPI.Apply(joint, UsdPhysics.Tokens.rotZ)
        driveAPI.GetTargetPositionAttr().Set(targetAngleDegree)
        driveAPI.GetDampingAttr().Set(0.0)
        driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

        jointStateXAPI = PhysxSchema.JointStateAPI.Apply(joint, UsdPhysics.Tokens.rotX)
        jointStateZAPI = PhysxSchema.JointStateAPI.Apply(joint, UsdPhysics.Tokens.rotZ)

        self._simulate_one_frame_with_prep()

        linkVel = self._get_ang_velocity(link)

        angleZDegree = jointStateZAPI.GetPositionAttr().Get()
        angleXDegree = jointStateXAPI.GetPositionAttr().Get()

        posErrTolerance = 0.15  # not sure why this large a tolerance is needed (vs. revolute joint test)
        velErrTolerance = 0.001

        self.assertAlmostEqual(angleZDegree, targetAngleDegree, delta=posErrTolerance)

        self.assertAlmostEqual(angleZDegree, -angleXDegree, delta=posErrTolerance)
        self.assertAlmostEqual(linkVel[2], -linkVel[0], delta=velErrTolerance)


    async def _test_attribute_changes(self, testAttributeType: int) -> None:
        stage = await self.new_stage()

        nbPosIters = 8
        nbVelIters = 1
        useFixBase = True

        gearing = 1.0
        offset = 0.0

        axis = JOINT_AXIS_X

        targetPosition = -0.1

        self._create_zero_gravity_scene(stage)

        (artPrim, rootLink, linkA, linkAJoint, linkB, linkBJoint) = self._create_simple_prismatic_setup(
            stage, useFixBase, axis, nbPosIters, nbVelIters)

        mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(linkBJoint, UsdPhysics.Tokens.rotX)
        mimicJointAPI.GetReferenceJointRel().AddTarget(linkAJoint.GetPath())
        mimicJointAPI.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotX)
        mimicJointAPI.GetGearingAttr().Set(gearing)
        mimicJointAPI.GetOffsetAttr().Set(offset)

        driveAPI = UsdPhysics.DriveAPI.Apply(linkAJoint, UsdPhysics.Tokens.linear)
        driveAPI.GetTargetPositionAttr().Set(targetPosition)
        driveAPI.GetDampingAttr().Set(0.0)
        driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

        if (testAttributeType == TEST_REFERENCE_JOINT):
            rootPath = str(stage.GetDefaultPrim().GetPath())
            linkCPos = Gf.Vec3f(0.0, 0.0, 1.0)
            linkCPath = rootPath + "/LinkC"
            linkC = self._create_body(stage, linkCPath, Gf.Vec3f(0.2), linkCPos, mass = 1.0)

            linkCJointPath = linkCPath + "/InboundJoint"
            linkCJoint = self._create_joint(stage, linkCJointPath, JOINT_TYPE_PRISMATIC, axis, str(rootLink.GetPath()), linkCPath,
                localPos0 = linkCPos).GetPrim()

            otherTargetPosition = targetPosition * 2.0

            otherDriveAPI = UsdPhysics.DriveAPI.Apply(linkCJoint, UsdPhysics.Tokens.linear)
            otherDriveAPI.GetTargetPositionAttr().Set(otherTargetPosition)
            otherDriveAPI.GetDampingAttr().Set(0.0)
            otherDriveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

        self._simulate_one_frame_with_prep()

        rootLinkPos = self._get_translate(rootLink)
        linkAPos = self._get_translate(linkA)

        posErrTolerance = 0.001

        linkAJointPos = self._get_joint_pos(linkAJoint, rootLinkPos, linkAPos)[axis]

        self.assertAlmostEqual(linkAJointPos, targetPosition, delta=posErrTolerance)

        if (testAttributeType == TEST_GEARING):
            gearing = 0.5
            mimicJointAPI.GetGearingAttr().Set(gearing)

        elif (testAttributeType == TEST_OFFSET):
            offset = 0.03
            mimicJointAPI.GetOffsetAttr().Set(offset)

        elif (testAttributeType == TEST_REFERENCE_JOINT):
            mimicJointAPI.GetReferenceJointRel().SetTargets([linkCJoint.GetPath()])

        self._simulate_one_frame()

        rootLinkPos = self._get_translate(rootLink)
        linkBPos = self._get_translate(linkB)

        linkBJointPos = self._get_joint_pos(linkBJoint, rootLinkPos, linkBPos)[axis]

        if (testAttributeType == TEST_GEARING):
            self.assertAlmostEqual(linkBJointPos, (-gearing * targetPosition) - offset, delta=posErrTolerance)

        elif (testAttributeType == TEST_OFFSET):
            self.assertAlmostEqual(linkBJointPos, (-gearing * targetPosition) - offset, delta=posErrTolerance)

        elif (testAttributeType == TEST_REFERENCE_JOINT):
            self.assertAlmostEqual(linkBJointPos, (-gearing * otherTargetPosition) - offset, delta=posErrTolerance)


    #
    # Ensure that changing the "gearing" attribute on a mimic joint has the expected effect
    #
    async def test_gearing_change(self):
        await self._test_attribute_changes(TEST_GEARING)


    #
    # Ensure that changing the "offset" attribute on a mimic joint has the expected effect
    #
    async def test_offset_change(self):
        await self._test_attribute_changes(TEST_OFFSET)


    #
    # Ensure that changing the "referenceJoint" attribute on a mimic joint has the expected effect
    #
    async def test_reference_joint_change(self):
        await self._test_attribute_changes(TEST_REFERENCE_JOINT)


    #
    # Ensure that removing the mimic joint API (or deleting the prim) after the stage has been attached
    # works as expected
    #
    async def test_remove_api_or_delete(self):
        REMOVE_API = 0
        DELETE_PRIM = 1

        operationList = [REMOVE_API, DELETE_PRIM]

        for operation in operationList:
            stage = await self.new_stage()

            nbPosIters = 16
            nbVelIters = 1
            useFixBase = True

            gearing = 1.0
            offset = 0.0

            axis = JOINT_AXIS_X

            targetPosition = -0.1

            self._create_zero_gravity_scene(stage)

            (artPrim, rootLink, linkA, linkAJoint, linkB, linkBJoint) = self._create_simple_prismatic_setup(
                stage, useFixBase, axis, nbPosIters, nbVelIters)

            mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(linkBJoint, UsdPhysics.Tokens.rotX)
            mimicJointAPI.GetReferenceJointRel().AddTarget(linkAJoint.GetPath())
            mimicJointAPI.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotX)
            mimicJointAPI.GetGearingAttr().Set(gearing)
            mimicJointAPI.GetOffsetAttr().Set(offset)

            driveAPI = UsdPhysics.DriveAPI.Apply(linkAJoint, UsdPhysics.Tokens.linear)
            driveAPI.GetTargetPositionAttr().Set(targetPosition)
            driveAPI.GetDampingAttr().Set(0.0)
            driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

            jointStateAPI = PhysxSchema.JointStateAPI.Apply(linkBJoint, UsdPhysics.Tokens.linear)

            self._simulate_one_frame_with_prep()

            rootLinkPos = self._get_translate(rootLink)
            linkAPos = self._get_translate(linkA)

            posErrTolerance = 0.001

            linkAJointPos = self._get_joint_pos(linkAJoint, rootLinkPos, linkAPos)[axis]

            self.assertAlmostEqual(linkAJointPos, targetPosition, delta=posErrTolerance)

            jointStateAPI.GetVelocityAttr().Set(0.0)

            newTargetPosition = 0.4 * targetPosition
            driveAPI.GetTargetPositionAttr().Set(newTargetPosition)

            if (operation == REMOVE_API):
                linkBJointPosFrame1 = jointStateAPI.GetPositionAttr().Get()
                linkBJoint.RemoveAPI(PhysxSchema.PhysxMimicJointAPI, UsdPhysics.Tokens.rotX)

            elif (operation == DELETE_PRIM):
                linkBPosFrame1 = self._get_translate(linkB)
                stage.RemovePrim(linkBJoint.GetPath())

            else:
                self.assertTrue(False)

            self._simulate_one_frame()

            rootLinkPos = self._get_translate(rootLink)
            linkAPos = self._get_translate(linkA)

            linkAJointPos = self._get_joint_pos(linkAJoint, rootLinkPos, linkAPos)[axis]

            self.assertAlmostEqual(linkAJointPos, newTargetPosition, delta=posErrTolerance)

            # the mimic joint has been removed -> changing the position drive target on linkA's joint
            # should not impact linkB's joint any longer (and remain unchanged)
            if (operation == REMOVE_API):
                linkBJointPosFrame2 = jointStateAPI.GetPositionAttr().Get()

                self.assertAlmostEqual(linkBJointPosFrame1, linkBJointPosFrame2, delta=posErrTolerance)

            elif (operation == DELETE_PRIM):
                linkBPosFrame2 = self._get_translate(linkB)

                self.assertAlmostEqual(linkBPosFrame1[axis], linkBPosFrame2[axis], delta=posErrTolerance)


    #
    # Ensure that adding the mimic joint API after the stage has been attached works as expected
    #
    async def test_add_api(self):
        stage = await self.new_stage()

        nbPosIters = 16
        nbVelIters = 1
        useFixBase = True

        gearing = 1.0
        offset = 0.0

        axis = JOINT_AXIS_X

        targetPosition = -0.1

        self._create_zero_gravity_scene(stage)

        (artPrim, rootLink, linkA, linkAJoint, linkB, linkBJoint) = self._create_simple_prismatic_setup(
            stage, useFixBase, axis, nbPosIters, nbVelIters)

        driveAPI = UsdPhysics.DriveAPI.Apply(linkAJoint, UsdPhysics.Tokens.linear)
        driveAPI.GetTargetPositionAttr().Set(targetPosition)
        driveAPI.GetDampingAttr().Set(0.0)
        driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

        jointStateAPI = PhysxSchema.JointStateAPI.Apply(linkBJoint, UsdPhysics.Tokens.linear)

        linkBJointPosFrame0 = jointStateAPI.GetPositionAttr().Get()

        self._simulate_one_frame_with_prep()

        rootLinkPos = self._get_translate(rootLink)
        linkAPos = self._get_translate(linkA)

        posErrTolerance = 0.001

        linkAJointPos = self._get_joint_pos(linkAJoint, rootLinkPos, linkAPos)[axis]

        self.assertAlmostEqual(linkAJointPos, targetPosition, delta=posErrTolerance)

        linkBJointPosFrame1 = jointStateAPI.GetPositionAttr().Get()

        # no mimic joint yet -> linkB's joint position should stay the same
        self.assertAlmostEqual(linkBJointPosFrame0, linkBJointPosFrame1, delta=posErrTolerance)

        mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(linkBJoint, UsdPhysics.Tokens.rotX)
        mimicJointAPI.GetReferenceJointRel().AddTarget(linkAJoint.GetPath())
        mimicJointAPI.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotX)
        mimicJointAPI.GetGearingAttr().Set(gearing)
        mimicJointAPI.GetOffsetAttr().Set(offset)

        self._simulate_one_frame()

        rootLinkPos = self._get_translate(rootLink)
        linkAPos = self._get_translate(linkA)

        linkAJointPos = self._get_joint_pos(linkAJoint, rootLinkPos, linkAPos)[axis]

        self.assertAlmostEqual(linkAJointPos, targetPosition, delta=posErrTolerance)

        linkBJointPosFrame2 = jointStateAPI.GetPositionAttr().Get()

        self.assertAlmostEqual(linkAJointPos, (-gearing * linkBJointPosFrame2), delta=posErrTolerance)


    #
    # Ensure that combining linear with angular axes works as expected
    #
    async def test_rack_and_pinion(self):
        PINION_MIMIC = 0  # pinion is the mimic joint target and has the drive
        RACK_MIMIC = 1    # rack is the mimic joint target and has the drive

        operationList = [PINION_MIMIC, RACK_MIMIC]

        for operation in operationList:
            stage = await self.new_stage()

            nbVelIters = 1
            useFixBase = True

            pinionRadius = 0.2
            pinionBoxHalfExtentX = pinionRadius / math.sqrt(2.0)

            pinionOffset = 45.0
            rackOffset = 0.1

            if (operation == PINION_MIMIC):
                nbPosIters = 16
                gearing = -180.0 / (pinionRadius * math.pi)  # get from distance on circle to matching angle (in degrees)
                offset = -pinionOffset
            elif (operation == RACK_MIMIC):
                nbPosIters = 128
                gearing = -(pinionRadius * math.pi) / 180.0  # get from angle (in degrees) to distance on circle
                offset = -rackOffset
            else:
                self.assertTrue(False)

            pinionAxis = JOINT_AXIS_Z
            rackAxis = JOINT_AXIS_X

            angleDelta = 5.0
            targetAngle = pinionOffset + angleDelta

            posDelta = 0.02  # about 5.7 degrees on the pinion, given pinionRadius
            targetPos = rackOffset + posDelta

            self._create_zero_gravity_scene(stage)

            rootPath = str(stage.GetDefaultPrim().GetPath())
            artRootPath = rootPath + "/RootLink"

            rackHalfExtent = Gf.Vec3f(0.5, 0.1, 0.1)
            pinionBoxHalfExtent = Gf.Vec3f(pinionBoxHalfExtentX, pinionBoxHalfExtentX, rackHalfExtent[2])

            pinionLinkPos = Gf.Vec3f(0.0, pinionRadius, 0.0)

            if (operation == PINION_MIMIC):
                pinionOffsetHalfRadians = 0.5 * (pinionOffset * math.pi) / 180.0
                quatDirPart = Gf.Vec3f(0.0, 0.0, math.sin(pinionOffsetHalfRadians))
            
                pinionLinkOrient = Gf.Quatf(math.cos(pinionOffsetHalfRadians), quatDirPart[0], quatDirPart[1], quatDirPart[2])
            else:
                pinionLinkOrient = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

            rackLinkJointFrame = Gf.Vec3f(0.0, -rackHalfExtent[1], 0.0)
            if (operation == RACK_MIMIC):
                rackLinkPos = rackLinkJointFrame + Gf.Vec3f(posDelta, 0.0, 0.0)
            else:
                rackLinkPos = rackLinkJointFrame

            (artPrim, rootLink) = self._create_articulation(stage, artRootPath, useFixBase, nbPosIters, nbVelIters)

            rootMass = 2.0
            massAPI = UsdPhysics.MassAPI(rootLink)
            massAPI.GetMassAttr().Set(rootMass)
            massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(rootMass))

            linkMass = 1.0

            pinionLinkPath = rootPath + "/PinionLink"
            pinionLink = self._create_body(stage, pinionLinkPath, 2.0 * pinionBoxHalfExtent, pinionLinkPos, pinionLinkOrient,
                mass = linkMass)

            pinionJointPath = pinionLinkPath + "/InboundJoint"
            pinionJoint = self._create_joint(stage, pinionJointPath, JOINT_TYPE_REVOLUTE, pinionAxis, artRootPath, pinionLinkPath,
                localPos0 = pinionLinkPos).GetPrim()

            rackLinkPath = rootPath + "/RackLink"
            rackLink = self._create_body(stage, rackLinkPath, 2.0 * rackHalfExtent, rackLinkPos, mass = linkMass)

            rackJointPath = rackLinkPath + "/InboundJoint"
            rackJoint = self._create_joint(stage, rackJointPath, JOINT_TYPE_PRISMATIC, rackAxis, artRootPath, rackLinkPath,
                localPos0 = rackLinkJointFrame).GetPrim()

            if (operation == PINION_MIMIC):
                mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(pinionJoint, UsdPhysics.Tokens.rotX)
                mimicJointAPI.GetReferenceJointRel().AddTarget(rackJointPath)

                driveAPI = UsdPhysics.DriveAPI.Apply(pinionJoint, UsdPhysics.Tokens.angular)
                driveAPI.GetTargetPositionAttr().Set(targetAngle)

            elif (operation == RACK_MIMIC):
                mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(rackJoint, UsdPhysics.Tokens.rotX)
                mimicJointAPI.GetReferenceJointRel().AddTarget(pinionJointPath)

                driveAPI = UsdPhysics.DriveAPI.Apply(rackJoint, UsdPhysics.Tokens.linear)
                driveAPI.GetTargetPositionAttr().Set(targetPos)

            mimicJointAPI.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotX)
            mimicJointAPI.GetGearingAttr().Set(gearing)
            mimicJointAPI.GetOffsetAttr().Set(offset)

            driveAPI.GetDampingAttr().Set(0.0)
            driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

            pinionJointStateAPI = PhysxSchema.JointStateAPI.Apply(pinionJoint, UsdPhysics.Tokens.angular)
            rackJointStateAPI = PhysxSchema.JointStateAPI.Apply(rackJoint, UsdPhysics.Tokens.linear)

            self._simulate_one_frame_with_prep()

            rackLinkPos = self._get_translate(rackLink)

            posErrTolerance = 0.002

            pinionJointPos = pinionJointStateAPI.GetPositionAttr().Get()
            rackJointPos = rackJointStateAPI.GetPositionAttr().Get()

            if (operation == PINION_MIMIC):
                self.assertAlmostEqual(pinionJointPos, targetAngle, delta=posErrTolerance)
                self.assertAlmostEqual(pinionJointPos, (-gearing * rackJointPos) - offset, delta=posErrTolerance)

                epxectedPos = (angleDelta * math.pi / 180.0) * pinionRadius
                self.assertAlmostEqual(rackLinkPos[0], epxectedPos, delta=posErrTolerance)

            elif (operation == RACK_MIMIC):
                self.assertAlmostEqual(rackJointPos, targetPos, delta=posErrTolerance)
                self.assertAlmostEqual(rackJointPos, (-gearing * pinionJointPos) - offset, delta=posErrTolerance)


    #
    # Ensure that a two-link arm behaves as expected with a non-zero gravity and the outermost link being
    # heavy (compared to the parent link).
    #
    async def test_simple_arm_with_gravity(self):
        DRIVE_REFERENCE_JOINT = 0  # reference joint has the drive
        DRIVE_TARGET_JOINT = 1     # target joint has the drive

        operationList = [DRIVE_REFERENCE_JOINT, DRIVE_TARGET_JOINT]

        for operation in operationList:
            stage = await self.new_stage()

            nbVelIters = 1
            nbPosIters = 16
            useFixBase = True

            box0HalfExtent = Gf.Vec3f(0.1, 0.5, 0.1)
            box1HalfExtent = Gf.Vec3f(box0HalfExtent[1], box0HalfExtent[0], box0HalfExtent[0])

            gearing = 1.0
            offset = 0.0

            self._create_scene_with_gravity(stage, 9.81)

            rootPath = str(stage.GetDefaultPrim().GetPath())
            artRootPath = rootPath + "/RootLink"

            link0Pos = Gf.Vec3f(0.0, box0HalfExtent[1], 0.0)
            link1Pos = Gf.Vec3f(-box1HalfExtent[0], 2.0 * box0HalfExtent[1], 0.0)

            (artPrim, rootLink) = self._create_articulation(stage, artRootPath, useFixBase, nbPosIters, nbVelIters)

            rootMass = 50.0
            massAPI = UsdPhysics.MassAPI(rootLink)
            massAPI.GetMassAttr().Set(rootMass)
            massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(rootMass))

            link0Mass = 1.0
            link1Mass = 10.0 * link0Mass

            link0Path = rootPath + "/Link0"
            link0 = self._create_body(stage, link0Path, 2.0 * box0HalfExtent, link0Pos, mass = link0Mass)

            link0JointPath = link0Path + "/InboundJoint"
            link0Joint = self._create_joint(stage, link0JointPath, JOINT_TYPE_REVOLUTE, JOINT_AXIS_Z, artRootPath, link0Path,
                localPos1 = -link0Pos).GetPrim()

            link1Path = rootPath + "/Link1"
            link1 = self._create_body(stage, link1Path, 2.0 * box1HalfExtent, link1Pos, mass = link1Mass)

            link1JointPath = link1Path + "/InboundJoint"
            link1Joint = self._create_joint(stage, link1JointPath, JOINT_TYPE_REVOLUTE, JOINT_AXIS_Z, link0Path, link1Path,
                localPos0 = link0Pos, localPos1 = Gf.Vec3f(box1HalfExtent[0], 0.0, 0.0)).GetPrim()

            if (operation == DRIVE_REFERENCE_JOINT):
                targetAngle = 5.0
                driveAPI = UsdPhysics.DriveAPI.Apply(link0Joint, UsdPhysics.Tokens.angular)
                driveAPI.GetTargetPositionAttr().Set(targetAngle)

            elif (operation == DRIVE_TARGET_JOINT):
                targetAngle = -5.0
                driveAPI = UsdPhysics.DriveAPI.Apply(link1Joint, UsdPhysics.Tokens.angular)
                driveAPI.GetTargetPositionAttr().Set(targetAngle)

            else:
                self.assertTrue(False)

            mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(link1Joint, UsdPhysics.Tokens.rotZ)
            mimicJointAPI.GetReferenceJointRel().AddTarget(link0JointPath)
            mimicJointAPI.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotZ)
            mimicJointAPI.GetGearingAttr().Set(gearing)
            mimicJointAPI.GetOffsetAttr().Set(offset)

            driveAPI.GetDampingAttr().Set(0.0)
            driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

            link0JointStateAPI = PhysxSchema.JointStateAPI.Apply(link0Joint, UsdPhysics.Tokens.angular)
            link1JointStateAPI = PhysxSchema.JointStateAPI.Apply(link1Joint, UsdPhysics.Tokens.angular)

            self._simulate_one_frame_with_prep()

            posErrTolerance = 0.007

            link0JointPos = link0JointStateAPI.GetPositionAttr().Get()
            link1JointPos = link1JointStateAPI.GetPositionAttr().Get()

            if (operation == DRIVE_REFERENCE_JOINT):
                self.assertAlmostEqual(link0JointPos, targetAngle, delta=posErrTolerance)
            elif (operation == DRIVE_TARGET_JOINT):
                self.assertAlmostEqual(link1JointPos, targetAngle, delta=posErrTolerance)

            self.assertAlmostEqual(link0JointPos, -link1JointPos, delta=posErrTolerance)


    def _create_chain_scenario(self, stage: Usd.Stage, rootPath: str, linkPosShift: Gf.Vec3f = Gf.Vec3f(0.0),
        useFixBase: bool = True, nbPosIters: int = 4, nbVelIters: int = 1,
        gearing1: float = 0.5, gearing2: float = 4.0 / 5.0) -> tuple[Usd.Prim, Usd.Prim, Usd.Prim]:

        boxHalfExtent = Gf.Vec3f(0.1, 0.5, 0.1)

        offset = 0.0

        artRootPath = rootPath + "/RootLink"

        link0Pos = linkPosShift + Gf.Vec3f(0.0, boxHalfExtent[1], 0.0)
        link1Pos = linkPosShift + Gf.Vec3f(0.0, 3.0 * boxHalfExtent[1], 0.0)
        link2Pos = linkPosShift + Gf.Vec3f(0.0, 5.0 * boxHalfExtent[1], 0.0)

        (artPrim, rootLink) = self._create_articulation(stage, artRootPath, useFixBase, nbPosIters, nbVelIters, linkPosShift)

        rootMass = 2.0
        massAPI = UsdPhysics.MassAPI(rootLink)
        massAPI.GetMassAttr().Set(rootMass)
        massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(rootMass))

        linkMass = 1.0

        jointChildFrame = Gf.Vec3f(0.0, -boxHalfExtent[1], 0.0)

        link0Path = rootPath + "/Link0"
        link0 = self._create_body(stage, link0Path, 2.0 * boxHalfExtent, link0Pos, mass = linkMass)

        link0JointPath = link0Path + "/InboundJoint"
        link0Joint = self._create_joint(stage, link0JointPath, JOINT_TYPE_REVOLUTE, JOINT_AXIS_Z, artRootPath, link0Path,
            localPos1 = jointChildFrame).GetPrim()

        link1Path = rootPath + "/Link1"
        link1 = self._create_body(stage, link1Path, 2.0 * boxHalfExtent, link1Pos, mass = linkMass)

        link1JointPath = link1Path + "/InboundJoint"
        link1Joint = self._create_joint(stage, link1JointPath, JOINT_TYPE_REVOLUTE, JOINT_AXIS_Z, link0Path, link1Path,
            localPos0 = -jointChildFrame, localPos1 = jointChildFrame).GetPrim()

        link2Path = rootPath + "/Link2"
        link2 = self._create_body(stage, link2Path, 2.0 * boxHalfExtent, link2Pos, mass = linkMass)

        link2JointPath = link2Path + "/InboundJoint"
        link2Joint = self._create_joint(stage, link2JointPath, JOINT_TYPE_REVOLUTE, JOINT_AXIS_Z, link1Path, link2Path,
            localPos0 = -jointChildFrame, localPos1 = jointChildFrame).GetPrim()

        mimicJoint1API = PhysxSchema.PhysxMimicJointAPI.Apply(link1Joint, UsdPhysics.Tokens.rotZ)
        mimicJoint1API.GetReferenceJointRel().AddTarget(link0JointPath)
        mimicJoint1API.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotZ)
        mimicJoint1API.GetGearingAttr().Set(gearing1)
        mimicJoint1API.GetOffsetAttr().Set(offset)

        mimicJoint2API = PhysxSchema.PhysxMimicJointAPI.Apply(link2Joint, UsdPhysics.Tokens.rotZ)
        mimicJoint2API.GetReferenceJointRel().AddTarget(link1JointPath)
        mimicJoint2API.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotZ)
        mimicJoint2API.GetGearingAttr().Set(gearing2)
        mimicJoint2API.GetOffsetAttr().Set(offset)

        return (link0Joint, link1Joint, link2Joint)


    #
    # Ensure that a chain of three links {0, 1, 2} with two mimic joints {(1, 0), (2, 1)} works.
    #
    async def test_mimic_chain(self):
        stage = await self.new_stage()

        nbVelIters = 1
        nbPosIters = 64
        useFixBase = True

        gearing1 = 0.5
        gearing2 = 4.0 / 5.0

        self._create_zero_gravity_scene(stage)

        rootPath = str(stage.GetDefaultPrim().GetPath())

        (link0JointPrim, link1JointPrim, link2JointPrim) = self._create_chain_scenario(stage, rootPath, Gf.Vec3f(0.0),
            useFixBase, nbPosIters, nbVelIters, gearing1, gearing2)

        targetAngle = 5.0

        driveAPI = UsdPhysics.DriveAPI.Apply(link0JointPrim, UsdPhysics.Tokens.angular)
        driveAPI.GetTargetPositionAttr().Set(targetAngle)
        driveAPI.GetDampingAttr().Set(0.0)
        driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

        link0JointStateAPI = PhysxSchema.JointStateAPI.Apply(link0JointPrim, UsdPhysics.Tokens.angular)
        link1JointStateAPI = PhysxSchema.JointStateAPI.Apply(link1JointPrim, UsdPhysics.Tokens.angular)
        link2JointStateAPI = PhysxSchema.JointStateAPI.Apply(link2JointPrim, UsdPhysics.Tokens.angular)

        self._simulate_one_frame_with_prep()

        posErrTolerance = 0.001

        link0JointPos = link0JointStateAPI.GetPositionAttr().Get()
        link1JointPos = link1JointStateAPI.GetPositionAttr().Get()
        link2JointPos = link2JointStateAPI.GetPositionAttr().Get()

        self.assertAlmostEqual(link0JointPos, targetAngle, delta=posErrTolerance)
        self.assertAlmostEqual(link1JointPos, (-gearing1 * link0JointPos), delta=posErrTolerance)
        self.assertAlmostEqual(link2JointPos, (-gearing2 * link1JointPos), delta=posErrTolerance)


    def predictMimicJointComplianceOutcome(self, rAA: float, rAB: float, rBB: float, rBA: float, gearing: float, offset: float, nf: float, dr: float, q0Start: float, q1Start : float, q0dotStart : float, q1dotStart : float, dt: float) -> tuple:

        r = rAA + gearing*(rAB + rBA) + gearing*gearing*rBB

        # erp and cfm wtih zero compliance
        erp = 0.9   #Value for TGS/GPU
        cfm = 0.0

        # Override erp and cfm if there is compliance
        if nf > 0 and dr > 0:
            kp = nf * nf / r
            kd = 2.0 * nf * dr / r
            cfm = 1.0 / (dt*(dt * kp + kd))
            erp = (dt * kp) / (dt * kp + kd)

        # We can now compute lambda
        c = q0Start + gearing*q1Start + offset
        b = c*erp/dt
        Jv = q0dotStart + gearing*q1dotStart
        lambda0 = -(b +Jv)/(r + cfm)
        lambda1 = gearing*lambda0
        # We can now compute the predicted state after 1 sim step
        q0dotEndPredicted = q0dotStart + rAA*lambda0 + rAB*lambda1
        q1dotEndPredicted = q1dotStart + rBA*lambda0 + rBB*lambda1
        q0EndPredicted = q0Start + dt*q0dotEndPredicted
        q1EndPredicted = q1Start + dt*q1dotEndPredicted

        return q0dotEndPredicted, q1dotEndPredicted, q0EndPredicted, q1EndPredicted

    
    async def run_mimic_compliance_correctness_test(self, axis):

        stage = await self.new_stage()

        ######################################################
        # Declare all params for articulation and mimic joint
        ######################################################

        # Articulation and scene params
        dt = 0.01
        nbPosIters = 1
        nbVelIters = 0
        useFixBase = True

        # joint params
        jointParentFrame = Gf.Vec3f(0.0, 0.0, 0.0)
        jointChildFrames = [Gf.Vec3f(-4.0, 0.0, 0.0), Gf.Vec3f(0.0, -4.0, 0.0), Gf.Vec3f(0.0, 0.0, -4.0)]
        jointChildFrame = jointChildFrames[axis]

        # axis of joint
        jointAxes = [JOINT_AXIS_X, JOINT_AXIS_Y, JOINT_AXIS_Z]
        rotTokens = [UsdPhysics.Tokens.rotX, UsdPhysics.Tokens.rotY, UsdPhysics.Tokens.rotZ]
        naturalFrequencyTokens = [MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTX, MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTY, MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTZ]
        dampingRatioTokens = [MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTX, MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTY, MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTZ]

        # params of root link
        # we set moi = (rootMass, rootMass, rootMass)
        rootMass = 1.0

        # params of child links
        # we set moi = (rootMass, rootMass, rootMass)
        linkMass = 1.0

        # params of mimic joint
        gearing = 1.63
        offset = 0.5
 
        # start state of both joints
        q0Start = 1.0
        q1Start = 0.0
        q0dotStart = 35.9
        q1dotStart = 0.0

        # The tolerance of difference between predicted outcome and measured outcome
        errTolerance = 0.0001

        # We can pre-compute the response matrix of the mimic joint with knowledge
        # of joint frames and masses etc.
        rAA = 1.0
        rBB = 2.0
        rAB = -1.0
        rBA = -1.0

        ##########################################################
        # Create the scene and Articulation
        ##########################################################

        jointAxis = jointAxes[axis]
        rotToken = rotTokens[axis]
        naturalFrequencyToken = naturalFrequencyTokens[axis]
        dampingRatioToken = dampingRatioTokens[axis]

        physicsScene = self._create_zero_gravity_scene(stage)

        # create and configure the articulation and the root link
        rootPath = str(stage.GetDefaultPrim().GetPath())
        artRootPath = rootPath + "/RootLink"
        (artPrim, rootLink) = self._create_articulation(stage, artRootPath, useFixBase, nbPosIters, nbVelIters)
        massAPI = UsdPhysics.MassAPI(rootLink)
        massAPI.GetMassAttr().Set(rootMass)
        massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(rootMass))

        # configure the child of the root and its inbound joint
        link0Path = rootPath + "/Link0"
        link0 = self._create_body(stage, link0Path, mass = linkMass)
        link0JointPath = link0Path + "/InboundJoint"
        link0JointPrim = self._create_joint(stage, link0JointPath, JOINT_TYPE_PRISMATIC, jointAxis, artRootPath, link0Path, localPos0 = jointParentFrame, localPos1 = jointChildFrame).GetPrim()
        link0JointStateAPI = PhysxSchema.JointStateAPI.Apply(link0JointPrim, UsdPhysics.Tokens.linear)

        # configure the child of link0 and its inbound joint
        link1Path = rootPath + "/Link1"
        link1 = self._create_body(stage, link1Path, mass = linkMass)
        link1JointPath = link1Path + "/InboundJoint"
        link1JointPrim = self._create_joint(stage, link1JointPath, JOINT_TYPE_PRISMATIC, jointAxis, link0Path, link1Path, localPos0 = jointParentFrame, localPos1 = jointChildFrame).GetPrim()
        link1JointStateAPI = PhysxSchema.JointStateAPI.Apply(link1JointPrim, UsdPhysics.Tokens.linear)

        # configure the mimic joint
        mimicJoint1API = PhysxSchema.PhysxMimicJointAPI.Apply(link0JointPrim, rotToken)
        mimicJoint1API.GetReferenceJointRel().AddTarget(link1JointPath)
        mimicJoint1API.GetReferenceJointAxisAttr().Set(rotToken)
        mimicJoint1API.GetGearingAttr().Set(gearing)
        mimicJoint1API.GetOffsetAttr().Set(offset)

        # i==0: create the attributes with compliance, run sim, test against theoretical values.
        # i==1: change the attributes after a sim step to deactivate compliance, run sim, test against theoretical values.
        # i==2: change the attributes after a sim step to reactivate compliance, run sim, test against theoretical values.
        nfs = [1000.0, 0.0, 1000.0]
        drs = [1.5, 0.0, 1.5]

        for i in range(3):

            nf = nfs[i];
            dr = drs[i]

            # Set the start state of the joints
            link0JointStateAPI.GetPositionAttr().Set(q0Start)
            link1JointStateAPI.GetPositionAttr().Set(q1Start)
            link0JointStateAPI.GetVelocityAttr().Set(q0dotStart)
            link1JointStateAPI.GetVelocityAttr().Set(q1dotStart)

            # Set the mimic joint compliance
            if 0==i:
                link0JointPrim.CreateAttribute(naturalFrequencyToken, Sdf.ValueTypeNames.Float).Set(nf)
                link0JointPrim.CreateAttribute(dampingRatioToken, Sdf.ValueTypeNames.Float).Set(dr)
            else:
                link0JointPrim.GetAttribute(naturalFrequencyToken).Set(nf)
                link0JointPrim.GetAttribute(dampingRatioToken).Set(dr)

            # Compute the predicted outcome
            predicted = self.predictMimicJointComplianceOutcome(rAA, rAB, rBB, rBA, gearing, offset, nf, dr, q0Start, q1Start, q0dotStart, q1dotStart, dt)
            q0dotEndPredicted = predicted[0]
            q1dotEndPredicted = predicted[1]
            q0EndPredicted = predicted[2]
            q1EndPredicted = predicted[3]

            # Simulate one step
            if 0 == i:
                self._prepare_for_simulation()
            self._simulate_one_frame(dt)

            # Measure the joint states
            q0EndMeasured = link0JointStateAPI.GetPositionAttr().Get()
            q1EndMeasured = link1JointStateAPI.GetPositionAttr().Get()
            q0dotEndMeasured = link0JointStateAPI.GetVelocityAttr().Get()
            q1DotEndMeasured = link1JointStateAPI.GetVelocityAttr().Get()

            # Compare measured joint states with predicted outcome
            self.assertAlmostEqual(q0EndPredicted, q0EndMeasured, delta=errTolerance)
            self.assertAlmostEqual(q1EndPredicted, q1EndMeasured, delta=errTolerance)
            self.assertAlmostEqual(q0dotEndPredicted, q0dotEndMeasured, delta=errTolerance)
            self.assertAlmostEqual(q1dotEndPredicted, q1DotEndMeasured, delta=errTolerance)


    async def test_mimic_compliance_correctness_rotx(self):
        await self.run_mimic_compliance_correctness_test(0)

    async def test_mimic_compliance_correctness_roty(self):
        await self.run_mimic_compliance_correctness_test(1)

    async def test_mimic_compliance_correctness_rotz(self):
        await self.run_mimic_compliance_correctness_test(2)


    #
    # Ensure disabling the target or reference joint of a mimic joint works as expected
    #
    async def test_disabled_joint(self):
        START_WITH_TARGET_DISABLED = 0
        START_WITH_TARGET_ENABLED = 1
        START_WITH_REFERENCE_DISABLED = 2
        START_WITH_REFERENCE_ENABLED = 3

        axis = JOINT_AXIS_Z

        operationList = [START_WITH_TARGET_DISABLED, START_WITH_TARGET_ENABLED, START_WITH_REFERENCE_DISABLED, START_WITH_REFERENCE_ENABLED]

        for operation in operationList:
            stage = await self.new_stage()

            nbPosIters = 32
            nbVelIters = 1
            useFixBase = True

            gearing = 1.0
            offset = 0.0

            targetPosition = -0.1

            self._create_zero_gravity_scene(stage)

            (artPrim, rootLink, linkA, linkAJoint, linkB, linkBJoint) = self._create_simple_prismatic_setup(
                stage, useFixBase, axis, nbPosIters, nbVelIters)

            mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(linkBJoint, UsdPhysics.Tokens.rotX)
            mimicJointAPI.GetReferenceJointRel().AddTarget(linkAJoint.GetPath())
            mimicJointAPI.GetGearingAttr().Set(gearing)
            mimicJointAPI.GetOffsetAttr().Set(offset)

            driveAPI = UsdPhysics.DriveAPI.Apply(linkAJoint, UsdPhysics.Tokens.linear)
            driveAPI.GetTargetPositionAttr().Set(targetPosition)
            driveAPI.GetDampingAttr().Set(0.0)
            driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

            linkAJointAPI = UsdPhysics.Joint(linkAJoint)
            linkBJointAPI = UsdPhysics.Joint(linkBJoint)

            if (operation == START_WITH_TARGET_DISABLED):
                linkBJointAPI.GetJointEnabledAttr().Set(False)
            elif (operation == START_WITH_TARGET_ENABLED):
                linkBJointAPI.GetJointEnabledAttr().Set(True)
            elif (operation == START_WITH_REFERENCE_DISABLED):
                linkAJointAPI.GetJointEnabledAttr().Set(False)
            elif (operation == START_WITH_REFERENCE_ENABLED):
                linkAJointAPI.GetJointEnabledAttr().Set(True)
            else:
                self.assertTrue(False)

            self._simulate_one_frame_with_prep()

            rootLinkPos = self._get_translate(rootLink)
            linkAPos = self._get_translate(linkA)
            linkBPos = self._get_translate(linkB)

            linkAJointPos = self._get_joint_pos(linkAJoint, rootLinkPos, linkAPos)[axis]
            linkBJointPos = self._get_joint_pos(linkBJoint, rootLinkPos, linkBPos)[axis]

            posErrTolerance = 0.001
            velErrTolerance = 0.001

            if (operation == START_WITH_REFERENCE_DISABLED):
                self.assertAlmostEqual(linkAJointPos, 0.0, delta=posErrTolerance)
            else:
                self.assertAlmostEqual(linkAJointPos, targetPosition, delta=posErrTolerance)

            if (operation == START_WITH_TARGET_DISABLED):
                self.assertAlmostEqual(linkBJointPos, 0.0, delta=posErrTolerance)

                # note: enabling a joint after the stage has been attached seems to always create
                #       a none articulation joint. Thus, there is no expectation for the mimic joint
                #       to work
                message = "Usd Physics: failed to find internal joint object for PhysxMimicJointAPI at " + str(linkBJoint.GetPath()) + (". "
                    "Please ensure that the prim is a supported joint type and is part of an articulation.\n")

                with utils.ExpectMessage(self, message):
                    linkBJointAPI.GetJointEnabledAttr().Set(True)
                    self._simulate_one_frame()

            elif (operation == START_WITH_TARGET_ENABLED):
                self.assertAlmostEqual(linkBJointPos, -gearing * targetPosition, delta=posErrTolerance)

                oldTargetPosition = targetPosition
                targetPosition = 0.5 * targetPosition

                driveAPI.GetTargetPositionAttr().Set(targetPosition)

                linkBJointAPI.GetJointEnabledAttr().Set(False)
                self._simulate_one_frame()

            elif (operation == START_WITH_REFERENCE_DISABLED):
                self.assertAlmostEqual(linkBJointPos, 0.0, delta=posErrTolerance)

                linkAJointAPI.GetJointEnabledAttr().Set(True)
                self._simulate_one_frame()

            elif (operation == START_WITH_REFERENCE_ENABLED):
                self.assertAlmostEqual(linkBJointPos, -gearing * targetPosition, delta=posErrTolerance)

                oldTargetPosition = targetPosition
                targetPosition = 0.5 * targetPosition

                driveAPI.GetTargetPositionAttr().Set(targetPosition)

                linkAJointAPI.GetJointEnabledAttr().Set(False)
                self._simulate_one_frame()

            linkAPos = self._get_translate(linkA)
            linkBPos = self._get_translate(linkB)

            linkAJointPos = self._get_joint_pos(linkAJoint, rootLinkPos, linkAPos)[axis]
            linkBJointPos = self._get_joint_pos(linkBJoint, rootLinkPos, linkBPos)[axis]

            if (operation == START_WITH_REFERENCE_ENABLED):
                # would expect the old target position but it seems the joint is not really disabled
                #self.assertAlmostEqual(linkAJointPos, oldTargetPosition, delta=posErrTolerance)
                self.assertAlmostEqual(linkAJointPos, targetPosition, delta=posErrTolerance)
            else:
                self.assertAlmostEqual(linkAJointPos, targetPosition, delta=posErrTolerance)

            if (operation == START_WITH_TARGET_DISABLED):
                # see note at expected error message above
                self.assertAlmostEqual(linkBJointPos, 0.0, delta=posErrTolerance)

            elif (operation == START_WITH_TARGET_ENABLED):
                self.assertAlmostEqual(linkBJointPos, -gearing * oldTargetPosition, delta=posErrTolerance)

            elif (operation == START_WITH_REFERENCE_DISABLED):
                # the mimic target joint does not get re-parsed just because the reference joint is
                # re-parsed. As a consequence, the mimic joint will not work. Even if it was reparsed,
                # the reference joint would end up being a non articulation joint and as such not
                # supported by mimic joints.
                self.assertAlmostEqual(linkBJointPos, 0.0, delta=posErrTolerance)

            elif (operation == START_WITH_REFERENCE_ENABLED):
                self.assertAlmostEqual(linkBJointPos, -gearing * oldTargetPosition, delta=posErrTolerance)


    async def _test_error_messages(self, testMessageScenario: int) -> None:

        stage = await self.new_stage()

        nbPosIters = 16
        nbVelIters = 1
        useFixBase = True

        gearing = 1.0
        offset = 0.0

        axis = JOINT_AXIS_X

        targetPosition = -0.1

        self._create_zero_gravity_scene(stage)

        (artPrim, rootLink, linkA, linkAJoint, linkB, linkBJoint) = self._create_simple_prismatic_setup(
            stage, useFixBase, axis, nbPosIters, nbVelIters)

        mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(linkBJoint, UsdPhysics.Tokens.rotX)
        mimicJointAPI.GetReferenceJointRel().AddTarget(linkAJoint.GetPath())
        mimicJointAPI.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotX)
        mimicJointAPI.GetGearingAttr().Set(gearing)
        mimicJointAPI.GetOffsetAttr().Set(offset)

        driveAPI = UsdPhysics.DriveAPI.Apply(linkAJoint, UsdPhysics.Tokens.linear)
        driveAPI.GetTargetPositionAttr().Set(targetPosition)
        driveAPI.GetDampingAttr().Set(0.0)
        driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

        referenceJoint = UsdPhysics.Joint(linkAJoint);
        targetJoint = UsdPhysics.Joint(linkBJoint);

        targetJointStateAPI = PhysxSchema.JointStateAPI.Apply(linkBJoint, UsdPhysics.Tokens.linear)

        if (testMessageScenario == TEST_MSG_EXCLUDE_FROM_ARTICULATION_TARGET):
            targetJoint.GetExcludeFromArticulationAttr().Set(True)
            message = ("Usd Physics: PhysxMimicJointAPI is only supported on articulation joints. "
                f"The joint at {targetJoint.GetPath()} has \"excludeFromArticulation\" set to true.\n")

        elif (testMessageScenario == TEST_MSG_EXCLUDE_FROM_ARTICULATION_REFERENCE):
            referenceJoint.GetExcludeFromArticulationAttr().Set(True)
            message = ("Usd Physics: PhysxMimicJointAPI is only supported on articulation joints. "
                f"The mimic joint at {targetJoint.GetPath()} points to reference joint {referenceJoint.GetPath()} "
                "which has \"excludeFromArticulation\" set to true.\n")

        elif (testMessageScenario == TEST_MSG_REFERENCE_JOINT_AXIS):
            token = "bla"
            mimicJointAPI.GetReferenceJointAxisAttr().Set(token)
            message = (f"Usd Physics: PhysxMimicJointAPI at {targetJoint.GetPath()} has unsupported token \"{token}\" in attribute \"referenceJointAxis\". "
                "Supported tokens are: {rotX, rotY, rotZ}.\n")

        elif (testMessageScenario == TEST_MSG_MULTIPLE_MIMIC_ON_1DOF_JOINT):
            PhysxSchema.PhysxMimicJointAPI.Apply(linkBJoint, UsdPhysics.Tokens.rotY)
            message = (f"Usd Physics: PhysxMimicJointAPI at {targetJoint.GetPath()}: prismatic and revolute joints can only have one PhysxMimicJointAPI "
                "instance applied.\n")

        elif (testMessageScenario == TEST_MSG_NO_REFERENCE_JOINT):
            mimicJointAPI.GetReferenceJointRel().ClearTargets(True)
            message = (f"Usd Physics: PhysxMimicJointAPI at {targetJoint.GetPath()} must have exactly 1 \"referenceJoint\" relationship defined.\n")

        elif (testMessageScenario == TEST_MSG_MULTIPLE_REFERENCE_JOINTS):
            mimicJointAPI.GetReferenceJointRel().AddTarget(linkBJoint.GetPath())
            message = (f"Usd Physics: PhysxMimicJointAPI at {targetJoint.GetPath()} must have exactly 1 \"referenceJoint\" relationship defined.\n")

        elif (testMessageScenario == TEST_MSG_INVALID_REFERENCE_JOINT_PATH):
            fakePath = str(rootLink.GetPath()) + "/Bla"
            mimicJointAPI.GetReferenceJointRel().ClearTargets(True)
            mimicJointAPI.GetReferenceJointRel().AddTarget(fakePath)
            message = (f"Usd Physics: PhysxMimicJointAPI at {targetJoint.GetPath()} points to a non existing prim at {fakePath} "
                "in attribute \"referenceJoint\".\n")

        elif (testMessageScenario == TEST_MSG_INVALID_REFERENCE_JOINT_PRIM_TYPE):
            fakePrimPath = str(stage.GetDefaultPrim().GetPath()) + "/Bla"
            fakePrim = UsdGeom.Xform.Define(stage, fakePrimPath)
            mimicJointAPI.GetReferenceJointRel().ClearTargets(True)
            mimicJointAPI.GetReferenceJointRel().AddTarget(fakePrimPath)
            message = (f"Usd Physics: PhysxMimicJointAPI at {targetJoint.GetPath()} points to prim {fakePrimPath} "
                "that is not a UsdPhysicsJoint (see attribute \"referenceJoint\").\n")

        elif (testMessageScenario == TEST_MSG_WRONG_JOINT_TYPE):
            linkCPath = str(stage.GetDefaultPrim().GetPath()) + "/LinkC"
            linkC = self._create_body(stage, linkCPath, Gf.Vec3f(0.1), Gf.Vec3f(0.0), mass = 1.0)
            linkCJointPath = linkCPath + "/InboundJoint"
            linkCJoint = UsdPhysics.SphericalJoint.Define(stage, linkCJointPath)

            linkCJoint.GetBody0Rel().AddTarget(rootLink.GetPath())
            linkCJoint.GetBody1Rel().AddTarget(linkCPath)

            mimicJointAPI.GetReferenceJointRel().ClearTargets(True)
            mimicJointAPI.GetReferenceJointRel().AddTarget(linkCJointPath)

            message = (f"Usd Physics: the joint at prim {linkCJointPath} has a type that is not supported by the mimic joint feature. "
                "Please refer to the USD documentation for a list of supported joint types.\n")

        elif (testMessageScenario == TEST_MSG_NOT_ALL_LINEAR_DOFS_LOCKED):
            linkCPath = str(stage.GetDefaultPrim().GetPath()) + "/LinkC"
            linkC = self._create_body(stage, linkCPath, Gf.Vec3f(0.1), Gf.Vec3f(0.0), mass = 1.0)
            linkCJointPath = linkCPath + "/InboundJoint"
            linkCJoint = UsdPhysics.Joint.Define(stage, linkCJointPath)

            linkCJoint.GetBody0Rel().AddTarget(rootLink.GetPath())
            linkCJoint.GetBody1Rel().AddTarget(linkCPath)

            linkCJointPrim = linkCJoint.GetPrim()

            limit = UsdPhysics.LimitAPI.Apply(linkCJointPrim, UsdPhysics.Tokens.transX)
            limit.GetLowAttr().Set(1.0)
            limit.GetHighAttr().Set(0.0)

            limit = UsdPhysics.LimitAPI.Apply(linkCJointPrim, UsdPhysics.Tokens.transY)
            limit.GetLowAttr().Set(1.0)
            limit.GetHighAttr().Set(0.0)

            # do not lock transZ to trigger the error

            mimicJointAPI.GetReferenceJointRel().ClearTargets(True)
            mimicJointAPI.GetReferenceJointRel().AddTarget(linkCJointPath)

            message = ("Usd Physics: a generic joint needs all linear degrees of freedom locked to support the mimic joint feature. "
                f"The joint at prim {linkCJointPath} does not meet the requirement.\n")

        elif (testMessageScenario == TEST_MSG_REFERENCE_AXIS_LOCKED):
            linkCPath = str(stage.GetDefaultPrim().GetPath()) + "/LinkC"
            linkC = self._create_body(stage, linkCPath, Gf.Vec3f(0.1), Gf.Vec3f(0.0), mass = 1.0)
            linkCJointPath = linkCPath + "/InboundJoint"
            linkCJoint = UsdPhysics.Joint.Define(stage, linkCJointPath)

            linkCJoint.GetBody0Rel().AddTarget(rootLink.GetPath())
            linkCJoint.GetBody1Rel().AddTarget(linkCPath)

            linkCJointPrim = linkCJoint.GetPrim()

            limit = UsdPhysics.LimitAPI.Apply(linkCJointPrim, UsdPhysics.Tokens.transX)
            limit.GetLowAttr().Set(1.0)
            limit.GetHighAttr().Set(0.0)

            limit = UsdPhysics.LimitAPI.Apply(linkCJointPrim, UsdPhysics.Tokens.transY)
            limit.GetLowAttr().Set(1.0)
            limit.GetHighAttr().Set(0.0)

            limit = UsdPhysics.LimitAPI.Apply(linkCJointPrim, UsdPhysics.Tokens.transZ)
            limit.GetLowAttr().Set(1.0)
            limit.GetHighAttr().Set(0.0)

            limit = UsdPhysics.LimitAPI.Apply(linkCJointPrim, UsdPhysics.Tokens.rotX)
            limit.GetLowAttr().Set(1.0)
            limit.GetHighAttr().Set(0.0)

            mimicJointAPI.GetReferenceJointRel().ClearTargets(True)
            mimicJointAPI.GetReferenceJointRel().AddTarget(linkCJointPath)

            message = ("Usd Physics: the mimic joint feature needs a non locked degree of freedom. The joint "
                f"at prim {linkCJointPath} has the axis \"{UsdPhysics.Tokens.rotX}\" locked\n.")

        elif (testMessageScenario == TEST_MSG_DIFFERENT_ARTICULATIONS):
            rootPath = str(stage.GetDefaultPrim().GetPath())
            artRootPath = rootPath + "/Root2Link"

            linkCPos = Gf.Vec3f(-1.0, 10.0, 0.0)

            (art2Prim, root2Link) = self._create_articulation(stage, artRootPath, useFixBase, nbPosIters, nbVelIters)

            rootMass = 2.0
            massAPI = UsdPhysics.MassAPI(root2Link)
            massAPI.GetMassAttr().Set(rootMass)
            massAPI.GetDiagonalInertiaAttr().Set(Gf.Vec3f(rootMass))

            linkCPath = rootPath + "/LinkC"
            linkC = self._create_body(stage, linkCPath, 2.0 * Gf.Vec3f(0.1), linkCPos, mass = 1.0)

            linkCJointPath = linkCPath + "/InboundJoint"
            linkCJoint = self._create_joint(stage, linkCJointPath, JOINT_TYPE_PRISMATIC, axis, artRootPath, linkCPath,
                localPos0 = linkCPos)

            mimicJointAPI.GetReferenceJointRel().ClearTargets(True)
            mimicJointAPI.GetReferenceJointRel().AddTarget(linkCJointPath)

            message = (f"PhysX Mimic Joint: \"{targetJoint.GetPath()}\": the joints to connect are not part of the same articulation.\n")

        else:
            self.assertTrue(False)

        with utils.ExpectMessage(self, message):
            self._prepare_for_simulation()

        self._simulate_one_frame()

        posErrTolerance = 0.001

        targetJointPos = targetJointStateAPI.GetPositionAttr().Get()

        if (testMessageScenario != TEST_MSG_MULTIPLE_MIMIC_ON_1DOF_JOINT):
            # the type of errors checked here should result in the mimic joint not
            # being created and thus the mimic joint should keep its start position

            self.assertAlmostEqual(targetJointPos, 0.0, delta=posErrTolerance)

        else:

            self.assertAlmostEqual(targetJointPos, -targetPosition, delta=posErrTolerance)


    #
    # Ensure that a mimic joint on an non-articulation joint triggers an error
    #
    async def test_message_exclude_from_articulation_target_joint(self):
        await self._test_error_messages(TEST_MSG_EXCLUDE_FROM_ARTICULATION_TARGET)


    #
    # Ensure that a mimic joint on an non-articulation joint triggers an error
    #
    async def test_message_exclude_from_articulation_reference_joint(self):
        await self._test_error_messages(TEST_MSG_EXCLUDE_FROM_ARTICULATION_REFERENCE)


    #
    # Ensure that an unsupported token for the reference joint axis triggers an error
    #
    async def test_message_reference_joint_axis(self):
        await self._test_error_messages(TEST_MSG_REFERENCE_JOINT_AXIS)


    #
    # Ensure that more than one mimic joint on a prismatic triggers an error
    #
    async def test_message_multiple_mimic_on_1dof_joint(self):
        await self._test_error_messages(TEST_MSG_MULTIPLE_MIMIC_ON_1DOF_JOINT)


    #
    # Ensure that defining no reference joint triggers an error
    #
    async def test_message_no_reference_joint(self):
        await self._test_error_messages(TEST_MSG_NO_REFERENCE_JOINT)


    #
    # Ensure that defining multiple reference joints triggers an error
    #
    async def test_message_multiple_reference_joints(self):
        await self._test_error_messages(TEST_MSG_MULTIPLE_REFERENCE_JOINTS)


    #
    # Ensure that defining an invalid path for the reference joint triggers an error
    #
    async def test_message_invalid_reference_joint_path(self):
        await self._test_error_messages(TEST_MSG_INVALID_REFERENCE_JOINT_PATH)


    #
    # Ensure that defining a non-joint prim for the reference joint triggers an error
    #
    async def test_message_invalid_reference_joint_prim_type(self):
        await self._test_error_messages(TEST_MSG_INVALID_REFERENCE_JOINT_PRIM_TYPE)


    #
    # Ensure that defining a wrong joint type for the reference joint triggers an error
    #
    async def test_message_wrong_joint_type(self):
        await self._test_error_messages(TEST_MSG_WRONG_JOINT_TYPE)


    #
    # Ensure that any non-locked linear degree of freedom on a D6 triggers an error
    #
    async def test_message_not_all_linear_dofs_locked(self):
        await self._test_error_messages(TEST_MSG_NOT_ALL_LINEAR_DOFS_LOCKED)


    #
    # Ensure that a locked reference axis triggers an error
    #
    async def test_message_reference_axis_locked(self):
        await self._test_error_messages(TEST_MSG_REFERENCE_AXIS_LOCKED)


    #
    # Ensure that target and reference joint on different articulations triggers an error
    #
    async def test_message_different_articulations(self):
        await self._test_error_messages(TEST_MSG_DIFFERENT_ARTICULATIONS)


    async def _test_error_messages_spherical(self, testMessageScenario: int) -> None:
        stage = await self.new_stage()

        nbPosIters = 16
        nbVelIters = 1
        useFixBase = True

        gearing = 1.0
        offset = 0.0

        targetAngleDegree = 15.0

        self._create_zero_gravity_scene(stage)

        (artPrim, rootLink, link, joint) = self._create_simple_spherical_setup(
            stage, useFixBase, nbPosIters, nbVelIters)

        mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(joint, UsdPhysics.Tokens.rotX)
        mimicJointAPI.GetReferenceJointRel().AddTarget(joint.GetPath())
        mimicJointAPI.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotZ)
        mimicJointAPI.GetGearingAttr().Set(gearing)
        mimicJointAPI.GetOffsetAttr().Set(offset)

        driveAPI = UsdPhysics.DriveAPI.Apply(joint, UsdPhysics.Tokens.rotZ)
        driveAPI.GetTargetPositionAttr().Set(targetAngleDegree)
        driveAPI.GetDampingAttr().Set(0.0)
        driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

        jointStateXAPI = PhysxSchema.JointStateAPI.Apply(joint, UsdPhysics.Tokens.rotX)
        jointStateZAPI = PhysxSchema.JointStateAPI.Apply(joint, UsdPhysics.Tokens.rotZ)

        if (testMessageScenario == TEST_MSG_SAME_AXIS):
            mimicJointAPI.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotX)
            message = (f"Usd Physics: PhysxMimicJointAPI at {joint.GetPath()} has same joint for mimic and reference. "
                "The axes to operate on must not be the same in this case.\n")

        else:
            self.assertTrue(False)

        with utils.ExpectMessage(self, message):
            self._prepare_for_simulation()

        self._simulate_one_frame()

        posErrTolerance = 0.001

        targetJointPos = jointStateXAPI.GetPositionAttr().Get()

        # the type of errors checked here should result in the mimic joint not
        # being created and thus the mimic joint should keep its start position

        self.assertAlmostEqual(targetJointPos, 0.0, delta=posErrTolerance)


    #
    # Ensure that same axis for target and reference on a spherical joint triggers an error
    #
    async def test_message_same_axis_on_spherical(self):
        await self._test_error_messages_spherical(TEST_MSG_SAME_AXIS)


    async def _test_error_messages_revolute(self, testMessageScenario: int) -> None:

        stage = await self.new_stage()

        nbPosIters = 16
        nbVelIters = 1
        useFixBase = True

        gearing = 1.0
        offset = 0.0

        axis = JOINT_AXIS_Z

        targetAngleDegree = 15.0

        self._create_zero_gravity_scene(stage)

        (artPrim, rootLink, linkA, linkAJoint, linkB, linkBJoint) = self._create_simple_revolute_setup(
            stage, useFixBase, axis, nbPosIters, nbVelIters)

        mimicJointAPI = PhysxSchema.PhysxMimicJointAPI.Apply(linkBJoint, UsdPhysics.Tokens.rotX)
        mimicJointAPI.GetReferenceJointRel().AddTarget(linkAJoint.GetPath())
        mimicJointAPI.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotX)
        mimicJointAPI.GetGearingAttr().Set(gearing)
        mimicJointAPI.GetOffsetAttr().Set(offset)

        driveAPI = UsdPhysics.DriveAPI.Apply(linkAJoint, UsdPhysics.Tokens.angular)
        driveAPI.GetTargetPositionAttr().Set(targetAngleDegree)
        driveAPI.GetDampingAttr().Set(0.0)
        driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

        referenceJoint = UsdPhysics.Joint(linkAJoint);
        targetJoint = UsdPhysics.Joint(linkBJoint);

        targetJointStateAPI = PhysxSchema.JointStateAPI.Apply(linkBJoint, UsdPhysics.Tokens.angular)

        if (testMessageScenario == TEST_MSG_WRAPPED_JOINT_LIMIT_TARGET):
            joint = UsdPhysics.RevoluteJoint(linkBJoint)
            joint.GetLowerLimitAttr().Set(-FLT_MAX)
            joint.GetUpperLimitAttr().Set(FLT_MAX)

            message = (f"Usd Physics: the revolute joint at prim {targetJoint.GetPath()} needs a finite limit set "
                "to be used by the mimic joint feature.\n")

        elif (testMessageScenario == TEST_MSG_WRAPPED_JOINT_LIMIT_REFERENCE):
            joint = UsdPhysics.RevoluteJoint(linkAJoint)
            joint.GetLowerLimitAttr().Set(-FLT_MAX)
            joint.GetUpperLimitAttr().Set(FLT_MAX)

            message = (f"Usd Physics: the revolute joint at prim {linkAJoint.GetPath()} needs a finite limit set "
                "to be used by the mimic joint feature.\n")

        else:
            self.assertTrue(False)

        with utils.ExpectMessage(self, message):
            self._prepare_for_simulation()

        self._simulate_one_frame()

        posErrTolerance = 0.001

        targetJointPos = targetJointStateAPI.GetPositionAttr().Get()

        # the type of errors checked here should result in the mimic joint not
        # being created and thus the mimic joint should keep its start position

        self.assertAlmostEqual(targetJointPos, 0.0, delta=posErrTolerance)


    #
    # Ensure that mimic on a joint with wrapped limits triggers an error
    #
    async def test_message_wrapped_joint_limit_target(self):
        await self._test_error_messages_revolute(TEST_MSG_WRAPPED_JOINT_LIMIT_TARGET)


    #
    # Ensure that reference joint with wrapped limits triggers an error
    #
    async def test_message_wrapped_joint_limit_reference(self):
        await self._test_error_messages_revolute(TEST_MSG_WRAPPED_JOINT_LIMIT_REFERENCE)


    #
    # Ensure the replicator logic works for articulations with mimic joints
    #
    async def test_mimic_joint_replication(self):
        stage = await self.new_stage()

        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        rootPath = "/World/envs"
        replicationPathPrefix = "/env"

        UsdGeom.Scope.Define(stage, rootPath)

        replicationCount = 3

        nbVelIters = 1
        nbPosIters = 64
        useFixBase = True

        gearing1 = 0.5
        gearing2 = 4.0 / 5.0

        targetAngle = 5.0

        self._create_zero_gravity_scene(stage)

        jointPrimList = []
        for i in range(replicationCount):
            (link0JointPrim, link1JointPrim, link2JointPrim) = self._create_chain_scenario(stage, rootPath + replicationPathPrefix + str(i),
                Gf.Vec3f(float(i) * 5.0, 0.0, 0.0), useFixBase, nbPosIters, nbVelIters, gearing1, gearing2)

            driveAPI = UsdPhysics.DriveAPI.Apply(link0JointPrim, UsdPhysics.Tokens.angular)
            driveAPI.GetTargetPositionAttr().Set(targetAngle)
            driveAPI.GetDampingAttr().Set(0.0)
            driveAPI.GetStiffnessAttr().Set(1.0e10)  # very stiff drive to get to target instantly

            jointPrimList.append([link0JointPrim, link1JointPrim, link2JointPrim])

        def replicationAttachFn(stageId):
            exclude_paths = []
            exclude_paths.append(rootPath)
            return exclude_paths

        def replicationAttachEndFn(stageId):
            return

        def hierarchyRenameFn(replicatePath, index):
            stringPath = rootPath + replicationPathPrefix + str(index + 1);
            return stringPath

        get_physx_replicator_interface().register_replicator(stageId, replicationAttachFn, replicationAttachEndFn, hierarchyRenameFn);

        self._prepare_for_simulation()

        self._check_physx_object_counts({"numBoxShapes": 0, "numDynamicRigids": 0, "numArticulations": 0, "numConstraints": 0})

        get_physx_replicator_interface().replicate(stageId, rootPath + "/env0", (replicationCount - 1))

        self._check_physx_object_counts({"numBoxShapes": 0, "numDynamicRigids": replicationCount * 4, "numArticulations": replicationCount, "numConstraints": 0})

        self._simulate_one_frame()

        posErrTolerance = 0.001

        for i in range(replicationCount):

            angle0 = self._get_joint_angle(stage, jointPrimList[i][0], JOINT_AXIS_Z)
            angle1 = self._get_joint_angle(stage, jointPrimList[i][1], JOINT_AXIS_Z)
            angle2 = self._get_joint_angle(stage, jointPrimList[i][2], JOINT_AXIS_Z)

            self.assertAlmostEqual(angle0, targetAngle, delta=posErrTolerance)
            self.assertAlmostEqual(angle1, (-gearing1 * angle0), delta=posErrTolerance)
            self.assertAlmostEqual(angle2, (-gearing2 * angle1), delta=posErrTolerance)
