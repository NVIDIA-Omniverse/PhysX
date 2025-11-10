# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx.scripts.physicsUtils as physicsUtils
from pxr import Gf, Usd, Sdf, UsdGeom, UsdPhysics, PhysxSchema
import math
from typing_extensions import Literal
from typing import Union


class ArticulationTestBase:
    """Class to contain general set up for articulation tests and helper functions specific to articulation"""
    async def articulation_stage_and_scene_setup(self):
        await self.new_stage(def_up_and_mpu=False)

        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        self._meters_per_stage_unit = 0.01
        self._kilograms_per_stage_unit = 10.0
        self._length_scale = 1.0 / self._meters_per_stage_unit
        self._mass_scale = 1.0 / self._kilograms_per_stage_unit
        self._force_scale = self._mass_scale * self._length_scale
        self._torque_scale = self._mass_scale * self._length_scale * self._length_scale
        UsdGeom.SetStageMetersPerUnit(self._stage, self._meters_per_stage_unit)
        UsdPhysics.SetStageKilogramsPerUnit(self._stage, self._kilograms_per_stage_unit)

        if not self._stage.GetDefaultPrim():
            root_prim = UsdGeom.Xform.Define(self._stage, "/World")
            self._stage.SetDefaultPrim(root_prim.GetPrim())
        self._default_prim_path = self._stage.GetDefaultPrim().GetPath()

        # add and setup physics scene
        self._scene = UsdPhysics.Scene.Define(self._stage, self._default_prim_path.AppendChild("PhysicsScene"))
        self._gravity_magnitude = 10.0 * self._length_scale
        self._scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._scene.CreateGravityMagnitudeAttr().Set(self._gravity_magnitude)

    def _disable_gravity(self):
        self._scene.CreateGravityMagnitudeAttr().Set(0.0)

    def _setup_two_link_articulation(self, make_fixed_base: bool = True):
        position = Gf.Vec3f(0.0, 0.0, 0.0)
        orientation = Gf.Quatf(1.0)
        color = Gf.Vec3f(165.0 / 255.0, 21.0 / 255.0, 21.0 / 255.0)
        self._link_length = 0.05 * self._length_scale
        density = 1000.0 * self._mass_scale / (self._length_scale ** 3.0)
        aspect_ratio = 0.1
        self._link_width = self._link_length * aspect_ratio
        size = Gf.Vec3f(self._link_width, self._link_length, self._link_width)

        root_link_prim = physicsUtils.add_rigid_box(self._stage, "/rootLink", size, position, orientation, color, density)
        self._root_link_path = root_link_prim.GetPath()
        self._link_mass = density * self._link_length * self._link_width * self._link_width

        self._base_link = UsdGeom.Cube.Define(self._stage, self._root_link_path)

        # make it fixed base:
        articulation_api = None
        if make_fixed_base:
            fixed_joint_path = self._stage.GetDefaultPrim().GetPath().AppendChild("baseFixedJoint")
            fixed_base_joint = UsdPhysics.FixedJoint.Define(self._stage, fixed_joint_path)
            relationship_targets = [self._root_link_path]
            fixed_base_joint.CreateBody1Rel().SetTargets(relationship_targets)
            UsdPhysics.ArticulationRootAPI.Apply(self._stage.GetDefaultPrim())
            articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(self._stage.GetDefaultPrim())
        else:
            # floating base apply to base
            UsdPhysics.ArticulationRootAPI.Apply(self._base_link.GetPrim())
            articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(self._base_link.GetPrim())
        # disable sleep
        articulation_api.CreateSleepThresholdAttr(0.0)

        # Box1
        position = Gf.Vec3f(0.0, self._link_length, 0.0)
        color = Gf.Vec3f(21.0 / 255.0, 21.0 / 255.0, 165.0 / 255.0)
        dynamic_link_prim = physicsUtils.add_rigid_box(self._stage, "/dynamicLink", size, position, orientation, color, density)
        self._dynamic_link_path = dynamic_link_prim.GetPath()
        self._dynamic_link = UsdGeom.Cube.Define(self._stage, self._dynamic_link_path)

    def _setup_fixed_joint(self):
        # Add spherical joint between the two bars:
        joint_path = self._dynamic_link_path.AppendChild("FixedJoint")
        joint = UsdPhysics.FixedJoint.Define(self._stage, joint_path)
        joint.CreateBody0Rel().SetTargets([self._root_link_path])
        joint.CreateBody1Rel().SetTargets([self._dynamic_link_path])
        joint_parent_pos = Gf.Vec3f(0, 0.5, 0)  # is scaled by link length
        self._joint_parent_pose = Gf.Quatf(1.0)
        self._joint_child_pose = Gf.Quatf(1.0)
        joint.CreateLocalPos0Attr().Set(joint_parent_pos)
        joint.CreateLocalRot0Attr().Set(self._joint_parent_pose)
        joint_child_pos = Gf.Vec3f(0, -0.5, 0)  # is scaled by link length
        joint.CreateLocalPos1Attr().Set(joint_child_pos)
        joint.CreateLocalRot1Attr().Set(self._joint_child_pose)

    def _setup_revolute_joint(self, limits: tuple = None):
        # Add spherical joint between the two bars:
        joint_path = self._dynamic_link_path.AppendChild("RevoluteJoint")
        joint = UsdPhysics.RevoluteJoint.Define(self._stage, joint_path)
        if limits:
            self.assertLessEqual(limits[0], limits[1])
            joint.CreateLowerLimitAttr(limits[0])
            joint.CreateUpperLimitAttr(limits[1])
        joint.CreateBody0Rel().SetTargets([self._root_link_path])
        joint.CreateBody1Rel().SetTargets([self._dynamic_link_path])
        axis = "X"
        joint.CreateAxisAttr(axis)
        joint_parent_pos = Gf.Vec3f(0, 0.5, 0)  # is scaled by link length
        self._joint_parent_pose = Gf.Quatf(1.0)
        self._joint_child_pose = Gf.Quatf(1.0)
        joint.CreateLocalPos0Attr().Set(joint_parent_pos)
        joint.CreateLocalRot0Attr().Set(self._joint_parent_pose)
        joint_child_pos = Gf.Vec3f(0, -0.5, 0)  # is scaled by link length
        joint.CreateLocalPos1Attr().Set(joint_child_pos)
        joint.CreateLocalRot1Attr().Set(self._joint_child_pose)
        return joint

    def _setup_spherical_joint(self, cone_angle0, cone_angle1):
        # Add spherical joint between the two bars:
        joint_path = self._dynamic_link_path.AppendChild("SphericalJoint")
        joint = UsdPhysics.SphericalJoint.Define(self._stage, joint_path)

        joint.CreateConeAngle0LimitAttr(cone_angle0)
        joint.CreateConeAngle1LimitAttr(cone_angle1)
        joint.CreateBody0Rel().SetTargets([self._root_link_path])
        joint.CreateBody1Rel().SetTargets([self._dynamic_link_path])
        axis = "Y"
        joint.CreateAxisAttr(axis)
        joint_parent_pos = Gf.Vec3f(0, 0.5, 0)  # is scaled by link length
        self._joint_parent_pose = Gf.Quatf(1.0)
        joint_offset = 0
        child_rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), joint_offset)
        self._joint_child_pose = Gf.Quatf(child_rot.GetQuat())
        joint.CreateLocalPos0Attr().Set(joint_parent_pos)
        joint.CreateLocalRot0Attr().Set(self._joint_parent_pose)
        joint_child_pos = Gf.Vec3f(0, -0.5, 0)  # is scaled by link length
        joint.CreateLocalPos1Attr().Set(joint_child_pos)
        joint.CreateLocalRot1Attr().Set(self._joint_child_pose)

    def _setup_D6_joint(self):
        # Add d6 joint between the two bars:
        joint_path = self._dynamic_link_path.AppendChild("D6Joint")
        joint = UsdPhysics.Joint.Define(self._stage, joint_path)
        joint.CreateBody0Rel().SetTargets([self._root_link_path])
        joint.CreateBody1Rel().SetTargets([self._dynamic_link_path])
        joint_parent_pos = Gf.Vec3f(0, 0.5, 0)  # is scaled by link length
        self._joint_parent_pose = Gf.Quatf(1.0)
        self._joint_child_pose = Gf.Quatf(1.0)
        joint.CreateLocalPos0Attr().Set(joint_parent_pos)
        joint.CreateLocalRot0Attr().Set(self._joint_parent_pose)
        joint_child_pos = Gf.Vec3f(0, -0.5, 0)  # is scaled by link length
        joint.CreateLocalPos1Attr().Set(joint_child_pos)
        joint.CreateLocalRot1Attr().Set(self._joint_child_pose)
        return joint

    def _setup_D6_driver(self, deflection_angle: float):
        # rig d6 driver (DO AFTER SPHERICAL JOINT):
        d6_path = self._stage.GetDefaultPrim().GetPath().AppendChild("D6DriverJoint")
        d6_joint = UsdPhysics.Joint.Define(self._stage, d6_path)
        d6_joint.CreateBody0Rel().SetTargets([self._root_link_path])
        d6_joint.CreateBody1Rel().SetTargets([self._dynamic_link_path])
        d6_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
        d6_joint.CreateExcludeFromArticulationAttr().Set(True)
        d6_joint.CreateLocalRot0Attr().Set(self._joint_parent_pose)
        d6_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        d6_joint.CreateLocalRot1Attr().Set(self._joint_child_pose)
        d6_joint.CreateBreakForceAttr().Set(1e20 * self._force_scale)
        d6_joint.CreateBreakTorqueAttr().Set(1e20 * self._torque_scale)
        drives = ["rotX", "rotZ"]

        gravity_torque = self._gravity_magnitude * self._link_length * self._link_mass * 0.5
        spring = gravity_torque / deflection_angle
        damping = 0.1 * spring

        self._drives = {}
        for d in drives:
            drive_api = UsdPhysics.DriveAPI.Apply(d6_joint.GetPrim(), d)
            drive_api.CreateTypeAttr("force")
            drive_api.CreateMaxForceAttr(20e6 * self._force_scale)
            angle = 0.0
            drive_api.CreateTargetPositionAttr(angle)
            drive_api.CreateDampingAttr(damping)
            drive_api.CreateStiffnessAttr(spring)
            self._drives[d] = drive_api

    def _setup_revolute_drive(self, deflection_angle: float):
        # rig drive (DO AFTER REVOLUTE JOINT):
        revolute_joint_path = self._dynamic_link_path.AppendChild("RevoluteJoint")
        joint = UsdPhysics.RevoluteJoint.Get(self._stage, revolute_joint_path)
        self.assertTrue(joint)
        drive = "angular"

        gravity_torque = self._gravity_magnitude * self._link_length * self._link_mass * 0.5
        spring = gravity_torque / deflection_angle
        damping = 2.0 * math.sqrt(spring * self._link_mass)

        self._drives = {}
        drive_api = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), drive)
        drive_api.CreateTypeAttr("force")
        drive_api.CreateMaxForceAttr(20e6 * self._force_scale)
        angle = 0.0
        drive_api.CreateTargetPositionAttr(angle)
        drive_api.CreateDampingAttr(damping)
        drive_api.CreateStiffnessAttr(spring)
        self._drives[drive] = drive_api
        return drive_api

    def _setup_revolute_joint_state(self, deflection_angle: float):
        # rig joint api (DO AFTER REVOLUTE JOINT):
        revolute_joint_path = self._dynamic_link_path.AppendChild("RevoluteJoint")
        joint = UsdPhysics.RevoluteJoint.Get(self._stage, revolute_joint_path)
        self.assertTrue(joint)
        token = "angular"

        self._joint_states = {}
        joint_state_api = PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), token)
        joint_state_api.CreatePositionAttr(deflection_angle)
        joint_state_api.CreateVelocityAttr(0.0)
        self._joint_states[token] = joint_state_api

    def _get_rotation_angle(self, xformable: UsdGeom.Xformable) -> float:
        """Get the rotation angle of a xformable

        Args:
            xformable: the xformable to get the rotation of.

        Returns:
            the rotation angle.

        Asserts:
            if the orthonormalization fails.
        """
        self.assertTrue(isinstance(xformable, UsdGeom.Xformable))
        xform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode().Default())
        self.assertTrue(xform.Orthonormalize(False))
        return xform.ExtractRotation().GetAngle()

    def _get_rotation_quaternion(self, xformable: UsdGeom.Xformable) -> Gf.Quatf:
        """Get the rotation quaternion of a xformable

        Args:
            xformable: the xformable to get the rotation of.

        Returns:
            the rotation quaternion.

        Asserts:
            if the orthonormalization fails.
        """
        self.assertTrue(isinstance(xformable, UsdGeom.Xformable))
        xform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode().Default())
        self.assertTrue(xform.Orthonormalize(False))
        return Gf.Quatf(xform.ExtractRotationQuat())

    def _axis_angle_to_quat(self, axis: Union[Literal["X", "Y", "Z"], Gf.Vec3d, Gf.Vec3f], angle: float) -> Gf.Quatf:
        """Convert a one-axis-rotation to a quaternion.

        Args:
            axis: the axis the rotation is around as a string.
            angle: the angle of the rotation.

        Returns:
            the quarternion equal to the one-axis-rotation.
        """
        if axis == "X":
            axis = Gf.Vec3d.XAxis()
        elif axis == "Y":
            axis = Gf.Vec3d.YAxis()
        elif axis == "Z":
            axis = Gf.Vec3d.ZAxis()
        elif type(axis) == Gf.Vec3f:
            axis = Gf.Vec3d(axis)

        assert type(axis) == Gf.Vec3d, 'axis_angle_to_quat: invalid axis, please use "X", "Y", "Z" or Gf.Vec3<d/f>.<X/Y/Z>Axis()'
        return Gf.Quatf(Gf.Rotation(axis, angle).GetQuat())

    def _get_link_translation_op(self, link):
        ops = link.GetOrderedXformOps()
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                return op
        return None

    async def _setup_stage_and_bodies(self, setupAsArticulation=False):
        stage = await self.new_stage()
        self._stage = stage

        defaultPrimXform = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(defaultPrimXform.GetPrim())

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

        self._defaultPrimPath = stage.GetDefaultPrim().GetPath()

        scene = UsdPhysics.Scene.Define(stage, self._defaultPrimPath.AppendChild("physicsScene"))
        scene.CreateGravityMagnitudeAttr().Set(0.0)

        # setup dynamic box
        self._dynamic_box = physicsUtils.add_rigid_box(
            stage, self._defaultPrimPath.AppendChild("dynamicBox"), size=Gf.Vec3f(1.0), position=Gf.Vec3f(0.0)
        )

        self._dummy_root_link = None
        if setupAsArticulation:
            # create dummy root link connected to world via a fixed joint
            self._dummy_root_link = physicsUtils.add_rigid_box(
                stage, self._defaultPrimPath.AppendChild("dummyRootLink"), size=Gf.Vec3f(1.0), position=Gf.Vec3f(0.0)
            )
            rootFixedJoint = UsdPhysics.FixedJoint.Define(
                stage, self._defaultPrimPath.AppendChild("rootArticulationFixedJoint")
            )
            UsdPhysics.ArticulationRootAPI.Apply(rootFixedJoint.GetPrim())
            rootFixedJoint.CreateBody1Rel().SetTargets([self._dummy_root_link.GetPath()])

    def _setup_joint(self, jointType, flipBodyRel=False, axis=None):
        self._joint = None
        if jointType == "revolute":
            self._joint = UsdPhysics.RevoluteJoint.Define(
                self._stage, self._defaultPrimPath.AppendChild("revoluteJoint")
            )

        elif jointType == "prismatic":
            self._joint = UsdPhysics.PrismaticJoint.Define(
                self._stage, self._defaultPrimPath.AppendChild("prismaticJoint")
            )
        elif jointType == "spherical":
            self._joint = UsdPhysics.SphericalJoint.Define(
                self._stage, self._defaultPrimPath.AppendChild("sphericalJoint")
            )
        elif jointType == "D6":
            self._joint = UsdPhysics.Joint.Define(self._stage, self._defaultPrimPath.AppendChild("sphericalJoint"))

        self.assertTrue(self._joint)

        if axis is not None:
            self._joint.CreateAxisAttr(axis)

        if flipBodyRel:
            if self._dummy_root_link:
                self._joint.CreateBody1Rel().SetTargets([self._dummy_root_link.GetPath()])
            self._joint.CreateBody0Rel().SetTargets([self._dynamic_box.GetPath()])
        else:
            if self._dummy_root_link:
                self._joint.CreateBody0Rel().SetTargets([self._dummy_root_link.GetPath()])
            self._joint.CreateBody1Rel().SetTargets([self._dynamic_box.GetPath()])

    def _setup_joint_drive(self, type: str, stiffness=0.0, damping=0.0, targetPos=None, targetVel=None):
        driveAPI = UsdPhysics.DriveAPI.Apply(self._joint.GetPrim(), type)
        driveAPI.CreateStiffnessAttr(stiffness)
        driveAPI.CreateDampingAttr(damping)
        if targetPos is not None:
            driveAPI.CreateTargetPositionAttr(targetPos)
        if targetVel is not None:
            driveAPI.CreateTargetVelocityAttr(targetVel)
        return driveAPI

    def _setup_joint_state(self, type: str, position=None, velocity=None):
        stateAPI = PhysxSchema.JointStateAPI.Apply(self._joint.GetPrim(), type)
        if position is not None:
            stateAPI.CreatePositionAttr(position)
        if velocity is not None:
            stateAPI.CreateVelocityAttr(velocity)
        return stateAPI

    def _flip_single_axis_joint_frames(self, joint, axis):
        flipQuat = {
            "X": Gf.Quatf(0, Gf.Vec3f(0, 1, 0)),
            "Y": Gf.Quatf(0, Gf.Vec3f(1, 0, 0)),  # 180 around x flips both y and z
            "Z": Gf.Quatf(0, Gf.Vec3f(1, 0, 0)),  # 180 around x flips both y and z
        }
        joint.CreateLocalRot0Attr().Set(flipQuat[axis])
        joint.CreateLocalRot1Attr().Set(flipQuat[axis])

    async def _prismatic_test_setup(
        self, axis, driveType, flipBodyRel=False, flipFrames=False, setupAsArticulation=False, targetValue=1.0
    ):
        await self._setup_stage_and_bodies(setupAsArticulation=setupAsArticulation)
        self._setup_joint(jointType="prismatic", flipBodyRel=flipBodyRel, axis=axis)

        if flipFrames:
            self._flip_single_axis_joint_frames(self._joint, axis)

        if driveType == "pos":
            self._driveAPI = self._setup_joint_drive(type="linear", stiffness=1.0, targetPos=targetValue)
        elif driveType == "vel":
            self._driveAPI = self._setup_joint_drive(type="linear", damping=1.0, targetVel=targetValue)
        elif driveType == "jointStatePos":
            self._stateAPI = self._setup_joint_state(type="linear", position=targetValue)
        elif driveType == "jointStateVel":
            self._stateAPI = self._setup_joint_state(type="linear", velocity=targetValue)

    async def _revolute_test_setup(
        self, axis, driveType, flipBodyRel=False, flipFrames=False, setupAsArticulation=False, targetValue=90.0
    ):
        await self._setup_stage_and_bodies(setupAsArticulation=setupAsArticulation)
        self._setup_joint(jointType="revolute", flipBodyRel=flipBodyRel, axis=axis)

        if flipFrames:
            self._flip_single_axis_joint_frames(self._joint, axis)

        if driveType == "pos":
            self._driveAPI = self._setup_joint_drive(type="angular", stiffness=1.0, targetPos=targetValue)
        elif driveType == "vel":
            self._driveAPI = self._setup_joint_drive(type="angular", damping=1.0, targetVel=targetValue)
        elif driveType == "jointStatePos":
            self._stateAPI = self._setup_joint_state(type="angular", position=targetValue)
        elif driveType == "jointStateVel":
            self._stateAPI = self._setup_joint_state(type="angular", velocity=targetValue)

    async def _d6_revolute_test_setup(
        self, axis, driveType, flipBodyRel=False, flipFrames=False, setupAsArticulation=False, createAxisLimitAPI=True, targetValue=90.0
    ):
        await self._setup_stage_and_bodies(setupAsArticulation=setupAsArticulation)
        self._setup_joint(jointType="D6", flipBodyRel=flipBodyRel)
        # lock all dof except the axis:
        limits = ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]
        freeAxis = "rot" + axis
        limits.remove(freeAxis)

        for limit in limits:
            limitAPI = UsdPhysics.LimitAPI.Apply(self._joint.GetPrim(), limit)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)

        # free axis limit to configure
        if createAxisLimitAPI:
            self._d6_limitAPI = limitAPI = UsdPhysics.LimitAPI.Apply(self._joint.GetPrim(), freeAxis)

        if flipFrames:
            self._flip_single_axis_joint_frames(self._joint, axis)

        if driveType == "pos":
            self._driveAPI = self._setup_joint_drive(type=freeAxis, stiffness=1.0, targetPos=targetValue)
        elif driveType == "vel":
            self._driveAPI = self._setup_joint_drive(type=freeAxis, damping=1.0, targetVel=targetValue)
        elif driveType == "jointStatePos":
            self._stateAPI = self._setup_joint_state(type=freeAxis, position=targetValue)
        elif driveType == "jointStateVel":
            self._stateAPI = self._setup_joint_state(type=freeAxis, velocity=targetValue)
