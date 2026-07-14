# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + Newton schemas

import unittest

import _physx
import physicsUtils
from physicsBase import PhysicsMemoryStageBaseTestCase, TestCategory, ExpectMessage
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema


NEWTON_MIMIC_API = "NewtonMimicAPI"
MIMIC_ENABLED = "newton:mimicEnabled"
MIMIC_JOINT_REL = "newton:mimicJoint"
MIMIC_COEF0 = "newton:mimicCoef0"
MIMIC_COEF1 = "newton:mimicCoef1"


class NewtonMimicAPITestMemoryStage(PhysicsMemoryStageBaseTestCase):
    category = TestCategory.Core

    def new_stage(self):
        if self._stage_attached:
            self.detach_stage()
        super().new_stage(attach_stage=False)
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self._stage, 1.0)
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.y)
        return self._stage

    def _simulate_one_frame(self):
        sim = _physx.acquire_physx_simulation_interface()
        sim.simulate(1.0 / 60.0, 0.0)
        sim.fetch_results()

    def _create_zero_gravity_scene(self, stage):
        scenePath = str(stage.GetDefaultPrim().GetPath()) + "/Scene"
        scene = UsdPhysics.Scene.Define(stage, scenePath)
        scene.CreateGravityMagnitudeAttr().Set(0.0)
        return scene

    def _create_body(self, stage, path, size, position, mass=1.0):
        xform = UsdGeom.Xform.Define(stage, path)
        physicsUtils._add_transformation(xform, position, Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        massAPI = UsdPhysics.MassAPI.Apply(prim)
        massAPI.CreateMassAttr().Set(mass)
        massAPI.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(mass))
        bodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        bodyAPI.CreateMaxAngularVelocityAttr().Set(1.0e10)
        bodyAPI.CreateLinearDampingAttr().Set(0.0)
        bodyAPI.CreateAngularDampingAttr().Set(0.0)
        bodyAPI.CreateSleepThresholdAttr().Set(0.0)
        bodyAPI.CreateStabilizationThresholdAttr().Set(0.0)
        physicsUtils.add_box(stage, path + "/Geom", size)
        return prim

    def _create_fixed_base_articulation(self, stage, rootPath, rootPosition=Gf.Vec3f(0.0)):
        rootLink = self._create_body(stage, rootPath, Gf.Vec3f(0.001), rootPosition, mass=1.0)
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, rootPath + "/InboundJoint")
        fixedJoint.CreateBody1Rel().AddTarget(rootPath)
        artPrim = fixedJoint.GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(artPrim)
        art = PhysxSchema.PhysxArticulationAPI.Apply(artPrim)
        art.CreateSolverPositionIterationCountAttr().Set(16)
        art.CreateSolverVelocityIterationCountAttr().Set(1)
        art.CreateSleepThresholdAttr().Set(0.0)
        art.CreateStabilizationThresholdAttr().Set(0.0)
        art.CreateEnabledSelfCollisionsAttr().Set(False)
        return rootLink

    def _add_prismatic_link(self, stage, artRootPath, linkPath, linkLocalPos):
        self._create_body(stage, linkPath, Gf.Vec3f(0.2), linkLocalPos, mass=1.0)
        jointPath = linkPath + "/InboundJoint"
        joint = UsdPhysics.PrismaticJoint.Define(stage, jointPath)
        joint.CreateAxisAttr("X")
        joint.CreateBody0Rel().AddTarget(artRootPath)
        joint.CreateBody1Rel().AddTarget(linkPath)
        joint.CreateLocalPos0Attr().Set(linkLocalPos)
        joint.CreateLowerLimitAttr().Set(-10.0)
        joint.CreateUpperLimitAttr().Set(10.0)
        return joint

    def _add_revolute_link(self, stage, artRootPath, linkPath, linkLocalPos, set_limits=True):
        self._create_body(stage, linkPath, Gf.Vec3f(0.2), linkLocalPos, mass=1.0)
        jointPath = linkPath + "/InboundJoint"
        joint = UsdPhysics.RevoluteJoint.Define(stage, jointPath)
        joint.CreateAxisAttr("Z")
        joint.CreateBody0Rel().AddTarget(artRootPath)
        joint.CreateBody1Rel().AddTarget(linkPath)
        joint.CreateLocalPos0Attr().Set(linkLocalPos)
        if set_limits:
            joint.CreateLowerLimitAttr().Set(-180.0)
            joint.CreateUpperLimitAttr().Set(180.0)
        return joint

    def _build_two_prismatic_articulation(self, stage, coef0, coef1, mimic_enabled=True, drive_target=-0.1):
        rootPath = str(stage.GetDefaultPrim().GetPath())
        artRootPath = rootPath + "/RootLink"
        self._create_fixed_base_articulation(stage, artRootPath)

        linkAPath = rootPath + "/LinkA"
        linkAPos = Gf.Vec3f(-1.0, 0.0, 0.0)
        linkAJoint = self._add_prismatic_link(stage, artRootPath, linkAPath, linkAPos)

        linkBPath = rootPath + "/LinkB"
        linkBPos = Gf.Vec3f(1.0, 0.0, 0.0)
        linkBJoint = self._add_prismatic_link(stage, artRootPath, linkBPath, linkBPos)

        drive = UsdPhysics.DriveAPI.Apply(linkAJoint.GetPrim(), UsdPhysics.Tokens.linear)
        drive.CreateTargetPositionAttr().Set(drive_target)
        drive.CreateDampingAttr().Set(0.0)
        drive.CreateStiffnessAttr().Set(1.0e10)

        bPrim = linkBJoint.GetPrim()
        assert bPrim.ApplyAPI(NEWTON_MIMIC_API)
        bPrim.CreateAttribute(MIMIC_COEF0, Sdf.ValueTypeNames.Float).Set(coef0)
        bPrim.CreateAttribute(MIMIC_COEF1, Sdf.ValueTypeNames.Float).Set(coef1)
        bPrim.CreateAttribute(MIMIC_ENABLED, Sdf.ValueTypeNames.Bool).Set(mimic_enabled)
        bPrim.CreateRelationship(MIMIC_JOINT_REL).SetTargets([linkAJoint.GetPath()])

        linkA = stage.GetPrimAtPath(linkAPath)
        linkB = stage.GetPrimAtPath(linkBPath)
        return (linkA, linkAJoint, linkB, linkBJoint)

    def _joint_disp(self, linkPrim, restX):
        return linkPrim.GetAttribute("xformOp:translate").Get()[0] - restX

    def _build_two_revolute_articulation(self, stage, coef0, coef1,
                                         drive_target_deg=15.0, follower_has_limits=True):
        rootPath = str(stage.GetDefaultPrim().GetPath())
        artRootPath = rootPath + "/RootLink"
        self._create_fixed_base_articulation(stage, artRootPath)

        linkAPath = rootPath + "/LinkA"
        linkAJoint = self._add_revolute_link(stage, artRootPath, linkAPath, Gf.Vec3f(-1.0, 0.0, 0.0))

        linkBPath = rootPath + "/LinkB"
        linkBJoint = self._add_revolute_link(stage, artRootPath, linkBPath, Gf.Vec3f(1.0, 0.0, 0.0),
                                             set_limits=follower_has_limits)

        drive = UsdPhysics.DriveAPI.Apply(linkAJoint.GetPrim(), UsdPhysics.Tokens.angular)
        drive.CreateTargetPositionAttr().Set(drive_target_deg)
        drive.CreateDampingAttr().Set(0.0)
        drive.CreateStiffnessAttr().Set(1.0e10)

        bPrim = linkBJoint.GetPrim()
        assert bPrim.ApplyAPI(NEWTON_MIMIC_API)
        bPrim.CreateAttribute(MIMIC_COEF0, Sdf.ValueTypeNames.Float).Set(coef0)
        bPrim.CreateAttribute(MIMIC_COEF1, Sdf.ValueTypeNames.Float).Set(coef1)
        bPrim.CreateAttribute(MIMIC_ENABLED, Sdf.ValueTypeNames.Bool).Set(True)
        bPrim.CreateRelationship(MIMIC_JOINT_REL).SetTargets([linkAJoint.GetPath()])

        linkA = stage.GetPrimAtPath(linkAPath)
        linkB = stage.GetPrimAtPath(linkBPath)
        return (linkA, linkAJoint, linkB, linkBJoint)

    # -- Positive tests ----------------------------------------------------

    def test_prismatic_follower_tracks_leader_with_unit_coef(self):
        stage = self.new_stage()
        self._create_zero_gravity_scene(stage)
        target = -0.1
        (linkA, _, linkB, _) = self._build_two_prismatic_articulation(stage, coef0=0.0, coef1=1.0, drive_target=target)
        self.attach_stage()
        self._simulate_one_frame()
        self.assertAlmostEqual(self._joint_disp(linkA, -1.0), target, delta=1e-3)
        self.assertAlmostEqual(self._joint_disp(linkB, 1.0), target, delta=1e-3)

    def test_prismatic_follower_negated_with_coef1_minus_one(self):
        stage = self.new_stage()
        self._create_zero_gravity_scene(stage)
        target = -0.1
        (linkA, _, linkB, _) = self._build_two_prismatic_articulation(stage, coef0=0.0, coef1=-1.0, drive_target=target)
        self.attach_stage()
        self._simulate_one_frame()
        self.assertAlmostEqual(self._joint_disp(linkB, 1.0), -self._joint_disp(linkA, -1.0), delta=1e-3)

    def test_prismatic_coef0_offset_applied(self):
        stage = self.new_stage()
        self._create_zero_gravity_scene(stage)
        target = -0.1
        coef0 = 0.05
        (linkA, _, linkB, _) = self._build_two_prismatic_articulation(stage, coef0=coef0, coef1=1.0, drive_target=target)
        self.attach_stage()
        self._simulate_one_frame()
        self.assertAlmostEqual(self._joint_disp(linkB, 1.0), coef0 + self._joint_disp(linkA, -1.0), delta=1e-3)

    def test_revolute_follower_tracks_leader_with_unit_coef(self):
        stage = self.new_stage()
        self._create_zero_gravity_scene(stage)
        (linkA, _, linkB, _) = self._build_two_revolute_articulation(
            stage, coef0=0.0, coef1=1.0, drive_target_deg=15.0)
        self.attach_stage()
        self._simulate_one_frame()
        orientA = linkA.GetAttribute("xformOp:orient").Get()
        orientB = linkB.GetAttribute("xformOp:orient").Get()
        # Drive should have rotated the leader away from identity (real part < 1).
        self.assertLess(orientA.GetReal(), 0.9999)
        # With coef0=0, coef1=1 the follower must match the leader.
        self.assertAlmostEqual(orientA.GetReal(), orientB.GetReal(), delta=1e-3)
        self.assertAlmostEqual(orientA.GetImaginary()[2], orientB.GetImaginary()[2], delta=1e-3)

    def test_mimic_disabled_suppresses_constraint(self):
        stage = self.new_stage()
        self._create_zero_gravity_scene(stage)
        target = -0.1
        (linkA, _, linkB, _) = self._build_two_prismatic_articulation(
            stage, coef0=0.0, coef1=1.0, mimic_enabled=False, drive_target=target)
        self.attach_stage()
        self._simulate_one_frame()
        self.assertAlmostEqual(self._joint_disp(linkA, -1.0), target, delta=1e-3)
        self.assertAlmostEqual(self._joint_disp(linkB, 1.0), 0.0, delta=1e-3)

    # -- Negative tests ----------------------------------------------------

    def test_reject_multi_dof_follower(self):
        stage = self.new_stage()
        self._create_zero_gravity_scene(stage)
        rootPath = str(stage.GetDefaultPrim().GetPath())
        artRootPath = rootPath + "/RootLink"
        self._create_fixed_base_articulation(stage, artRootPath)

        # single-DOF leader (prismatic).
        linkAPath = rootPath + "/LinkA"
        linkAJoint = self._add_prismatic_link(stage, artRootPath, linkAPath, Gf.Vec3f(-1.0, 0, 0))

        # multi-DOF follower (spherical) — must be rejected.
        linkBPath = rootPath + "/LinkB"
        self._create_body(stage, linkBPath, Gf.Vec3f(0.2), Gf.Vec3f(1.0, 0, 0))
        linkBJoint = UsdPhysics.SphericalJoint.Define(stage, linkBPath + "/InboundJoint")
        linkBJoint.CreateBody0Rel().AddTarget(artRootPath)
        linkBJoint.CreateBody1Rel().AddTarget(linkBPath)

        bPrim = linkBJoint.GetPrim()
        assert bPrim.ApplyAPI(NEWTON_MIMIC_API)
        bPrim.CreateAttribute(MIMIC_COEF0, Sdf.ValueTypeNames.Float).Set(0.0)
        bPrim.CreateAttribute(MIMIC_COEF1, Sdf.ValueTypeNames.Float).Set(1.0)
        bPrim.CreateRelationship(MIMIC_JOINT_REL).SetTargets([linkAJoint.GetPath()])

        with ExpectMessage(self, "NewtonMimicAPI is only supported on single-DOF joints",
                           partial_string_match=True):
            self.attach_stage()
            self._simulate_one_frame()

    def test_reject_multi_dof_leader(self):
        stage = self.new_stage()
        self._create_zero_gravity_scene(stage)
        rootPath = str(stage.GetDefaultPrim().GetPath())
        artRootPath = rootPath + "/RootLink"
        self._create_fixed_base_articulation(stage, artRootPath)

        # multi-DOF leader (spherical) — must be rejected when referenced.
        linkAPath = rootPath + "/LinkA"
        self._create_body(stage, linkAPath, Gf.Vec3f(0.2), Gf.Vec3f(-1.0, 0, 0))
        linkAJoint = UsdPhysics.SphericalJoint.Define(stage, linkAPath + "/InboundJoint")
        linkAJoint.CreateBody0Rel().AddTarget(artRootPath)
        linkAJoint.CreateBody1Rel().AddTarget(linkAPath)

        # single-DOF follower.
        linkBPath = rootPath + "/LinkB"
        linkBJoint = self._add_prismatic_link(stage, artRootPath, linkBPath, Gf.Vec3f(1.0, 0, 0))

        bPrim = linkBJoint.GetPrim()
        assert bPrim.ApplyAPI(NEWTON_MIMIC_API)
        bPrim.CreateAttribute(MIMIC_COEF0, Sdf.ValueTypeNames.Float).Set(0.0)
        bPrim.CreateAttribute(MIMIC_COEF1, Sdf.ValueTypeNames.Float).Set(1.0)
        bPrim.CreateRelationship(MIMIC_JOINT_REL).SetTargets([linkAJoint.GetPath()])

        with ExpectMessage(self, "NewtonMimicAPI is only supported on single-DOF joints",
                           partial_string_match=True):
            self.attach_stage()
            self._simulate_one_frame()

    def test_reject_revolute_follower_without_finite_limit(self):
        stage = self.new_stage()
        self._create_zero_gravity_scene(stage)
        self._build_two_revolute_articulation(
            stage, coef0=0.0, coef1=1.0, follower_has_limits=False)
        with ExpectMessage(self, "revolute joint without a finite limit set",
                           partial_string_match=True):
            self.attach_stage()
            self._simulate_one_frame()


if __name__ == "__main__":
    unittest.main()
