# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, Vt, Sdf, Usd
import math
from omni.physx.scripts.utils import get_spatial_tendon_attachment_candidates
from omni.physxtests import utils
from omni.physxtests.utils.physicsBase import (
    PhysicsMemoryStageBaseAsyncTestCase,
    PhysicsKitStageAsyncTestCase,
    TestCategory,
)
import omni.physx.scripts.physicsUtils as physicsUtils
import omni.kit.commands
import unittest


class ArticulationTestBase:
    # CALL AFTER physics base init via super().setUp()
    async def setup_articulations(self):
        # will create memory or kit stage based on what physicsBase the test class is derived from
        await self.new_stage(def_up_and_mpu=False)
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        self._metersPerStageUnit = 0.01
        self._kilogramsPerStageUnit = 1.0
        UsdGeom.SetStageMetersPerUnit(self._stage, self._metersPerStageUnit)
        UsdPhysics.SetStageKilogramsPerUnit(self._stage, self._kilogramsPerStageUnit)

        if not self._stage.GetDefaultPrim():
            rootPrim = UsdGeom.Xform.Define(self._stage, "/World")
            self._stage.SetDefaultPrim(rootPrim.GetPrim())
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()

        # add and setup physics scene
        self._scene = UsdPhysics.Scene.Define(self._stage, self._defaultPrimPath.AppendChild("PhysicsScene"))
        self._gravityMagnitude = 10.0 / self._metersPerStageUnit
        self._scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._scene.CreateGravityMagnitudeAttr().Set(self._gravityMagnitude)

        self._spatialTendonRootName = "Root"
        self._spatialTendonLeafName = "Leaf"

    def _test_float_equal(self, floatA: float, floatB: float, tol: float = 1.0e-6, msg: str = None):
        self.assertLess(math.fabs(floatA - floatB), tol, msg=msg)

    def _assertAlmostEqualVector3(self, vectorA, vectorB):
        for a, b in zip(vectorA, vectorB):
            self._test_float_equal(a, b)

    def _setup_two_link_articulation(self, makeFixedBase: bool = True):
        position = Gf.Vec3f(0.0, 0.0, 0.0)
        orientation = Gf.Quatf(1.0)
        color = Gf.Vec3f(165.0 / 255.0, 21.0 / 255.0, 21.0 / 255.0)
        self._linkLength = 5.0
        densityM = 1000.0
        densityCM = densityM * (self._metersPerStageUnit ** 3.0) / self._kilogramsPerStageUnit
        aspectRatio = 0.1
        self._linkWidth = self._linkLength * aspectRatio
        size = Gf.Vec3f(self._linkWidth, self._linkLength, self._linkWidth)

        prim = physicsUtils.add_rigid_box(self._stage, "/rootLink", size, position, orientation, color, densityCM)
        self._baseLinkPath = prim.GetPath()
        self._linkMass = densityCM * self._linkLength * self._linkWidth * self._linkWidth

        self._baseLink = UsdGeom.Cube.Define(self._stage, self._baseLinkPath)
        rootLinkPrim = self._stage.GetPrimAtPath(self._baseLinkPath)

        # make it fixed base:
        if makeFixedBase:
            fixedJointPath = self._stage.GetDefaultPrim().GetPath().AppendChild("baseFixedJoint")
            fixedBaseJoint = UsdPhysics.FixedJoint.Define(self._stage, fixedJointPath)
            val1 = [rootLinkPrim.GetPath()]
            fixedBaseJoint.CreateBody1Rel().SetTargets(val1)
            UsdPhysics.ArticulationRootAPI.Apply(self._stage.GetDefaultPrim())
        else:
            # floating base apply to base
            UsdPhysics.ArticulationRootAPI.Apply(self._baseLink.GetPrim())

        # Box1
        position = Gf.Vec3f(0.0, self._linkLength, 0.0)
        color = Gf.Vec3f(21.0 / 255.0, 21.0 / 255.0, 165.0 / 255.0)
        prim = physicsUtils.add_rigid_box(self._stage, "/dynamicLink", size, position, orientation, color, densityCM)
        self._dynamicLinkPath = prim.GetPath()
        self._dynamicLink = UsdGeom.Cube.Define(self._stage, self._dynamicLinkPath)

    def setupRevoluteJoint(self, limits: tuple = None):
        # Add spherical joint between the two bars:
        jointPath = self._dynamicLinkPath.AppendChild("RevoluteJoint")
        self._revoluteJoint = UsdPhysics.RevoluteJoint.Define(self._stage, jointPath)
        if limits:
            self.assertLessEqual(limits[0], limits[1])
            self._revoluteJoint.CreateLowerLimitAttr(limits[0])
            self._revoluteJoint.CreateUpperLimitAttr(limits[1])
        self._revoluteJoint.CreateBody0Rel().SetTargets([self._baseLinkPath])
        self._revoluteJoint.CreateBody1Rel().SetTargets([self._dynamicLinkPath])
        axis = "X"
        self._revoluteJoint.CreateAxisAttr(axis)
        jointParentPosition = Gf.Vec3f(0, 0.5, 0)  # is scaled by link length
        jointParentPose = Gf.Quatf(1.0)
        jointChildPose = Gf.Quatf(1.0)
        self._revoluteJoint.CreateLocalPos0Attr().Set(jointParentPosition)
        self._revoluteJoint.CreateLocalRot0Attr().Set(jointParentPose)
        jointChildPos = Gf.Vec3f(0, -0.5, 0)  # is scaled by link length
        self._revoluteJoint.CreateLocalPos1Attr().Set(jointChildPos)
        self._revoluteJoint.CreateLocalRot1Attr().Set(jointChildPose)

    def setupFixedTendonOnRevoluteJoint(self, deflectionAngle: float):
        rootApi = PhysxSchema.PhysxTendonAxisRootAPI.Apply(self._revoluteJoint.GetPrim(), "t1")

        gravityTorque = self._gravityMagnitude * self._linkLength * self._linkMass * 0.5
        spring = gravityTorque / deflectionAngle
        damping = 0.1 * spring

        rootApi.CreateStiffnessAttr().Set(spring)
        rootApi.CreateRestLengthAttr().Set(0.0)
        rootApi.CreateDampingAttr().Set(damping)
        PhysxSchema.PhysxTendonAxisAPI(rootApi, "t1").CreateGearingAttr().Set([1.0])
        PhysxSchema.PhysxTendonAxisAPI(rootApi, "t1").CreateForceCoefficientAttr().Set([1.0])

        return rootApi

    def _getLinkAngle(self):
        xform = self._dynamicLink.GetLocalTransformation()
        self.assertTrue(xform.Orthonormalize(False))
        return xform.ExtractRotation().GetAngle()


class PhysxFixedTendonTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def setUp(self):
        # preist: Clean up Kit stage to avoid GPU Crash (OM-32311)
        await utils.new_stage_setup()
        await super().setUp()
        await self.setup_articulations()

    async def test_fixed_tendon_stiffness1(self):
        self._setup_two_link_articulation()
        upper = 20
        lower = -10
        deflectionAngle = 3.8  # [deg]
        self.setupRevoluteJoint(limits=(lower, upper))
        rootApi = self.setupFixedTendonOnRevoluteJoint(deflectionAngle)
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(deflectionAngle, linkAngle)
        self._test_float_equal(deflectionAngle, linkAngle, tol=2.0, msg=debugMessage)

        rootApi.GetStiffnessAttr().Set(0.0)  # tests change listener
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(lower, linkAngle)
        self._test_float_equal(-lower, linkAngle, tol=2.0, msg=debugMessage)

    async def test_fixed_tendon_stiffness2(self):
        self._setup_two_link_articulation()
        upper = 20
        lower = -10
        deflectionAngle = 5.0  # [deg]
        self.setupRevoluteJoint(limits=(lower, upper))
        rootApi = self.setupFixedTendonOnRevoluteJoint(deflectionAngle)
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(deflectionAngle, linkAngle)
        self._test_float_equal(deflectionAngle, linkAngle, tol=2.0, msg=debugMessage)

        rootApi.CreateTendonEnabledAttr().Set(False)  # tests change listener
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(lower, linkAngle)
        self._test_float_equal(-lower, linkAngle, tol=0.1, msg=debugMessage)

        rootApi.GetTendonEnabledAttr().Set(True)  # tests change listener
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(deflectionAngle, linkAngle)
        # preist larger tol ok because we just check that the re-enable lifter from the lower
        # limit again
        self._test_float_equal(deflectionAngle, linkAngle, tol=2.0, msg=debugMessage)


class PhysxFixedTendonTestKitStage(PhysicsKitStageAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def setUp(self):
        await super().setUp()
        await self.setup_articulations()

    async def test_apply_remove_tendon_axis_api(self):
        self._setup_two_link_articulation()
        self.setupRevoluteJoint()

        omni.kit.commands.execute(
            "AddPhysicsComponent",
            usd_prim=self._revoluteJoint.GetPrim(),
            component="PhysxTendonAxisAPI",
            multiple_api_token="test",
        )

        self.assertTrue(PhysxSchema.PhysxTendonAxisAPI.Get(self._revoluteJoint.GetPrim(), "test"))

        omni.kit.commands.execute(
            "RemovePhysicsComponent",
            usd_prim=self._revoluteJoint.GetPrim(),
            component="PhysxTendonAxisAPI",
            multiple_api_token="test",
        )

        self.assertFalse(PhysxSchema.PhysxTendonAxisAPI.Get(self._revoluteJoint.GetPrim(), "test"))

        omni.kit.undo.undo()

        self.assertTrue(PhysxSchema.PhysxTendonAxisAPI.Get(self._revoluteJoint.GetPrim(), "test"))

        omni.kit.undo.redo()

        self.assertFalse(PhysxSchema.PhysxTendonAxisAPI.Get(self._revoluteJoint.GetPrim(), "test"))
