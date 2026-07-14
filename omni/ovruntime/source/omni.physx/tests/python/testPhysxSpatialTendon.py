# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests PhysxSpatialTendon.py for standalone ovruntime testing.
# Only the MemoryStage class is ported; the KitStage and AuthoringTestKitStage classes require Kit.

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
import math
from physicsBase import PhysicsMemoryStageBaseTestCase, TestCategory
import physicsUtils
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema


class TendonArticulationTestBase:
    """Local ArticulationTestBase for tendon tests (different from shared articulationTestBase)."""

    def setup_articulations(self):
        self.new_stage(def_up_and_mpu=False)
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
            UsdPhysics.ArticulationRootAPI.Apply(self._baseLink.GetPrim())

        # Box1
        position = Gf.Vec3f(0.0, self._linkLength, 0.0)
        color = Gf.Vec3f(21.0 / 255.0, 21.0 / 255.0, 165.0 / 255.0)
        prim = physicsUtils.add_rigid_box(self._stage, "/dynamicLink", size, position, orientation, color, densityCM)
        self._dynamicLinkPath = prim.GetPath()
        self._dynamicLink = UsdGeom.Cube.Define(self._stage, self._dynamicLinkPath)

    def setupRevoluteJoint(self, limits: tuple = None):
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
        jointParentPosition = Gf.Vec3f(0, 0.5, 0)
        jointParentPose = Gf.Quatf(1.0)
        jointChildPose = Gf.Quatf(1.0)
        self._revoluteJoint.CreateLocalPos0Attr().Set(jointParentPosition)
        self._revoluteJoint.CreateLocalRot0Attr().Set(jointParentPose)
        jointChildPos = Gf.Vec3f(0, -0.5, 0)
        self._revoluteJoint.CreateLocalPos1Attr().Set(jointChildPos)
        self._revoluteJoint.CreateLocalRot1Attr().Set(jointChildPose)

    def setupSpatialTendon(self, name: str = None):
        if name is None:
            name = ""
        rootName = name + self._spatialTendonRootName
        leafName = name + self._spatialTendonLeafName
        self._stiffness = 10.0
        rootApi = PhysxSchema.PhysxTendonAttachmentRootAPI.Apply(self._baseLink.GetPrim(), rootName)
        rootOffset = Gf.Vec3f(0.0, 1.0, 2.5)
        PhysxSchema.PhysxTendonAttachmentAPI(rootApi, rootName).CreateLocalPosAttr().Set(rootOffset)
        rootApi.CreateStiffnessAttr().Set(self._stiffness)
        rootApi.CreateDampingAttr().Set(self._stiffness * 0.1)

        leafApi = PhysxSchema.PhysxTendonAttachmentLeafAPI.Apply(self._dynamicLink.GetPrim(), leafName)
        PhysxSchema.PhysxTendonAttachmentAPI(leafApi, leafName).CreateParentLinkRel().AddTarget(self._baseLinkPath)
        PhysxSchema.PhysxTendonAttachmentAPI(leafApi, leafName).CreateParentAttachmentAttr().Set(rootName)

        offset = self._linkMass * self._gravityMagnitude / self._stiffness
        rootApi.CreateOffsetAttr().Set(offset)

        return [rootApi, PhysxSchema.PhysxTendonAttachmentAPI(leafApi, leafName)]

    def _getLinkAngle(self):
        xform = self._dynamicLink.GetLocalTransformation()
        self.assertTrue(xform.Orthonormalize(False))
        return xform.ExtractRotation().GetAngle()


class PhysxSpatialTendonTestMemoryStage(PhysicsMemoryStageBaseTestCase, TendonArticulationTestBase):
    category = TestCategory.Core

    def setUp(self):
        super().setUp()
        self.setup_articulations()

    def test_spatial_tendon_stiffness(self):
        self._setup_two_link_articulation()
        lower = -10
        upper = 20
        self.setupRevoluteJoint(limits=(lower, upper))
        rootApi, _ = self.setupSpatialTendon()
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(0.0, linkAngle)
        self._test_float_equal(0.0, linkAngle, tol=1.0, msg=debugMessage)

        rootApi.CreateTendonEnabledAttr().Set(False)
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(lower, linkAngle)
        self._test_float_equal(-lower, linkAngle, tol=0.1, msg=debugMessage)

        rootApi.GetStiffnessAttr().Set(self._stiffness * 1.001)
        rootApi.GetDampingAttr().Set(self._stiffness * 0.101)
        self.step(num_steps=5)
        debugMessage = "Target angle = {}; link angle = {}".format(lower, linkAngle)
        self._test_float_equal(-lower, linkAngle, tol=0.1, msg=debugMessage)

        rootApi.GetTendonEnabledAttr().Set(True)
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(0.0, linkAngle)
        self._test_float_equal(0.0, linkAngle, tol=2.0, msg=debugMessage)

    def test_tendon_api_removal_during_simulation(self):
        self._setup_two_link_articulation()
        lower = -10
        upper = 20
        self.setupRevoluteJoint(limits=(lower, upper))
        self.setupSpatialTendon()

        self.step(num_steps=1)
        self._baseLink.GetPrim().RemoveAppliedSchema("PhysxTendonAttachmentRootAPI:root")
        self.step(num_steps=1)
        # nothing else to test, should simply not break


if __name__ == "__main__":
    unittest.main()
