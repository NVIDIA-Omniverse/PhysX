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

    def setupSpatialTendon(self, name: str = None):
        if name is None:
            name = ""
        rootName = name + self._spatialTendonRootName
        leafName = name + self._spatialTendonLeafName
        self._stiffness = 10.0
        rootApi = PhysxSchema.PhysxTendonAttachmentRootAPI.Apply(self._baseLink.GetPrim(), rootName)
        # center attachment above dynamic link
        rootOffset = Gf.Vec3f(0.0, 1.0, 2.5)
        PhysxSchema.PhysxTendonAttachmentAPI(rootApi, rootName).CreateLocalPosAttr().Set(rootOffset)
        rootApi.CreateStiffnessAttr().Set(self._stiffness)
        rootApi.CreateDampingAttr().Set(self._stiffness * 0.1)

        leafApi = PhysxSchema.PhysxTendonAttachmentLeafAPI.Apply(self._dynamicLink.GetPrim(), leafName)
        PhysxSchema.PhysxTendonAttachmentAPI(leafApi, leafName).CreateParentLinkRel().AddTarget(self._baseLinkPath)
        PhysxSchema.PhysxTendonAttachmentAPI(leafApi, leafName).CreateParentAttachmentAttr().Set(rootName)
        # leave localPos, rest length and gearing to default values

        offset = self._linkMass * self._gravityMagnitude / self._stiffness
        rootApi.CreateOffsetAttr().Set(offset)

        return [rootApi, PhysxSchema.PhysxTendonAttachmentAPI(leafApi, leafName)]

    def _getLinkAngle(self):
        xform = self._dynamicLink.GetLocalTransformation()
        self.assertTrue(xform.Orthonormalize(False))
        return xform.ExtractRotation().GetAngle()


class PhysxSpatialTendonTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def setUp(self):
        # preist: Clean up Kit stage to avoid GPU Crash (OM-32311)
        await utils.new_stage_setup()
        await super().setUp()
        await self.setup_articulations()

    async def test_spatial_tendon_stiffness(self):
        self._setup_two_link_articulation()
        lower = -10
        upper = 20
        self.setupRevoluteJoint(limits=(lower, upper))
        rootApi, _ = self.setupSpatialTendon()
        # self._stage.Export("testSpatialTendon.usda")
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(0.0, linkAngle)
        self._test_float_equal(0.0, linkAngle, tol=1.0, msg=debugMessage)

        rootApi.CreateTendonEnabledAttr().Set(False)  # tests change listener
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(lower, linkAngle)
        self._test_float_equal(-lower, linkAngle, tol=0.1, msg=debugMessage)

        rootApi.GetStiffnessAttr().Set(self._stiffness * 1.001)  # tests change listener
        rootApi.GetDampingAttr().Set(self._stiffness * 0.101)
        self.step(num_steps=5)
        debugMessage = "Target angle = {}; link angle = {}".format(lower, linkAngle)
        self._test_float_equal(-lower, linkAngle, tol=0.1, msg=debugMessage)

        rootApi.GetTendonEnabledAttr().Set(True)
        self.step(num_steps=20)
        linkAngle = self._getLinkAngle()
        debugMessage = "Target angle = {}; link angle = {}".format(0.0, linkAngle)
        # preist larger tol ok because we just check that the re-enable lifter from the lower
        # limit again
        self._test_float_equal(0.0, linkAngle, tol=2.0, msg=debugMessage)

    async def test_tendon_api_removal_during_simulation(self):
        self._setup_two_link_articulation()
        lower = -10
        upper = 20
        self.setupRevoluteJoint(limits=(lower, upper))
        self.setupSpatialTendon()

        self.step(num_steps=1)
        self._baseLink.GetPrim().RemoveAppliedSchema("PhysxTendonAttachmentRootAPI:root")
        self.step(num_steps=1)
        # nothing else to test, Kit should simply not break


class PhysxSpatialTendonTestKitStage(PhysicsKitStageAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def setUp(self):
        await super().setUp()
        await self.setup_articulations()

    async def test_undo_redo_apply_tendon_attachment_api(self):
        self._setup_two_link_articulation()
        self.setupRevoluteJoint()

        omni.kit.commands.execute(
            "AddPhysicsComponent",
            usd_prim=self._dynamicLink.GetPrim(),
            component="PhysxTendonAttachmentRootAPI",
            multiple_api_token="test",
        )

        self.assertTrue(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "test"))

        omni.kit.undo.undo()

        self.assertFalse(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "test"))

        omni.kit.undo.redo()

        self.assertTrue(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "test"))

    async def test_undo_redo_remove_all_tendon_attachment_apis(self):
        self._setup_two_link_articulation()
        self.setupRevoluteJoint()

        omni.kit.commands.execute(
            "AddPhysicsComponent",
            usd_prim=self._dynamicLink.GetPrim(),
            component="PhysxTendonAttachmentRootAPI",
            multiple_api_token="root1",
        )
        omni.kit.commands.execute(
            "AddPhysicsComponent",
            usd_prim=self._dynamicLink.GetPrim(),
            component="PhysxTendonAttachmentRootAPI",
            multiple_api_token="root2",
        )

        self.assertTrue(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "root1"))
        self.assertTrue(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "root2"))
        self.assertFalse(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "root3"))

        # this call shold remove both root APIs at once
        omni.kit.commands.execute(
            "RemovePhysicsComponent",
            usd_prim=self._dynamicLink.GetPrim(),
            component="PhysxTendonAttachmentRootAPI",
        )

        self.assertFalse(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "root1"))
        self.assertFalse(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "root2"))

        omni.kit.undo.undo()

        self.assertTrue(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "root1"))
        self.assertTrue(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "root2"))

        omni.kit.undo.redo()

        self.assertFalse(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "root1"))
        self.assertFalse(PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), "root2"))


class PhysxSpatialTendonAuthoringTestKitStage(PhysicsKitStageAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def setUp(self):
        await super().setUp()
        await self.setup_articulations()
        self._TENDON_VIZ_SETTING = omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS
        self._settings = carb.settings.get_settings()
        self._tendon_display_setting_restore = self._settings.get(self._TENDON_VIZ_SETTING)
        # hide all as default setting for tests
        self._settings.set_int(self._TENDON_VIZ_SETTING, 0)
        # setup basic articulation with two attachments:
        self._setup_two_link_articulation()
        self.setupRevoluteJoint()
        [self._rootAPI, self._leafAPI] = self.setupSpatialTendon()
        self.assertTrue(self._rootAPI)
        self.assertTrue(self._leafAPI)

    async def tearDown(self):
        # restore tendon visibility
        self._settings.set(self._TENDON_VIZ_SETTING, self._tendon_display_setting_restore)
        return await super().tearDown()

    @staticmethod
    def _get_session_xform_path(bodyPath: Sdf.Path) -> Sdf.Path:
        sessionScopePath = Sdf.Path("/PhysxTendonSpatialAttachmentGeoms")
        bodyPathStr = bodyPath.pathString.replace("/", "_")
        return sessionScopePath.AppendChild(bodyPathStr)

    @staticmethod
    def _get_session_geom_path(bodyPath: Sdf.Path, instanceName: str):
        return PhysxSpatialTendonAuthoringTestKitStage._get_session_xform_path(bodyPath).AppendChild(instanceName)

    async def test_spatial_tendon_authoring_helper_geom_creation(self):
        # make all visible:
        self._settings.set_int(self._TENDON_VIZ_SETTING, 2)
        await self.wait(1)

        # check session meshes:
        rootGeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(
            self._baseLinkPath, self._spatialTendonRootName
        )
        leafGeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(
            self._dynamicLinkPath, self._spatialTendonLeafName
        )

        self.assertTrue(UsdGeom.Sphere.Get(self._stage, rootGeomPath))
        self.assertTrue(UsdGeom.Sphere.Get(self._stage, leafGeomPath))

    async def test_spatial_tendon_authoring_delete_helper_geom(self):
        # make all visible:
        self._settings.set_int(self._TENDON_VIZ_SETTING, 2)
        await self.wait(1)

        # Delete the leaf and ensure that the link does not have the API anymore after
        leafGeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(
            self._dynamicLinkPath, self._spatialTendonLeafName
        )
        utils.execute_and_check(self, "DeletePrims", paths=[leafGeomPath])

        await self.wait(1)

        self.assertFalse(
            PhysxSchema.PhysxTendonAttachmentRootAPI.Get(self._dynamicLink.GetPrim(), self._spatialTendonLeafName)
        )

    async def test_spatial_tendon_authoring_add_new_api(self):
        # make all visible:
        self._settings.set_int(self._TENDON_VIZ_SETTING, 2)
        await self.wait(1)

        # Add a new leaf and check if the new geom is created:
        newLeafName = "newLeafName"
        leafApi = PhysxSchema.PhysxTendonAttachmentLeafAPI.Apply(self._dynamicLink.GetPrim(), newLeafName)
        PhysxSchema.PhysxTendonAttachmentAPI(leafApi, newLeafName).CreateParentLinkRel().AddTarget(self._baseLinkPath)
        PhysxSchema.PhysxTendonAttachmentAPI(leafApi, newLeafName).CreateParentAttachmentAttr().Set(self._spatialTendonRootName)

        await self.wait(1)

        leafGeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(self._dynamicLinkPath, newLeafName)
        self.assertTrue(UsdGeom.Sphere.Get(self._stage, leafGeomPath))

    async def test_spatial_tendon_authoring_update_from_localPos(self):
        # make all visible:
        self._settings.set_int(self._TENDON_VIZ_SETTING, 2)
        await self.wait(1)

        leafGeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(
            self._dynamicLinkPath, self._spatialTendonLeafName
        )
        # get initial translation that must be identical to the localpos:
        leafGeom = UsdGeom.Sphere.Get(self._stage, leafGeomPath)
        translation = leafGeom.GetLocalTransformation().ExtractTranslation()

        # Add a new leaf and check if the new geom is created:
        oldOffset = self._leafAPI.GetLocalPosAttr().Get()
        self._assertAlmostEqualVector3(translation, oldOffset)
        newOffset = 2.0 * oldOffset
        self._leafAPI.CreateLocalPosAttr().Set(newOffset)

        await self.wait(1)

        translation = leafGeom.GetLocalTransformation().ExtractTranslation()
        self._assertAlmostEqualVector3(translation, newOffset)

    async def test_spatial_tendon_authoring_update_to_localPos(self):
        # make all visible:
        self._settings.set_int(self._TENDON_VIZ_SETTING, 2)
        await self.wait(1)

        leafGeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(
            self._dynamicLinkPath, self._spatialTendonLeafName
        )
        # get initial translation that must be identical to the localpos:
        leafGeom = UsdGeom.Sphere.Get(self._stage, leafGeomPath)
        translation = leafGeom.GetLocalTransformation().ExtractTranslation()

        # Add a new leaf and check if the new geom is created:
        oldOffset = self._leafAPI.GetLocalPosAttr().Get()
        self._assertAlmostEqualVector3(translation, oldOffset)
        newOffset = 2.0 * oldOffset
        leafGeom.GetPrim().GetAttribute("xformOp:translate").Set(newOffset)

        await self.wait(1)

        newLocalPos = self._leafAPI.GetLocalPosAttr().Get()
        self._assertAlmostEqualVector3(newLocalPos, newOffset)

    async def test_spatial_tendon_authoring_link_move(self):
        # make all visible:
        self._settings.set_int(self._TENDON_VIZ_SETTING, 2)
        await self.wait(1)

        leafGeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(
            self._dynamicLinkPath, self._spatialTendonLeafName
        )
        # get initial translation that must be identical to the localpos:
        leafGeom = UsdGeom.Sphere.Get(self._stage, leafGeomPath)
        initialTranslation = Gf.Vec3f(
            leafGeom.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        )

        # Move parent link
        delta = Gf.Vec3f(1.0, 2.0, 3.0)
        linkPos = Gf.Vec3f(self._dynamicLink.GetPrim().GetAttribute("xformOp:translate").Get())
        linkPos += delta
        self._dynamicLink.GetPrim().GetAttribute("xformOp:translate").Set(linkPos)

        await self.wait(1)

        movedTranslsation = leafGeom.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self._assertAlmostEqualVector3(initialTranslation + delta, movedTranslsation)

    async def test_spatial_tendon_autoconnect(self):
        # helpers
        async def setSelection(selection):
            omni.kit.commands.execute(
                "SelectPrims",
                old_selected_paths=[],
                new_selected_paths=selection,
                expand_in_stage=False,
            )
            await self.wait(2)

        async def resetParent(api):
            api.GetParentAttachmentAttr().Set("")
            api.GetParentLinkRel().SetTargets([])
            await self.wait(1)

        def assertNoParentSet(api):
            self.assertEqual(0, len(api.GetParentLinkRel().GetTargets()))
            self.assertFalse(api.GetParentAttachmentAttr().Get())

        # make all visible:
        self._settings.set_int(self._TENDON_VIZ_SETTING, 2)

        # create new attachments for connecting:
        attName = "att"
        attachmentAPI = PhysxSchema.PhysxTendonAttachmentAPI.Apply(self._dynamicLink.GetPrim(), attName)
        attName2 = "att2"
        attachmentAPI2 = PhysxSchema.PhysxTendonAttachmentAPI.Apply(self._dynamicLink.GetPrim(), attName2)
        leafName2 = "leaf2"
        leafAPI2 = PhysxSchema.PhysxTendonAttachmentLeafAPI.Apply(self._dynamicLink.GetPrim(), leafName2)
        leafAPI = PhysxSchema.PhysxTendonAttachmentLeafAPI.Apply(self._dynamicLink.GetPrim(), self._spatialTendonLeafName)
        rootAPI = PhysxSchema.PhysxTendonAttachmentRootAPI.Apply(self._baseLink.GetPrim(), self._spatialTendonRootName)

        # update to create all helper geoms:
        await self.wait(1)

        rootGeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(
            self._baseLinkPath, self._spatialTendonRootName
        )
        leafGeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(
            self._dynamicLinkPath, self._spatialTendonLeafName
        )
        attGeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(self._dynamicLinkPath, attName)
        att2GeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(self._dynamicLinkPath, attName2)
        leaf2GeomPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(self._dynamicLinkPath, leafName2)

        # connecting two attachments where both do not have parent info results in setting parent of first selected
        # to second selected:
        await setSelection([attGeomPath.pathString, att2GeomPath.pathString])
        self.assertEqual(self._dynamicLinkPath, attachmentAPI.GetParentLinkRel().GetTargets()[0])
        self.assertEqual(attName2, attachmentAPI.GetParentAttachmentAttr().Get())
        assertNoParentSet(attachmentAPI2)
        await resetParent(attachmentAPI)
        # other direction
        await setSelection([att2GeomPath.pathString, attGeomPath.pathString])
        self.assertEqual(self._dynamicLinkPath, attachmentAPI2.GetParentLinkRel().GetTargets()[0])
        self.assertEqual(attName, attachmentAPI2.GetParentAttachmentAttr().Get())
        assertNoParentSet(attachmentAPI)
        await resetParent(attachmentAPI2)

        # connecting an attachment to a leaf must result in the parent info of the leaf being set in either selection
        # order
        await setSelection([attGeomPath.pathString, leaf2GeomPath.pathString])
        self.assertEqual(self._dynamicLinkPath, PhysxSchema.PhysxTendonAttachmentAPI(leafAPI2, leafName2).GetParentLinkRel().GetTargets()[0])
        self.assertEqual(attName, PhysxSchema.PhysxTendonAttachmentAPI(leafAPI2, leafName2).GetParentAttachmentAttr().Get())
        self.assertEqual(0, len(attachmentAPI.GetParentLinkRel().GetTargets()))
        self.assertFalse(attachmentAPI.GetParentAttachmentAttr().Get())
        await resetParent(PhysxSchema.PhysxTendonAttachmentAPI(leafAPI2, leafName2))
        # other dir
        await setSelection([leaf2GeomPath.pathString, attGeomPath.pathString])
        self.assertEqual(self._dynamicLinkPath, PhysxSchema.PhysxTendonAttachmentAPI(leafAPI2, leafName2).GetParentLinkRel().GetTargets()[0])
        self.assertEqual(attName, PhysxSchema.PhysxTendonAttachmentAPI(leafAPI2, leafName2).GetParentAttachmentAttr().Get())
        assertNoParentSet(attachmentAPI)
        await resetParent(PhysxSchema.PhysxTendonAttachmentAPI(leafAPI2, leafName2))

        # try to connect it to existing leaf must fail:
        await setSelection([leafGeomPath.pathString, attGeomPath.pathString])
        assertNoParentSet(attachmentAPI)
        self.assertEqual(self._baseLinkPath, PhysxSchema.PhysxTendonAttachmentAPI(leafAPI, self._spatialTendonLeafName).GetParentLinkRel().GetTargets()[0])
        self.assertEqual(self._spatialTendonRootName, PhysxSchema.PhysxTendonAttachmentAPI(leafAPI, self._spatialTendonLeafName).GetParentAttachmentAttr().Get())

        # connecting to leaves must fail:
        await setSelection([leafGeomPath.pathString, leaf2GeomPath.pathString])
        assertNoParentSet(PhysxSchema.PhysxTendonAttachmentAPI(leafAPI2, leafName2))
        self.assertEqual(self._baseLinkPath, PhysxSchema.PhysxTendonAttachmentAPI(leafAPI, self._spatialTendonLeafName).GetParentLinkRel().GetTargets()[0])
        self.assertEqual(self._spatialTendonRootName, PhysxSchema.PhysxTendonAttachmentAPI(leafAPI, self._spatialTendonLeafName).GetParentAttachmentAttr().Get())

        # auto-connect to existing root must work both ways:
        await setSelection([rootGeomPath.pathString, attGeomPath.pathString])
        assertNoParentSet(PhysxSchema.PhysxTendonAttachmentAPI(rootAPI, self._spatialTendonRootName))
        self.assertEqual(self._baseLinkPath, attachmentAPI.GetParentLinkRel().GetTargets()[0])
        self.assertEqual(self._spatialTendonRootName, attachmentAPI.GetParentAttachmentAttr().Get())
        await resetParent(attachmentAPI)

        await setSelection([attGeomPath.pathString, rootGeomPath.pathString])
        assertNoParentSet(PhysxSchema.PhysxTendonAttachmentAPI(rootAPI, self._spatialTendonRootName))
        self.assertEqual(self._baseLinkPath, attachmentAPI.GetParentLinkRel().GetTargets()[0])
        self.assertEqual(self._spatialTendonRootName, attachmentAPI.GetParentAttachmentAttr().Get())
        # no reset here for tests below

        # connecting two attachments where one has parent info results in setting parent of the one without parent info:
        await setSelection([attGeomPath.pathString, att2GeomPath.pathString])
        self.assertEqual(self._dynamicLinkPath, attachmentAPI2.GetParentLinkRel().GetTargets()[0])
        self.assertEqual(attName, attachmentAPI2.GetParentAttachmentAttr().Get())
        self.assertEqual(self._baseLinkPath, attachmentAPI.GetParentLinkRel().GetTargets()[0])
        self.assertEqual(self._spatialTendonRootName, attachmentAPI.GetParentAttachmentAttr().Get())
        await resetParent(attachmentAPI2)
        # other direction
        await setSelection([att2GeomPath.pathString, attGeomPath.pathString])
        self.assertEqual(self._dynamicLinkPath, attachmentAPI2.GetParentLinkRel().GetTargets()[0])
        self.assertEqual(attName, attachmentAPI2.GetParentAttachmentAttr().Get())
        self.assertEqual(self._baseLinkPath, attachmentAPI.GetParentLinkRel().GetTargets()[0])
        self.assertEqual(self._spatialTendonRootName, attachmentAPI.GetParentAttachmentAttr().Get())
        await resetParent(attachmentAPI2)

    #@unittest.skip("OM-41039")
    async def test_spatial_tendon_authoring_parent_updates_and_visibility(self):
        # store selection:
        selection = omni.usd.get_context().get_selection()
        selected_paths = selection.get_selected_prim_paths()
        # make selected visible:
        self._settings.set_int(self._TENDON_VIZ_SETTING, 1)

        # delete parent rel of leaf:

        rootPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(
            self._baseLinkPath, self._spatialTendonRootName
        )
        leafPath = PhysxSpatialTendonAuthoringTestKitStage._get_session_geom_path(
            self._dynamicLinkPath, self._spatialTendonLeafName
        )

        leafApi = PhysxSchema.PhysxTendonAttachmentLeafAPI.Get(
            self._stage.GetPrimAtPath(self._dynamicLinkPath), self._spatialTendonLeafName
        )
        PhysxSchema.PhysxTendonAttachmentAPI(leafApi, self._spatialTendonLeafName).CreateParentAttachmentAttr().Set("")

        # need to wait one cycle for bodies with attchments to be registered with the visualizer
        await self.wait(1)

        # select dynamic link:
        selection.set_selected_prim_paths([self._dynamicLinkPath.pathString], False)

        # need to wait one cycle for session leaf geom to be created
        await self.wait(1)

        rootIm = UsdGeom.Imageable.Get(self._stage, rootPath)
        leafIm = UsdGeom.Imageable.Get(self._stage, leafPath)
    
        # since the attachment is not connected to the root yet, only the dynamic link leaf attachment is visible
        self.assertEqual("inherited", leafIm.ComputeVisibility(Usd.TimeCode.Default()))
        # and the root has not been created yet
        self.assertFalse(rootIm)

        # update parent:
        PhysxSchema.PhysxTendonAttachmentAPI(leafApi, self._spatialTendonLeafName).CreateParentAttachmentAttr().Set(self._spatialTendonRootName)

        await self.wait(1)

        # now both must be visible
        self.assertEqual("inherited", leafIm.ComputeVisibility(Usd.TimeCode.Default()))
        rootIm = UsdGeom.Imageable.Get(self._stage, rootPath)
        self.assertEqual("inherited", rootIm.ComputeVisibility(Usd.TimeCode.Default()))

        # de-select all and both should disappear
        selection.set_selected_prim_paths([], False)
        await self.wait(2)
        self.assertEqual("invisible", leafIm.ComputeVisibility(Usd.TimeCode.Default()))
        self.assertEqual("invisible", rootIm.ComputeVisibility(Usd.TimeCode.Default()))

        # restore selection
        selected_paths = selection.set_selected_prim_paths(selected_paths, False)

    async def test_spatial_tendon_attachment_candidate_extraction(self):
        get_spatial_tendon_attachment_candidates(Usd.Prim())
        # create new attachment:
        attNames = ["att"]
        PhysxSchema.PhysxTendonAttachmentAPI.Apply(self._dynamicLink.GetPrim(), attNames[-1])
        attNames.append("att2")
        PhysxSchema.PhysxTendonAttachmentAPI.Apply(self._dynamicLink.GetPrim(), attNames[-1])
        attNames.append("root2")
        PhysxSchema.PhysxTendonAttachmentRootAPI.Apply(self._dynamicLink.GetPrim(), attNames[-1])
        PhysxSchema.PhysxTendonAttachmentLeafAPI.Apply(self._dynamicLink.GetPrim(), "anotherLeaf")

        # on the base, must only get the root:
        baseCandidates = get_spatial_tendon_attachment_candidates(self._baseLink.GetPrim())
        self.assertEqual(len(baseCandidates), 1)
        self.assertEqual(baseCandidates[0], self._spatialTendonRootName)

        # on the dynamic link, must only get the attachments in attNames, NOT any leaf attachments:
        dynamicCandidates = get_spatial_tendon_attachment_candidates(self._dynamicLink.GetPrim())
        self.assertEqual(len(dynamicCandidates), len(attNames))
        self.assertEqual(dynamicCandidates, attNames)
