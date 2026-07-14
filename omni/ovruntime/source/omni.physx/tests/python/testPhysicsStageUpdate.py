# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests PhysicsStageUpdate.py for standalone ovruntime testing.
# Only the PhysicsStageUpdateTestMemoryStage class is ported (no Kit dependencies).
#
# The original Kit tests call get_physics_stage_update_node_interface().detach_node()
# in setUp to disconnect the Kit orchestrator node. In standalone mode, there is no
# Kit node system, so those calls are simply removed.

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
from physicsBase import PhysicsMemoryStageBaseTestCase, TestCategory
import physicsUtils
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema, UsdUtils
import _physx


class PhysicsStageUpdateTestMemoryStage(PhysicsMemoryStageBaseTestCase):
    category = TestCategory.Core

    def setUp(self):
        super().setUp()

    def tearDown(self):
        # The base class tearDown handles stage cache cleanup.
        # Since all tests use attach_stage=False, _stage_attached stays False
        # and release_stage won't try to detach via the simulation interface.
        super().tearDown()

    def create_base_scene(self, stage):
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")
        sphereActorPath = "/sphereActor"

        radius = 1.0
        position = Gf.Vec3f(0.0, 0.0, 4.0)
        orientation = Gf.Quatf(1.0)

        self.spherePrim = physicsUtils.add_rigid_sphere(stage, sphereActorPath, radius, position, orientation)

    def test_stage_update_attached(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self.spherePrim = None
        self.create_base_scene(stage)

        stage_update_iface = _physx.acquire_physx_stage_update_interface()

        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})
        stage_update_iface.on_detach()

    def test_stage_update_resume(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self.spherePrim = None
        self.create_base_scene(stage)

        stage_update_iface = _physx.acquire_physx_stage_update_interface()

        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})

        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})
        stage_update_iface.on_detach()

    def test_stage_update_detach(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self.spherePrim = None
        self.create_base_scene(stage)

        stage_update_iface = _physx.acquire_physx_stage_update_interface()

        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})

        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})

        stage_update_iface.on_detach()
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})
        stage_update_iface.on_detach()

    def test_stage_update_reset(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self.spherePrim = None
        self.create_base_scene(stage)

        stage_update_iface = _physx.acquire_physx_stage_update_interface()

        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})

        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})

        stage_update_iface.on_reset()
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})
        stage_update_iface.on_detach()

    def test_stage_update_simulate_on(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        epsilon = 0.02

        self.spherePrim = None
        self.create_base_scene(stage)

        stage_update_iface = _physx.acquire_physx_stage_update_interface()

        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})

        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)

        for i in range(10):
            stage_update_iface.on_update(i * 0.02, 0.02, True)

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(pos[2] < 4.0)
        stage_update_iface.on_detach()

    def test_stage_update_simulate_off(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        epsilon = 0.02

        self.spherePrim = None
        self.create_base_scene(stage)

        stage_update_iface = _physx.acquire_physx_stage_update_interface()

        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})

        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)

        for i in range(10):
            stage_update_iface.on_update(i * 0.02, 0.02, False)

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)
        stage_update_iface.on_detach()

    def test_stage_update_simulate_reset(self):
        stage = self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        epsilon = 0.02

        self.spherePrim = None
        self.create_base_scene(stage)

        stage_update_iface = _physx.acquire_physx_stage_update_interface()

        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})

        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)

        for i in range(10):
            stage_update_iface.on_update(i * 0.02, 0.02, True)

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(pos[2] < 4.0)

        stage_update_iface.on_reset()

        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)
        stage_update_iface.on_detach()


if __name__ == "__main__":
    unittest.main()
