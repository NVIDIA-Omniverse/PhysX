# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, Gf, UsdGeom, UsdPhysics, PhysxSchema
from omni.physxcct import get_physx_cct_interface
import omni.timeline
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase
from carb.eventdispatcher import get_eventdispatcher


class PhysXCctRegressionTests(PhysicsKitStageAsyncTestCase):
    async def test_regression_OM32058(self):
        stage = await self.new_stage()
        path = "/World/Capsule"
        capsule = UsdGeom.Capsule.Define(stage, path)
        capsule_cctapi = PhysxSchema.PhysxCharacterControllerAPI.Apply(capsule.GetPrim())

        physxcct = get_physx_cct_interface()
        physxcct.disable_gravity(path)

        # set init value and step
        init_move = Gf.Vec3f(1.0)
        capsule_cctapi.GetMoveTargetAttr().Set(init_move)

        # create update loop that changes the value during timeline play
        def on_update(_):
            if omni.timeline.get_timeline_interface().is_playing():
                capsule_cctapi.GetMoveTargetAttr().Set(Gf.Vec3f(2.0))

        update_sub = get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=on_update,
            observer_name="omni.physx.cct test_regression_OM32058",
        )

        # expect reset to init value after stop
        await self.step(5, stop_timeline_after=True)
        move_target = capsule_cctapi.GetMoveTargetAttr().Get()
        self.assertTrue(move_target == init_move)

        update_sub = None

    async def test_regression_OM36106(self):
        stage = await self.new_stage()
        path = "/World/Capsule"
        capsule = UsdGeom.Capsule.Define(stage, path)
        init_pose = capsule.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        PhysxSchema.PhysxCharacterControllerAPI.Apply(capsule.GetPrim())

        physxcct = get_physx_cct_interface()
        physxcct.disable_gravity(path)

        # expect reset of the initial pose
        await self.step(5, stop_timeline_after=True)

        stop_pose = capsule.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        epsilon = 0.001
        self.assertTrue(Gf.IsClose(init_pose, stop_pose, epsilon))
