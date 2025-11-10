# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx
import omni.physx.scripts.utils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physxtests import utils
from pxr import Usd, Gf, Sdf, UsdGeom, UsdPhysics, UsdUtils, PhysxSchema


class PhysicsMeshCollisionAPITestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    # OM-37862
    async def test_physics_torus_vs_capsule(self):
        stage = await self.new_stage()
        timeline = omni.timeline.get_timeline_interface()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # create static torus
        path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + 'Torus', True))
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type='Torus')
        torus_prim = stage.GetPrimAtPath(path)
        UsdPhysics.CollisionAPI.Apply(torus_prim)
        
        # create dynamic capsule
        path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + 'Capsule', True))
        capsule = UsdGeom.Capsule.Define(stage, path)
        capsule.GetRadiusAttr().Set(25)
        capsule.GetHeightAttr().Set(50)
        capsule.GetAxisAttr().Set("Y")
        position = Gf.Vec3f(0.0, 200.0, 0)
        capsule.AddTranslateOp().Set(position)
        capsule_prim = capsule.GetPrim()
        UsdPhysics.CollisionAPI.Apply(capsule_prim)
        UsdPhysics.RigidBodyAPI.Apply(capsule_prim)

        timeline.play()

        # run simulation expect the capsule to fall through the torus hole
        await utils.play_and_step_and_pause(self, 50)

        new_position = capsule_prim.GetAttribute("xformOp:translate").Get()

        print("Capsule position after simulation: " + str(new_position))
        self.assertTrue(abs(new_position[0]) < 20.0)
        self.assertTrue(new_position[1] < 0.0)
        self.assertTrue(abs(new_position[2]) < 20.0)

        timeline.stop()
        await self.new_stage()
