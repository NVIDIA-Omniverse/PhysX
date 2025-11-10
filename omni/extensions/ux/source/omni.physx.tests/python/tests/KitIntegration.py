# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx
import omni.physx.scripts.utils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physxtests import utils
from pxr import Usd, Gf, Sdf, UsdGeom, UsdPhysics, UsdUtils, PhysxSchema


class KitIntegrationTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    # OM-41245
    async def test_physics_pseudoroot_resync(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        boxActorPath = "/boxActor0"

        size = Gf.Vec3f(100.0)
        physicsUtils.add_rigid_box(stage, boxActorPath, size)
        
        self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})
        
        pseudo_root = stage.GetPseudoRoot()
        print(pseudo_root)
                
        rootLayer = stage.GetRootLayer()
        newLayer = Sdf.Layer.CreateAnonymous()
        stage.GetSessionLayer().subLayerPaths.append(newLayer.identifier)
        
        self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})
