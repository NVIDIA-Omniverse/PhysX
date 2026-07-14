# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests KitIntegration.py for standalone ovruntime testing.

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
from physicsBase import PhysicsMemoryStageBaseTestCase, TestCategory, check_stats
import physicsUtils
from pxr import Usd, Gf, Sdf, UsdGeom, UsdPhysics, UsdUtils, PhysxSchema


class KitIntegrationTestMemoryStage(PhysicsMemoryStageBaseTestCase):
    category = TestCategory.Core

    # OM-41245
    def test_physics_pseudoroot_resync(self):
        stage = self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        boxActorPath = "/boxActor0"

        size = Gf.Vec3f(100.0)
        physicsUtils.add_rigid_box(stage, boxActorPath, size)

        self.step()
        check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})

        pseudo_root = stage.GetPseudoRoot()
        print(pseudo_root)

        rootLayer = stage.GetRootLayer()
        newLayer = Sdf.Layer.CreateAnonymous()
        stage.GetSessionLayer().subLayerPaths.append(newLayer.identifier)

        self.step()
        check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})


if __name__ == "__main__":
    unittest.main()
