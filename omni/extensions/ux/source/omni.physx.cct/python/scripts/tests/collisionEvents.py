# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools
from omni.physxcct import get_physx_cct_interface, CctEvent
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase
from omni.physx.scripts import physicsUtils


class PhysXCctCollisionEventsTests(PhysicsKitStageAsyncTestCase):

    def setup_scene(self, stage):

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # Physics scene
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        self._cct_path = "/capsule"

        capsule_geom = UsdGeom.Capsule.Define(stage, self._cct_path)
        capsule_geom.CreateHeightAttr(50)
        capsule_geom.CreateRadiusAttr(25)
        capsule_geom.CreateAxisAttr("Z")
        # capsule_geom.CreatePurposeAttr(UsdGeom.Tokens.guide)

        capsule_geom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 75.0))
        capsule_geom.AddOrientOp().Set(Gf.Quatf(1.0))
        capsule_geom.AddScaleOp().Set(Gf.Vec3f(1.0))

        PhysxSchema.PhysxCharacterControllerAPI.Apply(capsule_geom.GetPrim())

    def _on_simulation_event(self, event):
        if event.type == int(CctEvent.COLLISION_DOWN):
            self.collision_down = event.payload['collision']
            self.cct_path = PhysicsSchemaTools.decodeSdfPath(event.payload['cctPath'][0], event.payload['cctPath'][1])
        if event.type == int(CctEvent.COLLISION_UP):
            self.collision_up = event.payload['collision']
            self.cct_path = PhysicsSchemaTools.decodeSdfPath(event.payload['cctPath'][0], event.payload['cctPath'][1])
        if event.type == int(CctEvent.COLLISION_SIDES):
            self.collision_sides = event.payload['collision']
            self.cct_path = PhysicsSchemaTools.decodeSdfPath(event.payload['cctPath'][0], event.payload['cctPath'][1])

    async def test_cct_collision_down_event(self):
        physxCct = get_physx_cct_interface()

        events = physxCct.get_cct_event_stream()
        self._simulation_event_sub = events.create_subscription_to_pop(self._on_simulation_event)

        self.collision_down = False
        self.collision_up = False
        self.collision_sides = False

        self.cct_path = None

        self.stage = await self.new_stage()

        self.setup_scene(self.stage)

        physxCct.enable_gravity(self._cct_path)

        prim = self.stage.GetPrimAtPath(self._cct_path)
        print(prim.GetAttribute("xformOp:translate").Get())

        for _ in range(75):
            print(prim.GetAttribute("xformOp:translate").Get())
            await self.step()

        self.assertTrue(self.collision_down)
        self.assertTrue(self.cct_path == self._cct_path)

        self._simulation_event_sub = None
