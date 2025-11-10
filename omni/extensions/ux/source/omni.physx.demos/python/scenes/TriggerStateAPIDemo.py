# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb, omni.timeline
import os
from omni.physx.scripts.physicsUtils import *
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, Vt, UsdUtils, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
from omni.debugdraw import get_debug_draw_interface
import omni.physx.scripts.utils as core_utils


class TriggerStateAPIDemo(demo.Base):
    title = "TriggerStateAPI"
    category = demo.Categories.TRIGGERS
    short_description = "Demo showing physics trigger state API usage"
    description = "Demo showing physics trigger usage with PhysxTriggerStateAPI. The blue box should fall to the ground, turning green during its fall as it enters a trigger, and turn back blue after it leaves the trigger. Triggers can be defined on a primitive with collisionAPI schema applied when the PhysX properties window is enabled. Press play (space) to run the simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage)

        # Box
        boxActorPath = defaultPrimPath + "/boxActor"

        size = Gf.Vec3f(25.0)
        position = Gf.Vec3f(0.0, 0.0, 300.0)
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        color = demo.get_primary_color(0)

        add_rigid_box(stage, boxActorPath, size, position, orientation, color)

        self._boxSize = 100.0
        self._boxPos = Gf.Vec3f(0.0, 0.0, 200.0)

        # Trigger
        self.triggerPath = defaultPrimPath + "/boxTrigger"
        triggerBox = UsdGeom.Cube.Define(stage, self.triggerPath)
        triggerBox.CreateSizeAttr(self._boxSize)
        triggerBox.AddTranslateOp().Set(self._boxPos)
        triggerBox.CreatePurposeAttr(UsdGeom.Tokens.guide)
        triggerUsdPrim = stage.GetPrimAtPath(self.triggerPath)
        UsdPhysics.CollisionAPI.Apply(triggerUsdPrim)
        PhysxSchema.PhysxTriggerAPI.Apply(triggerUsdPrim)
        self.triggerStateAPI = PhysxSchema.PhysxTriggerStateAPI.Apply(triggerUsdPrim)
        self.triggerCollisions = []

        self._boxPosWorld = None

    def update(self, stage, dt, viewport, physxIFace):
        self._stage = stage
        if not self._boxPosWorld:
            self._boxPosWorld = core_utils.get_world_position(stage, self.triggerPath)

        box_color = 0xffffff00
        get_debug_draw_interface().draw_box(self._boxPosWorld, carb.Float4(0.0, 0.0, 0.0, 1.0), Gf.Vec3f(self._boxSize), box_color, 3.0)

        triggerColliders = self.triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        set_difference = set(triggerColliders).symmetric_difference(set(self.triggerCollisions))
        list_difference = list(set_difference)
        self.triggerCollisions = triggerColliders

        if len(triggerColliders) > 0:
            for collision in triggerColliders:
                usdGeom = UsdGeom.Mesh.Get(stage, collision)
                color = Vt.Vec3fArray([demo.get_hit_color()])
                usdGeom.GetDisplayColorAttr().Set(color)

        for collision in list_difference:
            usdGeom = UsdGeom.Mesh.Get(stage, collision)
            color = Vt.Vec3fArray([demo.get_primary_color()])
            usdGeom.GetDisplayColorAttr().Set(color)
