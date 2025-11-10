# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math, random
from omni.physx.scripts.physicsUtils import *
from pxr import UsdGeom, Gf
import omni.physxdemos as demo
from .BasicSetup import get_usd_asset_path
from omni.debugdraw import get_debug_draw_interface

class OmniGraphMultipleSceneQueryDemo(demo.Base):
    title = "Multiple (OmniGraph)"
    category = demo.Categories.SCENE_QUERY
    short_description = "Demo using multiple Scene Query OmniGraph nodes with ActionGraph."
    description = "A demonstration of how to use PhysX Scene Query OmniGraph nodes with ActionGraph to detect colliders " \
        "in the scene."

    def create(self, stage, manual_animation=False):
        default_prim_path, scene = demo.setup_physics_scene(self, stage, primPathAsString = False, upAxis = UsdGeom.Tokens.y)

        self._manualAnimation = manual_animation
        self._stage = stage
        self._debugDraw = get_debug_draw_interface()
        self.hit_boxes = []
        self.globalTime = 0.0
        self.shapeType = 0
        self.text_array = None
        
        self.defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        createSQTestScene(stage, self.defaultPrimPath, 0.0, 2.0, 2.0)
        
        self._room = demo.get_demo_room(self, stage, zoom = 0.06, floorOffset = -2.0)

        for usdGeom in self.hit_boxes:
            if not usdGeom.GetDisplayColorAttr():
                usdGeom.CreateDisplayColorAttr()
            usdGeom.GetDisplayColorAttr().Set(Gf.Vec3f(0.0, 0.0, 1.0))

        self._stage.SetEndTimeCode(2147483648)

        graph = get_usd_asset_path("ogn_scene_query_multiple.usda")
        assert (stage.DefinePrim(default_prim_path).GetReferences().AddReference(graph))
        
    def on_shutdown(self):
        pass

# Copy of function in SceneQueryBaseDemo in the demo extension.
def createSQTestScene(stage, defaultPrimPath, killRadius, spread, heightScale):
    radius = 0.5
    size = Gf.Vec3f(radius*2.0)
    orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    color = demo.get_primary_color()
    linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
    angularVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
    density = 0.0

    nb_x = 32
    nb_y = 32
    halfScale = spread * 0.5
    offset_x = -(nb_x-1)*halfScale;
    offset_y = -(nb_y-1)*halfScale;
    count = 0
    random.seed(42)
    for y in range(0, nb_y):
        for x in range(0, nb_x):
            actorPath = defaultPrimPath + "/sqActors/actor" + str(count)
            count = count + 1

            rnd_x = random.uniform(-1.0, 1.0)
            rnd_y = random.uniform(-1.0, 1.0)
            rnd_z = random.uniform(-1.0, 1.0)
            rnd_w = random.uniform(-1.0, 1.0)
            orientation = Gf.Quatf(rnd_x, rnd_y, rnd_z, rnd_w)
            orientation.Normalize()

            rnd_x = random.uniform(-halfScale*0.5, halfScale*0.5)
            rnd_y = random.uniform(-halfScale*0.5, halfScale*0.5)
            rnd_z = random.uniform(0.0, heightScale)
            position = Gf.Vec3f(offset_x + rnd_x + x*spread, offset_y + rnd_y + y*spread, rnd_z)

            distToOrigin = math.sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2])
            if distToOrigin>killRadius:
                randomType = random.uniform(0.0, 1.0)

                if randomType > 0.75:
                    add_rigid_box(stage, actorPath, size, position, orientation, color, density, linVelocity, angularVelocity)
                else:
                    if randomType > 0.50:
                        add_rigid_sphere(stage, actorPath, radius, position, orientation, color, density, linVelocity, angularVelocity)
                    else:
                        if randomType > 0.25:
                            add_rigid_capsule(stage, actorPath, radius, radius*2.0, "Z", position, orientation, color, density, linVelocity, angularVelocity)
                        else:
                            add_rigid_cylinder(stage, actorPath, radius, radius*2.0, "Z", position, orientation, color, density, linVelocity, angularVelocity)
