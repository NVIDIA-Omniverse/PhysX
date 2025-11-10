# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb, math, omni.timeline
from omni.physx.scripts.physicsUtils import *
from pxr import UsdGeom, Gf, Vt
import omni.physxdemos as demo
from omni.physx import get_physx_scene_query_interface
from omni.debugdraw import get_debug_draw_interface
from .SceneQueryBaseDemo import createSQTestScene
from omni.physxui import get_physicsui_instance
from .DebugDrawHelpers import *
from .KeyboardHelpers import *

class OverlapMultipleDemo(demo.Base):
    title = "Overlap Multiple"
    category = demo.Categories.SCENE_QUERY
    short_description = "Demo showing 'overlap multiple' usage"
    description = "This example shows how to perform an 'overlap multiple' call with a sphere or box shape.\n\nAn 'overlap multiple' query is one that can return multiple results. \
In this case, we query the scene with the wireframe sphere or box and retrieve all the objects it touches - which are then rendered in green.\n\nPress play (space) to run the simulation."

    def create(self, stage, manual_animation=False):
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

        self._sub_keyboard = init_keyboard(self._on_keyboard_event)
        self._info_box = None
        self._info_box = None

    def on_shutdown(self):
        close_keyboard(self._sub_keyboard)
        self._sub_keyboard = None
        super().on_shutdown()

        if self._info_box:
            self._info_box.destroy()
            self._info_box = None

    def draw_info_text(self):
        text_shape = ""
        if self.shapeType == 0:
            text_shape = "Current shape: sphere"
        elif self.shapeType == 1:
            text_shape = "Current shape: box"

        if not self.text_array:
            self.text_array = ["'Overlap Multiple' sample. Press 1 or 2 to change query shape.", text_shape, ""]

        if self._room._captureUIText:
            self._room.update_ui_text(self.text_array)
        else:
            if not self._info_box:
                physicsui = get_physicsui_instance()
                if physicsui:
                    self._info_box = physicsui.create_text_info_box(self.text_array, screen_coords=[0, -200])
            else:
                self._info_box.set_text(self.text_array)


    # PT: this function is called by perform_sphere_overlap or perform_box_overlap for each result.
    def report_hit(self, hit):
        usdGeom = UsdGeom.Mesh.Get(self._stage, hit.rigid_body)
        if (hit.rigid_body != self._room._tableTopPath): # ignore the table top in the base scene for better visibility
            self.hit_boxes.append(usdGeom)
        return True # return True to continue the query

    def perform_sphere_overlap(self, origin, radius):
        # PT: we pass 'False' to overlap_sphere() to indicate an 'overlap multiple' query.
        numHits = get_physx_scene_query_interface().overlap_sphere(radius, origin, self.report_hit, False)
        return numHits

    def perform_box_overlap(self, origin, extent, rot):
        # PT: we pass 'False' to overlap_box() to indicate an 'overlap multiple' query.
        numHits = get_physx_scene_query_interface().overlap_box(extent, origin, rot, self.report_hit, False)
        return numHits

    def set_colors(self, stage, color, reset_array):
        for usdGeom in self.hit_boxes:
            if not usdGeom.GetDisplayColorAttr():
                usdGeom.CreateDisplayColorAttr()
            usdGeom.GetDisplayColorAttr().Set(color)

        if reset_array:
            self.hit_boxes = []
        return

    def draw_number_of_hits(self, numHits):
        if self.text_array:
            self.text_array[2] = "Number of hits: " + str(numHits)

    def update(self, stage, dt, viewport, physxIFace):
        self._stage = stage

        if omni.timeline.get_timeline_interface().is_playing() or self._manualAnimation:
            self.globalTime += dt

        self.draw_info_text()

        with Sdf.ChangeBlock():
            origColor = Vt.Vec3fArray([demo.get_primary_color()])
            self.set_colors(stage, origColor, True)

            numHits = 0

            origin = Gf.Vec3f(0.0) + Gf.Vec3f(self._room._originWorld)

            if self.shapeType == 0:
                radius = 20.0 + math.sin(self.globalTime * 0.5) * 15.0
                #radius = 35.0   #use this fixed radius for benchmarks

                draw_sphere(self._debugDraw, origin, radius, 0xFFFFFF00)
                numHits = self.perform_sphere_overlap(origin, radius)

            elif self.shapeType == 1:
                boxSize = 10.0 + math.sin(self.globalTime * 0.5) * 7.5
                rot = carb.Float4(0.0, 0.0, 0.0, 1.0)
                extent = carb.Float3(boxSize, boxSize, boxSize)

                draw_box(self._debugDraw, origin, extent, color = 0xFFFFFF00)
                numHits = self.perform_box_overlap(origin, extent, rot)

            self.draw_number_of_hits(numHits)

            hitColor = Vt.Vec3fArray([demo.get_hit_color()])
            self.set_colors(stage, hitColor, False)

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == carb.input.KeyboardInput.KEY_1:
                self.shapeType = 0
            elif event.input == carb.input.KeyboardInput.KEY_2:
                self.shapeType = 1
        return True
