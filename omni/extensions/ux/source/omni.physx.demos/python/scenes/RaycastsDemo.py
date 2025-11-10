# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb, math, omni.timeline
from omni.physx.scripts.physicsUtils import *
from pxr import UsdGeom, Gf, Vt
import omni.physxdemos as demo
from omni.physx import get_physx_scene_query_interface
from omni.debugdraw import get_debug_draw_interface
from .SceneQueryBaseDemo import *
from omni.physxui import get_physicsui_instance
from .DebugDrawHelpers import *
from .KeyboardHelpers import *

class RaycastsDemo(demo.Base):
    title = "Raycasts"
    category = demo.Categories.SCENE_QUERY
    short_description = "Demo showing raycast usage"
    description = "This example shows how to use raycasts.\n\nThree types of raycasts are demonstrated: 'raycast closest', 'raycast all' and 'raycast any'.\n\n\
Follow the instructions on screen.\n\nPress play (space) to run the simulation."

    def create(self, stage, manual_animation=False):
        self._manualAnimation = manual_animation
        self._stage = stage
        self._debugDraw = get_debug_draw_interface()
        self.hit_boxes = []
        self.globalTime = 0.0
        self.raycastMode = 0

        self.defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        createSQTestScene(stage, self.defaultPrimPath, 16.0, 2.0, 1.0)

        self._sub_keyboard = init_keyboard(self._on_keyboard_event)

        room = demo.get_demo_room(self, stage, zoom = 0.06, floorOffset = -2.0)
        self._info_box = None

    def on_shutdown(self):
        close_keyboard(self._sub_keyboard)
        self._sub_keyboard = None
        if self._info_box:
            self._info_box.destroy()
            self._info_box = None

    def draw_info_text(self):
        text = ["Raycast sample. Press 1, 2 or 3 to change raycast mode."]

        if self.raycastMode == 0:
            text.append("Current mode: raycast closest (single hit query)")
            text.append("The query reports the closest touched object, and hit information like impact position, normal, etc.")
            text.append("")
            text.append("We draw a small frame at the impact position in this sample.")
            text.append("This is the most common raycast query, used to implement bullets or picking.")
        elif self.raycastMode == 1:
            text.append("Current mode: raycast all (multiple hits query)")
            text.append("The query reports all touched objects and their information.")
            text.append("")
            text.append("In this sample we render these touched objects in green.")
            text.append("This query can be useful to implement custom filtering for example.")
        elif self.raycastMode == 2:
            text.append("Current mode: raycast any (boolean hit query)")
            text.append("The query reports whether the ray touches an object or not.")
            text.append("It early exits as soon as it finds a hit, even if it is not the closest one.")
            text.append("The ray is rendered in red if there was a hit, green if there was not.")
            text.append("This query can be useful to implement shadow rays or visibility rays for AI.")

        if self._room._captureUIText:
            self._room.update_ui_text(text)
        else:
            if not self._info_box:
                physicsui = get_physicsui_instance()
                if physicsui:
                    self._info_box = physicsui.create_text_info_box(text, screen_coords=[0, -200])
            else:
                self._info_box.set_text(text)

    def set_colors(self, stage, color, reset_array):
        for usdGeom in self.hit_boxes:
            usdGeom.GetDisplayColorAttr().Set(color)

        if reset_array:
            self.hit_boxes = []
        return

    def report_all_hits(self, hit):
        if "sqActors" in hit.rigid_body:
            usdGeom = UsdGeom.Mesh.Get(self._stage, hit.rigid_body)
            self.hit_boxes.append(usdGeom)
        return True

    def update(self, stage, dt, viewport, physxIFace):
        self._stage = stage

        if omni.timeline.get_timeline_interface().is_playing() or self._manualAnimation:
            self.globalTime += dt

        self.draw_info_text()

        origin = Gf.Vec3f(0.0) + Gf.Vec3f(self._room._originWorld)
        rayDir = Gf.Vec3f(0.0, 0.0, 0.0)
        rayDir[0] = math.sin(self.globalTime*0.1)
        rayDir[1] = math.cos(self.globalTime*0.1)

        maxDist = 1000.0
        endPoint = Gf.Vec3f(origin[0] + rayDir[0]*maxDist, origin[1] + rayDir[1]*maxDist, origin[2] + rayDir[2]*maxDist)

        with Sdf.ChangeBlock():
            origColor = Vt.Vec3fArray([demo.get_primary_color()])
            self.set_colors(stage, origColor, True)

        if self.raycastMode == 0:
            hitInfo = get_physx_scene_query_interface().raycast_closest(origin, rayDir, maxDist)
            if hitInfo["hit"]:
                hitPos = hitInfo["position"]
                hitNormal = hitInfo["normal"]
                impactPos = Gf.Vec3f(hitPos[0], hitPos[1], hitPos[2])
                impactNormalEnd = Gf.Vec3f(hitPos[0] + hitNormal[0], hitPos[1] + hitNormal[1], hitPos[2] + hitNormal[2])
                self._debugDraw.draw_line(origin, COLOR_YELLOW, impactPos, COLOR_YELLOW)
                self._debugDraw.draw_line(impactPos, 0xff00ffff, impactNormalEnd, 0xff00ffff)
                draw_frame(self._debugDraw, impactPos)
            else:
                self._debugDraw.draw_line(origin, COLOR_YELLOW, endPoint, COLOR_YELLOW)

        elif self.raycastMode == 1:
            with Sdf.ChangeBlock():
                self._debugDraw.draw_line(origin, COLOR_YELLOW, endPoint, COLOR_YELLOW)

                #origColor = Vt.Vec3fArray([demo.get_primary_color()])
                #self.set_colors(stage, origColor, True)

                get_physx_scene_query_interface().raycast_all(origin, rayDir, maxDist, self.report_all_hits)

                hitColor = Vt.Vec3fArray([demo.get_hit_color()])
                self.set_colors(stage, hitColor, False)

        elif self.raycastMode == 2:
            anyHit = get_physx_scene_query_interface().raycast_any(origin, rayDir, maxDist)
            if anyHit:
                self._debugDraw.draw_line(origin, COLOR_RED, endPoint, COLOR_RED)
            else:
                self._debugDraw.draw_line(origin, COLOR_GREEN, endPoint, COLOR_GREEN)

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == carb.input.KeyboardInput.KEY_1:
                self.raycastMode = 0
            elif event.input == carb.input.KeyboardInput.KEY_2:
                self.raycastMode = 1
            elif event.input == carb.input.KeyboardInput.KEY_3:
                self.raycastMode = 2
        return True
