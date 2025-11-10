# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.usd
from omni.physx import get_physx_interface
import omni.kit.notification_manager as nm
from omni.ui_scene import scene as sc
from omni.ui import color as cl

UI_GRAB_LINE_THICKNESS = 2
UI_GRAB_POINT_GRABBED_COLOR = cl("#FF2010FF")
UI_GRAB_POINT_TARGET_COLOR = cl("#90E110FF") # cl("#76b900FF") is NVIDIA green. This is a brighter variant.
UI_GRAB_LINE_COLOR = cl("#90E110FF")
UI_GRAB_CIRCLE_SIZE = 10
UI_GRAB_POINT_GRABBED_CIRCLE_SIZE = UI_GRAB_CIRCLE_SIZE + UI_GRAB_LINE_THICKNESS
UI_PUSH_CIRCLE_COLOR = cl("#FF201000")
UI_PUSH_CIRCLE_THICKNESS = 2
UI_PUSH_CIRCLE_SIZE_MIN = 5
UI_PUSH_CIRCLE_SIZE_MAX = 45
UI_PUSH_CIRCLE_DURATION = 30
UI_PUSH_CIRCLE_INTERLEAVE = 15

# UI scene manipulator for interaction.
class PhysicsInteractionManipulator(sc.Manipulator):
    
    def __init__(self):
        super().__init__()
        self._grab_target_position = None
        self._grabbed_position = None
        self._points_pushed = []
        self._push_period = 0

    def on_build(self):
        if (self._grab_target_position is not None and self._grabbed_position is not None):
            # Draw a line from the point we've grabbed to the point we are pulling towards.
            sc.Line(self._grab_target_position, self._grabbed_position, color=UI_GRAB_POINT_TARGET_COLOR, thickness=UI_GRAB_LINE_THICKNESS)

            # Draw a circle at the point we are pulling towards.
            with sc.Transform(transform=sc.Matrix44.get_translation_matrix(*self._grab_target_position), look_at=sc.Transform.LookAt.CAMERA):
                with sc.Transform(scale_to=omni.ui_scene.scene.Space.SCREEN):
                    sc.Arc(UI_GRAB_CIRCLE_SIZE, color=UI_GRAB_POINT_TARGET_COLOR)

            # Draw a circle at the point we have grabbed.
            with sc.Transform(transform=sc.Matrix44.get_translation_matrix(*self._grabbed_position)):
                with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=omni.ui_scene.scene.Space.SCREEN):
                    sc.Arc(UI_GRAB_POINT_GRABBED_CIRCLE_SIZE, color=UI_GRAB_POINT_GRABBED_COLOR, thickness=2, wireframe=True)

        # Push visualization.
        if len(self._points_pushed) > 0:
            self.invalidate()
            if self._push_period > 0:
                self._push_period += 1
                if self._push_period > UI_PUSH_CIRCLE_INTERLEAVE:
                    self._push_period = 0

            for point_pushed in self._points_pushed:
                if point_pushed[1] < UI_PUSH_CIRCLE_DURATION:
                    with sc.Transform(transform=sc.Matrix44.get_translation_matrix(*point_pushed[0])):
                        with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=omni.ui_scene.scene.Space.SCREEN):
                            sc.Arc(UI_PUSH_CIRCLE_SIZE_MIN + 
                                    (UI_PUSH_CIRCLE_SIZE_MAX - UI_PUSH_CIRCLE_SIZE_MIN) * point_pushed[1] / UI_PUSH_CIRCLE_DURATION, 
                                    color=(UI_PUSH_CIRCLE_COLOR + (int(255.0 - 255.0 * point_pushed[1] / UI_PUSH_CIRCLE_DURATION)) * 256 * 256 * 256), 
                                    thickness=UI_PUSH_CIRCLE_THICKNESS, wireframe=True)
                    point_pushed[1] += 1
                else:
                    self._points_pushed.remove(point_pushed)                
        else:
            self._push_period = 0


    def update_grab(self, target_position, grabbed_position):
        self._grab_target_position = target_position
        self._grabbed_position = grabbed_position
        # Triggers rebuilding.
        self.invalidate()

    def add_push(self, push_position):
        if self._push_period == 0:
            self._points_pushed.append([push_position, 0])
            self._push_period = 1 # This begins the interleave counter.
        self.invalidate()

    def destroy(self):
        self._grab_target_position = None
        self._grabbed_position = None
        self._points_pushed = []
        self._push_period = 0
        self.clear()

    def __del__(self):
        self.destroy()
