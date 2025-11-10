# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui as ui
import omni.kit.app
import omni.usd
from omni.timeline import get_timeline_interface
import carb
from omni.kit.viewport.utility import get_active_viewport_window
from omni.physxui.bindings._physxUI import PhysXUIOmniUISceneOverlay
from omni.physx import get_physx_interface
from .physicsViewportInteraction import PhysicsInteractionManipulator
from .physicsViewportMassEdit import *
from .physicsViewportInfoBox import *
from omni.ui_scene import scene as sc
from omni.physx.bindings._physx import SimulationEvent, PhysicsInteractionEvent

from enum import Enum

class PhysxUIMouseInteraction(Enum):
    DEFAULT = 0
    DISABLED = 1
    ENABLED = 2

class GestureAlwaysPass(sc.GestureManager):
    def can_be_prevented(self, gesture):
        return False

    def should_prevent(self, gesture, preventer):
        return False

class PhysxUIViewportOverlays:
    def __init__(self, viewport2_destroyed_poll_checking):
        self._viewport = None
        self._clip_vstack = None
        self._overlay_adapter = None
        self._overlay_scene_view = None

        # Physics Manipulator specifics
        self._input = None
        self._keyboard = None
        self._keyboard_sub = None
        self._physics_manipulator = None
        self._viewport_manipulator_strong_ref = None
        self._physics_manipulator_scene_view = None
        self._simulation_event_sub = None

        # viewport 2 update specifics
        self._viewport2_weak_ref = None
        self._viewport2_check_expired_ref_sub = None
        self._viewport2_destroyed_poll_checking = viewport2_destroyed_poll_checking

        self._mouse_interaction_state = PhysxUIMouseInteraction.DEFAULT
        self._mouse_interaction_simulation_dragging = False
        self._overlay_camera_scene_view = None
        self._mass_viewport_overlay = None

    def on_startup(self):
        self._attach_scene_view()
        self._attach_keyboard_events()
        events = get_physx_interface().get_simulation_event_stream_v2()
        self._simulation_event_sub = events.create_subscription_to_pop(self._on_simulation_event)

    def _attach_keyboard_events(self):
        app_window = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = app_window.get_keyboard()
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_key)

    def _detach_keyboard_events(self):
        if self._keyboard_sub:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._keyboard_sub)
            self._keyboard_sub = None
        self._keyboard = None
        self._input = None
        self._physics_manipulator = None

    def on_shutdown(self):
        self._detach_scene_view()
        self._detach_keyboard_events()
        self._simulation_event_sub = None
        self._stage_event_sub = None
        self._viewport2_check_expired_ref_sub = None

    def _on_key(self, e) -> bool:
        self.refresh_clipping_state()
        return True

    def on_mouse_shift_drag_start(self, sender):
        if get_active_gesture() or get_active_hover():
            return

        if not get_timeline_interface().is_playing():
            return

        if self._mouse_interaction_state == PhysxUIMouseInteraction.DEFAULT:
            if not (self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.LEFT_SHIFT) or self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.RIGHT_SHIFT)):
                return
        elif self._mouse_interaction_state == PhysxUIMouseInteraction.DISABLED:
            return

        self._mouse_interaction_simulation_dragging = True

        ray_origin, ray_direction = self._overlay_scene_view.get_ray_from_ndc(sc.Vector2(*sender.gesture_payload.mouse))
        get_physx_interface().update_interaction(ray_origin, ray_direction, PhysicsInteractionEvent.MOUSE_DRAG_BEGAN)

    def on_mouse_shift_dragged(self, sender):
        if not self._mouse_interaction_simulation_dragging:
            return

        ray_origin, ray_direction = self._overlay_scene_view.get_ray_from_ndc(sc.Vector2(*sender.gesture_payload.mouse))
        get_physx_interface().update_interaction(ray_origin, ray_direction, PhysicsInteractionEvent.MOUSE_DRAG_CHANGED)

    def on_mouse_shift_drag_end(self, sender):
        # Always reevaulate the clipping state because we may have allowed the gesture to continue even after shift is released.
        self.refresh_clipping_state()

        if not self._mouse_interaction_simulation_dragging:
            return

        ray_origin, ray_direction = self._overlay_scene_view.get_ray_from_ndc(sc.Vector2(*sender.gesture_payload.mouse))
        get_physx_interface().update_interaction(ray_origin, ray_direction, PhysicsInteractionEvent.MOUSE_DRAG_ENDED)
        self._detach_physics_manipulator()
        self._mouse_interaction_simulation_dragging = False

    def on_mouse_double_click(self, sender):
        if get_active_gesture() or get_active_hover():
            return

        if self._mouse_interaction_state == PhysxUIMouseInteraction.DEFAULT:
            if not (self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.LEFT_SHIFT) or self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.RIGHT_SHIFT)):
                return
        elif self._mouse_interaction_state == PhysxUIMouseInteraction.DISABLED:
            return

        ray_origin, ray_direction = self._overlay_scene_view.get_ray_from_ndc(sc.Vector2(*sender.gesture_payload.mouse))
        get_physx_interface().update_interaction(ray_origin, ray_direction, PhysicsInteractionEvent.MOUSE_LEFT_DOUBLE_CLICK)

    def is_picking_enabled(self):
        if self._clip_vstack:
            return not self._clip_vstack.content_clipping
        return True

    def enable_picking(self, enable):
        if self._clip_vstack:
            if enable:
                self._clip_vstack.content_clipping = False
            else:
                self._clip_vstack.content_clipping = True

    def create_text_info_box(self, text = [], text_color=INFO_BOX_TEXT_COLOR_DEFAULT, text_font_size = INFO_BOX_TEXT_FONT_SIZE_DEFAULT, text_alignment = ui.Alignment.CENTER, background_color = INFO_BOX_BACKGROUND_COLOR_DEFAULT, screen_coords = [0, 0], height = -1, width = -1):
        if not self._overlay_camera_scene_view.scene:
            return None
        with self._overlay_camera_scene_view.scene:
            info_box = PhysicsInfoBoxTextManipulator(text, text_color, text_font_size, text_alignment, background_color, screen_coords, height, width)
        return info_box

    def refresh_clipping_state(self):
        if self._clip_vstack:
            if (get_active_gesture() or get_active_hover() or
                (get_timeline_interface().is_playing() and
                    (self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.LEFT_SHIFT)
                    or self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.RIGHT_SHIFT)))
                ):
                self.enable_picking(False)
            else:
                self.enable_picking(True)

    def _attach_scene_view(self):
        viewport = get_active_viewport_window()
        if viewport is None:
            return

        frame = viewport.get_frame("omni.physx.ui.root_frame")
        with frame:
            self._clip_vstack = omni.ui.VStack(content_clipping=False)
            with self._clip_vstack:
                self._overlay_scene_view = sc.SceneView()
                viewport.viewport_api.add_scene_view(self._overlay_scene_view)
                with self._overlay_scene_view.scene:
                    self._overlay_adapter = PhysXUIOmniUISceneOverlay()
                    self._overlay_adapter.set_enable_picking_fn(self.enable_picking)
                    # If not using VP1, we have to make sure to pass the interaction events ourselves.
                    sc.Screen(gesture=[sc.DoubleClickGesture(self.on_mouse_double_click), sc.DragGesture(manager=GestureAlwaysPass(), on_began_fn=self.on_mouse_shift_drag_start, on_changed_fn=self.on_mouse_shift_dragged, on_ended_fn=self.on_mouse_shift_drag_end)])

        frame = viewport.get_frame("omni.physx.ui.info_box_frame")
        with frame:
            self._overlay_camera_scene_view = sc.SceneView()

        self._mass_viewport_overlay = PhysicsMassViewportOverlay(self)
        # under VP1 (viewport legacy) to properly use omni.ui.scene one should keep strong ref to the viewport returned
        # from get_active_viewport_window(), in order for frames where SceneViews are added not to be destroyed.
        # This however, prevents from properly destroying the viewport window when user closes it.
        # Additionally after the viewport is destroyed, the PhysicsManipulator overlays
        # will not be shown built anymore as they've not been recreated on the newly created viewport.
        # The joint overlays and joint manipulation gizmos will still be working even after viewport
        # destruction and recreation because they are not being drawn through omni.ui.scene.
        # That's the reason why have a dedicated _attach_physics_manipulator that is recreating PhysicsManipulator
        # at each cycle of grab and release
        #
        # under VP2 (viewport_next) get_active_viewport_window() returns a weakref.proxy, so it would be invalidated on window
        # destruction. Any access to it would lead to a ReferenceException and we should be re-creating
        # SceneViews. Initial solution was to track this expiration in an update event but to avoid
        # the overhead kit sdk opted to hide viewport window instead of destroying it.
        # If the "destruction of viewport" will ever come back, we should hook into whatever event it will
        # be made available to recreate frames and SceneViews.
        #
        # see OM-57695 for more sources to all relevant discussions.

    def _detach_scene_view(self):
        if self._overlay_scene_view:
            viewport = get_active_viewport_window()
            if viewport:
                viewport.viewport_api.remove_scene_view(self._overlay_scene_view)
        self._clip_vstack = None
        self._overlay_adapter = None
        # Releasing this object here brings crashes in tests with viewport_next
        # self._overlay_scene_view = None
        self._overlay_camera_scene_view = None
        if self._mass_viewport_overlay:
            self._mass_viewport_overlay.destroy()
            self._mass_viewport_overlay = None

    def _attach_physics_manipulator(self):
        viewport = get_active_viewport_window()
        if viewport is None:
            return

        # We MUST keep reference here for VP1 or the SceneView will not be drawn
        # See long comment inside _attach_scene_view
        self._viewport_manipulator_strong_ref = viewport
        frame = viewport.get_frame("omni.physx.ui.physics_manipulator_frame")
        frame.visible = True
        with frame:
            self._physics_manipulator_scene_view = sc.SceneView()
            viewport.viewport_api.add_scene_view(self._physics_manipulator_scene_view)
            with self._physics_manipulator_scene_view.scene:
                self._physics_manipulator = PhysicsInteractionManipulator()

    def _detach_physics_manipulator(self):
        if self._viewport_manipulator_strong_ref:
            frame = self._viewport_manipulator_strong_ref.get_frame("omni.physx.ui.physics_manipulator_frame")
            if frame:
                frame.visible = False
        self._viewport_manipulator_strong_ref = None
        self._physics_manipulator = None
        self._physics_manipulator_scene_view = None
        if self._physics_manipulator:
            self._physics_manipulator.destroy()

    def _on_simulation_event(self, event):
        if event.type == SimulationEvent.POINT_GRABBED.value:
            target = event.payload["grab_target_position"]
            source = event.payload["grabbed_position"]
            if not self._physics_manipulator:
                self._attach_physics_manipulator()
            self._physics_manipulator.update_grab(target, source)
        elif (event.type == SimulationEvent.POINT_RELEASED.value
              or event.type == SimulationEvent.PAUSED.value
              or event.type == SimulationEvent.STOPPED.value
              or event.type == SimulationEvent.DETACHED_FROM_STAGE.value
              ):
            self._detach_physics_manipulator()
        elif event.type == SimulationEvent.POINT_PUSHED.value:
            position = event.payload["pushed_position"]
            if not self._physics_manipulator:
                self._attach_physics_manipulator()
            self._physics_manipulator.add_push(position)

    def get_overlay_scene_view(self):
        return self._overlay_scene_view
