# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdGeom, Tf
from usdrt import Usd as UsdRt
import omni.kit.app
import omni.usd
import omni.timeline
import carb
import carb.events
from carb.eventdispatcher import get_eventdispatcher
from carb.settings import get_settings
import weakref
from .physicsViewportOverlayShared import *
from .physicsViewportOverlayInfobox import *
from .physicsViewportOverlayPrimInfo import *
from .physicsViewportOverlayTransformGizmo import PhysicsTransformGizmo, PhysicsTransformGizmoModel
from .physicsViewportOverlayJoints import *
from omni.ui_scene import scene as sc
from omni.kit.manipulator.prim.core.reference_prim_marker import ReferencePrimMarker


class PhysicsUIViewportOverlayManager:
    """
    Centralized manager for the individual sub-overlays (derived from PhysicsUIViewportOverlay)
    """
    def __init__(self, desc: dict):
        usd_context_name = desc.get("usd_context_name")
        self.usd_context = omni.usd.get_context(usd_context_name)
        self._viewport_api=desc.get("viewport_api")
        self.stage : Usd.Stage | None = None
        self.stage_usdrt : UsdRt.Stage | None = None
        self._usd_object_changed_listener = None
        self._usd_prim_paths_info_changed = set()
        self._usd_prim_paths_resynced = set()

        self._stage_event_sub = [
            get_eventdispatcher().observe_event(
                observer_name="omni.usdphysics.ui:PhysicsUIViewportOverlayManager",
                event_name=self.usd_context.stage_event_name(event),
                on_event=func
            )
            for event, func in (
                (omni.usd.StageEventType.OPENED, lambda _: self._on_stage_opened()),
                (omni.usd.StageEventType.CLOSING, lambda _: self._on_stage_closed()),
                (omni.usd.StageEventType.SIMULATION_START_PLAY, lambda _: self._on_stage_simulation_start_play()),
                (omni.usd.StageEventType.SIMULATION_STOP_PLAY, lambda _: self._on_stage_simulation_stop_play()),
            )
        ]

        self.simulation_active = False

        self._timeline_sub = None
        self._kit_update_sub = None

        self.xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        self.bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])

        self._current_time = Usd.TimeCode.Default()
        self._settings_subscriptions = []
        self._settings_subscriptions.append(get_settings().subscribe_to_node_change_events(
            '/persistent/physics/visualizationDisplayJoints', self._on_settings_changed
        ))

        self.gesture_manager = PhysicsUIGestureManager(self)

        viewport = desc.get("layer_provider")

        frame = viewport.get_frame("omni.usdphysics.ui.base_frame")
        with frame:
            self._clip_vstack = omni.ui.VStack(content_clipping=False)
            with self._clip_vstack:
                self.scene_view_world = sc.SceneView()
                self._viewport_api.add_scene_view(self.scene_view_world)
                with self.scene_view_world.scene:
                    self._overlay_transform_camera_aligned = sc.Transform(look_at=sc.Transform.LookAt.CAMERA)

        frame = viewport.get_frame("omni.usdphysics.ui.screen_frame")
        with frame:
            self.scene_view_screen = sc.SceneView()

            with self.scene_view_screen.scene:
                with ui_create_screen_scale_transform(True):
                    self._overlay_transform_screen = sc.Transform()

        self._registered_prim_property_types = []
        self._refresh_prim_property_types_registered()

        self._viewport_overlays = []
        self._viewport_overlays.append(PhysicsUIPrimInfoOverlay(self))
        self._viewport_overlays.append(PhysicsUIFloatingBoxViewportOverlay(self))

        self._transform_gizmo = PhysicsTransformGizmo(self,
            usd_context_name=usd_context_name, viewport_api=self._viewport_api
        )

        self._reference_prim_marker = ReferencePrimMarker(
            usd_context_name=usd_context_name, manipulator_model=weakref.proxy(self._transform_gizmo._model)
        )

        self._on_stage_opened()

    def _refresh_prim_property_types_registered(self):
        self._registered_prim_property_types.clear()

        if get_settings().get('/persistent/physics/visualizationDisplayJoints'):
            self._registered_prim_property_types.append(PhysicsUIPrimInfoOverlay.register_prim_property_type(JointEditManipulator))
            self._registered_prim_property_types.append(PhysicsUIPrimInfoOverlay.register_prim_property_type(RevoluteJointEditManipulator))
            self._registered_prim_property_types.append(PhysicsUIPrimInfoOverlay.register_prim_property_type(TranslationJointEditManipulator))
            self._registered_prim_property_types.append(PhysicsUIPrimInfoOverlay.register_prim_property_type(SphericalJointEditManipulator))
            self._registered_prim_property_types.append(PhysicsUIPrimInfoOverlay.register_prim_property_type(DistanceJointEditManipulator))
            self._registered_prim_property_types.append(PhysicsUIPrimInfoOverlay.register_prim_property_type(D6JointEditManipulator))


    def _on_settings_changed(self, item, event_type):
        self._refresh_prim_property_types_registered()

    def destroy(self):
        settings = get_settings()
        for setting_sub in self._settings_subscriptions:
            settings.unsubscribe_to_change_events(setting_sub)
        
        self._settings_subscriptions.clear()

        if self.stage:
            self._on_stage_closed()

        for registry in self._registered_prim_property_types:
            registry.unregister()
 
        self._registered_prim_property_types.clear()

        for viewport_overlay in self._viewport_overlays:
            viewport_overlay.destroy()

        self._viewport_overlays.clear()

        if self._viewport_api:
            if self.scene_view_world:
                self._viewport_api.remove_scene_view(self.scene_view_world)
            if self.scene_view_screen:
                self._viewport_api.remove_scene_view(self.scene_view_screen)

        self.scene_view_world = None
        self.scene_view_screen = None

        if self._transform_gizmo:
            self._transform_gizmo.destroy()
        self._transform_gizmo = None

        self._stage_event_sub = None

    def align_to_camera(self, coords):
        return self._overlay_transform_camera_aligned.transform_space(sc.Space.OBJECT, sc.Space.WORLD, coords)

    # Utility function since standard transform_space does not work for sc.Space.SCREEN
    def transform_world_to_screen(self, coords):
        # Use NDC as an intermediate step.
        coords = self.scene_view_world.scene.transform_space(sc.Space.WORLD, sc.Space.NDC, coords)
        return self._overlay_transform_screen.transform_space(sc.Space.NDC, sc.Space.OBJECT, coords)

    # Utility function since standard transform_space does not work for sc.Space.SCREEN
    def transform_screen_to_world(self, coords):
        # Use NDC as an intermediate step.
        coords = self._overlay_transform_screen.transform_space(sc.Space.OBJECT, sc.Space.NDC, coords)
        return self.scene_view_world.scene.transform_space(sc.Space.NDC, sc.Space.WORLD, coords)

    def refresh_clipping_state(self):
        if self._clip_vstack:
            if (self.gesture_manager.active_gesture is not None or self.gesture_manager.active_hover is not None):
                self._clip_vstack.content_clipping = True
            else:
                self._clip_vstack.content_clipping = False

    def _on_timeline_event(self, e: carb.events.IEvent):

        def set_current_time(timecode):
            self.xform_cache.SetTime(timecode)
            self.bbox_cache.SetTime(timecode)

        if (
            e.type == int(omni.timeline.TimelineEventType.CURRENT_TIME_TICKED)
            or e.type == int(omni.timeline.TimelineEventType.CURRENT_TIME_CHANGED)
        ):
            current_time = e.payload["currentTime"]
            if self.stage is not None:
                set_current_time(Usd.TimeCode(omni.usd.get_frame_time_code(current_time, self.stage.GetTimeCodesPerSecond())))
            else:
                set_current_time(Usd.TimeCode.Default())

        elif e.type == int(omni.timeline.TimelineEventType.STOP):
            set_current_time(Usd.TimeCode.Default())

    def _on_stage_opened(self):
        self.stage = self.usd_context.get_stage()
        if self.stage is None:
            return

        self.stage_usdrt = UsdRt.Stage.Attach(self.usd_context.get_stage_id())
        if self.stage_usdrt is None:
            carb.log_error("Failed to attach USDRT to stage.")

        self.simulation_active = omni.timeline.get_timeline_interface().is_playing()

        self._usd_prim_paths_info_changed.clear()
        self._usd_prim_paths_resynced.clear()

        self._kit_update_sub = get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=self._on_kit_update,
            observer_name="omni.usdphysics.ui:PhysicsUIViewportOverlayManager"
        )

        self._timeline_sub = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(
            self._on_timeline_event
        )

        self.refresh_usd_change_listener()

    def _on_stage_closed(self):
        self.simulation_active = False
        self.stage = None
        self.stage_usdrt = None
        self._kit_update_sub = None
        self._timeline_sub = None
        self.xform_cache.Clear()
        self.xform_cache.SetTime(Usd.TimeCode.Default())
        self.bbox_cache.Clear()
        self.bbox_cache.SetTime(Usd.TimeCode.Default())

    def _on_stage_simulation_start_play(self):
        self._usd_prim_paths_info_changed.clear()
        self._usd_prim_paths_resynced.clear()
        self.simulation_active = True
        self.refresh_usd_change_listener()

    def _on_stage_simulation_stop_play(self):
        self.simulation_active = False
        self.xform_cache.Clear()
        self.bbox_cache.Clear()
        self.refresh_usd_change_listener()

    def _on_usd_objects_changed(self, notice : Tf.Notice, stage):
        if stage == self.stage:
            for path in notice.GetChangedInfoOnlyPaths():
                self._usd_prim_paths_info_changed.add(path.GetPrimPath())

            for path in notice.GetResyncedPaths():
                self._usd_prim_paths_resynced.add(path.GetPrimPath())

    def _on_kit_update(self, event: carb.events.IEvent):
        self.xform_cache.Clear()
        self.bbox_cache.Clear()

        if not self.simulation_active:
            if len(self._usd_prim_paths_info_changed) > 0:
                for viewport_overlay in self._viewport_overlays:
                    if viewport_overlay is not None and viewport_overlay.subscribes_to_usd_prim_paths_info_changed():
                        viewport_overlay.on_usd_prim_paths_info_changed(self._usd_prim_paths_info_changed)

                self._usd_prim_paths_info_changed.clear()

            if len(self._usd_prim_paths_resynced) > 0:
                for viewport_overlay in self._viewport_overlays:
                    if viewport_overlay is not None and viewport_overlay.subscribes_to_usd_prim_paths_resynced():
                        viewport_overlay.on_usd_prim_paths_resynced(self._usd_prim_paths_resynced)

                self._usd_prim_paths_resynced.clear()

        for viewport_overlay in self._viewport_overlays:
            viewport_overlay.on_kit_update()

    def refresh_usd_change_listener(self):
        if not self.simulation_active and self.stage:
            # Check if any overlay subscribes to USD changes.
            for overlay in self._viewport_overlays:
                if (
                    overlay is not None
                    and (
                        overlay.subscribes_to_usd_prim_paths_info_changed()
                        or overlay.subscribes_to_usd_prim_paths_resynced()
                    )
                ):
                    if self._usd_object_changed_listener is None:
                        self._usd_object_changed_listener = Tf.Notice.Register(
                            Usd.Notice.ObjectsChanged,
                            self._on_usd_objects_changed,
                            self.stage
                        )
                    return

        if self._usd_object_changed_listener is not None:
            self._usd_object_changed_listener.Revoke()
            self._usd_object_changed_listener = None

    @property
    def visible(self):
        return True

    @visible.setter
    def visible(self, value):
        pass

    @property
    def categories(self):
        return ("usdphysics.ui")

    @property
    def name(self):
        return "USD Physics UI Viewport Overlay Manager"

    def get_transform_gizmo_model(self) -> PhysicsTransformGizmoModel:
        return self._transform_gizmo._model
