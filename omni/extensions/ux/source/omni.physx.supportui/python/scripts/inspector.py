# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui
import omni.kit.app
import carb.settings
from omni.physxsupportui.bindings._physxSupportUi import PhysxInspectorOverlay
from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorModel
from omni.kit.viewport.utility import get_active_viewport_window
from .inspector_window import PhysXInspectorWindow
from .inspector_simulation import InspectorSimulation
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui


class InspectorViewportOverlays:
    def __init__(self):
        self._viewport = None
        self._overlay_scene_view = None
        self._overlay_inspector = None
        self._frame = None

    def attach_overlay(self):
        self._viewport = get_active_viewport_window()
        frame = self._viewport.get_frame("omni.physx.supportui.root_frame")
        with frame:
            self._overlay_scene_view = omni.ui_scene.scene.SceneView()
            self._viewport.viewport_api.add_scene_view(self._overlay_scene_view)
            with self._overlay_scene_view.scene:
                self._overlay_inspector = PhysxInspectorOverlay()

    def detach_overlay(self):
        frame = self._viewport.get_frame("omni.physx.supportui.root_frame")
        # clear is necessary to prevent calls in the C++ overlay after DLL is unloaded (that will crash)
        if frame:
            frame.clear()
        self._viewport.viewport_api.remove_scene_view(self._overlay_scene_view)
        self._viewport = None
        self._overlay_scene_view = None
        self._overlay_inspector = None


class PhysXInspector:
    def __init__(self, icon_folder: str):
        self._icon_folder = icon_folder
        self._inspector_overlays = None
        self._inspector_simulation = None
        self._inspector_windows = []
        settings = carb.settings.get_settings()

        if settings.get(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED):
            self._enable_inspector()
        else:
            self._disable_inspector()
        self._settings_inspector_enabled_sub = omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED,
            self._physics_inspector_enabled_setting_changed,
        )

    def _physics_inspector_enabled_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            settings = carb.settings.get_settings()
            if settings.get(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED):
                self._enable_inspector()
            else:
                self._disable_inspector()

    def _enable_inspector(self):
        # stable id is needed to allow changing window name without breaking its docking state
        self._stable_window_id = 0
        self._inspector_overlays = InspectorViewportOverlays()
        self._inspector_overlays.attach_overlay()
        self._inspector_simulation = InspectorSimulation()
        self.add_inspector_window()

    def _disable_inspector(self):
        # Simulation
        if self._inspector_simulation:
            self._inspector_simulation.clean()
        self._inspector_simulation = None

        # Windows
        for window in self._inspector_windows:
            window.clean()
            window.destroy()
        self._inspector_windows = []

        # Overlays
        if self._inspector_overlays:
            self._inspector_overlays.detach_overlay()
        self._inspector_overlays = None

    def destroy(self):
        self._settings_inspector_enabled_sub = None
        self._disable_inspector()

    def add_inspector_window(self):
        model = PhysXInspectorModel([])
        self._stable_window_id = self._stable_window_id + 1
        new_window = PhysXInspectorWindow(
            title_prefix="Physics Inspector: ",
            title_stable_id=f"PhysicsInspector{self._stable_window_id}",
            model=model,
            inspector=self,
        )
        self._inspector_windows.append(new_window)
