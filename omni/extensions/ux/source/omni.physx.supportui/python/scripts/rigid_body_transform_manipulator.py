# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb.profiler
import carb.settings
from carb.eventdispatcher import get_eventdispatcher
from typing import List, Union
from pxr import Sdf, Usd
import omni.timeline
import omni.usd
from .rigid_body_transform_model import RigidBodyTransformModel
from omni.kit.manipulator.prim.core.prim_transform_manipulator import PrimTransformManipulator
from omni.kit.manipulator.transform.settings_listener import OpSettingsListener
from omni.kit.manipulator.transform.settings_constants import c
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
from .utils import refresh_manipulator_selector


class RigidBodyTransformManipulator(PrimTransformManipulator):
    def __init__(self, usd_context_name: str = "", viewport_api=None):
        super().__init__(usd_context_name, viewport_api, "omni.physxsupportui", RigidBodyTransformModel(usd_context_name))

        self._simulation_mode = omni.timeline.get_timeline_interface().is_playing()
        self._rigid_body_manipulator_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED)

        self._settings = carb.settings.get_settings()
        self._settings_rigid_body_manipulator_enabled_sub = omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED, self._settings_rigid_body_manipulator_enabled_setting_changed
        )

        usd_context = omni.usd.get_context()
        self._stage_event_sub = [
            get_eventdispatcher().observe_event(
                observer_name="omni.physx.supportui:RigidBodyTransformManipulator",
                event_name=usd_context.stage_event_name(event),
                on_event=func
            )
            for event, func in (
                (omni.usd.StageEventType.SIMULATION_START_PLAY, lambda _: self._set_simulation_mode(True)),
                (omni.usd.StageEventType.SIMULATION_STOP_PLAY, lambda _: self._set_simulation_mode(False)),
            )
        ]

        self._op_listener = OpSettingsListener()
        self._op_listener_sub = self._op_listener.subscribe_listener(self._on_op_listener_changed)

    def __del__(self):
        self.destroy()

    def destroy(self):
        self._settings_rigid_body_manipulator_enabled_sub = None
        self._stage_event_sub = None
        self._op_listener_sub = None
        if self._op_listener:
            self._op_listener.destroy()
            self._op_listener = None
        super().destroy()

    def is_on(self):
        return self._rigid_body_manipulator_enabled and self._simulation_mode

    def _settings_rigid_body_manipulator_enabled_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._rigid_body_manipulator_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED)

    def _set_simulation_mode(self, enabled: bool):
        self._simulation_mode = enabled
        refresh_manipulator_selector()

    def _on_op_listener_changed(self, type: OpSettingsListener.CallbackType, value: str):
        # enable / disable scale manipulator
        assert isinstance(self.model, RigidBodyTransformModel)
        if not self.is_on():
            return
        if type == OpSettingsListener.CallbackType.OP_CHANGED:
            self.enabled = not self._op_listener.selected_op == c.TRANSFORM_OP_SCALE
            refresh_manipulator_selector()

    def on_selection_changed(self, stage: Usd.Stage, selection: Union[List[Sdf.Path], None], *args, **kwargs) -> bool:  # override
        # only rigid bodies are supported by this manipulator, scale is unsupported.
        # A different manipulator will be selected by omni.kit.manipulator.selector if False is returned.

        assert isinstance(self.model, RigidBodyTransformModel)

        if not self.is_on():  # manipulator is disabled through user setting or simulation is not running
            return False

        if selection is None:
            self.model.on_selection_changed([])
            return False

        self.model.on_selection_changed(selection)

        # scale op unsupported - disable this manipulator. Manipulator Selector will pick other suitable manipulator.
        if self._op_listener.selected_op == c.TRANSFORM_OP_SCALE:
            return False

        for path in selection:
            prim = stage.GetPrimAtPath(path)
            if self.model.is_rigid_body_xform(prim):
                return True

        return False
