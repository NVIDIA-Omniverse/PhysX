# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import sys
from functools import partial
import carb.settings
import omni.kit.app
import omni.usd
import omni.ui as ui
import omni.kit.viewport.utility as vp_utils

PROGRESS_BAR_ENABLED = "/physics/progressBarEnabled"
PROGRESS_BAR_LABEL = "/physics/progressBarLabel"
PROGRESS_BAR_VALUE = "/physics/progressBarValue"


class PhysXProgressView:
    def __init__(self):
        self._viewport_overlay_frame = None
        self._viewport_window = None
        self._settings = carb.settings.get_settings()
        self._settings.set_default_bool(PROGRESS_BAR_ENABLED, False)
        self._settings.set_default_string(PROGRESS_BAR_LABEL, "")
        self._settings.set_default_float(PROGRESS_BAR_VALUE, 0.0)
        self._progress_bar_enabled = self._settings.get_as_bool(PROGRESS_BAR_ENABLED)
        self._progress_bar_label = self._settings.get_as_string(PROGRESS_BAR_LABEL)
        self._progress_bar_value = self._settings.get_as_float(PROGRESS_BAR_VALUE)
        self._progres_bar_percent = self._progress_bar_value * 100
        self._settings_change_sub_enabled = omni.kit.app.SettingChangeSubscription(
            PROGRESS_BAR_ENABLED, self._on_progress_settings_changed
        )
        self._settings_change_sub_label = omni.kit.app.SettingChangeSubscription(
            PROGRESS_BAR_LABEL, self._on_progress_settings_changed
        )
        self._settings_change_sub_value = omni.kit.app.SettingChangeSubscription(
            PROGRESS_BAR_VALUE, self._on_progress_settings_changed
        )

    def on_shutdown(self):
        self._settings_change_sub_enabled = None
        self._settings_change_sub_label = None
        self._settings_change_sub_value = None
        self._cleanup_overlay()

    def _on_progress_settings_changed(self, item, event_type):
        """ Event handler for progress settings changed"""
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._progress_bar_enabled = self._settings.get_as_bool(PROGRESS_BAR_ENABLED)
            if self._progress_bar_enabled:
                self._progress_bar_label = self._settings.get_as_string(PROGRESS_BAR_LABEL)
                self._progress_bar_value = self._settings.get_as_float(PROGRESS_BAR_VALUE)
                self._progres_bar_percent = self._progress_bar_value * 100
                if self._viewport_overlay_frame is None:
                    self._viewport_window = vp_utils.get_active_viewport_window()
                    if self._viewport_window:
                        self._viewport_overlay_frame = self._viewport_window.get_frame("omni.physx.ui.progress_bar")
                        self._viewport_overlay_frame.visible = True
                if self._viewport_overlay_frame:
                    self._build_ui()
            else:
                self._cleanup_overlay()

    def _cleanup_overlay(self):
        if self._viewport_overlay_frame:
            self._viewport_overlay_frame.visible = False
            self._viewport_overlay_frame = None
        self._viewport_window = None

    def _build_progress_bar(self):
        with ui.HStack(height=10, width=0):
            with ui.ZStack():
                ui.Rectangle(style={"background_color": 0xff000000}, width=ui.Fraction(1))
                with ui.HStack():
                    ui.Label(self._progress_bar_label)
                    ui.Spacer(width=10)
            with ui.ZStack():
                ui.Rectangle(style={"background_color": 0xff555555}, width=150)
                self._progress_rect = ui.Rectangle(style={"background_color": 0xffffff00}, width=ui.Percent(self._progres_bar_percent))
            ui.Spacer(width=5)

    def _build_ui(self):
        with self._viewport_overlay_frame:
            with ui.VStack():
                ui.Spacer()
                with ui.HStack(height=0):
                    ui.Spacer()
                    self._build_progress_bar()
                ui.Spacer(height=5)
