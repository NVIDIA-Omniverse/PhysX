# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni import ui
import carb
import carb.settings
from omni.kit.window.preferences.scripts.preferences_window import PreferenceBuilder
from omni.kit.widget.settings.settings_widget import SettingType, create_setting_widget
from omni.physxfabric import get_physx_fabric_interface
import omni.physxfabric.bindings._physxFabric as pxfabric

BUTTON_HEIGHT = 30



class PhysicsFcSettings(PreferenceBuilder):
    def __init__(self):
        super().__init__("Physics Fabric")
        self._widgets = []
        self._line_height = 20

    def on_shutdown(self):
        self._widgets = []

    def _add_setting_checkbox_and_label(self, name, path, **kwargs):
        with ui.HStack(height=self._line_height):
            self._widgets.append(create_setting_widget(path, SettingType.BOOL), **kwargs)
            # overpowering ui.Line from create_setting_widget to stay at ~zero width
            ui.Label(name, width=ui.Fraction(100))

    def _add_save_to_usd(self):
        button = ui.Button("Save current fabric state to USD", height=BUTTON_HEIGHT, width=300)

        def on_click():
            carb.log_warn("Saving data to USD.")
            get_physx_fabric_interface().save_to_usd()
            carb.log_warn("Physics fabric data saved to USD.")

        button.set_clicked_fn(on_click)
        
    def _build_fc_settings(self):
        self._add_setting_checkbox_and_label("Fabric enabled", pxfabric.SETTING_FABRIC_ENABLED)
        self._add_setting_checkbox_and_label("Use GPU interop", pxfabric.SETTING_FABRIC_USE_GPU_INTEROP)
        ui.Spacer(height=3)
        self._add_save_to_usd()


    def _build_update_settings(self):
        self._add_setting_checkbox_and_label("Update transformations", pxfabric.SETTING_FABRIC_UPDATE_TRANSFORMATIONS)
        self._add_setting_checkbox_and_label("Update velocities", pxfabric.SETTING_FABRIC_UPDATE_VELOCITIES)
        self._add_setting_checkbox_and_label("Update joint states", pxfabric.SETTING_FABRIC_UPDATE_JOINT_STATES)
        self._add_setting_checkbox_and_label("Update points", pxfabric.SETTING_FABRIC_UPDATE_POINTS)
        self._add_setting_checkbox_and_label("Update residuals", pxfabric.SETTING_FABRIC_UPDATE_RESIDUALS)

    def _build_section(self, name, build_func):
        with ui.CollapsableFrame(name, height=0):
            with ui.HStack():
                ui.Spacer(width=20)
                with ui.VStack():
                    build_func()

    def build(self):
        with ui.VStack(height=0):
            self._build_section("General", self._build_fc_settings)
            self._build_section("Update toggles", self._build_update_settings)
