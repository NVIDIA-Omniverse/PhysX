# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from functools import partial
import omni.ui as ui
import carb.settings
from omni.kit.widget.settings import SettingsWidgetBuilder
from omni.kit.widget.settings.settings_widget import create_setting_widget_combo
from omni.physx.scripts.pythonUtils import ScopeGuard
from .helpers import Helpers, PopupMenu
import omni.physx.bindings._physx as pb
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui


class StaticCollSettingsMenu(PopupMenu):
    def _build_menu(self):
        Helpers.create_static_coll_menu()


class DynamicCollSettingsMenu(PopupMenu):
    def _build_menu(self):
        Helpers.create_dynamic_coll_menu()


class SupportUiSettings(ui.Window):
    DEFAULT_PREFIX = "/defaults"

    def __init__(self):
        super().__init__(
            "SupportUI Settings",
            visible=False,
            auto_resize=True,
            flags=ui.WINDOW_FLAGS_POPUP | ui.WINDOW_FLAGS_NO_SAVED_SETTINGS | ui.WINDOW_FLAGS_NO_COLLAPSE
        )
        self._write_guard = ScopeGuard()
        self._reset_buttons = {}
        self._compact_menu = True

    def __del__(self):
        self._reset_buttons = {}

    def show(self, x, y):
        self.build()
        self.position_x, self.position_y = x, y
        self.visible = True
        self.focus()

    def hide(self):
        self.visible = False

    def get_value(self, path):
        return carb.settings.get_settings().get(path)

    def get_default_value(self, path):
        return carb.settings.get_settings().get(self.DEFAULT_PREFIX + path)

    def is_default_value(self, path, value):
        default_value = self.get_default_value(path)
        if default_value is None:
            return True  # report no change when there is no default value
        return value == default_value

    def build(self):
        from omni.kit.widget.settings import get_style
        self.frame.set_style(get_style())

        def build_section(name, build_func):
            with ui.CollapsableFrame(name, height=0):
                with ui.HStack():
                    ui.Spacer(width=20)
                    with ui.VStack():
                        build_func()

        with self.frame:
            with ui.VStack(height=0, spacing=1):
                build_section("Colliders", self._build_colliders_section)
                if not self._compact_menu:
                    ui.Spacer(height=0)  # combo box in the previous section messes up height so no spacer here needed
                else:
                    ui.Spacer(height=4)
                build_section("Rigid Body", self._build_rigid_body_section)
                ui.Spacer(height=4)
                if not self._compact_menu:
                    build_section("Mass Properties", self._build_mass_properties_section)
                    ui.Spacer(height=4)
                build_section("Info", self._build_info_section)

    def _build_colliders_section(self):
        self._add_bool_setting("Async Cooking at Stage Load / Prim Add", pxsupportui.SETTINGS_ASYNC_COOKING_AT_STAGE_LOAD)
        self._add_bool_setting("Avoid Changing Existing Colliders (when possible)", pxsupportui.SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS)
        if not self._compact_menu:
            self._add_bool_setting("Automatic Creation of Colliders", pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED)
            self._add_combo_setting("Colliders: Visualization", pb.SETTING_DISPLAY_COLLIDERS, ["None", "Show on Selected", "Show on All"])

    def _build_rigid_body_section(self):
        self._add_physics_inspector_button()
        if not self._compact_menu:
            self._add_bool_setting("Rigid Body Selection Mode", pxsupportui.SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED)

        self._add_bool_setting("Rigid Body Manipulator (in Simulation Mode)", pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED)
        self._add_bool_setting("    Allow Prim to Parent Search for Rigid Body", pxsupportui.SETTINGS_MANIP_MODE_ALLOW_RIGID_BODY_TRAVERSAL)
        self._add_bool_setting("    Allow Rotation While Translating", pxsupportui.SETTINGS_MANIP_MODE_ALLOW_ROT_WHILE_TRANSLATING)
        self._add_bool_setting("    Allow Translation on Other Axes While Translating", pxsupportui.SETTINGS_MANIP_MODE_ALLOW_TRAN_ON_OTHER_AXES_WHILE_TRANSLATING)
        self._add_bool_setting("    Allow Translation While Rotating", pxsupportui.SETTINGS_MANIP_MODE_ALLOW_TRAN_WHILE_ROTATING)
        self._add_bool_setting("    Allow Rotation on Other Axes While Rotating", pxsupportui.SETTINGS_MANIP_MODE_ALLOW_ROT_ON_OTHER_AXES_WHILE_ROTATING)

    def _build_mass_properties_section(self):
        if not self._compact_menu:
            self._add_combo_setting("Mass Properties: Visualization", pb.SETTING_DISPLAY_MASS_PROPERTIES, ["None", "Show on Selected", "Show on All"])

    def _add_physics_inspector_button(self):
        settings = carb.settings.get_settings()
        if settings.get_as_bool("/persistent/physics/placementModeEnabled") or settings.get_as_bool("/physics/placementModeEnabled"):
            physics_inspector_button = self._add_bool_setting("Physics Inspector (cannot enable when Zero-G is active)", pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED)
            physics_inspector_button.enabled = False
        else:
            physics_inspector_button = self._add_bool_setting("Physics Inspector", pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED)

    def _build_info_section(self):
        development_mode = carb.settings.get_settings().get(pb.SETTING_PHYSICS_DEVELOPMENT_MODE)
        if development_mode:
            self._add_bool_setting("Debug Logging", pxsupportui.SETTINGS_LOGGING_ENABLED)
        self._add_bool_setting("Show Floating Notifications", pxsupportui.SETTINGS_FLOATING_NOTIFICATIONS_ENABLED)

    def _update_reset_button(self, path, value):
        button, _ = self._reset_buttons.get(path)
        if button:
            button.visible = not self.is_default_value(path, value)

    def _reset_setting(self, path):
        default_value = self.get_default_value(path)
        _, model = self._reset_buttons.get(path)
        if model:
            model.set_value(default_value)

    def _setting_changed(self, path, value):
        bool_val = value.as_bool
        carb.settings.get_settings().set(path, bool_val)
        self._update_reset_button(path, bool_val)

    def _add_bool_setting(self, name, path, tooltip=""):
        with ui.HStack(skip_draw_when_clipped=True):
            SettingsWidgetBuilder._create_label(name, path, tooltip, additional_label_kwargs={"width": 310})
            value = self.get_value(path)
            model = ui.SimpleBoolModel()
            model.set_value(value)
            widget = SettingsWidgetBuilder.createBoolWidget(model)
            model.add_value_changed_fn(partial(self._setting_changed, path))
            if self.get_default_value(path) is None:
                self._reset_buttons[path] = None, model
            else:
                reset_btn = SettingsWidgetBuilder._build_reset_button(path)
                reset_btn.set_mouse_pressed_fn(lambda *_, path=path: self._reset_setting(path))
                reset_btn.set_tooltip("Click to set the default value")
                self._reset_buttons[path] = reset_btn, model
                self._update_reset_button(path, value)
        ui.Spacer(height=3)
        return widget

    def _add_combo_setting(self, name, path, values, tooltip=""):
        with ui.HStack(skip_draw_when_clipped=True):
            SettingsWidgetBuilder._create_label(name, path, tooltip, additional_label_kwargs={"width": 160})
            widget, model = create_setting_widget_combo(path, values)
            ui.Spacer(width=20)
        ui.Spacer(height=5)
