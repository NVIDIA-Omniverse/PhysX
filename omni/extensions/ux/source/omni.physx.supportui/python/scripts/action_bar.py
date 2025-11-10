# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import asyncio
import os
import carb
import carb.events
import carb.dictionary
import carb.settings
from carb.eventdispatcher import get_eventdispatcher
import omni.kit.commands
import omni.kit.viewport.utility
import omni.kit.notification_manager as nm
import omni.physx
import omni.physxui.scripts.commands
import omni.ui as ui
import omni.usd
from pxr import PhysicsSchemaTools, UsdGeom
from typing import Any, Dict, Callable
from omni.kit.usd_undo import UsdLayerUndo
from omni.physx.scripts.pythonUtils import ScopeGuard
from omni.physx.bindings._physx import SETTING_MASS_DISTRIBUTION_MANIPULATOR
from .utils import ui_wait, get_physics_physx_schemas_applied
from .helpers import Helpers
from .styles import Styles
from .extension_settings import SupportUiSettings, StaticCollSettingsMenu, DynamicCollSettingsMenu
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui


class CustomProgressValueModel(ui.AbstractValueModel):
    def __init__(self, value: float):
        super().__init__()
        self._value = value

    def set_value(self, value):
        try:
            value = float(value)
        except ValueError:
            value = None
        if value != self._value:
            # Tell the widget that the model is changed
            self._value = value
            self._value_changed()

    def get_value_as_float(self):
        return self._value

    def get_value_as_string(self):
        return f"{int(self.get_value_as_float() * 100)}%"


class CreateCollidersCommand(omni.kit.commands.Command):
    def __init__(self, stage, prim_paths):
        self._stage = stage
        self._prim_paths = prim_paths
        self._usd_undo = None
        self._usd_redo = None

    def do(self):
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        for prim_path in self._prim_paths:
            self._usd_undo.reserve(prim_path)
        if self._usd_redo is not None:
            self._usd_redo.undo()
        return True

    def undo(self):
        self._usd_redo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        for prim_path in self._prim_paths:
            self._usd_redo.reserve(prim_path)
        self._usd_undo.undo()


class ButtonDef:
    # button identification (keys to dict)
    BUTTON_RB_SEL = "rb_sel"
    BUTTON_CLEAR = "clear"
    BUTTON_STATIC = "static"
    BUTTON_DYNAMIC = "dynamic"
    BUTTON_AUTO_COLL = "auto_coll"
    BUTTON_MASS = "mass_distribution_manipulator"
    BUTTON_SETTINGS = "settings"

    @classmethod
    def define_buttons(cls, button_defs_dict: Dict[str, Any], icon_folder: str):
        def def_button(
            key: str,
            ui_text: str,
            icon_name: str,
            tooltip: str,
            width: int = Styles.BUTTON_SIZE_H,
            height: int = Styles.BUTTON_SIZE_V,
            style: str = Styles.BUTTON_STYLE
        ):
            """helps to define a toolbar button, adds it to button_defs_dict"""
            button_defs_dict[key] = [ui_text, os.path.join(icon_folder, icon_name), tooltip, width, height, style]

        def_button(cls.BUTTON_RB_SEL, "RB Selecton", "rigid_body_selection_mode.svg", "Rigid Body Selection Mode")
        def_button(cls.BUTTON_CLEAR, "Clear", "remove_physics_components.svg", "Remove Physics")
        def_button(cls.BUTTON_STATIC, "Static", "set_static.svg", "Set Static Collider")
        def_button(cls.BUTTON_DYNAMIC, "Dynamic", "set_dynamic.svg", "Set Dynamic Collider")
        def_button(cls.BUTTON_AUTO_COLL, "AutoColl", "collision_automatic_creation_static.svg", "Automatic Collider Generation")
        def_button(cls.BUTTON_MASS, "Mass", "mass_manipulator.svg", "Mass Distribution Manipulator")
        def_button(cls.BUTTON_SETTINGS, "Settings", "physics_settings.svg", "Settings")


class ActionBar:
    def __init__(self, icon_folder, support_ui_iface):
        self._physx_supportui_interface = support_ui_iface
        self._action_bar_visibility_changed_fn = None

        self._buttons_def = dict()
        ButtonDef.define_buttons(self._buttons_def, icon_folder)
        self._buttons = dict()  # this dictionary holds button instances. Key corresponds to the _buttons_def key

        self._settings_subs = []
        self._settings = carb.settings.get_settings()
        self._logging_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_LOGGING_ENABLED)
        self._selection = omni.usd.get_context().get_selection()

        self._window = None
        self._progress_bar = None
        self._progress_bar_model = CustomProgressValueModel(0)

        self._settings_window = SupportUiSettings()
        self._static_coll_settings_menu = StaticCollSettingsMenu()
        self._dynamic_coll_settings_menu = DynamicCollSettingsMenu()

        self._num_remaining_coll_tasks = 0
        self._num_total_coll_tasks = 0

        usd_context = omni.usd.get_context()
        self._stage_event_sub = get_eventdispatcher().observe_event(
            observer_name="omni.physx.supportui.ActionBar",
            event_name=usd_context.stage_event_name(omni.usd.StageEventType.SELECTION_CHANGED),
            on_event=lambda _: self._on_stage_selection_changed_event()
        )

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            SETTING_MASS_DISTRIBUTION_MANIPULATOR, self._mass_distribution_manipulator_enabled_setting_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_LOGGING_ENABLED, self._enable_logging_setting_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED, self._rb_selection_enabled_setting_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_TOOLBAR_BUTTONS_WITH_TEXT, self._toolbar_buttons_with_text_setting_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED, self._auto_coll_enabled_setting_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_ASYNC_COOKING_AT_STAGE_LOAD, self._async_cooking_at_stage_load_setting_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE, self._static_coll_simpl_type_setting_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE, self._dynamic_coll_simpl_type_setting_changed
        ))

        event_stream = self._physx_supportui_interface.get_event_stream()
        self._supportui_event_sub = event_stream.create_subscription_to_pop(self._on_supportui_event)

        self._scope_guard = ScopeGuard()

        self._create_toolbar()

    def clean(self):
        """Cleanup. ActionBar is being freed."""
        self._buttons_def = None
        self._buttons = None

        self._progress_bar = None
        self._progress_bar_static = None
        self._progress_bar_dynamic = None
        self._progress_bar_model = None
        self._supportui_event_sub = None
        self._stage_event_sub = None
        self._selection = None

        if self._settings_window:
            self._settings_window.hide()
            self._settings_window.destroy()
            self._settings_window = None

        if self._static_coll_settings_menu:
            self._static_coll_settings_menu.hide()
            self._static_coll_settings_menu = None
        if self._dynamic_coll_settings_menu:
            self._dynamic_coll_settings_menu.hide()
            self._dynamic_coll_settings_menu = None

        self._action_bar_visibility_changed_fn = None

        if self._window is not None:
            self._window.undock()
            self._window.visible = False
            self._window.destroy()
            self._window = None
            # update workspace (TODO is there a better way?)
            win = ui.Workspace.get_window(ActionBar.get_toolbar_name())
            if win:
                win.visible = False

        self._scope_guard = None
        self._settings_subs = []
        self._settings = None
        self._physx_supportui_interface = None

    @staticmethod
    def get_toolbar_name() -> str:
        return "Physics Authoring"

    @staticmethod
    def is_allowed_prim(prim):
        return prim.IsA(UsdGeom.Xform) or prim.IsA(UsdGeom.Gprim)

    def set_action_bar_visibility_changed_fn(self, action_bar_visiblity_changed_fn: Callable[[bool], None]):
        self._action_bar_visibility_changed_fn = action_bar_visiblity_changed_fn

    def _create_toolbar(self, visibility=True):
        show_buttons_with_text = self._settings.get_as_bool(pxsupportui.SETTINGS_TOOLBAR_BUTTONS_WITH_TEXT)

        def create_button(button, **kwargs):
            return ui.Button(
                button[0] if show_buttons_with_text is True else '',
                image_url=button[1],
                tooltip=button[2],
                image_width=button[3],
                image_height=button[4],
                width=button[3],
                height=button[4],
                style=button[5],
                **kwargs
            )

        def create_separator(axis, sep_style):
            if axis == ui.ToolBarAxis.X:
                ui.Line(height=Styles.BUTTON_SIZE_V + 4, width=2, alignment=ui.Alignment.H_CENTER, style=sep_style)
            else:
                ui.Line(width=Styles.BUTTON_SIZE_H + 4, height=2, alignment=ui.Alignment.V_CENTER, style=sep_style)

        def create_buttons(axis):
            stack_style = {"Button.Image:disabled": {"color": Styles.DISABLED_COLOR}}
            pb_style = {"margin": 1, "padding": 2, "secondary_color": Styles.SEPARATOR_COLOR}
            sep_style = {"color": Styles.DISABLED_COLOR}
            with self._window.frame:
                with ui.VStack():
                    bk = None  # tmp var holding button dict key
                    if axis == ui.ToolBarAxis.X:
                        stack = ui.HStack(style=stack_style)
                    else:
                        stack = ui.VStack(style=stack_style)
                    with stack:
                        # Rigid Body Selection Mode
                        bk = ButtonDef.BUTTON_RB_SEL
                        self._buttons[bk] = create_button(
                            self._buttons_def[bk],
                            clicked_fn=self.on_rb_sel_enabled_button_clicked
                        )
                        # separator
                        create_separator(axis, sep_style)
                        # clear marker
                        bk = ButtonDef.BUTTON_CLEAR
                        self._buttons[bk] = create_button(
                            self._buttons_def[bk],
                            clicked_fn=self.on_clear_button_clicked
                        )
                        # static marker
                        with ui.ZStack(width=Styles.BUTTON_SIZE_H, height=Styles.BUTTON_SIZE_V):
                            bk = ButtonDef.BUTTON_STATIC
                            self._buttons[bk] = create_button(
                                self._buttons_def[bk],
                                clicked_fn=lambda dynamic=False: self._create_colliders(dynamic),
                                mouse_pressed_fn=self._on_static_button_mouse_pressed,
                                mouse_released_fn=self._on_static_button_mouse_released
                            )
                            self._progress_bar_static = ui.ProgressBar(
                                self._progress_bar_model,
                                visible=False,
                                width=Styles.BUTTON_SIZE_H + 4,
                                height=Styles.BUTTON_SIZE_V + 4,
                                style=pb_style,
                                tooltip="Click me to cancel creation of static colliders")
                            self._progress_bar_static.set_mouse_pressed_fn(self.on_cancel_button_pressed)
                        # dynamic marker
                        with ui.ZStack(width=Styles.BUTTON_SIZE_H, height=Styles.BUTTON_SIZE_V):
                            bk = ButtonDef.BUTTON_DYNAMIC
                            self._buttons[bk] = create_button(
                                self._buttons_def[bk],
                                clicked_fn=lambda dynamic=True: self._create_colliders(dynamic),
                                mouse_pressed_fn=self._on_dynamic_button_mouse_pressed,
                                mouse_released_fn=self._on_dynamic_button_mouse_released
                            )
                            self._progress_bar_dynamic = ui.ProgressBar(
                                self._progress_bar_model,
                                visible=False,
                                width=Styles.BUTTON_SIZE_H + 4,
                                height=Styles.BUTTON_SIZE_V + 4,
                                style=pb_style,
                                tooltip="Click me to cancel creation of dynamic colliders")
                            self._progress_bar_dynamic.set_mouse_pressed_fn(self.on_cancel_button_pressed)
                        # separator
                        create_separator(axis, sep_style)
                        # auto coll
                        bk = ButtonDef.BUTTON_AUTO_COLL
                        self._buttons[bk] = create_button(
                            self._buttons_def[bk],
                            clicked_fn=self.on_auto_coll_enabled_button_clicked
                        )
                        # mass distribution
                        bk = ButtonDef.BUTTON_MASS
                        self._buttons[bk] = create_button(
                            self._buttons_def[bk],
                            clicked_fn=self.on_mass_distribution_manipulator_enabled_button_clicked
                        )
                        # settings
                        bk = ButtonDef.BUTTON_SETTINGS
                        self._buttons[bk] = create_button(
                            self._buttons_def[bk],
                            mouse_pressed_fn=self._on_settings_button_mouse_pressed
                        )
            self._update_action_bar_ui()

        async def dock(visible):
            await ui_wait()  # we have to wait here 10 frames otherwise it won't dock in Create (as it applies fixed layout)
            vp = omni.kit.viewport.utility.get_active_viewport_window()
            if vp and self._window:
                viewport_window = ui.Workspace.get_window(vp.title)
                if viewport_window:
                    tab_visible = viewport_window.dock_tab_bar_visible  # TEMP FIX for a bug that always shows Viewport tab
                    self._window.dock_in(viewport_window, ui.DockPosition.RIGHT)
                    viewport_window.dock_tab_bar_visible = tab_visible
            if self._window:
                self._window.visible = visible

        def on_visiblity_changed(visible):
            if self._action_bar_visibility_changed_fn:
                self._action_bar_visibility_changed_fn(visible)

        self._window = ui.ToolBar(
            ActionBar.get_toolbar_name(),
            axis_changed_fn=create_buttons,  # trigger toolbar creation on axis change
            visibility_changed_fn=on_visiblity_changed
        )

        create_buttons(ui.ToolBarAxis.X)

        if not self._window.docked:
            asyncio.ensure_future(dock(visibility))

    def is_visible(self):
        return self._window.visible if self._window else False

    def _on_supportui_event(self, event):
        if event.type == int(pxsupportui.SupportUiEventType.COLLIDER_CREATED):
            coll_type = event.payload['colliderType']
            simplification_type = event.payload['simplificationType']
            prim_path_encoded = event.payload['primPath']
            prim_path = PhysicsSchemaTools.decodeSdfPath(prim_path_encoded[0], prim_path_encoded[1])
            self._num_remaining_coll_tasks = event.payload['numRemainingCollTasks']
            self._num_total_coll_tasks = event.payload['numTotalCollTasks']
            if self._logging_enabled:
                coll_type_str = "static" if coll_type is False else "dynamic"
                simplification_type_str = Helpers._static_coll_approximations[simplification_type] if coll_type is False else Helpers._dynamic_coll_approximations[simplification_type]
                carb.log_info(f'COLLIDER_CREATED event [{self._num_remaining_coll_tasks+1}/{self._num_total_coll_tasks}] ({coll_type_str}: {simplification_type_str}) for "{str(prim_path)}".')
            if self._progress_bar is None:
                self._progress_bar = self._progress_bar_static if coll_type is False else self._progress_bar_dynamic
            self._update_action_bar_ui()
        elif event.type == int(pxsupportui.SupportUiEventType.RIGID_BODY_CREATED):
            if self._logging_enabled:
                prim_path_encoded = event.payload['primPath']
                prim_path = PhysicsSchemaTools.decodeSdfPath(prim_path_encoded[0], prim_path_encoded[1])
                carb.log_info(f'RIGID_BODY_CREATED event for "{str(prim_path)}".')
        elif event.type == int(pxsupportui.SupportUiEventType.RIGID_BODY_REMOVED):
            if self._logging_enabled:
                prim_path_encoded = event.payload['primPath']
                prim_path = PhysicsSchemaTools.decodeSdfPath(prim_path_encoded[0], prim_path_encoded[1])
                carb.log_info(f'RIGID_BODY_REMOVED event for "{str(prim_path)}".')

    def _on_stage_selection_changed_event(self):
        if self._window is not None:
            self._update_action_bar_ui()

    def _rb_selection_enabled_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._update_action_bar_ui()

    def _async_cooking_at_stage_load_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            pass  # nothing to do right now

    def _auto_coll_enabled_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._update_action_bar_ui()

    def _static_coll_simpl_type_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._update_action_bar_ui()

    def _dynamic_coll_simpl_type_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._update_action_bar_ui()

    def _mass_distribution_manipulator_enabled_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._update_action_bar_ui()

    def _enable_logging_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._logging_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_LOGGING_ENABLED)

    def _toolbar_buttons_with_text_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            if self._window is not None:
                self._window = None
                self._on_main_menu_click(None, True)  # recreate the whole toolbar

    def _allowed_prims_selected(self):
        selected_paths = self._selection.get_selected_prim_paths()
        stage = omni.usd.get_context().get_stage()
        for path in selected_paths:
            prim = stage.GetPrimAtPath(path)
            if self.is_allowed_prim(prim):
                return True
        return False

    def _update_action_bar_ui(self):
        if self._num_remaining_coll_tasks > 0:
            if self._progress_bar is not None:
                if not self._progress_bar.visible:
                    self._progress_bar.visible = True
                total = self._num_total_coll_tasks
                so_far = self._num_total_coll_tasks - self._num_remaining_coll_tasks
                val = so_far / total
                self._progress_bar.model.set_value(val)
        elif self._num_remaining_coll_tasks == 0:
            if self._progress_bar_static.visible:
                self._progress_bar_static.visible = False
            if self._progress_bar_dynamic.visible:
                self._progress_bar_dynamic.visible = False
            # update buttons + tooltips w/ current coll. approx. settings
            static_coll_approx = self._settings.get_as_int(pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE)
            static_coll_approx_str = Helpers._static_coll_approximations[static_coll_approx]
            self._buttons[ButtonDef.BUTTON_STATIC].tooltip = f'{self._buttons_def[ButtonDef.BUTTON_STATIC][2]} ({static_coll_approx_str})'
            dynamic_coll_approx = self._settings.get_as_int(pxsupportui.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE)
            dynamic_coll_approx_str = Helpers._dynamic_coll_approximations[dynamic_coll_approx]
            self._buttons[ButtonDef.BUTTON_DYNAMIC].tooltip = f'{self._buttons_def[ButtonDef.BUTTON_DYNAMIC][2]} ({dynamic_coll_approx_str})'
            self._buttons[ButtonDef.BUTTON_AUTO_COLL].checked = self._settings.get_as_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED)
            self._num_total_coll_tasks = 0
            self._progress_bar = None

        self._buttons[ButtonDef.BUTTON_MASS].checked = self._settings.get_as_bool(SETTING_MASS_DISTRIBUTION_MANIPULATOR)
        self._buttons[ButtonDef.BUTTON_RB_SEL].checked = self._settings.get_as_bool(pxsupportui.SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED)
        self._buttons[ButtonDef.BUTTON_RB_SEL].enabled = True  # always enabled even with no stage open
        sel = self._allowed_prims_selected() if self._num_remaining_coll_tasks == 0 else False
        processing = self._scope_guard.is_guarded() or self._physx_supportui_interface.get_num_of_colliders_to_create() > 0
        self._buttons[ButtonDef.BUTTON_CLEAR].enabled = not processing and len(self._selection.get_selected_prim_paths()) > 0
        self._buttons[ButtonDef.BUTTON_STATIC].enabled = not processing and sel
        self._buttons[ButtonDef.BUTTON_DYNAMIC].enabled = not processing and sel
        self._buttons[ButtonDef.BUTTON_AUTO_COLL].enabled = not processing
        self._buttons[ButtonDef.BUTTON_SETTINGS].enabled = not processing

    def on_clear_button_clicked(self):
        if self._scope_guard.is_guarded():
            return

        with self._scope_guard:
            omni.kit.commands.execute(
                "ClearPhysicsComponentsCommand",
                stage=omni.usd.get_context().get_stage(),
                prim_paths=self._selection.get_selected_prim_paths())
            if self._settings.get_as_bool(pxsupportui.SETTINGS_FLOATING_NOTIFICATIONS_ENABLED):
                nm.post_notification("Physics components were cleared.", duration=5)

        self._update_action_bar_ui()

    def _create_colliders(self, dynamic_coll):
        def get_disallowed_schema(prim):
            # In enforced mode (when user clicked 'set static/dynamic collider'), only allowed physics schemas are:
            # Note: the following are tested as substrings
            allowed_schemas = [
                "CollisionAPI",
                "RigidBodyAPI",
                "CookedDataAPI",
                "MassAPI",
                "TriggerAPI",
                "ForceAPI",
                "ArticulationAPI",
                "ArticulationRootAPI",
            ]

            # if none of the allowed schemas match as a substring applied schemas, return it
            schemas = get_physics_physx_schemas_applied(prim)
            for t in schemas:
                if all(t.typeName.find(api) == -1 for api in allowed_schemas):
                    return t
            return None

        if self._scope_guard.is_guarded():
            return

        with self._scope_guard:
            self._progress_bar = self._progress_bar_static if dynamic_coll is True else self._progress_bar_dynamic
            stage = omni.usd.get_context().get_stage()
            prim_paths = []

            for path in self._selection.get_selected_prim_paths():
                prim = stage.GetPrimAtPath(path)
                if self.is_allowed_prim(prim):
                    disallowed_schema = get_disallowed_schema(prim)
                    if disallowed_schema:
                        tn = disallowed_schema.typeName
                        msg = f'"{str(path)}" - skipping collider creation because it contains "{tn}" schema.'
                        carb.log_info(msg)
                        nm.post_notification(msg, status=nm.NotificationStatus.WARNING, duration=5)
                    else:
                        prim_paths.append(path)
                        ct = pxsupportui.SupportUiColliderType
                        coll_type = ct.DYNAMIC if dynamic_coll is True else ct.STATIC
                        self._physx_supportui_interface.create_colliders(PhysicsSchemaTools.sdfPathToInt(path), coll_type)

            if len(prim_paths) > 0:
                omni.kit.commands.execute("CreateCollidersCommand", stage=stage, prim_paths=prim_paths)

        self._update_action_bar_ui()

    def on_cancel_button_pressed(self, x, y, button, modifiers):
        self._physx_supportui_interface.clear_colliders_processing_queue()
        self._num_total_coll_tasks -= self._num_remaining_coll_tasks
        self._num_remaining_coll_tasks = 0
        self._update_action_bar_ui()

    def on_rb_sel_enabled_button_clicked(self):
        self._settings.set_bool(
            pxsupportui.SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED,
            not self._settings.get_as_bool(pxsupportui.SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED)
        )

    def on_auto_coll_enabled_button_clicked(self):
        self._settings.set_bool(
            pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED,
            not self._settings.get_as_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED)
        )

    def on_mass_distribution_manipulator_enabled_button_clicked(self):
        self._settings.set_bool(
            SETTING_MASS_DISTRIBUTION_MANIPULATOR,
            not self._settings.get_as_bool(SETTING_MASS_DISTRIBUTION_MANIPULATOR)
        )

    def _on_static_button_mouse_pressed(self, x, y, button, modifiers):
        if button == 0 or button == 1:
            self._static_coll_settings_menu.show_popup(x, y, button == 0)

    def _on_static_button_mouse_released(self, x, y, button, modifiers):
        if button == 0:
            self._static_coll_settings_menu.cancel_popup_task()

    def _on_dynamic_button_mouse_pressed(self, x, y, button, modifiers):
        if button == 0 or button == 1:
            self._dynamic_coll_settings_menu.show_popup(x, y, button == 0)

    def _on_dynamic_button_mouse_released(self, x, y, button, modifiers):
        if button == 0:
            self._dynamic_coll_settings_menu.cancel_popup_task()

    def _on_settings_button_mouse_pressed(self, x, y, button, modifiers):
        self._settings_window.show(x, y)
