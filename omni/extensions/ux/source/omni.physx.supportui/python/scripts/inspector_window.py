# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui as ui
import asyncio
import omni.kit.app
import carb
from omni.kit.window.popup_dialog import MessageDialog
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorModelState
from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorModel
from omni.physxsupportui import get_physx_supportui_private_interface
from .inspector_panel import InspectorPanel
from .inspector_selection import InspectorSelectionHandler
from .inspector_context_menu import InspectorContextMenuHandler
from .inspector_toolbar import InspectorToolbar

LABEL_WIDTH = 200
LINE_HEIGHT = 23


class PhysXInspectorWindow(ui.Window):
    def __init__(
        self,
        inspector,
        model: PhysXInspectorModel,
        title_stable_id: str,
        title_prefix: str,
    ):
        self._title_stable_id = title_stable_id
        self._title_prefix = title_prefix
        self._development_mode = carb.settings.get_settings().get(
            "physics/developmentMode"
        )
        self._supportui_private = get_physx_supportui_private_interface()

        self._inspector = inspector

        # Models
        self._model_inspector = model

        # Subscriptions
        self._sub_model_event = self._supportui_private.get_inspector_event_stream().create_subscription_to_pop(
            self._on_inspector_event, name="Inspector Window State"
        )
        self._sub_path_changed = self._model_inspector.get_inspected_prim_string_model().add_value_changed_fn(
            self._on_path_changed
        )

        # UI Handlers
        self._handler_selection = InspectorSelectionHandler(
            self._model_inspector, self._inspector
        )
        self._handler_context_menu = InspectorContextMenuHandler()

        # UI Elements
        self._inspector_panel = None
        super().__init__(
            self._generate_title(),
            visible=True,
            position_x=420,
            position_y=200,
            width=300,
            height=300,
            auto_resize=False,
            margin=0,
        )
        self.set_visibility_changed_fn(self._on_window_closed)

        async def dock_inspector():
            await omni.kit.app.get_app().next_update_async()
            num_windows = len(self._inspector._inspector_windows)
            if num_windows == 1:
                stage_window = ui.Workspace.get_window("Stage")
                if stage_window:
                    self.dock_in(stage_window, ui.DockPosition.BOTTOM)
            elif num_windows > 0:
                self.dock_in(
                    self._inspector._inspector_windows[0], ui.DockPosition.SAME
                )

        asyncio.ensure_future(dock_inspector())
        self._update_inspector_window()

    def _generate_title(self):
        selection = (
            self._model_inspector.get_inspected_prim_string_model().get_value_as_string()
        )
        title = f"{self._title_prefix}{selection}###{self._title_stable_id}"
        return title

    def _on_window_closed(self, _):
        if self._model_inspector is None:
            return

        def hide_confirmation_dialog(confirmation_message: MessageDialog):
            if confirmation_message:
                confirmation_message.hide()  # We can't destroy() it here because it crashes kit
                # These are needed to free the C++ model, being grabbed by the lambda
                confirmation_message._click_okay_handler = None
                confirmation_message._click_cancel_handler = None

        def disable_inspector_on_last_window(last_window: bool):
            if last_window:
                carb.settings.get_settings().set_bool(
                    pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, False
                )

        def discard_inspector(
            model, confirmation_message: MessageDialog, last_window: bool
        ):
            hide_confirmation_dialog(confirmation_message)
            supportui_private.reset_inspector_to_authoring_start()
            disable_inspector_on_last_window(last_window)

        def commit_inspector(
            model,
            confirmation_message: MessageDialog,
            last_window: bool,
            supportui_private,
        ):
            hide_confirmation_dialog(confirmation_message)
            supportui_private.commit_authoring_state()
            disable_inspector_on_last_window(last_window)

        supportui_private = self._supportui_private
        last_window = len(self._inspector._inspector_windows) <= 1
        # Grabbing also the C++ model to keep a reference to it until we commit or reset
        model = self._model_inspector
        is_simulation_dirty = (
            self._inspector._inspector_simulation.simulation_dirty.as_bool
        )
        if is_simulation_dirty:
            confirmation_message = MessageDialog(
                message=f"Do you want to commit changes done with the inspector?",
                title="Unsaved changes",
                ok_handler=lambda _: commit_inspector(
                    model, confirmation_message, last_window, supportui_private
                ),
                cancel_handler=lambda _: discard_inspector(
                    model, confirmation_message, last_window
                ),
            )
            confirmation_message.show()

        self._inspector._inspector_simulation.stop_authoring_simulation()
        self._inspector._inspector_windows.remove(self)
        self.clean()
        if not is_simulation_dirty:
            discard_inspector(model, None, last_window)

    def _show_selected_container(self, type : PhysXInspectorModelState):
        if type == PhysXInspectorModelState.DISABLED:
            self._ui_container_disabled.visible = True
            self._ui_container_authoring.visible = False
        else:
            self._ui_container_disabled.visible = False
            self._ui_container_authoring.visible = True
            self._handler_selection.refresh_selection()
        self._inspector_toolbar.on_inspector_event(type)

    def _on_inspector_event(self, e):
        self._show_selected_container(PhysXInspectorModelState(e.type))

    def _on_path_changed(self, model: ui.SimpleStringModel):
        self.title = self._generate_title()

    def clean(self):
        # Subscriptions
        self._supportui_private = None
        self._model_inspector.get_inspected_prim_string_model().remove_value_changed_fn(
            self._sub_path_changed
        )
        self._sub_model_event = None
        self._sub_path_changed = None

        # Models
        self._model_inspector = None

        # UI Handlers
        self._handler_context_menu.clean()
        self._handler_context_menu = None
        self._handler_selection.clean()
        self._handler_selection = None

        # UI Elements
        self._ui_container_disabled = None
        self._ui_container_authoring = None
        self._ui_lock_label = None
        self._ui_path_label = None
        self._ui_main_stack = None
        if self._inspector_panel:
            self._inspector_panel.clean()
            self._inspector_panel = None
        if self._inspector_toolbar:
            self._inspector_toolbar.clean()

    def _update_inspector_window(self):

        with self.frame:
            with ui.VStack():
                self._build_header()
                ui.Spacer(height=5)

                self._ui_main_stack = ui.ZStack()
                with self._ui_main_stack:
                    self._ui_container_disabled = ui.VStack(visible=False, height=0)
                    self._ui_container_authoring = ui.VStack(visible=False)
                    with self._ui_container_authoring:
                        with ui.ScrollingFrame(
                            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                            style_type_name_override="TreeView",
                        ):
                            self._inspector_panel = InspectorPanel(
                                self._model_inspector,
                                self._handler_selection,
                                self._handler_context_menu,
                            )
                    with self._ui_container_disabled:
                        ui.Label(
                            "Structural changes detected. Inspector needs parsing the stage again to re-enable authoring",
                            word_wrap=True,
                        )
                        ui.Button(
                            "Re-Enable authoring",
                            width=0,
                            height=0,
                            clicked_fn=self._on_re_enable_authoring,
                        )
                self._show_selected_container(
                    self._supportui_private.get_inspector_state()
                )
                self._inspector_panel.set_use_omni_ui(True)

    def _on_re_enable_authoring(self):
        self._supportui_private.enable_inspector_authoring_mode()

    def _on_reset_to_authoring_start(self):
        self._inspector._inspector_simulation.stop_authoring_simulation()
        self._supportui_private.reset_inspector_to_authoring_start()

    def _on_commit_authoring_state(self):
        self._inspector._inspector_simulation.stop_authoring_simulation()
        self._supportui_private.commit_authoring_state()

    def _build_header(self):
        self._inspector_toolbar = InspectorToolbar(
            self._model_inspector, self._inspector._icon_folder, self
        )
