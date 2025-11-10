# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui as ui
from omni.kit.widget.stage import StageWidget
from pxr import Usd, Sdf, Tf, UsdGeom
from typing import Callable, Optional
from functools import partial
import weakref
from pxr import UsdPhysics
import omni.usd
from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorModel
from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorModelState
import carb
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
import omni.kit.notification_manager as nm

from omni.kit.widget.options_button import OptionsButton
from omni.kit.widget.options_button.style import UI_STYLE
from omni.kit.widget.options_menu import (
    OptionItem,
    OptionCustom,
    OptionLabelMenuItemDelegate,
    OptionSeparator,
    OptionRadios,
)


class StagePickerDialog:
    def __init__(
        self,
        stage,
        on_select_fn: Callable[[Usd.Prim], None],
        title=None,
        select_button_text=None,
        filter_type_list=None,
        filter_lambda=None,
    ):
        self._weak_stage = weakref.ref(stage)
        self._on_select_fn = on_select_fn

        if filter_type_list is None:
            self._filter_type_list = []
        else:
            self._filter_type_list = filter_type_list

        self._filter_lambda = filter_lambda
        self._selected_paths = []

        def on_window_visibility_changed(visible):
            if not visible:
                self._stage_widget.open_stage(None)
            else:
                # Only attach the stage when picker is open. Otherwise the Tf notice listener in StageWidget kills perf
                self._stage_widget.open_stage(self._weak_stage())

        self._window = ui.Window(
            title if title else "Select Prim",
            width=400,
            height=400,
            visible=False,
            flags=0,
            visibility_changed_fn=on_window_visibility_changed,
        )
        with self._window.frame:
            with ui.VStack():
                with ui.Frame():
                    self._stage_widget = StageWidget(None, columns_enabled=["Type"])
                    self._selection_watch = SelectionWatch(
                        stage=stage,
                        on_selection_changed_fn=self._on_selection_changed,
                        filter_type_list=filter_type_list,
                        filter_lambda=filter_lambda,
                    )
                    self._stage_widget.set_selection_watch(self._selection_watch)

                def on_select(weak_self):
                    weak_self = weak_self()
                    if not weak_self:
                        return

                    selected_prim = None
                    if len(weak_self._selected_paths) > 0:
                        selected_prim = stage.GetPrimAtPath(
                            Sdf.Path(weak_self._selected_paths[0])
                        )

                    if weak_self._on_select_fn:
                        weak_self._on_select_fn(selected_prim)

                    weak_self._window.visible = False

                with ui.VStack(
                    height=0, style={"Button.Label:disabled": {"color": 0xFF606060}}
                ):
                    self._label = ui.Label("Selected Path(s):\n\tNone")
                    self._button = ui.Button(
                        select_button_text if select_button_text else "Select",
                        height=10,
                        clicked_fn=partial(on_select, weak_self=weakref.ref(self)),
                        enabled=False,
                    )

    def clean(self):
        self._window.set_visibility_changed_fn(None)
        self._window.destroy()
        self._window = None
        self._selection_watch = None
        self._stage_widget.open_stage(None)
        self._stage_widget.destroy()
        self._stage_widget = None
        self._filter_type_list = None
        self._filter_lambda = None
        self._selected_paths = None
        self._on_select_fn = None
        self._weak_stage = None
        self._label.destroy()
        self._label = None
        self._button.destroy()
        self._button = None

    def show(self):
        self._selection_watch.reset(1)
        self._window.visible = True
        if self._filter_lambda is not None:
            self._stage_widget._filter_by_lambda(
                {"relpicker_filter": self._filter_lambda}, True
            )
        if self._filter_type_list:
            self._stage_widget._filter_by_type(self._filter_type_list, True)
            self._stage_widget.update_filter_menu_state(self._filter_type_list)

    def hide(self):
        self._window.visible = False

    def _on_selection_changed(self, paths):
        self._selected_paths = paths
        if self._button:
            self._button.enabled = len(self._selected_paths) > 0
        if self._label:
            text = "\n\t".join(self._selected_paths)
            label_text = "Selected Path"
            label_text += f":\n\t{text if len(text) else 'None'}"
            self._label.text = label_text


# This is copied from  omni.kit.property.usd.relationship to add the .HasAPI clause
def filter_prims(stage, prim_list, type_list):
    if len(type_list) != 0:
        filtered_selection = []
        for item in prim_list:
            prim = stage.GetPrimAtPath(item.path)
            if prim:
                for type in type_list:
                    if prim.IsA(type) or prim.HasAPI(type):  # <-- Line added
                        filtered_selection.append(item)
                        break
        if filtered_selection != prim_list:
            return filtered_selection
    return prim_list


class SelectionWatch(object):
    def __init__(
        self,
        stage,
        on_selection_changed_fn,
        filter_type_list,
        filter_lambda,
        tree_view=None,
    ):
        self._stage = weakref.ref(stage)
        self._last_selected_prim_paths = None
        self._filter_type_list = filter_type_list
        self._filter_lambda = filter_lambda
        self._on_selection_changed_fn = on_selection_changed_fn
        self._targets_limit = 0
        if tree_view:
            self.set_tree_view(tree_view)

    def reset(self, targets_limit):
        self._targets_limit = targets_limit
        self.clear_selection()

    def set_tree_view(self, tree_view):
        self._tree_view = tree_view
        self._tree_view.set_selection_changed_fn(self._on_widget_selection_changed)
        self._last_selected_prim_paths = None

    def clear_selection(self):
        if not self._tree_view:
            return

        self._tree_view.model.update_dirty()
        self._tree_view.selection = []
        if self._on_selection_changed_fn:
            self._on_selection_changed_fn([])

    def _on_widget_selection_changed(self, selection):
        stage = self._stage()
        if not stage:
            return

        prim_paths = [str(item.path) for item in selection if item]

        # Deselect instance proxy items if they were selected
        selection = [item for item in selection if item and not item.instance_proxy]

        # Although the stage view has filter, you can still select the ancestor of filtered prims, which might not match the type.
        selection = filter_prims(stage, selection, self._filter_type_list)

        # or the ancestor might not match the lambda filter
        if self._filter_lambda is not None:
            selection = [
                item
                for item in selection
                if self._filter_lambda(stage.GetPrimAtPath(item.path))
            ]

        # Deselect if over the limit
        if self._targets_limit > 0 and len(selection) > self._targets_limit:
            selection = selection[: self._targets_limit]

        if self._tree_view.selection != selection:
            self._tree_view.selection = selection
            prim_paths = [str(item.path) for item in selection]

        if prim_paths == self._last_selected_prim_paths:
            return

        self._last_selected_prim_paths = prim_paths
        if self._on_selection_changed_fn:
            self._on_selection_changed_fn(self._last_selected_prim_paths)

    def enable_filtering_checking(self, enable: bool):
        """
        It is used to prevent selecting the prims that are filtered out but
        still displayed when such prims have filtered children. When `enable`
        is True, SelectionWatch should consider filtering when changing Kit's
        selection.
        """
        pass

    def set_filtering(self, filter_string: Optional[str]):
        pass


class InspectorToolbar:
    def __init__(
        self, model_inspector: PhysXInspectorModel, icons_folder: str, inspector_window
    ):
        self._stage_picker = None
        self._inspector_state = PhysXInspectorModelState.AUTHORING
        self._model_inspector = model_inspector
        self._icons_folder = icons_folder
        self._inspector_window = inspector_window
        self._style = UI_STYLE.copy()
        self._development_mode = carb.settings.get_settings().get(
            "physics/developmentMode"
        )

        self._simulation_dirty: ui.SimpleBoolModel = (
            self._inspector_window._inspector._inspector_simulation.simulation_dirty
        )
        self._simulation_dirty_id = self._simulation_dirty.add_value_changed_fn(
            self._simulation_dirty_changed
        )

        self._build_options_items()
        self._build_ui()

    def _build_options_items(self):
        # OptionRadio expects values to be only strings...So we have to expose control and property type as string models

        # Slider Control
        slider_control_radio_model = self._model_inspector.get_control_type_model()

        slider_control_radios = [
            ("Sliders / Drags", None),
            (
                "Automatic",
                str(int(pxsupportui.PhysXInspectorModelControlType.AUTOMATIC)),
            ),
            (
                "Joint States Position",
                str(int(pxsupportui.PhysXInspectorModelControlType.JOINT_STATE)),
            ),
            (
                "Joint Drives Target Position",
                str(int(pxsupportui.PhysXInspectorModelControlType.JOINT_DRIVE)),
            ),
            (
                "Joint Drives Target Velocities",
                str(int(pxsupportui.PhysXInspectorModelControlType.JOINT_VELOCITY)),
            ),
        ]
        slider_tooltips = [
            "",
            "Inspector will choose what property to show",
            "Sliders will control JointStateAPI",
            "Sliders will control DriveAPI",
            "Sliders will control Joint Velocities",
        ]

        # Limits / Gains
        # OptionRadio expects values to be only strings...So we have to expose control and property type as string models
        limit_gains_radio_model = self._model_inspector.get_property_type_model()

        limit_gains_radios = [
            ("Limits / Gains", None),
            (
                "Show Joint Limits",
                str(int(pxsupportui.PhysXInspectorModelPropertyType.SHOW_LIMITS)),
            ),
            (
                "Show Joint Stiffness / Damping",
                str(int(pxsupportui.PhysXInspectorModelPropertyType.SHOW_GAINS)),
            ),
        ]
        limit_gains_tooltips = [
            "",
            "Shows Joint Limits on the same row as the slider / drag",
            "Shows Joint Gains on the same row as the slider / drag",
        ]

        # Check if we are during simulation
        during_simulation = self._inspector_state == PhysXInspectorModelState.RUNNING_SIMULATION


        # Build the items
        self._items = [
            # Control Radios
            OptionRadios(
                slider_control_radios,
                model=slider_control_radio_model,
                default=slider_control_radio_model.as_string,
                tooltips=slider_tooltips,
            ),
            # Gains / LimitsRadios
            OptionRadios(
                limit_gains_radios,
                model=limit_gains_radio_model,
                default=limit_gains_radio_model.as_string,
                tooltips=limit_gains_tooltips,
            ),
            # Simulation
            OptionSeparator("Authoring Simulation"),
            OptionItem(
                "Enable Gravity",
                model=self._model_inspector.get_enable_gravity_model(),
                enabled=not during_simulation,
            ),
            OptionItem(
                "Use QuasiStatic mode",
                model=self._model_inspector.get_enable_quasi_static_mode_model(),
                enabled=not during_simulation,
            ),
            OptionItem(
                "Fix Articulation Base",
                model=self._model_inspector.get_fix_articulation_base_model(),
                enabled=not during_simulation,
            ),
            # Visualization
            OptionSeparator("Visualization"),
            OptionItem(
                "Show Joints Hierarchy", on_value_changed_fn=self._on_show_hierarchy
            ),
            OptionItem(
                "Show Viewport Mass Overlay",
                model=self._model_inspector.get_show_masses_and_inertia_model(),
            ),
        ]
        if self._development_mode:
            self._items.insert(
                len(self._items) - 2,
                OptionItem(
                    "Show Python UI",
                    on_value_changed_fn=self.on_show_omni_ui,
                    default=True,
                ),
            )

    def on_inspector_event(self, type):
        self._inspector_state = type
        self._build_options_items()
        self._button.model.rebuild_items(self._items)

    def _simulation_dirty_changed(self, model):
        self._btn_discard.visible = model.as_bool
        self._btn_commit.visible = model.as_bool

    def on_show_omni_ui(self, use_omni_ui):
        self._inspector_window._inspector_panel.set_use_omni_ui(use_omni_ui)

    def _on_show_hierarchy(self, show_hierarchy: bool):
        if show_hierarchy:
            self._inspector_window._inspector_panel.set_inspector_type(
                pxsupportui.PhysXInspectorModelInspectorType.INSPECTOR_TYPE_JOINTS_BODIES_HIERARCHY
            )
        else:
            self._inspector_window._inspector_panel.set_inspector_type(
                pxsupportui.PhysXInspectorModelInspectorType.INSPECTOR_TYPE_JOINTS_LIST
            )

    def _on_reset_to_authoring_start(self):
        self._simulation_dirty.set_value(False)
        nm.post_notification(
            f"Discarding Physics Inspector authoring changes.\nPreserving changes to Target Position or other attributes.",
            duration=5,
        )
        self._inspector_window._on_reset_to_authoring_start()

    def _on_commit_authoring_state(self):
        self._simulation_dirty.set_value(False)
        nm.post_notification(
            f"Committing Physics Inspector authoring simulation changes.",
            duration=3,
        )
        self._inspector_window._on_commit_authoring_state()

    def _on_new_inspector_window(self):
        self._inspector_window._inspector.add_inspector_window()

    def clean(self):
        self._simulation_dirty.remove_value_changed_fn(self._simulation_dirty_id)
        self._simulation_dirty = None
        self._model_inspector = None
        self._inspector_window = None
        if self._stage_picker:
            self._stage_picker.hide()
            self._stage_picker.clean()
            self._stage_picker = None

    def _build_ui(self):
        with ui.HStack(height=0):
            ui.Button(
                image_url=f"{self._icons_folder}/inspector/new_window.svg",
                style_type_name_override="OptionsButton",
                width=ui.Pixel(24),
                height=ui.Pixel(24),
                tooltip="Create new inspector panel",
                style=self._style,
                clicked_fn=self._on_new_inspector_window,
            )
            ui.Button(
                image_url=f"{self._icons_folder}/inspector/select_articulation.svg",
                style_type_name_override="OptionsButton",
                width=ui.Pixel(24),
                height=ui.Pixel(24),
                tooltip="Select Articulation for this inspector panel",
                style=self._style,
                clicked_fn=self._select,
            )
            ui.Button(
                image_url=f"{self._icons_folder}/inspector/sync_with_stage.svg",
                style_type_name_override="OptionsButton",
                width=ui.Pixel(24),
                height=ui.Pixel(24),
                tooltip="Use current stage selection for this inspector panel",
                style=self._style,
                clicked_fn=self._select_current,
            )
            ui.Spacer(width=5)
            model = ui.SimpleStringModel("[No selection]")
            self.label_selection = ui.StringField(
                model=model, height=ui.Pixel(24), read_only=True
            )
            self._btn_discard = ui.Button(
                image_url=f"{self._icons_folder}/inspector/discard.svg",
                style_type_name_override="OptionsButton",
                width=ui.Pixel(24),
                height=ui.Pixel(24),
                tooltip="Discards inspector authoring simulation changes.\nChanges to Target Position or other attributes will be preserved.",
                style=self._style,
                clicked_fn=self._on_reset_to_authoring_start,
            )
            self._btn_commit = ui.Button(
                image_url=f"{self._icons_folder}/inspector/commit.svg",
                style_type_name_override="OptionsButton",
                width=ui.Pixel(24),
                height=ui.Pixel(24),
                tooltip="Commit inspector authoring simulation changes",
                style=self._style,
                clicked_fn=self._on_commit_authoring_state,
            )
            self._simulation_dirty_changed(self._simulation_dirty)
            self._button = OptionsButton(self._items)

    def _select_current(self):
        usdcontext = omni.usd.get_context()
        selected_prims = usdcontext.get_selection().get_selected_prim_paths()
        self._model_inspector.set_selected_paths(selected_prims)
        if len(selected_prims) == 1:
            self.label_selection.model.set_value(selected_prims[0])
        elif len(selected_prims) > 1:
            self.label_selection.model.set_value(
                f"{selected_prims[0]} (+{len(selected_prims)-1})"
            )
        else:
            self.label_selection.model.set_value("None")

    def _select(self):

        def on_select_articulation(weak_self, skeleton_prim):
            weak_self = weak_self()
            if not weak_self:
                return
            if not skeleton_prim:
                return
            weak_self.label_selection.model.set_value(
                skeleton_prim.GetPath().pathString
            )
            self._model_inspector.set_selected_paths(
                [skeleton_prim.GetPath().pathString]
            )

        self._stage_picker = StagePickerDialog(
            omni.usd.get_context().get_stage(),
            partial(on_select_articulation, weakref.ref(self)),
            "Select Articulation",
            None,
            [UsdPhysics.ArticulationRootAPI],
        )
        self._stage_picker.show()
