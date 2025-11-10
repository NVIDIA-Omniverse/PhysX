# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui as ui
import carb.settings
import omni.kit.app
import weakref
from pxr import Sdf, Usd
from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorModel
from carb.eventdispatcher import get_eventdispatcher


class InspectorSelectionHandler:
    def __init__(self, model_inspector: PhysXInspectorModel, inspector):
        self._model_inspector = model_inspector
        self._inspector = inspector
        self._settings = carb.settings.get_settings()
        self._last_selected_prim_paths = []
        self._selection = None
        self._ui_tree_views: set[ui.TreeView] = set()
        self._disable_selection_listening = False
        self._inside_change_selected = False
        usdcontext = omni.usd.get_context()
        if usdcontext is not None:
            self._selection = usdcontext.get_selection()
            self._sub_stage_event = get_eventdispatcher().observe_event(
                observer_name="omni.physx.supportui:InspectorSelectionHandler",
                event_name=usdcontext.stage_event_name(omni.usd.StageEventType.SELECTION_CHANGED),
                on_event=lambda _: self._on_stage_selection_changed_event()
            )

    def clean(self):
        self._inspector = None
        self._model_inspector = None
        self._settings = None
        self._last_selected_prim_paths = None
        self._selection = None
        self._ui_tree_views: set[ui.TreeView] = set()
        self._selection = None
        self._sub_stage_event = None

    def add_tree_view(self, tree_view):
        self._ui_tree_views.add(weakref.ref(tree_view))
        # Setup initial selection
        selection = self._model_inspector.get_items_matching_paths(
            self._selection.get_selected_prim_paths()
        )
        self._disable_selection_listening = True
        tree_view.selection = selection
        self._disable_selection_listening = False

    def _on_stage_selection_changed_event(self):
        prim_paths = self._selection.get_selected_prim_paths()
        self.set_selection(prim_paths)

    def refresh_selection(self):
        prim_paths = self._last_selected_prim_paths
        self._last_selected_prim_paths = []
        self.set_selection(prim_paths)

    def set_selection(self, prim_paths):
        if not len(self._ui_tree_views) or self._disable_selection_listening:
            return

        if prim_paths == self._last_selected_prim_paths:
            return

        self._last_selected_prim_paths = prim_paths
        self._disable_selection_listening = True
        selection = self._model_inspector.get_items_matching_paths(prim_paths)
        to_remove = []
        for weak_tree_view in self._ui_tree_views:
            tree_view = weak_tree_view()
            if tree_view:
                tree_view.selection = selection
                # TODO: we could implement "expand all parent nodes" leading to selected ones, similar to stage widget
            else:
                to_remove.append(weak_tree_view)
        for weak_tree_view in to_remove:
            self._ui_tree_views.remove(weak_tree_view)

        self._disable_selection_listening = False

    def get_selection(self):
        return self._last_selected_prim_paths

    def change_property_for_selection(self, property_path: Sdf.Path):
        if self._inside_change_selected:
            return
        carb.profiler.begin(1, "change_property_for_selection")
        self._inside_change_selected = True
        selection = omni.usd.get_context().get_selection().get_selected_prim_paths()
        stage = omni.usd.get_context().get_stage()
        original_prop = stage.GetPropertyAtPath(property_path)
        changed_token = property_path.name
        props_to_change = []
        found_triggering_node = False
        for sel in selection:
            prop_path = Sdf.Path(sel).AppendProperty(changed_token)
            if prop_path != property_path:
                prop = stage.GetPropertyAtPath(prop_path)
                if prop.IsValid():
                    props_to_change.append(prop)
            else:
                found_triggering_node = True

        # We only change other properties if we are inside the selection
        if found_triggering_node:
            for prop in props_to_change:
                prop.Set(original_prop.Get(), Usd.TimeCode.Default())

        if len(props_to_change) > 0:
            self._model_inspector.refresh_model_values()
        carb.profiler.end(1)
        self._inside_change_selected = False

    def push_view_selection_to_stage(self, selection, current_tree_view):
        if self._disable_selection_listening:
            return
        prim_paths = [
            self._model_inspector.get_item_value_model(item, 1).as_string
            for item in selection
            if (
                item
                and Sdf.Path.IsValidPathString(
                    self._model_inspector.get_item_value_model(item, 1).as_string
                )
            )
        ]

        if prim_paths == self._last_selected_prim_paths:
            return

        self._last_selected_prim_paths = prim_paths

        self._disable_selection_listening = True
        for weak_tree_view in self._ui_tree_views:
            tree_view = weak_tree_view()
            if tree_view is not None and tree_view != current_tree_view:
                tree_view.selection = []
        self._selection.set_selected_prim_paths(prim_paths, False)
        self._disable_selection_listening = False
