# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui as ui
import omni.kit.app
from pxr import Sdf, UsdPhysics
from typing import Sequence
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorModel
from .inspector_selection import InspectorSelectionHandler


class InspectorContextMenuHandler:
    def __init__(self):
        self._ui_context_menu = None

    def clean(self):
        self._ui_context_menu = None

    def build_context_menu(
        self,
        tree_view_selection: Sequence[ui.AbstractItem],
        handler_selection: InspectorSelectionHandler,
        model: PhysXInspectorModel,
    ):
        num_joints = 0
        num_bodies = 0
        stage = omni.usd.get_context().get_stage()
        for item in tree_view_selection:
            usd_path = model.get_item_value_model(item, 1).as_string
            if not Sdf.Path.IsValidPathString(usd_path):
                continue
            prim = stage.GetPrimAtPath(usd_path)
            if prim:
                if prim.IsA(UsdPhysics.Joint):
                    num_joints = num_joints + 1
                elif prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(
                    UsdPhysics.CollisionAPI
                ):
                    num_bodies = num_bodies + 1
                if num_joints > 0 and num_bodies > 0:
                    break
        if num_joints == 0 and num_bodies == 0:
            return
        if self._ui_context_menu == None:
            self._ui_context_menu = ui.Menu("Context menu")

        self._ui_context_menu.clear()
        with self._ui_context_menu:
            if (
                model.get_inspector_type()
                == pxsupportui.PhysXInspectorModelInspectorType.INSPECTOR_TYPE_JOINTS_LIST
            ):
                ui.MenuItem(
                    "Select all connected joint colliders",
                    triggered_fn=model.select_all_connected_joint_shapes,
                )
                ui.MenuItem(
                    "Select all connected joint bodies",
                    triggered_fn=model.select_all_connected_links,
                )
                if tree_view_selection and len(tree_view_selection) > 0:
                    ui.MenuItem(
                        "Clear Joint Limits",
                        triggered_fn=lambda: self._clear_joint_limits(
                            model, handler_selection, tree_view_selection
                        ),
                    )
                    ui.MenuItem(
                        "Set Joint Limits",
                        triggered_fn=lambda: self._set_joint_limits(
                            model, handler_selection, tree_view_selection
                        ),
                    )
            else:
                if num_joints > 0:
                    ui.MenuItem(
                        "Select all connected joint colliders",
                        triggered_fn=model.select_all_connected_joint_shapes,
                    )
                    ui.MenuItem(
                        "Select all connected joint bodies",
                        triggered_fn=model.select_all_connected_links,
                    )
                if num_bodies > 0:
                    ui.MenuItem(
                        "Select all connected body colliders",
                        triggered_fn=model.select_all_connected_body_shapes,
                    )
                    ui.MenuItem(
                        "Select all connected body joints",
                        triggered_fn=model.select_all_connected_body_joints,
                    )
        self._ui_context_menu.show()

    def _change_limits(
        self,
        model,
        handler_selection: InspectorSelectionHandler,
        tree_view_selection: Sequence[ui.AbstractItem],
        lower_value: float,
        upper_value: float,
    ):
        item = tree_view_selection[0]
        prim_path = Sdf.Path(model.get_item_value_model(item, 1).as_string)
        lower_limit = prim_path.AppendProperty("physics:lowerLimit")
        upper_limit = prim_path.AppendProperty("physics:upperLimit")
        stage = omni.usd.get_context().get_stage()
        lower_attribute = stage.GetPropertyAtPath(lower_limit)
        upper_attribute = stage.GetPropertyAtPath(upper_limit)
        lower_attribute.Set(lower_value)
        upper_attribute.Set(upper_value)
        handler_selection.change_property_for_selection(lower_limit)
        handler_selection.change_property_for_selection(upper_limit)

    def _clear_joint_limits(
        self,
        model,
        handler_selection: InspectorSelectionHandler,
        tree_view_selection: Sequence[ui.AbstractItem],
    ):
        self._change_limits(
            model, handler_selection, tree_view_selection, float("inf"), float("inf")
        )

    def _set_joint_limits(
        self,
        model,
        handler_selection: InspectorSelectionHandler,
        tree_view_selection: Sequence[ui.AbstractItem],
    ):
        self._change_limits(model, handler_selection, tree_view_selection, -460.0, +460)
