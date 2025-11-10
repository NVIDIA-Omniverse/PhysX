# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui as ui
import carb.settings
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
import asyncio
from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorWidget
from omni.physxsupportui.bindings._physxSupportUi import (
    PhysXInspectorModelDataShapeType,
)
from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorModel
from .inspector_selection import InspectorSelectionHandler
from .inspector_context_menu import InspectorContextMenuHandler
from pxr import Sdf, Usd, UsdPhysics, PhysxSchema, UsdGeom
import omni.usd
import math
import sys
import carb
from .inspector_simulation import InspectorSimulation


class InspectorTreeDelegate(ui.AbstractItemDelegate):
    def __init__(
        self,
        model_inspector: PhysXInspectorModel,
        handler_selection: InspectorSelectionHandler,
        inspector_simulation: InspectorSimulation,
    ):
        super().__init__()
        self._model_inspector = model_inspector
        self._inspector_simulation = inspector_simulation
        self._handler_selection = handler_selection
        self._inside_change_selected = False

    def clean(self):
        self._model_inspector = None
        self._inspector_simulation = None
        self._handler_selection = None

    def is_supported_joint_type(self, prim: Usd.Prim) -> bool:
        if prim.HasAPI(PhysxSchema.PhysxMimicJointAPI):
            return False # We can only control the source joint
        elif prim.IsA(UsdPhysics.RevoluteJoint):
            return True
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            return True
        return False

    def get_default_slider_limit(self, prim: Usd.Prim) -> float:
        if prim.IsA(UsdPhysics.RevoluteJoint):
            return 460.0
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            return 1000.0
        return False

    def get_unsupported_joint_description(self, prim: Usd.Prim) -> str:
        if prim.HasAPI(PhysxSchema.PhysxMimicJointAPI):
            return "Mimic Joint"
        elif prim.IsA(UsdPhysics.SphericalJoint):
            return "Spherical Joint"
        elif prim.IsA(UsdPhysics.FixedJoint):
            return "Fixed Joint"
        elif prim.IsA(UsdPhysics.DistanceJoint):
            return "Distance Joint"
        elif prim.IsA(PhysxSchema.PhysxPhysicsRackAndPinionJoint):
            return "Rack and Pinion Joint"
        elif prim.IsA(PhysxSchema.PhysxPhysicsGearJoint):
            return "Gear Joint"
        elif prim.IsA(UsdPhysics.Joint):
            return "D6 Joint"
        return "Unknown Joint Type"

    def get_lower_limit_path(self, prim: Usd.Prim) -> Sdf.Path:
        if prim.IsA(UsdPhysics.Joint):
            return prim.GetPath().AppendProperty("physics:lowerLimit")
        return None

    def get_upper_limit_path(self, prim: Usd.Prim) -> Sdf.Path:
        if prim.IsA(UsdPhysics.Joint):
            return prim.GetPath().AppendProperty("physics:upperLimit")
        return None

    def _build_min_max_item(
        self,
        prim: Usd.Prim,
        min_or_max: str,
        main_model: ui.SimpleFloatModel,
        main_path: Sdf.Path,
        other_model: ui.SimpleFloatModel,
        other_path: Sdf.Path,
        default_limit: float,
    ):
        step = InspectorTreeDelegate._get_step_for_prim(prim)

        with ui.ZStack(
            style={
                ":disabled": {
                    "background_color": 0xFF444444,
                    "secondary_color": 0xFF444444,
                    "color": 0xFF777777,
                }
            }
        ):
            min_max_drag = ui.FloatDrag(
                model=main_model,
                step=step,
                precision=3,
                style={"background_color": 0xFF006600, "secondary_color": 0xFF006600},
            )
            min_max_label = ui.Button(
                "Set Limit", style={"background_color": 0xFF006666}
            )

        def change_limit_main(model: ui.SimpleFloatModel):
            if math.isinf(model.get_value_as_float()):
                min_max_label.visible = True
            else:
                min_max_label.visible = False
            min_max_drag.visible = not min_max_label.visible
            prim.GetAttribute(main_path.name).Set(model.get_value_as_float())

        def on_changed_other(model: ui.SimpleFloatModel):
            value = 1 if default_limit > 0 else -1
            if math.isinf(model.get_value_as_float()):
                setattr(min_max_drag, min_or_max, getattr(sys.float_info, min_or_max))
            else:
                setattr(min_max_drag, min_or_max, model.get_value_as_float() + value)
            prim.GetAttribute(other_path.name).Set(model.get_value_as_float())

        self._inspector_simulation.add_control_to_disable(min_max_drag)
        min_max_label.set_clicked_fn(lambda: main_model.set_value(default_limit))
        main_model.add_value_changed_fn(change_limit_main)
        other_model.add_value_changed_fn(on_changed_other)
        main_model.add_value_changed_fn(
            lambda _: self._handler_selection.change_property_for_selection(main_path)
        )
        change_limit_main(main_model)
        on_changed_other(other_model)

    def _build_gains_item(
        self, prim: Usd.Prim, property_name: str, model: ui.SimpleFloatModel
    ):
        step = InspectorTreeDelegate._get_step_for_prim(prim)
        prim_path = str(prim.GetPath())
        with ui.ZStack(
            style={
                ":disabled": {
                    "background_color": 0xFF444444,
                    "secondary_color": 0xFF444444,
                    "color": 0xFF777777,
                }
            }
        ):
            value_drag = ui.FloatDrag(
                model=model,
                step=step,
                precision=1,
                style={"background_color": 0xFF006600, "secondary_color": 0xFF006600},
            )
            btn_set_value = self._build_apply_drive_api(prim_path)

        def changed_value(model: ui.SimpleFloatModel):
            value = model.get_value_as_float()
            value_is_inf = math.isinf(value)
            btn_set_value.visible = value_is_inf
            value_drag.visible = not value_is_inf
            if not value_is_inf:
                prim.GetAttribute(property_name).Set(value)

        model.add_value_changed_fn(changed_value)
        property_path = prim.GetPath().AppendProperty(property_name)
        model.add_value_changed_fn(
            lambda _: self._handler_selection.change_property_for_selection(
                property_path
            )
        )
        changed_value(model)

    def _apply_drive_api(self, prim_path: str):
        selection = self._handler_selection.get_selection()
        if not prim_path in selection:
            selection = [prim_path]
        stage = omni.usd.get_context().get_stage()

        for path in selection:
            prim = stage.GetPrimAtPath(path)
            driveAPI = None
            if prim.IsA(UsdPhysics.RevoluteJoint):
                driveAPI = UsdPhysics.DriveAPI.Apply(prim, "angular")
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                driveAPI = UsdPhysics.DriveAPI.Apply(prim, "linear")
            if driveAPI:
                driveAPI.CreateStiffnessAttr(0.0)
                driveAPI.CreateDampingAttr(0.0)

    def _apply_joint_state_api(self, prim_path: str):
        selection = self._handler_selection.get_selection()
        if not prim_path in selection:
            selection = [prim_path]
        stage = omni.usd.get_context().get_stage()

        for path in selection:
            prim = stage.GetPrimAtPath(path)

            jointStateAPI = None
            if prim.IsA(UsdPhysics.RevoluteJoint):
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(prim, "angular")
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                jointStateAPI = PhysxSchema.JointStateAPI.Apply(prim, "linear")
            if jointStateAPI:
                jointStateAPI.CreatePositionAttr().Set(0)
                jointStateAPI.CreateVelocityAttr().Set(0)

    def _build_apply_joint_state_api(self, prim_path: str):
        btn_set_value = ui.Button(
            "Add JointStateAPI", style={"background_color": 0xFF006666}
        )
        btn_set_value.set_clicked_fn(lambda: self._apply_joint_state_api(prim_path))

    def _build_apply_drive_api(self, prim_path: str):
        btn_set_value = ui.Button(
            "Add DriveAPI", style={"background_color": 0xFFFF0000}
        )
        btn_set_value.set_clicked_fn(lambda: self._apply_drive_api(prim_path))
        return btn_set_value

    @staticmethod
    def _get_step_for_prim(prim: Usd.Prim) -> float:
        if prim.IsA(UsdPhysics.PrismaticJoint):
            meters_per_unit: float = UsdGeom.GetStageMetersPerUnit(prim.GetStage())
            millimeters_per_unit = meters_per_unit * 1000
            return 1 / millimeters_per_unit
        else:
            return 0.1

    def _build_value_item(
        self, prim: Usd.Prim, model: PhysXInspectorModel, item: ui.AbstractItem
    ):
        # TODO: We should detect if the joint is an articulation joint or maximal joitn and disable joint state position
        prim_path = str(prim.GetPath())
        value_attribute_name = self._model_inspector.get_joint_value_attribute_name(
            prim_path
        )
        match value_attribute_name:
            case "state:linear:physics:position" | "state:angular:physics:position":
                slider_color = 0xFF006666
                should_simulate_on_release = True
                force_show_drag = False
            case (
                "drive:linear:physics:targetVelocity"
                | "drive:angular:physics:targetVelocity"
            ):
                slider_color = 0xFF770077
                should_simulate_on_release = False
                force_show_drag = True
            case (
                "drive:linear:physics:targetPosition"
                | "drive:angular:physics:targetPosition"
            ):
                slider_color = 0xFFFF0000
                should_simulate_on_release = True
                force_show_drag = False
            case "No JointStateAPI":
                self._build_apply_joint_state_api(prim_path)
                return
            case "No JointStateAPI (Maximal Joint)":
                ui.Label(
                    "JointStateAPI not supported",
                    style_type_name_override="TreeView.Item",
                    width=0,
                )
                return
            case "No DriveAPI":
                self._build_apply_drive_api(prim_path)
                return
            case "No JointStateAPI or DriveAPI":
                self._build_apply_joint_state_api(prim_path)
                self._build_apply_drive_api(prim_path)
                return
            case value:
                ui.Label(value, style_type_name_override="TreeView.Item", width=0)
                return

        default_limit = self.get_default_slider_limit(prim)
        val_usd_model: ui.SimpleFloatModel = model.get_item_value_model(item, 3)
        min_usd_model: ui.SimpleFloatModel = model.get_item_value_model(item, 4)
        max_usd_model: ui.SimpleFloatModel = model.get_item_value_model(item, 5)

        step = InspectorTreeDelegate._get_step_for_prim(prim)
        with ui.ZStack():
            value_drag = ui.FloatDrag(
                model=val_usd_model,
                step=step,
                style={
                    "draw_mode": ui.SliderDrawMode.DRAG,
                    "background_color": slider_color,
                },
            )
            value_slider = ui.FloatSlider(
                width=ui.Fraction(1),
                model=val_usd_model,
                alignment=ui.Alignment.LEFT_CENTER,
                step=step,
                min=-360,
                max=+360,
                precision=3,
                style={
                    "draw_mode": ui.SliderDrawMode.FILLED,
                    "border_radius": 0,
                    "secondary_color": slider_color,
                    "background_color": 0xFF23211F,
                },
            )

        def choose_float_or_drag():
            # If we have no limits set, we can't show the slider
            if (
                force_show_drag
                or math.isinf(min_usd_model.get_value_as_float())
                or math.isinf(max_usd_model.get_value_as_float())
            ):
                value_slider.visible = False
                value_drag.visible = True
            else:
                value_drag.visible = False
                value_slider.visible = True

        def change_limit_min(min_model: ui.SimpleFloatModel):
            if min_model.get_value_as_float() == float("-inf"):
                value_slider.min = -default_limit
            else:
                value_slider.min = min_model.get_value_as_float()
            choose_float_or_drag()

        def change_limit_max(max_model: ui.SimpleFloatModel):
            if max_model.get_value_as_float() == float("+inf"):
                value_slider.max = +default_limit
            else:
                value_slider.max = max_model.get_value_as_float()
            choose_float_or_drag()

        val_usd_model.add_value_changed_fn(
            lambda m: self._on_value_changed(m, str(prim.GetPrimPath()))
        )

        # We want to step the authoring simulation as long as user is pressing on the slider
        value_slider.set_mouse_pressed_fn(
            lambda x, y, b, m: self._on_value_mouse_pressed(value_attribute_name)
        )
        value_drag.set_mouse_pressed_fn(
            lambda x, y, b, m: self._on_value_mouse_pressed(value_attribute_name)
        )
        value_slider.set_mouse_moved_fn(
            lambda x, y, b, m: self._on_value_mouse_moved(value_attribute_name)
        )

        # For joint state we start on every change, as it will run just a single simulation step (see start_authoring_simulation)
        if should_simulate_on_release:
            value_slider.set_mouse_released_fn(
                lambda x, y, b, m: self._on_value_mouse_released(value_attribute_name)
            )

        # We change slider min/max with upper and lower limits values
        min_usd_model.add_value_changed_fn(change_limit_min)
        max_usd_model.add_value_changed_fn(change_limit_max)
        change_limit_min(min_usd_model)
        change_limit_max(max_usd_model)
        choose_float_or_drag()

    def _on_value_mouse_pressed(self, value_attribute_name: str):
        self._inspector_simulation.start_authoring_simulation(value_attribute_name)

    def _on_value_mouse_released(self, value_attribute_name: str):
        self._inspector_simulation.start_authoring_simulation(value_attribute_name)

    def _on_value_mouse_moved(self, value_attribute_name: str):
        self._inspector_simulation.start_authoring_simulation(value_attribute_name)

    def _on_value_changed(self, model: ui.SimpleFloatModel, prim_path: str):
        if self._handler_selection is None:
            return
        selection = self._handler_selection.get_selection()
        if not prim_path in selection:
            selection = [prim_path]

        for joint_path in selection:
            # This disables the notice handler so we don't feedback here
            self._model_inspector.set_joint_value(joint_path, model.as_float)

        if len(selection) > 1:
            # If we've been touching other joints (due to multi selection), we need to get their sliders updated
            self._model_inspector.refresh_model_values()

    def _build_joint_row_column(
        self,
        prim: Usd.Prim,
        column_id: int,
        model: PhysXInspectorModel,
        item: ui.AbstractItem,
    ):
        property_type = pxsupportui.PhysXInspectorModelPropertyType(
            self._model_inspector.get_property_type_model().as_int
        )
        match property_type:
            case pxsupportui.PhysXInspectorModelPropertyType.SHOW_LIMITS:
                COLUMN_MAX = 1
                COLUMN_VAL = 2
                COLUMN_MIN = 3
            case pxsupportui.PhysXInspectorModelPropertyType.SHOW_GAINS:
                COLUMN_MAX = 1
                COLUMN_MIN = 2
                COLUMN_VAL = 3

        if self.is_supported_joint_type(prim):
            default_limit = self.get_default_slider_limit(prim)
            if column_id == COLUMN_MAX:
                match property_type:
                    case pxsupportui.PhysXInspectorModelPropertyType.SHOW_LIMITS:
                        self._build_min_max_item(
                            prim,
                            "max",
                            model.get_item_value_model(item, 4),
                            self.get_lower_limit_path(prim),
                            model.get_item_value_model(item, 5),
                            self.get_upper_limit_path(prim),
                            -default_limit,
                        )
                    case pxsupportui.PhysXInspectorModelPropertyType.SHOW_GAINS:
                        property_name = (
                            "drive:angular:physics:damping"
                            if prim.IsA(UsdPhysics.RevoluteJoint)
                            else "drive:linear:physics:damping"
                        )
                        self._build_gains_item(
                            prim, property_name, model.get_item_value_model(item, 7)
                        )

            elif column_id == COLUMN_VAL:
                self._build_value_item(prim, model, item)
            elif column_id == COLUMN_MIN:
                match property_type:
                    case pxsupportui.PhysXInspectorModelPropertyType.SHOW_LIMITS:
                        self._build_min_max_item(
                            prim,
                            "min",
                            model.get_item_value_model(item, 5),
                            self.get_upper_limit_path(prim),
                            model.get_item_value_model(item, 4),
                            self.get_lower_limit_path(prim),
                            +default_limit,
                        )
                    case pxsupportui.PhysXInspectorModelPropertyType.SHOW_GAINS:
                        property_name = (
                            "drive:angular:physics:stiffness"
                            if prim.IsA(UsdPhysics.RevoluteJoint)
                            else "drive:linear:physics:stiffness"
                        )
                        self._build_gains_item(
                            prim, property_name, model.get_item_value_model(item, 6)
                        )

        elif column_id == 2:
            ui.Label(
                self.get_unsupported_joint_description(prim),
                style_type_name_override="TreeView.Item",
                width=0,
                tooltip=prim.GetPath().pathString,
            )

    def _build_joint_row(
        self, prim: Usd.Prim, model: PhysXInspectorModel, item: ui.AbstractItem
    ):
        if self.is_supported_joint_type(prim):
            self._build_value_item(prim, model, item)
        else:
            ui.Label(
                self.get_unsupported_joint_description(prim),
                style_type_name_override="TreeView.Item",
                width=0,
                tooltip=prim.GetPath().pathString,
            )

    def build_widget(
        self,
        model: PhysXInspectorModel,
        item: ui.AbstractItem,
        column_id: int,
        level: int,
        expanded: bool,
    ):
        if (
            item is None
        ):  # OM-64107 and/or OM-52829: connected to other references to the issue in this file
            return
        stage = omni.usd.get_context().get_stage()
        usd_path_model = model.get_item_value_model(item, 1)
        if column_id == 0:
            name_model = model.get_item_value_model(item, 0)
            if name_model is not None:
                ui.Label(
                    name_model.as_string,
                    style_type_name_override="TreeView.Item",
                    width=0,
                    tooltip=usd_path_model.as_string,
                )
        else:
            if usd_path_model is not None and Sdf.Path.IsValidPathString(
                usd_path_model.as_string
            ):
                prim = stage.GetPrimAtPath(usd_path_model.as_string)
                if prim.IsValid() and prim.IsA(UsdPhysics.Joint):
                    if self._is_joints_list():
                        self._build_joint_row_column(prim, column_id, model, item)
                    else:
                        self._build_joint_row(prim, model, item)
                else:
                    ui.Label(
                        usd_path_model.as_string,
                        style_type_name_override="TreeView.Item",
                        width=0,
                        tooltip=usd_path_model.as_string,
                    )

    def build_branch(
        self,
        model: PhysXInspectorModel,
        item: ui.AbstractItem,
        column_id: int,
        level: int,
        expanded: bool,
    ):
        if item is None:
            return
        if column_id == 0:
            with ui.HStack(height=0):
                ui.Spacer(width=4 * level)
                ui.Label(
                    self._get_branch_text(expanded, model.can_item_have_children(item)),
                    style_type_name_override="TreeView.Item",
                    width=15,
                )

    def _get_branch_text(self, expanded: bool, can_have_children: bool):
        return ("-" if expanded else "+") if can_have_children else " "

    def _is_joints_list(self) -> bool:
        return (
            self._model_inspector.get_inspector_type()
            == pxsupportui.PhysXInspectorModelInspectorType.INSPECTOR_TYPE_JOINTS_LIST
        )

    def _get_value_column_name(
        self, control_type: pxsupportui.PhysXInspectorModelControlType
    ) -> str:
        match control_type:
            case pxsupportui.PhysXInspectorModelControlType.AUTOMATIC:
                return "[ Drive Target or Joint State ] Position"
            case pxsupportui.PhysXInspectorModelControlType.JOINT_DRIVE:
                return "Drive Target Position"
            case pxsupportui.PhysXInspectorModelControlType.JOINT_STATE:
                return "Joint State Position"
            case pxsupportui.PhysXInspectorModelControlType.JOINT_VELOCITY:
                return "Joint Target Velocity"
            case _:
                return "Unknown"

    def build_header(self, column_id: int):
        style_type_name = "TreeView.Header"
        header_label = ""
        control_type = pxsupportui.PhysXInspectorModelControlType(
            self._model_inspector.get_control_type_model().as_int
        )
        property_type = pxsupportui.PhysXInspectorModelPropertyType(
            self._model_inspector.get_property_type_model().as_int
        )
        if self._is_joints_list():
            match property_type:
                case pxsupportui.PhysXInspectorModelPropertyType.SHOW_LIMITS:
                    header_label = [
                        "Joint Name",
                        "Lower Limit",
                        self._get_value_column_name(control_type),
                        "Upper Limit",
                    ][column_id]
                case pxsupportui.PhysXInspectorModelPropertyType.SHOW_GAINS:
                    header_label = [
                        "Joint Name",
                        "Damping",
                        "Stiffness",
                        self._get_value_column_name(control_type),
                    ][column_id]
        else:
            if column_id == 0:
                header_label = "Name"
            else:
                header_label = "Value"

        with ui.HStack():
            ui.Spacer(width=10)
            ui.Label(header_label, style_type_name_override=style_type_name)


class InspectorTreeModel(ui.AbstractItemModel):
    def __init__(self, model_inspector: PhysXInspectorModel, model_item):
        super().__init__()
        self._model_inspector = model_inspector
        self._model_item = model_item

    def get_item_value_model_count(self, item):
        if self._is_joints_list():
            return 4
        else:
            return self._model_inspector.get_item_value_model_count(item)

    def get_item_children(self, item):
        if item is None:
            children = self._model_inspector.get_item_children(self._model_item)
            # We add a "dummy" none item, to correct the height of the treeview due to (OM-64107 and/or OM-52829)
            children.append(None)
            return children
        return self._model_inspector.get_item_children(item)

    def get_item_value_model(self, item, column_id):
        return self._model_inspector.get_item_value_model(item, column_id)

    def _is_joints_list(self) -> bool:
        return (
            self._model_inspector.get_inspector_type()
            == pxsupportui.PhysXInspectorModelInspectorType.INSPECTOR_TYPE_JOINTS_LIST
        )


class InspectorCategoryModel(ui.AbstractItemModel):
    def __init__(self, model_inspector: PhysXInspectorModel):
        super().__init__()
        self._flat_data_shape = False
        self._model_inspector = model_inspector
        self._inner_model_changed_id = model_inspector.add_item_changed_fn(
            self._inner_model_changed
        )

    def _is_joints_list(self) -> bool:
        return (
            self._model_inspector.get_inspector_type()
            == pxsupportui.PhysXInspectorModelInspectorType.INSPECTOR_TYPE_JOINTS_LIST
        )

    def clean(self):
        self._model_inspector.remove_item_changed_fn(self._inner_model_changed_id)
        self._model_inspector = None

    def _inner_model_changed(self, model, item):
        self._item_changed(None)

    def set_flat_data_shape(self, value):
        self._flat_data_shape = (
            PhysXInspectorModelDataShapeType(value)
            == PhysXInspectorModelDataShapeType.FLAT
        )

    def is_flat_data_shape(self):
        return self._flat_data_shape

    def get_item_value_model_count(self, item):
        if self.is_flat_data_shape():
            if self._is_joints_list():
                return 4
            else:
                return self._model_inspector.get_item_value_model_count(item)
        return 1

    def get_item_children(self, item):
        if self.is_flat_data_shape():
            return self._model_inspector.get_item_children(item)
        if not item:
            return self._model_inspector.get_item_children(item)
        return []

    def get_item_value_model(self, item, column_id):
        return self._model_inspector.get_item_value_model(item, column_id)


class InspectorCategoryDelegate(ui.AbstractItemDelegate):
    def __init__(
        self,
        delegate_tree: InspectorTreeDelegate,
        handler_selection: InspectorSelectionHandler,
        handler_context_menu: InspectorContextMenuHandler,
    ):
        super().__init__()
        self._delegate_tree = delegate_tree
        self._handler_selection = handler_selection
        self._handler_context_menu = handler_context_menu
        self._model_trees = []

    def clean(self):
        self._delegate_tree = None
        self._handler_selection = None
        self._handler_context_menu = None
        self._model_trees = []

    def build_branch(
        self, model: InspectorCategoryModel, item, column_id, level, expanded
    ):
        if model.is_flat_data_shape():
            self._delegate_tree.build_branch(
                model._model_inspector, item, column_id, level, expanded
            )
        else:
            super().build_branch(
                model._model_inspector, item, column_id, level, expanded
            )

    def build_widget(
        self, model: InspectorCategoryModel, item, column_id, level, expanded
    ):
        carb.profiler.begin(1, "InspectorTreeDelegate::build_widget")
        if model.is_flat_data_shape():
            self._delegate_tree.build_widget(
                model._model_inspector, item, column_id, level, expanded
            )
        else:
            style = (
                carb.settings.get_settings().get_as_string(
                    "/persistent/app/window/uiStyle"
                )
                or "NvidiaDark"
            )
            if style == "NvidiaLight":
                tree_style = {"CollapsableFrame": {"background_color": 0xFF535354}}
            else:
                tree_style = {"CollapsableFrame": {"background_color": 0xFF23211F}}

            text_model = model.get_item_value_model(item, column_id)
            with ui.VStack(
                content_clipping=True
            ):  # Blocks clicks going to the underlying treeview
                with ui.CollapsableFrame(text_model.as_string, style=tree_style):
                    tree_model = InspectorTreeModel(model._model_inspector, item)
                    self._model_trees.append(tree_model)
                    tree_view = ui.TreeView(
                        tree_model,
                        delegate=self._delegate_tree,
                        root_visible=False,
                        style={"margin": 0.5},
                        columns_resizable=True,
                        header_visible=False,
                        column_widths=[200, ui.Fraction(1)],
                    )
                    tree_view.set_selection_changed_fn(
                        lambda sel: self._handler_selection.push_view_selection_to_stage(
                            sel, tree_view
                        )
                    )
                    tree_view.set_mouse_pressed_fn(
                        lambda x, y, b, c: self._build_context_menu(b, tree_view, model)
                    )
                    self._handler_selection.add_tree_view(tree_view)

                    # OM-64107: we must set header_visible=True later in order to get correct treeview height
                    async def set_treeview_header_later():
                        await omni.kit.app.get_app().next_update_async()
                        tree_view.header_visible = True

                    # TODO: Figure out why we get error on the sliders
                    # Tried to call pure virtual function "AbstractValueModel::set_value"
                    asyncio.ensure_future(set_treeview_header_later())
        carb.profiler.end(1)

    def _build_context_menu(
        self, button: int, tree_view: ui.TreeView, model: InspectorCategoryModel
    ):
        if button == 1:
            self._handler_context_menu.build_context_menu(
                tree_view.selection, self._handler_selection, model._model_inspector
            )

    def build_header(self, column_id: int):
        self._delegate_tree.build_header(column_id)


class InspectorPanel:
    def __init__(
        self,
        model_inspector: PhysXInspectorModel,
        handler_selection: InspectorSelectionHandler,
        handler_context_menu: InspectorContextMenuHandler,
    ):
        # Models
        self._model_inspector = model_inspector
        self._model_category = InspectorCategoryModel(self._model_inspector)
        self._model_category.set_flat_data_shape(
            self._model_inspector.get_data_shape_model().get_value_as_int()
        )

        # UI Handlers
        self._handler_context_menu = handler_context_menu
        self._handler_selection = handler_selection

        # Delegates
        self._delegate_tree = InspectorTreeDelegate(
            self._model_inspector,
            self._handler_selection,
            handler_selection._inspector._inspector_simulation,
        )
        self._delegate_category = InspectorCategoryDelegate(
            self._delegate_tree, self._handler_selection, self._handler_context_menu
        )

        # Subscriptions
        self._sub_data_shape = (
            self._model_inspector.get_data_shape_model().add_value_changed_fn(
                self._on_data_shape_changed
            )
        )

        self._sub_control_type = (
            self._model_inspector.get_control_type_model().add_value_changed_fn(
                self._on_control_type_changed
            )
        )

        self._sub_property_type = (
            self._model_inspector.get_property_type_model().add_value_changed_fn(
                self._on_property_type_changed
            )
        )

        # UI Elements
        self._z_stack = None
        self._build_ui()

    def _build_ui(self):
        # Build UI
        if self._z_stack is None:
            self._z_stack = ui.ZStack()
        with self._z_stack:
            self._ui_root_omniui_widget = ui.TreeView(
                self._model_category,
                delegate=self._delegate_category,
                root_visible=False,
            )
            self._set_treeview_properties()
            self._ui_root_imgui_widget = PhysXInspectorWidget()
            self._ui_root_imgui_widget.model = self._model_inspector

    def clean(self):
        # Subscriptions
        self._model_inspector.get_data_shape_model().remove_value_changed_fn(
            self._sub_data_shape
        )
        self._sub_data_shape = None

        self._model_inspector.get_control_type_model().remove_value_changed_fn(
            self._sub_control_type
        )
        self._sub_control_type = None

        self._model_inspector.get_property_type_model().remove_value_changed_fn(
            self._sub_property_type
        )
        self._sub_property_type = None

        self._sub_path_changed = None
        self._sub_settings_lock = None

        # Models
        self._model_inspector = None
        self._ui_root_imgui_widget.model = None
        self._model_category.clean()
        self._model_category = None

        # Delegates
        self._delegate_category.clean()
        self._delegate_category = None
        self._delegate_tree.clean()
        self._delegate_tree = None

        # UI Elements
        self._ui_root_omniui_widget = None
        self._ui_root_imgui_widget = None

        # UI Handlers
        self._handler_context_menu = None
        self._handler_selection = None

    def set_inspector_type(self, type):
        self._model_inspector.set_inspector_type(type)
        self._ui_root_imgui_widget.inspector_type = type

    def set_use_omni_ui(self, use_omni_ui):
        self._ui_root_imgui_widget.visible = not use_omni_ui
        self._ui_root_omniui_widget.visible = use_omni_ui

    def _build_context_menu(self, button, tree_view, model):
        if button == 1:
            self._handler_context_menu.build_context_menu(
                tree_view.selection, self._handler_selection, model
            )

    def _set_treeview_properties(self):
        if self._model_category.is_flat_data_shape():
            self._ui_root_omniui_widget.columns_resizable = True
            self._ui_root_omniui_widget.header_visible = True
            if self._delegate_tree._is_joints_list():
                property_type = pxsupportui.PhysXInspectorModelPropertyType(
                    self._model_inspector.get_property_type_model().as_int
                )
                match property_type:
                    case pxsupportui.PhysXInspectorModelPropertyType.SHOW_LIMITS:
                        self._ui_root_omniui_widget.column_widths = [
                            ui.Fraction(0.55),
                            ui.Fraction(0.4),
                            ui.Fraction(1),
                            ui.Fraction(0.4),
                        ]

                    case pxsupportui.PhysXInspectorModelPropertyType.SHOW_GAINS:
                        self._ui_root_omniui_widget.column_widths = [
                            ui.Fraction(0.55),
                            ui.Fraction(0.4),
                            ui.Fraction(0.4),
                            ui.Fraction(1),
                        ]
            else:
                self._ui_root_omniui_widget.column_widths = [
                    ui.Fraction(0.5),
                    ui.Fraction(1),
                ]
            self._ui_root_omniui_widget.style = {"margin": 0.5}
            self._ui_root_omniui_widget.set_selection_changed_fn(
                lambda sel: self._handler_selection.push_view_selection_to_stage(
                    sel, self._ui_root_omniui_widget
                )
            )
            self._ui_root_omniui_widget.set_mouse_pressed_fn(
                lambda x, y, b, c: self._build_context_menu(
                    b, self._ui_root_omniui_widget, self._model_inspector
                )
            )
            self._handler_selection.add_tree_view(self._ui_root_omniui_widget)
        else:
            self._ui_root_omniui_widget.columns_resizable = False
            self._ui_root_omniui_widget.header_visible = False
            self._ui_root_omniui_widget.column_widths = []
            self._ui_root_omniui_widget.style = {}

    def _on_data_shape_changed(self, model: ui.SimpleIntModel):
        if self._handler_selection._inspector._inspector_simulation:
            self._handler_selection._inspector._inspector_simulation.stop_authoring_simulation()
        self._model_category.set_flat_data_shape(
            self._model_inspector.get_data_shape_model().get_value_as_int()
        )
        self._set_treeview_properties()

    def _on_control_type_changed(self, model: ui.SimpleStringModel):
        if self._handler_selection._inspector._inspector_simulation:
            self._handler_selection._inspector._inspector_simulation.stop_authoring_simulation()
        self._model_inspector.refresh_model_structure()
        self._set_treeview_properties()

    def _on_property_type_changed(self, model: ui.SimpleStringModel):
        if self._handler_selection._inspector._inspector_simulation:
            self._handler_selection._inspector._inspector_simulation.stop_authoring_simulation()
        self._model_inspector.refresh_model_structure()
        self._set_treeview_properties()
