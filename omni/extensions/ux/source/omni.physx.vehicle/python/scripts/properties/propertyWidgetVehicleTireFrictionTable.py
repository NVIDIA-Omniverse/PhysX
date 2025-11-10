# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui
import omni.usd
from pxr import UsdPhysics, PhysxSchema

from omni.usd.commands.usd_commands import ChangePropertyCommand

from omni.kit.property.physx.widgets import REMOVE_BUTTON_STYLE

from ..helpers import UI
from ..commands import (
    PhysXVehicleTireFrictionTableAddEntryCommand,
    PhysXVehicleTireFrictionTableRemoveEntryCommand,
    PhysXVehicleTireFrictionTableChangeEntryCommand
)
from .propertyWidgets import PropertyWidgetVehicleBase, PROPERTY_WIDGET_STYLE


class TableEntry:
    def __init__(self, index, tireFrictionTable, propertyWidget):
        self._index = index
        self._tireFrictionTable = tireFrictionTable
        self._propertyWidget = propertyWidget
    
    def on_friction_value_changed(self, model):
        floatVal = model.get_value_as_float()

        # important to NOT use a command here. While dragging, the values should change
        # to see effects interactively but we do not want to create a command for every
        # such value. At the end of dragging, the command will be created.

        frictionValues = self._tireFrictionTable.GetFrictionValuesAttr().Get()
        frictionValues[self._index] = floatVal
        self._tireFrictionTable.GetFrictionValuesAttr().Set(frictionValues)

    def on_friction_value_begin_edit(self, model):
        floatVal = model.get_value_as_float()
        self._oldFrictionValue = floatVal

    def on_friction_value_end_edit(self, model):
        floatVal = model.get_value_as_float()
        self._propertyWidget.no_redraw_on_next_command_match()
        PhysXVehicleTireFrictionTableChangeEntryCommand.execute(self._tireFrictionTable.GetPath().pathString, 
            self._index, floatVal, self._oldFrictionValue)

    def on_entry_remove(self):
        PhysXVehicleTireFrictionTableRemoveEntryCommand.execute(self._tireFrictionTable.GetPath().pathString, self._index)


class PropertyWidgetVehicleTireFrictionTable(PropertyWidgetVehicleBase):
    name = "physx_vehicle_tire_friction_table"

    def __init__(self):
        super().__init__("Vehicle Tire Friction Table", lambda prim: prim.IsA(PhysxSchema.PhysxVehicleTireFrictionTable),
            undoCommandRedrawList = [
                PhysXVehicleTireFrictionTableAddEntryCommand.__name__,
                PhysXVehicleTireFrictionTableRemoveEntryCommand.__name__,
                PhysXVehicleTireFrictionTableChangeEntryCommand.__name__,
                ChangePropertyCommand.__name__
            ]
        )

        self._tableEntryList = None
        self._frictionTableFloatDragList = None
        self._frictionTableFloatDragSubscriptions = None
        self._frictionTableButtonList = None
        
        self._materialTableComboBox = None

        self._defaultFrictionValueFloatDrag = None

    # { PropertyWidget

    def build_items(self):
        if (not self.is_valid()):
            return

        layout = omni.ui.VStack(spacing = UI.DEFAULT_WINDOW_SPACING_V, style = PROPERTY_WIDGET_STYLE)
        with layout:

            tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(self._prim)
            groundMaterialPaths = tireFrictionTable.GetGroundMaterialsRel().GetTargets()
            frictionValues = tireFrictionTable.GetFrictionValuesAttr().Get()
            frictionValueMin = 0
            frictionValueMax = 1.5
            frictionValueStep = 0.01
            index = 0
            self._tableEntryList = []
            self._frictionTableFloatDragList = []
            self._frictionTableFloatDragSubscriptions = []
            self._frictionTableButtonList = []

            buttonWidth = 20

            for materialPath in groundMaterialPaths:
                tableEntry = TableEntry(index, tireFrictionTable, self)
                self._tableEntryList.append(tableEntry)

                with omni.ui.HStack(height = 0, spacing = 5, style = PROPERTY_WIDGET_STYLE, direction = omni.ui.Direction.RIGHT_TO_LEFT):

                    button = omni.ui.Button(style = REMOVE_BUTTON_STYLE, width = buttonWidth,
                        clicked_fn = tableEntry.on_entry_remove)
                    self._frictionTableButtonList.append(button)

                    floatDragLayout = omni.ui.HStack(height = 0, spacing = 5, style = PROPERTY_WIDGET_STYLE)
                    with floatDragLayout:

                        (floatDrag, label, floatChangeHandle) = UI.create_float_drag_with_label(floatDragLayout, frictionValues[index],
                            frictionValueMin, frictionValueMax, frictionValueStep,
                            tableEntry.on_friction_value_changed, materialPath.pathString, widthList = [omni.ui.Percent(50)])
                        self._frictionTableFloatDragList.append((floatDrag, floatChangeHandle))
                        index = index + 1

                        subs = floatDrag.model.subscribe_begin_edit_fn(tableEntry.on_friction_value_begin_edit)
                        self._frictionTableFloatDragSubscriptions.append(subs)

                        subs = floatDrag.model.subscribe_end_edit_fn(tableEntry.on_friction_value_end_edit)
                        self._frictionTableFloatDragSubscriptions.append(subs)

            with layout:
                with omni.ui.HStack(height = 0, spacing = 5, style = PROPERTY_WIDGET_STYLE, direction = omni.ui.Direction.RIGHT_TO_LEFT):

                    omni.ui.Spacer(width = omni.ui.Pixel(buttonWidth))

                    innerLayout = omni.ui.VStack(spacing = UI.DEFAULT_WINDOW_SPACING_V, style = PROPERTY_WIDGET_STYLE)
                    with innerLayout:

                        # Collect physics materials of the stage
                        stageMaterialPaths = [""]
                        materialIndex = 0
                        stage = omni.usd.get_context().get_stage()
                        if (stage):
                            for prim in stage.Traverse():
                                if prim.HasAPI(UsdPhysics.MaterialAPI):
                                    primPath = prim.GetPath().pathString
                                    stageMaterialPaths.append(primPath)

                        # materials
                        (self._materialTableComboBox, label, changeFnHandle) = UI.create_combo_box_with_label(
                            innerLayout, stageMaterialPaths, materialIndex, self._on_material_selected, "Add Physics Material", "Select a material to add.",
                            widthList = [omni.ui.Percent(50)])

                        with innerLayout:
                            omni.ui.Spacer(height = omni.ui.Pixel(10))

                        defaultFrictionValue = tireFrictionTable.GetDefaultFrictionValueAttr().Get()
                        (self._defaultFrictionValueFloatDrag, label, self._defaultFrictionValueChangeHandle) = UI.create_float_drag_with_label(
                            innerLayout, defaultFrictionValue, frictionValueMin, frictionValueMax, frictionValueStep,
                            self._on_default_friction_value_changed, "Default Friction Value", widthList = [omni.ui.Percent(50)])

                        self._defaultFrictionValueBeginEditSubs = self._defaultFrictionValueFloatDrag.model.subscribe_begin_edit_fn(self._on_default_friction_value_begin_edit)
                        self._defaultFrictionValueEndEditSubs = self._defaultFrictionValueFloatDrag.model.subscribe_end_edit_fn(self._on_default_friction_value_end_edit)

    # } PropertyWidget

    # { PropertyWidgetVehicleBase

    def on_hide(self):
        # explicit callback removal to avoid memory leaks as the UI system does not seem to clean those up properly
        self._tableEntryList = None
        if (self._frictionTableFloatDragList is not None):
            for floatDragTuple in self._frictionTableFloatDragList:
                floatDragTuple[0].model.remove_value_changed_fn(floatDragTuple[1])
            self._frictionTableFloatDragList = None
        self._frictionTableFloatDragSubscriptions = None

        if (self._frictionTableButtonList is not None):
            for button in self._frictionTableButtonList:
                button.set_clicked_fn(None)
            self._frictionTableButtonList = None

        self._materialTableComboBox = None

        if (self._defaultFrictionValueFloatDrag is not None):
            self._defaultFrictionValueFloatDrag.model.remove_value_changed_fn(self._defaultFrictionValueChangeHandle)
            self._defaultFrictionValueFloatDrag = None
            self._defaultFrictionValueChangeHandle = None
        self._defaultFrictionValueBeginEditSubs = None
        self._defaultFrictionValueEndEditSubs = None

        super().on_hide()

    # } PropertyWidgetVehicleBase

    def _find_in_friction_table(self, materialPathToFind):
        index = 0
        tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(self._prim)
        groundMaterialPaths = tireFrictionTable.GetGroundMaterialsRel().GetTargets()
        for materialPath in groundMaterialPaths:
            if (materialPath == materialPathToFind):
                return index

            index = index + 1

        return -1

    def _on_material_selected(self, model, _):
        (selectedString, selectedIndex) = UI.get_selected_string_and_index(model)
        if (selectedIndex > 0):
            foundIndex = self._find_in_friction_table(selectedString)
            if (foundIndex < 0):
                PhysXVehicleTireFrictionTableAddEntryCommand.execute(self._primPath, selectedString, 1.0)

    def _on_default_friction_value_changed(self, model):
        floatVal = model.get_value_as_float()

        # important to NOT use a command here. While dragging, the values should change
        # to see effects interactively but we do not want to create a command for every
        # such value. At the end of dragging, the command will be created.

        tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(self._prim)
        tireFrictionTable.GetDefaultFrictionValueAttr().Set(floatVal)

    def _on_default_friction_value_begin_edit(self, model):
        floatVal = model.get_value_as_float()
        self._oldDefaultFrictionValue = floatVal

    def _on_default_friction_value_end_edit(self, model):
        floatVal = model.get_value_as_float()

        tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(self._prim)

        self.no_redraw_on_next_command_match()

        omni.kit.commands.execute(ChangePropertyCommand.__name__, prop_path=tireFrictionTable.GetDefaultFrictionValueAttr().GetPath(),
            value=floatVal, prev=self._oldDefaultFrictionValue)
