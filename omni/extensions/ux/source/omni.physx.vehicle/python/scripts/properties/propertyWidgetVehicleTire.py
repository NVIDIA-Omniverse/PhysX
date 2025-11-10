# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui
from pxr import Gf, Usd, PhysxSchema
from omni.kit.builtin.commands.usd_commands import ChangePropertyCommand

from ..helpers import UI
from .propertyWidgets import PropertyWidgetVehicleBase, PROPERTY_WIDGET_STYLE


class FrictionVsSlipValueTracker:
    def __init__(self, pointIndex, axisIndex, tire, propertyWidget,
        layout, value, minValue, maxValue, title):
        self._pointIndex = pointIndex
        self._axisIndex = axisIndex
        self._tire = tire
        self._propertyWidget = propertyWidget

        (self.floatDrag, label, self._handle) = UI.create_float_drag_with_label(
            layout, value, minValue, maxValue, 0.01,
            self._on_value_changed, title, widthList = [omni.ui.Percent(50)])

        self._beginSubs = self.floatDrag.model.subscribe_begin_edit_fn(self._on_value_begin_edit)
        self._endSubs = self.floatDrag.model.subscribe_end_edit_fn(self._on_value_end_edit)

    def tear_down(self):
        # explicit callback removal to avoid memory leaks as the UI system does not seem to clean those up properly
        if (self.floatDrag is not None):
            self.floatDrag.model.remove_value_changed_fn(self._handle)
            self.floatDrag = None

        self._beginSubs = None
        self._endSubs = None

        self._propertyWidget = None
        self._tire = None

    def set_value(self, frictionVsSlipValues):
        if (self.floatDrag is not None):
            self.floatDrag.model.set_value(frictionVsSlipValues[self._pointIndex][self._axisIndex])
    
    def _on_value_changed(self, model):
        floatVal = model.get_value_as_float()
        attr = self._tire.GetFrictionVsSlipGraphAttr()
        frictionVsSlipValues = attr.Get()

        # note: the copy seems necessary. Re-using the provided vec2fArray does not work
        frictionVsSlipValueList = []
        for v in frictionVsSlipValues:
            frictionVsSlipValueList.append(Gf.Vec2f(v))
        frictionVsSlipValueList[self._pointIndex][self._axisIndex] = floatVal

        if (self._axisIndex == 1):
            self._propertyWidget.adjust_min_max(floatVal, self._pointIndex)

        self._propertyWidget.plot_friction_vs_slip_graph(frictionVsSlipValueList)

        # important to NOT use a command here. While dragging, the values should change
        # to see effects interactively but we do not want to create a command for every
        # such value. At the end of dragging, the command will be created.

        attr.Set(frictionVsSlipValueList)

    def _on_value_begin_edit(self, model):
        self._propertyWidget.ignore_change_info_path(True)
        floatVal = model.get_value_as_float()
        self._oldFloatValue = floatVal

    def _on_value_end_edit(self, model):
        floatVal = model.get_value_as_float()

        attr = self._tire.GetFrictionVsSlipGraphAttr()
        attrPath = attr.GetPath().pathString
        frictionVsSlipValues = attr.Get()
        frictionVsSlipValueList = []
        newFrictionVsSlipValueList = []
        for v in frictionVsSlipValues:
            frictionVsSlipValueList.append(Gf.Vec2f(v))
            newFrictionVsSlipValueList.append(Gf.Vec2f(v))

        frictionVsSlipValueList[self._pointIndex][self._axisIndex] = self._oldFloatValue
        newFrictionVsSlipValueList[self._pointIndex][self._axisIndex] = floatVal

        omni.kit.commands.execute(ChangePropertyCommand.__name__, prop_path = attrPath, value = newFrictionVsSlipValueList,
            prev = frictionVsSlipValueList)

        self._propertyWidget.ignore_change_info_path(False)


class PropertyWidgetVehicleTire(PropertyWidgetVehicleBase):
    name = "physx_vehicle_tire"

    def __init__(self):
        super().__init__("Vehicle Tire Friction vs. Longitudinal Slip", lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleTireAPI))
        self._valueTrackers = []
        self._ignore_change_info_path = False
        self._graphFrame = None
        self._maxFrictionValue = 1.5
        self._maxFrictionValueString = "1.5"

    # { PropertyWidget

    def build_items(self):
        if (not self.is_valid()):
            return

        layout = omni.ui.VStack(spacing = UI.DEFAULT_WINDOW_SPACING_V, style = PROPERTY_WIDGET_STYLE)
        with layout:
            tire = PhysxSchema.PhysxVehicleTireAPI(self._prim)
            frictionVsSlipValues = tire.GetFrictionVsSlipGraphAttr().Get()
            if (len(frictionVsSlipValues) == 3):

                epsilon = 2e-07

                self._graphFrame = omni.ui.Frame(width = omni.ui.Percent(100), height = omni.ui.Pixel(100))
                self.plot_friction_vs_slip_graph(frictionVsSlipValues)

                self._valueTrackers.append(
                    FrictionVsSlipValueTracker(0, 1, tire, self, layout, frictionVsSlipValues[0][1],
                        epsilon, frictionVsSlipValues[1][1], "Friction at Zero Slip"))

                self._valueTrackers.append(
                    FrictionVsSlipValueTracker(1, 1, tire, self, layout, frictionVsSlipValues[1][1],
                        max(frictionVsSlipValues[0][1], frictionVsSlipValues[2][1]), self._maxFrictionValue, "Max Friction"))

                self._valueTrackers.append(
                    FrictionVsSlipValueTracker(1, 0, tire, self, layout, frictionVsSlipValues[1][0],
                        epsilon, 1.0 - epsilon, "Slip at Max Friction"))

                self._valueTrackers.append(
                    FrictionVsSlipValueTracker(2, 1, tire, self, layout, frictionVsSlipValues[2][1],
                        0, frictionVsSlipValues[1][1], "Friction at Slip 1 or Higher"))

    # } PropertyWidget

    # { PropertyWidgetVehicleBase

    def on_hide(self):
        for valTracker in self._valueTrackers:
            valTracker.tear_down()

        self._valueTrackers = []

        self._graphFrame = None

        super().on_hide()

    def on_rebuild(self, prim: Usd.Prim):
        super().on_rebuild(prim)

        tire = PhysxSchema.PhysxVehicleTireAPI(self._prim)

        # want to get informed if someone else changes the frictionVsSlip values
        self.register_change_info_path(tire.GetFrictionVsSlipGraphAttr().GetPath(), self._on_change_info_path)

    # } PropertyWidgetVehicleBase

    def plot_friction_vs_slip_graph(self, frictionVsSlipValues):

        UI.plot_graph(self._graphFrame, frictionVsSlipValues, 0.0, 1.0, 0.0, self._maxFrictionValue,
            minValXText = "0.0", maxValXText = "1.0", minValYText = "0.0", maxValYText = self._maxFrictionValueString,
            xTicks = [frictionVsSlipValues[1][0]], yTicks = [frictionVsSlipValues[1][1]])

    def adjust_min_max(self, value, pointIndex):
        if (pointIndex == 1):
            self._valueTrackers[0].floatDrag.max = value
            self._valueTrackers[0].floatDrag.model.max = value
            self._valueTrackers[3].floatDrag.max = value
            self._valueTrackers[3].floatDrag.model.max = value
        elif (pointIndex == 0):
            limit = max(value, self._valueTrackers[3].floatDrag.model.get_value_as_float())
            self._valueTrackers[1].floatDrag.min = limit
            self._valueTrackers[1].floatDrag.model.min = limit
        else:
            limit = max(value, self._valueTrackers[0].floatDrag.model.get_value_as_float())
            self._valueTrackers[1].floatDrag.min = limit
            self._valueTrackers[1].floatDrag.model.min = limit

    def ignore_change_info_path(self, ignore: bool):
        self._ignore_change_info_path = ignore

    def _on_change_info_path(self, path):
        if not self._ignore_change_info_path:
            if (len(self._valueTrackers) > 0):
                tire = PhysxSchema.PhysxVehicleTireAPI(self._prim)
                frictionVsSlipValues = tire.GetFrictionVsSlipGraphAttr().Get()

                self.plot_friction_vs_slip_graph(frictionVsSlipValues)

                self._valueTrackers[0].floatDrag.max = frictionVsSlipValues[1][1]
                self._valueTrackers[0].floatDrag.model.max = frictionVsSlipValues[1][1]
                self._valueTrackers[3].floatDrag.max = frictionVsSlipValues[1][1]
                self._valueTrackers[3].floatDrag.model.max = frictionVsSlipValues[1][1]
                limit = max(frictionVsSlipValues[0][1], frictionVsSlipValues[2][1])
                self._valueTrackers[1].floatDrag.min = limit
                self._valueTrackers[1].floatDrag.model.min = limit

                for valTracker in self._valueTrackers:
                    valTracker.set_value(frictionVsSlipValues)
