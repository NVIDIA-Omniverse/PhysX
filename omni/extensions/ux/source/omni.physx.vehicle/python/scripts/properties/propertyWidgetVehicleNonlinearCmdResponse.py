# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui
from pxr import Gf, Usd, UsdGeom, PhysxSchema

from ..helpers import UI
from .propertyWidgets import PropertyWidgetVehicleBase, PROPERTY_WIDGET_STYLE


class PropertyWidgetVehicleNonlinearCmdResponse(PropertyWidgetVehicleBase):

    def __init__(self, instanceToken: str):
        super().__init__(f"Nonlinear Command Response Graphs ({instanceToken})", lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI, instanceToken))
        self._instanceToken = instanceToken
        self._frame = None

    # { PropertyWidget

    def build_items(self):
        self._frame = omni.ui.Frame()
        self._draw_frame()

    # } PropertyWidget

    # { PropertyWidgetVehicleBase

    def on_hide(self):
        self._frame = []

        super().on_hide()

    def on_rebuild(self, prim: Usd.Prim):
        super().on_rebuild(prim)

        nonlinCmdResponseAPI = PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI(self._prim, self._instanceToken)

        # want to get informed if someone else changes the relevant attribute values
        self.register_change_info_path(nonlinCmdResponseAPI.GetCommandValuesAttr().GetPath(), self._on_change_info_path)
        self.register_change_info_path(nonlinCmdResponseAPI.GetSpeedResponsesPerCommandValueAttr().GetPath(), self._on_change_info_path)
        self.register_change_info_path(nonlinCmdResponseAPI.GetSpeedResponsesAttr().GetPath(), self._on_change_info_path)

    # } PropertyWidgetVehicleBase

    def _draw_frame(self):
        if (not self.is_valid()):
            return

        with self._frame:
            with omni.ui.VStack(spacing = UI.DEFAULT_WINDOW_SPACING_V, style = PROPERTY_WIDGET_STYLE):
                nonlinCmdResponseAPI = PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI(self._prim, self._instanceToken)

                commandValues = nonlinCmdResponseAPI.GetCommandValuesAttr().Get()
                speedResponsesPerCommandValue = nonlinCmdResponseAPI.GetSpeedResponsesPerCommandValueAttr().Get()
                speedResponses = nonlinCmdResponseAPI.GetSpeedResponsesAttr().Get()

                if (commandValues and speedResponsesPerCommandValue and speedResponses):
                    cmdValueCount = len(commandValues)
                    indexCount = len(speedResponsesPerCommandValue)
                    speedResponseCount = len(speedResponses)

                    if (cmdValueCount > 0):
                        for i in range(cmdValueCount):
                            if (i < indexCount):
                                cmdValue = commandValues[i]
                                startIdx = speedResponsesPerCommandValue[i]

                                if ((i + 1) < indexCount):
                                    endIdxPlus1 = speedResponsesPerCommandValue[i + 1]
                                else:
                                    endIdxPlus1 = speedResponseCount

                                if ((startIdx >= 0) and (startIdx < endIdxPlus1) and (startIdx < speedResponseCount) and (endIdxPlus1 <= speedResponseCount)):
                                    cmdValueStr = '%.3f' % cmdValue
                                    omni.ui.Label("Command Value: " + cmdValueStr, height = 0, alignment = omni.ui.Alignment.LEFT)

                                    graphFrame = omni.ui.Frame(width = omni.ui.Percent(100), height = omni.ui.Pixel(100))

                                    self._plot_response_graph(graphFrame, speedResponses, startIdx, endIdxPlus1)

                        return

                omni.ui.Label("No graphs defined", height = 0)
        
    def _plot_response_graph(self, frame, speedResponses, startIdx, endIdxPlus1):

        # adding a point at the front and back to make expected behavior clear (and also cover the single point case)
        usdContext = omni.usd.get_context()
        stage = usdContext.get_stage()
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
        lengthScale = 1.0 / metersPerUnit

        velMargin = lengthScale * 1.0

        endIdx = endIdxPlus1 - 1

        extraPointFront = Gf.Vec2f(speedResponses[startIdx][0] - velMargin, speedResponses[startIdx][1])
        extraPointBack = Gf.Vec2f(speedResponses[endIdx][0] + velMargin, speedResponses[endIdx][1])

        xTicks = []
        yTicks = []
        for idx in range(startIdx, endIdxPlus1):
            xTicks.append(speedResponses[idx][0])
            yTicks.append(speedResponses[idx][1])

        # note: the speed values are expected to be sorted
        yTicks.sort()

        speedResponsesExtended = [extraPointFront]
        speedResponsesExtended.extend(speedResponses[startIdx:endIdxPlus1])  # end+1 is expected
        speedResponsesExtended.append(extraPointBack)

        minXValString = '%.2f' % extraPointFront[0]
        maxXValString = '%.2f' % extraPointBack[0]

        UI.plot_graph(frame, speedResponsesExtended, extraPointFront[0], extraPointBack[0], 0.0, 1.0,
            minValXText = minXValString, maxValXText = maxXValString, minValYText = "0.0", maxValYText = "1.0",
            xTicks = xTicks, yTicks = yTicks)

    def _on_change_info_path(self, path):
        self._draw_frame()


class PropertyWidgetVehicleNonlinearCmdResponseDrive(PropertyWidgetVehicleNonlinearCmdResponse):
    name = "physx_vehicle_nonlinear_cmd_response_drive"

    def __init__(self):
        super().__init__(PhysxSchema.Tokens.drive)


class PropertyWidgetVehicleNonlinearCmdResponseSteer(PropertyWidgetVehicleNonlinearCmdResponse):
    name = "physx_vehicle_nonlinear_cmd_response_steer"

    def __init__(self):
        super().__init__(PhysxSchema.Tokens.steer)


class PropertyWidgetVehicleNonlinearCmdResponseBrakes0(PropertyWidgetVehicleNonlinearCmdResponse):
    name = "physx_vehicle_nonlinear_cmd_response_brakes0"

    def __init__(self):
        super().__init__(PhysxSchema.Tokens.brakes0)


class PropertyWidgetVehicleNonlinearCmdResponseBrakes1(PropertyWidgetVehicleNonlinearCmdResponse):
    name = "physx_vehicle_nonlinear_cmd_response_brakes1"

    def __init__(self):
        super().__init__(PhysxSchema.Tokens.brakes1)
