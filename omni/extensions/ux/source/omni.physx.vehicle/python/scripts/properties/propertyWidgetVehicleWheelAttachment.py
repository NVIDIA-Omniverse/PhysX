# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui
from pxr import Usd, PhysxSchema

from ..helpers import UI
from ..commands import (
    PhysXVehicleCommandResponseChangeEntryCommand,
    PhysXVehicleDifferentialChangeEntryCommand,
    PhysXVehicleSuspensionFrameTransformsAutocomputeCommand
)
from .propertyWidgets import (
    PropertyWidgetVehicleBase,
    create_suspension_frame_transforms_autocompute_ui,
    PROPERTY_WIDGET_STYLE
)


def getWheelListIndex(wheelIndexArrayAttribute, wheelIndex):
    wheelListIndex = -1  # larger or equal 0 if wheel is in the index list
    wheelIndexCount = 0

    if (wheelIndexArrayAttribute.HasValue()):
        wheelIndices = wheelIndexArrayAttribute.Get()
        wheelIndexCount = len(wheelIndices)
        idx = 0
        for wIdx in wheelIndices:
            if (wIdx == wheelIndex):
                wheelListIndex = idx
                break

            idx = idx + 1

    return (wheelListIndex, wheelIndexCount)


#
# UI and logic for configuring parts of the steer/brake response setup
#
class VehicleWheelCommandResponseSetup:
    def __init__(self, valueArrayAttribute, wheelIndexArrayAttribute, wheelIndex, wheelCount,
        defaultValue, valueLabelText, valueLabelTooltip, minValue, maxValue,
        propertyWidget):
        # valueArrayAttribute: USD attribute holding an array of float values
        # wheelIndexArrayAttribute: USD attribute holding an array of wheel indices
        # wheelIndex: the wheel index of the wheel attachment
        # wheelCount: the number of wheels of the vehicle
        # defaultValue: the value to show when the value array is empty
        # valueLabelText: string to use for the label of the value UI widget
        # valueLabelTooltip: string to use for the tooltip of the label
        # minValue: the minimum value for entries in the value array
        # maxValue: the maximum value for entries in the value array
        # propertyWidget: the property widget managing the authoring of the entry

        self._valueArrayAttribute = valueArrayAttribute
        self._wheelIndexArrayAttribute = wheelIndexArrayAttribute
        self._wheelIndex = wheelIndex
        self._wheelCount = wheelCount
        self._defaultValue = defaultValue
        self._valueLabelText = valueLabelText
        self._valueLabelTooltip = valueLabelTooltip
        self._minValue = minValue
        self._maxValue = maxValue
        self._propertyWidget = propertyWidget

        self._valueFloatDrag = None
        self._valueLabel = None
        self._valueChangeHandle = None
        self._valueBeginEditSub = None
        self._valueEndEditSub = None

        self._valueChanged = False

    def build(self, layout):
        (self._wheelListIndex, wheelIndexCount) = getWheelListIndex(self._wheelIndexArrayAttribute, self._wheelIndex)

        isWheelInList = (self._wheelListIndex >= 0)

        with layout:
            hasValidWheelIndex = (self._wheelIndex >= 0) and (self._wheelIndex < self._wheelCount)

            if (isWheelInList):
                value = self._defaultValue
                if (self._valueArrayAttribute.HasAuthoredValue()):
                    values = self._valueArrayAttribute.Get()
                    if (self._wheelListIndex < len(values)):
                        value = values[self._wheelListIndex]
            elif (hasValidWheelIndex and (wheelIndexCount == 0)):
                # if no wheel is defined, all wheels are affected by the command
                value = self._defaultValue
            else:
                value = 0.0

            if (hasValidWheelIndex):
                valueLabeTooltip = self._valueLabelTooltip
            else:
                valueLabeTooltip = ("To enable this feature, the wheel index of the wheel attachment "
                    "has to be set to a value that is unique among the wheels of a vehicle and needs to be in the range "
                    "[0, number_of_wheels-1]")

            (self._valueFloatDrag, self._valueLabel, self._valueChangeHandle) = UI.create_float_drag_with_label(
                layout, value, self._minValue, self._maxValue, 0.01,
                self._on_value_changed, self._valueLabelText,
                valueLabeTooltip, tooltipWidth = 200, widthList = [omni.ui.Percent(50)])

            if (not hasValidWheelIndex):
                self._valueFloatDrag.enabled = False
                self._valueLabel.enabled = False

            self._valueBeginEditSub = self._valueFloatDrag.model.subscribe_begin_edit_fn(
                self._on_value_begin_edit)

            self._valueEndEditSub = self._valueFloatDrag.model.subscribe_end_edit_fn(
                self._on_value_end_edit)

    def cleanup(self):
        # explicit callback removal to avoid memory leaks as the UI system does not seem to clean those up properly

        if (self._valueFloatDrag is not None):
            self._valueFloatDrag.model.remove_value_changed_fn(self._valueChangeHandle)
            self._valueBeginEditSub = None
            self._valueEndEditSub = None
            self._valueFloatDrag = None

        self._valueLabel = None

    def _on_value_changed(self, model):
        # important to NOT use a command here. While dragging, the values should change
        # to see effects interactively but we do not want to create a command for every
        # such value. At the end of dragging, the command will be created.

        self._valueChanged = True

        if (self._valueArrayAttribute and self._valueArrayAttribute.HasValue()):
            values = self._valueArrayAttribute.Get()
            valueCount = len(values)

            # note: somewhat questionable as it will only work with a value array
            #       that contains an entry for this wheel
            if ((self._wheelListIndex >= 0) and (self._wheelListIndex < valueCount)):
                floatVal = model.get_value_as_float()
                values[self._wheelListIndex] = floatVal
                self._valueArrayAttribute.Set(values)

    def _on_value_begin_edit(self, model):
        self._propertyWidget.ignore_change_info_path(True)

        floatVal = model.get_value_as_float()
        self._oldValue = floatVal
        self._valueChanged = False

    def _on_value_end_edit(self, model):
        if (self._valueChanged):
            # only do work if there was a change registered. Double-clicking on a float drag
            # can sometimes trigger a being/end sequence only

            floatVal = model.get_value_as_float()

            (success, newWheelListIndex) = PhysXVehicleCommandResponseChangeEntryCommand.execute(
                self._valueArrayAttribute.GetPath().pathString,
                self._wheelIndexArrayAttribute.GetPath().pathString,
                self._wheelListIndex, floatVal, self._oldValue, self._wheelIndex,
                self._defaultValue, self._wheelCount)

            if (success):
                self._wheelListIndex = newWheelListIndex

        self._propertyWidget.ignore_change_info_path(False)


#
# Manager for adding state to float drag callbacks
#
class FlatDragChangeCallbacks:
    def __init__(self, owner, index):
        self._owner = owner
        self._index = index

    def _on_value_changed(self, model):
        self._owner._on_value_changed(self._index, model)

    def _on_value_begin_edit(self, model):
        self._owner._on_value_begin_edit(self._index, model)

    def _on_value_end_edit(self, model):
        self._owner._on_value_end_edit(self._index, model)


#
# UI and logic for configuring parts of the multi wheel differential setup
#
class VehicleWheelDifferentialSetup:
    def __init__(self, valueArrayAttributes, wheelIndexArrayAttribute, wheelIndex,
        valueLabelTexts, valueLabelTooltips, minValues, maxValues,
        propertyWidget):
        # valueArrayAttributes: List of USD attributes each holding an array of float values
        # wheelIndexArrayAttribute: USD attribute holding an array of wheel indices
        # wheelIndex: the wheel index of the wheel attachment
        # valueLabelTexts: list of strings to use for the label of the value UI widgets
        # valueLabelTooltips: list of strings to use for the tooltip of the labels
        # minValues: list of minimum values for entries in the value arrays
        # maxValues: list of maximum values for entries in the value arrays
        # propertyWidget: the property widget managing the authoring of the entry

        self._valueArrayAttributes = valueArrayAttributes
        self._wheelIndexArrayAttribute = wheelIndexArrayAttribute
        self._wheelIndex = wheelIndex
        self._valueLabelTexts = valueLabelTexts
        self._valueLabelTooltips = valueLabelTooltips
        self._minValues = minValues
        self._maxValues = maxValues
        self._propertyWidget = propertyWidget

        self._valueFloatDrags = []
        self._valueLabels = []
        self._valueChangeHandles = []
        self._valueBeginEditSubs = []
        self._valueEndEditSubs = []
        self._valueFloatDragCallbacks = []

        self._oldValues = []
        self._valueChangedList = []

    def build(self, layout):
        (self._wheelListIndex, wheelIndexCount) = getWheelListIndex(self._wheelIndexArrayAttribute, self._wheelIndex)

        isWheelInList = (self._wheelListIndex >= 0)

        with layout:
            hasValidWheelIndex = (self._wheelIndex >= 0)

            valueArrayAttrCount = len(self._valueArrayAttributes)
            self._oldValues = [0.0] * valueArrayAttrCount
            self._valueChangedList = [False] * valueArrayAttrCount

            for i in range(valueArrayAttrCount):
                valueArrayAttr = self._valueArrayAttributes[i]

                if (isWheelInList):
                    value = 1.0 / wheelIndexCount
                    if (valueArrayAttr.HasAuthoredValue()):
                        values = valueArrayAttr.Get()
                        if (self._wheelListIndex < len(values)):
                            value = values[self._wheelListIndex]
                else:
                    value = 0.0

                if (hasValidWheelIndex):
                    valueLabeTooltip = self._valueLabelTooltips[i]
                else:
                    valueLabeTooltip = ("To enable this feature, the wheel index of the wheel attachment "
                        "has to be set to a value that is unique among the wheels of a vehicle and needs to be in the range "
                        "[0, number_of_wheels-1]")

                valueFloatDragCallbacks = FlatDragChangeCallbacks(self, i)
                self._valueFloatDragCallbacks.append(valueFloatDragCallbacks)

                (valueFloatDrag, valueLabel, valueChangeHandle) = UI.create_float_drag_with_label(
                    layout, value, self._minValues[i], self._maxValues[i], 0.01,
                    valueFloatDragCallbacks._on_value_changed, self._valueLabelTexts[i],
                    valueLabeTooltip, tooltipWidth = 200, widthList = [omni.ui.Percent(50)])

                self._valueFloatDrags.append(valueFloatDrag)
                self._valueLabels.append(valueLabel)
                self._valueChangeHandles.append(valueChangeHandle)

                if (not hasValidWheelIndex):
                    valueFloatDrag.enabled = False
                    valueLabel.enabled = False

                valueBeginEditSub = valueFloatDrag.model.subscribe_begin_edit_fn(
                    valueFloatDragCallbacks._on_value_begin_edit)
                self._valueBeginEditSubs.append(valueBeginEditSub)

                valueEndEditSub = valueFloatDrag.model.subscribe_end_edit_fn(
                    valueFloatDragCallbacks._on_value_end_edit)
                self._valueEndEditSubs.append(valueEndEditSub)

    def cleanup(self):
        # explicit callback removal to avoid memory leaks as the UI system does not seem to clean those up properly

        for i in range(len(self._valueFloatDrags)):
            self._valueFloatDrags[i].model.remove_value_changed_fn(self._valueChangeHandles[i])

        self._valueFloatDrags.clear()
        self._valueChangeHandles.clear()
        self._valueBeginEditSubs.clear()
        self._valueEndEditSubs.clear()
        self._valueLabels.clear()

        self._valueFloatDragCallbacks.clear()

        self._oldValues.clear()
        self._valueChangedList.clear()

    def _on_value_changed(self, index, model):
        # important to NOT use a command here. While dragging, the values should change
        # to see effects interactively but we do not want to create a command for every
        # such value. At the end of dragging, the command will be created.

        self._valueChangedList[index] = True

        valueArrayAttr = self._valueArrayAttributes[index]
        if (valueArrayAttr and valueArrayAttr.HasValue()):
            values = valueArrayAttr.Get()
            valueCount = len(values)

            # note: somewhat questionable as it will only work with a value array
            #       that contains an entry for this wheel
            if ((self._wheelListIndex >= 0) and (self._wheelListIndex < valueCount)):
                floatVal = model.get_value_as_float()
                values[self._wheelListIndex] = floatVal
                valueArrayAttr.Set(values)

    def _on_value_begin_edit(self, index, model):
        self._propertyWidget.ignore_change_info_path(True)

        floatVal = model.get_value_as_float()
        self._oldValues[index] = floatVal
        self._valueChangedList[index] = False

    def _on_value_end_edit(self, index, model):
        if (self._valueChangedList[index]):
            # only do work if there was a change registered. Double-clicking on a float drag
            # can sometimes trigger a being/end sequence only

            floatVal = model.get_value_as_float()

            valueArrayAttrPaths = []
            for attr in self._valueArrayAttributes:
                valueArrayAttrPaths.append(attr.GetPath().pathString)

            (success, newWheelListIndex) = PhysXVehicleDifferentialChangeEntryCommand.execute(
                valueArrayAttrPaths, index,
                self._wheelIndexArrayAttribute.GetPath().pathString,
                self._wheelListIndex, floatVal, self._oldValues[index], self._wheelIndex)

            if (success):
                self._wheelListIndex = newWheelListIndex

                # note: since entry removal will trigger a frame rebuild, there is no need
                #       to add special logic to ensure the entries of other value arrays show
                #       0 in the UI (for empty value arrays it would show a non-zero value)

        self._propertyWidget.ignore_change_info_path(False)


def findParentWithAPI(prim, api):
    vehiclePrim = prim.GetParent()
    pseudoRoot = prim.GetStage().GetPseudoRoot()
    while (vehiclePrim != pseudoRoot):
        if (vehiclePrim.HasAPI(api)):
            return vehiclePrim
        else:
            vehiclePrim = vehiclePrim.GetParent()

    return None


class PropertyWidgetVehicleWheelAttachmentAuthoring(PropertyWidgetVehicleBase):
    name = "physx_vehicle_wheel_attachment_authoring"

    def __init__(self):
        super().__init__("Vehicle Wheel Authoring Helpers", lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleWheelAttachmentAPI))

        self._vehiclePrim = None
        self._wheelIndex = -1

        self._frame = None

        self._steerCommandResponseEntry = None
        self._brakeCommandResponseEntries = []
        self._multiWheelDiffEntry = None

        # tracker to register attribute change callbacks only when the widget is shown.
        # Deregister/register as part of a forced rebuild can cause issues because
        # the callback logic does not seem to handle the cause of adding/removing
        # callbacks while a callback fires (at least not in combination with delayed
        # frame rebuild)
        self._changeInfoPathRegistrationComplete = False

        self._suspensionFrameTransformButton = None

    def _build_steering_items(self, steeringAPI, wheelCount, layout):
        labelText = "Steer Angle Multiplier"
        labelTooltipText = ("Multiplier that gets applied to maxSteerAngle of the PhysxVehicleSteeringAPI "
            "API schema to define the maximum steer angle for this wheel")

        wheelIndicesAttr = steeringAPI.GetWheelsAttr()
        angleMultipliersAttr = steeringAPI.GetAngleMultipliersAttr()

        if (not self._changeInfoPathRegistrationComplete):
            self.register_change_info_path(wheelIndicesAttr.GetPath(), self._on_change_info_path)
            self.register_change_info_path(angleMultipliersAttr.GetPath(), self._on_change_info_path)

        self._steerCommandResponseEntry = VehicleWheelCommandResponseSetup(angleMultipliersAttr, wheelIndicesAttr,
            self._wheelIndex, wheelCount, 1.0, labelText, labelTooltipText, -1.0, 1.0, self)

        self._steerCommandResponseEntry.build(layout)

        with layout:
            omni.ui.Line(alignment = omni.ui.Alignment.CENTER, width = omni.ui.Percent(100));

    def _build_brakes_items(self, brakesAPIList, brakesIndices, wheelCount, layout):
        for i in range(len(brakesAPIList)):
            brakesIndex = brakesIndices[i]
            brakesAPI = brakesAPIList[i]

            labelText = f"Brake{brakesIndex} Torque Multiplier"
            labelTooltipText = ("Multiplier that gets applied to maxBrakeTorque of the PhysxVehicleBrakesAPI "
                "API schema to define the maximum brake torque for this wheel")

            wheelIndicesAttr = brakesAPI.GetWheelsAttr()
            torqueMultipliersAttr = brakesAPI.GetTorqueMultipliersAttr()

            if (not self._changeInfoPathRegistrationComplete):
                self.register_change_info_path(wheelIndicesAttr.GetPath(), self._on_change_info_path)
                self.register_change_info_path(torqueMultipliersAttr.GetPath(), self._on_change_info_path)

            brakeCommandResponseEntry = VehicleWheelCommandResponseSetup(torqueMultipliersAttr, wheelIndicesAttr,
                self._wheelIndex, wheelCount, 1.0, labelText, labelTooltipText, 0.0, 1.0, self)

            brakeCommandResponseEntry.build(layout)

            self._brakeCommandResponseEntries.append(brakeCommandResponseEntry)

        with layout:
            omni.ui.Line(alignment = omni.ui.Alignment.CENTER, width = omni.ui.Percent(100));

    def _build_differential_items(self, diffAPI, isStandardDrive, layout):
        labelTextList = ["Drive Torque Ratio"]
        labelTooltipTextList = ["Ratio defining how much of the vehicle's drive torque will be delivered to this wheel"]

        wheelIndicesAttr = diffAPI.GetWheelsAttr()
        torqueRatiosAttr = diffAPI.GetTorqueRatiosAttr()

        if (not self._changeInfoPathRegistrationComplete):
            self.register_change_info_path(wheelIndicesAttr.GetPath(), self._on_change_info_path)
            self.register_change_info_path(torqueRatiosAttr.GetPath(), self._on_change_info_path)

        valueArrayAttributes = [torqueRatiosAttr]
        minValues = [-1.0]
        maxValues = [1.0]

        if (isStandardDrive):
            averageWheelSpeedRatiosAttr = diffAPI.GetAverageWheelSpeedRatiosAttr()
            valueArrayAttributes.append(averageWheelSpeedRatiosAttr)
            minValues.append(0.0)
            maxValues.append(1.0)

            if (not self._changeInfoPathRegistrationComplete):
                self.register_change_info_path(averageWheelSpeedRatiosAttr.GetPath(), self._on_change_info_path)

            labelTextList.append("Average Wheel Speed Ratio")
            labelTooltipTextList.append("Ratio defining the contribution of this wheel when computing the average wheel speed")

        self._multiWheelDiffEntry = VehicleWheelDifferentialSetup(valueArrayAttributes, wheelIndicesAttr,
            self._wheelIndex, labelTextList, labelTooltipTextList, minValues, maxValues, self)

        self._multiWheelDiffEntry.build(layout)

        with layout:
            omni.ui.Line(alignment = omni.ui.Alignment.CENTER, width = omni.ui.Percent(100));

    def build_frame(self):
        self.ignore_change_info_path(False)

        with self._frame:

            layout = omni.ui.VStack(spacing = UI.DEFAULT_WINDOW_SPACING_V, style = PROPERTY_WIDGET_STYLE)

            wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(self._prim)
            self._wheelIndex = wheelAttAPI.GetIndexAttr().Get()

            self._vehiclePrim = findParentWithAPI(self._prim, PhysxSchema.PhysxVehicleAPI)
            if (self._vehiclePrim is not None):
                wheelCount = 0
                for p in Usd.PrimRange(self._vehiclePrim):  # there is no GetDescendants()
                    if p.HasAPI(PhysxSchema.PhysxVehicleWheelAttachmentAPI):
                        wheelCount = wheelCount + 1

                # Helper to configure the wheel as a steerable wheel
                if (self._vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleSteeringAPI)):
                    self._build_steering_items(PhysxSchema.PhysxVehicleSteeringAPI(self._vehiclePrim), wheelCount, layout)

                # Helper to configure the wheel for brake 0/1
                brakeAPIList = []
                brakeIndices = []
                if (self._vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleBrakesAPI, PhysxSchema.Tokens.brakes0)):
                    brakeAPIList.append(PhysxSchema.PhysxVehicleBrakesAPI(self._vehiclePrim, PhysxSchema.Tokens.brakes0))
                    brakeIndices.append(0)

                if (self._vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleBrakesAPI, PhysxSchema.Tokens.brakes1)):
                    brakeAPIList.append(PhysxSchema.PhysxVehicleBrakesAPI(self._vehiclePrim, PhysxSchema.Tokens.brakes1))
                    brakeIndices.append(1)

                if (brakeAPIList):
                    self._build_brakes_items(brakeAPIList, brakeIndices, wheelCount, layout)

                # Helper to configure some wheel differential parameters
                if (self._vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI)):
                    isStandardDrive = False
                    if (self._vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleDriveStandardAPI)):
                        isStandardDrive = True
                    else:
                        vehicleAPI = PhysxSchema.PhysxVehicleAPI(self._vehiclePrim)
                        driveRel = vehicleAPI.GetDriveRel()
                        if (driveRel.HasAuthoredTargets()):
                            targets = driveRel.GetTargets()
                            if (len(targets) > 0):
                                drivePath = targets[0]
                                drivePrim = self._vehiclePrim.GetStage().GetPrimAtPath(drivePath)
                                if (drivePrim and drivePrim.HasAPI(PhysxSchema.PhysxVehicleDriveStandardAPI)):
                                    isStandardDrive = True

                    self._build_differential_items(PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI(self._vehiclePrim), isStandardDrive, layout)

            # Helper to compute some wheel simulation transformations based on the USD prim transformations and the vehicle center
            # of mass

            self._suspensionFrameTransformButton = create_suspension_frame_transforms_autocompute_ui(layout,
                self._on_compute_suspension_frame_transforms_pressed)

        if (not self._changeInfoPathRegistrationComplete):
            self._changeInfoPathRegistrationComplete = True

    def clear_frame(self):
        if (self._steerCommandResponseEntry is not None):
            self._steerCommandResponseEntry.cleanup()
            self._steerCommandResponseEntry = None

        for brakeCommandResponseEntry in self._brakeCommandResponseEntries:
            brakeCommandResponseEntry.cleanup()
        self._brakeCommandResponseEntries.clear()

        if (self._multiWheelDiffEntry is not None):
            self._multiWheelDiffEntry.cleanup()
            self._multiWheelDiffEntry = None

        # explicit callback removal to avoid memory leaks as the UI system does not seem to clean those up properly

        if (self._suspensionFrameTransformButton is not None):
            self._suspensionFrameTransformButton.set_clicked_fn(None)
            self._suspensionFrameTransformButton = None

    def schedule_for_rebuild(self):
        self.clear_frame()
        # scheduling for rebuild (see comment where set_build_fn is used)
        self._frame.rebuild()

    def ignore_change_info_path(self, ignore: bool):
        # a rebuild should be avoided, for example, if a float drag is in operation
        self._ignore_change_info_path = ignore

    def _on_change_info_path(self, path):
        # callback to get informed about relevant attributes changing (for example through an undo/redo).
        # This will trigger a rebuild unless explicitly disabled. Note that a rebuild can happen even if
        # the user just operates through the UI because some operations may cause multiple watched USD
        # attributes to change or change callbacks might fire with a delay etc.

        if (not self._ignore_change_info_path):
            self.schedule_for_rebuild()

    # { PropertyWidget

    def clean(self):
        super().clean()

    def build_items(self):
        if (not self.is_valid()):
            return

        self._frame = omni.ui.Frame(width = omni.ui.Percent(100), height = omni.ui.Pixel(100))
        # using a build callback to avoid the frame being rebuilt multiple times when multiple
        # watched USD attributes change as part of a single operation. This way, the rebuild
        # happens delayed (as part of the general UI update loop I assume)
        self._frame.set_build_fn(self.build_frame)

    # } PropertyWidget

    # { PropertyWidgetVehicleBase

    def on_hide(self):

        self.clear_frame()

        self._frame = None

        self._changeInfoPathRegistrationComplete = False

        super().on_hide()

    # } PropertyWidgetVehicleBase

    def _on_compute_suspension_frame_transforms_pressed(self):
        PhysXVehicleSuspensionFrameTransformsAutocomputeCommand.execute(self._primPath)
