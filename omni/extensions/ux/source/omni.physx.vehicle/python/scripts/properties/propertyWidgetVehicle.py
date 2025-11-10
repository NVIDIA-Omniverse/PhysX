# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui
import omni.usd
from pxr import PhysxSchema

from ..helpers import UI
from ..commands import (
    PhysXVehicleControllerEnableInputCommand,
    PhysXVehicleControllerEnableMouseCommand,
    PhysXVehicleControllerEnableAutoReverseCommand,
    PhysXVehicleControllerSetSteeringSensitivityCommand,
    PhysXVehicleControllerSetSteeringFilterTimeCommand,
    PhysXVehicleSuspensionFrameTransformsAutocomputeCommand
)
from .propertyWidgets import (
    PropertyWidgetVehicleBase,
    create_suspension_frame_transforms_autocompute_ui,
    PROPERTY_WIDGET_STYLE
)


DEFAULT_LABEL_TOOLTIP_WIDTH = 300


class PropertyWidgetVehicleControllerSettings(PropertyWidgetVehicleBase):
    name = "physx_vehicle_controller_settings"

    def __init__(self, physxVehicleInterface):
        super().__init__("Vehicle Controller Settings",
            lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleControllerAPI) and (not prim.HasAPI(PhysxSchema.PhysxVehicleTankControllerAPI)),
            undoCommandRedrawList = [
                PhysXVehicleControllerEnableInputCommand.__name__,
                PhysXVehicleControllerEnableMouseCommand.__name__,
                PhysXVehicleControllerEnableAutoReverseCommand.__name__,
                PhysXVehicleControllerSetSteeringSensitivityCommand.__name__,
                PhysXVehicleControllerSetSteeringFilterTimeCommand.__name__
            ]
        )
        self._physxVehicleInterface = physxVehicleInterface
        self._inputEnabledCheckBox = None
        self._inputEnabledHandle = None
        self._mouseEnabledCheckBox = None
        self._mouseEnabledLabel = None
        self._mouseEnabledHandle = None
        self._autoReverseEnabledCheckBox = None
        self._autoReverseEnabledHandle = None
        
        self._steeringSensitivityFloatDrag = None
        self._steeringSensitivityHandle = None
        self._steeringFilterTimeFloatDrag = None
        self._steeringFilterTimeHandle = None

    # { PropertyWidget

    def clean(self):
        self._physxVehicleInterface = None
        super().clean()

    def build_items(self):
        if (not self.is_valid()):
            return

        layout = omni.ui.VStack(spacing = UI.DEFAULT_WINDOW_SPACING_V, style = PROPERTY_WIDGET_STYLE)
        with layout:
            isInputEnabled = self._get_input_enabled()
            (self._inputEnabledCheckBox, label, self._inputEnabledHandle) = UI.create_checkbox_with_label(
                layout, isInputEnabled, self._toggle_input_enabled, "Input Enabled", 
                widthList = [omni.ui.Percent(50)])

            (self._mouseEnabledCheckBox, self._mouseEnabledLabel, self._mouseEnabledHandle) = UI.create_checkbox_with_label(
                layout, self._get_mouse_enabled(), self._toggle_mouse_enabled, "Mouse Enabled", 
                widthList = [omni.ui.Percent(50)])
            self._mouseEnabledCheckBox.enabled = isInputEnabled
            self._mouseEnabledLabel.enabled = isInputEnabled

            (self._autoReverseEnabledCheckBox, label, self._autoReverseEnabledHandle) = UI.create_checkbox_with_label(
                layout, self._get_auto_reverse_enabled(), self._toggle_auto_reverse_enabled, "Auto Reverse Enabled",
                widthList = [omni.ui.Percent(50)])

            (self._steeringSensitivityFloatDrag, label, self._steeringSensitivityHandle) = UI.create_float_drag_with_label(
                layout, self._get_steering_sensitivity(), 
                1.0, 10.0, 0.1,
                self._on_set_steering_sensitivity, "Steering Sensitivity", 
                tooltipText = ("Set the vehicle's steering sensitivity when using the gamepad. Must be 1.0 or greater. "
                "A setting of 1.0 generates a linear response between gamepad input and steering output. As the "
                "sensitivity is increased, the same gamepad input will generate a smaller steering output. However, "
                "regardless of the sensitivity, a full gamepad deflection will always generate a full steering deflection. "), 
                tooltipWidth = DEFAULT_LABEL_TOOLTIP_WIDTH,
                widthList=[omni.ui.Percent(50)])

            (self._steeringFilterTimeFloatDrag, label, self._steeringFilterTimeHandle) = UI.create_float_drag_with_label(
                layout, self._get_steering_filter_time(), 
                0.0, 10.0, 0.01,
                self._on_set_steering_filter_time, "Steering Filter Time",
                tooltipText = ("Set the vehicle's steering filter time constant which must 0.0 or greater. "
                "A setting of 0.0 turns the filter off. The higher the value the longer it takes for the steering to "
                "reach the desired angle."),  
                tooltipWidth = DEFAULT_LABEL_TOOLTIP_WIDTH,
                widthList=[omni.ui.Percent(50)])

    # } PropertyWidget

    # { PropertyWidgetVehicleBase

    def on_hide(self):
        # explicit callback removal to avoid memory leaks as the UI system does not seem to clean those up properly
        if (self._inputEnabledCheckBox is not None):
            self._inputEnabledCheckBox.model.remove_value_changed_fn(self._inputEnabledHandle)
            self._inputEnabledCheckBox = None

        if (self._mouseEnabledCheckBox is not None):
            self._mouseEnabledCheckBox.model.remove_value_changed_fn(self._mouseEnabledHandle)
            self._mouseEnabledCheckBox = None
        self._mouseEnabledLabel = None

        if (self._autoReverseEnabledCheckBox is not None):
            self._autoReverseEnabledCheckBox.model.remove_value_changed_fn(self._autoReverseEnabledHandle)
            self._autoReverseEnabledCheckBox = None

        if (self._steeringSensitivityFloatDrag is not None):
            self._steeringSensitivityFloatDrag.model.remove_value_changed_fn(self._steeringSensitivityHandle)
            self._steeringSensitivityFloatDrag = None
            
        if (self._steeringFilterTimeFloatDrag is not None):
            self._steeringFilterTimeFloatDrag.model.remove_value_changed_fn(self._steeringFilterTimeHandle)
            self._steeringFilterTimeFloatDrag = None

        super().on_hide()

    # } PropertyWidgetVehicleBase

    def _get_input_enabled(self):
        return self._physxVehicleInterface.get_input_enabled(self._primPath)

    def _get_mouse_enabled(self):
        return self._physxVehicleInterface.get_mouse_enabled(self._primPath)

    def _get_auto_reverse_enabled(self):
        return self._physxVehicleInterface.get_auto_reverse_enabled(self._primPath)

    def _get_steering_sensitivity(self):
        return self._physxVehicleInterface.get_steering_sensitivity(self._primPath)

    def _get_steering_filter_time(self):
        return self._physxVehicleInterface.get_steering_filter_time(self._primPath)

    def _toggle_input_enabled(self, model):
        value = model.get_value_as_bool()
        self.no_redraw_on_next_command_match()
        PhysXVehicleControllerEnableInputCommand.execute(self._primPath, value)
        self._mouseEnabledCheckBox.enabled = value
        self._mouseEnabledLabel.enabled = value
        self._mouseEnabledCheckBox.model.set_value(self._get_mouse_enabled())
 
    def _toggle_mouse_enabled(self, model):
        value = model.get_value_as_bool()
        self.no_redraw_on_next_command_match()
        PhysXVehicleControllerEnableMouseCommand.execute(self._primPath, value)

    def _toggle_auto_reverse_enabled(self, model):
        value = model.get_value_as_bool()
        self.no_redraw_on_next_command_match()
        PhysXVehicleControllerEnableAutoReverseCommand.execute(self._primPath, value)

    def _on_set_steering_sensitivity(self, model):
        value = model.get_value_as_float()
        self.no_redraw_on_next_command_match()
        PhysXVehicleControllerSetSteeringSensitivityCommand.execute(self._primPath, value)

    def _on_set_steering_filter_time(self, model):
        value = model.get_value_as_float()
        self.no_redraw_on_next_command_match()
        PhysXVehicleControllerSetSteeringFilterTimeCommand.execute(self._primPath, value)


class PropertyWidgetVehicleAuthoring(PropertyWidgetVehicleBase):
    name = "physx_vehicle_authoring"

    def __init__(self):
        super().__init__("Vehicle Authoring Helpers", lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleAPI))

        self._suspensionFrameTransformButton = None

    def __del__(self):
        pass

    # { PropertyWidget

    def clean(self):
        super().clean()

    def build_items(self):
        if (not self.is_valid()):
            return

        layout = omni.ui.VStack(spacing = UI.DEFAULT_WINDOW_SPACING_V, style = PROPERTY_WIDGET_STYLE)

        # Helper to compute some wheel simulation transformations based on the USD prim transformations and the vehicle center
        # of mass

        self._suspensionFrameTransformButton = create_suspension_frame_transforms_autocompute_ui(layout,
            self._on_compute_suspension_frame_transforms_pressed)

    # } PropertyWidget

    # { PropertyWidgetVehicleBase

    def on_hide(self):
        # explicit callback removal to avoid memory leaks as the UI system does not seem to clean those up properly
        if (self._suspensionFrameTransformButton is not None):
            self._suspensionFrameTransformButton.set_clicked_fn(None)
            self._suspensionFrameTransformButton = None

        super().on_hide()

    # } PropertyWidgetVehicleBase

    def _on_compute_suspension_frame_transforms_pressed(self):
        PhysXVehicleSuspensionFrameTransformsAutocomputeCommand.execute(self._primPath)
