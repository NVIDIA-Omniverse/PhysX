# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math

import carb
import omni.ui

from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema

from omni.kit.window.popup_dialog import MessageDialog

from .. helpers import UI

from . import vehicleWizardCommon as Common
from . import physxVehicleWizard as Wizard


# workaround until omni.ui supports alignment on V/HStack
DRIVE_CHECKBOX_SPACER_HEIGHT = 4
MISC_CHECKBOX_SPACER_HEIGHT = 8


class VehicleWizardBasics():
    def __init__(self, wizard):
        self._wizard = wizard

        self._axisComboIndexToAxis = None
        self._driveTypes = ["Standard", "Basic", "None"]

        self._columnWidth = omni.ui.Pixel(100)

        self._pathList = []

        self._updateState = True

    def tear_down(self):
        self._vehiclePrimPathStringField = None
        self._vehiclePrimPathStringFieldSubs = None
        self._vehiclePrimPathGetButton = None
        self._vehiclePrimPathClearButton = None
        self._axisComboBox = None
        self._chassisWidthDrag = None
        self._chassisWidthDragSubs = None
        self._chassisLengthDrag = None
        self._chassisLengthDragSubs = None
        self._chassisHeightDrag = None
        self._chassisHeightDragSubs = None
        self._chassisMassDrag = None
        self._chassisMassDragSubs = None
        self._scanSelectedButton = None
        self._driveComboBox = None
        self._horsepowerDrag = None
        self._engineMaxRPMDrag = None
        self._numberOfGearsDrag = None
        self._tankModeCheckBox = None
        self._numberOfAxlesDrag = None
        self._shareableComponentsCheckBox = None
        self._ackermannCorrectionCheckBox = None
        self._createButton = None
        self._resetButton = None
        self._backButton = None
        self._nextButton = None

    def set_up(self):
        labelTooltipWidth = Common.DEFAULT_LABEL_TOOLTIP_WIDTH
        defaultWidgetWidthList = Common.DEFAULT_WIDGET_WIDTH_LIST

        usdContext = omni.usd.get_context()
        self._stage = usdContext.get_stage()

        lengthScale = self._wizard.vehicleDataManager.vehicleData.unitScale.lengthScale
        massScale = self._wizard.vehicleDataManager.vehicleData.unitScale.massScale

        with self._wizard._window.frame:
            with omni.ui.ScrollingFrame(horizontal_scrollbar_policy=omni.ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=omni.ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED):
                with omni.ui.VStack(style = UI.STYLE_REFERENCE):
                    with omni.ui.VStack(height = Wizard.CONTENT_HEIGHT_WITHOUT_FOOTER):
                        with omni.ui.ScrollingFrame(horizontal_scrollbar_policy=omni.ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                            vertical_scrollbar_policy=omni.ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED):
                            with omni.ui.VStack(spacing = UI.DEFAULT_WINDOW_SPACING_V):

                                layout = UI.create_group_layout("Vehicle Prim")

                                with layout:
                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H, style={"margin": 0}):

                                        pathTooltip = ("The path of the prim to use as the vehicle prim. "
                                            "The vehicle prim is usually the root of a vehicle hierarchy containing all parts that move "
                                            "with the vehicle. The prim needs to be a UsdGeomXform. Leave empty to have the wizard "
                                            "create a separate prim hierarchy for the vehicle.")

                                        UI.create_wrapped_label("Path",
                                            tooltipText = pathTooltip,
                                            tooltipWidth = labelTooltipWidth)

                                        self._vehiclePrimPathStringField = omni.ui.StringField(
                                            width = omni.ui.Pixel(3.2 * self._columnWidth))
                                        self._vehiclePrimPathStringField.model.set_value(self._wizard.vehicleDataManager.vehicleData.vehiclePath)
                                        self._vehiclePrimPathStringFieldSubs = self._vehiclePrimPathStringField.model.subscribe_end_edit_fn(self._on_vehicle_prim_path_change)

                                        buttonWidth = omni.ui.Pixel(80)

                                        selectedToolTip = ("Use the path from the currently selected prim.")

                                        self._vehiclePrimPathGetButton = omni.ui.Button("Selected", width = buttonWidth,
                                            clicked_fn = self._on_get_vehicle_path_from_selected_clicked, tooltip = selectedToolTip)

                                        clearToolTip = ("Clear the path.")

                                        self._vehiclePrimPathClearButton = omni.ui.Button("Clear", width = buttonWidth,
                                            clicked_fn = self._on_clear_vehicle_path_clicked, tooltip = clearToolTip)

                                layout = UI.create_group_layout("Chassis Box")

                                with layout:
                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):

                                        UI.create_wrapped_label("Length", width = self._columnWidth,
                                            tooltipText = "The length of the chassis and the length of the collision object.",
                                            tooltipWidth = labelTooltipWidth)

                                        UI.create_wrapped_label("Width", width = self._columnWidth,
                                            tooltipText = "The width of the chassis and the width of the collision object.",
                                            tooltipWidth = labelTooltipWidth)

                                        UI.create_wrapped_label("Height", width = self._columnWidth,
                                            tooltipText = "The height of the chassis and the height of the collision object.",
                                            tooltipWidth = labelTooltipWidth)

                                        UI.create_wrapped_label("Mass", width = self._columnWidth,
                                            tooltipText = "The mass of the vehicle chassis, not including the wheels and tires.",
                                            tooltipWidth = labelTooltipWidth)

                                        axisTooltip = ("The vehicle points and drives forward in this direction. All vehicles in "
                                            "the scene will also use this longitudinal axis. The longitudinal axis is read from the "
                                            "prim that has PhysxVehicleContextAPI applied, if it exists on the stage.")

                                        axisLabel = UI.create_wrapped_label("Longitudinal Axis", width = self._columnWidth,
                                            tooltipText = axisTooltip,
                                            tooltipWidth = labelTooltipWidth)

                                        if (not self._wizard.vehicleAxesAreImmutable):
                                            axisLabel.enabled = True
                                        else:
                                            axisLabel.enabled = False

                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):

                                        chassisLengthMin = Wizard.MIN_VEHICLE_LENGTH * lengthScale
                                        chassisLengthMax = 1000.0 * lengthScale
                                        self._chassisLengthDrag = omni.ui.FloatDrag(
                                            min = chassisLengthMin, max = chassisLengthMax,
                                            step = 0.001 * lengthScale, width = self._columnWidth)
                                        self._chassisLengthDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.chassisLength)
                                        self._chassisLengthDrag.model.min = chassisLengthMin
                                        self._chassisLengthDrag.model.max = chassisLengthMax
                                        self._chassisLengthDragSubs = self._chassisLengthDrag.model.subscribe_end_edit_fn(self._on_length_change)
                                        # drag widgets that cause the whole page to be redrawn should only trigger the callback once the
                                        # dragging or typing has stopped. That is safer anyway to avoid temporary nonsense values while typing
                                        # a number.

                                        chassisWidthMin = Wizard.MIN_VEHICLE_WIDTH * lengthScale
                                        chassisWidthMax = 1000.0 * lengthScale
                                        self._chassisWidthDrag = omni.ui.FloatDrag(
                                            min = chassisWidthMin, max = chassisWidthMax,
                                            step = 0.001 * lengthScale, width = self._columnWidth)
                                        self._chassisWidthDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.chassisWidth)
                                        self._chassisWidthDrag.model.min = chassisWidthMin
                                        self._chassisWidthDrag.model.max = chassisWidthMax
                                        self._chassisWidthDragSubs = self._chassisWidthDrag.model.subscribe_end_edit_fn(self._on_width_change)

                                        chassisHeightMin = Wizard.MIN_VEHICLE_HEIGHT * lengthScale
                                        chassisHeightMax = 1000.0 * lengthScale
                                        self._chassisHeightDrag = omni.ui.FloatDrag(
                                            min = chassisHeightMin, max = chassisHeightMax,
                                            step = 0.001 * lengthScale, width = self._columnWidth)
                                        self._chassisHeightDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.chassisHeight)
                                        self._chassisHeightDrag.model.min = chassisHeightMin
                                        self._chassisHeightDrag.model.max = chassisHeightMax
                                        self._chassisHeightDragSubs = self._chassisHeightDrag.model.subscribe_end_edit_fn(self._on_height_change)

                                        chassisMassMin = 0.001 * massScale
                                        chassisMassMax = 100000.0 * massScale
                                        self._chassisMassDrag = omni.ui.FloatDrag(
                                            min = chassisMassMin, max = chassisMassMax, step = 0.01 * massScale,
                                            width = self._columnWidth)
                                        self._chassisMassDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.chassisMass)
                                        self._chassisMassDrag.model.min = chassisMassMin
                                        self._chassisMassDrag.model.max = chassisMassMax
                                        self._chassisMassDragSubs = self._chassisMassDrag.model.subscribe_end_edit_fn(self._on_mass_change)

                                        longitudinalAxis = self._wizard.vehicleDataManager.vehicleData.longitudinalAxis

                                        if (self._wizard.vehicleAxesAreImmutable):
                                            axisComboIndex = 0

                                            if (longitudinalAxis == Wizard.VehicleData.AXIS_X):
                                                axisTypes = ["X"]
                                            elif (longitudinalAxis == Wizard.VehicleData.AXIS_Y):
                                                axisTypes = ["Y"]
                                            elif (longitudinalAxis == Wizard.VehicleData.AXIS_Z):
                                                axisTypes = ["Z"]
                                            else:
                                                axisTypes = [""]
                                        else:
                                            verticalAxis = self._wizard.vehicleDataManager.vehicleData.verticalAxis

                                            if (verticalAxis == Wizard.VehicleData.AXIS_Z):
                                                axisTypes = ["X", "Y"]
                                                self._axisComboIndexToAxis = [Wizard.VehicleData.AXIS_X, Wizard.VehicleData.AXIS_Y]

                                                if (longitudinalAxis == Wizard.VehicleData.AXIS_X):
                                                    axisComboIndex = 0
                                                else:
                                                    axisComboIndex = 1

                                            elif (verticalAxis == Wizard.VehicleData.AXIS_Y):
                                                axisTypes = ["X", "Z"]
                                                self._axisComboIndexToAxis = [Wizard.VehicleData.AXIS_X, Wizard.VehicleData.AXIS_Z]

                                                if (longitudinalAxis == Wizard.VehicleData.AXIS_X):
                                                    axisComboIndex = 0
                                                else:
                                                    axisComboIndex = 1

                                            else:
                                                axisTypes = ["Y", "Z"]
                                                self._axisComboIndexToAxis = [Wizard.VehicleData.AXIS_Y, Wizard.VehicleData.AXIS_Z]

                                                if (longitudinalAxis == Wizard.VehicleData.AXIS_Y):
                                                    axisComboIndex = 0
                                                else:
                                                    axisComboIndex = 1

                                        self._axisComboBox = omni.ui.ComboBox(axisComboIndex, *axisTypes, width = self._columnWidth)
                                        self._axisComboBox.model.add_item_changed_fn(self._on_longitudinal_axis_change)

                                        if (not self._wizard.vehicleAxesAreImmutable):
                                            self._axisComboBox.enabled = True
                                        else:
                                            self._axisComboBox.enabled = False

                                    omni.ui.Spacer(height = omni.ui.Pixel(UI.DEFAULT_SPACING_V))

                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):

                                        scanToolTip = ("Scan the currently selected prims and their children to measure the "
                                            "Width, Length, Height and position of the bounding box. Compute the Mass of the vehicle using "
                                            "a constant mass density.")

                                        label = UI.create_wrapped_label("Scan Selected Prims",
                                            width = self._columnWidth,
                                            tooltipText = scanToolTip,
                                            tooltipWidth = labelTooltipWidth)

                                        self._scanSelectedButton = omni.ui.Button("Scan", width = self._columnWidth,
                                            clicked_fn = self._on_scan_selected_clicked)

                                layout = UI.create_group_layout("Drive")

                                with layout:
                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):

                                        driveToolTip = ("\"Standard\" provides an engine, transmission and clutch. \"Basic\" allows the vehicle to be "
                                            "driven with computed tire torques but without an engine and transmission simulation. \"None\" "
                                            "does not add any tire torques nor is steering handled (full user control is provided).")

                                        UI.create_wrapped_label("Type", width = self._columnWidth,
                                            tooltipText = driveToolTip,
                                            tooltipWidth = labelTooltipWidth)

                                        self._horsepowerLabel = UI.create_wrapped_label("Horsepower", width = self._columnWidth,
                                            tooltipText = "The peak engine Horsepower. For \"Standard\" and \"Basic\" drive types only.",
                                            tooltipWidth = labelTooltipWidth)

                                        self._engineMaxRPMLabel = UI.create_wrapped_label("RPM", width = self._columnWidth,
                                            tooltipText = "The peak engine rotation speed (revolutions per minute). For \"Standard\" drive type only.",
                                            tooltipWidth = labelTooltipWidth)

                                        self._numberOfGearsLabel = UI.create_wrapped_label("Number of Gears", width = self._columnWidth,
                                            tooltipText = "The number of gears in the transmission. For \"Standard\" drive type only.",
                                            tooltipWidth = labelTooltipWidth)

                                        tankModeToolTip = ("Wheel based tank configuration with all wheels assigned to 2 tracks left/right. "
                                            "For \"Standard\" drive type only.")

                                        self._tankModeLabel = UI.create_wrapped_label("Tank Mode", width = self._columnWidth,
                                            tooltipText = tankModeToolTip,
                                            tooltipWidth = labelTooltipWidth)

                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):

                                        self._driveComboBox = omni.ui.ComboBox(
                                            self._wizard.vehicleDataManager.vehicleData.driveTypeIndex,
                                            *self._driveTypes, width = self._columnWidth)

                                        self._driveComboBox.model.add_item_changed_fn(self._on_drive_change)

                                        horsepowerMin = 0.0
                                        horsepowerMax = 10000.0
                                        self._horsepowerDrag = omni.ui.FloatDrag(
                                            min = horsepowerMin, max = horsepowerMax,
                                            step = 1, width = self._columnWidth)
                                        self._horsepowerDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.horsepower)
                                        self._horsepowerDrag.model.min = horsepowerMin
                                        self._horsepowerDrag.model.max = horsepowerMax
                                        self._horsepowerDrag.model.add_value_changed_fn(self._on_horsepower_change)

                                        engineMaxRPMMin = 0.0
                                        engineMaxRPMMax = 500000.0
                                        self._engineMaxRPMDrag = omni.ui.FloatDrag(
                                            min = engineMaxRPMMin, max = engineMaxRPMMax,
                                            step = 1, width = self._columnWidth)
                                        self._engineMaxRPMDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.engineMaxRPM)
                                        self._engineMaxRPMDrag.model.min = engineMaxRPMMin
                                        self._engineMaxRPMDrag.model.max = engineMaxRPMMax
                                        self._engineMaxRPMDrag.model.add_value_changed_fn(self._on_engineMaxRPM_change)

                                        numberOfGearsMin = 1
                                        numberOfGearsMax = 30
                                        self._numberOfGearsDrag = omni.ui.IntDrag(
                                            min = numberOfGearsMin, max = numberOfGearsMax,
                                            step = 1, width = self._columnWidth)
                                        self._numberOfGearsDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.numberOfGears)
                                        self._numberOfGearsDrag.model.min = numberOfGearsMin
                                        self._numberOfGearsDrag.model.max = numberOfGearsMax
                                        self._numberOfGearsDrag.model.add_value_changed_fn(self._on_numberOfGears_change)

                                        with omni.ui.VStack(width = 0):
                                            omni.ui.Spacer(width = 0, height = omni.ui.Pixel(DRIVE_CHECKBOX_SPACER_HEIGHT))

                                            self._tankModeCheckBox = omni.ui.CheckBox(width = self._columnWidth)
                                            self._tankModeCheckBox.model.set_value(self._wizard.vehicleDataManager.vehicleData.tankMode)
                                            self._tankModeCheckBox.model.add_value_changed_fn(self._on_tank_mode_change)

                                layout = UI.create_group_layout("Misc")

                                numberOfAxlesMin = 1
                                numberOfAxlesMax = 10
                                (self._numberOfAxlesDrag, label, changeFnHandle) = UI.create_int_drag_with_label(layout,
                                    self._wizard.vehicleDataManager.vehicleData.numberOfAxles, numberOfAxlesMin, numberOfAxlesMax,
                                    0.1, self._on_number_of_axles_change, "Number of Axles",
                                    tooltipText = "The number of axles on the vehicle. Two tires are added per axle.",
                                    widthList = defaultWidgetWidthList)
                                self._numberOfAxlesDrag.model.min = numberOfAxlesMin
                                self._numberOfAxlesDrag.model.max = numberOfAxlesMax

                                with layout:
                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):
                                        UI.create_wrapped_label("Create Shareable Components", width = defaultWidgetWidthList[0],
                                            tooltipText = ("If enabled, vehicle components like wheel, tire, suspension, engine, gears etc. "
                                            "will be separate prims. This allows the setup of those components to be shared between multiple "
                                            "vehicle instances through relationships that point to these components."),
                                            tooltipWidth = labelTooltipWidth)

                                        with omni.ui.VStack(width = 0):
                                            omni.ui.Spacer(width = 0, height = omni.ui.Pixel(MISC_CHECKBOX_SPACER_HEIGHT))

                                            self._shareableComponentsCheckBox = omni.ui.CheckBox(width = 0)
                                            self._shareableComponentsCheckBox.model.set_value(self._wizard.vehicleDataManager.vehicleData.createShareableComponents)
                                            self._shareableComponentsCheckBox.model.add_value_changed_fn(self._on_shareable_components_change)

                                with layout:
                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):
                                        self._ackermannCorrectionLabel = UI.create_wrapped_label("Use Ackermann Steering Correction", width = defaultWidgetWidthList[0],
                                            tooltipText = ("If enabled, the wheels of the first axle will experience Ackermann correction "
                                            "(given that the first axle has a non-zero max steering angle defined). With Ackermann correction "
                                            "the inner wheel will turn more than the outer wheel when following the path around a curve. This "
                                            "avoids wheels having to slip sideways to stay on the path. Note that this option is only available "
                                            "for vehicles with a \"Standard\" or \"Basic\" drive type, if \"Tank Mode\" is disabled and if "
                                            "the number of axles is larger 1."),
                                            tooltipWidth = labelTooltipWidth)

                                        with omni.ui.VStack(width = 0):
                                            omni.ui.Spacer(width = 0, height = omni.ui.Pixel(MISC_CHECKBOX_SPACER_HEIGHT))

                                            self._ackermannCorrectionCheckBox = omni.ui.CheckBox(width = 0)
                                            self._ackermannCorrectionCheckBox.model.set_value(self._wizard.vehicleDataManager.vehicleData.useAckermannCorrection)
                                            self._ackermannCorrectionCheckBox.model.add_value_changed_fn(self._on_ackermann_correction_change)

                                self._refresh_widgets_enabled_state()

                    omni.ui.Spacer(height = omni.ui.Pixel(Common.DEFAULT_CONTROL_BUTTONS_SPACING_TOP))

                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):
                        buttonWidth = omni.ui.Pixel(Common.DEFAULT_CONTROL_BUTTON_WIDTH)

                        self._resetButton = omni.ui.Button("Reset", width = buttonWidth,
                            clicked_fn = self._on_reset_clicked)

                        self._createButton = omni.ui.Button("Create", width = buttonWidth,
                            clicked_fn = self._on_create_clicked)

                        self._backButton = omni.ui.Button("Back", width = buttonWidth,
                            clicked_fn = self._on_back_clicked)

                        self._nextButton = omni.ui.Button("Next", width = buttonWidth,
                            clicked_fn = self._on_next_clicked)

                        self._backButton.enabled = False


    def _refresh_widgets_enabled_state(self):
        if self._wizard.vehicleDataManager.vehicleData.driveTypeIndex is Wizard.DRIVE_TYPE_STANDARD:
            self._horsepowerLabel.enabled = True
            self._horsepowerDrag.enabled = True
            self._engineMaxRPMLabel.enabled = True
            self._engineMaxRPMDrag.enabled = True
            self._numberOfGearsLabel.enabled = True
            self._numberOfGearsDrag.enabled = True
            self._tankModeLabel.enabled = True
            self._tankModeCheckBox.enabled = True
            hasTwoOrMoreAxles = (self._wizard.vehicleDataManager.vehicleData.numberOfAxles > 1)
            ackermannWidgetsEnabled = ((not self._wizard.vehicleDataManager.vehicleData.tankMode) and
                hasTwoOrMoreAxles)
            self._ackermannCorrectionLabel.enabled = ackermannWidgetsEnabled
            self._ackermannCorrectionCheckBox.enabled = ackermannWidgetsEnabled
        elif self._wizard.vehicleDataManager.vehicleData.driveTypeIndex is Wizard.DRIVE_TYPE_BASIC:
            self._horsepowerLabel.enabled = True
            self._horsepowerDrag.enabled = True
            self._engineMaxRPMLabel.enabled = False
            self._engineMaxRPMDrag.enabled = False
            self._numberOfGearsLabel.enabled = False
            self._numberOfGearsDrag.enabled = False
            self._tankModeLabel.enabled = False
            self._tankModeCheckBox.enabled = False
            hasTwoOrMoreAxles = (self._wizard.vehicleDataManager.vehicleData.numberOfAxles > 1)
            self._ackermannCorrectionLabel.enabled = hasTwoOrMoreAxles
            self._ackermannCorrectionCheckBox.enabled = hasTwoOrMoreAxles
        else:
            self._horsepowerLabel.enabled = False
            self._horsepowerDrag.enabled = False
            self._engineMaxRPMLabel.enabled = False
            self._engineMaxRPMDrag.enabled = False
            self._numberOfGearsLabel.enabled = False
            self._numberOfGearsDrag.enabled = False
            self._tankModeLabel.enabled = False
            self._tankModeCheckBox.enabled = False
            self._ackermannCorrectionLabel.enabled = False
            self._ackermannCorrectionCheckBox.enabled = False

    def _on_vehicle_prim_path_change(self, model):
        path = model.get_value_as_string()
        self._wizard.vehicleDataManager.vehicleData.vehiclePath = path

    def _on_get_vehicle_path_from_selected_clicked(self):
        usdContext = omni.usd.get_context()

        selectedPaths = usdContext.get_selection().get_selected_prim_paths()

        if (selectedPaths):
            path = selectedPaths[0]

            prim = self._stage.GetPrimAtPath(path)

            if (prim and prim.IsA(UsdGeom.Xform)):
                # Note: Xform and not Xformable because geometry prims (meshes etc.) should be
                #       excluded or the wizard might end up creating nested geom prims which is
                #       not recommended.

                self._vehiclePrimPathStringField.model.set_value(path)
                # looks like this is not triggering the end change event
                self._on_vehicle_prim_path_change(self._vehiclePrimPathStringField.model)
            else:
                def on_okay_clicked(dialog: MessageDialog):
                    dialog.hide()

                errMsg = "The selected prim has to be a UsdGeomXform"

                errorDialog = MessageDialog(
                    width=400,
                    message=errMsg,
                    ok_handler=lambda dialog: on_okay_clicked(dialog),
                    ok_label="Okay",
                    title="Vehicle Creation Wizard",
                    disable_cancel_button=True
                )
                errorDialog.show()

    def _on_clear_vehicle_path_clicked(self):
        self._vehiclePrimPathStringField.model.set_value("")
        # looks like this is not triggering the end change event
        self._on_vehicle_prim_path_change(self._vehiclePrimPathStringField.model)

    def _on_number_of_axles_change(self, model):
        numberOfAxles = model.get_value_as_int()
        self._wizard.vehicleDataManager.vehicleData.set_number_of_axles(numberOfAxles)

        self._updateState = False
        if (numberOfAxles < 2):
            self._ackermannCorrectionCheckBox.model.set_value(self._wizard.vehicleDataManager.vehicleData.useAckermannCorrection)
        self._updateState = True

        self._refresh_widgets_enabled_state()

    def _on_shareable_components_change(self, model):
        self._wizard.vehicleDataManager.vehicleData.createShareableComponents = model.get_value_as_bool()

    def _on_longitudinal_axis_change(self, model, item):
        (longitudinalAxisString, comboBoxIndex) = UI.get_selected_string_and_index(model)
        self._wizard.vehicleDataManager.set_longitudinal_axis(self._axisComboIndexToAxis[comboBoxIndex])

    def _on_length_change(self, model):
        length = model.get_value_as_float()
        lengthScale = self._wizard.vehicleDataManager.vehicleData.unitScale.lengthScale

        self._wizard.vehicleDataManager.set_chassis_length(length)

        self._updateState = False
        self._chassisMassDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.chassisMass)
        self._engineMaxRPMDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.engineMaxRPM)
        self._horsepowerDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.horsepower)
        self._updateState = True

    def _on_width_change(self, model):
        width = model.get_value_as_float()
        lengthScale = self._wizard.vehicleDataManager.vehicleData.unitScale.lengthScale

        self._wizard.vehicleDataManager.set_chassis_width(width)

        self._updateState = False
        self._chassisMassDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.chassisMass)
        self._updateState = True

    def _on_height_change(self, model):
        height = model.get_value_as_float()
        lengthScale = self._wizard.vehicleDataManager.vehicleData.unitScale.lengthScale

        self._wizard.vehicleDataManager.set_chassis_height(height)

        self._updateState = False
        self._chassisMassDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.chassisMass)
        self._updateState = True

    def _on_mass_change(self, model):
        if self._updateState:
            self._wizard.vehicleDataManager.set_chassis_mass(model.get_value_as_float())

    def _buildPathList(self, path):
        prim = self._stage.GetPrimAtPath(path)

        if prim.IsValid():
            self._pathList.append(path)

            children = prim.GetAllChildren()

            for child in children:
                self._buildPathList(child.GetPrimPath())

    def _on_scan_selected_clicked(self):
        usdContext = omni.usd.get_context()

        # Recursively assemble a list of selected paths
        self._pathList.clear()
        selectedPaths = usdContext.get_selection().get_selected_prim_paths()

        for path in selectedPaths:
            self._buildPathList(path)

        # Remove all duplicates from the path list.
        self._pathList = list(dict.fromkeys(self._pathList))

        maxExtents = Gf.Vec3f(-Wizard.FLT_MAX, -Wizard.FLT_MAX, -Wizard.FLT_MAX)
        minExtents = Gf.Vec3f(Wizard.FLT_MAX, Wizard.FLT_MAX, Wizard.FLT_MAX)
        scanComplete = False

        for path in self._pathList:
            prim = self._stage.GetPrimAtPath(path)

            if prim.IsA(UsdGeom.Xformable):
                primXform = UsdGeom.Xformable(prim)
                purposeAttr = primXform.GetPurposeAttr()
                purpose = purposeAttr.Get()

                if purpose == UsdGeom.Tokens.default_ or purpose == UsdGeom.Tokens.render:
                    bbox = primXform.ComputeWorldBound(0, purpose)
                    alignedRange = bbox.ComputeAlignedRange()

                    bboxMin = alignedRange.GetMin()
                    bboxMax = alignedRange.GetMax()
                    extents = bboxMax - bboxMin

                    if extents.GetLength() < Wizard.FLT_MAX:
                        scanComplete = True

                        minExtents[0] = min(minExtents[0], bboxMin[0])
                        minExtents[1] = min(minExtents[1], bboxMin[1])
                        minExtents[2] = min(minExtents[2], bboxMin[2])

                        maxExtents[0] = max(maxExtents[0], bboxMax[0])
                        maxExtents[1] = max(maxExtents[1], bboxMax[1])
                        maxExtents[2] = max(maxExtents[2], bboxMax[2])

        if scanComplete:
            extents = maxExtents - minExtents
            position = 0.5 * (maxExtents + minExtents)

            lateralAxis = self._wizard.vehicleDataManager.get_lateral_axis_vector()

            chassisWidth = math.fabs(lateralAxis.GetDot(extents))
            chassisLength = math.fabs(self._wizard.vehicleDataManager.get_longitudinal_axis_vector().GetDot(extents))
            chassisHeight = math.fabs(self._wizard.vehicleDataManager.get_vertical_axis_vector().GetDot(extents))

            self._wizard.vehicleDataManager.vehicleData.position = position

            self._wizard.vehicleDataManager.set_chassis_width(chassisWidth, False)
            self._wizard.vehicleDataManager.set_chassis_length(chassisLength, False)
            self._wizard.vehicleDataManager.set_chassis_height(chassisHeight, False)
            self._wizard.vehicleDataManager.update()

            self._wizard._switch_active_page()
        else:
            carb.log_error("Nothing was scanned. Only prims with the default or render purpose are included.")

    def _on_horsepower_change(self, model):
        if self._updateState:
            self._wizard.vehicleDataManager.set_horsepower(model.get_value_as_float())

    def _on_engineMaxRPM_change(self, model):
        if self._updateState:
            self._wizard.vehicleDataManager.set_engine_max_rpm(model.get_value_as_float())

    def _on_numberOfGears_change(self, model):
        self._wizard.vehicleDataManager.vehicleData.numberOfGears = model.get_value_as_int()

    def _on_tank_mode_change(self, model):
        if self._updateState:
            tankMode = model.get_value_as_bool()
            self._wizard.vehicleDataManager.vehicleData.set_tank_mode(tankMode)

            self._updateState = False
            if (tankMode):
                self._ackermannCorrectionCheckBox.model.set_value(self._wizard.vehicleDataManager.vehicleData.useAckermannCorrection)
            self._updateState = True

            self._refresh_widgets_enabled_state()

    def _on_ackermann_correction_change(self, model):
        if self._updateState:
            self._wizard.vehicleDataManager.vehicleData.set_use_ackermann_correction(model.get_value_as_bool())

    def _on_drive_change(self, model, item):
        (driveType, driveTypeIndex) = UI.get_selected_string_and_index(model)
        self._wizard.vehicleDataManager.set_drive_type(driveTypeIndex)

        self._updateState = False
        self._horsepowerDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.horsepower)
        self._tankModeCheckBox.model.set_value(self._wizard.vehicleDataManager.vehicleData.tankMode)
        if (driveTypeIndex == Wizard.DRIVE_TYPE_NONE):
            self._ackermannCorrectionCheckBox.model.set_value(self._wizard.vehicleDataManager.vehicleData.useAckermannCorrection)
        self._updateState = True

        self._refresh_widgets_enabled_state()

    def _on_create_clicked(self):
        self._wizard.create_vehicle_command()

    def _on_reset_clicked(self):
        self._wizard.reset_data()
        self._wizard._switch_active_page()

    def _on_next_clicked(self):
        self._wizard.switch_to_next_page()

    def _on_back_clicked(self):
        return
