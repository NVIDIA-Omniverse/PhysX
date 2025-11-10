# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math

import carb
import omni.ui

from pxr import Usd, UsdGeom, Gf

from omni.kit.window.popup_dialog import MessageDialog

from omni.kit.property.physx.widgets import REMOVE_BUTTON_STYLE

from ..helpers import UI

from . import vehicleWizardCommon as Common
from . import physxVehicleWizard as Wizard


# workaround until omni.ui supports alignment on V/HStack
AXLE_CHECKBOX_SPACER_HEIGHT = 8
AXLE_FLOATDRAG_SPACER_HEIGHT = 4
WHEEL_CHECKBOX_SPACER_HEIGHT = 5


class VehicleWizardAxles():
    def __init__(self, wizard):
        self._wizard = wizard

        self._axleIndexWidth = omni.ui.Pixel(20)
        self._maxSteerWidth = omni.ui.Pixel(70)
        drivenWidth = 40
        drivenHorizontalOffset = 12
        self._drivenWidth = omni.ui.Pixel(drivenWidth)
        self._drivenHorizontalOffset = omni.ui.Pixel(drivenHorizontalOffset)  # workaround since there is no centered arrangement of widgets yet
        self._drivenCheckBoxWidth = omni.ui.Pixel(drivenWidth - drivenHorizontalOffset)
        self._weightDistributionWidth = omni.ui.Pixel(70)
        self._suspDampingWidth = omni.ui.Pixel(70)
        leftRightButtonWith = 40
        leftRightButtonSpacing = 2
        self._leftRightButtonWidth = omni.ui.Pixel(leftRightButtonWith)
        self._leftRightButtonSpacing = omni.ui.Pixel(leftRightButtonSpacing)
        self._scanSelPrimWidth = omni.ui.Pixel((2*leftRightButtonWith) + leftRightButtonSpacing)
        self._selectWheelAttPrimWidth = self._scanSelPrimWidth

        self._wheelIndexWidth = omni.ui.Pixel(50)
        self._columnWidth = omni.ui.Pixel(100)
        self._wheelAttPathWidth = omni.ui.Pixel(150)

        self._tireTypes = ["Summer", "All Season", "Slick"]
        self._queryTypes = ["Raycast", "Sweep"]

        self._pathList = []
        
        self._updateState = True

        self._minRadius = Wizard.compute_tire_radius(Wizard.MIN_VEHICLE_LENGTH)
        self._minWidth = Wizard.compute_tire_width(self._minRadius)

    def __del__(self):
        return

    def addAxle(self, layout, index):
        with layout:
            with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):

                omni.ui.Label(str(index + 1), word_wrap = True, width = self._axleIndexWidth)


                def _on_max_steer_angle_change(model):
                    self._wizard.vehicleDataManager.vehicleData.maxSteerAngle[index] = model.get_value_as_float()

                with omni.ui.VStack(width = 0):
                    omni.ui.Spacer(width = 0, height = omni.ui.Pixel(AXLE_FLOATDRAG_SPACER_HEIGHT))

                    maxSteerAngleMin = -90.0
                    maxSteerAngleMax = 90.0
                    maxSteerAngleDrag = omni.ui.FloatDrag(min = maxSteerAngleMin, max = maxSteerAngleMax, step = 0.1, width = self._maxSteerWidth)
                    maxSteerAngleDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.maxSteerAngle[index])
                    maxSteerAngleDrag.model.min = maxSteerAngleMin
                    maxSteerAngleDrag.model.max = maxSteerAngleMax
                    maxSteerAngleDrag.model.add_value_changed_fn(_on_max_steer_angle_change)
                    self._maxSteerAngleDrag.append(maxSteerAngleDrag)

                    if ((self._wizard.vehicleDataManager.vehicleData.driveTypeIndex is Wizard.DRIVE_TYPE_NONE) or
                        (self._wizard.vehicleDataManager.vehicleData.tankMode) or
                        (self._wizard.vehicleDataManager.vehicleData.useAckermannCorrection and (index != 0))):
                        maxSteerAngleDrag.enabled = False


                with omni.ui.HStack(width = self._drivenWidth, height = 0, spacing = 0):
                    omni.ui.Spacer(width = self._drivenHorizontalOffset, height = 0)

                    def _on_driven_change(model):
                        self._wizard.vehicleDataManager.vehicleData.driven[index] = model.get_value_as_bool()

                    with omni.ui.VStack(width = 0):
                        omni.ui.Spacer(width = 0, height = omni.ui.Pixel(AXLE_CHECKBOX_SPACER_HEIGHT))

                        drivenCheckBox = omni.ui.CheckBox(width = self._drivenCheckBoxWidth)
                        drivenCheckBox.model.set_value(self._wizard.vehicleDataManager.vehicleData.driven[index])
                        drivenCheckBox.model.add_value_changed_fn(_on_driven_change)
                        self._drivenCheckBox.append(drivenCheckBox)

                        if self._wizard.vehicleDataManager.vehicleData.driveTypeIndex is Wizard.DRIVE_TYPE_NONE:
                            drivenCheckBox.enabled = False


                def _on_weight_distr_change(model):
                    self._wizard.vehicleDataManager.vehicleData.weightDistribution[index] = model.get_value_as_float()

                with omni.ui.VStack(width = 0):
                    omni.ui.Spacer(width = 0, height = omni.ui.Pixel(AXLE_FLOATDRAG_SPACER_HEIGHT))

                    weightDistributionMin = 1.0
                    weightDistributionMax = 100.0
                    weightDistributionDrag = omni.ui.FloatDrag(min = weightDistributionMin, max = weightDistributionMax, step = 0.1, width = self._weightDistributionWidth)
                    weightDistributionDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.weightDistribution[index])
                    weightDistributionDrag.model.min = weightDistributionMin
                    weightDistributionDrag.model.max = weightDistributionMax
                    weightDistributionDrag.model.add_value_changed_fn(_on_weight_distr_change)
                    self._weightDistributionDrag.append(weightDistributionDrag)


                def _on_damping_change(model):
                    self._wizard.vehicleDataManager.vehicleData.damping[index] = model.get_value_as_float()

                with omni.ui.VStack(width = 0):
                    omni.ui.Spacer(width = 0, height = omni.ui.Pixel(AXLE_FLOATDRAG_SPACER_HEIGHT))

                    dampingMin = 0.0
                    dampingMax = 1.0
                    dampingDrag = omni.ui.FloatDrag(min = dampingMin, max = dampingMax, step = 0.01, width = self._suspDampingWidth)
                    dampingDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.damping[index])
                    dampingDrag.model.min = dampingMin
                    dampingDrag.model.max = dampingMax
                    dampingDrag.model.add_value_changed_fn(_on_damping_change)
                    self._dampingDrag.append(dampingDrag)


                with omni.ui.HStack(height = 0, spacing = self._leftRightButtonSpacing, style={"margin_width": 0}):
                    def _on_scan_selected_left_clicked():
                        self._on_scan_selected_clicked(index, 0)

                    scanSelectedButtonLeft = omni.ui.Button("Left", 
                        width = self._leftRightButtonWidth,
                        clicked_fn = _on_scan_selected_left_clicked)


                    def _on_scan_selected_right_clicked():
                        self._on_scan_selected_clicked(index, 1)

                    scanSelectedButtonRight = omni.ui.Button("Right", 
                        width = self._leftRightButtonWidth,
                        clicked_fn = _on_scan_selected_right_clicked)


                with omni.ui.HStack(height = 0, spacing = self._leftRightButtonSpacing, style={"margin_width": 0}):
                    def _on_select_wheel_att_prim_left_clicked():
                        self._on_select_wheel_att_prim_clicked(index, 0)

                    scanSelectedButtonLeft = omni.ui.Button("Left", 
                        width = self._leftRightButtonWidth,
                        clicked_fn = _on_select_wheel_att_prim_left_clicked)


                    def _on_select_wheel_att_prim_right_clicked():
                        self._on_select_wheel_att_prim_clicked(index, 1)

                    scanSelectedButtonRight = omni.ui.Button("Right", 
                        width = self._leftRightButtonWidth,
                        clicked_fn = _on_select_wheel_att_prim_right_clicked)


    def addWheel(self, layout, index, side):
        lengthScale = self._wizard.vehicleDataManager.vehicleData.unitScale.lengthScale
        massScale = self._wizard.vehicleDataManager.vehicleData.unitScale.massScale

        with layout:
            with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H, direction = omni.ui.Direction.RIGHT_TO_LEFT):

                def _on_remove_wheel_entry():
                    self._wizard.vehicleDataManager.vehicleData.reset_tire(2 * index + side)
                    self._wizard._switch_active_page()

                removeButton = omni.ui.Button(style = REMOVE_BUTTON_STYLE, width = omni.ui.Pixel(20),
                    clicked_fn = _on_remove_wheel_entry)

                with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):

                    tireIndex = 2 * index + side

                    if side == 0:
                        omni.ui.Label(str(index + 1) + " Left", word_wrap = True, width = self._wheelIndexWidth)
                    else:
                        omni.ui.Label(str(index + 1) + " Right", word_wrap = True, width = self._wheelIndexWidth)


                    def _on_radius_change(model):
                        self._wizard.vehicleDataManager.vehicleData.tireList[tireIndex].radius = model.get_value_as_float()

                    radiusMin = self._minRadius * lengthScale
                    radiusMax = 1000.0 * lengthScale
                    radiusDrag = omni.ui.FloatDrag(min = radiusMin, max = radiusMax, 
                        step = 0.001 * lengthScale, width = self._columnWidth)
                    radiusDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.tireList[tireIndex].radius)
                    radiusDrag.model.min = radiusMin
                    radiusDrag.model.max = radiusMax
                    radiusDrag.model.add_value_changed_fn(_on_radius_change)
                    self._radiusDrag.append(radiusDrag)


                    def _on_width_change(model):
                        self._wizard.vehicleDataManager.vehicleData.tireList[tireIndex].width = model.get_value_as_float()

                    widthMin = self._minWidth * lengthScale
                    widthMax = 1000.0 * lengthScale
                    widthDrag = omni.ui.FloatDrag(min = widthMin, max = widthMax, 
                        step = 0.001 * lengthScale, width = self._columnWidth)
                    widthDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.tireList[tireIndex].width)
                    widthDrag.model.min = widthMin
                    widthDrag.model.max = widthMax
                    widthDrag.model.add_value_changed_fn(_on_width_change)
                    self._widthDrag.append(widthDrag)


                    def _on_mass_change(model):
                        self._wizard.vehicleDataManager.vehicleData.tireList[tireIndex].mass = model.get_value_as_float()

                    massMin = 0.01 * massScale
                    massMax = 1000.0 * massScale
                    massDrag = omni.ui.FloatDrag(min = massMin, max = massMax, step = 0.1 * massScale, width = self._columnWidth)
                    massDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.tireList[tireIndex].mass)
                    massDrag.model.add_value_changed_fn(_on_mass_change)
                    massDrag.model.min = massMin
                    massDrag.model.max = massMax
                    self._massDrag.append(massDrag)


                    def _on_wheel_att_path_change(model):
                        str = model.get_value_as_string()
                        self._wizard.vehicleDataManager.vehicleData.tireList[tireIndex].path = str

                        self._wizard._switch_active_page()
                        # to be consistent with moving to other page and back (hide entry if attachment path
                        # was the only reason for the entry. Enable/disable collision option)

                    stringField = omni.ui.StringField(width = self._wheelAttPathWidth)
                    stringField.model.set_value(self._wizard.vehicleDataManager.vehicleData.tireList[tireIndex].path)
                    stringFieldSubs = stringField.model.subscribe_end_edit_fn(_on_wheel_att_path_change)
                    self._wheelAttPathStringFieldList.append(stringField)
                    self._wheelAttPathStringFieldSubsList.append(stringFieldSubs)


    def set_up(self):

        labelTooltipWidth = Common.DEFAULT_LABEL_TOOLTIP_WIDTH
        defaultWidgetWidthList = Common.DEFAULT_WIDGET_WIDTH_LIST
        extraHSpace = omni.ui.Pixel(10)
        
        self._maxSteerAngleDrag = []
        self._drivenCheckBox = []
        self._weightDistributionDrag = []
        self._dampingDrag = []
        self._radiusDrag = []
        self._widthDrag = []
        self._massDrag = []
        self._wheelAttPathStringFieldList = []
        self._wheelAttPathStringFieldSubsList = []

        self._usdContext = omni.usd.get_context()
        self._stage = self._usdContext.get_stage()

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

                                layout = UI.create_group_layout("Axles")

                                with layout:
                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):
                                        omni.ui.Spacer(width = self._axleIndexWidth)

                                        isDriveTypeNone = self._wizard.vehicleDataManager.vehicleData.driveTypeIndex is Wizard.DRIVE_TYPE_NONE

                                        steerAngleTooltip = ("The maximum angle the tires can steer. Not applicable to drive type \"None\" "
                                            "or if tank mode is enabled. If Ackermann correction is desired, only the first axle will be "
                                            "editable.")

                                        labelAlignment = omni.ui.Alignment.CENTER_TOP

                                        label = UI.create_wrapped_label("Steer Angle (degrees)", width = self._maxSteerWidth, 
                                            tooltipText = steerAngleTooltip, 
                                            tooltipWidth = labelTooltipWidth)
                                        label.alignment = labelAlignment

                                        if isDriveTypeNone:
                                            label.enabled = False

                                        label = UI.create_wrapped_label("Driven", width = self._drivenWidth,
                                            tooltipText = "Checked to send engine torques to this axle. Not applicable to drive type \"None\".",
                                            tooltipWidth = labelTooltipWidth)
                                        label.alignment = labelAlignment

                                        if isDriveTypeNone:
                                            label.enabled = False

                                        weightTooltip = ("The percentage of the vehicle weight supported by this axle. If the sum does "
                                            "not add up to 100%, each percentage will be normalized.")

                                        label = UI.create_wrapped_label("Weight Distribution", width = self._weightDistributionWidth,
                                            tooltipText = weightTooltip, tooltipWidth = labelTooltipWidth)
                                        label.alignment = labelAlignment

                                        dampingTooltip = ("The damping ratio for the suspension on this axle. 0.0 = no damping, "
                                            "0.3 = normal damping, 0.7 = critical damping, 1.0 = overdamped.")

                                        label = UI.create_wrapped_label("Suspension Damping", width = self._suspDampingWidth, 
                                            tooltipText = dampingTooltip, tooltipWidth = labelTooltipWidth)
                                        label.alignment = labelAlignment

                                        scanToolTip = ("Scan the currently selected prims to measure the Radius, Width and position of "
                                            "each wheel. Compute the Mass of each wheel using a constant mass density.")

                                        label = UI.create_wrapped_label("Scan Selected Prims", width = self._scanSelPrimWidth, 
                                            tooltipText = scanToolTip, tooltipWidth = labelTooltipWidth)
                                        label.alignment = labelAlignment

                                        selectWheelAttToolTip = ("Choose the currently selected prim as the wheel attachment prim and "
                                            "apply the required APIs in place (instead of creating a separate prim). The prim "
                                            "needs to be a UsdGeomXformable and a vehicle prim that is an ascendant has to be "
                                            "defined too.")

                                        label = UI.create_wrapped_label("Select Wheel Attachment", width = self._selectWheelAttPrimWidth, 
                                            tooltipText = selectWheelAttToolTip, tooltipWidth = labelTooltipWidth)
                                        label.alignment = labelAlignment

                                    omni.ui.Spacer(height = omni.ui.Pixel(UI.DEFAULT_SPACING_V))

                                for axle in range(self._wizard.vehicleDataManager.vehicleData.numberOfAxles):
                                    self.addAxle(layout, axle)

                                layout = UI.create_group_layout("Wheels")

                                tireTooltip = ("The type of tire used for all tires. Slicks are sticky racing tires, Summer tires " 
                                    "have higher than normal grip on smooth surfaces and All Season have the lowest friction values, " 
                                    "for smooth dry surfaces.")

                                with layout:
                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):

                                        label = UI.create_wrapped_label("Tire Type", width = self._wheelIndexWidth, 
                                            tooltipText = tireTooltip, tooltipWidth = labelTooltipWidth)

                                        self._typeComboBox = omni.ui.ComboBox(
                                            self._wizard.vehicleDataManager.vehicleData.tireTypeIndex,
                                            *self._tireTypes,
                                            width = self._columnWidth)
                                        self._typeComboBox.model.add_item_changed_fn(self._on_tire_type_change)

                                        omni.ui.Spacer(width = extraHSpace)

                                        if (self._wizard.vehicleDataManager.vehicleData.has_wheel_attachment_prim_defined()):
                                            canCreateCollisionGeometry = False
                                            if (self._wizard.vehicleDataManager.vehicleData.wheelCollisionGeometry):
                                                self._wizard.vehicleDataManager.vehicleData.wheelCollisionGeometry = False
                                        else:
                                            canCreateCollisionGeometry = True

                                        UI.create_wrapped_label("Enable Collisions", width = 0,
                                            tooltipText = ("If enabled, collision objects will be created for the wheels. Note that it "
                                                "will be important to filter out collisions between the wheels "
                                                "and the ground in this case. This option is disabled if wheel attachment prims are "
                                                "defined."), 
                                                tooltipWidth = labelTooltipWidth)

                                        with omni.ui.VStack(width = 0):
                                            omni.ui.Spacer(width = 0, height = omni.ui.Pixel(WHEEL_CHECKBOX_SPACER_HEIGHT))
                                
                                            self._wheelCollisionCheckbox = omni.ui.CheckBox(width = 0)
                                            self._wheelCollisionCheckbox.model.set_value(self._wizard.vehicleDataManager.vehicleData.wheelCollisionGeometry)
                                            self._wheelCollisionCheckbox.model.add_value_changed_fn(self._on_wheel_collision_change)
                                            self._wheelCollisionCheckbox.enabled = canCreateCollisionGeometry

                                        omni.ui.Spacer(width = extraHSpace)

                                        UI.create_wrapped_label("Query Type", width = 0,
                                            tooltipText = "Collision of the wheels with the ground surface is detected through scene queries "
                                            "along the suspension direction. Either raycasts or a sweeps can be used. Raycasts are faster "
                                            "while sweeps can represent the wheel shape better.", 
                                            tooltipWidth = labelTooltipWidth)

                                        self._queryTypeComboBox = omni.ui.ComboBox(
                                            self._wizard.vehicleDataManager.vehicleData.queryTypeIndex,
                                            *self._queryTypes,
                                            width = self._columnWidth)
                                        self._queryTypeComboBox.model.add_item_changed_fn(self._on_query_type_change)

                                    omni.ui.Spacer(height = omni.ui.Pixel(UI.DEFAULT_SPACING_V))

                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):

                                        UI.create_wrapped_label("", width = self._wheelIndexWidth, 
                                            tooltipText = "", 
                                            tooltipWidth = labelTooltipWidth)

                                        UI.create_wrapped_label("Radius", width = self._columnWidth, 
                                            tooltipText = "The radius of the tire and the radius of the tire collision object.", 
                                            tooltipWidth = labelTooltipWidth)

                                        UI.create_wrapped_label("Width", width = self._columnWidth,
                                            tooltipText = "The width of the tire and the width of the tire collision object.",
                                            tooltipWidth = labelTooltipWidth)

                                        UI.create_wrapped_label("Mass", width = self._columnWidth,
                                            tooltipText = "The mass of the wheel and tire, but not including the suspension parts.", 
                                            tooltipWidth = labelTooltipWidth)

                                        UI.create_wrapped_label("Path", width = self._columnWidth,
                                            tooltipText = ("The path of the wheel attachment prim (if specified). Leave empty if a "
                                            "prim should be created instead."), 
                                            tooltipWidth = labelTooltipWidth)

                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):

                                        UI.create_wrapped_label("Default", width = self._wheelIndexWidth,
                                            tooltipText = "Change these settings to apply them to all wheels.", 
                                            tooltipWidth = labelTooltipWidth)

                                        radiusMin = self._minRadius * lengthScale
                                        radiusMax = 1000.0 * lengthScale
                                        self._allRadiusDrag = omni.ui.FloatDrag(
                                            min = radiusMin, max = radiusMax, 
                                            step = 0.001 * lengthScale, width = self._columnWidth)
                                        self._allRadiusDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.tireRadius)
                                        self._allRadiusDrag.model.min = radiusMin
                                        self._allRadiusDrag.model.max = radiusMax
                                        self._allRadiusDragSubs = self._allRadiusDrag.model.subscribe_end_edit_fn(self._on_all_radius_change)
                                        # drag widgets that cause the whole page to be redrawn should only trigger the callback once the
                                        # dragging or typing has stopped. That is safer anyway to avoid temporary nonsense values while typing
                                        # a number.

                                        widthMin = self._minWidth * lengthScale
                                        widthMax = 1000.0 * lengthScale
                                        self._allWidthDrag = omni.ui.FloatDrag(
                                            min = widthMin, max = widthMax, 
                                            step = 0.001 * lengthScale, width = self._columnWidth)
                                        self._allWidthDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.tireWidth)
                                        self._allWidthDrag.model.min = widthMin
                                        self._allWidthDrag.model.max = widthMax
                                        self._allWidthDragSubs = self._allWidthDrag.model.subscribe_end_edit_fn(self._on_all_width_change)

                                        allMassMin = 0.01 * massScale
                                        allMassMax = 1000.0 * massScale
                                        self._allMassDrag = omni.ui.FloatDrag(
                                            min = allMassMin, max = allMassMax, step = 0.1 * massScale, 
                                            width = self._columnWidth)
                                        self._allMassDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.tireMass)
                                        self._allMassDrag.model.min = allMassMin
                                        self._allMassDrag.model.max = allMassMax
                                        self._allMassDragSubs = self._allMassDrag.model.subscribe_end_edit_fn(self._on_all_mass_change)

                                        UI.create_wrapped_label("...", width = self._wheelIndexWidth)

                                    omni.ui.Spacer(height = omni.ui.Pixel(UI.DEFAULT_SPACING_V))

                                    with omni.ui.HStack(height = 0, spacing = UI.DEFAULT_SPACING_H):
                                        for axle in range(self._wizard.vehicleDataManager.vehicleData.numberOfAxles):
                                            for side in range(2):
                                                if ((self._wizard.vehicleDataManager.vehicleData.tireList[2 * axle + side].position is not None) or
                                                    (self._wizard.vehicleDataManager.vehicleData.tireList[2 * axle + side].path)):
                                                    self.addWheel(layout, axle, side)

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

    def tear_down(self):
        self._maxSteerAngleDrag = None
        self._drivenCheckBox = None
        self._weightDistributionDrag = None
        self._dampingDrag = None
        self._typeComboBox = None
        self._allRadiusDrag = None
        self._allRadiusDragSubs = None
        self._allWidthDrag = None
        self._allWidthDragSubs = None
        self._allMassDrag = None
        self._allMassDragSubs = None
        self._radiusDrag = None
        self._widthDrag = None
        self._massDrag = None
        self._wheelAttPathStringFieldList = None
        self._wheelAttPathStringFieldSubsList = None
        self._wheelCollisionCheckbox = None
        self._queryTypeComboBox = None
        self._createButton = None
        self._resetButton = None
        self._backButton = None
        self._nextButton = None
    
    def _buildPathList(self, path):
        prim = self._stage.GetPrimAtPath(path)
        
        if prim.IsValid():
            self._pathList.append(path)
            
            children = prim.GetAllChildren()

            for child in children:
                self._buildPathList(child.GetPrimPath())

    def _on_scan_selected_clicked(self, index, side):
        scanComplete = False

        usdContext = omni.usd.get_context()
        
        self._pathList.clear()
        selectedPaths = usdContext.get_selection().get_selected_prim_paths()
   
        for path in selectedPaths:
            self._buildPathList(path)

        # Remove all duplicates from the path list.
        self._pathList = list(dict.fromkeys(self._pathList))
 
        maxExtents = Gf.Vec3f(-1.0e38, -1.0e38, -1.0e38)
        minExtents = Gf.Vec3f(1.0e38, 1.0e38, 1.0e38)

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

            tireRadius = 0.5 * math.fabs(self._wizard.vehicleDataManager.get_vertical_axis_vector().GetDot(extents))
            tireWidth = math.fabs(lateralAxis.GetDot(extents))

            tireMass = Wizard.compute_tire_mass( 
                tireRadius, 
                tireWidth,
                self._wizard.vehicleDataManager.vehicleData.unitScale)

            self._wizard.vehicleDataManager.vehicleData.tireList[2 * index + side].radius = tireRadius
            self._wizard.vehicleDataManager.vehicleData.tireList[2 * index + side].width = tireWidth
            self._wizard.vehicleDataManager.vehicleData.tireList[2 * index + side].mass = tireMass
            self._wizard.vehicleDataManager.vehicleData.tireList[2 * index + side].position = position

            self._wizard._switch_active_page()
        else:
            carb.log_error("Nothing was scanned. Only prims with the default or render purpose are included.")

    def _on_select_wheel_att_prim_clicked(self, index, side):
        usdContext = omni.usd.get_context()

        selectedPaths = usdContext.get_selection().get_selected_prim_paths()

        if (selectedPaths):
            path = selectedPaths[0]

            prim = self._stage.GetPrimAtPath(path)
            
            if (prim and prim.IsA(UsdGeom.Xformable)):
                tireListIdx = 2 * index + side
                isPrimAvailable = True
                for i in range(self._wizard.vehicleDataManager.vehicleData.numberOfAxles * 2):
                    if ((self._wizard.vehicleDataManager.vehicleData.tireList[i].path == path) and (i != tireListIdx)):
                        isPrimAvailable = False
                        break

                if (isPrimAvailable):
                    self._wizard.vehicleDataManager.vehicleData.tireList[tireListIdx].path = path
                    self._wizard._switch_active_page()
                else:
                    def on_okay_clicked(dialog: MessageDialog):
                        dialog.hide()

                    errMsg = "The selected prim is already assigned to a wheel attachment"

                    errorDialog = MessageDialog(
                        width=400,
                        message=errMsg,
                        ok_handler=lambda dialog: on_okay_clicked(dialog),
                        ok_label="Okay",
                        title="Vehicle Creation Wizard",
                        disable_cancel_button=True
                    )
                    errorDialog.show()
            else:
                def on_okay_clicked(dialog: MessageDialog):
                    dialog.hide()

                errMsg = "The selected prim has to be an UsdGeomXformable"

                errorDialog = MessageDialog(
                    width=400,
                    message=errMsg,
                    ok_handler=lambda dialog: on_okay_clicked(dialog),
                    ok_label="Okay",
                    title="Vehicle Creation Wizard",
                    disable_cancel_button=True
                )
                errorDialog.show()

    def _on_tire_type_change(self, model, item):
        (tireType, self._wizard.vehicleDataManager.vehicleData.tireTypeIndex) = UI.get_selected_string_and_index(model)

    def _on_all_radius_change(self, model):
        tireRadius = model.get_value_as_float()

        self._wizard.vehicleDataManager.set_tire_radius(tireRadius)
        
        self._updateState = False
        self._allMassDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.tireMass)
        self._updateState = True

        self._wizard._switch_active_page()  # update entries of selected wheel attachment prims that had no volume scan

    def _on_all_width_change(self, model):
        tireWidth = model.get_value_as_float()

        self._wizard.vehicleDataManager.set_tire_width(tireWidth)
        
        self._updateState = False
        self._allMassDrag.model.set_value(self._wizard.vehicleDataManager.vehicleData.tireMass)
        self._updateState = True

        self._wizard._switch_active_page()  # update entries of selected wheel attachment prims that had no volume scan

    def _on_all_mass_change(self, model):
        if self._updateState:
            tireMass = model.get_value_as_float()
            self._wizard.vehicleDataManager.set_tire_mass(tireMass)
            self._wizard._switch_active_page()  # update entries of selected wheel attachment prims that had no volume scan

    def _on_wheel_collision_change(self, model):
        self._wizard.vehicleDataManager.vehicleData.wheelCollisionGeometry = model.get_value_as_bool()

    def _on_query_type_change(self, model, item):
        (queryType, self._wizard.vehicleDataManager.vehicleData.queryTypeIndex) = UI.get_selected_string_and_index(model)

    def _on_create_clicked(self):
        self._wizard.create_vehicle_command()

    def _on_reset_clicked(self):
        self._wizard.reset_data()
        self._wizard._switch_active_page()

    def _on_next_clicked(self):
        self._wizard.switch_to_next_page()

    def _on_back_clicked(self):
        self._wizard.switch_to_previous_page()
