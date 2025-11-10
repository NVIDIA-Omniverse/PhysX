# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui

from pxr import Usd, UsdGeom, Gf

from .. helpers import UI

from . import vehicleWizardCommon as Common
from . import physxVehicleWizard as Wizard


class VehicleWizardNextSteps():
    def __init__(self, wizard):
        self._wizard = wizard
        
        self._columnWidth = omni.ui.Percent(100)

    def tear_down(self):
        self._createButton = None
        self._resetButton = None
        self._backButton = None
        self._nextButton = None


    def set_up(self):

        labelTooltipWidth = Common.DEFAULT_LABEL_TOOLTIP_WIDTH
        defaultWidgetWidthList = Common.DEFAULT_WIDGET_WIDTH_LIST
        
        usdContext = omni.usd.get_context()
        self._stage = usdContext.get_stage()

        metersPerUnit = UsdGeom.GetStageMetersPerUnit(self._stage)
        lengthScale = 1.0 / metersPerUnit

        with self._wizard._window.frame:
            with omni.ui.ScrollingFrame(horizontal_scrollbar_policy=omni.ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=omni.ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED):
                with omni.ui.VStack(style = UI.STYLE_REFERENCE):
                    with omni.ui.VStack(height = Wizard.CONTENT_HEIGHT_WITHOUT_FOOTER):
                        with omni.ui.ScrollingFrame(horizontal_scrollbar_policy=omni.ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                            vertical_scrollbar_policy=omni.ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED):
                            with omni.ui.VStack(spacing = UI.DEFAULT_WINDOW_SPACING_V):
                                with omni.ui.CollapsableFrame("Next Steps", height = 0):
                                    layout = omni.ui.VStack(spacing = UI.DEFAULT_SPACING_V,
                                        height = Wizard.CONTENT_HEIGHT_WITHOUT_FOOTER)
                                    # the height is being set here, as the UI does not seem to sum up the heights
                                    # of the widgets properly and the text extends beyond the collapsible frame
                                    # border

                                    with layout:
                                        labelText = ("Animate the vehicle's rendered mesh (if the render and simulation prim hierarchy are separate):")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("1. Load your vehicle mesh into the current layer.")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("2. Child the chassis mesh under the WizardVehicle/Vehicle prim.")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("3. Child the tires under the appropriate WizardVehicle/Vehicle/__WheelReferences.")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("4. The ChassisRender and tire Render prims can be deleted or hidden when they "
                                            + "are no longer needed. They are only created to help visualize the physics vehicle.")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        omni.ui.Spacer(height = omni.ui.Pixel(UI.DEFAULT_SPACING_V))

                                        labelText = ("Add ground collision:")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("1. Load a collision plane, mesh or height field to drive on. Try "
                                            + "Create > Physics > Ground Plane to start.")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("2. If collision objects are created for the tires, ensure the ground collision "
                                            + "object is a member of the WizardSharedVehicleData/GroundSurfaceCollisionGroup.")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("3. Set the ground collision object physics materal. A default "
                                            + "WizardSharedVehicleData/VehicleGroundMaterial is provided.")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("4. Custom physics materials need to be added to the tire friction tables which are "
                                            + "used for driving on the corresponding surfaces.")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        omni.ui.Spacer(height = omni.ui.Pixel(UI.DEFAULT_SPACING_V))

                                        labelText = ("Tune and customize the vehicle physics:")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("1. Tune suspension settings in the Property panel after selecting the "
                                            + "WizardVehicle/Vehicle/__WheelReferences or WizardVehicle/Suspension prims.")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("2. Tune engine, clutch and gear ratio settings in the Property panel after selecting the "
                                            + "WizardVehicle/Vehicle or WizardVehicle/Engine, /Clutch, /Gears and /AutoGearBox prims.")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        omni.ui.Spacer(height = omni.ui.Pixel(UI.DEFAULT_SPACING_V))

                                        labelText = ("Additional documentation:")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

                                        labelText = ("https://developer.nvidia.com/nvidia-omniverse-platform")
                                        label = omni.ui.Label(labelText, word_wrap = True, width = self._columnWidth)

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

                        self._resetButton.enabled = False
                        self._nextButton.enabled = False

    def _on_create_clicked(self):
        self._wizard.create_vehicle_command()

    def _on_reset_clicked(self):
        return

    def _on_next_clicked(self):
        return

    def _on_back_clicked(self):
        self._wizard.switch_to_previous_page()
