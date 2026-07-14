# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from omni.kit.property.physics import PhysicsWidget
from pxr import PhysxSchema
from omni.kit.property.physics.widgets import UiProp
from omni.kit.property.physics.utils import has_schema


class ExtendedVehicleContextWidget(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        #
        # - hide deprecated "upAxis", "forwardAxis"
        #

        adjusted_filtered_props = []
        for prop in filtered_props:
            if (prop.base_name == "physxVehicleContext:upAxis"):
                pass
            elif (prop.base_name == "physxVehicleContext:forwardAxis"):
                pass
            else:
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props


class ExtendedVehicleWidget(PhysicsWidget):
    def __init__(self, title, schema):
        super().__init__(title, schema)
        self.referenceFrameIsCenterOfMassProp = UiProp().from_custom(
            PhysxSchema.Tokens.referenceFrameIsCenterOfMass,
            "Center Of Mass Frame Is Reference",
            "",
            'bool',
            True,
            """Defines whether some vehicle wheel attachment properties
            are to be interpreted relative to the vehicle prim frame (unchecked)
            or relative to the vehicle center of mass frame (checked).
            The affected properties are: suspensionTravelDirection,
            suspensionFramePosition, suspensionFrameOrientation,
            suspensionForceAppPointOffset, wheelCenterOfMassOffset and
            tireForceAppPointOffset. Note that using the center of mass
            frame as reference (checked) is deprecated and will not be
            supported for much longer (it is currently treated as
            default for backwards compatibility reasons only). Changing
            the value while the simulation is running will have no effect."""
        )

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        #
        # - hide relationship to drive if the prim has the corresponding API applied instead
        # - hide deprecated "minLongitudinalSlipDenominator" attribute
        #

        adjusted_filtered_props = []
        for prop in filtered_props:
            if (prop.base_name == "physxVehicle:drive"):
               if ((not prim.HasAPI(PhysxSchema.PhysxVehicleDriveStandardAPI)) and (not prim.HasAPI(PhysxSchema.PhysxVehicleDriveBasicAPI))):
                    adjusted_filtered_props.append(prop)
            elif (prop.base_name == "physxVehicle:minLongitudinalSlipDenominator"):
                pass
            else:
                adjusted_filtered_props.append(prop)

        adjusted_filtered_props.append(self.referenceFrameIsCenterOfMassProp)
        return adjusted_filtered_props


class ExtendedVehicleWheelWidget(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        #
        # - hide deprecated "maxBrakeTorque", "maxHandBrakeTorque", "maxSteerAngle",
        #   "toeAngle" attributes
        #

        adjusted_filtered_props = []
        for prop in filtered_props:
            if (prop.base_name == "physxVehicleWheel:maxBrakeTorque"):
                pass
            elif (prop.base_name == "physxVehicleWheel:maxHandBrakeTorque"):
                pass
            elif (prop.base_name == "physxVehicleWheel:maxSteerAngle"):
                pass
            elif (prop.base_name == "physxVehicleWheel:toeAngle"):
                pass
            else:
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props


class ExtendedVehicleSuspensionWidget(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        #
        # - hide deprecated "maxCompression", "maxDroop", "camberAtRest",
        #   "camberAtMaxCompression", "camberAtMaxDroop" attributes
        #

        adjusted_filtered_props = []
        for prop in filtered_props:
            if (prop.base_name == "physxVehicleSuspension:maxCompression"):
                pass
            elif (prop.base_name == "physxVehicleSuspension:maxDroop"):
                pass
            elif (prop.base_name == "physxVehicleSuspension:camberAtRest"):
                pass
            elif (prop.base_name == "physxVehicleSuspension:camberAtMaxCompression"):
                pass
            elif (prop.base_name == "physxVehicleSuspension:camberAtMaxDroop"):
                pass
            else:
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props


class ExtendedVehicleTireWidget(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        #
        # - hide deprecated "latStiffX", "latStiffY", "longitudinalStiffnessPerUnitGravity",
        #   "camberStiffnessPerUnitGravity"
        #

        adjusted_filtered_props = []
        for prop in filtered_props:
            if (prop.base_name == "physxVehicleTire:latStiffX"):
                pass
            elif (prop.base_name == "physxVehicleTire:latStiffY"):
                pass
            elif (prop.base_name == "physxVehicleTire:longitudinalStiffnessPerUnitGravity"):
                pass
            elif (prop.base_name == "physxVehicleTire:camberStiffnessPerUnitGravity"):
                pass
            else:
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props


class ExtendedVehicleWheelAttachmentWidget(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        #
        # - hide relationships to wheel/tire/suspension/... if the prim has the corresponding APIs applied instead
        # - hide deprecated "driven", "suspensionForceAppPointOffset", "wheelCenterOfMassOffset",
        #   "tireForceAppPointOffset" attributes
        #

        adjusted_filtered_props = []
        for prop in filtered_props:
            if (prop.base_name == "physxVehicleWheelAttachment:wheel"):
                if (not prim.HasAPI(PhysxSchema.PhysxVehicleWheelAPI)):
                    adjusted_filtered_props.append(prop)
            elif (prop.base_name == "physxVehicleWheelAttachment:tire"):
                if (not prim.HasAPI(PhysxSchema.PhysxVehicleTireAPI)):
                    adjusted_filtered_props.append(prop)
            elif (prop.base_name == "physxVehicleWheelAttachment:suspension"):
                if (not prim.HasAPI(PhysxSchema.PhysxVehicleSuspensionAPI)):
                    adjusted_filtered_props.append(prop)
            elif (prop.base_name == "physxVehicleWheelAttachment:driven"):
                pass
            elif (prop.base_name == "physxVehicleWheelAttachment:suspensionForceAppPointOffset"):
                pass
            elif (prop.base_name == "physxVehicleWheelAttachment:wheelCenterOfMassOffset"):
                pass
            elif (prop.base_name == "physxVehicleWheelAttachment:tireForceAppPointOffset"):
                pass
            else:
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props


class ExtendedVehicleDriveStandardWidget(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        #
        # hide relationships to engine/gears/autoGearBox/clutch/... if the prim has the corresponding APIs applied instead
        #

        adjusted_filtered_props = []
        for prop in filtered_props:
            if (prop.base_name == "physxVehicleDriveStandard:engine"):
                if (not prim.HasAPI(PhysxSchema.PhysxVehicleEngineAPI)):
                    adjusted_filtered_props.append(prop)
            elif (prop.base_name == "physxVehicleDriveStandard:gears"):
                if (not prim.HasAPI(PhysxSchema.PhysxVehicleGearsAPI)):
                    adjusted_filtered_props.append(prop)
            elif (prop.base_name == "physxVehicleDriveStandard:autoGearBox"):
                if (not prim.HasAPI(PhysxSchema.PhysxVehicleAutoGearBoxAPI)):
                    adjusted_filtered_props.append(prop)
            elif (prop.base_name == "physxVehicleDriveStandard:clutch"):
                if (not prim.HasAPI(PhysxSchema.PhysxVehicleClutchAPI)):
                    adjusted_filtered_props.append(prop)
            else:
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props


class ExtendedVehicleMultiWheelDifferentialWidget(PhysicsWidget):

    def on_new_payload(self, payload):
        if not super().on_new_payload(payload):
            return False

        #
        # do not show if the prim has an API that inherits from this one.
        #
        # note: omitting most safety checks as the super class should handle it.
        #

        for prim_path in self._payload:
            prim = self._get_prim(prim_path)

            if (not has_schema(prim, PhysxSchema.PhysxVehicleTankDifferentialAPI)):
                return True

        return False


class ExtendedVehicleControllerBaseWidget(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        #
        # - hide deprecated "brake", "handbrake", "steerLeft", "steerRight" attributes
        #

        adjusted_filtered_props = []
        for prop in filtered_props:
            if (prop.base_name == "physxVehicleController:brake"):
               pass
            elif (prop.base_name == "physxVehicleController:handbrake"):
               pass
            elif (prop.base_name == "physxVehicleController:steerLeft"):
               pass
            elif (prop.base_name == "physxVehicleController:steerRight"):
               pass
            else:
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props


class ExtendedVehicleControllerWidget(ExtendedVehicleControllerBaseWidget):

    def on_new_payload(self, payload):
        if not super().on_new_payload(payload):
            return False

        #
        # do not show if the prim has an API that inherits from this one.
        #
        # note: omitting most safety checks as the super class should handle it.
        #

        for prim_path in self._payload:
            prim = self._get_prim(prim_path)

            if (not has_schema(prim, PhysxSchema.PhysxVehicleTankControllerAPI)):
                return True

        return False
