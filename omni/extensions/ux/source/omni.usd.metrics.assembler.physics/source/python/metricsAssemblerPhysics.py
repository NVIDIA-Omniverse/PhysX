# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import asyncio

import carb.settings
from carb import log_warn
from omni.metrics.assembler.core import get_metrics_assembler_interface
from pxr import Usd, UsdUtils
from pxr.PhysicsSchemaTools import units
from pxr.PhysicsSchemaTools.units_db import usd_attribute_units

class MetricsAssemblerPhysics:
    def __init__(self):
        self._custom_rules = []
        self._attributes_with_conditional_units = []

        self.initialize_rules()
        self.initialize_custom_callback()

    def add_rule(self, token, metersExponent, kilogramExponent):
        id = get_metrics_assembler_interface().register_custom_resolve_rule(token, metersExponent, kilogramExponent)
        self._custom_rules.append(id)

    def initialize_rules(self):
        for attribute_name, attribute_units in usd_attribute_units.items():
            if type(attribute_units) is dict:
                self._attributes_with_conditional_units.append(attribute_name)
                continue
            distance_exponent = attribute_units[units.UnitParameters.DISTANCE]
            mass_exponent = attribute_units[units.UnitParameters.MASS]
            if distance_exponent == 0 and mass_exponent == 0:
                continue
            self.add_rule(attribute_name, distance_exponent, mass_exponent)

    def initialize_custom_callback(self):
        def resolve_start(stageId):
            self._resolve_stage_id = stageId
            self._stage = UsdUtils.StageCache.Get().Find(Usd.StageCache.Id.FromLongInt(stageId))
            self._is_omni_physx_ui_enabled = True
            try:
                import omni.physx.bindings._physx as physx_bindings
                from omni.physxui import get_physxui_interface

                settings = carb.settings.acquire_settings_interface()
                self._saved_display_joints = settings.get_as_bool(physx_bindings.SETTING_DISPLAY_JOINTS)
                settings.set(physx_bindings.SETTING_DISPLAY_JOINTS, False)

                self._is_omni_physx_ui_enabled = get_physxui_interface().is_usd_notice_handler_enabled()
                get_physxui_interface().block_usd_notice_handler(True)
            except:
                pass

        def resolve_end(stageId):
            self._resolve_stage_id = 0
            self._stage = None
            try:
                import omni.physx.bindings._physx as physx_bindings
                from omni.physxui import get_physxui_interface

                settings = carb.settings.acquire_settings_interface()
                settings.set(physx_bindings.SETTING_DISPLAY_JOINTS, self._saved_display_joints)

                if self._is_omni_physx_ui_enabled:
                    get_physxui_interface().block_usd_notice_handler(False)
            except:
                pass

        def resolve_attribute(usdPrimPath, attributeName, stageUnits, resolveUnits, metersExponent, kilogramsExponent):
            meters_exponent_out = metersExponent
            kilograms_exponent_out = kilogramsExponent

            usd_prim = self._stage.GetPrimAtPath(usdPrimPath)
            exponents = units.get_attribute_unit_exponents(attributeName, usd_prim)
            if exponents is not None and type(exponents) is not dict:
                meters_exponent_out = exponents[units.UnitParameters.DISTANCE]
                kilograms_exponent_out = exponents[units.UnitParameters.MASS]
            else:
                log_warn(f"Failed to process attribute {usdPrimPath}:{attributeName}")
            return (True, meters_exponent_out, kilograms_exponent_out)

        self._reg_id = get_metrics_assembler_interface().register_custom_callback(
            resolve_start, resolve_end, None, resolve_attribute, self._attributes_with_conditional_units
        )

    def cleanup_rules(self):
        for id in self._custom_rules:
            get_metrics_assembler_interface().unregister_custom_resolve_rule(id)

        self._custom_rules = []

    def on_shutdown(self):
        self.cleanup_rules()
        get_metrics_assembler_interface().unregister_custom_callback(self._reg_id)
        self._attributes_with_conditional_units = []
