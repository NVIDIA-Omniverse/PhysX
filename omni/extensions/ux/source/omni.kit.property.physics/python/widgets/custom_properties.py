# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from __future__ import annotations

import carb
import omni.usd

from pxr import Sdf, Usd

from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidget

from .base import PhysicsWidget
from .uiprop import UiProp
from .. import database


class CustomPropertiesWidget(PhysicsWidget):
    """
    A "common widget" that displays externally registered custom properties.

    This widget is keyed by `widget_title_key` (the title used at registration time), and
    reads its properties + visibility predicate from `database.custom_properties`.
    """

    def __init__(self, widget_title: str, widget_title_key: str):
        self._widget_title_key = widget_title_key
        super().__init__(widget_title, Usd.SchemaBase)

    # PhysicsWidget calls refresh_apis() from its __init__ - override to keep it cheap.
    def refresh_apis(self):
        self._all_schema_keys = []
        self._all_schemas = []
        self._single_schema_prop_names_set = set()
        self._single_schema_prop_name_to_api = {}
        self._multi_schema_prop_names = {}

    def _is_main_schema_present(self):
        # This widget is not tied to a USD schema; its visibility is driven by the registered predicate.
        return True

    def _get_def(self):
        return database.get_custom_property_widget(self._widget_title_key)

    def on_new_payload(self, payload):
        # Bypass PhysicsWidget.on_new_payload (schema presence checks) and just use base behavior.
        if not UsdPropertiesWidget.on_new_payload(self, payload):
            return False

        if not self._payload or len(self._payload) == 0:
            return False

        d = self._get_def()
        if d is None:
            return False

        stage = payload.get_stage()
        if stage is None:
            stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False

        visible_when = d.visible_when or (lambda _prim: True)
        for prim_path in payload:
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return False
            try:
                if not visible_when(prim):
                    return False
            except Exception as e:
                carb.log_warn(f"[physx property] Custom widget predicate raised: {e}")
                return False

        return True

    def _filter_props_to_build(self, prim: Usd.Prim):
        d = self._get_def()
        if d is None:
            return []

        def apply_overrides(uip: UiProp, prop_def):
            # Display metadata overrides
            uip.metadata[Sdf.PropertySpec.DisplayNameKey] = prop_def.display_name
            uip.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop_def.display_group
            uip.metadata[Sdf.PropertySpec.DocumentationKey] = prop_def.doc
            uip.display_group = prop_def.display_group

            # Default override: ensure UI control state reflects registered default.
            uip.default = prop_def.default
            uip.metadata[Sdf.AttributeSpec.DefaultValueKey] = prop_def.default
            custom_data = uip.metadata.get(Sdf.PrimSpec.CustomDataKey, {})
            custom_data[Sdf.AttributeSpec.DefaultValueKey] = prop_def.default
            custom_data["default"] = prop_def.default
            uip.metadata[Sdf.PrimSpec.CustomDataKey] = custom_data

        out = []
        for prop_def in database.get_custom_properties(self._widget_title_key):
            # If the attribute exists, use from_property to get the correct runtime type/model,
            # then override UI metadata/default from registration.
            attr = prim.GetAttribute(prop_def.usd_name)
            if attr and attr.IsValid():
                try:
                    uip = UiProp().from_property(attr, base_name=prop_def.usd_name)
                    apply_overrides(uip, prop_def)
                except Exception:
                    uip = UiProp().from_custom(
                        prop_def.usd_name,
                        prop_def.display_name,
                        prop_def.display_group,
                        prop_def.type_name,
                        prop_def.default,
                        prop_def.doc,
                    )
            else:
                uip = UiProp().from_custom(
                    prop_def.usd_name,
                    prop_def.display_name,
                    prop_def.display_group,
                    prop_def.type_name,
                    prop_def.default,
                    prop_def.doc,
                )

            out.append(uip)

        return out

    # Custom widgets don't have component removal.
    def _show_remove_button(self):
        return False

    def is_visible(self):
        # MainFrameWidget enables/disables based on on_new_payload return.
        return True
