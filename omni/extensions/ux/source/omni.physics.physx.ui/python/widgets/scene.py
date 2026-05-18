# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from omni.kit.property.physics.widgets import ExtensionSchemaWidget, UiProp
from omni.kit.property.physics.utils import enable_widget, add_disabled_styles


class ExtendedPhysxSceneWidget(ExtensionSchemaWidget):
    def __init__(self, title, schema, parent_schema):
        super().__init__(title, schema, parent_schema)
        self._disabled = False
        self._bt_widget = None
        self._cs_widget = None

    def _set_widget_state(self):
        enable_widget(self._bt_widget, not self._disabled)
        enable_widget(self._cs_widget, not self._disabled)

        if self._disabled:
            self._bt_widget.model.set_value("GPU")
            self._cs_widget.model.set_value("PCM")

    def build_items(self):
        super().build_items()
        if self._bt_widget and self._cs_widget:
            self._set_widget_state()

    def _build_property_item(self, stage, prop, prim_paths):
        def changed(model):
            if self._bt_widget and self._cs_widget:
                if self._disabled != model.as_bool:
                    self._disabled = model.as_bool
                    self._set_widget_state()

        model = super()._build_property_item(stage, prop, prim_paths)
        if prop.base_name == "physxScene:enableGPUDynamics":
            model.add_value_changed_fn(changed)
            self._disabled = model.as_bool
        elif prop.base_name == "physxScene:broadphaseType":
            add_disabled_styles(model.value_widget)
            self._bt_widget = model.value_widget
        elif prop.base_name == "physxScene:collisionSystem":
            add_disabled_styles(model.value_widget)
            self._cs_widget = model.value_widget
        return model

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        filtered_props.append(UiProp().from_custom("physxScene:envIdInBoundsBitCount", "Number of bits used for EnvIDs in bounds", "Advanced", 'int', -1))

        return filtered_props
