# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from omni.kit.property.physics.widgets import ExtensionSchemaWidget
from omni.kit.property.physics.utils import enable_widget, add_disabled_styles


class ExtendedPhysxMaterialWidget(ExtensionSchemaWidget):
    def __init__(self, title, schema, parent_schema):
        super().__init__(title, schema, parent_schema)
        self._enabled = False
        self._compliant_damping_widget = None

    def _set_widget_state(self):
        enable_widget(self._compliant_damping_widget, self._enabled)

        if not self._enabled:
            self._compliant_damping_widget.model.set_value(0.0)
            self._compliant_acceleration_spring_widget.set_value(False)

    def build_items(self):
        super().build_items()
        self._set_widget_state()

    def _build_property_item(self, stage, prop, prim_paths):
        def changed(model):
            if self._enabled != model.as_bool:
                self._enabled = model.as_bool
                self._set_widget_state()

        model = super()._build_property_item(stage, prop, prim_paths)
        if prop.base_name == "physxMaterial:compliantContactStiffness":
            model.add_value_changed_fn(changed)
            self._enabled = model.as_bool
        elif prop.base_name == "physxMaterial:compliantContactDamping":
            add_disabled_styles(model.value_widget)
            self._compliant_damping_widget = model.value_widget
        elif prop.base_name == "physxMaterial:compliantContactAccelerationSpring":
            self._compliant_acceleration_spring_widget = model
        return model
