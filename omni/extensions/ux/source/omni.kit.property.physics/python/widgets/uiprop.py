# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from ..utils import Limits
from .. import database
from pxr import Sdf, Usd, UsdGeom, UsdPhysics
from omni.kit.property.usd.usd_property_widget import UsdPropertyUiEntry


class UiProp(UsdPropertyUiEntry):
    def __init__(self):
        self.apply_schema = None
        self.apply_to_prims = []
        self.from_spec = False
        self.default = None
        self.step = None
        self.info = None

    def _common(self):
        if self.instance_name is not None:
            self.display_group = self.instance_name

    def gen_multiply_fn(self, multiply_fn_name, modifier):
        # multiply with metersPerUnit / kilogramsPerUnit eq if needed
        mult_func = self.info.__dict__.get(multiply_fn_name)
        if mult_func:
            mult = mult_func(modifier)

            def multiply(val):
                if val is not None and val not in Limits.INFS:
                    return val * mult
                else:
                    return val

            self.default = multiply(self.default)
            self.info_min = multiply(self.info_min)
            self.info_max = multiply(self.info_max)
            self.step = multiply(self.step)

    def finalize(self, stage):
        # info can be defined externally before finalize is called, works as an override
        if not self.info:
            self.info = database.property_info.get(self.base_name)

            # multiapply apis might have infos per instance name
            if isinstance(self.info, dict):
                inst_type = database.property_instance_type[self.instance_name]
                self.info = self.info[inst_type]

        if self.info:
            # get range data
            custom_data = self.metadata.get(Sdf.PrimSpec.CustomDataKey, {})

            self.step = self.info.step
            self.info_min = self.info.min
            self.info_max = self.info.max

            # these multiply functions do not scale the actual values,
            # they only scale things like the default value, the range, and the step

            metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
            self.gen_multiply_fn("multiply_meter_fn", metersPerUnit)

            kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
            self.gen_multiply_fn("multiply_kg_fn", kilogramsPerUnit)

            if self.info_min is not None and self.info_max is not None:
                custom_data["range"] = {"min": self.info_min, "max": self.info_max}

            self.metadata[Sdf.PrimSpec.CustomDataKey] = custom_data

        # FIXME: remove after updating to master with a fix to this
        custom_data = self.metadata.get(Sdf.PrimSpec.CustomDataKey, {})
        custom_data["default"] = self.default
        self.metadata[Sdf.PrimSpec.CustomDataKey] = custom_data

    def from_property(self, prop, base_name=None, instance_name=None, prop_spec=None):
        super().__init__(prop.GetName(), prop.GetDisplayGroup(), prop.GetAllMetadata(), type(prop))
        self.base_name = base_name if base_name else prop.GetName()
        self.is_authored = prop.IsAuthored()
        self.instance_name = instance_name
        prop_spec = prop_spec if prop_spec else prop.GetPrim().GetPrimDefinition().GetSchemaPropertySpec(self.prop_name)
        self.default = prop.GetPrim().GetPrimDefinition().GetSchemaPropertySpec(self.prop_name).default
        self._common()
        return self

    def from_property_spec(self, prop_spec, prim, schema, instance_name=None, multi_prefix=None):
        metadata = {
            prop_spec.DocumentationKey: prop_spec.documentation,
            prop_spec.DisplayNameKey: prop_spec.displayName,
            prop_spec.DisplayGroupKey: prop_spec.displayGroup,
        }

        if isinstance(prop_spec, Sdf.AttributeSpec):
            if len(prop_spec.allowedTokens) > 0:
                metadata["allowedTokens"] = prop_spec.allowedTokens
            metadata[prop_spec.DefaultValueKey] = prop_spec.default
            metadata["typeName"] = str(prop_spec.typeName)
            metadata[prop_spec.CustomDataKey] = {prop_spec.DefaultValueKey: prop_spec.default}

        prop_name = prop_spec.name
        if instance_name is not None:
            prop_name = Usd.SchemaRegistry.MakeMultipleApplyNameInstance(prop_name, instance_name)

        super().__init__(prop_name, prop_spec.displayGroup, metadata, Usd.Attribute)
        self.apply_to_prims = [prim]
        if Usd.SchemaRegistry().IsMultipleApplyAPISchema(schema):
            # prop_spec.name becomes `prefix:__INSTANCE_NAME__:name`
            tokens = prop_spec.name.split(":", maxsplit=2)
            self.base_name = tokens[2]
        else:
            self.base_name = prop_spec.name
        self.is_authored = False
        self.instance_name = instance_name
        self.default = prop_spec.default
        self.apply_schema = schema
        self.from_spec = True
        self._common()
        return self

    def from_custom(self, name, display_name, display_group, type_name, default, doc=''):
        metadata = {
            Sdf.PropertySpec.DisplayNameKey: display_name,
            Sdf.PropertySpec.DisplayGroupKey: display_group,
            Sdf.AttributeSpec.DefaultValueKey: default,
            Sdf.PropertySpec.CustomDataKey: {Sdf.AttributeSpec.DefaultValueKey: default},
            Sdf.PrimSpec.TypeNameKey: type_name,
            Sdf.PropertySpec.DocumentationKey: doc
        }
        super().__init__(name, display_group, metadata, Usd.Attribute)
        self.base_name = name
        self.is_authored = False
        self.instance_name = None
        self.default = default
        return self

    def from_custom_rel(self, name, display_name, display_group, doc=''):
        metadata = {
            Sdf.PropertySpec.DisplayNameKey: display_name,
            Sdf.PropertySpec.DisplayGroupKey: display_group,
            Sdf.PropertySpec.DocumentationKey: doc,
        }
        super().__init__(name, display_group, metadata, Usd.Relationship)
        self.base_name = name
        self.is_authored = False
        self.instance_name = None
        return self

    def __repr__(self):
        return f"UiProp({self.base_name}, {self.instance_name})"
