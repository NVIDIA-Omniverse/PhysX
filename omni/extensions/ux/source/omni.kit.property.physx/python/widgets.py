# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from collections import defaultdict, namedtuple
from omni.kit.property.usd.usd_model_base import UsdBase
from pxr import UsdPhysics, Usd, Sdf, UsdShade, UsdGeom, PhysxSchema, Gf, Tf, PhysicsSchemaTools
from pxr.PhysicsSchemaTools import units
from pxr.Sdf import Path as SdfPath
from pxr.Sdf import PropertySpec as Spec
import omni.usd
import omni.ui as ui
import omni.kit.undo
import omni.physx.bindings._physx as pxb
from omni.physx import get_physx_interface
from omni.physx import get_physx_attachment_private_interface
from omni.physx.bindings._physx import SimulationEvent
from omni.kit.property.usd import PrimPathWidget
from omni.kit.property.usd.widgets import ICON_PATH
from omni.kit.window.property.templates import SimplePropertyWidget, build_frame_header, HORIZONTAL_SPACING
from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidget, UsdPropertyUiEntry
from omni.kit.property.usd.usd_property_widget_builder import UsdPropertiesWidgetBuilder
from omni.kit.property.usd.usd_attribute_model import GfVecAttributeModel, TfTokenAttributeModel
from omni.kit.property.material.scripts.usd_binding_widget import UsdBindingAttributeWidget
from omni.kit.material.library import MaterialUtils
from omni.kit.commands import execute
from omni.physx.scripts import utils
from omni.physx.scripts.utils import get_schema_instances, get_TfType_compatible
from omni.physxui import get_physxui_interface
from . import database
from . import databaseUtils as dbutils
from .builders import GearingWidgetBuilder, CustomTokenComboBuilder
from .utils import split_at_capitals, StringPrompt, OverlayButton, enable_widget, add_disabled_styles, Limits
from .utils import get_align_property_util, is_parent_api_instance, get_schema_name
from .externals import Prompt
from functools import partial
from sys import maxsize
from pathlib import Path
import asyncio
import math
import enum
import carb.settings
from omni.physx.bindings._physx import (
    MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTX,
    MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTY,
    MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTZ,
    MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTX,
    MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTY,
    MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTZ)

ADD_GLYPH = omni.kit.ui.get_custom_glyph_code("${glyphs}/menu_context.svg")
EYE_GLYPH = omni.kit.ui.get_custom_glyph_code("${glyphs}/eye.svg")
REFRESH_GLYPH = omni.kit.ui.get_custom_glyph_code("${glyphs}/menu_refresh.svg")
REMOVE_BUTTON_STYLE = style = {"image_url": str(Path(ICON_PATH).joinpath("remove.svg")), "margin": 0, "padding": 0}


def sort_props(props, schemas):
    order = []

    def get_order(schema):
        custom_order = database.custom_order_map.get(schema)
        if custom_order is not None:
            return custom_order

        default_order = database.default_order_map.get(schema)
        if default_order is not None:
            return default_order

        return []

    for schema in schemas:
        order += get_order(schema)

    indices = {name: pos for pos, name in enumerate(order)}
    props = sorted(props, key=lambda i: indices.get(i.base_name, maxsize))
    return props


def display_name_from_name(name):
    lower_split = [s.lower() for s in split_at_capitals(name)]
    exceptions = ["as", "of", "a", "in", "from"]
    capitalized = [s[0].upper() + s[1:] if s not in exceptions else s for s in lower_split]
    return " ".join(capitalized)


def display_name_from_prop_name(prop):
    return display_name_from_name(prop.prop_name.split(":")[-1])


def is_property_in_schema(schema_prop_names, instance_name, prop_path, api_path_func):
    instance_seg = prop_path.find(":" + instance_name + ":")
    if instance_seg != -1:
        api_path = prop_path[0:instance_seg + 1 + len(instance_name)]
        if api_path_func(api_path):
            base_name = prop_path[instance_seg + 1 + len(instance_name) + 1:]
            if base_name in schema_prop_names:
                return base_name
    return None


def has_schema(prim, schema):
    is_api_schema = Usd.SchemaRegistry().IsAppliedAPISchema(schema)
    return is_api_schema and prim.HasAPI(schema) or not is_api_schema and prim.IsA(schema)


def is_multi_api(schema):
    return Usd.SchemaRegistry().IsMultipleApplyAPISchema(schema)


def filter_property_with_multiapply_schema(prop, schema, prop_names):
    prim = prop.GetPrim()
    schema_type_name = Usd.SchemaRegistry().GetSchemaTypeName(schema)
    schema_instances = get_schema_instances(prim, schema_type_name)

    #hack for codeless schema
    def codeless_api_path_func(path):
        return True

    if isinstance(schema, Tf.Type):
        api_path_func = codeless_api_path_func
    else:
        api_path_func = getattr(schema, f"Is{schema_type_name}Path")

    prop_path = prop.GetPath().pathString
    for instance_name in schema_instances:
        base_name = is_property_in_schema(prop_names, instance_name, prop_path, api_path_func)
        if base_name:
            return UiProp().from_property(prop, base_name, instance_name)

    return None


def generate_prims(payload):
    stage = omni.usd.get_context().get_stage()
    for path in payload:
        prim = stage.GetPrimAtPath(path)
        if prim:
            yield prim


def generate_prims_with_cache(payload):
    lmt = round(carb.settings.get_settings().get(pxb.SETTING_ADDMENU_SUBTREE_LIMIT) / len(payload))

    stage = omni.usd.get_context().get_stage()
    for path in payload:
        prim = stage.GetPrimAtPath(path)
        if prim:
            dbutils.patch_refresh_cache_to_prim(prim, lmt)
            yield prim


def generate_from_prim_list(stage, prim_list):
    for item in prim_list:
        if isinstance(item, Usd.Prim):
            yield item
        else:
            yield stage.GetPrimAtPath(item)


def generate_from_prim_list_with_cache(stage, prim_list):
    lmt = round(carb.settings.get_settings().get(pxb.SETTING_ADDMENU_SUBTREE_LIMIT) / len(prim_list))

    for item in prim_list:
        if isinstance(item, Usd.Prim):
            dbutils.patch_refresh_cache_to_prim(item, lmt)
            yield item
        else:
            prim = stage.GetPrimAtPath(item)
            if prim:
                dbutils.patch_refresh_cache_to_prim(prim, lmt)
                yield prim


def get_components(payload, schema=None):
    components = []

    prims = [prim for prim in generate_prims_with_cache(payload)]

    for c in database.components.values():
        if schema is not None and get_TfType_compatible(c.main_schema) != schema:
            continue

        present = [c.is_present(prim) for prim in prims]
        if not any(present) and all([c.can_add(prim) for prim in prims]):
            components.append(c)

    return components


def add_component(component, payload):
    omni.kit.undo.begin_group()

    for prim in generate_prims_with_cache(payload):
        if component.is_present(prim) or not component.can_add(prim) or (component.can_show and not component.can_show(prim)):
            continue

        named_api = database.named_apis.get(component.name)
        if named_api is not None:
            def on_save(name: str = None):
                if component.add_component_fn is not None:
                    component.add_component_fn(usd_prim=prim, component=component.name)
                else:
                    execute("AddPhysicsComponent", usd_prim=prim, component=component.name, multiple_api_token=name)

                # auto-select attachment if user is creating an attachment
                if component.name in [x.__name__ for x in database.attachment_apis]:
                    get_physxui_interface().select_spatial_tendon_attachment_helper(prim.GetPath().pathString, name)

            prompt = StringPrompt(named_api.title, named_api.text, "Add", cancel_button_text="Cancel", ok_button_fn=on_save)
            prompt.show()
        else:
            if component.add_component_fn is not None:
                component.add_component_fn(usd_prim=prim, component=component.name)
            else:
                execute("AddPhysicsComponent", usd_prim=prim, component=component.name)

    omni.kit.undo.end_group()


def remove_component(component, payload, instance_name: str = None):
    omni.kit.undo.begin_group()

    for prim in generate_prims(payload):
        if component.remove_component_fn is not None:
            component.remove_component_fn(usd_prim=prim, component=component.name, multiple_instance_name=instance_name)
        else:
            execute("RemovePhysicsComponent", usd_prim=prim, component=component.name, multiple_api_token=instance_name)

    omni.kit.undo.end_group()


context_menu = ui.Menu("Context menu", name="physx_context_menu")


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

    def __repr__(self):
        return f"UiProp({self.base_name}, {self.instance_name})"


class MainFrameWidget(SimplePropertyWidget):
    name = "physx_main_frame_widget"
    instance = None
    popup_populate_fns = {}

    class ExtWidget:
        def __init__(self, widget):
            self.enabled = False
            self.widget = widget

    def __init__(self, subwidgets):
        super().__init__("Physics")

        self._base_subwidgets = [wdg[1](*wdg[2:]) for wdg in subwidgets]
        self._base_enabled = []
        self._ext_subwidgets = {}
        self._any_visible = False
        self._refresh_enabled = False
        MainFrameWidget.instance = self
        self._add_popup_menu()

        # shared checks
        self.is_a_omnijoint = False

    def clean(self):
        super().clean()
        self._popup_menu = None
        self._ext_popup_menus = {}
        MainFrameWidget.instance = None

    def get_subwidget_by_title(self, title):
        for widget in self._base_subwidgets:
            if widget._title == title:
                return widget
        return None

    def _add_popup_menu(self):
        def get_base_name(args):
            paths = args.get("attribute_paths")
            if paths is None or len(paths) == 0:
                return

            return str(paths[0]).split(".")[-1]

        def populate_fn(args):
            current = get_base_name(args)
            populate_cb = MainFrameWidget.popup_populate_fns.get(current)
            if populate_cb:
                populate_cb()

        menu = {
            "name": "PhysicsAttributMenu",
            "populate_fn": populate_fn,
        }

        self._popup_menu = omni.kit.context_menu.add_menu(menu, "attribute", "omni.kit.property.usd")
        MainFrameWidget.popup_populate_fns = {}

    @staticmethod
    def register_popup_menu_populate_fn(base_name, populate_fn):
        MainFrameWidget.popup_populate_fns[base_name] = populate_fn

    def build_items(self):
        if self._refresh_enabled:
            # refresh enabled data before rebuilding, visibility might have changed
            self.on_new_payload(self._payload)

        self._any_item_visible = self._any_visible

        if not self._any_visible:
            return

        for e, w in zip(self._base_enabled, self._base_subwidgets):
            if e:
                w._filter = self._filter
                w.build_impl()

        for ext in self._ext_subwidgets.values():
            if ext.enabled:
                ext.widget._filter = self._filter
                ext.widget.build_impl()

        self._collapsable_frame.visible = True
        self._collapsable_frame.name = "groupFrame"

    def _build_frame(self):
        super()._build_frame()

        if not self._any_visible:
            self._collapsable_frame.visible = False

    def request_rebuild(self):
        if self._collapsable_frame is not None:
            self._collapsable_frame.visible = True
        self._refresh_enabled = True
        super().request_rebuild()

    def is_a_omnijoint_check(self, payload):
        stage = payload.get_stage()
        for prim_path in payload:
            prim = stage.GetPrimAtPath(prim_path)
            if prim.GetTypeName() != "OmniJoint":
                return False
        return True

    def on_new_payload(self, payload):
        self._any_visible = False
        self._refresh_enabled = False
        MainFrameWidget.popup_populate_fns = {}

        if not super().on_new_payload(payload):
            return False

        self._base_enabled = [w.on_new_payload(payload) for w in self._base_subwidgets]

        ext_enabled = False
        for ext in self._ext_subwidgets.values():
            ext.enabled = ext.widget.on_new_payload(payload)
            ext_enabled |= ext.enabled

        self._any_visible = any(self._base_enabled) or ext_enabled

        # do shared checks
        if self._any_visible:
            self.is_a_omnijoint = self.is_a_omnijoint_check(payload)

        # we want to be there always so on change we can just rebuild this widget instead
        # of the whole property window
        return True

    def register_widget(self, name, widget):
        if name in self._ext_subwidgets:
            carb.log_error(f"'{name}' widget is already registered in the Physics property window frame. Please fix ASAP, your unregister will remove another widget!")
        else:
            self._ext_subwidgets[name] = MainFrameWidget.ExtWidget(widget)

    def unregister_widget(self, name):
        self._ext_subwidgets.pop(name)


class PhysicsWidget(UsdPropertiesWidget):
    def __init__(self, title, schema, builders):
        UsdPropertiesWidget.__init__(self, title, False)

        self._builders = builders
        self._main_schema = get_TfType_compatible(schema)
        self._is_main_multi = is_multi_api(self._main_schema)
        ext_apis = database.extension_api_map.get(schema, [])

        self._ext_apis = [get_TfType_compatible(schema) for schema in ext_apis]
        self._all_schema_keys = [schema] + database.widget_extension_map.get(schema, [])
        self._all_schemas = [get_TfType_compatible(schema) for schema in self._all_schema_keys]

        self._builders = builders
        self._button_frame = None
        self._allow_refresh = True
        self._property_utils = {}

        self._single_schema_prop_names_set = set()
        self._single_schema_prop_name_to_api = {}
        self._multi_schema_prop_names = {}
        self._ext_prop_specs = {}

        for schema in self._all_schemas:
            if is_multi_api(schema):
                self._multi_schema_prop_names[schema] = utils.getSchemaPropertyNames(schema)
            else:
                names = utils.getSchemaPropertyNames(schema)
                self._single_schema_prop_names_set.update(names)
                self._single_schema_prop_name_to_api.update({n: schema for n in names})

        for api in self._ext_apis:
            prim_def = utils.getSchemaPrimDef(api)
            self._ext_prop_specs[api] = [prim_def.GetSchemaPropertySpec(name) for name in prim_def.GetPropertyNames()]

    def clean(self):
        super().clean()
        self._button_frame = None

    def request_rebuild(self):
        if self._allow_refresh:
            super().request_rebuild()

    def _is_main_schema_present(self):
        if len(self._payload) == 0:
            return False

        for prim_path in self._payload:
            prim = self._get_prim(prim_path)

            if not prim or not has_schema(prim, self._main_schema):
                return False

        if self._is_main_multi:
            instances = get_schema_instances(prim, Usd.SchemaRegistry().GetSchemaTypeName(self._main_schema))
            if not len(instances):
                return False

        return True

    def _on_usd_changed(self, notice, stage):
        # If the stage unit parameters change, it will trigger a notice for the absoluteRootPath.
        if SdfPath.absoluteRootPath in notice.GetChangedInfoOnlyPaths() and stage == self._units_stage:
            stage_kilograms_per_unit = UsdPhysics.GetStageKilogramsPerUnit(stage)
            stage_meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
            if self._stage_kilograms_per_unit != stage_kilograms_per_unit or self._stage_meters_per_unit != stage_meters_per_unit:
                self._stage_kilograms_per_unit = stage_kilograms_per_unit
                self._stage_meters_per_unit = stage_meters_per_unit
                self.request_rebuild()

        super()._on_usd_changed(notice, stage)

    def on_new_payload(self, payload):
        if not UsdPropertiesWidget.on_new_payload(self, payload):
            return False

        if not self._payload or len(self._payload) == 0:
            return False

        return self._is_main_schema_present()

    def build_items(self):
        stage = omni.usd.get_context().get_stage()

        if stage:
            self._units_stage = stage
            self._stage_kilograms_per_unit = UsdPhysics.GetStageKilogramsPerUnit(stage)
            self._stage_meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)

        super().build_items()

    def _customize_props_layout(self, props):
        return sort_props(props, self._all_schema_keys)

    def _get_shared_properties_from_selected_prims(self, anchor_prim):
        shared_props_dict = None
        for prim_path in self._payload:
            prim = self._get_prim(prim_path)
            props = self._filter_props_to_build(prim)
            prop_dict = {prop.prop_name: prop for prop in props}
            if shared_props_dict is None:
                shared_props_dict = prop_dict
            else:
                intersect_shared_props = {}
                for prop_name, prop_info in shared_props_dict.items():
                    prop = prop_dict.get(prop_name)
                    if prop is not None:
                        if prop.from_spec:
                            prop_info.apply_schema = prop.apply_schema
                            prop_info.apply_to_prims += prop.apply_to_prims
                        intersect_shared_props[prop_name] = prop_info
                if len(intersect_shared_props) == 0:
                    return

                shared_props_dict = intersect_shared_props
        shared_props = list(shared_props_dict.values())
        return shared_props

    def _filter_props_to_build(self, prim):
        filtered_props = []

        # prims and applied apis
        for prop in prim.GetProperties():
            prop_name = prop.GetName()
            prop_prim = prop.GetPrim()
            if prop_name in self._single_schema_prop_names_set:
                # basically a HasAPI check, but cheaper and needs to be got to create the prop anyway
                prop_spec = prop_prim.GetPrimDefinition().GetSchemaPropertySpec(prop_name)
                if prop_spec is None:
                    def get_name():
                        name = self._single_schema_prop_name_to_api.get(prop_name)
                        return f"a {get_schema_name(name)}" if name else "an unknown API"
                    carb.log_warn(f"{prop_prim.GetPath().pathString}:{prop_name} is authored, but {get_name()} is not applied. Simulation will default to a fallback value. Edit this value in the Property window to autoapply the correct API or manually edit the 'apiSchemas' list!\n")
                else:
                    filtered_props.append(UiProp().from_property(prop, prop_spec=prop_spec))
            else:
                for schema, prop_names in self._multi_schema_prop_names.items():
                    uiprop = filter_property_with_multiapply_schema(prop, schema, prop_names)
                    if uiprop is not None:
                        filtered_props.append(uiprop)

        # generate non-applied extension api properties from spec
        for api in self._ext_apis:
            condition = database.extension_api_conditions.get(api)
            if condition is not None:
                if not all([condition(prim) for prim in generate_prims(self._payload)]):
                    continue

            if is_multi_api(self._main_schema):
                main_schema_type_name = Usd.SchemaRegistry().GetSchemaTypeName(self._main_schema)
                ext_schema_type_name = Usd.SchemaRegistry().GetSchemaTypeName(api)

                main_instances = set(get_schema_instances(prim, main_schema_type_name))
                ext_instances = set(get_schema_instances(prim, ext_schema_type_name))
                instances_not_applied = main_instances - ext_instances

                prefix = database.ext_multi_api_prefixes[api]

                for instance in instances_not_applied:
                    instance_props = [UiProp().from_property_spec(prop_spec, prim, api, instance, prefix) for prop_spec in self._ext_prop_specs[api]]
                    filtered_props += instance_props
            else:
                if not prim.HasAPI(api):
                    for prop_spec in self._ext_prop_specs[api]:
                        prop = UiProp().from_property_spec(prop_spec, prim, api)
                        filtered_props.append(prop)

        return filtered_props

    def _remove_prompt_check(self, title, on_yes):
        prompt = Prompt(
            "Remove component?", f"Are you sure you want to remove the '{title}' component?", "Yes", "No",
            ok_button_fn=on_yes, modal=True)
        prompt.show()

    def _show_remove_button(self):
        return True

    def _build_impl_with_remove_button(self, component):
        # behold the greatness of my omni.ui-fu! yeah, just kidding, items in a zstack (or wherever, but
        # it's important here) cannot stop mouse events going through the whole stack so I have to revert
        # the underlying frame's collapse when the remove button is clicked ...
        revert_last = False

        def on_collapsed_changed(collapsed):
            nonlocal revert_last
            if revert_last and collapsed:
                self._collapsable_frame.collapsed = False
            revert_last = False

        def on_remove_clicked_mouse(*_):
            def on_yes():
                nonlocal revert_last
                revert_last = True
                remove_component(component, self._payload)

            self._remove_prompt_check(component.title, on_yes)

        self._button_frame = ui.Frame()
        with self._button_frame:
            with ui.ZStack():
                super().build_impl()
                with ui.HStack():
                    ui.Spacer(width=ui.Fraction(0.5))
                    with ui.VStack(width=0):
                        ui.Spacer(height=5)
                        ui.Button(style=REMOVE_BUTTON_STYLE, height=16, width=16, identifier=f"remove_component {component.title}").set_mouse_pressed_fn(on_remove_clicked_mouse)
                    ui.Spacer(width=5)
        self._collapsable_frame.set_collapsed_changed_fn(on_collapsed_changed)

    def build_impl(self):
        self._button_frame = None
        component = database.components.get(get_schema_name(self._main_schema))
        if component is not None and self._show_remove_button():
            self._build_impl_with_remove_button(component)
        else:
            super().build_impl()

    def is_visible(self):
        return self._is_main_schema_present()

    def show_frame(self, show):
        self._collapsable_frame.visible = show
        if self._button_frame is not None:
            self._button_frame.visible = show

    def _build_frame(self):
        # need to double check here - frame can be refreshed on underlying USD change
        # or a change might have happened between payload check and UI build
        if self._is_main_schema_present():
            super()._build_frame()
            self.show_frame(self._any_item_visible)
        else:
            ui.VStack(height=0)
            self.show_frame(False)

    def build_property_item(self, stage, prop, prim_paths):
        def precompute_menu():
            sentinels_cpy = {k: v for k, v in database.property_sentinels.get(prop.base_name, {}).items()}
            sentinels_inv = {v: k for k, v in sentinels_cpy.items()}
            property_utils = self._property_utils.get(prop.base_name, {})
            items = []

            def add_base(name, value):
                sentinel_name = sentinels_inv.get(value)
                if sentinel_name is not None:
                    items.append((f"{name} ({sentinel_name})", value))
                else:
                    items.append((name, value))

            if prop.info is not None:
                add_base("Set Minimum", prop.info.min)
                add_base("Set Maximum", prop.info.max)
            if len(sentinels_cpy) != 0 and prop.info is not None:
                items.append(("", None))
            for name, value in sentinels_cpy.items():
                items.append((f"Set {name}", value))
            for name, value in property_utils.items():
                items.append((f"{name}", value))

            return items

        def populate_menu(model, menu_items):
            def set_value(model, value):
                # remove property when the value is from an N/A sentinel
                if value == database.not_authored_token:
                    for path in model._object_paths:
                        prim = model._stage.GetPrimAtPath(path.GetPrimPath())
                        prim.RemoveProperty(prop.base_name)
                    return

                # temporarily disable limits to set sentinels outside of that range
                disable_limits = isinstance(model, UsdBase)
                if disable_limits:
                    tmp_min, tmp_max = model._min, model._max
                    model._min, model._max = None, None

                # set value either in a singlechannel or a multichannel mode
                if isinstance(model, GfVecAttributeModel):
                    for i, item in enumerate(model._items):
                        item.model.set_value(value[i])
                    model.set_value(value)
                else:
                    model.set_value(value)

                if disable_limits:
                    model._min, model._max = tmp_min, tmp_max

            ui.Separator()

            for item in menu_items:
                n, v = item
                if n == "":
                    ui.Separator()
                else:
                    ui.MenuItem(n).set_triggered_fn(lambda v=v: v() if callable(v) else set_value(model, v))

        model = self._build_property_item(stage, prop, prim_paths)
        menu_items = precompute_menu()
        if menu_items:
            MainFrameWidget.instance.register_popup_menu_populate_fn(prop.base_name, partial(populate_menu, model, menu_items))

    def get_default_label_kwargs(self):
        label_kwargs = {}
        if self._filter.name:
            label_kwargs["highlight"] = self._filter.name
        return label_kwargs

    def _build_property_item(self, stage, prop, prim_paths):
        prop.finalize(stage)

        if not prop.metadata.get(Spec.DisplayNameKey):
            prop.override_display_name(display_name_from_prop_name(prop))

        label_kwargs = self.get_default_label_kwargs()
        model = None

        def build_widget():
            # create widget
            available_builders = [val for key, val in self._builders.items() if prop.prop_name.endswith(key)]
            if len(available_builders) != 0:
                builder_data = available_builders[0]
                models = builder_data[0](stage, prop, prim_paths, label_kwargs, None, *builder_data[1:])
            else:
                widget_kwargs = {"step": prop.step} if prop.step is not None else {}
                if UsdPropertiesWidgetBuilder._get_type_name(prop.metadata).type == Sdf.ValueTypeNames.Token.type:
                    widget_kwargs["model_cls"] = TfTokenAttributeModel

                units_text = units.get_attribute_units_as_text(prop.prop_name, stage)
                if units_text:
                    tooltip = widget_kwargs.get("tooltip", "")
                    widget_kwargs["tooltip"] =  tooltip + ("\n" if tooltip else "") + units_text[1]

                widget_kwargs["identifier"] = f"physprop_{prop.prop_name}"
                if MainFrameWidget.instance.is_a_omnijoint:
                    widget_kwargs["model_kwargs"] = {"auto_target_session_layer_def": False}
                models = UsdPropertiesWidgetBuilder.build(stage, prop.prop_name, prop.metadata, prop.property_type, prim_paths, label_kwargs, widget_kwargs)

            # register and return its model
            if not isinstance(models, list):
                model = models
                for prim_path in self._payload:
                    self._models[prim_path.AppendProperty(prop.prop_name)].append(model)

                # FIXME: fix uint min being overriden in UsdPropertiesWidgetBuilder.build
                if prop.metadata.get("typeName") == 'uint' and prop.info and prop.info_min:
                    model._min = prop.info_min

                return model, None
            else:
                for model in models:
                    for prim_path in prim_paths:
                        self._models[prim_path.AppendProperty(prop.prop_name)].append(model)
                # for per channel builders the last model is AbstractItemModel covering all
                return models[-1], models[:-1]

        sentinels = database.property_sentinels.get(prop.base_name)
        if sentinels is None:
            model, _ = build_widget()
        else:
            with ui.ZStack():
                model, models = build_widget()
                stc = ui.HStack()
                with stc:
                    ui.Spacer(width=173)
                    with ui.ZStack():
                        ui.Rectangle(name="value")
                        lbl = ui.Label("Overlay Text", name="label", alignment=ui.Alignment.CENTER)
                        add_disabled_styles(lbl)
                        setattr(model, "sentinel_widget", lbl)

                        sentinels_inverted = {v: k for k, v in sentinels.items()}
                        editing = False

                        def toggle_sentinel():
                            def get_sentinel():
                                na_sentinel = sentinels_inverted.get(database.not_authored_token)
                                if na_sentinel is not None:
                                    # NOTE: prop.is_authored is refreshed only when the widget is first loaded
                                    # so we have to query the usd attributes directly
                                    attributes = model._get_attributes()
                                    is_authored = any([hasattr(a, "IsAuthored") and a.IsAuthored() for a in attributes])

                                    if not is_authored:
                                        return na_sentinel

                                if isinstance(model.get_value(), Gf.Quatf):
                                    # Quatf is not hashable
                                    current_val = model.get_value()
                                    for k, v in sentinels.items():
                                        if v == current_val:
                                            return k
                                    return None

                                return sentinels_inverted.get(model.get_value())

                            sentinel = get_sentinel()
                            if sentinel is not None:
                                stc.visible = True
                                lbl.text = sentinel
                            else:
                                stc.visible = False

                        async def show_check():
                            await asyncio.sleep(0.3)  # matching ImGui double-click delay
                            if editing:
                                stc.visible = False

                        def begin_edit(*_):
                            nonlocal editing
                            editing = True
                            asyncio.ensure_future(show_check())

                        def end_edit(*_):
                            nonlocal editing
                            editing = False
                            toggle_sentinel()

                        def changed(*_):
                            if not editing:
                                toggle_sentinel()

                        model.add_begin_edit_fn(begin_edit)
                        model.add_end_edit_fn(end_edit)
                        if isinstance(model, ui.AbstractValueModel):
                            # singlechannel widget
                            model.add_value_changed_fn(changed)
                        else:
                            if models is not None:
                                # multichannel widget
                                for submodel in models:
                                    submodel.add_begin_edit_fn(begin_edit)
                                    submodel.add_end_edit_fn(end_edit)
                            else:
                                # single widget representing multiple channels
                                for item in model._items:
                                    item.model.add_begin_edit_fn(begin_edit)
                                    item.model.add_end_edit_fn(end_edit)
                            model.add_item_changed_fn(changed)

                        toggle_sentinel()
                    ui.Spacer(width=15)

        # FIXME: override change_on_edit_end (sdk might end up with this as a default someday ..)
        if hasattr(model, "_change_on_edit_end"):
            model._change_on_edit_end = database.property_change_on_edit_end_override.get(prop.base_name, True)

        # FIXME: override getters to return the multiplied default where neccessary
        if prop.info and (prop.info.multiply_meter_fn or prop.info.multiply_kg_fn) and prop.default:
            def get_default_value(default, property):
                return True, default

            def is_different_from_default(model, default):
                ret = not math.isclose(model.get_value(), default, abs_tol=1e-5)
                return ret

            def read_value(default, orig_fn, object, time_code):
                if isinstance(object, Usd.Attribute) and not object.IsAuthored():
                    return default
                return orig_fn(object, time_code)

            model._get_default_value = partial(get_default_value, prop.default)
            model.is_different_from_default = partial(is_different_from_default, model, prop.default)
            model._read_value = partial(read_value, prop.default, model._read_value)

            # refresh to update control state, model's inner variables and the widget
            model._update_value(force=True)
            model._value_changed()

        # catch change on ext non-applied API properties and apply them
        # this is better than the previous approach with change callbacks, we can apply the api
        # before the property is set, but still a FIXME, every function override should be removed
        # sooner or later
        if prop.apply_schema is not None:
            def create_placeholder_attributes(prop, model, orig_fn, attributes):
                # force skip refresh to prevent a visual glitch
                self._allow_refresh = False

                # apply new schema before the property is set
                for attribute in attributes:
                    if prop.instance_name is not None:
                        attribute.GetPrim().ApplyAPI(prop.apply_schema, prop.instance_name)
                    else:
                        attribute.GetPrim().ApplyAPI(prop.apply_schema)

                # write into the original list (so that the subsequent property change is not done
                # to the placeholder) and switch back the original method
                attributes[:] = model._get_attributes()
                model._create_placeholder_attributes = orig_fn
                self._allow_refresh = True

            model._create_placeholder_attributes = partial(
                create_placeholder_attributes, prop, model,
                model._create_placeholder_attributes
            )

        return model

    def _build_group_frame(self, name, props, stage):
        if len(props) == 0:
            return None

        is_instance = props[0].instance_name is not None

        def get_title():
            if not is_instance:
                return display_name_from_name(name)
            if self._main_schema in database.display_raw_instance_name_apis:
                return name
            return database.instance_display_names.get(name, display_name_from_name(name))

        def build_instance_frame(component=None):
            collapsed = not is_instance
            frame = ui.CollapsableFrame(title=get_title(), build_header_fn=build_frame_header, name="subFrame")
            with frame:
                with ui.HStack():
                    with ui.VStack(height=0, spacing=5, name="frame_v_stack"):
                        for prop in props:
                            self.build_property_item(stage, prop, self._payload)
                            collapsed |= prop.display_group_collapsed
                    if component is not None:
                        def on_remove_clicked():
                            def on_yes():
                                remove_component(component, self._payload, props[0].instance_name)
                            self._remove_prompt_check(component.title, on_yes)
                        ui.Button(style=REMOVE_BUTTON_STYLE, clicked_fn=on_remove_clicked, width=16, identifier=f"remove_component {component.title}")
            frame.collapsed = collapsed
            return frame

        if self._is_main_multi:
            instance_name = f"{get_schema_name(self._main_schema)}:{props[0].instance_name}"
            component = database.components.get(instance_name)
            if component is not None:
                return build_instance_frame(component)
            else:
                component = database.components.get(get_schema_name(self._main_schema))
                return build_instance_frame(component)
        else:
            return build_instance_frame()

    def build_nested_group_frames(self, stage, grouped_props):
        for prop in grouped_props.props:
            self.build_property_item(stage, prop, self._payload)

        instances = defaultdict(list)

        # init custom instance order if available
        custom_order = database.custom_instance_order_map.get(self._main_schema)
        if custom_order is not None:
            for inst_name in custom_order:
                instances[inst_name] = []

        for group in grouped_props.sub_groups.values():
            for prop in group.props:
                instances[prop.display_group].append(prop)

        for name, props in instances.items():
            self._build_group_frame(name, props, stage)

        if self._is_main_multi:
            schema_components = get_components(self._payload, self._main_schema)

            def on_click():
                context_menu.clear()
                with context_menu:
                    for c in schema_components:
                        ui.MenuItem(c.title).set_triggered_fn(partial(add_component, c, self._payload))
                context_menu.show()

            if len(schema_components) > 0 and self._main_schema in database.multi_api_add_button_text:
                ui.Button(f"{ADD_GLYPH} Add {database.multi_api_add_button_text[self._main_schema]}", clicked_fn=on_click)


class MassAPIWidget(PhysicsWidget):
    def _show_remove_button(self):
        return self._schema_present

    def build_items(self):
        if self._schema_present:
            super().build_items()
        else:
            ui.Label("Mass component is not present, computing mass properties from collision volumes and density.", name="label")

    def on_new_payload(self, payload):
        self._payload = payload

        if not payload or len(payload) == 0:
            return False

        allHaveRigidBody = True

        for prim_path in payload:
            prim = self._get_prim(prim_path)

            if not prim:
                continue

            # DEPRECATED
            hasDeformableBody = prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI)
            hasDeformableSurface = prim.HasAPI(PhysxSchema.PhysxDeformableSurfaceAPI)
            #~DEPRECATED
            hasParticle = prim.HasAPI(PhysxSchema.PhysxParticleAPI)
            hasRigidBody = prim.HasAPI(UsdPhysics.RigidBodyAPI)
            allHaveRigidBody = allHaveRigidBody and hasRigidBody

            # don't show if there's at least one prim that doesn't have either rigidbody or a collision
            if not hasRigidBody and not prim.HasAPI(UsdPhysics.CollisionAPI) and not hasDeformableBody and not hasDeformableSurface and not hasParticle:
                return False

        self._schema_present = super().on_new_payload(payload)

        # we want to show the autocompute message only when there's no schema and all prims have a rigidbody
        if not self._schema_present and not allHaveRigidBody:
            return False

        return True


class JointWidget(PhysicsWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)

        self._utils = dict()

        # fixed joint util
        fixed_joint_type_name = Usd.SchemaRegistry().GetSchemaTypeName(UsdPhysics.FixedJoint)
        u0 = get_align_property_util("Transform", False, self, True, True)
        u1 = get_align_property_util("Transform", True, self, True, True)
        self._utils[fixed_joint_type_name] = {
            "physics:localPos0": u0,
            "physics:localPos1": u1,
            "physics:localRot0": u0,
            "physics:localRot1": u1,
        }

        # revolute joint util
        revolute_joint_type_name = Usd.SchemaRegistry().GetSchemaTypeName(UsdPhysics.RevoluteJoint)
        self._utils[revolute_joint_type_name] = {
            "physics:localPos0": get_align_property_util("Position", False, self, True, False),
            "physics:localPos1": get_align_property_util("Position", True, self, True, False),
        }

        # prismatic joint util
        prismatic_joint_type_name = Usd.SchemaRegistry().GetSchemaTypeName(UsdPhysics.PrismaticJoint)
        u0 = get_align_property_util("Transform", False, self, True, True)
        u1 = get_align_property_util("Transform", True, self, True, True)
        self._utils[prismatic_joint_type_name] = {
            "physics:localPos0": u0,
            "physics:localPos1": u1,
            "physics:localRot0": u0,
            "physics:localRot1": u1,
        }

        # spherical joint util
        spherical_joint_type_name = Usd.SchemaRegistry().GetSchemaTypeName(UsdPhysics.SphericalJoint)
        self._utils[spherical_joint_type_name] = {
            "physics:localPos0": get_align_property_util("Position", False, self, True, False),
            "physics:localPos1": get_align_property_util("Position", True, self, True, False),
        }

    def on_new_payload(self, payload):
        if not super().on_new_payload(payload):
            return False

        last_type = None
        all_types_same = True

        for prim_path in payload:
            prim = self._get_prim(prim_path)

            if not prim:
                return False

            # all types the same check so that we can show the utility
            if all_types_same:
                curr_type = prim.GetTypeName()
                if last_type is not None and curr_type != last_type:
                    all_types_same = False
                last_type = curr_type

        self._property_utils = dict()

        if all_types_same:
            self._property_utils = self._utils.get(last_type, dict())
        return True


class ChildJointWidget(PhysicsWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
        self._joint_names = set(utils.getSchemaPrimDef(UsdPhysics.Joint).GetPropertyNames())
        self._joint_names = self._joint_names.union(set(utils.getSchemaPrimDef(PhysxSchema.PhysxJointAPI).GetPropertyNames()))

    def _filter_props_to_build(self, prim):
        filtered_props = [p for p in super()._filter_props_to_build(prim) if p.base_name not in self._joint_names]
        return filtered_props


class APIInheritanceCheckWidget(PhysicsWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
        self._any_visible = False

    def on_new_payload(self, payload):
        self._any_visible = False
        return super().on_new_payload(payload)

    def _build_frame(self):
        super()._build_frame()

        if not self._any_visible:
            self.show_frame(False)

    def _build_group_frame(self, name, props, stage):
        if is_parent_api_instance(get_schema_name(self._main_schema), props[0].instance_name, self._payload):
            return None

        self._any_visible = True

        return super()._build_group_frame(name, props, stage)


class FixedTendonWidget(APIInheritanceCheckWidget):
    instances = set()

    def __init__(self, title, schema, builders):
        addon = {
            "gearing": [GearingWidgetBuilder],
            "forceCoefficient": [GearingWidgetBuilder],
        }
        super().__init__(title, schema, dict(builders, **addon))
        self._obs = []
        self._switched = None

        FixedTendonWidget.instances.add(self)

    def clean(self):
        super().clean()
        self._cleanup()

    def _cleanup(self):
        self._obs = []
        self._switched = None
        FixedTendonWidget.instances.discard(self)

    def on_new_payload(self, payload):
        self._cleanup()
        ret = super().on_new_payload(payload)
        if ret:
            FixedTendonWidget.instances.add(self)
        return ret

    def _build_group_frame(self, name, props, stage):
        instance_name = props[0].instance_name

        def clicked(curr):
            if self._switched == curr:
                get_physxui_interface().set_tendon_visualization_filter(None)
                self._obs[curr].set_style({"color": ui.color.grey})
                self._switched = None
            else:
                get_physxui_interface().set_tendon_visualization_filter(instance_name)
                for instance in FixedTendonWidget.instances:
                    instance._switched = None
                    for ob in instance._obs:
                        ob.set_style({"color": ui.color.grey})
                self._obs[curr].set_style({"color": ui.color.white})
                self._switched = curr

        with ui.ZStack():
            frame = super()._build_group_frame(name, props, stage)
            if not frame:
                return
            with ui.HStack():
                ui.Spacer(width=ui.Fraction(0.5))
                with ui.VStack(width=0):
                    ui.Spacer(height=3)
                    ob = OverlayButton(EYE_GLYPH, frame, partial(clicked, len(self._obs)), height=16, width=16)
                    ob.set_style({"color": ui.color.grey})
                    self._obs.append(ob)
                ui.Spacer(width=5)


class SpatialTendonWidget(APIInheritanceCheckWidget):
    def _build_group_frame(self, name, props, stage):
        instance_name = props[0].instance_name
        body_path = self._payload[0].pathString

        def clicked(*_):
            get_physxui_interface().select_spatial_tendon_attachment_helper(body_path, instance_name)

        with ui.ZStack():
            frame = super()._build_group_frame(name, props, stage)
            if not frame:
                return
            with ui.HStack():
                ui.Spacer(width=ui.Fraction(0.5))
                with ui.VStack(width=0):
                    ui.Spacer(height=3)
                    OverlayButton(EYE_GLYPH, frame, clicked, height=16, width=16)
                ui.Spacer(width=5)

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        if self._main_schema == PhysxSchema.PhysxTendonAttachmentRootAPI:
            remove_props(["parentAttachment", "parentLink", "gearing"])
        elif len(self._payload) != 1:
            remove_props(["parentAttachment"])

        return filtered_props

    def _build_property_item(self, stage, prop, prim_paths):
        if prop.base_name == "parentAttachment":
            prop.finalize(stage)
            prim = stage.GetPrimAtPath(prim_paths.get_paths()[0])
            parent = utils.get_spatial_tendon_parent_link(prim, prop.instance_name)
            custom_names = [""]
            if parent is not None:
                custom_names.extend(utils.get_spatial_tendon_attachment_candidates(stage.GetPrimAtPath(parent)))

            label_kwargs = self.get_default_label_kwargs()
            model = CustomTokenComboBuilder(stage, prop, prim_paths, label_kwargs, None, custom_names)
            return model
        elif prop.base_name == "parentLink":
            model = super()._build_property_item(stage, prop, prim_paths)

            async def delayed_request():
                await omni.kit.app.get_app().next_update_async()
                self.request_rebuild()

            model.add_item_changed_fn(lambda *_: asyncio.ensure_future(delayed_request()))
            return model
        else:
            return super()._build_property_item(stage, prop, prim_paths)


class LimitWidget(PhysicsWidget):
    fake_component = database.Component(None, None, "PhysicsLimit", "Limit", None)

    def build_impl(self):
        # fake a limit component so we can produce an all-instance removing button
        self._build_impl_with_remove_button(LimitWidget.fake_component)


class DriveWidget(PhysicsWidget):
    fake_component = database.Component(None, None, "PhysicsDrive", "Drive", None)
    advanced_properties = set(database.default_order_map['PhysxDrivePerformanceEnvelopeAPI']) 

    def build_impl(self):
        # fake a drive component so we can produce an all-instance removing button
        self._build_impl_with_remove_button(DriveWidget.fake_component)

    def _build_group_frame(self, name, props, stage):

        if len(props) == 0:
            return None
        
        is_instance = props[0].instance_name is not None

        def get_title():
            if not is_instance:
                return display_name_from_name(name)
            if self._main_schema in database.display_raw_instance_name_apis:
                return name
            return database.instance_display_names.get(name, display_name_from_name(name))
        
        def build_instance_frame(component=None):

            collapsed = not is_instance
            parent_frame = ui.CollapsableFrame(
                title=get_title(), 
                build_header_fn=build_frame_header,
                name="parentFrame"
            )
            
            basic_props = [prop for prop in props if prop.base_name not in self.advanced_properties]
            advanced_props = [prop for prop in props if prop.base_name in self.advanced_properties]
            with parent_frame:
                with ui.HStack():
                    with ui.VStack(height=0, spacing=5, name="props_stack"):
                        for prop in basic_props:
                                self.build_property_item(stage, prop, self._payload)
                                collapsed |= prop.display_group_collapsed
                        if component is not None:
                            ui.Button(
                                style=REMOVE_BUTTON_STYLE,
                                clicked_fn=lambda: self._remove_prompt_check(
                                    component.title,
                                    lambda: remove_component(component, self._payload, props[0].instance_name)
                                ),
                                width=16,
                                tooltip="Remove Component",
                                identifier=f"remove_component_{component.title}"
                            )
                
                        child_frame = ui.CollapsableFrame(
                            title="Advanced",  
                            build_header_fn=build_frame_header,
                            name="childFrame",
                            collapsed=True
                        )
                        
                        with child_frame:
                            with ui.HStack():
                                with ui.VStack(height=0, spacing=5, name="props_stack"):
                                        for prop in advanced_props:
                                                self.build_property_item(stage, prop, self._payload)

            parent_frame.collapsed = collapsed
            return parent_frame


        if self._is_main_multi:
            instance_name = f"{get_schema_name(self._main_schema)}:{props[0].instance_name}"
            component = database.components.get(instance_name)
            if component is not None:
                return build_instance_frame(component)
            else:
                component = database.components.get(get_schema_name(self._main_schema))
                return build_instance_frame(component)
        else:
            return build_instance_frame()



class JointAxisWidget(PhysicsWidget):
    fake_component = database.Component(None, None, "PhysxJointAxis", "Properties", None)

    def build_impl(self):
        self._build_impl_with_remove_button(JointAxisWidget.fake_component)
  

class JointStateWidget(PhysicsWidget):
    fake_component = database.Component(None, None, "JointState", "Joint State", None)

    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
        self._disabled = True
        self._pos_widget = None
        self._vel_widget = None
        physxInterface = get_physx_interface()
        self._simulationEventSubcription = physxInterface.get_simulation_event_stream_v2().create_subscription_to_pop(
            self._on_simulation_event
        )

    def _on_simulation_event(self, event):
        if event.type == int(SimulationEvent.STOPPED):
            if not self._disabled:
                self._disabled = True
                if self._pos_widget:
                    self._pos_widget.enabled = False
                    self._vel_widget.enabled = False
        elif event.type == int(SimulationEvent.RESUMED):
            if self._disabled:
                self._disabled = False
                if self._pos_widget:
                    self._pos_widget.enabled = True
                    self._vel_widget.enabled = True

    def build_impl(self):
        # fake a joint state component so we can produce an all-instance removing button
        self._build_impl_with_remove_button(JointStateWidget.fake_component)

    def _build_property_item(self, stage, prop, prim_paths):

        model = super()._build_property_item(stage, prop, prim_paths)
        if prop.base_name == "physics:position":
            add_disabled_styles(model.value_widget)
            self._pos_widget = model.value_widget
            enable_widget(self._pos_widget, not self._disabled)
        elif prop.base_name == "physics:velocity":
            add_disabled_styles(model.value_widget)
            self._vel_widget = model.value_widget
            enable_widget(self._vel_widget, not self._disabled)
        return model


class PhysxLimitExtJointWidget(ChildJointWidget):
    def __init__(self, title, schema, builders, limit_instance):
        super().__init__(title, schema, builders)

        self._limit_api = PhysxSchema.PhysxLimitAPI
        self._limit_instance = limit_instance
        self._limit_type_name = Usd.SchemaRegistry().GetSchemaTypeName(self._limit_api)
        self._limit_prefix = database.ext_multi_api_prefixes[self._limit_api]
        self._limit_prop_names = [(name, f"{self._limit_prefix}:{limit_instance}:{name}") for name in utils.getSchemaPropertyNames(self._limit_api)]
        prim_def = utils.getSchemaPrimDef(self._limit_api)
        self._limit_prop_specs = [prim_def.GetSchemaPropertySpec(name) for name in prim_def.GetPropertyNames()]

    def _filter_props_to_build(self, prim):
        filtered_props = []
        instances = get_schema_instances(prim, self._limit_type_name)

        if self._limit_instance in instances:
            for base_name, full_name in self._limit_prop_names:
                prop = prim.GetProperty(full_name)
                filtered_props.append(UiProp().from_property(prop, base_name, self._limit_instance))
        else:
            for prop_spec in self._limit_prop_specs:
                filtered_props.append(UiProp().from_property_spec(prop_spec, prim, self._limit_api, self._limit_instance, self._limit_prefix))

        for prop in filtered_props:
            prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = "Advanced"
            prop.display_group_collapsed = True

        return super()._filter_props_to_build(prim) + filtered_props


class FixedJointWidget(ChildJointWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)

    def on_new_payload(self, payload):
        # Fixed joint now only inherits from Joint and has no properties itself
        return False


class RevoluteJointWidget(PhysxLimitExtJointWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders, "angular")


class PrismaticJointWidget(PhysxLimitExtJointWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders, "linear")


class DistanceJointWidget(PhysxLimitExtJointWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders, "distance")


class SphericalJointWidget(PhysxLimitExtJointWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders, "cone")


class GearJointWidget(ChildJointWidget):
    def __init__(self, title, schema, builders):
        super().__init__("Gear Joint", schema, builders)


class RackAndPinionJoint(ChildJointWidget):
    def __init__(self, title, schema, builders):
        super().__init__("Rack And Pinion Joint", schema, builders)


class LocalSpaceVelocitiesWidget(PhysicsWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
        self.localSpaceVelocityProp = UiProp().from_custom(
            pxb.METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES,
            "Velocities in Local Space",
            "",
            'bool',
            False,
            """If checked, both Linear and Angular Velocities
        are stored in Local Space, otherwise in Global Space.
        This value is by default inferred from global Physics Settings,
        but you have the option to override the value here."""
        )

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)
        filtered_props.append(self.localSpaceVelocityProp)
        return filtered_props


class ExtendedColliderWidget(PhysicsWidget):
    margin_title = "Margin"

    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
        self._min_model = None
        self._max_model = None
        self._deformable_b_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsDeformableBodyAPI")
        self._deformable_b_name = "omniphysics:deformableBodyEnabled"
        self._deformable_ignore_props = {
            "physics:approximation",
            "physics:simulationOwner",
            "physxCollision:torsionalPatchRadius",
            "physxCollision:minTorsionalPatchRadius",
        }

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def try_inactivate(approx_api, approx_name, curr_approx_name):
            for prop_spec in self._ext_prop_specs[approx_api]:
                prop = filtered_props_dict.get(prop_spec.name)
                if prop is not None:
                    prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = "Inactive"
                    prop.display_group_collapsed = True

        def is_part_of_deformable(prim):
            if not prim.IsA(UsdGeom.Mesh) and not prim.IsA(UsdGeom.TetMesh):
                return False
            while prim:
                rigid_body = UsdPhysics.RigidBodyAPI(prim)
                if rigid_body and rigid_body.GetRigidBodyEnabledAttr().Get():
                    return False
                if prim.HasAPI(self._deformable_b_type) and prim.GetAttribute(self._deformable_b_name).Get():
                    return True
                xformable = UsdGeom.Xformable(prim)
                if xformable and xformable.GetResetXformStack():
                    return False
                prim = prim.GetParent()

        if is_part_of_deformable(prim):
            filtered_props_mod = []
            for prop in filtered_props:
                if not prop.base_name in self._deformable_ignore_props:
                    filtered_props_mod.append(prop)
            return filtered_props_mod

        filtered_props_dict = {prop.base_name: prop for prop in filtered_props}
        mesh_api = UsdPhysics.MeshCollisionAPI(prim)
        if mesh_api:
            curr_approx_name = mesh_api.GetApproximationAttr().Get()
            curr_approx_api = utils.MESH_APPROXIMATIONS.get(curr_approx_name)
            for approx_name, approx_api in utils.MESH_APPROXIMATIONS.items():
                if approx_api is not None and (curr_approx_api != approx_api or not prim.HasAPI(approx_api)):
                    try_inactivate(approx_api, approx_name, curr_approx_name)

        if prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Cone):
            prop = UiProp().from_custom("physxConvexGeometry:margin", ExtendedColliderWidget.margin_title, "", 'float', 0)
            prop.display_group = "Advanced"
            prop.display_group_collapsed = True
            filtered_props.append(prop)

        return filtered_props

    def _build_property_item(self, stage, prop, prim_paths):
        def on_approximation_changed(model, _):
            new_api_str = str(model._allowed_tokens[model._current_index.as_int].token)
            new_api = utils.MESH_APPROXIMATIONS.get(new_api_str, None)
            if new_api is not None:
                for prim in generate_prims(prim_paths.get_paths()):
                    new_api.Apply(prim)
            self.request_rebuild()

        model = super()._build_property_item(stage, prop, prim_paths)
        if prop.base_name == "physics:approximation":
            model.add_item_changed_fn(on_approximation_changed)
        return model


# this widget is a base for invisible Add menu widgets
# * fill the add menu with components and extra items
# * refresh the property window on undo/redo of apply/unapply commands
class InvisibleMenuWidgetBase(SimplePropertyWidget):
    Info = namedtuple("Info", "show, enable")
    Cache = namedtuple("Cache", "is_a_base_joint")

    def __init__(self, title, menu_root, components=dict(), extras=[], show_limit_warnings=False):
        super().__init__(title, False)
        omni.kit.undo.subscribe_on_change(self._undo_redo_on_change)
        self._items = []
        self._last_id = None
        self._over_limit = False
        self._over_subtree_limit = False
        self._menu_root = menu_root
        self._components = components
        self._extras = extras
        self._show_limit_warnings = show_limit_warnings

        self._reset_show()
        self._add_menu_items()

    def _add_menu_items(self):
        def on_click_extra(e_id, payload):
            e_item = self._extras[e_id]
            omni.kit.undo.begin_group()
            for prim in generate_prims_with_cache(payload):
                if e_item.can_add(prim) and (e_item.can_show is None or e_item.can_show(prim)):
                    e_item.on_click(prim)
            omni.kit.undo.end_group()

        if self._show_limit_warnings:
            self._items.append(PrimPathWidget.add_button_menu_entry(
                f"{self._menu_root}/For perf reasons the list below is not filtered\nfor appliability for a large selection.\nSee Physics Preferences for changing the limit.",
                show_fn=lambda objects: self._is_over_selection_limit(objects),
                enabled_fn=lambda *_: False,
            ))

            self._items.append(PrimPathWidget.add_button_menu_entry(
                f"{self._menu_root}/For perf reasons the list below is not filtered\nfor appliability for a selection with large subtrees.\nSee Physics Preferences for changing the limit.",
                show_fn=lambda objects: self._is_over_subtree_limit(objects),
                enabled_fn=lambda *_: False,
            ))

            self._items.append(PrimPathWidget.add_button_menu_entry(
                f"{self._menu_root}/",
                show_fn=lambda objects: self._is_over_selection_limit(objects) or self._is_over_subtree_limit(objects),
                enabled_fn=lambda *_: False,
            ))

        for e_id, e in enumerate(self._extras):
            self._items.append(PrimPathWidget.add_button_menu_entry(
                f"{self._menu_root}/{e.title}",
                onclick_fn=partial(on_click_extra, e_id),
                show_fn=lambda objects, idx=e_id: self._get_extra_info(objects, idx).show,
                enabled_fn=lambda objects, idx=e_id: self._get_extra_info(objects, idx).enable,
            ))

        self._items.append(PrimPathWidget.add_button_menu_entry(
            "{self._menu_root}/",
            show_fn=lambda objects: self._has_any_extra_shown(objects),
        ))

        for c_id, c in enumerate(self._components.values()):
            self._items.append(PrimPathWidget.add_button_menu_entry(
                f"{self._menu_root}/{c.title}",
                onclick_fn=partial(add_component, c),
                show_fn=lambda objects, idx=c_id: self._get_component_info(objects, idx).show,
                enabled_fn=lambda objects, idx=c_id: self._get_component_info(objects, idx).enable,
            ))

    def _remove_menu_items(self):
        for item in self._items:
            PrimPathWidget.remove_button_menu_entry(item)

        self._items = []

    def clean(self):
        super().clean()
        omni.kit.undo.unsubscribe_on_change(self._undo_redo_on_change)
        self._remove_menu_items()

    def refresh_menu_items(self):
        self._reset_show()
        self._remove_menu_items()
        self._add_menu_items()

    def _undo_redo_on_change(self, cmds):
        ...

    def _reset_show(self):
        self._component_info = [self.Info(False, True) for _ in range(len(self._components.values()))]
        self._extra_info = [self.Info(False, True) for _ in range(len(self._extras))]
        self._any_extra_shown = False

    def _refresh(self, objects):
        self._last_id = id(objects)
        prim_list = objects.get("prim_list")
        stage = objects.get("stage")
        lmt = carb.settings.get_settings().get_as_int(pxb.SETTING_ADDMENU_SELECTION_LIMIT)
        self._over_selection_limit = len(objects.get("prim_list", [])) > lmt
        self._over_subtree_limit = False

        if prim_list is None or stage is None:
            self._reset_show()
            return

        if self._over_selection_limit:
            self._component_info = [self.Info(True, True) for _ in range(len(self._components.values()))]
            self._extra_info = [self.Info(True, True) for _ in range(len(self._extras))]
            self._any_extra_shown = True
            return

        prims = [p for p in generate_from_prim_list_with_cache(stage, prim_list)]

        for p in prims:
            if p._refresh_cache.over_subtree_limit:
                self._over_subtree_limit = True
                break

        # refresh components and extras
        self._refresh_all_with_early_exit(prims)

        self._any_extra_shown = any([ei.show for ei in self._extra_info])

    def _refresh_all_with_early_exit(self, prims):
        def _all_is_present(c):
            for prim in prims:
                if not c.is_present(prim):
                    return False
            return True

        def _any_can_add(c):
            for prim in prims:
                if c.can_add(prim):
                    return True
            return False

        def _any_can_show(c):
            for prim in prims:
                if c.can_show(prim):
                    return True
            return False

        for c_id, c in enumerate(self._components.values()):
            all_is_present = _all_is_present(c)
            any_can_add = _any_can_add(c)

            if c.can_show:
                any_can_show = _any_can_show(c)
                self._component_info[c_id] = self.Info(any_can_show, not all_is_present and any_can_add)
            else:
                self._component_info[c_id] = self.Info(not all_is_present and any_can_add, True)

        for e_id, e in enumerate(self._extras):
            any_can_add = _any_can_add(e)
            if e.can_show:
                any_can_show = _any_can_show(e)
                self._extra_info[e_id] = self.Info(any_can_show, any_can_add)
            else:
                self._extra_info[e_id] = self.Info(any_can_add, True)

    def _get_component_info(self, objects, idx):
        if self._last_id != id(objects):
            self._refresh(objects)

        return self._component_info[idx]

    def _get_extra_info(self, objects, idx):
        if self._last_id != id(objects):
            self._refresh(objects)

        return self._extra_info[idx]

    def _is_over_selection_limit(self, objects):
        if self._last_id != id(objects):
            self._refresh(objects)

        return self._over_selection_limit

    def _is_over_subtree_limit(self, objects):
        if self._last_id != id(objects):
            self._refresh(objects)

        return self._over_subtree_limit

    def _has_any_extra_shown(self, objects):
        if self._last_id != id(objects):
            self._refresh(objects)

        return self._any_extra_shown

    def on_new_payload(self, payload):
        return False


class InvisibleWidget(InvisibleMenuWidgetBase):
    name = "physx_invisible"

    def __init__(self):
        super().__init__(
            "Physics Invisible",
            "Physics",
            database.components,
            database.extra_add_items,
            True
        )

    def _undo_redo_on_change(self, cmds):
        if any(item in [
            "ApplyAPISchema", "UnapplyAPISchema",
            # DEPRECATED
            "AddDeformableBodyComponent", "AddDeformableSurfaceComponent",
            #~DEPRECATED
            "AddPhysicsComponent", "RemovePhysicsComponent", "ApplyCodelessAPISchema", "UnapplyCodelessAPISchema",
            # FIXME: ApplyAPISchema is not used for these two after optimizations
            # this needs to be solved some other way in general ..
            "SetRigidBody", "SetStaticCollider",
            "SetVolumeDeformableBody", "SetSurfaceDeformableBody",
            "CreateAutoVolumeDeformableHierarchy", "CreateAutoSurfaceDeformableHierarchy",
            # NOTE: backward compat
            "ApplyAPISchemaCommand", "UnapplyAPISchemaCommand", "AddDeformableBodyComponentCommand",  
            "AddPhysicsComponentCommand", "RemovePhysicsComponentCommand",
            # SupportUI
            "CreateCollidersCommand",
        ] for item in cmds):
            async def wait_and_rebuild():
                await omni.kit.app.get_app().next_update_async()
                MainFrameWidget.instance.request_rebuild()

            # delayed delayed refresh
            asyncio.ensure_future(wait_and_rebuild())


class JointVisualizationWidget(SimplePropertyWidget):
    name = "physx_joint_visualization"

    def __init__(self):
        super().__init__("Joint Edit Mode", False)

    def on_new_payload(self, payload):
        # disabling this panel for now
        # we don't have enough data to support local pose editing for bodies
        # and world pose editing for the joint itself isn't meaningful
        return False

        if not super().on_new_payload(payload):
            return False

        if not payload or len(payload) != 1:
            return False

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False

        path = str(payload[0])
        return get_physxui_interface().show_joint_edit_mode_ui(path)

    def is_visible(self):
        return True

    def build_items(self):
        edit_local_pose = False
        for path in self._payload:
            edit_local_pose = get_physxui_interface().get_joint_edit_local_pose_mode(str(path))

        def joint_edit_local_pose_changed(model):
            for path in self._payload:
                get_physxui_interface().set_joint_edit_local_pose_mode(str(path), model.as_bool)

        with ui.VStack():
            with ui.HStack(height=HORIZONTAL_SPACING):
                UsdPropertiesWidgetBuilder._create_label(
                    "Edit Local Pose Transform", {},
                    {
                        "tooltip": (
                            "Enabled: changes in transform will update 'local position/orientation' "
                            "data for the joint, which is used to cacluate the limits of the joint\n"
                            "Disabled: changes in transform will update the starting position of the body "
                            "without changing the limits of the joint (ex: open a drawer slightly)"
                        )
                    }
                )
                self._joint_model = ui.SimpleBoolModel(edit_local_pose)
                ui.CheckBox(self._joint_model, height=25)
                self._joint_model.add_value_changed_fn(joint_edit_local_pose_changed)


class PhysicsMaterialBindingWidget(UsdBindingAttributeWidget):
    name = "physx_material_binding"
    title = "Physics materials on selected models"

    def __init__(self):
        self._material_utils = MaterialUtils()

        super().__init__(
            title=PhysicsMaterialBindingWidget.title,
            material_purpose="physics",
            filter_fn=lambda p: database.has_any_material_api(p),
            get_materials_async_fn=self._material_utils.get_materials_from_stage_async
        )

    def is_visible(self):
        return True

    def on_new_payload(self, payload):
        ret = super().on_new_payload(payload)

        # handle particle system before because PS is not an UsdGeom.Gprim, Xform or Subset thus the call to super() will return False
        for path in payload:
            stage = payload.get_stage()
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsA(PhysxSchema.PhysxParticleSystem):
                return True

        if not ret:
            return False

        for path in payload:
            prim = self._get_prim(path)
            if (not prim or (
                not prim.HasAPI(UsdPhysics.CollisionAPI) and
                # DEPRECATED
                not prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI) and 
                not prim.HasAPI(PhysxSchema.PhysxDeformableSurfaceAPI) and
                #~DEPRECATED
                not prim.HasAPI("OmniPhysicsDeformableBodyAPI") and
                not prim.HasAPI("OmniPhysicsVolumeDeformableSimAPI") and
                not prim.HasAPI("OmniPhysicsSurfaceDeformableSimAPI")
                )):
                return False

        return True


class PhysicsDefaultMaterialBindingWidget(UsdBindingAttributeWidget):
    name = "physx_default_material_binding"
    title = "Physics default material"

    def __init__(self):
        self._material_utils = MaterialUtils()

        super().__init__(
            title=PhysicsDefaultMaterialBindingWidget.title,
            material_purpose="physics",
            filter_fn=lambda p: database.has_any_material_api(p),
            get_materials_async_fn=self._material_utils.get_materials_from_stage_async
        )

    def is_visible(self):
        return True

    def on_new_payload(self, payload):
        if not SimplePropertyWidget.on_new_payload(self, payload):
            return False

        if len(payload) == 0:
            return False

        for path in payload:
            prim = self._get_prim(path)
            if not prim or (not prim.IsA(UsdPhysics.Scene)):
                return False

        return True


class PhysicsCustomPropertiesWidget(UsdPropertiesWidget):
    name = "physx_custom_properties"

    def __init__(self):
        super().__init__("Additional Physics Properties", False)
        self._properties = dict()

    def clean_before_rebuild(self):
        for models in self._models.values():
            for model in models:
                model.clean()
        self._models = defaultdict(list)

    def set_properties(self, properties):
        self._properties = properties
        if self._collapsable_frame:
            self._collapsable_frame.rebuild()

    def on_new_payload(self, payload):
        if not super().on_new_payload(payload):
            return False

        if not self._payload or len(self._payload) == 0:
            return False

        return True

    def _customize_props_layout(self, props):
        for prop in props:
            data = self._properties.get(prop.prop_name)
            if data.display_name is not None:
                prop.override_display_name(data.display_name)
            type_name = prop.metadata.get(Sdf.PrimSpec.TypeNameKey)
            if data.read_only and type_name is not None:
                # this will use UsdPropertiesWidgetBuilder::_fallback_builder which is read-only
                type_name = None
        return props

    def _filter_props_to_build(self, props):
        return [prop for prop in props if prop.GetName() in self._properties]

    def _build_frame(self):
        self.clean_before_rebuild()
        super()._build_frame()
        if len(self._models) == 0:
            self._collapsable_frame.visible = False


class ExtendedSceneWidget(PhysicsWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
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
        self._set_widget_state()

    def _build_property_item(self, stage, prop, prim_paths):
        def changed(model):
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

        solveArtContactLastDocs = ("Order articulation contact constraints and articulation joint"
            "maximum velocity constraints so that they are solved after all other constraints in the solver")
        filtered_props.append(UiProp().from_custom("physxScene:solveArticulationContactLast", "Solve Articulation Contact Last", "", 'bool', False, solveArtContactLastDocs))

        return filtered_props


class ExtendedRigidMaterialWidget(PhysicsWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
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


class ExtendedDeformableBodyWidgetDeprecated(PhysicsWidget):
    is_kinematic_title = "Kinematic Enabled"
    sim_mesh_res_title = "Simulation Mesh Resolution"
    coll_simp_title = "Collision Mesh Simplification"
    coll_simp_remeshing_title = "Enable Remeshing"
    coll_simp_remeshing_res_title = "Remeshing Resolution"
    coll_simp_target_title = "Target Triangle Count"
    coll_simp_force_conforming_title = "Force Conforming"

    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
        self._kinematic_enabled = False

    def _set_widget_state(self):
        if self._sim_res_widget is not None:
            enable_widget(self._sim_res_widget, self._coll_simp_enabled or not self._kinematic_enabled)
        if self._coll_simp_remeshing_widget is not None:
            enable_widget(self._coll_simp_remeshing_widget, self._coll_simp_enabled and not self._kinematic_enabled)
        if self._coll_simp_remeshing_res_widget is not None:
            enable_widget(self._coll_simp_remeshing_res_widget, self._coll_simp_enabled and not self._kinematic_enabled)
            enable_widget(self._coll_simp_remeshing_res_sent_widget, self._coll_simp_enabled and not self._kinematic_enabled)
        if self._coll_simp_target_widget is not None:
            enable_widget(self._coll_simp_target_widget, self._coll_simp_enabled)
            enable_widget(self._coll_simp_target_sent_widget, self._coll_simp_enabled)
        if self._coll_simp_force_conforming_widget is not None:
            enable_widget(self._coll_simp_force_conforming_widget, self._coll_simp_enabled and not self._kinematic_enabled)

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)
        if prim.HasAttribute("physxDeformable:kinematicEnabled"):
            kinematicEnabled = prim.GetAttribute("physxDeformable:kinematicEnabled").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:kinematicEnabled",
                                  ExtendedDeformableBodyWidgetDeprecated.is_kinematic_title, "", 'bool', kinematicEnabled))
        if prim.HasAttribute("physxDeformable:simulationHexahedralResolution"):
            simulationHexahedralResolution = prim.GetAttribute("physxDeformable:simulationHexahedralResolution").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:simulationHexahedralResolution",
                                  ExtendedDeformableBodyWidgetDeprecated.sim_mesh_res_title, "", 'int', simulationHexahedralResolution))
        if prim.HasAttribute("physxDeformable:collisionSimplification"):
            collisionSimplification = prim.GetAttribute("physxDeformable:collisionSimplification").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:collisionSimplification",
                                  ExtendedDeformableBodyWidgetDeprecated.coll_simp_title, "", 'bool', collisionSimplification))
        if prim.HasAttribute("physxDeformable:collisionSimplificationRemeshing"):
            collisionSimplificationRemeshing = prim.GetAttribute("physxDeformable:collisionSimplificationRemeshing").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:collisionSimplificationRemeshing",
                                  ExtendedDeformableBodyWidgetDeprecated.coll_simp_remeshing_title, "", 'bool', collisionSimplificationRemeshing))
        if prim.HasAttribute("physxDeformable:collisionSimplificationRemeshingResolution"):
            collisionSimplificationRemeshingResolution = prim.GetAttribute("physxDeformable:collisionSimplificationRemeshingResolution").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:collisionSimplificationRemeshingResolution",
                                  ExtendedDeformableBodyWidgetDeprecated.coll_simp_remeshing_res_title, "", 'int', collisionSimplificationRemeshingResolution))
        if prim.HasAttribute("physxDeformable:collisionSimplificationTargetTriangleCount"):
            collisionSimplificationTargetTriangleCount = prim.GetAttribute("physxDeformable:collisionSimplificationTargetTriangleCount").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:collisionSimplificationTargetTriangleCount",
                                  ExtendedDeformableBodyWidgetDeprecated.coll_simp_target_title, "", 'int', collisionSimplificationTargetTriangleCount))
        if prim.HasAttribute("physxDeformable:collisionSimplificationForceConforming"):
            collisionSimplificationForceConforming = prim.GetAttribute("physxDeformable:collisionSimplificationForceConforming").Get()
            filtered_props.append(UiProp().from_custom("physxDeformable:collisionSimplificationForceConforming",
                                  ExtendedDeformableBodyWidgetDeprecated.coll_simp_force_conforming_title, "", 'bool', collisionSimplificationForceConforming))

        def add_props_to_groups(prop_list, group_name):
            nonlocal filtered_props
            for prop in filtered_props:
                if prop.base_name in prop_list:
                    prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = group_name

        add_props_to_groups(["physxDeformable:collisionSimplificationRemeshing",
                             "physxDeformable:collisionSimplificationRemeshingResolution",
                             "physxDeformable:collisionSimplificationTargetTriangleCount",
                             "physxDeformable:collisionSimplificationForceConforming"],
                            "Collision Mesh Simplification")

        add_props_to_groups(["physxDeformable:selfCollisionFilterDistance",
                             "physxDeformable:sleepThreshold",
                             "physxDeformable:settlingThreshold",
                             "physxDeformable:sleepDamping",
                             "physxDeformable:solverPositionIterationCount"],
                            "Advanced")

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        remove_props(["physxDeformable:restPoints",
                      "physxDeformable:collisionPoints", "physxDeformable:collisionRestPoints", "physxDeformable:collisionIndices",
                      "physxDeformable:simulationPoints", "physxDeformable:simulationRestPoints", "physxDeformable:simulationIndices",
                      "physxDeformable:simulationVelocities",
                      "physxDeformable:disableGravity",
                      "physxCollision:torsionalPatchRadius", "physxCollision:minTorsionalPatchRadius"])

        return filtered_props

    def build_items(self):
        self._sim_res_widget = None
        self._coll_simp_remeshing_widget = None
        self._coll_simp_remeshing_res_widget = None
        self._coll_simp_remeshing_res_sent_widget = None
        self._coll_simp_target_widget = None
        self._coll_simp_target_sent_widget = None
        self._coll_simp_force_conforming_widget = None
        super().build_items()
        self._set_widget_state()

    def _build_property_item(self, stage, prop, prim_paths):

        def changed_coll_simp(model):
            if self._coll_simp_enabled != model.as_bool:
                self._coll_simp_enabled = model.as_bool
                self._set_widget_state()

        def changed_kinematic(model):
            if self._kinematic_enabled != model.as_bool:
                self._kinematic_enabled = model.as_bool
                self._set_widget_state()

        model = super()._build_property_item(stage, prop, prim_paths)
        if prop.base_name == "physxDeformable:kinematicEnabled":
            model.add_value_changed_fn(changed_kinematic)
            self._kinematic_enabled = model.as_bool
        if prop.base_name == "physxDeformable:simulationHexahedralResolution":
            if not model._ambiguous and (model.as_int < prop.info.min or model.as_int > prop.info.max):
                with ui.HStack():
                    UsdPropertiesWidgetBuilder._create_label(ExtendedDeformableBodyWidgetDeprecated.sim_mesh_res_title)
                    ui.Spacer(width=8)
                    text = "Not available" if model.as_int <= 0 else f"{model.as_int}"
                    ui.Label(text, name="label")
            add_disabled_styles(model.value_widget)
            self._sim_res_widget = model.value_widget
        if prop.base_name == "physxDeformable:collisionSimplification":
            model.add_value_changed_fn(changed_coll_simp)
            self._coll_simp_enabled = model.as_bool
        if prop.base_name == "physxDeformable:collisionSimplificationRemeshing":
            add_disabled_styles(model.value_widget)
            self._coll_simp_remeshing_widget = model.value_widget
        if prop.base_name == "physxDeformable:collisionSimplificationRemeshingResolution":
            add_disabled_styles(model.value_widget)
            self._coll_simp_remeshing_res_widget = model.value_widget
            self._coll_simp_remeshing_res_sent_widget = model.sentinel_widget
        if prop.base_name == "physxDeformable:collisionSimplificationTargetTriangleCount":
            add_disabled_styles(model.value_widget)
            self._coll_simp_target_widget = model.value_widget
            self._coll_simp_target_sent_widget = model.sentinel_widget
        if prop.base_name == "physxDeformable:collisionSimplificationForceConforming":
            add_disabled_styles(model.value_widget)
            self._coll_simp_force_conforming_widget = model.value_widget

        return model


class ExtendedDeformableSurfaceWidgetDeprecated(PhysicsWidget):
    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def add_props_to_groups(prop_list, group_name):
            nonlocal filtered_props
            for prop in filtered_props:
                if prop.base_name in prop_list:
                    prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = group_name

        add_props_to_groups(["physxDeformable:sleepThreshold",
                             "physxDeformable:settlingThreshold",
                             "physxDeformable:sleepDamping",
                             "physxDeformable:solverPositionIterationCount"],
                             "Advanced")

        add_props_to_groups(["physxCollision:contactOffset",
                             "physxCollision:restOffset",
                             "physxDeformable:selfCollisionFilterDistance",
                             "physxDeformableSurface:collisionPairUpdateFrequency",
                             "physxDeformableSurface:collisionIterationMultiplier",
                             "physxDeformable:maxDepenetrationVelocity"],
                             "Collision")

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        remove_props(["physxDeformable:enableCCD", "physxCollision:torsionalPatchRadius",
                      "physxCollision:minTorsionalPatchRadius", "physxCollision:maxTorsionalPatchRadius",
                      "physxDeformableSurface:bendingStiffnessScale",
                      "physxDeformable:restPoints", "physxDeformable:simulationIndices",
                      "physxDeformable:simulationVelocities"])

        return filtered_props


class ExtendedDeformalbeSurfaceMaterialWidgetDeprecated(PhysicsWidget):

    prop_data_custom = {
        "physxDeformableSurfaceMaterial:bendDamping": ("Bend Damping", "", 'float', 0.0, "The amount of damping that gets applied to bending motions."),
        "physxDeformableSurfaceMaterial:elasticityDamping": ("Elasticity Damping", "", 'float', 0.0, "The amount of damping that gets applied to stretching motions."),
        "physxDeformableSurfaceMaterial:bendStiffness": ("Bend Stiffness", "", 'float', 0.0, "The bending stiffness defines the surface's resistance to bending."),
    }

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        for prop_name, (disp_name, disp_group, prop_type, default_val, docs) in self.prop_data_custom.items():
            if prim.HasAttribute(prop_name):
                prop = UiProp().from_custom(prop_name, disp_name, disp_group, prop_type, default_val, docs)
                filtered_props.append(prop)

        return filtered_props

    def build_items(self):
        super().build_items()

    def _build_property_item(self, stage, prop, prim_paths):
        model = super()._build_property_item(stage, prop, prim_paths)
        return model


class ExtendedDeformableBodyWidget(PhysicsWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
        self._surface_sim_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsSurfaceDeformableSimAPI")
        self._volume_sim_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsVolumeDeformableSimAPI")
        self._ext_surface_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("PhysxSurfaceDeformableBodyAPI")
        self._ext_volume_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("PhysxBaseDeformableBodyAPI")

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        #figure out whether this is for surface or volume deformable
        is_surface = False
        is_volume = False

        if prim.HasAPI(self._surface_sim_type):
            is_surface = True
        if prim.HasAPI(self._volume_sim_type):
            is_volume = True

        children = prim.GetChildren()
        for child in children:
            if child.HasAPI(self._surface_sim_type):
                is_surface = True
            if child.HasAPI(self._volume_sim_type):
                is_volume = True

        if is_surface == is_volume:
            return []

        ext_type = self._ext_surface_type if is_surface else self._ext_volume_type
        adjusted_filtered_props = []
        for prop in filtered_props:
            if not prop.apply_schema or (prop.apply_schema == ext_type):
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props

class ExtendedSurfaceDeformableMaterialWidget(PhysicsWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
        self.base_mat_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("OmniPhysicsDeformableMaterialAPI")
        self.ext_base_mat_type = Usd.SchemaRegistry().GetTypeFromSchemaTypeName("PhysxDeformableMaterialAPI")

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        # remove properties from UsdPhysics.DeformableMaterialAPI and PhysxSchema.PhysxDeformableMaterialAPI
        deformable_base_mat_props = utils.getSchemaPropertyNames(self.base_mat_type)
        deformable_base_mat_props.extend(utils.getSchemaPropertyNames(self.ext_base_mat_type))
        adjusted_filtered_props = []
        for prop in filtered_props:
            if prop.base_name in deformable_base_mat_props:
                pass
            else:
                adjusted_filtered_props.append(prop)

        return adjusted_filtered_props


class ExtendedAttachmentWidgetDeprecated(PhysicsWidget):
    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        remove_props(["collisionFilterIndices0",
                      "collisionFilterIndices1",
                      "filterType0",
                      "filterType1",
                      "pointsActor0",
                      "pointsActor1"])

        return filtered_props


class ExtendedComputeAutoAttachmentWidgetDeprecated(PhysicsWidget):

    prop_data_schema = {
        "physxAutoAttachment:enableDeformableVertexAttachments": ("Attach Overlapping Vertices", "", 'bool', "Enables vertex based attachments"),
        "physxAutoAttachment:deformableVertexOverlapOffset": ("Overlap Offset", "", 'float', "Defines at which distance attachments are created"),
        "physxAutoAttachment:enableRigidSurfaceAttachments": ("Attach Rigid Surface", "", 'bool', "Enables creating attachments on rigid surface"),
        "physxAutoAttachment:rigidSurfaceSamplingDistance": ("Surface Sampling Distance", "", 'float', "Defines distance at which attachments are created on rigid surface"),
        "physxAutoAttachment:enableCollisionFiltering": ("Collision Filtering", "", 'bool', "Enables collision filtering"),
        "physxAutoAttachment:collisionFilteringOffset": ("Filtering Offset", "", 'float', "Defines at which distance elements are being filtered"),
    }

    prop_data_custom = {
        "physxAutoAttachment:maskShapes": ("Mask Shapes", "", '', None, "The union of the shapes defines a mask volume within which attachment points are generated"),
    }

    class AType(enum.Enum):
        WORLD = 0
        RIGID = 1
        DEFORMABLE_BODY = 2
        DEFORMABLE_SURFACE = 3
        PARTICLES = 4

    def is_deformableVertexOverlapOffset_enabled(self):
        return self._bool_controllers["physxAutoAttachment:enableDeformableVertexAttachments"]

    def is_enableRigidSurfaceAttachments_enabled(self):
        return self._all_have_type[self.AType.RIGID]

    def is_rigidSurfaceSamplingDistance_enabled(self):
        return self._bool_controllers["physxAutoAttachment:enableRigidSurfaceAttachments"] and self._all_have_type[self.AType.RIGID]

    def is_collisionFilteringOffset_enabled(self):
        return self._bool_controllers["physxAutoAttachment:enableCollisionFiltering"] and self._none_have_type[self.AType.WORLD]

    def _update_types(self):
        self._all_have_type = {type_key: True for type_key in self.AType}
        self._none_have_type = {type_key: True for type_key in self.AType}

        stage = omni.usd.get_context().get_stage()
        for prim_path in self._payload:
            has_type = {type_key: False for type_key in self.AType}
            attachment = PhysxSchema.PhysxPhysicsAttachment(self._get_prim(prim_path))
            if attachment:
                for t in range(2):
                    targets = attachment.GetActorRel(t).GetTargets()
                    if len(targets) > 0:
                        target_prim = stage.GetPrimAtPath(targets[0])
                        if target_prim:
                            has_type[self.AType.RIGID] = has_type[self.AType.RIGID] or target_prim.HasAPI(UsdPhysics.CollisionAPI) or target_prim.HasAPI(UsdPhysics.RigidBodyAPI)
                            has_type[self.AType.DEFORMABLE_BODY] = has_type[self.AType.DEFORMABLE_BODY] or target_prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI)
                            has_type[self.AType.DEFORMABLE_SURFACE] = has_type[self.AType.DEFORMABLE_SURFACE] or target_prim.HasAPI(PhysxSchema.PhysxDeformableSurfaceAPI)
                            has_type[self.AType.PARTICLES] = has_type[self.AType.PARTICLES] or target_prim.HasAPI(PhysxSchema.PhysxParticleAPI)
                    else:
                        has_type[self.AType.WORLD] = True

            for type_key in self.AType:
                self._all_have_type[type_key] = self._all_have_type[type_key] and has_type[type_key]
                self._none_have_type[type_key] = self._none_have_type[type_key] and not has_type[type_key]


    #only called once for all instances
    def on_new_payload(self, payload):
        if not super().on_new_payload(payload):
            return False

        self._payload = payload
        self._update_types()

        return True

    def _set_widget_state(self):

        for prop_name, (widget, is_enabled_fn_name) in self._state_controlled.items():
            if widget is not None and is_enabled_fn_name is not None:
                is_enabled_fn = getattr(self, is_enabled_fn_name)
                enable_widget(widget, is_enabled_fn())

    def _filter_props_to_build(self, prim):

        #create custom attributes in case they are missing
        maskShapes_rel = prim.GetRelationship("physxAutoAttachment:maskShapes")
        if not maskShapes_rel:
            prim.CreateRelationship("physxAutoAttachment:maskShapes", True)

        filtered_props = super()._filter_props_to_build(prim)

        for prop_name, (disp_name, disp_group, prop_type, docs) in self.prop_data_schema.items():
            if prim.HasAttribute(prop_name) or prim.HasRelationship(prop_name):
                prim_def = utils.getSchemaPrimDef(self._main_schema)
                prop_spec = prim_def.GetSchemaPropertySpec(prop_name)
                prop = UiProp().from_custom(prop_name, disp_name, disp_group, prop_type, prop_spec.default, docs)
                filtered_props.append(prop)

        for prop_name, (disp_name, disp_group, prop_type, default_val, docs) in self.prop_data_custom.items():
            if prim.HasAttribute(prop_name) or prim.HasRelationship(prop_name):
                prop = UiProp().from_custom(prop_name, disp_name, disp_group, prop_type, default_val, docs)
                filtered_props.append(prop)

        return filtered_props

    def build_items(self):

        self._bool_controllers = {}
        self._state_controlled = {}

        super().build_items()

        def on_click():
            stage = omni.usd.get_context().get_stage()
            physxui_interface = get_physxui_interface()
            if stage is None or physxui_interface is None:
                return

            for prim_path in self._payload:
                prim = stage.GetPrimAtPath(prim_path)
                if prim.IsValid() and prim.IsA(PhysxSchema.PhysxPhysicsAttachment):
                    physxui_interface.refresh_attachment(prim_path.pathString)

        ui.Button(f"{REFRESH_GLYPH}", clicked_fn=on_click)

        # 'luckily' changing the attachment actor rels does triggers build_items,
        # so we don't need a callback
        self._update_types()
        self._set_widget_state()

    def _make_bool_controller(self, prop_name, model):

        def changed_bool(prop_name, model):
            if self._bool_controllers[prop_name] != model.as_bool:
                self._bool_controllers[prop_name] = model.as_bool
                self._set_widget_state()

        self._bool_controllers[prop_name] = model.as_bool and not model._ambiguous
        model.add_value_changed_fn(partial(changed_bool, prop_name))

    def _make_state_controlled(self, prop_name, model, is_enabled_fn_name):
            add_disabled_styles(model.value_widget)
            self._state_controlled[prop_name] = (model.value_widget, is_enabled_fn_name)

    def _build_property_item(self, stage, prop, prim_paths):

        model = super()._build_property_item(stage, prop, prim_paths)

        if prop.base_name == "physxAutoAttachment:enableDeformableVertexAttachments":
            self._make_bool_controller(prop.base_name, model)

        if prop.base_name == "physxAutoAttachment:deformableVertexOverlapOffset":
            self._make_state_controlled(prop.base_name, model, "is_deformableVertexOverlapOffset_enabled")

        if prop.base_name == "physxAutoAttachment:enableRigidSurfaceAttachments":
            self._make_bool_controller(prop.base_name, model)
            self._make_state_controlled(prop.base_name, model, "is_enableRigidSurfaceAttachments_enabled")

        if prop.base_name == "physxAutoAttachment:rigidSurfaceSamplingDistance":
            self._make_state_controlled(prop.base_name, model, "is_rigidSurfaceSamplingDistance_enabled")

        if prop.base_name == "physxAutoAttachment:enableCollisionFiltering":
            self._make_bool_controller(prop.base_name, model)

        if prop.base_name == "physxAutoAttachment:collisionFilteringOffset":
            self._make_state_controlled(prop.base_name, model, "is_collisionFilteringOffset_enabled")

        return model


class ExtendedTetrahedralMeshWidgetDeprecated(PhysicsWidget):
    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)
        return filtered_props


class ExtendedAutoDeformableAttachmentWidget(PhysicsWidget):

    prop_data_schema = {
        "physxAutoDeformableAttachment:enableDeformableVertexAttachments": ("Attach Overlapping Vertices", "", 'bool', "Enables vertex based attachments"),
        "physxAutoDeformableAttachment:deformableVertexOverlapOffset": ("Overlap Offset", "", 'float', "Defines at which distance attachments are created"),
        "physxAutoDeformableAttachment:enableRigidSurfaceAttachments": ("Attach Rigid Surface", "", 'bool', "Enables creating attachments on rigid surface"),
        "physxAutoDeformableAttachment:rigidSurfaceSamplingDistance": ("Surface Sampling Distance", "", 'float', "Defines distance at which attachments are created on rigid surface"),
        "physxAutoDeformableAttachment:enableCollisionFiltering": ("Collision Filtering", "", 'bool', "Enables collision filtering"),
        "physxAutoDeformableAttachment:collisionFilteringOffset": ("Filtering Offset", "", 'float', "Defines at which distance elements are being filtered"),
    }

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)
        filtered_props_mod = []

        for prop in filtered_props:
            if prop.base_name == "physxAutoDeformableAttachment:enableRigidSurfaceAttachments":
                pass
            elif prop.base_name == "physxAutoDeformableAttachment:rigidSurfaceSamplingDistance":
                pass
            elif prop.base_name in self.prop_data_schema:
                (disp_name, disp_group, prop_type, docs) = self.prop_data_schema[prop.base_name]
                prim_def = utils.getSchemaPrimDef(self._main_schema)
                prop_spec = prim_def.GetSchemaPropertySpec(prop.base_name)
                prop_mod = UiProp().from_custom(prop.base_name, disp_name, disp_group, prop_type, prop_spec.default, docs)
                filtered_props_mod.append(prop_mod)
            else:
                filtered_props_mod.append(prop)

        return filtered_props_mod

    def build_items(self):

        super().build_items()

        def on_click():
            stage = omni.usd.get_context().get_stage()
            physxui_interface = get_physxui_interface()
            physx_attachment_private_interface = get_physx_attachment_private_interface()
            if stage is None or physxui_interface is None:
                return

            for prim_path in self._payload:
                prim = stage.GetPrimAtPath(prim_path)
                if prim.IsValid():
                    physx_attachment_private_interface.setup_auto_deformable_attachment(prim_path.pathString)
                    physxui_interface.refresh_attachment(prim_path.pathString)

        ui.Button(f"{REFRESH_GLYPH}", clicked_fn=on_click)


    def _build_property_item(self, stage, prop, prim_paths):

        model = super()._build_property_item(stage, prop, prim_paths)
        return model

class ExtendedAttachmentElementFilterWidget(PhysicsWidget):
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)

    def _filter_props_to_build(self, prim):
        filtered_props = [p for p in super()._filter_props_to_build(prim) if p.base_name.startswith("physics:")]
        return filtered_props

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
    def __init__(self, title, schema, builders):
        super().__init__(title, schema, builders)
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


class SingleInstanceWidget(PhysicsWidget):
    def __init__(self, title, schema, instance_name, builders):
        PhysicsWidget.__init__(self, title, schema, builders)
        self._instance_name = instance_name

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)
        return [p for p in filtered_props if p.instance_name == self._instance_name]

    def build_nested_group_frames(self, stage, grouped_props):
        for group in grouped_props.sub_groups.values():
            for prop in group.props:
                self.build_property_item(stage, prop, self._payload)


class ExtendedParticleClothWidgetDeprecated(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def add_props_to_groups(prop_list, group_name):
            nonlocal filtered_props
            for prop in filtered_props:
                if prop.base_name in prop_list:
                    prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = group_name

        add_props_to_groups(["physxParticle:selfCollisionFilter",
                             "physxParticle:particleGroup",
                             "physxDeformable:settlingThreshold",
                             "physxDeformable:sleepDamping",
                             "physxDeformable:solverPositionIterationCount"],
                             "Advanced")

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        remove_props(["physxParticle:restPoints",
                      "physxParticle:springIndices",
                      "physxParticle:springStiffnesses",
                      "physxParticle:springDampings",
                      "physxParticle:springRestLengths"])

        return filtered_props


class ExtendedAutoParticleClothWidgetDeprecated(PhysicsWidget):

    springStretchStiffness_title = "Stretch Stiffness"
    springBendStiffness_title = "Bend Stiffness"
    springShearStiffness_title = "Shear Stiffness"
    springDamping_title = "Spring Damping"

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        if prim.HasAttribute("physxParticle:springStretchStiffness"):
            filtered_props.append(UiProp().from_custom("physxParticle:springStretchStiffness",
                                  ExtendedAutoParticleClothWidgetDeprecated.springStretchStiffness_title, "", 'float', True))

        if prim.HasAttribute("physxParticle:springBendStiffness"):
            filtered_props.append(UiProp().from_custom("physxParticle:springBendStiffness",
                                  ExtendedAutoParticleClothWidgetDeprecated.springBendStiffness_title, "", 'float', True))

        if prim.HasAttribute("physxParticle:springShearStiffness"):
            filtered_props.append(UiProp().from_custom("physxParticle:springShearStiffness",
                                  ExtendedAutoParticleClothWidgetDeprecated.springShearStiffness_title, "", 'float', True))

        if prim.HasAttribute("physxParticle:springDamping"):
            filtered_props.append(UiProp().from_custom("physxParticle:springDamping",
                                  ExtendedAutoParticleClothWidgetDeprecated.springDamping_title, "", 'float', True))

        return filtered_props

    def build_items(self):
        super().build_items()

    def _build_property_item(self, stage, prop, prim_paths):
        model = super()._build_property_item(stage, prop, prim_paths)
        return model


class ExtendedParticleSystemWidget(PhysicsWidget):

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def add_props_to_groups(prop_list, group_name):
            nonlocal filtered_props
            for prop in filtered_props:
                if prop.base_name in prop_list:
                    prop.metadata[Sdf.PropertySpec.DisplayGroupKey] = prop.display_group = group_name

        add_props_to_groups(["contactOffset",
                             "restOffset",
                             "solidRestOffset",
                             "fluidRestOffset"],
                             "Advanced")

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        gprim_props = UsdGeom.Gprim.GetSchemaAttributeNames()
        gprim_props.append('proxyPrim')
        gprim_props.append('globalSelfCollisionEnabled')
        gprim_props.append('nonParticleCollisionEnabled')
        remove_props(gprim_props)

        return filtered_props


class ExtendedMimicJointWidget(PhysicsWidget):

    
    prop_data_custom = {
        MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTX: ("Natural Frequency", "rotX", 'float', 0.0, "The natural frequency of mimic joint compliance. A mimic joint with naturalFrequency <= 0 will behave as a hard constraint. Larger values of naturalFrequency and dampingRatio will make the mimic joint stiffer and more akin to a hard constraint."),
        MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTY: ("Natural Frequency", "rotY", 'float', 0.0, "The natural frequency of mimic joint compliance. A mimic joint with naturalFrequency <= 0 will behave as a hard constraint. Larger values of naturalFrequency and dampingRatio will make the mimic joint stiffer and more akin to a hard constraint."),
        MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTZ: ("Natural Frequency", "rotZ", 'float', 0.0, "The natural frequency of mimic joint compliance. A mimic joint with naturalFrequency <= 0 will behave as a hard constraint. Larger values of naturalFrequency and dampingRatio will make the mimic joint stiffer and more akin to a hard constraint."),
        MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTX: ("Damping Ratio", "rotX", 'float', 0.0, "The damping ratio of mimic joint compliance. A mimic joint with dampingRatio <= 0 will behave as a hard constraint. Larger values of naturalFrequency and dampingRatio will make the mimic joint stiffer and more akin to a hard constraint."),
        MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTY: ("Damping Ratio", "rotY", 'float', 0.0, "The damping ratio of mimic joint compliance. A mimic joint with dampingRatio <= 0 will behave as a hard constraint. Larger values of naturalFrequency and dampingRatio will make the mimic joint stiffer and more akin to a hard constraint."),
        MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTZ: ("Damping Ratio", "rotZ", 'float', 0.0, "The damping ratio of mimic joint compliance. A mimic joint with dampingRatio <= 0 will behave as a hard constraint. Larger values of naturalFrequency and dampingRatio will make the mimic joint stiffer and more akin to a hard constraint."),
    }

    def _filter_props_to_build(self, prim):

        filtered_props = super()._filter_props_to_build(prim)

        for prop_name, (disp_name, disp_group, prop_type, default_val, docs) in self.prop_data_custom.items():
            mimicJoint = PhysxSchema.PhysxMimicJointAPI(prim, disp_group)
            if mimicJoint:
                prop = UiProp().from_custom(prop_name, disp_name, disp_group, prop_type, default_val, docs)
                filtered_props.append(prop)

        return filtered_props


    def _build_property_item(self, stage, prop, prim_paths):

        model = super()._build_property_item(stage, prop, prim_paths)

        if (prop.base_name == "referenceJoint"):
            def changed(model):
                self.request_rebuild()

            # changing the reference joint can change the decision whether the
            # reference joint axis attribute widget should be enabled/disabled.
            # Hence, a rebuild should get triggered by any change.
            model.value_model.add_value_changed_fn(changed)

        elif (prop.base_name == "referenceJointAxis"):
            # The reference joint axis attribute widget should be disabled if the reference
            # joint is a single degree of freedom joint (prismatic/revolute) because the
            # axis is implicitly defined in that case

            hasSingleDofRefJoints = False
            for primPath in prim_paths:
                prim = stage.GetPrimAtPath(primPath)
                mimicJoint = PhysxSchema.PhysxMimicJointAPI(prim, prop.instance_name)
                refJointRelTargets = mimicJoint.GetReferenceJointRel().GetTargets()
                if ((refJointRelTargets is not None) and (len(refJointRelTargets) > 0)):
                    refJointPath = refJointRelTargets[0]
                    refJointPrim = stage.GetPrimAtPath(refJointPath)
                    if (refJointPrim.IsValid() and (refJointPrim.IsA(UsdPhysics.PrismaticJoint) or refJointPrim.IsA(UsdPhysics.RevoluteJoint))):
                        hasSingleDofRefJoints = True
                        break

            if hasSingleDofRefJoints:
                add_disabled_styles(model.value_widget)
                enable_widget(model.value_widget, False)

        return model
