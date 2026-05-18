# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from .. import database
from ..utils import split_at_capitals, StringPrompt
from .. import databaseUtils as dbutils
from .uiprop import UiProp
import omni.usd
import omni.physx.bindings._physx as pxb
from omni.kit.commands import execute
from omni.physxui import get_physxui_interface
from omni.physx.scripts.utils import get_schema_instances, get_TfType_compatible
import carb
from sys import maxsize
from pxr import Usd, Tf
from pathlib import Path
from omni.kit.property.usd.widgets import ICON_PATH


ADD_GLYPH = omni.kit.ui.get_custom_glyph_code("${glyphs}/menu_context.svg")
EYE_GLYPH = omni.kit.ui.get_custom_glyph_code("${glyphs}/eye.svg")
REFRESH_GLYPH = omni.kit.ui.get_custom_glyph_code("${glyphs}/menu_refresh.svg")
REMOVE_BUTTON_STYLE = style = {"image_url": str(Path(ICON_PATH).joinpath("remove.svg")), "margin": 0, "padding": 0}


def sort_props(props, schemas):
    order = []

    for schema in schemas:
        order += database.get_property_order(schema)

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
