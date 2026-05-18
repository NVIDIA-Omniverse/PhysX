# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from .builders import ModelWithWidgetBuilder
from .widgets import PhysicsWidget, ExtensionSchemaWidget, InvisibleWidget, MainFrameWidget
from .utils import get_title_from_schema, get_schema_name, get_widget_name, list_schema_api_and_prim_classes, print_schema_api_and_prim_classes
from . import database
from omni.physx.scripts.utils import get_TfType_compatible
from pxr import Usd
import omni.kit.window.property as p
import omni.kit.property.usd as p_usd
import carb


class Manager:
    scheme = "prim"
    instance = None

    def __init__(self):
        Manager.instance = self

        self._frame_widget = MainFrameWidget()
        self._invisible_widget = InvisibleWidget()
        self._parent_schemas = set()
        self._parent_schema_groups = dict()

        w = p.get_window()

        w.register_widget(Manager.scheme, MainFrameWidget.name, self._frame_widget)
        w.register_widget(Manager.scheme, InvisibleWidget.name, self._invisible_widget)

    def on_shutdown(self):
        w = p.get_window()

        w.unregister_widget(Manager.scheme, MainFrameWidget.name)
        w.unregister_widget(Manager.scheme, InvisibleWidget.name)

        self._invisible_widget = None
        self._frame_widget = None
        ModelWithWidgetBuilder.shutdown()
        Manager.instance = None

    def register_widget(self, name, widget, position=1, parent_schema: str | None = None, after_widget: str | None = None):
        self._frame_widget.register_widget(name, widget, position, parent_schema=parent_schema, after_widget=after_widget)

    def unregister_widget(self, name):
        self._frame_widget.unregister_widget(name)

    def refresh(self):
        self._invisible_widget.refresh_menu_items()
        self._frame_widget.request_rebuild()

    def set_parent_schema_active(self, parent_schema: str, active: bool):
        self._frame_widget.set_parent_schema_active(parent_schema, active)
        self.refresh()

    def activate_parent_schema(self, parent_schema: str):
        self.set_parent_schema_active(parent_schema, True)

    def deactivate_parent_schema(self, parent_schema: str):
        self.set_parent_schema_active(parent_schema, False)

    def activate_parent_schema_group(self, group_name: str):
        parent_schema_group = self._parent_schema_groups.get(group_name)
        if parent_schema_group is None:
            carb.log_error(f"Parent schema group {group_name} not registered")
            return

        for parent_schema in parent_schema_group:
            self.activate_parent_schema(parent_schema)

    def deactivate_parent_schema_group(self, group_name: str):
        parent_schema_group = self._parent_schema_groups.get(group_name)
        if parent_schema_group is None:
            carb.log_error(f"Parent schema group {group_name} not registered")
            return

        for parent_schema in parent_schema_group:
            self.deactivate_parent_schema(parent_schema)

    def is_parent_schema_group_registered(self, group_name: str):
        return group_name in self._parent_schema_groups

    def is_parent_schema_registered(self, parent_schema: str):
        return parent_schema in self._parent_schemas

    def register_parent_schema(self, parent_schema: str, schema_title: str, widgets: dict | None = None, builders: dict | None = None, order: dict | None = None, extensions: dict | None = None, extras: dict | None = None, ignore: dict | None = None, internal_extensions: dict | None = None):
        if widgets is None:
            widgets = {}
        if builders is None:
            builders = {}
        if order is None:
            order = {}
        if extensions is None:
            extensions = {}
        if extras is None:
            extras = {}
        if ignore is None:
            ignore = {}
        if internal_extensions is None:
            internal_extensions = {}

        data = list_schema_api_and_prim_classes(parent_schema)

        self._parent_schemas.add(parent_schema)

        database.add_schema_aliases(parent_schema, {d['python_class']: d['schemaTypeName'] for t in ["prim", "api"] for d in data[t] if d["python_class"]})
        # FIXME: refactor so that for strings the string itself is usually the alias?
        database.add_schema_aliases(parent_schema, {d['schemaTypeName']: d['schemaTypeName'] for t in ["prim", "api"] for d in data[t] if not d["python_class"]})
        #
        database.add_property_builders(parent_schema, builders)
        database.add_property_orders(parent_schema, order)
        for api, extension_apis in extensions.items():
            database.add_extension_apis(parent_schema, api, extension_apis)
        for api, extras_apis in extras.items():
            database.add_extras_apis(parent_schema, api, extras_apis)
        for api, int_ext_apis in internal_extensions.items():
            database.add_internal_extension_apis(parent_schema, api, int_ext_apis)

        # Build reverse map: extension API schema type name -> parent API    
        def build_extension_to_parent(extensions_dict):
            extension_to_parent = {}
            for parent_api, ext_apis in extensions_dict.items():
                for ext_api in ext_apis:
                    if isinstance(ext_api, str):
                        ext_type_name = ext_api
                    else:
                        ext_type_name = Usd.SchemaRegistry().GetSchemaTypeName(get_TfType_compatible(ext_api))
                    extension_to_parent[ext_type_name] = parent_api
            return extension_to_parent

        extension_to_parent = build_extension_to_parent(extensions)
        internal_extension_to_parent = build_extension_to_parent(internal_extensions)

        def register_widget(d):
            schema_type_name = d['schemaTypeName']
            schema_class = d["python_class"] if d["python_class"] else d['schemaTypeName']
            if schema_class in ignore or schema_type_name in internal_extension_to_parent.keys():
                return

            component = database.components.get(get_schema_name(schema_class))
            title = component.title if component else get_title_from_schema(schema_class)
            if schema_title is not None:
                title = f"{title} ({schema_title})"

            parent_api = extension_to_parent.get(d['schemaTypeName'])
            if parent_api is not None:
                widget_instance = widgets.get(schema_class, ExtensionSchemaWidget)(title, schema_class, parent_api)
                after_widget = get_widget_name(parent_api)
            else:
                widget_instance = widgets.get(schema_class, PhysicsWidget)(title, schema_class)
                after_widget = None

            self._frame_widget.register_widget(get_widget_name(schema_class), widget_instance, parent_schema=parent_schema, after_widget=after_widget)

        for t in ["prim", "api"]:
            data[t] = sorted(data[t], key=lambda x: database.priority_dict.get(x['schemaTypeName'], 0))
            for d in data[t]:
                register_widget(d)

        schema_names = [d['schemaTypeName'] for t in ["prim", "api"] for d in data[t]]
        p_usd.register_schema(f"PhysicsWidgets_{parent_schema}", schema_names, options=p_usd.RegisteredSchemaCodes.PRIVATE)

        self._frame_widget.refresh_apis()

    def unregister_parent_schema(self, parent_schema):
        # FIXME: we should return some registration info so that we can unregister the widgets instead of relisting
        data = list_schema_api_and_prim_classes(parent_schema)
        for t in ["prim", "api"]:
            for d in data[t]:   
                schema_class = d["python_class"] if d["python_class"] else d['schemaTypeName']
                self._frame_widget.unregister_widget(get_widget_name(schema_class), skip_warn=True)

        database.remove_schema_aliases(parent_schema)
        database.remove_property_orders(parent_schema)
        database.remove_extension_apis(parent_schema)
        database.remove_extras_apis(parent_schema)
        database.remove_internal_extension_apis(parent_schema)

        schemas = p_usd.get_registered_schemas()
        if f"PhysicsWidgets_{parent_schema}" in schemas:
            del schemas[f"PhysicsWidgets_{parent_schema}"]

        self._parent_schemas.discard(parent_schema)

        self._frame_widget.refresh_apis()

    def register_parent_schema_group(self, group_name, parent_schemas):
        self._parent_schema_groups[group_name] = parent_schemas

    def unregister_parent_schema_group(self, group_name):
        self._parent_schema_groups.pop(group_name, None)
