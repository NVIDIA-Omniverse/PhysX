# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = [
    "register_parent_schema",
    "register_parent_schema_group",
    "unregister_parent_schema",
    "unregister_parent_schema_group",
    "register_schema",
    "unregister_schema",
    "activate_parent_schema",
    "activate_parent_schema_group",
    "deactivate_parent_schema",
    "deactivate_parent_schema_group",
    "is_parent_schema_group_registered",
    "is_parent_schema_registered",
    "register_widget",
    "unregister_widget",
    "register_custom_property_widget",
    "unregister_custom_property_widget",
    "register_custom_property",
    "unregister_custom_property",
    "refresh"
]

from .utils import get_title_from_schema, rebuild_property_window, get_widget_name
from .widgets import (
    PhysicsWidget,
    PhysicsMaterialBindingWidget,
    JointVisualizationWidget,
    PhysicsDefaultMaterialBindingWidget,
    CustomPropertiesWidget,
)
from .manager import Manager
from . import usdphysics
from . import database
import omni.ext
import omni.kit.property.usd as p_usd
import carb


def register_parent_schema(parent_schema, schema_title, widgets=None, builders=None, order=None, extensions=None, extras=None, ignore=None, internal_extensions=None):
    if Manager.instance is not None:
        Manager.instance.register_parent_schema(parent_schema, schema_title, widgets, builders, order, extensions, extras, ignore, internal_extensions)


def register_parent_schema_group(group_name: str, parent_schemas):
    if Manager.instance is not None:
        Manager.instance.register_parent_schema_group(group_name, parent_schemas)


def unregister_parent_schema(parent_schema):
    if Manager.instance is not None:
        Manager.instance.unregister_parent_schema(parent_schema)


def unregister_parent_schema_group(group_name: str):
    if Manager.instance is not None:
        Manager.instance.unregister_parent_schema_group(group_name)


def activate_parent_schema(parent_schema):
    if Manager.instance is not None:
        Manager.instance.activate_parent_schema(parent_schema)


def deactivate_parent_schema(parent_schema):
    if Manager.instance is not None:
        Manager.instance.deactivate_parent_schema(parent_schema)


def activate_parent_schema_group(group_name: str):
    if Manager.instance is not None:
        Manager.instance.activate_parent_schema_group(group_name)

def deactivate_parent_schema_group(group_name: str):
    if Manager.instance is not None:
        Manager.instance.deactivate_parent_schema_group(group_name)


def is_parent_schema_group_registered(group_name: str):
    if Manager.instance is not None:
        return Manager.instance.is_parent_schema_group_registered(group_name)
    return False

def is_parent_schema_registered(parent_schema: str):
    if Manager.instance is not None:
        return Manager.instance.is_parent_schema_registered(parent_schema)
    return False

# FIXME: update this to use the new structure
def register_schema(schema, order=None, component=None, custom_widget=None, prefix=None, alias=None):
    carb.error("register_schema is currently deprecated. Use register_parent_schema instead.")
    if Manager.instance is None:
        return

    title = get_title_from_schema(schema)

    if order is not None:
        database.add_order(schema, order)

    if prefix is not None and schema not in database.ext_multi_api_prefixes:
        database.ext_multi_api_prefixes[schema] = prefix

    if component is not None:
        title = component.title
        database.components[schema.__name__] = component

    if alias is not None:
        p_usd.register_schema(f"PhysicsWidgets_{schema.__name__}", [alias], options=p_usd.RegisteredSchemaCodes.PRIVATE)

    component = database.components.get(schema.__name__)
    widget_class = PhysicsWidget if custom_widget is None else custom_widget
    widget = widget_class(title, schema)

    Manager.instance._frame_widget.register_widget(get_widget_name(schema), widget)
    Manager.instance._invisible_widget.refresh_menu_items()
    rebuild_property_window()


# FIXME: update this to use the new structure
def unregister_schema(schema):
    if Manager.instance is None:
        return

    database.remove_order(schema)
    database.components.pop(schema.__name__, None)
    database.ext_multi_api_prefixes.pop(schema, None)

    schemas = p_usd.get_registered_schemas()
    widget_name = f"PhysicsWidgets_{schema.__name__}"
    if widget_name in schemas:
        del schemas[widget_name]

    Manager.instance._frame_widget.unregister_widget(get_widget_name(schema))
    Manager.instance._invisible_widget.refresh_menu_items()
    rebuild_property_window()


def register_widget(name, widget, after_widget: str | None = None):
    if Manager.instance is not None:
        Manager.instance.register_widget(name, widget, after_widget=after_widget)


def unregister_widget(name):
    if Manager.instance is not None:
        Manager.instance.unregister_widget(name)


def register_custom_property_widget(
    widget_title: str,
    parent_schema: str | None = None,
    visible_when=None,
    position: int = 1,
    widget_name: str | None = None,
):
    """
    Register a "custom properties widget" into the Physics mainframe.

    - **widget_title**: Title of the collapsible section in the property window.
    - **parent_schema**: Parent schema group name (used for active/inactive grouping).
    - **visible_when**: Callable(prim)->bool; must return True for *all* selected prims.
    - **position**: Mainframe position bucket (same meaning as other widgets).
    - **widget_name**: Optional unique widget id; defaults to a sanitized name from title.
    """
    if Manager.instance is None:
        return

    d = database.register_custom_property_widget(
        widget_title,
        parent_schema=parent_schema,
        visible_when=visible_when,
        position=position,
        widget_name=widget_name,
    )

    # Re-register to allow updating parent_schema/condition.
    Manager.instance._frame_widget.unregister_widget(d.widget_name, skip_warn=True)

    widget = CustomPropertiesWidget(widget_title, widget_title_key=widget_title)
    Manager.instance.register_widget(d.widget_name, widget, position=position, parent_schema=parent_schema)
    rebuild_property_window()


def unregister_custom_property_widget(widget_title: str):
    if Manager.instance is None:
        database.unregister_custom_property_widget(widget_title)
        return

    d = database.get_custom_property_widget(widget_title)
    if d is not None and d.widget_name is not None:
        Manager.instance._frame_widget.unregister_widget(d.widget_name, skip_warn=True)

    database.unregister_custom_property_widget(widget_title)
    rebuild_property_window()


def register_custom_property(
    usd_name: str,
    display_name: str,
    widget_title: str,
    *,
    display_group: str = "",
    type_name: str = "string",
    default="",
    doc: str = "",
):
    """
    Register a custom property (USD attribute name + UI label) to be displayed
    inside a previously registered custom properties widget.
    """
    database.register_custom_property(
        usd_name=usd_name,
        display_name=display_name,
        display_group=display_group,
        type_name=type_name,
        default=default,
        doc=doc,
        widget_title=widget_title,
    )
    if Manager.instance is not None:
        Manager.instance.refresh()
        rebuild_property_window()


def unregister_custom_property(usd_name: str, widget_title: str):
    database.unregister_custom_property(usd_name=usd_name, widget_title=widget_title)
    if Manager.instance is not None:
        Manager.instance.refresh()
        rebuild_property_window()


def refresh():
    if Manager.instance is not None:
        Manager.instance.refresh()


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.manager = Manager()

        self.manager.register_parent_schema(
            "UsdPhysics", None,
            usdphysics.widgets,
            usdphysics.property_builders,
            usdphysics.property_order,
            usdphysics.extensions,
            usdphysics.extras,
            usdphysics.ignore,
            usdphysics.internal_extensions,
        )

        self.manager.register_widget(JointVisualizationWidget.name, JointVisualizationWidget(), position=2)
        self.manager.register_widget(PhysicsMaterialBindingWidget.name, PhysicsMaterialBindingWidget(), position=2)
        self.manager.register_widget(PhysicsDefaultMaterialBindingWidget.name, PhysicsDefaultMaterialBindingWidget(), position=2)

    def on_shutdown(self):
        self.manager.on_shutdown()
        del self.manager
