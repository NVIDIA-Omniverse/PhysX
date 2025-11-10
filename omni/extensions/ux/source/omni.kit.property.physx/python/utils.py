# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import re
from pxr import Usd, Sdf, Gf, Tf, UsdPhysics, PhysxSchema, UsdGeom, Plug
import math
import omni.kit.window.property
import omni.kit.window.property as property_window
from omni.physxui.scripts import utils
import omni.usd
from .externals import Prompt
from functools import partial
import omni.ui
import asyncio
from collections import namedtuple
import carb


class Limits:
    FLT_INF = float("inf")
    FLT_NINF = float("-inf")
    FLT_MAX = 3.402823e+38
    FLT_NMAX = -3.402823e+38
    FLT_MIN = 1.17549435e-38
    DEG_MAX = 360
    RAD_MAX = 2 * math.pi
    INT_MAX = 2147483647

    # compound limits
    RANGE_FLT_NONNEG = [0.0, FLT_INF]
    RANGE_FLT_POS = [FLT_MIN, FLT_INF]
    RANGE_DEG = [-DEG_MAX, DEG_MAX]
    RANGE_RAD = [-RAD_MAX, RAD_MAX]

    INFS = [FLT_INF, FLT_NINF, FLT_MAX, FLT_NMAX, FLT_MIN]


# request to rebuild property window on next draw, it's ok to call multiple times
def rebuild_property_window():
    omni.kit.window.property.get_window()._window.frame.rebuild()


def get_display_name(metadata, default):
    return metadata.get(Sdf.PropertySpec.DisplayNameKey, default)


def split_at_capitals(title):
    return re.findall('.[^A-Z]*', title)


def get_schema_name(schema_type):
    if isinstance(schema_type, str):
        return schema_type
    elif isinstance(schema_type, Tf.Type):
        return Usd.SchemaRegistry().GetSchemaTypeName(schema_type)
    else:
        return schema_type.__name__


def get_widget_name(schema_type):
    return "physx_" + get_schema_name(schema_type)


def get_title_from_schema(schema_type):
    name = get_schema_name(schema_type)
    start = None
    if name.startswith("Physx"):
        start = 5
    elif name.startswith("Physics"):
        start = 7
    end = None
    if name.endswith("API"):
        end = -3
    return ' '.join(split_at_capitals(name[start:end]))


class StringPrompt(Prompt):

    def set_warning(self, warning):
        self._warning_label.text = warning

    def _on_ok_button_fn(self):
        value = self._string_field.model.get_value_as_string()
        if value.isidentifier():
            self.hide()
            if self._ok_button_fn:
                self._ok_button_fn(self._string_field.model.get_value_as_string())
        else:
            self.set_warning("The entered string must be a valid C/Python identifier!")

    def _build_ui(self):
        self._window = omni.ui.Window(
            self._title, visible=False, height=0, dockPreference=omni.ui.DockPreference.DISABLED
        )
        self._window.flags = (
            omni.ui.WINDOW_FLAGS_NO_COLLAPSE
            | omni.ui.WINDOW_FLAGS_NO_RESIZE
            | omni.ui.WINDOW_FLAGS_NO_SCROLLBAR
            | omni.ui.WINDOW_FLAGS_NO_RESIZE
            | omni.ui.WINDOW_FLAGS_NO_MOVE
        )

        if self._modal:
            self._window.flags = self._window.flags | omni.ui.WINDOW_FLAGS_MODAL

        with self._window.frame:
            with omni.ui.VStack(height=0):
                omni.ui.Spacer(width=0, height=10)
                with omni.ui.HStack(height=0):
                    omni.ui.Spacer()
                    self._text_label = omni.ui.Label(self._text, word_wrap=True, width=self._window.width - 80, height=0)
                    omni.ui.Spacer()
                omni.ui.Spacer(width=0, height=10)
                with omni.ui.HStack(height=0):
                    omni.ui.Spacer()
                    self._string_field = omni.ui.StringField(width=self._window.width - 80, height=0)
                    omni.ui.Spacer()
                omni.ui.Spacer(width=0, height=10)
                with omni.ui.HStack(height=0):
                    omni.ui.Spacer()
                    self._warning_label = omni.ui.Label("", word_wrap=True, width=self._window.width - 80, 
                                                        height=0, style={"color": 0xBB4444FF})
                    omni.ui.Spacer()
                omni.ui.Spacer(width=0, height=10)
                with omni.ui.HStack(height=0):
                    omni.ui.Spacer(height=0)
                    if self._ok_button_text:
                        ok_button = omni.ui.Button(self._ok_button_text, width=60, height=0)
                        ok_button.set_clicked_fn(self._on_ok_button_fn)
                    if self._middle_button_text:
                        middle_button = omni.ui.Button(self._middle_button_text, width=60, height=0)
                        middle_button.set_clicked_fn(self._on_middle_button_fn)
                    if self._cancel_button_text:
                        cancel_button = omni.ui.Button(self._cancel_button_text, width=60, height=0)
                        cancel_button.set_clicked_fn(self._on_cancel_button_fn)
                    omni.ui.Spacer(height=0)
                omni.ui.Spacer(width=0, height=10)


class OverlayButton(omni.ui.Button):
    def __init__(self, text, collapsable_frame, pressed_fn, **kwargs):
        super().__init__(text, **kwargs)

        self._revert_last = False
        self._collapsable_frame = collapsable_frame

        def on_collapsed_changed(collapsed):
            if self._revert_last:
                self._revert_last = False
                self._collapsable_frame.collapsed = not collapsed

        def on_pressed(*_):
            self._revert_last = True
            pressed_fn()

        self.set_mouse_pressed_fn(on_pressed)
        self._collapsable_frame.set_collapsed_changed_fn(on_collapsed_changed)


# add more disabled styles if using on other widgets than supported and kit had not already moved
# to better styling (in which case add_disabled_styles should be deprecated and removed anyway)
disabled_styles = {
    "ComboBox:disabled": {"color": omni.ui.color.grey},
    "CheckBox::greenCheck:disabled": {"font_size": 12, "border_radius": 1.5},
    "Slider:disabled": {"color": omni.ui.color.grey},
    "Label:disabled": {"color": omni.ui.color.grey},
}


def add_disabled_styles(widget):
    if widget.style:
        tmp = {}
        tmp.update(widget.style)
        tmp.update(disabled_styles)
        widget.set_style(tmp)
    else:
        widget.set_style(disabled_styles)


# NOTE: also use add_disabled_styles on the widget you want to use this on, otherwise you will run into visual issues
def enable_widget(widget, enable):
    async def delayed_enable():
        await omni.kit.app.get_app().next_update_async()
        widget.enabled = enable

    asyncio.ensure_future(delayed_enable())


def scroll_property_window_to_physx_component(component_name: str, num_update_delay_cycles: int = 5):
    widget_name = f"physx_{component_name}"

    async def delayed_scroll():
        # allow the property window to build first - sync a couple of frames
        for _ in range(num_update_delay_cycles):
            await omni.kit.app.get_app().next_update_async()
        if widget_name not in property_window.get_window()._widgets_top["prim"]:
            return
        widget = property_window.get_window()._widgets_top["prim"][widget_name]
        if widget._collapsable_frame is not None:
            # preist: scroll here y does not work properly if not in fullscreen
            # bug filed: OM-32251
            widget._collapsable_frame.scroll_here_y(0.0)  # 0.0 = top

    asyncio.ensure_future(delayed_scroll())


def get_align_property_util(text, body0base, widget, fixpos, fixrot):
    def align(fixpos, fixrot):
        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.XformCache()
        for path in widget._payload:
            prim = stage.GetPrimAtPath(path)
            utils.set_opposite_body_transform(stage, cache, prim, body0base, fixpos, fixrot)

    i = "1" if body0base else "0"
    return {f"Align {text} to Body {i}": partial(align, fixpos, fixrot)}


def generate_schema_aliases(aliases, schema_parent, prefix):
    prefix_len = len(prefix)
    for a in schema_parent.__dict__.values():
        if hasattr(a, "_GetStaticTfType"):
            aliases[a] = a._GetStaticTfType().typeName[prefix_len:]


def generate_codeless_schema_aliases(aliases, plugin_name):
    reg = Usd.SchemaRegistry()
    plugin = Plug.Registry().GetPluginWithName(plugin_name)
    if plugin is None:
        carb.log_warn(f"generate_codeless_schema_aliases: Could not load Plugin {plugin_name}")
        return
    all_types = Tf.Type.Find(Usd.SchemaBase).GetAllDerivedTypes()
    for tf_type in all_types:
        schema_name = reg.GetSchemaTypeName(tf_type)
        if plugin.DeclaresType(tf_type):
            aliases[tf_type] = schema_name


def is_parent_api_instance(api_name, instance_name, payload):
    """
    if we cannot find the api:instance name in the applied api schemas list
    it means this one is here because of a child API that has its own widget
    and the properties are also shown there, so we skip this one
    """
    first_prim = payload.get_stage().GetPrimAtPath(payload[0])
    schema_instance_name = f"{api_name}:{instance_name}"
    return schema_instance_name not in first_prim.GetPrimTypeInfo().GetAppliedAPISchemas()
