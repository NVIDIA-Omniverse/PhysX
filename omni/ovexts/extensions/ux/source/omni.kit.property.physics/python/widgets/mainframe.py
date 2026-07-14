# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from collections import defaultdict
import omni.kit.context_menu
from omni.kit.window.property.templates import SimplePropertyWidget
import carb
import omni.ui as ui

class MainFrameWidget(SimplePropertyWidget):
    name = "physx_main_frame_widget"
    instance = None
    popup_populate_fns = {}

    class SubWidget:
        def __init__(self, name, widget, parent_schema: str | None = None, after_widget: str | None = None):
            self.name = name
            self.enabled = False
            self.inactive = False
            self.widget = widget
            self.parent_schema = parent_schema
            self.after_widget = after_widget

    def __init__(self):
        super().__init__("Physics")
        self._subwidgets = [dict(), dict(), dict()]
        self._parent_schema_active: dict[str, bool] = {}
        self._any_visible = False
        self._refresh_enabled = False
        MainFrameWidget.instance = self
        self._add_popup_menu()

        # shared checks
        self.is_a_omnijoint = False

        self._listener = None

    def clean(self):
        super().clean()
        self._popup_menu = None
        self._ext_popup_menus = {}
        MainFrameWidget.instance = None

    # FIXME: make this a delayed request so it can be called on each widget register
    def refresh_apis(self):
        for subwidget in self.get_subwidgets():
            if hasattr(subwidget.widget, "refresh_apis"):
                subwidget.widget.refresh_apis()
        self.request_rebuild()

    def get_subwidgets(self):
        for p in self._subwidgets:
            for widget in p.values():
                yield widget

    def _get_visible_subwidgets_by_inactive(self, inactive: bool):
        for p in self._subwidgets:
            for widget in p.values():
                if widget.enabled and widget.inactive == inactive:
                    yield widget

    def get_subwidget_by_title(self, title):
        for subwidget in self.get_subwidgets():
            if subwidget.widget._title == title:
                return subwidget.widget
        return None

    def set_parent_schema_active(self, parent_schema: str, active: bool):
        if self._parent_schema_active.get(parent_schema, True) == active:
            return

        self._parent_schema_active[parent_schema] = active
        for subwidget in self.get_subwidgets():
            if subwidget.parent_schema == parent_schema:
                subwidget.inactive = not active
        self.request_rebuild()

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

    @staticmethod
    def _sort_with_after_constraints(subwidgets):
        """Sort subwidgets so that those with after_widget appear right after the referenced widget."""
        after_map = defaultdict(list)
        regular = []

        for sw in subwidgets:
            if sw.after_widget is not None:
                after_map[sw.after_widget].append(sw)
            else:
                regular.append(sw)

        result = []

        def _insert_with_children(sw):
            result.append(sw)
            for child in after_map.pop(sw.name, []):
                _insert_with_children(child)

        for sw in regular:
            _insert_with_children(sw)

        # append orphans whose parent was not found in the visible list
        for remaining in after_map.values():
            result.extend(remaining)

        return result

    def build_items(self):
        if self._refresh_enabled:
            # refresh enabled data before rebuilding, visibility might have changed
            self.on_new_payload(self._payload)

        self._any_item_visible = self._any_visible

        if not self._any_visible:
            return

        active_visible = self._sort_with_after_constraints(
            list(self._get_visible_subwidgets_by_inactive(False))
        )
        inactive_visible = self._sort_with_after_constraints(
            list(self._get_visible_subwidgets_by_inactive(True))
        )

        for subwidget in active_visible:
            subwidget.widget._filter = self._filter
            subwidget.widget.build_impl()

        if inactive_visible:
            inactive_frame = ui.CollapsableFrame("Inactive", collapsed=True)
            with inactive_frame:
                frame = ui.Frame()
                frame.enabled = True
                with frame:
                    for subwidget in inactive_visible:
                        subwidget.widget._filter = self._filter
                        subwidget.widget.build_impl()

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

        self._any_visible = False
        for subwidget in self.get_subwidgets():
            subwidget.enabled = subwidget.widget.on_new_payload(payload)
            self._any_visible |= subwidget.enabled

        # do shared checks
        if self._any_visible:
            self.is_a_omnijoint = self.is_a_omnijoint_check(payload)

        # we want to be there always so on change we can just rebuild this widget instead
        # of the whole property window
        return True

    def register_widget(self, name, widget, position=1, parent_schema: str | None = None, after_widget: str | None = None):
        if name in self._subwidgets:
            carb.log_error(f"'{name}' widget is already registered in the Physics property window frame. Please fix ASAP, your unregister will remove another widget!")
        else:
            subwidget = MainFrameWidget.SubWidget(name, widget, parent_schema=parent_schema, after_widget=after_widget)
            if parent_schema is not None:
                subwidget.inactive = not self._parent_schema_active.get(parent_schema, True)
            self._subwidgets[position][name] = subwidget

    def unregister_widget(self, name, skip_warn=False):
        for p in self._subwidgets:
            if name in p:
                p.pop(name)
                return
        if not skip_warn:
            carb.log_warning(f"'{name}' widget not found in the Physics property window frame. Please fix ASAP!")
