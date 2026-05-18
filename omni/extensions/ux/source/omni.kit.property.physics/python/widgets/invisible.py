# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from .utils import generate_prims_with_cache, generate_from_prim_list_with_cache, add_component
from .mainframe import MainFrameWidget
from .. import database
from omni.kit.window.property.templates import SimplePropertyWidget
from omni.kit.property.usd import PrimPathWidget
from collections import namedtuple
import omni.kit.undo
import carb
from functools import partial
import asyncio
import omni.physx.bindings._physx as pxb


# this widget is a base for invisible Add menu widgets
# * fill the add menu with components and extra items
# * refresh the property window on undo/redo of apply/unapply commands
class InvisibleMenuWidgetBase(SimplePropertyWidget):
    Info = namedtuple("Info", "show, enable")

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
