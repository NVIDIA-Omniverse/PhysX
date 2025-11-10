# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from typing import List
from functools import partial
import asyncio
import carb
from carb.eventdispatcher import get_eventdispatcher
import omni.ext
import omni.ui as ui
from omni.ui import color as cl
import omni.kit.app
import omni.stats
import carb.settings
import omni.usd
from pxr import Sdf, UsdPhysics, PhysxSchema
import omni.physx.bindings._physx as pb
from omni.physxuicommon import windowmenuitem


class ComboBoxItem(ui.AbstractItem):
    def __init__(self, text):
        super().__init__()
        self.model = ui.SimpleStringModel(text)


class ComboBoxModel(ui.AbstractItemModel):
    def __init__(self, stage, fnc_selected_scene_changed):
        super().__init__()

        self._fnc_selected_scene_changed = fnc_selected_scene_changed
        self._current_index = ui.SimpleIntModel(-1)
        self._current_index.add_value_changed_fn(lambda a: self._item_changed(None))
        self.add_item_changed_fn(self._on_item_changed)

        self._items = []

        if stage is None:
            return

        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                self._items.append(ComboBoxItem(str(prim.GetPath())))

        self._current_index.set_value(0)

    def _on_item_changed(self, model, item):
        if self._items is not None and len(self._items) > 0:
            idx = self._current_index.get_value_as_int()
            self._fnc_selected_scene_changed(self._items[idx].model.get_value_as_string())

    # AbstractItemModel interfaces

    def get_item_value_model(self, item=None, column_id=0):
        if item is None:
            return self._current_index
        return item.model

    def get_item_children(self, item):
        return self._items


class CollisionGroupsModel:
    def __init__(self):
        self.clear()

    def clear(self):
        self._coll_groups = dict()
        self._view_model = None
        self._inverted_filter = False

    def create(self, stage):
        self.clear()

        if stage is None:
            return

        # first pass: create dict. keys
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.CollisionGroup):
                self._coll_groups[prim.GetPath()] = set()

        # second pass: append filtered relationships
        # this will resolve also incomplete one-way relationships
        for path in self._coll_groups.keys():
            prim = stage.GetPrimAtPath(path)
            targets = prim.GetRelationship(UsdPhysics.Tokens.physicsFilteredGroups).GetTargets()
            for target in targets:
                self._coll_groups[path].add(target)
                if target in self._coll_groups.keys():
                    self._coll_groups[target].add(path)

        coll_groups_sorted = sorted(self._coll_groups.keys())
        self._view_model = self._create_view_model(coll_groups_sorted)

    @staticmethod
    def _create_view_model(coll_groups):
        coll_groups_len = len(coll_groups)

        if coll_groups_len == 0:
            return None

        sz = coll_groups_len + 1
        view_model = [[None for _ in range(sz)] for _ in range(sz)]

        i = sz - 1
        for cg in coll_groups:
            view_model[0][i] = cg
            i -= 1

        i = 1
        for cg in coll_groups:
            view_model[i][0] = cg
            i += 1

        i = 1
        for cg in coll_groups:
            diag = False
            j = sz - 1
            for cgh in coll_groups:
                if cgh is cg:
                    diag = True
                if diag:
                    view_model[i][j] = (cgh, cg)
                j -= 1
            i += 1

        return view_model


class CollisionGroupsWindow:
    WINDOW_NAME = "Collision Groups Filtering Matrix"

    SQUARE_STYLE = {
        "background_color": cl("#00000000"),
        # "debug_color": cl("#FF000055"),
        "margin": 5,
    }
    SQUARE_SIZE = 22

    CHECKBOX_STYLE = {
        "CheckBox::red": {"color": cl.white, "background_color": 0xFF5050C7},
        "CheckBox::green": {"color": cl.white, "background_color": 0xFF28A428}
    }

    @staticmethod
    def chkbox_name(value):
        return "green" if value else "red"  # update checkbox style name based on its value

    def __init__(self):
        self._window = None
        self._app = None

    def get_name(self):
        return self.WINDOW_NAME

    def on_startup(self):
        self._app = omni.kit.app.get_app()
        self._stage = omni.usd.get_context().get_stage()

        self._model = CollisionGroupsModel()

        self._grid_frame = None

        self._settings_subs = []

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pb.SETTING_SHOW_COLLISION_GROUPS_WINDOW, self._show_collision_groups_window_visibility_changed
        ))

        usd_context = omni.usd.get_context()
        self._stage_event_sub = [
            get_eventdispatcher().observe_event(
                observer_name="omni.physx.ui:CollisionGroupsWindow",
                event_name=usd_context.stage_event_name(event),
                on_event=func
            )
            for event, func in (
                (omni.usd.StageEventType.OPENED, lambda _: self._on_stage_open_close_event()),
                (omni.usd.StageEventType.CLOSED, lambda _: self._on_stage_open_close_event()),
            )
        ]

        self.create_window()

        def main_menu_click():
            settings = carb.settings.get_settings()
            val = settings.get_as_bool(pb.SETTING_SHOW_COLLISION_GROUPS_WINDOW)
            settings.set_bool(pb.SETTING_SHOW_COLLISION_GROUPS_WINDOW, not val)

        def is_ticked():
            return self._window.visible if self._window else False

        self._menu = windowmenuitem.MenuItem(f"Physics/{self.WINDOW_NAME}", "Window", main_menu_click, is_ticked)

    def on_shutdown(self):
        if self._menu:
            self._menu.remove()
            self._menu = None
        self._window = None
        self._settings_subs = []
        self._stage_event_sub = None

    def destroy_window(self):
        if self._window:
            self._window.destroy()
            self._window = None

    def create_window(self):
        self.destroy_window()

        self._window = ui.Window(
            self.WINDOW_NAME,
            width=750,
            height=400,
            padding=10,
            margin=10,
            visible=False,
            dockPreference=ui.DockPreference.RIGHT_TOP,
        )

        self._window.frame.set_build_fn(self._build_ui)
        self._window.set_visibility_changed_fn(self._visiblity_changed_fn)

    def _selected_ph_scene_changed(self, scene_path):
        if self._stage is None:
            return

        prim = self._stage.GetPrimAtPath(Sdf.Path(scene_path))

        if not prim.IsValid():
            return

        attr = prim.GetAttribute(PhysxSchema.Tokens.physxSceneInvertCollisionGroupFilter)

        new_val = False

        if attr.IsValid():
            new_val = attr.Get()

        if self._model._inverted_filter != new_val:
            self._model._inverted_filter = new_val
            self._create_grid(self._grid_frame, self._model._view_model)

    def _update_coll_group_target(self, path, targets: List[Sdf.Path]):
        if self._stage is None:
            return

        prim = self._stage.GetPrimAtPath(path)

        if not prim.IsValid():
            return

        rel = prim.GetRelationship(UsdPhysics.Tokens.physicsFilteredGroups)
        rel.SetTargets(targets)

    def _build_ui(self):
        if self._window is None or self._window.frame is None:
            return

        self._stage = omni.usd.get_context().get_stage()
        self._model.create(self._stage)

        with self._window.frame:
            with ui.VStack(height=0, spacing=5):
                with ui.HStack(spacing=5):
                    ui.Label("Physics Scene:", width=100)
                    self._scope_combo_model = ComboBoxModel(self._stage, self._selected_ph_scene_changed)
                    self._scope_combo = ui.ComboBox(self._scope_combo_model, width=350)
                ui.Spacer(height=5)
                ui.Line()
                ui.Spacer(height=5)
                self._grid_frame = ui.Frame()
                self._create_grid(self._grid_frame, self._model._view_model)

    def _create_cell(self, cell):
        if cell is None:
            ui.Rectangle(
                width=self.SQUARE_SIZE,
                height=self.SQUARE_SIZE,
                style=self.SQUARE_STYLE
            )
        elif type(cell) is Sdf.Path:
            ui.Label(cell.name).set_tooltip(str(cell))
        else:
            cg, cg_in = cell
            cgm = self._model._coll_groups[cg_in]
            filtered = cg in cgm
            value = filtered if self._model._inverted_filter else not filtered

            with ui.ZStack():
                ui.Rectangle(
                    width=self.SQUARE_SIZE,
                    height=self.SQUARE_SIZE,
                    style=self.SQUARE_STYLE
                )
                model = ui.SimpleBoolModel()
                model.set_value(value)
                chkbox = ui.CheckBox(
                    model,
                    width=0,
                    style=self.CHECKBOX_STYLE,
                    name=self.chkbox_name(value)
                )
                model.add_value_changed_fn(partial(self._coll_group_filter_changed, cg, cg_in, chkbox))

    def _create_grid(self, frame, vm):
        if frame is None or vm is None:
            return

        frame.clear()

        with frame:
            # generate UI per each column one by one
            # view model is always squared
            sz = len(vm[0])
            with ui.HStack(width=0, spacing=5):
                for h in range(sz):
                    with ui.VStack():
                        for v in range(sz):
                            cell = vm[v][h]
                            self._create_cell(cell)

    def _coll_group_filter_changed(self, path1: Sdf.Path, path2: Sdf.Path, chkbox, value):
        chkbox.name = self.chkbox_name(value.as_bool)

        cgm = self._model._coll_groups
        v = value.as_bool
        v = v if not self._model._inverted_filter else not v

        if v:
            try:
                cgm[path1].remove(path2)
                cgm[path2].remove(path1)
            except KeyError:
                # we don't need to throw an error if an item for removal does not exist - that's allowed
                pass
        else:
            cgm[path1].add(path2)
            cgm[path2].add(path1)

        # update USD
        self._update_coll_group_target(path1, [*cgm[path1]])
        self._update_coll_group_target(path2, [*cgm[path2]])

    def _release_model(self):
        self._model.clear()
        self._stage = None

    async def _refresh_window(self):
        if self._window is None or not self._window.visible:
            return
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()
        self._window.frame.rebuild() if self._window.frame is not None else self._build_ui()

    def _on_stage_open_close_event(self):
        asyncio.ensure_future(self._refresh_window())

    def _show_collision_groups_window_visibility_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            settings = carb.settings.get_settings()
            window_enabled = settings.get_as_bool(pb.SETTING_SHOW_COLLISION_GROUPS_WINDOW)
            if self._window:
                if window_enabled:
                    self._window.frame.rebuild() if self._window.frame is not None else self._build_ui()
                else:
                    self._release_model()
                self._window.visible = window_enabled

    def _visiblity_changed_fn(self, visible):
        # handle the case when user closes the window by the top right cross
        if not visible:
            settings = carb.settings.get_settings()
            window_enabled = settings.get_as_bool(pb.SETTING_SHOW_COLLISION_GROUPS_WINDOW)
            if window_enabled:
                settings.set_bool(pb.SETTING_SHOW_COLLISION_GROUPS_WINDOW, False)
        if self._menu:
            self._menu.refresh()
