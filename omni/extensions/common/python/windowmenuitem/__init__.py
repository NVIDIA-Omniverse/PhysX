# SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.kit.menu.utils
from omni.kit.menu.utils import MenuItemDescription
import omni.kit.actions.core

"""
Helper to handle adding and removing a menu item.
"""

class MenuItem():
    def __init__(self, menu_path, menu_group, toggle_fn, is_ticked_fn, ticked_value=False):
        self._menu_group = menu_group
        ext_id = self.__class__.__module__
        action_name = f'WindowMenuItemAction_{menu_path.replace(" ", "").replace("/", "")}'

        omni.kit.actions.core.get_action_registry().register_action(
            ext_id,
            action_name,
            toggle_fn,
            display_name=action_name,
            tag="MenuItem"
        )

        item_names = reversed(menu_path.split("/"))

        menu_item = MenuItemDescription(
            name=next(item_names),
            ticked=True,
            ticked_fn=is_ticked_fn,
            ticked_value=ticked_value,
            onclick_action=(ext_id, action_name)
        )

        curr_item = menu_item

        for item_name in item_names:
            curr_item = MenuItemDescription(
                name=item_name,
                sub_menu=[curr_item]
            )

        self._items = [curr_item]
        omni.kit.menu.utils.add_menu_items(self._items, self._menu_group)

    def on_shutdown(self):
        self.remove()

    def remove(self):
        if self._items:
            omni.kit.menu.utils.remove_menu_items(self._items, self._menu_group)
            self._items = None

    def refresh(self):
        omni.kit.menu.utils.refresh_menu_items(self._menu_group)


"""
Helper to handle a dynamically spawned ui.Window and its menu item. Will call on_shutdown on the child window if such method
is present. Use window_visibility_changed_fn to add a visibility callback, since there can be only one and this helper
already uses it.

Args:
    menu_item_name: Menu item name.
    spawn_window_fn: Function that returns created ui.Window instance
    spawn_immediately: Window will spawn either immediately with the helper's creation or only after the user selects
                       one of the menu items for the first time
    window_visibility_changed_fn: Custom visibility callback.
    spawn_visible: Make visible when spawning.


Example:

class PhysXDebugView():
    def __init__(self):
        self._menu = WindowMenuItem("PhysX Debug", lambda: PhysxDebugWindow(), spawn_immediately)

    def on_shutdown(self):
        self._menu.on_shutdown()
        self._menu = None

class PhysxDebugWindow(ui.Window):
    def __init__(self):
        ...

    def on_shutdown(self):
        ...
"""


class WindowMenuItem(MenuItem):
    import omni.ui

    def __init__(self, menu_path, spawn_window_fn, spawn_immediately=False, window_visibility_changed_fn=None, spawn_visible=True):
        menu_path = f"Physics/{menu_path}"
        self._window = None
        
        super().__init__(menu_path, "Window", self.toggle_window, self.is_visible)

        self._spawn_window_fn = spawn_window_fn
        self._window_visibility_changed_fn = window_visibility_changed_fn

        if spawn_immediately:
            self._spawn_window(spawn_visible)

    def set_window_visibility(self, visible: bool):
        self.show_window() if visible else self.hide_window()

    def hide_window(self):
        if self._window is None:
            return

        if isinstance(self._window, omni.ui.Window):
            self._window.visible = False
        else:
            self._window.hide()
        self.refresh()

    def show_window(self):
        if self._window is None:
            self._spawn_window()
            return

        if isinstance(self._window, omni.ui.Window):
            self._window.visible = True
            self._window.focus()
        else:
            self._window.show()
        self.refresh()

    def on_shutdown(self):
        self._window_visibility_changed_fn = None
        if self._window is not None:
            if hasattr(self._window, "on_shutdown"):
                self._window.on_shutdown()
            self.hide_window()
            self._window = None

        super().on_shutdown()

    def _spawn_window(self, visible=True):
        self._window = self._spawn_window_fn()
        self._window.visible = visible
        self._window.set_visibility_changed_fn(self._on_visibility_changed)
        self.refresh()

    def is_visible(self):
        return self._window.visible if self._window else False

    def toggle_window(self):
        if self._window is None:
            self._spawn_window()
        else:
            if self._window.visible:
                self.hide_window()
            else:
                self.show_window()

    def _on_visibility_changed(self, visible):
        if self._window_visibility_changed_fn is not None:
            self._window_visibility_changed_fn(visible)
        self.refresh()
