# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physxui import get_physxui_private_interface
from omni.timeline import get_timeline_interface, TimelineEventType

hotkey_handling_enabled = True

try:
    from omni.kit.hotkeys.core import get_hotkey_registry, KeyCombination
except ImportError:
    hotkey_handling_enabled = False


class InputManager:
    def __init__(self):
        timeline_events = get_timeline_interface().get_timeline_event_stream()
        self._timeline_sub = timeline_events.create_subscription_to_pop(self.on_timeline_event)
        self._actions = dict()
        self._hotkeys = []
        self._dirty = False

    def shutdown(self):
        self.use_actions(False)
        get_physxui_private_interface().clear_input_manager()

        self._actions = None
        self._hotkeys = None
        self._timeline_sub = None

    def register_keyboard_action(self, name, kb_input, modifier):
        get_physxui_private_interface().register_keyboard_action(name, kb_input, modifier)
        self._dirty = True

        if hotkey_handling_enabled:
            self._actions[name] = KeyCombination(kb_input, modifier)

    def register_gamepad_action(self, name, pad_input, pad_index):
        get_physxui_private_interface().register_gamepad_action(name, pad_input, pad_index)

    def unregister_action(self, name):
        get_physxui_private_interface().unregister_action(name)
        self._actions.pop(name, None)
        self._dirty = True

    def clear(self):
        get_physxui_private_interface().clear_input_manager()

    def use_actions(self, use):
        if not hotkey_handling_enabled:
            return

        hr = get_hotkey_registry()

        if use:
            for keycombination in self._actions.values():
                hotkeys = hr.get_all_hotkeys_for_key(keycombination)
                for hotkey in hotkeys:
                    if hr.deregister_hotkey(hotkey):
                        self._hotkeys.append(hotkey)
        else:
            for hotkey in self._hotkeys:
                hr.register_hotkey(hotkey)
            self._hotkeys.clear()

    def on_timeline_event(self, e):
        if e.type == int(TimelineEventType.CURRENT_TIME_TICKED):
            self.on_update(e)
        if e.type == int(TimelineEventType.PLAY):
            self.use_actions(True)
        if e.type == int(TimelineEventType.STOP):
            self.use_actions(False)
        if e.type == int(TimelineEventType.PAUSE):
            self.use_actions(False)

    def on_update(self, e):
        if self._dirty:
            self.use_actions(False)
            self.use_actions(True)
            self._dirty = False
