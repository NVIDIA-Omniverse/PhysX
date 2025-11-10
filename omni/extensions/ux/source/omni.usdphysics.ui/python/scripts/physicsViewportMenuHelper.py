# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import functools
from typing import Callable, List
import carb.settings
import omni.ui as ui
from omni.kit.viewport.menubar.core.model.category_model import CategoryStateItem
from omni.kit.viewport.menubar.core import SelectableMenuItem

class PhysicsViewportMenuHelper:
       # some helper functions

    def __init__(self):
        self.settings = carb.settings.acquire_settings_interface()

    def create_menu_simple_bool_item(self, text, setting) -> CategoryStateItem:
        # also set default value with whatever we have in the settings
        newItem = SelectableMenuItem(text, ui.SimpleBoolModel(self.settings.get_as_bool(setting)))
        newItem.model.add_value_changed_fn(
            lambda model, item=newItem: self.settings.set(setting, model.get_value_as_bool())
        )
        return newItem

    """
        Receives an itemsList list of two or three SelectableMenuItem("text", ui.SimpleBoolModel(True)) items
        and an itemSelectionChangeCallback which is a function of the form
            def itemSelectionChangeCallback(item : SelectableMenuItem)
                ... called when a selection changes ...
                ... inspect item.text and item.value_model.get_value_as_bool() here ...
        and make those checkbox items in the list behave like three radio boxes (one deselect the others).
        Grep for (#radio) for more context into why this function is necessary.
    """
    def create_menu_radioboxes(self, items: List[SelectableMenuItem], itemChangedCallback: Callable[[SelectableMenuItem], None]):

        # Helper to add like-radio behavior (exclusive checkmark) to a set of items
        def radio_value_changed(allItems, callbackFn, modelChanged):
            # only call the callback if we didn't click on the only-selected radio box again (that has no effect)
            sendCallback = True

            if modelChanged.get_value_as_bool():
                # if an item is now checked, uncheck the others
                modelChanged.set_value(True)
                for other in allItems:
                    if other.model == modelChanged:
                        continue
                    other.model.set_value(False)
            else:
                allOthersAreFalse = True
                for other in allItems:
                    if other.model == modelChanged:
                        continue
                    if other.model.get_value_as_bool() is True:
                        allOthersAreFalse = False
                        break
                if allOthersAreFalse:
                    sendCallback = False
                    modelChanged.set_value(True)

            if sendCallback:
                for item in items:
                    if item.model == modelChanged:
                        callbackFn(item)
                        break

        # binds two additional parameters to the like_radio function
        valueChangedCallback = functools.partial(radio_value_changed, items, itemChangedCallback)

        # Connect the radio boxes one to another in order to simulate the radio behavior
        for i in items:
            i.model.add_value_changed_fn(valueChangedCallback)

    def simulation_collision_store_setting_deprecated(self, item, setting):
        if item.text == "Simulation":
            self.settings.set_int(setting, 0)
        elif item.text == "Collision":
            self.settings.set_int(setting, 1)
        else:
            carb.log_error(f"Unrecognized text for radio item '{item.text}'")

    def deformable_mesh_type_store_setting(self, item, setting):
        if item.text == "Simulation: Default Pose":
            self.settings.set_int(setting, 0)
        elif item.text == "Simulation: Bind Pose":
            self.settings.set_int(setting, 1)
        elif item.text == "Simulation: Rest Shape":
            self.settings.set_int(setting, 2)
        elif item.text == "Collision: Default Pose":
            self.settings.set_int(setting, 3)
        elif item.text == "Collision: Bind Pose":
            self.settings.set_int(setting, 4)
        else:
            carb.log_error(f"Unrecognized text for radio item '{item.text}'")

    def none_selected_all_store_setting(self, item, setting):
        if item.text == "None":
            self.settings.set_int(setting, 0)
        elif item.text == "Selected":
            self.settings.set_int(setting, 1)
        elif item.text == "All":
            self.settings.set_int(setting, 2)
        else:
            carb.log_error(f"Unrecognized text for radio item '{item.text}'")

    """
        This is rather involved and needed to avoid a recursion loop: when a radio menu item is clicked, it also mutates
        the associated setting (through the callback as 2nd argument to create_menu_radioboxes). We also need to monitor
        scene changes outside of the user input, i.e. that setting could change by its own and we need to update the
        menu item checkboxes accordingly. Unfortunately if we just monitor for setting changes with subscribe_to_node_change_events,
        we will end up in a loop because:
            1. setting changes from the outside and we detect it via the subscribe_to_node_change_events callback
            2. we update the menu items and set some menu item to True
            3. this triggers the callback as 2nd argument to create_menu_radioboxes
            4. this tries to change that same setting again
            5. go to 1.. we have a loop.
        This function breaks the loop by unsubscribing from setting changes notifications as soon as a change notification
        occurs, set the right value, execute whatever the create_menu_radioboxes wants, and only after all this happened
        we re-enable the setting changes notifications.
    """
    def safe_subscribe_to_setting_change(self, sub, items, setting):
        if sub is not None:
            self.settings.unsubscribe_to_change_events(sub)
            if len(items) == 1:
                items[0].model.set_value(self.settings.get_as_bool(setting))
            else:
                items[self.settings.get_as_int(setting)].model.set_value(True)
        sub = self.settings.subscribe_to_node_change_events(
            setting, lambda item, event_type: self.safe_subscribe_to_setting_change(sub, items, setting)
        )

    def create_radioboxes_for_setting(self, setting, radioboxes, map_text_to_setting_value_fn):
        defaultIntValue = self.settings.get_as_int(setting)
        radioItems = []
        for i, text in enumerate(radioboxes):
            radioItems.append(SelectableMenuItem(text, ui.SimpleBoolModel(defaultIntValue == i)))
        self.create_menu_radioboxes(radioItems, lambda item, setting=setting: map_text_to_setting_value_fn(item, setting))
        self.safe_subscribe_to_setting_change(None, radioItems, setting)

    def create_checkbox_for_setting(self, text, setting):
        item = self.create_menu_simple_bool_item(text, setting)
        self.safe_subscribe_to_setting_change(None, [item], setting)
