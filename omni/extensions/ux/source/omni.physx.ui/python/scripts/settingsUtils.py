# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from collections import OrderedDict
import carb
import carb.settings
import omni.kit.app
from omni import ui
from omni.kit.settings import SettingType
from omni.kit.commands import execute


class Model(ui.AbstractValueModel):
    def __init__(self, path):
        super().__init__()
        self._path = path
        self._value = carb.settings.get_settings().get(path)

        def on_change(item, event_type, path=path):
            self._value = carb.settings.get_settings().get(path)
            self._value_changed()

        self._sub = omni.kit.app.SettingChangeSubscription(path, on_change)

    def get_value_as_bool(self):
        return self._value if self._value is not None else True

    def get_value_as_int(self):
        return self._value if self._value is not None else 0

    def get_value_as_float(self):
        return self._value if self._value is not None else 0

    def get_value_as_string(self):
        return self._value if self._value is not None else ""

    def set_value(self, value):
        execute("ChangeSetting", path=self._path, value=value)
        self._value = value
        self._value_changed()


class ComboItem(ui.AbstractItem):
    def __init__(self, model, value):
        super().__init__()
        self.model = model
        self.value = value


class ComboModel(ui.AbstractItemModel):
    def __init__(self, path, items):
        super().__init__()

        self._path = path
        self._children = [ComboItem(ui.SimpleStringModel(str(k)), v) for k, v in items.items()]
        self._value_to_index = {k: v for (k, v) in zip(items.values(), range(0, len(items)))}

        self._index = ui.SimpleIntModel(self._get_index_from_settings_value())
        self._index.add_value_changed_fn(self._index_changed)

        def on_change(item, event_type, path=path):
            self._index.set_value(self._get_index_from_settings_value())

        self._sub = omni.kit.app.SettingChangeSubscription(path, on_change)

    def _get_index_from_settings_value(self):
        return self._value_to_index[carb.settings.get_settings().get(self._path)]

    def _index_changed(self, ind):
        # if the values are different it's a change invoked by the user, otherwise it's a
        # callback from index changed from on_change -> SettingChangeSubscription
        value = self._children[ind.as_int].value
        if carb.settings.get_settings().get(self._path) is not value:
            execute("ChangeSetting", path=self._path, value=value)

        self._item_changed(None)

    def get_item_children(self, item):
        return self._children

    def get_item_value_model(self, item, column_id):
        if item is None:
            return self._index
        return item.model


def create_setting_widget(setting_path, setting_type, range_from=0, range_to=0, speed=1, **kwargs):
    model = Model(setting_path)

    if setting_type == SettingType.INT:
        widget = ui.IntDrag(model, min=range_from, max=range_to, drag_speed=speed, **kwargs)
    elif setting_type == SettingType.FLOAT:
        widget = ui.FloatDrag(model, min=range_from, max=range_to, drag_speed=speed, **kwargs)
    elif setting_type == SettingType.BOOL:
        widget = ui.CheckBox(model, **kwargs)
    elif setting_type == SettingType.STRING:
        widget = ui.StringField(model, **kwargs)
    elif setting_type == SettingType.COLOR3:
        widget = ui.ColorWidget(model, **kwargs)
    elif setting_type == SettingType.DOUBLE3:
        widget = ui.MultiFloatDragField(model, min=range_from, max=range_to, drag_speed=speed, **kwargs)
    elif setting_type == SettingType.INT2:
        widget = ui.IntDrag(model, min=range_from, max=range_to, drag_speed=speed, **kwargs)
    else:
        return None

    return (model, widget)


# items can be defined either as [1,20,99] or {"one": 1, "twenty": 20, "a lot": 99}
def create_setting_widget_combo(setting_path, items, **kwargs):
    if isinstance(items, list):
        name_to_value = OrderedDict(zip(items, items))
    elif isinstance(items, dict):
        name_to_value = items
    else:
        carb.log_error("items can be defined either as a list or a dict")
        return None

    model = ComboModel(setting_path, name_to_value)
    widget = ui.ComboBox(model, **kwargs)

    return (model, widget)
