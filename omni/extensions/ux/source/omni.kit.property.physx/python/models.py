# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.kit.property.usd.usd_attribute_model import TfTokenAttributeModel
from omni.physx.scripts import utils
import omni.physxcommands as commands


class CustomMetadataObjectModel:
    def __init__(self, stage, prop, prim_paths, metadata_name):
        self._stage = stage
        self._object_paths = prim_paths
        self._metadata_name = metadata_name

        # The following are not constants, they can get overridden in derived classes
        self._info_str_locally_overriden_value = "Forced (locally overridden) value"
        self._info_str_default_value = "Default value"
        self._info_str_mixed_values = "Mixed values"
        self._info_str_same_value_mixed_sources = "This value comes partially from locally overridden value(s), partially from default"

        self._ambiguous = False
        self._value = None
        self._info_str = ''

    def clean(self):
        self._stage = None

    def _get_objects(self):
        objects = []
        if not self._stage:
            return objects

        for path in self._object_paths:
            obj = self._stage.GetObjectAtPath(path)
            if obj and not obj.IsHidden():
                objects.append(obj)

        return objects

    def is_ambiguous(self):
        return self._ambiguous

    def get_value(self):
        return self._value

    def get_info_str(self):
        return self._info_str

    def set_value(self, value, clear_metadata_when_possible=False):
        do_dict = {}
        undo_dict = {}

        for o in self._get_objects():
            path = o.GetPath()
            do_dict[path] = None if value == self._get_default_value() and clear_metadata_when_possible else value
            undo_dict[path] = utils.get_custom_metadata(o, self._metadata_name)

        self._ambiguous = False
        self._value = value
        self._info_str = ''

        commands.SetCustomMetadataCommand.execute(self._stage, self._metadata_name, do_dict, undo_dict)

    def _determine_value(self):
        self._ambiguous = False
        self._value = None

        objs = self._get_objects()
        if not objs or len(objs) == 0:
            return

        self._value, self._info_str = self._get_metadata_or_default(objs[0])

        for o in objs[1:]:
            v, i = self._get_metadata_or_default(o)

            if i != self._info_str:
                self._info_str = self._info_str_same_value_mixed_sources

            if v != self._value:
                self._ambiguous = True
                self._value = False
                self._info_str = self._info_str_mixed_values
                break

    # methods to be overridden in derived classes
    def _get_default_value(self):
        return None

    def _get_metadata_or_default(self, object):
        info = self._info_str_locally_overriden_value
        value = utils.get_custom_metadata(object, self._metadata_name)

        if value is None:
            value = self._get_default_value()
            info = self._info_str_default_value

        return value, info

    # metadata utility methods
    def _clear_metadata(self, object):
        utils.clear_custom_metadata(object, self._metadata_name)

    def _set_metadata(self, object, value):
        utils.set_custom_metadata(object, self._metadata_name, value)
