# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

_builders_dict = {}
_builders = {}

def add_property_builders(parent_schema, builders):
    _builders_dict[parent_schema] = builders
    for k, v in builders.items():
        _builders[k] = v

def remove_property_builders(parent_schema):
    _builders_dict.pop(parent_schema, None)
    reconstruct_property_builders()

def reconstruct_property_builders():
    _builders.clear()
    for d in _builders_dict.values():
        for k, v in d.items():
            _builders[k] = v

def get_available_builders(prop_name):
    return [val for key, val in _builders.items() if prop_name.endswith(key)]
