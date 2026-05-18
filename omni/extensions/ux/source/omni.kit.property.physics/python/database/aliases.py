# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = [
    "schema_aliases",
    "add_schema_aliases",
    "remove_schema_aliases",
]

from collections import defaultdict

schema_aliases_dict = defaultdict(dict)
schema_aliases = {}

def add_schema_aliases(parent_schema, aliases):
    schema_aliases_dict[parent_schema].update(aliases)
    for k, v in aliases.items():
        schema_aliases[k] = v

def remove_schema_aliases(parent_schema):
    schema_aliases_dict.pop(parent_schema, None)
    reconstruct_schema_aliases()

def reconstruct_schema_aliases():
    schema_aliases.clear()
    for d in schema_aliases_dict.values():
        for k, v in d.items():
            schema_aliases[k] = v
