# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""The normalized dump of an OVStage population description, for comparing its C and Python forms.

Walks an ``ovstage_population_desc_t`` (the C accessor's struct, or the ctypes struct
``ovstage.population.build_descs`` makes from a Python ``Desc``) into the form
``gen_contract.py`` writes to ``tests/data/physics_population_desc.json``: each node is
``{"kind": <enum name>, <the union member its kind names>: ...}``, shared nodes written out
in full, keys in struct order.
"""

# @implements REQ-CAPI-POPULATION-DESC-001
# @covers AC-2 AC-3

from __future__ import annotations

import ctypes
import json

from ovstage import AttributeSemantic
from ovstage._src import bindings as _b
from ovstage.population import DeriveWhen, ExpressionKind, PrimPredicateKind, PropertyPredicateKind

_DLPACK_CODES = {0: "int", 1: "uint", 2: "float", 6: "bool"}
_PRIM_SUBPREDICATES = {"AND", "OR", "NOT", "HAS_PARENT", "HAS_ANCESTOR", "HAS_DESCENDANT"}
_PRIM_NO_VALUES = {"NONE", "ALL", "IS_INSTANCE"}
_PROPERTY_NO_VALUES = {"NONE", "ALL", "IS_ATTRIBUTE", "IS_RELATIONSHIP", "IS_CUSTOM", "IS_AUTHORED"}
_ARITHMETIC = {"ADD", "SUBTRACT", "MULTIPLY", "DIVIDE"}


def _string(s: _b.ovx_string_t) -> str:
    address = ctypes.c_void_p.from_buffer(s, _b.ovx_string_t.ptr.offset).value
    return ctypes.string_at(address, s.length).decode("utf-8") if address else ""


def _strings(array, count: int) -> list:
    return [_string(array[i]) for i in range(count)]


def _prim(p) -> dict:
    kind = PrimPredicateKind(p.kind).name
    node = {"kind": kind}
    if kind in _PRIM_SUBPREDICATES:
        node["subpredicates"] = [_prim(p.values.subpredicates[i]) for i in range(p.value_count)]
    elif kind == "HAS_PROPERTY":
        node["property_predicate"] = _property(p.values.property_predicate.contents)
    elif kind == "IS_TARGETED_BY":
        node["prim_predicate"] = _prim(p.values.targeted_by.prim_predicate.contents)
        node["property_predicate"] = _property(p.values.targeted_by.property_predicate.contents)
    elif kind not in _PRIM_NO_VALUES:
        node["strings"] = _strings(p.values.strings, p.value_count)
    return node


def _property(p) -> dict:
    kind = PropertyPredicateKind(p.kind).name
    node = {"kind": kind}
    if kind in ("AND", "OR", "NOT"):
        node["subpredicates"] = [_property(p.values.subpredicates[i]) for i in range(p.value_count)]
    elif kind == "TARGETS":
        node["prim_predicate"] = _prim(p.values.prim_predicate.contents)
    elif kind not in _PROPERTY_NO_VALUES:
        node["strings"] = _strings(p.values.strings, p.value_count)
    return node


def _optional(pointer, walk):
    return walk(pointer.contents) if pointer else None


def _expression(e) -> dict:
    kind = ExpressionKind(e.kind).name
    node = {"kind": kind}
    if kind == "NUMBER":
        node["number"] = float(e.values.number)
    elif kind == "BOOL":
        node["boolean"] = bool(e.values.boolean)
    elif kind == "STAGE_METADATA":
        node["name"] = _string(e.values.name)
    elif kind in _ARITHMETIC:
        node["operands"] = [_expression(e.values.operands[i].contents) for i in range(2)]
    elif kind != "UP_AXIS":
        node["name"] = _string(e.values.resolved.name)
        node["prim_predicate"] = _optional(e.values.resolved.prim_predicate, _prim)
        if kind == "RESOLVED_COLLECTION_MEMBERS":
            node["property_predicate"] = _optional(e.values.resolved.property_predicate, _property)
    return node


def _derived(d) -> dict:
    dtype = None
    if d.dtype.bits:
        dtype = {"code": _DLPACK_CODES[d.dtype.code], "bits": int(d.dtype.bits), "lanes": int(d.dtype.lanes)}
    return {
        "name": _string(d.name),
        "multi_apply_schema_family": _string(d.multi_apply_schema_family),
        "prim_predicate": _optional(d.prim_predicate, _prim),
        "when": DeriveWhen(d.when).name,
        "dtype": dtype,
        "semantic": AttributeSemantic(d.semantic).name,
        "is_array": bool(d.is_array),
        "value": _expression(d.value.contents),
    }


def dump_desc(desc) -> dict:
    """The normalized form of an ``ovstage_population_desc_t`` (a ctypes struct)."""
    selectors = []
    for i in range(desc.selector_count):
        s = desc.selectors[i]
        selectors.append({
            "prim_predicate": _prim(s.prim_predicate),
            "property_predicate": _property(s.property_predicate),
            "prim_metadata_paths": _strings(s.prim_metadata_paths, s.prim_metadata_path_count),
            "property_metadata_paths": _strings(s.property_metadata_paths, s.property_metadata_path_count),
            "derived_properties": [_derived(s.derived_properties[j]) for j in range(s.derived_property_count)],
        })
    return {
        "domains": "NONE" if desc.domains == 0 else int(desc.domains),
        "selectors": selectors,
        "stage_metadata_paths": _strings(desc.stage_metadata_paths, desc.stage_metadata_path_count),
    }


def dump_c_desc(pointer: int) -> str:
    """The normalized JSON of the description at a C ``const ovstage_population_desc_t*``."""
    desc = ctypes.cast(pointer, ctypes.POINTER(_b.ovstage_population_desc_t)).contents
    return json.dumps(dump_desc(desc), indent=2) + "\n"


def dump_python_desc(desc) -> str:
    """The normalized JSON of an ``ovstage.population.Desc``, through the ctypes structs OVStage builds from it."""
    from ovstage._src.selectors import build_descs

    array, count, keepalive = build_descs([desc])
    assert count == 1
    text = json.dumps(dump_desc(array[0]), indent=2) + "\n"
    del keepalive
    return text
