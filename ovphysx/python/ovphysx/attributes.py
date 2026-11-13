# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-ATTRIBUTES-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Discover runtime I/O types, units and capabilities without loading native code.

``get_attribute("RIGID_BODY", "angularVelocity")`` returns an immutable descriptor.
``list_attributes("ARTICULATION_JOINT", writable=True)`` filters the catalog.
Capabilities are static; live session admission remains authoritative. Layouts
with symbolic widths must be resolved from actual session groups, not evaluated
as Python expressions. Importing this module does not load PhysX, Warp or USD.
"""

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping

from ._attribute_data import DATA
from .types import SimObjectType


def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({k: _freeze(v) for k, v in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(v) for v in value)
    return value


@dataclass(frozen=True)
class Attribute:
    """One canonical name on a query target and representation.

    ``read`` and ``write`` contain independent layout, residency and condition
    metadata. Use their ``request_name`` for API calls; ``name`` can instead be
    the emitted read-group spelling (for example instancer ``positions``).
    Directional ``aliases`` list additional accepted requests. ``writability`` preserves the existing C classification; use the
    direction's ``supported`` flag for runtime-only spellings. ``units`` is the
    machine-readable unit ID; ``unit["display"]`` describes runtime values using
    distance, mass, sec, deg and rad. Conditional units are in ``unit["variants"]``;
    packed tuple units are in ``unit["components"]`` as registry IDs. No population
    scale-exponent metadata is exposed. A direction can override the default unit
    ID with ``units``; resolve it through ``UNITS``. Unsupported directions have
    no request name, layout or residency.
    """

    id: str
    name: str
    target: str
    representation: str
    description: str
    dtype: str
    semantic: str
    frame: str
    units: str
    unit: Mapping[str, Any] = field(hash=False)
    read: Mapping[str, Any] = field(hash=False)
    write: Mapping[str, Any] = field(hash=False)
    writability: str
    aliases: tuple[str, ...]
    population: Mapping[str, Any] | None = field(hash=False)
    notes: str


SCHEMA_VERSION = DATA["schema_version"]
TARGETS = _freeze(DATA["targets"])
UNITS = _freeze(DATA["units"])
POLICIES = _freeze(DATA["policies"])
NAMES = MappingProxyType({a["id"]: a["name"] for a in DATA["attributes"]})
_ATTRIBUTES = tuple(
    Attribute(
        a["id"],
        a["name"],
        u["target"],
        u["representation"],
        u["description"],
        u["dtype"],
        a["semantic"],
        u["frame"],
        u["units"],
        UNITS[u["units"]],
        _freeze(u["read"]),
        _freeze(u["write"]),
        u["writability"],
        tuple(u["aliases"]),
        _freeze(u.get("population")),
        u.get("notes", ""),
    )
    for a in DATA["attributes"]
    for u in a["uses"]
)
_INDEX = {(a.target, a.name, a.representation): a for a in _ATTRIBUTES}
for _a in _ATTRIBUTES:
    for _direction in (_a.read, _a.write):
        if _direction["supported"]:
            _INDEX[_a.target, _direction["request_name"], _a.representation] = _a
    for _alias in _a.aliases:
        _INDEX[_a.target, _alias, _a.representation] = _a
# The mutable generated data is not the query surface.
del DATA


def _target(value: str | int) -> str:
    if type(value) is str:
        name = value.upper().removeprefix("OVPHYSX_QUERY_TARGET_")
        if name in TARGETS:
            return name
    elif type(value) is int or isinstance(value, SimObjectType):
        for name, number in TARGETS.items():
            if number == value:
                return name
    raise ValueError(f"Unknown physics query target: {value!r}")


def list_attributes(
    target: str | int | None = None,
    *,
    readable: bool | None = None,
    writable: bool | None = None,
    representation: str | None = None,
) -> tuple[Attribute, ...]:
    """Return canonical descriptors in stable ID order, optionally filtered.

    ``target`` accepts a target name, a plain int, or SimObjectType; other enums are rejected. A valid target
    with no attributes returns an empty tuple; an unknown target raises ValueError.
    Both default and instancer representations are included unless filtered.
    """
    name = _target(target) if target is not None else None
    if representation not in (None, "default", "instancer"):
        raise ValueError(f"Unknown representation: {representation!r}")
    for flag in (readable, writable):
        if flag is not None and not isinstance(flag, bool):
            raise TypeError("readable and writable must be bool or None")
    return tuple(
        a
        for a in _ATTRIBUTES
        if (name is None or a.target == name)
        and (representation is None or a.representation == representation)
        and (readable is None or a.read["supported"] == readable)
        and (writable is None or a.write["supported"] == writable)
    )


def get_attribute(target: str | int, name: str, *, representation: str = "default") -> Attribute:
    """Look up a catalog name, request name or alias.

    Invalid targets/representations raise ValueError; unsupported names raise KeyError.
    """
    if representation not in ("default", "instancer"):
        raise ValueError(f"Unknown representation: {representation!r}")
    key = (_target(target), name, representation)
    try:
        return _INDEX[key]
    except KeyError:
        raise KeyError(f"No physics attribute {key!r}") from None


__all__ = ["Attribute", "SCHEMA_VERSION", "TARGETS", "UNITS", "POLICIES", "NAMES", "get_attribute", "list_attributes"]
