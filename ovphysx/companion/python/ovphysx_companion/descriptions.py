# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DESCRIPTION-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Immutable source-neutral stopped descriptions; no USD or simulation imports.

Parameter names retain their public schema spelling. Values use scalars or nested
tuples, quaternions are xyzw, and matrices are row-major USD row-vector matrices.
Geometry is prim-local in declared stage length units; local_to_world maps it to
the stage world frame. No runtime descriptor addresses or allocator are exposed.
"""
from dataclasses import dataclass

FAMILIES = frozenset(('body', 'shape', 'joint', 'articulation', 'tendon', 'vehicle', 'particle'))


@dataclass(frozen=True)
class Attribute:
    name: str
    type_name: str
    value: object
    authored: bool


@dataclass(frozen=True)
class Description:
    source: object
    path: str
    family: str
    type_name: str
    schemas: tuple
    attributes: tuple
    relationships: tuple  # (relationship name, tuple of composed absolute targets)
    local_to_world: tuple
    meters_per_unit: float
    kilograms_per_unit: float
    instance_path: str = ''
    frame: str = 'prim-local'
    units: str = 'stage'

    def attribute(self, name, default=None):
        return next((item.value for item in self.attributes if item.name == name), default)

    def targets(self, name):
        return next((targets for key, targets in self.relationships if key == name), ())


@dataclass(frozen=True)
class Diagnostic:
    source: object
    path: str
    code: str
    message: str
    severity: str = 'warning'


@dataclass(frozen=True)
class DescriptionResult:
    source: object
    descriptions: tuple
    diagnostics: tuple = ()
    capabilities: frozenset = FAMILIES
    time_code: object = None

    def query(self, *families):
        """Filter a retained immutable snapshot; absent families are explicit."""
        families = tuple(dict.fromkeys(families or sorted(self.capabilities)))
        diagnostics = list(self.diagnostics)
        for family in families:
            if family not in self.capabilities:
                diagnostics.append(Diagnostic(self.source, '', 'DESCRIPTION_UNSUPPORTED_FAMILY', str(family)))
        return DescriptionResult(self.source, tuple(d for d in self.descriptions if d.family in families),
                                  tuple(diagnostics), self.capabilities, self.time_code)

    def validate(self, session):
        """Reject application as current after the producing input revision changes."""
        session.validate(self.source)
        return self
