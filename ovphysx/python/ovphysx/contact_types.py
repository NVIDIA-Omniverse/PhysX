# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Contact report ctypes structures.

These ``ctypes.Structure`` mirrors of the C ABI structs returned by
:c:func:`ovphysx_get_contact_report` are used to access per-step contact data
in Python without copying. Field layouts must stay in sync with the C definitions
in ``ovphysx/include/ovphysx/ovphysx_types.h``.

The module has no native-library dependencies and is safe to import in any
Python process; the structures are populated by ctypes against pointers returned
from the C API.
"""

import ctypes
from ctypes import c_float, c_int32, c_int64, c_uint32, c_uint64

__all__ = [
    "ContactEventHeader",
    "ContactPoint",
    "FrictionAnchor",
]


class ContactEventHeader(ctypes.Structure):
    """Contact event header. Mirrors ``ovphysx_contact_event_header_t``."""

    _fields_ = [
        ("type", c_int32),
        ("stageId", c_int64),
        ("actor0", c_uint64),
        ("actor1", c_uint64),
        ("collider0", c_uint64),
        ("collider1", c_uint64),
        ("contactDataOffset", c_uint32),
        ("numContactData", c_uint32),
        ("frictionAnchorsDataOffset", c_uint32),
        ("numfrictionAnchorsData", c_uint32),
        ("protoIndex0", c_uint32),
        ("protoIndex1", c_uint32),
    ]


class ContactPoint(ctypes.Structure):
    """Per-contact-point data. Mirrors ``ovphysx_contact_point_t``."""

    _fields_ = [
        ("position", c_float * 3),
        ("normal", c_float * 3),
        ("impulse", c_float * 3),
        ("separation", c_float),
        ("faceIndex0", c_uint32),
        ("faceIndex1", c_uint32),
        ("material0", c_uint64),
        ("material1", c_uint64),
    ]


class FrictionAnchor(ctypes.Structure):
    """Friction anchor data. Mirrors ``ovphysx_friction_anchor_t``."""

    _fields_ = [
        ("position", c_float * 3),
        ("impulse", c_float * 3),
    ]
