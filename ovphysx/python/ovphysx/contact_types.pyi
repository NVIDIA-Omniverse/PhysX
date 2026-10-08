# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import ctypes

__all__: list[str]

class ContactEventHeader(ctypes.Structure):
    type: int
    attachHandle: int
    actor0: int
    actor1: int
    collider0: int
    collider1: int
    contactDataOffset: int
    numContactData: int
    frictionAnchorsDataOffset: int
    numfrictionAnchorsData: int
    protoIndex0: int
    protoIndex1: int

class ContactPoint(ctypes.Structure):
    position: ctypes.Array[ctypes.c_float]
    normal: ctypes.Array[ctypes.c_float]
    impulse: ctypes.Array[ctypes.c_float]
    separation: float
    faceIndex0: int
    faceIndex1: int
    material0: int
    material1: int

class FrictionAnchor(ctypes.Structure):
    position: ctypes.Array[ctypes.c_float]
    impulse: ctypes.Array[ctypes.c_float]
