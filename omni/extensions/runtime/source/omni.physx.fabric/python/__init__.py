# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .bindings._physxFabric import PhysXFabric
from .bindings._physxFabric import acquire_physx_fabric_interface
from .bindings._physxFabric import release_physx_fabric_interface

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_fabric_interface() -> PhysXFabric:
    return _get_interface(get_physx_fabric_interface, acquire_physx_fabric_interface)

from .scripts.extension import *
