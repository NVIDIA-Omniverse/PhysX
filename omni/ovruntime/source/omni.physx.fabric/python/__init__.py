# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = ["get_physx_fabric_interface"]

from .bindings._physxFabric import PhysXFabric, acquire_physx_fabric_interface

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_fabric_interface() -> PhysXFabric:
    return _get_interface(get_physx_fabric_interface, acquire_physx_fabric_interface)

from .scripts.extension import PhysxFabricExtension as _PhysXFabricExtension
