# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from .bindings._physxCooking import PhysxCookingServicePrivate, acquire_physx_cooking_service_private

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_cooking_service_private_interface() -> PhysxCookingServicePrivate:
    return _get_interface(get_physx_cooking_service_private_interface, acquire_physx_cooking_service_private)

from .scripts.extension import *
