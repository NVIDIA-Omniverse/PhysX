# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

__all__ = ["get_physx_vehicle_interface"]

from .bindings._physxVehicle import acquire_physx_vehicle_interface, IPhysxVehicle

from .ifacecache import cached_interface

def get_physx_vehicle_interface() -> IPhysxVehicle:
    return cached_interface(get_physx_vehicle_interface, acquire_physx_vehicle_interface)

from .scripts.extension import PhysxVehicleExtension
