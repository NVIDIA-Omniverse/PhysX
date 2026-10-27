# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#

from .bindings._physxCooking import PhysxCookingServicePrivate, acquire_physx_cooking_service_private

from .ifacecache import cached_interface

def get_physx_cooking_service_private_interface() -> PhysxCookingServicePrivate:
    return cached_interface(get_physx_cooking_service_private_interface, acquire_physx_cooking_service_private)

from .scripts.extension import *
