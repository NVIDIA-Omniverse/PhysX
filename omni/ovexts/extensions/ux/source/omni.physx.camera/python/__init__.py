# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

__all__ = ["get_physx_camera_interface"]

from .bindings._physxCamera import acquire_physx_camera_interface, IPhysxCamera

from .ifacecache import cached_interface

def get_physx_camera_interface() -> IPhysxCamera:
    return cached_interface(get_physx_camera_interface, acquire_physx_camera_interface)

from .scripts.extension import *
