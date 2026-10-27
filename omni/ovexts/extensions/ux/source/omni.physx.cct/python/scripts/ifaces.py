# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from ..bindings._physxCct import acquire_physx_cct_interface, IPhysxCct
from ..ifacecache import cached_interface, release_interface_caches

__all__ = [
    "get_physx_cct_interface",
]


def get_physx_cct_interface() -> IPhysxCct:
    return cached_interface(get_physx_cct_interface, acquire_physx_cct_interface)
