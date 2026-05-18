# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from ..bindings._physxCct import acquire_physx_cct_interface, IPhysxCct

__all__ = [
    "get_physx_cct_interface"
]


def get_physx_cct_interface() -> IPhysxCct:
    if not hasattr(get_physx_cct_interface, "iface"):
        get_physx_cct_interface.iface = acquire_physx_cct_interface()
    return get_physx_cct_interface.iface
