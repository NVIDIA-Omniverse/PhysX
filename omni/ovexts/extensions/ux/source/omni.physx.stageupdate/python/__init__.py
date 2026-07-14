# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = ["get_physx_stage_update_node_interface"]

from .bindings._physxStageUpdateNode import IPhysxStageUpdateNode, acquire_physx_stage_update_node_interface

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_stage_update_node_interface() -> IPhysxStageUpdateNode:
    return _get_interface(get_physx_stage_update_node_interface, acquire_physx_stage_update_node_interface)

from .scripts.extension import OmniPhysXStageUpdateExtension as _OmniPhysXStageUpdateExtension
