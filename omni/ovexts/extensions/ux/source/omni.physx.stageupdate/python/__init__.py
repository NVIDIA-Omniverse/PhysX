# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

__all__ = ["get_physx_stage_update_node_interface"]

from .bindings._physxStageUpdateNode import IPhysxStageUpdateNode, acquire_physx_stage_update_node_interface

from .ifacecache import cached_interface

def get_physx_stage_update_node_interface() -> IPhysxStageUpdateNode:
    return cached_interface(get_physx_stage_update_node_interface, acquire_physx_stage_update_node_interface)

from .scripts.extension import OmniPhysXStageUpdateExtension as _OmniPhysXStageUpdateExtension
