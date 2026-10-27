# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

__all__ = ["get_physx_graph_interface"]

from .bindings._physxGraph import acquire_physx_graph_interface, IPhysxGraph

from .ifacecache import cached_interface

def get_physx_graph_interface() -> IPhysxGraph:
    return cached_interface(get_physx_graph_interface, acquire_physx_graph_interface)

from .scripts.extension import PhysxGraphExtension as _PhysxGraphExtension
