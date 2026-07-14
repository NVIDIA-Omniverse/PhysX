# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = ["get_physx_graph_interface"]

from .bindings._physxGraph import acquire_physx_graph_interface, IPhysxGraph

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_graph_interface() -> IPhysxGraph:
    return _get_interface(get_physx_graph_interface, acquire_physx_graph_interface)

from .scripts.extension import PhysxGraphExtension as _PhysxGraphExtension
