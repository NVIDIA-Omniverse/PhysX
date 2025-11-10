# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .bindings._physx import PhysX, PhysXUnitTests, PhysXVisualization, PhysXCooking, PhysXCookingPrivate, PhysXSceneQuery, IPhysxSimulation, IPhysxBenchmarks
from .bindings._physx import IPhysxAttachmentPrivate, IPhysxPropertyQuery, IPhysxReplicator, IPhysxStageUpdate, IPhysxStatistics
from .bindings._physx import acquire_physx_interface, acquire_physxunittests_interface, acquire_physx_visualization_interface
from .bindings._physx import acquire_physx_scene_query_interface, acquire_physx_cooking_interface, acquire_physx_cooking_private_interface, acquire_physx_simulation_interface
from .bindings._physx import acquire_physx_benchmarks_interface, acquire_physx_attachment_private_interface, acquire_physx_property_query_interface
from .bindings._physx import acquire_physx_replicator_interface, acquire_physx_stage_update_interface, acquire_physx_statistics_interface

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_interface() -> PhysX:
    return _get_interface(get_physx_interface, acquire_physx_interface)

def get_physxunittests_interface() -> PhysXUnitTests:
    return _get_interface(get_physxunittests_interface, acquire_physxunittests_interface)

def get_physx_property_query_interface() -> IPhysxPropertyQuery:
    return _get_interface(get_physx_property_query_interface, acquire_physx_property_query_interface)

def get_physx_visualization_interface() -> PhysXVisualization:
    return _get_interface(get_physx_visualization_interface, acquire_physx_visualization_interface)

def get_physx_scene_query_interface() -> PhysXSceneQuery:
    return _get_interface(get_physx_scene_query_interface, acquire_physx_scene_query_interface)

def get_physx_cooking_interface() -> PhysXCooking:
    return _get_interface(get_physx_cooking_interface, acquire_physx_cooking_interface)

def get_physx_cooking_private_interface() -> PhysXCookingPrivate:
    return _get_interface(get_physx_cooking_private_interface, acquire_physx_cooking_private_interface)

def get_physx_simulation_interface() -> IPhysxSimulation:
    return _get_interface(get_physx_simulation_interface, acquire_physx_simulation_interface)

def get_physx_benchmarks_interface() -> IPhysxBenchmarks:
    return _get_interface(get_physx_benchmarks_interface, acquire_physx_benchmarks_interface)

def get_physx_attachment_private_interface() -> IPhysxAttachmentPrivate:
    return _get_interface(get_physx_attachment_private_interface, acquire_physx_attachment_private_interface)

def get_physx_replicator_interface() -> IPhysxReplicator:
    return _get_interface(get_physx_replicator_interface, acquire_physx_replicator_interface)

def get_physx_stage_update_interface() -> IPhysxStageUpdate:
    return _get_interface(get_physx_stage_update_interface, acquire_physx_stage_update_interface)

def get_physx_statistics_interface() -> IPhysxStatistics:
    return _get_interface(get_physx_statistics_interface, acquire_physx_statistics_interface)

from .scripts.extension import *
from .scripts import utils
