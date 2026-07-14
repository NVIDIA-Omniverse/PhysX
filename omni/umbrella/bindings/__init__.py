# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

"""
Physics Umbrella Python API

This module provides Python bindings for the Physics Umbrella framework,
allowing physics engines to integrate with Omniverse.
"""

from ._physics import *

__version__ = "110.0.0"
__all__ = [
    "acquire_physics_interface",
    "release_physics_interface",
    "acquire_physics_simulation_interface",
    "release_physics_simulation_interface",
    "acquire_physics_benchmarks_interface",
    "release_physics_benchmarks_interface",
    "acquire_physics_interaction_interface",
    "release_physics_interaction_interface",
    "acquire_physics_scene_query_interface",
    "release_physics_scene_query_interface",
    "acquire_physics_stage_update_interface",
    "release_physics_stage_update_interface",
    "IPhysics",
    "Simulation",
    "SimulationId",
    "SimulationFns",
    "StageUpdateFns",
    "SceneQueryFns",
    "InteractionFns",
    "BenchmarkFns",
    "SimulationRegistryEventType",
    "ContactEventType",
    "ContactEventHeader",
    "ContactData",
    "FrictionAnchor",
    "ContactEventHeaderVector",
    "ContactDataVector",
    "FrictionAnchorsDataVector",
    "ForceMode",
    "PhysicsStepContext",
    "PhysicsProfileStats",
    "SimulationEvent",
    "k_invalid_simulation_id",
    "k_invalid_subscription_id",
]
