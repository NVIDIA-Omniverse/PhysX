# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Init-time typed config tests — ONE create+release per file.

Tests that PhysXConfig entries passed during construction are applied correctly.
Runs in its own subprocess (see test_python_runtime.cmake).
"""

from ovphysx import ConfigBool, ConfigInt32, PhysX, PhysXConfig


def test_typed_config_at_init():
    """Comprehensive test for PhysXConfig entries passed at initialization.

    Covers:
        - Bool config entries
        - Int32 config entries
        - Multiple config entries at once
        - raw Carbonite setting override (non-typed path)
    """
    physx = PhysX(
        config=PhysXConfig(
            disable_contact_processing=True,
            collision_cone_custom_geometry=False,
            collision_cylinder_custom_geometry=True,
            num_threads=4,
            carbonite_overrides={"/physics/updateToUsd": False},
        )
    )
    try:
        assert physx.get_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING) is True
        assert physx.get_config_bool(ConfigBool.COLLISION_CONE_CUSTOM_GEOMETRY) is False
        assert physx.get_config_bool(ConfigBool.COLLISION_CYLINDER_CUSTOM_GEOMETRY) is True
        assert physx.get_config_int32(ConfigInt32.NUM_THREADS) == 4
    finally:
        physx.release()
