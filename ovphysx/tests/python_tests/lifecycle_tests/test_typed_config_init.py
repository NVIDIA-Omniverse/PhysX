# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Init-time typed config tests, ONE create+destroy per file.

Tests that PhysXConfig entries passed during construction are applied correctly.
Runs in its own subprocess (see test_python_runtime.cmake).
"""

import pytest

from ovphysx import GlobalConfigKey, PhysX, PhysXConfig, get_global_config


def test_typed_config_at_init():
    """Comprehensive test for PhysXConfig entries passed at initialization.

    Covers:
        - Bool config entries
        - Int32 config entries
        - Multiple config entries at once
        - raw Carbonite setting override (non-typed path)
    """
    with pytest.warns(DeprecationWarning, match=r"set_global_config\(\) for settings with a GlobalConfigKey") as caught:
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
        assert "Carbonite-only settings still use PhysXConfig.carbonite_overrides" in str(caught[0].message)
        assert get_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING) is True
        assert get_global_config(GlobalConfigKey.COLLISION_CONE_CUSTOM_GEOMETRY) is False
        assert get_global_config(GlobalConfigKey.COLLISION_CYLINDER_CUSTOM_GEOMETRY) is True
        assert get_global_config(GlobalConfigKey.NUM_THREADS) == 4
    finally:
        physx.destroy()
