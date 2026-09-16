# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Config validation tests, ONE create+destroy per file.

Tests that carbonite_overrides conflicts are caught and that non-conflicting
overrides work. ValueError tests are pure Python-side validation (no Carbonite
involved). The one happy-path test creates and destroys a PhysX instance.

Runs in its own subprocess (see test_python_runtime.cmake).
"""

import pytest

from ovphysx import PhysX, PhysXConfig


def test_config_validation():
    """Comprehensive test for config conflict detection and happy-path overrides.

    Covers:
        - Typed field conflict raises ValueError
        - Conflict fires even when typed field is None
        - Non-conflicting carbonite overrides work fine
    """
    # Typed field conflicts must raise ValueError (Python-side validation).
    with pytest.raises(ValueError, match="conflicts with typed field"):
        PhysX(
            config=PhysXConfig(
                carbonite_overrides={"/physics/numThreads": 8},
            )
        )

    with pytest.raises(ValueError, match="conflicts with typed field"):
        PhysX(
            config=PhysXConfig(
                carbonite_overrides={"/physics/disableContactProcessing": True},
            )
        )

    # A non-conflicting path succeeds.
    physx = PhysX(
        config=PhysXConfig(
            carbonite_overrides={"/physics/updateToUsd": False},
        )
    )
    physx.destroy()
