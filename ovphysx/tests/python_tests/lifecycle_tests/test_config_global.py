# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Config global behaviour test — ONE create+release cycle per file.

Verifies that config is process-global (Carbonite settings are shared across
all PhysX instances in the same process).

Runs in its own subprocess (see test_python_runtime.cmake).
"""

from ovphysx import ConfigBool, PhysX, PhysXConfig


def test_config_is_global_across_instances():
    """Config set via one instance is visible from another."""
    physx1 = PhysX(config=PhysXConfig(disable_contact_processing=True))
    try:
        physx2 = PhysX(config=PhysXConfig(disable_contact_processing=False))
        try:
            val1 = physx1.get_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING)
            val2 = physx2.get_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING)
            assert val1 == val2
            assert val2 is False
        finally:
            physx2.release()
    finally:
        physx1.release()
