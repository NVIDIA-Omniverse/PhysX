# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Unit tests for PhysX typed config API at runtime.

Init-time config tests (PhysXConfig at construction) live in
lifecycle_tests/ because they create their own PhysX instances and
must run in subprocesses.
"""

from ovphysx import ConfigBool, ConfigInt32


class TestTypedConfigRuntime:
    """Test typed config set/get at runtime."""

    def test_set_config_bool(self, physx_sdk):
        """Test setting a bool config entry at runtime."""
        key = ConfigBool.DISABLE_CONTACT_PROCESSING
        physx_sdk.set_config_bool(key, True)
        assert physx_sdk.get_config_bool(key) is True
        physx_sdk.set_config_bool(key, False)
        assert physx_sdk.get_config_bool(key) is False

    def test_set_config_int32(self, physx_sdk):
        """Test setting an int32 config entry at runtime."""
        physx_sdk.set_config_int32(ConfigInt32.NUM_THREADS, 8)
        assert physx_sdk.get_config_int32(ConfigInt32.NUM_THREADS) == 8
