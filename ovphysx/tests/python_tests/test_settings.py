# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for process-global configuration at runtime."""

import ctypes
import gc

from ovphysx import GlobalConfigKey, get_global_config, set_global_config
from ovphysx.api import _make_scoped_config_value


class TestGlobalConfigRuntime:
    """Test process-global config set/get at runtime."""

    def test_set_global_config_bool(self, physx_sdk):
        """Test setting a boolean global config entry at runtime."""
        key = GlobalConfigKey.DISABLE_CONTACT_PROCESSING
        set_global_config(key, True)
        assert get_global_config(key) is True
        set_global_config(key, False)
        assert get_global_config(key) is False


def test_scoped_string_value_keeps_encoded_storage_alive():
    value = _make_scoped_config_value("persistent-value")

    gc.collect()

    assert value._string_keepalive is not None
    assert ctypes.string_at(
        value.value.string_value.ptr,
        value.value.string_value.length,
    ) == b"persistent-value"
