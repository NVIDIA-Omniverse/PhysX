# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Legacy explicit-empty active CUDA GPU configuration.

@implements REQ-CAPI-CONFIG-001
@covers AC-5 AC-7 AC-8

Runs in its own subprocess (see test_python_runtime.cmake).
"""

import pytest

from ovphysx import ConfigKey, PhysX, PhysXConfig


def test_legacy_explicit_empty_active_cuda_gpus_is_configured():
    """An explicit empty GPU list remains distinct from an omitted list."""
    with pytest.warns(DeprecationWarning, match=r"set_global_config\(\) for settings with a GlobalConfigKey"):
        physx = PhysX(config=PhysXConfig(), active_cuda_gpus="")
    try:
        assert physx.get_config(ConfigKey.ACTIVE_CUDA_GPUS) == ""
    finally:
        physx.destroy()
