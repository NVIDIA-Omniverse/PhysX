# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Legacy creation rejects Python ints that do not fit int32. ONE create+destroy per file.

Runs in its own subprocess (see test_python_runtime.cmake).
"""

import pytest

from ovphysx import GlobalConfigKey, PhysX, PhysXConfig, get_global_config

_INT32_ERROR = "Configuration integer does not fit backend int32 storage"


def _reject_legacy(**fields):
    with pytest.warns(DeprecationWarning, match=r"set_global_config\(\)"):
        with pytest.raises(ValueError, match=_INT32_ERROR):
            PhysX(config=PhysXConfig(**fields))


def test_legacy_config_rejects_int_outside_int32():
    """Oversized legacy ints are rejected before create; representable ints apply."""
    assert get_global_config(GlobalConfigKey.NUM_THREADS) == 8
    assert get_global_config(GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB) == 256
    assert get_global_config(GlobalConfigKey.SCENE_MULTI_GPU_MODE) == 0

    _reject_legacy(num_threads=2**32 + 4)
    assert get_global_config(GlobalConfigKey.NUM_THREADS) == 8

    _reject_legacy(ovstage_read_pool_max_mb=2**31)
    assert get_global_config(GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB) == 256

    _reject_legacy(scene_multi_gpu_mode=2**31)
    assert get_global_config(GlobalConfigKey.SCENE_MULTI_GPU_MODE) == 0

    with pytest.warns(DeprecationWarning, match=r"set_global_config\(\)"):
        physx = PhysX(config=PhysXConfig(num_threads=3, ovstage_read_pool_max_mb=512))
    try:
        assert get_global_config(GlobalConfigKey.NUM_THREADS) == 3
        assert get_global_config(GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB) == 512
    finally:
        physx.destroy()
