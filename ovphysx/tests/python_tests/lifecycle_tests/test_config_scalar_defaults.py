# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Fresh-process scalar defaults. No instance is created.

@implements REQ-CAPI-CONFIG-001
@covers AC-5
@implements REQ-CAPI-COMPILED-CACHE-001
@covers AC-2

Runs in its own subprocess (see test_python_runtime.cmake).
"""

from ovphysx import CompiledStageCacheMode, GlobalConfigKey, ProfilerBackend, get_global_config


def test_scalar_defaults_before_create(monkeypatch):
    """Declared scalar defaults are visible before PhysX registers settings."""
    monkeypatch.delenv("OVPHYSX_PROFILER", raising=False)
    monkeypatch.delenv("OVPHYSX_PROFILER_OUTPUT", raising=False)
    monkeypatch.delenv("OVPHYSX_DISABLE_GPU", raising=False)

    assert get_global_config(GlobalConfigKey.NUM_THREADS) == 8
    assert get_global_config(GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB) == 256
    assert get_global_config(GlobalConfigKey.SCENE_MULTI_GPU_MODE) == 0
    assert get_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING) is False
    assert get_global_config(GlobalConfigKey.COLLISION_CONE_CUSTOM_GEOMETRY) is True
    assert get_global_config(GlobalConfigKey.COLLISION_CYLINDER_CUSTOM_GEOMETRY) is True
    assert get_global_config(GlobalConfigKey.PROFILER_BACKEND) == int(ProfilerBackend.NONE)
    assert get_global_config(GlobalConfigKey.COMPILED_STAGE_CACHE) == int(CompiledStageCacheMode.AUTOMATIC)
