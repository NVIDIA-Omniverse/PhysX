# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Scoped configuration behavior. ONE create+destroy cycle per file.

Verifies module-level process configuration and handle-owned instance config.

Runs in its own subprocess (see test_python_runtime.cmake).
"""

import pytest

from ovphysx import ConfigKey, GlobalConfigKey, PhysX, get_global_config, set_global_config


def test_scoped_config_global_and_instance_paths(monkeypatch):
    monkeypatch.setenv("OVPHYSX_DISABLE_GPU", "1")
    monkeypatch.setenv("OVPHYSX_PROFILER", "none")
    monkeypatch.setenv("OVPHYSX_PROFILER_OUTPUT", "environment-trace.json")
    assert get_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE) is False
    set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
    assert get_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE) is True
    with pytest.raises(ValueError):
        set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, 1)
    assert get_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE) is True
    set_global_config(GlobalConfigKey.CPU_ONLY, False)
    assert get_global_config(GlobalConfigKey.CPU_ONLY) is True
    set_global_config(GlobalConfigKey.PROFILER_BACKEND, 1)
    assert get_global_config(GlobalConfigKey.PROFILER_BACKEND) == 0
    set_global_config(GlobalConfigKey.PROFILER_OUTPUT_PATH, "configured-trace.json")
    assert get_global_config(GlobalConfigKey.PROFILER_OUTPUT_PATH) == "environment-trace.json"

    set_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING, True)
    assert get_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING) is True
    set_global_config(GlobalConfigKey.NUM_THREADS, 2)
    assert get_global_config(GlobalConfigKey.NUM_THREADS) == 2
    set_global_config(GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB, 64)
    assert get_global_config(GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB) == 64
    set_global_config(GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB, 0)
    assert get_global_config(GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB) == 0

    physx1 = PhysX()
    try:
        with pytest.raises(RuntimeError):
            set_global_config(GlobalConfigKey.PROFILER_BACKEND, 1)
        with pytest.raises(RuntimeError):
            set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, False)
        assert get_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE) is True

        assert physx1.get_config(ConfigKey.ACTIVE_CUDA_GPUS) is None
        physx1.set_config(ConfigKey.ACTIVE_CUDA_GPUS, "0")
        assert physx1.get_config(ConfigKey.ACTIVE_CUDA_GPUS) == "0"

        physx2 = PhysX()
        try:
            assert physx2.get_config(ConfigKey.ACTIVE_CUDA_GPUS) is None
            assert get_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING) is True
            with pytest.raises(ValueError):
                set_global_config(
                    GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB,
                    2**63 - 1,
                )
        finally:
            physx2.destroy()
    finally:
        physx1.destroy()

    # The last Python owner also shuts down the native process lifecycle.
    assert get_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE) is True
    set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, False)
    assert get_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE) is False
