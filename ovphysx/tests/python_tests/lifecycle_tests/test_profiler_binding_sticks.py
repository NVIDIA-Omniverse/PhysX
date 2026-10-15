# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Bound profiler selection survives a later environment change.

@implements REQ-CAPI-CONFIG-001
@covers AC-5

Runs in its own subprocess (see test_python_runtime.cmake).
"""

from ovphysx import GlobalConfigKey, PhysX, ProfilerBackend, get_global_config, set_global_config


def test_bound_profiler_ignores_later_environment(monkeypatch):
    """After NVTX binds, a later OVPHYSX_PROFILER change does not change the getter."""
    monkeypatch.delenv("OVPHYSX_PROFILER", raising=False)
    monkeypatch.delenv("OVPHYSX_PROFILER_OUTPUT", raising=False)
    set_global_config(GlobalConfigKey.PROFILER_BACKEND, int(ProfilerBackend.NVTX))
    set_global_config(GlobalConfigKey.PROFILER_OUTPUT_PATH, "a.json")

    first = PhysX()
    try:
        monkeypatch.setenv("OVPHYSX_PROFILER", "trace")
        monkeypatch.setenv("OVPHYSX_PROFILER_OUTPUT", "b.json")
        second = PhysX()
        try:
            assert get_global_config(GlobalConfigKey.PROFILER_BACKEND) == int(ProfilerBackend.NVTX)
            assert get_global_config(GlobalConfigKey.PROFILER_OUTPUT_PATH) is None
        finally:
            second.destroy()
    finally:
        first.destroy()
