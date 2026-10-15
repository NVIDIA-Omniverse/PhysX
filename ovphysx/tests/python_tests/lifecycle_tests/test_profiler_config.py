# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Profiler backend selection test -- ONE create+destroy per file.

@implements REQ-CAPI-PROFILER-001
@covers AC-2

The selection is resolved during instance creation, so the resulting state has to
be observed on a live instance. The default-off case is covered by the C test
(ProfilerBackend.DisabledByDefault), which can afford more than one instance per
process. Whether a consumer records the zones cannot be checked from the test
process; the NVTX case is verified manually and the trace case by the C test
ProfilerTraceBackend.WritesChromeTraceOnLastInstanceShutdown.

Runs in its own subprocess (see test_python_runtime.cmake).
"""

from ovphysx import GlobalConfigKey, PhysX, ProfilerBackend, get_global_config, set_global_config


def test_profiler_backend_config_is_observable(monkeypatch):
    """Global profiler config selects NVTX and reads back as NVTX."""
    monkeypatch.delenv("OVPHYSX_PROFILER", raising=False)
    set_global_config(GlobalConfigKey.PROFILER_BACKEND, ProfilerBackend.NVTX)
    physx = PhysX()
    try:
        assert get_global_config(GlobalConfigKey.PROFILER_BACKEND) == ProfilerBackend.NVTX
    finally:
        physx.destroy()
