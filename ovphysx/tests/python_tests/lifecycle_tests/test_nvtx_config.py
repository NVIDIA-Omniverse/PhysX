# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""NVTX profiling opt-in test, ONE create+destroy per file.

@implements REQ-CAPI-NVTX-001
@covers AC-2

The opt-in is resolved during instance creation, so the enabled state has to be
observed on a live instance. The default-off case is covered by the C test
(NvtxProfiling.DisabledByDefault), which can afford more than one instance per
process. Whether Nsight Systems records the ranges cannot be checked from the
test process. That is verified manually.

Runs in its own subprocess (see test_python_runtime.cmake).
"""

from ovphysx import ConfigBool, PhysX, PhysXConfig


def test_nvtx_enabled_config_is_observable():
    """PhysXConfig(nvtx_enabled=True) turns the setting on and reads back as True."""
    physx = PhysX(config=PhysXConfig(nvtx_enabled=True))
    try:
        assert physx.get_config_bool(ConfigBool.NVTX_ENABLED) is True
    finally:
        physx.destroy()
