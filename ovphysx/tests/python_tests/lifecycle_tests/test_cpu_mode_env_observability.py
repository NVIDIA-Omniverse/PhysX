# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Fresh-process coverage for get_cpu_mode default and OVPHYSX_DISABLE_GPU latch.

Runs in its own subprocess (see test_python_runtime.cmake). Verifies that an
observational pre-init get_cpu_mode() does not permanently miss a later
setenv of OVPHYSX_DISABLE_GPU before ovphysx_initialize (REQ-CAPI-CPU-001).
"""

import os

from ovphysx import PhysX


def test_get_cpu_mode_default_false_and_env_latches_at_initialize():
    os.environ.pop("OVPHYSX_DISABLE_GPU", None)

    assert PhysX.get_cpu_mode() is False

    os.environ["OVPHYSX_DISABLE_GPU"] = "1"
    # Pre-init reads stay live, so the env is visible before initialize.
    assert PhysX.get_cpu_mode() is True

    physx = PhysX(ignore_version_mismatch=True)
    try:
        # Post-init reads must use the latched process value, not a live getenv.
        os.environ.pop("OVPHYSX_DISABLE_GPU", None)
        assert PhysX.get_cpu_mode() is True
    finally:
        physx.destroy()
