# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Blocked Carbonite path tests — ONE create attempt per file.

Tests that internally-managed paths (e.g. /physics/cudaDevice) are rejected
by the C layer with OVPHYSX_API_INVALID_ARGUMENT (surfaced as RuntimeError).

Runs in its own subprocess (see test_python_runtime.cmake).
"""

import pytest

from ovphysx import PhysX, PhysXConfig


def test_blocked_carbonite_paths():
    """/physics/cudaDevice must be blocked -- use active_cuda_gpus instead."""
    with pytest.raises(RuntimeError):
        PhysX(
            config=PhysXConfig(
                carbonite_overrides={"/physics/cudaDevice": 1},
            )
        )
