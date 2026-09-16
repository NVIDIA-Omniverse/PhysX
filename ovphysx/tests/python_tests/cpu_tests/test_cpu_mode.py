# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Process-wide hard CPU-only mode query (REQ-CAPI-CPU-001).

cpu_tests/conftest.py calls PhysX.set_cpu_mode(True) in the session fixture
before creating any instance. This test depends on that fixture so the policy
is active when get_cpu_mode() is queried.
"""

from ovphysx import PhysX


def test_get_cpu_mode_reports_true_after_set(physx_sdk):
    assert PhysX.get_cpu_mode() is True
