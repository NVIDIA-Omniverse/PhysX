# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-REPLICATE-001
# @covers AC-2 AC-7

"""CPU-mode clone regression tests.

NVBugs 6401436 reproduced on the default CPU path: cloning a bare leaf rigid body (the prim
itself carries the rigid-body API, no Xform wrapper) returned success and then SIGSEGV'd on the
first step. Both step entry points crashed, so both are exercised here.
"""

import pytest
from test_utils import clone_bare_leaf_body_and_step


@pytest.mark.parametrize("step_api", ["sync", "async"])
def test_clone_bare_leaf_rigid_body_simulates_cpu(physx_sdk, step_api):
    """A cloned bare leaf body simulates and falls from its anchor on the CPU session."""
    clone_bare_leaf_body_and_step(physx_sdk, step_api=step_api)
