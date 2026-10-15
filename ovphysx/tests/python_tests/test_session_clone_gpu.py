# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-READ-SELECT-001
# @covers AC-2
# @maps_to TEST-READ-SELECT-001

"""Session read/write coverage for cloning into many environments (NVBug 6428316)."""

import numpy as np
from ovphysx.types import SimObjectType
from session_helpers import read_column, write_uniform
from test_utils import attach_usd_with_ovstage, data_path



# Covers test_preclone_binding_velocity_reaches_all_envs's post-clone count and velocity reach, with no binding.
def test_postclone_pattern_selects_and_drives_all_envs(physx_sdk):
    """After clone, the env pattern selects the source table and every cloned one, and a velocity
    write over that selection moves every env."""
    N = 32
    pattern = "/World/envs/*/table"
    stage = attach_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    g = int(np.ceil(np.sqrt(N)))
    physx_sdk.clone(
        source_path="/World/envs/env0",
        target_paths=[f"/World/envs/env{i}" for i in range(1, N)],
        anchor_transforms=[
            ((i % g) * 3.0, 0.0, (i // g) * 3.0, 0, 0, 0, 1)
            for i in range(1, N)
        ],
    )
    physx_sdk.wait_all()
    physx_sdk.warmup()

    tables = sorted(f"/World/envs/env{i}/table" for i in range(N))
    before = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "position", pattern=pattern)
    assert sorted(before) == tables

    # Drive every table: linear velocity x = 3.
    written = write_uniform(physx_sdk, stage, SimObjectType.RIGID_BODY, "linearVelocity", [3.0, 0.0, 0.0],
                            pattern=pattern)
    assert written == N
    physx_sdk.wait_all()

    for _ in range(60):
        physx_sdk.step(1 / 240.0)
    physx_sdk.wait_all()
    after = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "position", pattern=pattern)

    moved = sum(1 for path in tables if abs(after[path][0] - before[path][0]) > 0.3)
    assert moved == N, f"expected all {N} environments to move, got {moved}/{N}"
