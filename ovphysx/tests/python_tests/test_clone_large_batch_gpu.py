# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression coverage for NVBugs 6473884 (large clone batch use-after-free)."""

import pytest
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import data_path, load_usd_with_ovstage


@pytest.mark.parametrize("num_targets", [256, 512])
def test_large_clone_batch_survives_reset_cycle(physx_sdk, num_targets: int):
    """DirectGPU large clone batches must survive reset_stage() -> reload -> clone."""
    usd_path = data_path("basic_simulation.usda")

    def run_cycle(label: str) -> None:
        load_usd_with_ovstage(physx_sdk, usd_path)
        physx_sdk.wait_all()
        targets = [f"/World/envs/env{i}" for i in range(1, num_targets + 1)]
        physx_sdk.clone("/World/envs/env0", targets)
        physx_sdk.wait_all()
        physx_sdk.warmup()
        for _ in range(10):
            physx_sdk.step(1.0 / 60.0)
        physx_sdk.wait_all()

        # The tables (source + clones) are the scene's only rigid bodies, so a whole-set read
        # must see all num_targets + 1. That verifies the whole batch survived the cycle without
        # a use-after-free.
        with physx_sdk.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
            body_count = sum(g.prim_count for g in result.groups)
        assert body_count == num_targets + 1, (
            f"{label}: expected {num_targets + 1} bodies, got {body_count}"
        )

    run_cycle("first")
    physx_sdk.reset_stage()
    physx_sdk.wait_all()
    run_cycle("second")

    with physx_sdk.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        body_count = sum(g.prim_count for g in result.groups)
    expected = num_targets + 1
    assert body_count == expected, (
        f"second cycle: expected {expected} bodies, got {body_count}"
    )
