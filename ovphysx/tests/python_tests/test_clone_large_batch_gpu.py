# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression coverage for NVBugs 6473884 (large clone batch use-after-free)."""

import pytest
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import data_path, load_usd_with_ovstage


@pytest.mark.parametrize("num_targets", [256, 512])
@pytest.mark.parametrize(
    "scene,source,object_type,attribute,source_count",
    [
        ("basic_simulation.usda", "/World/envs/env0", SimObjectType.RIGID_BODY, "position", 1),
        ("two_articulations_gpu.usda", "/World/articulation", SimObjectType.ARTICULATION, "rootPosition", 2),
    ],
)
def test_large_clone_batch_survives_reset_cycle(physx_sdk, num_targets, scene, source, object_type, attribute, source_count):
    """DirectGPU large clone batches must survive reset_stage() -> reload -> clone."""
    usd_path = data_path(scene)
    expected = num_targets + source_count

    def run_cycle(label: str) -> None:
        load_usd_with_ovstage(physx_sdk, usd_path)
        physx_sdk.wait_all()
        targets = [f"/World/envs/env{i}" for i in range(1, num_targets + 1)]
        physx_sdk.clone(source, targets)
        physx_sdk.wait_all()
        physx_sdk.warmup()
        for _ in range(10):
            physx_sdk.step(1.0 / 60.0)
        physx_sdk.wait_all()

        # Read every original and clone after rebuilding the scene's views and stepping.
        with physx_sdk.read(object_type, [attribute], scope=ObjectScope.ALL) as result:
            body_count = sum(g.prim_count for g in result.groups)
        assert body_count == expected, f"{label}: expected {expected} objects, got {body_count}"

    run_cycle("first")
    physx_sdk.reset_stage()
    physx_sdk.wait_all()
    run_cycle("second")
