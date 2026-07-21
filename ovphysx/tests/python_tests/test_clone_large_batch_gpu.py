# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Regression coverage for NVBugs 6473884 (large clone batch use-after-free)."""

import pytest
from ovphysx.types import TensorType
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
        physx_sdk.warmup_gpu()
        for _ in range(10):
            physx_sdk.step(1.0 / 60.0)
        physx_sdk.wait_all()

        spot_paths = [
            f"/World/envs/env{i}/table"
            for i in (1, num_targets // 2, num_targets)
        ]
        binding = physx_sdk.create_tensor_binding(
            prim_paths=spot_paths,
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        try:
            assert binding.count == len(spot_paths), (
                f"{label}: expected {len(spot_paths)} spot-check bodies, got {binding.count}"
            )
        finally:
            binding.destroy()

    run_cycle("first")
    physx_sdk.reset_stage()
    physx_sdk.wait_all()
    run_cycle("second")

    wildcard = physx_sdk.create_tensor_binding(
        pattern="/World/envs/env*/table",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    try:
        expected = num_targets + 1
        assert wildcard.count == expected, (
            f"second cycle: wildcard binding expected {expected} bodies, got {wildcard.count}"
        )
    finally:
        wildcard.destroy()
