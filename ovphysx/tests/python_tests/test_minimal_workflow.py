# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

from test_utils import load_usd_with_ovstage


def test_minimal_simulation_step(physx_sdk):
    """Smoke test: load a USD stage, step once, wait for completion.

    Validates the most basic simulation workflow does not hang or crash.
    """
    import os

    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()

    physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()
