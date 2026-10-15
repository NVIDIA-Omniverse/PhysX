# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Pipeline variants retain their base scene's stage units and up axis."""

# @implements REQ-PACKAGING-SAMPLE-UNITS-001
# @covers AC-1

from pathlib import Path

import pytest

pytest.importorskip("pxr", reason="sample metadata validation requires OpenUSD")

from pxr import Usd  # noqa: E402


@pytest.mark.parametrize(
    "variant",
    [
        "boxes_falling_on_groundplane_gpu",
        "mixed_base_articulations_gpu",
        "empty_dynamic_boxes_cpu",
        "simple_physics_scene_cpu",
    ],
)
def test_pipeline_variant_stage_metadata(variant, codeless_schemas):
    data = Path(__file__).resolve().parents[2] / "data"
    stage = Usd.Stage.Open(str(data / f"{variant}.usda"))
    base = Usd.Stage.Open(str(data / f"{variant.rsplit('_', 1)[0]}.usda"))
    for key in ("metersPerUnit", "upAxis"):
        assert stage.HasAuthoredMetadata(key), key
        assert stage.GetMetadata(key) == base.GetMetadata(key), key
