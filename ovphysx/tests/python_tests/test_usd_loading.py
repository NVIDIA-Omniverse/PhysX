# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import pytest
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import data_path, destroy_ovstage_test_attachments, load_usd_with_ovstage


def test_attach_reset_workflow(physx_sdk):
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))

    destroy_ovstage_test_attachments(physx_sdk)
    reset_op = physx_sdk.reset_stage()
    physx_sdk.wait_op(reset_op)
    physx_sdk.wait_all()


def test_ovstage_path_prefix_rejected(physx_sdk):
    with pytest.raises(RuntimeError, match=r"path_prefix"):
        load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"), path_prefix="/Prefixed")


@pytest.mark.parametrize("usd_filenames", [["basic_simulation.usda"], ["api_surface_permutations.usda", "basic_simulation.usda"]])
def test_attach_multiple_files_sequentially(physx_sdk, usd_filenames):
    for filename in usd_filenames:
        load_usd_with_ovstage(physx_sdk, data_path(filename))
        destroy_ovstage_test_attachments(physx_sdk)
        physx_sdk.reset_stage()
        physx_sdk.wait_all()


def test_ovstage_invalid_path_raises(physx_sdk):
    with pytest.raises(RuntimeError):
        load_usd_with_ovstage(physx_sdk, "does_not_exist.usda")


def test_ovstage_attach_allows_immediate_read(physx_sdk):
    # A scene WITH rigid bodies, so an empty pre-step result reflects the clean-omission
    # contract rather than an empty scene.
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    # A read session must open immediately after attach, with no step. On this DirectGPU scene the
    # superset view does not exist until the first step, so the pre-step read is a clean omission:
    # zero groups, not an error (symmetric with the write's step-first rule).
    with physx_sdk.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        assert result.groups == []

    # After warmup the same read resolves the bodies, proving the pre-step emptiness was the
    # step-first contract and not a silently broken read.
    physx_sdk.warmup()
    physx_sdk.wait_all()
    with physx_sdk.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        assert len(result.groups) > 0
