# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import os

import pytest
from ovphysx.types import TensorType
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


def test_ovstage_attach_allows_immediate_tensor_binding(physx_sdk):
    usd_path = os.path.join(os.path.dirname(__file__), "../data/api_surface_permutations.usda")
    load_usd_with_ovstage(physx_sdk, usd_path)

    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/Cube"],
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )

    physx_sdk.wait_all()
    binding.destroy()
