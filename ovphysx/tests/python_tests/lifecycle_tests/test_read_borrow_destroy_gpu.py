# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-9
# @implements REQ-PYTHON-READ-001
# @covers AC-5

"""Explicit destroy keeps borrowed GPU read storage safe until its last view is dropped."""

import gc

import numpy as np
import pytest

from ovphysx import PhysX, PhysXConfig
from ovphysx.types import SimObjectType
from test_utils import attach_usd_with_ovstage, data_path


def test_destroy_rejects_live_gpu_read_view_and_allows_retry():
    sdk = PhysX(config=PhysXConfig(carbonite_overrides={"/physics/suppressReadback": True}))
    stage = result = array = view = None
    try:
        stage = attach_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
        sdk.step_sync(1.0 / 60.0)
        result = sdk.read_sync(SimObjectType.RIGID_BODY, ["position"])
        array = result.groups[0].tensors[0]
        assert array.device.is_cuda is True
        view = array[1:]
        expected = view.numpy().copy()
        result.close()
        result = array = None
        gc.collect()
        handle = sdk.handle
        attachment = sdk._capture_attachment_query()

        with pytest.raises(RuntimeError, match="read arrays and views"):
            sdk.destroy()
        assert sdk.handle == handle
        assert sdk.is_stage_attached()
        assert sdk._is_query_valid(attachment)
        sdk._release_attachment_query(attachment)
        assert sdk._attached_ovstage is stage
        np.testing.assert_array_equal(view.numpy(), expected)
        sdk.step_sync(1.0 / 60.0)
        np.testing.assert_array_equal(view.numpy(), expected)

        view = None
        gc.collect()
        sdk.destroy()
        sdk.destroy()
        with pytest.raises(RuntimeError, match="has been destroyed"):
            _ = sdk.handle
    finally:
        result = array = view = None
        gc.collect()
        sdk.destroy()
        if stage is not None:
            stage.destroy()
