# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-6 AC-9
# @maps_to TEST-PYTHON-TASK-001
# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001

"""Default Python operations exercised against the actual CPU runtime."""

import gc

import numpy as np
import pytest
from ovphysx import TaskError
from ovphysx.types import SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def _load(sdk):
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))


def test_default_read_uses_completed_query_and_preserves_its_snapshot(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    creation = sdk.query(SimObjectType.RIGID_BODY)
    with pytest.raises(RuntimeError, match="not collected"):
        _ = creation.result
    sdk.wait_task(creation.task)
    selector = creation.result
    operation = sdk.read(selector, ["position"], metadata=True)
    sdk.wait_all()
    read = operation.result
    assert read.groups == []
    assert read.metadata.total_prim_count == selector.metadata.total_prim_count > 0
    selector.close().wait()
    groups = 0
    while (group := read.fetch_next().wait()) is not None:
        groups += 1
        assert np.all(np.isfinite(group.tensors[0].numpy()))
    assert groups == read.metadata.attributes[0].group_count > 0
    read.close().wait()
    read.groups.clear()
    gc.collect()
    sdk.wait_all()


def test_multi_attribute_write_keeps_request_identity_without_metadata(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    selector = sdk.query(SimObjectType.RIGID_BODY, metadata=False).wait()
    names = ["linearVelocity", "angularVelocity"]
    targets = [np.array([2.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])]
    write = sdk.write(selector, names).wait()
    assert write.metadata is None and write.groups == []
    dictionary = selector.dictionary
    tokens = [dictionary.intern_token(name) for name in names]
    selector.close().wait()
    seen = set()
    while (group := write.fetch_next().wait()) is not None:
        index = group.attribute_index
        seen.add(index)
        assert group.attribute == tokens[index]
        group.tensors[0].assign(np.tile(targets[index], (group.prim_count, 1)).astype(np.float32))
        write.commit(group).wait()
    assert seen == {0, 1}
    write.close().wait()
    with sdk.read_sync(SimObjectType.RIGID_BODY, names) as read:
        for group in read.groups:
            index = tokens.index(group.attribute)
            expected = np.tile(targets[index], (group.prim_count, 1))
            np.testing.assert_allclose(group.tensors[0].numpy(), expected, atol=1e-6)
    group = None
    read.groups.clear()
    gc.collect()
    sdk.wait_all()


def test_failed_multi_attribute_open_is_cached_without_a_session(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    selector = sdk.query(SimObjectType.RIGID_BODY).wait()
    operation = sdk.write(selector, ["linearVelocity", "unknownAttribute", "angularVelocity"])
    with pytest.raises(TaskError, match="unknownAttribute"):
        sdk.wait_all()
    for access in (lambda: operation.result, operation.wait):
        with pytest.raises(TaskError, match="unknownAttribute"):
            access()
    selector.close().wait()
    assert sdk.wait_all() is None
