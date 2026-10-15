# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-9

"""Reject destruction inside an open read session, then clean up normally.

Runs one create/destroy cycle in its own subprocess (see test_python_runtime.cmake).
"""

import os

import pytest

from ovphysx.types import ObjectScope, SimObjectType
from test_utils import load_usd_with_ovstage

from ovphysx import PhysX


def _data_path(name):
    tests_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(tests_dir, "data", name)


def test_destroy_during_open_read_session_rejects_without_teardown():
    """An open result retains read arrays; rejection leaves its normal close usable."""
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    result = None
    try:
        load_usd_with_ovstage(physx, _data_path("boxes_falling_on_groundplane.usda"))
        physx.warmup()
        physx.wait_all()

        with physx.read_sync(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
            assert result.groups, "session should be live with data before destroy"
            handle = physx.handle
            with pytest.raises(RuntimeError, match="read arrays and views"):
                physx.destroy()
            assert physx.handle == handle
            assert result.groups[0].tensors[0].numpy().size > 0
        assert result._closed
    finally:
        result = None
        physx.destroy()
    assert physx._released
