# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Real native coverage for checked, idempotent PhysX destruction."""

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-1 AC-4

import os

import ovphysx.api as api
import pytest
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import load_usd_with_ovstage

from ovphysx import PhysX


def test_destroy_is_checked_idempotent_and_refcounted():
    """Destroying one instance keeps a simultaneous instance usable."""
    initial_refcount = api._PROCESS_LIFECYCLE_REFCOUNT
    first = PhysX()
    second = None
    try:
        tests_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        usd_path = os.path.join(tests_dir, "data", "basic_simulation.usda")
        load_usd_with_ovstage(first, usd_path)
        first.wait_all()
        second = PhysX()
        assert api._PROCESS_LIFECYCLE_REFCOUNT == initial_refcount + 2

        first.destroy()
        assert api._PROCESS_LIFECYCLE_REFCOUNT == initial_refcount + 1
        with pytest.raises(RuntimeError, match="has been destroyed"):
            _ = first.handle

        msg = "has been destroyed"
        with pytest.raises((RuntimeError, AttributeError), match=msg):
            first.step(0.016)
        with pytest.raises((RuntimeError, AttributeError), match=msg):
            first.step_sync(0.016)
        with pytest.raises((RuntimeError, AttributeError), match=msg):
            load_usd_with_ovstage(first, "foo.usda")
        with pytest.raises((RuntimeError, AttributeError), match=msg):
            first.reset_stage()
        with pytest.raises((RuntimeError, AttributeError), match=msg):
            first.wait_all()
        with pytest.raises((RuntimeError, AttributeError), match=msg):
            first.wait_op(0)
        with pytest.raises((RuntimeError, AttributeError), match=msg):
            first.warmup()

        # A read or write session on the destroyed instance must raise too.
        with pytest.raises((RuntimeError, AttributeError), match=msg):
            with first.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL):
                pass
        with pytest.raises((RuntimeError, AttributeError), match=msg):
            with first.write(SimObjectType.RIGID_BODY, "position"):
                pass

        second.wait_all()
        second.destroy()
        assert api._PROCESS_LIFECYCLE_REFCOUNT == initial_refcount

        first.destroy()
        second.destroy()
        assert api._PROCESS_LIFECYCLE_REFCOUNT == initial_refcount
    finally:
        try:
            first.destroy()
        except Exception:
            pass
        if second is not None:
            try:
                second.destroy()
            except Exception:
                pass
