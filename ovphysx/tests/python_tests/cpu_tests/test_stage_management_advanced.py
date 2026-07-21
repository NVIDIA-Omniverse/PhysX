# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Advanced stage management tests: path_prefix, reset/reload workflow.

These tests complement test_usd_loading.py (which covers ovstage attach/update
happy paths, path_prefix rejection, multi-file sequences) and do not duplicate that
coverage.
"""

import os

import pytest
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


# ---------------------------------------------------------------------------
# ovstage path_prefix
# ---------------------------------------------------------------------------


def test_ovstage_empty_path_prefix_succeeds(physx_sdk):
    """An explicit empty path_prefix is equivalent to the default."""
    op = load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"), path_prefix="")
    physx_sdk.wait_op(op)
    physx_sdk.wait_all()
    # A successful attach must allow stepping the simulation without crashing.
    physx_sdk.step_sync(1.0 / 60.0)


# ---------------------------------------------------------------------------
# Reset then re-add (stage lifecycle)
# ---------------------------------------------------------------------------


def test_reset_clears_stage_for_new_load(physx_sdk):
    """After reset, a new ovstage attach/update + step must succeed."""
    # Load scene A
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()
    physx_sdk.step_sync(1.0 / 60.0)

    # Reset stage
    physx_sdk.reset_stage()
    physx_sdk.wait_all()

    # Load a different scene B
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.step_sync(1.0 / 60.0)  # must not crash


# ---------------------------------------------------------------------------
# ovstage population error path
# ---------------------------------------------------------------------------


def test_ovstage_nonexistent_path_raises(physx_sdk):
    """ovstage population with a nonexistent path must raise RuntimeError.

    The helper raises before returning when the underlying load fails, so no
    wait_op() is needed — and the returned value is a tuple, which would
    fail wait_op() with TypeError and mask the real assertion.
    """
    with pytest.raises(RuntimeError) as exc_info:
        load_usd_with_ovstage(physx_sdk, data_path("does_not_exist_12345.usda"))
    # Error message should be non-empty
    assert str(exc_info.value)
