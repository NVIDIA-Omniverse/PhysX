# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Destroy-during-open-session teardown, ONE create+destroy.

Covers the path a plain open-after-destroy check (test_destroy.py) cannot reach: a *live* read
session whose parent PhysX is destroyed under it. The session teardown that runs from the context
``__exit__`` (``_ReadRelease.release_groups`` / ``_free_native``) must skip the native release when
it sees the parent handle is gone, instead of dereferencing it. Open-after-destroy never reaches
that graceful-degradation guard because there is no live session to orphan.

Runs in its own subprocess (see test_python_runtime.cmake).
"""

import os

from ovphysx.types import ObjectScope, SimObjectType
from test_utils import load_usd_with_ovstage

from ovphysx import PhysX


def _data_path(name):
    tests_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(tests_dir, "data", name)


def test_destroy_during_open_read_session_degrades_gracefully():
    """Destroying the SDK inside an open read ``with`` block must not fault.

    ``destroy()`` proactively releases the open session's stage-bound handles (via
    ``_release_stage_bound_read_handles`` -> ``release_groups``, while the instance handle is still
    live) and only then nulls that handle. So the ``__exit__`` teardown that follows finds the work
    already done: ``release_groups()`` returns at its already-released gate, and ``_free_native()``
    returns on the now-null handle. Neither calls back into the freed instance. Columns
    materialized before the destroy are plain Warp arrays and stay accessible.
    """
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    load_usd_with_ovstage(physx, _data_path("boxes_falling_on_groundplane.usda"))
    physx.warmup()
    physx.wait_all()

    with physx.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        assert result.groups, "session should be live with data before destroy"
        # Destroy the parent WHILE the session is open. This is the case open-after-destroy skips.
        physx.destroy()
        # Already-materialized columns are Warp arrays: attribute access makes no native call and
        # must not fault now that the instance handle is gone.
        touched = [g.tensors for g in result.groups]
        assert touched

    # Exiting the block ran ReadResult.close() against the destroyed parent without dereferencing
    # the freed instance. Reaching here without a fault, with the instance marked released, is the
    # assertion.
    assert physx._released
