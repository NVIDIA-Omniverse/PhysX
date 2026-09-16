# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Identity-keyed process-exit registry: unhashable and equal PhysX subclasses.

Each file in ``lifecycle_tests/`` runs in its own subprocess (see
test_python_runtime.cmake). This file constructs several simultaneous instances and
destroys them before return. It does not cycle create/destroy after shutdown.
"""

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-8

import ovphysx.api as api
from ovphysx import PhysX


class _UnhashablePhysX(PhysX):
    def __eq__(self, other):
        return isinstance(other, PhysX)


class _EqualPhysX(PhysX):
    def __eq__(self, other):
        return isinstance(other, _EqualPhysX)

    def __hash__(self):
        return 0


def _live_tracked_count() -> int:
    with api._PROCESS_LIFECYCLE_INSTANCES_LOCK:
        return sum(1 for tracked in api._PROCESS_LIFECYCLE_INSTANCES.values() if tracked() is not None)


def test_process_exit_registry_tracks_subclasses_by_identity():
    """WeakSet equality would raise or coalesce these. Identity tracking must not."""
    unhashable = None
    first_equal = None
    second_equal = None
    try:
        unhashable = _UnhashablePhysX()
        first_equal = _EqualPhysX()
        second_equal = _EqualPhysX()
        assert first_equal == second_equal
        assert _live_tracked_count() == 3
    finally:
        for instance in (unhashable, first_equal, second_equal):
            if instance is not None:
                instance.destroy()
    assert _live_tracked_count() == 0
