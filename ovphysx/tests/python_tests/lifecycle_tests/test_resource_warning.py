# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Regression test for NVBug 6172756: ResourceWarning on mid-run GC.

Scope: this file covers the *mid-run* leg of the fix — a ``PhysX()`` created
without a context manager and without an explicit ``release()`` whose
references go away **before** interpreter shutdown. The ``del physx;
gc.collect()`` below drops the refcount to zero and runs ``PhysX.__del__``
synchronously inside the test body. ``__del__`` emits the ResourceWarning
**and then calls ``self.release()`` itself**, so by the time the test
returns the native instance is fully torn down. The module-level
The Python API does not register a process-exit cleanup hook. Applications
should use ``with PhysX() as physx:`` or call ``release()`` explicitly;
mid-run garbage collection remains a warning-and-release fallback.

Each test file in ``lifecycle_tests/`` runs in its own subprocess (see
test_python.cmake), so this file performs exactly one PhysX() construction.
"""

import gc
import warnings

from ovphysx import PhysX


def test_resource_warning_emitted_on_unreleased_gc():
    """Garbage-collecting an unreleased PhysX must emit ResourceWarning."""
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", ResourceWarning)
        physx = PhysX()
        assert physx._released is False
        # Do NOT call release(); do NOT use the context manager.
        del physx
        gc.collect()

    resource_warnings = [w for w in caught if issubclass(w.category, ResourceWarning)]
    assert resource_warnings, (
        "Expected a ResourceWarning when PhysX is GC'd without release(); "
        f"got: {[(w.category.__name__, str(w.message)) for w in caught]}"
    )
    msg = str(resource_warnings[0].message).lower()
    assert "release" in msg, (
        f"ResourceWarning message must mention release(); got: {msg!r}"
    )
