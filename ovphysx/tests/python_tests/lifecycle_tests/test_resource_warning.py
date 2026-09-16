# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression test for NVBug 6172756: ResourceWarning on mid-run GC.

Scope: this file covers the *mid-run* leg of the fix: a ``PhysX()`` created
without an explicit ``destroy()`` whose references go away **before**
interpreter shutdown. The ``del physx;
gc.collect()`` below drops the refcount to zero and runs ``PhysX.__del__``
synchronously inside the test body. ``__del__`` emits the ResourceWarning
**and then calls ``self.destroy()`` itself**, so by the time the test
returns the native instance is fully torn down.

The Python process-exit registry uses weak references, so it does not keep
this otherwise unreachable instance alive. Applications should still call
``destroy()`` explicitly in a ``finally`` block; mid-run garbage collection
remains a warning-and-destroy fallback.

Each test file in ``lifecycle_tests/`` runs in its own subprocess (see
test_python_runtime.cmake), so this file performs exactly one PhysX() construction.
"""

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-5 AC-8

import gc
import warnings

from ovphysx import PhysX


def test_resource_warning_emitted_on_unreleased_gc():
    """Garbage-collecting an unreleased PhysX must emit ResourceWarning."""
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", ResourceWarning)
        physx = PhysX()
        assert physx._released is False
        # Do NOT call destroy().
        del physx
        gc.collect()

    resource_warnings = [w for w in caught if issubclass(w.category, ResourceWarning)]
    assert resource_warnings, (
        "Expected a ResourceWarning when PhysX is GC'd without destroy(); "
        f"got: {[(w.category.__name__, str(w.message)) for w in caught]}"
    )
    msg = str(resource_warnings[0].message).lower()
    assert "destroy" in msg, (
        f"ResourceWarning message must mention destroy(); got: {msg!r}"
    )
