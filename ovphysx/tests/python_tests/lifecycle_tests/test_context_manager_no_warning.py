# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Regression test for NVBug 6172756: context manager exit must not warn.

Pair with ``test_resource_warning.py``: using ``with PhysX() as physx:`` calls
``release()`` on exit, which sets ``_released = True`` — subsequent GC must
NOT emit a ResourceWarning. This guards against the bug-C fix accidentally
warning on the documented happy path.

Each test file in ``lifecycle_tests/`` runs in its own subprocess (see
test_python.cmake), so this file performs exactly one PhysX() construction.
"""

import gc
import warnings

from ovphysx import PhysX


def test_context_manager_marks_released_no_warning():
    """`with PhysX() as physx:` must release cleanly with no ResourceWarning."""
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", ResourceWarning)
        with PhysX() as physx:
            assert physx._released is False
        assert physx._released is True
        del physx
        gc.collect()

    resource_warnings = [w for w in caught if issubclass(w.category, ResourceWarning)]
    assert not resource_warnings, (
        "Context-manager exit must release cleanly without ResourceWarning; "
        f"got: {[str(w.message) for w in resource_warnings]}"
    )
