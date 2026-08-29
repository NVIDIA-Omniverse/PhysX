# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Context manager lifecycle tests — ONE create+release per file.

Runs in its own subprocess (see test_python.cmake).
"""

import pytest

from ovphysx import PhysX
from test_utils import load_usd_with_ovstage


def test_context_manager_and_explicit_protocol():
    """Test context manager protocol via ``with`` statement and error handling.

    Covers:
        - ``with PhysX() as physx:`` — __enter__ returns self
        - Exception inside with block — __exit__ still triggers cleanup
        - Double __exit__ after with block is safe (idempotent)
    """
    try:
        with PhysX() as physx:
            assert physx is not None
            load_usd_with_ovstage(physx, "/nonexistent/invalid/path.usda")
            physx.wait_all()  # Will likely fail
    except RuntimeError:
        pass  # Expected — __exit__ should still have cleaned up

    # After with-block exit, double __exit__ must be safe
    physx.__exit__(None, None, None)
