# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression test for NVBug 6172756: explicit destroy must not warn."""

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-5

import gc
import warnings

from ovphysx import PhysX


def test_destroy_marks_released_no_warning():
    """Garbage collection after explicit destroy emits no ResourceWarning."""
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", ResourceWarning)
        physx = PhysX()
        physx.destroy()
        del physx
        gc.collect()

    assert not any(issubclass(item.category, ResourceWarning) for item in caught)
