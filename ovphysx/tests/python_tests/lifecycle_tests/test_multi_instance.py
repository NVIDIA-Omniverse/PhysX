# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Multi-instance tests — ONE release cycle per file.

Carbonite/Python cannot be re-initialized after destroy, so sequential
create/destroy cycling is not supported.  This file validates that multiple
simultaneous instances can coexist and be released together.

Runs in its own subprocess (see test_python.cmake).
"""

from ovphysx import PhysX


def test_multiple_simultaneous_instances():
    """Test creating multiple instances simultaneously and releasing them."""
    NUM_INSTANCES = 3
    instances = []

    for i in range(NUM_INSTANCES):
        physx = PhysX()
        instances.append(physx)

    assert len(instances) == NUM_INSTANCES

    for physx in instances:
        physx.release()
