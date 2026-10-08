# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Multi-instance coexistence regression, ONE destroy cycle per file.

Carbonite/Python cannot be re-initialized after destroy, so sequential
create/destroy cycling is not supported. This file validates that multiple
simultaneous Python instances can coexist and be destroyed together.

Runs in its own subprocess (see scripts/test_python_runtime.cmake).
"""

from contextlib import ExitStack

from ovphysx import PhysX


def test_multiple_simultaneous_instances():
    """Create three simultaneous instances and destroy them together."""
    num_instances = 3
    instances = []

    with ExitStack() as cleanup:
        for _ in range(num_instances):
            physx = PhysX()
            cleanup.callback(physx.destroy)
            instances.append(physx)

        assert len(instances) == num_instances
