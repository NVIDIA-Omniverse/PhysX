# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# NOTE: This file is included verbatim in documentation.
# When editing, keep tutorial line ranges in sync.
"""
ContactBinding sample: reading contact forces between sensor and filter bodies.

This sample demonstrates:
1. Creating a contact binding before the first simulation step
2. Reading per-sensor net contact forces  [S, 3]
3. Reading a sensor x filter force matrix [S, F, 3]
4. Using the context-manager form to ensure proper cleanup
"""

import numpy as np
from pathlib import Path

from ovphysx import PhysX


def main():
    # [tutorial-start]

    # --- 1. Initialize SDK and load scene ---
    physx = PhysX(device="cpu")
    data_dir = Path(__file__).resolve().parent.parent / "data"
    physx.add_usd(str(data_dir / "boxes_falling_on_groundplane.usda"))
    physx.wait_all()

    # --- 2. Create a contact binding BEFORE the first step ---
    # sensor_patterns: bodies whose contact forces you want to read.
    # filter_patterns: bodies to measure contacts against (one per sensor).
    # The binding must be created before any step() call whose contacts you
    # want to observe.
    cb = physx.create_contact_binding(
        sensor_patterns=["/World/Cube1"],
        filter_patterns=["/World/GroundPlane/CollisionMesh"],
        filters_per_sensor=1,
        max_contact_data_count=256,
    )

    sensor_count = cb.sensor_count   # number of matched sensor prims
    filter_count = cb.filter_count   # number of filter prims per sensor

    print(f"Sensors: {sensor_count}, filters per sensor: {filter_count}")

    # --- 3. Simulate until boxes land ---
    for _ in range(120):
        physx.step(1.0 / 60.0, 0.0)
    physx.wait_all()

    # --- 4. Read net contact forces: shape [S, 3] ---
    # dt is taken automatically from the last step() call.
    net_forces = np.zeros((sensor_count, 3), dtype=np.float32)
    cb.read_net_forces(net_forces)
    print("Net contact forces [S, 3]:", net_forces)

    # --- 5. Read contact force matrix: shape [S, F, 3] ---
    force_matrix = np.zeros((sensor_count, filter_count, 3), dtype=np.float32)
    cb.read_force_matrix(force_matrix)
    print("Contact force matrix [S, F, 3]:", force_matrix)

    # --- 6. Clean up first demo ---
    cb.destroy()

    # [tutorial-end]

    # Context-manager usage (alternative to manual destroy):
    # Reset the stage so we can reuse the same PhysX instance.
    physx.reset()
    physx.wait_all()

    physx.add_usd(str(data_dir / "boxes_falling_on_groundplane.usda"))
    physx.wait_all()

    # [tutorial-context-manager]
    with physx.create_contact_binding(sensor_patterns=["/World/Cube1"]) as cb2:
        for _ in range(60):
            physx.step(1.0 / 60.0, 0.0)
        physx.wait_all()
        out = np.zeros((cb2.sensor_count, 3), dtype=np.float32)
        cb2.read_net_forces(out)
        print("Net forces (context manager):", out)
    # cb2 is automatically destroyed here
    # [tutorial-context-manager-end]

    physx.release()

    print("[SUCCESS]", flush=True)


if __name__ == "__main__":
    main()
