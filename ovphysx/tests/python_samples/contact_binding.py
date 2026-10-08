# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This file is included verbatim in documentation via literalinclude.
# Markers [tutorial-*] define the included ranges.
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

import ovphysx
from ovphysx import PhysX


_physx_schemas_registered = False


def attach_scene(physx, usd_path, stage_name):
    import ovstage

    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    # ovphysx ships its PhysX USD schemas as codeless resources and does not register
    # them itself. Register them with ovstage once, before the first population
    # call in the process.
    global _physx_schemas_registered
    if not _physx_schemas_registered:
        ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
        _physx_schemas_registered = True
    stage = ovstage.Stage(stage_name)
    ordinal = 1
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=ordinal, domains=ovstage.PopulationDomain.PHYSICS)
        stage.advance_write_floor(ordinal=ordinal).wait()
        physx.attach_ovstage(stage, read_ordinal=ordinal)
        return stage
    except Exception:
        stage.destroy()
        raise


def main():
    # [tutorial-start]

    # --- 1. Initialize SDK and load scene ---
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    stage = None
    data_dir = Path(__file__).resolve().parent.parent / "data"

    try:
        stage = attach_scene(physx, data_dir / "boxes_falling_on_groundplane.usda", "ovphysx-contact-sample")
        physx.wait_all()

        # --- 2. Create a contact binding BEFORE the first step ---
        # sensor_patterns: bodies whose contact forces you want to read.
        # filter_patterns: bodies to measure contacts against (one per sensor).
        # Cube1 lands on the static collider /World/BigBase, not the ground
        # plane, so the filter must name BigBase or the force matrix is zeros
        # while net_forces is not.
        # The binding must be created before any step() call whose contacts you
        # want to observe.
        cb = physx.create_contact_binding(
            sensor_patterns=["/World/Cube1"],
            filter_patterns=["/World/BigBase"],
            filters_per_sensor=1,
            max_contact_data_count=256,
        )

        sensor_count = cb.sensor_count   # number of matched sensor prims
        filter_count = cb.filter_count   # number of filter prims per sensor

        print(f"Sensors: {sensor_count}, filters per sensor: {filter_count}")

        # --- 3. Simulate until boxes land ---
        for _ in range(120):
            physx.step(1.0 / 60.0)
        physx.wait_all()

        # --- 4. Read net contact forces: shape [S, 3] ---
        # dt is taken automatically from the last successful stepping call.
        net_forces = np.zeros((sensor_count, 3), dtype=np.float32)
        cb.read_net_forces(net_forces)
        print("Net contact forces [S, 3]:", net_forces)

        # --- 5. Read contact force matrix: shape [S, F, 3] ---
        force_matrix = np.zeros((sensor_count, filter_count, 3), dtype=np.float32)
        cb.read_force_matrix(force_matrix)
        print("Contact force matrix [S, F, 3]:", force_matrix)
        # 1x1 matrix equals net force here (Cube1 vs BigBase). Fail loudly if
        # the filter again names a body Cube1 is not touching.
        assert np.linalg.norm(force_matrix) > 1.0, (
            "expected Cube1 vs BigBase contact after 120 steps; "
            f"force matrix was {force_matrix}"
        )

        # --- 6. Clean up first demo ---
        cb.destroy()

        # [tutorial-end]

        # Context-manager usage (alternative to manual destroy):
        # Reset the stage so the same PhysX instance can be reused.
        physx.reset_stage()
        physx.wait_all()
        physx.detach_ovstage()
        stage.destroy()
        stage = None
        stage = attach_scene(physx, data_dir / "boxes_falling_on_groundplane.usda", "ovphysx-contact-sample-reload")
        physx.wait_all()

        # [tutorial-context-manager]
        with physx.create_contact_binding(sensor_patterns=["/World/Cube1"]) as cb2:
            for _ in range(60):
                physx.step(1.0 / 60.0)
            physx.wait_all()
            out = np.zeros((cb2.sensor_count, 3), dtype=np.float32)
            cb2.read_net_forces(out)
            print("Net forces (context manager):", out)
        # cb2 is automatically destroyed here
        # [tutorial-context-manager-end]

        print("Contact binding sample completed successfully")
    finally:
        if stage is not None:
            physx.detach_ovstage()
            stage.destroy()
        physx.destroy()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
