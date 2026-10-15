# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This file is included verbatim in documentation via literalinclude.
# Markers [tutorial-*] define the included ranges.
"""
ContactBinding sample: reading contact forces between sensor and filter bodies.

This sample exercises the deprecated contact-binding API during its compatibility period.

This sample demonstrates:
1. Creating a contact binding before the first simulation step
2. Reading net normal and friction forces per sensor [S, 3] and adding them
3. Reading normal and friction force matrices [S, F, 3] and adding them
4. Using the context-manager form to ensure proper cleanup
"""

# @implements REQ-CAPI-OVSTAGE-XFORM-001
# @covers AC-1
# @implements REQ-PYTHON-SAMPLE-002
# @covers AC-1 AC-2

import numpy as np
from pathlib import Path

import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx import PhysX


_physx_schemas_registered = False


def attach_scene(physx, usd_path, stage_name):
    import ovstage

    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    # Register external PhysX and Newton schemas before the first population.
    global _physx_schemas_registered
    if not _physx_schemas_registered:
        ovstage.population.register_usd_schemas(
            [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
        )
        _physx_schemas_registered = True
    stage = ovstage.Stage(stage_name)
    ordinal = 1
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=ordinal, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=ordinal).wait()
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=ordinal))
        return stage
    except Exception:
        if physx.is_stage_attached(stage):
            physx.wait_task(physx.detach_ovstage())
        stage.destroy()
        raise


def main():
    # [tutorial-start]

    # --- 1. Initialize SDK and load scene ---
    # Prefer package data so a copied sample works. Fall back to the checked-in
    # sample's adjacent data directory when package data is absent. Resolve it
    # before creating an instance so a missing scene does not skip destroy().
    scene_name = "boxes_falling_on_groundplane.usda"
    usd_path = Path(ovphysx.__file__).resolve().parent / "samples" / "data" / scene_name
    if not usd_path.is_file():
        usd_path = Path(__file__).resolve().parent.parent / "data" / scene_name
    if not usd_path.is_file():
        raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")
    data_dir = usd_path.parent

    PhysX.set_cpu_mode(True)
    physx = PhysX()
    stage = None

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

        # --- 4. Read net normal and friction forces: shape [S, 3] ---
        # dt is taken automatically from the last successful stepping call.
        # Read both components from the same step before adding them.
        net_normal_forces = np.zeros((sensor_count, 3), dtype=np.float32)
        net_friction_forces = np.zeros_like(net_normal_forces)
        cb.read_net_normal_forces(net_normal_forces)
        cb.read_net_friction_forces(net_friction_forces)
        net_forces = net_normal_forces + net_friction_forces
        print("Net normal contact forces [S, 3]:", net_normal_forces)
        print("Net friction contact forces [S, 3]:", net_friction_forces)
        print("Total net contact forces [S, 3]:", net_forces)

        # --- 5. Read normal and friction force matrices: shape [S, F, 3] ---
        normal_force_matrix = np.zeros((sensor_count, filter_count, 3), dtype=np.float32)
        friction_force_matrix = np.zeros_like(normal_force_matrix)
        cb.read_normal_force_matrix(normal_force_matrix)
        cb.read_friction_force_matrix(friction_force_matrix)
        force_matrix = normal_force_matrix + friction_force_matrix
        print("Normal contact force matrix [S, F, 3]:", normal_force_matrix)
        print("Friction contact force matrix [S, F, 3]:", friction_force_matrix)
        print("Total contact force matrix [S, F, 3]:", force_matrix)
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
        # Detach the stage so the same PhysX instance can be reused.
        physx.wait_task(physx.detach_ovstage())
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
            cb2.read_net_normal_forces(out)
            print("Net normal contact forces (context manager):", out)
        # cb2 is automatically destroyed here
        # [tutorial-context-manager-end]

        print("Contact binding sample completed successfully")
    finally:
        if stage is not None:
            physx.wait_task(physx.detach_ovstage())
            stage.destroy()
        physx.destroy()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
