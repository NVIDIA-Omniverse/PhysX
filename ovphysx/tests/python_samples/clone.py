# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-CLONE-001
# @covers AC-1

# NOTE: This file is included verbatim in documentation via literalinclude.

"""
Clone sample demonstrating scene replication with the clone API.

This sample demonstrates:
1. Loading a USD scene with an environment hierarchy
2. Cloning the environment to create multiple copies
3. Running simulation with all clones
"""

from pathlib import Path

import numpy as np

import ovphysx
from ovphysx import PhysX
from ovphysx.types import ObjectScope, SimObjectType


def _to_host(column):
    """A read column as host NumPy, whether it came back as NumPy (CPU) or a Warp array (GPU)."""
    return column if isinstance(column, np.ndarray) else column.numpy()


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
    # Initialize PhysX SDK
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    stage = None

    try:
        # Prefer package data so a copied sample works. Fall back to the checked-in
        # sample's adjacent data directory when package data is absent.
        usd_path = (
            Path(ovphysx.__file__).resolve().parent
            / "samples"
            / "data"
            / "basic_simulation.usda"
        )
        if not usd_path.is_file():
            usd_path = Path(__file__).resolve().parent.parent / "data" / "basic_simulation.usda"
        if not usd_path.is_file():
            raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

        print(f"Loading USD scene through ovstage: {usd_path}")
        stage = attach_scene(physx, usd_path, "ovphysx-clone-sample")
        physx.wait_all()

        # Clone env0 to create env1, env2, env3
        targets = ["/World/envs/env1", "/World/envs/env2", "/World/envs/env3"]
        anchor_transforms = [
            (4.0 * env_idx, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            for env_idx in range(1, len(targets) + 1)
        ]
        print(f"Cloning /World/envs/env0 to {len(targets)} targets...")
        # CPU mode has no environment-id collision filtering, so place each
        # environment in a spatially disjoint lane.
        physx.clone("/World/envs/env0", targets, anchor_transforms=anchor_transforms)
        physx.wait_all()
        print(f"  Created {len(targets)} clones successfully")

        # Run simulation with all environments
        print("Running 10 simulation steps...")
        dt = 1.0 / 60.0
        for i in range(10):
            physx.step(dt)
        physx.wait_all()
        print("  All steps completed")

        # Read rigid-body positions across all environments via the session read API. After
        # cloning, the tables are the scene's only rigid bodies, so reading RIGID_BODY covers
        # exactly them, one row per environment.
        with physx.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
            positions = np.concatenate(
                [_to_host(group.tensors[0]).reshape(group.prim_count, -1) for group in result.groups]
            )
        for env_idx in range(positions.shape[0]):
            px, py, pz = positions[env_idx, 0:3]
            print(f"  env{env_idx}: pos=({px:.4f}, {py:.4f}, {pz:.4f})")

        print("Clone sample completed successfully")

    finally:
        if stage is not None:
            physx.detach_ovstage()
            stage.destroy()
        physx.destroy()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
