# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

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

from ovphysx import PhysX
from ovphysx.types import TensorType


def attach_scene(physx, usd_path, stage_name):
    import ovstage

    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    stage = ovstage.Stage(stage_name)
    ordinal = 1
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=ordinal, domains=ovstage.PopulationDomain.PHYSICS)
        # Population does not seal: the caller owns ordinal lifecycle, and
        # attach_ovstage() reads at a sealed ordinal.
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
    pose_binding = None

    try:
        # Load USD scene containing /World/envs/env0
        script_dir = Path(__file__).resolve().parent
        usd_path = script_dir / ".." / "data" / "basic_simulation.usda"

        print(f"Loading USD scene through ovstage: {usd_path}")
        stage = attach_scene(physx, usd_path, "ovphysx-clone-sample")
        physx.wait_all()

        # Clone env0 to create env1, env2, env3
        targets = ["/World/envs/env1", "/World/envs/env2", "/World/envs/env3"]
        print(f"Cloning /World/envs/env0 to {len(targets)} targets...")
        physx.clone("/World/envs/env0", targets)
        physx.wait_all()
        print(f"  Created {len(targets)} clones successfully")

        # Create a tensor binding to read rigid body poses across all environments
        # The pattern matches the "table" rigid body in every cloned environment
        pose_binding = physx.create_tensor_binding(
            pattern="/World/envs/env*/table",
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        print(f"  Rigid body binding: count={pose_binding.count}, shape={pose_binding.shape}")

        # Run simulation with all environments
        print("Running 10 simulation steps...")
        dt = 1.0 / 60.0
        for i in range(10):
            physx.step(dt)
        physx.wait_all()
        print("  All steps completed")

        # Read poses from all environments after simulation
        poses = np.zeros(pose_binding.shape, dtype=np.float32)
        pose_binding.read(poses)
        for env_idx in range(poses.shape[0]):
            px, py, pz = poses[env_idx, 0:3]
            print(f"  env{env_idx}: pos=({px:.4f}, {py:.4f}, {pz:.4f})")

        print("Clone sample completed successfully")

    finally:
        if pose_binding is not None:
            pose_binding.destroy()
        if stage is not None:
            physx.detach_ovstage()
            stage.destroy()
        physx.release()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
