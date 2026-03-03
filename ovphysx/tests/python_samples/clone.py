# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# NOTE: This file is included verbatim in documentation.
# When editing, keep tutorial line ranges in sync.

"""
Clone sample demonstrating scene replication with the clone API.

This sample demonstrates:
1. Loading a USD scene with an environment hierarchy
2. Cloning the environment to create multiple copies
3. Running simulation with all clones
"""

from pathlib import Path

import numpy as np

from ovphysx import OVPHYSX_TENSOR_RIGID_BODY_POSE_F32, PhysX


def main():
    # Initialize PhysX SDK
    physx = PhysX(device="cpu")

    # Load USD scene containing /World/envs/env0
    script_dir = Path(__file__).resolve().parent
    usd_path = script_dir / ".." / "data" / "basic_simulation.usda"

    print(f"Loading USD scene: {usd_path}")
    usd_handle, _ = physx.add_usd(str(usd_path))
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
        tensor_type=OVPHYSX_TENSOR_RIGID_BODY_POSE_F32,
    )
    print(f"  Rigid body binding: count={pose_binding.count}, shape={pose_binding.shape}")

    # Run simulation with all environments
    print("Running 10 simulation steps...")
    dt = 1.0 / 60.0
    for i in range(10):
        physx.step(dt, i * dt)
    physx.wait_all()
    print("  All steps completed")

    # Read poses from all environments after simulation
    poses = np.zeros(pose_binding.shape, dtype=np.float32)
    pose_binding.read(poses)
    for env_idx in range(poses.shape[0]):
        px, py, pz = poses[env_idx, 0:3]
        print(f"  env{env_idx}: pos=({px:.4f}, {py:.4f}, {pz:.4f})")

    # Clean up
    pose_binding.destroy()
    physx.remove_usd(usd_handle)
    physx.release()

    print("Done.")


if __name__ == "__main__":
    main()
