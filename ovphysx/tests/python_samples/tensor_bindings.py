# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# NOTE: This file is included verbatim in documentation.
# When editing, keep tutorial line ranges in sync.


#!/usr/bin/env python3
"""
Tensor bindings sample demonstrating simulation data exchange.

This sample demonstrates:
1. Loading a USD scene with physics objects
2. Creating tensor bindings for data exchange
3. Writing control inputs (joint drive velocity) using tensor API
4. Running extended simulation (1000 steps)
5. Reading physics outputs (link pose) using tensor API
"""

import math
from pathlib import Path

import numpy as np

from ovphysx import (
    OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32,
    OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32,
    PhysX,
)


def main():
    # Initialize PhysX SDK
    physx = PhysX(device="cpu")

    # Load USD scene with physics setup
    script_dir = Path(__file__).resolve().parent
    usd_path = script_dir / ".." / "data" / "links_chain_sample.usda"

    if not usd_path.exists():
        raise RuntimeError(f"USD scene not found: {usd_path}")

    print(f"Loading USD scene: {usd_path}")
    usd_handle, _ = physx.add_usd(str(usd_path))
    physx.wait_all()

    # Create tensor bindings for DOF velocity targets (control inputs)
    # Pattern matches all articulation links - the API will automatically map to DOFs
    print("Creating tensor binding for DOF velocity targets...")
    velocity_target_binding = physx.create_tensor_binding(
        pattern="/World/articulation/articulationLink*",
        tensor_type=OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32,
    )
    print(f"  DOF count: {velocity_target_binding.shape[1]}")

    # Create tensor binding for link poses (state observation)
    print("Creating tensor binding for link poses...")
    link_pose_binding = physx.create_tensor_binding(
        pattern="/World/articulation/articulationLink*",
        tensor_type=OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32,
    )
    print(f"  Link count: {link_pose_binding.shape[1]}, Pose dims: {link_pose_binding.shape[2]}")

    # Set joint drive velocities (alternating +25 and -25 rad/s)
    num_dofs = velocity_target_binding.shape[1]
    velocity_targets = np.zeros(velocity_target_binding.shape, dtype=np.float32)
    for i in range(num_dofs):
        velocity_targets[0, i] = 25.0 if i % 2 == 0 else -25.0

    print("Setting DOF velocity targets (alternating ±25 rad/s)...")
    velocity_target_binding.write(velocity_targets)
    print(f"  Velocity targets: {velocity_targets[0, :5]}... (first 5 DOFs)")

    # Run extended simulation with periodic readback
    print("\nRunning 1000 simulation steps...")

    # Allocate buffer for reading link poses
    link_poses = np.zeros(link_pose_binding.shape, dtype=np.float32)

    dt = 0.01
    for i in range(1000):
        # Step simulation forward
        current_time = i * dt
        physx.step(dt, current_time)
        physx.wait_all()

        # Read link poses periodically (tensor API is synchronous, no wait needed)
        if i % 100 == 0 or i == 999:
            link_pose_binding.read(link_poses)

            # Extract first link's pose (position + quaternion)
            # Pose format: [px, py, pz, qx, qy, qz, qw]
            px, py, pz = link_poses[0, 0, 0:3]
            qx, qy, qz, qw = link_poses[0, 0, 3:7]

            # Compute X-axis rotation angle (roll) from quaternion
            roll_x_rad = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
            deg_x = roll_x_rad * 180.0 / math.pi

            print(
                f"  Step {i:4d}: pos=({px:.6f}, {py:.6f}, {pz:.6f}), "
                f"quat(xyzw)=({qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}), "
                f"rotation_x={deg_x:.2f}°"
            )

    print("\nCompleted 1000 simulation steps successfully!")

    # Clean up tensor bindings
    velocity_target_binding.destroy()
    link_pose_binding.destroy()

    # Clean up PhysX
    physx.remove_usd(usd_handle)
    physx.release()

    print("\nDone.")


if __name__ == "__main__":
    main()
