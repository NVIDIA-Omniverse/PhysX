# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

#!/usr/bin/env python3
"""
Tensor bindings sample demonstrating simulation data exchange.

This sample demonstrates:
1. Loading a USD scene into an ovstage Stage
2. Creating tensor bindings for data exchange
3. Writing control inputs using tensor API
4. Running extended simulation
5. Reading physics outputs using tensor API
"""

import math
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
        physx.attach_ovstage(stage, read_ordinal=ordinal)
        return stage
    except Exception:
        stage.destroy()
        raise


def main():
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    stage = None
    velocity_target_binding = None
    link_pose_binding = None
    optional_pose_binding = None

    try:
        script_dir = Path(__file__).resolve().parent
        usd_path = script_dir / ".." / "data" / "links_chain_sample.usda"
        if not usd_path.exists():
            raise RuntimeError(f"USD scene not found: {usd_path}")

        print(f"Loading USD scene through ovstage: {usd_path}")
        stage = attach_scene(physx, usd_path, "ovphysx-tensor-bindings-sample")
        physx.wait_all()

        # [tutorial-start]
        print("Creating tensor binding for DOF velocity targets...")
        velocity_target_binding = physx.create_tensor_binding(
            pattern="/World/articulation/articulationLink*",
            tensor_type=TensorType.ARTICULATION_DOF_VELOCITY_TARGET,
        )
        print(f"  DOF count: {velocity_target_binding.shape[1]}")

        print("Creating tensor binding for link poses...")
        link_pose_binding = physx.create_tensor_binding(
            pattern="/World/articulation/articulationLink*",
            tensor_type=TensorType.ARTICULATION_LINK_POSE,
        )
        print(f"  Link count: {link_pose_binding.shape[1]}, Pose dims: {link_pose_binding.shape[2]}")

        optional_pose_binding = physx.create_tensor_binding(
            pattern="/World/optionalRigidBodies/*",
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        if optional_pose_binding.count == 0:
            print("  Optional rigid body pose binding is empty")
        optional_pose_binding.destroy()
        optional_pose_binding = None

        num_dofs = velocity_target_binding.shape[1]
        velocity_targets = np.zeros(velocity_target_binding.shape, dtype=np.float32)
        for i in range(num_dofs):
            velocity_targets[0, i] = 25.0 if i % 2 == 0 else -25.0

        print("Setting DOF velocity targets (alternating +/-25 rad/s)...")
        velocity_target_binding.write(velocity_targets)
        print(f"  Velocity targets: {velocity_targets[0, :5]}... (first 5 DOFs)")

        print("\nRunning 1000 simulation steps...")
        link_poses = np.zeros(link_pose_binding.shape, dtype=np.float32)

        dt = 0.01
        for i in range(1000):
            physx.step(dt)
            physx.wait_all()

            if i % 100 == 0 or i == 999:
                link_pose_binding.read(link_poses)
                px, py, pz = link_poses[0, 0, 0:3]
                qx, qy, qz, qw = link_poses[0, 0, 3:7]
                roll_x_rad = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
                deg_x = roll_x_rad * 180.0 / math.pi
                print(
                    f"  Step {i:4d}: pos=({px:.6f}, {py:.6f}, {pz:.6f}), "
                    f"quat(xyzw)=({qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}), "
                    f"rotation_x={deg_x:.2f} deg"
                )

        print("\nCompleted 1000 simulation steps successfully!")
        # [tutorial-end]
    finally:
        if optional_pose_binding is not None:
            optional_pose_binding.destroy()
        if velocity_target_binding is not None:
            velocity_target_binding.destroy()
        if link_pose_binding is not None:
            link_pose_binding.destroy()
        if stage is not None:
            physx.detach_ovstage()
            stage.destroy()
        physx.release()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
