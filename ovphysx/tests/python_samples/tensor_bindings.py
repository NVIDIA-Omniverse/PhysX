# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# NOTE: this sample demonstrates the deprecated tensor-binding API. New code should use the
# session read/write API (PhysX.read / PhysX.write).

# @implements REQ-PYTHON-SAMPLE-001
# @covers AC-1 AC-2

# NOTE: This file is included verbatim in documentation via literalinclude.

#!/usr/bin/env python3
"""
Tensor bindings sample demonstrating simulation data exchange.

.. deprecated:: 0.6.0
    The tensor-binding API shown here is deprecated in favor of the session read/write
    API (``PhysX.read`` / ``PhysX.write``). This sample is retained as the deprecated-API
    example and is removed with the API.

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

import ovphysx
from ovphysx import PhysX
from ovphysx.types import TensorType


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
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    stage = None
    velocity_target_binding = None
    link_pose_binding = None
    optional_pose_binding = None

    try:
        # Prefer package data so a copied sample works. Fall back to the checked-in
        # sample's adjacent data directory when package data is absent.
        usd_path = (
            Path(ovphysx.__file__).resolve().parent
            / "samples"
            / "data"
            / "links_chain_sample.usda"
        )
        if not usd_path.is_file():
            usd_path = Path(__file__).resolve().parent.parent / "data" / "links_chain_sample.usda"
        if not usd_path.is_file():
            raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

        print(f"Loading USD scene through ovstage: {usd_path}")
        stage = attach_scene(physx, usd_path, "ovphysx-tensor-bindings-sample")
        physx.wait_all()

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
        link_count = link_pose_binding.shape[1]
        if link_count < 2:
            raise RuntimeError(f"Fixture must expose more than one articulation link, got {link_count}")

        # Link 0 is the fixed root. This linear fixture's last link is its moving chain tip.
        link_index_to_print = link_count - 1
        initial_printed_position = None
        max_abs_position_delta = 0.0
        motion_tolerance = 1.0e-3

        dt = 0.01
        for i in range(1000):
            physx.step(dt)
            physx.wait_all()

            if i % 100 == 0 or i == 999:
                link_pose_binding.read(link_poses)
                displayed_pose = link_poses[0, link_index_to_print]
                if not np.all(np.isfinite(displayed_pose)):
                    raise RuntimeError(
                        f"Selected link {link_index_to_print} has a non-finite pose at step {i}: {displayed_pose}"
                    )

                position = displayed_pose[0:3]
                if initial_printed_position is None:
                    initial_printed_position = position.copy()
                else:
                    # This checks that the displayed link is not static. It does not
                    # require motion after the chain settles.
                    position_delta = float(np.max(np.abs(position - initial_printed_position)))
                    max_abs_position_delta = max(max_abs_position_delta, position_delta)

                px, py, pz = position
                qx, qy, qz, qw = displayed_pose[3:7]
                roll_x_rad = math.atan2(
                    2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)
                )
                deg_x = roll_x_rad * 180.0 / math.pi

                print(
                    f"  Step {i:4d}, link {link_index_to_print}: "
                    f"pos=({px:.6f}, {py:.6f}, {pz:.6f}), "
                    f"quat(xyzw)=({qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}), "
                    f"rotation_x={deg_x:.2f} deg"
                )

        if max_abs_position_delta <= motion_tolerance:
            raise RuntimeError(
                f"Printed link {link_index_to_print} max position delta {max_abs_position_delta:.6f} "
                f"did not exceed motion tolerance {motion_tolerance:.6f}"
            )

        print(
            f"\nCompleted 1000 simulation steps successfully! "
            f"Link {link_index_to_print} max position delta: {max_abs_position_delta:.6f}"
        )
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
        physx.destroy()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
