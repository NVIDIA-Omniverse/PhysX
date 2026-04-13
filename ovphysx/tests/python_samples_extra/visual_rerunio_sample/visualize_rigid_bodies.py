# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# NOTE: This file is included verbatim in documentation.
# When editing, keep tutorial line ranges in sync.

"""
Visualize rigid body simulation with Rerun.

Shows 11 cubes falling onto a ground plane, streaming rigid body transforms
to a Rerun web viewer. Open http://localhost:9090 in your browser to watch.

This demonstrates the "render handoff" pattern: ovphysx owns simulation,
you log what you need to your visualization tool. Rerun does not read USD,
so the sample explicitly logs all visual geometry from simulation data.
"""

import argparse
import time
from pathlib import Path
from urllib.parse import quote

import numpy as np
import rerun as rr

from ovphysx import PhysX
from ovphysx.types import TensorType


def main():
    parser = argparse.ArgumentParser(description="Visualize rigid body simulation with Rerun")
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Pause before simulation and run at real-time speed for live viewing",
    )
    args = parser.parse_args()

    # --- Rerun setup ---
    rr.init("ovphysx_rigid_bodies")
    server_uri = rr.serve_grpc()
    rr.serve_web_viewer(connect_to=server_uri, open_browser=False)
    viewer_url = f"http://localhost:9090/?url={quote(server_uri)}"
    print(f"Rerun web viewer: {viewer_url}")

    if args.interactive:
        print("Please open the web viewer in your browser. Pausing for 5 seconds...")
        time.sleep(5)

    # Set coordinate system: the USD scene is Z-up, meters
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # --- ovphysx setup ---
    sdk = PhysX(device="cpu")

    script_dir = Path(__file__).resolve().parent
    usd_path = script_dir / ".." / ".." / "data" / "boxes_falling_on_groundplane.usda"
    if not usd_path.exists():
        raise RuntimeError(f"USD scene not found: {usd_path}")

    print(f"Loading USD scene: {usd_path}")
    usd_handle, _ = sdk.add_usd(str(usd_path))
    sdk.wait_all()

    # Create tensor binding for rigid body poses: [N, 7] = [px, py, pz, qx, qy, qz, qw]
    pose_binding = sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    num_cubes = pose_binding.count
    print(f"Bound {num_cubes} rigid bodies, shape={pose_binding.shape}")

    # --- Log static geometry ---
    # Rerun doesn't read USD, so we log colliders explicitly for visual context.
    # Ground plane: collision plane at Z=0 with a 50x50 m mesh.
    # BigBase: static box collider the cubes land on (from the USD scene).
    rr.log(
        "world/ground",
        rr.Boxes3D(
            centers=[[0.0, 0.0, -0.025]],
            half_sizes=[[25.0, 25.0, 0.025]],
            colors=[[180, 180, 180]],
        ),
        static=True,
    )
    rr.log(
        "world/big_base",
        rr.Boxes3D(
            centers=[[7.725, -5.182, 4.075]],
            half_sizes=[[17.201, 6.087, 3.896]],
            colors=[[140, 140, 160]],
        ),
        static=True,
    )

    # --- Simulate and stream ---
    poses = np.zeros(pose_binding.shape, dtype=np.float32)
    dt = 1.0 / 60.0
    num_steps = 300  # 5 seconds at 60 Hz

    print(f"Simulating {num_steps} steps...")
    wall_start = time.monotonic()
    for i in range(num_steps):
        sdk.step(dt, i * dt)
        sdk.wait_all()

        pose_binding.read(poses)

        rr.set_time("step", sequence=i)
        rr.set_time("sim_time", duration=i * dt)

        # Log cubes: positions from columns 0:3, quaternions (xyzw) from columns 3:7.
        # .copy() ensures contiguous arrays for Rerun's Arrow serialization.
        rr.log(
            "world/cubes",
            rr.Boxes3D(
                centers=poses[:, 0:3].copy(),
                half_sizes=[0.5, 0.5, 0.5],
                quaternions=rr.Quaternion(xyzw=poses[:, 3:7].copy()),
            ),
        )

        # In interactive mode, sleep so wall-clock time doesn't run ahead of sim time
        if args.interactive:
            sim_time = (i + 1) * dt
            elapsed = time.monotonic() - wall_start
            if elapsed < sim_time:
                time.sleep(sim_time - elapsed)

    pose_binding.destroy()
    sdk.remove_usd(usd_handle)
    sdk.release()

    print("[SUCCESS]", flush=True)


if __name__ == "__main__":
    main()
