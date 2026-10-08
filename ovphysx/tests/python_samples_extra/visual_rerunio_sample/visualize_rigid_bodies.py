# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This file is included verbatim in documentation via literalinclude (full file).

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
from ovphysx.types import ObjectScope, SimObjectType

import ovphysx
from ovphysx import PhysX


_physx_schemas_registered = False


def read_rigid_body_poses(sdk):
    """Read every dynamic rigid body's pose as an ``[N, 7]`` array: xyz + quaternion (xyzw).

    PhysX.read emits one group per scene partition per attribute (a position column arrives as
    ``[N, 3]`` and an orientation column as ``[N, 4]``), but it does NOT guarantee the position and
    orientation groups arrive in the same partition order. Pair the two columns per partition by the
    interned ``group.prim_list`` handle before joining: concatenating each attribute independently and
    then ``hstack``-ing would glue a body's position to another partition's quaternion on a multi-scene
    or repartitioned read. Accumulate across partitions so a multi-scene stage renders all of them.
    ``tensor.numpy()`` copies a CUDA column back to the host automatically, so this works on CPU or GPU.
    """
    # prim_list is the interned handle for a partition's prim set. The position and orientation
    # groups of one partition share it, so it is the key that pairs them. dict insertion order keeps
    # the partitions in first-seen order.
    by_partition = {}
    with sdk.read(SimObjectType.RIGID_BODY, ["position", "orientation"], scope=ObjectScope.ALL) as result:
        for group in result.groups:
            # This sample assumes each group's rows are already in prim order. index_map would
            # permute them, and a multi-scene stage that used one would need it honoured per group.
            assert group.index_map is None and group.prim_index_map is None, (
                "read_rigid_body_poses does not handle index_map permutation"
            )
            slot = by_partition.setdefault(group.prim_list, {})
            for tensor in group.tensors:
                column = tensor.numpy()
                if column.shape[1] == 3:  # the position attribute
                    slot["position"] = column
                elif column.shape[1] == 4:  # the orientation quaternion
                    slot["orientation"] = column
    # hstack within a partition pairs row i of position with row i of orientation (same body, same
    # prim_list). Every partition must yield BOTH columns. A missing one is a read fault, so fail
    # loudly rather than drop the partition. A silent drop would hide lost bodies from every caller.
    rows = []
    for slot in by_partition.values():
        assert "position" in slot and "orientation" in slot, (
            "read partition is missing its position or orientation column; cannot build a pose"
        )
        rows.append(np.hstack([slot["position"], slot["orientation"]]))
    if not rows:
        return np.zeros((0, 7), dtype=np.float32)
    return np.concatenate(rows).astype(np.float32, copy=False)


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
    sdk = PhysX()

    script_dir = Path(__file__).resolve().parent
    usd_path = script_dir / ".." / ".." / "data" / "boxes_falling_on_groundplane.usda"
    if not usd_path.exists():
        raise RuntimeError(f"USD scene not found: {usd_path}")

    print(f"Loading USD scene: {usd_path}")
    stage = attach_scene(sdk, usd_path, "ovphysx-rerun-sample")
    sdk.wait_all()

    # PhysX.read streams simulation output. The scene's only dynamic rigid bodies are the cubes
    # (the ground plane and base are static), so reading RIGID_BODY covers exactly them. Warm up
    # first. On a DirectGPU scene the read yields nothing until the first step.
    sdk.warmup()
    sdk.wait_all()
    print(f"Streaming {len(read_rigid_body_poses(sdk))} rigid bodies via PhysX.read")

    # --- Log static geometry ---
    # Rerun does not read USD, so the colliders are logged explicitly for visual context.
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
    dt = 1.0 / 60.0
    num_steps = 300  # 5 seconds at 60 Hz

    print(f"Simulating {num_steps} steps...")
    wall_start = time.monotonic()
    for i in range(num_steps):
        sdk.step(dt)
        sdk.wait_all()

        poses = read_rigid_body_poses(sdk)

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

        # In interactive mode, sleep so wall-clock time does not run ahead of sim time
        if args.interactive:
            sim_time = (i + 1) * dt
            elapsed = time.monotonic() - wall_start
            if elapsed < sim_time:
                time.sleep(sim_time - elapsed)

    print(f"Visualization sample completed successfully ({num_steps} steps)")

    sdk.detach_ovstage()
    stage.destroy()
    sdk.destroy()
    print("Cleanup complete")


if __name__ == "__main__":
    main()
