# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

"""Closed-loop ovstage sample: drive control into ovphysx, read all output back.

This sample demonstrates the full ovstage round trip (ADR-0007):

1. Attach an ovstage Stage parsed from USD.
2. Each frame, author a *control* edit -- an alternating
   ``physics:velocity`` on every rigid body -- into ovstage, seal the write, and
   drain it into the running simulation with ``update_from_ovstage``.
3. Verify the velocity changed, step the simulation, and verify the expected
   sideways displacement.
4. Read the step *output* for every simulated object type via ``read`` and write
   each column straight back into ovstage.

The read is faithful to ovstage: each group carries the interned ``prim_list``
handle, so the write-back reuses it directly (``stage.query_from_path_list(
group.prim_list)``) with no path-string rebuild and no repack. ``read`` is a
context manager that keeps those handles valid for the ``with`` block.

The key invariant (see the ovstage Integration guide): control edits flow
through ``update_from_ovstage``; physics output is written back at ordinals that
``update_from_ovstage`` never covers, so physics never re-ingests its own output.
We interleave two ordinal lanes for that: an even "control" lane (drained) and an
odd "output" lane (never drained).
"""

from pathlib import Path

import numpy as np

import ovstage
from ovphysx import PhysX
from ovphysx.types import SimObjectType, ObjectScope

# The output attributes each simulated type can produce (see the ovstage
# Integration guide). read silently skips names a type does not emit, and a type
# with no objects in the scene simply yields no groups -- so iterating the full
# matrix is safe on any scene.
OUTPUT_ATTRIBUTES = {
    SimObjectType.RIGID_BODY: ["position", "orientation", "linearVelocity", "angularVelocity"],
    SimObjectType.ARTICULATION_LINK: ["position", "orientation", "linearVelocity", "angularVelocity"],
    SimObjectType.ARTICULATION_JOINT: ["jointPosition", "jointVelocity"],
    SimObjectType.VEHICLE_WHEEL: ["position", "orientation"],
    SimObjectType.DEFORMABLE_VOLUME: ["points", "velocities"],
    SimObjectType.DEFORMABLE_SURFACE: ["points", "velocities"],
    SimObjectType.PARTICLE_SET: ["points", "velocities"],
}


def attach_scene(physx, usd_path, read_ordinal):
    """Populate an ovstage Stage from USD and attach it, reading at read_ordinal."""
    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    stage = ovstage.Stage("ovphysx-output-read-sample")
    attached = False
    try:
        ovstage.population.open_usd(
            stage, str(usd_path), ordinal=read_ordinal, domains=ovstage.PopulationDomain.PHYSICS
        )
        # Population does not seal: the caller owns ordinal lifecycle, and
        # attach_ovstage() reads at a sealed ordinal. The control and output
        # ordinals below are sealed the same way.
        stage.advance_write_floor(ordinal=read_ordinal).wait()
        physx.attach_ovstage(stage, read_ordinal=read_ordinal)
        attached = True
        return stage
    except Exception as exc:
        if attached:
            try:
                physx.detach_ovstage()
            except Exception as detach_exc:
                # Keep the Stage alive through PhysX's attachment reference if
                # native detach fails; destroying it here would leave a dangling
                # native pointer.
                raise RuntimeError(
                    f"Failed to detach ovstage after scene setup failed: {exc}"
                ) from detach_exc
        stage.destroy()
        raise


def _lane_column(stage, array):
    """Build an ovstage vec/scalar column DLTensor from a NumPy array.

    ovstage folds the per-element component count into ``dtype.lanes`` of a flat
    ``shape=[N]`` column. A ``[N, C]`` array becomes lanes=C, shape=[N]; a 1D
    ``[N]`` array becomes lanes=1, shape=[N].
    """
    data = np.ascontiguousarray(array, dtype=np.float32)
    lanes = data.shape[-1] if data.ndim >= 2 else 1
    leading = data.size // lanes if lanes else 0
    return ovstage.make_dltensor(
        data,
        dtype=ovstage.DLDataType(ovstage.DLDataTypeCode.kDLFloat, 32, lanes),
        shape=[leading],
        ndim=1,
    ), leading


def author_velocity(stage, group, ordinal, velocity):
    """Author physics:velocity on every prim in `group` at `ordinal`, reusing the
    group's interned prim_list directly (no path rebuild, no repack)."""
    query = stage.query_from_path_list(group.prim_list)
    try:
        data = np.tile(np.asarray(velocity, dtype=np.float32), (group.prim_count, 1))
        tensor, _ = _lane_column(stage, data)
        stage.write_attribute(
            query, "physics:velocity", ordinal, tensor, is_array=False
        ).wait()
    finally:
        stage.release_query(query).wait()


def read_rigid_body_vectors(physx, attribute):
    """Return all fixed-width rigid-body values for one vector attribute."""
    columns = []
    with physx.read(SimObjectType.RIGID_BODY, [attribute], ObjectScope.ALL) as result:
        for group in result.groups:
            if not group.is_delete and not group.is_array and group.tensors:
                columns.append(np.asarray(group.tensors[0], dtype=np.float32))
    if not columns:
        raise RuntimeError(f"No rigid-body {attribute} values were read")
    return np.concatenate(columns, axis=0)


def dump_group_to_ovstage(stage, paths_dict, group, ordinal):
    """Write one read-output column group back into ovstage under sim:<attribute>.

    Reuses the group's interned ``prim_list`` (no repack) and resolves its
    ``attribute`` token through the shared path dictionary for a readable name.
    These output ordinals are never drained (see module docstring).
    """
    if group.is_delete or not group.tensors:
        return
    name = paths_dict.token_to_string(group.attribute) or "value"
    attr = f"sim:{name}"
    query = stage.query_from_path_list(group.prim_list)
    try:
        if group.is_array:
            # Ragged column: one tensor per prim.
            columns = [_lane_column(stage, t)[0] for t in group.tensors]
            stage.write_attribute(
                query,
                attr,
                ordinal,
                columns,
                is_array=True,
                semantic=group.semantic,
            ).wait()
        else:
            # Fixed column: one tensor stacked over all the group's prims.
            tensor, _ = _lane_column(stage, group.tensors[0])
            stage.write_attribute(
                query,
                attr,
                ordinal,
                tensor,
                is_array=False,
                semantic=group.semantic,
            ).wait()
    finally:
        stage.release_query(query).wait()


# [tutorial-start]
def main():
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    stage = None
    paths_dict = None

    # Two interleaved ordinal lanes past the parse ordinal (1): even ordinals
    # carry control edits (drained by physics); odd ordinals carry physics
    # output (never drained).
    control_ordinal = 2
    output_ordinal = 3

    try:
        script_dir = Path(__file__).resolve().parent
        usd_path = script_dir / ".." / "data" / "boxes_falling_on_groundplane.usda"
        print(f"Loading USD scene through ovstage: {usd_path}")
        stage = attach_scene(physx, usd_path, read_ordinal=1)
        physx.wait_all()
        # The path dictionary is process-shared with the attached Stage, so it
        # resolves the interned attribute tokens the read returns.
        paths_dict = ovstage.PathDictionary(stage)

        dt = 1.0 / 60.0
        for frame in range(5):
            # 1. app -> physics: alternate the sideways velocity so every frame
            #    demonstrates an observable change. The read hands back each body
            #    group's interned prim_list, which we reuse verbatim to author the
            #    control edit (no repack).
            target_velocity = (2.0 if frame % 2 == 0 else -2.0, 0.0, 0.0)
            velocity_before = read_rigid_body_vectors(physx, "linearVelocity")
            position_before = read_rigid_body_vectors(physx, "position")
            authored = False
            with physx.read(SimObjectType.RIGID_BODY, ["position"], ObjectScope.ALL) as result:
                for group in result.groups:
                    if not group.is_array:
                        author_velocity(stage, group, control_ordinal, velocity=target_velocity)
                        authored = True
            if not authored:
                raise RuntimeError("No rigid bodies were available for the velocity control write")

            # Waiting for write_attribute only completes the write operation. The
            # write floor seals the ordinal so update_from_ovstage can consume it.
            stage.advance_write_floor(ordinal=control_ordinal).wait()
            physx.update_from_ovstage(control_ordinal, control_ordinal)

            velocity_after = read_rigid_body_vectors(physx, "linearVelocity")
            if velocity_after.shape != velocity_before.shape:
                raise RuntimeError("Rigid-body count changed while applying the velocity control")
            if not np.allclose(velocity_after[:, 0], target_velocity[0], atol=1.0e-5):
                raise RuntimeError(
                    f"physics:velocity was not applied: expected x={target_velocity[0]:.3f}, "
                    f"observed range [{velocity_after[:, 0].min():.3f}, "
                    f"{velocity_after[:, 0].max():.3f}]"
                )
            if np.allclose(velocity_after[:, 0], velocity_before[:, 0], atol=1.0e-5):
                raise RuntimeError("physics:velocity did not change any rigid-body x velocity")

            # 2. step.
            physx.step(dt)
            physx.wait_all()

            position_after = read_rigid_body_vectors(physx, "position")
            if position_after.shape != position_before.shape:
                raise RuntimeError("Rigid-body count changed during the simulation step")
            x_displacement = position_after[:, 0] - position_before[:, 0]
            expected_displacement = target_velocity[0] * dt
            if not np.allclose(x_displacement, expected_displacement, atol=1.0e-4):
                raise RuntimeError(
                    f"Unexpected sideways displacement: expected {expected_displacement:.6f}, "
                    f"observed range [{x_displacement.min():.6f}, {x_displacement.max():.6f}]"
                )
            print(
                f"  frame {frame}: velocity x changed "
                f"{velocity_before[:, 0].mean():.3f} -> {velocity_after[:, 0].mean():.3f} "
                f"for {velocity_after.shape[0]} body(s); mean dx={x_displacement.mean():.6f}"
            )

            # 3. physics -> app: read every output type and dump it back to ovstage
            #    at the output ordinal (which physics never drains).
            dumped = 0
            for object_type, attributes in OUTPUT_ATTRIBUTES.items():
                with physx.read(object_type, attributes, ObjectScope.ALL) as result:
                    for group in result.groups:
                        dump_group_to_ovstage(stage, paths_dict, group, output_ordinal)
                        dumped += 1
            stage.advance_write_floor(ordinal=output_ordinal).wait()
            print(f"  frame {frame}: dumped {dumped} output column group(s) to ovstage")

            # 4. advance both lanes; the output ordinal we just wrote is skipped forever.
            control_ordinal += 2
            output_ordinal += 2

        print("Closed-loop ovstage output read completed successfully")
    finally:
        if paths_dict is not None:
            paths_dict.destroy()
        if stage is not None:
            physx.detach_ovstage()
            stage.destroy()
        physx.release()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
# [tutorial-end]
