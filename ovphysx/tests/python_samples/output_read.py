# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-FRAME-001
# @covers AC-2 AC-3 AC-4

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

"""Closed-loop ovstage sample: drive control into ovphysx, write default output back.

This sample demonstrates the full ovstage round trip (ADR-0007):

1. Attach an ovstage Stage parsed from USD.
2. Each frame, author a *control* edit -- an alternating
   ``physics:velocity`` on every rigid body -- into ovstage, seal the write, and
   drain it into the running simulation with ``update_from_ovstage``.
3. Verify the velocity changed, then call the
   ``ovphysx.utils.step_and_write_to_ovstage`` utility to step once and write
   the default dynamic physics output back into ovstage.
4. Verify the expected sideways displacement and the emitted world transforms.

This sample opts into an application-owned output cache: the first helper call
reads current OVStage transform and point-instancer state, releases those views,
and retains owned Warp copies for later frames. Fixed rigid-body poses become
``omni:fabric:worldMatrix`` without changing local transform or reset-stack state.
Other output groups still use shadow ``sim:<name>`` attributes and preserve their
array shape and semantic.

The key invariant (see the ovstage Integration guide): control edits flow
through ``update_from_ovstage``. Physics output is written back at ordinals that
``update_from_ovstage`` never covers, so physics never re-ingests its own output.
Two ordinal lanes are interleaved for that: an even "control" lane (drained) and an
odd "output" lane (never drained).
"""

from pathlib import Path

import numpy as np
import ovstage
import warp as wp
from ovphysx.types import ObjectScope, SimObjectType
from ovphysx.utils import OvStageOutputCache, step_and_write_to_ovstage

import ovphysx
from ovphysx import PhysX

_physx_schemas_registered = False


def attach_scene(physx, usd_path, read_ordinal):
    """Populate an ovstage Stage from USD and attach it, reading at read_ordinal."""
    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    # ovphysx ships its PhysX USD schemas as codeless resources and does not register
    # them itself. Register them with ovstage once, before the first population
    # call in the process.
    global _physx_schemas_registered
    if not _physx_schemas_registered:
        ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
        _physx_schemas_registered = True
    stage = ovstage.Stage("ovphysx-output-read-sample")
    attached = False
    try:
        ovstage.population.open_usd(
            stage, str(usd_path), ordinal=read_ordinal, domains=ovstage.PopulationDomain.PHYSICS
        )
        stage.advance_write_floor(ordinal=read_ordinal).wait()
        physx.attach_ovstage(stage, read_ordinal=read_ordinal)
        attached = True
        return stage
    except Exception as exc:
        if attached:
            try:
                physx.detach_ovstage()
            except Exception as detach_exc:
                # If native detach fails, keep the Stage alive through PhysX's
                # attachment reference. Destroying it here would leave a dangling
                # native pointer.
                raise RuntimeError(f"Failed to detach ovstage after scene setup failed: {exc}") from detach_exc
        stage.destroy()
        raise


def _lane_column(stage, array):
    """Build an ovstage vec/scalar column DLTensor from a Warp or NumPy array.

    ovstage folds the per-element component count into ``dtype.lanes`` of a flat
    ``shape=[N]`` column. A ``[N, C]`` array becomes lanes=C, shape=[N]. A 1D
    ``[N]`` array becomes lanes=1, shape=[N]. Warp exposes native read lanes as
    the trailing dimension, so this CPU sample converts that public shape to the
    ovstage write schema explicitly.
    """
    if isinstance(array, wp.array):
        array = array.numpy()
    data = np.ascontiguousarray(array, dtype=np.float32)
    lanes = data.shape[-1] if data.ndim >= 2 else 1
    leading = data.size // lanes if lanes else 0
    return (
        ovstage.make_dltensor(
            data,
            dtype=ovstage.DLDataType(ovstage.DLDataTypeCode.kDLFloat, 32, lanes),
            shape=[leading],
            ndim=1,
        ),
        leading,
    )


def author_velocity(stage, group, ordinal, velocity):
    """Author physics:velocity on every prim in `group` at `ordinal`, reusing the
    group's interned prim_list directly (no path rebuild)."""
    query = stage.query_from_path_list(group.prim_list)
    try:
        data = np.tile(np.asarray(velocity, dtype=np.float32), (group.prim_count, 1))
        tensor, _ = _lane_column(stage, data)
        stage.write_attribute(query, "physics:velocity", ordinal, tensor, is_array=False).wait()
    finally:
        stage.release_query(query).wait()


def read_rigid_body_vectors(physx, attribute):
    """Return all fixed-width rigid-body values for one vector attribute."""
    columns = []
    with physx.read(SimObjectType.RIGID_BODY, [attribute], scope=ObjectScope.ALL) as result:
        for group in result.groups:
            if not group.is_delete and not group.is_array and group.tensors:
                columns.append(np.asarray(group.tensors[0].numpy(), dtype=np.float32))
    if not columns:
        raise RuntimeError(f"No rigid-body {attribute} values were read")
    return np.concatenate(columns, axis=0)


def _read_stage_rows(stage, paths, query, attribute, ordinal):
    """Copy one fixed OVStage attribute into a path-to-row dictionary."""
    rows = {}
    with stage.read_attributes(query, [attribute], ovstage.OrdinalRange.latest(ordinal)) as read:
        read.wait()
        for group in read.groups():
            try:
                if group.is_delete:
                    continue
                tensor = group.tensor(0)
                lanes = int(tensor.dtype.lanes) or 1
                values = np.asarray(group.array(0)).reshape(-1, lanes)
                group_paths = paths.get_path_strings(group.prim_list)
                for local in range(group.prim_count):
                    prim_index = group.prim_index(local)
                    data_index = group.data_row_index(local) if group.has_data_index_map else local
                    rows[group_paths[prim_index]] = np.array(values[data_index], copy=True)
            finally:
                stage.release_group(group)
    return rows


def verify_output_xforms(physx, stage, ordinal):
    """Verify the utility published each current rigid-body world transform."""
    checked = 0
    with ovstage.PathDictionary(stage) as paths:
        xform = paths.intern_token("omni:fabric:worldMatrix")
        with physx.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
            for physics_group in result.groups:
                if physics_group.is_delete or physics_group.is_array or not physics_group.tensors:
                    continue
                if physics_group.index_map is not None or physics_group.prim_index_map is not None:
                    raise RuntimeError("The sample does not support sparse rigid-body pose groups")
                expected_positions = np.asarray(physics_group.tensors[0].numpy(), dtype=np.float64)
                expected_paths = paths.get_path_strings(physics_group.prim_list)
                with stage.query_from_path_list(physics_group.prim_list) as query:
                    matrices = _read_stage_rows(stage, paths, query, xform, ordinal)
                for row_index, path in enumerate(expected_paths):
                    matrix = np.asarray(matrices[path], dtype=np.float64).reshape(4, 4)
                    if not np.allclose(matrix[3, :3], expected_positions[row_index], atol=1.0e-5):
                        raise RuntimeError(f"World transform translation for {path} did not match PhysX")
                    if not np.all(np.isfinite(matrix)):
                        raise RuntimeError(f"World transform for {path} contained a non-finite value")
                    checked += 1
    if not checked:
        raise RuntimeError("No rigid-body world transforms were available to verify")


# [tutorial-start]
def main():
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    stage = None
    output_cache = None

    # Two interleaved ordinal lanes past the parse ordinal (1). Even ordinals
    # carry control edits (drained by physics), odd ordinals carry physics
    # output (never drained).
    control_ordinal = 2
    output_ordinal = 3

    try:
        script_dir = Path(__file__).resolve().parent
        usd_path = script_dir / ".." / "data" / "boxes_falling_on_groundplane.usda"
        print(f"Loading USD scene through ovstage: {usd_path}")
        stage = attach_scene(physx, usd_path, read_ordinal=1)
        physx.wait_all()
        output_cache = OvStageOutputCache(physx)

        dt = 1.0 / 60.0
        for frame in range(5):
            # App -> physics: alternate the sideways velocity so every frame
            # demonstrates an observable change. The read hands back each group's
            # interned prim_list, which is reused verbatim to author the control
            # edit (no repack).
            target_velocity = (2.0 if frame % 2 == 0 else -2.0, 0.0, 0.0)
            velocity_before = read_rigid_body_vectors(physx, "linearVelocity")
            position_before = read_rigid_body_vectors(physx, "position")
            authored = False
            with physx.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
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

            # Step once and write the default dynamic output set at an
            # ordinal that physics never drains.
            dumped = step_and_write_to_ovstage(
                physx,
                dt=dt,
                output_ordinal=output_ordinal,
                cache=output_cache,
            )

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
            verify_output_xforms(physx, stage, output_ordinal)
            print(
                f"  frame {frame}: velocity x changed "
                f"{velocity_before[:, 0].mean():.3f} -> {velocity_after[:, 0].mean():.3f} "
                f"for {velocity_after.shape[0]} body(s); mean dx={x_displacement.mean():.6f}; "
                f"wrote {dumped} OVStage attribute(s)"
            )

            # Advance both lanes. The output ordinal just written is never drained.
            control_ordinal += 2
            output_ordinal += 2

        print("Closed-loop ovstage output read completed successfully")
    finally:
        if output_cache is not None:
            output_cache.close()
        if stage is not None:
            physx.detach_ovstage()
            stage.destroy()
        physx.destroy()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
# [tutorial-end]
