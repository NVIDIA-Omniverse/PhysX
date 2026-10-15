# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-WRITE-001

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

"""Session write sample using the explicit synchronous conveniences.

This is the runnable companion to the `ovphysx-session-write` skill and to
`output_read.py` (its read-side mirror): open a `PhysX.write_sync()` session for one
attribute, fill every mapped group's tensor in place, commit each filled group,
then step and verify the consequence.

Two things this sample demonstrates directly, both called out in the skill:

1. Read simulated results from a STATE attribute, not the control input you wrote:
   this sample writes `linearVelocity` but verifies its effect by reading `position`
   after stepping, not by reading `linearVelocity` back.
2. `PhysX.write_sync()` collects the opening task and raises `TaskError` for an
   unrecognized or unwritable attribute before exposing a session -- there is no
   separate public Python writability query to consult first (see the skill).
"""

# [tutorial-start]
# @implements REQ-CAPI-OVSTAGE-XFORM-001
# @covers AC-1

from pathlib import Path

import numpy as np
import ovstage

import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx import PhysX, TaskError
from ovphysx.types import ObjectScope, SimObjectType

_physx_schemas_registered = False


def attach_scene(physx, usd_path):
    """Populate an ovstage Stage from USD and attach it at ordinal 1."""
    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    # Register before population; this scene can omit the Newton provider.
    global _physx_schemas_registered
    if not _physx_schemas_registered:
        schema_roots = [str(p) for p in physx_schema_paths()]
        try:
            schema_roots.append(str(ovphysx.newton_schema_root()))
        except FileNotFoundError:
            pass  # newton-usd-schemas is optional; this scene has no newton:* attributes.
        ovstage.population.register_usd_schemas(schema_roots)
        _physx_schemas_registered = True
    stage = ovstage.Stage("ovphysx-session-write-sample")
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=1, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
        return stage
    except Exception as exc:
        if physx.is_stage_attached(stage):
            try:
                physx.wait_task(physx.detach_ovstage())
            except Exception as detach_exc:
                raise RuntimeError(f"Failed to detach ovstage after scene setup failed: {exc}") from detach_exc
        stage.destroy()
        raise


def read_rigid_body_vectors(physx, attribute):
    """Return all fixed-width rigid-body values for one vector attribute."""
    columns = []
    with physx.read_sync(SimObjectType.RIGID_BODY, [attribute], scope=ObjectScope.ALL) as result:
        for group in result.groups:
            if not group.is_delete and not group.is_array and group.tensors:
                columns.append(np.asarray(group.tensors[0].numpy(), dtype=np.float32))
    if not columns:
        raise RuntimeError(f"No rigid-body {attribute} values were read")
    return np.concatenate(columns, axis=0)


def main():
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    stage = None

    try:
        script_dir = Path(__file__).resolve().parent
        usd_path = script_dir / ".." / "data" / "boxes_falling_on_groundplane.usda"
        print(f"Loading USD scene through ovstage: {usd_path}")
        stage = attach_scene(physx, usd_path)
        physx.wait_all()

        # Warm up explicitly so this sequence also works on DirectGPU, which
        # requires a first step before commit. CPU writes do not require it.
        physx.warmup()
        physx.wait_all()

        position_before = read_rigid_body_vectors(physx, "position")
        num_bodies = position_before.shape[0]

        # Write a control input: push every rigid body sideways.
        target_velocity = np.array([2.0, 0.0, 0.0], dtype=np.float32)
        written = 0
        with physx.write_sync(SimObjectType.RIGID_BODY, "linearVelocity") as session:
            for group in session.groups:
                if not group.tensors:
                    continue
                tensor = group.tensors[0]
                tensor.assign(np.tile(target_velocity, (group.prim_count, 1)))
                session.commit_sync(group)
                written += group.prim_count
        if written != num_bodies:
            raise RuntimeError(f"Wrote linearVelocity for {written} bodies, expected {num_bodies}")

        dt = 1.0 / 60.0
        physx.step_sync(dt)

        # Verify the consequence by reading a STATE attribute (position), not the
        # control input (linearVelocity) that was just written.
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
            f"Wrote linearVelocity x={target_velocity[0]:.3f} for {written} body(s); "
            f"stepped once; observed mean dx={x_displacement.mean():.6f} "
            f"(expected {expected_displacement:.6f})"
        )

        # The explicit synchronous adapter surfaces a failed opening task before
        # yielding a session. This deliberately requests an attribute
        # name that does not exist for RIGID_BODY to demonstrate that failure
        # mode. The same applies to a real attribute that is READ_ONLY
        # for this object type; see the Writable attributes by
        # object type table in the ovphysx-session-write skill for which names
        # apply.
        try:
            with physx.write_sync(SimObjectType.RIGID_BODY, "notAnAttribute"):
                pass
        except TaskError as exc:
            print(f"Writing an unwritable/unknown attribute raised as expected: {exc}")
        else:
            raise RuntimeError("Expected TaskError for an unwritable attribute, none was raised")

        print("Session write sample completed successfully")
    finally:
        if stage is not None:
            physx.wait_task(physx.detach_ovstage())
            stage.destroy()
        physx.destroy()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
# [tutorial-end]
