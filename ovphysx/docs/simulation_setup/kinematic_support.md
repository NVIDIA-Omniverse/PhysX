<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Kinematic Support Geometry

Kinematic support geometry is moved by an external controller while dynamic
bodies respond to it through contact. Pallets, fixtures, end-effector-held
supports, and conveyors all fit this description, but they use two different
motion mechanisms.

## Translating Supports and Kinematic Targets

Use a kinematic rigid body when the support itself moves through space.

Before you begin, confirm the following:

- The stage authors a `PhysicsScene` prim, the support body, and a dynamic rider
  body resting on the support (refer to [Physics Scene](physics_scene.md)).
- DirectGPU (`eENABLE_DIRECT_GPU_API`) is disabled. The ovstage transform-update
  path that drives a kinematic target is not applied to rigid actors under
  DirectGPU.
- The producer that publishes transforms can write both the local and the
  resolved world transform of the support at the same ovstage ordinal.

Author and drive the support as follows:

1. Apply `PhysicsRigidBodyAPI` and `PhysicsCollisionAPI` to the body.
2. Set `physics:kinematicEnabled = true`.
3. Bind a physics material with enough static and dynamic friction for the load.
4. Write a new target pose before each step.

The support is driven correctly when a read of `RIGID_BODY_POSE` places the
support at the pose you wrote for that step, and the rider's position along the
motion axis has advanced with it over successive steps. A rider displacement near
zero while the support moves means the support was teleported rather than driven
to a kinematic target, which is the failure the shipped samples check for.

Publish the body's local `omni:xform` and resolved
`omni:fabric:worldMatrix` attributes through ovstage at a new control ordinal,
seal the ordinal, and pass it to `update_from_ovstage()` before stepping. For a
kinematic body, the runtime converts that transform update to
`PxRigidDynamic::setKinematicTarget()`. Preserve the body's authored scale in
both matrices; changing scale is structural and reconstructs the PhysX actor
instead of updating its target.

The local and world matrices are equal only when the body's parent has an
identity world transform, as in the shipped samples. Under a transformed
parent, compose `omni:fabric:worldMatrix` from the local transform and the
parent's resolved world transform. If a producer moves a parent, it must also
republish the affected descendants' resolved world matrices. A gRPC or other
transport should express the same operation as consistent local and world
ovstage writes, seal the control ordinal, and drain it before stepping.

This transform-update path is not applied to rigid actors after DirectGPU
(`eENABLE_DIRECT_GPU_API`) is enabled. Leave DirectGPU disabled when controlling
kinematic supports through ovstage. GPU dynamics without DirectGPU can still be
used.

Do not substitute a legacy `RIGID_BODY_POSE` tensor write. That path calls
`setGlobalPose` and teleports the body; it does not provide the contact velocity
needed to carry a rider. The tensor-binding write surface is deprecated (superseded by the
session write API, `ovphysx_write` / `PhysX.write`) and does not expose a separate
kinematic-target tensor.

C++ applications can also use `ovphysx_get_physx_ptr()` and call
`PxRigidDynamic::setKinematicTarget()` directly. Refer to
[PhysX Interop](../tutorials/physx_interop.md).

## Stationary Conveyors and Surface Velocity

Use `PhysxSurfaceVelocityAPI` when the support stays in place but its contact
surface moves:

```usda
def Cube "Conveyor" (
    prepend apiSchemas = [
        "PhysicsCollisionAPI",
        "PhysicsRigidBodyAPI",
        "PhysxSurfaceVelocityAPI"
    ]
)
{
    bool physics:kinematicEnabled = true
    bool physxSurfaceVelocity:surfaceVelocityEnabled = true
    vector3f physxSurfaceVelocity:surfaceVelocity = (1, 0, 0)
    bool physxSurfaceVelocity:surfaceVelocityLocalSpace = false
}
```

Apply the surface-velocity API to the rigid-body prim, not only to a child
collider. Runtime changes use the same `physxSurfaceVelocity:*` attributes
through the ovstage control-ordinal path. Nonzero `physics:velocity` and
`physics:angularVelocity` on a kinematic body are retained as a legacy
surface-velocity shortcut, but new code should use
`PhysxSurfaceVelocityAPI`.

Surface velocity uses contact modification. Leave DirectGPU disabled for scenes
that need it. GPU dynamics without DirectGPU can still be used.

## Combining Target Motion and Surface Velocity

The two mechanisms are additive. A kinematic target contributes the support's
physical step velocity; `PhysxSurfaceVelocityAPI` contributes an additional
contact-target velocity. Use both only when the intended surface motion is
relative to an already-moving support. Adding surface velocity to compensate
for a teleported pallet is not equivalent to driving a kinematic target and can
double-drive cargo after the target path is corrected.

The C and Python samples in [Runnable Samples](#runnable-samples) exercise three
isolated lanes in one scene:

- a translating platform driven by ovstage transform updates;
- a stationary platform driven only by surface velocity;
- a translating platform with additional surface velocity.

Each lane checks rider displacement, and the combined lane checks that its
motion is observably greater than either independent lane.

## Sleep, Friction, and Ordering

A surface-velocity change does not wake a body that has settled to sleep.
Disable sleeping for continuously controlled loads by setting
`physxRigidBody:sleepThreshold = 0`, or call
`ovphysx_rigid_body_view_wake_up()` (`TensorBinding.wake_up()` in Python)
immediately before motion begins. Kinematic support motion and surface velocity
also require sufficient friction to transmit tangential motion; bind an
explicit physics material instead of relying on defaults.

Before you enter the control loop, confirm the following:

- The stage is attached, and the first ordinal the loop writes is above the
  ordinal that `attach_ovstage()` consumed.
- Sleeping is disabled on the controlled loads, or the loop wakes them before
  motion begins.
- DirectGPU is disabled, as required by both mechanisms on this page.

Then, for each frame:

1. Publish and seal ovstage transform and surface-velocity control edits.
2. Call `update_from_ovstage()` for exactly those control ordinals.
3. Step and wait for completion.
4. Read poses and velocities.

The loop is correct when each read pose matches the edit published for that
ordinal, and the rider's displacement grows monotonically across frames instead
of stalling after the first one. A rider that stops advancing while edits keep
arriving indicates a sleeping load or insufficient friction.

## Reset and Readback

Use `reset_stage()` and reload or reattach the baseline scene for deterministic
reruns. Replaying only poses and velocities from an in-contact mid-run state
does not restore PhysX contact caches and is not a deterministic reset.

Read resolved runtime state with `RIGID_BODY_POSE` and
`RIGID_BODY_VELOCITY`, or use the ovstage output-read API. Reading the source
USD only shows authored values; it does not prove that the live PhysX actor
received a target or that contact carried the rider.

## Runnable Samples

Python:

```{literalinclude} ../../tests/python_samples/kinematic_support.py
:language: python
```

C:

```{literalinclude} ../../tests/c_samples/kinematic_support_c/main.c
:language: c
```
