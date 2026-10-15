<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# PhysX Interop -- Direct PhysX SDK Access

This tutorial shows how to get a raw PhysX SDK pointer from ovphysx and call
PhysX methods directly. The sample moves a kinematic rigid body using
`PxRigidDynamic::setKinematicTarget()`, then reads it back through ovphysx's session
read API (`ovphysx_read`) to confirm the runtime observed the move.

## Prerequisites

- Complete the [Hello World](hello_world.md) tutorial.
- Familiarity with the ovphysx read API (`ovphysx_query` / `ovphysx_read`).
- A C++17 compiler.

## Setup

The ovphysx SDK ships PhysX headers under `include/physx/`. Include them in your
project through the `ovphysx_PHYSX_INCLUDE_DIR` CMake variable (set automatically by
`find_package(ovphysx)`). No PhysX library linking is needed.

## CMakeLists.txt

```{literalinclude} ../../tests/c_samples/physx_interop_cpp/CMakeLists.txt
:language: cmake
```

Note: `ovphysx_PHYSX_INCLUDE_DIR` is set automatically by `find_package(ovphysx)`.

## Source

```{literalinclude} ../../tests/c_samples/physx_interop_cpp/main.cpp
:language: cpp
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

## How It Works

Before you work through these steps, confirm the three
[Prerequisites](#prerequisites): the [Hello World](hello_world.md) tutorial is
complete, a C++17 compiler is available, and `find_package(ovphysx)` has set
`ovphysx_PHYSX_INCLUDE_DIR` so the shipped PhysX headers resolve. The sample
then proceeds as follows:

1. **Create instance and load USD** — standard ovphysx workflow.
2. **Step once** — initializes the PhysX scene so rigid body actors exist.
3. **Read the address** — query `OVPHYSX_QUERY_TARGET_RIGID_ACTOR` and read
   `physxPointer`. The value is a host `uint64`. The process-global
   `PxPhysics` object is the one-row `OVPHYSX_QUERY_TARGET_PHYSICS` query. Its
   path-list entry is `/` and is not a USD prim.
4. **Cast and validate** — a rigid-actor address is a `PxRigidDynamic*` or a
   `PxRigidStatic*`, so cast to `PxRigidActor*` first, then
   use `is<PxRigidDynamic>()` to validate the concrete type before calling
   `setKinematicTarget()`.
5. **Step again** — PhysX moves the kinematic body to the target pose.
6. **Verify** — read rigid-body positions back through ovphysx (`ovphysx_query` + `ovphysx_read`) to confirm the runtime observed the PhysX-pointer-driven move.

## Query target and PhysX pointer

`physxPointer` is the borrowed PhysX address for the query target you already
chose. Standalone joints, custom joints, and articulation joints are separate
targets: `OVPHYSX_QUERY_TARGET_JOINT`, `OVPHYSX_QUERY_TARGET_CUSTOM_JOINT`,
and `OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT`. A path of another kind is
absent from that query.

## Pointer Lifecycle

- Treat pointers as invalid after stage detachment or instance destruction.
- `ovphysx_step()` does **not** invalidate pointers.
- Do not call `release()` on returned pointers -- ovphysx owns them.
- Objects explicitly created through `PxPhysics` follow PhysX SDK ownership rules.
- Use the PhysX SDK headers shipped for the same ovphysx build.

## Disabling Simulation

Do not toggle `PxActorFlag::eDISABLE_SIMULATION` on an address from
`physxPointer`. Use the ovstage `disableSimulation` attribute
(`ovphysx_write()` / `OVPHYSX_ATTR_DISABLE_SIMULATION`) instead. Refer to
[PhysX Pointer Interop](../developer_guide.md#disabling-simulation) in the
developer guide.

## Thread Safety

PhysX APIs on returned pointers must only be called **between** simulation
steps — after `wait_task()` completes for the preceding step and before the next
`ovphysx_step()` call. Refer to the
[developer guide](../developer_guide.md#thread-safety) for details.

## Result

The sample succeeds when its final read-back line reports the kinematic body at
the target position:

```text
SUCCESS: ovphysx_read observed /World/KinematicCube at (3.000, 2.000, 0.000) -- the runtime saw the move made through the raw PhysX pointer.
Cleanup complete
```

A `FAILED:` line instead of `SUCCESS:` means the read-back position did not
match the target the direct PhysX `setKinematicTarget()` call requested, and the
process exits non-zero.
