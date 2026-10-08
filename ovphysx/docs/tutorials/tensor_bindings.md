<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Tensor Bindings -- Read and Write Simulation Data

> **Deprecated since ovphysx 0.6.** The tensor-binding API this tutorial teaches is superseded
> by the session read/write API — `ovphysx_read` / `ovphysx_write` in C, `PhysX.read` /
> `PhysX.write` in Python. Bindings keep working through the deprecation window but are not the
> recommended path for new code. The runnable session samples are
> `tests/c_samples/output_read_c/` and `tests/python_samples/output_read.py`. Refer to
> [Migrating to the Session Read/Write API](#migrating-to-the-session-readwrite-api).

This tutorial shows how to read and write simulation data through tensor bindings after you attach an ovstage-populated scene. You learn how to use path patterns to bind multiple physics objects in one call, including runtime-only clone paths.

## Migrating to the session read/write API

The session API replaces a binding's single packed `TensorType` with an `(object type,
attribute name)` pair passed to `ovphysx_query` + `ovphysx_read` / `ovphysx_write`. Two model
changes matter when porting:

- **Packed columns split into per-attribute columns.** `RIGID_BODY_POSE` `(N, 7)` becomes two
  attributes — `position` `(N, 3)` and `orientation` `(N, 4)`; `RIGID_BODY_VELOCITY` `(N, 6)`
  becomes `linearVelocity` `(N, 3)` and `angularVelocity` `(N, 3)`.
- **Angular units are per-axis, not per column.** An `ARTICULATION_JOINT` joint row holds one value
  per unlocked reduced-coordinate DOF *axis*, so the unit of `jointPosition`, `jointVelocity`,
  `jointPositionTarget`, and `jointVelocityTarget` follows the axis kind, not the column: an **angular
  axis (revolute, or a D6 rotational DOF) is always in degrees**, and a **prismatic axis stays in the
  stage's base length unit** (no conversion). The joint's axis kind fixes this — authoring a
  `JointStateAPI` does not change it. A binding value on an angular axis must be scaled to degrees,
  but scaling a prismatic axis by 57.3× is a bug — so convert by the axis kind you authored in USD
  (no per-row attribute reports it back), not by blanket-converting the column. Refer to
  [ovstage integration](../ovstage_integration.md) for the full per-axis rule. Everything else,
  including the body / link / root `angularVelocity` columns, is the engine's native **radians** with
  **no conversion** — scaling those by 57.3× is likewise a bug.

**Tensor binding to session API mapping**

This table maps each tensor-binding `TensorType` to its session object type and
attributes:

| Tensor binding `TensorType` | Session object type | Session attribute(s) |
|---|---|---|
| `RIGID_BODY_POSE` | `RIGID_BODY` | `position`, `orientation` |
| `RIGID_BODY_VELOCITY` | `RIGID_BODY` | `linearVelocity`, `angularVelocity` (rad/s — no conversion) |
| `RIGID_BODY_FORCE` / `RIGID_BODY_WRENCH` | `RIGID_BODY` | `force` / `wrench` (write-only) |
| `RIGID_BODY_MASS` / `RIGID_BODY_INERTIA` | `RIGID_BODY` | `mass` / `inertia` |
| `RIGID_BODY_COM_POSE` | `RIGID_BODY` | `centerOfMassPosition`, `centerOfMassOrientation` |
| `ARTICULATION_DOF_POSITION` / `ARTICULATION_DOF_VELOCITY` | `ARTICULATION_JOINT` | `jointPosition` / `jointVelocity` (per-axis units — refer to the per-axis angular-unit rule in this section) |
| `ARTICULATION_DOF_POSITION_TARGET` / `..._VELOCITY_TARGET` | `ARTICULATION_JOINT` | `jointPositionTarget` / `jointVelocityTarget` (per-axis units — refer to the per-axis angular-unit rule in this section) |
| `ARTICULATION_ROOT_POSE` | `ARTICULATION` | `rootPosition`, `rootOrientation` |
| `ARTICULATION_ROOT_VELOCITY` | `ARTICULATION` | `rootLinearVelocity`, `rootAngularVelocity` (rad/s — no conversion) |
| `ARTICULATION_LINK_POSE` / `ARTICULATION_LINK_VELOCITY` | `ARTICULATION_LINK` | `position` + `orientation` / `linearVelocity` + `angularVelocity` (rad/s — no conversion) |
| `ARTICULATION_LINK_WRENCH` | `ARTICULATION_LINK` | `wrench` (write-only) |

This is the common subset. The full attribute vocabulary is the `OVPHYSX_ATTR_*` macros in
`ovphysx_types.h`, and which `(object type, attribute)` pairs are writable is queryable at
runtime through `ovphysx_writability`.

## Prerequisites

- Complete the [Hello World](hello_world.md) tutorial.
- A USD scene that contains physics-enabled prims matching your binding pattern.
  This tutorial uses `links_chain_sample.usda`, which ships with every package under
  `ovphysx/samples/data/` in the wheel, `<sdk-root>/samples/data/` in the C/C++ SDK,
  and `tests/data/` in a repository checkout.

For the physics concepts behind the quantities these tensors expose — rigid
bodies, articulations, joints and drives, deformables — and how to author them in
USD, refer to the Simulation Setup pages, starting with
[Rigid Bodies](../simulation_setup/rigid_bodies.md) and
[Articulations](../simulation_setup/articulations.md).

## Code Language

### Python

This complete sample attaches `links_chain_sample.usda`, creates a DOF
velocity-target binding, a link-pose binding, and an optional rigid-body pose
binding from path patterns, writes velocity targets, steps, and reads link
poses back:

```{literalinclude} ../../tests/python_samples/tensor_bindings.py
:language: python
```

### C

Create tensor bindings, write control targets, step, and read back state:

```{literalinclude} ../../tests/c_samples/tensor_bindings_c/main.c
:language: c
```

For GPU tensor bindings with CUDA, refer to `tensor_bindings_gpu_c/` in the samples
directory. GPU dynamics are enabled by default (`physxScene:enableGPUDynamics`
defaults to `true`); set it to `false` to opt into CPU dynamics. For maximum
performance in tensor-heavy loops, GPU dynamics alone is not enough: enable
DirectGPU TensorAPI before creating the `PhysX` instance with
`/physics/suppressReadback=true`. Refer to
[Warmup and Determinism](../developer_guide.md#warmup-and-determinism).

## Empty Optional Bindings

A tensor binding that matches zero physics objects is valid. This is useful when absence
is a legitimate result for the current scene, such as optional assets or broad
inspection queries. Empty bindings remain zero-count views; if topology changes
and matching physics objects are added or recreated, destroy the old binding and create a
new one. For optional queries, keep the default `raise_if_empty=False` and
check `binding.count` before allocating or reading tensors. Use
`raise_if_empty=True` only when zero matches are a configuration error for your
application.

> **Point-instancer limitation.** TensorBindingsAPI does not expose per-instance
> rows for rigid bodies created by `UsdGeom.PointInstancer`. With the default
> `raise_if_empty=False`, a rigid-body binding that targets only the point
> instancer has count zero; the opt-in `raise_if_empty=True` mode raises
> instead. Use the ovstage [output read](../ovstage_integration.md) API
> for simulated instance readback. For control, author the point instancer's
> `positions`, `orientations`, `velocities`, and `angularVelocities` arrays
> through ovstage and pass those control ordinals to `update_from_ovstage()`.
> Use standalone rigid-body prims when per-body tensor bindings are required.

## Binding Lifetime

Tensor bindings are views of the physics objects realized for the current stage.
Create them after loading USD and reuse them across simulation steps. A normal
`step()` or `step_sync()` does not invalidate a binding.

Do not keep cached bindings across application-owned topology changes. Before
`reset()`, before removing USD data that contains bound objects, or before
loading or reparsing a stage so bound objects are destroyed and recreated,
destroy cached bindings when practical. If a stale binding survives one of those
lifecycle operations, only destroy it; do not read or write through it. Create a
replacement binding after the operation completes. In reset-heavy episode code,
the reset path should clear cached bindings because that path is where the
application changes the stage.

`step()` is asynchronous: in-stream tensor reads and writes do not need extra
synchronization, but out-of-stream consumers must call `wait_op()` or
`wait_all()` before reading results. Refer to the
[Execution Model](../developer_guide.md#execution-model) for details.

## Tensor Type Reference

Use this table to pre-allocate tensors without probing `binding.shape` at runtime.
Python callers can also inspect `binding.spec` for the DLPack dtype and layout
returned by `ovphysx_get_tensor_binding_spec()`. Allocate buffers from
`binding.shape` and `binding.dtype`; most tensor types are float32, but runtime
bool bindings such as `TensorType.RIGID_BODY_DISABLE_SIMULATION`,
`TensorType.RIGID_BODY_DISABLE_GRAVITY`, and
`TensorType.ARTICULATION_BODY_DISABLE_GRAVITY` report uint8, as does the
read-only enum binding `TensorType.ARTICULATION_DOF_DRIVE_TYPE`.

Inspect `binding.native_device` before choosing where to allocate. It returns a
Python-owned `DLDevice`: CPU-only property bindings report `kDLCPU` even in a
DirectGPU scene, while other bindings follow their native TensorAPI view. That
view is CUDA for DirectGPU and CPU otherwise, even when the scene uses GPU
dynamics. In C, query the same value with
`ovphysx_get_tensor_binding_native_device()`. This is the no-staging device;
the query does not change existing read/write behavior.

The standalone rigid-body property, articulation DOF/body property, shape
property, and deformable-material tables in
[Tensor Type Reference](#tensor-type-reference) are CPU-only. Their data,
index, and mask buffers must be host-resident even when the simulation runs on
GPU; CUDA and CUDA-managed buffers are rejected rather than copied to host.
Fixed and spatial tendon property tensors use the simulation device instead.

Symbols:
- `N`: rigid body count in the binding
- `A`: articulation count in the binding
- `L`: max link count across matched articulations
- `D`: max DOF count across matched articulations
- `T`: max tendon count across matched articulations (fixed or spatial, depending on type)
- `M`: generalized coordinate count — `numDofs` for fixed-base, `numDofs + 6` for floating-base articulations
- `S`: max collision shape count per body/link in the binding
- `R`, `C`: Jacobian shape from `getJacobianShape()` — fixed-base: `R=(L-1)*6, C=D`; floating-base: `R=(L-1)*6+6, C=D+6`
- `B`: volume deformable body count in the binding
- `V`: max simulation node count across matched volume deformables
- `Vr`: max rest node count across matched volume deformables
- `E`: max simulation element count across matched volume deformables (tetrahedra, K=4)
- `F`: max collision element count across matched volume deformables; K = `getNumNodesPerElement()` (4 for tetmesh)
- `P`: deformable material count in the binding
- `Bs`: surface deformable body count in the binding
- `Vs`: max simulation node count across matched surface deformables
- `Es`: max simulation element count across matched surface deformables (triangles, K=3)

**Rigid Body State**

These constants expose per-body simulation state on a rigid-body binding:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_RIGID_BODY_POSE_F32` | `(N, 7)` | 2D | yes | yes | `pos.xyz + quat.xyzw` | World-frame pose; writes teleport with `setGlobalPose` and are not kinematic-target control |
| `OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32` | `(N, 6)` | 2D | yes | yes | `lin.xyz + ang.xyz` | World-frame linear and angular velocity |
| `OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32` | `(N, 6)` | 2D | yes | no | `lin_acc.xyz + ang_acc.xyz` | World-frame linear and angular acceleration |
| `OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32` | `(N, 3)` | 2D | no | yes | `force.xyz` | Write-only force at center of mass (control input) |
| `OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32` | `(N, 9)` | 2D | no | yes | `force.xyz + torque.xyz + pos.xyz` | Write-only wrench-at-position in world frame |

**Rigid Body Properties (standalone, non-articulated bodies)**

These constants expose mass, inertia, and runtime flags on standalone rigid bodies:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_RIGID_BODY_MASS_F32` | `(N,)` | 1D | yes | yes | mass scalar | Scalar mass per rigid body |
| `OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32` | `(N,)` | 1D | yes | no | inverse mass scalar | Computed from mass; read-only |
| `OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32` | `(N, 9)` | 2D | yes | yes | row-major 3x3 | Inertia tensor in body frame |
| `OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32` | `(N, 9)` | 2D | yes | no | row-major 3x3 | Computed from inertia; read-only |
| `OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32` | `(N, 7)` | 2D | yes | yes | `pos.xyz + quat.xyzw` | COM local pose in body frame |
| `OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL` | `(N,)` | 1D | yes | yes | uint8 flag | Nonzero disables simulation at runtime |
| `OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL` | `(N,)` | 1D | yes | yes | uint8 flag | Nonzero disables gravity at runtime; live PhysX flags only |

Rigid body property tensors in this table are CPU tensors even when the
simulation is running on GPU. State tensors such as pose, velocity,
acceleration, force, and wrench use the binding's native TensorAPI view; query
the binding device rather than inferring it from GPU dynamics.

For Python bindings, `binding.prim_paths` returns row metadata only; tensor
reads and writes keep using the tabulated shapes. Rigid-body bindings return one
rigid-body object path per row. Articulation bindings return one articulation
root object path per `A` row; link names remain available through
`binding.body_names`.

**Rigid Body Shape Properties**

These constants expose per-collision-shape material and offset values on a
rigid-body binding:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32` | `(N, S, 3)` | 3D | yes | yes | `(static_friction, dynamic_friction, restitution)` | Per-shape material properties |
| `OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32` | `(N, S)` | 2D | yes | yes | offset scalar per shape | Distance at which contacts are generated |
| `OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32` | `(N, S)` | 2D | yes | yes | offset scalar per shape | Rest separation between shapes |

Shape property tensors in this table are CPU tensors even when the simulation
is running on GPU.

**Volume Deformable Body State**

Symbols: `B` = volume deformable body count, `V` = max simulation nodes, `Vr` = max rest nodes, `E` = max simulation elements (tetrahedra, K=4), `F` = max collision elements (triangles, K=3).

These constants expose simulation and rest mesh state on volume deformable bodies:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32` | `(B, V, 3)` | 3D | yes | yes | `pos.xyz` | Simulation mesh node positions |
| `OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32` | `(B, V, 3)` | 3D | yes | yes | `vel.xyz` | Simulation mesh node velocities |
| `OVPHYSX_TENSOR_DEFORMABLE_SIM_KINEMATIC_TARGET_F32` | `(B, V, 4)` | 3D | yes | yes | `pos.xyz + flag` | Simulation mesh kinematic targets |
| `OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32` | `(B, Vr, 3)` | 3D | yes | no | `pos.xyz` | Rest mesh node positions |
| `OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32` | `(B, E, 4)` | 3D | yes | no | int32 node indices | Tetrahedral simulation element connectivity |
| `OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32` | `(B, F, K)` | 3D | yes | no | int32 node indices | Collision element connectivity; K=4 for volume tetmesh |

Volume deformable body tensors require DirectGPU mode. Enable
`/physics/suppressReadback=true` before constructing the `PhysX` instance.

**Surface Deformable Body State**

Symbols: `Bs` = surface deformable body count, `Vs` = max simulation nodes, `Vr` = max rest nodes, `Es` = max simulation elements (triangles, K=3).

These constants expose simulation and rest mesh state on surface deformable bodies:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32` | `(Bs, Vs, 3)` | 3D | yes | yes | `pos.xyz` | Simulation mesh node positions |
| `OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32` | `(Bs, Vs, 3)` | 3D | yes | yes | `vel.xyz` | Simulation mesh node velocities |
| `OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32` | `(Bs, Vr, 3)` | 3D | yes | no | `pos.xyz` | Rest mesh node positions |
| `OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32` | `(Bs, Es, 3)` | 3D | yes | no | int32 node indices | Triangular simulation element connectivity |

Surface deformable body tensors require DirectGPU mode. Enable
`/physics/suppressReadback=true` before constructing the `PhysX` instance.

**Deformable Material Properties**

These constants expose per-material friction and elasticity values on a
deformable-material binding:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32` | `(P,)` | 1D | yes | yes | scalar | Dynamic friction per deformable material |
| `OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32` | `(P,)` | 1D | yes | yes | scalar | Young's modulus per deformable material |
| `OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32` | `(P,)` | 1D | yes | yes | scalar | Poisson's ratio per deformable material |

Deformable material property tensors in this table are CPU tensors even when
the simulation is running on GPU.

**Articulation Root State**

These constants expose root-body pose, velocity, and center-of-mass values, one
row per articulation:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32` | `(A, 7)` | 2D | yes | yes | `pos.xyz + quat.xyzw` | Root body transform in the exposed view world frame |
| `OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32` | `(A, 6)` | 2D | yes | yes | `lin.xyz + ang.xyz` | Root body velocity per articulation |
| `OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32` | `(A, 3)` | 2D | yes | no | `pos.xyz` | Articulation COM in the exposed view world frame; subspace origin removed |
| `OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32` | `(A, 3)` | 2D | yes | no | `pos.xyz` | Articulation COM in the root link's center-of-mass (mass) frame, not its actor/prim frame |

**Articulation Link State**

These constants expose per-link state and the write-only external wrench:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32` | `(A, L, 7)` | 3D | yes | no | `pos.xyz + quat.xyzw` | Per-link pose; padded links are zero |
| `OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32` | `(A, L, 6)` | 3D | yes | no | `lin.xyz + ang.xyz` | Per-link velocity; read-only |
| `OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32` | `(A, L, 6)` | 3D | yes | no | `lin_acc.xyz + ang_acc.xyz` | Per-link linear and angular acceleration; read-only |
| `OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32` | `(A, L, 9)` | 3D | no | yes | `force.xyz + torque.xyz + pos.xyz` | Write-only per-link external wrench |

**Articulation DOF State and Control**

These constants expose joint-space state and the drive targets that control it,
in articulation DOF order:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32` | `(A, D)` | 2D | yes | yes | joint position scalar per DOF | Joint-space position in articulation DOF order |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32` | `(A, D)` | 2D | yes | yes | joint velocity scalar per DOF | Joint-space velocity in articulation DOF order |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32` | `(A, D)` | 2D | yes | yes | target position scalar per DOF | Position-control targets |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32` | `(A, D)` | 2D | yes | yes | target velocity scalar per DOF | Velocity-control targets |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32` | `(A, D)` | 2D | yes | yes | actuation scalar per DOF | Readback is from staging buffer; can differ from solver-applied force |

**Articulation DOF Properties**

These constants expose the per-DOF drive gains, limits, and clamps:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32` | `(A, D)` | 2D | yes | yes | stiffness scalar per DOF | PD position-control stiffness |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32` | `(A, D)` | 2D | yes | yes | damping scalar per DOF | PD velocity-control damping |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_LIMIT_F32` | `(A, D, 2)` | 3D | yes | yes | `(lower, upper)` per DOF | Joint position limits |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_VELOCITY_F32` | `(A, D)` | 2D | yes | yes | max velocity scalar per DOF | Per-DOF velocity clamp |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32` | `(A, D)` | 2D | yes | yes | max force scalar per DOF | Per-DOF force/torque clamp |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_ARMATURE_F32` | `(A, D)` | 2D | yes | yes | armature scalar per DOF | Added inertia at each DOF |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_FRICTION_PROPERTIES_F32` | `(A, D, 3)` | 3D | yes | yes | `(static, dynamic, viscous)` per DOF | Friction coefficients at each DOF |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8` | `(A, D)` | 2D | yes | no | uint8 per DOF | `0`=none, `1`=force, `2`=acceleration; read-only, padded DOF columns read 0 |

**Articulation Body Properties**

These constants expose per-link mass, inertia, and gravity flags:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32` | `(A, L)` | 2D | yes | yes | mass scalar per link | Scalar mass per articulation link |
| `OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32` | `(A, L, 7)` | 3D | yes | yes | `pos.xyz + quat.xyzw` | COM local pose in body frame per link |
| `OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32` | `(A, L, 9)` | 3D | yes | yes | row-major 3x3 | Inertia tensor in COM frame per link |
| `OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32` | `(A, L)` | 2D | yes | no | inverse mass scalar per link | Computed from mass; read-only |
| `OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32` | `(A, L, 9)` | 3D | yes | no | row-major 3x3 | Computed from inertia; read-only |
| `OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL` | `(A, L)` | 2D | yes | yes | uint8 flag per link | Nonzero disables gravity per link at runtime; padded link columns ignored on write |

**Articulation Shape Properties**

These constants expose per-collision-shape material and offset values on an
articulation binding:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32` | `(A, S, 3)` | 3D | yes | yes | `(static_friction, dynamic_friction, restitution)` | Per-shape material properties per link |
| `OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32` | `(A, S)` | 2D | yes | yes | offset scalar per shape | Distance at which contacts are generated |
| `OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32` | `(A, S)` | 2D | yes | yes | offset scalar per shape | Rest separation between shapes |

Shape property tensors in this table are CPU tensors even when the simulation
is running on GPU.

**Articulation Inverse Dynamics Queries (read-only)**

These read-only constants expose the derived dynamics quantities computed by the
solver:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32` | `(A, R, C)` | 3D | yes | no | row-major | Shape from `getJacobianShape()`; refer to `R` and `C` in the [Tensor Type Reference](#tensor-type-reference) symbol list |
| `OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32` | `(A, M, M)` | 3D | yes | no | row-major square | Generalized mass matrix; shape from `getGeneralizedMassMatrixShape()` |
| `OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32` | `(A, M)` | 2D | yes | no | force scalar per generalized coordinate | Combined Coriolis and centrifugal forces |
| `OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32` | `(A, M)` | 2D | yes | no | force scalar per generalized coordinate | Gravity compensation forces |
| `OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32` | `(A, L, 6)` | 3D | yes | no | `force.xyz + torque.xyz` | Incoming joint force and torque per link |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32` | `(A, D)` | 2D | yes | no | scalar per DOF | Projected joint forces |

The generalized joint coordinates in these inverse dynamics tensors use the direction
authored by each USD joint relationship: the sign is positive when `body0` is
the articulation parent and negative when `body1` is the parent. If `S_dof` is
the diagonal matrix of those signs, use `T=S_dof` for a fixed base and
`T=diag(I6,S_dof)` for a floating base. The returned values are
`J=J_physx*T`, `M=T*M_physx*T`, `c=T*c_physx`, and `g=T*g_physx`. The packed
centroidal result follows `[A|b]=[A_physx*T|b_physx]`, so its six root columns
and bias column are unchanged. Angular generalized-coordinate dimensions use
radians and receive no degree conversion; prismatic and floating-translation
dimensions retain their linear units.

**Fixed Tendon Properties**

These constants expose per-tendon gains, limits, and lengths on articulations
that author fixed tendons:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32` | `(A, T)` | 2D | yes | yes | stiffness scalar per tendon | Requires articulation with fixed tendons |
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32` | `(A, T)` | 2D | yes | yes | damping scalar per tendon | Requires articulation with fixed tendons |
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32` | `(A, T)` | 2D | yes | yes | limit stiffness scalar per tendon | Requires articulation with fixed tendons |
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32` | `(A, T, 2)` | 3D | yes | yes | `(lower, upper)` per tendon | Fixed tendon position limits |
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32` | `(A, T)` | 2D | yes | yes | rest length scalar per tendon | Requires articulation with fixed tendons |
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32` | `(A, T)` | 2D | yes | yes | offset scalar per tendon | Requires articulation with fixed tendons |

**Spatial Tendon Properties**

These constants expose per-tendon gains and offsets on articulations that author
spatial tendons:

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32` | `(A, T)` | 2D | yes | yes | stiffness scalar per tendon | Requires articulation with spatial tendons |
| `OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32` | `(A, T)` | 2D | yes | yes | damping scalar per tendon | Requires articulation with spatial tendons |
| `OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32` | `(A, T)` | 2D | yes | yes | limit stiffness scalar per tendon | Requires articulation with spatial tendons |
| `OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32` | `(A, T)` | 2D | yes | yes | offset scalar per tendon | Requires articulation with spatial tendons |

For canonical enum definitions and low-level semantics, refer to `include/ovphysx/ovphysx_types.h`.

## Result

After this tutorial, you can create tensor bindings, push batched simulation inputs, and read back batched results. For new code, prefer the session read/write API — refer to [Migrating to the Session Read/Write API](#migrating-to-the-session-readwrite-api).
