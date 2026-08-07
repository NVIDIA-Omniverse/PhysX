# Tensor Bindings: Read and Write Simulation Data

This tutorial shows how to read and write simulation data through tensor bindings after you attach an ovstage-populated scene. You learn how to use path patterns to bind multiple prims in one call.

## Prerequisites

- Complete the [Hello World](hello_world.md) tutorial.
- Use a USD scene that contains physics-enabled prims matching your binding pattern.

For the physics concepts behind the quantities these tensors expose — rigid
bodies, articulations, joints and drives, deformables — and how to author them in
USD, refer to the Simulation Setup pages, starting with
[Rigid Bodies](../simulation_setup/rigid_bodies.md) and
[Articulations](../simulation_setup/articulations.md).

## Code Language

### Python

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
[GPU Warmup and Determinism](../developer_guide.md#gpu-warmup-and-determinism).

## Empty Optional Bindings

A tensor binding that matches zero prims is valid. This is useful when absence
is a legitimate result for the current scene, such as optional assets or broad
inspection queries. Empty bindings remain zero-count views; if topology changes
and matching prims are added or recreated, destroy the old binding and create a
new one. For optional queries, keep the default `raise_if_empty=False` and
check `binding.count` before allocating or reading tensors. Use
`raise_if_empty=True` only when zero matches are a configuration error for your
application.

> **Point-instancer limitation.** TensorBindingsAPI does not expose per-instance
> rows for rigid bodies created by `UsdGeom.PointInstancer`. With the default
> `raise_if_empty=False`, a rigid-body binding that targets only the point
> instancer has count zero; the opt-in `raise_if_empty=True` mode raises as
> described above. Use the ovstage [output read](../ovstage_integration.md) API
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
Python callers can also inspect `binding.spec` for the native DLPack metadata
returned by `ovphysx_get_tensor_binding_spec()`. Allocate buffers from
`binding.shape` and `binding.dtype`; most tensor types are float32, but runtime
bool bindings such as `TensorType.RIGID_BODY_DISABLE_SIMULATION`,
`TensorType.RIGID_BODY_DISABLE_GRAVITY`, and
`TensorType.ARTICULATION_BODY_DISABLE_GRAVITY` report uint8, as does the
read-only enum binding `TensorType.ARTICULATION_DOF_DRIVE_TYPE`.

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

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_RIGID_BODY_POSE_F32` | `(N, 7)` | 2D | yes | yes | `pos.xyz + quat.xyzw` | World-frame rigid body transforms |
| `OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32` | `(N, 6)` | 2D | yes | yes | `lin.xyz + ang.xyz` | World-frame linear and angular velocity |
| `OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32` | `(N, 6)` | 2D | yes | no | `lin_acc.xyz + ang_acc.xyz` | World-frame linear and angular acceleration |
| `OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32` | `(N, 3)` | 2D | no | yes | `force.xyz` | Write-only force at center of mass (control input) |
| `OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32` | `(N, 9)` | 2D | no | yes | `force.xyz + torque.xyz + pos.xyz` | Write-only wrench-at-position in world frame |

**Rigid Body Properties (standalone, non-articulated bodies)**

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
acceleration, force, and wrench use the simulation device.

For Python bindings, `binding.prim_paths` returns row metadata only; tensor
reads and writes keep using the shapes above. Rigid-body bindings return one
rigid body prim path per row. Articulation bindings return one articulation
root prim path per `A` row; link names remain available through
`binding.body_names`.

**Rigid Body Shape Properties**

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32` | `(N, S, 3)` | 3D | yes | yes | `(static_friction, dynamic_friction, restitution)` | Per-shape material properties |
| `OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32` | `(N, S)` | 2D | yes | yes | offset scalar per shape | Distance at which contacts are generated |
| `OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32` | `(N, S)` | 2D | yes | yes | offset scalar per shape | Rest separation between shapes |

Shape property tensors in this table are CPU tensors even when the simulation
is running on GPU.

**Volume Deformable Body State**

Symbols: `B` = volume deformable body count, `V` = max simulation nodes, `Vr` = max rest nodes, `E` = max simulation elements (tetrahedra, K=4), `F` = max collision elements (triangles, K=3).

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

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32` | `(Bs, Vs, 3)` | 3D | yes | yes | `pos.xyz` | Simulation mesh node positions |
| `OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32` | `(Bs, Vs, 3)` | 3D | yes | yes | `vel.xyz` | Simulation mesh node velocities |
| `OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32` | `(Bs, Vr, 3)` | 3D | yes | no | `pos.xyz` | Rest mesh node positions |
| `OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32` | `(Bs, Es, 3)` | 3D | yes | no | int32 node indices | Triangular simulation element connectivity |

Surface deformable body tensors require DirectGPU mode. Enable
`/physics/suppressReadback=true` before constructing the `PhysX` instance.

**Deformable Material Properties**

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32` | `(P,)` | 1D | yes | yes | scalar | Dynamic friction per deformable material |
| `OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32` | `(P,)` | 1D | yes | yes | scalar | Young's modulus per deformable material |
| `OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32` | `(P,)` | 1D | yes | yes | scalar | Poisson's ratio per deformable material |

Deformable material property tensors in this table are CPU tensors even when
the simulation is running on GPU.

**Articulation Root State**

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32` | `(A, 7)` | 2D | yes | yes | `pos.xyz + quat.xyzw` | Root body transform per articulation |
| `OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32` | `(A, 6)` | 2D | yes | yes | `lin.xyz + ang.xyz` | Root body velocity per articulation |

**Articulation Link State**

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32` | `(A, L, 7)` | 3D | yes | no | `pos.xyz + quat.xyzw` | Per-link pose; padded links are zero |
| `OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32` | `(A, L, 6)` | 3D | yes | no | `lin.xyz + ang.xyz` | Per-link velocity; read-only |
| `OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32` | `(A, L, 6)` | 3D | yes | no | `lin_acc.xyz + ang_acc.xyz` | Per-link linear and angular acceleration; read-only |
| `OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32` | `(A, L, 9)` | 3D | no | yes | `force.xyz + torque.xyz + pos.xyz` | Write-only per-link external wrench |

**Articulation DOF State and Control**

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32` | `(A, D)` | 2D | yes | yes | joint position scalar per DOF | Joint-space position in articulation DOF order |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32` | `(A, D)` | 2D | yes | yes | joint velocity scalar per DOF | Joint-space velocity in articulation DOF order |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32` | `(A, D)` | 2D | yes | yes | target position scalar per DOF | Position-control targets |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32` | `(A, D)` | 2D | yes | yes | target velocity scalar per DOF | Velocity-control targets |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32` | `(A, D)` | 2D | yes | yes | actuation scalar per DOF | Readback is from staging buffer; may differ from solver-applied force |

**Articulation DOF Properties**

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

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32` | `(A, L)` | 2D | yes | yes | mass scalar per link | Scalar mass per articulation link |
| `OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32` | `(A, L, 7)` | 3D | yes | yes | `pos.xyz + quat.xyzw` | COM local pose in body frame per link |
| `OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32` | `(A, L, 9)` | 3D | yes | yes | row-major 3x3 | Inertia tensor in COM frame per link |
| `OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32` | `(A, L)` | 2D | yes | no | inverse mass scalar per link | Computed from mass; read-only |
| `OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32` | `(A, L, 9)` | 3D | yes | no | row-major 3x3 | Computed from inertia; read-only |
| `OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL` | `(A, L)` | 2D | yes | yes | uint8 flag per link | Nonzero disables gravity per link at runtime; padded link columns ignored on write |

**Articulation Shape Properties**

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32` | `(A, S, 3)` | 3D | yes | yes | `(static_friction, dynamic_friction, restitution)` | Per-shape material properties per link |
| `OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32` | `(A, S)` | 2D | yes | yes | offset scalar per shape | Distance at which contacts are generated |
| `OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32` | `(A, S)` | 2D | yes | yes | offset scalar per shape | Rest separation between shapes |

Shape property tensors in this table are CPU tensors even when the simulation
is running on GPU.

**Articulation Dynamics Queries (read-only)**

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32` | `(A, R, C)` | 3D | yes | no | row-major | Shape from `getJacobianShape()`; see `R`, `C` in symbol legend above |
| `OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32` | `(A, M, M)` | 3D | yes | no | row-major square | Generalized mass matrix; shape from `getGeneralizedMassMatrixShape()` |
| `OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32` | `(A, M)` | 2D | yes | no | force scalar per generalized coordinate | Combined Coriolis and centrifugal forces |
| `OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32` | `(A, M)` | 2D | yes | no | force scalar per generalized coordinate | Gravity compensation forces |
| `OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32` | `(A, L, 6)` | 3D | yes | no | `force.xyz + torque.xyz` | Incoming joint force and torque per link |
| `OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32` | `(A, D)` | 2D | yes | no | scalar per DOF | Projected joint forces |

**Fixed Tendon Properties**

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32` | `(A, T)` | 2D | yes | yes | stiffness scalar per tendon | Requires articulation with fixed tendons |
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32` | `(A, T)` | 2D | yes | yes | damping scalar per tendon | Requires articulation with fixed tendons |
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32` | `(A, T)` | 2D | yes | yes | limit stiffness scalar per tendon | Requires articulation with fixed tendons |
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32` | `(A, T, 2)` | 3D | yes | yes | `(lower, upper)` per tendon | Fixed tendon position limits |
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32` | `(A, T)` | 2D | yes | yes | rest length scalar per tendon | Requires articulation with fixed tendons |
| `OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32` | `(A, T)` | 2D | yes | yes | offset scalar per tendon | Requires articulation with fixed tendons |

**Spatial Tendon Properties**

| Constant | Shape | Dimensionality | Read | Write | Component layout | Behavioral note |
|---|---|---|---|---|---|---|
| `OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32` | `(A, T)` | 2D | yes | yes | stiffness scalar per tendon | Requires articulation with spatial tendons |
| `OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32` | `(A, T)` | 2D | yes | yes | damping scalar per tendon | Requires articulation with spatial tendons |
| `OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32` | `(A, T)` | 2D | yes | yes | limit stiffness scalar per tendon | Requires articulation with spatial tendons |
| `OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32` | `(A, T)` | 2D | yes | yes | offset scalar per tendon | Requires articulation with spatial tendons |

For canonical enum definitions and low-level semantics, refer to `include/ovphysx/ovphysx_types.h`.

## Result

After this tutorial, you can create tensor bindings, push batched simulation inputs, and read back batched results.
