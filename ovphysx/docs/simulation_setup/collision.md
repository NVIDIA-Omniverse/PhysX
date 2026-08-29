<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Colliders

A collider lets geometry participate in collision. It is the base that rigid
bodies and articulation links build on. Apply `PhysicsCollisionAPI` to a
geometry prim and the collision representation is created implicitly from the USD
geometry. If the collider prim (or an ancestor) has `PhysicsRigidBodyAPI`, the
collider moves with that dynamic body; otherwise it is static.

This page covers authoring colliders for scenes that ovphysx loads. For the base
authoring pattern (core `UsdPhysics` typed APIs plus codeless PhysX schemas), refer to
[Physics Schemas](../physics_schemas.md); for the physics scene and ground, refer to
[Physics Scene](physics_scene.md).

## Static Colliders

A collider with no rigid body on it or above it is static: it does not move, but
dynamic bodies rest on and bounce off it (this is how a ground is authored).

```usda
def Cube "ground" (
    prepend apiSchemas = ["PhysicsCollisionAPI"]
)
{
    double size = 100
}
```

```python
from pxr import UsdGeom, UsdPhysics

cube = UsdGeom.Cube.Define(stage, "/World/ground")
cube.CreateSizeAttr(100.0)
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
```

## Primitive Colliders

The following `UsdGeom` primitives are supported with `PhysicsCollisionAPI`, and
the resulting collision shape maps to the geometry exactly: `Sphere`, `Cube`,
`Capsule`, `Cylinder`, `Cone`. Primitives are the cheapest and most stable
choice — prefer them whenever they approximate the object well.

![Primitive colliders: cylinder, sphere, box, capsule, cone](images/collision_primitives.png)

```python
from pxr import UsdGeom, UsdPhysics

sphere = UsdGeom.Sphere.Define(stage, "/World/sphere")
sphere.CreateRadiusAttr(2.0)
UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
```

Boxes are authored as a `Cube` with non-uniform scale; capsules, cylinders, and
cones take `radius`, `height`, and `axis` attributes.

### Rounded Cones and Cylinders

A cone or cylinder can be given a rounded edge with a positive convex margin
(a PhysX-specific, codeless attribute):

```python
from pxr import Sdf

cylinder_prim.CreateAttribute("physxConvexGeometry:margin", Sdf.ValueTypeNames.Float).Set(0.1)
```

Cones and cylinders can also be approximated with convex meshes, which ignores
the margin but is often faster (prefer it unless you need smooth rolling
behavior). In ovphysx this is controlled per instance through `PhysXConfig`:

```python
from ovphysx import PhysX, PhysXConfig

# Approximate cylinders/cones with convex meshes instead of exact custom geometry.
physx = PhysX(config=PhysXConfig(
    collision_cylinder_custom_geometry=False,
    collision_cone_custom_geometry=False,
))
```

## Mesh Colliders

Mesh colliders (`UsdGeom.Mesh` with `PhysicsCollisionAPI`) require an
**approximation**, chosen with `PhysicsMeshCollisionAPI`'s
`physics:approximation` attribute:

- `none`, `meshSimplification` — full / simplified **triangle mesh**. Valid only
  for **static or kinematic** bodies, not dynamic ones.
- `convexHull` — a single convex hull. For dynamic bodies with a roughly convex
  shape.
- `convexDecomposition` — several convex hulls. For dynamic bodies with a
  non-convex shape.
- `boundingSphere`, `boundingCube` — a coarse, cheap primitive bound.
- `sdf` — a signed distance field, for dynamic bodies needing high-detail
  non-convex contact (refer to [SDF Colliders](#sdf-colliders)).

![Convex approximation options: convex hull, convex decomposition, bounding sphere, bounding cube](images/collision_approximation.png)

> **A dynamic rigid body cannot use a plain triangle mesh** (`none` /
> `meshSimplification`). Use a convex approximation, convex decomposition, or
> SDF. Triangle meshes are for static or kinematic geometry.

```python
from pxr import UsdGeom, UsdPhysics

mesh = UsdGeom.Mesh.Define(stage, "/World/mesh")
mesh.CreateFaceVertexCountsAttr(vertex_counts)
mesh.CreateFaceVertexIndicesAttr(indices)
mesh.CreatePointsAttr(points)

UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
mesh_collision.CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
```

### Merging Multiple Meshes

`PhysxMeshMergeCollisionAPI` (codeless) applied to an `Xformable` prim defines a
collection of meshes that are merged into a single collider before the
approximation is computed. The source meshes can be sourced from anywhere in the
stage. If some of the collection's meshes are descendants of a rigid body, only
those descendants move with the body — a natural consequence of the UsdPhysics
rule that only rigid-body transforms are updated.

### SDF Colliders

Signed-Distance-Field (SDF) collision enables dynamic and kinematic rigid bodies
with high-detail mesh colliders — the standard choice for contact-rich
interaction between highly non-convex shapes (for example robotic assembly).
Enable it by setting the mesh approximation to `sdf` and applying the codeless
`PhysxSDFMeshCollisionAPI`:

```python
from pxr import Sdf, UsdPhysics

UsdPhysics.CollisionAPI.Apply(sdf_mesh_prim)
mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(sdf_mesh_prim)
mesh_collision.CreateApproximationAttr("sdf")

sdf_mesh_prim.ApplyAPI("PhysxSDFMeshCollisionAPI")
sdf_mesh_prim.CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.Int).Set(300)
```

Notes:

- Multi-material triangle-mesh colliders are not supported with SDF.
- **Sparse SDFs** add a hierarchical structure with fewer samples far from the
  surface, saving memory at similar fidelity. Enable them with a nonzero
  `physxSDFMeshCollision:sdfSubgridResolution`.

![A sparse SDF: a background SDF plus high-resolution subgrids near surfaces](images/sdf_subgrids.png)

For high-throughput, on-GPU access to SDF samples and gradients at runtime, use
ovphysx's SDF view: `PhysX.create_sdf_view()` in Python or
`ovphysx_create_sdf_view()` in C (GPU instances only). SDF views are tied to the
attached USD stage; after `reset_stage()` or `detach_ovstage()`, destroy existing
views and create replacements after re-attaching a stage.

## Cooking Mesh Colliders

Generating a collision approximation from mesh data is called **cooking**, and
its output is **cooking data**. ovphysx cooks colliders that need it when a stage
is attached, blocking until the required data is available (from cache or freshly
computed). To persist cooked results across runs, configure a cooked-collider
cache directory through `PhysXConfig(cooked_collider_cache_dir=...)` — refer to the
cooked-collider cache (UJITSO) section of the
[Developer Guide](../developer_guide.md) for details.

## Rest and Contact Offsets

The **contact offset** and **rest offset** tune when and where contacts are
generated and resolved. Both are attributes of the codeless `PhysxCollisionAPI`
applied to a collider prim.

- **Contact offset** — the distance from the surface at which contacts start
  being generated. The default auto-computes a value from gravity, timestep, and
  geometry extent. Increase it for fast-moving or thin objects that tunnel
  through each other in one step (an alternative is CCD, refer to
  [Rigid Bodies](rigid_bodies.md#continuous-collision-detection)). Larger offsets
  can cost performance because more contacts are generated.
- **Rest offset** — a small distance from the surface at which the effective
  contact takes place. It may be positive, zero, or negative. A negative rest
  offset is useful when the collision geometry is slightly larger than the render
  mesh, so contact occurs at the visually correct distance.

![The effects of rest and collision offsets](images/collision_rest_offset.png)

```python
from pxr import Sdf

prim.ApplyAPI("PhysxCollisionAPI")
prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.02)
prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
```

Contact and rest offsets can also be read/written at runtime in bulk through the
`RIGID_BODY_CONTACT_OFFSET` / `RIGID_BODY_REST_OFFSET` tensor types — refer to the
[Tensor Bindings](../tutorials/tensor_bindings.md) reference.

## Collision Filtering

Two mechanisms disable collision between specific colliders.

### Collision Group Filtering

Collision groups (`UsdPhysicsCollisionGroup`) enable or disable collision between
sets of colliders. Every collider in a group collides with every other collider,
and each group carries a list of other groups it should (or should not) collide
with. By default the lists are opt-out: every group collides with every other,
and the lists name pairs that do not collide. Setting
`physxScene:invertCollisionGroupFilter` on the scene inverts this to opt-in: no
pairs collide except those named.

Assign colliders to a group with a `Usd.CollectionAPI` (`colliders` collection)
on the group prim, and relate groups to filter with the group's
`filteredGroups` relationship.

### Pairwise Collision Filtering

When groups are too coarse, `UsdPhysicsFilteredPairsAPI` disables collision
between specific stage hierarchies. Pairwise filtering takes precedence over
group filtering. The `filteredPairs` relationship names the hierarchies to
filter; all child objects (colliders, rigid bodies, articulations) are filtered
with the pair.

```python
from pxr import UsdPhysics

filtered = UsdPhysics.FilteredPairsAPI.Apply(box_a_prim)
filtered.CreateFilteredPairsRel().AddTarget("/World/boxC")
```

## Performance and Stability Tips

- **Keep sizes and dimension ratios within roughly 1/100.** Both the sizes of
  interacting colliders and the ratio of a collider's own dimensions should stay
  within about two orders of magnitude to avoid floating-point instability.
- **Prefer primitives** (sphere, capsule, box, plane) — cheapest and most
  stable.
- **Convex meshes** (convex hull / decomposition) are next. For GPU
  compatibility, a convex mesh's largest dimension should not exceed ~100x its
  insphere radius; otherwise cooking may produce a CPU-only convex that will not
  interact with GPU-only features (deformables, particles) and may be slower.
- **Cylinders and cones** model wheels and similar shapes with smooth surfaces;
  approximate them with convex meshes if precise rolling is not required.
- **Triangle meshes** suit large static or kinematic geometry; keep triangles
  evenly sized and apply the 1/100 ratio to individual triangles.
- **SDF meshes** are best for dynamic bodies needing high-detail non-convex
  collision.

For deeper contact-quality tuning (GPU contact limits, mass ratios,
depenetration, friction, timestep), refer to the
[Collision Tuning](../guides/collision_tuning.md) guide.

## CPU / GPU Collider Compatibility

Not every collider pair is supported in both CPU and GPU simulation. In general,
CPU collision geometry generates contacts on the CPU and GPU geometry on the GPU;
most geometry supports both and picks the device automatically. GPU-only features
(deformables, particles) require GPU simulation. Some CPU-only geometry (for
example an oblong convex hull, or a multi-material triangle mesh) still works with
GPU simulation by generating contacts on the CPU, which can carry a performance
penalty in large GPU scenes such as RL environments.

![Rigid body collider compatibility table](images/collider_compatibility.png)
