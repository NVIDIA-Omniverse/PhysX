# Colliders

A collider lets geometry participate in collision. It is the base for physics
interaction: rigid bodies, and later articulation links, build on it.

Apply `PhysicsCollisionAPI` to a geometry prim (`Cube`, `Sphere`, …). A collider is
**static on its own** — it doesn't move; dynamic bodies rest on or bounce off it
(for example a ground plane). It becomes **dynamic** only
when an **ancestor drives its motion**; how a prim becomes such a driver is documented
by the driving APIs themselves.

Tested reference (CI-loaded): `tests/data/simple_physics_scene.usda`
(installed: `samples/data/simple_physics_scene.usda`).

## Key Attributes

The attributes you set most when authoring colliders (what each is for, not its
full definition):

- `physics:collisionEnabled` (`UsdPhysicsCollisionAPI`) — turn this collider on or off.
- `physics:approximation` (`UsdPhysicsMeshCollisionAPI`) — how a mesh is approximated for collision (values in [Mesh Approximations](#mesh-approximations)).

For the complete attribute set with types and default values, see the schema, not
this list (it would drift): core `Physics*` attributes are defined by the OpenUSD
`UsdPhysics` schema (stock `usd-core`); PhysX-specific `Physx*` attributes are in
ovphysx's shipped codeless `generatedSchema.usda`. Both are rendered in the
[Omni Physics documentation](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html).

## Primitive vs Mesh Colliders

- **Primitives** (`Cube`, `Sphere`, `Cylinder`, `Cone`, `Capsule`) collide as
  their exact shape. `PhysicsCollisionAPI` alone is enough. Prefer them when they
  approximate the object well — they are the cheapest and most stable.
- **Meshes** need a collision **approximation**, chosen with
  `PhysicsMeshCollisionAPI`'s `physics:approximation` attribute (values in
  [Mesh Approximations](#mesh-approximations)). Apply both `PhysicsCollisionAPI`
  and `PhysicsMeshCollisionAPI` to the mesh prim; its geometry (`points`,
  `faceVertexCounts`, `faceVertexIndices`, `normals`) is authored as for any
  `UsdGeom.Mesh`. The raw mesh is not used directly for dynamic collision.

## Mesh Approximations

| `physics:approximation` | Collision shape | Use for |
|-------------------------|-----------------|---------|
| `none`, `meshSimplification` | Full / simplified triangle mesh | **Static or kinematic only** — not valid for dynamic bodies |
| `convexHull` | One convex hull | Dynamic bodies with roughly convex shape |
| `convexDecomposition` | Several convex hulls | Dynamic bodies with non-convex shape |
| `boundingSphere`, `boundingCube` | Primitive bound | Coarse, cheap dynamic approximation |
| `sdf` | Signed distance field | Dynamic bodies needing high-detail non-convex contact (PhysX schema `PhysxSDFMeshCollisionAPI`) |

Key constraint: a **dynamic** rigid body cannot use a plain triangle mesh
(`none` / `meshSimplification`). Use a convex approximation, convex
decomposition, or SDF instead. Triangle meshes are for static/kinematic geometry.

## What Lives Elsewhere

These are colliders topics, but out of scope for this base reference; see the
Omni Physics documentation
(https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html):

- **Contact / rest offsets** (`PhysxCollisionAPI`) — tuning when contacts
  generate.
- **Collision groups and pairwise filtering** (`UsdPhysicsCollisionGroup`,
  `UsdPhysicsFilteredPairsAPI`) — disabling collision between selected colliders.
- **CPU/GPU collider compatibility** — which collider pairs are supported in each
  simulation mode.

These use PhysX-specific (codeless) schemas where noted; register them and apply
by identifier as shown in `SKILL.md`.
