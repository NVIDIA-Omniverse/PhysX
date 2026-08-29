# Rigid Body

How to author a rigid body that simulates in a scene. Assumes the physics scene
and ground from `scene_setup.md`.

A rigid body moves without deforming. Its motion comes from gravity, collisions,
and constraints. The simulator writes its transform (dynamic) or reads it
(kinematic).

Tested reference scenes (CI-loaded, authoritative for the shapes below):
`tests/data/simple_physics_scene.usda` (dynamic + kinematic bodies, static
ground) and `tests/data/basic_simulation.usda`. They ship in the wheel/SDK as
`samples/data/simple_physics_scene.usda` and `samples/data/basic_simulation.usda`.
Read them for the full enforced layout; this file states only the essentials.

## Three Parts of a Working Rigid Body

On USD prims, combine:

1. **Rigid body** — `PhysicsRigidBodyAPI` on an `Xformable` prim. That prim and
   all descendants move together.
2. **Collider** — `PhysicsCollisionAPI` on the geometry, so the body collides
   (see `collision.md`). Without it the body falls through everything.
3. **Mass** — `PhysicsMassAPI` with `physics:mass`, or leave it and let the
   simulator infer mass from collider volume and density (see Mass).

The tested idiom puts the body API on an `Xform` and the collider on a **child**
geometry prim; the collider is associated with the body by hierarchy.

## Key Attributes

What you set most when authoring rigid bodies (intent, not full definitions):

- `physics:rigidBodyEnabled` (`UsdPhysicsRigidBodyAPI`) — disable to make the body's colliders static.
- `physics:kinematicEnabled` (`UsdPhysicsRigidBodyAPI`) — kinematic (app-driven) vs dynamic (simulated).
- `physics:velocity`, `physics:angularVelocity` (`UsdPhysicsRigidBodyAPI`) — initial linear / angular velocity.
- `physics:mass` (`UsdPhysicsMassAPI`) — explicit mass; unset means inferred (see Mass).
- `physics:density` (`UsdPhysicsMassAPI`) — density used to infer mass from collider volume.
- `physics:centerOfMass`, `physics:diagonalInertia`, `physics:principalAxes` (`UsdPhysicsMassAPI`) — explicit mass distribution.

For the complete attribute set with types and default values, see the schema, not
this list (it would drift): core `Physics*` attributes are defined by the OpenUSD
`UsdPhysics` schema (stock `usd-core`); PhysX-specific `Physx*` attributes are in
ovphysx's shipped codeless `generatedSchema.usda`. Both are rendered in the
[Omni Physics documentation](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html).

For a complete, CI-tested body in a scene (body `Xform` + child collider, placed
above the ground), see `simple_physics_scene.usda`.

## Dynamic vs Kinematic

- **Dynamic** (default): the simulator moves the body under gravity and
  collisions.
- **Kinematic**: the application controls motion; the body pushes dynamic bodies
  but ignores forces. Set `physics:kinematicEnabled = 1` (or
  `rb_api.CreateKinematicEnabledAttr(True)`). See the `KinematicCube` prim in
  `simple_physics_scene.usda`.

For a purely static collider, the simplest approach is to not apply
`PhysicsRigidBodyAPI` at all — geometry with only `PhysicsCollisionAPI` is static
(this is how the ground is authored). Use `physics:rigidBodyEnabled = 0` only to
disable an existing rigid body while keeping the API applied; its colliders then
act as static geometry.

## Colliders

A rigid body needs a collider to interact. Colliders — primitive vs mesh,
approximations, static vs dynamic — are covered in `collision.md`. What is
specific to rigid bodies: the collider goes on the body prim or on a prim below
it, and moves with the body. For multiple or offset colliders, add several child
geometry prims under the body `Xform`; all move rigidly with it.

## Mass

- **Explicit:** `PhysicsMassAPI` with `physics:mass` (and optionally center of
  mass, diagonal inertia, principal axes). No collider required.
- **Inferred:** with no mass set, mass comes from collider volume × density.
  Default density is 1000 (water, kg/m³ at `metersPerUnit = 1`). With no collider
  and no mass, mass defaults to 1.0. Explicit mass wins over density.

## PhysX-Specific Properties

CCD, per-body solver settings, and applied forces live on PhysX schemas
(`PhysxRigidBodyAPI`, `PhysxForceAPI`, ...). These are codeless in ovphysx (no
typed Python class). Register the schemas, then apply generically — see the
codeless-schema pattern and its tested sample referenced in `SKILL.md`. In
`.usda`, list them in `apiSchemas` and set their attributes directly.

## Common Pitfalls (silent failures)

Authoring mistakes usually load without error and simply misbehave:

- **Falls through the ground** — body or ground has no `PhysicsCollisionAPI`.
- **Does not move** — kinematic, `physics:rigidBodyEnabled` off, or no gravity on
  the scene.
- **Wrong fall direction** — gravity direction and up axis disagree.
- **PhysX schema ignored** — used a typed `PhysxSchema.*` class instead of
  registering codeless schemas + `prim.ApplyAPI("Physx...")`.

## Validate

Load the file and step it; the box should fall and settle. The minimal
load/step/release pattern is in `tests/python_samples/hello_world.py` (installed
in the wheel only as `samples/python_samples/hello_world.py`; the C/C++ SDK ships
the C equivalent `samples/c_samples/hello_world_c/main.c`) — see the
`basic-workflow` skill.
