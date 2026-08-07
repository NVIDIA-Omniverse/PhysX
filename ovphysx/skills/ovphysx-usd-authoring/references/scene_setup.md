# Physics Scene and Ground

Foundation every simulatable scene needs: stage metadata, a physics scene prim
with gravity, and a static ground collider. The rigid-body reference builds on
this.

Tested reference scene (CI-loaded): `tests/data/simple_physics_scene.usda`
(installed in the wheel/SDK as `samples/data/simple_physics_scene.usda`) — the
complete, enforced example. This file states only the essentials.

## What a Simulatable Scene Needs

1. **Stage metadata:** a default prim, `metersPerUnit`, and `upAxis`.
2. **A `PhysicsScene` prim** with gravity.
3. **Geometry with a collider** so bodies can rest or collide (see
   `collision.md`).

## Units and Up Axis

- `metersPerUnit` sets the length unit (`1` = meters). Keep all sizes,
  positions, and velocities in that unit.
- `upAxis` is `"Y"` or `"Z"`. Point `physics:gravityDirection` down the same
  axis (up axis `Y` → gravity `(0, -1, 0)`).
- Keep units and up axis consistent across every layer and asset.

## Physics Scene and Ground

Author a `PhysicsScene` prim with gravity, and a static ground: a large, flat
`Cube` with `PhysicsCollisionAPI` and no rigid body (see `collision.md`). The
complete, CI-tested form is `simple_physics_scene.usda` (installed:
`samples/data/simple_physics_scene.usda`); see `rigid_body.md` for adding a body.

## Key Attributes (PhysicsScene)

- `physics:gravityDirection` (`UsdPhysicsScene`) — unit vector gravity points along.
- `physics:gravityMagnitude` (`UsdPhysicsScene`) — gravity strength (length unit per second squared).

Stage metadata (`metersPerUnit`, `upAxis`, default prim) is set through
`Usd.Stage` / `UsdGeom`, not the physics schema — see Units and Up Axis above.
For attribute types and defaults, see the OpenUSD `UsdPhysics` schema (it would
drift if copied here).

## Notes

- Solver tuning (time steps per second, GPU dynamics, CCD) lives on
  `PhysxSceneAPI`, a PhysX-specific (codeless) schema — apply it as shown in
  `SKILL.md`, not with a typed `PhysxSchema` class.
- An infinite `Plane` collider also works as ground, but the CI-tested shape is
  the scaled `Cube` above; prefer it unless you need an unbounded floor.
