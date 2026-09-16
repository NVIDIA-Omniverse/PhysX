---
name: ovphysx-usd-authoring
description: Author USD physics content for ovstage to populate and ovphysx to simulate (rigid bodies, colliders, mass, physics scene). Use when creating or editing .usda or USD scenes for ovstage and ovphysx, not when calling the runtime step or tensor API. Keywords USD authoring, UsdPhysics, PhysxSchema, RigidBodyAPI, CollisionAPI, MassAPI, PhysicsScene, .usda, simulate scene.
license: Apache-2.0
compatibility: "Core UsdPhysics-authored scenes can be populated through ovstage and simulated by ovphysx 0.4.1+ (wheel or SDK). The bundled codeless PhysX schemas and ovphysx.codeless_schema_paths() require ovphysx 0.5.1+. This skill first shipped in 0.5.2. Authoring needs a text editor (.usda route) or Python with a USD runtime (Python route); core UsdPhysics works with stock usd-core, and PhysX-specific schema attributes require registering the codeless schemas."
allowed-tools: Read Write Shell
metadata:
  version: "0.1.1"
  author: NVIDIA Omniverse Physics
  tags: "ovphysx, physics, usd, authoring"
---

# Author USD Physics for ovphysx

This skill explains how to author a USD scene for ovstage to populate and
ovphysx to simulate. ovphysx consumes an attached `ovstage.Stage`, not USD or
the authoring tool's USD runtime directly. Build and save the scene first, use
ovstage to populate an `ovstage.Stage`, call `advance_write_floor()` to seal the
ordinal, and attach with `attach_ovstage()`. Drain later edits with
`update_from_ovstage()`, and clear the stage with `reset_stage()`. See
`basic-workflow` and `docs/ovstage_integration.md` for the populate/seal/attach
sequence.

## When to Use

Use this skill when you need to create or edit USD content that defines physics:
a rigid body, a collider, mass properties, or the physics scene itself.

Do not use this skill for the runtime API (creating an instance, stepping,
reading or writing simulation state). Those are covered by `basic-workflow`,
`ovphysx-output-read` (the session read/write API; the deprecated
`tensor-bindings-cpu` / `tensor-bindings-gpu` skills remain only for existing binding
code), and `clone-environments`.

## Prerequisites

The `.usda` text route needs only a text editor. For a standalone Python
authoring or validation tool, install the optional stock USD package in that
tool's environment:

```bash
python -m pip install usd-core
```

`usd-core` supplies the tool's `pxr` modules. It is not an ovphysx package
dependency or the simulator's USD runtime, and ovphysx does not accept that
runtime directly. Save the authored USD, use ovstage to populate it into an
`ovstage.Stage`, seal the write ordinal, and attach that stage to ovphysx. PyPI
currently provides no Linux aarch64 `usd-core` wheel or source distribution; on
that platform, use the `.usda` text route, author on a supported host, or supply
a compatible OpenUSD Python build.

## Authoring Routes

There are two ways to author the same USD content. Both produce a `.usda` (or
`.usd`) file that ovstage can populate for ovphysx simulation.

1. **Hand-authored `.usda` text.** Write USD ASCII directly. No Python
   dependency. Schemas are applied through the `apiSchemas` metadata list, for
   example `prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]`.

2. **Python with a USD runtime.** Use the `pxr` modules to build the stage.
   - Core physics schemas (`UsdPhysics.RigidBodyAPI`, `CollisionAPI`, `MassAPI`,
     `Scene`, `MaterialAPI`) are part of stock `usd-core` and have typed
     bindings: `UsdPhysics.RigidBodyAPI.Apply(prim)`.
   - PhysX-specific schemas (for example `PhysxSceneAPI`, `PhysxRigidBodyAPI`,
     CCD attributes) ship with ovphysx as *codeless* schemas: no typed Python
     class. `codeless_schema_paths()` only returns the resource directories; you
     must register them, then apply by identifier:

     ```python
     import ovphysx
     from pxr import Plug, Sdf, Usd, UsdPhysics

     Plug.Registry().RegisterPlugins([str(p) for p in ovphysx.codeless_schema_paths()])
     stage = Usd.Stage.CreateInMemory()
     prim = stage.DefinePrim("/World/Box", "Cube")
     UsdPhysics.RigidBodyAPI.Apply(prim)
     UsdPhysics.CollisionAPI.Apply(prim)
     if not prim.ApplyAPI("PhysxRigidBodyAPI"):
         raise RuntimeError("Failed to apply PhysxRigidBodyAPI")
     attribute = prim.CreateAttribute(
         "physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool
     )
     if not attribute.Set(True):
         raise RuntimeError("Failed to set physxRigidBody:disableGravity")
     if not stage.GetRootLayer().Export("physx_rigid_body.usda"):
         raise RuntimeError("Failed to export physx_rigid_body.usda")
     ```

     This writes `physx_rigid_body.usda`, ready for ovstage to populate for
     ovphysx simulation.

     Tested reference (source checkout only; not shipped in the wheel/SDK):
     `tests/python_samples_extra/codeless_schemas/register_codeless_schemas.py`.
     For registration ordering and DCC-host setup, see
     `docs/physics_schemas.md`.

The minimal rigid-body setup in this skill uses only core `UsdPhysics`, so it
works with stock `usd-core` without registering codeless schemas. Register the
codeless schemas only when you need PhysX-specific attributes.

## Workflow

1. Identify the physics topic you are authoring and open the matching file under
   `references/` (see the index below). Read it before writing USD.
2. Author the physics scene first (`references/scene_setup.md`), then add the
   bodies and colliders.
3. Choose one route (`.usda` or Python) and keep the whole scene in that route.
4. Validate by populating the file into an `ovstage.Stage`, sealing the write
   ordinal, attaching the stage to ovphysx, and stepping a few frames (see
   Validation). Confirm prims move as expected.

## Reference Index

Read only the file you need; each is self-contained but assumes the scene setup.

| Topic | File | Covers |
|-------|------|--------|
| Physics scene and ground | `references/scene_setup.md` | Stage metadata (units, up axis), the `PhysicsScene` prim, gravity, a static ground collider. |
| Colliders | `references/collision.md` | `PhysicsCollisionAPI`, static vs dynamic, primitive vs mesh colliders, mesh approximations. The base other physics types build on. |
| Rigid body | `references/rigid_body.md` | Rigid body API, key attributes, attaching a collider (see colliders), mass (implicit vs explicit), dynamic vs kinematic. |

More topics (joints, articulations, materials, instancing) will be added here as
separate reference files.

## Validation

Populate the authored file into an `ovstage.Stage`, seal the write ordinal,
attach the stage to ovphysx, and step it; a dynamic body above the ground should
fall and come to rest. Use the synchronous populate/seal/attach/step/destroy
pattern in the `basic-workflow` skill (Python and C), replacing its input path
with the file you authored.

## References

- Reference files: `references/scene_setup.md`, `references/collision.md`, `references/rigid_body.md`
- Tested scenes (installed under `samples/data/`):
  `tests/data/simple_physics_scene.usda`, `tests/data/basic_simulation.usda`
- Runtime populate/attach/step: `tests/python_samples/hello_world.py` (installed in the
  wheel only as `samples/python_samples/hello_world.py`; SDK ships the C
  equivalent `samples/c_samples/hello_world_c/main.c`); `basic-workflow` skill
- Omni Physics documentation: https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html
