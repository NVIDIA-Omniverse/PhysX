<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-UTILS-001
title: USD Physics Authoring Utilities
status: implemented
owner: ovphysx
---

## Description

ovphysx exposes a pure-Python `ovphysx.utils` subpackage of USD physics authoring
helpers. It closes a gap in the Python surface: `ovphysx.api.PhysX` simulates a
stage but nothing in the package helps build one, so a consumer authoring a scene
against the wheel has to spell out the `UsdGeom` definition, the xform op stack,
the collision API, the rigid body API and the mass API by hand for every prim.

The helpers are recovered from the `omni.physx.scripts` package that the Kit
extension used to ship. That package was deleted from the repository when the
non-ovphysx ovruntime Python bindings were retired, so it is not a live source
tree that this subpackage duplicates. Only the utilities that work without an
Omniverse Kit runtime are revived, and of those, only the ones worth a place on
the surface.

That leaves three sets of names behind rather than one, and they are kept apart
because the ground for each is different. Of the 187 public top-level names that
package exposed, **129 are on the surface**, **3 were recovered and kept but are
not public**, **38 were never recovered** because they cannot work outside Kit or
carry no physics content, and **17 were recovered and then dropped** on a
usefulness re-audit. Those four figures account for all 187 with nothing left
over, and AC-2 accounts for the eight further names in the flat surface that are
not recoveries. "The excluded surface" below itemizes the 38, "The dropped
surface" the 17 and "The privatized surface" the 3, giving the ground per table
row rather than per category -- a row may cover several names that share one
ground -- and AC-9 pins all three sets as a testable criterion, so no decision
can be undone silently.

A fourth set sits outside that accounting rather than inside it. The 187 is a
census of one commit's surface, so a name that left the package earlier is not
among them to be classified: "Superseded before the census" below records the 46
such names a history sweep found, pins the nine of them worth pinning, and leaves
the 38 / 17 / 3 / 129 figures untouched, because none of the 46 was ever a
candidate for recovery.

What that excludes is the binding-based form of those helpers, not the
capability. `ovphysx.utils` is the home for supported ovphysx helpers that do not
belong on the top-level API, which is a wider remit than USD authoring alone, so
an equivalent written against ovphysx's own public surface is in scope even where
its `omni.physx` ancestor was dropped. AC-13 states the boundary such a helper
has to respect.

### The excluded surface

These are the 38 names that were never recovered. The test for inclusion is
"usable outside Kit", not "core": a helper is left behind here only where it
fails that test or has no physics content at all. Three grounds admit a name to
this table -- it acquired a removed C++ binding interface, it needed Kit itself,
or it has no physics content -- and the reason is recorded per table row rather
than per category, because a category alone does not say which replacement a
consumer should reach for. A row groups the names one ground covers, so the
thirteen binding accessors share a single sentence that applies to each of them,
while a name whose ground is its own gets a row of its own.

| Excluded | Origin | Why |
|---|---|---|
| `get_physx_interface`, `get_physx_simulation_interface`, `get_physx_cooking_interface`, `get_physx_cooking_private_interface`, `get_physx_scene_query_interface`, `get_physx_attachment_private_interface`, `get_physx_property_query_interface`, `get_physx_replicator_interface`, `get_physx_stage_update_interface`, `get_physx_statistics_interface`, `get_physx_visualization_interface`, `get_physx_benchmarks_interface`, `get_physxunittests_interface` | `ifaces.py` | Each acquires one of the `omni.physx.bindings._physx` interfaces that were removed with the bindings. ovphysx's equivalent is the `PhysX` class over the C ABI, not an accessor. |
| `new_memory_stage`, `release_memory_stage` | `utils.py` | Wrap `attach_stage` / `detach_stage` on the simulation interface. ovphysx owns stage attachment through its own public API. |
| `ExpectMessage`, `safe_import_tests` | `utils.py` | Kit test scaffolding. `ExpectMessage` drives `get_physxunittests_interface().start_logger_check*`; `safe_import_tests` exists to load `omni.physxtests` into the Kit test runner. |
| `get_initial_collider_pairs` | `physicsUtils.py` | Attaches the stage, steps the simulation and reads the contact report, all through the simulation interface, then decodes the reported collider ids with `PhysicsSchemaTools.intToSdfPath`, which was retired with the bindings. That decode has no port rather than no author yet: ovphysx reports contact identity as an opaque `ObjectKey` handle assigned by the runtime, not as a uint64-encoded `SdfPath` (ADR-0019), so an equivalent has to resolve its pairs through `PhysX.get_scene_query_paths_from_ids` and is a different function, not a recovered one. Both measured Isaac callers are Kit extensions, which could not import an ovphysx version of it either way. |
| `compute_conforming_tetrahedral_mesh`, `compute_voxel_tetrahedral_mesh` | `deformableUtils.py` | Thin wrappers over `get_physx_cooking_interface()`, and ovphysx's public C headers under `ovphysx/include/` expose no cooking entry point at all -- the single `cook`-matching name there is `OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY` in `ovphysx_types.h`, a cache-location setting rather than a call that cooks -- so there is nothing to rewrite them against. Authoring an auto deformable hierarchy does not need them, because the runtime cooks at simulation time; author-time cooking has no ovphysx equivalent. |
| `create_auto_deformable_attachment` | `deformableUtils.py` | Not recoverable in USD alone; see the note below. |
| `loadTetFile` | `deformableMeshUtils.py` | Resolves `${kit}` through `carb.tokens`, and is broken independently of Kit: it uses `os`, `re` and `carb` without importing any of them, so it raised `NameError` on every call. |
| `PhysxExtension` | `extension.py` | The Kit extension lifecycle object (`omni.ext.IExt`). There is no extension to start or shut down. |
| `Query`, `QueryManager`, `Result` | `propertyQueryRigidBody.py` | Built on `get_physx_property_query_interface().query_prim`. Mass and bounding-box introspection outside Kit needs a fresh design against the C API rather than a revival of the async callback wrapper. |
| `AssetFolders`, `get_server_path`, `get_asset_path`, `get_s3_web_path` | `assets_paths.py` | Locate the physics demo assets. `get_server_path` and `get_asset_path` read the Kit settings `physics/demoDevelopmentMode` and `/physics/testsAssetsPath`; all four address content ovphysx neither ships nor consumes. |
| `OV_PATH`, `S3_BUCKET`, `S3_PATH`, `S3_REGION`, `get_s3_upload_path` | `assets_paths_base.py` | The S3 bucket coordinates for that same demo-asset content and the upload path its sync script builds. Portable, but with no physics content and nothing in ovphysx to address. |
| `autoassign`, `ScopeGuard`, `get_all_submodules` | `pythonUtils.py` | Generic Python sugar with no physics content; `get_all_submodules` exists only to enumerate Kit extension submodules for the test loader. |

Being a convenience alias, a preset or a niche helper is *not* grounds for
exclusion from this table, since dropping one would break the port of a snippet
that used it. The `add_cube` / `add_collider_cube` / `add_rigid_cube` spellings
of the `add_box` family, the `add_pbd_material_water` and `add_pbd_material_viscous`
presets and `poisson_sample_mesh` are all therefore present. So are the voxel
tetrahedral mesh generators `create_tetra_voxels`, `create_tetra_voxel_box`,
`create_tetra_voxel_sphere` and `create_triangle_mesh_cube`, reimplemented without
`numpy` so that no authoring submodule declares a dependency the wheel does not
carry. The implementation uses list-backed grids in place of NumPy. For a
non-positive dimension it returns an empty mesh: the original allocated
its grid with `numpy.zeros((dimx, dimy, dimz))`, which raises `ValueError` for a
negative dimension, while the two normalized wrapper helpers divided by zero
after an empty zero-resolution grid. The rewrite gives both empty-grid cases one
result rather than making a caller distinguish two incidental exceptions, and
the four helper docstrings state that contract. The
table above is the whole of what could not be recovered: nothing is in
it for being small, redundant or unfashionable.

#### Why the attachment helper cannot be revived in USD alone

`create_auto_deformable_attachment` is the one exclusion that looks wrong at first
reading, so the mechanism is recorded rather than the category. Almost all of the
original is validation, a `UsdGeom.Scope.Define`, an
`ApplyAPI("PhysxAutoDeformableAttachmentAPI")` and two relationship sets -- already
codeless in style and reproducible with `pxr` alone. The runtime side looks
declarative to match: the load path discovers every prim carrying that API and
calls `updateAutoDeformableAttachment`
(`ovphysx/ovruntime/source/omni.physx/plugins/usdLoad/LoadStage.cpp:542`, into
`ovphysx/ovruntime/source/omni.physx/plugins/attachment/PhysXAttachment.cpp:2852`).

That function recomputes attachment data into prims that already exist, hashing
its inputs to skip the work when they have not changed. It never defines an
attachment prim. The child prims an attachment needs are created
exclusively by `setupAutoDeformableAttachment` (same file, line 2810), which the
runtime never calls on its own -- outside the plugin's own translation units it
is reachable only through
`ovphysx/ovruntime/include/private/omni/physx/IPhysxAttachmentPrivate.h` (line
221), wired up at
`ovphysx/ovruntime/source/omni.physx/plugins/PhysX.cpp:2711` -- and which is
written to create them all up front precisely so that none is created
asynchronously. Nothing under `ovphysx/include/` surfaces it: the private
attachment interface is not part of the ovphysx C ABI, and its only non-plugin
callers in the tree are the runtime's own native tests.

So a pure-USD helper would author an attachment root with no attachment prims
under it, and nothing in the load path would fill them in. The result is a stage
that looks authored and silently does not attach, which is worse than the absent
helper. Reviving it needs a public ovphysx entry point onto the setup call, not a
recovery of the Python wrapper; no such entry point exists.

### The dropped surface

These are the 17 names that were recovered, made to work, and then dropped on a
usefulness re-audit. Every one of them is portable -- none needs Kit or a removed
binding -- so none belongs in the table above, and the distinction is worth
keeping: an excluded name has no ovphysx equivalent to offer a consumer, while a
dropped name's behavior is either still reachable under a sibling's name or was
never ovphysx's to provide.

The line between this table and "The privatized surface" below is whether a
separately callable function survives. A dropped name leaves none: its logic may
survive absorbed into the caller that used it, but no function in the package
answers to it. A privatized name keeps the same function object under a
leading-underscore spelling, and a caller inside `ovphysx.utils` still reaches it
by name.

A fourth ground appears here that the excluded set does not use: **duplicate of a
sibling helper**. Alongside it are the same "no physics content" test as above,
read as *serves no physics purpose* rather than *names no physics schema*, and
the observation that a helper existed only to be an argument or an inlined step
of its single caller.

| Dropped | Origin | Why |
|---|---|---|
| `hasAPI` | `utils.py` | Duplicate of a sibling helper: the same predicate as `has_schema` with its two arguments swapped, differing only in spelling -- and in `hasAPI`'s loop running on after it matches instead of returning. The justification the recovery gave it, that `Usd.Prim.HasAPI` cannot see a codeless schema, is false: the delivered code calls `prim.HasAPI` with a schema identifier string throughout, including from `descendant_has_api` and `ancestor_has_api` in the same module. |
| `get_derived_schemas` | `utils.py`, as `getDerivedSchemas` | A one-line passthrough on the same ground as the four metadata accessors below, and the only reason it is not in their row is that what it passes through to is a `pxr` call rather than a `Usd.Object` one: its whole body is `[derived.typeName for derived in get_tf_type_compatible(schema).GetAllDerivedTypes()]`. The one thing it adds over `GetAllDerivedTypes()` is the identifier-to-`TfType` resolution its own public sibling `get_tf_type_compatible` already offers beside it, so a caller who wants derived type names writes the comprehension. No caller inside the subpackage, and none measured outside it. |
| `add_tetra` | `deformableMeshUtils.py`, as `addTetra` | Three `list.extend` calls appending one tetrahedron's four unshared vertices. No schema, no token and no invariant of its own beyond "vertices are unshared", and **no caller anywhere in the tree** -- not in `utils/`, not in the tests, not in the samples, not in the docs. Its near-twin `add_triangle` has one caller and is privatized rather than dropped for exactly that reason. The measured Isaac occurrences are inside Isaac's own vendored copy of the module, which defines and calls its own. |
| `HALF_PI` | `utils.py` and `physicsUtils.py` | A constant in `constants.py` that spells neither a PhysX schema token nor a schema-derived limit. `math.pi / 2` is the whole of it, its own documentation restated the value rather than claiming anything for it, and it had no caller, no test and no measured consumer. The original declared it twice, once in each origin module. |
| `voxel_sphere_test`, `voxel_pass_all_test` | `deformableMeshUtils.py` | Occupancy predicates that existed only to be passed to `create_tetra_voxels` by `create_tetra_voxel_sphere` and `create_tetra_voxel_box`. Each is now a local function in the one helper that used it, so the behavior is unchanged and the surface is two names smaller. |
| `isDefined` | `utils.py` | A prim-exists check that also logged a warning, called from one place. Inlined into `add_physics_scene`, warning included; `Usd.Stage.GetPrimAtPath(...).IsValid()` is what a caller wanting the check alone would write. |
| `get_default_particle_system` | `particleUtils.py` | Find-or-create for a particle system, called from one place. Inlined into `poisson_sample_mesh`, which is the only helper that needs it and which returns a path rather than a prim, so the typed `PhysxSchema.PhysxParticleSystem` return the original promised had nothing left to mean once the schemas went codeless. |
| `has_custom_metadata`, `get_custom_metadata`, `set_custom_metadata`, `clear_custom_metadata` | `utils.py` | One-line passthroughs to `Usd.Object.HasCustomDataKey`, `GetCustomDataByKey`, `SetCustomDataByKey` and `ClearCustomDataByKey`, with no physics content: the metadata key is the caller's. The physics-specific pair over the same two USD calls, `set_local_space_velocities` and `clear_local_space_velocities`, is on the surface and is what a consumer authoring `physics:localSpaceVelocities` wants. |
| `explodeTriangleMesh`, `explodeTetraMesh` | `deformableMeshUtils.py` | Push each triangle or tetrahedron away from the centroid for an exploded-view render. No physics purpose: nothing simulates, collides with or is authored from the result. |
| `CameraTransformHelper` | `utils.py` | Caches a prim's world transform and reports its right / up / forward vectors for a viewport camera. No physics purpose, and `get_basis` and `get_forward_vector` remain for the basis math a joint or collider frame needs. |
| `get_spatial_tendon_parent_link`, `get_spatial_tendon_attachment_candidates` | `utils.py` | Populate the parent-link picker in Kit's property window. They run without Kit, so they are not excluded above, but the only thing they are for is a UI ovphysx does not have; a consumer authoring tendons reads the relationship directly. |

The `explode*` pair is the one row where a fifth ground was considered and
rejected. "Outside ovphysx's remit" would fit it, but it is the third ground
restated: what makes those two mesh helpers unlike the tetrahedral math that
stays -- `calculate_tetra_volume`, `verify_tetra_mesh`, `convert_tetra_to_triangle_soup`
-- is not that they touch no schema, since neither do those, but that nothing
downstream of them is simulated. Reading "no physics content" as the purpose the
helper serves rather than the API it calls covers both rows and keeps the grounds
at four.

Six of the seventeen had an in-tree caller, and every one of those callers was
rewired rather than dropped with it: `isDefined` came out of `add_physics_scene`,
the two occupancy predicates out of `create_tetra_voxel_box` and
`create_tetra_voxel_sphere`, `get_default_particle_system` out of
`poisson_sample_mesh`, and `set_custom_metadata` and `clear_custom_metadata` out
of `set_local_space_velocities` and `clear_local_space_velocities`. All six
callers are still on the surface and still do what they did, as is `has_schema`,
which is the sibling `hasAPI` duplicated. AC-9 pins those seven present, because
a set of absence assertions would pass equally well if a caller had been deleted
alongside the name it used.

### The privatized surface

These are the 3 names that were recovered, are still needed, and are no longer
public. Each has exactly one caller inside `ovphysx.utils` and no measured caller
outside it, so its behavior is load-bearing while its place among the flat helper
names is not. Each keeps its body under a leading-underscore spelling in the
submodule that defined it, and leaves `__all__`. "The dropped surface" above
states the line between the two tables.

A privatized name is still **useful** in the epic's term, so it stays inside the
census's recovered-and-kept count while leaving the flat surface, and
"Reconciliation" below splits that count into a public and a private part.

A helper with a measured external consumer stays public however thin it is,
which is what keeps the `add_cube` trio and the static-collider pair on the
surface; what lands here is the case in between, needed inside and unclaimed
outside.

| Privatized | Now spelled | Origin | Why it is not public |
|---|---|---|---|
| `create_unused_path` | `paths._create_unused_path` | `utils.py` | One caller, `joints.create_joint`. No test, sample, doc reference or measured consumer. It also disagrees with its own public sibling on the uniquifier convention -- a bare `0` / `1` / `2` suffix over concatenated strings where `get_stage_next_free_path` uses `_01` over an `Sdf.Path` -- so a caller reaching for both got two naming schemes out of one module. `get_stage_next_free_path` is the one a consumer wants and stays public: it has ten cross-module callers and encodes the increment convention, the default-prim reprefixing and a documented relative-path crash workaround. |
| `add_triangle` | `mesh._add_triangle` | `deformableMeshUtils.py`, as `addTriangle` | One caller, `mesh.convert_tetra_to_triangle_soup`. Three `list.extend` calls, no schema, no token and no invariant beyond "vertices are unshared". Public only because Python has no privacy -- the same ground that dropped `voxel_sphere_test` and `voxel_pass_all_test`, and it would be dropped too but for the caller. |
| `get_default_particle_system_path` | `particles._get_default_particle_system_path` | `particleUtils.py` | One caller, `particles.poisson_sample_mesh`. A `get_*` name that mutates the stage: it defines `/World` and makes it the stage default prim when there is none. Its sibling `get_default_particle_system` is in "The dropped surface" above on exactly that ground -- "silently defines `/World` and sets the stage default prim" -- and the two differ only in that this one still has the caller that one lost. |

None of the three is renamed beyond the underscore, and none changes behavior, so
`poisson_sample_mesh` and `create_joint` author exactly what they authored before.
AC-9 pins all three as absent from the public surface and pins each private
spelling present and callable; the ground for that second assertion is stated
there.

### Superseded before the census

The three tables above are scoped to the 187 names the census measures, which is the
public surface of `omni.physx.scripts` at `e7222e50c6^` -- the commit before the
package was removed. A name that left the package earlier is invisible to that
census by construction, however loudly it was once part of the surface, and a
sweep of the package's whole history found 46 such names (the appendix at the end
of this section records them, with the method and its limits). Nine of the 46 are
pinned here as a fourth list, `SUPERSEDED_NAMES` in `test_utils_surface.py`. They
are neither excluded, dropped nor privatized: they were never candidates for
recovery, because they no longer existed to recover.

What the nine have in common is stronger than being old. All nine were removed on
2026-03-20 by `233417d51a` ("OMPE-18178: Remove deprecated deformable schemas,
phase I"), all nine carried deprecation language in their final source --
`TetMeshData` and five siblings under a `# DEPRECATED` comment in
`deformableUtils.py`, the three particle-cloth helpers under a docstring reading
"DEPRECATED: Will be replaced by new deformable implementation in future release"
-- and every API schema any of them applies is gone from the schemas ovphysx
ships. `PhysxDeformableAPI`, `PhysxDeformableBodyAPI`,
`PhysxDeformableSurfaceAPI`, `PhysxDeformableBodyMaterialAPI`,
`PhysxDeformableSurfaceMaterialAPI`, `PhysxParticleClothAPI`,
`PhysxAutoParticleClothAPI` and the `TetrahedralMesh` prim type appear in neither
`schemas/physx/source/physxSchema/schema.usda` nor its `generatedSchema.usda`.
The one `PhysxDeformableBodyAPI` match in `schema.usda` is a comment recording a
naming clash with the legacy schema, not a declaration. So a recovered version of
any of the nine would not merely be unsupported: it would raise from
`codeless.apply_api`, because the identifier it applies names nothing in the
registry. That is the test the list rests on, and it is why "superseded" rather
than "dropped" is the right word -- a dropped name's behavior is still reachable
under a sibling's name in this package, while these nine have no portable form at
all.

Replacements, where they are recorded, come from the migration guide that shipped
with the removal, `e7222e50c6^:omni/ovexts/dev_guide/deformables/deformable_migration.rst`,
under "Deformable Utils". Four of the nine have a row there; the other five do
not, and the difference is stated rather than smoothed over.

| Superseded | Module at removal | Replacement, and why it is pinned |
|---|---|---|
| `add_physx_deformable_body` | `deformableUtils.py` | Guide row: → `set_physics_volume_deformable_body` for single-prim setups, and `create_auto_volume_deformable_hierarchy` for hierarchies that want simulation and collision meshes generated. Applies `PhysxDeformableAPI` and `PhysxDeformableBodyAPI`, both absent from the shipped schemas. |
| `add_deformable_body_material` | `deformableUtils.py` | Guide row: → `add_deformable_material`. Applies `PhysxDeformableBodyMaterialAPI`, absent. |
| `add_physx_particle_cloth` | `particleUtils.py` | Guide row: → `set_physics_surface_deformable_body`, and `create_auto_surface_deformable_hierarchy` for hierarchies. The guide's own note is worth carrying: the port is *not* behavior-preserving, because particle cloth was a PBD mass-spring system and surface deformables are XPBD FEM corotational linear elasticity, so the two do not share a physical model. Applies `PhysxParticleClothAPI`, absent. |
| `add_physx_particle_cloth_with_constraints` | `particleUtils.py` | Guide row, mapped explicitly to "N/A". There is no replacement; the spring constraints it authored have no counterpart, since a surface deformable's physical properties come from its material and the rest-shape attributes of its simulation mesh. Applies `PhysxParticleClothAPI` and `PhysxAutoParticleClothAPI`, both absent. |
| `add_physx_deformable_surface` | `deformableUtils.py` | No guide row. Pinned on the schema test -- it applies `PhysxDeformableAPI` and `PhysxDeformableSurfaceAPI`, both absent -- and on being the sibling a porter meets in the same file, two definitions below `add_physx_deformable_body`. No documented demand is claimed for it. |
| `add_deformable_surface_material` | `deformableUtils.py` | No guide row. Same two grounds: applies `PhysxDeformableSurfaceMaterialAPI`, absent, and sits immediately after `add_deformable_body_material`, whose replacement the guide does record. |
| `TetMeshData` | `deformableUtils.py` | No guide row. A container for tetrahedral-mesh point and index arrays, and the first `# DEPRECATED` marker in the file. Pinned as the type the two volume helpers passed around, so a porter reading their old call sites meets it before it meets them. |
| `create_skin_mesh_from_tetrahedral_mesh` | `deformableUtils.py` | No guide row. Authors a `TetrahedralMesh` prim, a type absent from the shipped schemas, so it cannot be recovered even codeless. |
| `create_spring_grid` | `particleUtils.py` | No guide row. Builds the spring topology `add_physx_particle_cloth_with_constraints` consumed, which the guide maps to "N/A", so it is superseded by the same absence one step upstream. |

Two exclusions from the nine are worth a sentence each, so the choice is legible.
`deformable_deprecated_on` was removed by the same commit, but it is a
`carb.settings` read used to pick an asset folder, not an authoring helper: its
real ground is the one "The excluded surface" already gives for names that
acquired a removed binding interface, and putting it here would blur what the
list is for. The other 36 losses in the sweep are older and thinner --
`setKinematicBody`, `addPBDMaterial`, `unapplyAPISchema` and their generation --
and none of them has a single occurrence anywhere in the measured Isaac trees, so
pinning them would assert absence no consumer can notice.

One live call site remains, and it does not change the decision. IsaacLab `main`
still calls `add_physx_deformable_body` at
`source/isaaclab/isaaclab/sim/schemas/schemas.py:963`. That call site is legacy
regardless of ovphysx: the enclosing function guards on
`PhysxSchema.PhysxDeformableBodyAPI` and reaches for `PhysxDeformableAPI`, and
`233417d51a` removed both, so it is already broken against any build carrying that
commit. Isaac Sim 6.0.1-rc.7 and IsaacLab `release/3.0.0-beta2` have both migrated
to the replacements this document retains, which is the direction of travel a
recovered version would have been arguing against.

### How the classification maps to the epic's terms

OMPE-96497 asks for the `omni.physx` Python utilities to be classified as
**useful**, **deprecated**, **obsolete** or **not appropriate for ovphysx**. The
sections above sort them by what forced each decision, which is the more useful
axis for a consumer looking for a replacement, so the same 187 names are restated
here in the epic's four terms. Every name lands in exactly one:

| Epic term | Count | Which names |
|---|---|---|
| useful | 132 | Everything recovered and kept, whether or not it is public: the 129 on the flat surface -- that surface less the seven additions AC-2 names and `METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES`, which came from the bindings rather than from `omni.physx.scripts` -- plus the 3 of "The privatized surface", which are kept and called and only their public spelling is withdrawn. Usefulness is a property of the behavior, so a privatization does not move a name out of this row. |
| deprecated | 0 | Empty as an *inherited* status; see below. Not to be confused with the eight `camelCase` aliases AC-15 keeps, which this surface deprecates on its own initiative rather than finding deprecated. |
| obsolete | 12 | From "The dropped surface": `hasAPI`, `get_derived_schemas`, `HALF_PI`, `add_tetra`, `voxel_sphere_test`, `voxel_pass_all_test`, `isDefined`, `get_default_particle_system`, `has_custom_metadata`, `get_custom_metadata`, `set_custom_metadata` and `clear_custom_metadata` -- each superseded by a sibling helper, absorbed into its only caller, or a one-line passthrough to the expression that supersedes it. |
| not appropriate for ovphysx | 43 | All 38 of "The excluded surface", plus `CameraTransformHelper`, `explodeTriangleMesh`, `explodeTetraMesh`, `get_spatial_tendon_parent_link` and `get_spatial_tendon_attachment_candidates`, which run outside Kit but serve a viewport, an exploded-view render or a property window that ovphysx does not have. |

**`deprecated` is empty on the evidence, not by omission.** The question that row
answers is which of the 187 original names arrived already marked as on their way
out, not which names this surface itself deprecates -- the eight aliases of AC-15
are the latter, and they are aliases for renamed helpers rather than members of
the 187. The whole
`omni.physx` Python package at the commit before its removal contains no
`@deprecated` decorator, no `DeprecationWarning`, and not one occurrence of the
string "deprecat" in any case -- checked across every file in it, not sampled.
Nothing in it was marked as on its way out, so no recovered helper inherited that
status and none of the four buckets above could honestly be that one.

**That is true of the audit commit, and only because the deprecated helpers had
already gone.** The string is absent there because `233417d51a` (2026-03-20)
removed every helper carrying it three months earlier: nine names whose final
source read "DEPRECATED: Will be replaced by new deformable implementation in
future release". Read as a statement about the 187 the census measures, the zero
is exact -- none of them arrived deprecated, because a deprecated one did not
survive to be counted. Read as a statement about the package's history it would
be wrong, and "Superseded before the census" above is where that population
actually went: nine of those names are pinned there, with the replacements the
migration guide records. Both sentences are needed, and neither replaces the
other.

One appearance of the word is worth a reader's attention even though it does not
change that. Isaac Sim carries the tet-mesh helper family in
`source/deprecated/isaacsim.core.utils/python/impl/deformable_mesh_utils.py`, a
module whose docstring reads "Deprecated deformable mesh utility functions".
Five of the names it holds -- `explodeTriangleMesh`, `explodeTetraMesh`,
`voxel_sphere_test`, `voxel_pass_all_test` and `loadTetFile` -- are ones this
surface also excludes, so the two judgements agree independently. That module
opens with `from omni.physx.scripts import deformableUtils`, so it is already
broken against any build after that package was removed.

It stays a third party's label on a third party's copy, applied for reasons
internal to their tree: it corroborates the exclusions above rather than
governing them, and none of the helpers kept here inherits that status from it.
It is recorded because it is the only evidence-backed appearance of the word
anywhere near this surface, so a reader who finds it should know it has already
been weighed.

### Reconciliation

Two identities have to close, one over the census and one over the delivered
surface, and they close on different populations. They are written out because
the privatized names are the reason they no longer read the same count twice.

**Census side -- every one of the 187 lands in exactly one bucket:**

| | Count |
|---|---|
| Recovered and kept, public | 129 |
| Recovered and kept, not public -- "The privatized surface" | 3 |
| Never recovered -- "The excluded surface" | 38 |
| Recovered then dropped -- "The dropped surface" (12 obsolete + 5 not appropriate) | 17 |
| **Total** | **187** |

The epic's four terms restate the same 187 as 132 useful, 0 deprecated, 12
obsolete and 43 not appropriate. The 132 useful is the first two rows together --
129 + 3 -- because a privatized name is still useful; the 43 is the 38 excluded
plus the five dropped names that serve a viewport, an exploded-view render or a
property window.

**Surface side -- the 138 flat helper names decompose as:**

| | Count |
|---|---|
| Recovered from `omni.physx.scripts` and public | 129 |
| From the retired bindings rather than the scripts package: `METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES` | 1 |
| Additions the codeless conversion introduced: `TOKEN_TRIANGLE_MESH`, `TOKEN_SDF`, `TOKEN_SPHERE_FILL`, `SCENE_UPDATE_TYPE_SYNCHRONOUS`, `SCENE_UPDATE_TYPE_ASYNCHRONOUS`, `SCENE_UPDATE_TYPE_DISABLED` | 6 |
| Additions admitted on AC-13's own terms: `step_and_write_to_ovstage`, `OvStageOutputCache` | 2 |
| **Flat public helper names** | **138** |

So `138 - 7 - 2 = 129`, matching the census's first row rather than its
useful count. Adding the fifteen submodule names to the 138 gives the 153
`__all__` entries AC-2 states. The eleven `codeless` names are outside both
figures, the eight deprecated aliases are outside them too -- an alias is an
alias for a name already counted -- and so are the three privatized names.

Function names are `snake_case`; classes such as `OvStageOutputCache` use
`UpperCamelCase`. The recovery originally carried the original's
inconsistent mix of `camelCase` and `snake_case` over verbatim, on the argument
that existing Isaac Sim and IsaacLab code and the published body of
`omni.physx.scripts` snippets would then port by changing only the import line.
That argument was measured against the public `isaac-sim/IsaacSim` (`main`,
6.0.1-rc.7) and `isaac-sim/IsaacLab` (`release/3.0.0-beta2`) trees and did not
survive:

- **13 attributed call sites** exist across both repositories, touching **8 of
  the 51** camel names, in **5 files**. The other 43 camel names have no external
  consumer at all.
- **No** documentation, tutorial, sample or standalone example in either tree
  uses any of them.
- Those consumers are already snake-leaning: roughly **60%** of their calls into
  `omni.physx.scripts` use `snake_case` names. Verbatim naming was therefore
  never a one-line migration -- it only propagated the original's inconsistency
  into a new surface.
- The call sites are nonetheless **live**. Isaac Sim pins omni physics
  `110.1.13` (commit `c38f7d1e`, 2026-06-04), which predates the deletion commit
  `e7222e50c6`, so the rename lands on working code.

The 51 camel names are therefore spelled conventionally -- `setCollider` is
`set_collider`, `createAPISchemaPropertyCache` is
`create_api_schema_property_cache` -- and the eight with measured consumers keep
a deprecated alias under the old spelling so that the live call sites above are
not broken without warning. AC-15 states that alias surface and bounds it at
those eight; nothing else keeps a camel spelling.

The compatibility argument binds the names, not the defects. Where a recovered
helper was demonstrably broken, it is fixed rather than reproduced bug-for-bug,
because a revived helper that silently misbehaves is worse than one that never
shipped: the consumer has no reason to suspect it. The affected helper's
docstring states the contract the correction leaves and names the sibling that
provides the old behavior where there is one, so a caller porting a snippet
reads what the helper does now; the account of each deviation lives in AC-12.

The subpackage is organized by theme rather than mirroring the original file
split, and re-exports the authoring surface flat from `ovphysx.utils`, so both
the submodule path and the flat name reach every authoring helper. One submodule
is held back from that flattening: `ovphysx.utils.codeless` is reached only
through its own name, because its helpers are spelled generically enough
(`apply_api`, `set_attr`, `get_attr`) that a flat `ovphysx.utils.set_attr` would
read as part of the authoring surface rather than as low-level schema access.
AC-2 states the carve-out.

Importing the subpackage must not load the ovphysx native library, contact a USD
resolver, or start a simulation. It needs `pxr` and the standard library only, so
it stays usable in a process that only authors USD and never instantiates
`PhysX`. Diagnostics go through the standard `logging` module, replacing the
`carb` logging the original code used.

`pxr` is required to *use* an authoring helper rather than to import the
subpackage: submodules load on first attribute access, so a process with
`ovstage` and no USD can still reach `simulation`. AC-1a states this and records
why it has to hold.

`pxr` is deliberately not declared as an ovphysx
dependency: an authoring process supplies its own USD runtime, matching what
`ovphysx.schemas` and the bundled `codeless_schemas` sample already do.
Consequently `ovphysx.utils` is reached through an explicit
`import ovphysx.utils` and is not imported from `ovphysx/__init__.py`, so
`import ovphysx` keeps working for simulation-only consumers that have no `pxr`.
"Why `pxr` is caller-supplied" below records what forces that and what it costs.

ovphysx ships the PhysX and Omni deformable schemas as *codeless* artifacts, so
there is no compiled `PhysxSchema` Python module and no typed
`PhysxSchema.PhysxRigidBodyAPI` class. Every PhysX-specific call is therefore
expressed in codeless form: API schemas are applied by schema identifier and
their properties authored by name, and concrete PhysX prim types are defined by
type name. Core `UsdPhysics` schemas keep their typed bindings. The codeless
failure modes are legible only with
help -- `ApplyAPI` on an unregistered schema set raises an error naming a
pxr-internal function, authoring a property whose owning API was never applied
fails on a name USD never declared, and a registration arriving after the schema
registry was built fails with no error at all -- so the subpackage centralizes
codeless access in one module that reports each of them with an actionable
message. "Why PhysX schemas are reached codeless" below records what forces that
and the typed-facade alternatives it rules out.

The third of those is the one where "actionable" is hardest to honor, and AC-14
states what it takes. USD's schema registry is built once, on first access, and
has no reload, reset or refresh, so a registration that arrives after it cannot
be made to work by anything the same process can do: `RegisterPlugins` still
reports the plugin roots as registered and `Tf.Type.FindByName` still resolves
the schema types, while no schema applies. Advice that sends the caller back to
`register_schemas()` is therefore worse than none, and the only two routes that
can succeed are reordering the call ahead of the first stage or registry access
in a fresh process, and presetting `PXR_PLUGINPATH_NAME`, which USD reads while
constructing the registry rather than afterwards.

The `custom_execute_fn` / `execute_command_fn` hook that the original `set*` and
`remove*` helpers carried is not revived. It existed only to route schema
application through `omni.kit.commands` for undo, its contract passed the typed
schema class as `api=`, and no caller outside the Kit command layer supplied it.

### Why PhysX schemas are reached codeless

The forcing constraint is that ovphysx ships the PhysX and Omni deformable
schemas **data-only**. The schema source sets `skipCodeGeneration = true`, and
ovphysx's schema export stages only the registration data -- `plugInfo.json` and
`generatedSchema.usda` -- into the wheel. Consequently `pxr` in an ovphysx
environment exposes no `PhysxSchema` module at all: `from pxr import
PhysxSchema`, the only import form any recovered module used, raises
`ImportError` at the import line. Core `UsdPhysics` is unaffected, because its
typed bindings are part of stock `usd-core`. That data-only export is the
position and not an accident of what the export happens to copy: ovphysx ships
no typed `PhysxSchema` facade, compiled or pure-Python, and depends on none.

Reaching the schemas through raw `prim.ApplyAPI("...")` and the generic property
API, with no helper module at all, was the other alternative rejected here: it
would leave all three of the failure modes above at every call site rather than
translating them in one place.

**The pure-Python facade, stated at its strongest.** This is the alternative most
likely to be re-proposed, so it is recorded with the measurements rather than
only the conclusion; the behaviours below were executed against stock `usd-core`
riding on ovphysx's own schema registration, and the generated-output instability
was checked against the facade revision in the pinned schema dependency package.
Such a facade exists, works, and is already in ovphysx's dependency closure: it
is generated in this repository, committed beside the schema source, and shipped
under `lib/python/` in that package, beside the very registration data ovphysx's
export is derived from. It re-creates the typed surface (`Apply`, `Get`,
`Get<Attr>Attr`, `Create<Attr>Attr`, `Tokens`) over `prim.ApplyAPI` and named
attribute access, and on stock `usd-core` it produces **byte-identical USD** to
the codeless form, multiple-apply instance naming included -- which is what makes
its rejection non-obvious. Adopting it would have cost roughly one copy step in
the export. It loses for three reasons, none of which is "it would not work":

1. **It does not restore the import line users copy.** The facade is a
   *top-level* `PhysxSchema`, not a `pxr` submodule, so `from pxr import
   PhysxSchema` still raises `ImportError`. It would fix ovphysx's own call-site
   spelling while leaving a copied user snippet broken at line 1. This reason
   survives the rename above: a snippet's `from pxr import PhysxSchema` line has
   nothing to do with how this surface spells its own helpers, and it is the
   line that fails first.
2. **Its classes are not registered `TfType`s.** Handed a facade class,
   `prim.HasAPI(cls)`, `prim.RemoveAPI(cls)` and `prim.IsA(cls)` all raise
   `TypeError`. That covers 8 of the 43 executable `PhysxSchema.` call sites in
   the recovered code, so a facade-based port would have needed the codeless
   spelling at exactly those sites anyway -- and it converts a clean import-time
   failure into a late runtime one whose message does not hint that a codeless
   schema is the cause.
3. **It is unstable generated output.** The revision in the pinned dependency
   package lacks the compatibility check the current generator emits, so
   `bool(PhysxSchema.PhysxParticleSystem(prim))` returns `True` for *any* prim.
   One recovered particle helper validated a prim with exactly that idiom, so a
   facade-based port pinned at the revision ovphysx actually consumes would have
   shipped a silent validation bug. The delivered helper uses an explicit
   type-name check instead.

**Depending on the separately published PhysX USD schema wheel** loses for all of
the above plus one more: it carries a second copy of `generatedSchema.usda`. USD
dedupes plugins **by name**, first registration wins silently, and the loser does
not appear in the plugin registry at all -- so a divergence between the two copies
resolves by import order, with no diagnostic and no way to observe it.

**Two arguments against the facade are false**, and are recorded so they do not
return. A typed facade call does *not* silently
no-op: with the schemas unregistered,
`PhysxSchema.PhysxRigidBodyAPI.Apply(prim)` raises the *same* `Tf.ErrorException`
that `ovphysx.utils.codeless` exists to translate. (The real silent-authoring
hazard nearby is different, and is standard USD behaviour with compiled bindings
too: calling `Create<Attr>Attr()` without `Apply` authors a property the runtime
ignores, which is the one case `codeless.set_attr` uniquely catches.) And the
facade's broken inherited-attribute enumeration -- `GetSchemaAttributeNames`
reporting only a class's own attributes -- does not affect ovphysx, whose schema
introspection reads `Usd.SchemaRegistry` prim definitions directly and so
composes inherited properties either way.

**Where the deformable pose attributes stay raw.** Seven property writes in
`ovphysx/python/ovphysx/utils/deformable.py` author
`deformablePose:default:omniphysics:purposes` and
`deformablePose:default:omniphysics:points` through `Usd.Prim.CreateAttribute`
rather than through `codeless.set_attr`, and that is a decision rather than an
oversight. Both properties *are* declared by `OmniPhysicsDeformablePoseAPI` in
the shipped `generatedSchema.usda`, so `codeless.set_attr` would resolve them
once the API is applied; the two spellings are equivalent on the happy path.
They differ when the application does not happen. Those writes sit in the
bind-pose pass that `create_auto_volume_deformable_hierarchy` and
`create_auto_surface_deformable_hierarchy` run over every point-based prim in an
arbitrary subtree, and that pass calls `codeless.try_apply_api` and deliberately
does not check its bool, because a prim USD refuses the API on -- one on a
non-editable layer, say -- is not grounds for abandoning the rest of the
hierarchy. `codeless.set_attr` would turn each such refusal into a
`CodelessSchemaError` and do exactly that, where `CreateAttribute` authors the
property anyway and leaves the hierarchy consistent for the runtime to pick up.
The originals authored these same seven writes with `CreateAttribute` too
(`e7222e50c6^:omni/ovruntime/source/omni.physx/python/scripts/deformableUtils.py`,
lines 386, 389, 392, 394, 620, 623 and 626), so they are carried over unchanged
and the recovered helpers keep the warn-and-continue contract their ancestors
had. AC-10's "authored by name" is satisfied either way: both forms write the
same property name to the same prim, and neither reaches for a typed
`PhysxSchema` class. The one other place the subpackage writes a property
without going through `ovphysx.utils.codeless` is
`apply_api_schema_property_cache` in `schema.py`, which replays a snapshot whose
names, value types and instance templates all came from the schema registry and
is deliberately generic over codeless and compiled schemas alike; it is not a
PhysX-specific call site in AC-10's sense. Those two are the whole of it.

**A deformable failure can leave authored state behind.** When one of these
helpers returns `False`, it does not unwind what it already authored. The two
`create_auto_*` helpers, both `set_physics_*` helpers and both material helpers
interleave authoring operations -- schema applications, prim definitions and
property writes -- with `return False`, so a caller that ignores the bool keeps
partially authored state. `add_deformable_material` can leave a defined
`UsdShade.Material` prim with neither its API applied nor its properties set.
Nor is every rejection early: a `UsdGeom.Camera` root passes
`create_auto_volume_deformable_hierarchy`'s `Imageable`-and-not-`Gprim` check,
and with `cooking_src_simplification_enabled` set it is then refused by
`add_auto_deformable_mesh_simplification`, which requires a `UsdGeom.Scope` or
`UsdGeom.Xform` -- after the body API and the cooking-source relationship are
authored. `remove_deformable_body` strips the deformable APIs and their
properties but leaves a generated simulation mesh prim in place by design, so it
is not a full undo. All of the above is inherited rather than introduced here,
and this requirement keeps it. One API stays behind by design and is not
inherited: a `UsdPhysics.CollisionAPI` whose ownership `remove_deformable_body`
cannot establish, on the terms AC-8 states. The inherited helper removed it
regardless, and preserving it is the correction. AC-8 states which loss that
direction accepts.

**What codeless access costs.** PhysX schema identifiers and property names are
**strings**, so a typo is a runtime error rather than an attribute error at
import, and no type checker can see it; the translated diagnostics AC-10 and
AC-14 require are the mitigation, and they are the reason
`ovphysx.utils.codeless` exists as a module. PhysX token constants have no
codeless equivalent, which is why the token names AC-2 lists are declared by this
surface rather than recovered from any ancestor spelling.
Registering the codeless schemas is process-global and one-way, so any test
process that registers them must own its own process rather than mutate a shared
registry.

### Why `pxr` is caller-supplied

Two constraints force it, and both are properties of how ovphysx is built rather
than preferences:

- **ovphysx's USD runtime is the py-less `ovstage` runtime, which exposes no
  `pxr` at all.** A bundled `usd-core` would therefore be a *second* USD library
  in the same process, each with its own singletons and plugin registry.
  ovphysx's packaging guidance already warns against installing `usd-core` beside
  it for that reason.
- **`usd-core` publishes no linux-aarch64 wheel**, while ovphysx's Python
  dependency lockfile is resolved with `required-environments` forcing
  linux-aarch64 to resolve. A declared `usd-core` dependency therefore does not
  merely go unused on that platform -- it cannot be locked at all.

Declaring it as an optional extra removes neither problem: the extra is still
resolved by the lockfile on every required environment, and a consumer who
installs it still ends up with two USD libraries in one process.

The accepted cost is that the authoring helpers are **not usable out of the
box**. A consumer who installs only ovphysx gets an `ImportError` on
`import ovphysx.utils`, and the documentation is the only thing that tells them
why. Testing a caller-supplied dependency requires supplying it but does not
require declaring it, so the Python *test* project declares stock pip `usd-core`
(REQ-PACKAGING-PYTESTUSD-001) and the suite runs against the test venv's copy in
its own pytest process, never beside ovstage; the test environment is therefore
not the environment a consumer installs. Where PyPI ships no `usd-core` wheel
(linux-aarch64) the test stage does not run this suite at all, so `ovphysx.utils`
has no test coverage on that platform.

One consequence is permanent rather than a gap to close: **the type-checked
surface of `ovphysx.utils` is limited by the same aarch64 constraint.** Type
checking covers `.pyi` files only, and runs in a dev dependency group that cannot
gain `usd-core` for the reason above, so any stub importing `pxr` would fail on
an unresolved import and the obvious fix is blocked. In practice the authoring
submodules can never be stubbed: they are inline-annotated Python covered by the
package's PEP 561 marker and are not type-checked at all. Only a stub that avoids
`pxr` entirely -- such as one for a native-facing, non-authoring helper -- can
exist in this subpackage, and AC-2 fixes where it lives, since the package
directory wins module resolution over a same-named sibling module file for
CPython and type checkers alike.

That combination -- annotations no gate here reads, shipped under a marker that
tells a consumer's checker to trust them -- means an annotation contradicting
its own function delivers a wrong answer rather than no answer. Six return-type
contradictions and two accepted-input contradictions were inherited verbatim and
are corrected: the four `transform.py` xform-op setters annotated to return an
op while answering `False` or `None` for a non-Xformable prim,
`create_particles_grid` annotated `typing.List[Gf.Vec3f]` while returning a
`(positions, velocities)` tuple, `add_physx_particle_system` annotated
`Usd.Prim` while its own docstring promises `None` on failure, and
`setup_transform_as_scale_orient_translate` and
`copy_transform_as_scale_orient_translate` annotated for `UsdGeom.Xformable`
alone although both implementations also accept `Usd.Prim` for every input.
There is also one implicit Optional. These are annotation
corrections and not behavior changes -- no runtime path is altered, and the
`False` / `None` asymmetry among the four setters is inherited and left as it
is, since either value is falsy -- so they are deliberately not counted among
the defects AC-12 records, which each carry a regression test that a
type-checker gate would be the only equivalent for. TEST-PYTHON-UTILS-001's
coverage gaps record what the absent gate costs.

### Appendix: the pre-census sweep

The 46 names "Superseded before the census" refers to, recorded here because the
evidence took a full history walk to produce and will not be cheaply
reconstructible once the package's directory paths are further from anyone's
memory. Nine of these are pinned in `SUPERSEDED_NAMES`; the rest are recorded
only here.

**Method.** The sweep walked all 378 snapshots of the package from its first
commit, `2b47564c00` (2020-03-21), to `e7222e50c6^`, through the five successive
directory paths the package occupied, and collected every public name that ever
appeared in the twelve audited files. 233 names existed at some point against the
187 that survived to the census. Consumer measurements were taken against Isaac
Sim `987015050efebfd0cd5d3736ae47fffe5adee308` and IsaacLab
`bffdce9d7467f349bfc8ab111fe633a0bb234851`, plus IsaacLab `main` at
`b0542fe2d45bf91c4e1d9ef6952b9c709c80b4e8`.

**Two honest bounds.** First, `git log` applies pathspec history simplification,
so a name that existed only on a TREESAME side branch of a merge could be
invisible to the walk; the 46 is a floor, not a proof of exhaustion. Second, the
figure depends on the file set: restricting to the twelve audited files gives 46,
while counting every `.py` the directory ever held -- test scaffolding, demos,
Kit-only command modules -- would give 160. The narrower number is the one this
document uses, because the wider one counts names that were never part of the
utility surface being recovered.

| Removed | Commit | Module at removal | Names |
|---|---|---|---|
| 2020-04-05 | `7d21c0f468` | `extension.py` | `Extension`, `get_extension` |
| 2020-04-28 | `56ed61ed52` | `utils.py` | `setKinematicBody` |
| 2020-05-28 | `5a353264b9` | `utils.py` | `setRigidBodySubtree` |
| 2020-10-06 | `a26f80b9ea` | `physicsUtils.py` / `utils.py` | `add_velocity`, `unapplyAPISchema`, `unapplyMultipleAPISchema` |
| 2020-11-20 | `1023586409` | `utils.py` | `addDefaultMaterials`, `defaultMaterials` |
| 2021-03-08 | `c23a54ad31` | `utils.py` | `switchEditorMenuValue` |
| 2021-05-28 | `c5ca158368` | `utils.py` | `addMaterial`, `addPhysicsMaterial` |
| 2021-06-02 | `5e47960475` | `physicsUtils.py` | `applyPhysxSoftBodyApi` |
| 2021-08-03 | `c7c204c823` | `physicsUtils.py` | `create_conforming_tetrahedral_mesh`, `create_voxel_tetrahedral_mesh` |
| 2021-12-21 | `5fc4add3bb` | `utils.py` / `physicsUtils.py` | `addDeformableBodyMaterial`, `addDeformableSurfaceMaterial`, `addDiffuseParticles`, `addIsosurface`, `addPBDMaterial`, `addPhysxParticleSystem`, `applyPhysxDeformableBodyApi`, `triangulateMesh` |
| 2022-02-18 | `57e1a4fc6a` | `particleUtils.py` / `physicsUtils.py` | `add_physx_isosurface`, `compute_particle_radius_from_mesh` |
| 2022-06-27 | `10e2cff5ed` | `utils.py` | `set_opposite_body_transform` |
| 2022-12-15 | `79de262b75` | `deformableUtils.py` | `applyPhysxFEMClothApi` |
| 2023-03-15 | `1c8baefb59` | `utils.py` | `getSchemaAttributeNames`, `getSchemaRelationshipNames` |
| 2023-03-20 | `79fcd0b869` | `utils.py` | `register_stage_update_node` |
| 2023-05-04 | `c4f54fd8d1` | `utils.py` | `familyHasConflictingAPI` |
| 2023-10-16 | `b7d1153dac` | `physicsUtils.py` | `resolveContactEventPaths` |
| 2025-01-31 | `b3d7b43491` | `utils.py` | `get_cooked_data_attributes`, `release_cooked_data` |
| 2025-03-07 | `de841bbf41` | `assets_paths.py` | `get_assets_path` |
| 2025-12-09 | `474554b39d` | `assets_paths.py` | `deformable_beta_on` |
| 2026-03-20 | `233417d51a` | `deformableUtils.py` | `TetMeshData`, `add_deformable_body_material`, `add_deformable_surface_material`, `add_physx_deformable_body`, `add_physx_deformable_surface`, `create_skin_mesh_from_tetrahedral_mesh` |
| 2026-03-20 | `233417d51a` | `particleUtils.py` | `add_physx_particle_cloth`, `add_physx_particle_cloth_with_constraints`, `create_spring_grid` |
| 2026-03-20 | `233417d51a` | `assets_paths.py` | `deformable_deprecated_on` |

The nine pinned names are the last two `233417d51a` rows less
`deformable_deprecated_on`, which the previous section explains. Every commit hash
and date in this table resolves in this repository as recorded.

## Acceptance Criteria

- AC-1: `import ovphysx.utils` and every submodule import succeed with only
  `pxr` and the standard library present. No import triggers ovphysx native
  library loading, and none requires `carb`, `omni.usd`, `omni.kit.*`,
  `omni.physx`, or any other Kit module at import time or call time.

- AC-1a: `pxr` is required only to reach an authoring name, not to import the
  subpackage. `import ovphysx.utils` and
  `from ovphysx.utils import OvStageOutputCache, step_and_write_to_ovstage`
  both succeed with no
  USD present at all, because submodules load on first attribute access.
  `simulation` is the submodule this holds for; every other one imports `pxr`
  at module scope and raises when first touched in such a process.
  A dependency-free manifest maps every known flat export to the submodule that
  owns it, and `__getattr__` imports only that owner. The manifest is checked
  exactly against every loaded owner's `__all__`, including ownership and
  duplicates, so changing either side without the other fails the surface test.
  This direct lookup is what lets the simulation helper resolve without first
  importing a `pxr`-dependent owner; it does not depend on submodule order.

  Module discovery holds there too, and deliberately asymmetrically.
  `dir(ovphysx.utils)` skips the submodules it cannot import and reports what is
  discoverable -- the fifteen submodule names plus `simulation`'s public names
  with no USD present, the whole flat surface where `pxr` is -- so `dir()` and
  module discovery answer instead of raising. That guarantee does not extend to
  interactive completion: a completer can call `getattr` on a returned submodule
  candidate and reach its missing `pxr` dependency. `help()` and pydoc are also
  outside the guarantee because pydoc reads `__all__`.
  `ovphysx.utils.__all__` stays strict and raises naming `pxr`, because
  it is what `from ovphysx.utils import *` binds and a partial star-import would
  hand a caller a surface silently missing helpers, whose absence then surfaces
  as a `NameError` on the helper rather than on the dependency. Only an
  `ImportError` is skipped, so a submodule failing for any other reason is
  reported rather than vanishing from `dir()`.

  The manifest also distinguishes a dependency failure from an absent name.
  Looking up a known flat export imports its owner and propagates that owner's
  `ImportError`, including through explicit `from` import, `getattr` with a
  default and `hasattr`, so a missing authoring dependency is not reported as a
  missing helper. A genuinely unknown ordinary or dunder name raises an
  unchained `AttributeError` without importing a flattened submodule, so
  `getattr` with a default and `hasattr` give their normal module-protocol
  answers for that name.

  This is load-bearing rather than a nicety. The `output_read` sample reaches
  the helper through `ovphysx.utils` in a venv that has `ovstage` and no USD,
  which worked while `ovphysx.utils` was a module file holding only that
  helper. Importing the fifteen submodules eagerly in `__init__` reintroduces
  `pxr` as a hard requirement of that import path and breaks the sample, and
  routing it at `ovphysx.utils.simulation` instead does not help, since
  importing a submodule runs the parent package's `__init__` first.

- AC-2: Every public helper is reachable both as `ovphysx.utils.<name>` and as
  `ovphysx.utils.<submodule>.<name>`, with one deliberate exception:
  `ovphysx.utils.codeless`'s eleven public names -- `apply_api`,
  `try_apply_api`, `set_attr`, `set_attrs`, `set_rel`, `get_attr`,
  `remove_api`, `register_schemas`, `schema_is_registered`, `instanced_name` and
  `CodelessSchemaError` -- are reachable *only* as
  `ovphysx.utils.codeless.<name>` and are deliberately not flat re-exported.
  `ovphysx.utils.__all__` lists the whole flat surface -- the fifteen submodule
  names plus the 138 flat helper names, 153 entries in all -- and contains no
  name absent from the modules.

  The dependency-free flat-export manifest names all 138 helpers and each
  owning submodule independently of those modules' `__all__` lists. The two
  representations are required to match exactly after the owners load, so a
  change to an owner's list fails unless its manifest entry follows, a
  coordinated deletion still fails the surface-size pin, and one name cannot
  appear under two owners.

  Functions in `__all__` use `snake_case`, constants use `UPPER_SNAKE_CASE`,
  and `OvStageOutputCache` uses the class convention `UpperCamelCase`; the 51 names the original spelled `camelCase` are renamed, and the
  eight deprecated aliases AC-15 keeps under the old spelling are deliberately
  not listed here. Eight exported names are additions rather than recoveries and
  so have no ancestor spelling at all: `TOKEN_TRIANGLE_MESH`, `TOKEN_SDF`,
  `TOKEN_SPHERE_FILL`, `SCENE_UPDATE_TYPE_SYNCHRONOUS`,
  `SCENE_UPDATE_TYPE_ASYNCHRONOUS` and `SCENE_UPDATE_TYPE_DISABLED`, which the
  codeless conversion introduced because the `PhysxSchema.Tokens.*` lookups they
  replace have no codeless equivalent, and `step_and_write_to_ovstage` and `OvStageOutputCache`, which
  AC-13 admits on its own terms. The remaining 130 are 129 names recovered from
  `omni.physx.scripts` itself plus
  `METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES`, which the original imported from
  `omni.physx.bindings._physx` and this subpackage spells out as a local
  constant; it arrived already conventionally spelled and so is untouched by the
  rename.

  Reachability has a filesystem precondition, and it is checked rather than
  assumed: `ovphysx.utils` resolves to the package directory `ovphysx/utils/`,
  and no `utils.py` or `utils.pyi` module file sits beside it. Such a file would
  not conflict at merge time and would not fail the build, but the package
  directory wins resolution for CPython and for type checkers alike, so the
  module file would never be read and every name only it declared would be
  unreachable. A stub for anything in this subpackage therefore lives inside the
  package, as `ovphysx/utils/<submodule>.pyi`.

- AC-3: Schema introspection resolves a schema type from a type-name string or a
  schema class, enumerates a schema's property names, attributes and
  relationships, applies multiple-apply instance naming, reports applied API
  schemas on a prim and its ancestors and descendants, and snapshots and restores
  an API schema's attribute and relationship values, including for
  multiple-apply schemas where a snapshot can be replayed onto a different
  instance. All of it works for codeless schemas, so it records a property's
  `Sdf` value type rather than the typed schema class the original resolved, and
  restores through the generic `Usd.Prim` property API. A snapshot records
  *authored* state only: an attribute carrying nothing but its schema fallback
  is recorded as unset and is left unauthored on restore, so a
  remove-and-restore cycle does not turn every attribute the API declares into
  an explicit layer opinion and does not detach the prim from later changes to
  those schema defaults.

  The relationship half holds to that same contract, and the one thing that
  differs is a property of USD rather than a choice: USD has no fallback for
  relationship targets at all -- `Usd.PrimDefinition` supplies fallback values
  for attributes only -- so there is no resolved default for the capture side to
  exclude. A fallback is not the whole of "unauthored", though. An untargeted
  relationship composes to an empty target list, and so does one authored with
  no targets, so the target list alone does not say which a prim has: the
  capture records `Usd.Relationship.HasAuthoredTargets()` too, storing `None`
  targets for a relationship with no opinion and the target list itself --
  empty or not -- for one with an opinion. The restore then skips `None` and
  replays an empty list, so a relationship that carried no opinion stays
  unauthored and one authored as `rel <name> = None` comes back.

  "Authored" is USD's own sense of the word for both halves: an opinion arriving
  over a reference, an inherit, a specialize, a variant or a weaker sublayer
  counts as authored and is recorded, for `HasAuthoredValue` and
  `HasAuthoredTargets` alike. The snapshot does not distinguish a local opinion
  from a composed one and is not required to.

- AC-4: Transform helpers set or create the `translate`, `orient` and `scale`
  xform ops on an Xformable, preserving the precision of an existing op and
  creating new ops at float precision; rewrite or copy a local transform as the
  physics-default scale-orient-translate stack while preserving
  `resetXformStack`; and report translation, world position, basis and
  axis-aligned vectors, unit scale factor, and the relative transform between a
  joint's two bodies. `setup_transform_as_scale_orient_translate` and
  `copy_transform_as_scale_orient_translate` accept either a `Usd.Prim` or a
  `UsdGeom.Xformable` for each input. A helper given a prim that is not an
  Xformable logs a warning and returns without raising.
  `setup_transform_as_scale_orient_translate` and
  `copy_transform_as_scale_orient_translate` perform every composed USD read
  outside an `Sdf.ChangeBlock`; neither encloses its read-and-write sequence in
  one, because composed stage state is not valid to inspect while change
  processing is deferred.

- AC-5: Authoring helpers create `UsdGeom` cube, sphere, capsule, cylinder, cone
  and xform prims at a collision-free stage path with a display color and an
  xform op stack, and their `add_collider_*` and `add_rigid_*` variants
  additionally apply `UsdPhysics.CollisionAPI`, and `UsdPhysics.RigidBodyAPI`
  plus `UsdPhysics.MassAPI` with the requested density and initial velocities.
  Further helpers apply mass, density and force APIs, define and bind physics
  materials, define a physics scene, apply and remove collider and rigid-body
  API sets over a prim or its subtree, manage collision groups and filtered
  pairs, create typed joints between two prims, and read and write the physics
  custom metadata keys, including `physics:localSpaceVelocities`.
  `add_plane_collider` refuses a target path already held by a prim with
  `ValueError` before changing the stage. The codeless `Gear` and
  `RackAndPinion` joint variants require their concrete prim definitions to be
  registered before `create_joint` changes the stage; otherwise it raises
  `CodelessSchemaError` naming the missing type.
  Applying the rigid-body API set to a prim that already has
  `UsdPhysics.RigidBodyAPI` completes the set with `PhysxRigidBodyAPI`, enables
  the body, and authors the requested kinematic state.
  `set_collider` leaves a prim untouched when its `omni:no_collision` attribute
  resolves to true; an authored false value does not opt out of collision.

- AC-6: Mesh helpers build `UsdGeom.Mesh` prims for a square, cube, concave
  shape, cylinder and cone with correct points, normals, face indices and face
  vertex counts; and compute triangulation, surface extraction from a
  tetrahedral mesh, signed tetrahedron volume, inverted-tetrahedron repair, tet
  mesh validation, tetrahedron-to-triangle-soup conversion, and a point set's
  bounding box diagonal. Any helper that took a `carb.Float3` sequence accepts
  any indexable three-component point, including `Gf.Vec3f`. Computing a
  bounding box diagonal from an empty iterable raises `ValueError`.
  `create_triangle_mesh_square`, `create_tetra_voxel_box`,
  `create_tetra_voxel_sphere` and `create_triangle_mesh_cube` return empty point
  and index lists when any integer grid dimension is non-positive.

- AC-7: Particle helpers define a `PhysxParticleSystem` prim and set only
  the attributes the caller supplied, author `UsdGeom.Points` and
  `UsdGeom.PointInstancer` particle sets, configure a particle set's system
  relationship, self-collision, fluid flag, group, mass and density, apply a PBD
  particle material, generate a particle position and velocity grid, and apply
  the anisotropy, smoothing, isosurface and diffuse-particle feature APIs.

  `add_physx_particle_system` requires the `PhysxParticleSystem` concrete prim
  definition to be registered before it changes the stage. If the schemas were
  never registered, or registration was attempted after USD built its registry,
  it raises `CodelessSchemaError` naming the type and leaves the stage unchanged
  rather than defining an inert unknown-type prim.

  All three particle-set constructors take a target path as either a `str` or an
  `Sdf.Path`, on the same terms as the rest of the subpackage.
  `add_physx_particleset_pointinstancer` required an `Sdf.Path`, because it
  built its prototype paths from `path.pathString` before converting where its
  two siblings convert first, so a `str` raised `AttributeError` out of the
  helper. It now converts first, as they do.

  This widens an accepted domain rather than correcting a behavior, so it is
  recorded here and not among AC-12's inherited defects, on the same footing as
  AC-6's statement that a helper which took a `carb.Float3` sequence accepts any
  indexable three-component point. Every call that worked before works
  identically and authors the same stage; nothing an existing caller can observe
  changes, and an observable deviation is what an AC-12 entry exists to carry.
  The inherited-defect total is therefore unmoved by it, at twenty-eight.

- AC-8: Deformable helpers apply the volume and surface deformable body API sets
  to a tetrahedral or triangular mesh, build the auto volume and surface
  deformable prim hierarchies, apply deformable and surface deformable
  materials, apply and remove the auto mesh simplification and hexahedral mesh
  APIs, and tear a deformable body down by removing its API schema properties
  from the root prim and every prim of its subtree. The subtree, not one level
  of children: the auto-hierarchy helpers apply
  `OmniPhysicsDeformablePoseAPI` to every `UsdGeom.PointBased` prim below the
  root at any depth, so a removal bounded to the children cannot undo what they
  author. Every prim of the range is torn down the same way, the supplied prim
  included: one prim can carry a body API, a simulation API, the auto-hierarchy
  APIs and their `physxDeformableBody` properties at once, and a removal that
  reaches it takes the whole set rather than the part the supplied prim would
  have lost. The set is the auto body API and its two sub-component APIs, the
  two legacy `Physx*DeformableBodyAPI` spellings this module removes but never
  applies, `OmniPhysicsDeformableBodyAPI`, the volume, surface and curves
  simulation APIs, and every `OmniPhysicsDeformablePoseAPI` instance.

  Before either auto-hierarchy helper removes or authors anything, it collects
  the visual geometry its bind-pose pass will use and requires every collected
  `UsdGeom.PointBased` prim to have points at default time. If one does not,
  the helper logs a warning, returns `False` and leaves the stage unchanged.

  The single-prim helpers apply the same pre-mutation rule to the geometry that
  defines their rest shape. A volume `UsdGeom.TetMesh` requires default-time
  `points` and `tetVertexIndices`; a surface `UsdGeom.Mesh` requires
  default-time `points`, `faceVertexCounts` and `faceVertexIndices`. If any
  required value is absent, the helper names it and the prim in a warning,
  returns `False` and leaves the stage unchanged.

  **A property is taken on its own terms, not the API's.** Removing an API
  schema drops the `apiSchemas` entry and leaves every property authored under
  it, so a prim can hold a deformable property with no deformable API, and the
  strip therefore does not require the API to be applied -- a property orphaned
  by an earlier partial teardown is still taken. What protects a property is an
  API that survives the removal and declares it. Several of these APIs share a
  property, and `OmniPhysicsBodyAPI` is the case that decides a caller's state:
  it is a built-in of `OmniPhysicsDeformableBodyAPI` and declares three of its
  properties, so it stays applied after that removal only when the caller
  applied it in its own right. That survival is the attribution the
  collision-API rule below has no equivalent of, so here the properties can be
  kept without guessing: the caller's body API and its three properties stay,
  while `omniphysics:mass`, which only the deformable API declares, goes.

  The property half of this reaches only what the current edit target can
  delete. A property whose opinion arrives over a reference or an inherit arc,
  or from a layer stronger than the edit target, keeps its resolved value once
  its API is gone; the API removal does compose over such an arc, so the prim is
  left carrying a deformable property and no deformable API. That is a property
  of `Usd.Prim.RemoveProperty` and applies to every remove helper this
  requirement covers, not to the deformable teardown alone. Editing a non-local
  opinion needs a layer policy this requirement does not state, so these helpers
  do not attempt one and do not report the case.

  `UsdPhysics.CollisionAPI` is the exception, and is removed only from a
  prim carrying a deformable simulation API. It is the one API here a
  caller can own independently, so sitting in the hierarchy does not establish
  ownership: an unrelated collider keeps its API and the properties authored
  under it. What stands in for ownership is a *deformable simulation* API --
  `OmniPhysicsVolumeDeformableSimAPI` or `OmniPhysicsSurfaceDeformableSimAPI` --
  on the prim the removal was asked for or on any prim of its subtree, which is
  the same range the rest of the teardown covers. An auto hierarchy generates no
  mesh below an immediate child, but `set_physics_*_deformable_body` applies a
  simulation API and `UsdPhysics.CollisionAPI` to whatever path it is given, so a
  caller can author a standalone body deeper and then remove a root above it. A
  scan narrower than the strip loop would take that prim's simulation API and
  leave its collision API behind. Every
  collision API they author lands on a prim they also give one of those two: the
  prim each `set_physics_*_deformable_body` is called on, the simulation mesh of
  a surface hierarchy, and the simulation mesh of a volume hierarchy whose
  `collision_tetmesh_path` is its `simulation_tetmesh_path`, which that helper's
  own signature permits. The stand-in is not exact in either direction, and the
  paragraph below states what each direction costs. A body API is not evidence
  -- an auto hierarchy's root
  carries one and is never made a collider. Every other prim keeps its collision
  API rather than having it guessed away: a volume hierarchy's *separate*
  collision mesh, the one collider these helpers author beside no simulation API,
  and a prim carrying only the legacy `Physx*DeformableBodyAPI` spellings this
  module removes but never applies.

  **The rule chooses between two losses.** Nothing on the stage tells a
  generated collision mesh from a collider the caller authored at the same path:
  the runtime marks neither, and finds a body's collision geometry by scanning
  the subtree for a point-based prim with an enabled collision API, which either
  answers. Keeping an unattributable collider leaves a static collider on a prim
  the caller asked to have cleaned, which is visible and removable. Taking one
  deletes the properties authored under it, which nothing recovers. The rule
  keeps it, and pays for that in the other direction: a collision API the caller
  authored on a prim these helpers then mark as simulation geometry is taken
  with the body, that prim being indistinguishable from the merged volume
  layout's one mesh, where the helper applied the collision API itself.

  The teardown the two `create_auto_*` helpers run before they build exempts the
  paths that build re-authors -- the simulation and collision mesh paths it was
  given. Each accepts a mesh the caller authored at one of those paths, and each
  reapplies the collision API and not the properties authored under it, so
  taking it there would destroy caller state and give back an API. No return
  after that teardown reapplies it either, so exempting those paths is also what
  keeps an aborted build from leaving them stripped. Every other prim is judged
  as a direct removal judges it: a collision API these helpers own at a mesh path
  the rebuild abandons is removed, since leaving it makes a second enabled
  collider in the subtree, where the runtime binds whichever one its traversal
  reaches first and rejects a surface body whose collider is not its own
  simulation mesh.

  **A moved collision path is the wall this rule does not clear.** A volume
  layout's separate collision mesh is the one collider these helpers author with
  no simulation API beside it, so a rebuild given a new `collision_tetmesh_path`
  cannot attribute the collider at the old one and leaves it, and the subtree it
  returns holds the two enabled colliders the paragraph above calls a defect.
  Nothing a rebuild can read names the layout a previous run authored:
  `PhysxAutoDeformableBodyAPI` declares `physxDeformableBody:autoDeformableBodyEnabled`
  and `physxDeformableBody:cookingSourceMesh` and no other property, and the
  cooking source relationship names the source mesh rather than either generated
  mesh. Nor does the abandoned mesh carry a marker unique to a generated one: it
  has `UsdPhysics.CollisionAPI`, which a caller can author, and an
  `OmniPhysicsDeformablePoseAPI:default` instance with its
  `deformablePose:default:omniphysics:purposes`, which the bind-pose pass gives
  every point-based prim under the root. Its type answers for a caller's tet
  mesh equally. Clearing the wall needs ownership these helpers do not persist, so
  until they do, moving a collision path leaves the old collider for the caller
  to remove.

- AC-9: The subpackage exposes none of the 58 names the Description leaves off
  the public surface, nor the nine it records as superseded before the census,
  and they are pinned name by name rather than by category, so a name
  cannot drift back in under a category that still reads as excluded. The pin has
  the same category shape the Description does, because the categories
  have genuinely different rationales and a single flat list would lose that: the
  38 of "The excluded surface", which were never recovered because they need a
  removed binding, need Kit, or have no physics content; the 17 of "The
  dropped surface", which were recovered and then dropped as a duplicate of a
  sibling helper, as an inlined step of their only caller, or for serving no
  physics purpose; and the 3 of "The privatized surface", which were recovered,
  are still called from inside the subpackage, and only lost their public
  spelling. Absence is required of all alike, the length of each list is
  asserted so a row cannot be quietly deleted from the pin instead of from the
  surface, and the lists are asserted pairwise disjoint, since a name arriving in
  two of them would leave its actual ground ambiguous.

  The privatized list, `PRIVATIZED_NAMES`, carries one assertion the other three
  do not, and needs it. For those three, absence *is* the whole claim; for this
  one, absence is half of it, because a name deleted outright would satisfy an
  absence assertion just as well as a name renamed, and the list would then be
  claiming a body that is no longer there. So each row also names the
  leading-underscore spelling its implementation now has, and that spelling is
  required to be present and callable in the submodule that owns it, and absent
  from that submodule's `__all__` so the withdrawal cannot be undone by
  re-exporting the private name. The three are `create_unused_path`, now
  `paths._create_unused_path`; `add_triangle`, now `mesh._add_triangle`; and
  `get_default_particle_system_path`, now
  `particles._get_default_particle_system_path`.

  The fourth list is the nine of "Superseded before the census",
  `SUPERSEDED_NAMES`. It is separate from the other three rather than folded into
  any of them, because those three are scoped to the 187 names the census measures
  and these nine had already left the package when that census was taken. Their
  ground is neither exclusion, a usefulness re-audit nor a privatization: every
  API schema they
  apply is absent from the schemas ovphysx ships, so a recovered version would
  raise from `codeless.apply_api` rather than author anything. Pinning them keeps
  the 38 / 17 / 3 / 129 figures untouched and stops a future recovery pass from
  reintroducing a name whose schema no longer exists. The nine are
  `add_physx_deformable_body`, `add_physx_deformable_surface`,
  `add_deformable_body_material`, `add_deformable_surface_material`,
  `TetMeshData`, `create_skin_mesh_from_tetrahedral_mesh`,
  `add_physx_particle_cloth`, `add_physx_particle_cloth_with_constraints` and
  `create_spring_grid`.

  Absent, in full, from "The excluded surface": the thirteen binding accessors
  `get_physx_interface`, `get_physx_simulation_interface`,
  `get_physx_cooking_interface`, `get_physx_cooking_private_interface`,
  `get_physx_scene_query_interface`,
  `get_physx_attachment_private_interface`,
  `get_physx_property_query_interface`, `get_physx_replicator_interface`,
  `get_physx_stage_update_interface`, `get_physx_statistics_interface`,
  `get_physx_visualization_interface`, `get_physx_benchmarks_interface` and
  `get_physxunittests_interface`; `new_memory_stage`, `release_memory_stage`,
  `ExpectMessage`, `safe_import_tests`, `get_initial_collider_pairs`,
  `compute_conforming_tetrahedral_mesh`, `compute_voxel_tetrahedral_mesh`,
  `create_auto_deformable_attachment`, `loadTetFile`, `PhysxExtension`,
  `Query`, `QueryManager`, `Result`, `AssetFolders`, `get_server_path`,
  `get_asset_path`, `get_s3_web_path`, `OV_PATH`, `S3_BUCKET`, `S3_PATH`,
  `S3_REGION`, `get_s3_upload_path`, `autoassign`, `ScopeGuard` and
  `get_all_submodules`.

  Absent, in full, from "The dropped surface": `hasAPI`, `get_derived_schemas`,
  `HALF_PI`, `add_tetra`, `voxel_sphere_test`,
  `voxel_pass_all_test`, `isDefined`, `get_default_particle_system`,
  `has_custom_metadata`, `get_custom_metadata`, `set_custom_metadata`,
  `clear_custom_metadata`, `explodeTriangleMesh`, `explodeTetraMesh`,
  `CameraTransformHelper`, `get_spatial_tendon_parent_link` and
  `get_spatial_tendon_attachment_candidates`. The six helpers that absorbed one
  of them -- `add_physics_scene`, `create_tetra_voxel_box`, `create_tetra_voxel_sphere`,
  `poisson_sample_mesh`, `set_local_space_velocities` and
  `clear_local_space_velocities` -- and `has_schema`, the sibling `hasAPI`
  duplicated, are required to be present and callable, so a drop that took a
  caller or the surviving spelling with it fails here rather than at a consumer's
  call site.

  Absent, in full, from "The privatized surface": `create_unused_path`,
  `add_triangle` and `get_default_particle_system_path`, each with the surviving
  private spelling required present as described above.

  No module under `ovphysx/python/ovphysx/utils/` imports `carb` or any `omni.*`
  module at any scope, module-level or inside a function body, so an exclusion
  cannot be reintroduced as a deferred import.

- AC-10: PhysX API schemas are applied by schema identifier and their properties
  authored by name, with no reference to a compiled `PhysxSchema` module.
  Applying or removing an API whose schema is not registered, or naming an
  unknown API or a property whose owning API was not applied, raises an error
  that states which of those causes applies and how to fix it; `remove_api`
  reports those two causes exactly as `apply_api` does, and removing an API that
  was never applied stays the no-op success USD reports it as, so a caller may
  strip a family of mutually exclusive APIs without first asking which one is
  present. `schema_is_registered` is what tells an unregistered schema set apart
  from an identifier that does not name a known API, and it answers
  `False` unless *every* codeless schema root ovphysx ships reached the registry:
  the PhysX schemas and the Omni deformable schemas are separate USD plugins that
  register independently, so a probe of one alone would report success while the
  helpers authoring against the other failed and the diagnosis blamed the
  caller's spelling. Alongside `apply_api`, which raises, `try_apply_api` returns
  USD's own refusal as a bool for a caller that reports it another way -- the
  deformable helpers warn and return `False`, which is the contract their
  ancestors had -- while still raising the same error for the two causes above,
  which USD raises on rather than answering.

  USD reports all of them with one `Tf.ErrorException`, so they are told apart
  by asking the schema registry about the call rather than about the raise, and
  before the call rather than after it -- USD answers a mistake sometimes by
  raising and sometimes with `False`, and which it uses is not a property of the
  mistake. A call the registry resolves means USD refused an operation it
  understood, which is the bool. A call it cannot resolve is a mistake in the
  call, which is the raise, and there are four: the identifier names nothing; or
  it names a multiple-apply schema and no instance name came with it, or one USD
  does not allow, the empty string for instance; or it names a single-apply
  schema and an instance name came with it. The last three matter because the
  identifier alone is valid there, so an answer derived from the identifier
  alone would report a mistake in the call as a refusal and hide it behind a
  bool. `remove_api` answers on the same terms, and nothing infers "unknown
  identifier" from a raise alone, since a valid identifier reported as unknown
  sends a caller to look for a mistake they did not make.

  A refused *property* write is not a bool anywhere. `set_attr` and `set_rel`
  raise `CodelessSchemaError` when USD does not author the property, whether
  USD reports that by answering `False` or by raising: both leave the property
  unauthored, `set_attr` returns the attribute rather than a status, and a
  helper that reported success would hand a caller a stage it does not have.
  The message names the edit target's layer where that layer is what refused
  the write, which is checked rather than assumed -- a write refused for
  another reason, a value of the wrong type for instance, gets no layer to
  look at and USD's own error as the chained cause.
  A successful property write does not format the supplied value or relationship
  targets for a diagnostic that is never emitted, and therefore does not invoke
  their `repr` methods.
  A codeless operation given an invalid `Usd.Prim` raises
  `CodelessSchemaError` with an invalid-prim diagnosis rather than leaking a
  native exception while it tries to access the prim or its stage and edit target.

  Defining a codeless concrete prim has the same registry precondition as
  applying a codeless API. Before any stage mutation, the particle-system and
  codeless-joint helpers require concrete definitions for
  `PhysxParticleSystem`, `PhysxPhysicsGearJoint` and
  `PhysxPhysicsRackAndPinionJoint`, respectively. A missing definition raises
  `CodelessSchemaError` with the registration diagnosis; this includes both a
  process that did not register the schemas and one that called the default
  registration helper after USD had already built its registry.
  A registration helper registers the
  codeless schemas shipped with the wheel into a stock USD runtime and is
  idempotent. Attributes the caller did not supply are left unauthored so they
  keep their schema fallback.

- AC-11: No `set*` or `remove*` helper accepts a `custom_execute_fn` or
  `execute_command_fn` argument.

- AC-12: The defects inherited from `omni.physx.scripts` are corrected, and the
  docstring of each affected helper states the contract the correction leaves --
  what the helper does on the corrected point -- and names the sibling helper
  that provides the old behavior, where one exists. The docstring is not where
  the correction's history lives: the mechanism, the evidence and the argument
  for the direction chosen belong to the entries below.

  **Asking for the contract and nothing about the original is a deliberate
  narrowing.** This criterion formerly required the docstring to describe the
  correction, which put migration history on the public surface. A caller
  reading `add_rigid_box` needs the contract it has rather than the contract it
  had: what changed between two releases belongs to the entries below, and only
  the pointer to the sibling is of use at the call site.
  Nothing is withdrawn from the record; the twenty-eight entries below carry every
  defect in full.

  This list is the set found so far, not a closed one: a further inherited
  defect is fixed on the same terms, with its own docstring statement of the
  contract, without amending this criterion.
  The ones fixed to date: `remove_pair_filter` removes
  `UsdPhysics.FilteredPairsAPI` rather than applying it, and removes the
  `physics:filteredPairs` relationship with it, since dropping the API alone
  leaves the targets authored -- the prim still carries a filter opinion in an
  exported layer and a later `add_pair_filter` adds to the stale target list
  instead of replacing it; `create_api_schema_property_cache` snapshots authored
  state only, per AC-3, where the original recorded fallback-resolved values and
  restored all of them unconditionally -- and that holds for the relationship
  half as well as the attribute half, on the terms AC-3 records: relationships
  have no fallback to exclude at capture time, but that is not the whole of
  "unauthored", because an unauthored relationship and one authored with an
  empty target list both compose to an empty list. The capture therefore records
  `Usd.Relationship.HasAuthoredTargets()` alongside the targets -- `None`
  targets for a relationship with no opinion, an empty list for one whose
  opinion is empty -- and the guard that keeps an unset one unset is
  `apply_api_schema_property_cache`'s skip of `None` targets, where the original
  replayed `CreateFooRel().SetTargets(value)` for every relationship the API
  declares and so authored `rel <name> = None` onto a prim that carried no such
  opinion; `remove_collider` strips
  the mesh approximation APIs from instanceable prims as well as meshes, and
  strips every approximation API `set_collider` can apply, the PhysX-only
  `PhysxSDFMeshCollisionAPI` and `PhysxSphereFillCollisionAPI` included;
  `set_collider` guards on `PhysicsCollisionAPI`, the name USD records for an
  applied `UsdPhysics.CollisionAPI`, so a prim that is already a collider is left
  alone instead of having a second approximation API stacked on the first;
  `remove_collider_subtree` visits instanceable prims as well as gprims, mirroring
  `set_collider_subtree`, so a collider applied over a subtree can also be removed
  over it; `descendant_has_api` and `ancestor_has_api` stop at a prim that really
  resets its xform stack, asked through `UsdGeom.Xformable.GetResetXformStack()`,
  where the original tested `HasAttribute("xformOp:reset")` -- an attribute USD
  does not define, a reset stack being the `!resetXformStack!` token inside
  `xformOpOrder` -- so the guard never fired and both walks crossed every
  transform-reset boundary their documentation promised to respect;
  `setup_transform_as_scale_orient_translate` and
  `copy_transform_as_scale_orient_translate` perform their composed xform reads
  outside `Sdf.ChangeBlock`, where the originals put their whole read-and-write
  sequence inside one even though USD defers change processing there and does
  not permit inspection of composed stage state until the block closes;
  `poisson_sample_mesh` returns the particle system path on success and an empty
  path on failure, so the two are distinguishable, and returns that empty path
  rather than raising `AssertionError` out of the creation call when the stage's
  default particle system path is already held by a prim of another type, which
  its search for the particle system type name alone cannot see;
  `create_auto_volume_deformable_hierarchy` returns False when a requested
  hexahedral simulation mesh could not be applied, rather than reporting success
  after falling back to a tetrahedral one; `add_pair_filter` compares prim
  paths by value, so duplicate entries or a mix of `str` and `Sdf.Path`
  spellings never make a prim filter against itself; `remove_collider`
  removes the authored properties of the approximation and cooked-data APIs
  along with the APIs themselves; `add_capsule`, `add_cylinder` and `add_cone`
  author the extent each shape's own `UsdGeom` schema computes for it, where the
  original wrote `[(-radius, -radius, -height), (radius, radius, height)]` in
  all three -- an extent laid out as though the shape always ran along Z, so it
  ignores `axis`, uses the full `height` where a cylinder and a cone reach half
  of it either side of their centre, and omits the capsule's two hemispherical
  caps; `remove_deformable_body` removes every deformable API it owns across the
  supplied prim's whole subtree, including that prim, where the original
  iterated `prim.GetChildren()` and so left an `OmniPhysicsDeformablePoseAPI`
  instance and its bind-pose properties on any skinned mesh nested deeper than
  one level -- the depth the two `create_auto_*_deformable_hierarchy` helpers
  actually author over; `remove_deformable_body` removes
  `UsdPhysics.CollisionAPI` only from a prim carrying a deformable simulation
  API, where the original removed it from every immediate child of the
  supplied prim, so a collider a caller authored inside the subtree lost the
  API and the properties authored under it -- on the removal path and on both
  `create_auto_*` paths, which run that teardown before rebuilding, so
  authoring a deformable body over a subtree destroyed a caller's collider
  too; and `verify_tetra_mesh` rejects a vertex index below
  zero on the same terms as one past the end of the point list, where the
  original tested `i >= len(points)` alone and Python's wraparound indexing then
  presented the volume check with a plausible tetrahedron built from the wrong
  points; and `add_physx_particle_system`, `add_physx_particleset_points` and
  `add_physx_particleset_pointinstancer` refuse an occupied target path with a
  `ValueError`, where the original spelled that precondition as an `assert` --
  which `python -O` compiles out, after which `Usd.Stage.DefinePrim` and the two
  `UsdGeom` `Define` calls retyped whatever prim was already at the path, kept
  its authored properties and authored the particle data on top, so an optimized
  build rewrote a caller's prim in place instead of refusing to touch it; the
  `add_plane_collider` helper refuses an occupied target path with `ValueError`
  before calling `UsdGeom.Plane.Define`, where the original defined over the
  existing prim, changed its type to `Plane`, left its authored properties in
  place and added collision state on top; the two
  `create_auto_*_deformable_hierarchy` helpers validate the default-time point
  array of every visual `UsdGeom.PointBased` prim before their teardown, where
  the originals began changing the hierarchy and only later passed an
  unauthored points attribute's `None` value to a typed USD write, which raised
  `Tf.ErrorException` and left the partial changes behind; the two single-prim
  deformable helpers validate every default-time geometry value their rest
  shape needs before applying an API or computing topology, where the originals
  either raised while using a missing value or began applying the deformable
  APIs and then failed to author the missing rest-shape data; the
  six `add_rigid_*` shape constructors apply `UsdPhysics.RigidBodyAPI` and
  `UsdPhysics.MassAPI` for every density, where the original's shared
  `_add_rigid` read a density of exactly 0.0 as a request for a static collider
  and applied neither, so `add_rigid_box(..., density=0.0)` returned a bare
  collider; and `extract_triangle_surface_from_tetra` counts how often each face
  occurs and keeps the ones occurring once, where the original toggled a face in
  and out of a dictionary and so tracked parity -- a face shared by three
  tetrahedra was deleted on its second occurrence and re-added on its third, so
  an interior face came back out as surface and three tetrahedra over one shared
  face yielded ten triangles where nine occur once; `set_physics` completes and
  updates the rigid-body API set when `UsdPhysics.RigidBodyAPI` is already
  applied, where the original returned before applying `PhysxRigidBodyAPI`,
  enabling the body or authoring the requested kinematic state; and
  `compute_bounding_box_diagonal` raises `ValueError` for an empty iterable,
  where the original left all six float sentinels unchanged and returned a
  finite value that described no bounds; `set_collider` reads the value of the
  `omni:no_collision` attribute, where the original treated the existence of an
  authored false attribute as an opt-out and left the prim without collision;
  `create_triangle_mesh_square` returns empty point and index lists when either
  grid dimension is non-positive. The original divided by zero for `(0, 1)`
  and `(1, 0)`, already returned a valid empty mesh when one dimension was
  negative and the other positive, and returned invalid indices without points
  when both were negative. `create_tetra_voxel_box`,
  `create_tetra_voxel_sphere` and their `create_triangle_mesh_cube` caller
  return the empty mesh produced for a non-positive grid before normalizing its
  points, where the originals divided by zero for a zero-resolution grid.

  **The six `add_rigid_*` constructors count as one defect, not six**, on the
  rule the two entries below already state: it is one guard, in one shared
  private helper that all six call, with one fix. The contract statement is
  carried by all six docstrings, because the promise a caller reads is theirs.

  **The direction of that fix was a choice, and the two options are not
  symmetric.** Applying the APIs unconditionally and documenting zero as a
  static-collider sentinel would each close the contradiction. AC-5 and all six
  docstrings promise the rigid body without qualification, `add_collider_*` is
  already the spelling for a static collider beside every one of them, and
  `UsdPhysics` gives zero its own meaning -- `physics:density` falls back to 0
  and is documented "if non-zero", so zero is the schema's spelling for
  carrying no density opinion, not for being static. Documenting the sentinel
  would therefore keep a surprising API and a second spelling for something the
  surface already has. What it costs is a behavior change for a caller who
  passes 0.0 today and gets a static collider; that caller wants
  `add_collider_*`, and the docstring names it.

  **The three particle-set constructors count as one defect, not three**, on
  the same rule as the three shape constructors above: one guard expression
  copied into three helpers, the same mechanism, the same symptom -- an existing
  prim of another type silently retyped and written over -- and the same fix,
  making the guard a statement `-O` cannot remove. The contract statement is
  carried by all three docstrings, and each names the `ValueError` a caller
  sees. The count of that exception
  change is deliberate: this list records where a recovered helper's behavior
  differs from the code a consumer is porting, and the exception class is the
  part of the difference a caller can observe on the path that already worked.

  **The two transform-stack helpers' `Sdf.ChangeBlock` correction counts as one
  defect, not two.** They copied the same wrapper around the same sequence of
  composed reads and xform-op writes, with the same invalid read ordering, and
  remove it on the same terms.

  **The three voxel wrappers count as one defect, not three.** The two normalized
  wrappers copy the same step after the same empty-grid result and fail on the
  same zero-resolution input; the surface-cube wrapper delegates to one of them.
  All three return the empty result before normalization.

  **The two auto-hierarchy helpers' missing-points correction counts as one
  defect, not two.** Both used the same bind-pose traversal, read the same
  unauthored value and failed in the same typed write after the same destructive
  teardown. The shared preflight and one parametrized regression cover that
  class for both the volume and surface layouts.

  **The five single-prim missing-geometry inputs count as one defect, not five.**
  They expose the same absent preflight over the two helpers and the same
  contract: all geometry required to build the rest shape must exist before
  either helper mutates the stage. One shared validation path and one
  parametrized regression cover the two volume fields and three surface fields.

  **The property cache's relationship half completes the second entry rather
  than adding an entry of its own.** It is one defect with two property kinds,
  not two defects: the same helper pair, the same mechanism -- an unconditional
  replay -- the same symptom, an opinion authored onto a prim that had none,
  and the same fix, leaving an unset value unset. Splitting it in two would
  make the count less informative rather than more: the figure tracks defects
  and not the property kinds each one happens to span. The escape clause above
  -- a further inherited defect is fixed on the same terms without amending
  this criterion -- is for a genuinely new defect; widening the recorded scope
  of one already listed is an edit to this AC, which is what this is. What the
  entry previously left implicit, and a reviewer reasonably read as absent, is
  that the relationship guard lives on the replay side; it now says so.

  **The authored-empty relationship narrows that same entry a second time, and
  again adds no entry.** The `if not targets` skip the entry described was too
  wide: an empty target list is what a relationship authored with no targets
  snapshots as, so the skip that stopped the original from inventing an opinion
  also discarded a real one. That is the same helper pair and the same
  property kind as the entry already covers, and the correction is to the
  branch condition the entry names, so it belongs inside it. The escape clause
  does not apply, for the same reason it did not the first time. It is worth
  saying plainly that the fix is on both sides now rather than only the replay:
  authoredness is a bit the capture has to carry, because nothing downstream
  can reconstruct it from an empty list.

  **`remove_deformable_body` carries two entries**, on the rule
  `remove_collider`'s three follow: one helper, two unrelated deviations. The
  first is the depth of the deformable-API sweep, and its symptom is authored
  state left behind. The second is the breadth of the collision-API sweep, and
  its symptom is a caller's authored state destroyed. Different mechanism,
  opposite failure, separate fix, so the count is two.

  **The three shape constructors count as one defect, not three.** It is one
  wrong extent expression, copied verbatim into `add_capsule`, `add_cylinder`
  and `add_cone`: the same mechanism, the same symptom -- an authored `extent`
  that disagrees with the bound USD computes for the same prim -- and the same
  fix, deferring to the schema's own extent computation. The three shapes'
  correct extents differ, but that is the geometry differing, not the defect.
  The count tracks defects rather than helpers throughout this list, which is
  why `remove_collider` carries three entries for its three unrelated
  deviations and why `descendant_has_api` and `ancestor_has_api` share one; the
  same rule applied here gives one. The contract statement is carried by all
  three docstrings, as that shared pair's is by both of theirs.

  `remove_collider`'s removal of the approximation and cooked-data APIs'
  authored properties is the same defect class as the first. `Usd.Prim.RemoveAPI`
  and `codeless.remove_api` drop the `apiSchemas` entry and leave every property
  authored under it in place, so the original left
  `physxConvexHullCollision:hullVertexLimit` -- and a cooked buffer for a mesh
  about to be cooked differently -- authored on a prim that no longer carries
  the API declaring it. A prim re-approximated after a `remove_collider` was
  then tuned by a value nothing on it applies, and exported a cooked buffer for
  the previous approximation. It is fixed with the same mechanism
  `remove_pair_filter` uses, `remove_api_schema_properties`.

  The fix is deliberately bounded to the mutually exclusive APIs.
  `PhysicsCollisionAPI`, `PhysxCollisionAPI` and `PhysicsMeshCollisionAPI` are
  re-applied by every `set_collider` call, so their properties --
  `physxCollision:contactOffset` and `physxCollision:restOffset` above all --
  are not orphaned by a remove-and-reset cycle and keep surviving it, which is
  what lets a caller re-approximate a collider without silently losing its
  contact tuning. Removing those stays the caller's to ask for, through
  `remove_api_schema_properties`, and `create_api_schema_property_cache` is what
  carries them across it; the worked example in that helper's docstring is the
  one place the split is written down for a caller, and it turns on it.
  The multiple-apply `PhysxCookedDataAPI` instances go with the approximations
  rather than with the re-applied APIs: `set_collider` does not re-apply them,
  and a cooked buffer keyed to an approximation the prim no longer has is the
  same orphan with a worse failure mode, since it is a cache the runtime could
  read rather than a number a caller might notice.

- AC-13: A submodule whose helpers drive a simulation rather than author a stage
  is admissible in `ovphysx.utils`, and is the correct home for a supported
  helper that does not belong on the top-level `PhysX` class. Such a submodule
  imports runtime dependencies such as `ovstage`, `warp` and `numpy` inside
  its function bodies rather than at module
  scope, and reaches `ovphysx.api` either the same way or under
  `if TYPE_CHECKING:` for annotations only. A module-scope import of a
  native-free sibling such as `ovphysx.types` is permitted, since it loads no
  native library. AC-1 therefore continues to hold for the whole subpackage:
  `import ovphysx.utils` still needs only `pxr` and the standard library and
  still loads no native library. Because such a submodule imports no `pxr` at
  module scope, it is also the one reachable under AC-1a with no USD present at
  all. At call time such a helper may require
  `ovstage`, `numpy` and a live `PhysX` instance; that asymmetry with the
  authoring submodules is accepted, not accidental. Its public names are
  re-exported flat from `ovphysx.utils` on the same terms as every other
  submodule's (AC-2). The requirement that owns such a helper's behavior is the
  requirement written for that helper, not this one; this AC governs only its
  location and its import discipline, so the implementing file carries both
  annotations.

- AC-14: The registration-ordering failure is stated accurately and is
  detectable on request. `ovphysx.utils.codeless`'s unregistered-schemas
  message describes the constraint as it actually is -- USD builds the schema
  registry once and never rebuilds it, so the process the message was raised in
  cannot be repaired -- and offers only recoveries that can succeed: reordering
  `register_schemas()` ahead of the first stage or registry access in a new
  process, or presetting `PXR_PLUGINPATH_NAME`, which is named as the route that
  works even where USD is already initialized. It does not direct the caller to
  a `register_schemas()` call in the failed process, which would report success
  and register nothing usable. `register_schemas(verify=True)` raises
  `CodelessSchemaError` when any of the schema roots did not reach the schema
  registry -- naming the ones that did not, and both causes, since a root that
  is simply not staged is indistinguishable at that point from a late
  registration -- and returns normally when they all did; `verify` defaults to
  off because the check
  builds the schema registry and so locks out every subsequent schema plugin
  registration in the process, ovphysx's and any other library's alike.

- AC-15: The 51 helpers the original spelled `camelCase` are renamed to
  `snake_case`, and exactly eight of the old spellings survive as deprecated
  aliases: `setCollider`, `setRigidBody`, `removeCollider`, `removePhysics`,
  `removeRigidBodySubtree`, `hasSchema`, `createJoint` and
  `extractTriangleSurfaceFromTetra`. Those are the eight with measured consumers
  in Isaac Sim and IsaacLab, per the Description; the other 43 renamed names get
  no alias, because nothing outside this repository calls them.

  Each alias forwards to the renamed helper and returns exactly what it returns,
  and each raises a `DeprecationWarning` naming both spellings, so a consumer
  sees which call to change and can grep for either name. `functools.wraps`
  carries the target's signature and docstring across, so `help()` and
  `inspect.signature` answer for the alias as they do for the helper. Aliasing
  is one-directional: no internal call site calls an alias, so the warning a
  consumer sees is always attributable to their own code.

  The aliases are reachable as `ovphysx.utils.<oldName>` and by explicit
  `from ovphysx.utils import <oldName>`, and are **absent** from `__all__`, from
  `dir(ovphysx.utils)`, from `from ovphysx.utils import *` and from the rendered
  API documentation, so the documented surface and the star-import surface show
  only the new spellings. That combination is not free under AC-1a's lazy
  loading: `ovphysx.utils.__getattr__` resolves a public name through the flat
  owner manifest, which deliberately contains no aliases, so it consults an
  explicit alias table naming the eight and their owning submodules first. The
  table is what makes exclusion work, and its length is the enforcement point
  for the bound of eight -- a ninth alias is a policy change, not an
  implementation detail.

  Each alias is defined in the submodule that owns the renamed helper. The
  warning-raising wrapper itself comes from one private module,
  `ovphysx/python/ovphysx/utils/_deprecation.py`, which exports nothing and is
  not a submodule of the flat surface.

  **Keyword parameters follow the same rule as helper names, with a narrower
  exception set.** The 34 parameters the original spelled `camelCase` -- across
  22 public helpers in `authoring`, `filtering`, `joints`, `materials`, `mesh`,
  `particles`, `planes` and `schema` -- are renamed to `snake_case`, so no
  public signature carries a camel spelling. A caller reading a rendered
  signature or an `inspect.signature` result sees one convention throughout,
  which is the whole point of renaming the helpers and was left half done while
  the parameters kept the old one.

  Three parameters keep the old spelling accepted, on three helpers:
  `approximationShape` on `set_collider` and on `set_rigid_body`, and
  `schemaName` on `has_schema`. Those three are not a selection but the entire
  intersection of the helpers with a camel parameter and the eight with
  measured consumers above, so the rule is the same one the aliases follow: a
  spelling stays reachable exactly where a call site outside this repository is
  known to use it. The remaining 19 helpers, carrying 31 parameters between
  them, are renamed outright with nothing kept, because nothing outside this
  repository calls them at all -- the same ground on which 43 of the 51 renamed
  helper names got no alias.

  A kept spelling behaves as a deprecated name does: passing it raises a
  `DeprecationWarning` naming both spellings, and the value reaches the helper
  unchanged. Passing both spellings in one call is a `TypeError` rather than a
  silent winner, since nothing in such a call says which value was meant, and
  that holds for the camel spelling passed as a keyword over a positional value
  as well as over the renamed keyword: all three renamed parameters may be
  supplied positionally, and a check on the keywords alone would warn and then
  let Python raise its own duplicate-argument error, telling the caller to
  migrate to the spelling they had already used.

  The enforcement point for the bound of three is not a table in
  `utils/__init__.py`, because a parameter is resolved inside the call rather
  than by `__getattr__` and so needs no table to resolve at all. The wrapper
  instead records the pair it accepts as `__deprecated_parameters__`, and the
  bound is asserted against the set discovered by walking the flat surface, so
  a fourth shim fails the bound rather than going unrecorded. The wrapper comes
  from the same private `_deprecation.py` module the aliases do, and follows its
  conventions -- `functools.wraps`, `DeprecationWarning`, a warning attributed
  to the first caller outside that module's wrappers, and a message naming both
  spellings. Caller attribution also holds when a deprecated parameter is
  passed through its helper's deprecated alias.

  `tesselation`, on `create_mesh_cylinder` and `create_mesh_cone`, is
  deliberately untouched by any of this. It is already `snake_case`; what is
  wrong with it is that "tessellation" has two `l`s, which is a different
  question from a casing convention and is not decided here.

## Test References

- TEST-PYTHON-UTILS-001

## Code References

- ovphysx/python/ovphysx/utils/__init__.py
- ovphysx/python/ovphysx/utils/_deprecation.py
- ovphysx/python/ovphysx/utils/codeless.py
- ovphysx/python/ovphysx/utils/constants.py
- ovphysx/python/ovphysx/utils/schema.py
- ovphysx/python/ovphysx/utils/transform.py
- ovphysx/python/ovphysx/utils/paths.py
- ovphysx/python/ovphysx/utils/shapes.py
- ovphysx/python/ovphysx/utils/planes.py
- ovphysx/python/ovphysx/utils/joints.py
- ovphysx/python/ovphysx/utils/materials.py
- ovphysx/python/ovphysx/utils/filtering.py
- ovphysx/python/ovphysx/utils/authoring.py
- ovphysx/python/ovphysx/utils/mesh.py
- ovphysx/python/ovphysx/utils/particles.py
- ovphysx/python/ovphysx/utils/deformable.py
- ovphysx/python/ovphysx/utils/simulation.py (location and import discipline
  only; the helper's behavior is owned by REQ-PYTHON-FRAME-001)
- ovphysx/python/ovphysx/utils/simulation.pyi
- ovphysx/tests/python_tests/utils_tests/conftest.py
- ovphysx/tests/python_tests/utils_tests/test_utils_surface.py
- ovphysx/tests/python_tests/utils_tests/test_utils_authoring.py
- ovphysx/tests/python_tests/utils_tests/test_utils_schema.py
- ovphysx/tests/python_tests/utils_tests/test_utils_bugfixes.py
- ovphysx/tests/python_tests/utils_tests/test_utils_deprecated_aliases.py
- ovphysx/tests/python_tests/test_utils_package_layout.py
- ovphysx/tests/python_tests/test_type_stubs.py (its stub-filename tuple carries
  `utils/simulation.pyi`, which is what keeps the in-package stub path from
  regressing to a sibling `utils.pyi`)
- ovphysx/docs/python_api.rst
- ovphysx/repo.toml (mocks caller-supplied `pxr` during documentation builds
  and preserves source-form default values for that mock)
- ovphysx/python/pyproject.toml (the `[tool.setuptools.package-data]`
  `"ovphysx.utils"` entry, which pins the in-package stub into the wheel
  independently of how setuptools resolves the top-level `"*.pyi"` glob across a
  subpackage boundary)
- ovphysx/scripts/test_python_runtime.cmake (runs utils_tests/ as its own pytest
  process, which the schema-registration ordering requires)

## Dependencies

- None
