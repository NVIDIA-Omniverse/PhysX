<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-UTILS-001
maps_to: REQ-PYTHON-UTILS-001
type: integration
---

## Scenario

The `ovphysx.utils` authoring helpers are exercised against in-memory USD stages,
and the shape of the subpackage's public surface is inspected, both without
creating a native `PhysX` instance or attaching a stage. `type: integration`
rather than `unit` because the scenario is not process-local: it registers USD
schema plugins into a process-global, one-way registry, needs its own pytest
invocation for that reason, spawns child interpreters to reach the
registration-ordering branches, and resolves its schema tree out of a staged
install layout.

Which file covers which AC is tabulated under "Test code and coverage map"
below, after the outcomes; what the suite does *not* cover is under "Known
coverage gaps" after that.

## Given

- The installed `ovphysx` package and a USD runtime providing `Usd`, `UsdGeom`,
  `UsdPhysics`, `Sdf` and `Gf`. There is no `PhysxSchema` module: the PhysX
  schemas are codeless. The Python test stage supplies USD from the packman
  package in `deps/usd-py-tests.packman.xml`, put on `PYTHONPATH` by
  `test_python_runtime.cmake`; no `usd-core` dependency is declared anywhere,
  per REQ AC-1 and the REQ Description's "Why `pxr` is caller-supplied", which
  records that supplying USD to the test stage is not declaring it, so the test
  environment is not the environment a consumer installs.
- ovphysx's codeless PhysX schemas registered into that runtime before any stage
  is opened, via `ovphysx.utils.codeless.register_schemas()`, in a session-scoped
  fixture that every stage-opening case requests.
- A deliberately *half*-staged schema tree, built by the session-scoped
  `half_staged_schema_root` fixture: a copy of the staged `PhysxSchema` root with
  the `OmniUsdPhysicsDeformableSchema` one left out. No install layout produces
  that state, and it cannot be registered in the pytest process, whose registry
  was built from the complete tree, so the fixture yields the tree and a child
  interpreter registers it. It degrades on the same terms as the schema fixture
  above, and additionally when the staged tree does not carry both roots, since a
  tree already missing one is not a half of anything.
- An in-memory stage per authoring case, created with
  `Usd.Stage.CreateInMemory()`.
- Seven module-level name lists in `test_utils_surface.py`, none of them a
  fixture: the fifteen submodule names, the eleven `codeless` names AC-2 exempts
  from flat re-export (`CODELESS_ONLY_NAMES`), and the five lists AC-9 pins.
  Those carry AC-9's category structure into the suite rather than
  flattening it: `EXCLUDED_NAMES`, the 38 never recovered, grouped in source
  order by the `omni.physx.scripts` module each name came from;
  `DROPPED_NAMES`, the 17 recovered and then dropped on usefulness, grouped by
  the `ovphysx.utils` submodule that held each one before the drop, which is not
  the same axis as the lists either side of it; `PRIVATIZED_NAMES`, the three
  recovered names that are still called from inside the subpackage and no longer
  public;
  `SUPERSEDED_NAMES`, the nine that had already left the package before
  the commit the 187-name census measures, grouped by the module they were
  removed from; and `SURVIVORS_OF_DROPPED_NAMES`, the six helpers a dropped name was
  folded into plus `has_schema`, the sibling `hasAPI` duplicated. Absence is
  parametrized over `EXCLUDED_NAMES + DROPPED_NAMES + SUPERSEDED_NAMES` and the
  keys of `PRIVATIZED_NAMES` together, so all four are held
  to the same standard, while the length of each is asserted separately -- a
  parametrized absence case cannot notice a row deleted from a list rather than
  from the surface -- and the four are asserted pairwise disjoint, since a name
  in two of them would leave its recorded ground ambiguous.

  `PRIVATIZED_NAMES` is a mapping rather than a tuple, because absence is only
  half of what it claims. Each key maps to its owning submodule and the
  leading-underscore spelling the implementation now carries, and a separate case
  asserts that spelling present, callable, and absent from the submodule's own
  `__all__`. REQ AC-9 states why both halves are needed.

  The package carries a dependency-free owner manifest for all 138 flat
  exports. A case constructs the actual owner mapping from each loaded
  submodule's `__all__`, rejects duplicate names and requires both directions of
  the manifest to equal it exactly. A separate case still asserts the flat
  surface is 138 helpers and `__all__` 153 entries, so the owner check and the
  surface-size check fail for different forms of drift.
- Two independent tables of AC-15's eight deprecated aliases, one per file, and
  the duplication is the point. `test_utils_surface.py` maps each old spelling to
  its owning submodule and its new spelling; `test_utils_deprecated_aliases.py`
  maps each to a call that exercises it against a stage. Each file asserts its own
  table equals `ovphysx.utils._DEPRECATED_ALIASES`, so a ninth alias fails both
  rather than being silently untested by one of them, and neither table derives
  its new spelling from the other: the surface file's is written out, the
  behavioral file's is read off the alias's own `__wrapped__`, so an alias wired
  to the wrong target disagrees with one of the two.
- One table of AC-15's three kept `camelCase` parameters, in
  `test_utils_surface.py`, keyed by the helper and the old spelling and mapped
  to the renamed parameter. Unlike the alias tables it is checked against a set
  *discovered* from the flat surface rather than against a table the
  implementation carries, because there is none to check: a parameter needs no
  entry anywhere to resolve, so the wrapper's own `__deprecated_parameters__`
  record is the only statement of what is shimmed, and walking the surface for
  it is what makes a fourth shim fail the bound. `test_utils_deprecated_aliases.py`
  asserts the set it exercises equals that same discovered set, so the two
  files agree on the bound the way the two alias tables do.

Two environments degrade to skips rather than failures, because neither is a defect
in the subpackage under test. Without `pxr` there is no USD runtime to author into,
and the whole suite skips at module scope. In a raw source checkout
`codeless_schema_paths()` finds no staged schema tree, since it resolves the wheel
or `_install/` layout; there, only the cases that open a stage skip, because the
schema fixture is deliberately not autouse. The fixture distinguishes both from a
third case, schemas already registered through ovphysx's own plugin path, in which
case everything runs.

That leniency is scoped to a developer checkout, and it is switched off where it
would be dishonest. The CMake step sets `OVPHYSX_REQUIRE_CODELESS_SCHEMAS=1` for
this suite's invocation; under it both degradations become hard failures -- the
missing schema tree in the fixture, the missing `pxr` in a `pytest_configure`
hook, which has to be earlier because the per-file `importorskip` runs before any
fixture. Without that switch a staged run with no schema tree skips every
stage-opening case, which is all of this subpackage's behavioral coverage, and
still exits 0, so neither the CMake exit-code guard nor its JUnit failure count
sees anything wrong.

**Every degradation in the suite goes through one helper**, `_unavailable` in the
conftest, so the switch cannot be escaped by a case that decides for itself. Three
cases needed that and did not have it: the two late-registration cases and the
half-staged one call `pytest.skip` after their child interpreter reports back, so
the decision cannot be expressed as a fixture dependency, and calling `skip`
directly put the whole of AC-14 and the unregistered-schemas half of AC-10 outside
the guard -- able to vanish from a green CI run without the run noticing. They now
take a `codeless_unavailable` fixture, which is the same helper handed over as a
callable, and fail rather than skip under the switch.

## When

- The subpackage and each submodule are imported in a subprocess with `carb`, the
  `omni` namespace, `ovstage` and `numpy` made unimportable. The blocker is a
  `find_spec` meta-path finder, and the subprocess asserts the blocker itself
  works before trusting the imports it guards: a `find_module` blocker is ignored
  from Python 3.12 on and would pass vacuously.
- Each submodule's source is parsed and its module-scope imports inspected for
  `carb`, `omni.*`, `ovstage`, `numpy` and `ovphysx.*`.
- Each submodule's source is walked in full, at every scope, for an import of
  `carb` or anything under `omni`. This is the check the module-scope pass and
  the import-time subprocess both miss: a deferred `import carb` inside a
  function body satisfies each of them and still breaks the helper outside Kit.
- `ovphysx.utils.__all__`, the flat attribute surface and the per-submodule
  attribute surface are compared against each other and against the excluded-name
  list. The dependency-free flat-export manifest and its reverse owner mapping
  are compared exactly with the loaded owners' `__all__` lists.
- In a child interpreter with `pxr` unimportable, `dir(ovphysx.utils)` is taken
  and `ovphysx.utils.__all__` is requested, so the tolerant and the strict side
  of the same lazy loading are exercised in the one process that can tell them
  apart. A known authoring export is requested by direct lookup, `getattr` with
  a default, `hasattr` and explicit `from` import; unknown ordinary and dunder
  attributes are requested there too. The submodule names `dir()` is checked
  against come from the package directory's own `.py` files rather than a
  second hardcoded list.
- The installed package directory is inspected for the shape of `ovphysx.utils`:
  that `utils/` is a directory holding an `__init__.py`, and that neither
  `utils.py` nor `utils.pyi` exists beside it.
- Authoring helpers are called on a fresh stage: shape, collider and rigid-body
  constructors; density, velocity and material helpers; collider and rigid-body
  API application and removal; collision group and filtered pair management; joint
  creation; custom metadata accessors; procedural mesh and tetrahedral mesh
  helpers; the transform op helpers, including on a bare `Usd.Prim` and a
  non-Xformable prim; the
  particle system, particle set and feature API helpers; and the deformable body,
  material and teardown helpers.
- The schema introspection and property cache helpers are exercised over
  single-apply and multiple-apply schemas and over a relationship-bearing schema.
- Codeless access is exercised directly: an API is applied by identifier and by
  identifier plus instance name, its properties are authored and read back, an
  unknown identifier and an unknown property name are requested, and the
  registration helper is called a second time. `remove_api` is called for an
  applied API, twice more for APIs that are not on the prim -- one single-apply,
  one multiple-apply instance -- and once for an unknown identifier.
- The prim-taking codeless operations are also called with an invalid
  `Usd.Prim`, so their public diagnostic is distinguished from an incidental
  attribute access on a missing stage.
- Successful `set_attr` and `set_rel` calls are given values and targets whose
  `repr` raises, so evaluating a failure-only diagnostic on the success path is
  observable. Child interpreters exercise concrete-prim preflight for
  `PhysxParticleSystem`, `PhysxPhysicsGearJoint` and
  `PhysxPhysicsRackAndPinionJoint` both without registering the schemas and
  after calling the default registration helper too late.
- `codeless.try_apply_api` is exercised indirectly, as the seam every codeless
  apply in `deformable.py` goes through, and directly through a monkeypatch that
  refuses one identifier.
- USD's own refusal of an operation it resolved is constructed by calling
  `Usd.Layer.SetPermissionToEdit(False)` on a stage's root layer after the prim
  and its APIs are authored, and every codeless entry point is then called
  against it:   `try_apply_api`, `apply_api`, `remove_api`, `set_attr` and
  `set_rel`, plus `try_apply_api` for an unknown identifier on that same layer,
  which is what separates the refusal from the diagnosis. A second set of
  calls, on a writable layer, gives a valid identifier an instance name its
  schema contradicts, in each of the three ways it can. USD raises out of
  both writes on such a layer, so their `False` return is reached by patching
  `Usd.Attribute.Set` and `Usd.Relationship.SetTargets` instead -- nothing in
  the subpackage sits between the helper and the write to patch in its place.
- A deformable root is given a child collider the helpers did not author: a
  `UsdGeom.Mesh` carrying `UsdPhysics.CollisionAPI` and an authored
  `physics:collisionEnabled`, added before the hierarchy is built. Point-based
  on purpose, so the `create_auto_*` bind-pose pass visits it. Both auto
  hierarchies are then built over that subtree, rebuilt a second time, and torn
  down.
- The registration ordering is exercised in child interpreters, since the
  session's own registry was built correctly and USD's is process-global and
  one-way. One child opens an in-memory stage first and then calls
  `register_schemas(verify=True)`, `apply_api` and `remove_api`; another calls
  `register_schemas(verify=True)` before importing anything that opens a stage
  and then applies an API.
- A third child registers the half-staged tree in the supported order -- nothing
  is late about it -- and then reports what the registry knows, what
  `schema_is_registered()` answers, and what applying an API from each root
  does. It reaches the tree by rebinding `ovphysx.codeless_schema_paths`, which
  `register_schemas()` imports at call time, so the resolver runs as it would
  against an install with one root missing.
- Collider removal is exercised over a subtree, not only a single prim: a root
  Xform holding a nested Xform, four meshes carrying the `sdf`, `sphereFill`,
  `convexHull` and `meshSimplification` approximations, one of them additionally
  a `PhysxCookedDataAPI` instance, an instanceable Xform referencing a mesh
  prototype, and a Scope that was never a collider. `remove_collider_subtree` runs
  on the root; `set_rigid_body` then `remove_rigid_body_subtree` run over the same
  shape for the traversal the two share.
- The capsule, cylinder and cone constructors run in all three flavors -- plain,
  `add_collider_*` and `add_rigid_*` -- parametrized over the three shapes.
- `create_mesh_square_axis` runs for the `Y` and the `Z` axis, and
  `create_mesh_concave` for a unit half-size, both through `create_mesh`.
- A surface deformable body is authored on a triangulated mesh built by
  `create_mesh` from `create_triangle_mesh_square`, refused on a quad mesh, and
  built as a hierarchy with a skin mesh under the root and the simplification and
  guide-purpose options on. The remove-side helpers then run in sequence against
  an auto volume hierarchy: hexahedral mesh, mesh simplification, mesh
  simplification re-added, auto deformable body, deformable body.
- The authored stage is exported to a `.usda` string and reopened.
- Every public `set*` and `remove*` helper's signature is inspected for a
  `custom_execute_fn` or `execute_command_fn` parameter.
- The inputs that triggered the inherited defects are exercised: a successful and
  a failing `poisson_sample_mesh` call, and a third on a stage whose default
  particle system path is already held by an `Xform`; an auto volume deformable
  hierarchy requested with a hexahedral simulation mesh, one whose hexahedral
  apply is made
  to fail, one asking for no hexahedral mesh under the same injected failure, and
  one rooted at a Gprim; `add_pair_filter` over distinct paths, over a list mixing
  `str` and `Sdf.Path` spellings of one prim, and over a list holding two
  equal-but-distinct duplicate strings; `remove_pair_filter` after an
  `add_pair_filter`, followed by an `add_pair_filter` naming a different prim; a
  property cache captured from a prim carrying two authored attributes out of the
  twenty its API declares, a second captured from one instance of a
  multiple-apply API with one of its four attributes authored, and a third
  captured from a prim whose relationship-bearing API carries no authored
  targets at all;
  `remove_collider` on an instanceable prim; `ancestor_has_api` and
  `descendant_has_api` over three-deep Xform hierarchies whose middle prim really
  resets its xform stack, over the same two hierarchies without the reset, and
  over a non-Xformable `Scope` that reaches the guard; each of the six
  `add_rigid_*` constructors called with `density=0.0` and again with a non-zero
  density, alongside its own `add_collider_*` sibling; and
  `extract_triangle_surface_from_tetra` over three tetrahedra sharing one face
  and over two sharing the same one; `set_physics` on a pre-applied, disabled
  `UsdPhysics.RigidBodyAPI`, with both changes of kinematic state; and
  `compute_bounding_box_diagonal` on an empty list and an empty one-shot
  iterator; `add_plane_collider` on a path already held by a cube; both auto
  deformable hierarchy helpers over a point-based visual prim with no authored
  points at default time; the single-prim volume helper with `points` and
  `tetVertexIndices` omitted one at a time; the single-prim surface helper with
  `points`, `faceVertexCounts` and `faceVertexIndices` omitted one at a time;
  and both transform-stack rewrite helpers after `Sdf.ChangeBlock` is replaced
  with a test double that raises; `set_collider` on otherwise identical prims
  whose `omni:no_collision` attributes are authored false and true;
  `create_triangle_mesh_square` with either or both grid dimensions zero or negative;
  and all three voxel mesh helpers with zero and negative resolutions.
- `poisson_sample_mesh`'s failure is injected at the creation call rather than at
  the lookup, on a second stage carrying no particle system, because the helper
  reuses the first system it finds and the stage the success case ran on now has
  one, which puts the creation branch out of reach there. The occupied-default-path
  case needs no injection at all: it is reached by defining an `Xform` at the path
  `particles._get_default_particle_system_path` returns, which the type-name
  search steps straight past.
- The hexahedral failure is injected by monkeypatching `codeless.try_apply_api`
  to refuse `PhysxAutoDeformableHexahedralMeshAPI` and delegate every other
  identifier. USD itself refuses an apply only for reasons -- an uneditable
  layer, an instance proxy -- that would equally refuse the applies preceding it,
  so no input to this helper reaches the branch on its own.
- Authored results are read back through `pxr` on the same stage, and the root
  layer is exported to a string to inspect what was authored rather than what
  composes.
- Each of the eight deprecated aliases is called twice over: once under
  `warnings.catch_warnings(record=True)` to collect what it raises, and once on a
  stage of its own beside its renamed target on a second stage, so the two runs
  start from identical empty stages. `warnings.simplefilter("always")` defeats
  the once-per-location deduplication that would otherwise make the second and
  later cases in a run see nothing. Five of the eight mutate the prim they are
  handed, so the comparison uses two stages rather than calling twice on one.
- Each of the three kept `camelCase` parameters is driven from one row holding
  both spellings, a value, the prim to author against and the rest of the call,
  so the warning case, the equivalence case and the two both-spellings cases
  assemble their calls from the same description and cannot drift apart in what
  they pass. The two collider helpers mutate the prim they are handed and refuse
  one that is already a collider, so the equivalence case uses two stages here
  as well. Each parameter is also passed through the corresponding deprecated
  helper alias, and the filenames of both warnings are captured.

## Then

- Every import succeeds and no import loads the ovphysx native library or requires
  a Kit module; no submodule source imports `carb`, `omni.*`, `ovstage` or
  `numpy` at module scope (REQ AC-1).
- For each of the fourteen flattened submodules, every name in that submodule's own
  `__all__` resolves flat as well, and to the same object; and every name in
  `ovphysx.utils.__all__` resolves (REQ AC-2). An independent dependency-free
  manifest maps all 138 flat names to their owners, and the suite requires its
  forward and reverse mappings to equal the loaded submodules' `__all__` lists
  exactly, with no duplicate owner. What the spellings *are* is pinned one step
  further too: a
  separate case pins `OvStageOutputCache` as the exported class and asserts no
  other entry in `__all__` carries a mixed-case spelling, so
  an unrenamed camel helper fails rather than waiting for review, and a
  companion case walks the signature of every callable on that surface and
  asserts the same of its parameter names, so an unrenamed camel parameter
  fails there too. The three kept spellings do not show up in that walk: the
  shim takes them out of `**kwargs`, so `inspect.signature` reports the renamed
  parameter alone (REQ AC-15).
  Whether a renamed helper corresponds to the right ancestor is still review's
  job. `codeless` is the one submodule AC-2 exempts, and
  a dedicated case asserts the carve-out positively: each of its eleven public
  names is reachable through `ovphysx.utils.codeless` and absent from both
  `ovphysx.utils.__all__` and the flat attribute surface.
- `test_utils_package_layout.py` confirms `utils/` is a package directory with an
  `__init__.py`, and fails if a `utils.py` or `utils.pyi` ever appears beside it,
  naming the shadowing consequence and the fix in the failure message. This is a
  filesystem check with no import of the subpackage, so it runs in an environment
  without `pxr`, which is why it sits in `python_tests/` rather than
  `utils_tests/`. It is a regression guard rather than a behavioral test: the
  condition it forbids passes a git merge and a build silently and shows up only
  as an `ImportError` at the point of use (REQ AC-2, and AC-1 for the import it
  protects).
- The same file asserts that `from ovphysx.utils import OvStageOutputCache, step_and_write_to_ovstage`
  succeeds with `pxr` unimportable, and that an authoring name under the same
  conditions fails. The second half guards the first: it proves the block is in
  force, so the pxr-free assertion cannot pass vacuously. Both run in a child
  interpreter with a `pxr`-rejecting meta-path finder installed, so the outcome
  does not depend on whether the test environment happens to have USD (REQ
  AC-1a). This is the check that fails when the submodules are imported eagerly
  in `__init__`, a condition otherwise visible only when the `output_read`
  sample runs in its USD-free venv.
- Under the same block, `dir(ovphysx.utils)` succeeds and reports the submodule
  names -- read off the package directory rather than a second list -- plus
  `OvStageOutputCache` and `step_and_write_to_ovstage`, while `ovphysx.utils.__all__` still raises naming
  `pxr`. The two cases pin AC-1a's asymmetry from both sides: `dir()` degrades so
  module discovery answers, and the star-import surface does not, so it cannot
  go partial in silence. There is no end-to-end completer case or guarantee: a
  completer can call `getattr` on a returned submodule candidate and reach its
  missing `pxr` dependency. `help()` and pydoc are outside the tolerant
  guarantee because pydoc reads `__all__`. The `dir()` case carries its own vacuity guard
  rather than leaning on the case below -- it asserts
  `add_rigid_box` is *absent* from `dir()`, which a working block guarantees and
  a lapsed one would contradict by reporting the whole flat surface (REQ AC-1a).
- Under that block, a genuinely unknown ordinary or dunder name raises an
  unchained `AttributeError`, so `getattr` with a default and `hasattr` return
  their normal module-protocol answers. A known authoring export instead
  propagates the owner's `ModuleNotFoundError` naming `pxr` through direct
  lookup, `getattr` with a default, `hasattr` and explicit `from` import, so the
  unavailable dependency cannot masquerade as an absent helper (REQ AC-1a).
- Schema introspection returns the expected property, attribute and relationship
  names for single-apply and multiple-apply schemas, including properties
  inherited through the prim definition; reports applied API schemas on a prim and
  through its ancestors and descendants; enumerates multiple-apply instance names;
  and a property cache captured before removal restores the same attribute and
  relationship values after reapplication, including replaying a multiple-apply
  snapshot onto a different instance name. The restored prim carries authored
  opinions for exactly the two attributes that were authored before the snapshot
  and for none of the other eighteen the API declares, the same holds for the
  one authored instanced attribute out of the four its multiple-apply API
  declares (REQ AC-3, AC-12).

  The relationship half of that same clause is asserted separately, and it has
  to be, because it is guarded in a different place. A snapshot taken from a
  prim carrying `PhysicsFilteredPairsAPI` with no targets authored replays
  without authoring the relationship: `HasAuthoredTargets()` stays False, the
  name is absent from `GetAuthoredProperties()`, and the exported root layer
  carries no mention of `physics:filteredPairs` at all -- an exported layer
  being where an orphaned opinion outlives the process that made it, and
  `rel physics:filteredPairs = None` the spelling the orphan takes. It also
  asserts what the snapshot recorded, as `None` targets rather than merely as
  something falsy: a falsy target list is what the authored-empty prim below
  snapshots too, so an assertion that could not tell the two apart would hold
  whatever the capture did. Dropping the `None` skip in
  `apply_api_schema_property_cache` fails this case (REQ AC-3, AC-12).

  Two further cases carry the other direction, which is the one that clause
  cannot state on its own. A relationship *authored* with an empty target list
  is an opinion of its own -- `rel physics:filteredPairs = None` is exactly what
  a caller writes to override an inherited target list with nothing -- and it
  composes to an empty target list just as an unauthored one does, so a snapshot
  storing `GetTargets()` alone recorded the same thing for both and the replay
  dropped the authored one. Each case authors the empty target list, observes
  `HasAuthoredTargets()` True and the `rel <name> = None` spelling in the
  exported layer, snapshots, removes the properties and the API, observes the
  opinion gone, reapplies and restores, and asserts the opinion is back in the
  prim and in the exported layer. One does it for the single-apply cache over
  `UsdPhysics.FilteredPairsAPI`; the other for the multiple-apply cache over
  `PhysxTendonAttachmentAPI`, which is the schema this is expressible on at all
  -- it is multiple-apply and declares a `parentLink` relationship, where
  `PhysxLimitAPI`, the schema the other multiple-apply cases use, declares none
  and any assertion over its empty relationship list is vacuous. Both fail
  against a capture that stores the target list without
  `HasAuthoredTargets()` (REQ AC-3, AC-12). `has_schema` matches an applied
  API by name and rejects an unknown one (REQ AC-3). The derived-type case that
  sat beside it went with `get_derived_schemas`: the helper is in "The dropped
  surface" now, and a case asserting the descendants of `UsdPhysics.Joint`
  through a helper that no longer exists had nothing left to cover.
- PhysX APIs applied by identifier appear in `Usd.Prim.GetAppliedSchemas()` and
  their properties round-trip, including a multiple-apply instance; an unknown
  identifier and an unknown property each raise `CodelessSchemaError` naming the
  cause; the registration helper is idempotent, registering nothing on a second
  call; and an attribute the caller omitted has no authored value but still reads
  its schema fallback. `remove_api` returns True for the applied API and leaves
  it off `GetAppliedSchemas()`, returns True again for both APIs that were never
  applied rather than raising, and raises `CodelessSchemaError` naming the
  unknown identifier -- the same cause `apply_api` reports for it. The reopened
  `.usda` carries the same applied PhysX API schemas and property values, and no
  test imports `PhysxSchema` (REQ AC-10).
- Successful `set_attr` and `set_rel` calls do not evaluate `repr` on their
  supplied value or targets, proven with accepted float and string subclasses
  whose `repr` raises. The values and targets are read back from USD, so the case
  cannot pass by skipping the write (REQ AC-10).
- An invalid `Usd.Prim` passed to a codeless operation produces
  `CodelessSchemaError` with an invalid-prim diagnosis, rather than leaking
  a native exception while trying to access the prim or its absent stage and
  edit target (REQ AC-10).
- In child interpreters with no successful schema registration and with the
  default registration call made after USD built its registry,
  `add_physx_particle_system`, `create_joint(..., "Gear", ...)` and
  `create_joint(..., "RackAndPinion", ...)` each raise
  `CodelessSchemaError` naming their concrete prim type before changing the
  stage. Each operation gets a fresh stage whose exported layer is compared
  byte for byte before and after the call (REQ AC-5, AC-7, AC-10).
- On a layer that is not editable, `try_apply_api` and `remove_api` answer
  `False` for an identifier the registry knows, the prim keeps the API the
  refused removal named, and `apply_api` raises a `CodelessSchemaError` that
  reports the layer and does *not* report the identifier as unknown. The
  unknown identifier still raises with its own cause on that same layer, so the
  two are shown distinct rather than one of them merely reachable. `set_attr`
  and `set_rel` raise `CodelessSchemaError` naming that layer and leave the
  attribute with no authored value and the relationship with no authored
  targets, and a patched write answering `False` raises the same way -- a
  refused write is never reported as a write, whichever way USD reports the
  refusal (REQ AC-10).
- A valid identifier whose instance-name usage contradicts the schema raises
  rather than answering `False`, for `try_apply_api` and `remove_api` alike:
  a single-apply identifier given an instance name, a multiple-apply one given
  none, and a multiple-apply one given the empty string, which USD allows no
  schema. Three parametrized cases, each asserting the message names the
  usage and does *not* report the identifier as unknown. It is the case that
  separates asking the registry about the call from asking it about the
  identifier: the identifier resolves in both, and only the pair does not (REQ
  AC-10).
- In the child that opens a stage first, `register_schemas(verify=True)` raises
  `CodelessSchemaError` rather than returning as if it had worked, and the
  `CodelessSchemaError` a subsequent `apply_api` raises says the process cannot
  be repaired and names `PXR_PLUGINPATH_NAME` as a recovery, so the message
  cannot be read as an instruction to call `register_schemas()` again here. A
  `remove_api` in the same child raises with the same two clauses, which is the
  unregistered-schemas half of AC-10's removal contract (REQ AC-10). In
  the child that registers first, the same verified call returns and the API
  applies (REQ AC-14). Both cases degrade where the unregistered branch is
  unreachable: an environment that already carries the schemas on its plugin
  path from process start, and a layout with no staged schema tree. They degrade
  through the conftest's fail-or-skip helper, so under
  `OVPHYSX_REQUIRE_CODELESS_SCHEMAS=1` an unreachable branch is a failure rather
  than a skip -- previously they called `pytest.skip` themselves and stood
  outside the guard.
- In the child that registers the half-staged tree, the PhysX root reaches the
  registry and an API from it applies, while `register_schemas(verify=True)`
  raises naming `OmniPhysicsDeformableBodyAPI` and not `PhysxRigidBodyAPI`,
  `schema_is_registered()` answers False, and applying the deformable API reports
  the registration failure -- "cannot be repaired", `PXR_PLUGINPATH_NAME` -- and
  not the unknown-identifier message that would send the caller looking at their
  spelling. That is what the second sentinel identifier buys, and it is asserted
  nowhere else: a probe of the PhysX root alone would call this state registered
  (REQ AC-10, AC-14). The case degrades where the deformable schemas are on the
  plugin path from process start, since the partial state is unreachable there --
  through the same fail-or-skip helper, for the same reason.
- No public `set*` or `remove*` helper exposes a `custom_execute_fn` or
  `execute_command_fn` parameter (REQ AC-11).
- Each of the eight deprecated aliases resolves as `ovphysx.utils.<oldName>` and
  by explicit `from ovphysx.utils import <oldName>`, while being absent from
  `__all__`, from `dir(ovphysx.utils)`, from the owning submodule's `__all__` and
  from what `from ovphysx.utils import *` binds -- with the new spelling asserted
  present in that same star-import, so the case cannot pass by the star-import
  having bound nothing. Each alias's `__wrapped__` is its target, and its
  signature and docstring equal the target's. `ovphysx.utils._DEPRECATED_ALIASES`
  holds exactly those eight names (REQ AC-15).
- Called for real, each alias raises exactly one `DeprecationWarning`, whose
  message names both the old and the new spelling, and produces the same return
  value and the same exported root layer as its renamed target run from the same
  starting point. Calling the new spelling under
  `simplefilter("error", DeprecationWarning)` does not raise, so the deprecation
  attaches to the alias and not to the helper (REQ AC-15).
- Passed by keyword, each of the three kept `camelCase` parameters raises
  exactly one `DeprecationWarning` naming both spellings, and produces the same
  return value and the same exported root layer as the renamed spelling run from
  the same starting point -- which is what shows the value reached the helper
  rather than being warned about and dropped, leaving the helper on its default.
  Passing both spellings at once raises `TypeError` naming both, whether the
  renamed one arrives as a keyword or positionally, and the positional case
  additionally asserts that *no* `DeprecationWarning` was raised: without the
  positional half of the guard the call still fails, but on Python's own
  duplicate-argument error and after having told the caller to migrate to the
  spelling they used. The discovered shim set equals the set of rows exercised,
  so a fourth shim cannot be added without a case. For all three parameters,
  passing the old spelling through the helper's deprecated alias emits two
  warnings and both name the test file as their origin, rather than the private
  wrapper module (REQ AC-15).
- `poisson_sample_mesh` returns a non-empty particle system path on success, the
  same path when called twice on one stage, creates only one particle system, and
  returns an empty path when the particle system cannot be resolved. It returns
  the empty path and logs an error, rather than raising, when its default path is
  already occupied: nothing is authored -- no sampling API on the mesh, and the
  occupying prim keeps its own type -- where the omni.physx original reached its
  creation call's precondition assert and raised `AssertionError` instead.
- The three particle-set constructors are given a path already held by a
  `UsdGeom.Cube` carrying an authored `size`, and each is asserted to raise
  `ValueError` and to leave that prim's type and its `size` untouched, which is
  what makes the case about the guard rather than about the exception class
  alone: the stripped `assert` cost a prim, not a message, and a `Define` over
  it produces something that reads as a valid prim of two types at once.
  Nothing else in the suite runs under `-O`, and the property being fixed is
  only observable there, so a second case launches a child interpreter with
  `-O`, checks `__debug__` without an `assert` -- `-O` would compile out the
  check that `-O` is in force -- and requires the same rejection and the same
  untouched prim from it. That child needs no schema registration, because the
  guard rejects before any codeless call is reached, which is also why it can
  be a bare `-c` program rather than a staged environment (REQ AC-12).
- `add_plane_collider` is given a path already held by a `UsdGeom.Cube` with an
  authored `size`; it raises `ValueError`, leaves the exported layer byte for
  byte unchanged, preserves the cube type and size, and applies no collision
  API (REQ AC-5, AC-12).
- The auto volume deformable hierarchy returns True and carries
  `PhysxAutoDeformableHexahedralMeshAPI` when the hexahedral request succeeds,
  False and without that API when the apply is refused, True again for the same
  call with no hexahedral request under the same injected refusal -- which is what
  shows the False came from the hexahedral branch and not from the monkeypatch at
  large -- and False for a Gprim root rather than half-building it;
  `add_pair_filter` never targets a prim at itself for any of the three input
  shapes and adds each intended target exactly once; `remove_pair_filter` leaves no
  `UsdPhysics.FilteredPairsAPI`, no `physics:filteredPairs` relationship and no
  mention of that relationship anywhere in the exported root layer, and a later
  `add_pair_filter` naming a different prim targets only that prim rather than
  appending to the removed target list; `remove_collider` leaves no mesh approximation
  API on an instanceable prim, and leaves neither
  `PhysxSDFMeshCollisionAPI` nor `PhysxSphereFillCollisionAPI` on a mesh authored
  with the `sdf` or `sphereFill` approximation, with the same case asserting that
  `MESH_APPROXIMATIONS` still maps those two tokens to those two APIs; and
  `set_collider` on a prim that already carries `PhysicsCollisionAPI` applies
  neither a collision API nor a second approximation API, proven both against a
  prim whose `UsdPhysics.CollisionAPI` came from a direct `Apply` and against one
  `set_collider` itself authored, where a second call with a different
  approximation leaves the first approximation in place. An authored false
  `omni:no_collision` attribute permits the collider authoring, while an
  authored true value leaves the exported layer unchanged (REQ AC-5, AC-12).
- Each auto deformable hierarchy helper is given a point-based visual prim with
  no authored points at default time. Both return `False`, log the visual path
  and the missing default-time points, and leave the exported layer byte for
  byte unchanged, proving the validation runs before teardown or authoring (REQ
  AC-8, AC-12).
- Five parametrized single-prim cases omit each required default-time geometry
  value in turn: volume `points` and `tetVertexIndices`, then surface `points`,
  `faceVertexCounts` and `faceVertexIndices`. Each helper returns `False`, logs
  the missing property and prim path, and leaves the exported layer byte for
  byte unchanged, proving validation precedes both topology work and API
  application (REQ AC-8, AC-12).
- After `remove_collider_subtree`, no prim in the subtree carries
  `UsdPhysics.CollisionAPI`, `UsdPhysics.MeshCollisionAPI` or any applied schema
  whose name mentions collision or cooked data -- the instanceable prim
  included, which is the defect: the omni.physx original walked gprims only, so
  the collider `set_collider_subtree` had applied to an instanceable prim survived
  the removal, and `remove_collider`'s own instanceable handling was unreachable
  from a subtree. The Scope that was never a collider is left with no applied
  schema at all, the instanceable prim is still instanceable, and the exported
  root layer carries no `prepend apiSchemas` and at least one `delete
  apiSchemas` opinion, which is the form the removal has to take to cross the
  reference.   `remove_rigid_body_subtree` leaves the root without
  `UsdPhysics.RigidBodyAPI` or `PhysxRigidBodyAPI` and the same subtree clean
  (REQ AC-12, AC-5).
- `ancestor_has_api` and `descendant_has_api` stop at a prim that resets its xform
  stack: with the API applied on the far side of the reset, each answers False,
  and each answers True over the same hierarchy without the reset, so the
  negative comes from the boundary rather than from a walk that found nothing.
  The spelling is pinned beside the behaviour -- the middle prim's `xformOpOrder`
  is asserted to be `["!resetXformStack!", "xformOp:translate"]`, and
  `HasAttribute("xformOp:reset")` asserted False on that same prim -- because a
  guard that reads plausibly is how the original survived unnoticed: it tested an
  attribute USD does not define, so neither walk ever stopped where its
  documentation said it would. A non-Xformable `Scope` reaches the guard and the
  descent continues past it rather than raising (REQ AC-12, AC-3).
- **No case asserts a substring of a docstring.** The contract statements AC-12
  requires are not test oracles: an assertion of that kind verifies that a
  string appears in a docstring, which no caller depends on, and in exchange
  makes the API documentation unrewritable without editing a suite that is not
  about behavior. Each of the twenty-eight fixed defects is covered by the
  behavioral or structural regression assertions in the cases in this list
  instead. The one `__doc__`
  reference in the suite is `test_utils_surface.py`'s
  `alias.__doc__ == target.__doc__`, which checks that an alias mirrors its
  target's metadata rather than what the docstring says.
- `remove_collider` takes the approximation and cooked-data APIs' authored
  properties with the APIs, and deliberately leaves `PhysxCollisionAPI`'s. Two
  cases pin the split, because it is the whole of that defect. The first
  authors a `convexHull` collider with a `physxConvexHullCollision:hullVertexLimit`,
  a `physxCookedData:<token>:buffer` and both `physxCollision` offsets, removes
  the collider, and asserts the hull limit and the cooked buffer are no longer
  authored while the two offsets survive; it then re-approximates as
  `convexDecomposition` and asserts the exported layer carries no
  `physxConvexHullCollision` or `physxCookedData` opinion at all, since an
  exported layer is where an orphan outlives the process that made it. The second
  executes the worked example in `create_api_schema_property_cache`'s docstring
  end to end -- snapshot, `remove_collider`, explicit
  `remove_api_schema_properties`, re-approximate, restore -- and asserts the prim
  ends carrying exactly the two offsets and nothing else under `physx`. That
  example is the one place the split is written down for a caller, so it is
  executed rather than trusted (REQ AC-12).
- `add_capsule`, `add_cylinder` and `add_cone` author an extent that matches the
  shape, on all three axes: nine parametrized cases over radius 0.25 and height
  1.5 assert the authored `extent` equals a bound spelled out in the case
  itself -- `radius` across the axis, `height / 2` along it for the cylinder and
  the cone, and `height / 2 + radius` along it for the capsule's caps -- and
  equals the bound USD computes for the same shape defined on a throwaway prim
  with no `extent` authored, which is the schema's own answer arrived at without
  going through the helper. The old expression is pinned as a negative in the
  same case, so none of the nine can pass against
  `[(-radius, -radius, -height), (radius, radius, height)]`: it is the wrong
  size on Z and the wrong shape on X and Y (REQ AC-12).
- `remove_deformable_body` leaves no deformable API and no authored bind-pose or
  `physxDeformableBody` property on any prim of the subtree it is given, over a
  hierarchy whose skin mesh is nested two levels below the root -- which is
  where the children-only walk left `OmniPhysicsDeformablePoseAPI:default` and
  its points and purposes, and the depth the two
  `create_auto_*_deformable_hierarchy` helpers actually author over. The
  exported root layer carries no `deformablePose` property and no `prepend
  apiSchemas` naming a deformable API, only the `delete apiSchemas` opinions the
  removal has to take. A second case hands the helper the simulation mesh's own
  path rather than the root's and asserts that prim's own sim and pose APIs and
  their properties go, since the children-only walk skipped the supplied prim as
  well as the deep ones, while the root above it keeps
  `PhysxAutoDeformableBodyAPI` -- so the sweep did not climb out of the subtree
  it was handed (REQ AC-12, AC-8).
- A collider the helpers did not author, sitting as an immediate child of a
  deformable root, keeps `UsdPhysics.CollisionAPI` and its authored
  `physics:collisionEnabled` through `remove_deformable_body` and through both
  builds of each `create_auto_*` hierarchy over that subtree -- the creation
  path being the one a caller meets first, since both helpers run the teardown
  before rebuilding. Four cases, over the volume and the surface layout on each
  path. The exported root layer is checked to still carry
  `physics:collisionEnabled`, since an exported layer is where a destroyed
  opinion is destroyed for good. No case can pass by a removal that takes
  nothing: each removal-path case asserts the simulation mesh keeps no
  deformable API, and then, in the surface layout, that it loses the collision
  API the helper applied beside its simulation API, or, in the volume layout,
  that the generated collision mesh keeps the one no simulation API accounts for
  (REQ AC-12, AC-8).
- Sixteen further cases fix the rule in place, in both directions.

  Five pin what a deformable simulation API accounts for. A volume hierarchy
  built with one path for both meshes loses that mesh's collision API to a
  removal asked for the root, the helper having applied it there itself; the
  case fails if the volume simulation API is read only on the prim the removal
  names. A collision API the caller applied to a hierarchy's simulation mesh
  goes the same way in both layouts, that prim being indistinguishable from the
  merged one, which is the loss AC-8 records as accepted. And a single-prim
  volume body, the one layout where `set_physics_volume_deformable_body` applies
  both APIs to one prim, loses its collision API to a removal asked for that
  prim. And a standalone single-prim volume body authored two levels below a
  root, carrying the auto-hierarchy APIs and a `physxDeformableBody` property as
  well, is torn down completely by a removal asked for that root: no prim of the
  subtree keeps a schema or property naming a deformable family, and that prim
  keeps no applied schema at all, `OmniPhysicsBodyAPI` included -- which names no
  deformable family and so is asserted on its own, being a built-in of
  `OmniPhysicsDeformableBodyAPI` that has to go when that one does. The
  subtree half of that oracle matches on the naming families rather than on a
  list of API names, so an API added to the deformable schemas and missed by the
  teardown fails it. Two earlier shapes of the removal each fail it: an
  ownership scan bounded to the root's immediate children leaves the collision
  API on a prim whose simulation API the strip loop has already taken, and a
  teardown that applies the body-API and `physxDeformableBody` steps to the
  supplied prim alone leaves five APIs on the deep prim.

  Six pin the paths a build's teardown exempts. A collision API the caller
  authored at a path a helper generates -- the volume collision mesh, the
  surface simulation mesh -- survives the first build and a rebuild. A build
  that aborts after its teardown, a `UsdGeom.Camera` root refused by the mesh
  simplification step, leaves the collision API and the authored
  `physics:collisionEnabled` at the simulation mesh path it had claimed; that
  case fails if the exemption is dropped, since no return after the teardown
  reapplies the API. And a rebuild that moves the simulation mesh to a new path
  leaves no collision API at the old one, in both layouts, the volume one built
  merged so that its abandoned mesh is a collider the helpers own; those two
  fail if the teardown takes nothing, which left a second enabled collider in
  the subtree. Their abandoned mesh is given authored points first, because the
  rebuild's bind-pose pass reads them and a generated mesh has none before
  cooking. The sixth is the other half of that rebuild: given a new
  `collision_tetmesh_path` as well, a volume rebuild leaves the old collision
  mesh's collision API enabled, which is the limitation AC-8 records and the
  Known coverage gaps size. It passes against the code as it stands, so it is a
  pin on the current behaviour rather than a regression.

  Three put the same ownership question to a *property* rather than an API,
  which is where the two share a mechanism: a property outlives the removal
  exactly when an API that survives it declares it. A prim holding
  `omniphysics:deformableBodyEnabled` and `omniphysics:mass` with no deformable
  API applied -- the state `codeless.remove_api` leaves, since it drops the
  `apiSchemas` entry and not the properties -- loses both, at the root and two
  levels down; that case fails against a strip conditioned on the API being
  applied. A prim carrying `OmniPhysicsBodyAPI` in its own right *and*
  `OmniPhysicsDeformableBodyAPI` keeps the body API and all three properties the
  two share -- `omniphysics:simulationOwner` among them, so a relationship and
  its targets are covered as well as two attributes -- and still loses
  `omniphysics:mass`, which only the deformable one
  declares; that case fails against an unconditional strip of the deformable
  API's property set, which is what the supplied prim used to get. The third
  pins the other side, so keeping a caller's application cannot pass by keeping
  every application: where `OmniPhysicsBodyAPI` is present only as the built-in,
  the prim ends with no applied schema at all. It passes against the code as it
  stands and is a pin rather than a regression (REQ AC-8).

  A fourth asks what the strip reaches rather than what it keeps. Removing a
  name no layer authored changes nothing, so the teardown is never to be offered
  one. The case gives a prim an authored property of the API being removed, an
  authored `physxDeformableBody` property, a property the same API declares but
  which is left at its schema default, and a property in the look-alike
  `physxDeformableBodyExtra` namespace; it then counts the names reaching
  `Usd.Prim.RemoveProperty` and requires exactly the two authored ones. The
  count is what makes it a regression rather than a pin: the resulting stage is
  the same either way, since removing an unauthored name is a no-op, so a case
  reading only the stage cannot tell the two apart. An earlier teardown offered
  68 names where this one offers two (REQ AC-8).

  One covers `OmniPhysicsCurvesDeformableSimAPI` and its three rest attributes.
  No helper here applies it -- the schema calls curves a work in progress -- so
  it is the case that separates "every deformable API" from "every API this
  module authors". It passes against the code as it stands only because the
  same change added the API to the teardown's family (REQ AC-8).
- Each of the six `add_rigid_*` shape constructors called with `density=0.0`
  yields a prim carrying `UsdPhysics.RigidBodyAPI`, `UsdPhysics.MassAPI` and
  `UsdPhysics.CollisionAPI`, with `physics:density` authored at 0.0 and both
  velocity attributes authored, where the original applied neither the rigid
  body nor the mass API and returned a bare collider. The same call with a
  non-zero density is asserted in the same case, so it cannot pass against a
  guard inverted rather than removed, and so is the `add_collider_*` sibling
  the docstring sends a caller to -- collision API, no rigid body and no
  mass API -- since that is the route out for anyone who was relying on the old
  reading (REQ AC-12, AC-5).
- `extract_triangle_surface_from_tetra` over three tetrahedra sharing one face
  returns the nine faces that occur once and not the shared one, which is
  asserted by name against the input's own vertex numbering rather than by
  count alone, the helper reindexing its output. Over two tetrahedra sharing
  the same face it returns six triangles, which is where the parity algorithm
  and this one agree and is what shows the fix changes nothing a caller with a
  well-formed mesh sees. Against the original the first assertion reports ten
  (REQ AC-12, AC-6).
- `verify_tetra_mesh` warns once for a vertex index below zero and returns,
  in the same voice and on the same terms as for an index past the end of the
  point list, and warns for neither over a valid mesh. The five points are
  chosen so that the wrapped tetrahedron `[-1, 1, 2, 3]` has a positive volume,
  asserted in the case: that is what leaves the range check as the only thing in
  the helper that can object to it, and it is why the original reported nothing
  at all rather than reporting the wrong problem (REQ AC-12).
- `set_or_add_translate_op` followed by
  `setup_transform_as_scale_orient_translate` leaves exactly the ops
  `xformOp:translate`, `xformOp:orient`, `xformOp:scale`, in that order;
  `get_translation` returns the authored translation before and after the
  rewrite; `get_axis_aligned_vector("Z", 2.0)` returns `(0, 0, 2)`;
  `get_unit_scale_factor` returns the reciprocal of the stage's own
  `metersPerUnit`, asserted against a deliberately unusual authored value of 0.5
  rather than merely being positive; and `get_world_position` returns the centre
  of the prim's world bound, compared against the position the prim was authored
  at. The world-position case needs a prim with geometry to say anything: over a
  bare `add_rigid_xform` the world bound is empty and the helper answers the
  origin however the prim is translated, which is how an `is not None` assertion
  held there for as long as it did. `get_basis` returns the spelled-out
  up/forward/right triple for both the `Y` and the `Z` up-axis, each one
  right-handed under `Gf.Cross` and each forward equal to `get_forward_vector`'s
  for the same axis -- a basis that is mirrored, or whose forward disagrees with
  the other helper, would place a caller's camera or joint frame in a mirrored
  world and nothing else here would notice. A non-Xformable prim produces a logged
  warning and no exception, observable through `caplog` because `carb.log_warn`
  became standard `logging` (REQ AC-4).
- `setup_transform_as_scale_orient_translate` and
  `copy_transform_as_scale_orient_translate` both execute without entering an
  `Sdf.ChangeBlock`; the raising test double pins that structural rule directly,
  while the test also asserts the rewritten op stacks and the copied local
  transform. Composed stage reads cannot occur while change processing is deferred
  (REQ AC-4, AC-12).
- `setup_transform_as_scale_orient_translate` accepts a bare `Usd.Prim`, rewrites
  its op stack to translate-orient-scale, preserves its translation, and carries
  the matching `Usd.Prim | UsdGeom.Xformable` input annotation (REQ AC-4).
- `copy_transform_as_scale_orient_translate` accepts every combination of
  `Usd.Prim` and `UsdGeom.Xformable` for source and destination, with matching
  input annotations. For both reset-stack values, the destination receives the
  source's translation, rotation, nonuniform scale and reset flag in a
  translate-orient-scale stack. The source's matrix, op order and reset flag
  remain unchanged (REQ AC-4).
- The representative constructor case in `test_utils_authoring.py` runs
  `add_rigid_box`, `add_collider_sphere`,
  `add_box`, `add_ground_plane` and `add_rigid_xform`: each yields a prim of the
  expected `UsdGeom` type, `add_box` at a path that does not collide with the
  `add_rigid_box` already there, `add_collider_sphere` carrying
  `UsdPhysics.CollisionAPI` and not `RigidBodyAPI`, and `add_rigid_box`
  additionally `UsdPhysics.RigidBodyAPI` and `UsdPhysics.MassAPI` with the
  requested density and linear velocity. `add_rigid_body_material` returns `True`
  -- `is True`, not `is not None`, since the helper answers `False` when the path
  is occupied by a non-Material prim and `False is not None` holds --
  `add_physics_material_to_prim`'s binding is read back through
  `UsdShade.MaterialBindingAPI` and asserted to name the material under the
  `physics` purpose, the purpose mattering as much as the target because a
  physics material bound under the default purpose is overridden by any render
  material on the same prim; `add_physics_scene`, `add_collision_group` with its
  add / query / remove membership cycle, `add_pair_filter` / `remove_pair_filter`,
  `create_joint` for a `Revolute` joint, and the local-space-velocities metadata
  set / read / clear cycle all run; `remove_collider` and `remove_rigid_body` leave
  no collision or rigid-body API on a mesh prim (REQ AC-5).
- `set_physics` completes a pre-applied `UsdPhysics.RigidBodyAPI`: from a body
  authored disabled with either kinematic state, it applies
  `PhysxRigidBodyAPI`, enables the body and authors the opposite requested
  kinematic state (REQ AC-5, AC-12).
- The capsule, cylinder and cone constructors each yield a prim of the matching
  `UsdGeom` type carrying the requested radius, height and axis, no collision API
  on the plain one, `UsdPhysics.CollisionAPI` without `RigidBodyAPI` on the
  `add_collider_*` one, and both plus `UsdPhysics.MassAPI` with the requested
  density and linear and angular velocity on the `add_rigid_*` one. The plain
  prim's display color and its `translate` / `orient` / `scale` op stack are
  asserted as well, which are the two clauses AC-5 names that the box and sphere
  cases above leave unread (REQ AC-5).
- `create_mesh_cube`, `create_mesh_cylinder` and `create_mesh_cone` each yield a
  `UsdGeom.Mesh` with non-empty points, face vertex counts and indices whose
  counts sum to the index count; `create_tetra_voxel_box` yields an index count
  divisible by four; `calculate_tetra_volume` returns 1/6 for the unit corner tetrahedron;
  `extract_triangle_surface_from_tetra` and `convert_tetra_to_triangle_soup` return
  non-empty results; `create_triangle_mesh_square(2, 2)` returns 9 points and 24
  indices; and `compute_bounding_box_diagonal` accepts `Gf.Vec3f` input and
  returns the expected length. The same helper raises `ValueError` for both an
  empty list and an empty iterator. `create_triangle_mesh_square` returns empty
  point and index lists when either or both dimensions are zero or negative.
  All three voxel mesh helpers return empty point and index lists for zero and
  negative resolutions (REQ AC-6, AC-12).
- `create_mesh_square_axis` for the `Y` axis yields four points flat in `Y`, a
  single quad face, one normal per point pointing up the axis, the reversed
  winding that makes the front face point that way, and `doubleSided` false; for
  the `Z` axis it additionally carries a four-element `st` primvar, which is the
  textured variant `add_quad_plane` builds on. `create_mesh_concave` yields ten
  points, one normal each, a face-vertex-count sum equal to the index count, no
  index out of range, and the two mid-plane points pulled in towards the far face
  that are what make the shape concave rather than a box (REQ AC-6).
- `verify_tetra_mesh` is asserted through `caplog`, which is the only thing it
  can be asserted through: it returns nothing and only emits `logger.warning`, so
  called without a log assertion it cannot fail for any input, well-formed or
  corrupt. Each of its three checks is given the input that trips it -- an index
  count that is not a whole number of tetrahedra, an index past the end of the
  point list, and a tetrahedron wound so its signed volume is negative -- and the
  well-formed `create_tetra_voxel_box` output is asserted to produce no records at
  all, so a helper that warned unconditionally would fail too.
  `fixup_tetra_mesh_volumes` is then given the inverted tetrahedron the third
  check reports: it returns the re-wound index list, `calculate_tetra_volume` over
  the result comes back at +1/6 where the input measured -1/6, and
  `verify_tetra_mesh` over the repaired mesh is silent (REQ AC-6).
- `add_physx_particle_system` authors a `PhysxParticleSystem` prim whose
  `contactOffset` is the value passed and whose `restOffset`, which the caller
  omitted, has no authored value; `add_pbd_particle_material` applies
  `PhysxPBDMaterialAPI`; `create_particles_grid` returns the expected position
  and velocity counts and the requested velocity;
  `add_physx_particleset_points` yields a `UsdGeom.Points` prim carrying
  `PhysxParticleSetAPI` and `add_physx_particleset_pointinstancer` a
  `UsdGeom.PointInstancer`; and each of the four feature helpers is asserted
  individually, `PhysxParticleAnisotropyAPI`, `PhysxParticleSmoothingAPI` and
  `PhysxParticleIsosurfaceAPI` on the system and `PhysxDiffuseParticlesAPI` on the
  point set. `add_physx_particleset_pointinstancer` now takes either path
  spelling, as its two siblings always did, so both halves of that domain are
  covered: this case supplies an `Sdf.Path`, and a case of its own supplies a
  `str` and reads the prototype relationship's targets back, since the
  prototype paths are what the helper built from `path.pathString` before it
  converted and are therefore the part a `str` used to break (REQ AC-7).
- `add_deformable_material` applies `OmniPhysicsDeformableMaterialAPI` and
  `add_surface_deformable_material` applies
  `OmniPhysicsSurfaceDeformableMaterialAPI`;
  `create_auto_volume_deformable_hierarchy` returns True and leaves both
  `PhysxAutoDeformableHexahedralMeshAPI` and
  `PhysxAutoDeformableMeshSimplificationAPI` on the root; and
  `remove_deformable_body` leaves no applied schema whose name contains
  `Deformable` on that same root prim (REQ AC-8).
- `set_physics_surface_deformable_body` on a triangulated mesh applies
  `OmniPhysicsDeformableBodyAPI`, `OmniPhysicsSurfaceDeformableSimAPI` and
  `UsdPhysics.CollisionAPI`, and authors `omniphysics:restShapePoints` equal to
  the mesh's own points and one `omniphysics:restTriVtxIndices` entry per
  triangle. On a quad mesh it returns False and authors nothing at all, which is
  the contract that matters: the surface sim API is defined over triangles only
  (REQ AC-8).
- `create_auto_surface_deformable_hierarchy` leaves
  `PhysxAutoDeformableBodyAPI`, `PhysxAutoDeformableMeshSimplificationAPI` and
  `OmniPhysicsDeformableBodyAPI` on the root with
  `physxDeformableBody:cookingSourceMesh` targeting the source mesh; the
  generated simulation mesh is a `UsdGeom.Mesh` carrying
  `OmniPhysicsSurfaceDeformableSimAPI`, `UsdPhysics.CollisionAPI` and the `guide`
  purpose that hides it in favour of the skin mesh; and the skin mesh carries
  `OmniPhysicsDeformablePoseAPI:default` with a bind-pose point array equal to
  its own points (REQ AC-8).
- Each deformable remove-side helper drops its own API and nothing else:
  `remove_auto_deformable_hexahedral_mesh` and
  `remove_auto_deformable_mesh_simplification` each leave
  `PhysxAutoDeformableBodyAPI` in place, `add_auto_deformable_mesh_simplification`
  puts the simplification API back, and `remove_auto_deformable_body` takes the
  auto body, the simplification API it owns and every authored
  `physxDeformableBody` property -- including a custom one the API does not
  declare, which is what the namespace sweep beside the schema-property removal
  is for -- with it, while leaving the
  `OmniPhysicsDeformableBodyAPI` it fed -- that one is a separate API and
  `remove_deformable_body` is what takes it. `remove_deformable_body` then leaves
  no `Deformable`-named applied schema on any prim of that hierarchy and no
  authored `deformablePose` property on the skin mesh, which is AC-8's "root
  prim and its children" clause asserted on the children too. The generated
  collision mesh, the one prim of this layout the helper gives
  `UsdPhysics.CollisionAPI`, keeps it: a removal asked for the root has nothing
  that tells that prim from a collider of the caller's own (REQ AC-8).

  That hierarchy is one level deep -- its skin mesh is an immediate child of the
  root -- so "any prim of the hierarchy" is all this case can say for the
  hierarchy it builds, and the general claim is not one it was ever making. The
  general claim is AC-12's, and the two cases in
  `test_utils_bugfixes.py` are what carry it: the removal is asserted over a
  hierarchy whose skin mesh is two levels down, which is where the
  children-only walk left an `OmniPhysicsDeformablePoseAPI` instance behind, and
  over the simulation mesh's own path, which is where it left the supplied
  prim's (REQ AC-8, AC-12).
- Every one of the 67 names REQ AC-9 enumerates -- the 38 of "The excluded
  surface", the 17 of "The dropped surface", the three of "The privatized
  surface" and the nine of "Superseded before
  the census" alike -- is absent from
  `ovphysx.utils`, as an attribute and from `__all__`; the four lists are the
  asserted lengths and are pairwise disjoint; and no submodule source imports
  `carb` or
  any `omni.*` module at any scope (REQ AC-9). Each enumeration is asserted whole
  rather than sampled, which is what the length assertions are for.

  The three privatized names carry a second assertion the other 64 do not:
  `paths._create_unused_path`, `mesh._add_triangle` and
  `particles._get_default_particle_system_path` are each asserted present and
  callable in their own submodule, and absent from that submodule's `__all__`.
  REQ AC-9 states why.

  Against
  those, the seven survivors are asserted present and callable:
  `add_physics_scene`, `create_tetra_voxel_box`, `create_tetra_voxel_sphere`,
  `poisson_sample_mesh`, `set_local_space_velocities`,
  `clear_local_space_velocities` and `has_schema`. Absence alone would be
  satisfied just as well by a drop that deleted the caller with the name it
  called, and that is the failure mode a re-audit actually risks.
- The subpackage and every submodule import successfully with `ovstage` and
  `numpy` unimportable; no submodule source carries a module-scope import of
  `ovstage` or `numpy`, nor one of `ovphysx.api`. A module-scope import of the
  native-free `ovphysx.types`, and an `ovphysx.api` import under
  `if TYPE_CHECKING:`, both pass this check. This is what keeps a
  simulation-driving submodule from silently making the whole authoring surface
  unimportable for a caller who has `pxr` but no ovstage runtime (REQ AC-13, and
  AC-1 for the subpackage as a whole).
- `simulation` is the one submodule of that kind today, and
  `OvStageOutputCache` and `step_and_write_to_ovstage` resolve both flat and
  through it on the same terms
  as every other public name (REQ AC-13, AC-2). Its *behavior* is covered by
  TEST-PYTHON-FRAME-001, not here; `test_step_and_write_to_ovstage.py` needs a
  built native tree and the ovstage runtime, so it runs in the native Python test
  stage rather than alongside the pure-`pxr` authoring cases above.

## Test code and coverage map

| File | Covers |
|---|---|
| `ovphysx/tests/python_tests/utils_tests/conftest.py` | AC-10 (schema registration), the half-staged tree AC-14's partial-registration case needs, and the shared `stage` fixture |
| `ovphysx/tests/python_tests/utils_tests/test_utils_surface.py` | AC-1, AC-2's flat-export owner manifest and surface, AC-9, AC-11, AC-13, AC-15's surface half (the bounds of eight names and three parameters, the exclusion from `__all__` / `dir()` / `import *`, the preserved signature, and the absence of camel spellings from the documented surface -- helper names and parameter names alike) |
| `ovphysx/tests/python_tests/utils_tests/test_utils_authoring.py` | AC-4, including execution and result checks for both transform-stack rewrites, AC-5, AC-6, AC-7, AC-8, AC-10, and AC-12's `Sdf.ChangeBlock` transform regression |
| `ovphysx/tests/python_tests/utils_tests/test_utils_schema.py` | AC-3, AC-14, AC-10's invalid-prim and remove-side diagnostics, concrete-prim preflight under no and late registration, successful property writes without eager `repr`, and its refusal clause -- the bool a known identifier gets on a layer that is not editable, and the raise a refused property write gets -- AC-12's property-cache clause, and AC-4's world-position clause, which is asserted nowhere else -- and which this file now genuinely carries: the case compares the helper's answer against the position the prim was authored at, where it previously asserted only that the answer was not `None` over a prim with no bound, which the origin satisfies for every input |
| `ovphysx/tests/python_tests/utils_tests/test_utils_deprecated_aliases.py` | AC-15's behavioral half: each alias and each kept `camelCase` parameter is exercised for real and compared against its renamed counterpart; every alias-plus-parameter warning pair is attributed to the caller |
| `ovphysx/tests/python_tests/utils_tests/test_utils_bugfixes.py` | AC-12, AC-5's subtree, `omni:no_collision` value, occupied-plane guard and unconditional and pre-applied rigid-body clauses -- the only place the `add_rigid_*` family is asserted at a zero density -- AC-3's ancestor/descendant walk clause, AC-8's hierarchy-visual and single-prim required-geometry preflights and deformable-removal clause below one level of children -- which the deformable case in `test_utils_authoring.py` cannot reach, its hierarchy being one level deep -- and AC-8's collision-API ownership clause, on the removal path and on both `create_auto_*` paths, plus AC-6's `Gf.Vec3f`, empty-bound, non-positive grid and surface-extraction clauses, and AC-4's non-Xformable warning and bare-prim clauses, each of which is asserted nowhere else |
| `ovphysx/tests/python_tests/test_utils_package_layout.py` | AC-1, AC-1a, AC-2 (shadowing guard, pxr-free import path, known-export dependency failures versus genuinely unknown attributes, and AC-1a's tolerant `dir()` / strict `__all__` asymmetry; no `pxr` needed) |
| `ovphysx/tests/python_tests/test_type_stubs.py` | AC-2 (the stub for `ovphysx.utils.simulation` is present at the in-package path; no `pxr` needed) |

`utils_tests/` runs as its own pytest invocation, orchestrated by
`scripts/test_python_runtime.cmake`, which also `--ignore`s it from the main run.
That separation is required, not tidiness: USD builds its schema registry once, on
first access, so a registration arriving after any other test has opened a stage is
silently ineffective. Only a process that runs nothing else can guarantee the
registration is first.

## Known coverage gaps

The measurements below are the authoring-suite snapshot from before
`OvStageOutputCache` was added. They retain that snapshot's denominators; the
cache behavior is covered separately by TEST-PYTHON-FRAME-001.

Recorded so the rows above are not read as claiming more than they cover. The
suite is a breadth sample of each AC's helper family, not a per-helper sweep, and
the scale of that is worth stating plainly, in two figures restated on
2026-09-09 against that snapshot, after three names were dropped
and three privatized. The word-match figure was re-measured directly and the
profiling figure comes from a fresh run; which is which is said per row, because
they are not the same standard of evidence.

- **110 of the 126 callable public names in `ovphysx.utils.__all__` are actually
  executed by the suite; 16 are never called at all.** The underlying measurement
  is the 2026-09-09 profiling run: a
  `sys.setprofile` hook recorded every code object entered during a full run of
  `utils_tests/`, `test_utils_package_layout.py` and `test_type_stubs.py` with
  the codeless schemas staged. Each public callable was passed through
  `inspect.unwrap` before its `__code__` was looked up in that set. No two
  unwrapped public names share a code object, so the three deprecated-parameter
  wrappers cannot credit one another. Reaching a helper only indirectly, through
  another helper that calls it, counts as executed — which is why this figure is
  an upper bound on what is *asserted* about a helper rather than a measure of it.

  **A fresh 2026-09-09 profiling run confirms this row.** Its earlier derivation
  started from the previous run's exact enumeration: of the five callable names
  that left the public surface, `add_tetra` was the one on the never-called list, and
  `add_triangle`, `create_unused_path`, `get_default_particle_system_path` and
  `get_derived_schemas` were all on the executed side, so the split moves from
  105 / 26 of 131 to 101 / 25 of 126. `HALF_PI` was a constant and never entered
  the callable denominator. Deleting `test_derived_schemas` alongside its helper
  changes no other name's status: the only thing it reached beyond
  `get_derived_schemas` was `get_tf_type_compatible`, which several surviving
  helpers still call. The run above now measures that derived enumeration
  directly.

  **The zero-density case moves four more names onto the executed side by the
  same derivation.** It calls `add_collider_box`, `add_collider_cube`,
  `add_rigid_cube` and `add_rigid_sphere` directly, all four of which were on
  the never-called list, so the split moves from 101 / 25 to 105 / 21 with the
  denominator unchanged -- no name left or joined the surface. `add_cube` and
  `add_sphere` are *not* among them: `add_rigid_cube` reaches `add_rigid_box`
  and `add_box`, not the cube spellings.

  **The collision-ownership cases move one more name across, and this row
  carried it wrongly before them.** `set_physics_volume_deformable_body` is
  called directly by two of them, so it belongs on the executed side and the
  split moves from 105 / 21 to 106 / 20, the denominator again unchanged. Its
  first call site landed before this row was last restated rather than with
  those cases, so the row named it as never called while the suite was already
  calling it -- a stale enumeration, not a coverage change. Unlike the two steps
  above, the call sites here were counted directly with the row's own
  word-match rule, so this correction is a measurement.

  **The occupied-plane regression moves one more name across.** It calls
  `add_plane_collider` directly, so the split moves from 106 / 20 to 107 / 19,
  with the denominator unchanged.

  **The transform `Sdf.ChangeBlock` regression moves one more name across.** It
  calls `copy_transform_as_scale_orient_translate` directly, while
  `setup_transform_as_scale_orient_translate` was already on the executed side,
  so the split moves from 107 / 19 to 108 / 18.

  **The non-positive voxel regression moves two more names across.** It calls
  `create_tetra_voxel_sphere` and `create_triangle_mesh_cube` directly, so the
  split moves from 108 / 18 to 110 / 16. `create_tetra_voxel_box` was already
  on the executed side.

  The 16 are `add_pbd_material_viscous`, `add_pbd_material_water`, `add_cube`,
  `add_cube_ground_plane`, `add_density`, `add_force_torque`, `add_joint_fixed`,
  `add_mass`, `add_quad_plane`, `create_joints`, `get_aligned_body_transform`,
  `remove_static_collider`, `set_static_collider`,
  `set_physics_scene_asyncsimrender`,
  `step_and_write_to_ovstage` and `triangulate_mesh`.
- **33 of the 137 public helper names are not so much as named anywhere under
  `tests/python_tests/`** — word-matched against the concatenated text of every
  `.py` file there, so a name matched by a comment counts as mentioned. The
  walk skips `.venv/` and the uv cache: sweeping them in reads a name as
  mentioned because a vendored third-party package happens to contain the same
  identifier, which is how `add_xform` first came back as covered.
  This
  figure covers the eleven non-callable constants that the profiling figure
  cannot see, and it is the backstop for anything the per-AC list below misses.
  The 33 are eight of the eleven constants -- all but `COOKED_DATA_TOKENS`,
  `MESH_APPROXIMATIONS` and
  `METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES` -- plus
  `add_cube`, `add_cube_ground_plane`, `add_density`,
  `add_force_torque`, `add_joint_fixed`, `add_mass`, `add_pbd_material_viscous`,
  `add_pbd_material_water`, `add_sphere`, `add_xform`,
  `configure_particle_set`,
  `create_joints`, `create_tetra_voxels`, `cube_tetrahedra`,
  `ensure_material_on_path`,
  `get_aligned_body_transform`, `get_stage_next_free_path`,
  `remove_static_collider`, `set_collider_subtree`, `set_or_add_orient_op`,
  `set_or_add_scale_op`, `set_or_add_scale_orient_translate`,
  `set_physics_scene_asyncsimrender`,
  `set_static_collider` and `triangulate_mesh`.

  **The pre-applied rigid-body case moves this row from 37 to 36.** It calls
  `set_physics` directly, so the helper leaves the unreferenced set. The
  execution split does not change because an earlier `set_rigid_body` case
  already reached it indirectly.

  **The occupied-plane regression moves this row from 36 to 35.** It names
  `add_plane_collider` in an executable call, so that helper leaves the
  unreferenced set as well as the never-called set above.

  **The transform `Sdf.ChangeBlock` regression moves this row from 35 to 34.**
  Its direct call names `copy_transform_as_scale_orient_translate`, which was
  previously absent from every test source.

  **The non-positive voxel regression moves this row from 34 to 33.** It names
  `create_triangle_mesh_cube`, which was previously absent from every test
  source. `create_tetra_voxel_sphere` was already named.

  **The 45 this row used to carry was wrong by one, and the enumeration beside it
  was one name short of its own figure.** Re-running the match at the tip before
  the drops, with this row's own rule, gave **46** rather than 45, and the missing
  name was `set_physics` -- which the gap list further down already described,
  correctly, as unreferenced by name, so the figure and the prose disagreed with
  each other and the prose was the half that was right. `set_physics` was in
  that corrected set of 38. That correction and the surface cut then move the
  figure in opposite directions: 46 at the old tip, less `HALF_PI`, `add_tetra`, `add_triangle` and
  `create_unused_path`, which left the surface and took their own unnamed rows
  with them, gives 42. The other two names cut, `get_derived_schemas` and
  `get_default_particle_system_path`, were on the *named* side and so only shrink
  the denominator, from 143 to 137.

  **The 42 became 38 with the zero-density case**, which names `add_collider_box`,
  `add_collider_cube`, `add_rigid_cube` and `add_rigid_sphere`. Unlike the
  profiling row above, this one was re-measured rather than derived: the match
  was re-run with this row's own rule and returned the 38 enumerated here.
  `add_cube` and `add_sphere` stay on it, being neither called nor spelled by
  any case.

  The 48 this row carried before that is **superseded, not reconciled.** It was
  matched
  against the pre-rename spellings, so most of its 48 strings no longer exist in
  either the package or the suite, and the two figures cannot be differenced name
  by name. What can be said is which names the cases added since put on the named
  side: `verify_tetra_mesh` and `fixup_tetra_mesh_volumes` from the
  tetrahedral-mesh case, and `COOKED_DATA_TOKENS` and
  `apply_api_schema_property_cache` from the two `remove_collider` cases.

The two disagree in both directions, and neither one alone would be honest: a
helper can execute without being named (`configure_particle_set` and
`cube_tetrahedra` are reached only through callers) and a helper can be named
without executing (`add_quad_plane` appears in a comment). The list below is organized by AC and is intended
to be complete for the named clauses of each AC. Whether to close the rest of
these gaps is the epic owner's decision, taken separately from this MR; the
subset closed here was chosen for the value of the surface behind it.

**Where an AC names a case the suite does not run**

- **AC-5's shape family is sampled, not swept.** The capsule, cylinder and cone
  constructors run in all three flavors, alongside `add_rigid_box`,
  `add_collider_sphere`, `add_box`, `add_ground_plane` and `add_rigid_xform`;
  the zero-density case adds every `add_rigid_*` and every `add_collider_*`
  spelling of the six shapes, `add_collider_box`, `add_collider_cube`,
  `add_rigid_cube` and `add_rigid_sphere` among them.
  Unreferenced: `add_cube` and `add_sphere` -- the plain spellings, which no
  case reaches even through the rigid and collider ones above, `add_cube` being
  one of the `add_box` cube spellings the REQ Description keeps under "The
  excluded surface", where being a convenience alias is stated not to be
  grounds for exclusion -- `add_xform`, `add_cube_ground_plane`,
  and `add_quad_plane`, which is named only in a test comment.
- **AC-6 names five shapes and the suite asserts point and normal values for
  two.** The AC requires correct points, normals, face indices and face vertex
  counts for a square, cube, concave shape, cylinder and cone. Read against the
  suite, it holds for the square and the concave shape: those two have their
  point values, their per-point normals and their winding asserted. For the cube,
  cylinder and cone only the *shape* of the result is asserted -- non-empty
  points, non-empty face vertex counts, and index counts that sum consistently --
  so a generator that produced a geometrically wrong but structurally consistent
  mesh would pass. Nothing here narrows AC-6, which states what the helpers must
  produce; the mismatch is the suite's, and this is the plain statement of which
  two of the five it stands behind.
- **AC-6's tetrahedral generators are largely untested.** The square and concave
  mesh paths now run, through the generic `create_mesh`, and
  `create_triangle_mesh_square`'s point and index lists are authored into a real
  `UsdGeom.Mesh` by the surface-deformable case. Unreferenced by name:
  `triangulate_mesh`, `create_tetra_voxels` and `cube_tetrahedra`; of those,
  `create_tetra_voxels` and `cube_tetrahedra` run indirectly through
  `create_tetra_voxel_box`.
  `create_tetra_voxel_sphere` and `create_triangle_mesh_cube` now run only for
  empty non-positive inputs; their positive-resolution geometry remains
  unasserted.
  `add_triangle` and `add_tetra` used to head this row and no longer belong to
  it: `add_tetra` is dropped, and `add_triangle` is now the private
  `mesh._add_triangle`, still unnamed by the suite and still reached through
  `convert_tetra_to_triangle_soup`, but no longer part of the public surface this
  figure is taken over.
- **AC-8's volume-deformable entry point is reached only for its teardown.**
  `set_physics_volume_deformable_body` runs in two collision-ownership cases,
  which assert the APIs it applies and then what a removal takes back. Nothing
  asserts the rest-shape attributes it authors, its `UsdPhysics.RigidBodyAPI`
  and non-`TetMesh` rejections, or its surface-face computation. The row above
  this list carried it as unreferenced until those cases were counted; it is the
  assertions that are partial, not the call.
- **AC-4's remaining transform helpers are untested.** `get_basis` and
  `get_forward_vector` now run; `get_aligned_body_transform`,
  `set_or_add_orient_op`, `set_or_add_scale_op` and
  `set_or_add_scale_orient_translate` are unreferenced. AC-4's
  `resetXformStack` and op-precision preservation clauses remain unasserted: the
  transform-stack case calls both helpers, but its source has no reset flag and
  only reads the resulting stack and local transform. The `Sdf.ChangeBlock`
  regression closes only that structural part of this row.
- **The static-collider and remaining bare set/remove helpers lack direct coverage.**
  `remove_static_collider`, `set_collider_subtree` and `set_static_collider` are
  unreferenced by name. `set_collider_subtree` and `remove_physics` are each
  exercised through a caller -- `set_rigid_body` and
  `remove_rigid_body_subtree` -- so what is missing is a case that pins their
  own contract, not first entry into the code. `remove_collider_subtree`,
  `remove_rigid_body_subtree` and `set_physics` are now covered directly.

  `remove_physics` used to head that first list and does not belong there:
  `test_utils_surface.py`'s alias table carries
  `"removePhysics": ("authoring", "remove_physics")`, which is exactly the kind of
  incidental mention this row's word-match rule counts as named. It has been
  struck here, and it was never in the unnamed enumeration above, so only the
  prose was wrong. `set_physics` was the converse and moved the other way, into
  both lists; the pre-applied rigid-body case now removes it from each.

  `remove_multiple_api_schema_properties` used to be listed here as unreferenced
  by name, and that was wrong: `test_utils_schema.py` calls it by name in the
  multiple-apply property-cache case. The substance of the gap survived the
  correction, because the call established nothing -- the assertion after it
  compared the restored key set, which an unremoved property satisfies just as
  well as a removed one, since either way it is still authored. That is now
  closed: the case asserts the attribute has no authored value between the
  removal and the restore, which is what its single-apply sibling
  `remove_api_schema_properties` already did. What remains open is the rest of
  its contract -- an instance name that names no applied instance, and a prefix
  that matches nothing.
- **Two `False` returns are unreachable from this suite, and were previously
  undeclared.** `materials.ensure_material_on_path` answers `False` when the path
  is already occupied by a prim that is not a `UsdShade.Material`; no case
  occupies a material path that way, so the branch never runs. That is also the
  only route to `add_rigid_body_material`'s own `False` return, which is what the
  `is True` assertion on its success path is now written against -- the assertion
  is sharp, the failure path is still unexercised.
  `particles.add_pbd_particle_material` and `deformable.add_deformable_material`
  reach the same helper and inherit the same gap.
- **`paths.get_stage_next_free_path`'s error handling is unexercised, which is
  not the same as the helper being untested.** The helper runs constantly: every
  `shapes.py` and `planes.py` constructor and `joints.create_joint` call it, so
  its uniquifying loop and its default-prim prefixing are covered by every shape
  case in the suite. What no case reaches is its `ValueError` for a string that
  is not a valid path, and its relative-path auto-correction -- the
  `MakeAbsolutePath` rewrite and the `logger.warning` that announces it. Listing
  it among the unreferenced names above was therefore misleading in both
  directions: it is reached, and the part that is missing is the part the listing
  did not name.
- **A removal takes a property only where the property is local, and no case
  covers the rest.** `Usd.Prim.RemoveProperty` deletes the spec in the current
  edit target, so a property whose opinion arrives over a reference, an inherit
  or a specialize arc, or which sits in a layer stronger than the edit target,
  keeps its resolved value after the API above it is gone. The API removal
  itself does compose: `RemoveAPI` writes a `delete apiSchemas` opinion, so the
  schema goes even across a reference. The result is a prim with no deformable
  API and a resolved deformable property, which is the same shape as the orphan
  the three property cases above assert is cleaned -- and the removal cannot
  clean this one. This is not specific to `remove_deformable_body`: it holds for
  `remove_api_schema_properties` and so for every remove helper in the module.
  The suite authors every fixture into a single in-memory root layer, so no case
  reaches it, and none is planned here -- editing a non-local opinion needs a
  policy this requirement does not state.
- **`deformable.py`'s `False` returns are covered at seven of 52.**
  The module has 52 distinct `return False` statements. A failure can leave
  authored state behind; REQ-PYTHON-UTILS-001, "A deformable failure can leave
  authored state behind", records why that is kept.
  The previous full-run line trace reached three:
  `create_auto_volume_deformable_hierarchy`'s Gprim-root rejection, the same
  helper's hexahedral-apply failure (reached only by monkeypatching the
  `try_apply_api` seam, as the AC-12 row below records), and
  `set_physics_surface_deformable_body`'s rejection of a mesh with non-triangular
  faces. The four new return statements have direct regression cases: one in
  each auto-hierarchy helper for a visual prim with no default-time points, and
  one in each single-prim helper for missing required geometry. The derived
  coverage is therefore seven, and the other 45 are unexercised. This is the
  largest single gap in the suite and it was previously unsized, which is worse than
  being large: the earlier gap list named AC-8's untested entry points without
  saying that the failure half of the whole module is essentially unvisited.

  Of the 52, 21 guard an authoring operation, and they split by what USD does
  when it refuses one. Fourteen go through `codeless.try_apply_api`, which
  reports a refused application as `False`, so a layer that is not editable
  reaches those fourteen. The other seven wrap a typed `Apply` or check a
  `Define` result, and USD raises `Tf.ErrorException` out of both on such a
  layer rather than answering `False`, so only a monkeypatched seam reaches
  them. None of the 21 is reachable through an unregistered schema or an unknown
  identifier: `codeless` raises `CodelessSchemaError` for both. Four
  propagate a failed helper call without a warning of their own: the
  mesh-simplification call in each `create_auto_*` helper,
  `add_deformable_material`'s `ensure_material_on_path` and
  `add_surface_deformable_material`'s `add_deformable_material`. The remaining
  27 are input or precondition
  checks -- a bad path, a wrong prim type, a conflicting or missing API,
  unsuitable topology -- and an input reaches those and the four propagated ones.

  **This gap stays open by decision, and the breakdown above is not a work
  list.** The four new input-precondition returns are covered because they
  repair two reported missing-geometry defects; they do not turn this into a
  sweep of the other input and precondition returns. The owner has settled that
  any broader work is its own change. Read the enumeration as a measure of what
  the suite does not reach, not as unfinished work in this one.
- **A rebuild given a new `collision_tetmesh_path` leaves the collider on the
  old one**, so the subtree it returns holds two enabled colliders and the
  runtime binds whichever its traversal reaches first. Nothing a rebuild can
  read attributes the old collider: `PhysxAutoDeformableBodyAPI` declares only
  `physxDeformableBody:autoDeformableBodyEnabled` and
  `physxDeformableBody:cookingSourceMesh`, that relationship names the cooking
  source rather than either generated mesh, and the abandoned mesh carries no
  marker unique to a generated one -- `UsdPhysics.CollisionAPI`, which a caller
  can author, an `OmniPhysicsDeformablePoseAPI:default` instance and its
  `purposes` property, which the bind-pose pass gives every point-based prim
  under the root, and a type a caller's own tet mesh has too. AC-8 records the limitation
  and `test_rebuilding_a_moved_collision_path_leaves_its_collider` pins it, so
  a later fix has to change that case rather than add one. Cleaning the collider
  needs ownership these helpers do not persist, which is a change to what they
  author and not a change to the suite.

**Where a branch cannot be reached from this suite**

- **AC-10's unregistered API-schema branch is reached only from a child process,
  and only for the late-registration shape of it.** The session-scoped fixture
  registers the codeless schemas before any stage opens, and USD's registry is
  process-global and one-way, so `schema_is_registered()` returning `False`, and
  the `CodelessSchemaError` path that depends on it, cannot be exercised in the
  pytest process at all. AC-14's cases supply the missing process and cover the
  branch where the schemas exist but were registered too late, for `apply_api`
  and `remove_api` alike. The case the second sentinel exists for, one schema
  root registering while the other does not, is covered by a third child over a
  half-staged tree built for it.

  The concrete-prim preflight is broader: another child covers both no
  registration and a default `register_schemas()` call made after USD initialized,
  for the particle system and both codeless joint types. Still uncovered for the
  API-schema entry points: the same branch with no schema tree present at all,
  and `get_attr` and `set_rel`'s appended hint, which is asserted only through
  `apply_api` and `remove_api`.
- **One `codeless` raise is still unexercised: `set_rel`'s, for a relationship
  the prim does not carry.** The row above understated it, declaring only the
  *appended hint* as uncovered, which reads as though the raise itself is
  covered and the hint is a detail on top of it. Neither half is.

  The two write-refusal raises this row used to declare uncovered are now
  covered, and by a real refusal rather than an injected one: the cases above
  make the root layer non-editable, which reaches `apply_api`'s raise and the
  `Tf.ErrorException` handlers in `set_attr` and `set_rel`. What still needs a
  patched USD method is the *other* half of USD's refusal contract, the `False`
  return, since a non-editable layer makes both writes raise.
- **AC-12's hexahedral branch is reached only through an injected failure.**
  Routing `deformable.py`'s applies through `codeless.try_apply_api` opened the
  seam this gap used to record as absent, so the branch is now covered, but only
  by monkeypatching that seam: no input to
  `create_auto_volume_deformable_hierarchy` reaches it, for the reason given
  under "When" above. The Gprim-root rejection remains the input-driven case for
  the same "report failure rather than half-build" contract.

**Where an assertion is shallower than the row it supports**

- **AC-2's current flat names and owners are pinned independently; their
  ancestry is not.** The dependency-free manifest closes the former gap: it is
  compared exactly with every loaded owner's `__all__` and with the reverse
  owner mapping, while the separate size assertion pins 138 helpers. What
  remains unpinned is the correspondence between a renamed helper and the
  `omni.physx.scripts` name it came from -- a rename to a *different*
  conventional spelling than the one chosen would pass everything here. Only
  review catches that, and for the eight aliased names the alias table is a
  second, independent statement of it.
- **The subpackage's inline annotations are not type-checked by CI at all.**
  `ovphysx/python/pyrightconfig.json` sets `"include": ["ovphysx/**/*.pyi"]`, so
  the pinned pyright that `scripts/test_pyright.cmake` runs -- reached from
  `ci_validate.cmake` and `validate_all.cmake` -- reads no `.py` file in the
  tree. Every authoring submodule here is inline-annotated `.py`, and the REQ
  Description's "the type-checked surface of `ovphysx.utils` is limited by the
  same aarch64 constraint" records why they cannot be stubbed instead: a stub
  importing `pxr` would fail on an unresolved import in a dev group that cannot
  gain `usd-core`. So no gate in this repository reads these annotations.

  What that costs is specific, and it is not "the annotations might be wrong in
  a way nobody notices". The wheel ships `py.typed`, so a consumer's own
  checker *does* read them and does trust them -- meaning an annotation that
  contradicts its function is a wrong answer delivered to the consumer's
  checker, not a missing one. A helper annotated `-> typing.List[Gf.Vec3f]`
  that returns a `(positions, velocities)` tuple makes the consumer's checker
  reject correct unpacking code and accept incorrect list code, and nothing on
  this side fails. An annotation restricted to `UsdGeom.Xformable` similarly
  rejects a valid bare-`Usd.Prim` call. Six return-annotation contradictions,
  two accepted-input contradictions and one implicit Optional were present on
  that footing and are corrected in this MR.

  A second limit compounds the first and survives any change of scope: even
  pointed at these `.py` files, a checker in this environment cannot resolve
  `pxr`, so every `pxr` type reads as Unknown and everything is assignable to
  it. Four of the six return contradictions were therefore invisible to
  pyright and were found by a syntactic pass instead -- `-> UsdGeom.XformOp` beside
  `return False` passes, while `-> typing.List[UsdGeom.XformOp]` beside
  `return False` fails, purely because `List[Unknown]` is a known generic and
  bare `UsdGeom.XformOp` is not. Widening the pyright scope would thus catch
  less than it appears to, which is part of why it is not this MR's remedy;
  the decision to leave the scope alone is the epic owner's, taken separately.
  Trunk additionally carries type errors of its own in `_bindings.py` and
  `config.py` that a widened scope would surface.

  Two complaints in this subpackage are left standing, both deliberately. The
  four in `paths.py` are artefacts of that same unresolved `pxr`:
  `get_stage_next_free_path` reassigns its `typing.Union[str, Sdf.Path]`
  parameter to `Sdf.Path(path)` and then uses `Sdf.Path` methods on it, which
  is correct at runtime, but with `Sdf.Path` Unknown the reassignment cannot
  narrow and the union's `str` arm is what gets checked. The fifth is real but
  is not an annotation defect here: `step_and_write_to_ovstage` reads
  `physx._attached_ovstage`, which `api.py` sets in `__init__` and `api.pyi`
  does not declare, so a checker resolving `PhysX` from the stub cannot see it.
  Closing that means declaring a private attribute in a public stub that *is*
  in CI's pyright scope -- a surface decision outside this subpackage, and so
  outside this requirement.
- **AC-5's display color and xform op stack are asserted for one shape family
  only**, the capsule / cylinder / cone case, and not for the box, sphere, plane
  or xform constructors. AC-5 also does not cover the force and torque helpers
  (`add_force_torque`, `add_mass`, `add_density`), `create_joints`,
  `add_joint_fixed` or
  `set_physics_scene_asyncsimrender`; `ensure_material_on_path` and
  `get_stage_next_free_path` have rows of their own above, since for both of them
  "not covered" would be the wrong summary. `create_joint`
  is exercised for one of its joint types, `Revolute`. The local-space-velocities
  metadata pair is covered, which is the whole of AC-5's custom-metadata clause
  now that the four generic accessors are dropped.
- AC-5's material binding **is** read back now, so this row is closed rather
  than restated: the case reads the direct binding under the `physics` purpose
  and asserts it names the material authored. What it does not cover is the
  helper's behavior over a prim that already carries a different physics binding.
- **AC-7 asserts particle set type and system membership, but not the
  self-collision, fluid, group, mass and density arguments individually**, and
  neither `configure_particle_set`, `add_pbd_material_water` nor
  `add_pbd_material_viscous` is referenced by name; the first does run, through
  `add_physx_particleset_points`, while the two presets are never called.
  `particles._get_default_particle_system_path` is called by name, but only to
  obtain the path the occupied-default-path case then occupies: nothing asserts
  its own contract either. It is no longer a gap in the *public* surface, since
  it is now private, but the case still depends on it and would still break if it
  changed. The PBD material
  case asserts the API is applied, not the friction and density values passed.
- **The six constants the codeless conversion introduced are unreferenced**:
  `TOKEN_TRIANGLE_MESH`, `TOKEN_SDF`, `TOKEN_SPHERE_FILL` and the three
  `SCENE_UPDATE_TYPE_*` values. The two collision tokens are exercised by their
  literal string spelling, and `MESH_APPROXIMATIONS` is asserted to still map
  those spellings to the right APIs, but nothing reads the constants. Likewise
  `COOKED_DATA_TOKENS`, `AXES_INDICES` and `MAX_FLOAT`. `HALF_PI` was on this
  list and is now in "The dropped surface": being unnamed by the suite was never
  the ground for dropping it.
- **AC-13's `step_and_write_to_ovstage` has no behavioral coverage here at all**,
  by design: only its resolvability and the subpackage's import discipline are
  checked. Its behavior belongs to TEST-PYTHON-FRAME-001, which covers it through
  `test_step_and_write_to_ovstage.py` in the native Python test stage.
