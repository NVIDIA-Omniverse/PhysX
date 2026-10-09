<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OVSTAGE-SCHEMA-001
title: Application-Owned Registration Of The Codeless PhysX Schemas
status: implemented
owner: ovphysx
---

## Description

ovphysx ships its PhysX USD schema definitions (`PhysxSchema`,
`OmniUsdPhysicsDeformableSchema`) as codeless USD plugins and nothing else about
USD: no OpenUSD runtime, no schema registration, no environment mutation. The
application owns the USD runtime(s) in its process -- the namespaced runtime
inside OVStage, a stock OpenUSD it authors with, or both -- and registers
ovphysx's schemas with them explicitly. ovphysx only tells the application
where the schemas are.

The schema payload is one tree, `<root>/schemas/physx` (`<sdk>/schemas/physx`
for the SDK, `ovphysx/schemas/physx` in the wheel): a root `plugInfo.json`
whose `Includes` reaches every module, plus one
`<Module>/resources/{plugInfo.json,generatedSchema.usda}` per module, all
`Type: resource`. Registering the root alone registers every module.

Without registration OVStage population resolves only the authored properties
of physics prims; schema-declared fallbacks never appear and deformable and
particle prims are populated without the attributes the runtime needs. USD
assembles its schema registry once, on first read, so the application must
register before the first population call in the process; OVStage reports a
late registration with `OVSTAGE_ERROR_OP_FAILED`. That ordering obligation
belongs to the application and is stated in the API documentation. ovphysx does
not work around a missed registration, but it does refuse to attach a stage
populated without one: population drops every Physx* API it cannot resolve, so
such a stage simulates with schema defaults the asset never authored (NVBugs
6745103: an articulation authored with self-collision off and capped joint
velocities lost both and diverged inside PhysX).

The parser also reads a subset of the Newton USD schema (`newton-usd-schemas`,
a separate package ovphysx does not ship) as fallbacks for the PhysX spellings,
`newton:velocityLimit` for `physxJoint:maxJointVelocity` among them. OVStage
populates an attribute only when a registered schema defines it, so a scene
that authors `newton:*` attributes loses them silently unless the application
registers that schema too (NVBugs 6742287: a joint authoring only
`newton:velocityLimit` ran unlimited once the pinned OVStage stopped bundling
the schema). OVStage keeps no queryable record of registered schema families;
the only observable is the status of a registration that arrives after
population, and that status is per registered path, so ovphysx can check the
registration only of a Newton schema it can locate itself: the installed
`newton-usd-schemas` Python package. The Python wrapper performs that check and
warns; the C API, which cannot locate the package, documents the obligation.

Provenance: this replaces the implicit registration ovphysx used to perform
(an `OV_PXR_PLUGINPATH_2511` environment append at library start, on `import
ovphysx`, and from a public `ovphysx_register_schema_paths()`, plus an
`ovstage_population_register_usd_schemas()` call inside
`ovphysx_create_instance()`), which the ovphysx owner rejected because the user,
not ovphysx, owns USD.

## Acceptance Criteria

- AC-1: **Discovery only.** `ovphysx_get_codeless_schema_root()` fills an
  `ovphysx_string_t` with the NUL-terminated directory `<root>/schemas/physx`
  derived from the ovphysx library location (SDK, wheel, or copied-runtime
  layout, honoring `OVPHYSX_LIB`), and `ovphysx.codeless_schema_root()` /
  `ovphysx.codeless_schema_paths()` return the same tree from Python. The
  returned root holds `plugInfo.json` and the `PhysxSchema` and
  `OmniUsdPhysicsDeformableSchema` modules. Neither call initializes ovphysx,
  acquires Carbonite, loads or imports USD, or triggers the Python native
  bootstrap.
- AC-2: **Fail-closed discovery.** `ovphysx_get_codeless_schema_root(NULL)`
  returns `OVPHYSX_API_INVALID_ARGUMENT`. When no `schemas/physx/plugInfo.json`
  exists next to the library it returns `OVPHYSX_API_ERROR`, sets the output to
  an empty string, and `ovphysx_get_last_error()` names the missing file and
  `OVPHYSX_LIB`. Nothing is memoized: a later call against a complete layout
  succeeds.
- AC-3: **No implicit registration or environment mutation.** Loading
  `libovphysx`, `import ovphysx`, `ovphysx_get_codeless_schema_root()`, and
  `ovphysx_create_instance()` neither call
  `ovstage_population_register_usd_schemas()` nor read, set, or append
  `OV_PXR_PLUGINPATH_2511` or `PXR_PLUGINPATH_NAME`; `ovphysx_attach_ovstage()`
  touches neither environment variable. The former public
  `ovphysx_register_schema_paths()` and `ovphysx.register_schema_paths()` do
  not exist.
- AC-4: **Application-side registration works end to end.** Registering the
  root from AC-1 through `ovstage_population_register_usd_schemas()` (C) or
  `ovstage.population.register_usd_schemas()` (Python) before the first
  population makes populated deformable body and deformable material prims
  resolve their schema-declared attributes in the tensor-binding reads. The bundled C and Python samples, the C++ and Python test helpers, and
  the benchmarks perform this registration themselves.
- AC-5: **Attach refuses an unregistered population.** `ovphysx_attach_ovstage()`
  (and `PhysX.attach_ovstage()`) verifies the registration by passing the AC-1
  root to `ovstage_population_register_usd_schemas()`, resolved from the loaded
  ovstage at run time. `OVSTAGE_OK` (the family was registered in time, so the
  call is a no-op) lets the attach proceed unchanged. `OVSTAGE_ERROR_OP_FAILED`
  (population already ran without it) fails the attach with `OVPHYSX_API_ERROR`
  before any runtime attach happens; `ovphysx_get_last_error()` names the
  missing `register_usd_schemas` call and appends ovstage's own detail, and the
  Python wrapper raises `RuntimeError` with that text. A missing schema tree
  (AC-2) fails the attach the same way. The probe runs before the instance's
  attach payload is touched, so a refused attach leaves the instance detached.
  A late result is remembered for the process: the late call itself registers
  the plugins, so a repeated probe would report `OVSTAGE_OK` while the registry
  USD built without them stays as it is, and every later attach in the process
  fails with the same error. ovstage keys the registration on the plugin
  family, not the path: schemas discovered through `OV_PXR_PLUGINPATH_2511` or
  registered from another copy of the tree make the probe report `OVSTAGE_OK`.
  A registration ovstage cannot observe (another USD consumer read the schema
  definitions first) is not detected; the Carbonite setting
  `/ovphysx/schemas/requireRegistration` (create-time config entry, default
  `true`) set to `false` turns the refusal into a `CARB_LOG_WARN` with the same
  text and lets the attach proceed.
- AC-6: **Newton schema discovery and attach-time warning (Python).**
  `ovphysx.newton_schema_root()` returns the directory of the installed
  `newton-usd-schemas` package (the one holding its `plugInfo.json`), located
  through `importlib.util.find_spec` without importing the package (its import
  requires `pxr` and registers the schema with that stock USD runtime), and
  raises `FileNotFoundError` naming `pip install newton-usd-schemas`, the
  GitHub repository and `register_usd_schemas` when it is not installed;
  `ovphysx.schemas.find_newton_schema_root()` returns `None` instead. Before
  its native attach, `PhysX.attach_ovstage()` checks, once per process, that
  this package was registered before the first population by passing its path
  to `ovstage.population.register_usd_schemas()`: `OVSTAGE_OK` is silent; an
  error (population already ran without it) emits a `RuntimeWarning` naming
  the dropped `newton:*` attributes, the two-root `register_usd_schemas` call
  with `newton_schema_root()`, and the silencing setting. When the package is
  not installed, the default state of a consumer whose scenes author no
  `newton:*` attribute, the same check logs a `WARNING` on the `ovphysx`
  Python logger carrying the install hint and raises no Python warning, so
  suites that promote warnings to errors are unaffected. Because the check
  precedes the native attach, a warning promoted to an
  error propagates from an instance that holds no reference to the stage, and
  the next attach proceeds (the check has latched). A second attach in the
  process repeats neither warning. The attach itself is otherwise unaffected:
  the scene simulates with the `newton:*` attributes population dropped.
  OVStage keys registration on the plugin family, so a complete Newton schema
  registered from any other directory before population is recognized and
  produces no warning. The probe is a real registration while USD's registry
  is still open, so it runs only once USD has built its schema definitions,
  which the wrapper learns by registering a uniquely named throwaway
  descriptor that declares no schema types (OVStage reports it late exactly
  then, and a typeless plugin selects no schema for anyone); a procedurally
  authored stage attached before any USD population in the process is left
  alone, the check stays armed, and the application's own later registration
  of the family is the one USD uses. A registration OVStage cannot observe (another USD consumer
  read the schema definitions first) is not detected; the Carbonite setting
  `/ovphysx/schemas/warnMissingNewtonSchema` (create-time config entry,
  `PhysXConfig(carbonite_overrides=...)`, default `true`) set to `false`, or to
  the string forms the native parser accepts (`"false"`, `"0"`), disables the
  check for that instance. The C API performs no such check. The bundled
  Python samples, test helpers, benchmarks and the no-GPU smoke environment
  depend on `newton-usd-schemas` (lower-bounded, not pinned) and register it
  together with the codeless root.

## Test References

- [TEST-CAPI-OVSTAGE-SCHEMA-001](../../tests/capi/TEST-CAPI-OVSTAGE-SCHEMA-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_get_codeless_schema_root` -- AC-1, AC-2, AC-3)
- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_get_codeless_schema_root`, `createInstanceInternal` -- AC-1, AC-2, AC-3; `verify_physx_schemas_registered`, `ovphysx_attach_ovstage` -- AC-5)
- ovphysx/tests/python_tests/cpu_tests/test_attach_requires_physx_schemas.py (AC-5)
- ovphysx/python/ovphysx/api.py (`_warn_if_newton_schema_unregistered`, `PhysX.attach_ovstage` -- AC-6)
- ovphysx/tests/python_tests/cpu_tests/test_newton_schema_registration.py (AC-6)
- ovphysx/src/include/UsdSchemaPaths/UsdSchemaPaths.h and ovphysx/src/UsdSchemaPaths/UsdSchemaPaths.cpp (`getCodelessSchemaRoot` -- AC-1, AC-2)
- ovphysx/python/ovphysx/schemas.py (`codeless_schema_root`, `codeless_schema_paths` -- AC-1; `newton_schema_root`, `find_newton_schema_root` -- AC-6)
- ovphysx/python/ovphysx/__init__.py (import has no registration side effect -- AC-3)
- ovphysx/scripts/export_codeless_schema.py and ovphysx/scripts/package_deps.py (stage the tree AC-1 describes)
- ovphysx/tests/c_unittests/test_schema_paths.cpp (AC-1, AC-2, AC-3)
- ovphysx/tests/python_tests/lifecycle_tests/test_import_has_no_usd_side_effects.py (AC-1, AC-3)
- ovphysx/tests/c_unittests/test_utilities.h (`register_physx_schemas_with_ovstage`), ovphysx/tests/python_tests/test_utils.py (`register_physx_schemas_with_ovstage`), ovphysx/tests/c_samples/common/ovstage_sample.h (`ovphysx_sample_register_physx_schemas`), ovphysx/tests/python_samples/*.py (`attach_scene`) -- AC-4, and the Python ones register the Newton schema too -- AC-6

## Dependencies

- [REQ-PACKAGING-NOUSD-001](../packaging/REQ-PACKAGING-NOUSD-001.md) -- the payload this requirement's tree is part of
- ADR-0018 (`ovphysx/ovruntime/plc/adr/ADR-0018-usd-free-runtime-build.md`) -- the decision that ovphysx ships no USD and the application owns registration
