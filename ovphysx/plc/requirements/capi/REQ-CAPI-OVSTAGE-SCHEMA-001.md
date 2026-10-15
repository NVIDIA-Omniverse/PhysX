<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OVSTAGE-SCHEMA-001
title: Application-Owned Delivery And Registration Of The Codeless PhysX Schemas
status: implemented
owner: ovphysx
---

## Description

Applications provision PhysX schema resources independently from ovphysx using
`physx-usd-schemas`. The same platform-neutral wheel supports Python
installation and native data-only extraction, retaining `.dist-info/METADATA`.
OVPhysX ships neither the resources nor schema-location APIs and consumes no
generated PhysX schema headers or Packman schema package at build time.

Application recipes locate `PhysxSchema/resources` and
`OmniUsdPhysicsDeformableSchema/resources` under the external provider package,
check both `plugInfo.json` and `generatedSchema.usda`, and pass
both directories to the application's USD runtime. Metadata discovery does not
import the provider, whose normal import registers with stock USD and needs pxr.
Native consumers need no Python runtime after extracting the wheel. There is no
path relationship to the ovphysx library, SDK, or `OVPHYSX_LIB`.

Without registration OVStage population can omit both authored PhysX properties
and schema-declared fallbacks; deformable and particle prims can lack the
attributes the runtime needs. USD
assembles its schema registry once, on first read, so the application must
register before the first USD schema read in the process, including population
or export; OVStage reports a late registration with `OVSTAGE_ERROR_OP_FAILED`.
That ordering obligation
belongs to the application. Once OVStage has observed USD schema definitions
being built, native attach checks both PhysX plugin families and refuses if
OVStage reports a missing family, within the detection limits in AC-8. Verification neither
locates an external provider nor supplies schema definitions for the application.
An attach while USD schema definitions remain unbuilt leaves verification armed
for a later attach. Both population and export build those definitions.

The parser also reads a subset of the Newton USD schema (`newton-usd-schemas`,
a separate package ovphysx does not ship), the joint-level `newton:velocityLimit`
among them. OVStage
populates an attribute only when a registered schema defines it, so a scene
that authors `newton:*` attributes loses them silently unless the application
registers that schema too (NVBugs 6742287: a joint authoring only
`newton:velocityLimit` ran unlimited once the pinned OVStage stopped bundling
the schema). OVStage keeps no queryable record of registered schema families;
the only observable is the status of a registration that arrives after
the first USD schema read. That status reflects plugin family identity,
independent of the registered path. The Newton warning probes the installed
`newton-usd-schemas` Python package because it can locate that package's
descriptor; registration of the same family from another complete copy is
recognized. The Python wrapper performs that check and
warns; the C API, which cannot locate the package, documents the obligation.

Provenance: this replaces the implicit registration ovphysx used to perform
(an `OV_PXR_PLUGINPATH_2511` environment append at library start, on `import
ovphysx`, and from a public `ovphysx_register_schema_paths()`, plus an
`ovstage_population_register_usd_schemas()` call inside
`ovphysx_create_instance()`), which the ovphysx owner rejected because the user,
not ovphysx, owns USD.

## Acceptance Criteria

- AC-1: **External delivery and relocation.** Neither the SDK nor ovphysx wheel
  contains PhysX schema resources. Python applications discover the separately
  installed provider via `importlib.metadata.distribution`; native applications
  provide its extracted-wheel root to the sample helper (the examples use
  `PHYSX_USD_SCHEMAS_ROOT`). Both find the two module resource directories after
  a fresh install or relocation; native applications resolve the new root after
  moving it. Discovery loads no USD, provider module, or ovphysx native code.
- AC-2: **Dependency compatibility and resource validation.** Python manifests
  declare supported-version constraints and `uv.lock` fixes resolved artifacts;
  unsatisfied constraints fail during dependency resolution. The native sample
  installer reads its artifact URL/digest from `common/physx_schemas.lock.json`
  and verifies the SHA-256 before extraction. Discovery imposes no exact release
  number: both recipes require both modules' `plugInfo.json` and
  `generatedSchema.usda`. Missing providers and incompatible/incomplete resource
  layouts fail before population with installation/root repair guidance. The
  installer also rejects an incorrect package identity or missing metadata.
  A later valid discovery succeeds. Layout validation does not establish
  semantic compatibility of arbitrary schema versions.
- AC-3: **No implicit registration or schema-location API.** Importing ovphysx
  and loading/creating its native runtime do not register PhysX schemas or
  mutate USD plugin-path variables. `ovphysx_get_codeless_schema_root`,
  `ovphysx.codeless_schema_root`, and `ovphysx.codeless_schema_paths` are removed.
  The former registration APIs also remain absent. The Newton-only probe in
  AC-5 is unchanged.
- AC-4: **Application registration before USD schema reads.** C/C++ and Python
  samples, tests, and benchmarks register both external resource directories
  through OVStage before their first USD schema read, including population or
  export. Populated PhysX/deformable schemas resolve their declared attributes.
  Stock-USD authoring registers the
  same resources before schema-registry access. Late registration retains the
  USD/OVStage error behavior; ovphysx never retries population with missing data.
- AC-5: **Newton schema discovery and attach-time warning (Python).**
  `ovphysx.newton_schema_root()` returns the directory of the installed
  `newton-usd-schemas` package (the one holding its `plugInfo.json`), located
  through `importlib.util.find_spec` without importing the package (its import
  requires `pxr` and registers the schema with that stock USD runtime), and
  raises `FileNotFoundError` naming `pip install newton-usd-schemas`, the
  GitHub repository and `register_usd_schemas` when it is not installed;
  `ovphysx.schemas.find_newton_schema_root()` returns `None` instead. Before
  its native attach, `PhysX.attach_ovstage()` checks this package's registration
  once per process after USD schema definitions are built by passing its path
  to `ovstage.population.register_usd_schemas()`: `OVSTAGE_OK` is silent; an
  error (USD already read schema definitions without it, through population or
  export) emits a `RuntimeWarning` naming the `newton:*` attributes population can
  drop, registration together with the external PhysX schemas, the schema
  documentation and `newton_schema_root()`, and the
  silencing setting. When the package is
  not installed, the default state of a consumer whose scenes author no
  `newton:*` attribute, the same check logs a `WARNING` on the `ovphysx`
  Python logger carrying the install hint and raises no Python warning, so
  suites that promote warnings to errors are unaffected. Because the check
  precedes the native attach, a warning promoted to an
  error propagates from an instance that holds no reference to the stage, and
  the next attach proceeds (the check has latched). A second attach in the
  process repeats neither warning. The attach itself is otherwise unaffected:
  any `newton:*` attributes already dropped by population remain missing.
  OVStage keys registration on the plugin family, so a complete Newton schema
  registered from any other directory before the first USD schema read is
  recognized and produces no warning. The probe is a real registration while USD's registry
  is still open, so it runs only once USD has built its schema definitions,
  which the wrapper learns by registering a uniquely named throwaway
  descriptor that declares no schema types (OVStage reports it late exactly
  then, and a typeless plugin selects no schema for anyone); a procedurally
  authored stage attached while USD schema definitions remain unbuilt is left
  alone, the check stays armed, and the application's own later registration
  of the family is the one USD uses. A registration OVStage cannot observe (another USD consumer
  read the schema definitions first) is not detected; the Carbonite setting
  `/ovphysx/schemas/warnMissingNewtonSchema` (create-time config entry,
  `PhysXConfig(carbonite_overrides=...)`, default `true`) set to `false`, or to
  the string forms the native parser accepts (`"false"`, `"0"`), disables the
  check for that instance. The C API performs no such check. The bundled
  Python samples, test helpers, benchmarks and the no-GPU smoke environment
  depend on `newton-usd-schemas` (lower-bounded, not pinned) and register it
  together with both external PhysX module resource directories.

- AC-6: **Local schema development.** `--devschema` builds/provisions the local
  provider wheel outside the ovphysx product. Normal and devschema builds fetch
  no `usd_ext_physics` package and consume no generated PhysX schema headers; the flag
  changes only the provider wheel, without requiring a clean native rebuild. Native and Python test/sample/benchmark runners and focused pytest
  use that selected provider. Missing local data fails instead of falling back
  to the published package. Same-version edits select fresh provider contents;
  the published digest remains the default, while an explicit local wheel uses
  its supplied digest and package-identity/completeness validation, accepting
  a different local version. A normal
  build clears the selection. `cmake -DDEV_SCHEMA=ON -P scripts/validate_all.cmake` forwards the
  development flavor to the build.
- AC-7: **Copy-paste sample import.** Shipped Python samples under
  `tests/python_samples/` that call `physx_schema_paths` import the recipe from
  the installed wheel namespace
  `ovphysx.samples.python_samples.schema_support`, so a reader who copies a
  sample (or a docs `{literalinclude}` of it) into their own folder does not
  need a sibling `schema_support/` directory. The recipe directory remains
  shippable for readers who adapt it outside the wheel (NVBugs 6904272).

- AC-8: **Native attach guards missing PhysX registration.** When the loaded
  OVStage population bridge can report registration ordering, native
  `ovphysx_attach_ovstage()` checks registration of the `physxSchema` and
  `omniUsdPhysicsDeformableSchema` families once USD's schema definitions are
  built. A missing family rejects admission with an actionable registration
  error before acquiring stage ownership or creating a task. The error names
  the external provider, registration before the first USD schema read (population
  or export), and process restart after a late registration; it never references
  the removed schema-location APIs.
  The failed result remains sticky for the process because a late probe changes
  plugin registration bookkeeping without repairing the frozen schema definitions.
  Repeated and concurrent attaches cannot turn that failed probe into success.
  Registration of both families from another directory or through OVStage's
  plugin-path variable is accepted. Typeless verification descriptors provide
  no schema definitions and select no provider. On Linux, invalid or relative
  temporary-directory overrides fall back to a private directory under `/tmp`;
  they neither bypass verification nor put probe files in the launch directory.
  Probe directories are created exclusively with owner-only mode on Linux and
  a protected effective-user/SYSTEM DACL on Windows; their descriptors inherit
  those restrictions and are removed after synchronous registration.
  While USD schema definitions remain unbuilt, attach does not probe the real
  family identities and leaves the check armed; an application can still register
  its own provider afterwards. The legacy
  Carbonite setting `/ovphysx/schemas/requireRegistration` defaults to `true`;
  setting it to `false` downgrades refusal to a warning without repairing the
  missing data or clearing the failed result. This escape hatch also supports
  procedural stages in a process whose earlier USD schema read, including an
  export with no USD population, occurred without the PhysX schemas. Python
  reaches the same native guard; its separate Newton warning remains governed
  by AC-5. The check observes plugin bookkeeping, not a
  read-only view of frozen schema definitions. If the application attempted
  late registration before the first guard check, OVStage has already inserted
  the family identities despite returning an ordering error; this case cannot
  be detected. The application must restart on that original error, regardless
  of a subsequent successful attach. Registry access hidden from OVStage by
  another USD consumer also remains outside detection.

## Test References

- [TEST-CAPI-OVSTAGE-SCHEMA-001](../../tests/capi/TEST-CAPI-OVSTAGE-SCHEMA-001.md)

## Code References

- ovphysx/tests/python_samples/schema_support/__init__.py (AC-1, AC-2)
- ovphysx/tests/c_samples/common/schema_support.h (AC-1, AC-2)
- ovphysx/tests/c_samples/common/fetch_physx_schemas.py (AC-2)
- ovphysx/scripts/package_deps.py and ovphysx/scripts/build_wheel.cmake (AC-1)
- ovphysx/scripts/verify_pyless_closure.py (AC-1, AC-3)
- ovphysx/tests/c_samples/common/ovstage_sample.h and ovphysx/tests/c_unittests/test_utilities.h (AC-4)
- ovphysx/tests/python_samples/*.py and ovphysx/tests/python_tests/test_utils.py (AC-4, AC-5, AC-7)
- ovphysx/tests/python_tests/test_documentation_contracts.py (AC-7)
- ovphysx/python/ovphysx/utils/codeless.py (stock-USD registration, AC-2, AC-4)
- ovphysx/python/ovphysx/__init__.py (AC-3)
- ovphysx/python/ovphysx/schemas.py and ovphysx/python/ovphysx/api.py (AC-5)
- ovphysx/tests/python_tests/cpu_tests/test_newton_schema_registration.py (AC-5)

- ovphysx/scripts/build.cmake and ovphysx/scripts/build_dev_schema_provider.cmake (AC-6)
- ovphysx/scripts/dev_schema_provider.cmake and ovphysx/scripts/fetch_test_schemas.cmake (AC-6)
- ovphysx/scripts/run_uv_pytest.py (AC-6)
- ovphysx/src/ovphysx/ovphysx.cpp (AC-8)
- ovphysx/src/include/internal/sdk/PhysxSchemaRegistration.hpp (AC-8)
- ovphysx/tests/python_tests/cpu_tests/test_attach_requires_physx_schemas.py (AC-8)

## Dependencies

- [REQ-PACKAGING-USDFREE-001](../packaging/REQ-PACKAGING-USDFREE-001.md) -- excludes schema resources from the product
