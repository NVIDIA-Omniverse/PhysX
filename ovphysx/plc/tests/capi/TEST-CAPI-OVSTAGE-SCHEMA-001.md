<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-SCHEMA-001
maps_to: REQ-CAPI-OVSTAGE-SCHEMA-001
type: integration
---

## Scenario

External provider discovery and application registration work independently of
the ovphysx installation. Covers AC-1 through AC-8.

## Given

- A separately installed `physx-usd-schemas` Python wheel or native
  extraction, containing both resource modules and distribution metadata.
- Temporary provider installations with missing resources, different versions,
  incorrect package identity, and an initializer that raises if imported.
- Fresh subprocesses for USD registration and population-order tests.

## When

- The application recipes discover a fresh provider, move it to another root,
  and discover it again from current metadata or the new explicit native root.
- Discovery runs with an absent provider, a differently versioned provider, or
  a missing module resource file, before any population call.
- Dependency resolution runs offline with only a wheel below the declared
  minimum version available.
- OVStage samples and tests register both resources before population.
- Stock-USD registration runs before and after its schema registry is built.
- The existing Newton registration/probe scenarios run unchanged apart from
  discovery of the separately installed PhysX provider.

## Then

- AC-1: `ExternalSchemaProvider.FreshAndRelocatedInstallation` and
  `test_schema_provider.py` locate both modules in the relocated installation;
  `SchemaPaths.SeparatelyInstalledProviderContainsBothModules` checks the real
  extraction. Discovery imports neither the provider nor pxr.
- AC-2: Native `MissingProviderFailsBeforePopulation` and
  `IncompleteProviderNamesMissingFile`, plus `test_schema_provider.py`, reject
  absent and incomplete providers with installation/root guidance. Native,
  Python-sample and authoring discovery accept a different provider version.
  The resolver rejects a candidate below the manifest constraint; the native
  installer rejects an incorrect package identity while retaining digest checks.
- AC-3: `test_codeless_schemas.py` asserts removed Python locators are absent;
  `lifecycle_tests/test_import_has_no_usd_side_effects.py` verifies no environment
  mutation, pxr import, or native bootstrap. The install/wheel package gate and
  `test_verify_pyless_closure.py` reject bundled `generatedSchema.usda` files.
- AC-4: C/Python samples and the existing deformable integration cases register
  the real provider before population. `test_session_physics.cpp` covers
  deformable body and material session reads. The codeless registration samples and
  `utils_tests/test_utils_schema.py` verify stock-USD registration ordering and
  fallback resolution. `test_real_provider_registration_and_fallbacks_after_relocation` copies the
  real provider, populates a PhysX fallback in fresh subprocesses, moves the
  provider, and repeats. Native/Python consumer runs exercise installed
  packages; no sample obtains schemas from an ovphysx library path.
  The installed-wheel `profiling.py` sample and the native no-GPU `cpu_usd`
  smoke explicitly register both external PhysX families before population.
- AC-5: `cpu_tests/test_newton_schema_registration.py` keeps registered,
  unregistered, warning-as-error, hidden-package, opt-out, alternate-root, and
  procedural-first coverage. `test_codeless_schemas.py` keeps Newton discovery
  and missing-package diagnostics without importing its package.
- AC-7: `test_documentation_contracts.py::test_copy_paste_samples_use_installed_schema_support`
  parses every `tests/python_samples/*.py` and fails on any `import` or
  `from ... import` whose module is `schema_support`. A sample that calls
  `physx_schema_paths` must import it from
  `ovphysx.samples.python_samples.schema_support`. Each doc page that embeds
  one of those samples must `{literalinclude}` that file by its path relative
  to the page.

## Coverage

Native discovery: `tests/c_unittests/test_schema_paths.cpp`.
Python discovery: `tests/python_tests/test_schema_provider.py` and
`tests/python_tests/test_codeless_schemas.py`.
Registration/population: C/Python samples and the existing deformable/Newton suites.
USD registry access performed by an unrelated host before OVStage can observe it
remains undetectable and documented, not claimed as covered.

## Local provider selection (AC-6)

`test_drop_stages_devschema_wheel_builder_inputs` in
`test_oss_release_packaging.py` copies the tracked schema source subset using
the real export configuration. The staged tree must include `scripts/build_wheel.py`,
`python/pyproject.toml`, `python/README.md`, and
`python/physx_usd_schemas/__init__.py`, while excluding the internal publishing
configuration `repo_tools.toml`. This prevents devschema builds from losing the
provider wheel builder or its inputs during source export.

`test_devschema_provider.py` exercises native CMake selection and Python metadata
precedence with a local provider, refusal to fall back when the selected provider
is missing, the unchanged published default, and the focused pytest wrapper.
Custom build paths (including spaces and absolute paths) must ignore stale
default-directory stamps. Explicit `BUILD_DIR` takes precedence over
`OVPHYSX_BUILD_DIR`.
The CMake path-selection probe includes `scripts/build_directory.cmake` with an
explicit project root, covering the default, command-line, environment, and
command-line-over-environment cases without unrelated host-tool discovery.
It runs the wheel-to-provider handoff twice with different resource bytes at the
same version and checks the selected contents and digest-specific location.
It also checks that a differently versioned local override retains digest and
package-identity validation, and that an incomplete cached provider is rejected.
On Windows, extraction and cached-provider reuse succeed when resource paths
exceed `MAX_PATH`, without changing the returned provider root.
Full `cmake -DDEV_SCHEMA=ON -P scripts/validate_all.cmake` exercises the actual schema wheel builder,
native registration, Python samples, and package isolation against local data.
Repeat with `-DBUILD_DIR=<custom-path>` to verify that packaging still uses the
shared dependencies and license archive under `_build`.
Build/packaging validation also runs with the worktree's `usd_ext_physics` link
unavailable and verifies that dependency fetching does not recreate it. Both
published and local wheel selection work without PhysX schema headers on C++ targets.
The existing native missing-resource diagnostic test covers the bounded module/
filename diagnostic used with GCC warnings-as-errors.

## Native PhysX registration guard (AC-8)

`cpu_tests/test_attach_requires_physx_schemas.py` runs each registration case in
a fresh interpreter, clearing inherited USD plugin-path variables except in the
case that explicitly tests that registration route. The test discovers the
external provider through sample helpers and never calls removed ovphysx schema
location APIs. It disables the unrelated Newton warning and collects 0.7 task
receipts for successful attach and detach.

- With no PhysX schemas registered before USD population, native attach fails
  with registration guidance, leaves the instance detached, and fails again on
  a repeated attempt. The test must distinguish this failure from a missing
  provider, broken import, or unsuccessful population setup.
- Registration of both external resource modules before population permits
  attach, simulation, and detach. Registration of only one family is insufficient.
- The same families registered from another root or the OVStage plugin-path
  environment variable are accepted.
- On Linux, relative, nonexistent, and file-valued `TMPDIR` overrides still
  allow a registered stage to attach and still reject missing registration.
  The relative-directory leftover assertion checks only this helper's
  `ovphysx-schema-probe-*` artifacts; unrelated runtime files such as OVStage's
  lock file are permitted. Existing file-valued overrides remain untouched.
  A separate live-root harness verifies that the probe selects `/tmp`, not the
  relative override or the launch directory. The existing
  `test_relative_temp_dir_is_not_used` cooking-cache test also completes without
  a schema-probe error.
- Platform-level probe validation checks Linux mode `0700`, Windows protected
  user/SYSTEM directory permissions and child inheritance, exclusive unique
  directory creation, and cleanup after synchronous descriptor use.
- Setting `/ovphysx/schemas/requireRegistration` to false permits attachment
  with a warning. Native log assertions inspect stdout and stderr together.
- A procedurally populated stage attached while USD schema definitions remain
  unbuilt succeeds; later application registration of the real provider still
  works. If the
  subsequent USD population omits registration, a later attach still refuses it.
- Export-only positive and negative cases use fresh processes: author and seal a
  procedural stage, attach and detach while USD schema definitions remain
  unbuilt, then export it with `export_typed_hierarchy_to_usd_file()` without any
  USD population. Registering both families before export permits the next
  attach; omitting registration makes it fail repeatedly with no attachment and
  provider and registration guidance. The successful pre-export attach must
  leave verification armed for that later schema read.
- The dependency limitation is characterized separately: populate without
  schemas, attempt late application registration and observe its ordering
  error, then attach for the first time. OVStage's changed plugin bookkeeping
  makes the guard accept; that acceptance does not establish schema availability.
  The original registration error requires a process restart.

The probe is process-scoped, not evidence of per-stage population provenance.
A procedural stage attached after an unrelated USD schema read, including
population or export without the schemas, is subject to the same refusal and
documented opt-out. Registration ordering hidden from OVStage by another USD
consumer remains outside the detection claim.
So does a late application registration attempted before the first guard check.
