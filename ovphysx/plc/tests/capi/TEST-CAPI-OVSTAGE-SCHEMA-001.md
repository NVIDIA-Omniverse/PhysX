<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-SCHEMA-001
maps_to: REQ-CAPI-OVSTAGE-SCHEMA-001
type: integration
---

## Scenario

ovphysx tells the application where its codeless PhysX schemas are and does
nothing else about them: discovery works from every shipped layout and fails
closed, no ovphysx entry point registers the schemas on the application's behalf
or touches the USD plugin-path environment, an application that registers the
reported root with OVStage before its first population gets schema fallbacks
resolved, an ovstage populated without that registration is refused at
attach, and an attach whose population ran without the separately installed
Newton USD schema is warned about. Covers REQ-CAPI-OVSTAGE-SCHEMA-001 AC-1
through AC-6.

## Given

- An installed SDK at `_install/` (`cmake -P scripts/install.cmake`) with
  `_install/schemas/physx/plugInfo.json` and the `PhysxSchema` and
  `OmniUsdPhysicsDeformableSchema` module directories.
- For the layout cases: temporary fake trees, `sdk/lib/libovphysx.so` next to
  `sdk/schemas/physx/plugInfo.json`, and `app/libovphysx.so` next to
  `app/schemas/physx/plugInfo.json` under a directory that also carries an
  unrelated `schemas/`, selected through `OVPHYSX_LIB`; and a `bad/lib`
  layout with no schema tree.
- `OV_PXR_PLUGINPATH_2511` and `PXR_PLUGINPATH_NAME` set to sentinel values.
- For the end-to-end case: a fresh process, the shared C++ or Python test
  helper registering the root with `ovstage_population_register_usd_schemas()`
  / `ovstage.population.register_usd_schemas()` before its first population.
- For the attach gate: three fresh CPU-mode Python interpreters populating
  `tests/data/basic_simulation.usda` with the plugin-path variables scrubbed
  from their environment: one without any registration, one registering the
  root first, and one without registration but created with the Carbonite
  override `/ovphysx/schemas/requireRegistration = false`.
- For the Newton check: the `newton-usd-schemas` package installed in the test
  environment, a one-hinge articulation whose revolute joint authors only
  `newton:velocityLimit = 111` and a `newton:ovphysxTestOnly = 7` that no
  shipped schema defines, and fresh CPU-mode interpreters that register the
  codeless root and, respectively: also register `ovphysx.newton_schema_root()`;
  also register a complete copy of that package from another directory;
  register nothing else; register nothing else with `RuntimeWarning` promoted
  to an error; hide the package from `importlib.util.find_spec`; register
  nothing else but create the instance with
  `/ovphysx/schemas/warnMissingNewtonSchema` set to `False` or to the string
  `"false"`; or first attach a procedurally authored `PhysicsScene`
  (`ovphysx.population.PrimBatch`) with no USD population in the process, then
  register a copy of the Newton schema whose `NewtonJointAPI` also declares
  `newton:ovphysxTestOnly`, and only then populate the USD scene.

## When

- `ovphysx_get_codeless_schema_root()` is called against the installed SDK,
  against each fake layout, with `NULL`, and from eight threads at once.
- A fresh interpreter runs `import ovphysx` and `ovphysx.codeless_schema_root()`
  with the plugin-path variables unset and, separately, pre-set.
- A deformable body and a deformable material are populated (`DOMAIN_PHYSICS`)
  after the helper registration and read back through the tensor-binding API.
- Each attach-gate interpreter populates the scene, seals ordinal 1, creates a
  CPU-mode `PhysX` and calls `attach_ovstage(stage, read_ordinal=1)`; the
  unregistered one calls it a second time after the refusal.
- Each Newton-check interpreter populates and seals the scene, attaches with
  `RuntimeWarning`s recorded, detaches and attaches again, steps once and reads
  `jointMaxVelocity` for `ARTICULATION_JOINT` at `ObjectScope.ALL`; the
  error-promoting one attaches inside `warnings.simplefilter("error")`, then
  attaches again; the procedural one also reads `newton:ovphysxTestOnly` on
  the hinge back through `ovstage` (`read_attributes`) after population.
  `ovphysx.newton_schema_root()` is also called in-process with the package
  present (after dropping it from `sys.modules`) and with `find_spec` hidden.

## Then

- Against the installed SDK the call succeeds with a NUL-terminated path whose
  directory holds `plugInfo.json`, `PhysxSchema/resources/plugInfo.json`,
  `PhysxSchema/resources/generatedSchema.usda`, and
  `OmniUsdPhysicsDeformableSchema/resources/plugInfo.json`; the fake SDK layout
  resolves to `sdk/schemas/physx` and the copied-runtime layout to
  `app/schemas/physx`, never to the unrelated sibling (REQ AC-1) --
  `SchemaPaths.InstalledSdkShipsCodelessSchemaRoot`,
  `SchemaPaths.ResolvesSdkLayoutFromOvphysxLib`,
  `SchemaPaths.ResolvesCopiedRuntimeLayout` in
  `tests/c_unittests/test_schema_paths.cpp`.
- `NULL` yields `OVPHYSX_API_INVALID_ARGUMENT`; the `bad/lib` layout yields
  `OVPHYSX_API_ERROR`, an empty output, and a last error naming
  `schemas/physx/plugInfo.json` and `OVPHYSX_LIB`; switching `OVPHYSX_LIB` to
  the good layout on the next call succeeds (REQ AC-2) --
  `SchemaPaths.NullOutputIsRejected`,
  `SchemaPaths.MissingSchemasFailWithActionableErrorAndRetry`.
- Both sentinel environment values are unchanged after the query, and the
  eight concurrent calls all succeed with the same root (REQ AC-3, AC-1) --
  `SchemaPaths.QueryDoesNotTouchUsdEnvironment`,
  `SchemaPaths.ConcurrentQueriesReturnTheSameRoot`.
- After `import ovphysx` and the Python query both variables are still unset,
  or still hold their pre-set values; `ovphysx._native_bootstrapped` is `False`
  and `pxr` is not imported; the reported root is a directory with the root
  registry and at least one complete module (REQ AC-1, AC-3) --
  `tests/python_tests/lifecycle_tests/test_import_has_no_usd_side_effects.py`.
- The deformable body and deformable material tensor reads find the populated
  objects and their schema-declared attributes rather than defaults (REQ AC-4)
  -- `test_tensor_binding.cpp` GPU cases
  (`GpuDeformableBodyReadWriteAndReadOnly` and
  `GpuDeformableMaterialReadIndexedAndMaskedWrite`), which run after
  `test_utils::attach_usd_with_ovstage()` registered the root; the Python suite
  goes through `test_utils.attach_usd_with_ovstage()` the same way. Their scene
  authors the properties it reads, so they exercise the registered path without
  isolating the schema fallbacks.
- The unregistered interpreter's attach raises `RuntimeError` whose text names
  `register_usd_schemas`, `codeless_schema_root` and the dropped `Physx*` APIs,
  the repeated attach raises the same error (the instance stayed detached), and
  no step ran; the registered interpreter attaches, steps once and detaches
  cleanly; the opted-out interpreter attaches and steps, and its log carries the
  same text as a warning ending in `requireRegistration is false` (REQ AC-5) --
  `tests/python_tests/cpu_tests/test_attach_requires_physx_schemas.py`. The
  cases skip when `ovstage.population.available()` is false.
- The interpreters that registered the Newton schema, from the installed
  package or from the copy in another directory, attach without a
  `RuntimeWarning` and read back `111`; the one that did not attaches with
  exactly one `RuntimeWarning` naming the late registration,
  `newton_schema_root`, `register_usd_schemas` and
  `warnMissingNewtonSchema`, and reads back `FLT_MAX`; with the warning
  promoted to an error the first attach raises that `RuntimeWarning` and the
  retry attaches (the instance held no reference to the stage); the one with
  the package hidden gets no `RuntimeWarning` and exactly one `WARNING`
  record on the `ovphysx` logger carrying `pip install newton-usd-schemas`
  and the GitHub URL, while the registered and unregistered interpreters log
  nothing; both opted-out ones (`False` and
  `"false"`) get none. The procedural-first interpreter gets no warning, reads
  `111`, and reads `7` for `newton:ovphysxTestOnly`, so the application's copy
  is the family USD used. In every case the second attach adds no warning (REQ
  AC-6) -- `tests/python_tests/cpu_tests/test_newton_schema_registration.py`.
  `newton_schema_root()` returns a directory holding `plugInfo.json` and
  `generatedSchema.usda` without importing `newton_usd_schemas`, and raises
  `FileNotFoundError` with the install hint when hidden (REQ AC-6) --
  `tests/python_tests/test_codeless_schemas.py`
  (`test_newton_schema_root_*`).

## Coverage

- AC-1, AC-2, AC-3: automated in `test_schema_paths.cpp` (CTest label
  `cpp-unit`) and `lifecycle_tests/test_import_has_no_usd_side_effects.py`.
- AC-4: automated indirectly by the deformable tensor-binding cases above; there is no dedicated negative case that populates without
  registration and asserts missing fallbacks, because a late registration
  poisons process-global USD state and would need its own process.
- AC-5: automated in `cpu_tests/test_attach_requires_physx_schemas.py`, one
  fresh interpreter per case for the same reason.
- AC-6: automated in `cpu_tests/test_newton_schema_registration.py` (one fresh
  interpreter per case) and `test_codeless_schemas.py`. The undetectable case
  (a registration OVStage cannot observe because another USD consumer read the
  schema definitions first) is documented, not tested.
