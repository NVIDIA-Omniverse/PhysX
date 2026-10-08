<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-NOUSD-001
title: ovphysx Ships USD-Free By Default And Bundles No USD Runtime
status: superseded
superseded_by: REQ-PACKAGING-USDFREE-001
owner: ovphysx
---

> **Retired 2026-09-01 (ADR-0027 Flip-B); replaced by REQ-PACKAGING-USDFREE-001.**
> The product guarantee — `libovphysx.so` links no USD, no monolith in the payload
> or wheel, USD resolved only from ovstage (AC-3, AC-4, AC-10, AC-11, AC-12) —
> survives unchanged in REQ-PACKAGING-USDFREE-001 (AC-2/3, AC-4, AC-6, AC-7). The
> mechanism-describing criteria do not: ADR-0027 removed the `OVPHYSX_NO_USD`
> option, so AC-1 (defaults ON), AC-2 (both arms build), AC-5 (option help text),
> AC-7 (re-derives the ovruntime flag), AC-9 ("the two arms agree"), AC-13 (the
> `OVRUNTIME_ENABLE_TESTS`-incompatible gate) and the "OFF arm retained" language
> (incl. *Known limitations*, which bounds the OFF arm's extra surface) are void —
> there is no OFF arm; the developer build adds a test-only static USD library
> that is never packaged (REQ-USDLIB-LOADABLE-001). History left intact; no
> source file should annotate this REQ.

## Description

Neither `libovphysx.so` nor the internal sidecar `libovphysx_internal.so` links
a USD library, and the ovphysx product payload contains no USD at all: no
namespaced monolith (`libov_<ver>usd_ms`), no USD dependency closure (TBB,
MaterialX, Alembic, Imath, OpenSubdiv, draco, hdStorm), no USD plugin registry
(`plugins/usd`), and no USD-runtime configuration (`config.toml`). The only
USD-related artifact ovphysx ships is data: the codeless PhysX schema tree
`schemas/physx` governed by REQ-CAPI-OVSTAGE-SCHEMA-001.

Both halves exist for the same reason: USD's namespaced monolith owns
process-wide singletons -- the `TfType` registry, the `TF_DEBUG` symbol table,
the `UsdUtilsStageCache` -- so a process that reaches two distinct monolith
images at two distinct paths registers those singletons twice and aborts. ovstage
brings its own namespaced USD runtime and the application owns whatever OpenUSD
it authors with; ovphysx never loads, preloads, detects, version-checks, or
configures either. `libovphysx.so`
has a `DT_NEEDED` on `libovstage.so`, and `libovstage.so` resolves its own USD
and TBB closure through its own `RUNPATH`.

The link-time half is a CMake option, `OVPHYSX_NO_USD`, which sets ovruntime's
`OVRUNTIME_PHYSX_NO_USD=ON` for the `OvruntimePhysX` subbuild. ovphysx never
attaches a native `UsdStage`: its scene loading goes entirely through ovstage
population, which is already USD-free. The mechanism `OVRUNTIME_PHYSX_NO_USD`
itself uses — conditional compilation, and the relocation of genuinely USD-only
work behind the `IPhysicsDataWrite` sink and the parse-backend registry — is
governed by ADR-0018, not by this requirement.

The guarantee is **build-time, not merely link-time** (AC-13). A configuration
that compiles against the OpenUSD SDK and then happens to link none of it holds
its property by archive-member non-extraction: any new `#include <pxr/...>`
compiles silently and the property is lost the moment one of those members is
pulled in. So under `OVRUNTIME_PHYSX_NO_USD` no ovruntime target is given the
USD or physxSchema header directories at all, the one module that is USD-only
by construction is not configured, and the ovphysx targets themselves are given
no USD include or link directory in either arm. A stray pxr include is a "no
such file or directory" at the point it is introduced, which is the enforcement
mechanism -- there is no counter, ratchet or symbol budget to keep in sync.

The option **defaults to `ON` and is the shipping configuration**; it was
introduced as an experimental opt-in — its old name carried an `EXPERIMENTAL_`
marker — and was promoted once its remaining functional gap (scene
replication/cloning) closed. The `OFF` arm is retained, not deprecated: it is the
USD-linked variant that ovruntime's own USD-backed tests and samples build
against, and the variant a Kit-style consumer attaching a native `UsdStage`
needs. It is a development aid, not a shipped configuration.

The payload half is enforced, not merely intended: `install.cmake` and
`build_wheel.cmake` scan the complete `_install/` and wheel staging trees and
fail the build on any USD file, any USD import, any `plugins/usd` registry, or a
missing codeless schema tree (AC-10 through AC-12).

## Acceptance Criteria

- AC-1: **`OVPHYSX_NO_USD` defaults to `ON`.** A from-scratch configure of
  `ovphysx/CMakeLists.txt` with no explicit `-D` flag resolves
  `OVPHYSX_NO_USD=ON` and, from it, `OVRUNTIME_PHYSX_NO_USD=ON` for the
  `OvruntimePhysX` subbuild.
- AC-2: **Both arms build.** A from-scratch build of the `ovphysx` CMake target
  succeeds with `OVPHYSX_NO_USD=ON` and with `OVPHYSX_NO_USD=OFF`.
- AC-3: **No USD `NEEDED` in any ovphysx binary.** With `OVPHYSX_NO_USD=ON`, the
  dynamic sections (`readelf -d`) of the built `libovphysx.so` and
  `libovphysx_internal.so` list no USD shared library (no `*usd_ms.so`, no
  `libusd_*.so`) as `NEEDED`; on Windows neither `ovphysx.dll` nor
  `ovphysx_internal.dll` imports one.
- AC-4: **No undefined USD symbols.** With `OVPHYSX_NO_USD=ON`, the dynamic
  symbol tables of `libovphysx.so` and `libovphysx_internal.so` have zero
  undefined (`UND`) symbols whose demangled name contains `pxr`, `::Usd`,
  `::Sdf`, `::Tf`, or `::Gf`.
- AC-5: **The option's help text names what each arm is for.** The CMake
  `option()` help text and the comment block above it state that `ON` is the
  shipping product configuration and that `OFF` selects the USD-linked variant
  for tests, samples, and native-`UsdStage` consumers, so a caller learns which
  arm it is selecting at configure time rather than at run time. Any capability
  not available under `ON` is listed under *Known limitations* below.
- AC-6: **Unsupported operations fail loudly, never silently.** Any operation a
  caller can explicitly request that is not supported under `ON` logs a specific
  error identifying the unsupported operation and returns a failure status to
  its caller; through the public C API that surfaces as a
  non-`OVPHYSX_API_SUCCESS` status, never a success with no effect. The one
  documented exception is the USD write-back sink: with no USD stage to write
  to, `getDataWrite()` is null by construction and every write-back call site is
  a deliberate no-op rather than an error.
- AC-7: **Toggling the option re-derives the ovruntime flag.** The
  `OVRUNTIME_PHYSX_NO_USD` cache variable is re-derived from `OVPHYSX_NO_USD` on
  every top-level reconfigure, so switching arms cannot leave a stale value
  behind from an earlier configure in either direction. A parent project that
  consumes ovphysx via `add_subdirectory()` may override
  `OVRUNTIME_PHYSX_NO_USD` and that override is respected.
- AC-8: **The full ovstage-attach surface works under the option.**
  `ovphysx_attach_ovstage()` attaches and loads a scene, creating rigid bodies,
  joints, articulations, collision shapes, deformable bodies and particle
  systems; `ovphysx_clone()` replicates a loaded scene; those objects are
  readable and writable through the public read/write surface; and
  `ovphysx_destroy_instance()` tears the instance down cleanly.
- AC-9: **The two arms agree on default-build behavior.** A retype or relocation
  made so that a code path compiles USD-free leaves the `OFF` configuration
  behaving identically. In particular, an object registered under a
  source-agnostic object key must remain findable through the USD path-keyed
  lookups the `OFF` build still uses: a registration path that populates only
  one of the two indexes is a defect in the `OFF` build, not merely a gap under
  the option.
- AC-10: **The product payload ships no USD.** No file matching
  `libov_*usd_ms.so*`, `ov_*usd_ms.dll`, `libusd_*.so*`, `usd_*.dll`,
  `libomni_usd_resolver*`, or USD's dependency closure (`libtbb*`, `tbb*.dll`,
  `libMaterialX*`, `libAlembic*`, `libImath*`, `libosd*`, `libdraco*`,
  `hdStorm*` and their Windows names) appears anywhere under the installed SDK
  tree (`_install/`) or inside the built wheel, no `plugins/usd` directory
  exists in either, and neither tree contains a `config.toml`. The only schema
  payload is `schemas/physx` (SDK) / `ovphysx/schemas/physx` (wheel): a root
  `plugInfo.json` plus `<Module>/resources/{plugInfo.json,generatedSchema.usda}`
  for every shipped module.
- AC-11: **No shipped ovphysx binary depends on USD.** No shared library or
  plugin under `_install/` or in the wheel has a `DT_NEEDED` (Linux) or PE
  import (Windows) on a USD library, and no ovphysx source compiles against a
  USD header: the ovphysx CMake targets receive no USD include or link
  directory, and `libovphysx` performs no USD preload, detection, or version
  check at instance creation.
- AC-12: **Packaging fails closed.** `scripts/install.cmake` and
  `scripts/build_wheel.cmake` run `scripts/verify_pyless_closure.py
  --require-schemas` over the complete `_install/` and wheel staging trees and
  fail with a specific error on any AC-10 file, any AC-11 import, a `plugins/usd`
  directory, or a missing or incomplete `schemas/physx` tree. A payload that
  violates AC-10 or AC-11 is not a producible output.
- AC-13: **USD is absent at compile time, not just at link time.** With
  `OVRUNTIME_PHYSX_NO_USD=ON`:
  - every `ovruntime_add_external_system_includes(<target>)` call reached by the
    configure passes `NO_USD`, so no target's generated `flags.make` /
    `CXX_INCLUDES` carries a `-isystem` for `target-deps/usd/*/include`,
    `.../include/pxr/external`, `.../include/boost`, or `USD_EXT_PHYSICS_DIR`;
  - `source/omni.physics.usd` — the module in which every translation unit
    includes pxr — is not added as a subdirectory, so neither its objects nor
    its `install()` rules exist in that configuration;
  - consequently a translation unit that adds `#include <pxr/...>` fails to
    compile rather than compiling and being dropped by the linker.
  `OVRUNTIME_ENABLE_TESTS` is incompatible with the option and the configure
  fails with a named error rather than silently re-admitting USD through the
  test targets.
- AC-14: **No shipped SDK header includes pxr.** No header installed into the
  SDK `include/` tree `#include`s `pxr/`, so the shipped headers are
  self-contained against a consumer that has no OpenUSD. `common/` — the
  USD-free base library — owns no pxr-typed source or header; pxr-typed material
  that used to sit there (`TypeCast.h`, `PrimUtilities.{h,cpp}`,
  `ImguiDrawingUtils.{h,cpp}`) lives in `source/omni.physics.usd/`, and
  `UsdMaterialParsing.{h,cpp}` was deleted as dead — `OvstageSource::getMaterialBinding(ObjectKey)`
  is the pxr-free replacement in use.

## Known limitations

The `ON` arm — the shipping product — does not offer the following. Each is
USD-only by construction and has no ovstage equivalent; none is reachable
through ovphysx's public C or Python API, so this list bounds the `OFF` arm's
extra surface rather than describing a hole in the product.

- **Kit-viewport-only rendering and cosmetic authoring.** Diffuse-particle and
  isosurface render prims, anisotropy primvars, the renderer-skip hint, prim
  visibility and instancer-prototype radius all need a real USD prim to author
  onto. Where they route through the write sink they are no-ops; where they
  cannot, they are compiled out.
- **USD authoring with no ovstage equivalent.** Deformable-attachment authoring
  needs whole-prim create/destroy, which the write sink does not offer;
  point-instancer transform write-back needs the `UsdGeomPointInstancer` typed
  schema; the joint-state initial-state restore path authors through a raw
  schema handle.
- **Kit-only entry points.** Stage-id attach and session attach resolve a Kit
  `UsdStage` through the USD stage cache. Prim property queries answer every
  request with an invalid-USD-stage error. Custom shape / joint / instancer
  token registration is unavailable because the USD module is not linked into
  the target at all. The infinite voxel map remains USD-typed; it is live
  functionality in the `OFF` build, not dead code.
- **Legacy path-integer decoding.** The one remaining API arm that decodes a
  `uint64_t` as the in-memory bit pattern of a USD path — the
  never-attached-foreign-stage collision-representation fallback — is USD-only,
  since no attached source exists to mint a portable key.
- **The embedded runtime still imports one ovstage accessor that names USD.**
  `OvstageParseBackend` imports `ovstage_get_usd_stage_id` from `libovstage`
  to report a backing-stage identifier. It is a plain-integer ovstage C API,
  not a USD type or library dependency, and ovstage marks it temporary; it is
  outside this requirement.
- **CI does not build both arms.** CI builds the default (`ON`) arm only, so
  AC-2's `OFF` half and AC-9 are verified by hand. See
  TEST-PACKAGING-NOUSD-001.

## Test References

- [TEST-PACKAGING-NOUSD-001](../../tests/packaging/TEST-PACKAGING-NOUSD-001.md) --
  partly automated. AC-10, AC-11 and AC-12 are automated by the install- and
  wheel-time `verify_pyless_closure.py --require-schemas` gate and by
  `tests/python_tests/test_verify_pyless_closure.py` and
  `tests/python_tests/test_package_deps.py`
  (`test_package_deps_stages_no_usd_payload`,
  `test_package_deps_exports_codeless_schemas_into_install_tree`); AC-3 is
  automated for the installed tree by the same gate's `DT_NEEDED` / PE-import
  scan. AC-1, AC-2, AC-4, AC-7, AC-9, AC-13 and AC-14 are verified by hand (a
  two-configuration build, `readelf -d` / `readelf --dyn-syms` on each
  resulting library, and a grep of the NO_USD tree's generated `flags.make`
  files and installed `include/` tree); see that TEST for the exact commands and
  expected outcomes. AC-8's runtime picture is verified against the
  installed-SDK `c_unittests` GPU suite, which is not driven by this TEST.

## Code References

- ovphysx/CMakeLists.txt (`OVPHYSX_NO_USD` option, its documentation comment,
  the `OVRUNTIME_PHYSX_NO_USD` cache-var wiring, and the absence of any USD
  include or link directory on the `ovphysx` target -- AC-1, AC-2, AC-5, AC-7,
  AC-11)
- ovphysx/src/ovphysxInternal/CMakeLists.txt (the sidecar links only `ovphysx`;
  no USD library, include, or link directory -- AC-3, AC-4, AC-11)
- ovphysx/scripts/package_deps.py (stages Carbonite and PhysX runtime payloads;
  exports the codeless schema tree; stages no USD -- AC-10)
- ovphysx/scripts/export_codeless_schema.py (the `schemas/physx` tree AC-10
  requires)
- ovphysx/scripts/verify_pyless_closure.py (`FORBIDDEN_FILENAMES`,
  `FORBIDDEN_DT_NEEDED`, `FORBIDDEN_PE_IMPORTS`, `_check_no_usd_plugin_registry`,
  `_check_codeless_schemas` -- AC-10, AC-11, AC-12)
- ovphysx/scripts/install.cmake and ovphysx/scripts/build_wheel.cmake (the
  `--require-schemas` verifier invocations that fail the build -- AC-12)
- ovphysx/src/CarboniteLoader/CarboniteLoader.cpp and
  ovphysx/src/ovphysx/ovphysx.cpp (`createInstanceInternal`: Carbonite and
  PhysX startup with no USD preload, detection, or version check -- AC-11)
- ovphysx/ovruntime/source/omni.physx/plugins/PhysX.cpp (path-keyed creation and
  simulation-interface entry points; compile-time prerequisite for AC-2 under
  `OVRUNTIME_PHYSX_NO_USD`)
- ovphysx/ovruntime/source/omni.physx/plugins/PhysXReplicator.cpp (the
  replication path AC-8 now covers)
- ovphysx/ovruntime/include/omni/physics/parse/IPhysicsDataWrite.h (the pxr-free
  write sink USD-only authoring is relocated behind — AC-6's null-sink no-op
  contract)
- ovphysx/ovruntime/source/omni.physics.usd/UsdPhysicsDataWrite.cpp (the sole
  implementation of that sink; not built at all in the `ON` configuration)
- ovphysx/ovruntime/cmake/ExternalSystemIncludes.cmake
  (`ovruntime_add_external_system_includes()`'s `NO_USD` argument — the
  mechanism behind AC-13's first bullet)
- ovphysx/ovruntime/CMakeLists.txt (the `if(NOT OVRUNTIME_PHYSX_NO_USD)` gate on
  `add_subdirectory(source/omni.physics.usd)`, and the
  `OVRUNTIME_ENABLE_TESTS` incompatibility error — AC-13)
- ovphysx/ovruntime/source/common/CMakeLists.txt (`OvruntimeCommon` compiled
  `NO_USD` in every configuration; the install lists that no longer ship a
  pxr-including header — AC-13, AC-14)
- ovphysx/ovruntime/source/omni.physics.usd/CMakeLists.txt (`TypeCast.h`,
  `PrimUtilities.{h,cpp}` and `ImguiDrawingUtils.{h,cpp}` relocated out of
  `common/` — AC-14)

## Dependencies

- ADR-0018 (`ovphysx/ovruntime/plc/adr/ADR-0018-usd-free-runtime-build.md`) —
  governs the underlying `OVRUNTIME_PHYSX_NO_USD` mechanism, the design
  decisions and rejected alternatives behind the remaining limitations above,
  the public wire-format change that the port required, and the decision to ship
  USD-free by default with no USD in the payload.
- ADR-0021 — object-key generation-tag semantics, which bound what the relocated
  interfaces can resolve.
- [REQ-CAPI-OVSTAGE-SCHEMA-001](../capi/REQ-CAPI-OVSTAGE-SCHEMA-001.md) -- the
  codeless schema tree that is the one USD-related artifact AC-10 allows, and
  the application-owned registration that replaces ovphysx's former implicit
  registration.
