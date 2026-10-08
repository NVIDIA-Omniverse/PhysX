<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-USDFREE-001
title: ovphysx ships one USD-free build; the payload carries no USD and the USD parsing library is never packaged
status: implemented
owner: ovphysx
---

> **Landed 2026-09-01 (ADR-0027 Flip-B), reconciled 2026-09-03 with the
> USD-free payload gate.** The `OVPHYSX_NO_USD` option is gone and ovphysx
> configures ovruntime USD-free unconditionally; `omni.physics.usd` is a test-only
> static library with no `install()`; the install and wheel steps fail closed on
> any USD file, import, plugin registry or `config.toml` in the payload.

## Description

Under ADR-0027 there is one ovphysx build, and it is USD-free. This requirement
replaces REQ-PACKAGING-NOUSD-001: the durable product guarantee — neither
`libovphysx.so` nor the internal sidecar `libovphysx_internal.so` links USD, and
the ovphysx payload contains no USD at all — is unchanged and strengthened, while
the two-arm/option framing of the retired requirement (`OVPHYSX_NO_USD` defaults
ON, "both arms build", the option help text, the `OVRUNTIME_ENABLE_TESTS`
incompatibility gate) is gone: there is no OFF arm.

The payload carries no namespaced monolith (`libov_<ver>usd_ms`), no USD
dependency closure (TBB, MaterialX, Alembic, Imath, OpenSubdiv, draco, hdStorm),
no USD plugin registry (`plugins/usd`) and no USD-runtime configuration
(`config.toml`). The only USD-related artifact ovphysx ships is data: the
codeless PhysX schema tree `schemas/physx` governed by
REQ-CAPI-OVSTAGE-SCHEMA-001. USD's namespaced monolith owns process-wide
singletons (`TfType`, `TF_DEBUG`, `UsdUtilsStageCache`), so a process reaching
two monolith images at two paths registers them twice and aborts. ovstage brings
its own namespaced USD runtime and the application owns whatever OpenUSD it
authors with; ovphysx never loads, preloads, detects, version-checks or
configures either. `libovphysx.so` has a `DT_NEEDED` on `libovstage.so`, which
resolves its own USD and TBB closure through its own `RUNPATH`.

The ovruntime developer/test build additionally produces the `omni.physics.usd`
test-only static library (REQ-USDLIB-LOADABLE-001), which links pxr — but it is a
test artifact linked only by test executables and is never staged into the
product payload.

## Acceptance Criteria

- AC-1: **One build, no option.** A from-scratch configure and build of the
  `ovphysx` CMake target produces the shipping artifacts with no
  `OVPHYSX_NO_USD` / `OVRUNTIME_PHYSX_NO_USD` flag; the option does not exist.
  The ovphysx CMake targets receive no USD include or link directory, and the
  production ovruntime targets are compiled without the USD header directories
  (REQ-BUILD-UNIBUILD-001); `source/omni.physics.usd` is configured only under
  `OVRUNTIME_ENABLE_TESTS`.
- AC-2: **No shipped ovphysx binary depends on USD.** No shared library or plugin
  under `_install/` or in the wheel — `libovphysx.so` and
  `libovphysx_internal.so` included — has a `DT_NEEDED` (Linux) or PE import
  (Windows) on a USD library (`readelf -d` lists no `*usd_ms.so`).
- AC-3: **No undefined USD symbols.** The dynamic symbol tables of
  `libovphysx.so` and `libovphysx_internal.so` have zero undefined (`UND`)
  symbols whose demangled name contains `pxr`, `::Usd`, `::Sdf`, `::Tf`, or
  `::Gf`.
- AC-4: **The product payload ships no USD.** No file matching
  `libov_*usd_ms.so*`, `ov_*usd_ms.dll`, `libusd_*.so*`, `usd_*.dll`,
  `libomni_usd_resolver*`, or USD's dependency closure (`libtbb*`, `tbb*.dll`,
  `libMaterialX*`, `libAlembic*`, `libImath*`, `libosd*`, `libdraco*`,
  `hdStorm*` and their Windows names) appears anywhere under the installed SDK
  tree (`_install/`) or inside the built wheel, no `plugins/usd` directory
  exists in either, and neither tree contains a `config.toml`. The only schema
  payload is `schemas/physx` (SDK) / `ovphysx/schemas/physx` (wheel): a root
  `plugInfo.json` plus `<Module>/resources/{plugInfo.json,generatedSchema.usda}`
  for every shipped module.
- AC-5: **The test-only USD parsing library is never packaged.** No file produced
  by the `omni.physics.usd` library (`*OvruntimePhysicsUsd*`) appears under the
  installed SDK tree or in the wheel. It exists only in the ovruntime developer
  build tree, linked into the unit-test executables.
- AC-6: **ovphysx touches no USD at run time.** `libovphysx` performs no USD
  preload, detection, or version check at instance creation, and loads no USD
  image of its own: the only USD ovphysx brings into a process is what
  `libovstage.so` resolves transitively through its own `RUNPATH`. What the host
  application loads for itself is outside this requirement.
- AC-7: **Packaging fails closed.** `scripts/install.cmake` and
  `scripts/build_wheel.cmake` run `scripts/verify_pyless_closure.py
  --require-schemas` over the complete `_install/` and wheel staging trees and
  fail with a specific error on any AC-4 file, any AC-2 import, a `plugins/usd`
  directory, or a missing or incomplete `schemas/physx` tree. A payload that
  violates AC-2 or AC-4 is not a producible output.
- AC-8: **No shipped SDK header includes pxr.** No header installed into the SDK
  `include/` tree `#include`s `pxr/`, so the shipped headers are self-contained
  against a consumer that has no OpenUSD. The `common/` utilities headers
  (`OmniPhysXUtilities.h`, `Utilities.h`) are pxr-free since ADR-0027 Flip-B
  (their pxr halves moved into `omni.physics.usd`, REQ-PUBLICAPI-HANDLE-001
  AC-3/AC-4) and are installed unconditionally like the rest of `common/`.

## Test References

- [TEST-PACKAGING-USDFREE-001](../../tests/packaging/TEST-PACKAGING-USDFREE-001.md)
  — AC-2, AC-4 and AC-7 automated by the install/wheel-time verifier gate and
  its pytest coverage; AC-1, AC-3, AC-5, AC-6 and AC-8 are a `readelf`/grep pass
  by hand at review.

## Code References

- ovphysx/CMakeLists.txt (no `OVPHYSX_NO_USD` option; ovruntime configured USD-free unconditionally; no USD include or link directory on `ovphysx` — AC-1)
- ovphysx/src/ovphysxInternal/CMakeLists.txt (the sidecar links only `ovphysx`; no USD library, include, or link directory — AC-2, AC-3)
- ovphysx/scripts/package_deps.py (stages Carbonite and PhysX runtime payloads; exports the codeless schema tree; stages no USD — AC-4, AC-7)
- ovphysx/scripts/export_codeless_schema.py (the `schemas/physx` tree AC-4 requires)
- ovphysx/scripts/verify_pyless_closure.py (`FORBIDDEN_FILENAMES`, `FORBIDDEN_DT_NEEDED`, `FORBIDDEN_PE_IMPORTS`, `_check_no_usd_plugin_registry`, `_check_codeless_schemas` — AC-2, AC-4, AC-7)
- ovphysx/scripts/install.cmake and ovphysx/scripts/build_wheel.cmake (the `--require-schemas` verifier invocations that fail the build — AC-4, AC-5, AC-7)
- ovphysx/src/CarboniteLoader/CarboniteLoader.cpp and ovphysx/src/ovphysx/ovphysx.cpp (`createInstanceInternal`: Carbonite and PhysX startup with no USD preload, detection, or version check — AC-6)
- ovphysx/ovruntime/source/omni.physics.usd/CMakeLists.txt (`STATIC`, tests-only, no `install()` — AC-5)
- ovphysx/ovruntime/CMakeLists.txt (`add_subdirectory(source/omni.physics.usd)` under `OVRUNTIME_ENABLE_TESTS` — AC-1, AC-5)
- ovphysx/ovruntime/source/common/CMakeLists.txt (`common/` headers installed unconditionally, all pxr-free — AC-8)

## Dependencies

- REQ-USDLIB-LOADABLE-001 — the test-only USD library this requirement keeps out of the payload.
- REQ-BUILD-UNIBUILD-001 — the single USD-free `omni.physx` compilation this payload is built from.
- REQ-CAPI-OVSTAGE-SCHEMA-001 — the codeless `schemas/physx` tree, the one USD-related artifact that does ship.
- Supersedes REQ-PACKAGING-NOUSD-001 (ADR-0027); its durable payload guarantees (AC-10 through AC-14 there) migrate here as AC-2, AC-4, AC-6, AC-7 and AC-8.
