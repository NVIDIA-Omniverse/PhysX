<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-NOUSD-001
maps_to: REQ-PACKAGING-NOUSD-001
type: integration
---

> **Retired 2026-09-01 (ADR-0027 Flip-B); replaced by TEST-PACKAGING-USDFREE-001.**
> The `readelf` no-monolith / no-USD-`NEEDED` checks and the `package_deps`
> validation live in TEST-PACKAGING-USDFREE-001, dropped down to the single
> USD-free build; the two-configuration build and option-help-text checks retired
> with the option. History left intact below.

## Coverage status

Partly automated.

- **Automated:** AC-10, AC-11 and AC-12 -- every `install.cmake` and
  `build_wheel.cmake` run executes `scripts/verify_pyless_closure.py
  --require-schemas` over the complete `_install/` and wheel staging trees and
  fails on any violation, so every successful install and wheel build is a
  passing run of the same checks; the verifier's own policy is covered offline
  by `tests/python_tests/test_verify_pyless_closure.py`
  (`test_verify_rejects_namespaced_usd_monolith_and_its_closure`,
  `test_verify_rejects_usd_plugin_registry_directory`,
  `test_verify_rejects_direct_usd_monolith_import`,
  `test_verify_rejects_upstream_modular_usd_library_names`,
  `test_verify_rejects_usd_runtime_config_file`,
  `test_verify_require_schemas_accepts_complete_codeless_tree`,
  `test_verify_require_schemas_rejects_nonfunctional_registry_and_module`,
  `test_verify_require_schemas_rejects_missing_root_registry_and_module_payload`),
  and the staging entry points (with the copy helpers mocked) by
  `tests/python_tests/test_package_deps.py`
  (`test_package_deps_stages_no_usd_payload`,
  `test_package_deps_exports_codeless_schemas_into_install_tree`). AC-3 is
  automated for the installed tree by the same gate's `DT_NEEDED` / PE-import
  scan of `lib/libovphysx.so` and `lib/libovphysx_internal.so`.
- **Manual:** AC-1, AC-2, AC-4, AC-7, AC-9, AC-13 and AC-14 -- the
  two-configuration build, the `readelf` comparison, and the
  `flags.make` / installed-`include/` greps below. See *Why the
  two-configuration pass is not automated*.

## Scenario

The shipping ovphysx configuration is USD-free: `OVPHYSX_NO_USD` defaults to
`ON`, neither `libovphysx.so` nor `libovphysx_internal.so` links USD, and the
product payload contains no USD library, no USD dependency closure, no USD
plugin registry and no USD configuration -- only the codeless `schemas/physx`
tree. The `OFF` arm still builds and still links USD, and toggling between the
arms re-derives `OVRUNTIME_PHYSX_NO_USD` rather than leaving a stale value. The
USD-free property is enforced at compile time, not just at link time: no
ovruntime target is given the USD header directories, the USD-only module is not
configured at all, the ovphysx targets receive no USD include or link directory,
and no shipped SDK header includes pxr. Covers REQ-PACKAGING-NOUSD-001 AC-1
through AC-4, AC-7, and AC-10 through AC-14.

Runtime behavior under the option is bounded by REQ-PACKAGING-NOUSD-001 AC-8 and
its Known limitations and is verified separately against the installed-SDK
`c_unittests` GPU suite; the design reasoning is in ADR-0018.

## Given

- An `ovphysx/` checkout with dependencies fetched (`./build.sh --generate`
  once, or an existing `_build/`).
- `readelf` and `c++filt` on `PATH`.
- For the automated half: an installed SDK at `_install/` (`cmake -P
  scripts/install.cmake`) and a built wheel staging tree
  (`cmake -P scripts/build_wheel.cmake`).

## When

- **Default build (no flag):** `cmake -S . -B _build && cmake --build _build
  --target ovphysx ovphysx_internal`, then read back `OVPHYSX_NO_USD` and
  `OVRUNTIME_PHYSX_NO_USD` from `_build/CMakeCache.txt`.
- **Control build (option off):** `cmake -S . -B _build_control
  -DOVPHYSX_NO_USD=OFF && cmake --build _build_control --target ovphysx
  ovphysx_internal`.
- For each build, locate the built shared libraries (Linux:
  `<build-dir>/linux-x86_64/release/libovphysx.so` and
  `libovphysx_internal.so`) and run on each:
  - `readelf -d <path> | grep NEEDED`
  - `readelf --dyn-syms <path> | awk '$7=="UND"' | c++filt | grep -iE
    "pxr|::Usd|::Sdf|::Tf\b|::Gf"`
- **Toggle pass:** reconfigure the control tree in place with `cmake -S . -B
  _build_control -DOVPHYSX_NO_USD=ON` and read back `OVRUNTIME_PHYSX_NO_USD`
  from its cache, then toggle it back.
- **Payload scan:** `find _install _dist \( -name 'libov_*usd_ms.so*' -o -name
  'ov_*usd_ms.dll' -o -name 'libtbb*' -o -name 'libMaterialX*' -o -name
  'libAlembic*' -o -name 'libImath*' -o -name 'libosd*' -o -name 'libdraco*' -o
  -name 'config.toml' -o -path '*/plugins/usd' \)`, the same scan inside the
  unpacked wheel, and `ls _install/schemas/physx/plugInfo.json
  _install/schemas/physx/*/resources/plugInfo.json`.
- **Compile-time USD scan (default build):**
  - `grep -rl 'target-deps/usd' _build --include=flags.make`
  - `ls -d _build/ovruntime/source/omni.physics.usd`
  - `grep -rn '#include *[<"]pxr/' _install/include`
  - a negative control: add `#include <pxr/usd/usd/prim.h>` to any
    `source/common/` or `src/ovphysx/` translation unit and rebuild.
- **Automated pass:** `cmake -P scripts/install.cmake` and `cmake -P
  scripts/build_wheel.cmake` (each runs the verifier), then
  `cd tests/python_tests && ./run_pytest.sh test_verify_pyless_closure.py
  test_package_deps.py`.
- **Negative packaging pass:** drop a file named `libov_25.11usd_ms.so` or a
  `plugins/usd/plugInfo.json` into `_install/plugins/`, or delete
  `_install/schemas/physx/plugInfo.json`, and rerun `python
  scripts/verify_pyless_closure.py --dir _install --require-schemas`.

## Then

- The default build's cache reads `OVPHYSX_NO_USD=ON` and
  `OVRUNTIME_PHYSX_NO_USD=ON` with no flag passed (REQ AC-1).
- Both the default and the control build succeed (REQ AC-2).
- Neither default-build library's `NEEDED` list contains a `*usd_ms.so` or
  `libusd_*.so` entry; the control build's `libovphysx.so` does (REQ AC-3, and
  AC-1's "the `OFF` arm is unchanged" half).
- Neither default-build library's undefined-dynsym grep produces output -- zero
  undefined symbols whose demangled name contains `pxr`, `::Usd`, `::Sdf`,
  `::Tf`, or `::Gf`. The control build's `libovphysx.so` produces many
  (REQ AC-4).
- After each toggle, `OVRUNTIME_PHYSX_NO_USD` matches the `OVPHYSX_NO_USD` value
  just passed — no stale value survives in either direction (REQ AC-7).
- The payload scan finds nothing in `_install/`, in the SDK archive, or in the
  wheel, and the `schemas/physx` listing shows the root registry plus the
  `PhysxSchema` and `OmniUsdPhysicsDeformableSchema` module registries
  (REQ AC-10).
- No `flags.make` under the default build mentions `target-deps/usd` -- neither
  under `_build/ovruntime/` nor for the `ovphysx` and `ovphysx_internal`
  targets -- and `_build/ovruntime/source/omni.physics.usd` is not configured --
  no such directory and no `OvruntimePhysicsUsd` rule in the generated
  makefiles. The control build has both (REQ AC-11, AC-13).
- The negative control fails the compile with `fatal error: pxr/usd/usd/prim.h:
  No such file or directory` rather than building and being silently dropped by
  the linker; and configuring the default arm with `-DOVRUNTIME_ENABLE_TESTS=ON`
  fails with the named "the ovruntime tests are USD-backed" error (REQ AC-13).
- No header under `_install/include` `#include`s `pxr/`; in particular
  `include/common/` contains no pxr-typed header (REQ AC-14).
- `install.cmake` and `build_wheel.cmake` print `[PASS] py-less, USD-free
  package contents verified` for `_install/` and the wheel staging tree, and the
  two pytest files pass (REQ AC-10, AC-11, AC-12).
- The negative packaging pass exits non-zero and names the violation -- `forbidden
  file: plugins/libov_25.11usd_ms.so`, `USD plugin registry shipped:
  plugins/usd`, or `missing codeless schema root registry:
  schemas/physx/plugInfo.json` -- rather than passing (REQ AC-12).

## Why the two-configuration pass is not automated

No CTest/CMake target runs the build-and-compare across both arms. Automating it
needs a second configure+build pass wired into `validate_all.cmake`, which
rebuilds `OvruntimePhysX` a second time under a different flag — real CI-time
cost for an arm the product does not ship. The single-arm half is cheap and is
covered: the default arm is what CI builds, and the payload, import and schema
checks run on every install and wheel build. The remaining gap is the
`OFF`-arm comparison (AC-1's control half, AC-2's `OFF` half, AC-7, AC-9), the
undefined-symbol comparison of AC-4, and AC-13's negative control, which by
construction is a build that must fail; it is run by hand on from-scratch Linux
builds with the exact commands above, and is recorded under
REQ-PACKAGING-NOUSD-001's Known limitations.
