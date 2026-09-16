<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-USDFREE-001
maps_to: REQ-PACKAGING-USDFREE-001
type: integration
---

## Coverage status

**Landed 2026-09-01 (ADR-0027 Flip-B); partly automated.**

- **Automated:** AC-2, AC-4 and AC-7 — every `install.cmake` and
  `build_wheel.cmake` run executes `scripts/verify_pyless_closure.py
  --require-schemas` over the complete `_install/` and wheel staging trees and
  fails on any violation, so every successful install and wheel build is a
  passing run of the same checks. The verifier's own policy is covered offline by
  `tests/python_tests/test_verify_pyless_closure.py`
  (`test_verify_rejects_namespaced_usd_monolith_and_its_closure`,
  `test_verify_rejects_usd_plugin_registry_directory`,
  `test_verify_rejects_direct_usd_monolith_import`,
  `test_verify_rejects_upstream_modular_usd_library_names`,
  `test_verify_rejects_usd_runtime_config_file`,
  `test_verify_require_schemas_accepts_complete_codeless_tree`,
  `test_verify_require_schemas_rejects_nonfunctional_registry_and_module`,
  `test_verify_require_schemas_rejects_missing_root_registry_and_module_payload`),
  and the staging entry points by `tests/python_tests/test_package_deps.py`
  (`test_package_deps_stages_no_usd_payload`,
  `test_package_deps_exports_codeless_schemas_into_install_tree`).
- **Manual:** AC-1, AC-3, AC-5, AC-6 and AC-8 — a single-configuration
  `readelf`/grep pass run by hand at review (commands below); no CI step asserts
  them yet.

```bash
# from ovphysx/
grep -rn "OVPHYSX_NO_USD\|OVRUNTIME_PHYSX_NO_USD" CMakeLists.txt scripts ovruntime/CMakeLists.txt ovruntime/source   # AC-1: empty
readelf -d _install/lib/libovphysx.so _install/lib/libovphysx_internal.so | grep -i usd                          # AC-2: empty
readelf --dyn-syms -W _install/lib/libovphysx.so | c++filt | awk '$7=="UND"{print $8}' | grep -E "pxr|::Usd|::Sdf|::Tf|::Gf"   # AC-3: empty
find _install _dist -iname "*OvruntimePhysicsUsd*"                        # AC-5: empty
grep -rn "UsdVersionCheck\|preloadUsd\|detectUsd" src                    # AC-6: empty
grep -rln "#include <pxr/\|#include \"pxr/" _install/include              # AC-8: empty
ls _install/include/common/utilities/OmniPhysXUtilities.h _install/include/common/utilities/Utilities.h   # AC-8: both listed
```

## Scenario

The one shipping ovphysx build links no USD, stages no USD library, closure,
plugin registry or configuration, and stages neither the test-only USD parsing
library nor a pxr-including SDK header; the only USD in the process is the one
ovstage resolves for itself.

## Given

- A from-scratch build and install of the `ovphysx` CMake target (no
  `OVPHYSX_NO_USD` flag; the option does not exist).
- The built wheel and the `_install/` SDK tree.
- An ovruntime developer build (`OVRUNTIME_ENABLE_TESTS=ON`) that additionally
  produced the `OvruntimePhysicsUsd` static archive.

## When

- `readelf -d` and `readelf --dyn-syms` are run on the installed
  `libovphysx.so` and `libovphysx_internal.so`.
- `_install/` and the wheel are scanned for the AC-4 file patterns, a
  `plugins/usd` directory, a `config.toml`, and for `*OvruntimePhysicsUsd*`.
- The installed SDK `include/` tree is grepped for `#include <pxr/`.
- `scripts/install.cmake` / `scripts/build_wheel.cmake` are run, and the
  verifier pytest files are run offline.

## Then

- AC-1: The build produces the artifacts with no `OVPHYSX_NO_USD` flag anywhere
  in its configure.
- AC-2/AC-3: neither library lists a USD `NEEDED` or an undefined pxr symbol.
- AC-4: No USD library, closure library, `plugins/usd` directory or
  `config.toml` appears under `_install/` or in the wheel; the codeless
  `schemas/physx` tree is complete.
- AC-5: No `*OvruntimePhysicsUsd*` artifact appears under `_install/` or in the
  wheel.
- AC-6: `libovphysx` contains no USD preload, detection or version-check code
  path.
- AC-7: an install or wheel build whose staging tree violates AC-2 or AC-4, or
  lacks the schema tree, fails with a specific verifier error.
- AC-8: No installed SDK header includes `pxr/`; `OmniPhysXUtilities.h` and
  `Utilities.h` are installed and are themselves pxr-free.

## Notes

- Replaces TEST-PACKAGING-NOUSD-001, dropping its two-configuration build and its
  option-help-text checks; there is one build. The run-time monolith
  detection/version-check cases that earlier revisions of this TEST carried over
  are gone with the code: ovphysx no longer detects or version-checks USD at all.
