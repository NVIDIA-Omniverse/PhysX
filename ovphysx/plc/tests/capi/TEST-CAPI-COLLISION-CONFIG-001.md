<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-COLLISION-CONFIG-001
maps_to: REQ-CAPI-COLLISION-CONFIG-001
type: integration
---

## Scenario

`CollisionGeometryConfigTest.CreationEntriesAndRawSettingsSelectShapeGeometry`
in `tests/c_unittests/test_global_settings.cpp` checks actual PhysX geometry,
so a setting that only round-trips cannot satisfy AC-1 or AC-2.
Creation entries and the typed global setter share `applyConfigEntry`; the
creation checks exercise their common mapping and value conversion.

## Given

- A static cone and cylinder scene populated and sealed through ovstage.
- No attached stage; save both process-global geometry settings for restoration.

## When

- Before each creation, use the raw runtime settings to select the opposite
  geometry so retained values cannot satisfy the typed request.
- Create an instance with typed entries requesting a custom cone and an
  approximated cylinder, then repeat with the choices reversed.
- Attach the scene and inspect each `PxShape::getGeometry().getType()` and
  the corresponding typed getter.
- Detach, write the opposite choices through raw C entries for
  `/physics/collisionApproximateCones` and
  `/physics/collisionApproximateCylinders`, and reattach and inspect again.
- Run `test_typed_config_at_init` in
  `tests/python_tests/lifecycle_tests/test_typed_config_init.py` and
  `test_carbonite_overrides_conflicts_typed_path` in
  `tests/python_tests/cpu_tests/test_physxconfig_validation.py`.

## Then

- Typed true produces `eCONVEXCORE`; typed false produces `eCONVEXMESH`,
  independently for each shape. Typed getters report the requested values,
  including the Python creation path (REQ AC-1).
- Raw true produces `eCONVEXMESH` and typed false; raw false produces
  `eCONVEXCORE` and typed true (REQ AC-2).
- Python rejects either canonical raw path with `ValueError` identifying the
  overlapping typed field (REQ AC-3).
