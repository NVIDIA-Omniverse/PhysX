<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-COLLISION-CONFIG-001
title: Typed Cone and Cylinder Configuration Selects Collision Geometry
status: implemented
owner: ovphysx
---

## Description

The cone and cylinder custom-geometry controls address the process-global
settings used when PhysX creates their collision shapes. This baseline comes
from NVBugs 6827231. Configure these controls before stage attachment;
changing a setting does not rebuild existing shapes.

## Acceptance Criteria

- AC-1: **Typed geometry selection.** The cone and cylinder
  `OVPHYSX_CONFIG_COLLISION_*_CUSTOM_GEOMETRY` keys select `eCONVEXCORE` when
  true (the runtime default) and `eCONVEXMESH` when false. Creation entries and
  `ovphysx_set_global_config()` apply that choice before attachment, and
  `ovphysx_get_global_config_bool()` reports the public custom-geometry value.
  The corresponding `PhysXConfig` fields use the same values.
- AC-2: **Runtime polarity.** The typed keys address
  `/physics/collisionApproximateCones` and
  `/physics/collisionApproximateCylinders` with inverse polarity. A raw
  Carbonite C entry retains the runtime polarity: true requests a convex mesh
  and the typed getter reports false.
- AC-3: **Python path validation.** `PhysXConfig.carbonite_overrides` rejects
  either canonical approximation path as overlapping its typed field.

## Test References

- [TEST-CAPI-COLLISION-CONFIG-001](../../tests/capi/TEST-CAPI-COLLISION-CONFIG-001.md)

## Code References

- `ovphysx/src/ovphysx/ovphysx.cpp`: `s_boolKeyPaths`, `applyConfigEntry`,
  `ovphysx_get_global_config_bool` (AC-1, AC-2).
- `ovphysx/include/ovphysx/ovphysx_config.h`: cone/cylinder entry builders;
  `ovphysx/include/ovphysx/ovphysx_types.h`: typed keys (AC-1).
- `ovphysx/python/ovphysx/config.py`: typed fields, paths and conflict validation
  (AC-1, AC-3).

## Dependencies

None.
