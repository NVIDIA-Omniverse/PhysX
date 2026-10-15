<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-PREF-COOK-001
title: Replace legacy cooking controls with supported cache configuration
status: proposed
owner: physics-ux
---

## Description

Replace legacy cooking controls with supported cache configuration. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Cache directory mapping.** Replace legacy Ujitso enable/max-process controls with Cooked Collider Cache Directory mapped to PhysXConfig.cooked_collider_cache_dir / OVPHYSX_GLOBAL_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY. Empty/unset means runtime-managed temporary cache; a nonempty directory requests persistence. Apply before first runtime bootstrap and mark later changes Requires Restart; legacy booleans/counts shall not be replayed via Carbonite overrides.
- AC-2: **Cache action scope.** Expose companion cooking result-cache invalidation only with its declared scope, preserving already-owned Cooked results. Runtime disk-cache purge and legacy releaseObjects are not implied; unavailable purge is explicitly labelled. Never delete a user cache directory as an emulation.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-PREF-COOK-001](../../tests/features/TEST-UX-PREF-COOK-001.md) — Configure cache location without restoring legacy cooking switches.

## Code References

- [configuration](../../../../../python/ovphysx_companion/configuration.py) — existing companion substrate; host integration remains proposed.
- [cooking](../../../../../python/ovphysx_companion/cooking.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.

- [Reviewed option mapping and API evidence](../../CONFIGURATION-MAPPING.md).
- [config.py](../../../../../../python/ovphysx/config.py) — PhysXConfig fields and typed mapping, current API reference.
- [ovphysx_types.h](../../../../../../include/ovphysx/ovphysx_types.h) — typed config enums and active_cuda_gpus contract, current API reference.
- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — global config, lifecycle and device constraints, current API reference.
- [ovphysx.cpp](../../../../../../src/ovphysx/ovphysx.cpp) — typed path mapping and blocked CUDA escape hatch, current API reference.
- [CarboniteLoader.cpp](../../../../../../src/CarboniteLoader/CarboniteLoader.cpp) — first-bootstrap cache configuration, current API reference.

## Dependencies

- [ADR-0004](../../adr/ADR-0004-physics-preferences.md).
- [REQ-UX-PREF-CONFIG-001](REQ-UX-PREF-CONFIG-001.md).
