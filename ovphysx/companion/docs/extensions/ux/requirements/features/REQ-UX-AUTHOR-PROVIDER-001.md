<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-AUTHOR-PROVIDER-001
title: Use the companion as the supported authoring provider
status: proposed
owner: physics-ux
---

## Description

Use companion Python services and versioned native tables from the loaded ovphysx owner for authoring. Customer-installed USD is optional and accessed only by Python helpers. No legacy-first probe or separately linked ovruntime provider is part of this design.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Supported provider.** The selected PhysX tooling provider shall use the optional companion Python package and versioned native service tables of the loaded ovphysx owner. ABI/size mismatch and absent operation capability shall fail before resource acquisition; no private wheel internals, legacy authoring SDK or second runtime image shall be loaded.
- AC-2: **USD boundary.** Native computation shall receive copied source-neutral data and remain USD-free. Python authoring shall use customer USD on the approved host thread. If population needs an incompatible USD image, a worker shall exchange copied values/paths/revisions only; SDK pointers and USD objects shall not cross.
- AC-3: **Selection and bootstrap.** One configured provider shall own each contribution; failure shall not hot-fallback to legacy omni.physx. Apply initialization-only policy before the first native authoring or playback bootstrap, report conflicts and drain owned handles before unloading modules.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-AUTHOR-PROVIDER-001](../../tests/features/TEST-UX-AUTHOR-PROVIDER-001.md) — Negotiate companion tables and shared-owner bootstrap before registration.
- [TEST-UX-AUTHOR-PROVIDER-002](../../tests/features/TEST-UX-AUTHOR-PROVIDER-002.md) — Keep customer USD in Python and isolate incompatible population in a copied-data worker.

## Code References

- [compatibility](../../../../../python/ovphysx_companion/compatibility.py) — existing companion substrate; host integration remains proposed.
- [migration](../../../../../python/ovphysx_companion/migration.py) — existing companion substrate; host integration remains proposed.
- [native](../../../../../python/ovphysx_companion/native.py) — existing companion substrate; host integration remains proposed.
- [plugins](../../../../../python/ovphysx_companion/plugins.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.

- [Current ovruntime build](../../../../../../ovruntime/CMakeLists.txt) — shared runtime implementation; the companion must use this loaded owner rather than link a second image.

## Dependencies

- [ADR-0027](../../adr/ADR-0027-authoring-runtime-provider.md).
- [REQ-UX-AUTHOR-STAGE-001](REQ-UX-AUTHOR-STAGE-001.md).
