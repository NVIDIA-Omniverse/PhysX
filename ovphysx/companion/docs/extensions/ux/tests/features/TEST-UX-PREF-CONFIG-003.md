<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PREF-CONFIG-003
maps_to: REQ-UX-PREF-CONFIG-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Apply typed threads and NVTX at the runtime-owned boundary. Verifies REQ-UX-PREF-CONFIG-001 AC-4, AC-5.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A fresh process with requested thread count 4 and NVTX enabled; retired error/dispatcher/profiler/names/empty-scene values; recorded typed configuration and create calls; a separate case where companion native authoring initialized shared services first.

## When

- Open UI, Play, inspect configuration order, request thread count 8 after bootstrap, reset and Stop/Play; exercise thread bounds 0/256 and invalid -1/257.

## Then

- num_threads=4 and profiler_backend="nvtx" use their typed keys before first creation; invalid thread values never commit. Old profiler does not silently set NVTX; other retired keys never reach the backend. (REQ-UX-PREF-CONFIG-001 AC-4).
- The shared configuration coordinator applies typed requests before the first native authoring, preview or Play bootstrap; later thread changes/reset remain conservatively restart-pending. Errors retain prior effective state and no extra handle or legacy settings call appears. If prior shared SDK initialization prevents application, the conflict is reported and effective settings are not falsely updated. (REQ-UX-PREF-CONFIG-001 AC-5).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-PREF-CONFIG-001](../../requirements/features/REQ-UX-PREF-CONFIG-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

