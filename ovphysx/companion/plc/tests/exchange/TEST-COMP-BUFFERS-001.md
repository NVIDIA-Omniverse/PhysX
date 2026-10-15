<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-BUFFERS-001
maps_to: REQ-COMP-BUFFERS-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU-copy surface. Covers [REQ-COMP-BUFFERS-001](../../requirements/exchange/REQ-COMP-BUFFERS-001.md) AC-1 through AC-4.

## Scenario

Retain two array views across the next update and release them in reverse order.

## Given

- A typed CPU result and, on supported GPU builds, an asynchronously produced device result with known values.
- Two Python views and a consumer that reads after the producer advances.

## When

- Advance the producer, drop the original result, read both views, then release them in reverse order; request CPU transfer of the device result.

## Then

- Metadata reconstructs the expected values and coordinate interpretation without guessing layout (REQ AC-1).
- Values remain valid until the last lease ends, then storage is released once (REQ AC-2).
- CPU reads occur only after completion and transfer; a device-only consumer incurs no mandatory host copy (REQ AC-3).
- Garbage collection of the original result and first view does not invalidate the remaining view (REQ AC-4).

## Executable coverage

- [Cooked result tests](../../../tests_native/test_cooking.py): copied immutable arrays, retained output after service close and cache invalidation.
- [Core read consumer](../../../tests_native/test_camera.py): copied completed output survives later native simulation.
- [Native event queue](../../../tests_native/test_events.py): copied callback history survives subscription and runtime-owner disposal.

No generic wrapper or companion device allocation/view capability is
advertised; core read sessions retain their existing DLPack/device completion
contract. Native-to-host copies are explicit at the producer boundary. Missing
GPU qualification is not counted as a passing test.
