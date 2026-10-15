<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-MASS-002
maps_to: REQ-COMP-MASS-001
type: integration
---

## Scenario

Compute analytical primitive and compound mass without attaching a scene; reject malformed inputs.

This is executed subset coverage, not the entire requirement. See its Implementation scope.

## Given

- A built/staged CPU SDK and the companion package in the locked Python environment.
- Customer-USD tests run in a separate process from native fixtures.

## When

- Execute the cases listed below through the sample guide test commands.

## Then

- Box/sphere/capsule results agree with analytical values without scene attachment (REQ AC-1).
- Translation, rotation, parallel-axis inertia and bounds agree with known results (REQ AC-2).
- Nonfinite, degenerate, extreme and invalid quaternion inputs identify the bad shape; failure leaves output unchanged (REQ AC-3).

## Executable coverage

- [tests_native/test_mass.py](../../../tests_native/test_mass.py).
