<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-OMNIPVD-001
maps_to: REQ-PYTHON-OMNIPVD-001
type: integration
---

## Scenario

Python validates typed startup fields and streams to a ready TCP listener.

## Given

- Python config-enum parity checks and destination validation cases.
- A loopback listener bound to an ephemeral port before instance creation.

## When

- Validation tests construct accepted and rejected `PhysXConfig` values.
- A subprocess creates TCP-configured `PhysX`, attaches `simple_physics_scene.usda`, steps, and destroys it.

## Then

- Python fields and enum values match the C surface (REQ AC-1).
- Wrong types and invalid destination values raise before native startup (REQ AC-2).
- The bounded listener completes without error and receives more than eight bytes (REQ AC-3).
