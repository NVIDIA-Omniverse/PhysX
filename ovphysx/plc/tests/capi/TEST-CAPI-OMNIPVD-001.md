<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OMNIPVD-001
maps_to: REQ-CAPI-OMNIPVD-001
type: integration
---

## Scenario

Typed startup config preserves FILE defaults and rejects invalid TCP state.

## Given

- Existing typed config values and FILE recording tests.
- Valid and invalid typed and raw OmniPVD destination entries, including embedded NULs in submitted and
  preloaded Carbonite strings.

## When

- Config enums are compared with Python parity values and their named builders are compiled and used.
- Instances are created with default FILE, invalid TCP, and live-runtime mutation cases.

## Then

- Existing key ordinals remain stable and all four appended fields are available (REQ AC-1).
- Invalid tuples, exact-length strings containing embedded NULs, and live-runtime changes return their prescribed
  errors (REQ AC-2).
- Existing FILE tests still produce non-empty timestamped recordings without selecting a transport (REQ AC-3).
