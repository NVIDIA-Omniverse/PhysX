<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CONFIG-002
maps_to: REQ-CAPI-CONFIG-001
type: integration
---

## Scenario

Runtime contact-processing configuration affects the already attached scene.

## Given

- An attached CPU scene whose settled contact sensors report nonzero vertical
  net normal forces through the session read API.
- A submitted step whose receipt has not yet been collected.

## When

- Disable contact processing through the scoped global setter.
- Collect the pending step, complete another simulation step, and read net normal
  forces and raw contacts.
- Re-enable contact processing through the legacy configuration-entry setter,
  complete another step, and read net normal forces again.

## Then

- Both setters succeed without replacing the attachment, and the pending step
  receipt remains collectible after the scoped setter drains it (REQ AC-4, AC-7).
- While disabled, vertical net normal forces are approximately zero and no raw
  contact-force array groups are returned (REQ AC-4).
- Re-enabling restores nonzero vertical net normal forces (REQ AC-4, AC-7).

Implemented by `ovphysx/tests/python_tests/cpu_tests/test_contact_query_cpu.py`,
`test_cpu_disable_contact_processing_zeroes_the_contact_read`.
