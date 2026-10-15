<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-AUTHOR-STAGE-001
maps_to: REQ-UX-AUTHOR-STAGE-001
type: integration
---

## Coverage status

Specification only — not implemented. Existing companion component tests verify the substrate, not this Kit scenario.

## Scenario

Associate two host stage generations, run delayed computation, replace/close the stage and drain only the old authoring/preview resources.

## Given

Stopped stage S1 with no physics scene, a late-registering companion provider and a host fixture that separately counts EditSession associations, native attachments and bootstrap. Add one physics scene before the playback subcase.

## When

Open S1, register the provider twice, refresh panels, then Play/Pause/Stop A. Explicitly activate Inspector B, step B and close it.

## Then

- Duplicate open/registration yields one EditSession generation. Opening panels and associating S1 create no native attachment; generic authored USD editing works with the runtime absent. (REQ AC-1).
- Only A receives timeline lifecycle calls; only explicit preview activation creates B. B uses a copied source and its own task queue. Closing B neither collects nor publishes A work; Stop preserves the authoring association. (REQ AC-3).

## Why not implemented

The migrated Kit feature and host integration fixture are not present. Component examples are integration starting points; they do not establish full mode, widget or command parity.

Mapped requirement: [REQ-UX-AUTHOR-STAGE-001](../../requirements/features/REQ-UX-AUTHOR-STAGE-001.md).
