<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-OMNICLIENT-001
maps_to: REQ-PACKAGING-OMNICLIENT-001
type: integration
---

## Coverage status

**Implemented 2026-09-08; automated with one downstream compatibility smoke.**

- **Automated:** SDK/wheel absence and fail-closed policy, plus process-isolated
  PhysX-first and OVStage-first local USD population, attach, and step coverage.
- **Downstream smoke:** An IsaacLab environment with OmniClient 2.72.3 loaded
  before the tested ovphysx wheel verifies that constructing `PhysX()` neither
  rejects nor replaces the host's client mappings.

## Scenario

ovphysx leaves OmniClient ownership to the application's OVStage package while
preserving USD population and physics startup in both load orders.

## Given

- A built and installed ovphysx SDK and wheel with their exact OVStage
  dependency available.
- A process-isolated run for each of the PhysX-first and OVStage-first startup
  orders.
- The local `tests/data/basic_simulation.usda` scene.

## When

- The SDK and wheel contents are inspected.
- The PhysX-first process records runtime mappings immediately before and after
  constructing `PhysX()`.
- Each process populates the local USD through OVStage, seals the write ordinal,
  attaches the Stage to PhysX, steps once, and waits for completion.

## Then

- No OmniClient library, connection library, or provenance file is present in
  either ovphysx artifact. (REQ AC-1)
- Constructing `PhysX()` adds no OmniClient or connection-library mapping and
  performs no version rejection. (REQ AC-2)
- Both startup orders populate, attach, and step successfully, and the final
  process contains one OVStage-owned mapping for each asset runtime library.
  (REQ AC-3)
