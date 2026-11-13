<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-OVSTAGE-001
title: "Explicit ovstage input and output flow"
status: implemented
owner: ovphysx
---

## Description

Ovstage is application-owned scene data. The host owns population, revisions and output destinations; companion services do not turn output into implicit simulation input.

## Acceptance Criteria

- AC-1: **Inputs.** Population and authored changes are consumed through an explicit input update at a sealed revision; the application retains stage lifetime through detach.
  Companion population adapters/examples populate ovphysx's physics population
  description with RENDERING beside it and seal before attach;
  `schemas.population_description(bundles)` includes core physics selectors, derived
  values and stage metadata together with plugin parameters in its single Desc; the seal composes
  world transforms, so physics-only hosts need no transform preparation. Direct
  world-matrix producers keep their authored matrices. After earlier instance work completes, a core-owner
  adapter resolves the Stage native code currently owns from the owner's
  committed and pending keepalives; uncollected attach or detach receipts do
  not make a stale committed Stage authoritative.
- AC-2: **Outputs.** A step/read does not implicitly author ovstage or USD; host-selected ovstage output uses the reserved `companion:output:` namespace
  and an output ordinal excluded from later input update processing. Publication
  rejects solver-input names, including custom-physics `companion:` parameters,
  before any write, and custom-physics schemas cannot declare `companion:output:`
  parameters, so later same-object reparsing cannot consume previously published
  output as input.
- AC-3: **Snapshots.** Snapshot authoring accepts copied data or a retained stable revision without acquiring or replacing a live simulation attachment.
- AC-4: **USD notices.** When Python also writes USD, the host tags or suppresses corresponding output notices independently of ovstage ordinal filtering so those writes do not feed back into solver input.

## Test References

- [TEST-COMP-OVSTAGE-001](../../tests/exchange/TEST-COMP-OVSTAGE-001.md)

## Code References

- [Host input/output adapter](../../../python/ovphysx_companion/exchange.py): StageFlow, OutputColumn and OutputNotices.
- [Schema input contract](../../../python/ovphysx_companion/schemas.py): `OUTPUT_NAMESPACE` and ExternalSchema parameter validation.
- [Native plugin parameter validation](../../../src/Native.cpp): `validParameter` rejects `companion:output:` inputs.
- [Core update endpoint](../../../../python/ovphysx/api.py) and [independent preview update](../../../python/ovphysx_companion/preview.py), reused without a second scheduler.
- [Customer-USD output transactions](../../../python/ovphysx_companion/usd/session.py) and [mesh writeback](../../../python/ovphysx_companion/usd/writeback.py).
- [Stageupdate sample](../../../samples/ovstage_flow.py).

## Ordering and ownership

StageFlow borrows the exact Stage retained by its supplied owner. For a core
owner it resolves native ownership after earlier work rather than trusting the
last collected lifecycle receipt. The host passes its already consumed ordinal,
serializes every writer, seals authored changes and calls consume. Runtime
changes may be realized at the next explicit host step; consume never steps or
fetches. Publication requires all sealed input to have been consumed. It writes
only the chosen numeric columns, seals a strictly newer output ordinal and moves
its cursor past that ordinal without forwarding it as input. Subsequent consume
calls begin strictly after that cursor.

OutputColumn owns an immutable CPU copy. Fixed rows and explicitly lane-typed
ragged arrays are supported; no implicit device transfer, transform conversion or
source-ID interning occurs. Host code chooses coordinate conventions and output
attributes inside the reserved `companion:output:` namespace. Input properties, including
physics mass, geometry, transforms and custom-physics parameters such as
`companion:stiffness`, are not valid output destinations. The whole `companion:`
namespace is not output-only: custom joint and geometry schemas use it for solver
inputs. Skipping the output ordinal does not hide a stored value from a later
reparse of the same prim. ExternalSchema rejects `companion:output:` parameters. Authoring jobs can use copied inputs while the live owner is idle
without acquiring/replacing its attachment. Output errors latch the adapter; no
ovstage rollback is promised. Repair the source/owner before creating a new flow.

Customer USD independently routes notices through OutputNotices.notice and uses
its suppress scope as EditSession.output_guard. Publication, rollback and undo/redo
therefore do not become solver-input notices. Native ovstage and customer USD run
in separate processes when they embed different USD images.

## Dependencies

- [REQ-COMP-IDENTITY-001](REQ-COMP-IDENTITY-001.md)
- [REQ-COMP-JOBS-001](REQ-COMP-JOBS-001.md)
- [Companion ADR-0002](../../adr/ADR-0002-ovstage-and-explicit-output.md)
