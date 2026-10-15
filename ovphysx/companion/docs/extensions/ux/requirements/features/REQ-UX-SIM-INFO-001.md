<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-SIM-INFO-001
title: Retain simulation information using explicit owner diagnostics
status: proposed
owner: physics-ux
---

## Description

Retain simulation information using explicit owner diagnostics. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Identity.** The information view shall label selected engine, host stage, native owner and playback/preview role, actual pipeline and output destination. It shall not infer GPU state or Fabric activity from old preference booleans.
- AC-2: **Counters.** Use completed companion statistics and supported inspection counters with their actual units and scope. Unsupported legacy metrics are unavailable, not zero; opening the panel creates no simulation or polling step.
- AC-3: **Output controls.** Present ovstage output and explicitly selected Python preview/bake destinations. Do not reactivate a legacy Fabric driver or USD writer when switching the view. Stop/detach clears owner data and leaves unrelated preview/recording views alone.

## Test References

Specification only — not implemented.

- [TEST-UX-SIM-INFO-001](../../tests/features/TEST-UX-SIM-INFO-001.md)

## Code References

- [native](../../../../../python/ovphysx_companion/native.py) — existing companion substrate; host integration remains proposed.
- [inspection](../../../../../python/ovphysx_companion/inspection.py) — existing companion substrate; host integration remains proposed.
- [configuration](../../../../../python/ovphysx_companion/configuration.py) — existing companion substrate; host integration remains proposed.
- [exchange](../../../../../python/ovphysx_companion/exchange.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0008-debug-window](../../adr/ADR-0008-debug-window.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
