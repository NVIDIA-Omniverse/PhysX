<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-USD-EDIT-001
title: "Explicit USD edit transactions"
status: implemented
owner: ovphysx
---

## Description

Python USD result-application helpers are independent of Kit. They validate the full result before applying edits to an explicit caller-owned stage, target layer and time policy.

## Acceptance Criteria

- AC-1: **Targets.** Each mutating helper takes an explicit stage and edit target and declares default-value versus time-sample policy; it never selects another stage or saves implicitly.
- AC-2: **Preflight.** Before mutation, helpers validate identity/revision/topology, destination paths, schema availability, writable edit target and array/layout consistency.
- AC-3: **Failure behavior.** Application uses a reversible batch: validation failure leaves the stage unchanged and an application exception rolls back helper-owned edits or returns an explicit recoverable patch with no success claim.
- AC-4: **Host policy.** USD edits execute on the application-approved thread; Kit wraps the same batch in undo/redo, while standalone callers choose persistence and preview-layer lifetime.

## Implementation scope

EditSession validates explicit local layers, source revisions, approved thread and
non-reentrant batches. Helpers preflight before mutation; exceptions roll back and
RecoveryError exposes recoverable layer text if restoration fails. Patches preserve
explicit stage-to-layer time mapping and reject conflicting undo/redo. Whole-layer
patches require host serialization. Variant/mapped edit targets are rejected;
property-granular patches and Kit registration are not advertised capabilities.

## Test References

- [TEST-COMP-USD-EDIT-002](../../tests/python/TEST-COMP-USD-EDIT-002.md) - executed Python subset; see its scenario and limitations.
- [TEST-COMP-USD-EDIT-001](../../tests/python/TEST-COMP-USD-EDIT-001.md) - executed supported-contract scenario.

## Code References

Companion implementation:

- [usd/session.py](../../../python/ovphysx_companion/usd/session.py)
- [usd/writeback.py](../../../python/ovphysx_companion/usd/writeback.py)
- [usd/attachments.py](../../../python/ovphysx_companion/usd/attachments.py)
- [Executable subset tests](../../../tests/test_usd_helpers.py)

Existing integration anchors; these do not establish native implementation:

- [ovphysx/python/ovphysx/utils/deformable.py](../../../../python/ovphysx/utils/deformable.py)
- [ovphysx/python/ovphysx/utils/codeless.py](../../../../python/ovphysx/utils/codeless.py)

## Dependencies

- [REQ-COMP-PYTHON-001](REQ-COMP-PYTHON-001.md)
- [REQ-COMP-IDENTITY-001](../exchange/REQ-COMP-IDENTITY-001.md)
- [Companion ADR-0004](../../adr/ADR-0004-customer-usd-python.md)
