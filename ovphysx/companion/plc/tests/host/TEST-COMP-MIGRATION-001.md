<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-MIGRATION-001
maps_to: REQ-COMP-MIGRATION-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-MIGRATION-001; executable evidence and qualification limits follow.

## Scenario

Audit the consumer ledger and exercise migration fixtures in supported and deliberately unsupported configurations.

## Given

- The complete interface/extension coverage ledger, public capability catalog and a helper compatibility list.
- A legacy-provider fixture only if a same-process compatibility mode is advertised; otherwise an explicit unsupported-mode probe.

## When

- Resolve all ledger entries to requirements/tests; smoke supported helpers without starting simulation and request excluded interfaces/modes.

## Then

- All 28 directories and interface families have resolvable dispositions including no-consumer cases (REQ AC-1).
- Supported helpers preserve their stated semantics and incompatible calls fail with migration guidance without implicit attach (REQ AC-2).
- Excluded hooks are absent; any replication/output adapter has documented semantic evidence instead of name-only substitution (REQ AC-3).
- Advertised coexistence passes the owner/dependency/output checks; unsupported coexistence is explicit and process boundaries carry data only (REQ AC-4).

## Executable coverage

[test_migration.py](../../../tests/test_migration.py):

The 45-family/28-extension ledger resolves to REQ/TEST records. Whitelisted helpers resolve and author without importing the runtime. Unknown commands, private/test identities and same-process legacy coexistence raise explicit migration errors. docs/migration.md records cloning/output semantic differences.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
