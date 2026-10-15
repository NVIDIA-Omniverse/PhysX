<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-MIGRATION-001
title: "Consumer coverage and explicit compatibility limits"
status: implemented
owner: ovphysx
---

## Description

Maintain an explicit disposition for every inventoried UX extension and interface family: core reuse, companion service, Python USD utility, host-only integration or deliberate non-export.

## Acceptance Criteria

- AC-1: **Coverage ledger.** The companion coverage ledger maps all 28 inventoried UX directories and every interface family to a disposition and requirement/test reference; no-consumer and identity-only headers are labelled as such.
- AC-2: **Utility compatibility.** Migration records supported Python helper/command names and behavior, providing adapters or explicit migration errors; schema authoring utilities do not require simulation initialization.
- AC-3: **Non-export.** Unit-test/benchmark interfaces, vehicle test hooks and empty graph/GPU plugin identities do not become production APIs; cloning/replication and Fabric output use core/host contracts only after semantic comparison.
- AC-4: **Legacy isolation.** Legacy/native coexistence is not assumed: any supported same-process mode verifies owner, SDK, CUDA, settings, schemas and output writers; otherwise the host uses process isolation and never exchanges SDK pointers across owners.

## Test References

- [TEST-COMP-MIGRATION-001](../../tests/host/TEST-COMP-MIGRATION-001.md) - executed migration and ledger checks.

## Code References

Supported adapters and their executable checks are listed below; lower-level anchors retain their own PLC ownership.

- [ovphysx/ovruntime/include/omni/physx/PhysXRuntime.h](../../../../ovruntime/include/omni/physx/PhysXRuntime.h)
- [ovphysx/python/ovphysx/api.py](../../../../python/ovphysx/api.py)
- [ovphysx/python/ovphysx/utils/simulation.py](../../../../python/ovphysx/utils/simulation.py)

- [migration.py](../../../python/ovphysx_companion/migration.py)
- [migration.md](../../../docs/migration.md)
- [test_migration.py](../../../tests/test_migration.py)
- [coverage.md](../../../plc/coverage.md)

## Dependencies

- [REQ-COMP-API-001](../architecture/REQ-COMP-API-001.md)
- [REQ-COMP-HOST-001](REQ-COMP-HOST-001.md)
- [REQ-COMP-PREVIEW-001](../live/REQ-COMP-PREVIEW-001.md)
- [Companion ADR-0006](../../adr/ADR-0006-host-services-and-ux-migration.md)

## Implementation scope

The 45-family/28-extension ledger is checked against linked contracts. The migration
guide maps helpers and host commands to implemented services. Supported legacy
physicsUtils names delegate to core authoring without runtime initialization;
unknown helper names return focused MigrationError diagnostics. Private/test
interfaces are not exported; legacy coexistence policy belongs in the migration guide. Authored custom clones are qualified; binary
callback replication is not. StageFlow has explicit data/output semantics rather
than an implicit Fabric scheduler substitution.
