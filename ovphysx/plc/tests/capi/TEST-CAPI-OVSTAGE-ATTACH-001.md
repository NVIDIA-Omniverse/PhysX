<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-ATTACH-001
maps_to: REQ-CAPI-OVSTAGE-ATTACH-001
type: integration
---

## Scenario

Direct attachment fails closed when required articulation and joint schema data
is unsealed, while a seal that covers selected physics data remains sufficient.
`read_ordinal` 0 is rejected before the runtime attach.

## Given

- The two-articulation fixture is populated at ordinal 1.
- One Stage is unsealed; another leaves only an unrelated matched-root
  attribute outside an attribute-scoped seal.
- A caller-owned Stage is created for the zero-ordinal argument check.

## When

- `ovphysx_attach_ovstage()` is called on the unsealed Stage, then retried on
  the same Stage after sealing.
- `ovphysx_attach_ovstage()` is called on the attribute-scoped sealed Stage.
- `ovphysx_attach_ovstage()` is called with `read_ordinal == 0`.

## Then

- The unsealed attach returns `OVPHYSX_API_ERROR` with no partial attachment,
  and the sealed retry succeeds (REQ AC-1).
- The attribute-scoped sealed attach succeeds (REQ AC-2).
- The zero-ordinal attach returns `OVPHYSX_API_INVALID_ARGUMENT` and a later
  valid attach on the same handle still succeeds (REQ AC-3).
- `test_usd_loading.cpp`: `UnsealedAttachFailsAndSealedRetrySucceeds`,
  `ScopedSealAllowsUnrelatedUnsealedData`, and
  `AttachOvstageRejectsZeroReadOrdinal`.
