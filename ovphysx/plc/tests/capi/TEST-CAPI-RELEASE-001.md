<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-RELEASE-001
maps_to: REQ-CAPI-RELEASE-001
type: unit
---

## Scenario
Query release distinguishes terminal failure from an unconfirmed attempt.
Group release retains the descriptor when runtime admission rejects cleanup.

## Given
The internal read accessors are injected with controlled release accessors.
A read session has one fetched group for the group-release case.

## When
- Query release reports failure, then success, then has no accessor.
- Group release rejects its first attempt, then succeeds on the next attempt,
  and is called once more after success.

## Then
- Query release returns `0`, `1`, and `-1`, respectively (REQ AC-1).
- Group release returns `-1`, `1`, and `1`. The backend is called twice: a
  rejected attempt must not discard the descriptor or make the next cleanup
  silently succeed without calling the backend (REQ AC-2).

## Code References
- `ovphysx/tests/internal_unittests/test_token_scope_sidecar.cpp`
  - `SidecarReleaseStatus.ReleaseQueryReportsReportedFailureAsTerminalZero`
  - `SidecarSessionIdentity.RejectedGroupReleasePreservesDescriptor`
