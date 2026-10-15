<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ASYNC-002
maps_to: REQ-CAPI-ASYNC-002
type: integration
---

## Coverage status

Session cases are automated. Articulation lazy-attach failure diagnostics are
checked by source review; there is no automated attach-failure fixture.

## Scenario

Session operations return collectable tasks without weakening ownership,
iteration, simulation ordering or error reporting.

## Given

- A stepped CPU scene with rigid bodies and an owning instance.
- A second live instance, rigid-body and whole-articulation queries, read/write
  sessions and real unsupported write requests.

## When

- Submit and wait every session operation and the query-scoped articulation update.
- Attempt the articulation update with a wrong-type query and unsupported flags.
- Fetch to exhaustion and inspect the output only after waiting.
- Attempt null-output and wrong-instance calls, then reuse the original owner.
- Leave an accepted write failure uncollected while waiting on later success.
- Step between opening a session and reading or committing it.
- Release a read and then its query while an owning-instance step is pending.
- Hold a peer runtime callback on another thread while driving the idle owner's
  query, cached-query validation, write and read sessions to completion.
- From an application log callback, attempt cached-query validation and a read
  on an idle instance. Before returning from that callback, repeat both calls
  on a separate client thread with the tensor backend already initialized.

## Then

- Every accepted operation has a nonzero owned task, including EOF and release;
  rejected calls have zero identity and cleared outputs (REQ AC-1, AC-2, AC-3).
- Exhaustion completes successfully with a null group (REQ AC-4).
- Foreign-instance calls leave the original resources usable; repeated absent
  releases succeed (REQ AC-5).
- Later success leaves the earlier execution error for wait-all; mid-session
  operations respect prior simulation and spent commits fail (REQ AC-3, AC-6).
- Read and query releases succeed after pending owner simulation; both step
  receipts remain collectable (REQ AC-3, AC-5, AC-6).
- Every output operation completes before the peer's held callback returns;
  committed values read back on the selected owner, while output reentry from
  the callback thread is refused (REQ AC-6).
- Log-callback output reentry returns `INVALID_STATE`, clears the query-valid
  flag and read handle, and creates no task. Both calls succeed on the separate
  client thread before the callback returns (REQ AC-2, AC-6).
- Read tensor descriptors remain usable after group release and until session
  release (REQ AC-7).
- Unsupported flags and wrong-type queries are rejected before admission; a
  valid articulation query updates CPU link state
  without a simulation step (REQ AC-8).

## Automated coverage

- `ovphysx/tests/c_unittests/test_async_api.cpp`: the cases above.
- `ovphysx/tests/c_unittests/test_output_read.cpp` and `test_write_api.cpp`:
  CPU/GPU data paths, snapshots, CUDA handoff and write-group lifetime.
- `ovphysx/tests/c_unittests/test_physx_async.cpp`: gated real simulation,
  partial timeout, retry and unrelated task collection.
- `ovphysx/tests/c_unittests/test_multi_instance.cpp`:
  `MultiInstance.OutputCompletesWhilePeerRuntimeCallbackIsBlocked` and
  `PhysXTestFixture.OutputReentryFromLogCallbackIsRejected`. The log-callback case
  uses the real log dispatcher without injecting a CUDA failure or reproducing
  the backend mutex deadlock.
- `ovphysx/tests/python_tests/lifecycle_tests/test_multi_instance_output.py`:
  300 synchronous write/read round trips during peer stepping, plus a serialized
  control, with committed-value and peer-isolation checks.

## Coverage limits

For articulation lazy-attach failure, verify that the accepted task retains a
captured runtime diagnostic, or an attach-specific fallback if none was captured
(REQ AC-3). The failing attach path is not exercised by the integration suite.

Allocation failure and CUDA driver cleanup failure are checked by source review;
the installed library has no fault-injection switches. GPU cases require a
working CUDA device and report skips explicitly when it is unavailable.
