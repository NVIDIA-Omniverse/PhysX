<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-THREADING-001
maps_to: REQ-CAPI-THREADING-001
type: integration
---

## Scenario

Create-time worker-pool overrides take effect before process-wide pool startup,
and ovphysx imposes no worker count of its own.

## Given

Each C unit test case runs in its own gtest pass, so it gets a fresh process
where no earlier case has started Carbonite tasking.

## When

One case creates an instance without worker overrides. A second case creates an
instance with distinct explicit Carbonite tasking and TBB limits, then attempts
global config writes and additional instance creation with both slash-prefixed
and slashless worker paths. A third case attempts invalid tasking and TBB
counts before retrying with a default construction.

## Then

In the unconfigured case both worker settings still read zero, meaning ovphysx
wrote no default and left the pools sized to the machine, and the live
`ITasking` descriptor reports at least one worker. In the override case that
descriptor reports exactly the requested count and TBB's effective
`max_allowed_parallelism` matches its requested value. Later overrides of
either pool and either path spelling return `OVPHYSX_API_ERROR` and leave the
running pool sizes unchanged. Global worker-pool writes take
`g_createInstanceMutex` before the started-pool check and hold it through
apply, so they cannot race a concurrent create into a successful no-op.
Negative tasking counts and non-positive explicit TBB limits return
`OVPHYSX_API_INVALID_ARGUMENT` without changing ISettings, and a subsequent
default create succeeds in that same process.
