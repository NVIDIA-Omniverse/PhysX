<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OMNIPVD-LATE-001
title: Sequential Late OmniPVD Recording
status: implemented
owner: ovphysx
---

## Description

An OvPhysX instance exposes synchronous controls for sequential late OmniPVD
recording sessions to exact FILE or TCP destinations. A startup session belongs
to the handle whose creation started it; peer handles neither report nor stop
that owner's session.

## Acceptance Criteria

- AC-1: The C ABI appends the destination types, `OVPHYSX_API_INVALID_STATE`, `OVPHYSX_CONFIG_OMNIPVD_RECORDING_CAPABLE`, and exactly `ovphysx_start_recording`, `ovphysx_stop_recording`, and `ovphysx_is_recording` without changing existing values.
- AC-2: The capability entry and helper select process-wide late-recording capability before the first instance is created and default to false; startup output implies capability. Explicit capability installs the provider/sampler and scene readback while idle, while the default null-provider path has no OmniPVD overhead.
- AC-3: On Windows x86_64 and Linux x86_64/aarch64, start on a default, incapable runtime returns `OVPHYSX_API_INVALID_STATE` and names `omnipvd_recording_capable` in the error both before and after first attach.
- AC-4: An explicit OmniPVD creation setting on a second live instance returns `OVPHYSX_API_ERROR`; an unconfigured peer may share the first runtime.
- AC-5: A successful start opens the exact destination, emits parseable OmniPVD object commands after stepping, and reports the one process-wide owning instance as active.
- AC-6: Invalid destinations and destination-open failures leave recording inactive, so start may be retried.
- AC-7: Start while any shared-runtime recording is active returns `OVPHYSX_API_INVALID_STATE` without replacing its destination; start while the shared runtime has no live physics stage, and stop on an inactive instance, also return `OVPHYSX_API_INVALID_STATE` with accurate diagnostics.
- AC-8: Stop finalizes the stream and permits another session to the same or a different FILE/TCP destination; destroying the instance that owns an active recording finalizes it with object-destruction commands.
- AC-9: Detaching the active stage finalizes the recording and clears its process-wide owner, including when a peer handle started it. Capability-only reattach remains dormant; startup-output reattach starts a new startup session owned by the reattaching handle, so a stale peer cannot query or stop it.
- AC-10: Cold startup ownership is reserved for the creating handle when instance creation succeeds and becomes active when lazy stage attach starts sampling; a peer reports inactive and cannot stop it. The owner can stop that session and start late output, while a later startup-output reattach relatches ownership to the reattaching handle.
- AC-11: The synchronous API follows the OvPhysX same-thread contract. Before start, stop, or recording-owner destruction mutates the shared runtime, it drains pending operations from both the calling handle and the current live-stage owner and revalidates that owner; an operation consumed by this internal safe point remains observable through `ovphysx_wait_op`. Callers serialize recording/attach/detach/destroy transitions across all handles; concurrent calls have no guarantee.

## Test References

- TEST-CAPI-OMNIPVD-LATE-001 (AC-1 through AC-11)

## Code References

- ovphysx/include/ovphysx/ovphysx.h
- ovphysx/include/ovphysx/ovphysx_config.h
- ovphysx/include/ovphysx/ovphysx_types.h
- ovphysx/docs/developer_guide.md
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/tests/c_unittests/test_c_api_compatibility.c
- ovphysx/tests/c_unittests/test_omnipvd_recording.cpp

## Dependencies

- REQ-OMNIPVD-LATE-001
