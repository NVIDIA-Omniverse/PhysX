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
recording sessions to exact FILE or TCP destinations. Creation and attachment
leave recording dormant. An explicitly started session belongs to its starting
handle; peer handles neither report nor stop that owner's session.

## Acceptance Criteria

- AC-1: The C API exposes the destination types, `OVPHYSX_API_INVALID_STATE`, the BOOL global key `OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE` (`0x0000000c`), and the canonical `ovphysx_ovd_start_recording`, `ovphysx_ovd_stop_recording`, and `ovphysx_ovd_is_recording` exports. Deprecated recording forwarding exports remain available in 0.7; retired startup config numeric slots remain reserved.
- AC-2: The scoped global setter selects process-wide recording capability before plain first-instance creation and defaults to false. It accepts only BOOL and rejects writes after first successful create until successful shutdown with no live instances, including after last-instance destruction; failed creates do not freeze the setter and configured values survive shutdown. The getter reports the configured bool. Explicit capability installs the provider/sampler and scene readback while idle without starting output, while the default null-provider path has no OmniPVD overhead.
- AC-3: On Windows x86_64 and Linux x86_64/aarch64, start on a default, incapable runtime returns `OVPHYSX_API_INVALID_STATE` and directs callers to set `OVD_RECORDING_CAPABLE` before the first instance, both before and after first attach.
- AC-4: Retired typed and raw OmniPVD startup configuration returns `OVPHYSX_API_INVALID_ARGUMENT` with migration guidance, including on second-live-instance creation. Legacy typed getters reject retired slots with the same result and guidance. An unconfigured peer may share the first runtime.
- AC-5: A successful start opens the exact destination, emits parseable OmniPVD object commands after stepping, and reports the one process-wide owning instance as active.
- AC-6: Invalid destinations and destination-open failures leave recording inactive, so start may be retried.
- AC-7: Start while any shared-runtime recording is active returns `OVPHYSX_API_INVALID_STATE` without replacing its destination; start while the shared runtime has no live physics stage, and stop on an inactive instance, also return `OVPHYSX_API_INVALID_STATE` with accurate diagnostics.
- AC-8: Stop finalizes the stream and permits another session to the same or a different FILE/TCP destination; destroying the instance that owns an active recording finalizes it with object-destruction commands.
- AC-9: Detaching the active stage finalizes the recording and clears its process-wide owner, including when a peer handle started it. Reattach remains dormant until explicit start; a stale peer cannot query or stop the new owner's session.
- AC-10: Creation, attach, and reattach never start standalone output or reserve recording ownership, even with inherited startup-output settings. First-live-instance creation clears inherited output-enabled=true with a warning directing callers to global capability and explicit recording; it does not provision capability. Starting explicitly after attach and before the first simulation step captures the initial world and first simulated frame without a warmup step.
- AC-11: The synchronous API follows the OvPhysX same-thread contract. Before start, stop, status query, or recording-owner destruction mutates the shared runtime, it drains pending operations from both the calling handle and the current live-stage owner and revalidates that owner; a task completed by this internal safe point is not collected by it and remains observable through `ovphysx_wait_task` / `ovphysx_wait_all`. Callers serialize recording/attach/detach/destroy transitions across all handles; concurrent calls have no guarantee.

## Test References

- TEST-CAPI-OMNIPVD-LATE-001 (AC-1 through AC-11)

## Code References

- ovphysx/include/ovphysx/ovphysx.h
- ovphysx/include/ovphysx/ovphysx_config.h
- ovphysx/include/ovphysx/ovphysx_legacy_types.h
- ovphysx/include/ovphysx/ovphysx_types.h
- ovphysx/docs/developer_guide.md
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/tests/c_unittests/test_c_api_compatibility.c
- ovphysx/tests/c_unittests/test_omnipvd_recording.cpp

## Dependencies

- REQ-OMNIPVD-LATE-001
- REQ-CAPI-CONFIG-001
