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
recording sessions to FILE or TCP destinations. Creation and attachment
do not start a session and preserve an existing shared recording. An explicitly
started session belongs to its starting
handle; peer handles neither report nor stop that owner's session.

## Acceptance Criteria

- AC-1: The C API exposes the destination types, `OVPHYSX_API_INVALID_STATE`, the BOOL global key `OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE` (`0x0000000c`), and the canonical `ovphysx_ovd_start_recording`, `ovphysx_ovd_stop_recording`, and `ovphysx_ovd_is_recording` exports. Deprecated recording forwarding exports remain available in 0.7; retired startup config numeric slots remain reserved.
- AC-2: The scoped global setter selects process-wide recording capability before plain first-instance creation and defaults to false. It accepts only BOOL and rejects writes after first successful create until successful shutdown with no live instances, including after last-instance destruction; failed creates do not freeze the setter and configured values survive shutdown. The getter reports the configured bool. Explicit capability installs the provider/sampler and scene readback while idle without starting output, while the default null-provider path has no OmniPVD overhead.
- AC-3: On Windows x86_64 and Linux x86_64/aarch64, start on a default, incapable runtime returns `OVPHYSX_API_INVALID_STATE` and directs callers to set `OVD_RECORDING_CAPABLE` before the first instance, both before and after first attach.
- AC-4: Retired typed and raw OmniPVD startup configuration returns `OVPHYSX_API_INVALID_ARGUMENT` with migration guidance, including on second-live-instance creation. Legacy typed getters reject retired slots with the same result and guidance. An unconfigured peer may share the first runtime.
- AC-5: A successful start opens the selected destination, emits parseable OmniPVD object commands after stepping, and reports the one process-wide owning instance as active. FILE retains the existing `file_path` layout: a literal, case-sensitive `.ovd` suffix selects an exact filename; otherwise it selects a directory. Admitted explicit start recursively creates missing parents or the selected directory. Directory captures reserve distinct final `YYYY_MM_DD_HH_MM_SS_CC_rec.ovd` filenames exclusively and preserve prior files; exact filenames retain replacement behavior. There is no new transport or directory field, intermediate file, stop-time rename or import publication.
- AC-6: Invalid destinations and destination-open failures on an idle runtime leave recording inactive, so start may be retried. Rejected descriptor, capability, stage-readiness and active-takeover checks do not create the requested output paths; a filesystem-preparation failure may leave directories created during that attempt.
- AC-7: Start while any shared-runtime recording is active returns `OVPHYSX_API_INVALID_STATE` without replacing its destination; start while the shared runtime has no live physics stage, and stop on an inactive instance, also return `OVPHYSX_API_INVALID_STATE` with accurate diagnostics.
- AC-8: Stop finalizes the stream and permits another session to the same or a different FILE/TCP destination; destroying the instance that owns an active recording finalizes it with object-destruction commands.
- AC-9: **Detach and ownership.**
  - Detaching the recording owner's stage or the last live attachment attempts
    to finalize an active recording, including when a stageless peer handle
    started it. Successful finalization clears its process-wide owner.
  - A finalization failure is logged and does not by itself fail the detach task.
  - A non-owner detach preserves the recording while another attachment remains
    live. Detaching an unattached recording owner is a no-op and leaves its
    session active.
  - Reattach does not auto-start a session; a stale peer cannot query or stop the
    new owner's session.
- AC-10: Creation, attach, and reattach never start output or reserve recording ownership, even with inherited Carbonite output or destination settings. Those settings are inert: creation does not warn about startup recording or provision capability from them. `/physics/omniPvdRecordingCapable` stores the global capability key; an inherited value in the shared Carbonite registry is honoured and reported by the getter, although submitting that raw path through creation or global configuration APIs remains invalid. Starting explicitly after attach and before the first simulation step captures the initial world and first simulated frame without a warmup step.
- AC-11: Recording start/stop and recording-owner retirement reach a shared
  safe point after every instance's earlier jobs, preserving uncollected task
  outcomes. Recording-status queries read only recording state and do not wait
  for instance work. Pending companion previews reject recording transitions
  before teardown and remain fetchable. Callers serialize recording transitions.
- AC-12: Attach and detach while recording is active or starting execute on the
  caller after every instance reaches the shared safe point. They still return
  task receipts and preserve submission order; detach may finalize the recording.
  This is the synchronous exception to normal queued lifecycle execution.

## Test References

- [TEST-CAPI-OMNIPVD-LATE-001](../../tests/capi/TEST-CAPI-OMNIPVD-LATE-001.md) (AC-1 through AC-12)

## Code References

- ovphysx/include/ovphysx/ovphysx.h
- ovphysx/include/ovphysx/ovphysx_config.h
- ovphysx/include/ovphysx/ovphysx_legacy_types.h
- ovphysx/include/ovphysx/ovphysx_types.h
- ovphysx/docs/developer_guide.md
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/tests/c_unittests/test_c_api_compatibility.c
- ovphysx/tests/c_unittests/test_omnipvd_recording.cpp
- ovphysx/companion/tests_native/test_ovd.py (pending-preview detach rejection)

## Dependencies

- REQ-CAPI-CONFIG-001
- REQ-CAPI-OVD-NAMES-001
