<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-OMNIPVD-LATE-001
maps_to: REQ-PYTHON-OMNIPVD-LATE-001
type: integration
---

## Scenario

Python rejects default start and uses scoped capability for explicit sequential
FILE/TCP sessions, including capture before the first simulation step.

## Given

- Canonical and invalid Python destinations, including inferred directories and
  exact filenames with missing parents.
- Separate fresh subprocesses for default and capability-enabled creation;
  Windows x86_64 and Linux x86_64/aarch64 execute these cases.
- A ready loopback TCP listener.
- Scoped capability setup before plain `PhysX()`.

## When

- Destination factories validate their fields.
- FILE and TCP destination strings are marshalled through a synchronous native-call boundary that forces Python
  garbage collection before reading the borrowed `ctypes` fields.
- A default instance attempts late start, while an opted-in instance remains dormant until start.
- The module-level capability setter rejects a wrong type and a write with a
  live object. After the last Python owner destroys its instance and shuts
  down the native lifecycle, it accepts a new write and retains the prior
  pending value until replaced.
- Pass the global capability key to the instance setter in a fresh subprocess,
  then read capability and instance GPU selection and query recording status.
- An existing `.ovd` directory fails exact-file open and is followed by an exact-file retry, rejected active retarget, stop, unrecorded step, and a
  second exact-file recording.
- Forward a directory path unchanged as FILE, explicitly start after attach,
  stop and restart into it, then compare the first capture's bytes.
- Forward `runs/capture.ovd/` unchanged through the native-call string lifetime
  probe. In a subprocess with a temporary cwd, start bare `capture.ovd`.
- A late TCP destination is marshalled through `PhysX.start_recording()`, sampled, stopped, and followed by an exact
  late FILE session on the same handle.
- TCP subprocesses explicitly start before or after the first simulation step;
  native runtime/C coverage is authoritative for semantic OVD command parsing.
- A fresh FILE subprocess verifies dormant creation/attach, explicitly starts,
  checks exclusive recording ownership against a peer, stops, and restarts.
  Detach/reattach stays dormant before another explicit start under the peer.
- All Python recording and lifecycle calls execute sequentially on one test thread.

## Then

- Only canonical destination tuples are accepted; directory paths, including
  non-ASCII strings and the `.ovd/` directory escape, remain FILE tuples and
  are forwarded without a new factory or transport (REQ AC-1).
- `GlobalConfigKey.OVD_RECORDING_CAPABLE` is `0x0000000c`, defaults false,
  uses BOOL, and provisions dormant capability through plain `PhysX()`.
  Rejected writes leave it unchanged; cross-scope instance writes raise
  `ValueError` without changing capability or `ACTIVE_CUDA_GPUS`, and recording
  remains dormant. After first successful create,
  writes remain rejected until successful shutdown with no live instances,
  which retains configured values. Removed startup fields and capability
  constructor arguments are rejected (REQ AC-2).
- On Windows x86_64 and Linux x86_64/aarch64, Python default start reports `INVALID_STATE`,
  directs callers to set `OVD_RECORDING_CAPABLE` before the first instance, and opted-in
  FILE/TCP recording succeeds (REQ AC-3).
- The three `PhysX` methods invoke the corresponding native operations in the late FILE/TCP paths, and
  destination strings remain valid for the complete synchronous native call (REQ AC-4).
- Failure leaves recording inactive, retry succeeds, active retarget reports `INVALID_STATE`, and stop clears active
  state (REQ AC-5).
- The same handle restarts from late FILE to FILE and from late TCP to FILE after stop, and each completed exact FILE
  recording extends beyond the OVD header; the native C test's canonical reader is authoritative for semantic
  `CREATE_OBJECT` and teardown validation (REQ AC-6).
- Missing parents of an exact `.ovd` filename are created at admitted start.
  A bare exact filename writes beyond the header in the caller's cwd;
  canonical C reader coverage remains authoritative for objects and frames.
  Inferred directories stay absent before attach/start and during rejected
  active retarget, then contain distinct timestamped captures after restart.
  Earlier bytes are preserved, and no `tmp.ovd` is used (REQ AC-5, AC-6, AC-7).
- Create, attach, and reattach do not auto-start a session. Explicit start before any step
  captures the first simulated frame. Only the starting object reports active
  and can stop its session; after detach and dormant reattach, a peer may own a
  new explicitly started session (REQ AC-7).
- Sequential single-thread execution covers the supported contract; no concurrent-call behavior is claimed (REQ
  AC-8).

## Executable coverage

`tests/python_tests/test_omnipvd_recording.py`:

- `test_explicit_recording_owner_and_reattach_stay_dormant`
- `test_tcp_streams_to_ready_listener` (before-first-step and after-first-step)
- `TestOvdRecording.test_ovd_file_produced_before_first_step`
- `TestOvdRecording.test_ovd_not_produced_until_explicit_start`
- `TestOvdRecording.test_ovd_exact_filename_creates_missing_parents_at_start`
- `test_file_path_infers_directory_only_after_explicit_start_and_preserves_captures`
- `test_late_file_recording_retries_rejects_takeover_and_restarts`
- `test_start_recording_keeps_destination_strings_alive_through_native_call`
- `test_bare_exact_filename_records_in_current_directory`
- `test_retired_ovd_carbonite_overrides_rejected_without_freezing_capability`
- `test_global_ovd_capability_rejects_instance_scope_without_mutation`

`tests/python_tests/lifecycle_tests/test_config_global.py` covers scoped
capability type, retention, and lifecycle boundaries.

`tests/python_tests/cpu_tests/test_physxconfig_validation.py` covers canonical
destination validation (`test_ovd_destination_*` and `test_ovd_tcp_destination_*`),
with no native initialization.
