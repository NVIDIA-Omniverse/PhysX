<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-OMNIPVD-LATE-001
maps_to: REQ-PYTHON-OMNIPVD-LATE-001
type: integration
---

## Scenario

Python rejects default late start, opts into sequential late FILE/TCP sessions, and preserves configured startup TCP
output.

## Given

- Canonical and invalid Python destinations.
- Separate fresh subprocesses for default and capability-only late-recording creation, plus startup-output TCP
  creation; Windows x86_64 and Linux x86_64/aarch64 execute the Python default/capability cases.
- A ready loopback TCP listener.

## When

- Destination factories validate their fields.
- FILE and TCP destination strings are marshalled through a synchronous native-call boundary that forces Python
  garbage collection before reading the borrowed `ctypes` fields.
- A default instance attempts late start, while an opted-in instance remains dormant until start.
- A directory-open failure is followed by an exact-file retry, rejected active retarget, stop, unrecorded step, and a
  second exact-file recording.
- A late TCP destination is marshalled through `PhysX.start_recording()`, sampled, stopped, and followed by an exact
  late FILE session on the same handle.
- A startup-output subprocess exercises only configured startup TCP streaming; native runtime/C coverage is
  authoritative for semantic OVD command parsing.
- A cold-startup FILE subprocess attaches and steps, verifies exclusive creator ownership against a peer, stops, and
  restarts to an exact late FILE destination; detach/reattach then restarts configured startup output under the
  reattaching peer, which stops it before another exact late FILE session.
- All Python recording and lifecycle calls execute sequentially on one test thread.

## Then

- Only canonical destination tuples are accepted (REQ AC-1).
- The config field and enum select the create-time capability and default to unset/false (REQ AC-2).
- On Windows x86_64 and Linux x86_64/aarch64, Python default late start reports `INVALID_STATE` and opted-in
  FILE/TCP recording succeeds (REQ AC-3).
- The three `PhysX` methods invoke the corresponding native operations in the late FILE/TCP paths, and both exact
  destination strings remain valid for the complete synchronous native call (REQ AC-4).
- Failure leaves recording inactive, retry succeeds, active retarget reports `INVALID_STATE`, and stop clears active
  state (REQ AC-5).
- The same handle restarts from late FILE to FILE and from late TCP to FILE after stop, and each completed exact FILE
  recording extends beyond the OVD header; the native C test's canonical reader is authoritative for semantic
  `CREATE_OBJECT` and teardown validation (REQ AC-6).
- Cold startup ownership is reserved for the creator, becomes active on lazy stage attach, and is observed after a
  peer attaches and steps; the peer remains inactive and cannot stop it, and the creator restarts to late FILE after
  stop. Startup-output reattach relatches ownership to the reattaching peer, which must stop that session before
  another late FILE start (REQ AC-7).
- Sequential single-thread execution covers the supported contract; no concurrent-call behavior is claimed (REQ
  AC-8).
