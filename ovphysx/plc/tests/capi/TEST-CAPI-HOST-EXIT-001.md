<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-HOST-EXIT-001
maps_to: REQ-CAPI-HOST-EXIT-001
type: integration
---

## Scenario

`tests/c_unittests/test_host_exit.cpp` verifies normal host termination after one
and three complete SDK lifecycles in fresh subprocesses, both with empty instances
and after attaching and releasing a USD stage.

## Given

- An ovphysx-only C API host registers an `atexit` callback before creating an instance.
- The host writes `payload\n` to a fully buffered file. Linux neither flushes nor
  closes it; Windows explicitly flushes before exit, matching its retained
  Carbonite process-exit behavior.
- Stage variants explicitly flush on both platforms and register their callback
  after the final lifecycle. OVStage has its own terminating handler, outside
  ovphysx's framework ownership; these variants test exit-crash prevention.

## When

- The host initializes the SDK, creates and destroys a CPU-only instance, and shuts down.
- The stage variants attach a minimal USD scene and destroy the attachment and
  caller-owned stage before destroying the instance.
- The host repeats the lifecycle for the three-cycle case, then calls `exit(23)`.

## Then

- The parent observes exit code 23 and the host callback marker (REQ AC-1).
- The file contains all eight payload bytes after process exit (REQ AC-1).
- The same assertions pass after three complete lifecycles (REQ AC-2).
- The stage variants also exit with code 23, including Windows DLL/static teardown;
  printing a successful test result before an exit crash is a failure (REQ AC-3).
