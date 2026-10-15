<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-HOST-EXIT-001
title: Preserve Host Exit Cleanup
status: implemented
owner: ovphysx
---

## Description

After a balanced Linux SDK lifecycle, ovphysx leaves normal process termination to
the host, including buffered-file flushing. Windows preserves its existing
Carbonite ExitProcess hook for safe dependency DLL teardown; hosts must flush
buffered files explicitly before exiting.
This guarantee applies when ovphysx owns the framework; a framework populated
by an external host remains that host's teardown responsibility. Independently
loaded libraries such as OVStage may install their own terminating exit handlers;
ovphysx does not control those handlers or promise implicit flushing through them.

## Acceptance Criteria

- AC-1: **Host exit semantics.** After creating and destroying an instance and calling
  `ovphysx_shutdown()`, normal C `exit()` preserves the requested exit code, invokes
  host `atexit` callbacks registered before instance creation, and on Linux flushes
  host file streams implicitly. Windows callers explicitly flush their streams.
- AC-3: **Stage dependency teardown.** After attaching and releasing a caller-owned
  OVStage instance, SDK shutdown and normal process exit preserve the requested
  exit code without crashing, including after repeated lifecycles. The host flushes
  its output explicitly and registers any required exit callback after dependency
  initialization.
- AC-2: **Repeatable lifecycle.** A subsequent initialize/create/destroy/shutdown cycle
  in the same process succeeds and preserves the same host-exit behavior.

## Test References

- [TEST-CAPI-HOST-EXIT-001](../../tests/capi/TEST-CAPI-HOST-EXIT-001.md)

## Code References

- ovphysx/src/CarboniteLoader/CarboniteLoader.cpp
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/tests/c_unittests/test_host_exit.cpp

## Dependencies

- [REQ-CAPI-CACHE-001](REQ-CAPI-CACHE-001.md)
