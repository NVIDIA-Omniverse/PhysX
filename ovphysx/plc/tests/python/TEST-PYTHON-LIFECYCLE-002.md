<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-LIFECYCLE-002
maps_to: REQ-PYTHON-LIFECYCLE-001
type: integration
---

## Scenario

Python interpreter exit applies the existing non-throwing finalizer policy
to every still-live `PhysX` instance before module and native-library
finalization.

## Given

- A Python subprocess with one or two successfully constructed `PhysX`
  instances retained until interpreter exit.
- At least one instance attaches a scene and waits for its pending work.
- No instance calls `destroy()` explicitly.
- A separate subprocess retains a `read()` Warp array (and its `PhysX`
  instance) until interpreter exit.
- A separate in-process lifecycle case constructs an `__eq__`-only unhashable
  subclass and two equal hashable subclasses.
- A separate lifecycle case drops the only strong reference to a live
  instance before interpreter exit.

## When

- The subprocess exits normally.
- A separate subprocess exits through an uncaught `KeyboardInterrupt`.
- The read-array subprocess exits with the array still globally reachable.
- The subclass case inspects the process-exit registry, then destroys.
- The mid-run lifecycle case forces garbage collection.

## Then

- The registered Python `atexit` callback invokes the finalizer policy once
  for every retained live instance. Tests register a witness *before* the
  first `PhysX()` so LIFO runs the library callback first; they do not
  unregister or replace that callback (REQ AC-8).
- A two-instance process discharges both lifecycle tokens and reaches final
  process shutdown without hanging or crashing (REQ AC-8).
- The read-array subprocess invokes native `ovphysx_shutdown()` before
  interpreter teardown, not only cache removal (REQ AC-8).
- Unhashable and equal-hashable subclasses are each tracked distinctly and
  can be constructed without `TypeError` (REQ AC-8).
- The normal-exit subprocess succeeds, while the `KeyboardInterrupt` subprocess
  preserves its non-successful interrupted exit after cleanup (REQ AC-8).
- The weak process-exit registry does not extend ordinary object lifetime:
  mid-run garbage collection still emits `ResourceWarning` and destroys the
  instance through its existing finalizer (REQ AC-8).
