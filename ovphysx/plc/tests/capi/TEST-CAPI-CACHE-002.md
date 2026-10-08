<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CACHE-002
maps_to: REQ-CAPI-CACHE-001
type: integration
---

## Scenario

Python process exit destroys a still-live `PhysX` instance before
interpreter finalization and removes its process-private cooked-collider
cache through the normal last-loader shutdown path.

## Given

- A Python subprocess with `cooked_collider_cache_dir` unset and its OS
  temp environment redirected to a unique empty directory.
- A scene whose mesh colliders require cooking.
- The `PhysX` instance is retained until process exit so ordinary
  reference-counted garbage collection cannot destroy it first.

## When

- The subprocess attaches the scene, waits for cooking, and exits without
  calling `PhysX.destroy()`.
- The scenario is run once by normal return and once through an uncaught
  `KeyboardInterrupt`.

## Then

- The Python `atexit` callback runs before interpreter finalization and
  destroys the retained instance (REQ AC-5).
- The subprocess leaves no `ovphysx-cache-*` directory under the
  redirected temp root on POSIX (REQ AC-5).
- Normal exit returns successfully; the `KeyboardInterrupt` case terminates
  non-successfully only after completing the cook and running cleanup
  (REQ AC-5).
