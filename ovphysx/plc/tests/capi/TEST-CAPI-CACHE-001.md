<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CACHE-001
maps_to: REQ-CAPI-CACHE-001
type: integration
---

## Scenario

The process-private cooked-collider cache fallback is torn down cleanly
on the common sequential-lifecycle path, and the last-loader cleanup
survives a concurrent create racing a concurrent destroy without
crashing, deadlocking, or emitting the "failed to remove" WARN it would
raise on a real teardown failure.

## Given

- A build with `cooked_collider_cache_dir` unset, so the process-private
  fallback of ADR-0001 is active.
- Native log capture available to observe `[CarboniteLoader]` WARN/ERROR
  output during and after instance teardown.

## When

- A single instance is created and destroyed sequentially (no
  concurrency), draining `g_activeLoaders` to zero.
- A second scenario creates two instances, then destroys one on a
  background thread while the other is created/destroyed concurrently
  from the main thread, so the "last loader" refcount transition and a
  fresh `initialize()` can interleave.

## Then

- The sequential case produces no `[CarboniteLoader]` WARN/ERROR about
  failing to remove the process-private cache, confirming AC-1 and AC-3's
  common-case (first-attempt) path stays silent.
- The concurrent case completes without crash, hang, or a "failed to
  remove" WARN attributable to the cleanup racing the new loader's
  initialization, confirming the AC-2 serialization holds under a
  destroy/create race.

## Notes

Deterministically reproducing the underlying write-back race itself (a
cook's compute callback firing before `carb.datastore` finishes flushing
to disk) is out of scope for this test: this repo has no hook into
`carb.ujitso.default`/`carb.datastore`'s write-back completion, which is
precisely why the retry loop in AC-3 exists instead of a wait. This test
instead pins the two properties this repo *can* verify without that hook:
the common-case path stays silent, and the last-loader/initialize race
named in AC-2 does not corrupt state or deadlock.
