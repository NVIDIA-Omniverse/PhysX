<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-LOG-001
maps_to: REQ-CAPI-LOG-001
type: integration
---

## Scenario

Applications configure and drain the single ovphysx callback under filtering,
replacement, concurrency, and reentrant-call conditions.

## Given

- A live ovphysx process with native test-message emission available.
- Callback state that records metadata and concurrent invocation count.

## When

- Levels are set through DEFAULT, the four delivery levels, and NONE.
- `omni_physx_sdk` test records, a real `ovphysx_internal` invalid-path record,
  and a real `omni.physx` runtime warning are emitted at visible levels and
  while the ovphysx source level is NONE.
- A host-global Carbonite disable is followed by
  `ovphysx_set_log_level(OVPHYSX_LOG_WARNING)` and a real `omni.physx`
  GPU-broadphase warning site.
- Callbacks are set, replaced, disabled, filtered, invoked concurrently, and
  used to attempt forbidden reentrant calls.
- Flush is called with zero, finite, and infinite timeouts.
- A terminal shutdown is started while an accepted callback and a concurrent
  replacement transition are blocked.
- Shutdown is also called while a live handle remains for explicit destruction.
- `GlobalLifecycle.CallbackAfterLiveShutdownWaitsForNextBootstrap` installs a new
  callback after live-handle shutdown. Test records are emitted before and after
  the next bootstrap; only the latter reach the pending callback.
- `GlobalLifecycle.SidecarLogPolicySurvivesFrameworkRestart` emits the existing
  sidecar invalid-path diagnostic at NONE, INFO, then NONE across three complete
  lifecycles. It must be visible only at INFO, including after framework restart.
- In a dedicated process before initialization, the library level is set to
  VERBOSE, default console output is disabled, and a callback is registered.
  Two initialize/create/destroy/shutdown cycles each register their own callback
  before initialization and emit records before and after shutdown.

## Then

- Enum values and delivery match AC-1 and AC-2.
- The ovphysx source level suppresses all three named target sources at NONE and
  restores their configured delivery without changing Carbonite's
  process-global threshold or enablement. A host-global disable remains in
  force after `ovphysx_set_log_level(OVPHYSX_LOG_WARNING)`
  (`LogCallback.WarningDoesNotOverrideHostGlobalDisable`). The real
  GPU-broadphase warning does not reach the registered logger
  (`test_warning_does_not_override_host_global_disable`).
- Only the newest callback receives later messages and disable drains AC-3.
- Longest raw-prefix matching, later-rule equal-length ties, and copied non-NUL
  filter input satisfy AC-4.
- Maximum callback concurrency is one and reentrant callback, level, default
  output, flush, and shutdown calls fail AC-5. Python lifecycle coverage is
  owned by TEST-PYTHON-LIFECYCLE-001.
- Flush observes its accepted-message barrier and timeout contract AC-6; it
  makes no finite-time claim for records still buffered by Carbonite.
- Shutdown remains blocked until the accepted callback completes, disables the
  callback before returning, waits through the concurrent replacement instead
  of reporting transition-busy after teardown, and permits no later delivery,
  satisfying AC-3 and the shutdown half of AC-6.
- Live-handle shutdown also disables and drains callback delivery before the
  retained handle is explicitly destroyed, pinning the callback/user-data
  lifetime boundary in AC-3 and AC-6. Shutdown closes logger registration until
  bootstrap, so a late replacement cannot attach to the retiring framework and
  leave stale registration state for the next lifecycle. The live-handle regression
  checks this gate on both retained-framework and release-capable platforms.
- A focused Linux debugger probe additionally installs a callback through the public
  setter at entry to `releaseProcessCacheDirAtExit`, after `shutdownLogCallback`
  returns. The next lifecycle must deliver records. This exercises the precise
  late-replacement teardown window without adding a production synchronization hook;
  it is manual qualification, not an automated CI test.
- Both lifecycle cycles preserve the configured level and stored default-output
  preference. Each pre-initialize callback receives the emitted WARNING test
  record while its instance is live; shutdown disables it, and later emissions
  and the next cycle do not deliver to the retired registration
  (`GlobalLifecycle.PreInitLoggingConfigSurvivesLifecycleCycles`).
  This checks AC-3 and AC-6 across repeated bootstrap use without treating
  shutdown as a reset of process logging preferences.
- A real internal-sidecar invalid-path record and a real `omni.physx` runtime
  warning follow AC-7's controlled-source policy; the runtime warning is visible
  at WARNING, reports the `omni.physx` channel, and is absent at NONE.
  A process that attaches first at NONE and then at WARNING still emits that
  GPU-broadphase warning once -- a muted `WARN_ONCE` must not consume the latch.
  Synthetic production emitters are not used.
