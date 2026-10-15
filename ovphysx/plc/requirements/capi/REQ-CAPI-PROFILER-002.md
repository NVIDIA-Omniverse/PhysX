<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-PROFILER-002
title: Binding the Selected Profiler Backend
status: implemented
owner: ovphysx
---

## Description

Three emitters feed `carb::profiler` in an ovphysx process: the omni.physx runtime's
zones over stage load, cooking and step orchestration; the ovphysx C API entry
points (REQ-CAPI-PROFILER-001); and the PhysX SDK's thread-local zones, which the
runtime bridges across (REQ-SIM-PROFILER-001). A kitless ovphysx process loads no
profiler backend, so all three reach no consumer and the work they cover is
unattributed.

ovphysx therefore binds a Carbonite profiler backend when one is selected. Two are
supported: `carb.profiler-nvtx`, linked statically, and `carb.profiler-cpu`, a
shared plugin already in the shipped plugin set, which writes a chrome trace.

This requirement covers backend binding, its lifetime, and the trace destination.
Which zones exist and how the selection is expressed belong to REQ-CAPI-PROFILER-001 and
REQ-SIM-PROFILER-001.

## Acceptance Criteria

- AC-1: With `/physics/profilerBackend` set to `NONE`, no profiler plugin is loaded
  or registered and `carb::profiler::registerProfilerForClient()` is not called, so
  the process profiler pointer stays null and each zone is a branch that records
  nothing -- the behavior of a build without this sink.
- AC-2: The selection is read in `CarboniteLoader::loadPhysxPlugins()` before its
  "PhysX runtime already started" early return, so an instance that selects a
  backend in a process where an earlier instance left it off still gets its zones
  published.
- AC-3: The capture mask is set to `carb::profiler::kCaptureMaskAll` before the
  profiler is started. The mask must admit both the runtime zones, which pass 0
  and so resolve to `kCaptureMaskDefault` (bit 1), and the PhysX SDK bridge,
  which uses `kPhysicsProfilerMask` (bit 8). The order is required because
  `IProfiler` documents changing the mask on a started profiler as undefined.
- AC-4: Binding happens at most once per loader epoch, and the profiler is released
  only when the last live `CarboniteLoader` shuts down. This differs from the cache,
  which follows the datastore/framework lifetime (REQ-CAPI-CACHE-001 AC-1).
  An earlier per-instance profiler release would silence the zones of an instance
  still simulating, and for the trace backend would write the
  file while work was still being recorded. Release is `CARB_PROFILE_SHUTDOWN()`
  followed by `carb::profiler::deregisterProfilerForClient()`, in that order and
  with no interface pointer outliving the framework.
- AC-5: A plugin that fails to load or register is reported as a warning and leaves
  instance creation successful. A profiling opt-in that cannot be honoured degrades
  to no zones; it does not fail the workload it was meant to measure.
- AC-6: `registerProfilerForClient()` binds the framework's *default* `IProfiler`,
  which need not be the backend just selected: a process that already has a backend
  keeps it, and the zones go there instead. ovphysx compares the bound interface
  against the selected backend's and logs which one won -- `INFO` when the zones
  publish to the selected backend, `WARN` when they do not -- because the symptom is
  otherwise a capture with no rows in it and nothing to explain why.
- AC-7: Binding is explicit, in three steps, because loading or registering the
  plugin alone does not bind it and each step covers a way the binding silently
  fails to an empty capture:
  - The selected backend is named as this client's `carb::profiler::IProfiler`
    default. The install tree ships more than one backend shim, and an unqualified
    acquire may resolve to the wrong one. Being per-client, this does not disturb
    another tenant's choice.
  - It is then acquired once through the *non-optional* entry point, which is what
    initializes a freshly registered plugin. An optional acquire returns null for an
    uninitialized plugin rather than initializing it, and
    `carb::profiler::registerProfilerForClient()` acquires optionally -- so without
    this step the profiler stays unbound and every zone is dropped, with no error
    logged anywhere.
  - The client is then deregistered and re-registered, because
    `registerProfilerForClient()` is a one-shot per module and this module's
    profiler may already be bound by the load hook it installs.
- AC-8: After binding, ovphysx calls
  `omni::physx::runtime::refreshProfilerCallback()`. The runtime installs the SDK
  profiler callback when it creates the PhysX SDK, once per direct-runtime
  lifecycle;
  without the refresh, an instance that asks for profiling in a process whose first
  instance did not would find no callback installed, and every SDK zone would be
  dropped before it reached the bound backend (REQ-SIM-PROFILER-001 AC-6).
- AC-9: The two backends differ in how they are made available, and in what a
  missing one costs:
  - `NVTX` is `carb.profiler-nvtx.static`, linked into the ovphysx library and
    registered under the selection. A dependency package without it configures,
    links and runs: the build defines `OVPHYSX_CARB_NVTX_BACKEND=0`, registration
    compiles out, selecting it warns, and the other backend is unaffected. This path
    is defensive rather than expected -- the pinned package carries the library for
    every shipped platform and configuration, and it is the only Carbonite profiler
    backend built as a static library on any of them.
  - `TRACE` is `carb.profiler-cpu.plugin`, a shared plugin loaded on demand from the
    installed plugins directory. Carbonite skips a plugin whose name is already
    loaded, so loading it is safe even when a host loaded it first.
- AC-10: Selecting `TRACE` configures the plugin before startup: `saveProfile` on,
  `compressProfile` off so the artifact is directly readable, and `filePath` from
  `/physics/profilerOutputPath` when that is non-empty. With no path configured the
  plugin's own timestamped default in the working directory is used, and the choice
  is logged either way. The file is written by the AC-4 release, so it exists once
  the last instance has shut down and not before. Because the destination is fixed on
  the plugin before it starts, a later instance in the same epoch asking for a
  different path does not get one and is told so at `WARN` -- otherwise it would go
  looking for a trace that was never going to be written there.
- AC-11: The first backend to bind wins for the life of the *loader epoch* -- the
  span during which at least one instance is alive -- because the profiler is bound
  once and started once. An instance selecting a different backend while one is bound
  is told at `WARN` which backend is actually bound, and publishes to that one. This
  includes selecting `NONE`: it does not release the backend or silence the zones
  mid-epoch, for the same reason AC-4 defers release to the last loader. Once the
  last loader shuts
  down (AC-4) the latch is cleared, so a process that destroys every instance may
  select a different backend for the next epoch.
- AC-12: The static NVTX plugin is registered with Carbonite at most once per
  framework, tracked by a flag that AC-4's last-loader release does not clear.
  Linux owned-framework shutdown resets the flag; the next bootstrap registers the
  plugin again. Windows retains the framework and registration until process exit.
  Static plugin registration is not idempotent -- Carbonite requires unique plugin names and fails
  a duplicate -- so a second epoch that re-registered would take the AC-5 warning
  path and leave profiling unbound. Later epochs re-acquire and re-bind the plugin
  that is already registered. The trace backend needs no equivalent: Carbonite skips
  loading a plugin whose name is already present.

## Test References

- TEST-CAPI-PROFILER-002

## Code References

- ovphysx/CMakeLists.txt (`carb.profiler-nvtx.static` lookup,
  `OVPHYSX_CARB_NVTX_BACKEND`)
- ovphysx/deps_manifest.toml (`carb.profiler-cpu.plugin` in the shipped plugin set)
- ovphysx/src/include/internal/CarboniteLoaderState.hpp
- ovphysx/src/include/internal/GlobalState.hpp
- ovphysx/src/ovphysx/GlobalState.cpp
- ovphysx/src/CarboniteLoader/CarboniteLoader.cpp
  (`enableProfilerBackendIfRequested`, `registerProfilerBackendPlugin`,
  `configureTraceBackend`, `disableProfilerBackend`, the last-loader release in
  `CarboniteLoader::shutdown`)
- ovphysx/ovruntime/source/omni.physx/plugins/Setup.cpp
  (`OmniPhysxProfileCallback` -- the consumer of this binding on the runtime side,
  which routes the SDK zones to it; REQ-SIM-PROFILER-001 AC-2)
- ovphysx/ovruntime/include/omni/physx/PhysXRuntime.h
  (`refreshProfilerCallback`, the AC-8 entry point)

## Dependencies

- REQ-CAPI-PROFILER-001 - the selection this binds, and its environment-variable and
  config-entry forms.
- REQ-SIM-PROFILER-001 - the runtime side, which owns the SDK profiler callback and
  what it publishes.
- [ADR-0029](../../../ovruntime/plc/adr/ADR-0029-pluggable-profiler-backend.md) - the
  pluggable-backend decision, its alternatives and its consequences.
