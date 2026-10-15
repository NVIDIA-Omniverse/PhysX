<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-PROFILER-002
maps_to: REQ-CAPI-PROFILER-002
type: unit
---

## Scenario

Binding a Carbonite profiler backend follows the `/physics/profilerBackend`
selection: nothing is bound while it is `NONE`, an instance that selects one binds
even when the PhysX runtime is already started, a second such instance does not
fail, and the loader reports which profiler the framework actually bound.

For the NVTX backend, whether the zones reach Nsight cannot be observed from inside
the process. What is observable is the loader's own INFO/WARN report, which is only
emitted on the binding path -- so its presence and absence stand in for the binding
decision, and the timeline itself is verified by manual capture.

The trace backend is the one case with a checkable artifact, and it is asserted
directly: the file exists after the last instance shuts down, and its contents show
both the chrome-trace format and the ovphysx zone names.

## Given

- A C unit-test process in which the shared CPU instance already exists, so the
  Carbonite framework is up (log capture requires it) and the PhysX runtime is
  already started -- the state that drives `loadPhysxPlugins()` into its
  "runtime already started" early return.
- Log capture started at `OVPHYSX_LOG_INFO`, since the loader's report is INFO and
  the process default threshold is WARNING.
- For the AC-10 and AC-11 cases, a second instance that has bound `TRACE` and is
  held alive across the creations that follow it, so those run inside an open loader
  epoch rather than starting a new one.
- A separate, dedicated process for the trace case, with no shared instance in it,
  because the trace is written by the last-loader release and any other live
  instance would defer the write past the end of the test.

## When

- An instance is created and destroyed with the backend set to `NONE`.
- An instance is created and destroyed with the backend set to `NVTX`.
- A second instance is created and destroyed with `NVTX`, in the same process,
  after the backend has already been bound.
- With a backend bound and an instance still alive, a further instance is created and
  destroyed selecting `NONE`.
- In the dedicated process, with a `TRACE` instance bound to one output path and still
  alive, another instance is created and destroyed selecting `TRACE` with a different
  output path, and the first is then destroyed.
- In the dedicated process, a full workflow runs with the backend set to `TRACE` and
  `OVPHYSX_CONFIG_PROFILER_OUTPUT_PATH` pointing at a temporary file. While that
  instance keeps the profiler epoch open, a second instance is created and destroyed;
  the first instance is destroyed after the workflow.

## Then

- The `NONE` creation logs neither outcome: no INFO "Profile zones are publishing to
  the ... backend" and no WARN "profile zones go there rather than to the selected
  backend". Nothing was bound, so the zones keep reaching no consumer (REQ AC-1).
- The `NVTX` creation logs exactly one of the two outcomes, proving binding ran
  despite the early return (REQ AC-2), and reports which profiler bound (REQ AC-6).
  Silence is the regression this asserts against: it is what moving the binding call
  after the early return looks like from inside the process.
- The outcome is the INFO form, which is only emitted when the bound profiler is the
  selected backend -- so this case also covers the three binding steps of REQ AC-7.
  Each of them was observed failing during development: omitting the client default,
  or acquiring only through the optional entry point, produces the WARN form with no
  error anywhere else, which is exactly the silent-empty-capture failure this
  assertion exists to prevent.
- In this binary nothing else binds a Carbonite profiler, so the outcome is expected
  to be the INFO form; the WARN form failing the case would indicate a foreign
  backend, not a defect in this requirement.
- Both `NVTX` creations succeed, so a repeated binding neither fails instance
  creation nor is attempted twice (REQ AC-5), and the sticky selection still reads
  `NVTX` afterwards.
- The `NONE` creation made while a backend is bound and an instance is still alive
  logs the WARN naming the bound backend and stating the zones keep publishing to it
  (REQ AC-11). The bound backend is neither released nor silenced mid-epoch, which
  is the case that separates AC-11 from the AC-1 unbound path: the same selection
  means "bind nothing" in an empty process and "you are not the one who decides" in
  an occupied one.
- In the dedicated process, the second `TRACE` creation asking for a different output
  path logs the WARN naming the already-configured destination; no file ever appears
  at the path it asked for, and the trace is written to the destination bound first
  when the epoch ends (REQ AC-10). It has to be the dedicated process: this case needs
  `TRACE` to be the backend that actually bound, and in the shared process a case that
  selects `NVTX` binds it first and holds it for the whole run.
- In the dedicated process, the configured trace path does not exist before the run,
  exists and is non-empty after the instance is destroyed, and contains a
  chrome-trace phase field (`"ph"`) plus the zone names `ovphysx_step` and
  `ovphysx_get_version_string` (REQ AC-9 trace half, REQ AC-10). Exact event
  checks find one complete (`"ph":"X"` with duration) nested
  `ovphysx_create_instance` event and two complete
  `ovphysx_destroy_instance` events: the nested destroy and the final destroy
  that flushes the trace (REQ-CAPI-PROFILER-001 AC-5). This proves the lifecycle
  zones are paired both inside and at the end of the profiler epoch. Together the
  events prove the plugin loaded, bound, wrote uncompressed JSON, and received
  instance, process-level, and lifecycle API zones -- a gzip artifact or an empty
  event array would fail on the content assertions while still passing the existence
  one, so the failure message reports the head of the file.

## Not covered here

- The capture mask and the mask-before-startup ordering (REQ AC-3): `IProfiler` is
  not reachable from the test binary, which links neither the Carbonite headers nor
  `carb.static`. Partially covered in practice by the trace case, whose SDK zones
  use bit 8 and would be absent from the file if the mask were left at its default.
- The last-loader release (REQ AC-4) for the NVTX backend: the test binary keeps the
  shared CPU instance alive for its whole run, so that release path only executes at
  process teardown, where nothing can observe it. The trace case does cover the rule
  end to end, since the file only appears when the release runs. Cache tests do
  not supply profiler-refcount coverage: their directory now follows datastore
  shutdown rather than last-loader release.
- The build without `carb.profiler-nvtx.static` (REQ AC-9 NVTX half): a
  configure-time property, reported by the CMake status line naming the missing
  library and by `OVPHYSX_CARB_NVTX_BACKEND=0` compiling the registration out.
- First-writer-wins across two differing *active* backends (REQ AC-11) -- `NVTX`
  asked for while `TRACE` is bound, or the reverse. The `NONE`-while-bound case
  above covers the latch and its report; which of two active backends wins is the
  same single-bind path AC-2 covers, and asserting it would additionally require the
  test process to tolerate an NVTX binding it cannot observe.
- The AC-8 refresh. Every zone takes one route to `carb::profiler`, so there is no
  per-process routing decision left for this requirement to get wrong; what the
  refresh still covers is the SDK profiler callback being installed for an instance
  that asks for profiling after an earlier one did not (REQ-SIM-PROFILER-001 AC-6).
  Verified by capture: with a backend selected only by a *second* instance, the
  SDK zones must appear. It cannot be asserted in-process for the same reason
  REQ-SIM-PROFILER-001 records -- the PhysX SDK is created once and the harness has no
  seam to re-create it.

## Test location

`ovphysx/tests/c_unittests/test_profiling.cpp`
(`ProfilerBackend.CarbBackendStaysUnboundWhenDisabled`,
`ProfilerBackend.CarbBackendBindsOnALaterInstance`,
`ProfilerBackend.CarbBackendBindingIsIdempotent`,
`ProfilerBackend.SelectingNoneWhileBoundKeepsTheBoundBackend`, and
`ProfilerTraceBackend.WritesChromeTraceOnLastInstanceShutdown` plus
`ProfilerTraceBackend.SecondDestinationInTheSameEpochIsReported`, which
`scripts/test_cpp.cmake` runs in its own `profiler-trace` pass).
