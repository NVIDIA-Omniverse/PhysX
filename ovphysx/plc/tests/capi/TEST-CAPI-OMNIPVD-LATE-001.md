<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OMNIPVD-LATE-001
maps_to: REQ-CAPI-OMNIPVD-LATE-001
type: integration
---

## Scenario

The C API opts into explicit sequential FILE and TCP sessions without active
takeover. Default creation is incapable; capable creation and attachment stay dormant.

## Given

- Fresh-process default and capability-enabled creation on Windows x86_64 and Linux x86_64/aarch64;
  only the default null-provider path is overhead-free.
- Capability setup uses the scoped BOOL global key followed by plain creation.
- An inherited output-enabled Carbonite setting before standalone runtime initialization.
- The inherited setting is tested both with and without explicitly provisioned capability.
- Attached scenes before and after their first simulation step.
- A second live create with a retired OmniPVD creation setting, and an unconfigured peer handle.
- An asynchronous step queued by the live-stage owner before a peer recording transition.
- Exact FILE and ready-listener TCP destinations, plus invalid and unopenable destinations.
- An explicit TCP case using a live Vehicle stage before the transition to FILE.

## When

- Query recording status while an uncollected simulation step is pending, then collect its receipt.

- The ABI declarations are compiled from C and C++.
- Default start rejection, capable dormant creation/attach, and second-live-create cases are
  exercised in isolated process lifecycles; default incapability is checked before and after attach.
- The capability setter is tried with a wrong tag, with a live instance,
  after last-instance destruction, after shutdown with a live handle, and after
  successful shutdown with no live instances. A create failure after settings
  bootstrap is followed by a setter retry. Retired typed and raw OVD entries are
  tried at create and post-create, including both Carbonite path spellings.
- Retired numeric slots are read through the legacy typed getters.
- Recording is queried, started, stopped, retried after failure, and finalized by explicit stop or instance destroy;
  start is also attempted while the shared runtime has no live stage, before initial attach and between detach and
  reattach.
- An active retarget is rejected; after stop, simulation continues and another destination starts successfully.
- A peer-owned active recording is detached by the stage owner, queried as inactive by both handles, and followed by
  a new owner session after another stage is attached; the stale peer attempts to query and stop that new session.
- In a fresh process, create and attach without recording, start explicitly before
  the first simulation step, then stop and parse the stream. Detach/reattach is
  followed by a status query and another explicit start under a different owner.
- All calls execute sequentially on one test thread, as required by the same-thread contract.
- The peer starts, stops, and is destroyed while owning a recording immediately after the stage owner queues an
  asynchronous step.

## Then

- The owner still reports active recording during a pending step and the step receipt remains collectable (REQ AC-5, AC-11).

- The destination, status, capability, and three-function declarations compile from C and C++ (REQ AC-1).
- Capability defaults false; the scoped key is BOOL at `0x0000000c`, and plain
  creation remains dormant until explicit start. Invalid type writes do
  not alter capability. After first successful create, the scoped setter rejects
  changes until successful shutdown with no live instances, even after
  last-instance destruction; shutdown with a live handle does not reopen it.
  Failed create after settings bootstrap does not freeze the setter. The
  configured value survives shutdown, provisions the shared runtime, and is
  reported by the getter (REQ AC-1, AC-2).
- On Windows x86_64 and Linux x86_64/aarch64, default start returns `INVALID_STATE`
  directing callers to set `OVD_RECORDING_CAPABLE` before the first instance (REQ AC-3).
- Retired typed/raw OVD creation or post-create configuration returns
  `INVALID_ARGUMENT` with global capability/recording guidance without mutation,
  including on second-live-instance creation. Legacy typed getters reject
  retired slots with the same result and guidance; an unconfigured peer can
  share the runtime (REQ AC-4).
- Successful FILE and TCP starts become active only for their owning handle and the canonical reader observes
  object creation plus a completed simulation frame; a peer query remains false (REQ AC-5).
- Validation/open failure leaves the instance inactive and a valid retry succeeds (REQ AC-6).
- Active takeover by the owner or a peer is rejected without replacement and reports that recording is already
  active; a peer cannot stop the owner's session, inactive stop returns `INVALID_STATE`, and starts
  while the shared runtime has no live stage return `INVALID_STATE` with a readiness diagnostic (REQ AC-7).
- Public stop finalizes output and permits restart to the same or a different exact destination;
  destroying an unrelated peer preserves the active owner, destroying a peer recording owner finalizes its stream
  without tearing down the live stage, and stage-owner destruction finalizes active output with final object
  destruction (REQ AC-8). Parse matching destruction commands for every recorded
  scene and rigid actor; shared SDK retention and trailing collection removals do
  not require SDK destruction to be the stream's last command.
- Active detach finalizes the stream and clears the peer owner. Reattach remains
  dormant; after another explicit start, a stale peer cannot query or stop the
  new owner's session (REQ AC-9).
- Create, attach, and reattach do not start output or reserve ownership, including
  with inherited startup-output settings. First-live-instance creation clears
  inherited output-enabled=true and logs a warning directing callers to global
  capability and explicit recording (REQ AC-10). Without explicit capability, the
  getter remains false and start rejects with `INVALID_STATE` and capability
  guidance; inherited output does not provision the provider (REQ AC-2, AC-3, AC-10).
  Explicit
  start before the first step snapshots the initial world and completes that
  step's pre/post frames without a warmup step (REQ AC-10).
- Each peer transition drains the attached owner's queued step before snapshot/finalization, preserves the step's
  later `ovphysx_wait_task` result (the internal drain retires execution resources without collecting the task),
  and revalidates stage ownership.
  Sequential single-thread execution covers the supported contract; no concurrent-call behavior is claimed (REQ
  AC-11).

## Executable coverage

`tests/c_unittests/test_omnipvd_recording.cpp`:

- `OmniPvdGlobalConfig.PlainCreateSupportsLateRecording`
- `OmniPvdGlobalConfig.PendingBoolValueRejectsWrongTypes`
- `OmniPvdGlobalConfig.RetiredOvdEntriesAreRejected`
- `OmniPvdGlobalConfig.RetiredRawOvdPathsAreRejected`
- `OmniPvdGlobalConfig.RetiredOvdGettersAreRejected`
- `OmniPvdGlobalConfig.InheritedOutputEnabledDoesNotAutoStart`
- `OmniPvdGlobalConfig.InheritedOutputEnabledDoesNotProvideCapability`
- `OmniPvdGlobalConfig.ModernWritesStayFrozenUntilShutdown`
- `OmniPvdGlobalConfig.ShutdownWithLiveInstanceKeepsModernWritesFrozen`
- `OmniPvdColdCreation.DefaultInstanceHasNoProviderAndRejectsLateStart`
- `OmniPvdRecording.ActiveDetachFinalizesAndReattachCanRecord`
- `OmniPvdRecording.LateFileRecordingRejectsActiveTakeoverAndCanRestart`
- `OmniPvdRecording.FailedLateFileOpenCanRetryAndActiveDestroyFinalizes`
- `OmniPvdRecording.ExplicitTcpCanStopAndRestartToFile`
- `OmniPvdRecording.PublicExportCompatibility`
- `OmniPvdRecording.PublicExportsCaptureFirstSimulationFrame`
