<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OMNIPVD-LATE-001
maps_to: REQ-CAPI-OMNIPVD-LATE-001
type: integration
---

## Scenario

The C API opts into explicit sequential FILE and TCP sessions without active
takeover. Default creation is incapable; creation and attachment do not start a session.

## Given

- Fresh-process default and capability-enabled creation on Windows x86_64 and Linux x86_64/aarch64;
  only the default null-provider path is overhead-free.
- Capability setup uses the scoped BOOL global key followed by plain creation.
- An inherited output-enabled Carbonite setting before standalone runtime initialization.
- The inherited setting is tested both with and without explicitly provisioned capability.
- Attached scenes before and after their first simulation step.
- A second live create with a retired OmniPVD creation setting, and an unconfigured peer handle.
- An asynchronous step queued by the live-stage owner before a peer recording transition.
- Two live attachments with recording owned by one handle and an uncollected
  step queued on the other, plus a stageless recording owner for no-op detach.
- A non-owner attachment with an idle recording owner, and a separate case with
  a recording-owner step still queued.
- A recording owner with an independent companion preview that has simulated
  without fetching its results.
- Exact FILE paths, inferred directories and ready-listener TCP destinations,
  plus invalid and unopenable destinations.
- Missing nested parents, case-sensitive suffixes, an existing `.ovd`
  directory, and a regular file blocking directory creation.
- An explicit TCP case using a live Vehicle stage before the transition to FILE.

## When

- Query recording status while an uncollected simulation step is pending, then collect its receipt.
- Query status on another thread during repeated recording starts/stops; verify
  each completed transition reports the correct owner state. This case is also
  a race-detector target on platforms with an instrumented runtime.

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
- Start into a missing directory before the first step, stop and restart into
  it, then compare the first capture's bytes.
- Try exact and directory starts without capability or a live stage, malformed
  FILE descriptors, active directory retarget, and blocked-directory retry.
- A stageless recording owner submits a no-op detach before the stage owner
  detaches the last live attachment. Both handles query status, another stage
  attaches, and a new owner starts; the stale peer queries and tries to stop it.
- Detach an attached recording owner while a peer step is pending; parse the
  capture before any status query or peer wait, poll the peer task twice, then
  start a new recording on the still-attached peer.
- Detach a non-owner after collecting the recording owner's step, then verify
  the idle owner remains recording and can continue and stop its session.
- Separately detach a non-owner while the recording owner's step is still queued;
  collect that step afterward and verify the same recording ownership.
- Attempt native recording-owner detach while a companion preview is pending,
  fetch the preview, continue core recording, then close the preview and retry.
- In a fresh process, create and attach without recording, start explicitly before
  the first simulation step, then stop and parse the stream. Detach/reattach is
  followed by a status query and another explicit start under a different owner.
- Instance work and recording transitions execute sequentially on one test thread.
  The explicitly thread-safe, nonblocking status query also runs on a second thread.
- The peer starts, stops, and is destroyed while owning a recording immediately after the stage owner queues an
  asynchronous step.

## Then

- The owner still reports active recording during a pending step; the status
  query does not wait for instance work and the step receipt remains collectable
  (REQ AC-5, AC-11).

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
- FILE uses the literal lowercase `.ovd` suffix to select an exact file; other
  paths select a directory, including `.OVD`, `.ovd.backup` and a trailing
  separator. Admitted start creates missing parents recursively. Directory
  capture starts before the first step, uses a final timestamp/counter filename,
  and preserves earlier bytes across restart without `tmp.ovd` or rename
  (REQ AC-5, AC-8, AC-10).
- Validation/open failure leaves the instance inactive and a valid retry succeeds (REQ AC-6).
- Rejected malformed, incapable, unattached and active-retarget starts do not
  create their requested output paths. A blocked directory preserves its
  existing regular file; retry succeeds and active destruction finalizes the
  later directory capture with parsed object teardown (REQ AC-6, AC-7, AC-8).
- Active takeover by the owner or a peer is rejected without replacement and reports that recording is already
  active; a peer cannot stop the owner's session, inactive stop returns `INVALID_STATE`, and starts
  while the shared runtime has no live stage return `INVALID_STATE` with a readiness diagnostic (REQ AC-7).
- Public stop finalizes output and permits restart to the same or a different destination;
  destroying an unrelated peer preserves the active owner, destroying a peer recording owner finalizes its stream
  without tearing down the live stage, and stage-owner destruction finalizes active output with final object
  destruction (REQ AC-8). Parse matching destruction commands for every recorded
  scene and rigid actor; shared SDK retention and trailing collection removals do
  not require SDK destruction to be the stream's last command.
- Last-attachment detach finalizes the stream and clears the stageless peer
  owner. A preceding no-op detach on that unattached owner leaves recording
  active. Reattach remains dormant; after another explicit start, the stale
  peer cannot query or stop the new owner's session (REQ AC-9).
- Owner detach with pending peer work produces a parseable finalized capture
  before any status query or peer wait. It contains object destruction and
  frame-2 stop commands for two distinct scene contexts: both the owner's step
  and the peer's step reached fetch completion. The peer task is immediately
  collectable once; a second wait returns `NOT_FOUND`. The peer can start a new
  recording, and the former owner cannot stop it (REQ AC-9, AC-11).
- Non-owner detach preserves the original owner's active recording while another
  attachment remains live, both when the owner is idle and when its step is still
  queued. The queued step's task result remains collectable; the detached
  non-owner reports no recording ownership (REQ AC-9, AC-11).
- Pending-preview detach returns `INVALID_STATE` without issuing a task receipt
  or retiring the core attachment. The preview remains fetchable and recording
  remains active; retry after preview close succeeds and finalizes parseable
  output with object-destruction commands (REQ AC-9, AC-11).
- Create, attach, and reattach do not start output or reserve ownership, including
  with inherited startup-output settings. Creation logs no startup-recording
  warning (REQ AC-10).
  Without a configured or inherited capability value, the getter remains false
  and start rejects with `INVALID_STATE` and capability
  guidance; inherited output does not provision the provider (REQ AC-2, AC-3, AC-10).
  Explicit
  start before the first step snapshots the initial world and completes that
  step's pre/post frames without a warmup step (REQ AC-10).
- Each peer transition drains the attached owner's queued step before snapshot/finalization, preserves the step's
  later `ovphysx_wait_task` result (the internal drain retires execution resources without collecting the task).
  Sequential single-thread execution covers the supported contract; no concurrent-call behavior is claimed (REQ
  AC-11).

- Attach and detach during active recording run synchronously after the shared
  safe point, preserve submission order, and keep prior task results collectable
  (REQ AC-12).

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
- `OmniPvdRecording.OwnerDetachDrainsPeerAndPreservesTaskResult`
- `OmniPvdRecording.NonOwnerDetachPreservesIdleOwnerRecording`
- `OmniPvdRecording.NonOwnerDetachPreservesPendingOwnerRecording`
- `OmniPvdRecording.LateFileRecordingRejectsActiveTakeoverAndCanRestart`
- `OmniPvdRecording.FailedLateFileOpenCanRetryAndActiveDestroyFinalizes`
- `OmniPvdRecording.ExplicitTcpCanStopAndRestartToFile`
- `OmniPvdRecording.PublicExportCompatibility`
- `OmniPvdRecording.PublicExportsCaptureFirstSimulationFrame`
- `OmniPvdColdCreation.DirectoryStartRequiresCapabilityWithoutCreatingOutput`
- `OmniPvdRecording.DirectoryRecordingCapturesFirstFrameAndPreservesPriorFiles`
- `OmniPvdRecording.FilePathSuffixSelectsExactFileOrTimestampedDirectory`
- `OmniPvdRecording.BareExactFilenameRecordsInCurrentDirectory`
- `OmniPvdRecording.Utf8ExactFileCreatesParentsAndRetainsReplacementBehavior`
- `OmniPvdRecording.DirectoryFailureCanRetryAndActiveDestroyFinalizes`

`companion/tests_native/test_ovd.py`:

- `test_recording_owner_detach_rejects_pending_preview`


`tests/c_unittests/test_worker_lifecycle.cpp`:

- `WorkerLifecycle.RecordedDetachIsSynchronousAndDrainsPeers`
- `WorkerLifecycle.RecordedDetachThenAttachKeepsOrder`

`tests/c_unittests/test_instance_lanes.cpp`:

- `Lanes.RecordingStatusWaitsForNoInstanceWork`
- `OmniPvdSelectiveCapture.StatusQueriesDuringRecordingTransitions`: concurrent
  status queries return success while starts/stops publish each completed owner
  state; the same case can run with a race-instrumented native library.
- `OmniPvdSelectiveCapture.StatusQueriesDuringRecordingTransitions`: concurrent
  status queries return success while starts/stops publish each completed owner
  state; the same case can run with a race-instrumented native library.

## Receiver control with queued work

`OmniPvdSelectiveCapture.TcpPendingRequestWaitsForQueuedPeerWithoutCollectingItsReceipt`
holds a peer worker, queues a real job, buffers a TCP receiver selection, and
submits the owner's next step. Submission waits for the queued peer job. After
release, that peer receipt remains collectable, the owner step succeeds, and the
stream acknowledges the applied selection. The existing Lanes, InstanceExecutor
and WorkerLifecycle suites retain the current queue, wait and reentry coverage.
