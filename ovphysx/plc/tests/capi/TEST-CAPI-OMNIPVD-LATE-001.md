<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OMNIPVD-LATE-001
maps_to: REQ-CAPI-OMNIPVD-LATE-001
type: integration
---

## Scenario

The C API opts into sequential late FILE and TCP sessions without active takeover, while default creation remains
incapable and startup sessions remain owned by their creating handles.

## Given

- Fresh-process default, capability-only, and startup-output creation on Windows x86_64 and Linux x86_64/aarch64;
  the latter two are capable, while only the default null-provider path is overhead-free.
- A created OvPhysX instance whose scene has already stepped.
- A second live create with an explicit OmniPVD creation setting, and an unconfigured peer handle.
- An asynchronous step queued by the live-stage owner before a peer recording transition.
- Exact FILE and ready-listener TCP destinations, plus invalid and unopenable destinations.
- A startup TCP case using a live Vehicle stage before the transition to late FILE.

## When

- The ABI declarations are compiled from C and C++.
- Default late start, capability-only dormant creation, startup-output creation, and the second-live-create cases are
  exercised in isolated process lifecycles; default incapability is checked before and after attach.
- Recording is queried, started, stopped, retried after failure, and finalized by explicit stop or instance destroy;
  start is also attempted while the shared runtime has no live stage, before initial attach and between detach and
  reattach.
- An active retarget is rejected; after stop, simulation continues and another destination starts successfully.
- A peer-owned active recording is detached by the stage owner, queried as inactive by both handles, and followed by
  a new owner session after another stage is attached; the stale peer attempts to query and stop that new session.
- The creating handle queries and publicly stops its startup session, while a peer queries false and cannot stop it;
  the owner then restarts recording to an exact late destination in a fresh-process cold-start scenario. A later
  startup-output reattach restarts its configured session under the reattaching handle before another late start.
- All calls execute sequentially on one test thread, as required by the same-thread contract.
- The peer starts, stops, and is destroyed while owning a recording immediately after the stage owner queues an
  asynchronous step.

## Then

- The appended destination, status, capability, and three-function ABI surface remains compatible (REQ AC-1).
- Capability defaults false, the entry/helper selects it before first creation, and startup output implies it (REQ
  AC-2).
- On Windows x86_64 and Linux x86_64/aarch64, default late start returns `INVALID_STATE` naming
  `omnipvd_recording_capable` (REQ AC-3).
- A second live instance receives `ERROR` when it tries to apply either a legacy or capability OmniPVD creation
  setting, while an unconfigured peer can share the runtime (REQ AC-4).
- Successful FILE and TCP starts become active only for their owning handle and the canonical reader observes
  object creation plus a completed simulation frame; the creating handle likewise owns a startup session, while a
  peer query remains false (REQ AC-5).
- Validation/open failure leaves the instance inactive and a valid retry succeeds (REQ AC-6).
- Active takeover by the owner or a peer is rejected without replacement and reports that recording is already
  active; a peer cannot stop the owner's startup or late session, inactive stop returns `INVALID_STATE`, and starts
  while the shared runtime has no live stage return `INVALID_STATE` with a readiness diagnostic (REQ AC-7).
- Public stop finalizes startup or late output and permits restart to the same or a different exact late destination;
  destroying an unrelated peer preserves the active owner, destroying a peer recording owner finalizes its stream
  without tearing down the live stage, and stage-owner destruction finalizes active output with final object
  destruction (REQ AC-8).
- Active detach finalizes the stream and clears the peer owner. Capability-only reattach is dormant, while configured
  startup output reattach starts a session owned by the reattaching handle; a stale peer cannot query or stop it (REQ
  AC-9).
- Cold startup ownership is reserved when creation succeeds, becomes active on lazy stage attach, and remains
  exclusive to the creating handle; after stop and late output, startup-output reattach relatches ownership to the
  reattaching handle (REQ AC-10).
- Each peer transition drains the attached owner's queued step before snapshot/finalization, preserves the step's
  later `ovphysx_wait_op` result through the internal synchronization watermark, and revalidates stage ownership.
  Sequential single-thread execution covers the supported contract; no concurrent-call behavior is claimed (REQ
  AC-11).
