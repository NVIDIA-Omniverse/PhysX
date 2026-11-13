---
name: ovd-record
compatibility: Requires a checked, profile or debug PhysX SDK build and CMake 3.21 or newer; Release disables OmniPVD recording.
description: Use when recording selected PhysX SDK objects to an OmniPVD file or TCP stream, starting capture late, or choosing recording scope through a receiver's discovery catalog.
metadata:
  author: "NVIDIA Omniverse Physics Team"
  version: "0.1.2"
  tags: "physx, omnipvd, ovd, recording"
---

# Record a finalized OVD file

Use the installed `ovd-record --help` for a six-step headless SDK example:

```sh
ovd-record "new capture.ovd" --late-steps 20
ovd-inspect summary "new capture.ovd"
```

The sample refuses existing output paths, records a named moving body, checks twelve
pre/post states against live SDK values, and closes its file before scene teardown.
`--change-step 3` changes X velocity before the third recorded step.
`--gpu` requests GPU simulation; exit 77 means unavailable, not a passing GPU run.
CPU capture remains the required baseline. Use checked, profile or debug SDK builds;
Release disables OmniPVD. Locate tools on PATH or in the user's SDK install `bin`.
If the sample is absent, build `<SDK>/pvddom/workflow` with CMake (checked configuration),
then install it. The standalone inspector build alone does not build PhysX or the recorder.

For an existing application, follow its scene ownership and stepping loop:

1. At initialization, create `PxOmniPvd` with `PxCreateOmniPvd(*foundation)` and pass it
   as the final argument to `PxCreatePhysics`. Check all returned pointers.
   There is no setter that retrofits the sampler onto an existing `PxPhysics`.
2. At an idle scene boundary, obtain `physics->getOmniPvd()->getWriter()`.
   Create an owned `createOmniPvdFileWriteStream()`, select a new path with
   `setFileName`, and check `openStream()` before `writer->setWriteStream(*stream)`.
   This SDK file stream truncates on open; preserve existing recordings.
3. Check `startSampling(filter)`. It writes the selected current state before subsequent
   changes, so late capture works. Omitting the filter records everything. Complete
   the requested finite `simulate`/`fetchResults` pairs.
4. Check the writer's `eSTREAM_WRITE_FAILURE` status, `stopSampling()`, then the owned
   stream's `closeStream()`. Stopping sampling alone does not flush or close the file.
   Finalize before releasing actors/scene when their final state must stay queryable.
   Keep the bound stream object alive until the writer cannot access it; destroy it
   through `destroyOmniPvdFileWriteStream` after physics/recorder cleanup.

A fresh late capture starts a new scene frame sequence. In this sample, post-step
frames are 2,4,...,12; frame 13 is an opened pre-step interval. Verify actual markers
and application step logs before using that mapping elsewhere. STOP does not freeze
the last START interval. For a new capture, stop sampling and bind a fresh file before
starting again; appending requires explicit segment handling.

## Select objects and data at the producer

Use the matching SDK's `PxOmniPvdFilterDesc` from `omnipvd/PxOmniPvd.h` with
`startSampling(&filter)`. The SDK copies its lists at start. `validateFilterDesc()`
can check it without writing or GPU transfers, but does not lock or reserve the
validated objects. Supply external synchronization for validation and start:
every scene must be outside simulate/collide through completed fetchResults or
fetchResultsFinish; exclude concurrent simulation, result fetching (including
fetchResultsParticleSystem), creation, release and scene-membership changes.

| Descriptor field | Meaning |
|---|---|
| `scenes` / `nbScenes` | Selected scenes, including their future members |
| `objects` / `nbObjects` | Individual supported objects; combined with scenes as a union |
| Both lists empty | All SDK objects, including objects outside scenes |
| `recordSecondaryData=false` | Primary identity, poses, velocities, joint/vertex state, geometry and materials |
| `recordSecondaryData=true` (default) | Primary plus supported forces, contacts, configuration, drives and diffuse data |

There is no transform-only mask. Explicit selectors are rigid statics/dynamics,
whole articulations, particle systems or individual particle/diffuse buffers, and
deformable surfaces/volumes. Links, joints, constraints, shapes, materials, meshes
and aggregates are not explicit selectors; supported data follows owning objects
and native dependencies. A selected particle system includes its buffers. One
selected buffer brings its owner's identity/materials without sibling payloads.
Base particle positions/velocities/phases and deformable vertex state are primary;
all diffuse particle payloads are secondary. Ordinary GPU and DirectGPU remain in
their original simulation mode. A filter cannot enable payloads the producer does
not record, and selective output is not a guarantee of proportional GPU transfer cost.

With secondary data, contacts are included if either endpoint is selected.
Required contact/joint/constraint counterparts bring identity and current motion;
their other connections do not recursively expand selection. Selecting the ground
can include many touching actors. DirectGPU counterpart state can await the next
fetchResults. `isObjectRecorded()` includes selected objects/dependencies but
returns false for connection-only actors even when their state was emitted.
Releasing a selected object removes it; address reuse does not select its replacement.

Stop and start with another descriptor for a local filter change. Without an active
discovery scope, a new unfiltered start resets to all objects/all supported data.
Raw `OmniPvdWriter` calls are not automatically filtered. A module must honor
`onStartSampling(PxOmniPvd&, const PxOmniPvdFilterDesc*)` and the applicable object
selection itself; copy descriptor/list data retained beyond the callback.

## Discover and change selection over TCP

Prepare the listening `OmniPvdSocketReadStream` receiver before the producer's
first write. The producer opens the outbound, bidirectional TCP connection.
The transport is trusted plaintext; the SDK does not change network policy.

1. Bind the owned socket write stream. To avoid an initial detailed snapshot,
   check `startCaptureDiscovery(&scope)` instead of `startSampling()`. This emits
   names, types, handles and ownership without geometry, motion or GPU readback.
   The copied scene/object lists are the maximum catalog and later selection
   scope; empty lists permit all SDK objects. The scope's secondary flag is ignored.
   `isSampling()` is false while waiting; `isCaptureDiscoveryActive()` is true.
2. At the externally synchronized all-scene safe point described above, call
   `processCaptureRequests()` repeatedly from the application's loop. Optional
   `hasPendingCaptureRequest()` only polls/buffers; it does not apply a selection.
   A `PxOmniPvdCaptureFilter` argument may restrict or deny validated requests.
   Its replacement arrays must remain valid until `processCaptureRequests()` returns.
3. The receiver calls `requestCapture(OmniPvdCaptureRequest)` with live catalog or
   stream handles, a nonzero increasing connection-local `requestId`, and
   `recordSecondaryData`. Handles are neither stable names nor cross-run IDs.
   `requestCaptureCatalog()` refreshes a supported active discovery catalog without
   changing selection. Discovery scope also restricts subsequent local starts.
4. A successful send is pending. Read matching `PxOmniPvdMetaData.captureRequestId`,
   `captureRequestStatus` and `captureEpoch` from the forward stream. Status `1`
   means applied; `2` invalid, `3` busy and `4` denied retain the prior policy.
   Accepted selections retire recorded objects and write a fresh selected snapshot
   on the same connection. Retain policy/epoch with context, segment and frame
   identity; rediscover those boundaries before comparing observations.
5. `stopSampling()` ends both discovery and sampling. Finalize the owned stream
   separately. Keep stream/callback owners alive through their last use. Calls that
   close/reopen/destroy a reader must not race a blocking read; concurrent close is
   not supported cancellation. The receiver must save/finalize a file for inspection.

Forward stream and integration versions do not replace reverse-control capability
negotiation. Older producers remain readable; unsupported `requestCapture()` returns
false without sending control bytes. Use producer-side selection when available.
Open a new TCP connection for a recreated PxPhysics instance.

Custom modules retire their records in `onCaptureReset()` before the replacement
snapshot and rebuild through `onStartSampling()`. Reset runs without the writer
lock: use `PxOmniPvd::ScopedExclusiveWriter` for its writes. Copy temporary callback
filter data. The object-capture callback, in contrast, runs with the writer locked
and must not start/stop recording or modify SDK objects or the callback registry.

Inspect finalized files with `ovd-inspect`. For an existing file investigation use
`ovd-inspect`; for two-file velocity evidence use `ovd-compare`.
The `ovd-record` command above is a finite FILE example, not a TCP receiver or a
selective-capture CLI. Do not invent command-line selection flags for it. Use the
SDK APIs above in an existing producer, or `ovphysx-ovd-investigation` for the
OvPhysX C/Python start wrapper. The bounded inspector does not decode arbitrary
contact/particle/deformable arrays. Omitted or unwritten values are not zero.
