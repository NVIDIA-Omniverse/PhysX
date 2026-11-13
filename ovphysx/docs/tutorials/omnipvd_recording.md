<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# OmniPVD Recording -- Capture Physics Internals to FILE or TCP

This tutorial shows how to record OmniPVD data from ovphysx to `.ovd` files
for offline inspection or stream it live to a TCP listener. OmniPVD captures
selected PhysX object and contact state each frame so you can debug and
visualize physics behavior.

## Prerequisites

- Install ovphysx and confirm native libraries load.
- Install the required PhysX schema provider in the same Python environment:
  `python -m pip install physx-usd-schemas`. See [Physics Schemas](../physics_schemas.md)
  for the native SDK installation route.
- A USD scene with physics objects. This tutorial uses `links_chain_sample.usda`,
  which ships with every package under `ovphysx/samples/data/` in the wheel,
  `<sdk-root>/samples/data/` in the C/C++ SDK, and `tests/data/` in a repository
  checkout.
- For headless inspection: a separately installed PhysX SDK `ovd-inspect` executable.
  A compatible Kit application with the OmniPVD extension is an optional visual consumer.
  Build and install the inspector from an SDK source distribution using
  `physx/pvddom/README.md` ("Installed tools and CMake consumers"); configure
  `PVDDOM_BUILD_INSPECTION=ON`, `PVDDOM_BUILD_TOOLS=ON` and the matching `PVDRUNTIME_ROOT`.

## Headless capture and investigation

The wheel includes `ovphysx/samples/python_samples/omnipvd_inspection.py` and
`simple_physics_scene.usda`. Locate the sample relative to the installed package,
then run it from any working directory:

```python
from pathlib import Path
import subprocess
import sys
import ovphysx

sample = Path(ovphysx.__file__).resolve().parent / "samples/python_samples/omnipvd_inspection.py"
subprocess.run([
    sys.executable, str(sample),
    "--inspector", "/absolute/sdk-install/bin/ovd-inspect",
    "--output-dir", "/absolute/new-capture-directory",
    "--device", "cpu",
], check=True)
```

On Windows, supply the installed `ovd-inspect.exe` path. The inspector is a separate
SDK tool, not a wheel dependency; the sample never searches a monorepo sibling or
loads a viewer. The `ovd-record`, `ovd-inspect` and `ovd-compare` skills ship
with the PhysX SDK. No MCP service is required.

Run `--device gpu` in a separate process and a different new directory for a GPU
capture. This explicitly enables GPU dynamics and DirectGPU and requires actual
CUDA-backed live state reads; a CPU fallback fails. CPU mode is process-wide and
sticky, so it cannot be switched back in the same process.

The sample validates the inspector and required PhysX schemas before creating its
output directory. It preserves `scene.usda`, `capture.ovd` and `evidence.json`.
Existing output directories are refused. Capability is provisioned before creating the first
runtime, the FILE parent directory exists before start, and stop is checked before
running the inspector while the instance and borrowed stage remain alive. The
report includes a live read after the final unrecorded step, showing that
inspection did not wait for teardown. Failed stop,
missing tooling, invalid output and mismatched observations fail the command.

The report selects `/World/Cube1`, `/World/Cube2` and `/World/KinematicCube` by exact
name/class in the reported context/segment. This sample verifies completed recorded
frames 2 and 20 against live post-step reads 1 and 10; those numbers are not a general
OVD-to-application frame mapping. Both dynamic bodies fall. The fixed kinematic
body's Carried initial snapshot is corroborated by ten live reads: it is not a fresh
recorded observation. Inspector sample provenance is retained alongside a capture
SHA-256; inspection never changes source bytes.

For another capture, discover contexts, frame markers and completion flags with
`ovd-inspect summary FILE` before selecting an exact object and attribute. Match
objects across captures by explicit semantic names or a caller-supplied mapping;
pointer handles are not cross-run identities. `ovd-inspect compare --help` is not a
subcommand help form: use `ovd-inspect --help` for its required left/right selectors,
frame range, signed offset and tolerance. Comparison defaults to linear velocity
with version-1 evidence. Paired `--attribute NAME --distance vec3|position|orientation`
flags emit version 2 with a nested `measure {attributeName, distanceKind, units}`,
`leftValue`, `rightValue` and numeric `componentCount`. Vec3/position require
explicit units; orientation uses radians and xyzw quaternion values. Units label
without rescaling. The bounded loader accepts supported flat SDK/OvPhysX captures, not
every OVD accepted by a compatible viewer. Exit 0 is Equal and 1 Different.
Version 2 includes `leftFrameCompleted`/`rightFrameCompleted`: true means completed,
false means an open START interval, and null means the frame is absent. Report
completion independently of value equality; Equal does not prove the run finished.

A load rejection exits 2 with one stdout `physx.ovd.error` version-1 object
(`code`, `message`, `byteOffset`, `commandIndex`, `role`, `limitOption`) and stderr.
Every Incomparable result, including selection/query failures such as a missing
actor or recorded frame and valid queries lacking exact samples, emits
`physx.ovd.inspection-evidence` JSON with `status: Incomparable`, an explanatory
`message`, plus stderr, and exits 2.
Command-line parsing errors and summary/inspect selection/query failures
exit 2 with stderr only. Output failures exit
2 with a diagnostic and may follow valid or partial stdout. Annotation I/O can
fail after comparison evidence was emitted, so check stderr and exit status
before claiming a sidecar was saved. Distinguish errors by `format`; a rejected
load does not prove corruption or absence of the requested data.

Optional missing references may be accepted; incompatible edges and malformed
schema remain rejected. An immutable stopped recording cut during its final command
can be inspected by explicitly passing `--recover-tail` to either `summary` or `inspect`. Check
`loadInfo.recoveredTail`, `terminalError` and the stderr warning. The retained
validated prefix may include an open frame; explain its incomplete-data caveat.
Recovery never repairs malformed payloads, limits, I/O failures or required
references. Comparison always loads strictly. Preserve source captures.

For a trusted capture of justified size, retry only if `code` is `Limit` and
`limitOption` identifies a known option from `ovd-inspect --help`. Increase that
positive-integer option alone by a justified bounded amount. Both payload caps
remain active: raising `--max-attribute-bytes` may expose `--max-retained-bytes`
or conversely. Each later diagnosed limit requires separate justification;
acceptance is not guaranteed. Null/unknown `limitOption` or another error code
stops budget retries. Do not guess options from English messages or suppress
validation. Preserve rejected captures for a compatible reader or diagnosis.
Exact samples are required by default; Missing or Carried samples are Incomparable.
Use `--allow-carried` only when carried prior known state is acceptable; it never
supplies missing values.

Explain evidence in the ordinary conversation. If saved annotations are requested,
use `--annotations NEW_PATH` for a new non-source path to preserve the SDK's
versioned comparison evidence as a separate sidecar. Review identity, frame alignment, source sample metadata,
tolerance/units and coverage with the SDK `ovd-compare` skill. This sidecar has no
viewer import guarantee. The sample's `evidence.json` is a separate workflow report,
not that annotation format. When contacts occur, the capture can contain
scene-level contact impulse arrays; this inspector cannot report them, so use a
compatible OmniPVD reader or viewer. This fixture does not establish that
contacts occurred. PxJoint reaction forces are distinct and are not recorded.
Do not infer forces or impulses from velocities.

## Required Config

Set `GlobalConfigKey.OVD_RECORDING_CAPABLE` to `True` with module-level
`set_global_config()` before `PhysX()`. In C, initialize and set
`OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE` with
`ovphysx_config_value_bool(true)` before plain `ovphysx_create_instance()`.
This process-wide BOOL bootstrap option installs the provider and sampler when
`PxPhysics` is created. It defaults to false; without it, late start fails.
Capability alone does not start output but retains sampler, listener, mutex, and
scene readback costs. Only the default disabled configuration avoids those costs.
The scoped setter is rejected after the first successful create until successful
shutdown with no live instances, including after all native instances are destroyed.
Python's `PhysX.destroy()` releases its lifecycle owner; releasing the last Python
owner performs that shutdown, provided no native instances remain, and reopens
the setter. Its configured value survives shutdown.
The getter reports the configured bool. Failed creates do not freeze the setter.

Attach and initialize the scene, then call `start_recording()` with an
`OmniPvdDestination`. To capture from the first simulated frame, start before
the first step. FILE accepts an exact lowercase `.ovd` filename or a directory
path and creates missing parents recursively at explicit start. TCP requires a
ready listener. Stop finalizes the output and allows a new destination on the
same instance. TCP is synchronous trusted plaintext.

Creation, attach, and reattach never start output automatically and preserve an
existing shared session. Old startup
configuration submitted through ovphysx is rejected; inherited Carbonite output
and destination settings are inert, while an inherited capability value is
honoured. FILE writes directly to its final filename, without a
`tmp.ovd`, stop-time rename, or directory import. See the
[API migration guide](../ovd_api_migration.md).

### FILE paths

Use the existing FILE destination for either mode:

```python
from ovphysx import OmniPvdDestination

exact_file = OmniPvdDestination.file("captures/factory/capture.ovd")
timestamped_files = OmniPvdDestination.file("captures/factory")
```

The literal, case-sensitive `.ovd` suffix selects an exact filename. Every
other nonempty path selects a directory, including `capture.OVD`,
`capture.ovd.backup` and `capture.ovd/`. An existing directory called
`capture.ovd` requires a trailing separator; its existence does not override
the suffix rule.

An admitted explicit start creates missing directories recursively. Directory
mode opens a local-time `YYYY_MM_DD_HH_MM_SS_CC_rec.ovd` filename exclusively;
the counter starts at `01` and advances past existing names. Restarting into
the same directory preserves previous captures. Exact-file mode can replace
the existing named file. Capability and attachment do not prepare output paths.

## Select objects and data

Initial selection runs in the producer and applies to FILE and TCP destinations.
Scene and object paths form a union. FILE retains the filter until stop/restart;
an active TCP receiver can replace it after an applied acknowledgment. To
record one robot's primary state, pass a filter when starting:

```python
from ovphysx import OmniPvdFilter, OmniPvdDestination

selection = OmniPvdFilter(record_secondary_data=False, objects=["/World/robot"])
physx.start_recording(OmniPvdDestination.file("robot.ovd"), filter=selection)
# The same filter can stream to a ready listener:
# physx.start_recording(OmniPvdDestination.tcp("127.0.0.1", 5425), filter=selection)
```

Selected objects always include primary data: identity and relationships,
poses, velocities, joint state, geometry and materials. Set
`record_secondary_data=True` to include all other supported data, such as
forces, contacts, configuration and drive targets. This is the default;
`False` records primary data only. Python requires a `bool` for this option
and rejects integers, including `0` and `1`, and other non-boolean values.
All vehicle-specific telemetry is secondary, including its vehicle velocity
fields. The vehicle actor's ordinary primary state remains recorded.

`scenes` and `objects` contain exact absolute prim paths and combine as a
union. Populated lists resolve only within this `PhysX` instance's attachment.
A scene includes its current and future members. Empty lists select all objects
across the shared runtime. Omitting `filter` preserves unrestricted recording,
including an unattached peer start. Paths and `record_secondary_data` are
copied during start; stop and restart to choose a different initial filter.

When secondary data is enabled, contact pairs include either selected endpoint.
Rigid actors connected to selected actors by a recorded contact, joint or
constraint include their identity, scene, shapes and current pose/velocity state.
Their other contacts and joints do not expand the selection. A contact pair whose
bodies fall asleep while touching keeps the connected actor recorded, although
the scene's contact arrays leave the pair out until the bodies wake. When no
recorded connection requires an actor, its recording ends at the next scene
recording update. Reconnecting records its current state again. A connected actor
that leaves its scene stops updating at once. It stays in the recording outside
every scene and aggregate until a new connection records it again, it joins a
selected scene, or it is released. In Direct GPU API scenes, a rigid dynamic or
articulation link that has already been simulated and becomes connected between
simulation steps gets its pose and velocity at the next `fetchResults()`.

| Path list | Selectable native objects |
|---|---|
| `scenes` | Physics scenes and their current/future members |
| `objects` | Rigid statics/dynamics, whole articulations, PBD particle systems, individual particle/diffuse buffers, deformable surfaces and volumes |

An exact prototype path includes all its native instances. A fixed-base
root-joint alias is accepted when the registry identifies a whole articulation.
Actual articulation links, joints, constraints, shapes, materials, meshes and
aggregates cannot be selected directly. Their supported data follows selected owners
and the native dependency rules. Whole-articulation selection includes its
links and joints. A particle system includes all its buffers; to exclude sibling
buffer payloads, select one particle set's exact prim path:

```python
selection = OmniPvdFilter(record_secondary_data=False, objects=["/World/particles"])
```

Particle and deformable primary state works with ordinary GPU readback and
DirectGPU simulation. Selected particle buffers, including the base particles
of a diffuse buffer, record active `positionInvMasses` and `velocities` entries
with four float32 components each, or 32 bytes per particle per recorded frame.
`positionInvMasses` contains XYZ and inverse mass. Primary `phases` use one
uint32 per particle and are recorded in snapshots and after phase updates.
With secondary data enabled, `diffusePositionLifeTime` and `diffuseVelocities`
add 32 bytes per active diffuse particle; primary-only recording omits all
diffuse data while retaining the base particles.
Deformable surfaces emit actor-local `positions` and `velocities`; volumes emit
`collisionPositions`, `simulationPositions` and `velocities`, with four float32
components per vertex. Static topology is primary data, and diffuse parameters
are secondary. Selection does not change simulation mode.

Every attribute already supported by the producer belongs to primary or
secondary data. Synthetic geometry, drives and configurations follow their
owner. Tendons and deformable attachments remain outside this coverage when the
native producer emits no corresponding OVD class.

C callers initialize `ovphysx_omnipvd_recording_options_t` with
`OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT`, which sets the `bool`
`record_secondary_data` field to `true`, and call
`ovphysx_ovd_start_recording(handle, &destination, &selection)`.
Set that field to `false` for primary data only. Passing `NULL` as the options
argument retains unrestricted recording.
Lengths and counts are explicit; views are borrowed only during the call.
Malformed lists or paths return `INVALID_ARGUMENT`; missing or unsupported
objects return `NOT_FOUND` before the destination opens. The C start signature
has changed from two arguments to three: rebuild callers of the old form and
pass `NULL` as the third argument to retain unrestricted recording. See the
[C start migration](../ovd_api_migration.md#c-start-signature).

## Receiver requests over TCP

An updated native receiver can request a new filter while reading an active TCP
stream. `OmniPvdSocketReadStream::requestCapture()` sends an
`OmniPvdCaptureRequest`: a nonzero `requestId` that increases with every
request on the connection, the `recordSecondaryData` boolean, and scene and
object handles observed in the stream (`scenes`/`nbScenes` and
`objects`/`nbObjects`). Selected objects always include primary data: poses,
velocities, geometry and materials. Set `recordSecondaryData` to `false` for
primary data only. Its default value, `true`, also records all supported
secondary data.

Requests use the OVC3 control packet and require the producer's revised-filter
capability. Earlier experimental control masks are not reinterpreted. This
negotiation is independent of the unchanged forward recording format and its
OmniPVD stream and PhysX integration versions. Older one-way streams remain
readable. To limit the initial recording, select scenes or objects when starting
the sender; empty lists include primary payloads for every SDK object.
The receiver sends handles, while the sender's C/Python start descriptor uses
attachment paths. A successful send means the request is pending.
Handles are process-local addresses, valid only for the current run and TCP
connection. Every OvPhysX TCP recording or discovery start opens a new connection.
Requests do not accept names or USD paths. To repeat a selection in a later run,
pass the saved USD paths in the start filter, or select handles from the new
stream or discovery catalog.

OvPhysX polls receiver input when the recording owner submits a public simulation
step. Empty polls do not wait for peer jobs. A complete request uses the same
all-instance safe point as recording start: earlier queued work finishes without
collecting its receipts before selection changes. Already queued batches retain
their order; the new policy takes effect before the next submitted owner step.
Competing native authoring can postpone application until a later submission.
This request handling can make step submission wait for earlier jobs. Keep
submitting owner steps while waiting for an acknowledgement; status queries do
not pump control. Accepted requests replace the policy and
write a fresh snapshot on the same connection. The receiver observes ordinary
`PxOmniPvdMetaData.captureRequestId`, `captureRequestStatus`, and `captureEpoch`
attributes through `OmniPvdReader`; status values are `1` applied, `2` invalid,
`3` busy, and `4` denied. Wait for the matching applied
acknowledgment before treating the new policy as active.

A receiver can half-close its control writes while continuing to read forward
recording. A terminal socket or malformed-control error ends and cleans the
session at the next safe point; another FILE or TCP recording can then start.

The runtime accepts only supported live objects in the recording owner's attachment:
rigid actors, whole articulations, deformable actors, particle systems and
individual particle buffers. Links, joints, aggregates, shapes, materials and
meshes are invalid explicit selections; their data follows selected owners.
Receiver empty/all lists select that owner's scenes; foreign objects are
denied and stale IDs are invalid. A recording started by an instance without its own attachment still sends
unrestricted output, but receiver control stays denied for that whole session. FILE sessions use
their initial policy until restart. Older TCP producers do not advertise
control support, and `requestCapture()` returns false without sending control
bytes to them. The Python API starts the sender; receiver requests use the
native OmniPVD socket API.

## Code

### Python

This sample provisions capability, attaches `links_chain_sample.usda`, records
to an exact FILE destination before the first step, then stops and tears down:

```{literalinclude} ../../tests/python_samples/omnipvd_recording.py
:language: python
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### C++

**CMakeLists.txt**

The C++ sample uses the same `find_package(ovphysx)` and `ovphysx::ovphysx`
target as every other sample:

```{literalinclude} ../../tests/c_samples/omnipvd_recording_cpp/CMakeLists.txt
:language: cmake
```

**Source**

The C++ sample starts an exact FILE recording after attaching the scene and
before the first simulation step:

```{literalinclude} ../../tests/c_samples/omnipvd_recording_cpp/main.cpp
:language: cpp
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### Late TCP Recording

Opt in before creating the first instance, before attaching and stepping the
scene. Start the trusted-plaintext TCP listener before calling
`start_recording()`; the connect is synchronous. This complete sequence
simulates before recording, records to TCP, stops, continues simulating, and
then records to a FILE destination. Replace `scene.usda` with a path to your own
USD scene, such as the `links_chain_sample.usda` file named in the
prerequisites:

```python
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import GlobalConfigKey, OmniPvdDestination, PhysX, newton_schema_root, set_global_config
from ovphysx.samples.python_samples.schema_support import physx_schema_paths

set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
physx = PhysX()
# Register PhysX and Newton schemas before the first USD schema read (population or export).
ovstage.population.register_usd_schemas([*[str(p) for p in physx_schema_paths()], str(newton_schema_root())])
stage = ovstage.Stage("late-recorded-scene")
ovstage.population.open_usd_with_desc(
    stage,
    "scene.usda",
    ordinal=1,
    time=ovstage.population.DEFAULT_TIME,
    descs=physics_population_descs(rendering=True),
)
stage.advance_write_floor(ordinal=1).wait()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

physx.step_sync(1 / 60)  # work before recording is allowed
physx.start_recording(
    OmniPvdDestination.tcp("127.0.0.1", 5425, timeout_ms=3000)
)
assert physx.is_recording()
physx.step_sync(1 / 60)  # streamed to the TCP listener
physx.stop_recording()

physx.step_sync(1 / 60)  # simulation continues without recording
physx.start_recording(OmniPvdDestination.file("capture.ovd"))
physx.step_sync(1 / 60)  # written to capture.ovd
physx.stop_recording()

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
```

Receiver-driven selection uses the attachment that the recording owner has when the recording starts. An instance without its own attachment can start an ordinary TCP recording while a peer is attached, but no receiver selection request is applied in that session, even after the owner attaches a stage. Such requests are acknowledged as denied (`4`) unless they are invalid (`2`) or busy (`3`). To use receiver selection, attach before starting the recording, or stop and restart it after attaching.

### Receiver-selected TCP discovery in Python

Use `start_recording(..., wait_for_selection=True)` when the receiver should choose what to record before
the first detailed snapshot. It opens an owned TCP session and publishes a
lightweight object catalog while waiting for the receiver's selection. It
requires recording capability and this instance's own live attachment. Start
a receiver that supports TCP selection control first.

With `physx` already attached as above:

```python
from ovphysx import OmniPvdDestination

physx.start_recording(
    OmniPvdDestination.tcp("127.0.0.1", 5425, timeout_ms=3000),
    wait_for_selection=True,
)
assert physx.is_recording()  # This instance owns the waiting session.
physx.step_sync(1 / 60)
# The receiver can select objects and data through the TCP control protocol.
physx.stop_recording()  # Stops either a waiting or a sampling session.
```

The optional `filter` accepts the same scene and object paths as `start_recording()`.
Discovery ignores its `record_secondary_data`: each receiver selection chooses
whether secondary data is recorded. A missing filter or empty lists select all supported objects in
this instance's attachment. Scene/object lists bound the maximum discovery and
future recording scope; receiver selection cannot widen them. An empty receiver request selects the scenes
and objects of that scope that still exist. Discovery requires a TCP
destination; passing a FILE destination raises `ValueError` before calling C.
Use `is_recording()` to query session ownership: it is also `True` while waiting
for receiver selection, and is `False` for peer instances.

#### C and C++

To adapt the complete C++ sample in [Code](#code), keep its initialization,
stage attachment, and cleanup. The sample already enables
recording capability and starts recording explicitly. Keep this configuration
when replacing its FILE destination and recording loop with the TCP sequence
below. Use `create_recording_instance()` for instance creation, checking its
returned status. Remove the sample's final file-count check; a TCP session
writes no local file:

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>

static ovphysx_result_t create_recording_instance(ovphysx_instance_handle_t* handle)
{
    const ovphysx_result_t configured = ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE, ovphysx_config_value_bool(true));
    if (configured.status != OVPHYSX_API_SUCCESS)
        return configured;
    return ovphysx_create_instance(handle);
}
```

Call this helper after `ovphysx_initialize()` and before creating any instance.
After it succeeds and a physics stage is attached,
replace the sample's FILE recording sequence with `record_tcp(handle)`,
then perform the sample's cleanup and return its result.
The preliminary step demonstrates starting after simulation has already begun;
omit it to record from the first simulation frame:

```c
#include <ovphysx/ovphysx.h>

static int record_tcp(ovphysx_instance_handle_t handle)
{
    const ovphysx_omnipvd_destination_t destination = {
        OVPHYSX_OMNIPVD_TRANSPORT_TCP,
        OVPHYSX_LITERAL(""),
        OVPHYSX_LITERAL("127.0.0.1"),
        5425,
        3000,
    };

    ovphysx_result_t result = ovphysx_step_sync(handle, 1.0f / 60.0f);
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;

    result = ovphysx_ovd_start_recording(handle, &destination, NULL);
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;

    bool recording = false;
    result = ovphysx_ovd_is_recording(handle, &recording);
    if (result.status != OVPHYSX_API_SUCCESS || !recording)
        return 1;

    result = ovphysx_step_sync(handle, 1.0f / 60.0f);
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;

    result = ovphysx_ovd_stop_recording(handle);
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;
    return 0;
}
```

If the listener is not ready, `start_recording()` fails without consuming the
session; start the listener and retry. After `stop_recording()`, the same
instance can later record to another TCP listener or to a FILE path.

FILE selects an exact filename or a timestamped directory capture by suffix,
and TCP connects synchronously to an
already-ready listener. Failed opens can be retried. Only one stream can be
active in the shared runtime, and an active stream cannot be replaced. After
stop, the owning instance can start another FILE or TCP session. The instance
that explicitly starts recording owns it; peers report inactive and cannot stop
its session.

While recording is active:

- Attach and detach wait for all instance work and finish before returning.
- Task results remain available through `wait_task()` or `wait_all()`.
- Detaching the recording owner, or the last live attachment, tries to finish
  the recording.
- Detaching a non-owner keeps the recording alive while another attachment
  remains.

Call `stop_recording()` before detach so recording errors are returned to you.
Finalization failures from implicit detach are only logged. Reattach never
starts a new recording. Pending companion preview results reject detach; fetch
or close the preview and retry.

Serialize calls on each handle. Different instances may be attached, detached
and destroyed concurrently; the SDK serializes their lifecycle work internally.
If a destroy overlaps another thread's destroy, recording start or stop, or
detach during a recording, one call may return the retryable
`OVPHYSX_API_INVALID_STATE`. Retry after the competing transition finishes.
Recording is supported on Windows x86_64 and Linux x86_64/aarch64.
Initial and restarted sessions each capture the current core PhysX,
PhysXExtensions (including joints and custom geometry), and PhysXVehicle state.
Stopping releases only the session's telemetry handles; the next start takes a
fresh full-state snapshot.

## What Happens at Runtime

1. FILE prepares missing directories and opens its final exact or timestamped
   filename, or TCP connects, before sampling begins.
2. Start snapshots the current world; each subsequent step records its state.
3. Stop finalizes output. Another recording can start on the same capable runtime.

## Inspecting .ovd Files in Kit

Before you begin, stop the FILE recording to finalize its output. Inspect the
exact requested filename or the new timestamped `.ovd` inside the selected
directory. If output is missing or incomplete, check the runtime log and verify
that the path is writable and no regular file blocks a required directory.
You also need the compatible
Kit-based application with the OmniPVD extension named in the prerequisites.

Then inspect the recording:

1. Open a compatible Kit-based application (for example, USD Composer or Isaac Sim with a compatible OmniPVD extension generation).
2. Enable the **OmniPVD** extension (`omni.physx.pvd`) from **Window > Extensions**.
3. Use **File > Open** or the OmniPVD panel to load the `.ovd` file.
4. Use the timeline scrubber to step through recorded frames and inspect shapes, contacts, and solver state.

Inspection succeeded when the OmniPVD panel lists your `.ovd` file as imported
and the timeline scrubber advances through recorded frames, with shapes,
contacts, and solver state updating as you scrub. A file the panel refuses to
import can be unreadable, malformed, truncated, or incompatible with the reader.
Check the reader's reported error first. If it reports a format or version
mismatch, refer to [Capture Format and Compatibility](#capture-format-and-compatibility)
for the two version checks the reader applies.

For more details on the Kit-side OmniPVD workflow, refer to the PhysX Visual Debugger documentation included with the Kit application you use for inspection.

## Capture Format and Compatibility

ovphysx OmniPVD capture is optional. When enabled, the runtime writes PhysX OmniPVD
recordings in the **OVD** binary format. FILE opens its final exact or timestamped
filename at start and closes it at stop.

**OVD capture format and compatibility policy**

This table records the format, version, and platform policy for ovphysx OmniPVD
capture:

| Item | Policy |
|---|---|
| Format | PhysX OmniPVD OVD command stream (not USD, not a general capture/replay API) |
| OmniPVD stream version | Each recording begins with a 12-byte OmniPVD version header. The writer in the PhysX SDK pinned by this ovphysx build emits **0.4.0** (`OMNI_PVD_VERSION_*` in `OmniPvdDefines.h`). |
| PhysX OVD integration version | The stream's `PxOmniPvdMetaData` object independently identifies the PhysX object schema using `PX_PHYSICS_OVD_INTEGRATION_VERSION_MAJOR` and `PX_PHYSICS_OVD_INTEGRATION_VERSION_MINOR` in the PhysX SDK's `PxOmniPvd.h`. Read these macros from the SDK used by this build. Integration-major changes can remove or change existing schema; integration-minor changes are additive. This version is separate from the 0.4.0 command-stream version. |
| Canonical reader | Kit OmniPVD extension `omni.physx.pvd` (import OVD → cached USDA stage for scrubbing / inspection) |
| Reader compatibility | Both versions must be compatible. The OmniPVD runtime reader rejects a command-stream version newer than its own `(major, minor, patch)`. Separately, when the integration metadata is present, the PvdDom reader paired with this build's pinned PhysX SDK accepts integration major 3 or older and rejects a stream with a newer integration major; it treats missing metadata as legacy. Integration-minor changes are additive, and this reader does not reject solely because the stream minor is newer. |
| ovphysx version coupling | ovphysx does not define a separate OVD version. Compatibility follows the PhysX / OmniPVD runtime pinned by that ovphysx build and the Kit OmniPVD extension that reads the file. Prefer matching generations: inspect with a Kit build whose OmniPVD extension supports both the recorded OmniPVD stream version and the PhysX OVD integration major. |
| Platform | OmniPVD recording is supported on Windows x86_64 and Linux x86_64/aarch64. PVDRuntime is statically linked into the shipped native library; no separate PVDRuntime `.dll`/`.so` is deployed. |

This is the public compatibility story for ovphysx OmniPVD capture. If you need a
guaranteed cross-release matrix beyond the two Reader compatibility checks in
[Capture Format and Compatibility](#capture-format-and-compatibility), confirm
with the OmniPVD owners before treating an older Kit reader as supported against
a newer writer.

## Troubleshooting

These symptoms cover the configuration and lifecycle mistakes that stop a
recording from appearing:

**OmniPVD recording symptoms and fixes**

| Symptom | Cause | Fix |
|---|---|---|
| No `.ovd` file after simulation | Capability or explicit start was omitted | Set `OVD_RECORDING_CAPABLE` before creation, attach the scene, and start a FILE recording |
| Output is incomplete | Recording was not finalized | Call `stop_recording()` before teardown |
| FILE `start_recording()` fails ("failed to open the destination or start sampling") | Path is not writable, a regular file blocks a required directory, or an existing `.ovd` directory was selected as an exact file | Fix the blocked path or permissions; add a trailing separator for a directory ending in `.ovd`, then retry |
| TCP start fails | Listener was not ready or address/port was wrong | Start the trusted plaintext listener before calling `start_recording()`, then retry |

## Result

After this tutorial you can stream live OmniPVD data over TCP or capture `.ovd`
recordings from an ovphysx simulation for offline inspection in Kit.
