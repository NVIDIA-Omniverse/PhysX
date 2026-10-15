<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Extension workflow samples

These runnable, Kit-free samples exercise the same Python helpers a UX extension
can call. Statistics, primitive mass and vertex matching use real native services.
The deformable writeback sample uses mapped CPU output fixtures. Native CPU
embedding/interpolation and copied core-read output are implemented and tested
separately; actual GPU deformable extraction still needs hardware qualification.

| Workflow | Sample | Extension adapter responsibilities |
|---|---|---|
| Select two simulation meshes, match world-meter vertices in a native worker, apply pairs, undo/redo | [attachment_authoring.py](attachment_authoring.py) | Selection, worker scheduling, command history, cancellation |
| Publish completed deformable output to a preview layer, bake two time samples, reject stale work | [deformable_writeback.py](deformable_writeback.py) | Solver completion, GPU-to-CPU transfer, visual embedding, frame/unit conversion, timeline, notice filtering |
| Borrow playback, step through the host and read copied scene statistics | [live_statistics.py](live_statistics.py) | Existing core owner, timeline, step/wait order |
| Map points into triangle/tetrahedron elements and interpolate copied positions | [mesh_mapping.py](mesh_mapping.py) | Source revisions, frame/unit declarations, mapping diagnostics |
| Compute primitive compound mass in a stopped job | [mass_properties.py](mass_properties.py) | Shape/material extraction, source revision, result display/application |
| Compute stopped mesh/primitive mass from customer USD and apply overrides | [stopped_mass_editor.py](stopped_mass_editor.py) | Selection, copied worker, source revision, command history |
| Deliver copied contacts, triggers and runtime events to a graph | [event_monitor.py](event_monitor.py) | Reporting schemas, host completion, graph thread, overflow recovery |
| Cook collider arrays and retain them across cache invalidation | [cooking.py](cooking.py) | Source revision, collision approximation choice and viewport drawing |
| Generate separate simulation/collision tetrahedral meshes and author a deformable | [deformable_cooking.py](deformable_cooking.py) | Native worker, customer-USD stage, source validation and undo/redo |
| Follow completed body state and restore camera opinions | [camera_follow.py](camera_follow.py) | Native worker, input revision, explicit camera layer and enable/disable |
| Inspect stopped schema data without simulation | [stopped_inspection.py](stopped_inspection.py) | Selection, property model and stale-result handling |
| Classify asset defects and apply a selected reversible repair | [asset_validation.py](asset_validation.py) | Check selection, worker dispatch, result display and host undo |
| Control a capsule against ground/wall and release it | [character_controller.py](character_controller.py) | Input mapping, clock, gravity policy and optional camera |
| Copy debug primitives for viewport rendering | [debug_geometry.py](debug_geometry.py) | Visualization policy, host stepping and renderer thread |
| Deduplicate umbrella and PhysX stageupdate callbacks | [host_stageupdate.py](host_stageupdate.py) | One coordinator per attachment, frame IDs, input/output callbacks |
| Record shared core/Inspector state and import OVD into customer USD | [ovd_recording.py](ovd_recording.py) | Recording destination, time/unit policy and viewer presentation |
| Apply filtered host vehicle input before one native step and capture wheel telemetry | [vehicle_control.py](vehicle_control.py) | Input mapping, clock, pause routing and telemetry widgets |
| Compute sprung masses/suspension frames and apply through customer USD | [vehicle_authoring.py](vehicle_authoring.py) | Component selection, source revision, native worker and undo/redo |

## Native contract

The companion package requires ABI version 1 of the bridge exported by this feature
build of ovphysx. It does not load another simulator library. `ToolsSession(physics)`
retains its owner; close it before `physics.destroy()`. Destruction/shutdown refuses
live leases and can be retried after closing sessions. Detach makes old sessions
stale; acquire a new session after reattachment. Serialize live operations with
core operations on that instance. Scene counters are copied at an idle owner.

Population examples and custom schema/vehicle descriptors use ALL domains with
OVStage 0.3 so parent transforms compose into world matrices before sealing.
Hosts choosing PHYSICS-only follow the core transform preparation contract;
direct world-matrix producers retain their authored values. Physics snapshot
capture still excludes unrelated render payload.

`compute_mass_properties(library, shapes)` is a stopped, SI computation for boxes,
spheres and X-axis capsules. Volumes add even where shapes overlap. It returns kg,
m³, body-local meters and kg·m²; rotations are xyzw. It rejects invalid/degenerate
inputs and results outside normal float range. The extended `mass.compute_mass_properties`
accepts immutable `mass.Request` snapshots with closed triangular meshes and collider/body
mass, COM, inertia and principal-axis overrides. `usd.mass.capture_mass_request` resolves
collider/body/material density precedence and converts customer stage units; matching
results apply atomically through `usd.mass.apply_mass_result`. Meshes require one
connected, closed, consistently oriented, non-self-intersecting boundary; either global
winding is accepted. Unsupported approximations and transforms fail explicitly.
`AuthoringService.compute_general_mass` runs this computation on copied worker input.

`AuthoringService(library)` runs copied mass and element-mapping inputs off the timeline. `job.result()`
delivers on the calling thread; it may be repeated. Accepted cancellation yields
`CancelledError` even when native work must finish. Service close suppresses
undelivered results and waits for quiescence. Retain the source revision alongside
the job and validate it before applying results. Live core task ordering is separate.

Vertex matching uses CPU float32 XYZ data in a shared Cartesian frame. Distance is
inclusive; equal-distance ties choose the lowest target index, several source
vertices may share a target, and unmatched vertices return -1. The USD helper
converts captured geometry to world meters. No-match proposals fail before editing.
The native utility sorts target X coordinates to narrow each search; broad search
radii can still produce quadratic work. `mesh.embed_points` also computes native
triangle/tetrahedron correspondence with explicit topology revision, frame and units.
Tetrahedra return containment or closest-face projection; triangles return barycentric
weights and a signed normal offset. Lowest-element ties are deterministic. Unmatched
points remain explicit and `require_complete` rejects incomplete maps. Reference
geometry must be nondegenerate. `interpolate_elements` can interpolate constant vector
fields with zero offsets even if their values form a collapsed triangle. Exact triangle filter computation and CPU surface/volume sampling are available
through their separate services; see the attachment and particle workflows below.

## Run

Use a Python 3.10+ environment with customer-installed `pxr` and NumPy. Attachment
authoring also needs ovphysx for its authoring helpers and the separate
`physx-usd-schemas` package. Registration reuses `ovphysx.utils.codeless`; do it before
opening any stage. No compiled USD schema bindings are used. Attachment matching
runs in a child process with the native SDK; the editor process loads only customer
USD. A production extension can retain a worker and supply its own transport.
The exercised combination is Linux x86_64, Python 3.12, USD 26.8 and NumPy 2.4.2.
Other combinations are not yet qualified.

From `ovphysx/companion`, install the pure Python package with
`python -m pip install -e './python[usd]' physx-usd-schemas`. With those dependencies
already installed, you can instead set `PYTHONPATH=python`. Then run:

```bash
python samples/attachment_authoring.py
python samples/deformable_writeback.py
# Separate native processes, without customer pxr:
python samples/live_statistics.py
python samples/mass_properties.py
python samples/mesh_mapping.py
python samples/cooking.py
# Customer USD with explicit native worker:
python samples/deformable_cooking.py
```

The USD samples print the resulting layer; native samples print copied data.
All check their own expected outcomes. Nothing is
saved to disk implicitly. In a built checkout, the locked Python test environment
can run them with `PYTHONPATH=python ../tests/python_tests/.venv/bin/python` in
place of `python` after preparing that environment with the test wrapper below.

For a Linux source build, prepare the SDK with the standard build/install scripts.
From `ovphysx`, select that SDK and its matching ovstage dependency, then run the
suites as separate processes:

```bash
export OVPHYSX_LIB="$PWD/_install/lib/libovphysx.so"
export OVSTAGE_LIBRARY_PATH_HINT="$PWD/ovruntime/_build/target-deps/ovstage/bin"
export LD_LIBRARY_PATH="$PWD/_install/lib:$PWD/_install/plugins:$PWD/ovruntime/_build/target-deps/ovstage/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
tests/python_tests/run_pytest.sh --confcutdir=../../companion/tests_native ../../companion/tests_native -q -o addopts=
tests/python_tests/run_pytest.sh --confcutdir=../../companion/tests ../../companion/tests -q -o addopts=
```

The standard `cmake -P scripts/test_python_runtime.cmake` supplies platform-specific
environment setup and runs both suites separately on qualified Linux x86-64 builds.
Windows companion runs require the explicit
`-DOVPHYSX_TEST_UNQUALIFIED_COMPANION=ON` opt-in; those runs do not qualify that platform. An
installed wheel resolves its own library dependencies and needs no source paths.

The wrapper uses the existing locked dependencies. `--confcutdir` keeps the main
suite's native-library fixtures out of this customer-USD process. On Windows use
`run_pytest.bat` with the same arguments. A missing dependency/schema fails this
suite instead of silently skipping its behavioral coverage.

## Adapt to an extension

Create an `EditSession` on the host-approved USD thread, supplying the stage and
an output-notice guard when a solver-input observer is installed. Capture meshes
before submitting native computation. Mesh snapshots carry immutable points,
polygon topology, transform, length units and a session-scoped source revision.
Return data from the worker to the same thread and call the helper with the
explicit destination layer. Never pass the USD stage through a native binding.

External stage edits invalidate captured results automatically. Call
`session.invalidate()` for selection, cancellation, runtime-owner/source-remapping
or other host changes that do not produce USD notices. Close the session when the
stage/tool closes. A native attachment handle is not interchangeable with this
Python source revision; that bridge is still to be implemented.

Wrap the returned `EditPatch` in a host command. Undo/redo checks the target layer
has not changed and restores only that layer. Serialize all stage/layer writers
while a helper executes. Patches store full layer snapshots, so use small tool or
preview layers and release patches when no longer needed. This initial mechanism
is not optimized for long recordings. `RecoveryError` exposes the layer and its
`before_text` when rollback or the host notice guard fails; repair the host policy
and restore explicitly before accepting more work.

For writeback, finish the native mapping and any GPU transfer first. Supply CPU
NumPy positions in **destination-mesh local coordinates and stage length units**;
velocities use those length units per second. The helper rejects other declared
frames/units and implicit device arrays. A destination's vertex count need not
match the simulation mesh, but it must match the mapped output. `time_code=None`
authors default values; a numeric value is in stage time codes, converted to the
target layer's local time by USD. Simulation seconds are separate frame metadata.
Choose an output layer strong enough for the intended composition.

Attachment authoring supports surface/TetMesh snapshots, vertex pairs, element
correspondence, collision groups and managed disabling, preserving unrelated
properties/children on update. Instance proxies remain unsupported. The mapped
writeback helper below computes interpolation and frame conversion after the
caller supplies completed CPU data and an explicit runtime association. Device
synchronization remains the caller's responsibility. Qualification limits are recorded in the
[operation matrix](../docs/compatibility.md).

## Inspector

`inspector_preview.py` creates native preview B beside playback A, exercises
independent pending intervals and closes B without consuming A's task. Pass
`--playback-first` to retire the last core owner before B, and
`--recording /tmp/inspector.ovd` to exercise recording transitions in either order.

`inspector_authoring.py` runs in customer USD. It captures an immutable flattened
snapshot, simulates through an explicit native worker, commits only the selected
rigid-body poses and exercises undo/redo. Transform scale and original transform
opinions are preserved. Pass the appropriate installed native Python executable
to `simulate_snapshot` when the host and worker use different environments.
The worker keeps ovstage population's USD image out of the customer-USD process.
This sample covers CPU rigid-body preview. The independent PreviewSession samples
also exercise articulation telemetry and CPU raycast, overlap, and sweep queries.
Kit integration and GPU preview execution still require separate qualification.

## Deformable construction and output data

`usd.apply_deformable_body` consumes immutable simulation, collision and visual mesh
records. It authors role APIs, separate rest/bind data, material bindings and optional
visual embedding. The helper owns only its declared prims/properties. Rebuilding
preserves user fields, requires a new version when topology changes, and invalidates
old embedding when no new mapping is supplied. Role migrations are explicit host work.

`output.capture_deformable_output` reads every matching core deformable prim and
returns owned CPU NumPy arrays tagged with owner/path/step/time. The caller completes
the step first; GPU copying requires `copy_gpu=True`. `usd.map_deformable_output` checks
the runtime association, interpolates through the native kernel, and converts into
the captured destination frame before `write_simulation_output` applies a batch.
Simulation positions are local, velocities are world vectors, and units are stage
units. Triangle normal-offset velocities include the native unit-normal derivative. The transport
contract is tested with controlled reads; this host has no working GPU driver to
qualify a real deformable simulation roundtrip.

`usd.apply_element_attachment` authors vertex/triangle or vertex/tetrahedron
correspondence. `usd.apply_collision_filter` uses paired element groups and
keeps TetMesh volume and surface domains separate. `usd.remove_owned_attachment`
disables an owned attachment/filter rather than deleting user children or tuning.

## External native-plugin schemas

[external_schema_authoring.py](external_schema_authoring.py) authors a real
`CompanionSpringJoint` and `CompanionBallAPI`, then verifies their parameters in a
separate production-population process. It includes an unauthored schema default.
Run it with the same customer-USD environment as the authoring examples.

`schemas.register_customer_schemas` registers and verifies all resources before the
customer schema registry is used. `schemas.register_population_schemas` separately
registers them before ovstage population. Pass `schemas.population_description`
to `open_usd_with_desc` to retain normal physics columns and the declared external
parameters. Registering a plugin in customer USD cannot register it in the worker.
These helpers establish the data path for the native solver/contact registration
and per-object immutable parameter records in the compiled samples below.

## Native joint and geometry extensions

Build and run the [compiled plugin samples](native/README.md) using
[native_extensions.py](native_extensions.py). The spring and ball consume real
external schema parameters, execute native solver/contact/query callbacks, and
retire only after their owners release every object. Python handles registration,
safe joint flags/refresh and optional customer-USD authoring. v1 loading requires
hard CPU mode; GPU/DirectGPU qualification remains open.

## Cooking and cache ownership

`CookingService(physics)` retains the existing runtime independently of its bootstrap
core. It creates no simulation attachment. Its `capabilities` distinguish seven CPU
operations: triangle, convex, decomposition, SDF, sphere filling, conforming tetrahedra
and voxel tetrahedra. Copied-data dispatch runs beside unrelated pending owners.
Callback reentry and exclusive recording transport transitions return Busy;
the runtime owns authoring admission, and the host schedules the worker.

`cook(...).result()` returns immutable copied arrays and SDK-specific blobs. Triangle
arrays for unsimplified triangles include the original polygon for each triangle.
Simplified triangle, convex/decomposition and SDF recipes expose blobs without a
guaranteed cooked surface; `collision_overlay` rejects missing surface data. Tetrahedral recipes declare
their frame/units and preserve input length units. Cache identity includes geometry,
topology, normalized parameters and the SDK fingerprint. `invalidate(key)` affects only
that service's recipe; retained results stay valid after eviction or service close.
`save` / `load` use checksummed data archives and reject incompatible builds or recipes.
Job cancellation suppresses results; close waits for any running native kernel.

`cook_volume` composes a conforming collision mesh, voxel simulation mesh and visual
embedding. Each topology has its own revision. `require_complete()` rejects unmatched
or projected outside visual vertices before authoring. The sample authors real cooked
meshes with distinct vertex counts through customer USD; it does not claim a GPU
deformable simulation test on this CPU-only host.

## Camera and stopped inspection

`capture_body_output` uses completed core read sessions and returns copied world
pose/velocity records. The host supplies the sealed input revision, stage length
scale and completed step/time; GPU copies require explicit opt-in. `FollowCamera`
uses body-local meter offsets and velocity look-ahead. `DroneCamera` consumes
host-provided local velocity and yaw/pitch input without taking keyboard/timeline
ownership. Missing/recreated targets disable following; repeated/older steps hold.

`usd.camera.CameraOutput` writes to an explicit customer-USD layer and converts
world meters through the camera's parent transform. Restore mode requires exclusive
ownership of that layer until close; conflicting edits fail instead of being lost.
`FollowCameraOutput` restores on target loss, disable or edit-session close.
The sample reads a real native trajectory in a worker, publishes frames in the
customer-USD process, then restores the original camera opinions.

`usd.descriptions.capture_descriptions` produces immutable shape, joint,
articulation-root, tendon, vehicle/wheel and particle input descriptions. Public
schema parameters include defaults and authored provenance; relationships,
transforms, instance paths and length/mass units are copied. Queries on retained
snapshots import no USD and create no runtime. These describe authored input,
not solver realization. Validate the source revision before using a retained
model to apply an edit.

## Classified validation

`validation.CHECKS` declares each check's dependency class. Pure checks consume
immutable descriptions. Mesh cooking checks require an explicit CookingService;
Busy/missing capabilities make `Report.complete` and `Report.valid` false rather
than declaring the asset valid. `ValidationPreview` uses a distinct native
attachment and offers a bounded motion check without consuming playback work.
`usd.validation.apply_repairs` takes selected diagnostic indices, validates the
source and every destination, and returns a reversible edit patch. No checker
silently applies a repair. Articulation-state and compatibility checks are available through the
explicit validation facade; GPU-convex predicates remain limited by the
advertised geometry/backend qualification.

## Character controllers

`CharacterController` borrows a ToolsSession or PreviewSession and controls one
ovstage-created capsule. Populate PHYSICS and RENDERING: the physics domain alone
omits capsules carrying only PhysxCharacterControllerAPI. Movement uses world
stage length units and seconds; copied collision bits distinguish side/above/below.
The explicit query mask selects static, dynamic and other controller collisions.
Simulation collision-group filtering is not implicitly applied. Call while idle,
then let the host step/fetch. Optional Python gravity adds acceleration before
native collision resolution; it does not own input devices or the clock.

Close removes only native controller state. A source repopulation or reattachment
can recreate it with a new generation. Stale commands cannot act on the replacement.
Close controllers before their sessions; cleanup after owner retirement is safe.
Teleport/resize do not check overlaps; resize preserves the feet. CPU capsule
support is advertised; GPU scenes are unavailable in this table version.

## Debug geometry

`DebugDraw` controls visualization on one existing owner. Configure known parameter
bits, scale and optional world-stage culling bounds while idle, then capture after
host step completion. Scale multiplies the attachment viewport gizmo scale; settings
apply on the next completed step. Frames carry native attachment/step identity,
world-stage vertices and packed AARRGGBB colors. Points/lines/triangles are immutable
Python tuples and can be queued to a renderer after native owner shutdown.

Capture copies PhysX's existing debug buffer. Actor/shape wireframes use poses from
the start of each scene's latest simulation step (its last substep when substepping),
so they can trail bodies rendered at post-step poses. Contact and friction primitives
describe contacts computed for that step. `frame.step` is the owner's native execution
counter at capture, including internal substeps and scenes; it is not a host-frame
number or a promise that all vertices use post-step poses.

A frame exceeding max_vertices raises SnapshotLimitError without partial output.
Pending calls return Busy. The host exclusively owns the visualization policy;
close disables it rather than restoring unrelated settings. Existing selection
scope is respected. CPU extraction is advertised; export/PVD connection and GPU
telemetry are separate optional capabilities not supplied by this table.

## Host coordination

`FrameCoordinator` binds a host update loop to one attachment and creating thread.
Pass that same coordinator to both legacy stageupdate entry points: matching frame
IDs reuse the completed record and do not repeat callbacks or simulation. Pause
stops new work. Callback failures latch/pause the loop; repair/reset requires
explicit adapter retirement and reconstruction. Close does not detach a borrowed
owner. All host callers must honor this election; direct external C calls are not
intercepted. Inspector uses another coordinator and attachment.

`DeferredEdit` delivers a copied job result only on the customer-USD thread after
validating the input revision. Host cancellation suppresses publication even if
the native job already completed. The apply callback uses a shared USD helper and
returns its patch for host undo/redo. The authoring service still owns worker drain.
`cpu_tool_availability` negotiates CCT/debug/vehicle tables and requires the qualified hard
CPU mode. Missing actions have a specific reason suitable for disabled UI controls.

## OVD data import

`RecordingCoordinator` delegates to the core's process recording capability;
call `set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)` before the
first `PhysX()`. This bool defaults to false and stays dormant until explicit
recording start. Writes freeze after the first successful instance creation
until full successful shutdown with no live instances; the configured value is
retained. Create and attach remain dormant. Start before the first simulation
step to capture it. Pending Inspector work blocks
recording transitions until its host fetches it. Inspector close does not finalize
the core-owned capture. The native decoder needs only the already loaded library,
not a live runtime/attachment. Inputs, records, strings and payloads have explicit
bounds; malformed commands report a byte offset and publish no partial document.

`usd.ovd.convert` creates a generic data stage in the selected customer layer.
It preserves schema, typed values, list changes, messages and raw frame ticks;
each context/handle/lifetime gets a distinct prim with visibility and rigid pose
samples. Conversion uses command-index time by default because scene-local OVD
ticks are not seconds. A host can supply a monotone mapping and use frame markers
for playback controls. `object_at` resolves handles at explicit playback times.
Recording meters-per-unit is explicit; a reset root scales to existing stage units.
Unmatched destruction is diagnosed; it does not invent an object. This data importer
does not reconstruct collider meshes or own viewer UI. The sample demonstrates
native recording/decoding in a worker and customer-USD conversion with undo/redo.

## Vehicles

`VehicleMath` uses the retained library for stopped sprung masses (COM-relative
wheel coordinates), rigid suspension-frame transforms and Ackermann steering.
No physics instance or stage is initialized. Inputs use meters/kilograms/radians;
quaternions are XYZW. The authoring sample runs native computation in a worker,
then converts the copied results into the customer's stage units in a reversible
USD batch. Suspension frames mean maximum compression and explicitly select
actor-local or COM-local coordinates. Shared suspension components require equal
per-wheel masses; split them before assigning different values. Scaled/sheared
vehicle transforms are rejected by this authoring helper.

`vehicle.population_desc()` adds the suspension reference-frame metadata to the
normal physics domain. Pass it to `open_usd_with_desc`; otherwise absent metadata
keeps the runtime's legacy COM-frame default. Complete vehicle replacement must
republish shared wheel/tire/suspension/drive dependencies too; an isolated vehicle
subtree is not sufficient for every runtime source backend.

`VehicleController` controls one managed CPU direct/engine vehicle through a
ToolsSession or PreviewSession. The host calls pre_step once before its own step;
FrameCoordinator.before_step is the sample integration point. Optional response
rate limits normalized input changes per second. Auto-reverse converts braking at
low speed into reverse throttle and brakes an already reversing vehicle before
selecting first-forward gear. Its threshold uses stage length units per second.
Auto-reverse takes ownership of gear selection; manual input preserves the existing
transmission unless an explicit direction is supplied. No input device is polled.

Telemetry copies native engine/wheel values with attachment, record generation
and completed-step identity. Ground fields and engine fields are None when absent;
unmapped ground objects also have None identities. Native object-record changes
invalidate cached ground contacts until the next vehicle update, without
dereferencing retired actor/shape/material pointers. Wheel pose frame is explicit;
forces use stage mass-times-length per second squared, angles use radians. Applied
input fields report the current latch while dynamic values describe the last
completed native step. Copies remain usable after owner retirement.

Pause, disable and close clear throttle, both brakes and steer, preserving gear,
body pose and velocity. The host routes timeline pause to both vehicle and clock
adapters. Busy operations do not advance Python filtering and close can be retried
after pending work completes. Replacement invalidates the old generation; create
a new controller. Raw-wheel/tank controllers and GPU scenes are unavailable from
this table; ordinary core typed wheel controls remain separate.

### Scoped settings panel

`scoped_settings.py` constructs an Inspector preview using copied thread, pipeline
and solver options, reads retained configuration and applies an idle owner-local
contact policy. `CATALOG` supplies scope, default policy, mutability phase and
persistence ownership for each exposed field. Host preferences use a caller-owned
mapping and never mutate native settings. Process/global changes are explicit
calls to the existing typed core API; they are never temporary Inspector setup.

Run `scoped_settings.py --author` in a separate customer-USD process for scene
gravity (world meters/sec²), solver and GPU opinions with undo/redo. The host must
explicitly republish and recreate for construction-time scene changes. Snapshots
report captured resource requests, not successful GPU realization. The helper
never returns dispatcher/CUDA pointers or transfers their lifetime to Python.

## Copied event delivery

`EventQueue(owner)` accepts a borrowed playback session or independent Inspector
preview. Subscribe before stepping, author contact/trigger reporting explicitly,
then poll after owner completion on the queue's creating host thread. A graph
adapter can use `queue.dispatch(handler)` from the frame coordinator's after-step
hook; Python never executes on a native callback thread.

Records carry exact attachment, conservative record epoch, observed FIFO sequence,
step/substep and production phase. Contacts include body/collider/material keys,
point-instancer indices, impulses, normals and contact points in scene units.
Trigger enter/leave and derived stay preserve distinct instance pairs. Stays only
cover enters observed by this queue; late subscription cannot discover preexisting
overlaps. Lifecycle records carry native object indices, never pointers. Process
error records have attachment zero because the native stream cannot identify a
reliable originating owner.

Overflow is sticky and raises `EventOverflow`; its retained batch is diagnostic
only. Pause the consumer, close the queue, explicitly rebuild/reset the owner and
subscribe before resuming. `allow_incomplete=True` permits diagnostic inspection,
not recovery. Closing discards queued records and stops further delivery, including
close from a host handler. Copied batches survive owner/queue disposal. CPU contact
and trigger queues are supported; GPU/DirectGPU contact/trigger queues report unavailable.

## Retaining CPU arrays for a viewport

Use the copied arrays returned by cooking, inspection, debug and output services.
Each result documents its frame, units, producing identity and array layout; retained
copies survive later simulation, service close and cache invalidation. Device inputs
are rejected before conversion. Core DLPack consumers use core read sessions and
completion/transfer explicitly; no generic buffer wrapper is needed.

### Inspector selection and scene queries

Run `inspector_tools.py` in the native worker environment. It resolves a viewport
pick, reads mass/inertia and local/world joint frames, lists nearby bodies and
copies articulation DOF telemetry from an independent PreviewSession. The same
`SceneQueries` / `Inspector` accept a borrowed ToolsSession for playback. The host
completes its selected owner before reading; neither facade steps or waits.

`queries.Geometry` supports sphere, box, capsule and an existing cooked convex
shape at an explicit pose. Use `Mode.ALL` plus capacity to obtain a sorted bounded
set with `total` and `truncated`; `Mode.ANY` is an existence test. Include/exclude
filters take resolved body/shape references, allowing individual point instances.
GPU query/inspection and triangle/compound query geometry report unavailable.

`viewport_drag.py` demonstrates the viewport gesture lifecycle: ray pick, default CPU force grab (explicit D6 remains available),
point spring, host-owned stepping against a collision wall and cancel with zero
velocity. `native/read_only.h` demonstrates exceptional read-only C++ SDK access;
Python consumers use `Inspector.read` instead of pointer callbacks.

`particle_authoring.py` captures a triangulated source in customer USD, sends a
copied request to an isolated native sampling worker, applies a selected particle
set through an explicit edit layer and demonstrates undo/redo. Surface/volume
sampling is CPU; optional smoothing/anisotropy/isosurface are capability-gated.

`ovstage_flow.py` demonstrates authored input sealing/consumption, host-owned core
stepping, and explicit publication of copied world-position data. Its output
ordinal is excluded from subsequent input. For customer USD mirrors, separately
use `OutputNotices` with `EditSession.output_guard`; ordinals alone do not filter
USD notices. `OutputColumn` also supports copied ragged arrays with explicit lanes.

`attachment_proposals.py` demonstrates static mesh extraction, native stopped
triangle/tetrahedron binding and exact surface collision-filter proposals in a
worker, followed by owned customer-USD edits and undo/redo. It recaptures inputs
between authored commands. `usd.attachments.match_transform_attachment` and
`apply_transform_attachment` bind selected vertices to rigid transform frames;
see the affine-anchor fixture in `tests/test_attachment_elements.py`.

## Migration and host commands

Use [the migration guide](../docs/migration.md) to match legacy helpers/interfaces
to services and [the compatibility matrix](../docs/compatibility.md) to gate actions.
`batch_colliders.py` demonstrates explicit selection, bounded progress, cancellation,
world-bound metrics, meter-to-stage contact offsets and a single atomic undo patch.
It reuses the existing core collider helper and never initializes simulation.
`inspector_authoring.py` also shows `PreviewDisplay`: temporary output is cleared
on cancel before a fresh selected commit, preserving newer authored input.
`debug.collision_overlay` supplies stopped cooked triangles, per-shape colors and
transformed normals; `debug.export_availability` reports unsupported diagnostic
exports separately from core OVD recording.


## Host contracts and qualification limits

`CompletedFrame.step` and its time are local to the coordinator since creation.
Native Inspector and DebugDraw counters reflect native execution, including prior
steps and internal substeps. Compare owner identity and use the host's recorded
completion mapping; do not equate the counters.

`Interaction.pick` chooses the closest eligible dynamic body. Static colliders
are excluded and can therefore lie in front of the selected body. Hosts requiring
occlusion use `SceneQueries.raycast` with both static and dynamic enabled, then
explicitly grab an eligible closest hit, or apply their visibility filter.

`PreviewSession.update` consumes a sealed input range; it does not step the solver
or promise immediate effective mass values. Rebuild a stopped Inspector preview
when an edit requires immediate native mass realization. A fresh invalid ordinal
range returns stale-input status without retiring the live preview; retired owners
have separate absent-owner results.

Stopped mass uses supported geometric colliders and excludes disabled collision.
It differs from live Inspector effective mass where runtime chooses convex/fallback
approximations or counts disabled collision shapes. Compare against live Inspector
only for equivalent geometry, shape selection and authored overrides. Particle
sampling requires positive closed shells; nested shells representing cavities are
unsupported and must not be presented as filled-solid sampling.

Physics snapshots preserve typed particle input and authored particle API metadata,
including unknown APIs, as data. Customers and the chosen native worker must supply
matching schema registration. These transport guarantees do not qualify particle
or deformable GPU preview execution.

Deformable output records carry native identity, path and completion information,
not a source revision or topology generation. Retain the matching sealed input and
mapping, validate them before applying output, and invalidate them after deletion
or recreation at the same path. Rigid output capture accepts scalar body groups;
select a pattern excluding PointInstancers and handle their array output through
instance-aware reads.

Event dispatch consumes its polled batch before handlers run. An exception is
fail-stop and the remaining tail is not requeued. Hosts needing retry own the batch
and delivery cursor by calling `poll()` themselves.

Recording transport and runtime lifecycle share process-wide admission with native
companion operations. An unrelated cook can make recording or destroy return Busy;
complete it and retry. A rejected explicit `start_recording` leaves the attachment
valid and recording inactive; retry after pending work completes.

Completed body/deformable capture uses core `read_sync`, which waits for its
query, read session and group fetches before returning the copied output.
