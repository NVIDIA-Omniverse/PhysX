<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovphysx companion

Optional authoring, simulation tooling and native extension services above
ovphysx. Implemented services include borrowed native sessions and scene statistics,
stopped primitive/closed-mesh mass computation, nearest vertex matching, cancellable mass jobs,
customer-USD edit sessions, attachment authoring and mapped CPU deformable writeback.
Inspector has an independent native CPU attachment and customer-USD snapshot/commit helpers.
The optional C++ SDK directly exposes the runtime custom joint/geometry interfaces
through `custom_interfaces.h`; ordinary main headers require no PhysX SDK.
The optional compiled-module loader has spring/ball samples, owner-safe
parameter copying, live joint controls and safe retirement. CPU collision cooking,
volume tetrahedralization, compatible caches, follow/drone cameras and copied
stopped descriptions are implemented. Native CPU capsule controllers and copied
debug geometry have runnable extension-loop samples. OVD recording coordination,
bounded native decoding and reversible customer-USD data conversion are implemented.
Vehicle support includes stopped calculations, CPU drive/wheel telemetry, host input
filtering/auto-reverse and reversible customer-USD suspension authoring. The [operation matrix](docs/compatibility.md) declares supported CPU operations
and explicit mode exclusions. Copied event queues deliver contact, trigger,
step, object and typed error records on the host thread, with explicit overflow
and disposal contracts. Customer-USD mass capture and reversible application
complete the stopped mass editor workflow.

Start with the [runnable extension-flow samples](samples/README.md) and
[Python tests](tests/test_usd_helpers.py).

The [Kit extension migration designs](docs/extensions/README.md) map legacy
runtime and UX workflows onto these services. Their proposed host requirements
and unimplemented test specifications include additional service gaps; the
implemented CPU subset does not establish complete legacy extension parity.

- [PLC index](plc/README.md): architectural decisions, requirements and mapped
  test specifications.
- [Coverage ledger](plc/coverage.md): service responsibilities, legacy interface
  dispositions and all 28 inventoried UX extension directories.


The companion lives beside `ovruntime`, under `ovphysx/companion`. Public
headers, facades, bindings and Python utilities belong here. Reusable
physics computation stays in ovruntime; the minimal service-acquisition bridge
is compiled into the existing ovphysx runtime owner. One loaded runtime can
host several native attachments: the Inspector service owns a separate
preview attachment beside playback. The public ovphysx facade still allows one
live facade attachment, so the companion bridge/lifetime integration is required.
A separate folder does not imply a second runtime library.

C++ computes and returns typed data. Optional Python utilities use the
customer-installed USD to author attachments, deformables and other physics
schemas and to write simulation output. Kit owns widgets, timeline policy,
selection, input, commands and undo.

The Python package lives in `python/ovphysx_companion/`; it is separately
installable and ships no USD or native runtime. See the sample guide for the
supported services, installation and the locked test invocation. Native headers and
facades live in `include/` and `src/`; ctypes binds the table acquired from the existing
core library. Primitive computation is implemented in ovruntime's cooking library.
The requirement dependency links identify prerequisites for each later service.

Scoped settings live in `ovphysx_companion.configuration`: a classified catalog,
typed core delegation, copied preview options, retained attachment snapshots and
idle owner-local contact policy. Optional `usd.configuration` authors scene
gravity/solver/GPU opinions with explicit source/layer and undo. See the
[settings sample](samples/scoped_settings.py).

Live CPU tools include `Inspector` (body/joint/material/articulation copies) and
`SceneQueries` (raycast, sweep, overlap) on either ToolsSession or PreviewSession.
See [the Inspector sample](samples/inspector_tools.py) and
[query contract](include/ovphysx_companion/queries.h) for identity, units, filters,
sidedness, capacity and completion semantics. Native object IDs remain opaque;
Python never receives an SDK pointer. `Inspector.path` performs explicit reverse
lookup in the selected owner. Customer-USD editing stays in the optional `usd`
package and uses copied data.

Runtime interaction borrows live playback with `ToolsSession(physics)`; only an
explicit `PreviewSession` creates an independent attachment. The viewport sample
applies forces/drag to the existing ovphysx simulation and the host steps it.

CPU interaction is available in `ovphysx_companion.interaction`: force/impulse at
a point, default native force grabbing with damping restoration, and explicit collision-aware D6 dragging with flag/velocity restoration.
`native_access.h` provides optional build-matched, synchronous read-only C++ SDK
leases. Python receives copied typed data and never runs a native pointer callback.
See `samples/viewport_drag.py` and `samples/native/read_only.h`.

`SamplingService` provides stopped CPU surface/volume Poisson sampling with copied
geometry, revision-scoped resampling and draining cancellation. Optional customer
USD helpers author selected particle sets and copied surfaces. See
`samples/particle_authoring.py` for the separate editor/worker workflow. Native
smoothing, anisotropy, isosurface and GPU sampling are not advertised.

`StageFlow` coordinates explicit sealed ovstage input and selected numeric output
without stepping. Each published output ordinal is excluded from later input
updates. `OutputNotices` independently suppresses customer-USD output notices.
See `samples/ovstage_flow.py` and `tests/test_exchange.py`.

The [migration guide](docs/migration.md) and [compatibility matrix](docs/compatibility.md)
are the implementation entrypoints. `usd.colliders.ColliderBatch` supplies cancellable
selection preparation, world-bound metrics and atomic undoable collider authoring.
`usd.preview.PreviewDisplay` owns transient Inspector output without overwriting
new authored input on cancel. Custom plugins report optional callback capabilities,
release per-object state on deletion, and support independent authored clones and
unit-scale point instances. See the corresponding tests and runnable samples.

Mouse grabbing defaults to native force picking. The host can use
`Interaction.pick(origin, direction)` and `grab.move_ray(origin, direction)`, or
supply a filtered `SceneQueries` hit to `Interaction.grab(body, point,
pick_distance=hit.distance)`. Native code applies the legacy impulse, point-velocity
damping and gravity assistance per actual substep and restores angular damping
on close. `grab.set_picking_force(value)` adjusts the per-grab coefficient, including zero, for the next substep. Close on pointer release, Pause/Stop, focus loss or input disable.
`Interaction.joint_drag(...)` explicitly selects the D6 alternative. Both use the
existing live owner; neither advances physics. See the [viewport sample](samples/viewport_drag.py).
