<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Compatibility and operation availability

The companion has no separate version. Its Python package and the core it is
qualified with share this branch's `ovphysx/VERSION`.
The separately installed Python package contains no native runtime or USD.
Compatibility is negotiated against the loaded library: each service table
requires ABI **1** and its exact structure size before resource acquisition.
An absent symbol reports `UNAVAILABLE`; a different layout/version reports
`ABI_MISMATCH`. A core package version alone does not advertise companion services.
[The machine-readable declaration](../python/ovphysx_companion/compatibility.py)
and [all-table canary tests](../tests_native/test_compatibility.py) cover this pairing.
C++ native plugins additionally match the installed SDK fingerprint, platform,
compiler/runtime and calling convention. Rebuild plugins with the consuming SDK.

This implementation is qualified on Linux x86_64, Python 3.12, customer
`usd-core==26.8` (`Usd.GetVersion() == (0,26,8)`) and the codeless PhysX schemas
shipped with that core SDK. Other platforms are build targets, not tested platform
claims. USD compatibility is feature/schema based: missing modules, required API
entrypoints, TetMesh support or required codeless definitions produce dependency
errors before publication. The helper-specific schema/property checks remain
necessary even with the same USD version. Do not initialize the schema registry
before registering external codeless resources.

Customer USD and ovstage's population USD are separate registrations. Never load
two USD images in one process. Use the editor/worker samples to transfer copied
geometry, explicit paths, units, revisions and mappings between processes. The
native companion and shipped runtime are USD-free; ovstage population is an
application dependency. No helper auto-installs or substitutes the customer's USD.

## Operation matrix

“CPU” below describes the operation's execution and buffers, not permission to
read an arbitrary scene's GPU state. All live operations require a valid explicit
owner and an idle/completed phase unless their header states otherwise. Runtime
capability tables and typed errors remain authoritative.

| Operations | CPU behavior | GPU / DirectGPU contract |
|---|---|---|
| Borrow session, owner identity, completed statistics | Existing owner; copied counters, no attach/step | No GPU-only statistics advertised; not a GPU synchronization primitive |
| Primitive/mesh mass, point matching, triangle filters | Stopped geometry; copied Cartesian data | CPU operations; pass ready host input, no implicit device-pointer access |
| Collision cooking: convex, triangle/simplified triangle, decomposition, SDF, sphere fill | Separate recipe bits; copied topology/blobs | CPU cook only; an SDF cook is not a custom-geometry SDF fallback |
| Conforming/voxel tetrahedral cooking | Separate cook kind bits and topology versions | CPU cook only; does not advertise a deformable simulation backend |
| Triangle/tet embedding, position and velocity interpolation | Separate mesh/motion tables; copied results and explicit frames/domains | CPU interpolation; GPU arrays require explicit completion and transfer before USD |
| Surface/volume particle sampling | Separate sampling bits; cancellable copied-input jobs | Smoothing, anisotropy, isosurface and GPU sampling unavailable |
| Preview/validation attachment | Independent native owner, resources and ordinals | Qualified hard CPU mode; explicit GPU preview rejected under hard CPU policy; GPU qualification absent |
| Typed body/joint/material/articulation inspection | Copied records and generation-aware references | Live service rejects unsupported pipeline; no raw GPU/SDK addresses |
| Raycast, sweep, overlap | CPU queries with declared geometry/filters/capacity | No DirectGPU query contract advertised |
| Force/impulse, torque, wake/sleep, default force grab and explicit D6 drag | CPU live owner; exact instances; restoration on retirement; force grabs also support links with preserve velocity | GPU/DirectGPU actions unavailable |
| Native CCT, vehicle controls and telemetry | CPU service bits, host input policy | No GPU/DirectGPU controller capability advertised |
| Follow/drone camera | Copied CPU state; optional USD publication | Backend independent after explicit state transfer |
| Event queues | Copied bounded owner/step/object/contact/error events | No automatic device-memory delivery; configured event support still applies |
| Debug points/lines/triangles and stopped overlays/normals | Completed copied frames, culling and declared units | Live CPU draw; RepX and legacy PVD unavailable separately from OVD |
| OVD recording/decode/conversion | Core recording coordination; stopped CPU decoder; Python USD conversion | Decoder is CPU; recording does not grant GPU inspection or PVD connectivity |
| Settings and StageFlow | Classified settings, sealed inputs, explicit output columns | Setting a requested pipeline does not qualify a provider; ready CPU publication only |
| Custom joint | CPU rigid constraints, refresh/dirty/flags, native callbacks | GPU/DirectGPU, articulation/tensor DOFs unavailable; articulation actors rejected before create |
| Custom geometry | CPU rigid contacts/mass/bounds; optional raycast/overlap/sweep/debug bits | GPU-only particle/deformable pairs, DirectGPU collision data, custom SDF fallback and mesh merge unavailable |
| Python USD authoring/writeback and collider batches | Customer USD, explicit source/layer/time, reversible edits | No simulation initialization; device data must already be ready on CPU |

Custom plugins support independent authored clones and unit-scale point instances.
A custom point-instancer prototype receives per-instance shapes, including mixed
ordinary/custom compounds. Non-unit instance scale is rejected before plugin create.
Geometry parameter changes require explicit recreation with a new sealed input;
joint refresh is a separate safe-phase call. SDK binary replication of custom
callback state is not advertised: use authored ovstage clones/repopulation. Optional
geometry bits reflect actual callback pointers, not merely table acquisition.

`Status.BUSY` is retryable admission failure; `NOT_FOUND`/`STALE` identify invalid
owner/object generations. Accepted asynchronous execution failure belongs to the
job/task outcome. Neither is an absent capability. User code should handle typed
status, not match native log strings.

## Additional-platform qualification

The required companion integration suites currently run on Linux x86_64. On
Windows and Linux aarch64 they are explicitly unqualified, rather than passing
by exclusion; `OVPHYSX_TEST_UNQUALIFIED_COMPANION=ON` enables the same suites for
qualification. Core and runtime tests remain required on their supported platforms.
Windows native plugin compiler/CRT descriptor compatibility and direct worker DLL
dependency discovery must be qualified before that companion gate is enabled.

Copied-data cooking and sampling share authoring admission only with recording
transport changes. They do not acquire playback/preview execution scopes or the
attachment configuration lock. Cancellation suppresses publication and drain waits
for the bounded kernel to return. Embedding accepts at most one million vertices,
elements and query points, and ten million point/element comparisons per request;
larger inputs are rejected before result allocation and must be partitioned.
