<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Companion coverage ledger

This is a design-to-contract map, not a statement of implemented parity.
Requirement labels below abbreviate `REQ-COMP-<LABEL>-001`; each links to its
acceptance criteria and mapped TEST. The [PLC index](README.md) lists complete
IDs and verification status. The implemented native/Python subsets and remaining service gaps are tracked there; this ledger does not establish full UX parity.

## Report topics and responsibility boundary

| Report topic | Contract coverage |
|---|---|
| Source scope, static scan limitations and consumer disposition | [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| Package placement, shared owner bridge and capability discovery | [BOUNDARY](requirements/architecture/REQ-COMP-BOUNDARY-001.md), [API](requirements/architecture/REQ-COMP-API-001.md), [OWNER](requirements/architecture/REQ-COMP-OWNER-001.md) |
| ovstage population, revision lifetime and explicit output | [OVSTAGE](requirements/exchange/REQ-COMP-OVSTAGE-001.md), [IDENTITY](requirements/exchange/REQ-COMP-IDENTITY-001.md) |
| Python/C++ split, optional customer USD and native dependency closure | [BOUNDARY](requirements/architecture/REQ-COMP-BOUNDARY-001.md), [PYTHON](requirements/python/REQ-COMP-PYTHON-001.md), [USD-EDIT](requirements/python/REQ-COMP-USD-EDIT-001.md) |
| Authoring snapshots, cancellation, buffers and task ordering | [JOBS](requirements/exchange/REQ-COMP-JOBS-001.md), [BUFFERS](requirements/exchange/REQ-COMP-BUFFERS-001.md), [IDENTITY](requirements/exchange/REQ-COMP-IDENTITY-001.md) |
| Stopped mass, cooking and physics descriptions | [MASS](requirements/authoring/REQ-COMP-MASS-001.md), [COOK](requirements/authoring/REQ-COMP-COOK-001.md), [DESCRIPTION](requirements/authoring/REQ-COMP-DESCRIPTION-001.md) |
| Attachments: native geometry versus Python schema/filter authoring | [ATTACH-COMPUTE](requirements/authoring/REQ-COMP-ATTACH-COMPUTE-001.md), [ATTACH-USD](requirements/python/REQ-COMP-ATTACH-USD-001.md) |
| Deformables: native topology/embedding, Python construction and dynamic USD output | [DEFORM-COOK](requirements/authoring/REQ-COMP-DEFORM-COOK-001.md), [DEFORM-USD](requirements/python/REQ-COMP-DEFORM-USD-001.md), [DEFORM-WRITE](requirements/python/REQ-COMP-DEFORM-WRITE-001.md) |
| Particles and validation | [PARTICLES](requirements/authoring/REQ-COMP-PARTICLES-001.md), [VALIDATE](requirements/authoring/REQ-COMP-VALIDATE-001.md) |
| Queries, live inspection, events, debug frames and statistics | [QUERY](requirements/live/REQ-COMP-QUERY-001.md), [EVENTS](requirements/live/REQ-COMP-EVENTS-001.md), [DEBUG](requirements/live/REQ-COMP-DEBUG-001.md) |
| Manipulation, native pointer leases and settings | [INTERACTION](requirements/live/REQ-COMP-INTERACTION-001.md), [CONFIG](requirements/architecture/REQ-COMP-CONFIG-001.md) |
| Controller inputs/telemetry and cameras | [VEHICLE](requirements/controllers/REQ-COMP-VEHICLE-001.md), [CCT](requirements/controllers/REQ-COMP-CCT-001.md), [CAMERA](requirements/controllers/REQ-COMP-CAMERA-001.md) |
| Custom joint/geometry ABI, external schemas and plugin lifetime | [NATIVE-ABI](requirements/native/REQ-COMP-NATIVE-ABI-001.md), [NATIVE-SCHEMA](requirements/native/REQ-COMP-NATIVE-SCHEMA-001.md), [JOINT](requirements/native/REQ-COMP-JOINT-001.md), [GEOMETRY](requirements/native/REQ-COMP-GEOMETRY-001.md) |
| Preview ownership, restoration and legacy coexistence | [PREVIEW](requirements/live/REQ-COMP-PREVIEW-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| OVD decoding/conversion versus existing recording | [OVD](requirements/diagnostics/REQ-COMP-OVD-001.md) |
| Kit UI, graph, command/undo and timeline responsibilities | [HOST](requirements/host/REQ-COMP-HOST-001.md), [USD-EDIT](requirements/python/REQ-COMP-USD-EDIT-001.md) |
| Distribution and first vertical-slice acceptance | [BOUNDARY](requirements/architecture/REQ-COMP-BOUNDARY-001.md), [PYTHON](requirements/python/REQ-COMP-PYTHON-001.md), [API](requirements/architecture/REQ-COMP-API-001.md), [MASS](requirements/authoring/REQ-COMP-MASS-001.md), [COOK](requirements/authoring/REQ-COMP-COOK-001.md) |

## Interface dispositions

The 45 rows preserve every operation-family row from the source report.
"No direct call found" refers to that static/manual investigation, not proof
that no external consumer exists. Optional service requirements apply when
the capability is implemented and advertised; they do not turn an absent
legacy header into a working provider.

| Legacy interface / operation family | Disposition | Requirements |
|---|---|---|
| `IPhysx`: `startSimulation`, `updateSimulation`, `updateTransformations`, `resetSimulation`, force-load/release | Core lifecycle plus managed preview | [OWNER](requirements/architecture/REQ-COMP-OWNER-001.md), [PREVIEW](requirements/live/REQ-COMP-PREVIEW-001.md), [HOST](requirements/host/REQ-COMP-HOST-001.md) |
| `IPhysx`: simulation/on-step subscriptions, error stream, `isRunning` | Ordered live subscriptions | [EVENTS](requirements/live/REQ-COMP-EVENTS-001.md) |
| `IPhysx`: `getObjectId`, `getPhysXPtr`, `getPhysXPtrFast`, reverse path lookup, object-change subscription | Explicit identity and scoped native access | [IDENTITY](requirements/exchange/REQ-COMP-IDENTITY-001.md), [INTERACTION](requirements/live/REQ-COMP-INTERACTION-001.md) |
| `IPhysx`: GPU/solver overrides, reset preferences/stage settings | Scoped core/scene/host settings | [CONFIG](requirements/architecture/REQ-COMP-CONFIG-001.md) |
| `IPhysx`: `getVehicleDriveState`, `getWheelState`, wheel/body data | Optional vehicle telemetry | [VEHICLE](requirements/controllers/REQ-COMP-VEHICLE-001.md) |
| `IPhysx`: `updateInteraction`, RepX export, legacy PVD connection | Interaction and separately gated exports | [INTERACTION](requirements/live/REQ-COMP-INTERACTION-001.md), [DEBUG](requirements/live/REQ-COMP-DEBUG-001.md) |
| `IPhysxSimulation`: attach/detach, simulate/fetch, output flags/change tracking | Core attachment/tasks and explicit output | [OWNER](requirements/architecture/REQ-COMP-OWNER-001.md), [OVSTAGE](requirements/exchange/REQ-COMP-OVSTAGE-001.md), [HOST](requirements/host/REQ-COMP-HOST-001.md) |
| `IPhysxSimulation`: contact and trigger subscriptions | Contact/trigger event queue | [EVENTS](requirements/live/REQ-COMP-EVENTS-001.md) |
| `IPhysxSimulation`: force-at-position, torque, wake/sleep, instanced variants | Core writes with force-mode parity checks | [INTERACTION](requirements/live/REQ-COMP-INTERACTION-001.md) |
| `IPhysxSceneQuery`: raycast, sweep, overlap; any/closest/all; primitive and shape queries | Core queries plus missing shape-query adapters | [QUERY](requirements/live/REQ-COMP-QUERY-001.md) |
| `IPhysxJoint`: `getJointStateData`, `getJointFramePhysxToUsdQuat` | No direct UX call found; expose only needed typed inspection | [QUERY](requirements/live/REQ-COMP-QUERY-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `IPhysxPrivate`: `getPhysXScene` and private native access | No wholesale private API export; narrow inspection/leases | [QUERY](requirements/live/REQ-COMP-QUERY-001.md), [INTERACTION](requirements/live/REQ-COMP-INTERACTION-001.md) |
| `IPhysxPropertyQuery::queryPrim` | Stopped native mass derivation plus Python edits | [MASS](requirements/authoring/REQ-COMP-MASS-001.md) |
| `IPhysxCooking`: `precookMesh`, convex representation request/cancel, tetrahedral meshing, auto-deformable cooking, cache release | Raw cooking and deformable topology jobs | [COOK](requirements/authoring/REQ-COMP-COOK-001.md), [DEFORM-COOK](requirements/authoring/REQ-COMP-DEFORM-COOK-001.md) |
| `IPhysxCookingService` | Lower-level cooking substrate behind supported data API | [COOK](requirements/authoring/REQ-COMP-COOK-001.md) |
| `IPhysxCookingPrivate`: statistics, refresh set, runtime cache release | Narrow cache invalidation/diagnostics, no private table export | [COOK](requirements/authoring/REQ-COMP-COOK-001.md), [DEBUG](requirements/live/REQ-COMP-DEBUG-001.md) |
| `IPhysxCookingServicePrivate` | Job progress/results, no context-destruction controls | [JOBS](requirements/exchange/REQ-COMP-JOBS-001.md), [COOK](requirements/authoring/REQ-COMP-COOK-001.md) |
| `IPhysxUsdLoad`: collision, tendon, vehicle, particle-system and articulation parsing; descriptor release | Source-neutral stopped descriptions; legacy private header absent | [DESCRIPTION](requirements/authoring/REQ-COMP-DESCRIPTION-001.md) |
| `IPhysxAttachmentPrivate`: point/tet finders, barycentric mapping, sampling, auto-attachment setup/update | Native correspondence plus Python attachment authoring | [ATTACH-COMPUTE](requirements/authoring/REQ-COMP-ATTACH-COMPUTE-001.md), [ATTACH-USD](requirements/python/REQ-COMP-ATTACH-USD-001.md) |
| `IPhysxParticlesPrivate`: sampler and postprocess lifecycle | Optional sampler/postprocess replacement; legacy header absent | [PARTICLES](requirements/authoring/REQ-COMP-PARTICLES-001.md) |
| `IPhysxAssetValidator`: joint state validation/fix, backward compatibility, convex GPU compatibility | Classified validation and Python repair | [VALIDATE](requirements/authoring/REQ-COMP-VALIDATE-001.md) |
| `IPhysxVisualization`: enable/scale/culling/parameters; point/line/triangle arrays | Core live debug snapshots | [DEBUG](requirements/live/REQ-COMP-DEBUG-001.md) |
| `IPhysxVisualization`: collision representations, mesh keys, shape debug draw, normals, per-shape color | Stopped cooked representations and presentation | [COOK](requirements/authoring/REQ-COMP-COOK-001.md), [DESCRIPTION](requirements/authoring/REQ-COMP-DESCRIPTION-001.md), [DEBUG](requirements/live/REQ-COMP-DEBUG-001.md) |
| `IPhysxUI` | Host facade for presentation/notices | [HOST](requirements/host/REQ-COMP-HOST-001.md) |
| `IPhysxUIPrivate` | Host-only overlays/input/test internals; no native export | [HOST](requirements/host/REQ-COMP-HOST-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `IUsdPhysicsUI`, `IUsdPhysicsUIPrivate` | Host editor registration, distinct from solver registration | [HOST](requirements/host/REQ-COMP-HOST-001.md), [NATIVE-ABI](requirements/native/REQ-COMP-NATIVE-ABI-001.md) |
| `IPhysxSupportUi` | Host batch collider authoring over jobs | [HOST](requirements/host/REQ-COMP-HOST-001.md), [JOBS](requirements/exchange/REQ-COMP-JOBS-001.md), [USD-EDIT](requirements/python/REQ-COMP-USD-EDIT-001.md) |
| `IPhysxSupportUiPrivate` | Host Inspector model plus preview contract | [DESCRIPTION](requirements/authoring/REQ-COMP-DESCRIPTION-001.md), [PREVIEW](requirements/live/REQ-COMP-PREVIEW-001.md), [HOST](requirements/host/REQ-COMP-HOST-001.md) |
| `IPhysxSupportUiRigidBodyManipulator` | Controlled collision-aware manipulation | [INTERACTION](requirements/live/REQ-COMP-INTERACTION-001.md) |
| `IPhysxStageUpdate` | One host lifecycle owner; no implicit authoring pump | [HOST](requirements/host/REQ-COMP-HOST-001.md), [OVSTAGE](requirements/exchange/REQ-COMP-OVSTAGE-001.md), [JOBS](requirements/exchange/REQ-COMP-JOBS-001.md) |
| `IPhysxStageUpdateNode` | Host-only update-node and live-sync policy | [HOST](requirements/host/REQ-COMP-HOST-001.md) |
| `IPhysxVehicle` | Optional authoring/control/telemetry | [VEHICLE](requirements/controllers/REQ-COMP-VEHICLE-001.md) |
| `IPhysxCamera` | Higher-level state consumer and Python/host camera edits | [CAMERA](requirements/controllers/REQ-COMP-CAMERA-001.md) |
| `IPhysxCct` | Optional native CCT plus host input | [CCT](requirements/controllers/REQ-COMP-CCT-001.md) |
| `IPhysxStatistics` | Versioned counters, host presentation | [DEBUG](requirements/live/REQ-COMP-DEBUG-001.md) |
| `IPhysXPvd` (header `IPhysxPvd.h`) | Core recording; optional native decode and Python USD conversion | [OVD](requirements/diagnostics/REQ-COMP-OVD-001.md) |
| `IPhysxGraph` | Identity-only header; node scheduling remains host-owned | [HOST](requirements/host/REQ-COMP-HOST-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `IPhysxReplicator` | No direct UX call found; core clone only after semantic comparison | [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `IPhysxFabric` | No direct method call found; host output adapter via ovstage | [OVSTAGE](requirements/exchange/REQ-COMP-OVSTAGE-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `IPhysxFoundation` | Runtime/platform owner with typed core configuration | [CONFIG](requirements/architecture/REQ-COMP-CONFIG-001.md) |
| `IPhysxGpu` | Identity-only header; backend capabilities, no callable export | [API](requirements/architecture/REQ-COMP-API-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `IPhysxSettings` | Settings catalog classified by scope | [CONFIG](requirements/architecture/REQ-COMP-CONFIG-001.md) |
| `IPhysxUnitTests`, `IPhysxBenchmarks` (in `IPhysxTests.h`) | Test/benchmark hooks stay internal; replace product mass-print use | [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md), [MASS](requirements/authoring/REQ-COMP-MASS-001.md), [QUERY](requirements/live/REQ-COMP-QUERY-001.md), [DEBUG](requirements/live/REQ-COMP-DEBUG-001.md) |
| `IPhysxVehicleTesting` (header `IPhysxVehicleTests.h`) | Test-only vehicle hook; no production export | [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `IPhysxCustomJoint`, `IPhysxCustomGeometry` | Native SDK with explicit schema, callback and lifetime contracts | [NATIVE-ABI](requirements/native/REQ-COMP-NATIVE-ABI-001.md), [NATIVE-SCHEMA](requirements/native/REQ-COMP-NATIVE-SCHEMA-001.md), [JOINT](requirements/native/REQ-COMP-JOINT-001.md), [GEOMETRY](requirements/native/REQ-COMP-GEOMETRY-001.md) |

## UX extension dispositions

Every one of the 28 scanned directories is included. Tests, bundles and
examples have explicit treatment; they are not counted as new runtime APIs.

| UX extension directory | Responsibility / use case | Requirements |
|---|---|---|
| `omni.kit.property.physics` | Host property widgets and shared USD schema helpers | [HOST](requirements/host/REQ-COMP-HOST-001.md), [USD-EDIT](requirements/python/REQ-COMP-USD-EDIT-001.md), [DESCRIPTION](requirements/authoring/REQ-COMP-DESCRIPTION-001.md) |
| `omni.physics.physx.ui` | Host capability/UI integration | [HOST](requirements/host/REQ-COMP-HOST-001.md), [API](requirements/architecture/REQ-COMP-API-001.md) |
| `omni.physics.ui` | Umbrella engine selection and visualization adapter | [HOST](requirements/host/REQ-COMP-HOST-001.md), [DEBUG](requirements/live/REQ-COMP-DEBUG-001.md) |
| `omni.physics.isaacsimready` | Host capability/variant selection | [HOST](requirements/host/REQ-COMP-HOST-001.md), [API](requirements/architecture/REQ-COMP-API-001.md) |
| `omni.physics.stageupdate` | Single elected umbrella timeline owner | [HOST](requirements/host/REQ-COMP-HOST-001.md), [OVSTAGE](requirements/exchange/REQ-COMP-OVSTAGE-001.md) |
| `omni.physx.stageupdate` | Core lifecycle integration and output publication | [HOST](requirements/host/REQ-COMP-HOST-001.md), [OWNER](requirements/architecture/REQ-COMP-OWNER-001.md), [OVSTAGE](requirements/exchange/REQ-COMP-OVSTAGE-001.md) |
| `omni.physx.ui` | Mass, collision, attachment, particle, tendon and debug tools | [MASS](requirements/authoring/REQ-COMP-MASS-001.md), [COOK](requirements/authoring/REQ-COMP-COOK-001.md), [DESCRIPTION](requirements/authoring/REQ-COMP-DESCRIPTION-001.md), [ATTACH-USD](requirements/python/REQ-COMP-ATTACH-USD-001.md), [PARTICLES](requirements/authoring/REQ-COMP-PARTICLES-001.md), [DEBUG](requirements/live/REQ-COMP-DEBUG-001.md), [CONFIG](requirements/architecture/REQ-COMP-CONFIG-001.md) |
| `omni.usdphysics.ui` | Generic schema menus and editor joint registration | [HOST](requirements/host/REQ-COMP-HOST-001.md), [USD-EDIT](requirements/python/REQ-COMP-USD-EDIT-001.md) |
| `omni.physx.commands` | Shared Python authoring in host undo commands | [USD-EDIT](requirements/python/REQ-COMP-USD-EDIT-001.md), [ATTACH-USD](requirements/python/REQ-COMP-ATTACH-USD-001.md), [DEFORM-USD](requirements/python/REQ-COMP-DEFORM-USD-001.md), [PARTICLES](requirements/authoring/REQ-COMP-PARTICLES-001.md) |
| `omni.physx.supportui` | Collider batches, Inspector and collision-aware manipulation | [HOST](requirements/host/REQ-COMP-HOST-001.md), [COOK](requirements/authoring/REQ-COMP-COOK-001.md), [PREVIEW](requirements/live/REQ-COMP-PREVIEW-001.md), [INTERACTION](requirements/live/REQ-COMP-INTERACTION-001.md) |
| `omni.physx.graph` | Host query/event/property/immediate cooking nodes | [QUERY](requirements/live/REQ-COMP-QUERY-001.md), [EVENTS](requirements/live/REQ-COMP-EVENTS-001.md), [MASS](requirements/authoring/REQ-COMP-MASS-001.md), [COOK](requirements/authoring/REQ-COMP-COOK-001.md), [HOST](requirements/host/REQ-COMP-HOST-001.md) |
| `omni.physx.asset_validator` | Classified checks and selected Python repairs | [VALIDATE](requirements/authoring/REQ-COMP-VALIDATE-001.md), [ATTACH-COMPUTE](requirements/authoring/REQ-COMP-ATTACH-COMPUTE-001.md) |
| `omni.physx.vehicle` | Vehicle authoring, input and telemetry | [VEHICLE](requirements/controllers/REQ-COMP-VEHICLE-001.md), [USD-EDIT](requirements/python/REQ-COMP-USD-EDIT-001.md) |
| `omni.physx.camera` | Camera state consumers and restoration | [CAMERA](requirements/controllers/REQ-COMP-CAMERA-001.md) |
| `omni.physx.cct` | Native controller movement plus host input/camera | [CCT](requirements/controllers/REQ-COMP-CCT-001.md), [HOST](requirements/host/REQ-COMP-HOST-001.md) |
| `omni.physx.pvd` | Optional viewer and Python USD conversion | [OVD](requirements/diagnostics/REQ-COMP-OVD-001.md) |
| `omni.kit.tool.tetmesh_importer` | Host file import and Python mesh authoring; cook only when requested | [DEFORM-USD](requirements/python/REQ-COMP-DEFORM-USD-001.md), [COOK](requirements/authoring/REQ-COMP-COOK-001.md), [HOST](requirements/host/REQ-COMP-HOST-001.md) |
| `omni.usd.metrics.assembler.physics` | Units authoring, notice blocking and joint display coordination | [HOST](requirements/host/REQ-COMP-HOST-001.md), [USD-EDIT](requirements/python/REQ-COMP-USD-EDIT-001.md) |
| `omni.physx.bundle` | Package dependency/capability selection | [API](requirements/architecture/REQ-COMP-API-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `omni.physx.demos` | Host browser; capability-gated individual examples | [HOST](requirements/host/REQ-COMP-HOST-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `omni.physx.demos.benchmarks` | Benchmark fixtures; no production test-hook export | [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md), [DEBUG](requirements/live/REQ-COMP-DEBUG-001.md) |
| `omni.physx.internal` | Explicit capability checks for retained internal scenes | [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `omni.physx.tests` | Port fixtures to supported contracts, retain private test hooks only in tests | [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `omni.physx.tests.mini` | Core lifecycle/mode fixtures | [OWNER](requirements/architecture/REQ-COMP-OWNER-001.md), [CONFIG](requirements/architecture/REQ-COMP-CONFIG-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `omni.physx.tests.visual` | Host viewport/command fixture integration | [HOST](requirements/host/REQ-COMP-HOST-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `omni.physx.vehicle.tests` | Controller behavior tests without production testing API | [VEHICLE](requirements/controllers/REQ-COMP-VEHICLE-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `omni.physicsexample` | Host example scaffolding; not the custom PhysX SDK sample | [HOST](requirements/host/REQ-COMP-HOST-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |
| `omni.custom.physics.example` | Separate custom simulation/parser example; not evidence of joint/geometry callback use | [HOST](requirements/host/REQ-COMP-HOST-001.md), [MIGRATION](requirements/host/REQ-COMP-MIGRATION-001.md) |

## Boundaries that must survive migration

- Custom joint solver registration and editor billboard/manipulator registration
  are separate capabilities.
- Existing Python helper/command names, settings constants and schema-conflict
  helpers need a compatibility disposition beyond the native interface list.
- The native custom extension sample must use real external schemas. Existing
  examples that implement another simulation backend do not prove this path.
- Stopped mass and collision inspection are authoring computations; they must
  not acquire playback ownership just to answer a UI query.
- Attachment/deformable USD application is Python-owned. Native geometry data
  must be complete enough for that application, including mapping and topology.
- A separate companion package cannot duplicate the loaded runtime/SDK host.
  Inspector defaults to a distinct native attachment and ovstage through a
  qualified companion bridge. Independent public PhysX instances may now
  attach distinct sources alongside this native preview. The preview service
  retains its dedicated native owner and shared-runtime lifecycle guards.
  Worker isolation remains a fallback for unavailable/incompatible capabilities.
  See [Companion ADR-0007](adr/ADR-0007-preview-ownership.md).

Each implementation change should narrow or extend the relevant requirement
and its test evidence, then update this ledger if the consumer disposition
changes. Keep observations about historical interfaces separate from claims
about the current supported API.

The [extension workflow audit](../docs/extensions/ux/WORKFLOW-COVERAGE.md)
adds source-derived Kit migration obligations in a separate RT/UX namespace.
Those requirements and test specifications do not change the implementation
status of this companion service ledger. Full extension parity requires the
host adapters and additional service modes listed in that audit.
