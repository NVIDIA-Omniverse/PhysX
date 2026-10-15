<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Source workflow coverage

This audit expands the original 108-row feature sheet to all **28 legacy UX extension directories**. It keeps workflow requirements separate from the implemented companion service contracts. All migrated extension requirements and TESTs are proposed/specification-only. A missing-capability error is required safe behavior, but it does not complete the positive workflow.

The source pin is the requested design branch at `e48e40f4b1`, based on release `5f3904474e`. The fetched release head `33342a9b50` was also compared: product Python changes in the inspected extension tree are blank-line cleanup; changes also include docs, extension versions and tests. The requested branch remains the reproducible baseline.

## Audit method and limits

- Classified all 994 files across 28 directories, including product source, tests, demos, resources and packaging.
- Parsed every Python source file and inventoried 87 command classes, 35 graph descriptors with complete pins, 12 validator classes and 88 demo classes.
- Inspected menu/property registries, dialogs, Inspector toolbar/model/simulation code, commands, graph descriptors/implementations, validator rules, importer, metrics and readiness lifecycle. Compared their dependencies with the current companion modules and capability limits.
- Static coverage establishes a bounded source/workflow inventory. It does not prove every execution path, dynamically loaded contribution, asset-dependent demo, UI interaction or GPU workflow. Acceptance requires the positive/negative host scenarios, not file counts.


## Extension dispositions

| Legacy directory | Files / product source | Workflow requirements |
|---|---:|---|
| `omni.kit.property.physics` | 31 / 24 | [REQ-UX-PROPERTY-GENERIC-001](requirements/features/REQ-UX-PROPERTY-GENERIC-001.md), [REQ-UX-SCHEMA-EDIT-001](requirements/features/REQ-UX-SCHEMA-EDIT-001.md), [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md), [REQ-UX-MASS-001](requirements/features/REQ-UX-MASS-001.md) |
| `omni.physics.physx.ui` | 25 / 15 | [REQ-UX-HOST-001](requirements/features/REQ-UX-HOST-001.md), [REQ-UX-PROPERTY-GENERIC-001](requirements/features/REQ-UX-PROPERTY-GENERIC-001.md), [REQ-UX-SCENE-WIDGET-001](requirements/features/REQ-UX-SCENE-WIDGET-001.md), [REQ-UX-DEFORM-WIDGET-001](requirements/features/REQ-UX-DEFORM-WIDGET-001.md), [REQ-UX-TENDON-WIDGET-001](requirements/features/REQ-UX-TENDON-WIDGET-001.md), [REQ-UX-PARTICLE-WIDGET-001](requirements/features/REQ-UX-PARTICLE-WIDGET-001.md), [REQ-UX-MATERIAL-WIDGET-001](requirements/features/REQ-UX-MATERIAL-WIDGET-001.md) |
| `omni.physics.ui` | 22 / 5 | [REQ-UX-HOST-001](requirements/features/REQ-UX-HOST-001.md), [REQ-UX-MENU-BASIC-001](requirements/features/REQ-UX-MENU-BASIC-001.md), [REQ-UX-MENU-MATERIAL-001](requirements/features/REQ-UX-MENU-MATERIAL-001.md), [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md), [REQ-UX-VIEWPORT-001](requirements/features/REQ-UX-VIEWPORT-001.md), [REQ-UX-EDITOR-JOINT-001](requirements/features/REQ-UX-EDITOR-JOINT-001.md) |
| `omni.physics.isaacsimready` | 20 / 9 | [REQ-UX-READINESS-001](requirements/features/REQ-UX-READINESS-001.md), [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md) |
| `omni.physics.stageupdate` | 12 / 5 | [REQ-UX-HOST-001](requirements/features/REQ-UX-HOST-001.md), [REQ-UX-AUTHOR-STAGE-001](requirements/features/REQ-UX-AUTHOR-STAGE-001.md) |
| `omni.physx.stageupdate` | 14 / 5 | [REQ-UX-HOST-001](requirements/features/REQ-UX-HOST-001.md), [REQ-UX-AUTHOR-STAGE-001](requirements/features/REQ-UX-AUTHOR-STAGE-001.md) |
| `omni.physx.ui` | 93 / 70 | [REQ-UX-MENU-BASIC-001](requirements/features/REQ-UX-MENU-BASIC-001.md), [REQ-UX-MENU-MATERIAL-001](requirements/features/REQ-UX-MENU-MATERIAL-001.md), [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md), [REQ-UX-MENU-PROVIDER-001](requirements/features/REQ-UX-MENU-PROVIDER-001.md), [REQ-UX-SCHEMA-EDIT-001](requirements/features/REQ-UX-SCHEMA-EDIT-001.md), [REQ-UX-VIEWPORT-001](requirements/features/REQ-UX-VIEWPORT-001.md), [REQ-UX-OVERLAY-001](requirements/features/REQ-UX-OVERLAY-001.md), [REQ-UX-PREFERENCES-001](requirements/features/REQ-UX-PREFERENCES-001.md), [REQ-UX-PREF-CONFIG-001](requirements/features/REQ-UX-PREF-CONFIG-001.md), [REQ-UX-PREF-COOK-001](requirements/features/REQ-UX-PREF-COOK-001.md), [REQ-UX-STAGE-SETTINGS-001](requirements/features/REQ-UX-STAGE-SETTINGS-001.md), [REQ-UX-STAGE-POLICY-001](requirements/features/REQ-UX-STAGE-POLICY-001.md), [REQ-UX-INTERACTION-SETTINGS-001](requirements/features/REQ-UX-INTERACTION-SETTINGS-001.md), [REQ-UX-DEBUG-CONTROL-001](requirements/features/REQ-UX-DEBUG-CONTROL-001.md), [REQ-UX-DEBUG-DATA-001](requirements/features/REQ-UX-DEBUG-DATA-001.md), [REQ-UX-MOUSE-001](requirements/features/REQ-UX-MOUSE-001.md), [REQ-UX-MASS-001](requirements/features/REQ-UX-MASS-001.md), [REQ-UX-PROGRESS-001](requirements/features/REQ-UX-PROGRESS-001.md), [REQ-UX-ATTACHMENT-PAIRS-001](requirements/features/REQ-UX-ATTACHMENT-PAIRS-001.md), [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md), [REQ-UX-COLLISION-FILTER-001](requirements/features/REQ-UX-COLLISION-FILTER-001.md), [REQ-UX-SIM-INFO-001](requirements/features/REQ-UX-SIM-INFO-001.md) |
| `omni.usdphysics.ui` | 47 / 19 | [REQ-UX-EDITOR-JOINT-001](requirements/features/REQ-UX-EDITOR-JOINT-001.md), [REQ-UX-VIEWPORT-001](requirements/features/REQ-UX-VIEWPORT-001.md), [REQ-UX-OVERLAY-001](requirements/features/REQ-UX-OVERLAY-001.md), [REQ-UX-PROPERTY-GENERIC-001](requirements/features/REQ-UX-PROPERTY-GENERIC-001.md) |
| `omni.physx.commands` | 10 / 1 | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md), [REQ-UX-SCHEMA-EDIT-001](requirements/features/REQ-UX-SCHEMA-EDIT-001.md), [REQ-UX-MENU-BASIC-001](requirements/features/REQ-UX-MENU-BASIC-001.md), [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md), [REQ-UX-MENU-MATERIAL-001](requirements/features/REQ-UX-MENU-MATERIAL-001.md), [REQ-UX-MENU-PROVIDER-001](requirements/features/REQ-UX-MENU-PROVIDER-001.md), [REQ-UX-COLLISION-FILTER-001](requirements/features/REQ-UX-COLLISION-FILTER-001.md), [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md), [REQ-UX-ATTACHMENT-PAIRS-001](requirements/features/REQ-UX-ATTACHMENT-PAIRS-001.md), [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md), [REQ-UX-TENDON-WIDGET-001](requirements/features/REQ-UX-TENDON-WIDGET-001.md) |
| `omni.physx.supportui` | 88 / 46 | [REQ-UX-TOOL-SELECTION-001](requirements/features/REQ-UX-TOOL-SELECTION-001.md), [REQ-UX-COLLIDER-001](requirements/features/REQ-UX-COLLIDER-001.md), [REQ-UX-INSPECTOR-001](requirements/features/REQ-UX-INSPECTOR-001.md), [REQ-UX-GIZMO-001](requirements/features/REQ-UX-GIZMO-001.md) |
| `omni.physx.graph` | 129 / 91 | [REQ-UX-GRAPH-IDENTITY-001](requirements/features/REQ-UX-GRAPH-IDENTITY-001.md), [REQ-UX-GRAPH-EVENT-001](requirements/features/REQ-UX-GRAPH-EVENT-001.md), [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md), [REQ-UX-GRAPH-IMMEDIATE-001](requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md), [REQ-UX-GRAPH-PROPERTY-001](requirements/features/REQ-UX-GRAPH-PROPERTY-001.md), [REQ-UX-GRAPH-VEHICLE-001](requirements/features/REQ-UX-GRAPH-VEHICLE-001.md) |
| `omni.physx.asset_validator` | 34 / 19 | [REQ-UX-VALIDATE-DATA-001](requirements/features/REQ-UX-VALIDATE-DATA-001.md), [REQ-UX-VALIDATE-MIGRATE-001](requirements/features/REQ-UX-VALIDATE-MIGRATE-001.md), [REQ-UX-VALIDATE-PREVIEW-001](requirements/features/REQ-UX-VALIDATE-PREVIEW-001.md) |
| `omni.physx.vehicle` | 77 / 25 | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md), [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `omni.physx.camera` | 33 / 22 | [REQ-UX-CAMERA-001](requirements/features/REQ-UX-CAMERA-001.md), [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `omni.physx.cct` | 36 / 18 | [REQ-UX-CCT-001](requirements/features/REQ-UX-CCT-001.md), [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md), [REQ-UX-GRAPH-IDENTITY-001](requirements/features/REQ-UX-GRAPH-IDENTITY-001.md) |
| `omni.physx.pvd` | 91 / 27 | [REQ-UX-PVD-RECORD-001](requirements/features/REQ-UX-PVD-RECORD-001.md), [REQ-UX-PVD-VIEW-001](requirements/features/REQ-UX-PVD-VIEW-001.md) |
| `omni.kit.tool.tetmesh_importer` | 5 / 2 | [REQ-UX-TET-IMPORT-001](requirements/features/REQ-UX-TET-IMPORT-001.md) |
| `omni.usd.metrics.assembler.physics` | 13 / 3 | [REQ-UX-METRICS-001](requirements/features/REQ-UX-METRICS-001.md) |
| `omni.physx.bundle` | 7 / 0 | [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md) |
| `omni.physx.demos` | 79 / 6 | [REQ-UX-DEMOS-001](requirements/features/REQ-UX-DEMOS-001.md), [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md) |
| `omni.physx.demos.benchmarks` | 12 / 0 | [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md), [REQ-UX-DEMOS-001](requirements/features/REQ-UX-DEMOS-001.md) |
| `omni.physx.internal` | 22 / 0 | [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md), [REQ-UX-DEMOS-001](requirements/features/REQ-UX-DEMOS-001.md) |
| `omni.physx.tests` | 44 / 0 | [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md) |
| `omni.physx.tests.mini` | 4 / 0 | [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md) |
| `omni.physx.tests.visual` | 19 / 0 | [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md) |
| `omni.physx.vehicle.tests` | 7 / 0 | [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md) |
| `omni.physicsexample` | 8 / 4 | [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md), [REQ-UX-DEMOS-001](requirements/features/REQ-UX-DEMOS-001.md) |
| `omni.custom.physics.example` | 12 / 6 | [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md), [REQ-UX-DEMOS-001](requirements/features/REQ-UX-DEMOS-001.md) |

## Detailed operation catalogs

- [Command classes and constructor contracts](COMMAND-CATALOG.md).
- [Serialized graph types and pin contract](GRAPH-CATALOG.md).
- [Individual validator rules](VALIDATOR-CATALOG.md).
- [Demo qualification inventory](DEMO-CATALOG.md).
- [Original feature-sheet rows](FEATURE-COVERAGE.md), retained with revised dispositions.

## What the original feature sheet missed

The added requirements explicitly cover command compatibility; generic rigid/collision/joint/articulation property models; collision filters and information views; attachment-pair discovery/islands; deformable hierarchy and writeback; particle authoring/postprocess actions; static, migration and preview validation; TetGen import; metrics conversion; readiness/variants; editor joint registration; and bundle/example/test ownership. These obligations are not satisfied by a generic “host owns UI” statement.

## Companion availability versus full UX parity

| Workflow | Existing companion substrate | Remaining obligation / gap |
|---|---|---|
| [REQ-UX-INSPECTOR-001](requirements/features/REQ-UX-INSPECTOR-001.md) — Inspector | Independent PreviewSession, Inspector, USD capture/commit and PreviewDisplay | Full model/drive widgets, quasi-static/gravity/fixed-base policies, selection lock, multiple-window lifecycle and joint-state authoring parity. |
| [REQ-UX-OVERLAY-001](requirements/features/REQ-UX-OVERLAY-001.md) — Stopped/live overlays | DebugDraw, cooked triangle overlays and copied descriptions | Tendon-specific draw, deformable bind/rest/collision representations, particle/fluid/diffuse output, renderer picking and all legacy diagnostics. |
| [REQ-UX-GRAPH-IMMEDIATE-001](requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md) — Immediate graph geometry | Cooking and typed attachment filters | Dedicated geometry contacts, bounds overlap, penetration, geometry raycast and intersecting-face providers. No alias to live SceneQueries. |
| [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) — Scene-query graph shapes | Primitive and bound sphere/box/capsule/convex queries | Legacy arbitrary-prim mesh/plane/custom/compound shape modes, pin/range/ordering semantics and saved-graph compatibility. |
| [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) — Particle workflows | CPU surface/volume sampling and Python particle authoring | Diffuse, smoothing, anisotropy, isosurface and live/GPU extraction; each needs a separately qualified provider. |
| [REQ-UX-VALIDATE-DATA-001](requirements/features/REQ-UX-VALIDATE-DATA-001.md) — Validator catalog | Limited joint-frame/mass/transform/cook checks and ValidationPreview | All individual source rules, compatibility/Newton/mimic repairs and equivalent diagnostics/repair tests. |
| [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) — Deformable/attachment construction | Typed cooked meshes, embedding, static endpoint proposals and Python authoring | Full hierarchy/preset/pose/dialog behavior, curves/unsupported domains and equivalent auto-attachment discovery. |
| [REQ-UX-DEFORM-WRITEBACK-001](requirements/features/REQ-UX-DEFORM-WRITEBACK-001.md) — Deformable writeback | Completed copied CPU buffers, interpolation and USD output utilities | Real GPU/simulation extraction and domain/frame coverage for every retained visual/collision topology. |
| [REQ-UX-GIZMO-001](requirements/features/REQ-UX-GIZMO-001.md) — Mouse and gizmos | Exact-owner native force grab with legacy spring/gravity/damping, force/torque and explicit D6 point drag | Host repeating push policy, deformable/GPU picking and rotational target/axis-lock/snapping behavior. |
| [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) — Vehicle and camera UX | CPU vehicle computations/telemetry and follow/drone utilities | Full wizard/friction/nonlinear tables, live widgets/inputs and exact legacy camera tuning. |
| [REQ-UX-TET-IMPORT-001](requirements/features/REQ-UX-TET-IMPORT-001.md) — Import/metrics/readiness | EditSession, source revisions, schemas and capability descriptors | TetGen parser/asset integration, dimensional conversion rules and safe variant-selection host adapters. |
| [REQ-UX-PVD-VIEW-001](requirements/features/REQ-UX-PVD-VIEW-001.md) — OVD viewer | Recording coordination, bounded decode and USD conversion | Full recording browser/property/seek/diagnostic overlays. Legacy PVD/RepX remains intentionally separate and unsupported. |
| [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) — Commands and schemas | Many public ovphysx.utils helpers and transactional USD primitives | Complete registered-command/preset/property behavior and host undo/redo integration; the migration helper whitelist is not full compatibility. |
| [REQ-UX-PACKAGE-001](requirements/features/REQ-UX-PACKAGE-001.md) — Provider packaging and host lifecycle | Separate Python package, ABI tables, explicit owners and FrameCoordinator | Actual Kit dependencies, adapters, thread scheduling, reload tests and customer-USD/ovstage integration. |

These are proposed extension obligations, not changes to the completed CPU subset's implementation status. Extending a native service must first extend its companion/runtime REQ and TEST; adding a widget does not make an unavailable backend supported. GPU and additional platforms remain unqualified until exercised.

The temporary evidence directories have been removed. The public workflow catalogs retain the audit findings. Source citations are historical path/commit references; retrieving those historical trees is optional and is not a build or CI dependency.
