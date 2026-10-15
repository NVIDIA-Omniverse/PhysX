<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Graph Catalog

Names are inventoried from the inspected release source. Every row remains a **proposed host migration**, not an implemented adapter. [Machine-readable contracts](graph-contracts.json) retain every input/output/state pin descriptor, including defaults and role metadata. The table uses descriptor keys; CCT keys require confirmation against generated runtime registration names before saved-graph migration. Exact source anchors are in the internal source manifest.


| type | version | requirement |
| --- | --- | --- |
| `OgnCharacterController` | `1` | [REQ-UX-CCT-001](requirements/features/REQ-UX-CCT-001.md) |
| `OgnControlsSettings` | `2` | [REQ-UX-CCT-001](requirements/features/REQ-UX-CCT-001.md) |
| `OgnSpawnCapsule` | `1` | [REQ-UX-CCT-001](requirements/features/REQ-UX-CCT-001.md) |
| `omni.physx.graph.GenerateContacts` | `1` | [REQ-UX-GRAPH-IMMEDIATE-001](requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md) |
| `omni.physx.graph.OnContactEventAdvanced` | `1` | [REQ-UX-GRAPH-EVENT-001](requirements/features/REQ-UX-GRAPH-EVENT-001.md) |
| `omni.physx.graph.OnContactEventBasic` | `1` | [REQ-UX-GRAPH-EVENT-001](requirements/features/REQ-UX-GRAPH-EVENT-001.md) |
| `omni.physx.graph.ImmediateComputeBoundsOverlaps` | `1` | [REQ-UX-GRAPH-IMMEDIATE-001](requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md) |
| `omni.physx.graph.ImmediateComputeGeometryBounds` | `1` | [REQ-UX-GRAPH-IMMEDIATE-001](requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md) |
| `omni.physx.graph.ImmediateComputeGeometryOverlaps` | `1` | [REQ-UX-GRAPH-IMMEDIATE-001](requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md) |
| `omni.physx.graph.ImmediateComputeGeometryPenetrations` | `1` | [REQ-UX-GRAPH-IMMEDIATE-001](requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md) |
| `omni.physx.graph.ImmediateComputeGeometryRaycastHits` | `1` | [REQ-UX-GRAPH-IMMEDIATE-001](requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md) |
| `omni.physx.graph.ImmediateComputeMeshIntersectingFaces` | `1` | [REQ-UX-GRAPH-IMMEDIATE-001](requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md) |
| `omni.physx.graph.ImmediateGenerateGeometryContacts` | `1` | [REQ-UX-GRAPH-IMMEDIATE-001](requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md) |
| `omni.physx.graph.PropertyQueryRigidBody` | `1` | [REQ-UX-GRAPH-PROPERTY-001](requirements/features/REQ-UX-GRAPH-PROPERTY-001.md) |
| `omni.physx.graph.SceneQueryOverlapBoxAll` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQueryOverlapBoxAny` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQueryOverlapPrimAll` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQueryOverlapPrimAny` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQueryOverlapSphereAll` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQueryOverlapSphereAny` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQueryRaycastAll` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQueryRaycastAny` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQueryRaycastClosest` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQuerySweepBoxAll` | `2` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQuerySweepBoxAny` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQuerySweepBoxClosest` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQuerySweepPrimAll` | `2` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQuerySweepPrimAny` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQuerySweepPrimClosest` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQuerySweepSphereAll` | `2` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQuerySweepSphereAny` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.SceneQuerySweepSphereClosest` | `1` | [REQ-UX-GRAPH-QUERY-001](requirements/features/REQ-UX-GRAPH-QUERY-001.md) |
| `omni.physx.graph.OnTriggerCollider` | `1` | [REQ-UX-GRAPH-EVENT-001](requirements/features/REQ-UX-GRAPH-EVENT-001.md) |
| `VehicleGetDriveState` | `1` | [REQ-UX-GRAPH-VEHICLE-001](requirements/features/REQ-UX-GRAPH-VEHICLE-001.md) |
| `VehicleGetWheelState` | `1` | [REQ-UX-GRAPH-VEHICLE-001](requirements/features/REQ-UX-GRAPH-VEHICLE-001.md) |

## Pin and execution contract

For each inventoried type, preserve its serialized version, exact input/output/state pin names and metadata, token/path and instance representations, array sizes and execution pins. The source manifest stores complete descriptors. Negative/infinite range conventions, normalized directions, both-sides filtering, closest/any/all ordering, truncation, caller geometry frames and contact found/persist/lost are adapter contracts, not inferred from similarly named companion methods. Missing pin/mode support remains a parity gap.

## Pin contracts

The JSON contract is authoritative for all descriptor metadata. Missing defaults below mean the descriptor does not specify one; do not invent a compatibility default.

### OgnCharacterController

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `activate` | `execution` | `(unspecified)` |
| inputs | `deactivate` | `execution` | `(unspecified)` |
| inputs | `gravity` | `bool` | `true` |
| inputs | `speed` | `int` | `500` |
| inputs | `setupControls` | `token` | `"Auto"` |
| inputs | `controlsSettings` | `bundle` | `(unspecified)` |
| inputs | `capsulePath` | `path` | `(unspecified)` |
| inputs | `fpCameraPathToken` | `token` | `(unspecified)` |
| outputs | `done` | `execution` | `(unspecified)` |

### OgnControlsSettings

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `forward` | `token` | `"W"` |
| inputs | `backward` | `token` | `"S"` |
| inputs | `left` | `token` | `"A"` |
| inputs | `right` | `token` | `"D"` |
| inputs | `up` | `token` | `"E"` |
| inputs | `down` | `token` | `"Q"` |
| inputs | `mouseSensitivity` | `float` | `25` |
| inputs | `gamepadSensitivity` | `float` | `25` |
| outputs | `control_settings` | `bundle` | `(unspecified)` |

### OgnSpawnCapsule

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `spawn` | `execution` | `(unspecified)` |
| inputs | `capsulePos` | `float[3]` | `[0, 0, 0]` |
| inputs | `capsuleHeight` | `float` | `100` |
| inputs | `capsuleRadius` | `float` | `50` |
| outputs | `done` | `execution` | `(unspecified)` |
| outputs | `path` | `path` | `(unspecified)` |

### omni.physx.graph.GenerateContacts

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `shape0` | `token` | `(unspecified)` |
| inputs | `shape1` | `token` | `(unspecified)` |
| inputs | `contactDistance` | `float` | `0.01` |
| inputs | `meshContactMargin` | `float` | `0.01` |
| inputs | `toleranceLength` | `float` | `0.1` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `contactCount` | `int` | `(unspecified)` |
| outputs | `contactPoints` | `pointf[3][]` | `(unspecified)` |
| outputs | `contactNormals` | `normalf[3][]` | `(unspecified)` |
| outputs | `contactDepths` | `float[]` | `(unspecified)` |

### omni.physx.graph.OnContactEventAdvanced

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `bodyPaths` | `token[]` | `(unspecified)` |
| inputs | `targetBodies` | `target` | `(unspecified)` |
| outputs | `foundExecOut` | `execution` | `(unspecified)` |
| outputs | `persistsExecOut` | `execution` | `(unspecified)` |
| outputs | `lostExecOut` | `execution` | `(unspecified)` |
| outputs | `contactingBody` | `token` | `(unspecified)` |
| outputs | `inputBody` | `token` | `(unspecified)` |
| outputs | `contactingCollider` | `token` | `(unspecified)` |
| outputs | `inputBodyCollider` | `token` | `(unspecified)` |
| outputs | `contactingFaces` | `int[]` | `(unspecified)` |
| outputs | `inputBodyFaces` | `int[]` | `(unspecified)` |
| outputs | `contactingMaterials` | `int[]` | `(unspecified)` |
| outputs | `inputBodyMaterials` | `int[]` | `(unspecified)` |
| outputs | `contactPoints` | `pointf[3][]` | `(unspecified)` |
| outputs | `contactNormals` | `normalf[3][]` | `(unspecified)` |
| outputs | `contactDepths` | `float[]` | `(unspecified)` |
| outputs | `contactImpulses` | `vectorf[3][]` | `(unspecified)` |

### omni.physx.graph.OnContactEventBasic

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `bodyPaths` | `token[]` | `(unspecified)` |
| inputs | `targetBodies` | `target` | `(unspecified)` |
| outputs | `foundExecOut` | `execution` | `(unspecified)` |
| outputs | `persistsExecOut` | `execution` | `(unspecified)` |
| outputs | `lostExecOut` | `execution` | `(unspecified)` |
| outputs | `contactingBody` | `token` | `(unspecified)` |
| outputs | `inputBody` | `token` | `(unspecified)` |

### omni.physx.graph.ImmediateComputeBoundsOverlaps

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `primsBundle` | `bundle` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `overlapsPair0` | `token[]` | `(unspecified)` |
| outputs | `overlapsPair1` | `token[]` | `(unspecified)` |

### omni.physx.graph.ImmediateComputeGeometryBounds

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `primsBundle` | `bundle` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `primsBundle` | `bundle` | `(unspecified)` |

### omni.physx.graph.ImmediateComputeGeometryOverlaps

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `primsBundle` | `bundle` | `(unspecified)` |
| inputs | `overlapsPair0` | `token[]` | `(unspecified)` |
| inputs | `overlapsPair1` | `token[]` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `overlaps` | `bool[]` | `(unspecified)` |

### omni.physx.graph.ImmediateComputeGeometryPenetrations

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `primsBundle` | `bundle` | `(unspecified)` |
| inputs | `overlapsPair0` | `token[]` | `(unspecified)` |
| inputs | `overlapsPair1` | `token[]` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `overlaps` | `bool[]` | `(unspecified)` |
| outputs | `penetrationVectors` | `normalf[3][]` | `(unspecified)` |
| outputs | `penetrationDepths` | `float[]` | `(unspecified)` |

### omni.physx.graph.ImmediateComputeGeometryRaycastHits

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `origin` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `raycastRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| inputs | `primsBundle` | `bundle` | `(unspecified)` |
| inputs | `prims` | `target` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `primsHit` | `target` | `(unspecified)` |
| outputs | `positions` | `pointf[3][]` | `(unspecified)` |
| outputs | `normals` | `normalf[3][]` | `(unspecified)` |
| outputs | `distances` | `float[]` | `(unspecified)` |
| outputs | `faceIndexes` | `int[]` | `(unspecified)` |

### omni.physx.graph.ImmediateComputeMeshIntersectingFaces

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `primsBundle` | `bundle` | `(unspecified)` |
| inputs | `overlapsPair0` | `token[]` | `(unspecified)` |
| inputs | `overlapsPair1` | `token[]` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `overlaps` | `bool[]` | `(unspecified)` |
| outputs | `faceIndices` | `bundle` | `(unspecified)` |

### omni.physx.graph.ImmediateGenerateGeometryContacts

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `primsBundle` | `bundle` | `(unspecified)` |
| inputs | `overlapsPair0` | `token[]` | `(unspecified)` |
| inputs | `overlapsPair1` | `token[]` | `(unspecified)` |
| inputs | `contactDistance` | `float` | `0.01` |
| inputs | `meshContactMargin` | `float` | `0.01` |
| inputs | `toleranceLength` | `float` | `0.1` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `contacts` | `bundle` | `(unspecified)` |

### omni.physx.graph.PropertyQueryRigidBody

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `prim` | `target` | `(unspecified)` |
| inputs | `timeout` | `int` | `10000` |
| inputs | `use_local_frame` | `bool` | `false` |
| outputs | `centerOfMass` | `float[3]` | `(unspecified)` |
| outputs | `mass` | `float` | `(unspecified)` |
| outputs | `diagonalInertia` | `float[3]` | `(unspecified)` |
| outputs | `principalAxes` | `float[4]` | `(unspecified)` |
| outputs | `AABBMin` | `float[3]` | `(unspecified)` |
| outputs | `AABBMax` | `float[3]` | `(unspecified)` |

### omni.physx.graph.SceneQueryOverlapBoxAll

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `position` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `dimensions` | `['float[3]', 'double[3]']` | `(unspecified)` |
| inputs | `rotation` | `['float[3]', 'double[3]']` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `colliderPrimPaths` | `token[]` | `(unspecified)` |
| outputs | `colliderPrims` | `target` | `(unspecified)` |
| outputs | `bodyPrimPaths` | `token[]` | `(unspecified)` |
| outputs | `bodyPrims` | `target` | `(unspecified)` |

### omni.physx.graph.SceneQueryOverlapBoxAny

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `position` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `dimensions` | `['float[3]', 'double[3]']` | `(unspecified)` |
| inputs | `rotation` | `['float[3]', 'double[3]']` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `overlap` | `bool` | `(unspecified)` |

### omni.physx.graph.SceneQueryOverlapPrimAll

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `primPath` | `token` | `(unspecified)` |
| inputs | `prim` | `target` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `colliderPrimPaths` | `token[]` | `(unspecified)` |
| outputs | `colliderPrims` | `target` | `(unspecified)` |
| outputs | `bodyPrimPaths` | `token[]` | `(unspecified)` |
| outputs | `bodyPrims` | `target` | `(unspecified)` |

### omni.physx.graph.SceneQueryOverlapPrimAny

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `primPath` | `token` | `(unspecified)` |
| inputs | `prim` | `target` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `overlap` | `bool` | `(unspecified)` |

### omni.physx.graph.SceneQueryOverlapSphereAll

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `position` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `radius` | `['float', 'double']` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `colliderPrimPaths` | `token[]` | `(unspecified)` |
| outputs | `colliderPrims` | `target` | `(unspecified)` |
| outputs | `bodyPrimPaths` | `token[]` | `(unspecified)` |
| outputs | `bodyPrims` | `target` | `(unspecified)` |

### omni.physx.graph.SceneQueryOverlapSphereAny

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `position` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `radius` | `['float', 'double']` | `(unspecified)` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `overlap` | `bool` | `(unspecified)` |

### omni.physx.graph.SceneQueryRaycastAll

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `origin` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `raycastRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| inputs | `sortByDistance` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `colliderPrimPaths` | `token[]` | `(unspecified)` |
| outputs | `colliderPrims` | `target` | `(unspecified)` |
| outputs | `bodyPrimPaths` | `token[]` | `(unspecified)` |
| outputs | `bodyPrims` | `target` | `(unspecified)` |
| outputs | `positions` | `pointf[3][]` | `(unspecified)` |
| outputs | `normals` | `normalf[3][]` | `(unspecified)` |
| outputs | `distances` | `float[]` | `(unspecified)` |
| outputs | `faceIndexes` | `int[]` | `(unspecified)` |
| outputs | `materialPaths` | `token[]` | `(unspecified)` |
| outputs | `materialPrims` | `target` | `(unspecified)` |

### omni.physx.graph.SceneQueryRaycastAny

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `origin` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `raycastRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `hit` | `bool` | `(unspecified)` |

### omni.physx.graph.SceneQueryRaycastClosest

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `origin` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `raycastRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `hit` | `bool` | `(unspecified)` |
| outputs | `colliderPrimPath` | `token` | `(unspecified)` |
| outputs | `colliderPrim` | `target` | `(unspecified)` |
| outputs | `bodyPrimPath` | `token` | `(unspecified)` |
| outputs | `bodyPrim` | `target` | `(unspecified)` |
| outputs | `position` | `pointf[3]` | `(unspecified)` |
| outputs | `normal` | `normalf[3]` | `(unspecified)` |
| outputs | `distance` | `float` | `(unspecified)` |
| outputs | `faceIndex` | `int` | `(unspecified)` |
| outputs | `materialPath` | `token` | `(unspecified)` |
| outputs | `materialPrim` | `target` | `(unspecified)` |

### omni.physx.graph.SceneQuerySweepBoxAll

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `origin` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `dimensions` | `['float[3]', 'double[3]']` | `(unspecified)` |
| inputs | `rotation` | `['float[3]', 'double[3]']` | `(unspecified)` |
| inputs | `sweepRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| inputs | `sortByDistance` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `colliderPrims` | `target` | `(unspecified)` |
| outputs | `bodyPrims` | `target` | `(unspecified)` |
| outputs | `positions` | `pointf[3][]` | `(unspecified)` |
| outputs | `normals` | `normalf[3][]` | `(unspecified)` |
| outputs | `distances` | `float[]` | `(unspecified)` |
| outputs | `faceIndexes` | `int[]` | `(unspecified)` |
| outputs | `materialPrims` | `target` | `(unspecified)` |

### omni.physx.graph.SceneQuerySweepBoxAny

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `origin` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `dimensions` | `['float[3]', 'double[3]']` | `(unspecified)` |
| inputs | `rotation` | `['float[3]', 'double[3]']` | `(unspecified)` |
| inputs | `sweepRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `hit` | `bool` | `(unspecified)` |

### omni.physx.graph.SceneQuerySweepBoxClosest

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `origin` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `dimensions` | `['float[3]', 'double[3]']` | `(unspecified)` |
| inputs | `rotation` | `['float[3]', 'double[3]']` | `(unspecified)` |
| inputs | `sweepRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `hit` | `bool` | `(unspecified)` |
| outputs | `colliderPrim` | `target` | `(unspecified)` |
| outputs | `bodyPrim` | `target` | `(unspecified)` |
| outputs | `position` | `pointf[3]` | `(unspecified)` |
| outputs | `normal` | `normalf[3]` | `(unspecified)` |
| outputs | `distance` | `float` | `(unspecified)` |
| outputs | `faceIndex` | `int` | `(unspecified)` |
| outputs | `materialPrim` | `target` | `(unspecified)` |

### omni.physx.graph.SceneQuerySweepPrimAll

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `prim` | `target` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `sweepRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| inputs | `sortByDistance` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `colliderPrims` | `target` | `(unspecified)` |
| outputs | `bodyPrims` | `target` | `(unspecified)` |
| outputs | `positions` | `pointf[3][]` | `(unspecified)` |
| outputs | `normals` | `normalf[3][]` | `(unspecified)` |
| outputs | `distances` | `float[]` | `(unspecified)` |
| outputs | `faceIndexes` | `int[]` | `(unspecified)` |
| outputs | `materialPrims` | `target` | `(unspecified)` |

### omni.physx.graph.SceneQuerySweepPrimAny

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `prim` | `target` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `sweepRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `hit` | `bool` | `(unspecified)` |

### omni.physx.graph.SceneQuerySweepPrimClosest

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `prim` | `target` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `sweepRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `hit` | `bool` | `(unspecified)` |
| outputs | `colliderPrim` | `target` | `(unspecified)` |
| outputs | `bodyPrim` | `target` | `(unspecified)` |
| outputs | `position` | `pointf[3]` | `(unspecified)` |
| outputs | `normal` | `normalf[3]` | `(unspecified)` |
| outputs | `distance` | `float` | `(unspecified)` |
| outputs | `faceIndex` | `int` | `(unspecified)` |
| outputs | `materialPrim` | `target` | `(unspecified)` |

### omni.physx.graph.SceneQuerySweepSphereAll

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `origin` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `radius` | `['float', 'double']` | `(unspecified)` |
| inputs | `sweepRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| inputs | `sortByDistance` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `colliderPrimPaths` | `token[]` | `(unspecified)` |
| outputs | `colliderPrims` | `target` | `(unspecified)` |
| outputs | `bodyPrimPaths` | `token[]` | `(unspecified)` |
| outputs | `bodyPrims` | `target` | `(unspecified)` |
| outputs | `positions` | `pointf[3][]` | `(unspecified)` |
| outputs | `normals` | `normalf[3][]` | `(unspecified)` |
| outputs | `distances` | `float[]` | `(unspecified)` |
| outputs | `faceIndexes` | `int[]` | `(unspecified)` |
| outputs | `materialPaths` | `token[]` | `(unspecified)` |
| outputs | `materialPrims` | `target` | `(unspecified)` |

### omni.physx.graph.SceneQuerySweepSphereAny

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `origin` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `radius` | `['float', 'double']` | `(unspecified)` |
| inputs | `sweepRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `hit` | `bool` | `(unspecified)` |

### omni.physx.graph.SceneQuerySweepSphereClosest

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `execIn` | `execution` | `(unspecified)` |
| inputs | `origin` | `['pointf[3]', 'pointd[3]']` | `(unspecified)` |
| inputs | `direction` | `['vectorf[3]', 'vectord[3]']` | `(unspecified)` |
| inputs | `radius` | `['float', 'double']` | `(unspecified)` |
| inputs | `sweepRange` | `['float', 'double']` | `(unspecified)` |
| inputs | `bothSides` | `bool` | `false` |
| outputs | `execOut` | `execution` | `(unspecified)` |
| outputs | `hit` | `bool` | `(unspecified)` |
| outputs | `colliderPrimPath` | `token` | `(unspecified)` |
| outputs | `colliderPrim` | `target` | `(unspecified)` |
| outputs | `bodyPrimPath` | `token` | `(unspecified)` |
| outputs | `bodyPrim` | `target` | `(unspecified)` |
| outputs | `position` | `pointf[3]` | `(unspecified)` |
| outputs | `normal` | `normalf[3]` | `(unspecified)` |
| outputs | `distance` | `float` | `(unspecified)` |
| outputs | `faceIndex` | `int` | `(unspecified)` |
| outputs | `materialPath` | `token` | `(unspecified)` |
| outputs | `materialPrim` | `target` | `(unspecified)` |

### omni.physx.graph.OnTriggerCollider

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `triggersRelationships` | `target` | `(unspecified)` |
| inputs | `triggersPaths` | `token[]` | `(unspecified)` |
| inputs | `listenToAllTriggers` | `bool` | `(unspecified)` |
| outputs | `enterExecOut` | `execution` | `(unspecified)` |
| outputs | `leaveExecOut` | `execution` | `(unspecified)` |
| outputs | `triggerCollider` | `token` | `(unspecified)` |
| outputs | `otherCollider` | `token` | `(unspecified)` |
| outputs | `triggerBody` | `token` | `(unspecified)` |
| outputs | `otherBody` | `token` | `(unspecified)` |

### VehicleGetDriveState

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `vehiclePath` | `token` | `(unspecified)` |
| outputs | `accelerator` | `float` | `(unspecified)` |
| outputs | `brake0` | `float` | `(unspecified)` |
| outputs | `brake1` | `float` | `(unspecified)` |
| outputs | `steer` | `float` | `(unspecified)` |
| outputs | `currentGear` | `int` | `(unspecified)` |
| outputs | `targetGear` | `int` | `(unspecified)` |
| outputs | `automaticTransmission` | `bool` | `(unspecified)` |
| outputs | `engineRotationSpeed` | `float` | `(unspecified)` |

### VehicleGetWheelState

| Direction | Pin | Type | Declared default |
| --- | --- | --- | --- |
| inputs | `vehicleWheelAttachmentPaths` | `token[]` | `(unspecified)` |
| outputs | `wheelRotationSpeeds` | `float[]` | `(unspecified)` |
| outputs | `wheelRotationAngles` | `float[]` | `(unspecified)` |
| outputs | `wheelSteerAngles` | `float[]` | `(unspecified)` |
| outputs | `groundPlanes` | `float[4][]` | `(unspecified)` |
| outputs | `groundHitPositions` | `float[3][]` | `(unspecified)` |
| outputs | `groundPhysXActors` | `token[]` | `(unspecified)` |
| outputs | `groundPhysXShapes` | `token[]` | `(unspecified)` |
| outputs | `groundPhysXMaterials` | `token[]` | `(unspecified)` |
| outputs | `groundContactStates` | `bool[]` | `(unspecified)` |
| outputs | `suspensionJounces` | `float[]` | `(unspecified)` |
| outputs | `suspensionForces` | `float[3][]` | `(unspecified)` |
| outputs | `tireFrictions` | `float[]` | `(unspecified)` |
| outputs | `tireLongitudinalSlips` | `float[]` | `(unspecified)` |
| outputs | `tireLateralSlips` | `float[]` | `(unspecified)` |
| outputs | `tireLongitudinalDirections` | `float[3][]` | `(unspecified)` |
| outputs | `tireLateralDirections` | `float[3][]` | `(unspecified)` |
| outputs | `tireForces` | `float[3][]` | `(unspecified)` |
