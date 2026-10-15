<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Reviewed ovphysx configuration mapping

This is the migration whitelist for the preference and stage-setting requirements. The old sheet is an inventory, not an instruction to retain every control. “Removed” means absent from the migrated active UI and never forwarded; preserve stored legacy metadata/preferences inertly so opening the UI is nondestructive. UI-only settings do not become ovphysx configuration. Public API exposure establishes a candidate destination, not proof that changing it has the advertised solver effect.

Current companion/API review: `a76179e1a8` (original typed-option audit: `c52311f56fb18ac7d8a080440eaa037fb17b665c`). Runtime application belongs to the sole ovphysx provider; UI queues typed requests without creating a handle. `PhysXConfig` feeds creation configuration and the C global-config functions are process-global. Where the API does not guarantee hot application, defer conservatively; do not claim that Stop resets process bootstrap.

Companion native authoring can initialize the shared SDK before Play. The host configuration coordinator must apply bootstrap-bound preferences before the first native authoring, preview or playback operation. Report later changes as pending restart; a successful setter alone is not effective-behavior evidence. Independent instances share this process configuration and cannot pretend to isolate conflicting bootstrap settings.

## Preferences

| Sheet row / old option | Disposition and exact destination | Application / migration rule |
| --- | --- | --- |
| 22 Preferences host | Audited descriptors only | Keep requested/effective/pending/error distinct. Never forward an unknown key just because Carbonite accepts a string. |
| 23 Reset on Stop | Remove toggle; adapter always restores | Stored false is ignored. No ovphysx reset preference is invented. |
| 24 Use Active CUDA Context | Replace with CUDA Device: `ovphysx_create_args.active_cuda_gpus` (Python `PhysX(active_cuda_gpus=...)`) | Not a semantic rename of context reuse. No automatic old-boolean migration. Empty=no ordinal override; `-1`=automatic; one valid ordinal=explicit device. A different deterministic ordinal after first GPU scene needs a new process. Do not write `/physics/cudaDevice`: ovphysx rejects that escape hatch. |
| 25 Scene Multi-GPU Mode | Public `PhysXConfig.scene_multi_gpu_mode` / `OVPHYSX_GLOBAL_CONFIG_SCENE_MULTI_GPU_MODE` exists, but remove this integration's control | Distributes separate scenes across GPUs, not one scene across devices. The integration has one scene. Nonempty `active_cuda_gpus` takes precedence in the general API; ignore legacy multi-GPU values here. |
| 26 Menu scan limits | Retain as UX policy, no ovphysx destination | 0 through uint32 max; a positive cap limits expensive provider scans, not generic USD authoring. |
| 27 Attachment / Particle Authoring | Retain as authoring-provider UI policy, no ovphysx destination | Declared authoring reload boundary; do not remove authored APIs on disable. |
| 28 Default Simulator | Remove | Umbrella activation owns this decision; do not replay None/PhysX to runtime. |
| 29 Reset Preferences | Reset supported owned request model only | No private IPhysx reset helper; effective restart-bound values remain pending. |
| 30 Simulation Threads | Direct: `PhysXConfig.num_threads` / `OVPHYSX_GLOBAL_CONFIG_NUM_THREADS` | Keep UI 0–256, 0=automatic. These UI bounds are not claimed as native API validation. Supply before bootstrap; conservatively require restart thereafter. |
| 30 Error Stop Threshold | Remove | No reviewed typed ovphysx equivalent. A private error-count key is not a supported runtime contract. |
| 31 CPU Dispatcher / USD Prim Names / Simulate Empty Scene | Remove | No reviewed public product mapping; no Carbonite escape-hatch replay. |
| 31 Expose SDK Profiler Data | Replace with NVTX Profiling: `PhysXConfig.profiler_backend="nvtx"` / `OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND` | NVTX API/SDK ranges for Nsight, not a promise of the old profiler interface. Explicit opt-in before creation; do not automatically map the old boolean. |
| 32 Ujitso Enable / Max Processes | Remove old controls; offer Cooked Collider Cache Directory: `PhysXConfig.cooked_collider_cache_dir` / `OVPHYSX_GLOBAL_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY` | This is a replacement UI capability, not a boolean/count rename. Unset uses a process-private temporary cache; nonempty directory requests persistence. Runtime consumes directory at first bootstrap; later change requires restart. Internal Ujitso keys still exist, but are not adopted as product preferences. |
| 33 Release Mesh Cache | Replace with companion result-cache invalidation | Invalidate completed authoring results through CookingService; do not claim to purge the persistent runtime cache, free live solver shapes or implement releaseObjects. |

## Stage settings

| Sheet row / old group | Disposition and exact destination | Application / migration rule |
| --- | --- | --- |
| 34 Host / reset | Retain stage metadata model | Only whitelist entries are active; preserve unknown/removed keys and unrelated customLayerData. Reset removes the selected override. |
| 35 Update to USD / XformCommonAPI / velocities / local velocity | Remove | ovstage output is governed by the runtime output REQ, not legacy USD-writer toggles. |
| 36 Particle output to USD | Remove | Completed simulation output remains ovstage. Explicit companion Python writeback/export is a separate user operation with owned copied buffers, not a revived runtime USD-writer toggle. |
| 37 Joint body transform check tolerance | Remove | No reviewed public equivalent; a legacy parser key is insufficient. |
| 38 Minimum Simulation Frame Rate | Retain as runtime-adapter catch-up policy | 0 uncapped or 1–1000; next Play epoch only. It is not `PhysXConfig` and not the scene's `timeStepsPerSecond`. |
| 39 Extended Joint Angles | Remove | No reviewed exposed equivalent in ovphysx; do not silently forward a private key. |
| 40 Approximate Cones / Cylinders with Convex Meshes | Remove old flags. Separate candidate controls: Use Custom Cone Geometry / Use Custom Cylinder Geometry → `PhysXConfig.collision_cone_custom_geometry` / `collision_cylinder_custom_geometry`, matching `OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY` / `OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY` | Typed controls invert `/physics/collisionApproximateCones` / `/physics/collisionApproximateCylinders`: true (default) selects a convex core; false selects a convex mesh. `CollisionGeometryConfigTest.SelectsConeAndCylinderShapeGeometry` verifies actual shape types. Apply whitelisted requests before Play attach, not live recook. |
| 41 Mouse Enabled | Retain UX dispatch / interaction adapter policy | Immediate cancellation on disable, no ovphysx global config entry. |
| 42 Grab / Ignore Invisible / With Force / coefficient / acceleration | Retain validated interaction adapter inputs | Supported modes only; coefficient 0–10 and acceleration 0–1000. Not assumed typed backend preferences. |
| 43 Visualization Distance | Retain UX overlay policy | Shared threshold 1–10000 stage units; no backend config entry. |
| 44 USDRT Traversal | Remove | Renderer traversal is not an ovphysx stage-control option. |

Stage metadata containing a supported backend request is not isolated backend state: typed ovphysx settings are process-global. The runtime provider composes the selected stage over explicit defaults, serializes/conflict-checks writes and rejects unavailable modes before attach. Switching to a stage with no override must not inherit previous stage values. Changes during a play epoch remain pending.

Python attachment and deformable writeback use optional customer USD, an explicit edit target and undo transaction; see [writeback requirements](requirements/features/REQ-UX-DEFORM-WRITEBACK-001.md). ovstage remains the native simulation input/output representation. No C++ USD dependency is introduced.

## Evidence and verification boundaries


- [config.py](../../../../python/ovphysx/config.py) — PhysXConfig fields and typed mapping.
- [ovphysx_types.h](../../../../include/ovphysx/ovphysx_types.h) — typed config enums and active_cuda_gpus contract.
- [ovphysx.h](../../../../include/ovphysx/ovphysx.h) — global config, lifecycle and device constraints.
- [ovphysx.cpp](../../../../src/ovphysx/ovphysx.cpp) — typed path mapping and blocked CUDA escape hatch.
- [CarboniteLoader.cpp](../../../../src/CarboniteLoader/CarboniteLoader.cpp) — first-bootstrap cache configuration.
- [test_config_blocked_paths.py](../../../../tests/python_tests/lifecycle_tests/test_config_blocked_paths.py) — blocked CUDA-key tests.
- [test_config_validation.py](../../../../tests/python_tests/lifecycle_tests/test_config_validation.py) — typed versus Carbonite conflict tests.

Requirements: [REQ-UX-PREFERENCES-001](requirements/features/REQ-UX-PREFERENCES-001.md), [REQ-UX-PREF-CONFIG-001](requirements/features/REQ-UX-PREF-CONFIG-001.md), [REQ-UX-PREF-COOK-001](requirements/features/REQ-UX-PREF-COOK-001.md), [REQ-UX-STAGE-SETTINGS-001](requirements/features/REQ-UX-STAGE-SETTINGS-001.md), [REQ-UX-STAGE-POLICY-001](requirements/features/REQ-UX-STAGE-POLICY-001.md), [REQ-UX-INTERACTION-SETTINGS-001](requirements/features/REQ-UX-INTERACTION-SETTINGS-001.md). No API or UI tests are claimed executed by this mapping review.
