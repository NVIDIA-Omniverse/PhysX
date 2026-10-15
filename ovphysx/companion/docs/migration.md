<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Building UX extensions on the companion

Use an explicit `ToolsSession` for playback or `PreviewSession` for Inspector.
Share one `FrameCoordinator` among stageupdate adapters. The coordinator updates
sealed input, steps once per host frame and delivers copied output on the host
thread. Inspector owns a different ovstage/native attachment and output target;
closing it never collects playback tasks. The core facade's second attach remains
an error. Kit owns widgets, selection, graph registration, input and command stacks.

The [coverage ledger](../plc/coverage.md) maps all 45 interface-family rows and 28
UX directories to REQ/TEST records, including identity-only and no-consumer cases.
The [compatibility matrix](compatibility.md) declares operation and dependency
limits. Each [sample](../samples/README.md) demonstrates a supported data/owner
contract; there is no wholesale `acquire_physx_interface` replacement.

## Python helpers and commands

| Existing caller intent | Supported replacement and behavior | Example |
|---|---|---|
| `physicsUtils.add_box`, `add_rigid_box`, sphere/capsule/cylinder/cone/cube/xform variants | `ovphysx.utils.shapes` implementations; explicit stage/path, stage units, same core signatures | `migration.authoring_helper('physicsUtils.add_rigid_box')` |
| `physicsUtils.add_mass`, `add_density`, `add_force_torque` | `ovphysx.utils.authoring`; codeless schemas and authored values, no simulation initialization | [Migration tests](../tests/test_migration.py) |
| `physicsUtils.add_collision_to_collision_group`, remove/membership helpers | `ovphysx.utils.filtering`; explicit stage and paths | `migration.HELPERS` lists exact supported names |
| Collider selection/batch command | `usd.colliders.ColliderBatch`: bounded preparation, world bounds in meters, cancellation, one commit/undo patch; core `set_collider` implementation | [Batch collider sample](../samples/batch_colliders.py) |
| Attachment/element/filter creation commands | Capture source revisions, compute copied proposals through `AuthoringService`, then `usd.attachments.apply_attachment_proposal` / `apply_filter_proposal` on the approved thread | [Attachment proposals](../samples/attachment_proposals.py) |
| Transform attachment authoring | `capture_transform`, `match_transform_attachment`, `apply_transform_attachment`; local anchor conversion and owned schema edits | [Attachment tests](../tests/test_attachment_elements.py) |
| `deformableUtils` construction commands | `CookingService` plus `usd.deformables.apply_deformable_body`; explicit simulation/collision/visual roles, material, topology and embedding | [Deformable construction](../samples/deformable_cooking.py) |
| Deformable output update | `usd.writeback.map_deformable_output` / `write_simulation_output`; explicit step/time, mapping, units and preview/bake destination | [Writeback tests](../tests/test_usd_helpers.py) |
| `particleUtils` sampling/set authoring | `SamplingService` plus `usd.particles.apply_particle_set` / `apply_surface`; no absent postprocess provider implied | [Particle sample](../samples/particle_authoring.py) |
| Property mass query/edit | `usd.mass` captures/apply helpers and `AuthoringService` computation | [Mass editor](../samples/stopped_mass_editor.py) |
| Joint/body inspector queries | `Inspector` copied records, `SceneQueries` typed hits and `EventQueue` records | [Inspector tools](../samples/inspector_tools.py) |
| Manipulator dragging | `Interaction` default managed force grab, CPU force/impulse and explicit D6 drag; host supplies selected object and pointer input | [Viewport drag](../samples/viewport_drag.py) |
| Viewport collision/debug overlays | `DebugDraw` completed frames plus cooked representations; host draws/colorizes them | [Host stageupdate](../samples/host_stageupdate.py) |
| Settings constants / scene configuration commands | `configuration` classified catalog and `usd.configuration`; process, owner, scene and host scope are explicit | [Scoped settings](../samples/scoped_settings.py) |
| Asset validation/repair commands | Stopped descriptions, declared check dependencies, dedicated `ValidationPreview` for simulation, selected revision-checked repairs | [Validation sample](../samples/asset_validation.py) |
| Kit undo/redo | Wrap the returned `EditPatch`; host calls `undo()`/`redo()` and chooses persistence | [Host tests](../tests/test_host_edits.py) |

`migration.authoring_helper` resolves only its documented whitelist. Unknown helper
or command names raise `MigrationError` with the explicit-helper migration route.
It does not guess positional argument compatibility or install legacy module
aliases. Use the replacement's signature. In particular, legacy command names are
not globally registered by this package. A host command captures selection/source,
starts copied computation, uses `DeferredEdit` for revision/cancellation delivery,
and returns the helper's patch to its command stack. The optional USD package is
shared by standalone and Kit callers; neither caller gets a separate implementation.
Existing core utilities remain available directly for broader schema authoring.

## Cloning, output and excluded interfaces

Core environment replication is a simulation-owner operation with its own reset,
view, topology and instance rules. It is not equivalent to a legacy
`IPhysxReplicator` registration callback or arbitrary USD subtree copy. For custom
plugins use **authored ovstage clone followed by population/recreation**; tests
verify independent immutable parameters and native states. Binary SDK replication
of plugin callback state is unqualified and has no companion adapter.

`StageFlow` consumes explicit sealed input ranges and excludes published output
ordinals from later input. This replaces the data movement needed by UX adapters;
it does not replace Fabric's scheduler or implicitly author customer USD.
`OutputNotices` separately suppresses USD output/undo/rollback notices. The host
chooses each output destination and maintains one writer. Core cloning and output
must be selected on these semantics, never just because a legacy method name looks
similar.

Private/test interfaces have no acquisition adapter. Follow the interface ledger
and select an explicit companion service with its documented owner and capabilities.
Legacy Kit and standalone runtime coexistence is unqualified: use separate
processes exchanging copied data and revisions, never SDK pointers or USD objects.
This is an integration policy, not an isolation check enforced by a boolean helper.
