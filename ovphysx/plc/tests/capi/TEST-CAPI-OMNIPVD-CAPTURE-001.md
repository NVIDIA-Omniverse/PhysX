<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OMNIPVD-CAPTURE-001
maps_to: REQ-CAPI-OMNIPVD-CAPTURE-001
type: integration
---

## Scenario

Sequential capture sessions apply copied selectors to the requesting attachment and serialize primary data plus optional secondary data.

## Given

- A capable runtime with `simple_physics_scene.usda`, two attachments with identical paths, and an unattached peer.
- Capability is selected through the scoped global BOOL key before the first instance. CPU selective cases run in separate processes before any shared default instance can freeze capability.
- The public `omnipvd_selective_scenes.usda` and `two_articulations.usda` fixtures, and a canonical OmniPVD reader.
- The public `omnipvd_selective_mirrors.usda` fixture, with shared dynamic and static bodies in two scenes and an unrelated actor.
- Exact FILE and loopback TCP destinations plus invalid descriptors.
- The public `omnipvd_selective_resources.usda`, `omnipvd_selective_particles.usda`, and surface/volume deformable fixtures, with GPU dynamics in both readback modes.

- A loopback receiver that can retain the discovery catalog before sending any recording request.

## When

- Compile the boolean descriptor field, default initializer and new signature from C and C++.
- Start with invalid scene and object lists, and with malformed, missing and wrong-type scene and object paths into missing nested destination parents, then retry valid capture.
- Decode object, scene-union and whole-articulation sessions with `record_secondary_data` set to `false`, `true` and its default value; also start with a null filter.
- Mutate caller storage, attempt active takeover and peer stop, then stop and restart with a different object.
- Resolve identical paths from separate attachments and start from an unattached peer with a populated scene list, a populated object list, and empty lists.
- Receive a selected primary-data TCP stream, remove its transport handshake and decode its OmniPVD payload.
- Reject shape, material, joint and articulation-link prims in the object list; select a particle system, one of two particle buffers, and each deformable actor type; decode finite current particle position/inverse-mass and velocity arrays plus snapshot phase counts with secondary data both enabled and disabled, and deformable position/velocity arrays across completed simulation frames.
- Select the exact prototype path in `ovstage_output_instancer.usda` with primary and secondary data and inspect both native instances while excluding the separate fixed actor.
- Select each shared body path in `omnipvd_selective_mirrors.usda` and decode both scenes' native actors while excluding unrelated actors.
- Select the converted fixed-base root-joint path in `articulation_selection.usda`, then reject an attachment-owned D6 joint created through the runtime API without a population record.

- Start discovery with FILE, no attachment, an owner-wide scope and an object-only scope; poll ownership, step without receiver selection, stop, and detach the owner. Decode the forward catalog before any recording request.
- Buffer a receiver selection before the first warmup, inspect waiting state after warmup, then submit a public step. Hold a peer's queued work while the owner performs an empty control poll, then buffer a request before another owner step.

## Then

- The three-argument C start signature and every options offset, including `record_secondary_data`, agree with the ctypes layout (REQ AC-1).
- Invalid descriptors create neither destination files nor missing parent directories, leave no active session, and preserve an existing sentinel file; error status distinguishes malformed and unresolved paths (REQ AC-2).
- Only the correct attachment's selected objects and union members appear, including articulation links (REQ AC-3).
- `SharedBodyPathIncludesMirrorsInEveryScene` records the primary and mirror actors for each selected dynamic or static prim, with no unrelated actor objects (REQ AC-3).
- Caller mutations do not retarget a session, peer stop and takeover are rejected, and the restarted segment has the new selection (REQ AC-4).
- FILE and TCP contain selected poses, velocities and geometry for both boolean values; secondary data such as mass appears only when `record_secondary_data` is `true`, including the default initializer. A null filter records all supported data and objects; empty selectors from an unattached peer include shared-runtime objects (REQ AC-5).
- Dependent-resource and articulation-link paths are rejected before opening output; selected particle/deformable payloads have the expected active vertex/particle counts, finite components and updated solver state in ordinary GPU and DirectGPU modes. Selected particle buffers emit active position/inverse-mass and velocity entries, with one uint32 phase per active particle in the snapshot. These base-particle payloads remain present with secondary data disabled; diffuse payload qualification remains in the SDK suite. An individually selected buffer emits no sibling position payload; deformable velocity arrays remain present and current (REQ AC-6).
- `PrimaryPrototypePathIncludesEveryNativeInstance` and `GenericPrototypePathIncludesEveryNativeInstance` capture both native instances at a prototype path rather than only the first registry record, while the separate fixed actor emits no pose (REQ AC-3, AC-6).
- `GenericFixedBaseJointPathCapturesItsArticulation` decodes the whole-articulation registry object behind a fixed-base alias; `RuntimeCreatedJointPathIsRejectedAsExplicitRoot` rejects an actual runtime-only joint (REQ AC-6).

- Discovery rejects FILE and an unattached owner before connection, reports active status only to its owner, emits only allowed identities without ordinary actor/shape creation or motion/geometry payloads, and finalizes on stop/detach (REQ AC-7).
- Warmup leaves the buffered request pending without activating sampling; the next public step applies it and emits its acknowledgment. An empty control poll steps without draining the held peer. A pending request delays owner-step submission until earlier peer work finishes, and both task receipts remain collectable (REQ AC-8).

Automated cases: `OmniPvdSelectiveCapture.*` and `GpuModes/OmniPvdSelectiveCaptureGpuTest.*` in `ovphysx/tests/c_unittests/test_omnipvd_recording.cpp`, `ovphysx/tests/c_unittests/test_c_api_compatibility.c`, and the descriptor layout case in `ovphysx/tests/python_tests/test_abi_checks.py`.

Discovery cases in the same public C++ test file: OmniPvdSelectiveCapture.TcpDiscoveryRejectsFileAndMissingAttachment, TcpDiscoveryCatalogIsOwnedAndHasNoSimulationPayload, TcpDiscoverySelectionCannotWidenInitialScope and TcpDiscoveryReceiverChoosesSecondaryData. These cover waiting ownership, owner-only catalog rows, no initial payload, stop/detach, denied scope expansion, subsequent selected recording and the receiver's choice of secondary data.

Receiver submission ordering and empty-poll coverage: `OmniPvdSelectiveCapture.TcpPendingRequestWaitsForQueuedPeerWithoutCollectingItsReceipt`. `TcpDiscoverySelectionCannotWidenInitialScope` also buffers its requests before the first warmup and confirms they remain pending until a public step (AC-8).
