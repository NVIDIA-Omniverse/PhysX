<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-COMPANION-001
maps_to: REQ-CAPI-COMPANION-001
type: integration
---

## Scenario

Acquire a companion table, borrow a real CPU attachment, and reject destructive
lifetime transitions until the session is closed.

## Given

- A core instance with an ovstage-populated scene, dedicated C API table, and ctypes wrapper.
- Output canaries for rejected negotiation and counters from a scene with known bodies.

## When

- Negotiate wrong version/size, open sessions, query statistics, request destroy/shutdown, detach and reattach, then close sessions.
- Create an independent companion preview; retire owners in both orders, with and without recording, and reject recording transitions while preview work is pending.
- Hold concurrent companion operations, attempt a process transition, and release the operations one at a time. Exercise two separate coordinator objects and nested transition/native-borrow scopes.
- Inject a retryable native destroy status into Python and retry after it clears.

## Then

- Invalid negotiation does not write output; valid tables contain only the supported narrow operations (REQ AC-1).
- Lease rejection preserves native and Python ownership; normal calls remain usable and close releases the gate (REQ AC-2).
- Handles do not alias after reuse, stale generations fail, closing one session preserves peers, and copied counters report the selected owner (REQ AC-3).

- Last-core retirement preserves preview stepping; preview retirement preserves core recording and task receipts. Shutdown rejects the preview lease without consuming initialization (REQ AC-4).

- Admission and lease state do not cross coordinator objects; same-owner recursion is rejected, internal transition nesting succeeds, and a transition remains refused until both concurrent operations return (REQ AC-4, AC-5).

- A build-matched C++ object borrow rejects recursive core waiting/collection without modifying output or consuming tasks (REQ AC-5).

## Executable coverage

- [C tests](../../../tests/c_unittests/test_companion.cpp): `CompanionDiscovery.*`, `CompanionBridge.*`, `CompanionCoordinator.*`.
- [Python integration](../../../companion/tests_native/test_bridge.py): all cases.
- [Python failure injection](../../../tests/python_tests/test_lifecycle_failure_injection.py): `test_companion_lease_rejection_retains_owner_until_retry`.

- [C11 header compilation](../../../tests/c_unittests/test_companion_header.c) checks the standalone companion header and fixed-width table prefix.

- [Mixed native lifetime](../../../companion/tests_native/test_preview.py): `test_last_core_owner_and_recording_in_fresh_process`, `test_preview_pending_isolation_and_close_preserves_playback_receipt`, `test_recording_gate_tracks_every_pending_preview` (fetch/close retirement orders).

- [Native plugin lifetime](../../../companion/tests_native/test_plugins.py): module service leases, last-core retirement, safe close, and separate table negotiation (REQ AC-1, AC-4).

- [Cooking ownership](../../../companion/tests_native/test_cooking.py): table/output canaries, last-core teardown preserving cooking, successful cooking during a pending preview and result validity after service close (REQ AC-1, AC-4).

CCT discovery rejects incompatible versions without modifying the output table;
[the native CCT tests](../../../companion/tests_native/test_cct.py) exercise this
through the public shared library.

[Debug frame tests](../../../companion/tests_native/test_debug.py) verify table
negotiation canaries, owned frame release and copies after native owner retirement.

[OVD decoder tests](../../../companion/tests_native/test_ovd.py) verify exact table
negotiation, failed-document canaries and stopped decoding after runtime teardown.

Native-access coverage is executed and passing:
[test_companion_access.cpp](../../../tests/c_unittests/test_companion_access.cpp),
`CompanionNativeAccess.BuildNegotiationPreservesRejectedOutput` and
`CompanionAccess.TypedBorrowRejectsRecursiveWaitAndStaleRecord`.

The C++ `CppWrapperRetainsBusyOwnerAcrossResetAndMove` case verifies reset rejection preserves the handle and move assignment transfers both owners without hidden destruction (AC-2).
