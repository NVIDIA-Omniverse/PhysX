<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-SESSION-001
title: Session creation results and request identity
status: implemented
owner: ovphysx
---

## Description

`ovphysx_query`, `ovphysx_query_selection`, `ovphysx_read` and `ovphysx_write`
return optional, immutable summaries from their creating task, and a write session
opened for several attributes identifies every fetched group by its index in the
original request. Outputs follow the task contract of
[REQ-CAPI-ASYNC-002](REQ-CAPI-ASYNC-002.md): a receipt reports admission only, and
handles identify pending creation immediately and may be used by later submitted
calls; summaries are usable only after successful completion.

A summary describes one creation. It is not a live counter, it does not signal GPU
completion, and a write summary does not predict commit success. Tokens and
prim lists resolve through the attached Stage's own path dictionary; ovphysx has no
query-scoped dictionary or discovery accessor.

## Acceptance Criteria

- AC-1: **Optional outputs.** The query forms take a nullable `ovstage_query_result_t*`,
  `ovphysx_read` a nullable `ovphysx_read_result_t*`, and `ovphysx_write` a nullable
  `ovphysx_write_result_t*`. Handle and result slots are zeroed before validation. After
  acceptance the handle is nonzero, including while creation is pending or after
  it fails; only successful completion fills a supplied result. Rejection leaves
  both zero. Release retires a failed handle without a backend resource. NULL skips optional discovery,
  counting and summary work; required validation and write request identity still run.

- AC-2: **Query discovery snapshot.** A query result reports, as of creation, the number of
  matched prim identities and the attribute tokens the object type advertises. The
  attribute array belongs to the query and stays valid until accepted
  `ovphysx_release_query`. `query_result_id` and `all_handle` are zero; they are not OVStage
  resources and are never passed to OVStage query or release calls. Discovery is refreshed
  only by creating a new query. A query matching nothing is valid and reports zero.

- AC-3: **Read summary.** `total_prim_count` is the membership evaluated for this session,
  in the units of query discovery: for articulation joints it counts joints, while a joint
  group's `prims.count` counts its DOF rows. `attribute_count` equals the request length, and
  `attributes` holds one row per request in original order: the canonical token, the
  number of groups emitted for that request, and an `ovphysx_read_omission_t` bitmask. An
  unsupported name has `OVX_INVALID_TOKEN`, zero groups and
  `OVPHYSX_READ_OMISSION_UNSUPPORTED_ATTRIBUTE`, and is not interned.
  `OVPHYSX_READ_OMISSION_UNSUPPORTED_CONFIGURATION` and `OVPHYSX_READ_OMISSION_NOT_READY`
  classify applicable selected partitions that produced no group. Bits may coexist and may
  accompany a nonzero group count, which is partial availability. Zero bits with zero
  groups is a valid empty answer. Per-request group counts equal the groups a caller
  fetches to exhaustion for that request. Rows stay valid until accepted read release.

- AC-4: **Write summary.** `total_prim_count` is defined as for a read. `attribute_count`
  equals the request length, and `attributes` holds one row per request in original order:
  the canonical token from the attached Stage's dictionary and the number of groups planned
  for that request. Rows stay valid until accepted write release. A write-only control need
  not appear in read discovery.

- AC-5: **Request validation.** Admission rejects, with `OVPHYSX_API_INVALID_ARGUMENT`, a
  NULL attribute array with a nonzero count, an entry whose token is `OVX_INVALID_TOKEN` and
  whose string is NULL with a nonzero length, and for writes a NULL array or zero count. A
  token-bearing entry with a NULL string is not rejected for its ignored string length. The
  accepted task then canonicalizes every request before publishing a usable backend session,
  following ovstage's `ovx_token_or_string_t` contract: a non-NULL string takes precedence
  and the token is ignored; an entry with a NULL string names its attribute by token, which
  must resolve in the owning Stage's dictionary, or the task fails naming the request index;
  a canonical name requested twice, including a string
  and a token naming the same attribute, fails the task; a write request naming an
  attribute the object type cannot write fails the task. Read strings the type does not
  support are compared by their bytes, so different unknown names are separate unsupported
  rows. A zero-length read request opens a valid empty session, and a validated write whose
  query matches no object opens a valid empty session. A failed creation keeps its failed handle but publishes no
  summary or group.

- AC-6: **Known read failure fails creation.** A backend failure already known while a read
  is opened fails the creating task, keeps its failed handle and publishes no
  backend session or summary.
  Expected unavailability is reported through omission bits and is never a failure. A later
  fetch failure, such as a detached attachment, is a task error and never end of
  iteration.

- AC-7: **Write group identity.** `ovphysx_fetch_write_next` fills an
  `ovphysx_fetched_write_group_t`: `group` is the original producer-owned
  `ovstage_map_group_t` pointer and `attribute_index` indexes the original request table.
  Commit takes that `group` pointer; a copied descriptor is not a live group. Fetch order is
  not part of this contract: only `attribute_index` identifies the attribute, never tensor
  shape, fetch position or read row order.
  Successful exhaustion leaves `group == NULL` and `attribute_index == SIZE_MAX`; a failed
  fetch is never exhaustion. The outer struct may be copied after completion; the group it
  points to keeps the lifetime of REQ-CAPI-WRITE-001 AC-4.

- AC-8: **Selector state and ownership.** Accepted query release ends use of the query and
  its discovery result, but read and write sessions already opened from it remain valid,
  because each captures the selector state it needs at open. A whole-type query evaluates
  membership at each session open under the type's scope rule. Where active-actor reports
  are available, `OVPHYSX_SCOPE_ACTIVE` uses the most recently completed step's set;
  the existing awake fallback for scenes without those reports uses current sleep state.
  A selected query keeps the
  membership it resolved at creation. Handles, summaries and groups belong to their creating
  instance and attachment: after detach and reattachment with equal prim paths, operations on
  the older query, read or write fail, while a new query works.

- AC-9: **Dictionary route.** Summary tokens, group tokens and prim lists resolve through
  the attached Stage's path dictionary, reached in C as
  `ovstage_get_path_dictionary(stage)`. The public API has no
  query-dictionary or query-result fetch entry point. Token-only requests remain valid.

- AC-10: **Read group shape and lifetime.** `ovphysx_fetch_read_next` hands out OVStage's
  own `ovstage_read_group_t` carrying exactly one attribute column: `column_count == 1`,
  and `columns[0]` holds the emitted `attribute`, `semantic`, `is_array`, `data` and
  `meta`, with `is_delete` false and `ordinal` zero. `read_group_id` and `prims` stay on
  the group, and attributes that share prim rows still come back as separate groups. The
  group struct and its `prims.list` stay valid until accepted `ovphysx_release_group` for
  that `read_group_id` or accepted read release. The `columns[0]` descriptor and its
  numeric storage belong to the read session: they outlive the group's release and stay
  valid until accepted `ovphysx_release_read`. A fixed group's tensor stacks its rows along
  the leading dimension with the tuple width in `dtype.lanes`, except the five
  whole-articulation inverse-dynamics columns: one lane, and each row's dimensions as
  trailing axes (`jacobian` `[N, R, C]`, `massMatrix` `[N, M, M]`, `coriolisForce` and
  `gravityForce` `[N, M]`, `centroidalMomentum` `[N, 6, numDofs + 7]`; REQ-READ-INVDYN-001
  AC-1).

## Known limitations

- Execution follows REQ-CAPI-ASYNC-002. Ordered calls do not provide isolation
  between separate calls or a cross-call snapshot.

## Test References

- [TEST-CAPI-SESSION-001](../../tests/capi/TEST-CAPI-SESSION-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_query`, `ovphysx_query_selection`,
  `ovphysx_read`, `ovphysx_write`, `ovphysx_fetch_write_next` -- AC-1 through AC-9;
  `ovphysx_fetch_read_next` -- AC-10)
- ovphysx/include/ovphysx/ovphysx_types.h (`ovphysx_read_omission_t`,
  `ovphysx_read_attribute_result_t`, `ovphysx_read_result_t`,
  `ovphysx_write_attribute_result_t`, `ovphysx_write_result_t`,
  `ovphysx_fetched_write_group_t` -- AC-3, AC-4, AC-7)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (output clearing, admission validation and
  handle reservation and successful result publication -- AC-1, AC-5)
- ovphysx/include/ovphysx/ovphysx_types.h (the read-group note -- AC-10)
- ovphysx/src/ovphysxInternal/ovphysxInternalInterop.cpp and
  ovphysx/src/include/internal/sidecar/ovphysxInternalInterop.h (result forwarding
  across the runtime boundary -- AC-1 through AC-7; `ovphysx_internal_fetch_read_next`
  retaining the group descriptor by `read_group_id` -- AC-10)
- ovphysx/ovruntime/include/omni/physx/IOvxPhysicsRead.h and
  ovphysx/ovruntime/include/omni/physx/IOvxPhysicsWrite.h (the runtime contract this
  surface forwards, specified by REQ-READ-RESULT-001 and REQ-INPUT-CORE-001; the
  single-column group shape is REQ-READ-CORE-001)
- ovphysx/tests/c_unittests/test_session_results.cpp

## Dependencies

- [REQ-CAPI-ASYNC-002](REQ-CAPI-ASYNC-002.md) (receipts, output timing, ownership)
- [REQ-CAPI-WRITE-001](REQ-CAPI-WRITE-001.md) (write session lifecycle and commit)
- [REQ-CAPI-SELECT-001](REQ-CAPI-SELECT-001.md) (selected query membership)
