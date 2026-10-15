<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-SESSION-001
maps_to: REQ-CAPI-SESSION-001
type: integration
---

## Coverage status

Automated cases exist for AC-1 through AC-5 and AC-7 through AC-9. AC-6 is
specification only; see *Why not fully covered* below.

## Scenario

Query, read and write creation publish immutable summaries only after successful
completion; invalid requests publish nothing; equal-shaped write attributes stay
distinguishable by original request index; sessions outlive their query but not
their attachment.

## Given

- A CPU instance attached through OVStage to
  `tests/data/boxes_falling_on_groundplane.usda` and stepped once.
- The attached Stage's path dictionary, from
  `stage->vtable->get_path_dictionary(stage->context)`.

## When

- Create a whole-type rigid-body query with a result, then read `position`, two
  different unknown names and `orientation` with a read result; release the query;
  fetch and release every group.
- Read and write with one entry holding a Stage-dictionary `position` token and a NULL
  string of nonzero length.
- Submit a read and a write whose two requests are `position` by string and by that
  token, and a read whose token is invalid, with output slots prefilled with nonzero
  values.
- Open a write for `linearVelocity` and `angularVelocity` with a result, release
  the query, fetch every group, fill each by its `attribute_index`, and commit in
  reverse fetch order; read both attributes back.
- Open a write for `position`, an unknown name and `linearVelocity`.
- Create a selected query whose pattern matches nothing, open a read and a write
  over it, release the query and fetch the write.
- Open a query, a read and a write; reset the stage and reattach the same file; use
  the old query, read and write; create a new query.

## Then

- The query result's count is nonzero, `query_result_id` and `all_handle` are zero,
  and the read summary's membership equals it (REQ AC-1, AC-2, AC-3).
- Read rows keep request order: supported rows have tokens, groups and zero bits;
  each unknown row has `OVX_INVALID_TOKEN`, zero groups and only the
  unsupported-attribute bit (REQ AC-3, AC-5).
- Fetching after query release succeeds, and the groups fetched per emitted token
  equal each row's group count (REQ AC-3, AC-8, AC-9).
- The token-bearing entry is admitted despite its malformed string and reports that
  token (REQ AC-5, AC-9).
- Duplicate string/token requests and the invalid token are accepted, fail at wait,
  and leave the handle zero and the summary zeroed (REQ AC-1, AC-5).
- The multi-attribute write reports two distinct tokens; every fetched group has an
  index below two; per-index group counts equal the summary; reverse-order commits
  of the retained original pointers succeed; the read-back values follow the index
  each group was filled for; exhaustion reports `NULL` and `SIZE_MAX`
  (REQ AC-4, AC-7, AC-8).
- The write naming an unknown middle attribute fails at wait and publishes no
  handle or summary (REQ AC-5).
- The no-match selection reports zero in every summary with zero omission bits, and
  the write fetched after query release reaches exhaustion (REQ AC-2, AC-3, AC-4,
  AC-5, AC-7, AC-8).
- After reattachment, reading from the old query fails with zeroed outputs, fetches on
  the old read and write fail with `NULL` / `SIZE_MAX` outputs, the new query differs
  from the old handle and reports matches, and every old handle still releases
  (REQ AC-7, AC-8).

## Automated coverage

`ovphysx/tests/c_unittests/test_session_results.cpp`:

- `SessionResultsTest.ReadMetadataPreservesRequestIdentityAndQueryIndependence`
- `SessionResultsTest.CanonicalDuplicatesAndInvalidTokensFailWithoutPublishedOutputs`
- `SessionResultsTest.MultiAttributeWriteUsesOriginalRequestIndexAndOriginalGroupPointer`
- `SessionResultsTest.InvalidMiddleWriteAttributePublishesNothing`
- `SessionResultsTest.SelectedEmptyQueryReturnsValidEmptySessionSummaries`
- `SessionResultsTest.ReattachingEqualPathsDoesNotReviveOldSessions`

Evidence status: no execution result is recorded in this specification.

Omission-bit classification for unready and unsupported configurations, and ACTIVE
membership shared by read and write sessions, are covered at the runtime boundary by
TEST-READ-RESULT-001.

## Why not fully covered

- AC-6: the installed library has no fault-injection switch that makes a backend
  gather fail during open, so failure-at-open is verified by source review of the
  runtime read and its TEST-READ-RESULT-001 specification.
