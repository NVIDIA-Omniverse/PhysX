<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-SIM-OVSTAGE-BINDING-RESOLVE-001
maps_to: REQ-SIM-OVSTAGE-BINDING-RESOLVE-001
type: integration
---

## Scenario

Resolve explicit rigid-body paths without repeated stage-wide string queries,
while retaining row identity, live-membership checks, and existing diagnostics.

## Given

- A real OVStage instance with live, absent, deleted, and recreated paths.
- Batched requests containing reordered paths, repeated/canonical keys, and an
  invalid key, plus a 512-path cold request.
- CPU rigid-body bindings over explicit paths and mixed literal/wildcard paths
  that refer to the same articulation root body more than once.

## When

- Existence is checked cold, then checked again from the memo.
- The memo is invalidated after deletion and recreation.
- Bindings resolve requested paths and return their row metadata.

## Then

- Source results match the input slots and duplicate keys, with a single cold
  query; all-missing input produces false answers despite interned handles (AC-3).
- Missing stage availability reports failure with no positive cold results;
  successful all-missing reads can be memoized (AC-4).
- Invalidating the memo exposes deletion and recreation (AC-5).
- Binding rows preserve requested order and body-identity deduplication (AC-1).
- Existing partial-miss diagnostics and runtime clone tests remain valid
  (AC-1, AC-2).

## Automation

- Native target ovstage_source_bucket_unittests, CTest ovstage-source-bucket:
  BatchExistencePreservesOrderDuplicatesAndMissingPaths,
  BatchExistenceDoesNotTreatInternedPathsAsLive,
  BatchExistenceTracksDeletionAndRecreationAfterMemoInvalidation,
  BatchExistenceUsesOneProbeForManyColdPaths, and
  BatchExistenceReportsUnavailableStageWithoutPositiveAnswers.
- tests/c_unittests/test_tensor_binding.cpp:
  BatchedRigidBodyPathsPreserveOrderAndDeduplicate,
  BatchedRigidBodyPatternsDeduplicateArticulationAliases, and existing
  ExplicitPrimPathPartialMissStillLogsError and zero-match tests.
- Existing tests/python_tests/cpu_tests/test_tensor_bindings_api.py:
  TestContactBinding.test_contact_binding_matches_runtime_clones.
- Repeat the 4k DELMIA binding-creation measurement; correctness tests use query
  counts rather than wall-clock thresholds.
- Read-operation failure, malformed group, and truncated-read cleanup paths are
  reviewed explicitly; this integration fixture does not inject transport faults.
