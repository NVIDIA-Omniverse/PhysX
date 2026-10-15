<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-READ-CONTACT-002
title: Contact Query Through the ovphysx Read API
status: implemented
owner: ovphysx
---

## Description

`ovphysx_query_contacts` opens a contact-sensor query whose rows the caller chooses, each paired with its
own filter objects: the public form of the contact binding's sensor and filter patterns on the read API.
Each sensor and filter selection is an `ovphysx_query_selection_t` (explicit prim handles or a glob), so it
follows `ovphysx_query_selection`'s rules. The query is read with `ovphysx_read` like any query. Python
exposes it as the `contacts=` keyword of `PhysX.query`, with explicit `read_sync` / `read_tokens_sync` conveniences. The runtime resolves and pairs the
selections; this requirement owns the public surface that validates them and reports the runtime's refusal.

## Acceptance Criteria

- AC-1: **Malformed arguments are refused before a task is created.** `ovphysx_query_contacts(handle,
  selections, selection_count, filters_per_sensor, out_query, out_summary)` returns `OVPHYSX_API_INVALID_ARGUMENT`
  synchronously, with no task, when `out_query` or `selections` is NULL, `selection_count` is 0,
  `filters_per_sensor` exceeds 21845, a selection's `filters` is NULL while `filters_per_sensor` is not 0,
  or any sensor or filter selection names neither a pattern nor prim handles, has a count without handles,
  contains `OVX_INVALID_PRIMPATH`, or has a pattern with an embedded NUL, an oversized path component, or
  no leading `/` (a relative pattern, which every pattern-taking entry point refuses, as
  `ovphysx_query_selection` does). Each message names the selection or filter at fault.
  A valid call opens a query that `ovphysx_read` reads and `ovphysx_release_query` releases.
  The optional `out_summary` is filled only on successful task completion, counts selected reporting
  sensors, and advertises the query's applicable attribute names. Its nested arrays remain query-owned.
- AC-2: **A refusal by the runtime fails the task with the runtime's message.** When the runtime refuses
  the query (a filter matching neither one object nor one per sensor, or a sensor naming one filter object
  twice), `ovphysx_wait_task` reports `OVPHYSX_API_ERROR` and the task's error text is the runtime's
  message, which names the rule; `out_query` stays 0.
- AC-3: **Python mirrors it on `PhysX.query`.** The `contacts=` keyword takes a list of
  `ovphysx.types.ContactSelection(sensors, filters)`, each selection a glob `str` or a list of prim
  handles, and returns an operation that completes with the contact query; its rows come back in selection order. Before any native call it
  raises `ValueError` when `contacts` is empty, combined with `prim_paths` / `pattern`, used with a type
  other than `CONTACT_SENSOR` or with `scope=ACTIVE`, when its selections carry different numbers of
  filters, when a selection is an empty pattern or an empty handle list, when `filters` is a `str` rather
  than a list of selections (it would be one filter per character), or when a handle list holds a
  non-integer. A runtime refusal raises the
  native error when its operation is collected (AC-2). Each query call opens a fresh contact query. `read_sync` / `read_tokens_sync` open, read and release their own query; repeated calls over the same rows reduce
  through one device reducer (REQ-READ-CONTACT-001 AC-9); an array kept past its result keeps its own read's
  values.
- AC-4: **The filtered attributes are public, read-only names.** `OVPHYSX_ATTR_NORMAL_CONTACT_FORCE_MATRIX`
  (`"normalContactForceMatrix"`) and `OVPHYSX_ATTR_FRICTION_CONTACT_FORCE_MATRIX`
  (`"frictionContactForceMatrix"`) name the force matrices a contact query with filters reads; each row is
  a sensor's F filter forces, F × 3 float32 values (a Warp array of shape `(S, 3F)`, CPU or CUDA as the
  scene runs). `ovphysx_writability` classifies both `OVPHYSX_WRITABILITY_READ_ONLY` for
  `OVPHYSX_QUERY_TARGET_CONTACT_SENSOR`.
- AC-5: **The filtered detail names are public, read-only names.** `OVPHYSX_ATTR_FILTERED_CONTACT_FORCE`,
  `_POINT`, `_NORMAL`, `_SEPARATION` and `OVPHYSX_ATTR_FILTERED_FRICTION_FORCE`, `_POINT` name the per-element
  detail a contact query with filters reads: array groups, one tensor per sensor with at least one element
  this step (a list of Warp arrays in Python, shape `(n,)` or `(n, 3)`, CPU or CUDA as the scene runs),
  ordered by filter column. `OVPHYSX_ATTR_FILTERED_CONTACT_COUNT` and `OVPHYSX_ATTR_FILTERED_FRICTION_COUNT`
  name their int32 count companions, one row per sensor with F lanes (a Warp array of shape `(S, F)`), which
  split each sensor's tensor into per-filter runs. `ovphysx_writability` classifies all eight
  `OVPHYSX_WRITABILITY_READ_ONLY` for `OVPHYSX_QUERY_TARGET_CONTACT_SENSOR`.

## Test References

- [TEST-CAPI-READ-CONTACT-003](../../tests/capi/TEST-CAPI-READ-CONTACT-003.md)

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_query_contacts`)
- ovphysx/include/ovphysx/ovphysx_types.h (`ovphysx_contact_sensor_selection_t`)
- ovphysx/include/ovphysx/ovphysx_attributes.h (the `OVPHYSX_ATTR_*_CONTACT_FORCE_MATRIX` names: AC-4; the `OVPHYSX_ATTR_FILTERED_*` names: AC-5)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`ovphysx_query_contacts`, `contactSelectionError`,
  `runtimeSelection`: AC-1, AC-2; the contact rows of `kWritabilityTable`: AC-4, AC-5)
- ovphysx/python/ovphysx/types.py (`ContactSelection`), ovphysx/python/ovphysx/api.py
  (`PhysX.query`, the `contacts=` keyword of `read_sync` / `read_tokens_sync`) and
  ovphysx/python/ovphysx/_bindings.py (`ovphysx_contact_sensor_selection_t`): AC-3
- ovphysx/tests/c_unittests/test_query_contacts.cpp, ovphysx/tests/python_tests/test_contact_query.py and
  ovphysx/tests/python_tests/cpu_tests/test_contact_query_cpu.py; ovphysx/tests/c_unittests/test_writability.cpp
  and ovphysx/tests/python_tests/test_writability.py (AC-4, AC-5)

## Dependencies

- REQ-CAPI-SELECT-001 (`ovphysx_query_selection_t` and its validation rules)
- REQ-CAPI-READ-CONTACT-001 (the contact-sensor type and its net-force attributes)
- REQ-READ-CONTACT-002 (ovruntime: the contact query this surface forwards to — resolution, pairing, rows)
- REQ-READ-CONTACT-003 and REQ-READ-CONTACT-004 (ovruntime: the force matrices and the filtered detail)
