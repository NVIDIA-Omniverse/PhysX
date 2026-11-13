<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ARTICULATION-NAMES-001
maps_to: REQ-CAPI-ARTICULATION-NAMES-001
type: integration
---

## Scenario

The articulation DOF, body, and joint name getters and tensor-binding prim-path
getter make a short caller buffer detectable while preserving a valid ordered
prefix.

## Given

- An articulation binding for `links_chain_sample.usda`.
- Metadata reporting more than one DOF, body, and joint name.
- A name array with capacity one less than the corresponding metadata count
  and a canary entry immediately beyond the advertised capacity.
- An articulation binding selecting both articulations in
  `two_articulations.usda`, with a one-entry path buffer and a canary second
  entry.

## When

- Each articulation name getter is called with the undersized capacity.
- Each getter is also called with capacity equal to the metadata count.
- The DOF-name getter is called with a NULL zero-capacity buffer and with a
  non-NULL zero-capacity buffer.
- The prim-path getter is called with full capacity and with capacity one.

## Then

- Each undersized call returns `OVPHYSX_API_BUFFER_TOO_SMALL` and reports the
  complete metadata count through `out_count` (REQ AC-1).
- Each undersized call writes the same ordered, NUL-terminated prefix returned
  by the complete call and leaves the canary entry unchanged (REQ AC-2).
- Each complete call returns `OVPHYSX_API_SUCCESS`, reports the complete count,
  and writes every name (REQ AC-3).
- The NULL zero-capacity name call returns `OVPHYSX_API_INVALID_ARGUMENT`; the
  non-NULL call returns `OVPHYSX_API_BUFFER_TOO_SMALL`, reports complete demand,
  and leaves its canary unchanged (REQ AC-1, AC-2).
- The short prim-path call returns `OVPHYSX_API_BUFFER_TOO_SMALL`, reports both
  selected paths, writes the same first path as the complete call, and leaves
  the second canary entry unchanged (REQ AC-4).

## Implementation

- `ovphysx/tests/c_unittests/test_articulation_metadata.cpp`
  (`GetDofNamesZeroCapacity`, `NameGettersReportDemandAndBufferTooSmall`,
  `GetPrimPathsReportsDemandAndBufferTooSmall`)
