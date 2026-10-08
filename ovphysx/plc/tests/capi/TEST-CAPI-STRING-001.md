<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-STRING-001
maps_to: REQ-CAPI-STRING-001
type: integration
---

## Scenario

All ovphysx-produced string shapes honor their trailing-NUL guarantee while
input substring views remain accepted by length.

## Given

- Empty and populated error state, config buffers, metadata name/path arrays,
  and log callback strings; object-change forwarding is checked structurally
  from the SdfPath `c_str()` producer through the public trampoline.
- A non-NUL-terminated input buffer with an explicit length.

## When

- Each API returns or invokes its string-bearing result.

## Then

- Inputs are consumed by length without reading beyond it (AC-1).
- Empty and populated errors satisfy AC-2.
- Config getters satisfy AC-3 for a full buffer, a one-byte buffer, an
  undersized buffer, and a retry using the reported required capacity.
  Metadata names and path results satisfy the remaining AC-3 cases.
- Runtime log callback strings satisfy AC-4. Object-change paths are structural
  code evidence only: the SdfPath producer passes `std::string::c_str()` and the
  public trampoline preserves that pointer and length. No existing deterministic
  public stimulus produces a per-object event for a non-vacuous runtime test.
