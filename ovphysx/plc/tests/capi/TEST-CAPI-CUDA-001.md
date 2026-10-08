<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CUDA-001
maps_to: REQ-CAPI-CUDA-001
type: integration
---

## Scenario

Direct OVStage attachment propagates an explicit `active_cuda_gpus` ordinal
during synchronous scene attachment. The regression runs in an isolated
process so no earlier attachment can change the private selector state.

## Given

- A process with process-wide CPU-only mode disabled.
- An ovphysx instance created with `active_cuda_gpus="0"`.
- Before attachment, the internal read-only test seam reports the automatic
  attach-time selector value `-1`.
- Multi-GPU scene distribution is set to `1` before attachment to represent
  stale process-global state from an earlier configuration.
- A minimal USD scene populated into an OVStage and sealed at its read ordinal.

## When

- `ovphysx_attach_ovstage()` attaches the populated OVStage.
- The internal selector and public typed multi-GPU mode are read after the
  synchronous call returns.

## Then

- OVStage attachment succeeds.
- The internal observation reports `0`, demonstrating that the public request
  reached the attach-time selector before the call returned (REQ AC-1).
- Multi-GPU scene distribution is `0` (REQ AC-2).
- The attached OVStage and ovphysx instance are destroyed without a stage leak.
