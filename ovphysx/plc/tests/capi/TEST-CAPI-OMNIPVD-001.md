<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OMNIPVD-001
maps_to: REQ-CAPI-OMNIPVD-001
type: integration
---

## Scenario

Retired typed and raw OVD startup keys are rejected with migration guidance.

## Given

- Reserved bool slots 3/4, int32 slots 2/3, and string slots 0/2/3.
- Raw OVD startup keys with slash-prefixed and slashless Carbonite paths.
- Fresh and live-instance lifecycles, scoped capability values, and unrelated legacy keys.

## When

- Compare current config enums with Python parity values and compile the retained headers.
- Submit each retired key through creation and post-create configuration, including valid old values.
- Read the retired numeric keys through legacy typed getters.
- Read scoped capability after rejection; exercise unrelated legacy configuration.

## Then

- Retired startup builders/header are absent; reserved slots and unrelated key ordinals remain stable (REQ AC-1).
- Every retired typed/raw key returns `OVPHYSX_API_INVALID_ARGUMENT`, with guidance to the global capability and explicit recording start, for both raw path spellings. Legacy typed getters reject retired slots with the same result and guidance (REQ AC-2).
- Rejection leaves configured capability unchanged. `InheritedOutputEnabledDoesNotAutoStart` and `PlainCreateSupportsLateRecording` verify dormant create/attach separately; unrelated legacy keys retain their behavior (REQ AC-3).

## Executable coverage

`tests/c_unittests/test_omnipvd_recording.cpp`:

- `OmniPvdGlobalConfig.RetiredOvdEntriesAreRejected`
- `OmniPvdGlobalConfig.RetiredRawOvdPathsAreRejected`
- `OmniPvdGlobalConfig.RetiredOvdGettersAreRejected`
- `OmniPvdGlobalConfig.PlainCreateSupportsLateRecording`
- `OmniPvdGlobalConfig.InheritedOutputEnabledDoesNotAutoStart`
