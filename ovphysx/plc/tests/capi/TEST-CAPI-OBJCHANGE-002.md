<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OBJCHANGE-002
maps_to: REQ-CAPI-OBJCHANGE-002
type: integration
---

## Scenario

Object-change callbacks tell the caller when a cached native pointer must be
dropped, and they stay silent for initial population and clone. Covers REQ
AC-1 through AC-4.

## Given

- Public comments on `ovphysx_object_destroyed_fn`,
  `ovphysx_all_objects_destroyed_fn`, and `ovphysx_subscribe_object_changes`.
- For detach: an attached `basic_simulation.usda` stage that has completed one
  step, with a subscription in place.
- For clone: a subscription registered before attach, and a clone issued before
  the first step. Clone after the first step is rejected.
- For initial population: a subscription registered before attach.

## When

- The header comments are read (`AC-1`).
- `ovphysx_detach_ovstage` runs with both the per-object destruction callback and
  the all-objects callback set (`ResetDeliversNoPerObjectDestruction`).
- A fresh stage is attached, its attach ordinal is replayed through
  `ovphysx_update_from_ovstage`, and one step runs
  (`InitialPopulationDeliversNoCreatedCallbacks`).
- `/World/envs/env0` is cloned before the first step
  (`CloneDeliversNoObjectChangeCallbacks`).

## Then

- The destruction and all-objects comments say the cached pointer is unusable
  once the callback returns and must not be `release()`d. The subscribe comment
  says a `physxPointer` read runs only after the triggering call returns, not
  from inside the creation callback (REQ AC-1).
- Detach reports at least one all-objects callback and zero per-object
  destruction callbacks (REQ AC-2).
- Initial attach, ordinal replay, and the following step report zero created
  and zero destroyed callbacks (REQ AC-3).
- The successful clone reports zero created, destroyed, and all-objects
  callbacks (REQ AC-4).
