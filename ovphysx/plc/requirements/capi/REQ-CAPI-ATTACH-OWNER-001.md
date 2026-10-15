<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-ATTACH-OWNER-001
title: Per-Instance Ovstage Attach Ownership
status: implemented
owner: ovphysx
---

## Description

Each ovphysx instance owns at most one live runtime attachment, and several
instances may be attached at the same time. The facade passes that instance's
attachment handle to runtime operations. Attaching, detaching, or destroying
one instance does not select or tear down another instance's attachment.

The same ovstage instance cannot be attached twice. Process-wide limits remain
the hard CPU-only mode, one SDK tolerance scale, and recording transitions that
require every attached instance to be idle.

## Acceptance Criteria

- AC-1: `ovphysx_attach_ovstage()` on a second instance with a separate stage
  succeeds while the first instance is still attached. `ovphysx_is_stage_attached`
  confirms each instance owns its stage. The first instance's retained query
  remains valid, and both instances can step.
- AC-2: `ovphysx_detach_ovstage()` detaches only the calling instance's runtime
  attachment. The peer remains attached, its retained query remains valid,
  and it can still step.
- AC-3: `omni_sdk_physx_destroy()` releases that instance's runtime attachment
  even when its ordinary private teardown attempt fails. It does not submit a
  nested public detach task. An
  uncollected task failure alone does not make an internal drain fail.
  Destruction must not leave a runtime attachment owned by a handle that no
  longer exists or prevent that ovstage instance from being attached again.
- AC-4: Detaching one instance leaves the peer's existing nonempty rigid-body-pose
  read finite and unchanged until the peer itself steps. The detached instance
  reports `false` from `ovphysx_is_stage_attached` for its former stage.

## Test References

- TEST-CAPI-ATTACH-OWNER-001

## Code References

- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_attach_ovstage`, `ovphysx_detach_ovstage`, `omni_sdk_physx_destroy`)
- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_attach_ovstage`, `ovphysx_is_stage_attached`)
- ovphysx/tests/c_unittests/test_multi_instance.cpp (two live attaches, detach and destroy of one, same-stage rejection)
- ovphysx/tests/c_unittests/test_global_lifecycle.cpp (destroy-under-failed-op attachment release, across shutdown/reinitialize)
- ovphysx/tests/python_tests/lifecycle_tests/test_multi_instance.py (two live GPU reads)

## Dependencies

- REQ-CAPI-DETACH-001 (teardown-call ordering within a single instance's detach)
