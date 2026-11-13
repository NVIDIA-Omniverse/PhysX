<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-READ-001
title: Read producer handoff, stream ordering and capture
status: implemented
owner: ovphysx
---

## Description

`ovphysx_read` takes the caller's producer handoff, an `ovstage_cuda_sync_t`, as
`ovphysx_commit_group` does. A DirectGPU device group is gathered on the caller's stream
with no host wait, a held query keeps the read's device columns for its next read, and a
read over a held query can be captured into a CUDA graph and replayed, so one graph can
cover a step's read, policy and write. This requirement owns that contract at the C
surface; the runtime behavior behind it is
[REQ-READ-DEVICE-001](../../../ovruntime/plc/requirements/output/REQ-READ-DEVICE-001.md)
AC-11 through AC-13, and the decision
[ADR-0055](../../../ovruntime/plc/adr/ADR-0055-capturable-session-reads.md).

## Acceptance Criteria

- AC-1: **Producer handoff and stream ordering.** `ovphysx_read` takes
  `ovstage_cuda_sync_t cuda_sync`; the read observes everything queued on
  `cuda_sync.stream` and `cuda_sync.wait_event`, and `{0, 0}` asserts nothing is
  outstanding. A DirectGPU device group of rigid bodies, links, whole articulations,
  joints, tendons, point instancers, deformables or particle sets is gathered on the
  device like a CUDA library call on `stream`: after the work queued on it and after
  `wait_event`, with work queued on it afterwards ordered after the gathers, and no host
  wait. A zero `stream` gathers on the legacy default stream. A `stream` of another CUDA
  context than the scene's is waited for on the host, and so is the producer of every
  other group.

- AC-2: **Completion handoff.** Outside a capture, a device group's `data.cuda_sync` is
  `{0, event}`, the event recorded after its gathers; a consumer on the stream the read was
  given is already ordered after them when that stream is in the scene's CUDA context. A
  captured read's groups carry `{0, 0}`.

- AC-3: **Held columns and release.** Once a read over a held query is released, the
  query's next read with the same attributes and rows hands out the same column pointers
  and gathers into them after the read before it. The query keeps one set of columns per
  scene and kind of object. `ovphysx_release_read` does not wait on the host for columns
  the query keeps, nor for a captured read's replays: the query's next read waits for the
  last replay's gathers on the device. When the query already keeps columns of that scene
  and kind, from another read released since, release frees the read's own after a host
  wait instead.

- AC-4: **Capture.** A read on a stream being captured records its gathers, and each
  replay refreshes its columns from the state current when it runs. It fails before
  anything is queued, leaving the capture valid, when the query holds no plan for it (no
  released read with the same attributes and rows, rows changed since, or another open read
  over the query), when it holds a host-resident or contact group,
  when its stream belongs to another CUDA context, or when it would use the legacy default
  stream while a blocking stream of its context is being captured.

## Test References

- [TEST-CAPI-READ-001](../../tests/capi/TEST-CAPI-READ-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_read` "The caller's stream, and capture",
  `ovphysx_release_read` -- AC-1 through AC-4)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`ovphysx_read`, forwarding `cuda_sync` -- AC-1)
- ovphysx/src/ovphysxInternal/ovphysxInternalInterop.cpp (`ovphysx_internal_read_outputs`,
  forwarding `cuda_sync` to the runtime read -- AC-1)
- ovphysx/tests/c_unittests/test_read_capture.cpp

## Dependencies

- [REQ-CAPI-SESSION-001](REQ-CAPI-SESSION-001.md) -- the read session this orders
- [REQ-CAPI-WRITE-001](REQ-CAPI-WRITE-001.md) AC-5, AC-17 -- the commit's handoff and capture,
  which this mirrors
- [REQ-READ-DEVICE-001](../../../ovruntime/plc/requirements/output/REQ-READ-DEVICE-001.md)
- [ADR-0055](../../../ovruntime/plc/adr/ADR-0055-capturable-session-reads.md)
