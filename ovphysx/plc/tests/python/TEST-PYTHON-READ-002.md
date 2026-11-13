<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-READ-002
maps_to: REQ-PYTHON-READ-001
type: integration
---

## Scenario

A read step captured into a CUDA graph on Warp's stream, through the public Python API, replays
against the state after each step, alone or with a policy and a write (AC-8).

## Given

- A fresh process -- device settings are per process -- with a DirectGPU instance created on CUDA
  device 0, so physics shares Warp's primary context.
- `boxes_falling_on_groundplane_gpu.usda`, stepped once, with a held rigid-body query.
- Then `two_articulations_gpu.usda`, stepped once, with held link and joint queries.

## When

- Under `wp.ScopedCapture` on Warp's stream, before any read over the query: a marker fill, then a
  `position` read and a `mass` read on that stream. The graph is replayed after the capture.
- A `position` read over the rigid-body query is made on the stream outside a capture and
  released, its arrays dropped; a `position` write is opened. Under `wp.ScopedCapture`: the read,
  a Warp kernel writing each position plus a device shift array into the write's column, and the
  commit. The scene steps before each of two replays, with the shift reloaded each time.
- `position` reads over the links and `jointPosition` reads over the joints are made on the stream
  and released, and both reads are captured into one graph; reads without a stream, still to be
  released, are pending when the capture starts. The scene steps five times before each of two
  replays.

## Then

- Both reads before any read over the query fail when collected, naming the missing read and the
  host column, and the replay runs the marker fill: the capture went on (REQ AC-8).
- Each replay leaves every body at its position before the replay plus the shift it ran with: the
  read refreshed its arrays from the state after the step (REQ AC-8).
- Each replay's arrays match a read outside the capture, and the values differ from the previous
  replay's; the pending releases are not performed inside the capture, which would invalidate it
  (REQ AC-8).

## Code References

- `ovphysx/tests/python_tests/test_read_capture_gpu.py`: `test_read_step_replays_from_a_cuda_graph`.

## Scenario: A stream of another CUDA context

A read on a capturing stream that does not share the scene's CUDA context fails before anything is
queued on it (AC-8).

### Given

- The suite's shared DirectGPU instance, created without a CUDA device, so physics runs in its own
  context, and `boxes_falling_on_groundplane_gpu.usda` stepped once, with a held rigid-body query.

### When

- Under `wp.ScopedCapture` on Warp's stream, a marker fill, then a `position` read on that stream;
  the graph is replayed after the capture, and the read is repeated outside a capture.

### Then

- The captured read fails when collected, with an error naming another CUDA context, and the
  replay runs the marker fill; outside the capture the read waits for the stream on the host and
  returns device columns (REQ AC-8).

### Code References

- `ovphysx/tests/python_tests/test_read_capture_gpu.py`:
  `test_read_capture_on_another_cuda_context_is_refused`.

## Scenario: A producer's event

A read given the event a producer stream recorded reads what that stream wrote, and holds the event's
owner until its open is collected (AC-8). The write reaches PhysX before the read whether or not the
read waits for the event, so this scenario does not show that wait: the runtime's
[TEST-READ-DEVICE-003](../../../ovruntime/plc/tests/output/TEST-READ-DEVICE-003.md) does.

### Given

- A fresh process with a DirectGPU instance created on CUDA device 0, and
  `boxes_falling_on_groundplane_gpu.usda` stepped once, with a held rigid-body query.
- A second Warp stream, the producer.

### When

- A `position` write is filled with 7 on the producer stream and committed there, and the producer
  records a `wp.Event`.
- A `position` read on Warp's stream is opened with `cuda_wait_event` the event's handle and
  `event_owner` the event, and the test drops its own reference before collecting the open.

### Then

- The event is still alive when the open is collected, and once Warp's stream is synchronized every
  position the read returns is 7 (REQ AC-8).

### Code References

- `ovphysx/tests/python_tests/test_read_capture_gpu.py`: `test_read_takes_a_producer_event_and_keeps_its_owner`.

## Scenario: An IsaacLab-style control step

A control step over cloned robots -- read, policy, write -- replays from one CUDA graph between physics
steps (AC-8).

### Given

- A DirectGPU instance created on CUDA device 0, `ant_envs_gpu.usda` (one position-driven Ant) cloned
  into 16 environments, stepped once, and held articulation and joint queries, each read once on Warp's
  stream and released.
- An open `jointPositionTarget` write over the joint query.

### When

- Under `wp.ScopedCapture` on Warp's stream: a read of the root state, a read of the joint positions and
  velocities, a Warp kernel gathering them into per-environment observations, a Warp kernel evaluating a
  fixed linear policy into the write's column, and the commit.
- Twenty times: the graph is replayed, the stream synchronized, and the scene stepped four times. Every
  fifth time, half the environments' joints are then reset to the middle of their limits, at rest,
  through a masked write outside the graph, with no step before the next replay.

### Then

- At every replay the observations match a read outside the graph made just before it, the actions match
  the policy evaluated on the host from those observations, and a read of `jointPositionTarget` returns
  them (REQ AC-8).

### Code References

- `ovphysx/tests/python_samples/rl_step_capture.py`, run by `scripts/test_python_samples.cmake`.

All cases need a CUDA device and run in the GPU pass. The native capture path is covered in the
runtime by [TEST-READ-DEVICE-003](../../../ovruntime/plc/tests/output/TEST-READ-DEVICE-003.md).
