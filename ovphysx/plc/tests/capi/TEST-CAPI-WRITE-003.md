<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-WRITE-003
maps_to: REQ-CAPI-WRITE-001
type: integration
---

## Scenario

A write step captured into a CUDA graph on Warp's stream, through the public Python API,
replays into the simulation with new inputs at every replay (AC-17).

## Given

- A fresh process -- device settings are per process -- with a DirectGPU instance created
  on CUDA device 0, so physics shares Warp's primary context.
- `boxes_falling_on_groundplane_gpu.usda`, stepped once, a held rigid-body query and an
  open `position` write over it, and a device source buffer.
- Then `two_articulations.usda`, stepped once, held joint and articulation queries and an
  open `jointPosition` write over every joint.
- Then `ovstage_output_instancer.usda` on a DirectGPU scene, with a translate op on its point
  instancer, stepped once and read once, and a held query selecting the instancer.

## When

- Under `wp.ScopedCapture` on Warp's stream: the source is copied into the group's column
  and the group committed with that stream. The graph is replayed twice, the source
  reloaded with new positions before each replay and the open session fetched past its
  last group; the scene then steps.
- A second write captures the same fill and a masked commit whose device mask is loaded
  outside the graph; it is replayed with the even rows selected, then the odd rows.
- A third capture holds the joint fill, the joint commits and
  `update_articulations_kinematic(query=...)` with the same stream; the source is set to
  5 degrees and the graph replayed. The scene then steps before each of two more replays, of
  -4 and 2.5 degrees.
- A fourth capture holds an instancer `position` fill and commit, with local positions at
  x = 1 and 6. The graph is replayed, the instancer is moved to x = 20 through a stage update
  and the graph replayed again; the write is then released, and a fresh write commits the same
  values outside any capture.

## Then

- Capturing writes nothing; each replay leaves every body at the values the source held
  when it ran, and the scene steps on from them (REQ AC-17).
- Each masked replay writes the rows its mask selects at that replay and leaves the others
  where they were (REQ AC-17).
- After the third replay every joint reads 5 degrees and the two outer links of each
  articulation have moved with no step: the refresh ran inside the graph (REQ AC-17). Each
  later replay, after a step, leaves every joint at its new value and moves the outer links
  again.
- The instancer replay first reads back x = 1 and 6; after the move it reads x = -19 and -14,
  since the graph still writes in the frame the instancer had when the write was opened. The
  fresh write reads back x = 1 and 6 (REQ AC-17).
- With replay disabled, or the refresh left out of the graph, the respective checks fail.

## Code References

- `ovphysx/tests/python_tests/test_write_capture_gpu.py`:
  `test_write_step_replays_from_a_cuda_graph`.

## Scenario: A reused column filled on another stream

A reused device column filled on another stream than Warp's, made to wait for Warp's stream first
as the guide says, takes each write's values (AC-17).

### Given

- A fresh DirectGPU instance on CUDA device 0, which shares Warp's CUDA context, and
  `boxes_falling_on_groundplane_gpu.usda` stepped once, a held rigid-body query and a Warp
  stream standing in for another framework's.

### When

- Twice: a `position` write is opened over the query, a stream wrapping that stream's handle
  waits for Warp's current stream, the column is filled on it and committed with it, so the
  commit is ordered on that stream; the stream is synchronized, the positions are read, and the
  write is released.

### Then

- Each write reads back the values it filled, and the second write reused the first one's
  column (REQ AC-17).

### Code References

- `ovphysx/tests/python_tests/test_write_capture_gpu.py`:
  `test_reused_column_filled_on_another_stream`.

## Scenario: A stream of another CUDA context

A commit on a capturing stream that does not share the scene's CUDA context is refused
before anything is queued on it (AC-17).

### Given

- The suite's shared DirectGPU instance, created without a CUDA device, so physics runs in
  its own context, and `boxes_falling_on_groundplane_gpu.usda` stepped once.
- An open `position` write over a held rigid-body query.

### When

- Under `wp.ScopedCapture` on Warp's stream, the fill is queued and the group committed
  with that stream; the write is then released.

### Then

- The commit fails with an error naming another CUDA context, the capture ends cleanly,
  and the release fails because it keeps the spent group's column (REQ AC-17).

### Code References

- `ovphysx/tests/python_tests/test_write_capture_gpu.py`:
  `test_capture_on_another_cuda_context_is_refused`.

## Scenario: A group applied on the host

A host-applied group committed on a capturing stream is refused when the commit is made,
and stays live to commit after the capture (AC-17).

### Given

- The suite's shared DirectGPU instance and `boxes_falling_on_groundplane_gpu.usda`
  stepped once.
- An open `mass` write over a held rigid-body query, its host-resident group filled
  with 3.0.

### When

- Under `wp.ScopedCapture` on Warp's stream, a marker fill is queued and the group
  committed with that stream; after the capture its graph is replayed and the same group
  committed with `{0, 0}`.

### Then

- The commit inside the capture raises at the call with an error naming the capture, the
  replay runs the marker fill, so the capture went on, and the second commit succeeds:
  every body reads mass 3.0 (REQ AC-17). Refused inside the task instead, the group
  would be spent and the second commit would raise.

### Code References

- `ovphysx/tests/python_tests/test_write_capture_gpu.py`:
  `test_host_group_refused_inside_a_capture_commits_after_it`.

All four cases need a CUDA device and run in the GPU pass. The native capture path is covered
in the runtime by
[TEST-INPUT-DEVICE-006](../../../ovruntime/plc/tests/input/TEST-INPUT-DEVICE-006.md).
