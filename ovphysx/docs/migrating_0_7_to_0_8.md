<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Migration from 0.7 to 0.8

This page is for an application that already runs against ovphysx 0.7. The
[Execution Model](developer_guide.md#execution-model) describes 0.8 in full.

## Calls can now return before they finish

By default, each instance runs work on a dedicated worker thread. Calls still
run in submission order.

Update 0.7 applications as follows:

- `attach_ovstage`, `update_from_ovstage`, `detach_ovstage`, and `clone` now
  return before their work may finish. Wait for the returned task before code
  outside the instance uses the result.
- Calls submitted to the same instance remain ordered. For example, you may
  submit attach, clone, and step without waiting between them.
- Errors found during execution are reported by the task wait (`TaskError` in
  Python), not by the original call.
- An accepted query, read, or write returns a nonzero handle immediately. If
  its task later fails, release that handle.
- Most query, read, and write calls run immediately when the instance is idle.
  Otherwise they queue behind earlier work. Use other outputs, such as summaries
  and fetched groups, only after a successful wait.
- A read or commit with a CUDA stream or event waits for earlier work, then
  runs on the calling thread.
- Keep buffers, masks, fetched groups, output slots, CUDA streams, and CUDA
  events alive until the task finishes. Strings and selection data may be
  released after the call returns.
- Callbacks may run on the worker thread or on a thread performing an infinite
  wait. Do not submit or wait for instance work from inside a callback.
- Collect tasks regularly. This reports failures and prevents the queue from
  filling. `ovphysx_destroy_instance()` cancels work that has not started;
  Python `destroy()` waits for outstanding tasks.
- To inspect progress without waiting or collecting, use
  `ovphysx_query_task_status()` (`ovphysx_task_status_t`) or Python
  `PhysX.task_status()` (`TaskStatus`). A finished task reports `SUCCEEDED` or
  `FAILED` until `wait_task()` or `wait_all()` collects it, then `NOT_FOUND`.
  Status queries do not replace those waits.
- Before CUDA graph capture, finish pending tasks on every instance that uses
  that CUDA context.

Special cases:

- Attach and detach reject while a companion operation is active. While an
  accepted attach or detach is pending, later companion calls report busy.
- During OmniPVD recording, attach and detach finish before returning.
- `is_recording()` no longer waits for instance work. It reports true only for
  the instance that owns the recording.
- Set `ASYNC_EXECUTION` to false before creating an instance to keep the 0.7
  calling-thread behavior. In that mode, device-based CUDA setup may leave its
  newly created context current, as it did in 0.7.

## Attribute names follow ovstage 0.4

ovphysx 0.8 builds on ovstage 0.4, which changes how an attribute entry names
its attribute:

- The C type `ovx_string_or_token_t` is now `ovx_token_or_string_t`. Its layout
  is unchanged: a `token` followed by a `string`.
- A non-NULL `string.ptr` now takes precedence and the token beside it is
  ignored. Only an entry with a NULL string names its token. In 0.7 the token
  took precedence.
- `OVX_INVALID_TOKEN` is now `2**64 - 1` (`UINT64_MAX`), not 0. Token 0 is the
  interned empty string, so a zero-initialized entry with a NULL string now
  names `""`. Reads and writes still reject it. Initialize `token` to
  `OVX_INVALID_TOKEN` when you name the attribute by string.
- Read rows for an unsupported attribute, and debug-render attribute tokens
  with no stage attached, carry `OVX_INVALID_TOKEN` instead of 0. Compare
  against `OVX_INVALID_TOKEN` (Python: `2**64 - 1`) rather than 0.
- Python attribute lists still take either a string or an integer token. An
  integer token must not be 0 or `2**64 - 1`.

## Physics-only population needs no world-matrix cleanup

The 0.7 guide told producers that populate with `PopulationDomain.PHYSICS`
alone to delete `omni:fabric:worldMatrix` before sealing (Python
`stage.delete_attributes()`, C `ovstage_delete_attributes()`), so ovphysx would
compose `omni:xform` up the parent chain. Remove that code when you move to the
physics population description:

- Sealing the population ordinal with `advance_write_floor()`, which attach
  already requires, replaces each prim's local-matrix seed with its composed
  world matrix. A body under a transformed parent attaches at its world pose
  without the deletion or an explicit `compute_hierarchy()` call.
- Vehicle-wheel output needs that sealed column.

[When the physics description alone is enough](ovstage_integration.md#when-the-physics-description-alone-is-enough)
describes the 0.8 behavior.

## Inverse-dynamics reads carry their own shape

In 0.7, `jacobian`, `massMatrix`, `coriolisForce`, `gravityForce`, and
`centroidalMomentum` flattened each row's matrix into `dtype.lanes`, and the
`jacobianShape` column gave the Jacobian's rows and columns. In 0.8, each of
these tensors carries its dimensions as axes and has one lane. With `N` the
group's rows and `M = numDofs + (fixedBase ? 0 : 6)`:

| Attribute | 0.7 lanes | 0.8 shape |
|---|---|---|
| `jacobian` | `R * C` | `[N, R, C]` |
| `massMatrix` | `M * M` | `[N, M, M]` |
| `coriolisForce`, `gravityForce` | `M` | `[N, M]` |
| `centroidalMomentum` | `6 * (numDofs + 7)` | `[N, 6, numDofs + 7]` |

Update 0.7 applications as follows:

- `jacobianShape` and `OVPHYSX_ATTR_JACOBIAN_SHAPE` are removed. Read `R` and
  `C` from the `jacobian` tensor's shape. A read that still names
  `jacobianShape` reports it as an unsupported attribute and returns no group
  for it.
- Index each matrix through its axes instead of unflattening the lanes. In
  Python, a `jacobian` array changes from `[N, R * C]` to `[N, R, C]`.

[Articulation inverse dynamics cohorts](read_write/data_model.md#articulation-inverse-dynamics-cohorts)
describes the layout.
