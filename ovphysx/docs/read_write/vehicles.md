<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Vehicle examples

A car has three useful selections. `RIGID_BODY` addresses chassis pose and velocity;
`VEHICLE` addresses controls and transmission; `VEHICLE_WHEEL` addresses wheel attachments.
The chassis and vehicle selectors name the same car prim. Wheel selection names the child
attachments separately. `ALL` rigid bodies also includes unrelated bodies, and `ALL` wheels
includes wheels from every vehicle.

The complete samples use the mixed-drive scene in
`tests/data/vehicle_read_write.usda` in the source distribution and explicitly select its
`/World/Standard` car. They write half accelerator, complete one step, then read chassis position,
engine angular speed and wheel angular speeds. Accelerator is a persistent input; the two angular
speeds are telemetry in radians per second.

## C++ using the C API

The complete C++ source, `tests/c_samples/vehicle_read_write_cpp/main.cpp`, includes task
completion, typed tensor access, session cleanup, instance creation and stage attachment. Each
query, open, fetch, commit and release receipt is checked and waited exactly once. An accepted
release transfers ownership even if execution later fails. The sample uses CPU tensors and
completes its fills before committing with zero CUDA synchronization handles.

Build its `tests/c_samples/vehicle_read_write_cpp/CMakeLists.txt` project against an
installed ovphysx SDK and ovstage. The program follows; its shared task and stage helpers
are defined in `tests/c_samples/common/ovstage_sample.h`.

```{literalinclude} ../../tests/c_samples/vehicle_read_write_cpp/main.cpp
:language: cpp
```

## Python

The complete Python source, `tests/python_samples/vehicle_read_write.py`, includes attachment
and cleanup, and reuses the adjacent sample's `attach_scene` helper. Run it with the ovphysx and
ovstage Python packages installed. `print_column` resolves row identities through the query's
attached Stage dictionary and uses a NumPy view of the CPU tensor for printing before
releasing the read session.

```{literalinclude} ../../tests/python_samples/vehicle_read_write.py
:language: python
```

Neither sample authors runtime control changes back into USD. See [readable vehicle data](readable.md#whole-vehicles)
and [writable vehicle data](writable.md#whole-vehicles) for drive applicability, column types,
validation and reset ordering.
