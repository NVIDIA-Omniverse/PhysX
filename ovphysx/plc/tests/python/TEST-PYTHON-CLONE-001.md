<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-CLONE-001
maps_to: REQ-PYTHON-CLONE-001
type: integration
---

## Scenario

The shipped clone samples place cloned environments in spatially disjoint lanes,
and the runtime's env-id diagnostic is observable through the documented log
channel.

## Given

- The packaged Python and C clone samples, each cloning `/World/envs/env0` into
  three targets with explicit anchor transforms.
- A hard CPU-only instance with `enable_python_logging()` active and a handler
  attached to the `ovphysx` logger.

## When

- The Python and C sample validation runs execute the clone samples.
- A co-located clone runs in CPU mode with the log bridge enabled.

## Then

- Both samples clone successfully and report one distinct position per
  environment, matching the anchor transform each target was given, rather than
  four copies drifting from a shared pose (REQ AC-1).
- The `ovphysx` logger receives the runtime's `EnvIds requested but gpu dynamic
  is disabled` and `EnvIds requested but gpu broadphase is not set` records,
  confirming the documented observation channel carries the diagnostic
  (REQ AC-2).

## Development-Time Verification

- Measured with a scratch probe: with `enable_python_logging()` enabled, a
  co-located hard-CPU clone bridges both env-id records to the `ovphysx` logger,
  while `warnings.catch_warnings(record=True)` around the same call captures
  none. This confirms the log bridge, not `warnings`, is the Python-visible
  channel. The probe is not part of the committed automated test run.
