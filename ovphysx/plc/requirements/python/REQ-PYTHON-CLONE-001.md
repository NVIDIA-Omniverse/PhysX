<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-CLONE-001
title: Observable CPU Clone Collision-Isolation Limitation
status: implemented
owner: ovphysx
---

## Description

PhysX environment-id collision isolation engages only under GPU dynamics + GPU
broadphase, so co-located clones share one collision space in CPU mode and push
each other apart. The shipped clone samples shall demonstrate the safe CPU
recipe by placing each environment in a spatially disjoint lane, and the clone
documentation shall state the CPU limitation together with the channel that
carries the runtime's existing diagnostic.

The runtime already detects and reports the condition
(`PhysXReplicator::replicate` logs `EnvIds requested but gpu dynamic is
disabled` / `... gpu broadphase is not set`). That single native record is the
signal for every frontend; the Python wrapper adds no second, independently
conditioned warning of its own, because `PhysX.get_cpu_mode()` reports only
process-wide hard CPU-only mode and cannot see a scene's resolved dynamics or
broadphase.

## Acceptance Criteria

- AC-1: The shipped Python and C clone samples pass explicit, spatially
  disjoint anchor transforms instead of co-locating copies, so a caller who
  starts from a sample gets collision-isolated environments in CPU mode.
- AC-2: The cloning tutorial, developer guide, public C API documentation,
  Python `clone()` docstring, and clone skill state that environment ids
  isolate clones only under GPU dynamics + GPU broadphase, prescribe spatial
  separation for CPU clones, and name `enable_python_logging()` (Python) and
  `ovphysx_set_log_callback()` (C) as the way to observe the runtime's
  diagnostic, which reaches the Carbonite log rather than Python `warnings`.

## Test References

- TEST-PYTHON-CLONE-001

## Code References

- ovphysx/python/ovphysx/api.py (`PhysX.clone` docstring)
- ovphysx/tests/python_samples/clone.py
- ovphysx/tests/c_samples/clone_c/main.c
- ovphysx/include/ovphysx/ovphysx.h
- ovphysx/docs/tutorials/cloning.md
- ovphysx/docs/developer_guide.md
- ovphysx/skills/clone-environments/SKILL.md

## Dependencies

- ovphysx/ovruntime/plc/requirements/replication/REQ-REPLICATE-001.md
