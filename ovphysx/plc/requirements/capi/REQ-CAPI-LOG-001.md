<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-LOG-001
title: Application Log Callback Contract
status: implemented
owner: ovphysx
---

## Description

ovphysx exposes the SDD V2 severity ABI and one replaceable application log
callback with source metadata, filtering, serialized delivery, and bounded
drain behavior.

## Acceptance Criteria

- AC-1: Severity values are `DEFAULT=0`, `VERBOSE=1`, `INFO=2`, `WARNING=3`,
  `ERROR=4`, and `NONE=5`; DEFAULT restores the WARNING library default.
- AC-2: The callback receives severity, NUL-terminated message and channel,
  Unix-epoch timestamp seconds, and user data.
- AC-3: Setting a callback replaces the prior callback; NULL disables it only
  after prior accepted calls finish. Every successful shutdown flushes
  Carbonite, disables the callback, and waits for accepted calls before
  returning, including when live handles remain solely for explicit
  destruction. Invalid arguments leave the prior registration unchanged. Any
  other returned error may follow publication, so the candidate callback and
  user data remain valid until a later successful replace, disable, or shutdown
  drains the slot.
- AC-4: A copied comma-separated `channel=level` filter applies raw-prefix
  matching, with the longest matching prefix taking priority; if two matching
  rules have the same prefix length, the later rule wins.
- AC-5: Callback invocations are serialized, and callback-time attempts to set
  the callback, set the level, change default output, flush, or shut down return
  an error. Python bridge ownership and lifecycle behavior are specified by
  REQ-PYTHON-LIFECYCLE-001 AC-7.
- AC-6: `ovphysx_flush_log` supports poll, finite-nanosecond, and infinite waits
  for records already accepted by the ovphysx callback dispatcher. Carbonite
  records still buffered upstream are outside that finite-time barrier.
  Every successful shutdown flushes Carbonite first, then disables and drains
  the callback. Continued instance work after live-handle shutdown is
  unsupported except for explicit destruction.
- AC-7: The ovphysx source level changes only the named `omni_physx_sdk`,
  `omni.physx`, and `ovphysx_internal` Carbonite source policies. Any unnamed
  source and all host and dependency sources and channels remain unchanged.
  Source-level updates preserve Carbonite's effective host policy, including a
  process-global disabled state; they must not reopen a named channel that the
  host has disabled globally.
  The application callback continues to receive process-stream records admitted
  by its own minimum severity and channel filter. `OVPHYSX_LOG_NONE` mutes only
  the three named sources; it is not a whole-runtime or process mute.

## Test References

- TEST-CAPI-LOG-001

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h
- ovphysx/include/ovphysx/ovphysx.h
- ovphysx/src/include/LogManager.hpp
- ovphysx/src/include/ovphysx_test_utils.h
- ovphysx/src/ovphysx/LogManager.cpp
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/python/ovphysx/types.py
- ovphysx/python/ovphysx/_bindings.py
- ovphysx/python/ovphysx/api.py
- ovphysx/python/ovphysx/api.pyi
- ovphysx/python/ovphysx/__init__.py
- ovphysx/python/ovphysx/__init__.pyi
- ovphysx/tests/c_unittests/test_c_api_compatibility.c
- ovphysx/tests/c_unittests/test_log_level.cpp
- ovphysx/tests/c_unittests/test_global_lifecycle.cpp
- ovphysx/scripts/test_cpp.cmake
- ovphysx/tests/python_tests/cpu_tests/test_log_level.py

## Dependencies

- ADR-0024
- REQ-PYTHON-LIFECYCLE-001
