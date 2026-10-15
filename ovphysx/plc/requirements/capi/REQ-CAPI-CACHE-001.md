<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CACHE-001
title: Process-Private Cooked-Collider Cache Teardown
status: implemented
owner: ovphysx
---

## Description

Without a configured cooked-collider cache directory, ovphysx uses a private
temporary directory. Its lifetime follows the datastore, including asynchronous
write-back after cooking completes. Linux SDK shutdown stops that datastore before
removing the directory. Windows retains its framework and datastore until process
exit to protect dependency DLL teardown; a framework release callback attempts
cache removal after datastore shutdown, but another dependency's terminating exit
hook may prevent it. Windows directory removal remains best-effort.
Python destroys remaining instances and shuts down the
SDK before interpreter finalization.
These teardown guarantees apply to the SDK-owned framework. When reusing a
framework populated by an external host, that host retains teardown ownership.

## Acceptance Criteria

- AC-1: **Datastore lifetime.** Destroying an instance, including the last instance,
  preserves the private directory while the datastore remains active. A successful
  `ovphysx_shutdown()` with no live instances on Linux stops the datastore before
  removing its private directory. Windows retains the directory while its
  datastore remains resident until process exit.
- AC-2: **Serialized cleanup.** Linux framework release and all directory removal hold the
  same bootstrap mutex as loader initialization, preventing a new loader from
  reusing resources during teardown.
- AC-3: **Ordered removal.** Cache deletion follows dependency-plugin shutdown,
  rather than a delay or repeated deletion racing background writes. Successful
  removal clears the remembered path. A subsequent SDK lifecycle can create a
  fresh datastore with a newly configured directory on Linux. Windows retains
  its original cache directory for the process lifetime.
- AC-4: **Diagnosable failure.** A filesystem removal failure emits a warning with
  the directory and error on stderr, which remains available after Carbonite stops.
  The remembered path remains available for a later cleanup attempt.
- AC-5: **Python exit.** Normal interpreter exit, including an uncaught
  `KeyboardInterrupt`, destroys all tracked live instances and shuts down the SDK
  before interpreter finalization. Linux leaves no private cache after successful
  filesystem removal; Windows removal remains best-effort as described above.
  Abrupt termination, hard crashes, and forking after SDK
  initialization are outside this guarantee.

## Test References

- [TEST-CAPI-CACHE-001](../../tests/capi/TEST-CAPI-CACHE-001.md)
- [TEST-CAPI-CACHE-002](../../tests/capi/TEST-CAPI-CACHE-002.md)

## Code References

- ovphysx/src/CarboniteLoader/CarboniteLoader.cpp
- ovphysx/src/include/internal/CarboniteLoaderState.hpp
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/python/ovphysx/api.py
- ovphysx/tests/python_tests/test_ujitso_cooking_cache.py

## Dependencies

- [ADR-0001: Cooked-collider cache location](../../adr/ADR-0001-cooked-collider-cache-location.md)
