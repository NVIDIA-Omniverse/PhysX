<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-OMNIPVD-001
title: Python Retired OmniPVD Startup Configuration
status: implemented
owner: ovphysx
---

## Description

Python exposes the global recording capability and explicit recording methods,
without a separate startup-recording configuration path.

## Acceptance Criteria

- AC-1: `ovphysx.legacy_ovd_config` is absent. Python config enums omit the retired OVD startup keys, including bool capability slot 4; unrelated enum values remain aligned with C.
- AC-2: `PhysXConfig` rejects removed startup output/destination fields and `omnipvd_recording_capable` with `TypeError` and global-capability/explicit-recording migration guidance before native creation. Unrelated legacy fields remain available. The removed-field guard preserves constructor compatibility: default `inspect.signature` reports the effective constructor parameters and annotations of `PhysXConfig` and its subclasses, including added dataclass fields and inherited, mixin, variadic or custom `__new__` constructors. A custom metaclass `__call__` takes precedence over class constructors; callable instances retain their `__call__` signatures. Factories decorated with `functools.wraps(PhysXConfig)` and class references retained across a config-module reload keep their constructor signatures. Normal positional construction, `dataclasses.replace`, deepcopy and pickle remain usable; unknown keywords raise the standard `TypeError`.
- AC-3: TCP recording requires explicit start with `OmniPvdDestination.tcp`; a ready loopback listener receives more than the fixed eight-byte socket handshake after start and step, including when start precedes the first simulation step.

## Test References

- TEST-PYTHON-OMNIPVD-001

## Code References

- ovphysx/python/ovphysx/config.py
- ovphysx/python/ovphysx/types.py
- ovphysx/tests/python_tests/cpu_tests/test_public_ovd_api.py
- ovphysx/tests/python_tests/cpu_tests/test_physxconfig_validation.py (removed-field migration errors and constructor compatibility; AC-2)
- ovphysx/tests/python_tests/test_types_sync.py
- ovphysx/tests/python_tests/test_omnipvd_recording.py

## Dependencies

- REQ-CAPI-OMNIPVD-001
- REQ-PYTHON-OMNIPVD-LATE-001
