<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-OMNIPVD-001
title: Python OmniPVD Startup Transport
status: implemented
owner: ovphysx
---

## Description

`PhysXConfig` exposes the typed OmniPVD startup destination and validates it
before forwarding it to the C API.

## Acceptance Criteria

- AC-1: Python exposes transport, address, port, and timeout fields with config-enum values matching C.
- AC-2: Python rejects wrong field types, noncanonical transports, incomplete TCP tuples, invalid ports, and negative or overflowing timeouts.
- AC-3: A ready loopback listener receives more than the fixed eight-byte socket handshake after attach and step.

## Test References

- TEST-PYTHON-OMNIPVD-001

## Code References

- ovphysx/python/ovphysx/config.py
- ovphysx/python/ovphysx/types.py
- ovphysx/tests/python_tests/cpu_tests/test_physxconfig_validation.py
- ovphysx/tests/python_tests/test_types_sync.py
- ovphysx/tests/python_tests/test_omnipvd_recording.py

## Dependencies

- REQ-CAPI-OMNIPVD-001
