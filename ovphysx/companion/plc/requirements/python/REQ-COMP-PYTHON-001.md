<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-PYTHON-001
title: "Optional customer-USD Python utilities"
status: implemented
owner: ovphysx
---

## Description

The base ovphysx_companion package exposes native data services. An optional usd subpackage lazily uses the caller-installed pxr and codeless schemas, without supplying another USD runtime.

## Acceptance Criteria

- AC-1: **Base import.** Importing the base companion without pxr or Kit succeeds, performs no simulation initialization and does not import the USD utility modules.
- AC-2: **Customer USD.** Invoking a USD helper uses the installed customer pxr; the companion neither bundles, preloads, auto-installs nor replaces USD or compiled USD Python bindings.
- AC-3: **Dependency errors.** Missing pxr, unsupported USD/schema versions or unregistered required schemas produce focused errors before editing; supported combinations are documented and tested.
- AC-4: **Shared helpers.** Standalone and Kit callers use the same companion USD helper implementation; existing ovphysx authoring helpers are reused or delegated with documented compatibility behavior.
- AC-5: **Single version.** The package version and qualified core version are `ovphysx/VERSION` in the source tree, an sdist, a wheel built from that sdist in an isolated environment, and an installed package; no companion file restates it.

## Implementation scope

Base imports are lazy and work without USD, Kit, ovstage or native initialization.
Helpers use customer USD and preflight required APIs/schema definitions; missing
or unsupported dependencies fail before edits. Qualified combination: Python 3.12,
usd-core 26.8 and the codeless schemas of the core built from the same tree
(one shared `ovphysx/VERSION`) on Linux x86_64. Other combinations
are not claimed as tested. Shared core authoring is reused by the migration helper
and ColliderBatch. See the dependency and mode matrix.

The package version is `ovphysx/VERSION`; no companion file restates it.
`setup.py` stamps a generated `_version.py` into sdist and wheel outputs only, so
a wheel built from the sdist keeps that version without the parent tree.

## Test References

- [TEST-COMP-PYTHON-002](../../tests/python/TEST-COMP-PYTHON-002.md) - executed Python subset; see its scenario and limitations.
- [TEST-COMP-PYTHON-001](../../tests/python/TEST-COMP-PYTHON-001.md) - executed supported-contract scenario.

## Code References

Companion implementation:

- [__init__.py](../../../python/ovphysx_companion/__init__.py)
- [setup.py](../../../python/setup.py) and [pyproject.toml](../../../python/pyproject.toml)
- [Packaging test](../../../tests/test_packaging.py)
- [usd/__init__.py](../../../python/ovphysx_companion/usd/__init__.py)
- [usd/session.py](../../../python/ovphysx_companion/usd/session.py)
- [usd/attachments.py](../../../python/ovphysx_companion/usd/attachments.py)
- [Executable subset tests](../../../tests/test_usd_helpers.py)

Existing integration anchors; these do not establish native implementation:

- [ovphysx/python/ovphysx/utils/__init__.py](../../../../python/ovphysx/utils/__init__.py)
- [ovphysx/python/ovphysx/utils/deformable.py](../../../../python/ovphysx/utils/deformable.py)
- [ovphysx/python/ovphysx/utils/codeless.py](../../../../python/ovphysx/utils/codeless.py)

- [compatibility.md](../../../docs/compatibility.md)
- [migration.py](../../../python/ovphysx_companion/migration.py)
- [test_migration.py](../../../tests/test_migration.py)

## Dependencies

- [REQ-COMP-BOUNDARY-001](../architecture/REQ-COMP-BOUNDARY-001.md)
- [Companion ADR-0004](../../adr/ADR-0004-customer-usd-python.md)
