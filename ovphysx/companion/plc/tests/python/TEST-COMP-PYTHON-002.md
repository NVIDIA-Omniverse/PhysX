<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-PYTHON-002
maps_to: REQ-COMP-PYTHON-001
type: integration
---

## Scenario

Import without optional dependencies, then execute shared helpers with registered and missing schemas.

This automated scenario covers the Python subset only; it does not claim the
full native/GPU/Kit obligation. See the [requirement scope](../../requirements/python/REQ-COMP-PYTHON-001.md).

## Given

- A clean child process blocking pxr, NumPy, Kit, ovstage and ovphysx imports, plus a separate customer-USD process with the staged codeless schemas.

## When

- Import the base and USD packages; call EditSession without pxr, try attachment authoring without registration, and run both samples with registered schemas.

## Then

- Base import neither imports optional modules nor initializes simulation (REQ AC-1).
- The USD helper uses customer pxr; the package contains no USD bindings (REQ AC-2).
- Missing pxr or the attachment schema raises DependencyError before mutation (REQ AC-3).
- Both standalone sample workflows use the shared helpers and attachment registration delegates to the core codeless utility (REQ AC-4).

## Executable coverage

[tests/test_usd_helpers.py](../../../tests/test_usd_helpers.py):

- `test_lazy_import_and_missing_customer_usd`
- `test_unregistered_attachment_schema_fails_before_mutation`
- `test_extension_samples_run`
- `test_extension_sample_cli`

Run the separate customer-USD invocation in the [sample instructions](../../../samples/README.md).
