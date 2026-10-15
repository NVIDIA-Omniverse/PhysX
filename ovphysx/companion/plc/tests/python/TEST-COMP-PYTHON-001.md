<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-PYTHON-001
maps_to: REQ-COMP-PYTHON-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-PYTHON-001; executable evidence and qualification limits follow.

## Scenario

Import and invoke USD helpers in clean environments with no USD and supported customer USD installations.

## Given

- A core/companion-only environment and environments for every declared supported customer USD/schema combination.
- A small deformable-authoring script and a module-load/initialization trace.

## When

- Import the base package in all environments; invoke the USD helper with missing, unregistered and valid dependencies.

## Then

- Base imports succeed with no pxr/Kit imports or runtime startup (REQ AC-1).
- The valid call uses the preinstalled pxr image and no alternate USD payload appears in the package or process (REQ AC-2).
- Invalid setups fail before stage mutation with the missing dependency/version/schema named (REQ AC-3).
- Standalone execution calls the shared authoring implementation and compatibility wrappers delegate rather than fork it (REQ AC-4).
- An sdist, an isolated wheel built from it and the source tree all report `ovphysx/VERSION` as the package and qualified core version (REQ AC-5).

## Executable coverage

[test_usd_helpers.py](../../../tests/test_usd_helpers.py):

Blocked optional imports, missing USD and unregistered schemas fail before mutation. tests/test_migration.py verifies unsupported USD entrypoints and shared authoring without runtime import. Deformable construction and batch collider helpers run against customer usd-core 26.8 and the bundled codeless schemas.

[test_packaging.py](../../../tests/test_packaging.py):
`test_sdist_to_isolated_wheel_keeps_shared_version` builds an sdist from a copy
of the package, then an isolated wheel from that sdist with `uv build`. It checks
that the sdist carries `_version.py` but no `VERSION`, and that wheel metadata,
`__version__` and `QUALIFIED_CORE_VERSION` all match `ovphysx/VERSION`.
`test_source_tree_has_no_generated_version_stamp` keeps the stamp out of the
source tree (REQ AC-5). The sdist check needs the suite's `uv run` environment
and package index (the same index the suite's `uv sync` uses).

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
