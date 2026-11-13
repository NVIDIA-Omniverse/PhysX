<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-OVD-INVESTIGATION-001
title: Finalized Capture Inspection Sample
status: implemented
owner: ovphysx
---

## Description

The import-safe `omnipvd_inspection.py` sample preserves a real OvPhysX FILE
capture and verifies it with an explicitly supplied, separately installed
PhysX SDK inspector. Its evidence distinguishes recorded state from live reads.

## Acceptance Criteria

- AC-1: **Lifecycle.** The sample provisions recording capability before the first instance, attaches the scene, starts recording, steps, and explicitly stops before inspection. It inspects while the instance and stage remain alive, then records a successful post-inspection live read at step 11 before teardown. Importing the sample starts no instance.
- AC-2: **Motion.** In separate CPU and GPU processes, ten live reads establish that `/World/Cube1` and `/World/Cube2` fall while `/World/KinematicCube` remains fixed. Completed recorded frames 2/20 must match both dynamic bodies' live post-step position and velocity at steps 1/10 within 1e-5. GPU mode requires CUDA-backed reads and GPU scene settings; CPU mode requires CPU reads, a CPU-mode observation and no GPU scene settings.
- AC-3: **Provenance.** `evidence.json` retains the inspector's exact object identity, context, segment, requested/source frame, sample state, command and pre-membership metadata. Dynamic frame 2/20 selections require Exact writes and matching requested/source frames. Carried kinematic state is described as an initial snapshot corroborated by live reads, never as a fresh recorded observation.
- AC-4: **Preservation.** The sample requires a new output directory, preserves the capture and report there, and verifies the capture SHA-256 remains unchanged during inspection. Existing output is refused without overwriting it.
- AC-5: **Failures.** Missing/failed inspection tooling, directory creation failure, failed stop, absent/ambiguous selection, invalid evidence and live/recorded disagreement fail the workflow. No success report is produced after failed stop or disagreement. Optional Newton schemas are registered before population when available; scenes without Newton attributes work without the Newton package, but require the external PhysX schemas. The wheel sample does not locate or build a monorepo sibling or contain an OVD parser.
- AC-6: **Inspector evidence.** The helper returns one JSON object from a successful inspection with a supported command format and version: version 1 for tool metadata, summary, inspect and legacy comparison; version 2 for a generic comparison requested with both `--attribute` and `--distance`. A comparison with only one of those flags, or evidence for the other comparison mode, fails. A compare exit 1 is returned as a Different result only when its supported evidence has status `Different` and actual comparison evidence. The helper preserves recorded values and identities without changing the caller's return type. Exit 2, other exit 1 responses, malformed or non-object output (including nonstandard `NaN` and infinity constants), and unsupported formats or versions fail. Returning Different evidence does not itself mark an investigation successful.

## Test References

- [TEST-PYTHON-OVD-INVESTIGATION-001](../../tests/python/TEST-PYTHON-OVD-INVESTIGATION-001.md)

## Code References

- ovphysx/tests/python_samples/omnipvd_inspection.py (`inspect_json`, `run`, `read_live`, `finalize_and_inspect`)
- ovphysx/tests/ovd_workflow/test_workflow.py
- ovphysx/tests/ovd_workflow/test_inspection_helper.py
- ovphysx/scripts/test_python_samples.cmake

## Dependencies

- [REQ-PYTHON-OMNIPVD-LATE-001](REQ-PYTHON-OMNIPVD-LATE-001.md)
- [REQ-CAPI-OVD-NAMES-001](../capi/REQ-CAPI-OVD-NAMES-001.md)
