<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Runtime simulation ADR review

Release behavior baseline: `release/omni/110.4` at `5f3904474e6a8cac7231279179a362cdd9dfedc0`.
Current ovphysx API/utility reference: repository commit `c52311f56fb18ac7d8a080440eaa037fb17b665c`, inspected 2026-09-18 in the main checkout.
Imported from `feature/omni-110.4-ui-adrs` at `e48e40f4b1dd23262bad3cdbf3e554d59ee959e6` into the companion feature branch. Current companion review baseline is `a76179e1a8`; original accepted decisions are preserved, with dated owner-scope and interaction clarifications.

The runtime review now contains **nine accepted simulation ADRs** and **one rejected broad proposal**. The [25 UX proposals and 108-row sheet mapping](../../ux/adr/README.md) remain a separate review.

## Review order

[Runtime requirements and test specifications](../README.md) provide the implementation acceptance contracts and scenario mapping.

- [Simulation boundary and supported hooks](ADR-0001-ovphysx-stageupdate-lifecycle.md).
- [Attach: remember the opened stage without loading](ADR-0003-ovstage-attach.md).
- [Start/resume: load on Play and capture initial state](ADR-0004-start-resume-state-capture.md).
- [Update: scene timestep, remainder and substeps](ADR-0005-simulation-update-and-substepping.md).
- [Pause: preserve simulation state](ADR-0006-simulation-pause.md).
- [Stop: destroy simulation and restore initial state](ADR-0007-stop-and-state-restoration.md).
- [Detach: tear down any session and forget the stage](ADR-0008-ovstage-detach.md).
- [Ray input: raycast, impulses, damping and gravity](ADR-0009-raycast-and-mouse-forces.md).
- [Output: existing utilities, mappings and ordinals](ADR-0010-ovstage-simulation-output.md).

[ADR-0002 is withdrawn](ADR-0002-ovphysx-runtime-services-and-output.md); its former authoring/services scope does not apply. UX dependencies now reference the specific simulation decisions and separate authoring lifecycle, not that rejected proposal.

## Lifecycle and implementation constraints

- Force load, reset simulation and release objects are unsupported; `onReset` means supported timeline Stop.
- `enableUpdate=false` provides no editor maintenance. Stop always restores initial state; runtime-owned capture is separate from the utility output cache.
- Attach and output APIs already exist in current ovphysx. Kit-to-ovstage ownership/ordinal wiring is integration work, not a missing backend stage-adoption API.
- Stage open only remembers the stage. Play captures current initial state, creates the instance and loads ovstage. Pause retains that session; Stop destroys it and restores the host stage without reattaching. Each later Play loads afresh.
- Assume one physics scene, with one timestep and one remainder. Preserve the legacy catch-up formula using the existing ovphysx step API.
- The helper performs a step itself. The proposal calls it once per admitted substep, without a preceding extra step.
- Native force-grab formulas are specified. DirectGPU damping/impulse parity and joint/deformable interaction access need verification; unsupported modes must be explicit.
- Visible deformable/particle shadows and descendant-transform propagation need host integration beyond the helper. A complete snapshot/output mapping is required before admitting each object kind.
- The [UX requirements](../../ux/README.md) remove optional Stop restoration and legacy simulation controls, scope stopped preview to an explicit independent companion instance B and explicitly gate controller services. The separate authoring attach/detach hook does not load or step another simulation.

## Source evidence

The release links below target the immutable source revision. Current-API links pin the separately inspected revision because modern ovphysx lives at a different repository path.

| Evidence | Finding carried into the runtime decisions |
| --- | --- |
| [ovphysx/include/ovphysx/ovphysx.h](../../../../../include/ovphysx/ovphysx.h) | Existing backend attach/update/detach APIs used during the play session; backend attach is distinct from the Kit stage-open hook. Single attached source, borrowed native pointers, scene queries and explicit stepping. |
| [ovphysx/src/ovphysx/ovphysx.cpp](../../../../../src/ovphysx/ovphysx.cpp) | ovphysx step invokes explicit simulate; synchronous stepping tracks internal simulation time. |
| [ovphysx/ovruntime/source/omni.physx/plugins/PhysXUpdate.cpp](../../../../../ovruntime/source/omni.physx/plugins/PhysXUpdate.cpp) | Explicit simulate dispatches with noStepping=true; frame-level accumulation must live in the adapter. |
| [ovphysx/python/ovphysx/utils/simulation.py](../../../../../python/ovphysx/utils/simulation.py) | step_and_write_to_ovstage, OvStageOutputCache, exact output groups, one step per call, write-floor and CUDA ownership. |
| [ovphysx/docs/ovstage_integration.md](../../../../../docs/ovstage_integration.md) | Caller-owned input/output ordinals, output cache lifetime and utility usage. |
| [ovphysx/docs/read_write/writable.md](../../../../../docs/read_write/writable.md) | Force/wrench units, whole-group writes, DirectGPU/kinematic limits and read-only articulation-link poses. |

## Public ovruntime header inventory

All **50 headers** below are under `omni/ovruntime/include/omni`; declarations, types,
settings and utility headers are included so the public surface is not reduced to simulation stepping.
Private headers are excluded from the public count, but private services used by the UX were examined as migration gaps.
Interface declarations in this directory do not imply their implementation lives in the runtime extension group.
The disposition column preserves the initial broad UX migration survey. It is historical feature-review context, not the scope of the simulation-only runtime ADRs or an assertion of current C API parity. In particular, cooking, preview, controller and diagnostic entries do not add services to ADR-0001.

| Header | Surface and migration disposition |
| --- | --- |

## Review boundary

The runtime ADRs have been reviewed and now have [derived REQ/TEST specifications](../README.md). Runtime implementation and automated adapter coverage have not been added. The 25 UX proposals and their requirements/tests remain a separate review.
