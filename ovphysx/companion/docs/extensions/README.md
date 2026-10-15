<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Extension migration designs

The companion now owns the imported extension design set: **10 runtime ADRs** (nine accepted, one rejected), **25 proposed UX ADRs**, **14 runtime requirements / 45 test specifications**, and **58 UX requirements / 88 test specifications**. Runtime and UX ADR numbers are independent namespaces; companion service decisions remain in [companion PLC](../../plc/README.md). No extension implementation is imported by this documentation move.

- [Runtime designs and acceptance contracts](runtime/README.md): playback lifecycle, ovstage input/output, restoration and ray interaction. Accepted behavior is retained, with clarifications for independent preview ownership and companion interaction execution.
- [UX designs and acceptance contracts](ux/README.md): rewritten around companion Python services, versioned native tables and customer-installed USD helpers. UI, commands, undo and USD authoring remain Python; native computation remains USD-free.
- [Source workflow audit and remaining gaps](ux/WORKFLOW-COVERAGE.md): 28 extension directories, command/node/validator inventories, source-derived requirements and explicit parity obligations.

## Architecture boundary

Kit UI and graph adapters call companion services. Companion binds versioned side interfaces from the already loaded ovphysx owner. ovphysx owns simulation; ovstage is the native stage representation for source ingestion and completed simulation output. The Python companion bridges customer USD authoring and explicit writeback. It does not make USD a compiled dependency or make ovstage the UI undo model.

Playback instance A follows Play/Pause/Stop. Inspector explicitly creates independent preview instance B, with its own source, result generation and display. Authoring association itself creates neither. Per-instance runtime state is independent; process bootstrap configuration and SDK ownership remain shared.

## Review and implementation

UX ADRs remain proposed, including the proposal to restore collision-group authoring and Simulation Info diagnostics omitted by the original branch. The existing accepted companion service ADRs do not approve the new Kit UX policies. Test documents are specifications, not passing automation. Implement retained positive workflows as well as cancellation, stale-result and unavailable-capability paths; an unavailable result is not parity.

Use the UX index to select a requirement, its companion module anchors and acceptance scenarios. Consult the source catalogs for command identities, serialized graph pins and validator rules. Extend companion only for documented missing service behavior; keep Kit registration, rendering and undo in host adapters. Add executed evidence before marking any scenario implemented.

## Provenance and validation

Imported from `feature/omni-110.4-ui-adrs` at `e48e40f4b1dd23262bad3cdbf3e554d59ee959e6`; 214 original design files were relocated without renumbering. Source code was inspected at that revision and compared with release `33342a9b50b3160b717d102fc927f2fbcc64367a`. Companion review baseline: `a76179e1a8`.

Run `python3 ovphysx/companion/docs/extensions/tools/validate.py` for standalone structural validation. Validation has no dependency on the historical source branch. Public exports use the same validator with `--public` to check for internal URLs. This is document validation; it does not execute Kit workflows.

The temporary evidence directories have been removed. The public workflow catalogs retain the audit findings. Source citations are historical path/commit references; retrieving those historical trees is optional and is not a build or CI dependency.
