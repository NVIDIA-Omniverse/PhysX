<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Companion PLC

This component has **8 accepted ADRs, 35 implemented requirements and 52
executed test specifications covering 150 acceptance criteria** for the advertised
CPU/customer-USD contract. The [operation matrix](../docs/compatibility.md) declares
unsupported modes and the tested dependency combination. Implementation completion
does not imply GPU qualification or wholesale legacy Kit parity.

The [migration guide](../docs/migration.md) maps legacy helpers and host commands
to supported APIs. The [samples](../samples/README.md) cover independent Inspector,
cooking/attachments/deformables, writeback, controllers, queries, manipulation,
events/debug, settings, OVD and compiled custom plugins. The Python package stays
optional; native code stays USD-free and uses the existing runtime owner.

The records use the existing repository ADR/REQ/TEST shapes. At the requested
component boundary, companion records live here and use their own ADR number
sequence plus `REQ-COMP-*` / `TEST-COMP-*` identifiers. Refer to an ADR as
**Companion ADR-NNNN** to distinguish it from the shared runtime sequence.


## Implementation ownership

| Location | Responsibility |
|---|---|
| Companion | Supported facades, data records, native bindings, optional customer-USD Python helpers, consumer contracts and integration tests. |
| ovruntime | Reusable computation, parser/population integration and custom callback lifecycle; its implementation changes retain native PLC ownership. |
| Core ovphysx | Minimal owner acquisition bridge and existing attach/step/task/read/write/query/recording operations; its implementation changes retain core PLC ownership. |
| Host/Kit adapter | Timeline, UI, commands/undo, selection/input and output-notice policy. |
| Application/ovstage | Stage lifetime, schema population, sealed input revisions and explicit output destinations. |

The existing-source links in each requirement are integration anchors, not
implementation claims. When implementing a requirement, add the actual
companion source paths, update native/core requirements where their behavior
changes, annotate implementing files and record exact executable test cases.
Existing service records are implemented for their stated qualification scope.
New proposals remain proposed until implemented; mark a record implemented only
when every acceptance criterion has executed evidence. Extension UX specifications
retain their separate proposed status.

## Decisions

| ADR | Decision |
|---|---|
| [Companion ADR-0001](adr/ADR-0001-optional-companion-and-owner-bridge.md) | Optional companion with one runtime owner |
| [Companion ADR-0002](adr/ADR-0002-ovstage-and-explicit-output.md) | Ovstage as scene data with explicit output publication |
| [Companion ADR-0003](adr/ADR-0003-data-services-and-jobs.md) | Data-oriented authoring and live service contracts |
| [Companion ADR-0004](adr/ADR-0004-customer-usd-python.md) | Customer USD authoring belongs in optional Python utilities |
| [Companion ADR-0005](adr/ADR-0005-attachment-and-deformable-split.md) | Attachment and deformable computation separated from USD edits |
| [Companion ADR-0006](adr/ADR-0006-host-services-and-ux-migration.md) | Host UX over optional runtime capabilities |
| [Companion ADR-0007](adr/ADR-0007-preview-ownership.md) | Inspector preview in a dedicated native attachment |
| [Companion ADR-0008](adr/ADR-0008-native-extension-sdk.md) | Native extension SDK with explicit ABI and lifetime |

## Requirements and verification

Every row links both directions through REQ Test References and TEST `maps_to`.
TEST-*-002 records retain the focused vertical-slice evidence; TEST-*-001 records
now link the complete supported scenarios. Each TEST states its execution and
qualification limits. The 2026-10-01 full SDK aggregate passes 243/243 CTest
groups, including all 237 GPU-enabled native runtime groups, C++ unit/samples,
source-link, wheel, Python samples and Python runtime. The companion suites pass
230 native Python cases and 118 customer-USD/pure helper cases. Force interaction
qualification passes eight Python cases and six dual-source native cases with
430 assertions, including default force picking, coefficient tuning, zero due
substeps, exact instance/owner lifetime and retained-acceleration cancellation.
Direct custom-interface C++ tests cover rejected acquisition, original registration
signatures, retained-runtime lifetime and header isolation. The separate companion
wheel passes isolated import without USD, ovstage or core initialization and
contains no native SDK or USD payload.
Public source export remains a required check.

The 2026-10-02 review qualification passes 235 companion native Python cases,
138 customer-USD/helper cases and 17 core lifecycle cases. The strict Debug CPU
suite passes 501 cases with one existing MBP skip. The complete GPU-enabled
runtime sweep passes 237/237 groups. Installed SDK consumers, CI
preflight and the staged public-source release-tree validation pass. Nineteen
selected new regression cases fail against the preceding Python implementation.

Integration with the current core query/read/write API uses explicit completed
`read_sync` capture. The merged Release CPU suite passes 555 cases with the same
MBP skip; native companion, customer-USD and lifecycle suites pass 235, 138 and
17 cases respectively. Against trunk's updated PhysX package, all 245 GPU-enabled
runtime groups pass, including the bounded volume-skinning guide regression.
CI preflight, the unchanged USD-only coverage ratchet and public-source export
validation pass. The ovstage-only admission test is classified explicitly because
it rejects input before parsing and has no USD reporting counterpart.

The subsequent OVStage 0.3 integration passes 236 native companion cases,
including nested populated world matrices and playback/preview poses, 138
customer-USD cases, 17 lifecycle cases and 555 Release CPU cases with one MBP
skip. All native targets rebuild; preflight, parity and public export validation
pass. At the owner's request the remaining full OVStage 0.3 runtime sweep is
delegated to CI; the 245-group result above predates that dependency update.

| Area | Requirement | Mapped test specification |
|---|---|---|
| architecture | [REQ-COMP-BOUNDARY-001](requirements/architecture/REQ-COMP-BOUNDARY-001.md) - Optional companion and native dependency boundary | [TEST-COMP-BOUNDARY-001](tests/architecture/TEST-COMP-BOUNDARY-001.md) / [subset](tests/architecture/TEST-COMP-BOUNDARY-002.md) |
| architecture | [REQ-COMP-API-001](requirements/architecture/REQ-COMP-API-001.md) - Versioned companion discovery and capability negotiation | [TEST-COMP-API-001](tests/architecture/TEST-COMP-API-001.md) / [subset](tests/architecture/TEST-COMP-API-002.md) |
| architecture | [REQ-COMP-OWNER-001](requirements/architecture/REQ-COMP-OWNER-001.md) - Owner-scoped sessions and teardown | [TEST-COMP-OWNER-001](tests/architecture/TEST-COMP-OWNER-001.md) / [subset](tests/architecture/TEST-COMP-OWNER-002.md) |
| exchange | [REQ-COMP-IDENTITY-001](requirements/exchange/REQ-COMP-IDENTITY-001.md) - Typed scene identity and source revision validation | [TEST-COMP-IDENTITY-001](tests/exchange/TEST-COMP-IDENTITY-001.md), [002](tests/exchange/TEST-COMP-IDENTITY-002.md) |
| exchange | [REQ-COMP-BUFFERS-001](requirements/exchange/REQ-COMP-BUFFERS-001.md) - Owned data buffers and device readiness | [TEST-COMP-BUFFERS-001](tests/exchange/TEST-COMP-BUFFERS-001.md) / [subset](tests/exchange/TEST-COMP-BUFFERS-002.md) |
| exchange | [REQ-COMP-JOBS-001](requirements/exchange/REQ-COMP-JOBS-001.md) - Cancellable authoring jobs and ordered live tasks | [TEST-COMP-JOBS-001](tests/exchange/TEST-COMP-JOBS-001.md) / [subset](tests/exchange/TEST-COMP-JOBS-002.md) |
| exchange | [REQ-COMP-OVSTAGE-001](requirements/exchange/REQ-COMP-OVSTAGE-001.md) - Explicit ovstage input and output flow | [TEST-COMP-OVSTAGE-001](tests/exchange/TEST-COMP-OVSTAGE-001.md) |
| python | [REQ-COMP-PYTHON-001](requirements/python/REQ-COMP-PYTHON-001.md) - Optional customer-USD Python utilities | [TEST-COMP-PYTHON-001](tests/python/TEST-COMP-PYTHON-001.md), [002](tests/python/TEST-COMP-PYTHON-002.md) |
| python | [REQ-COMP-USD-EDIT-001](requirements/python/REQ-COMP-USD-EDIT-001.md) - Explicit USD edit transactions | [TEST-COMP-USD-EDIT-001](tests/python/TEST-COMP-USD-EDIT-001.md), [002](tests/python/TEST-COMP-USD-EDIT-002.md) |
| authoring | [REQ-COMP-MASS-001](requirements/authoring/REQ-COMP-MASS-001.md) - Stopped mass and inertia computation | [TEST-COMP-MASS-001](tests/authoring/TEST-COMP-MASS-001.md) / [subset](tests/authoring/TEST-COMP-MASS-002.md) |
| authoring | [REQ-COMP-COOK-001](requirements/authoring/REQ-COMP-COOK-001.md) - Cooking jobs and compatible cache results | [TEST-COMP-COOK-001](tests/authoring/TEST-COMP-COOK-001.md) |
| authoring | [REQ-COMP-DESCRIPTION-001](requirements/authoring/REQ-COMP-DESCRIPTION-001.md) - Stopped physics descriptions for inspection | [TEST-COMP-DESCRIPTION-001](tests/authoring/TEST-COMP-DESCRIPTION-001.md) |
| authoring | [REQ-COMP-ATTACH-COMPUTE-001](requirements/authoring/REQ-COMP-ATTACH-COMPUTE-001.md) - Native attachment and filter computation | [TEST-COMP-ATTACH-COMPUTE-001](tests/authoring/TEST-COMP-ATTACH-COMPUTE-001.md) / [subset](tests/authoring/TEST-COMP-ATTACH-COMPUTE-002.md) |
| python | [REQ-COMP-ATTACH-USD-001](requirements/python/REQ-COMP-ATTACH-USD-001.md) - Python attachment authoring through customer USD | [TEST-COMP-ATTACH-USD-001](tests/python/TEST-COMP-ATTACH-USD-001.md), [002](tests/python/TEST-COMP-ATTACH-USD-002.md) |
| authoring | [REQ-COMP-DEFORM-COOK-001](requirements/authoring/REQ-COMP-DEFORM-COOK-001.md) - Deformable topology and embedding results | [TEST-COMP-DEFORM-COOK-001](tests/authoring/TEST-COMP-DEFORM-COOK-001.md) |
| python | [REQ-COMP-DEFORM-USD-001](requirements/python/REQ-COMP-DEFORM-USD-001.md) - Python deformable construction | [TEST-COMP-DEFORM-USD-001](tests/python/TEST-COMP-DEFORM-USD-001.md) |
| python | [REQ-COMP-DEFORM-WRITE-001](requirements/python/REQ-COMP-DEFORM-WRITE-001.md) - Deformable simulation output to customer USD | [TEST-COMP-DEFORM-WRITE-001](tests/python/TEST-COMP-DEFORM-WRITE-001.md), [002](tests/python/TEST-COMP-DEFORM-WRITE-002.md) |
| authoring | [REQ-COMP-PARTICLES-001](requirements/authoring/REQ-COMP-PARTICLES-001.md) - Particle sampling and postprocessing data services | [TEST-COMP-PARTICLES-001](tests/authoring/TEST-COMP-PARTICLES-001.md) |
| authoring | [REQ-COMP-VALIDATE-001](requirements/authoring/REQ-COMP-VALIDATE-001.md) - Classified validation and explicit repair application | [TEST-COMP-VALIDATE-001](tests/authoring/TEST-COMP-VALIDATE-001.md) |
| live | [REQ-COMP-QUERY-001](requirements/live/REQ-COMP-QUERY-001.md) - Scene queries and live inspection reuse core operations | [TEST-COMP-QUERY-001](tests/live/TEST-COMP-QUERY-001.md) |
| live | [REQ-COMP-EVENTS-001](requirements/live/REQ-COMP-EVENTS-001.md) - Ordered contact trigger and runtime event delivery | [TEST-COMP-EVENTS-001](tests/live/TEST-COMP-EVENTS-001.md) |
| live | [REQ-COMP-DEBUG-001](requirements/live/REQ-COMP-DEBUG-001.md) - Diagnostic frames and statistics snapshots | [TEST-COMP-DEBUG-001](tests/live/TEST-COMP-DEBUG-001.md) / [subset](tests/live/TEST-COMP-DEBUG-002.md) |
| live | [REQ-COMP-INTERACTION-001](requirements/live/REQ-COMP-INTERACTION-001.md) - Safe interaction and scoped native access | [TEST-COMP-INTERACTION-001](tests/live/TEST-COMP-INTERACTION-001.md) / [force picking](tests/live/TEST-COMP-INTERACTION-002.md) |
| live | [REQ-COMP-PREVIEW-001](requirements/live/REQ-COMP-PREVIEW-001.md) - Isolated preview and explicit commit or restore | [TEST-COMP-PREVIEW-001](tests/live/TEST-COMP-PREVIEW-001.md), [002](tests/live/TEST-COMP-PREVIEW-002.md), [003](tests/live/TEST-COMP-PREVIEW-003.md) |
| controllers | [REQ-COMP-VEHICLE-001](requirements/controllers/REQ-COMP-VEHICLE-001.md) - Optional vehicle computations control and telemetry | [TEST-COMP-VEHICLE-001](tests/controllers/TEST-COMP-VEHICLE-001.md) |
| controllers | [REQ-COMP-CCT-001](requirements/controllers/REQ-COMP-CCT-001.md) - Optional native character controller lifecycle | [TEST-COMP-CCT-001](tests/controllers/TEST-COMP-CCT-001.md) |
| controllers | [REQ-COMP-CAMERA-001](requirements/controllers/REQ-COMP-CAMERA-001.md) - Camera behavior as a state consumer | [TEST-COMP-CAMERA-001](tests/controllers/TEST-COMP-CAMERA-001.md) |
| architecture | [REQ-COMP-CONFIG-001](requirements/architecture/REQ-COMP-CONFIG-001.md) - Scoped settings and initialization ownership | [TEST-COMP-CONFIG-001](tests/architecture/TEST-COMP-CONFIG-001.md) |
| native | [REQ-COMP-NATIVE-ABI-001](requirements/native/REQ-COMP-NATIVE-ABI-001.md) - Build-matched native extension registration | [TEST-COMP-NATIVE-ABI-001](tests/native/TEST-COMP-NATIVE-ABI-001.md) |
| native | [REQ-COMP-NATIVE-SCHEMA-001](requirements/native/REQ-COMP-NATIVE-SCHEMA-001.md) - External schemas and immutable plugin parameters | [TEST-COMP-NATIVE-SCHEMA-001](tests/native/TEST-COMP-NATIVE-SCHEMA-001.md) |
| native | [REQ-COMP-JOINT-001](requirements/native/REQ-COMP-JOINT-001.md) - Custom joint behavior and lifecycle | [TEST-COMP-JOINT-001](tests/native/TEST-COMP-JOINT-001.md) |
| native | [REQ-COMP-GEOMETRY-001](requirements/native/REQ-COMP-GEOMETRY-001.md) - Custom geometry callbacks and destruction | [TEST-COMP-GEOMETRY-001](tests/native/TEST-COMP-GEOMETRY-001.md) |
| diagnostics | [REQ-COMP-OVD-001](requirements/diagnostics/REQ-COMP-OVD-001.md) - Recording reuse and Python OVD-to-USD conversion | [TEST-COMP-OVD-001](tests/diagnostics/TEST-COMP-OVD-001.md) |
| host | [REQ-COMP-HOST-001](requirements/host/REQ-COMP-HOST-001.md) - Kit adapter owns UI scheduling commands and presentation | [TEST-COMP-HOST-001](tests/host/TEST-COMP-HOST-001.md) |
| host | [REQ-COMP-MIGRATION-001](requirements/host/REQ-COMP-MIGRATION-001.md) - Consumer coverage and explicit compatibility limits | [TEST-COMP-MIGRATION-001](tests/host/TEST-COMP-MIGRATION-001.md) |

## Coverage and verification policy

The [coverage ledger](coverage.md) maps the report's complete interface table,
all 28 extension directories, and cross-cutting contracts to these records.
It distinguishes optional support from host-only behavior and intentional
non-export. A capability is advertised only after its promised behavior has
executable evidence; a header, proposed requirement or test specification is
not enough.

Exercise native/data contracts without USD and Python authoring with supported
customer USD installations. Test every advertised platform/backend combination;
GPU cases skipped for unavailable hardware remain unverified. Native plugin
release/unload tests should also run with available memory/race diagnostics.
Host adapters first use headless scheduling/undo fixtures, then a real Kit
integration fixture when that adapter is implemented.

Numerical tests declare fixture-specific absolute/relative tolerances and use
analytical/reference expectations. Lifecycle tests check exact call counts,
owner identity and post-close delivery, including failure/rollback paths.
Package validation checks both native dependency closure and the base Python
import without pxr. The current Python subset does not establish native performance or full UX parity.

## Qualification limits

These results qualify the supported Linux x86_64 CPU contract with Python 3.12
and customer USD 26.8. GPU execution, other platforms and same-process legacy
Kit coexistence are not qualified. The full SDK aggregate includes all
237/237 native runtime groups with GPU-enabled runtime coverage. Those runtime results
do not qualify companion GPU operations that the operation matrix excludes.
Native registry allocation-failure injection remains unautomated, as recorded
in the runtime lifetime TEST. The runtime test-summary checker reports 693
pre-existing name-only entries; this is not a clean repository-wide summary gate.

## Release scope

The product ADR/REQ/TEST records and coverage ledger are self-contained and
included in the public source selection. The historical source investigation
and its pinned internal evidence are internal-only. Public copies remove
marked internal navigation blocks; product contracts remain available.
