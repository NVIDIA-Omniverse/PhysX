<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-CONFIG-001
maps_to: REQ-COMP-CONFIG-001
type: integration
---

## Coverage status

Implemented CPU integration coverage. Covers [REQ-COMP-CONFIG-001](../../requirements/architecture/REQ-COMP-CONFIG-001.md) AC-1 through AC-4.

## Scenario

Set process, instance, scene and UI options before and after their allowed phase.

## Given

- Playback A and companion-native B with distinct stages, a settings descriptor catalog and snapshots of effective values; compatible alternate thread/device choices and incompatible tolerance/provider candidates.

## When

- Apply each setting in its allowed phase, create B with copied options and try incompatible candidates; retry immutable options after initialization/attach and read retained values for both owners.

## Then

- All options expose scope/default/phase metadata and UI preferences persist only through the host (REQ AC-1).
- Core options use their existing path; B uses copied descriptors without rewriting global defaults, and scene options become explicit authored edits (REQ AC-2).
- Late invalid changes are rejected with unchanged effective values (REQ AC-3).
- The catalog includes the OVD capability as a before-first-instance bool
  with default false. Companion and core reads/writes observe the same retained
  value, non-bools are rejected unchanged, capability alone starts no capture,
  late writes (including the retained value) fail, and full successful shutdown
  with no live instances reopens writes (REQ AC-1, AC-2, AC-3).
- Supported owner-local resource choices remain distinct; incompatible SDK tolerances/providers and hard CPU-only violations fail without affecting A. Borrowed-context lifetime and runtime resource retention are preserved (REQ AC-4).

## Executable coverage

- [Native configuration cases](../../../tests_native/test_configuration.py): copied B/C options, unchanged A/defaults, owner-local contact policy, peer pending work, stale/ABI/output canaries, typed core lifecycle rejection, hard CPU policy, metadata and host thread affinity.
- `test_configuration_catalog_and_host_preferences` protects process catalog
  completeness. `test_configuration_ovd_recording_capability_default_and_core_delegation`
  uses a clean native subprocess for the OVD default, typed delegation, dormant
  capture and lifecycle-write boundary.
- [Customer-USD cases](../../../tests/test_configuration_usd.py): unit conversion, explicit default-time opinions, undo/redo, stale/invalid atomic rejection and settings sample.

Six native and three customer-USD cases pass. The native resource/callback
fixture passes 75 assertions; the active-recording SDK retention/provider
fixture passes 33 assertions. Actual GPU context borrowing remains a hardware
qualification check, not a CPU pass or a capability advertised by this helper.
