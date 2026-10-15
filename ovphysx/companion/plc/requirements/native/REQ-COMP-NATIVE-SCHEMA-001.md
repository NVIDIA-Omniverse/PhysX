<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-NATIVE-SCHEMA-001
title: "External schemas and immutable plugin parameters"
status: implemented
owner: ovphysx
---

## Description

Custom type registration supplies behavior; schema registration and ovstage population supply its data. Both must work with real external schemas.

## Implementation scope

Real external schemas are registered separately in customer USD and population.
Immutable typed records preserve declared defaults and owner-specific edits.
Joint refresh, geometry teardown/repopulation, authored ovstage clones and custom
point instances are exercised. The ovstage feed includes registered external
families so deletion reaches native object retirement. Unknown schema/property
requests fail explicitly. Binary SDK replication of custom state is not advertised.

## Acceptance Criteria

- AC-1: **Separate registration.** External codeless schema resources are registered in the customer USD runtime and separately in the population runtime before use; registration in one is not assumed to affect the other. PhysX resource paths come from separately installed `physx-usd-schemas` metadata without importing USD; missing packages or either incomplete resource directory raise `FileNotFoundError` with installation guidance.
- AC-2: **Population.** A genuine external joint type and custom geometry applied API retain their classification and declared custom attributes through production ovstage population, without substituting built-in schema names.
- AC-3: **Parameters.** Native callbacks receive validated immutable typed parameter records or versioned blobs with documented layout, default values and lifetime; no callback resolves a USD stage.
- AC-4: **Edits and instances.** Parameter edits produce new data and schedule safe dirty/recreation handling; clones/instances receive distinct state and unknown schemas/parameters fail explicitly.

## Test References

- [TEST-COMP-NATIVE-SCHEMA-001](../../tests/native/TEST-COMP-NATIVE-SCHEMA-001.md) - executed registration/population, immutable ingestion, edited state and clone cases.

## Code References

- [Registration and population helpers](../../../python/ovphysx_companion/schemas.py)
- [External schema roundtrip sample](../../../samples/external_schema_authoring.py)
- [Codeless plugin resources](../../../samples/native/schema/resources/plugInfo.json)
- [Schema property definitions](../../../samples/native/schema/resources/generatedSchema.usda)
- [Isolated registration/population tests](../../../tests/test_external_schemas.py)

- [C module/control table](../../../include/ovphysx_companion/native.h)
- [Build-matched C++ SDK](../../../include/ovphysx_companion/native_sdk.h)
- [Module owner and callback bridge](../../../src/Native.cpp)
- [Python control plane](../../../python/ovphysx_companion/plugins.py)
- [Compiled spring sample](../../../samples/native/spring.cpp)
- [Compiled ball sample](../../../samples/native/ball.cpp)
- [Native integration fixtures](../../../tests_native/test_plugins.py)

Runtime integration anchors:

- [ovphysx/ovruntime/source/omni.physics.ovstage/OvstageWalker.cpp](../../../../ovruntime/source/omni.physics.ovstage/OvstageWalker.cpp)
- [ovphysx/python/ovphysx/utils/codeless.py](../../../../python/ovphysx/utils/codeless.py)
- [ovphysx/ovruntime/include/omni/physx/IPhysxCustomJoint.h](../../../../ovruntime/include/omni/physx/IPhysxCustomJoint.h)
- [ovphysx/ovruntime/include/omni/physx/IPhysxCustomGeometry.h](../../../../ovruntime/include/omni/physx/IPhysxCustomGeometry.h)

## Dependencies

- [REQ-COMP-NATIVE-ABI-001](REQ-COMP-NATIVE-ABI-001.md)
- [REQ-COMP-OVSTAGE-001](../exchange/REQ-COMP-OVSTAGE-001.md)
- [REQ-COMP-IDENTITY-001](../exchange/REQ-COMP-IDENTITY-001.md)
- [Companion ADR-0008](../../adr/ADR-0008-native-extension-sdk.md)
