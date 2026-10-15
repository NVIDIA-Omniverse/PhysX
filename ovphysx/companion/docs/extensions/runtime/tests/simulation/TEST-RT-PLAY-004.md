<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-PLAY-004
maps_to: REQ-RT-PLAY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Play cannot displace an existing backend attachment owner. Verifies REQ-RT-PLAY-001 AC-2.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; the live stage has exactly one physics scene.
- A separate legitimate ovphysx owner already holds that stage's backend attachment, with recorded instance identity, object state and completed step count.
- The Kit adapter has only remembered the same host stage in Stopped and owns no backend resources.

## When

- Request Play through the generic hooks and allow the attempted adapter initialization/cleanup to finish.
- Read state through the original owner and then release it through its own normal teardown.

## Then

- Play reports an attachment-ownership failure, never reaches Running and leaves the original owner's attachment, objects and step count unchanged; only resources acquired by the unsuccessful adapter attempt are cleaned up. The borrowed host stage remains valid. (REQ-RT-PLAY-001 AC-2).

## Why not implemented

The ovphysx-backed Kit adapter and its ownership instrumentation are not implemented in this companion design set. This requires an integration case with the new adapter; existing attach tests are component evidence only.

- [Current ovphysx attachment contract](../../../../../../include/ovphysx/ovphysx.h) documents the process-wide attached-source owner.

Mapped requirement: [REQ-RT-PLAY-001](../../requirements/simulation/REQ-RT-PLAY-001.md).
