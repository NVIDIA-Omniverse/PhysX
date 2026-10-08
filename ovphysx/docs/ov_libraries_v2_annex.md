<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# OV Libraries V2 Transitional Annex: Timeout Type

OV Libraries V2 is the Omniverse convention that gives sibling libraries a shared
`ovx_` prefix for types they have in common. Its shared `ovx_` utility header is
not yet available. Until it is published, ovphysx defines the following local,
byte-identical timeout type under its own library prefix:

| Local definition | Representation | Unit and reserved values | Future shared definition |
| --- | --- | --- | --- |
| `ovphysx_timeout_t` | `uint64_t` | Nanoseconds; `OVPHYSX_TIMEOUT_POLL` is `0`, and `OVPHYSX_TIMEOUT_INFINITE` is `UINT64_MAX` | `ovx_timeout_ns_t` |

When the shared header becomes available, ovphysx will migrate this local type
to `ovx_timeout_ns_t`. The migration is a mechanical rename: the byte size,
alignment, value representation, units, and timeout semantics remain unchanged.

## Scope

This page records only the transitional `ovphysx_timeout_t` definition added
for the native async timeout contract. It is not a complete inventory of every
ovphysx-local type or macro that may eventually move to the shared `ovx_`
header, and it must not be used as evidence that the broader transitional-type
review is complete.
