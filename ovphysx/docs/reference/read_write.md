<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Read and Write Contract

The complete rules for reading and writing simulation data. The
[Simulation Data](../simulation_data/selecting_objects.md) guides introduce the same API in short form.

| Page | What It Covers |
| --- | --- |
| [Overview](../read_write/index.md) | How a read or write session works from start to end. |
| [Data Model and Semantics](../read_write/data_model.md) | Groups, shapes, units and what happens when data is not available. |
| [Device Residency and Interop](../read_write/device.md) | Where each value lives, DLPack ownership, CUDA synchronization and CUDA graphs. |
| [Readable Data](../read_write/readable.md) | Every value you can read, by object type. |
| [Discovering Attribute Metadata](../read_write/discovery.md) | How a program finds the attributes, types and units at run time. |
| [Detailed I/O Attribute Reference](../read_write/attributes/index.md) | Exact names, types, layouts and units for every object type. |
| [Writable Data](../read_write/writable.md) | Every value you can write, and when each write takes effect. |
| [Vehicle Examples](../read_write/vehicles.md) | Reading and writing vehicle data in C++ and Python. |
| [Path Patterns](../read_write/path_patterns.md) | The full syntax for selecting objects by path. |
| [Known Limitations and Gaps](../read_write/limitations.md) | What the read and write API does not cover yet. |

```{toctree}
:hidden:
:maxdepth: 1

/read_write/index
/read_write/data_model
/read_write/device
/read_write/readable
/read_write/discovery
/read_write/attributes/index
/read_write/writable
/read_write/vehicles
/read_write/path_patterns
/read_write/limitations
```
