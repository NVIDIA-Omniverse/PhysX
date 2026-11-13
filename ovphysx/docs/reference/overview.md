<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Overview and Migration

The full overview of ovphysx and the developer guide, plus guides for moving code between
releases. The developer guide also covers expert topics such as caches, threading, logging
callbacks and direct access to PhysX objects.

| Page | What It Covers |
| --- | --- |
| [ovphysx Overview](../ovphysx_overview.md) | What ovphysx contains, its versioning and API policy, system requirements and runtime warnings. |
| [Developer Guide](../developer_guide.md) | The execution model, configuration, lifecycle, threading, errors, logging and advanced features. |
| [Migration from 0.6 to 0.7](../migrating_0_6_to_0_7.md) | Code changes needed to move from release 0.6 to 0.7. |
| [Migration from 0.7 to 0.8](../migrating_0_7_to_0_8.md) | Code changes needed to move from release 0.7 to 0.8. |
| [Instance and OVD API Migration](../ovd_api_migration.md) | Moving to the current instance and OmniPVD recording API. |
| [Migrating off Tensor Bindings](../tensor_binding_migration.md) | Replacing the deprecated tensor bindings with reads and writes. |
| [OV Libraries V2 Transitional Annex](../ov_libraries_v2_annex.md) | A transitional note on the shared timeout type. |

```{toctree}
:hidden:
:maxdepth: 1

/ovphysx_overview
/developer_guide
/migrating_0_6_to_0_7
/migrating_0_7_to_0_8
/ovd_api_migration
/tensor_binding_migration
/ov_libraries_v2_annex
```
