<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# C API Reference

The full C API is defined in these headers:

- [`include/ovphysx/ovphysx.h`](../include/ovphysx/ovphysx.h) -- API functions
- [`include/ovphysx/ovphysx_types.h`](../include/ovphysx/ovphysx_types.h) -- types and enums
- [`include/ovphysx/ovphysx_attributes.h`](../include/ovphysx/ovphysx_attributes.h) -- `OVPHYSX_ATTR_*` attribute-name macros; include it alongside `ovphysx.h`, which does not include it
- [`include/ovphysx/ovphysx_geometry_query.h`](../include/ovphysx/ovphysx_geometry_query.h) -- geometry queries (SDF shape evaluation); include it alongside `ovphysx.h`, which does not include it
- [`include/ovphysx/ovphysx_legacy.h`](../include/ovphysx/ovphysx_legacy.h) -- deprecated families (tensor bindings, binding-coupled control, scene queries, contact bindings, SDF views, object-type lookup)
- [`include/ovphysx/ovphysx_legacy_types.h`](../include/ovphysx/ovphysx_legacy_types.h) -- types used only by the legacy header
- [`include/ovphysx/ovphysx_config.h`](../include/ovphysx/ovphysx_config.h) -- scoped config value builders (`ovphysx_config_value_*`)
- [`include/ovphysx/ovphysx_legacy_config.h`](../include/ovphysx/ovphysx_legacy_config.h) -- deprecated typed entry builders

C++ convenience wrappers (experimental, C++17):

- [`include/ovphysx/experimental/ovphysx.hpp`](../include/ovphysx/experimental/ovphysx.hpp) -- RAII instance wrapper
- [`include/ovphysx/experimental/Helpers.hpp`](../include/ovphysx/experimental/Helpers.hpp) -- RAII helpers, including `WaitResult`
- [`include/ovphysx/experimental/OvStageOutput.hpp`](../include/ovphysx/experimental/OvStageOutput.hpp) -- application-owned CPU/CUDA world-transform publication in `ovphysx::utils`; see [ovstage integration](ovstage_integration.md#c-utility-publish-fixed-world-transforms)
- [`include/ovphysx/experimental/TensorBinding.hpp`](../include/ovphysx/experimental/TensorBinding.hpp) -- RAII tensor binding wrapper (**deprecated**; use `ovphysx_read` / `ovphysx_write`)

For rendered documentation with full descriptions, refer to the
[built HTML docs](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/api.html).

## C API Functions

### Articulation Kinematic Update

Use `ovphysx_update_articulations_kinematic()` after writing articulation DOF
positions or velocities when link state must reflect the new generalized state
before the next simulation step. Pass a live `OVPHYSX_QUERY_TARGET_ARTICULATION`
query to select the articulations and POSITION and/or VELOCITY flags. The Python method is
`PhysX.update_articulations_kinematic()`, and the experimental C++ wrapper is
`PhysX::updateArticulationsKinematic()`. The C and C++ calls return a task
receipt; wait for it before using updated link poses. Python waits internally
and returns `None`, raising any execution failure; its `scope` argument creates
the articulation query. The native operation currently
executes inline. It updates articulation forward kinematics only; after any
required first GPU warmup, it does not run a normal simulation step or
collision/contact processing.

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx.h
```

## Geometry Queries

Questions asked of the scene's collision geometry at your own inputs. Declared in
`ovphysx_geometry_query.h`; include it alongside `ovphysx.h`, which does not include it.

Use `ovphysx_evaluate_sdf_shapes()` to evaluate signed distance fields at your own points. Pass a
live `OVPHYSX_QUERY_TARGET_SDF_SHAPE` query, a points tensor `[N, Q, 3]` and an output tensor
`[N, Q, 4]` (gradient, then signed distance), float32 on the scene's CUDA device, plus the producer
handoff for the points, as for `ovphysx_commit_group`. The output is written when the task completes. The Python method is `PhysX.evaluate_sdf_shapes()`, which opens
and releases the query itself and waits internally; the experimental C++ wrapper is
`PhysX::evaluateSdfShapes()`. See [SDF shapes](read_write/index.md#sdf-shapes) for rows,
frame and device requirements.

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_geometry_query.h
```

## Legacy C API

Deprecated families. `ovphysx.h` includes this header so existing includes keep compiling.

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_legacy.h
```

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_legacy_types.h
```

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_legacy_config.h
```

## C API Types

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_types.h
```

## Attribute names

`OVPHYSX_ATTR_*` string literals for the read and write vocabulary. Include
`ovphysx_attributes.h` alongside `ovphysx.h`, which does not include it.

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_attributes.h
```

## Scoped Config Values

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_config.h
```

## OVD Recording

Set the global recording capability before the first successful instance creation,
attach a scene, then pass an explicit destination to `ovphysx_ovd_start_recording()`.
Start before the first simulation step to capture it. Startup configuration is
removed; see the [OVD migration guide](ovd_api_migration.md).
The old recording names remain deprecated forwarding exports
in 0.7. The rest of the 0.6 to 0.7 source break is the
[migration guide](migrating_0_6_to_0_7.md).

## C++ Wrappers (Experimental)

C++17 RAII wrappers and helpers in the `ovphysx` namespace.

```{eval-rst}
.. doxygennamespace:: ovphysx
   :members:
   :undoc-members:
```
