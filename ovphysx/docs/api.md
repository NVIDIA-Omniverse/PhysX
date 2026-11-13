<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# C API Reference

The full C API is defined in the headers listed below. Paths are relative to
the C/C++ SDK or the ovphysx source root. The Python wheel does not install an
SDK `include/` tree. For C/C++ development, download the matching ovphysx SDK
from [PhysX GitHub Releases](https://github.com/NVIDIA-Omniverse/PhysX/releases).

For header copies from an installed wheel's build, open `docs/html/api.html`
inside its `ovphysx` package and use **Header downloads** on that rendered page.
The [published HTML reference](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/api.html)
also provides header downloads and full API descriptions; it tracks the latest
published documentation and may differ from your installed version.

- `include/ovphysx/ovphysx.h` -- API functions
- `include/ovphysx/ovphysx_types.h` -- types and enums
- `include/ovphysx/ovphysx_attributes.h` -- `OVPHYSX_ATTR_*` attribute-name macros; include it alongside `ovphysx.h`, which does not include it
- `include/ovphysx/ovphysx_geometry_query.h` -- geometry queries (SDF shape evaluation); include it alongside `ovphysx.h`, which does not include it
- `include/ovphysx/ovphysx_population.h` -- the physics population description for OVStage population; include it alongside `ovphysx.h`, which does not include it
- `include/ovphysx/ovphysx_utils.h` -- utilities, such as an articulation-joint group regrouped into one array per joint for ovstage; include it alongside `ovphysx.h`, which does not include it
- `include/ovphysx/ovphysx_legacy.h` -- deprecated families (tensor bindings, binding-coupled control, scene queries, contact bindings, SDF views, object-type lookup)
- `include/ovphysx/ovphysx_legacy_types.h` -- types used only by the legacy header
- `include/ovphysx/ovphysx_config.h` -- scoped config value builders (`ovphysx_config_value_*`)
- `include/ovphysx/ovphysx_legacy_config.h` -- deprecated typed entry builders

C++ convenience wrappers (experimental, C++17):

- `include/ovphysx/experimental/ovphysx.hpp` -- RAII instance wrapper
- `include/ovphysx/experimental/Helpers.hpp` -- RAII helpers, including `WaitResult`
- `include/ovphysx/experimental/OvStageOutput.hpp` -- application-owned CPU/CUDA world-transform publication in `ovphysx::utils`; see [ovstage integration](ovstage_integration.md#c-utility-publish-fixed-world-transforms)
- `include/ovphysx/experimental/TensorBinding.hpp` -- RAII tensor binding wrapper (**deprecated**; use `ovphysx_read` / `ovphysx_write`)

```{eval-rst}
Header downloads
----------------

These copies match this documentation build. For a complete C/C++ development
package, use the SDK linked above.

* :download:`ovphysx.h <../include/ovphysx/ovphysx.h>`
* :download:`ovphysx_types.h <../include/ovphysx/ovphysx_types.h>`
* :download:`ovphysx_attributes.h <../include/ovphysx/ovphysx_attributes.h>`
* :download:`ovphysx_geometry_query.h <../include/ovphysx/ovphysx_geometry_query.h>`
* :download:`ovphysx_population.h <../include/ovphysx/ovphysx_population.h>`
* :download:`ovphysx_utils.h <../include/ovphysx/ovphysx_utils.h>`
* :download:`ovphysx_legacy.h <../include/ovphysx/ovphysx_legacy.h>`
* :download:`ovphysx_legacy_types.h <../include/ovphysx/ovphysx_legacy_types.h>`
* :download:`ovphysx_config.h <../include/ovphysx/ovphysx_config.h>`
* :download:`ovphysx_legacy_config.h <../include/ovphysx/ovphysx_legacy_config.h>`
* :download:`experimental/ovphysx.hpp <../include/ovphysx/experimental/ovphysx.hpp>`
* :download:`experimental/Helpers.hpp <../include/ovphysx/experimental/Helpers.hpp>`
* :download:`experimental/OvStageOutput.hpp <../include/ovphysx/experimental/OvStageOutput.hpp>`
* :download:`experimental/TensorBinding.hpp <../include/ovphysx/experimental/TensorBinding.hpp>`
```

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
the articulation query, or `query` passes one you hold. It updates articulation
forward kinematics only; it does not run a simulation step, collision/contact
processing or a warmup, so a DirectGPU scene must have stepped once.

On a DirectGPU scene the refresh is ordered on the CUDA stream and event in its
`ovstage_cuda_sync_t` argument (`cuda_stream=` / `cuda_wait_event=` in Python),
as a write commit is: it runs after the work queued there, including a write
committed on that stream, and the task completes once the refresh is queued.
It can therefore be captured into a CUDA graph with the commits it follows, on a
stream of the scene's CUDA context; see
[Capturing a write step](read_write/device.md#capturing-a-write-step-in-a-cuda-graph).
On a scene without the DirectGPU API it runs on the host and cannot be captured.

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

## Physics Population Description

OVStage populates physics data from USD only through a population description. Declared in
`ovphysx_population.h`; include it alongside `ovphysx.h`, which does not include it.

`ovphysx_get_physics_population_desc()` returns the description of everything ovphysx reads. Pass
it alone to an OVStage `*_with_desc` population entry point to populate physics, or beside a
description with `domains = OVSTAGE_POPULATION_DOMAIN_RENDERING` to populate render prims as well. It
needs no instance and is safe from any thread. The Python function is
`ovphysx.population.physics_population_desc()`. The
[physics population description reference](population/physics-population-description.md) lists
what it selects and derives.

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_population.h
```

## Utilities

Conversions of ovphysx output into other layouts. Declared in `ovphysx_utils.h`; include it
alongside `ovphysx.h`, which does not include it.

Use `ovphysx_create_joint_arrays_from_read_group()` or `ovphysx_create_joint_arrays_from_map_group()`
to view an articulation-joint group, which has one row per DOF, as one array per joint, the form
ovstage stores, without copying. See
[ovstage integration](ovstage_integration.md#c-utility-joint-groups-as-one-array-per-joint). The
Python counterpart is `ovphysx.utils.split_joint_group`.

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_utils.h
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

## Attribute token interning

`ovphysx_intern_attribute_name` converts an attribute string such as `"position"`
into a numeric ID in the attached ovstage's path dictionary. Include
`ovphysx_attribute_tokens.h` and pass the `.string` member of an `OVPHYSX_NAME_*`
object. The portable name object itself never caches a token.

The numeric token is valid only in that dictionary. Keep token caches with its
owner, invalidate them when the dictionary is replaced or destroyed, and never
reuse a token with another stage's dictionary. On failure the helper clears the
output token and forwards the dictionary error. Release a nonempty `result.error`
with `path_dictionary_release_error(dictionary, result.error)` before destroying
the dictionary. Invalid arguments return `OVX_API_ERROR` with an empty error string.

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_attribute_tokens.h
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
