<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# C API Reference

The full C API is defined in these headers:

- [`include/ovphysx/ovphysx.h`](../include/ovphysx/ovphysx.h) -- API functions
- [`include/ovphysx/ovphysx_types.h`](../include/ovphysx/ovphysx_types.h) -- types and enums
- [`include/ovphysx/ovphysx_config.h`](../include/ovphysx/ovphysx_config.h) -- typed config entry builders

C++ convenience wrappers (experimental, C++17):

- [`include/ovphysx/experimental/ovphysx.hpp`](../include/ovphysx/experimental/ovphysx.hpp) -- RAII instance wrapper
- [`include/ovphysx/experimental/Helpers.hpp`](../include/ovphysx/experimental/Helpers.hpp) -- RAII helpers (WaitResult, etc.)
- [`include/ovphysx/experimental/TensorBinding.hpp`](../include/ovphysx/experimental/TensorBinding.hpp) -- RAII tensor binding wrapper

For rendered documentation with full descriptions, refer to the
[built HTML docs](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/api.html).

## C API Functions

### Articulation Kinematic Update

Use `ovphysx_update_articulations_kinematic()` after writing articulation DOF
positions when link-pose tensors must reflect the new joint positions before
the next simulation step. The Python method is
`PhysX.update_articulations_kinematic()`, and the experimental C++ wrapper is
`PhysX::updateArticulationsKinematic()`. The operation is synchronous and
updates articulation forward kinematics only; after any required first GPU
warmup, it does not run a normal simulation step or collision/contact
processing.

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx.h
```

## C API Types

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_types.h
```

## Typed Config Entries

```{eval-rst}
.. doxygenfile:: include/ovphysx/ovphysx_config.h
```

## C++ Wrappers (Experimental)

C++17 RAII wrappers and helpers in the `ovphysx` namespace.

```{eval-rst}
.. doxygennamespace:: ovphysx
   :members:
   :undoc-members:
```
