<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Hello World: Populate ovstage and Step

This tutorial shows the smallest end-to-end ovphysx workflow: create an instance,
populate an ovstage Stage from USD, attach it, step simulation, and clean up
resources. You can use this flow as the starting point for larger integrations.

## Prerequisites

- Prepare a readable USD file for ovstage population.

## Code Language

### Python

Install the package first:

```bash
pip install ovphysx
```

```{literalinclude} ../../tests/python_samples/hello_world.py
:language: python
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### C

Download the OVPhysX SDK and matching native OVStage archive as described in
the [SDK Quickstart](quickstart.md), and extract them as separate package roots.

**CMakeLists.txt**

Every C sample uses `find_package(ovphysx)` and links against `ovphysx::ovphysx`. Here is the `CMakeLists.txt` for `hello_world_c`:

```{literalinclude} ../../tests/c_samples/hello_world_c/CMakeLists.txt
:language: cmake
```

Build by pointing `CMAKE_PREFIX_PATH` at both package roots (refer to
[SDK Quickstart](quickstart.md) for details).

**Source**

```{literalinclude} ../../tests/c_samples/hello_world_c/main.c
:language: c
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

## Result

After this tutorial, you can step the simulation from both Python and C and release all resources cleanly.
