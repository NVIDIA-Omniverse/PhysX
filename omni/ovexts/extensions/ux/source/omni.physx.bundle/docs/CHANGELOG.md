<!-- SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Changelog

## [110.3.2] - 2026-08-20

### Core

**General**

    - Fixed

        - Colliders of instanceable prims could end up at the wrong transform
        - A crash when creating a particle set and memory could not be allocated
        - A crash when visualizing deformable attachments or collision filters that reference missing prims

**Fabric**

    - Fixed

        - Transforms of nested rigid bodies were not written back to USD when the simulation stops
        - Crashes when simulating deformable bodies with Fabric
        - Crashes when running more than one simulation session in a single process, including on multi-GPU systems

## [110.3.1] - 2026-08-11

### Core

**General**

    - Added

        - Support for the Newton ``newton:velocityLimit`` joint attribute

    - Changed

        - Updated PhysX SDK to version 5.9.1-110.3

    - Fixed

        - Crashes caused by instanced sphere points with custom colliders
        - Dangling internal actors after changing a simulation owner
        - Out-of-bounds reads while parsing rigid-body instancers and updating mass properties
        - Handling of missing default-simulator settings and PhysX error event streams
        - A data race in PhysX Replicator profiling statistics
        - Use-after-free issues involving released active actors and asynchronous cooking data during simulation reset
        - PVD setting changes while PhysX scenes are live
        - Crashes caused by short Bezier splines
        - Rigid-body type changes when no replacement actor can be created
        - SDF collider triangle winding and unintended modification of caller-owned SDF descriptors during cooking
        - A null PhysX foundation dereference in CUDA helper extensions

**Fabric**

    - Fixed

        - Use-after-free issues in the Fabric DirectGPU path after releasing a PhysX scene
        - A crash when the Fabric interface is unavailable

**TensorAPI**

    - Fixed

        - Tensor views are invalidated when their physics scene is deleted
        - Out-of-bounds reads for articulations with ragged link and joint paths

### UI

**General**

    - Fixed

        - Physics Inspector hanging when toggled from the UI
        - Menu actions remaining registered after their menu items are removed

## [110.3.0] - 2026-07-28
Internal release.

## [110.2.1] - 2026-07-28

### Core

**General**

    - Changed

        - Joint maximum velocity now defaults to the schema's unlimited value

    - Fixed

        - GPU teardown crashes in the gyroscopic snippet and acceleration-copy task
        - CUDA context manager use-after-free when PhysX scenes are live
        - Crashes caused by invalid GPU narrow-phase and convex-hull cooking input
        - Picker use-after-free after releasing a scene
        - Mimic-joint gearing and offset signs when the joint body order is reversed
        - PhysX Replicator crashes during out-of-memory handling and parallel compound-shape replication
        - Shutdown use-after-free issues in physics stage-update and graph plugins

**TensorAPI**

    - Fixed

        - Shared-material lookup returning the wrong material when a key is requested again
        - Duplicate matching of nested rigid bodies beneath recursive leaf patterns
        - Use-after-free when releasing deformable-material views

### UI

**General**

    - Changed

        - Joint Create menu availability now follows the same selection limits as the Add menu

    - Fixed

        - A crash when joint authoring encounters an invalid stage entry

## [110.2.0] - 2026-06-09
Internal release.
