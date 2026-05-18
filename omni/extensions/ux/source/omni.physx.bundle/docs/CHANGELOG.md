# Changelog

## [110.0.7] - 2026-03-06

### Core

**General**

    - Changed

        - Suppresed warnings for ununsed articulation joint attributes (info only)

    - Fixed

        - Fixed shutdown crash where stepper tasks could access freed CUDA context.
        - Scope to Display PhysX Scene is Missing

**Fabric**

    - Added

        - Arm64 support for deformable fabric updates

    - Fixed

        - Fixed Fabric update when simulation was stopped through reset_simulation

## [110.0.6] - 2026-02-26

### Core

**General**

    - Changed

        - Renaming of some functions and python bindings on the Physics Simulation interface.

**Fabric**

    - Fixed

        - GPU Fabric synchronization causing viewport not being updated.


## [110.0.5] - 2026-02-19

### Core

**General**

    - Fixed

        - Tensor demos failure when creating a simulation view.

## [110.0.4] - 2026-02-12

### Core

**General**

    - Added

        - The volume deformable body feature now supports UsdGeomTetMesh with left handed orientation.
        - Physics property window widgets support for multiple simulation engines.

    - Fixed

        - ResetXformOp correctly skips transformation hierarchy update for physics
        - Nested bodies transformation update fixed

**Fabric**

    - Fixed

        - Support for transformation changes under changed transform

**TensorAPI**

    - Changed

        - Removed Python dependency on carb; SimulationView.set_gravity/get_gravity now use plain sequences/tuples. Migration guide:
Replace sim_view.set_gravity(carb.Float3(x, y, z)) with sim_view.set_gravity([x, y, z]) (or any object with x/y/z attributes).
Update gravity = sim_view.get_gravity() usages to treat the result as a 3‑tuple (x, y, z) instead of a carb.Float3 (i.e., use indexing or tuple unpacking).

## [110.0.3] - 2026-02-05
Version bump.

## [110.0.2] - 2026-01-27

### Core

**General**

    - Removed

        - Removed deprecated solver residual reporting from PhysX SDK, physx schema and omni.physx

## [110.0.1] - 2026-01-13
Version bump.

## [110.0.0] - 2025-11-28
Internal release.
