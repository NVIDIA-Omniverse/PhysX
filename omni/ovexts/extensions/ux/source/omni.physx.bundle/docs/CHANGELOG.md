# Changelog

## [110.1.13] - 2026-06-04

### UI

**General**

    - Fixed

        - Cleaned up stale widgets that caused errors when property window filtering was used
        - Maximum Joint Velocity property now shows the proper default value

## [110.1.12] - 2026-05-27

### UI

**General**

    - Fixed

        - Collision debug visualization while simulation is running

## [110.1.11] - 2026-05-26

### Core

**General**

    - Fixed

        - PhysX stepper deadlock while debug vis was enabled during simulation
        - Deformables fabric reset initial transformations

### UI

**General**

    - Fixed

        - Newton Mimic Joint widget visibility
        - Variant switch process of invalid paths
        - Physics umbrella getSimulationTimeStepsPerSecond correctly resolves Newton vs PhysX schema

## [110.1.10] - 2026-05-20

### Core

**General**

    - Fixed

        - SimulationInfoWindow title now matches its menu label
        - Missing removal button on Articulation Root property panel
        - Improved deformable-rigid attachment and contact load transmission

## [110.1.9] - 2026-05-14

### Core

**General**

    - Fixed

        - Fix attachment failures for deformable to articulation link
        - Fix the DeformablePostSolveCallback crash issue
        - Setup PhysX deformable volume correctly for hexahedral tet mesh solver
        - GPU selection logic and improved CUDA context/device handling

**TensorAPI**

    - Changed

        - Tensor API invalidates views when deleting deformables, and warns if view is still in use.

    - Fixed

        - Fix NULL physXPtr dereference in fabric deformable body managers

## [110.1.8] - 2026-04-30

### Core

**General**

    - Fixed

        - Surface deformable - rigid attachments ignored YZ-axis correction with PGS solver
        - Articulation joint force readback has been made more accurate for scenes configured with TGS solver and physxScene:enableExternalForcesEveryIteration set to True.

### UI

**General**

    - Added

        - Fabric support for Rigid Body Manipulator

## [110.1.7] - 2026-04-23
Version bump.

## [110.1.6] - 2026-04-16
Version bump.

## [110.1.5] - 2026-04-14
Version bump.

## [110.1.4] - 2026-04-10
Version bump.

## [110.1.3] - 2026-04-02

### Core

**TensorAPI**

    - Fixed

        - Exposed correctly SimulationView

### UI

**General**

    - Changed

        - Variant switcher logic to respect local changes


## [110.1.2] - 2026-03-30

### Core

**General**

    - Removed

        - Removed deprecated deformable and particle cloth schemas and functionality from Omni PhysX

## [110.1.1] - 2026-03-03

### Core

**General**

    - Fixed

        - Fix crash if deformable PhysX shape creation fails.

## [110.1.0] - 2026-02-09
Internal release.
