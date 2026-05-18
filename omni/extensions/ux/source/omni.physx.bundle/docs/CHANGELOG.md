# Changelog

## [109.0.10] - 2025-12-15

### Core

**Fabric**

    - Fixed

        - Avoid unnecessary cache rebuild in GPU interop with Fabric

## [109.0.9] - 2025-12-11

### Core

**General**

    - Changed

        - PhysX PxArray, PxBitMap interface changes as a preparation for pinned host OOM handling.

## [109.0.8] - 2025-12-10

**General**

    - Changed

        - Removing beta-status from new defromable body feature and making it the default in UI

    - Fixed

        - Slow shutdown fixes for pvd and tensors

**C++ API**

    - Fixed

        - Creating a PhysX PxScene now sets the eDISABLE_SLEEPING flag for the DIRECT_GPU_API mode

## [109.0.7] - 2025-11-28

### Core

**Fabric**

    - Fixed

        - Direct GPU mode uses the correct GPU in a multi-GPU setup
        - Fix crash due to 0 deformable skinning vertices when running in Fabric

## [109.0.6] - 2025-11-25

### Core

**General**

    - Fixed

        - A non-articulation UsdPhysicsJoint with all angular degrees of freedom configured to be free or limited might not have reached the specified angular drive targets in certain setups.
        - An articulation UsdPhysicsJoint might not have reached the specified angular drive target if the local joint frame orientations defined by the localRot attributes were non identity.
        - Kinematic body runtime switch fix
        - Top level collisionAPI transformation update fix
        - Collider UI now shows additional mesh approximation properties even when the additional API is not applied

**Fabric**

    - Fixed

        - Fabric TfToken change tracking fixed

### UI

**General**

    - Added

        - Added UI for toggling physics simulations

## [109.0.5] - 2025-11-19
Version bump.

## [109.0.4] - 2025-10-24
Version bump.

## [109.0.3] - 2025-10-14
Version bump.

## [109.0.2] - 2025-10-06
Version bump.

## [109.0.1] - 2025-10-05
Version bump.

## [109.0.0] - 2025-08-22

### Core

**Fabric**

    - Changed
        - Moved to Fabric Path Token

## [108.0.2] - 2025-07-04
Version bump.

## [108.0.1] - 2025-06-30

### Core

**General**

    - Fixed

        - Fix mesh winding order support for some mesh cooking code paths

### Vehicles

**General**

    - Fixed

        - Fix test_up_and_forward_axis_combinations test in Kit to work with PxConvexCore cylinders

## [108.0.0] - 2025-06-18
Internal release.
