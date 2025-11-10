# Changelog
omni.physx.pvd Changelog

## [107.3.24] - 2025-09-05
Bug Fix: References that did not resolve to an object were previously not selectable but failed silently; this behavior is now corrected.
Reference Handling:
  Handles with value 0 are shown as "NULL".
  Handles that don’t resolve are displayed as "INVALID".
  Handles that do resolve to an object display the USD Prim name; if the object is inactive, it is shown grayed out but still displays its name.
  The handle value is now displayed alongside the USD Prim name for references.

Property Array Improvements:
  Property arrays with more than 10 values are now paginated, facilitating inspection of large arrays.

Navigation Enhancements:
  "Previous" and "Next Reference" buttons have been added below the search bar in the OVD Tree.
  Contextual navigation ensures that using "jump to reference" buttons in the property view returns to the exact scroll position and pagination state.

## [107.3.23] - 2025-09-02
Version bump.

## [107.3.22] - 2025-08-27
Version bump.

## [107.3.21] - 2025-08-18
Version bump.

## [107.3.20] - 2025-08-04
Version bump.

## [107.3.19] - 2025-07-31
Version bump.

## [107.3.18] - 2025-07-23
Fixed a bug in the OmniPvd function ovdToUsdOverWithLayerCreation exposed in Python as ovd_to_usd_over_with_layer_creation, related to a stream of warning messages in USD.

## [107.3.17] - 2025-07-22
Version bump.

## [107.3.16] - 2025-07-18
Version bump.

## [107.3.15] - 2025-07-08
Version bump.

## [107.3.14] - 2025-07-03
Version bump.

## [107.3.13] - 2025-06-25
Version bump.

## [107.3.12] - 2025-06-18
Version bump.

## [107.3.11] - 2025-06-12
Version bump.

## [107.3.10] - 2025-06-06
Version bump.

## [107.3.9] - 2025-05-30
Fixed a bug in the OmniPvd property view connected to the Select button for references, now returns the currently active selected object.

## [107.3.8] - 2025-05-28
Version bump.

## [107.3.7] - 2025-05-22
Version bump.

## [107.3.6] - 2025-05-15
Version bump.

## [107.3.5] - 2025-05-14
Version bump.

## [107.3.4] - 2025-05-11
Version bump.

## [107.3.3] - 2025-05-06
Version bump.

## [107.3.2] - 2025-05-02
Version bump.

## [107.3.1] - 2025-04-28
Version bump.

## [107.3.0] - 2025-04-10
Version bump.

## [107.2.2] - 2025-04-04
Version bump.

## [107.2.1] - 2025-04-03
Adding in a stop gap baking feature function ovd_to_usd_over_with_layer_creation exposed through the omni.physx.pvd interface

## [107.2.0] - 2025-03-21
Version bump.

## [107.0.18] - 2025-02-07
Deprecation of the PhysX baking UI feature

## [107.0.16] - 2024-12-05
Version bump.

## [107.0.15] - 2024-12-02
Version bump.

## [107.0.14] - 2024-11-17
Initial OpenUSD 24.05 and Python 3.11 version.

## [106.5.0] - 2024-11-17
Version bump.

## [106.4.7] - 2024-11-15
Version bump.

## [106.4.6] - 2024-11-14
Version bump.

## [106.4.5] - 2024-11-06
Version bump.

## [106.4.4] - 2024-11-01
- Added parsing of OVD files that contain PxConvexCoreGeometry objects, speficically
  - PxConvexCoreCylinder and PxConvexCoreCone
  - Backwards compatible with OVD files that contain PxCustomGeometryExtCylinderCallbacks and PxCustomGeometryExtConeCallbacks

## [106.4.3] - 2024-10-30
- OVD Direct GPU API (PxDirectGPUAPI) recording for functions setRigidDynamicData and setArticulationData
  - PxRigidBody has two additional attributes : force and torque
  - PxArticulationJointReducedCoordinate has an additional attribute : jointForce
  - Excluded : tendon data

## [106.4.2] - 2024-10-28
Version bump.

## [106.4.1] - 2024-10-15
Version bump.

## [106.4.0] - 2024-10-07
- Added parsing of OVD files that contain pre-simulation and post-simulation data
- Added the OVD Timeline
  - Allows for precisce simulation step scrubbing OVD files
  - Allows for more precise inspection of files with pre-simulation and post-simulation data
  - Is implemented as an alternative to the Kit timeline for a simulation step centric inspection
  - Does not skip frames in replay mode
  - Allows to inspect scene setup, user set operations and simulation results separately
  - Recreates the original classic PVD timeline
