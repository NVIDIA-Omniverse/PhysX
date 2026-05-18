# Changelog
omni.physx.pvd Changelog

## [109.0.10] - 2025-12-15
Version bump.

## [109.0.9] - 2025-12-11
Version bump.

## [109.0.8] - 2025-12-10
Version bump.

## [109.0.7] - 2025-11-28
- Significantly improved frame stepping performance for large OVD scenes (from ~29 seconds to ~9 seconds per frame in tested scenarios)
- Moved recursive visibility update from Python to C++ (`update_ovd_visibility`) for much faster execution
- Added C++ handle-to-prim cache (`build_handle_cache`, `invalidate_handle_cache`) to avoid expensive stage traversal
- Added batch handle resolution (`get_handle_prim_names_batch`) to resolve multiple handles in a single C++ call
- Added prim cache for gizmo drawing to avoid full stage traversal on every frame
- Added early exit in gizmo drawing when no gizmos are enabled
- Added thread-safe mutex protection for prim cache access to prevent race conditions between event handlers and render thread

- Added automatic setting of the USD stage metersPerUnit metadata based on the PhysX tolerancesScale::length value from OVD files.

- Fixed OVD Timeline simulation step input to only load frames when editing is complete, avoiding loading intermediate frames when typing (e.g., typing "500" no longer loads frames 5, 50, and then 500).

- Added support for unique list attributes (OmniPvdWriter::registerUniqueListAttribute) to display in the property window with paginated array display
- Fixed pagination alignment so page index 0 correctly shows elements starting at index 0
- Fixed next page button navigation for arrays with more than 10 elements
- Fixed reference navigation history (< > buttons) to properly update the property window when navigating back/forward
- Fixed reference navigation to preserve scroll position when navigating between attributes on the same prim
- Refactored processCustomAttribute into a single template function to reduce code duplication

## [109.0.6] - 2025-11-25
Version bump.

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
Version bump.

## [108.0.2] - 2025-07-04
Version bump.

## [108.0.1] - 2025-06-30
Version bump.

## [108.0.0] - 2025-05-01
Added automatic directory creation of the output directory hieararchy, defined by the Kit setting /persistent/physics/omniPvdOvdRecordingDirectory.
Added an automatic addition of a slash to the end of the directory string, to make sure the directory path stored in /persistent/physics/omniPvdOvdRecordingDirectory is well formed.

## [107.3.18] - 2025-07-23
Fixed a bug in the OmniPvd function ovdToUsdOverWithLayerCreation exposed in Python as ovd_to_usd_over_with_layer_creation, related to a stream of warning messages in USD.

## [107.2.0] - 2025-04-03
Adding in a stop gap baking feature function ovd_to_usd_over_with_layer_creation exposed through the omni.physx.pvd interface

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
