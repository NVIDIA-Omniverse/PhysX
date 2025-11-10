# Changelog

## [107.3.24] - 2025-09-05

### Core

**General**

    - Added

        - New custom attribute physxScene:solveArticulationContactLast.  This reorders the solver as a potential aid for gripping scenarios.

    - Fixed

        - Fixed physics umbrella contact event type python binding
        - Fixed crash when PhysX mesh cooking stream overflows 
        - Fixed a crash when trying to doing a scene query without the cooking service being available

## [107.3.23] - 2025-09-02

### Core

**General**

    - Fixed

        - Omni Physics Umbrella fixes
        - Quasistatic mode now works properly when omni.physx.fabric is enabled
        - Fixed crash when update was called without omni.physx initialized
        - Fixed crash when material failed to be created within PhysX SDK
        - Fixed crash when default scene material was released and new scene created

**Fabric**

    - Fixed

        - Update Kinematic Bodies in omni.physx.fabric by default 

## [107.3.22] - 2025-08-27

### Core

**General**

    - Fixed

        - FSD & Particle Postprocess (Isosurface/Smoothing/Anisotropy) USD writes would cause static rigid collider transforms to be reset to identity,leading to erroneous collisions.   
        - Deformable render mesh skinning now works with PhysxAutoDeformableMeshSimplificationAPI on surface deformables. The remeshing option will lead to bad results however, it should be disabled.
        - Fixed crash in SurfaceDeformablePostSolveCallback::onPostSolve() when an invalid buffer is used in surface deformable skinning
        - Fixed index out of range crash in PhysXUsdPhysicsInterface::createDeformableCollisionFilter() 

**Fabric**

    - Fixed

        - Omni.physx.fabric extension shutdown turns off the fabric output setting

## [107.3.21] - 2025-08-18

### Core

**General**

    - Fixed

        - Cooking read data from USD with precedence

**Fabric**

    - Changed

        - Omni.physx.fabric resume does not write initial pose into fabric for every resume, just for the initial one

    - Fixed

        - Fix deformable surface does not reset to original position when stopped

## [107.3.20] - 2025-08-04

### Core

**General**

    - Fixed

        - Fixed PhysXReplicator multithreading crash

## [107.3.19] - 2025-07-31

### Core

**General**

    - Changed

        - Stage update resume no longer triggers parsing if app simulation play is disabled

### UI

**General**

    - Changed

        - SupportUI initial stage traversal switched to USDRT

    - Fixed

        - Fix index out of range issue in deformable mesh visualizer

## [107.3.18] - 2025-07-23

### Core

**TensorAPI**

    - Fixed

        - Fixed issue when TensorAPI used stale pointers if all physics objects were destroyed

## [107.3.17] - 2025-07-22

### Core

**TensorAPI**

    - Fixed

        - Fixed TensorAPI USDRT initialization, happens now only if Fabric cloning is used

## [107.3.16] - 2025-07-18

### Core

**General**

    - Fixed

        - Fixed parsing of filtered pairs on deformable body collision meshes.
        - Fixed overflow on internal API flags leading to deformable mesh simplification being broken.

**TensorAPI**

    - Fixed

        - Fixed fabric cloning for deformable and particle materials.

## [107.3.15] - 2025-07-08

### Core

**General**

    - Changed

        - Using USDRT to populate tensor prims

    - Fixed

        - Fixed a regression in the GPU geometry code that could cause ghost contacts between a sphere and a triangle mesh.

## [107.3.14] - 2025-07-03

### Core

**General**

    - Fixed

        - Fixed crash when changing volume deformable body simulation mesh resolution after simulating  for the first time.
        - Fixed parsing of deformable body collision group, enabling group filtering on collision mesh directly.

**Python Bindings API**

    - Fixed

        - Fixed hasconflictingapis_DeformableBodyAPI, removed UsdPhysicsCollisionAPI as a conflict

**Fabric**

    - Changed

        - Mismatched prototypes on point instancer error lowered to warning

## [107.3.13] - 2025-06-25

### Core

**Fabric**

    - Fixed

        - Multiple GPU CUDA buffers access

## [107.3.12] - 2025-06-18

### Core

**General**

    - Fixed

        - When running with suppressed GPU data readback (setting "suppressReadback"), non-articulation joints were not able to break if the force exceeded the break threshold. The joints do break now but there is a potential performance penalty if the stage has breakable joints.
        - Revised DeformableSchemaChecker Asset Validator, supporting some more use-cases, particularly collision filtering for attachments.
        - Crash in CollisionGroups processing
        - Various validator rules fixed

**Fabric**

    - Fixed

        - Improved performance for initialization of rigid bodies when omni.physx.fabric is enabled

## [107.3.11] - 2025-06-12

### Core

**General**

    - Fixed

        - Setting pose for articulation and bodies is not executed on CPU when suppress readback is enabled
        - Deformables wrong initial transformation

## [107.3.10] - 2025-06-06

### Core

**General**

    - Changed

        - Refactored tensor API for new deformable schema and introduced support for surface deformables.

    - Fixed

        - Speedup of some omni.physx.tests tests.
        - On release clear both trigger and other shape.

## [107.3.9] - 2025-05-30

### Core

**General**

    - Fixed

        - Fixed collision scene graph instancing transformation
        - Fixed default simulator None crash with collision groups

## [107.3.8] - 2025-05-28

### Core

**General**

    - Fixed

        - Replication schemaAPI caching

**Fabric**

    - Fixed

        - Kinematics bodies switch fixed when omni.physx.fabric is used

**TensorAPI**

    - Fixed

        - Duplicate names for joints and links leads to undefined behavior

### UI

**General**

    - Added

        - Now emits warning message if having unsupported limit defined on an articulated joint.
        - Added articulated joint check to the asset validator to detect the pressence of unsupported attributes.

## [107.3.7] - 2025-05-22

### Core

**General**

    - Changed

        - Skip structural changes for prototypes

    - Fixed

        - Translation and rotation joint limit are now locked by default instead of set to out-of-range values.
        - Fixed multiple materials handling on non-triangle mesh approximations
        - Initialization for rigid body Fabric upadate can work with Fabric data

**TensorAPI**

    - Fixed

        - Fixed scale for kinematic bodies updated in TensorAPI

### UI

**General**

    - Changed

        - Collision visualization for a triangle mesh that belongs to a dynamic rigid body is visualized in dark red color, this indicates that this is not valid for PhysX and a fallback to convex hull will happen. Validation rule for this case was added to the asset validator.

## [107.3.6] - 2025-05-15

### Core

**General**

    - Added

        - Usd object ID validity check on release

    - Fixed

        - Rigid-deformables attachment points conversion

## [107.3.5] - 2025-05-14

### Core

**General**

    - Fixed

        - Fixed potential crashes in the context of failed particle sampling.
        - Joint Demo emitting error message.

## [107.3.4] - 2025-05-11

### Core

**General**

    - Fixed

        - Collider widget approximation property switching.
        - Added more API schemas to a geometry widget private list to have them not show in generic widgets.
        - Various deformable visualizer fixes and support for simulation and collision meshes using guide purpose.

## [107.3.3] - 2025-05-06

### Core

**General**

    - Fixed

        - Fixed crash in createObject when invalid materials were parsed

## [107.3.2] - 2025-05-02

### Core

**General**

    - Added

        - Physics umbrella - new extensions wrapping PhysX simulation, omni.physics, omni.physics.physx and omni.physics.stageupdate, this project does enable support for multiple physics integrations.

    - Changed

        - Muted error when velocity gets applied to non root articulaton links

**Fabric**

    - Changed

        - Change tracking processing is reading data from incoming change source rather than reading from fabric with precedence

    - Fixed

        - TensorAPI set kinematic bodies flushes transformation updates to Fabric to make transformations valid

### UI

**General**

    - Added

        - Asset validator rule for physics schema correctness added
        - Asset validator rule for joint pose validation



## [107.3.1] - 2025-04-28

### Core

**General**

    - Added

        - Add IPhysxJoint interface exposing joint state data
        - New deformable beta feature, that can be enabled in Physics Settings
        - Support for PhysxSplinesSurfaceVelocityAPI
        - New joint friction model and new drive model

    - Deprecated

        - The old joint friction model relying on friction coefficient

**USD Schemas**

    - Added

        - PhysxSchemaAddition codeless schema with PhysX specific APIs PhysxJointAxisAPI, PhysxDrivePerformanceEnvelopeAPI, PhysxSplinesSurfaceVelocityAPI, and APIs for deformable beta feature.
        - OmniUsdPhysicsDeformableSchema codeless schema with general deformable beta APIs

**Python Bindings API**

    - Added

        - Names of API and attributes for codeless schema: JOINT_AXIS_API and attributes and PERF_ENV_API and attributes

**TensorAPI**

    - Added

        - get_dof_friction_properties/set_dof_friction_properties and get_dof_drive_model_properties/set_dof_drive_model_properties

    - Fixed

        - Tensor API max joint velocity per axis

## [107.3.0] - 2025-04-10
Version bump.

## [107.2.2] - 2025-04-04

### Core

**General**

    - Added

        - Support for PhysxSplinesSurfaceVelocityAPI
        - Mass, convex approximation and joint bodies validation were added as omni.asset_validator checkers.

    - Changed

        - Backward compatibility validation was reimplemented as an omni.asset_validator checker.

    - Fixed

        - For non-articulation joints, it was not possible to set different type/maxForce/damping/stiffness attribute values on PhysicsDriveAPI::rotY and PhysicsDriveAPI::rotZ. The attribute values were always shared among the two axes.

## [107.2.1] - 2025-03-25
Version bump.

## [107.2.0] - 2025-03-21
Version bump.

## [107.0.18] - 2025-02-07
Version bump.

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

### Core

**General**

    - Fixed

        - Fix ConvexMeshData demo warnings and improve description
        - Fixed the Lego Technic Buggy demo
        - Fixed physx.tensors SDF warning when parsing invalid patterns

    - Removed

        - Removed the Museum demo
        - Removed the Franka Deformable demo
        - Removed the Paint Ball Emitter demo

## [106.4.5] - 2024-11-06

### Core

**General**

    - Fixed

        - Avoid Physics Inspector crash when accessing an invalid prim
        - Create fabric attributes optimization
        - Flush updates optimization
        - Processing EnvIds does not anymore reinsert objects from a scene

**TensorAPI**

    - Fixed

        - Fixed a crash in the rigid body contact view

## [106.4.4] - 2024-11-01

### Core

**General**

    - Changed

        - "Compatibility with Fabric Hierarchy and FSD" reverted.

    - Fixed

        - Demo tests filtering in non-local mode.

## [106.4.3] - 2024-10-30

### Core

**TensorAPI**

    - Fixed

        - A crash in rigid body contact reporting when using PhysX replication was fixed.

## [106.4.2] - 2024-10-28

### Core

**General**

    - Added

        - PhysX flushes certain changes from USD to Fabric before reading from Fabric and after writing to USD. PhysX is compatible with FSD.

**Fabric**

    - Added

        - omni.physx.fabric writes new RigidBody world space TRS attributes.

    - Changed

        - PhysX listens to local matrix and writes world and local matrices in Fabric. PhysX is compatible with FabricHierarchy.

### Vehicles

**General**

    - Fixed

        - Crash when PhysxVehicleAckermannSteeringAPI was applied and used after the simulation had started. Note that applying PhysxVehicleAckermannSteeringAPI after the simulation has started is not supported.

### UI

**General**

    - Added

        - New viewport overlay for visual joint authoring.
        - Several new features added to the Simulation Data Visualizer window.

## [106.4.1] - 2024-10-15
Version bump.

## [106.4.0] - 2024-10-07
Version bump.
