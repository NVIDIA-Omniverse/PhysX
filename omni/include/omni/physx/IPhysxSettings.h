// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once
#include <omni/physics/IUsdPhysicsSettings.h>

#define DEFINE_PHYSX_SETTING(name, path)                                                                               \
    static constexpr char name[] = PHYSICS_SETTINGS_PREFIX path;                                                         \
    static constexpr char name##Default[] = DEFAULT_SETTING_PREFIX PHYSICS_SETTINGS_PREFIX path;

#define DEFINE_PERSISTENT_PHYSX_SETTING(name, path)                                                                    \
    static constexpr char name[] = PERSISTENT_SETTINGS_PREFIX PHYSICS_SETTINGS_PREFIX path;                              \
    static constexpr char name##Default[] =                                                                            \
        DEFAULT_SETTING_PREFIX PERSISTENT_SETTINGS_PREFIX PHYSICS_SETTINGS_PREFIX path;


namespace omni
{

namespace physx
{

/// \defgroup private Private

/** \addtogroup Settings
 *  @{
 */

/////////////////////////
///////// Preferences
/////////////////////////

/** @rst
    (bool) See :ref:`Create Temporary Default PhysicsScene When Needed<Create Temporary Default PhysicsScene When Needed>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingAutocreatePhysicsScene, "/autocreatePhysicsScene");
/** @rst
    (bool) See :ref:`Reset simulation on stop<Reset simulation on stop>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingResetOnStop, "/resetOnStop");
/** @rst
    (bool) See :ref:`Use Active CUDA Context<Use Active CUDA Context>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingUseActiveCudaContext, "/useActiveCudaContext");
/// \ingroup private
constexpr bool kSettingUseActiveCudaContextDefaultVal = false;
/// (int) Device ordinal of a CUDA-enabled GPU for PhysX to use. -1 will autoselect a device.
DEFINE_PHYSX_SETTING(kSettingCudaDevice, "/cudaDevice");
/// \ingroup private
constexpr int kSettingCudaDeviceDefaultVal = -1;
/// (string) Simulation engine name.
DEFINE_PHYSX_SETTING(kSettingDefaultSimulator, "/defaultSimulator");
/** @rst
    (int) See :ref:`simulation_on_multiple_gpus` and :ref:`Use Physics Scene Multi-GPU Mode<Use Physics Scene Multi-GPU Mode>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingSceneMultiGPUMode, "/sceneMultiGPUMode");

// Simulator

/** @rst
    (int) See :ref:`Num Simulation Threads<Num Simulation Threads>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingNumThreads, "/numThreads");
/// (int) Stop simulation after this number of PhysX errors is reached.
DEFINE_PHYSX_SETTING(kSettingMaxNumberOfPhysXErrors, "/maxNumberOfPhysXErrors");
/** @rst
    (bool) See :ref:`Use PhysX CPU Dispatcher<Use PhysX CPU Dispatcher>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingPhysxDispatcher, "/physxDispatcher");
/** @rst
    (bool) See :ref:`Expose PhysX SDK Profiler Data<Expose PhysX SDK Profiler Data>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingExposeProfilerData, "/exposeProfilerData");
/// (bool) Expose the prim path names in PhysX SDK name, this will set the string name for the PhysX SDK objects.
DEFINE_PHYSX_SETTING(kSettingExposePrimPathNames, "/exposePrimPathNames");
/// @private
DEFINE_PHYSX_SETTING(kSettingForceParseOnlySingleScene, "/forceParseOnlySingleScene");
/// @private
DEFINE_PHYSX_SETTING(kSettingSimulateEmptyScene, "/simulateEmptyScene");
/// (bool) Disable sleeping for all PhysX SDK objects, useful for debugging, should not be used for production.
DEFINE_PHYSX_SETTING(kSettingDisableSleeping, "/disableSleeping");
/// (bool) Enable synchronous CUDA kernel launches. This is very useful if you need to pin point the CUDA kernel that is failing.
DEFINE_PHYSX_SETTING(kSettingSynchronousKernelLaunches, "/enableSynchronousKernelLaunches");
/// (bool) Disable contact processing in omni.physx.
DEFINE_PHYSX_SETTING(kSettingDisableContactProcessing, "/disableContactProcessing");

/** @rst
    (bool) See :ref:`Enable Local Mesh Cache<Enable Local Mesh Cache>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingUseLocalMeshCache, "/useLocalMeshCache");
/** @rst
    (int) See :ref:`Local Mesh Cache Size MB<Local Mesh Cache Size MB>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingLocalMeshCacheSizeMB, "/localMeshCacheSizeMB");

/// @private
DEFINE_PHYSX_SETTING(kSettingUjitsoCookingDevKey, "/cooking/ujitsoCookingDevKey");
/** @rst
    (bool) See :ref:`Enable UJITSO Collision Cooking<Enable UJITSO Collision Cooking>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingUjitsoCollisionCooking, "/cooking/ujitsoCollisionCooking");
/** @rst
    (bool) Controls if PhysX should attempt to use remote caching for UJITSO artifacts.  Additional settings must be used to specify the remote cache location.  This is a boot-time only setting.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingUjitsoRemoteCacheEnabled, "/cooking/ujitsoRemoteCacheEnabled");
/** @rst
    (int) See :ref:`UJITSO Cooking Max Process Count<UJITSO Cooking Max Process Count>`. Changeable through :ref:`Physics Preferences`.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingUjitsoCookingMaxProcessCount, "/cooking/ujitsoCookingMaxProcessCount");

/////////////////////////
///////// Stage Settings
/////////////////////////

// Update

/** @rst
    (bool) See :ref:`Update to USD<Update to USD>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingUpdateToUsd, "/updateToUsd");
/** @rst
    (int) See :ref:`Update velocities to USD<Update velocities to USD>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingUpdateVelocitiesToUsd, "/updateVelocitiesToUsd");
/** @rst
    (bool) See :ref:`Output Velocities in Local space<Output Velocities in Local space>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingOutputVelocitiesLocalSpace, "/outputVelocitiesLocalSpace");
/** @rst
    (bool) See :ref:`Update Particles to USD<Update Particles to USD>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingUpdateParticlesToUsd, "/updateParticlesToUsd");
/** @rst
    (bool) See :ref:`Update Residuals to USD<Update Residuals to USD>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingUpdateResidualsToUsd, "/updateResidualsToUsd");
DEFINE_PHYSX_SETTING(kSettingFabricEnabled, "/fabricEnabled");

// Simulator

/** @rst
    (int) See :ref:`Min Simulation Frame Rate<Min Simulation Frame Rate>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_EXT_SETTING(kSettingMinFrameRate, PERSISTENT_SETTINGS_PREFIX "/simulation/minFrameRate");

using omni::physics::kSettingJointBodyTransformCheckTolerance;
using omni::physics::kSettingJointBodyTransformCheckToleranceDefault;

/** @rst
    (bool) Support for joint angle computation in range (-360, 360) for revolute joints and D6 joints
    that have limits on a rotational degree of freedom. This allows, for example, to define
    revolute joint rotational limits in range (-360, 360). Affects non-articulation joints only.
    This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingEnableExtendedJointAngles, "/enableExtendedJointAngles");

// Collision
DEFINE_PHYSX_SETTING(kSettingCollisionApproximateCones, "/collisionApproximateCones");
DEFINE_PHYSX_SETTING(kSettingCollisionApproximateCylinders, "/collisionApproximateCylinders");

// Mouse interaction
/** @rst
    (bool) See :ref:`Mouse Interaction Enabled<Mouse Interaction Enabled>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingMouseInteractionEnabled, "/mouseInteractionEnabled");
/** @rst
    (bool) See :ref:`Mouse Grab<Mouse Grab>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingMouseGrab, "/mouseGrab");
/** @rst
    (bool) See :ref:`Mouse Grab Ignore Invisible<Mouse Grab Ignore Invisible>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingMouseGrabIgnoreInvisible, "/mouseGrabIgnoreInvisible");
/** @rst
    (bool) See :ref:`Mouse Grab With Force<Mouse Grab With Force>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingMouseGrabWithForce, "/forceGrab");
/** @rst
    (float) See :ref:`Mouse Push Acceleration<Mouse Push Acceleration>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingMousePush, "/mousePush");
/** @rst
    (float) See :ref:`Mouse Grab Force Coefficient<Mouse Grab Force Coefficient>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingMousePickingForce, "/pickingForce");


/////////////////////////
///////// Others
/////////////////////////

/// (bool) Turns on physics development mode that sets up a physics-oriented UI setup making e.g. physics demo or debug windows visible by default.
DEFINE_PHYSX_SETTING(kSettingPhysicsDevelopmentMode, "/developmentMode");
/// \ingroup private
DEFINE_PHYSX_SETTING(kSettingSuppressReadback, "/suppressReadback");
/// \ingroup private
DEFINE_PHYSX_SETTING(kSettingNumEventPumpsForTestStageSetup, "/numEventPumpsForTestStageSetup");
/// \ingroup private
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingOverrideGPU, "/overrideGPUSettings");
/// (bool) Enables a robotics-oriented logging channel. Can be switched in the Console window's filter settings.
DEFINE_PHYSX_SETTING(kSettingLogRobotics, "/logRobotics");
/** @rst
    (bool) Enables a :ref:`simulation_on_multiple_gpus` feature's logging channel. Can be switched in the Console window's filter settings.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingLogSceneMultiGPU, "/logSceneMultiGPU");
/// (string) Sets a path prefix to all external asset paths used by the physics demos. When empty it's set based on the value of /physics/demoDevelopmentMode to point to either S3 or nucleus path.
DEFINE_PHYSX_SETTING(kSettingDemoAssetsPath, "/demoAssetsPath");
/// (string) Sets a path prefix for all external test asset paths used by physics tests.
DEFINE_PHYSX_SETTING(kSettingTestsAssetsPath, "/testsAssetsPath");
/// (int) Maximal amount of selected prims that will be processed filtering items in the Add menu for relevance. No filtering will be done for selections larger than this limit.
DEFINE_PHYSX_SETTING(kSettingAddMenuSelectionLimit, "/addMenuSelectionLimit");
/// (int) Maximal amount of prims in a subtree of the selected prims to be processed when filtering items in the Add menu for relevance. No filtering will be done on subtrees over this limit.
DEFINE_PHYSX_SETTING(kSettingAddMenuSubtreeLimit, "/addMenuSubtreeLimit");

// Debug viz

/** @rst
    (bool) Enables viewport visualization of collision-mesh approximations using solid render meshes. This matches the Physics Debug Window setting Collision Mesh Debug Visualization
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingVisualizationCollisionMesh, "/visualizationCollisionMesh");

/** @rst
    (int) Enables viewport visualization of colliders.

    .. list-table:: Accepted values

        * - 0
          - Disabled
        * - 1
          - Selected only
        * - 2
          - All

    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayColliders, "/visualizationDisplayColliders");

/** @rst
    (bool) Displays normals for collider viewport visualization.
    @endrst */    
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayColliderNormals, "/visualizationDisplayColliderNormals");

using omni::physics::kSettingDisplayJoints;
using omni::physics::kSettingDisplayJointsDefault;

/** @rst
    (int) Enables display of icons in the viewport for rigid bodies that allow viewing mass property info.

    .. list-table:: Accepted values

        * - 0
          - Disabled
        * - 1
          - Selected only
        * - 2
          - All

    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayMassProperties, "/visualizationDisplayMassProperties");

/** @rst
    (bool) Toggles the display of the :ref:`Simulation Settings Window`.
    @endrst */    
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplaySimulationOutput, "/visualizationSimulationOutput");

/** @rst
    (bool) Toggles the display of the :ref:`Simulation Data Visualizer Window`.
    @endrst */    
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplaySimulationDataVisualizer, "/visualizationSimulationDataVisualizer");

/** @rst
    (bool) Automatically enable the :ref:`Simulation Settings Window` whenever Fabric is active
    @endrst */    
DEFINE_PHYSX_SETTING(kSettingAutoPopupSimulationOutputWindow, "/autoPopupSimulationOutputWindow");

/** @rst
    (int) Enables viewport visualization of tendons for joints.

    .. list-table:: Accepted values

        * - 0
          - Disabled
        * - 1
          - Selected only
        * - 2
          - All

    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayTendons, "/visualizationDisplayTendons");

// DEPRECATED, will be replaced by new deformable implementation in future release.
/** @rst
    (int) Enables viewport debug visualization overlay of deformable bodies (deprecated).

    .. list-table:: Accepted values

        * - 0
          - Disabled
        * - 1
          - Selected only
        * - 2
          - All

    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayDeformableBodies, "/visualizationDisplayDeformableBodies");

/** @rst
    (int) Sets the mode for the deformable body debug visualization (deprecated).

    .. list-table:: Accepted values

        * - 0
          - Simulation
        * - 1
          - Collision

    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayDeformableBodyType, "/visualizationDisplayDeformableBodyType");

/** @rst
    (int) Enables viewport debug visualization overlay of deformable surfaces (deprecated).

    .. list-table:: Accepted values

        * - 0
          - Disabled
        * - 1
          - Selected only
        * - 2
          - All

    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayDeformableSurfaces, "/visualizationDisplayDeformableSurfaces");
//~DEPRECATED

/** @rst
    (int) Enables viewport debug visualization overlay of deformables.

    .. list-table:: Accepted values

        * - 0
          - Disabled
        * - 1
          - Selected only
        * - 2
          - All

    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayDeformables, "/visualizationDisplayDeformables");

/** @rst
    (int) Configures which deformable mesh type is being visualized.

    .. list-table:: Accepted values

        * - 0
          - Simulation Default Pose
        * - 1
          - Simulation Bind Pose
        * - 2
          - Simulation Rest Shape
        * - 3
          - Collision Default Pose
        * - 4
          - Collision Bind Pose

    @endrst */

DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayDeformableMeshType, "/visualizationDisplayDeformableMeshType");

/** @rst
    (bool) Enables deformable attachment visualization.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayDeformableAttachments, "/visualizationDisplayDeformableAttachments");

/** @rst
    (bool) Toggles between new deformable feature (beta) and deprecated deformable/particle-cloth graphical user interfaces.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingEnableDeformableBeta, "/enableDeformableBeta");

/** @rst
    (int) Enables viewport debug visualization overlay of particles.

    .. list-table:: Accepted values

        * - 0
          - Disabled
        * - 1
          - Selected only
        * - 2
          - All

    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticles, "/visualizationDisplayParticles");

/** @rst
    (bool) Toggles the display of diffuse particles for the particle viewport debug visualization.
    @endrst */    
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticlesShowDiffuseParticles,
                                "/visualizationDisplayParticlesShowDiffuseParticles");

/** @rst
    (int) Sets the particle position mode for the particle viewport debug visualization. 

    .. list-table:: Accepted values

        * - 0
          - Show simulation particle positions
        * - 1
          - Show smoothing-post-processed particle positions

    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticlesParticlePositions,
                                "/visualizationDisplayParticlesParticlePositions");

/** @rst
    (int) Sets the particle radius mode for the particle viewport debug visualization. 

    .. list-table:: Accepted values

        * - 0
          - Particle-nonparticle contact offset
        * - 1
          - Particle-nonparticle rest offset
        * - 2
          - Particle-Particle contact offset
        * - 3
          - Fluid- or solid particle rest offset (applicable radius is auto-determined)
        * - 4
          - Fluid particles anisotropy
        * - 5
          - Render geometry of particle object

    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticlesParticleRadius, "/visualizationDisplayParticlesParticleRadius");


/// DEPRECATED, particle cloth will be replaced by new deformable implementation in future release.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticlesClothMeshLines, "/visualizationDisplayParticlesClothMeshLines");

/** @rst
    (bool) Toggles the display of particle sets for the particle viewport debug visualization.
    @endrst */    
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticlesShowParticleSetParticles,
                                "/visualizationDisplayParticlesShowParticleSetParticles");
//~DEPRECATED


/** @rst
    (bool) Toggles the display of fluid surfaces for the particle viewport debug visualization.
    @endrst */    
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticlesShowFluidSurface,
                                "/visualizationDisplayParticlesShowFluidSurface");
/// DEPRECATED, will be replaced by new deformable implementation in future release.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticlesShowDeformableParticles,
                                "/visualizationDisplayParticlesShowDeformableParticles");
/// DEPRECATED, will be replaced by new deformable implementation in future release.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticlesShowDeformableMesh,
                                "/visualizationDisplayParticlesShowDeformableMesh");

/// DEPRECATED
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayAttachments, "/visualizationDisplayAttachments");
/// DEPRECATED
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayAttachmentsHideActor0, "/visualizationDisplayAttachmentsHideActor0");
/// DEPRECATED
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayAttachmentsHideActor1, "/visualizationDisplayAttachmentsHideActor1");
//~DEPRECATED

/** @rst
    (float) Sets the relative spacing between tetrahedrals of the deformable body visualization.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingVisualizationGap, "/visualizationGap");

/** @rst
    (bool) Visualizes colliders as AABBs at the specified distance from the active camera.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDebugVisSimplifyAtDistance, "/visualizationSimplifyAtDistance");

/** @rst
    (bool) Use USDRT for stage traversal when displaying colliders. Improves performance if USDRT is available.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDebugVisQueryUsdrtForTraversal, "/visualizationQueryUsdrtForTraversal");

// Authoring
/** @rst
    (bool) Toggles the :ref:`Mass Distribution Manipulator<Mass Distribution Manipulator>`.
    @endrst */    
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingMassDistributionManipulator, "/massDistributionManipulator");
DEFINE_PHYSX_SETTING(kSettingEnableParticleAuthoring, "/enableParticleAuthoring");
DEFINE_PHYSX_SETTING(kSettingEnableAttachmentAuthoring, "/enableAttachmentAuthoring");

// Pvd
/** @rst
    (string) Sets the IP address of the consuming networked PVD client.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingPVDIPAddress, "/pvdIP");
/** @rst
    (bool) Toggles the output to a file or a network client.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingPVDStreamToFile, "/pvdStreamToFile");
/** @rst
    (string) Sets the output directory for the collected PVD files.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingPVDOutputDirectory, "/pvdOutputDirectory");
/** @rst
    (bool) Toggles the output of profiling data.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingPVDProfile, "/pvdProfile");
/** @rst
    (bool) Toggles the output of debug data.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingPVDDebug, "/pvdDebug");
/** @rst
    (bool) Toggles the output of memory data.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingPVDMemory, "/pvdMemory");
/** @rst
    (bool) Toggles collection of PVD telemetry. Only if this setting is true, do the other PVD settings matter.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingPVDEnabled, "/pvdEnabled");

// OmniPvd

/** @rst
    (string) Sets the directory for OmniPVD output files.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kOmniPvdOvdRecordingDirectory, "/omniPvdOvdRecordingDirectory");
/** @rst
    (bool) Toggles collection of OmniPVD telemetry. Only if this setting is true, do the other OmniPVD settings matter.
    @endrst */
DEFINE_PHYSX_SETTING(kOmniPvdOutputEnabled, "/omniPvdOutputEnabled");
/** @rst
    (bool) Toggles if the Stage is an OmniPVD stage or not. Only non-OmniPVD stages get recorded into OVD files.
    @endrst */
DEFINE_PHYSX_SETTING(kOmniPvdIsOVDStage, "/omniPvdIsOVDStage");
/** @rst
    (bool) Toggles if the Stage is being recorded into OmniPVD.
    @endrst */
DEFINE_PHYSX_SETTING(kOmniPvdIsRecording, "/omniPvdIsRecording");

// Tests runner
/// \ingroup private
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingTestRunnerFilter, "/testRunnerFilter");
/// \ingroup private
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingTestRunnerSelection, "/testRunnerSelection");
/// \ingroup private
DEFINE_PHYSX_SETTING(kSettingTestRunnerStatus, "/testRunnerStatus");
/// \ingroup private
DEFINE_PHYSX_SETTING(kSettingTestRunnerRepeats, "/testRunnerRepeats");

// Collision Groups
/// \ingroup private
DEFINE_PHYSX_SETTING(kSettingShowCollisionGroupsWindow, "/showCollisionGroupsWindow");

// Custom metadata attributes
/// \ingroup private
static constexpr char kLocalSpaceVelocitiesMetadataAttributeName[] = "physics:localSpaceVelocities";

/** @}*/

} // namespace physx
} // namespace omni
