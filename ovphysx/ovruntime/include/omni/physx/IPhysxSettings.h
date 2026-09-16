// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-OMNIPVD-TRANSPORT-001
 * @covers AC-1
 *
 * @implements REQ-OMNIPVD-LATE-001
 * @covers AC-1 AC-2
 */

#pragma once
#include <omni/physics/IUsdPhysicsSettings.h>

#include <cstddef>
#include <cstdint>
#include <string>

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
/// \ingroup private
/// (bool) Force process-wide CPU-only PhysX startup before any CUDA probing.
DEFINE_PHYSX_SETTING(kSettingForceCpuMode, "/forceCpuMode");
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
/// (bool) Emit the PhysX SDK profile zones as NVTX ranges, for capture with Nsight Systems.
/// Independent of kSettingExposeProfilerData: either, both, or neither sink can be active.
DEFINE_PHYSX_SETTING(kSettingNvtxEnabled, "/nvtxEnabled");
/// (bool) Expose the prim path names in PhysX SDK name, this will set the string name for the PhysX SDK objects.
DEFINE_PHYSX_SETTING(kSettingExposePrimPathNames, "/exposePrimPathNames");
/// @private
DEFINE_PHYSX_SETTING(kSettingForceParseOnlySingleScene, "/forceParseOnlySingleScene");
/// @private
DEFINE_PHYSX_SETTING(kSettingSimulateEmptyScene, "/simulateEmptyScene");
/// (bool) Enable synchronous CUDA kernel launches. This is very useful if you need to pin point the CUDA kernel that is failing.
DEFINE_PHYSX_SETTING(kSettingSynchronousKernelLaunches, "/enableSynchronousKernelLaunches");
/// (bool) Disable contact processing in omni.physx.
DEFINE_PHYSX_SETTING(kSettingDisableContactProcessing, "/disableContactProcessing");

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
    (bool) See :ref:`Update to USD using XformCommonAPI<Update to USD using XformCommonAPI>`. This is a :ref:`per-stage setting<Physics Settings>`.
    @endrst */
DEFINE_PHYSX_SETTING(kSettingUpdateToUsdUsingXformCommonAPI, "/updateToUsdUsingXformCommonAPI");
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
/// (int) Total memory, in mebibytes, that the DirectGPU ovstage-read buffer pool may RETAIN per CUDA
/// context for reuse across reads. A read allocates the columns it needs either way; the pool normally
/// hands them back on the next read instead of freeing them -- a large win for the columns read every
/// step -- but retaining them costs device memory. Once a context's pooled buffers reach this budget,
/// further released buffers are freed at release rather than kept, which is no worse than before the
/// pool existed. The default (256) comfortably holds the state working set of a large articulation
/// scene -- 8192 environments of 60 DOFs is tens of MB of joint/link/root columns, plus a mass matrix --
/// while a one-off multi-hundred-MB Jacobian falls outside it. Raise it for a scene that reads such
/// heavy inverse dynamics columns every step and has the memory; lower it to bound the footprint; 0 or a
/// negative value disables the pool (nothing is retained).
DEFINE_PHYSX_SETTING(kSettingOvstageReadPoolMaxMB, "/ovstageReadPoolMaxMB");
/// (bool) Assign replicator environment ids at body creation during stage attach/parse (GPU
/// dynamics + GPU broadphase scenes only): bodies get the scene-partition primvar id, or 0. Set
/// before attach by clone-driving consumers (ovphysx) so a later cloneEnvironments() finds the
/// source already collision-isolated from its copies — ids can only be assigned to objects
/// outside a scene, so assigning at creation avoids remove/re-add churn of live scene objects.
/// \ingroup private
DEFINE_PHYSX_SETTING(kSettingReplicatorEnvIdsOnAttach, "/replicatorEnvIdsOnAttach");
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


/** @rst
    (bool) Toggles the display of particle sets for the particle viewport debug visualization.
    @endrst */    
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticlesShowParticleSetParticles,
                                "/visualizationDisplayParticlesShowParticleSetParticles");

/** @rst
    (bool) Toggles the display of fluid surfaces for the particle viewport debug visualization.
    @endrst */    
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingDisplayParticlesShowFluidSurface,
                                "/visualizationDisplayParticlesShowFluidSurface");

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

enum class OmniPvdTransport : uint8_t
{
    eFile,
    eTcp,
};

struct OmniPvdDestination
{
    OmniPvdTransport transport{ OmniPvdTransport::eFile };
    std::string fileTarget;
    std::string tcpAddress;
    uint16_t tcpPort{ 0 };
    uint32_t tcpTimeoutMs{ 0 };
};

inline bool copyOmniPvdConfigString(const char* value, size_t length, std::string& result)
{
    if ((!value && length != 0) || (value && std::string(value, length).find('\0') != std::string::npos))
        return false;
    result.assign(value ? value : "", length);
    return true;
}

inline bool normalizeOmniPvdDestination(
    const char* transport,
    size_t transportLength,
    const char* fileTarget,
    size_t fileTargetLength,
    const char* tcpAddress,
    size_t tcpAddressLength,
    int64_t tcpPort,
    int64_t tcpTimeoutMs,
    OmniPvdDestination& destination,
    const char*& errorMessage)
{
    std::string transportValue;
    destination = {};
    errorMessage = nullptr;
    if (!copyOmniPvdConfigString(transport, transportLength, transportValue) || transportValue.empty())
    {
        errorMessage = "OmniPVD transport must be 'file' or 'tcp'";
        return false;
    }
    if (!copyOmniPvdConfigString(fileTarget, fileTargetLength, destination.fileTarget) ||
        !copyOmniPvdConfigString(tcpAddress, tcpAddressLength, destination.tcpAddress))
    {
        errorMessage = "OmniPVD destination strings must not contain embedded NUL bytes";
        return false;
    }
    if (transportValue == "file")
    {
        destination.transport = OmniPvdTransport::eFile;
        return true;
    }
    if (transportValue != "tcp")
    {
        errorMessage = "OmniPVD transport must be 'file' or 'tcp'";
        return false;
    }
    if (destination.tcpAddress.empty())
    {
        errorMessage = "OmniPVD TCP address must be non-empty";
        return false;
    }
    if (tcpPort < 1 || tcpPort > 65535)
    {
        errorMessage = "OmniPVD TCP port must be in 1..65535";
        return false;
    }
    if (tcpTimeoutMs < 0 || tcpTimeoutMs > INT32_MAX)
    {
        errorMessage = "OmniPVD TCP timeout must be in 0..INT32_MAX milliseconds";
        return false;
    }
    destination.transport = OmniPvdTransport::eTcp;
    destination.tcpPort = static_cast<uint16_t>(tcpPort);
    destination.tcpTimeoutMs = static_cast<uint32_t>(tcpTimeoutMs);
    return true;
}

/** @rst
    (string) Sets the directory for OmniPVD output files.
    @endrst */
DEFINE_PERSISTENT_PHYSX_SETTING(kOmniPvdOvdRecordingDirectory, "/omniPvdOvdRecordingDirectory");
/** OmniPVD startup destination transport: "file" or "tcp". */
DEFINE_PHYSX_SETTING(kOmniPvdTransport, "/omniPvdTransport");
/** OmniPVD TCP peer address. */
DEFINE_PHYSX_SETTING(kOmniPvdTcpAddress, "/omniPvdTcpAddress");
/** OmniPVD TCP peer port. */
DEFINE_PHYSX_SETTING(kOmniPvdTcpPort, "/omniPvdTcpPort");
/** OmniPVD TCP send timeout in milliseconds. */
DEFINE_PHYSX_SETTING(kOmniPvdTcpTimeoutMs, "/omniPvdTcpTimeoutMs");
/** @rst
    (bool) Toggles collection of OmniPVD telemetry. Only if this setting is true, do the other OmniPVD settings matter.
    @endrst */
DEFINE_PHYSX_SETTING(kOmniPvdOutputEnabled, "/omniPvdOutputEnabled");
/** (bool) Declares whether OmniPVD recording is available in this process. */
DEFINE_PHYSX_SETTING(kOmniPvdRecordingCapable, "/omniPvdRecordingCapable");
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
