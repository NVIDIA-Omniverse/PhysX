// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSceneQuery.h>
#include <omni/physx/IPhysxCooking.h>
#include <omni/physx/IPhysxVisualization.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxAttachmentPrivate.h>
#include <omni/physx/IPhysxPropertyQuery.h>
#include <private/omni/physx/ui/VisualizerMode.h>
#include <private/omni/physx/ui/ParticleVisualizationModes.h>
#include <private/omni/physx/PhysxUsd.h>
#include <physicsSchemaTools/physicsSchemaTokens.h>
#include <carb/BindingsPythonUtils.h>
#include <common/utilities/pyboost11.h>
#include "BindingsImpl.h"
#include "ResultBuffer.h"

#include <pybind11/stl_bind.h>

#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include <memory>
#include <string>
#include <vector>

//See comment for gMimicJointNaturalFrequencyAttributeName and remove accordingly.
#include <omni/physx/PhysxTokens.h>

CARB_BINDINGS("carb.physx.python")

DISABLE_PYBIND11_DYNAMIC_CAST(carb::events::IEventStream)
void PhysXReplicatorBindings(pybind11::module& m);
void PhysXCookingBindings(pybind11::module& m);
void PhysXCookingPrivateBindings(pybind11::module& m);
void PhysXStageUpdateBindings(pybind11::module& m);
void PhysXStatisticsBindings(pybind11::module& m);
void PhysXObjectChangedCallbackBindings(pybind11::class_<omni::physx::IPhysx>& dev);

using ContactEventHeaderVector = std::vector<omni::physx::ContactEventHeader>;
using ContactDataVector = std::vector<omni::physx::ContactData>;
using FrictionAnchorsDataVector = std::vector<omni::physx::FrictionAnchor>;
using PhysxPropertyQueryArticulationLinkVector = std::vector<omni::physx::PhysxPropertyQueryArticulationLink>;

PYBIND11_MAKE_OPAQUE(ContactEventHeaderVector);
PYBIND11_MAKE_OPAQUE(ContactDataVector);
PYBIND11_MAKE_OPAQUE(FrictionAnchorsDataVector);
PYBIND11_MAKE_OPAQUE(PhysxPropertyQueryArticulationLinkVector);

namespace pybind11
{
namespace detail
{
PYBOOST11_TYPE_CASTER(pxr::UsdPrim, _("UsdPrim"));
PYBOOST11_TYPE_CASTER(pxr::TfType, _("TfType"));
} // namespace detail
} // namespace pybind11

namespace
{

inline const pxr::SdfPath& intToPath(const uint64_t& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "Change to make the same size as pxr::SdfPath");

    return reinterpret_cast<const pxr::SdfPath&>(path);
}


using notify_contact_report_function = std::function<void(const ContactEventHeaderVector& contactHeaders,
    const ContactDataVector& contactData)>;
using PythonContactCallbackSubscriptionRegistry = omni::physx::EventSubscriptionRegistry<notify_contact_report_function>;

using notify_full_contact_report_function = std::function<void(const ContactEventHeaderVector& contactHeaders,
    const ContactDataVector& contactData, const FrictionAnchorsDataVector& frictionAnchorsData)>;
using PythonFullContactCallbackSubscriptionRegistry = omni::physx::EventSubscriptionRegistry<notify_full_contact_report_function>;

struct MarshalCallbacks
{
    omni::physx::SubscriptionId subscribeContactReportCallback(omni::physx::IPhysxSimulation* physxSim,
        const notify_contact_report_function& onContactReport);

    omni::physx::SubscriptionId subscribeContactReportFullCallback(omni::physx::IPhysxSimulation* physxSim,
        const notify_full_contact_report_function& onContactReport);

    void unsubscribeContactReportCallback(omni::physx::IPhysxSimulation* physxSim, omni::physx::SubscriptionId id)
    {
        mPythonContactCallbackRegistry.removeEvent(id);
        if (mPythonContactCallbackRegistry.map.empty())
        {
            physxSim->unsubscribePhysicsContactReportEvents(mContactEventSub);
            if (mBufferMutex)
            {
                delete mBufferMutex;
                mBufferMutex = nullptr;
            }
        }
    }

    void unsubscribeContactReportFullCallback(omni::physx::IPhysxSimulation* physxSim, omni::physx::SubscriptionId id)
    {
        mPythonContactCallbackFullRegistry.removeEvent(id);
        if (mPythonContactCallbackFullRegistry.map.empty())
        {
            physxSim->unsubscribePhysicsFullContactReportEvents(mContactEventSub);
            if (mBufferFullMutex)
            {
                delete mBufferFullMutex;
                mBufferFullMutex = nullptr;
            }
        }
    }

    void getFullContactReportTuple(omni::physx::IPhysxSimulation* physxSim,
        const ContactEventHeaderVector** contactEventHeaderVector, const ContactDataVector** contactDataVector, const FrictionAnchorsDataVector** frictionAnchorsDataVector)
    {
        const uint64_t timestamp = physxSim->getSimulationTimestamp();
        if (!mBufferFullMutex)
        {
            mBufferFullMutex = new carb::tasking::MutexWrapper();
        }
        if (mBufferFullMutex)
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(*mBufferFullMutex);
            if (timestamp != mCurrentTimestamp)
            {
                mContactHeadersReportBuffer.clear();
                mContactDataReportBuffer.clear();
                mFrictionAnchorsDataReportBuffer.clear();
                const omni::physx::ContactEventHeader* contactEventHeadersBuffer = nullptr;
                const omni::physx::ContactData* contactDataBuffer = nullptr;
                const omni::physx::FrictionAnchor* frictionAnchorDataBuffer = nullptr;
                uint32_t numContactData = 0;
                uint32_t numFrictionAnchorData = 0;

                const uint32_t numContactHeaders = physxSim->getFullContactReport(&contactEventHeadersBuffer, &contactDataBuffer, numContactData,
                    &frictionAnchorDataBuffer, numFrictionAnchorData);

                if (numContactHeaders)
                {
                    mContactHeadersReportBuffer.resize(size_t(numContactHeaders));
                    memcpy(mContactHeadersReportBuffer.data(), contactEventHeadersBuffer,
                        numContactHeaders * sizeof(omni::physx::ContactEventHeader));
                }

                if (numContactData)
                {
                    mContactDataReportBuffer.resize(size_t(numContactData));
                    memcpy(mContactDataReportBuffer.data(), contactDataBuffer,
                        numContactData * sizeof(omni::physx::ContactData));
                }

                if (numFrictionAnchorData)
                {
                    mFrictionAnchorsDataReportBuffer.resize(size_t(numFrictionAnchorData));
                    memcpy(mFrictionAnchorsDataReportBuffer.data(), frictionAnchorDataBuffer,
                        numFrictionAnchorData * sizeof(omni::physx::FrictionAnchor));
                }

                mCurrentTimestamp = timestamp;
            }
        }

        *contactEventHeaderVector = &mContactHeadersReportBuffer;
        *contactDataVector = &mContactDataReportBuffer;
        *frictionAnchorsDataVector = &mFrictionAnchorsDataReportBuffer;
    }

    void getContactReportTuple(omni::physx::IPhysxSimulation* physxSim,
        const ContactEventHeaderVector** contactEventHeaderVector, const ContactDataVector** contactDataVector)
    {
        const uint64_t timestamp = physxSim->getSimulationTimestamp();
        if (!mBufferMutex)
        {
            mBufferMutex = new carb::tasking::MutexWrapper();
        }
        if (mBufferMutex)
        {
            std::lock_guard<carb::tasking::MutexWrapper> lock(*mBufferMutex);
            if (timestamp != mCurrentTimestamp)
            {
                mContactHeadersReportBuffer.clear();
                mContactDataReportBuffer.clear();
                mFrictionAnchorsDataReportBuffer.clear();
                const omni::physx::ContactEventHeader* contactEventHeadersBuffer = nullptr;
                const omni::physx::ContactData* contactDataBuffer = nullptr;
                const omni::physx::FrictionAnchor* frictionAnchorDataBuffer = nullptr;
                uint32_t numContactData = 0;
                uint32_t numFrictionAnchorData = 0;

                const uint32_t numContactHeaders = physxSim->getContactReport(&contactEventHeadersBuffer, &contactDataBuffer, numContactData);

                if (numContactHeaders)
                {
                    mContactHeadersReportBuffer.resize(size_t(numContactHeaders));
                    memcpy(mContactHeadersReportBuffer.data(), contactEventHeadersBuffer,
                        numContactHeaders * sizeof(omni::physx::ContactEventHeader));
                }

                if (numContactData)
                {
                    mContactDataReportBuffer.resize(size_t(numContactData));
                    memcpy(mContactDataReportBuffer.data(), contactDataBuffer,
                        numContactData * sizeof(omni::physx::ContactData));
                }

                mCurrentTimestamp = timestamp;
            }
        }

        *contactEventHeaderVector = &mContactHeadersReportBuffer;
        *contactDataVector = &mContactDataReportBuffer;
    }

public:
    omni::physx::SubscriptionId mContactEventSub;
    PythonContactCallbackSubscriptionRegistry mPythonContactCallbackRegistry;

    omni::physx::SubscriptionId mContactEventFullSub;
    PythonFullContactCallbackSubscriptionRegistry mPythonContactCallbackFullRegistry;

    ContactEventHeaderVector    mContactHeaders;
    ContactDataVector           mContactData;
    FrictionAnchorsDataVector   mFrictionAnchorsData;

    ContactEventHeaderVector    mContactHeadersReportBuffer;
    ContactDataVector           mContactDataReportBuffer;
    FrictionAnchorsDataVector   mFrictionAnchorsDataReportBuffer;
    uint64_t                    mCurrentTimestamp {0};
    carb::tasking::MutexWrapper* mBufferMutex{ nullptr };
    carb::tasking::MutexWrapper* mBufferFullMutex{ nullptr };
};

static MarshalCallbacks gMarshalCallbacks;

void OnPythonContactReportEventFn(const omni::physx::ContactEventHeader* contactHeaders, uint32_t numHeaders,
    const omni::physx::ContactData* contactData, uint32_t numContactData, void* userData)
{
    if (contactHeaders && numHeaders && contactData)
    {
        MarshalCallbacks* contactCallback = reinterpret_cast<MarshalCallbacks*> (userData);
        contactCallback->mContactHeaders.clear();

        contactCallback->mContactHeaders.resize(size_t(numHeaders));
        memcpy(contactCallback->mContactHeaders.data(), contactHeaders, numHeaders * sizeof(omni::physx::ContactEventHeader));

        if (numContactData)
        {
            contactCallback->mContactData.resize(size_t(numContactData));
            memcpy(contactCallback->mContactData.data(), contactData, numContactData * sizeof(omni::physx::ContactData));
        }
        else
        {
            contactCallback->mContactData.clear();
        }

        try
        {
            for (PythonContactCallbackSubscriptionRegistry::EventMap::const_reference ref : contactCallback->mPythonContactCallbackRegistry.map)
            {
                ref.second(contactCallback->mContactHeaders, contactCallback->mContactData);
            }
        }
        catch (const std::runtime_error& e)
        {
            CARB_LOG_ERROR("Failed to marshal contact report callback to python. Error: %s", e.what());
        }
    }
}

omni::physx::SubscriptionId MarshalCallbacks::subscribeContactReportCallback(omni::physx::IPhysxSimulation* physxSim,
                                                           const notify_contact_report_function& onContactReport)
{
    if (mPythonContactCallbackRegistry.map.empty())
    {
        mContactEventSub = physxSim->subscribePhysicsContactReportEvents(
            OnPythonContactReportEventFn, (void*)&gMarshalCallbacks);
    }
    if (!mBufferMutex)
    {
        mBufferMutex = new carb::tasking::MutexWrapper();
    }
    return mPythonContactCallbackRegistry.addEvent(onContactReport);
}

void OnPythonFullContactReportEventFn(const omni::physx::ContactEventHeader* contactHeaders, uint32_t numHeaders,
    const omni::physx::ContactData* contactData, uint32_t numContactData,
    const omni::physx::FrictionAnchor* frictionAnchors, uint32_t numFrictionAnchors, void* userData)
{
    if (contactHeaders && numHeaders && contactData)
    {
        MarshalCallbacks* contactCallback = reinterpret_cast<MarshalCallbacks*> (userData);
        contactCallback->mContactHeaders.clear();

        contactCallback->mContactHeaders.resize(size_t(numHeaders));
        memcpy(contactCallback->mContactHeaders.data(), contactHeaders, numHeaders * sizeof(omni::physx::ContactEventHeader));

        if (numContactData)
        {
            contactCallback->mContactData.resize(size_t(numContactData));
            memcpy(contactCallback->mContactData.data(), contactData, numContactData * sizeof(omni::physx::ContactData));
        }
        else
        {
            contactCallback->mContactData.clear();
        }

        if (numFrictionAnchors && frictionAnchors)
        {
            contactCallback->mFrictionAnchorsData.resize(size_t(numFrictionAnchors));
            memcpy(contactCallback->mFrictionAnchorsData.data(), frictionAnchors, numFrictionAnchors * sizeof(omni::physx::FrictionAnchor));
        }
        else
        {
            contactCallback->mFrictionAnchorsData.clear();
        }

        try
        {
            for (PythonFullContactCallbackSubscriptionRegistry::EventMap::const_reference ref : contactCallback->mPythonContactCallbackFullRegistry.map)
            {
                ref.second(contactCallback->mContactHeaders, contactCallback->mContactData, contactCallback->mFrictionAnchorsData);
            }
        }
        catch (const std::runtime_error& e)
        {
            CARB_LOG_ERROR("Failed to marshal contact report callback to python. Error: %s", e.what());
        }
    }
}

omni::physx::SubscriptionId MarshalCallbacks::subscribeContactReportFullCallback(omni::physx::IPhysxSimulation* physxSim,
                                                           const notify_full_contact_report_function& onContactReport)
{
    if (mPythonContactCallbackFullRegistry.map.empty())
    {
        mContactEventSub = physxSim->subscribePhysicsFullContactReportEvents(
            OnPythonFullContactReportEventFn, (void*)&gMarshalCallbacks);
    }
    if (!mBufferMutex)
    {
        mBufferMutex = new carb::tasking::MutexWrapper();
    }
    return mPythonContactCallbackFullRegistry.addEvent(onContactReport);
}

#define ADD_SETTING(attr_name, path) \
    m.attr(attr_name) = py::str(path); \
    m.attr(attr_name "_DEFAULT") = py::str(path##Default); \
    docModule += " - " + std::string(attr_name) + " = \"" + std::string(path) + "\"\n";

PYBIND11_MODULE(_physx, m)
{
    using namespace carb;
    using namespace omni::physx;

    const char* docString;
    std::string docModule = R"(
    This module contains python bindings to the C++ omni::physx interface.

    omni::physx contains several interfaces:

        * PhysX -- Main interface used for physics simulation.
        * PhysXVisualization -- Interface for debug visualization control.
        * PhysXUnitTests -- Interface for unit tests.

    )";

    docModule += "\nSettings attributes available in this module:\n";

    // Preferences
    ADD_SETTING("SETTING_AUTOCREATE_PHYSICS_SCENE", kSettingAutocreatePhysicsScene);
    ADD_SETTING("SETTING_RESET_ON_STOP", kSettingResetOnStop);
    ADD_SETTING("SETTING_USE_ACTIVE_CUDA_CONTEXT", kSettingUseActiveCudaContext);
    ADD_SETTING("SETTING_CUDA_DEVICE", kSettingCudaDevice);
    ADD_SETTING("SETTING_DEFAULT_SIMULATOR", kSettingDefaultSimulator);
    ADD_SETTING("SETTING_PHYSICS_SCENE_MULTIGPU_MODE", kSettingSceneMultiGPUMode);

    ADD_SETTING("SETTING_NUM_THREADS", kSettingNumThreads);
    ADD_SETTING("SETTING_MAX_NUMBER_OF_PHYSX_ERRORS", kSettingMaxNumberOfPhysXErrors);
    ADD_SETTING("SETTING_PHYSX_DISPATCHER", kSettingPhysxDispatcher);
    ADD_SETTING("SETTING_EXPOSE_PROFILER_DATA", kSettingExposeProfilerData);
    ADD_SETTING("SETTING_EXPOSE_PRIM_PATH_NAMES", kSettingExposePrimPathNames);
    ADD_SETTING("SETTING_FORCE_PARSE_ONLY_SINGLE_SCENE", kSettingForceParseOnlySingleScene);
    ADD_SETTING("SETTING_SIMULATE_EMPTY_SCENE", kSettingSimulateEmptyScene);
    ADD_SETTING("SETTING_DISABLE_SLEEPING", kSettingDisableSleeping);
    ADD_SETTING("SETTING_ENABLE_SYNCHRONOUS_KERNEL_LAUNCHES", kSettingSynchronousKernelLaunches);
    ADD_SETTING("SETTING_DISABLE_CONTACT_PROCESSING", kSettingDisableContactProcessing);

    ADD_SETTING("SETTING_USE_LOCAL_MESH_CACHE", kSettingUseLocalMeshCache);
    ADD_SETTING("SETTING_LOCAL_MESH_CACHE_SIZE_MB", kSettingLocalMeshCacheSizeMB);

    ADD_SETTING("SETTING_UJITSO_COOKING_DEV_KEY", kSettingUjitsoCookingDevKey);
    ADD_SETTING("SETTING_UJITSO_COLLISION_COOKING", kSettingUjitsoCollisionCooking);
    ADD_SETTING("SETTING_UJITSO_REMOTE_CACHE_ENABLED", kSettingUjitsoRemoteCacheEnabled);
    ADD_SETTING("SETTING_UJITSO_COOKING_MAX_PROCESS_COUNT", kSettingUjitsoCookingMaxProcessCount);

    // Stage settings
    ADD_SETTING("SETTING_UPDATE_TO_USD", kSettingUpdateToUsd);
    ADD_SETTING("SETTING_UPDATE_VELOCITIES_TO_USD", kSettingUpdateVelocitiesToUsd);
    ADD_SETTING("SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE", kSettingOutputVelocitiesLocalSpace);
    ADD_SETTING("SETTING_UPDATE_PARTICLES_TO_USD", kSettingUpdateParticlesToUsd);
    ADD_SETTING("SETTING_UPDATE_RESIDUALS_TO_USD", kSettingUpdateResidualsToUsd);

    ADD_SETTING("SETTING_MIN_FRAME_RATE", kSettingMinFrameRate);
    ADD_SETTING("SETTING_JOINT_BODY_TRANSFORM_CHECK_TOLERANCE", kSettingJointBodyTransformCheckTolerance);
    ADD_SETTING("SETTING_ENABLE_EXTENDED_JOINT_ANGLES", kSettingEnableExtendedJointAngles);

    ADD_SETTING("SETTING_COLLISION_APPROXIMATE_CONES", kSettingCollisionApproximateCones);
    ADD_SETTING("SETTING_COLLISION_APPROXIMATE_CYLINDERS", kSettingCollisionApproximateCylinders);

    ADD_SETTING("SETTING_MOUSE_INTERACTION_ENABLED", kSettingMouseInteractionEnabled);
    ADD_SETTING("SETTING_MOUSE_GRAB", kSettingMouseGrab);
    ADD_SETTING("SETTING_MOUSE_GRAB_IGNORE_INVISBLE", kSettingMouseGrabIgnoreInvisible);
    ADD_SETTING("SETTING_MOUSE_GRAB_WITH_FORCE", kSettingMouseGrabWithForce);
    ADD_SETTING("SETTING_MOUSE_PUSH", kSettingMousePush);
    ADD_SETTING("SETTING_MOUSE_PICKING_FORCE", kSettingMousePickingForce);

    // Others
    ADD_SETTING("SETTING_PHYSICS_DEVELOPMENT_MODE", kSettingPhysicsDevelopmentMode);
    ADD_SETTING("SETTING_SUPPRESS_READBACK", kSettingSuppressReadback);
    ADD_SETTING("SETTING_NUM_EVENT_PUMPS_FOR_TEST_STAGE_SETUP", kSettingNumEventPumpsForTestStageSetup);
    ADD_SETTING("SETTING_OVERRIDE_GPU", kSettingOverrideGPU);
    ADD_SETTING("SETTING_LOG_ROBOTICS", kSettingLogRobotics);
    ADD_SETTING("SETTING_LOG_SCENEMULTIGPU", kSettingLogSceneMultiGPU);
    ADD_SETTING("SETTING_DEMO_ASSETS_PATH", kSettingDemoAssetsPath);
    ADD_SETTING("SETTING_TESTS_ASSETS_PATH", kSettingTestsAssetsPath);
    ADD_SETTING("SETTING_ADDMENU_SELECTION_LIMIT", kSettingAddMenuSelectionLimit);
    ADD_SETTING("SETTING_ADDMENU_SUBTREE_LIMIT", kSettingAddMenuSubtreeLimit);

    ADD_SETTING("SETTING_VISUALIZATION_COLLISION_MESH", kSettingVisualizationCollisionMesh);
    ADD_SETTING("SETTING_DISPLAY_COLLIDERS", kSettingDisplayColliders);
    ADD_SETTING("SETTING_DISPLAY_COLLIDER_NORMALS", kSettingDisplayColliderNormals);
    ADD_SETTING("SETTING_DISPLAY_MASS_PROPERTIES", kSettingDisplayMassProperties);
    ADD_SETTING("SETTING_DISPLAY_JOINTS", kSettingDisplayJoints);
    ADD_SETTING("SETTING_DISPLAY_SIMULATION_OUTPUT", kSettingDisplaySimulationOutput);
    ADD_SETTING("SETTING_AUTO_POPUP_SIMULATION_OUTPUT_WINDOW", kSettingAutoPopupSimulationOutputWindow);

    ADD_SETTING("SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER", kSettingDisplaySimulationDataVisualizer);

    ADD_SETTING("SETTING_DISPLAY_TENDONS", kSettingDisplayTendons);

    // DEPRECATED
    ADD_SETTING("SETTING_DISPLAY_DEFORMABLE_BODIES", kSettingDisplayDeformableBodies);
    ADD_SETTING("SETTING_DISPLAY_DEFORMABLE_BODY_TYPE", kSettingDisplayDeformableBodyType);
    ADD_SETTING("SETTING_DISPLAY_DEFORMABLE_SURFACES", kSettingDisplayDeformableSurfaces);
    ADD_SETTING("SETTING_DISPLAY_ATTACHMENTS", kSettingDisplayAttachments);
    ADD_SETTING("SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_0", kSettingDisplayAttachmentsHideActor0);
    ADD_SETTING("SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_1", kSettingDisplayAttachmentsHideActor1);
    //~DEPRECATED

    ADD_SETTING("SETTING_DISPLAY_DEFORMABLES", kSettingDisplayDeformables);
    ADD_SETTING("SETTING_DISPLAY_DEFORMABLE_MESH_TYPE", kSettingDisplayDeformableMeshType);
    ADD_SETTING("SETTING_DISPLAY_DEFORMABLE_ATTACHMENTS", kSettingDisplayDeformableAttachments);

    ADD_SETTING("SETTING_ENABLE_DEFORMABLE_BETA", kSettingEnableDeformableBeta);

    ADD_SETTING("SETTING_DISPLAY_PARTICLES", kSettingDisplayParticles);
    ADD_SETTING("SETTING_VISUALIZATION_GAP", kSettingVisualizationGap);
    ADD_SETTING("SETTING_DEBUG_VIS_SIMPLIFY_AT_DISTANCE", kSettingDebugVisSimplifyAtDistance);
    ADD_SETTING("SETTING_DEBUG_VIS_QUERY_USDRT_FOR_TRAVERSAL", kSettingDebugVisQueryUsdrtForTraversal);

    // Authoring
    ADD_SETTING("SETTING_MASS_DISTRIBUTION_MANIPULATOR", kSettingMassDistributionManipulator);
    ADD_SETTING("SETTING_ENABLE_PARTICLE_AUTHORING", kSettingEnableParticleAuthoring);
    ADD_SETTING("SETTING_ENABLE_ATTACHMENT_AUTHORING", kSettingEnableAttachmentAuthoring);

    // particle visualization options:
    ADD_SETTING("SETTING_DISPLAY_PARTICLES_SHOW_DIFFUSE", kSettingDisplayParticlesShowDiffuseParticles);
    ADD_SETTING("SETTING_DISPLAY_PARTICLES_POSITION_TYPE", kSettingDisplayParticlesParticlePositions);
    ADD_SETTING("SETTING_DISPLAY_PARTICLES_RADIUS_TYPE", kSettingDisplayParticlesParticleRadius);
    ADD_SETTING("SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES", kSettingDisplayParticlesShowParticleSetParticles);
    ADD_SETTING("SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE", kSettingDisplayParticlesShowFluidSurface);

    // DEPRECATED
    ADD_SETTING("SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH_LINES", kSettingDisplayParticlesClothMeshLines);
    ADD_SETTING("SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES", kSettingDisplayParticlesShowDeformableParticles);
    ADD_SETTING("SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH", kSettingDisplayParticlesShowDeformableMesh);
    //~DEPRECATED

    ADD_SETTING("SETTING_PVD_IP_ADDRESS", kSettingPVDIPAddress);
    ADD_SETTING("SETTING_PVD_STREAM_TO_FILE", kSettingPVDStreamToFile);
    ADD_SETTING("SETTING_PVD_OUTPUT_DIRECTORY", kSettingPVDOutputDirectory);
    ADD_SETTING("SETTING_PVD_PROFILE", kSettingPVDProfile);
    ADD_SETTING("SETTING_PVD_DEBUG", kSettingPVDDebug);
    ADD_SETTING("SETTING_PVD_MEMORY", kSettingPVDMemory);
    ADD_SETTING("SETTING_PVD_ENABLED", kSettingPVDEnabled);

    ADD_SETTING("SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY", kOmniPvdOvdRecordingDirectory);
    ADD_SETTING("SETTING_OMNIPVD_ENABLED", kOmniPvdOutputEnabled);
    ADD_SETTING("SETTING_OMNIPVD_IS_OVD_STAGE", kOmniPvdIsOVDStage);
    ADD_SETTING("SETTING_OMNIPVD_IS_RECORDING", kOmniPvdIsRecording);

    ADD_SETTING("SETTING_TEST_RUNNER_FILTER", kSettingTestRunnerFilter);
    ADD_SETTING("SETTING_TEST_RUNNER_SELECTION", kSettingTestRunnerSelection);
    ADD_SETTING("SETTING_TEST_RUNNER_STATUS", kSettingTestRunnerStatus);
    ADD_SETTING("SETTING_TEST_RUNNER_REPEATS", kSettingTestRunnerRepeats);

    ADD_SETTING("SETTING_SHOW_COLLISION_GROUPS_WINDOW", kSettingShowCollisionGroupsWindow);

    docModule += "\n";

    // Custom attributes
    m.attr("METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES") = py::str(kLocalSpaceVelocitiesMetadataAttributeName);

    m.attr("MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTX") = py::str(gMimicJointNaturalFrequencyAttributeName[0]);
    m.attr("MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTY") = py::str(gMimicJointNaturalFrequencyAttributeName[1]);
    m.attr("MIMIC_JOINT_ATTRIBUTE_NAME_NATURAL_FREQUENCY_ROTZ") = py::str(gMimicJointNaturalFrequencyAttributeName[2]);

    m.attr("MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTX") = py::str(gMimicJointDampingRatioAttributeName[0]);
    m.attr("MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTY") = py::str(gMimicJointDampingRatioAttributeName[1]);
    m.attr("MIMIC_JOINT_ATTRIBUTE_NAME_DAMPING_RATIO_ROTZ") = py::str(gMimicJointDampingRatioAttributeName[2]);

    // codeless schema API and attributes

    m.attr("PERF_ENV_API") = py::str(pxr::PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI);
    m.attr("JOINT_AXIS_API") = py::str(pxr::PhysxAdditionAPITokens->PhysxJointAxisAPI);

    m.attr("PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ANGULAR") = py::str(pxr::PhysxAdditionAttrTokens->maxActuatorVelocityAngular);
    m.attr("PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_LINEAR") = py::str(pxr::PhysxAdditionAttrTokens->maxActuatorVelocityLinear);
    m.attr("PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ROTX") = py::str(pxr::PhysxAdditionAttrTokens->maxActuatorVelocityRotX);
    m.attr("PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ROTY") = py::str(pxr::PhysxAdditionAttrTokens->maxActuatorVelocityRotY);
    m.attr("PERF_ENV_ATTR_MAX_ACTUATOR_VELOCITY_ROTZ") = py::str(pxr::PhysxAdditionAttrTokens->maxActuatorVelocityRotZ);

    m.attr("PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ANGULAR") = py::str(pxr::PhysxAdditionAttrTokens->velocityDependentResistanceAngular);
    m.attr("PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_LINEAR") = py::str(pxr::PhysxAdditionAttrTokens->velocityDependentResistanceLinear);
    m.attr("PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ROTX") = py::str(pxr::PhysxAdditionAttrTokens->velocityDependentResistanceRotX);
    m.attr("PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ROTY") = py::str(pxr::PhysxAdditionAttrTokens->velocityDependentResistanceRotY);
    m.attr("PERF_ENV_ATTR_VELOCITY_DEPENDENT_RESISTANCE_ROTZ") = py::str(pxr::PhysxAdditionAttrTokens->velocityDependentResistanceRotZ);

    m.attr("PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ANGULAR") = py::str(pxr::PhysxAdditionAttrTokens->speedEffortGradientAngular);
    m.attr("PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_LINEAR") = py::str(pxr::PhysxAdditionAttrTokens->speedEffortGradientLinear);
    m.attr("PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ROTX") = py::str(pxr::PhysxAdditionAttrTokens->speedEffortGradientRotX);
    m.attr("PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ROTY") = py::str(pxr::PhysxAdditionAttrTokens->speedEffortGradientRotY);
    m.attr("PERF_ENV_ATTR_SPEED_EFFORT_GRADIENT_ROTZ") = py::str(pxr::PhysxAdditionAttrTokens->speedEffortGradientRotZ);

    m.attr("JOINT_AXIS_ATTR_ARMATURE_ANGULAR") = py::str(pxr::PhysxAdditionAttrTokens->armatureAngular);
    m.attr("JOINT_AXIS_ATTR_ARMATURE_LINEAR") = py::str(pxr::PhysxAdditionAttrTokens->armatureLinear);
    m.attr("JOINT_AXIS_ATTR_ARMATURE_ROTX") = py::str(pxr::PhysxAdditionAttrTokens->armatureRotX);
    m.attr("JOINT_AXIS_ATTR_ARMATURE_ROTY") = py::str(pxr::PhysxAdditionAttrTokens->armatureRotY);
    m.attr("JOINT_AXIS_ATTR_ARMATURE_ROTZ") = py::str(pxr::PhysxAdditionAttrTokens->armatureRotZ);
    
    m.attr("JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ANGULAR") = py::str(pxr::PhysxAdditionAttrTokens->maxJointVelocityAngular);
    m.attr("JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_LINEAR") = py::str(pxr::PhysxAdditionAttrTokens->maxJointVelocityLinear);
    m.attr("JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ROTX") = py::str(pxr::PhysxAdditionAttrTokens->maxJointVelocityRotX);
    m.attr("JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ROTY") = py::str(pxr::PhysxAdditionAttrTokens->maxJointVelocityRotY);
    m.attr("JOINT_AXIS_ATTR_MAX_JOINT_VELOCITY_ROTZ") = py::str(pxr::PhysxAdditionAttrTokens->maxJointVelocityRotZ);
    
    m.attr("JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ANGULAR") = py::str(pxr::PhysxAdditionAttrTokens->staticFrictionEffortAngular);
    m.attr("JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_LINEAR") = py::str(pxr::PhysxAdditionAttrTokens->staticFrictionEffortLinear);
    m.attr("JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ROTX") = py::str(pxr::PhysxAdditionAttrTokens->staticFrictionEffortRotX);
    m.attr("JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ROTY") = py::str(pxr::PhysxAdditionAttrTokens->staticFrictionEffortRotY);
    m.attr("JOINT_AXIS_ATTR_STATIC_FRICTION_EFFORT_ROTZ") = py::str(pxr::PhysxAdditionAttrTokens->staticFrictionEffortRotZ);
    
    m.attr("JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ANGULAR") = py::str(pxr::PhysxAdditionAttrTokens->dynamicFrictionEffortAngular);
    m.attr("JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_LINEAR") = py::str(pxr::PhysxAdditionAttrTokens->dynamicFrictionEffortLinear);
    m.attr("JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ROTX") = py::str(pxr::PhysxAdditionAttrTokens->dynamicFrictionEffortRotX);
    m.attr("JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ROTY") = py::str(pxr::PhysxAdditionAttrTokens->dynamicFrictionEffortRotY);
    m.attr("JOINT_AXIS_ATTR_DYNAMIC_FRICTION_EFFORT_ROTZ") = py::str(pxr::PhysxAdditionAttrTokens->dynamicFrictionEffortRotZ);
    
    m.attr("JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ANGULAR") = py::str(pxr::PhysxAdditionAttrTokens->viscousFrictionCoefficientAngular);
    m.attr("JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_LINEAR") = py::str(pxr::PhysxAdditionAttrTokens->viscousFrictionCoefficientLinear);
    m.attr("JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ROTX") = py::str(pxr::PhysxAdditionAttrTokens->viscousFrictionCoefficientRotX);
    m.attr("JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ROTY") = py::str(pxr::PhysxAdditionAttrTokens->viscousFrictionCoefficientRotY);
    m.attr("JOINT_AXIS_ATTR_VISCOUS_FRICTION_COEFFICIENT_ROTZ") = py::str(pxr::PhysxAdditionAttrTokens->viscousFrictionCoefficientRotZ);
    
    
  
    py::enum_<SimulationEvent>(m, "SimulationEvent", R"(
        Simulation events used by simulation event stream.
        )")
        .value("RESUMED", eResumed, "Simulation resumed, no additional data are send in the event")
        .value("PAUSED", ePaused, "Simulation paused, no additional data are send in the event")
        .value("STOPPED", eStopped, "Simulation stopped, no additional data are send in the event")
        .value("CONTACT_FOUND", eContactFound,
            R"(Contact found event header: sends header information regarding which colliders started to collide; contains the following in a dictionary:

            .. code-block:: text

                'actor0':int2 - Usd path to rigid body actor 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'actor1':int2 - Usd path to rigid body actor 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'collider0':int2 - Usd path to collider 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'collider1':int2 - Usd path to collider 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'numContactData':int - Num contact data sent after the header is sent.
                'stageId':long1 - Current USD stage id, long array with one item.
            )")
        .value("CONTACT_LOST", eContactLost,
            R"(Contact lost event header: sends header information regarding which colliders lost contact; contains the following in a dictionary:

            .. code-block:: text

                'actor0':int2 - Usd path to rigid body actor 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'actor1':int2 - Usd path to rigid body actor 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'collider0':int2 - Usd path to collider 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'collider1':int2 - Usd path to collider 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'numContactData':int - Num contact data sent after the header is sent.
                'stageId':long1 - Current USD stage id, long array with one item.
            )")
        .value("CONTACT_PERSISTS", eContactPersists,
            R"(Contact persists event header: sends header information regarding which colliders are still in contact; contains the following in a dictionary:

            .. code-block:: text

                'actor0':int2 - Usd path to rigid body actor 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'actor1':int2 - Usd path to rigid body actor 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'collider0':int2 - Usd path to collider 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'collider1':int2 - Usd path to collider 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'numContactData':int - Num contact data sent after the header is sent.
                'stageId':long1 - Current USD stage id, long array with one item.
            )")
        .value("CONTACT_DATA", eContactData,
            R"(Contact data sent after each header contact information is sent; contains the following in a dictionary:

            .. code-block:: text

                'position':float3 - Contact position
                'normal':float3 - Contact normal
                'impulse':float3 - Contact impulse
                'separation':float - Separation value for collisions.
                'faceIndex0':int - USD face index 0.
                'faceIndex1':int - USD face index 0.
                'material0':int2 - Usd path to material 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'material1':int2 - Usd path to material 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
            )")
        .value("JOINT_BREAK", eJointBreak,
            R"(Joint break event; contains the following in a dictionary:

            .. code-block:: text

                'jointPath':int2 - Usd path to joint that did break decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
            )")
        .value("POINT_GRABBED", ePointGrabbed,
            R"(Point grabbed; contains the following in a dictionary:

            .. code-block:: text

                'grabbed_position':float3 - Current world position of grabbed point.
                'grab_force_position':float3 - Current world position of the position being grabbed towards.
            )")
        .value("POINT_RELEASED", ePointReleased,
               R"(Point released.
            )")
        .value("POINT_PUSHED", ePointPushed,
            R"(Point pushed; contains the following in a dictionary:

            .. code-block:: text

                'pushed_position':float3 - World position of point pushed.
            )")
        .value("ATTACHED_TO_STAGE", eAttachedToStage,
               R"(When physx stage attachment (initialization) finished.
            )")
        .value("DETACHED_FROM_STAGE", eDetachedFromStage,
           R"(When physx stage detachment (deinitialization) finished.
            )");

    py::enum_<ErrorEvent>(m, "ErrorEvent", R"(
        Error events used by physics error event stream.
        )")
        .value("USD_LOAD_ERROR", eUsdLoadError, "Usd load error, event has dictionary with key 'errorString':string")
        .value("PHYSX_ERROR", ePhysxError, "PhysX runtime error, event has dictionary with key 'errorString':string")
        .value("PHYSX_TOO_MANY_ERRORS", ePhysxTooManyErrors, "PhysX exceeded maximum number of reported errors, event has dictionary with key 'errorString':string")
        .value("PHYSX_CUDA_ERROR", ePhysxCudaError, "PhysX GPU Cuda error, event has dictionary with key 'errorString':string");

    m.doc() = docModule.c_str();

    m.def("hasconflictingapis_RigidBodyAPI", &hasconflictingapis_RigidBodyAPI);
    m.def("hasconflictingapis_CollisionAPI", &hasconflictingapis_CollisionAPI);
    m.def("hasconflictingapis_ArticulationRoot", &hasconflictingapis_ArticulationRoot);
    m.def("hasconflictingapis_PhysxDeformableBodyAPI_deprecated", &hasconflictingapis_PhysxDeformableBodyAPI_deprecated);
    m.def("hasconflictingapis_PhysxDeformableSurfaceAPI_deprecated", &hasconflictingapis_PhysxDeformableSurfaceAPI_deprecated);
    m.def("hasconflictingapis_DeformableBodyAPI", &hasconflictingapis_DeformableBodyAPI);
    m.def("hasconflictingapis_PhysxParticleSamplingAPI", &hasconflictingapis_PhysxParticleSamplingAPI);
    m.def("hasconflictingapis_PhysxParticleClothAPI_deprecated", &hasconflictingapis_PhysxParticleClothAPI_deprecated);
    m.def("hasconflictingapis_Precompute", &hasconflictingapis_Precompute);
    m.def("isOverConflictingApisSubtreeLimit", &isOverConflictingApisSubtreeLimit);
    m.def("descendantHasAPI", &descendantHasAPI);
    m.def("ancestorHasAPI", &ancestorHasAPI);

    m.def("hasconflictingapis_RigidBodyAPI_WRet",
        [](const pxr::UsdPrim& prim, bool check_itself, bool check_prim) {
            pxr::UsdPrim ret;
            bool fail = hasconflictingapis_RigidBodyAPI_WRet(prim, ret, check_itself, check_prim);
            return py::make_tuple(fail, ret);
        }, py::arg("prim"), py::arg("check_itself") = false, py::arg("check_prim") = true);

    m.def("hasconflictingapis_CollisionAPI_WRet",
        [](const pxr::UsdPrim& prim, bool check_itself, bool check_prim) {
            pxr::UsdPrim ret;
            bool fail = hasconflictingapis_CollisionAPI_WRet(prim, ret, check_itself, check_prim);
            return py::make_tuple(fail, ret);
        }, py::arg("prim"), py::arg("check_itself") = false, py::arg("check_prim") = true);
    m.def("hasconflictingapis_ArticulationRoot_WRet",
        [](const pxr::UsdPrim& prim, bool check_itself, bool check_prim) {
            pxr::UsdPrim ret;
            bool fail = hasconflictingapis_ArticulationRoot_WRet(prim, ret, check_itself, check_prim);
            return py::make_tuple(fail, ret);
        }, py::arg("prim"), py::arg("check_itself") = false, py::arg("check_prim") = true);

    py::class_<IPhysx> dev = defineInterfaceClass<IPhysx>(m, "PhysX", "acquire_physx_interface", "release_physx_interface");
    dev.doc() = R"(
        This interface is the main access point to omni.physx extension.
        It contains functions that can control the simulation, modify the simulation
        or work directly with physics objects.
    )";

    m.def("release_physx_interface_scripting", [](IPhysx* iface) { carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); });  // OM-60917

    docString = R"(
        Forces load physics objects from USD into PhysX. By default physics is not loaded;
        this function forces a load of all physics from USD into PhysX.
    )";
    dev.def("force_load_physics_from_usd", wrapInterfaceFunction(&IPhysx::forceLoadPhysicsFromUSD), docString);

    docString = R"(
        Forces release of all physics objects from PhysX.
    )";
    dev.def("release_physics_objects", wrapInterfaceFunction(&IPhysx::releasePhysicsObjects), docString);

    docString = R"(
        Sets number of threads for simulation.

        Args:
            numThreads:  Number of threads that physics should use
    )";
    dev.def("set_thread_count", wrapInterfaceFunction(&IPhysx::setThreadCount), docString, py::arg("numThreads"));

    docString = R"(
        Override CPU vs GPU simulation settings.

        Args:
            gpuSetting:  Integer defining the behavior: -1 - use setting from schema, 0 - force CPU, 1 - force GPU
    )";
    dev.def("overwrite_gpu_setting", wrapInterfaceFunction(&IPhysx::overwriteGPUSetting), docString, py::arg("gpuSetting"));

    docString = R"(
        Get CPU vs GPU simulation settings.

        Returns:
            Integer defining the behavior: -1 - use setting from schema, 0 - force CPU, 1 - force GPU
    )";
    dev.def("get_overwrite_gpu_setting", wrapInterfaceFunction(&IPhysx::getOverwriteGPUSetting), docString);

    docString = R"(
        Override PhysX solver settings.

        Args:
            solverSetting:  Integer defining the behavior: -1 - use setting from schema, 0 - force PGS, 1 - force TGS
    )";
    dev.def("overwrite_solver_type", wrapInterfaceFunction(&IPhysx::overwriteSolverType), docString, py::arg("solverSetting"));

    docString = R"(
        Start simulation, store initial USD data. Call this before manually stepping simulation.
    )";
    dev.def("start_simulation", wrapInterfaceFunction(&IPhysx::startSimulation), docString);

    docString = R"(
        Update physics simulation by one step.

        Args:
            elapsedStep:  Simulation step time (seconds), time elapsed between last step and this step.
            currentTime:  Current time (seconds), might be used for time sampled transformations to apply.
    )";
    dev.def("update_simulation", wrapInterfaceFunction(&IPhysx::updateSimulation), docString, py::arg("elapsedStep"), py::arg("currentTime"));

    docString = R"(
        Update transformations after simulation is done.

        Args:
            updateToFastCache:      Update transformation in fast cache.
            updateToUsd:            Update transformations in USD.
            updateVelocitiesToUsd:  Update velocities in USD.
    )";
    dev.def("update_transformations",
        [](const IPhysx* self, bool updateToFastCache, bool updateToUsd, bool updateVelocitiesToUsd, bool outputVelocitiesLocalSpace) {
        self->updateTransformations(updateToFastCache, updateToUsd, updateVelocitiesToUsd, outputVelocitiesLocalSpace);
        }, docString,
        py::arg("updateToFastCache"), py::arg("updateToUsd"), py::arg("updateVelocitiesToUsd") = false, py::arg("outputVelocitiesLocalSpace") = false);

    docString = R"(
        Reset physics simulation and set back original transformations stored during start_simulation.
        This will also remove all data from PhysX.
    )";
    dev.def("reset_simulation", wrapInterfaceFunction(&IPhysx::resetSimulation), docString);

    docString = R"(
        Simulation event stream sending various simulation events defined in SimulationEvent enum.

        Returns:
            Event stream sending the simulation events.
    )";
    dev.def("get_simulation_event_stream_v2", wrapInterfaceFunction(&IPhysx::getSimulationEventStreamV2), docString);

    docString = R"(
        Error event stream sending various error events defined in ErrorEvent enum.

        Returns:
            Event stream sending the physics errors.
    )";
    dev.def("get_error_event_stream", wrapInterfaceFunction(&IPhysx::getErrorEventStream), docString);

    docString = R"(
        Reconnect to PVD (PhysX Visual Debugger)
    )";
    dev.def("reconnect_pvd", wrapInterfaceFunction(&IPhysx::reconnectPVD), docString);

    docString = R"(
        Save scene to RepX, if scene is not loaded, load scene, save and release.

        Args:
            path: Path to save the RepX file.
    )";
    dev.def("save_scene_to_repx", wrapInterfaceFunction(&IPhysx::saveSceneToRepX), docString, py::arg("path"));

    docString = R"(
        Check if simulation loop is running, this function returns true if play was pressed or if
        IPhysxSimulation was attached.
    )";
    dev.def("is_running", wrapInterfaceFunction(&IPhysx::isRunning), docString);

    docString = R"(
        Returns true if asynchronous simulation and rendering is enabled for one of the scenes in the simulation.
    )";
    dev.def("is_asyncsimrender_enabled", wrapInterfaceFunction(&IPhysx::isAsyncSimRenderEnabled), docString);

    docString = R"(
            Subscribes to physics step events.

            Subscription cannot be changed in the onUpdate callback

            Args:
                fn: The callback to be called on every physics step.

            Returns:
                The subscription holder.
    )";
    dev.def(
        "subscribe_physics_step_events",
        [](IPhysx* iface, std::function<void(float)> fn) {
            using namespace std::placeholders;

            return carb::createPySubscription(std::move(fn),
                std::bind(iface->subscribePhysicsOnStepEvents, false, 0, _1, _2),
                [iface](SubscriptionId id) {
                    // Release the GIL since unsubscribe can block on a mutex and deadlock
                    py::gil_scoped_release gsr;
                    iface->unsubscribePhysicsOnStepEvents(id);
                });
        },
        docString, py::arg("fn"));

    docString = R"(
            Subscribes to physics pre-step or post-step events.

            Subscription cannot be changed in the onUpdate callback

            Args:
                fn:         The callback to be called right before or after every physics step.
                pre_step:   Whether fn has to be called right *before* the physics step. If this is false,
                            it will be called right *after* the physics step.
                order:      An integer value used to order the callbacks: 0 means "highest priority", 1 is "less priority" and so on.

            Returns:
                The subscription holder.
    )";
    dev.def("subscribe_physics_on_step_events",
        [](IPhysx* iface, std::function<void(float)> fn, bool pre_step, int order) {
            using namespace std::placeholders;

            return carb::createPySubscription(std::move(fn),
                std::bind(iface->subscribePhysicsOnStepEvents, pre_step, order, _1, _2),
                [iface](SubscriptionId id) {
                    // Release the GIL since unsubscribe can block on a mutex and deadlock
                    py::gil_scoped_release gsr;
                    iface->unsubscribePhysicsOnStepEvents(id);
                });
        }, docString,
        py::arg("fn"), py::arg("pre_step"), py::arg("order"));

    docString = R"(
            Sets simulation layer. This layer is used when simulation output transformations are written to USD.

            Args:
                layer: The layer that we simulate to.
    )";
    dev.def("set_simulation_layer", wrapInterfaceFunction(&IPhysx::setSimulationLayer), docString, py::arg("layer"));

    docString = R"(
        Resets physics preferences to their default values.
    )";
    dev.def("reset_settings_in_preferences", wrapInterfaceFunction(&IPhysx::resetSettingsInPreferences), docString);

    docString = R"(
        Resets physics per-stage settings  to their default values.
    )";
    dev.def("reset_settings_in_stage", wrapInterfaceFunction(&IPhysx::resetSettingsInStage), docString);

    docString = R"(
        Resets all physics settings to their default values.
    )";
    dev.def("reset_settings", wrapInterfaceFunction(&IPhysx::resetSettings), docString);

    docString = R"(
        Resets a specific physics setting to its default value.

        Args:
            setting: The setting to reset to its default value.
    )";
    dev.def("reset_setting", wrapInterfaceFunction(&IPhysx::resetSetting), docString, py::arg("setting"));

    docString = R"(
            Gets rigid body current transformation in a global space.

            Args:
                path: The USD path to the rigid body.

            Returns:
                Return a dictionary with transformation info::

                    'ret_val': bool - whether transformation was found
                    'position': float3 - rigid body position
                    'rotation': float4 - rigid body rotation (quat - x,y,z,w)
    )";
    dev.def("get_rigidbody_transformation", [](IPhysx* iface, const char* path) {
        // A.B. TODO figure out how to use directly SdfPath from python
        const pxr::SdfPath rbPath = pxr::SdfPath(path);
        carb::Float3 pos;
        carb::Float4 rot;
        const bool retVal = iface->getRigidBodyTransformation(rbPath, pos, rot);
        py::dict transformInfo;
        transformInfo["ret_val"] = retVal;
        if (retVal)
        {
            transformInfo["position"] = pos;
            transformInfo["rotation"] = rot;
        }
        return transformInfo;
    }, docString,
        py::arg("path"));

    docString = R"(
        Set Voxelmap Voxels

        Args:
            stage_id: Stage containing source mesh primitive.
            input_path: path to input primitive
            sx: voxel range start X
            sy: voxel range start Y
            sz: voxel range start Z
            ex: voxel range end X
            ey: voxel range end Y
            ez: voxel range end Z
            type: voxel type
            sub_type: voxel subtype
            update: update flag, if zero, writing changes to USD is postponed, if non-zero, all accumulated changes are written to USD
    )";

    dev.def("set_voxel_range", [](const IPhysx* self, long int stageId, const char* path, const int sx, const int sy, const int sz,
           const int ex, const int ey, const int ez, const int type, const int subType, const int update)
    {
            return self->setVoxelRange(stageId, pxr::SdfPath(path), sx, sy, sz, ex, ey, ez, type, subType, update);
    },
    docString, py::arg("stage_id"), py::arg("path"), py::arg("sx"), py::arg("sy"), py::arg("sz"),
    py::arg("ex"), py::arg("ey"), py::arg("ez"), py::arg("type"), py::arg("subtype"), py::arg("update"));

    docString = R"(
        Check if GPU readback is suppressed for currently running simulation.

        Returns:
            True if simulation is running with suppressed readback.  Always returns false when simulation is not running.
    )";
    dev.def("is_readback_suppressed", wrapInterfaceFunction(&IPhysx::isReadbackSuppressed), docString);

    static const py::int_ VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE = py::int_(omni::physx::usdparser::VehicleControllerDesc::automaticGearValue);
    m.attr("VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE") = VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE;

    docString = R"(
        Set the internal dynamics state of a vehicle back to the rest state.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Vehicle USD path.
    )";
    dev.def(
        "set_vehicle_to_rest_state",
        [](const IPhysx* physX, const char* path)
        {
            usdparser::ObjectId id = physX->getObjectId(pxr::SdfPath(path), PhysXType::ePTVehicle);
            physX->setVehicleToRestState(id);
        },
        docString, py::arg("path"));

    docString = R"(
        Get the linear velocity of a vehicle (vehicle prim needs to have vehicle API applied).

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Vehicle USD path.
            direction: carb.Float3: Unit length direction vector along which the linear
                       velocity should get computed. The vector is considered to be
                       relative to the center of mass frame of the vehicle. If None is
                       passed in, then the local forward direction of the vehicle will be used.

        Returns:
            The velocity along the provided direction vector.
    )";
    dev.def(
        "compute_vehicle_velocity",
        [](const IPhysx* physX, const char* path, const carb::Float3* direction)
        {
            usdparser::ObjectId id = physX->getObjectId(pxr::SdfPath(path), PhysXType::ePTVehicle);
            const float speed = physX->computeVehicleVelocity(id, direction);
            return speed;
        },
        docString, py::arg("path"), py::arg("direction"));

    static const py::int_ VEHICLE_DRIVE_STATE_ACCELERATOR = py::int_(0);
    static const py::int_ VEHICLE_DRIVE_STATE_BRAKE0 = py::int_(1);
    static const py::int_ VEHICLE_DRIVE_STATE_BRAKE1 = py::int_(2);
    static const py::int_ VEHICLE_DRIVE_STATE_STEER = py::int_(3);
    static const py::int_ VEHICLE_DRIVE_STATE_CLUTCH = py::int_(4);
    static const py::int_ VEHICLE_DRIVE_STATE_CURRENT_GEAR = py::int_(5);
    static const py::int_ VEHICLE_DRIVE_STATE_TARGET_GEAR = py::int_(6);
    static const py::int_ VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME = py::int_(7);
    static const py::int_ VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT = py::int_(8);
    static const py::int_ VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED = py::int_(9);
    static const py::int_ VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION = py::int_(10);
    m.attr("VEHICLE_DRIVE_STATE_ACCELERATOR") = VEHICLE_DRIVE_STATE_ACCELERATOR;
    m.attr("VEHICLE_DRIVE_STATE_BRAKE0") = VEHICLE_DRIVE_STATE_BRAKE0;
    m.attr("VEHICLE_DRIVE_STATE_BRAKE1") = VEHICLE_DRIVE_STATE_BRAKE1;
    m.attr("VEHICLE_DRIVE_STATE_STEER") = VEHICLE_DRIVE_STATE_STEER;
    m.attr("VEHICLE_DRIVE_STATE_CLUTCH") = VEHICLE_DRIVE_STATE_CLUTCH;
    m.attr("VEHICLE_DRIVE_STATE_CURRENT_GEAR") = VEHICLE_DRIVE_STATE_CURRENT_GEAR;
    m.attr("VEHICLE_DRIVE_STATE_TARGET_GEAR") = VEHICLE_DRIVE_STATE_TARGET_GEAR;
    m.attr("VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME") = VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME;
    m.attr("VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT") = VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT;
    m.attr("VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED") = VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED;
    m.attr("VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION") = VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION;

    docString = R"(
        Get the drive state of a vehicle.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Vehicle USD path.

        Returns:
            A dictionary with the following key, value pairs. An empty dictionary is returned for an invalid path,
            when the simulation is not running or when the vehicle does not have a Basic or Standard drive type.

            | VEHICLE_DRIVE_STATE_ACCELERATOR, d (in range [0, 1]),
            | VEHICLE_DRIVE_STATE_BRAKE0, d (in range [0, 1]),
            | VEHICLE_DRIVE_STATE_BRAKE1, d (in range [0, 1]),
            | VEHICLE_DRIVE_STATE_STEER, d (in range [-1, 1]),
            | VEHICLE_DRIVE_STATE_CLUTCH, d (in range [0, 1], only applicable to PhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_CURRENT_GEAR, i (only applicable to PhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_TARGET_GEAR, i (only applicable to PhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME, d (in seconds, negative value if no gear shift is in process. Only applicable to PhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT, d (in seconds, only applicable toPhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED, d (in radians per second, only applicable to PhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION, i (only applicable to PhysxVehicleDriveStandard drive type)
    )";
    dev.def(
        "get_vehicle_drive_state",
        [](const IPhysx* physX, const char* path)
        {
            usdparser::ObjectId id = physX->getObjectId(pxr::SdfPath(path), PhysXType::ePTVehicleController);
            if (id != usdparser::kInvalidObjectId)
            {
                VehicleDriveState driveState;
                bool success = physX->getVehicleDriveState(id, driveState);

                if (success)
                {
                    py::dict driveDict;

                    driveDict[VEHICLE_DRIVE_STATE_ACCELERATOR] = driveState.accelerator;
                    driveDict[VEHICLE_DRIVE_STATE_BRAKE0] = driveState.brake0;
                    driveDict[VEHICLE_DRIVE_STATE_BRAKE1] = driveState.brake1;
                    driveDict[VEHICLE_DRIVE_STATE_STEER] = driveState.steer;
                    driveDict[VEHICLE_DRIVE_STATE_CLUTCH] = driveState.clutch;
                    driveDict[VEHICLE_DRIVE_STATE_CURRENT_GEAR] = driveState.currentGear;
                    driveDict[VEHICLE_DRIVE_STATE_TARGET_GEAR] = driveState.targetGear;
                    driveDict[VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME] = driveState.gearSwitchTime;
                    driveDict[VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT] = driveState.autoboxTimeSinceLastShift;
                    driveDict[VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED] = driveState.engineRotationSpeed;
                    driveDict[VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION] = driveState.automaticTransmission;

                    return driveDict;
                }
                else
                {
                    CARB_LOG_ERROR("get_vehicle_drive_state: call failed for vehicle at path \"%s\". "
                        "Make sure the vehicle has a Standard or Basic drive type defined. "
                        "Make sure the vehicle has the PhysxVehicleControllerAPI schema applied.\n", path);
                }
            }
            else
            {
                CARB_LOG_ERROR("get_vehicle_drive_state: object ID for vehicle at path \"%s\" could not be found. "
                    "Make sure the path is correct. Make sure the simulation is running. Make sure the vehicle has a Standard or Basic drive type defined. "
                    "Make sure the vehicle has the PhysxVehicleControllerAPI schema applied.\n", path);
            }

            py::dict emptyDict;
            return emptyDict;
        },
        docString, py::arg("path"));

    static const py::int_ VEHICLE_WHEEL_STATE_LOCAL_POSE_POSITION = py::int_(0);
    static const py::int_ VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION = py::int_(1);
    static const py::int_ VEHICLE_WHEEL_STATE_ROTATION_SPEED = py::int_(2);
    static const py::int_ VEHICLE_WHEEL_STATE_ROTATION_ANGLE = py::int_(3);
    static const py::int_ VEHICLE_WHEEL_STATE_STEER_ANGLE = py::int_(4);
    static const py::int_ VEHICLE_WHEEL_STATE_GROUND_PLANE = py::int_(5);
    static const py::int_ VEHICLE_WHEEL_STATE_GROUND_ACTOR = py::int_(6);
    static const py::int_ VEHICLE_WHEEL_STATE_GROUND_SHAPE = py::int_(7);
    static const py::int_ VEHICLE_WHEEL_STATE_GROUND_MATERIAL = py::int_(8);
    static const py::int_ VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION = py::int_(9);
    static const py::int_ VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE = py::int_(10);
    static const py::int_ VEHICLE_WHEEL_STATE_SUSPENSION_FORCE = py::int_(11);
    static const py::int_ VEHICLE_WHEEL_STATE_TIRE_FRICTION = py::int_(12);
    static const py::int_ VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_DIRECTION = py::int_(13);
    static const py::int_ VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION = py::int_(14);
    static const py::int_ VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP = py::int_(15);
    static const py::int_ VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP = py::int_(16);
    static const py::int_ VEHICLE_WHEEL_STATE_TIRE_FORCE = py::int_(17);
    static const py::int_ VEHICLE_WHEEL_STATE_IS_ON_GROUND = py::int_(18);
    m.attr("VEHICLE_WHEEL_STATE_LOCAL_POSE_POSITION") = VEHICLE_WHEEL_STATE_LOCAL_POSE_POSITION;
    m.attr("VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION") = VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION;
    m.attr("VEHICLE_WHEEL_STATE_ROTATION_SPEED") = VEHICLE_WHEEL_STATE_ROTATION_SPEED;
    m.attr("VEHICLE_WHEEL_STATE_ROTATION_ANGLE") = VEHICLE_WHEEL_STATE_ROTATION_ANGLE;
    m.attr("VEHICLE_WHEEL_STATE_STEER_ANGLE") = VEHICLE_WHEEL_STATE_STEER_ANGLE;
    m.attr("VEHICLE_WHEEL_STATE_GROUND_PLANE") = VEHICLE_WHEEL_STATE_GROUND_PLANE;
    m.attr("VEHICLE_WHEEL_STATE_GROUND_ACTOR") = VEHICLE_WHEEL_STATE_GROUND_ACTOR;
    m.attr("VEHICLE_WHEEL_STATE_GROUND_SHAPE") = VEHICLE_WHEEL_STATE_GROUND_SHAPE;
    m.attr("VEHICLE_WHEEL_STATE_GROUND_MATERIAL") = VEHICLE_WHEEL_STATE_GROUND_MATERIAL;
    m.attr("VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION") = VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION;
    m.attr("VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE") = VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE;
    m.attr("VEHICLE_WHEEL_STATE_SUSPENSION_FORCE") = VEHICLE_WHEEL_STATE_SUSPENSION_FORCE;
    m.attr("VEHICLE_WHEEL_STATE_TIRE_FRICTION") = VEHICLE_WHEEL_STATE_TIRE_FRICTION;
    m.attr("VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_DIRECTION") = VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_DIRECTION;
    m.attr("VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION") = VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION;
    m.attr("VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP") = VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP;
    m.attr("VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP") = VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP;
    m.attr("VEHICLE_WHEEL_STATE_TIRE_FORCE") = VEHICLE_WHEEL_STATE_TIRE_FORCE;
    m.attr("VEHICLE_WHEEL_STATE_IS_ON_GROUND") = VEHICLE_WHEEL_STATE_IS_ON_GROUND;

    docString = R"(
        Get the wheel state.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Wheel attachment USD path.

        Returns:
            A dictionary with the following key, value pairs. An empty dictionary is returned for an invalid path.

            | VEHICLE_WHEEL_STATE_LOCAL_POSE_POSITION, (d, d, d),
            | VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION, (d, d, d, d)
            | VEHICLE_WHEEL_STATE_ROTATION_SPEED, d (in radians per second),
            | VEHICLE_WHEEL_STATE_ROTATION_ANGLE, d (in radians),
            | VEHICLE_WHEEL_STATE_STEER_ANGLE, d (in radians),
            | VEHICLE_WHEEL_STATE_GROUND_PLANE, (d, d, d, d) (first 3 entries are plane normal n, fourth entry is d of equation dot(n, v) + d = 0). Only valid if wheel is on ground, see VEHICLE_WHEEL_STATE_IS_ON_GROUND,
            | VEHICLE_WHEEL_STATE_GROUND_ACTOR, string (USD path of the actor prim the wheel is driving on). Only valid if wheel is on ground, see VEHICLE_WHEEL_STATE_IS_ON_GROUND,
            | VEHICLE_WHEEL_STATE_GROUND_SHAPE, string (USD path of the collider prim the wheel is driving on). Only valid if wheel is on ground, see VEHICLE_WHEEL_STATE_IS_ON_GROUND,
            | VEHICLE_WHEEL_STATE_GROUND_MATERIAL, string (USD path of the material prim the wheel is driving on). Only valid if wheel is on ground, see VEHICLE_WHEEL_STATE_IS_ON_GROUND,
            | VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION, (d, d, d) (hit position on the ground in world space). Only valid if wheel is on ground, see VEHICLE_WHEEL_STATE_IS_ON_GROUND,
            | VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE, d,
            | VEHICLE_WHEEL_STATE_SUSPENSION_FORCE, (d, d, d),
            | VEHICLE_WHEEL_STATE_TIRE_FRICTION, d,
            | VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_DIRECTION, (d, d, d),
            | VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION, (d, d, d),
            | VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP, d,
            | VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP, d,
            | VEHICLE_WHEEL_STATE_TIRE_FORCE, (d, d, d),
            | VEHICLE_WHEEL_STATE_IS_ON_GROUND, i, whether the wheel did touch the ground or is in air (=0). If the vehicle is disabled or sleeping then the wheel will be treated as not touching ground too.
    )";
    dev.def(
        "get_wheel_state",
        [](const IPhysx* physX, const char* path)
        {
            usdparser::ObjectId id = physX->getObjectId(pxr::SdfPath(path), PhysXType::ePTVehicleWheelAttachment);
            if (id != usdparser::kInvalidObjectId)
            {
                VehicleWheelState wheelState;
                bool success = physX->getWheelState(&id, 1, &wheelState);

                if (success)
                {
                    py::dict wheelDict;
                    wheelDict[VEHICLE_WHEEL_STATE_LOCAL_POSE_POSITION] = wheelState.localPosePosition;
                    wheelDict[VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION] = wheelState.localPoseQuaternion;
                    wheelDict[VEHICLE_WHEEL_STATE_ROTATION_SPEED] = wheelState.rotationSpeed;
                    wheelDict[VEHICLE_WHEEL_STATE_ROTATION_ANGLE] = wheelState.rotationAngle;
                    wheelDict[VEHICLE_WHEEL_STATE_STEER_ANGLE] = wheelState.steerAngle;
                    wheelDict[VEHICLE_WHEEL_STATE_GROUND_PLANE] = wheelState.groundPlane;
                    wheelDict[VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION] = wheelState.groundHitPosition;

                    if (wheelState.isOnGround)
                    {
                        pxr::SdfPath path;

                        path = physX->getPhysXObjectUsdPath(wheelState.groundActor);
                        wheelDict[VEHICLE_WHEEL_STATE_GROUND_ACTOR] = std::string(path.GetText());

                        path = physX->getPhysXObjectUsdPath(wheelState.groundShape);
                        wheelDict[VEHICLE_WHEEL_STATE_GROUND_SHAPE] = std::string(path.GetText());

                        path = physX->getPhysXObjectUsdPath(wheelState.groundMaterial);
                        wheelDict[VEHICLE_WHEEL_STATE_GROUND_MATERIAL] = std::string(path.GetText());
                    }
                    else
                    {
                        wheelDict[VEHICLE_WHEEL_STATE_GROUND_ACTOR] = std::string();
                        wheelDict[VEHICLE_WHEEL_STATE_GROUND_SHAPE] = std::string();
                        wheelDict[VEHICLE_WHEEL_STATE_GROUND_MATERIAL] = std::string();
                    }

                    wheelDict[VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE] = wheelState.suspensionJounce;
                    wheelDict[VEHICLE_WHEEL_STATE_SUSPENSION_FORCE] = wheelState.suspensionForce;
                    wheelDict[VEHICLE_WHEEL_STATE_TIRE_FRICTION] = wheelState.tireFriction;
                    wheelDict[VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_DIRECTION] = wheelState.tireLongitudinalDirection;
                    wheelDict[VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION] = wheelState.tireLateralDirection;
                    wheelDict[VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP] = wheelState.tireLongitudinalSlip;
                    wheelDict[VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP] = wheelState.tireLateralSlip;
                    wheelDict[VEHICLE_WHEEL_STATE_TIRE_FORCE] = wheelState.tireForce;
                    wheelDict[VEHICLE_WHEEL_STATE_IS_ON_GROUND] = wheelState.isOnGround;

                    return wheelDict;
                }
                else
                {
                    CARB_LOG_ERROR("get_wheel_state: call failed for wheel attachment at path \"%s\".\n", path);
                }
            }
            else
            {
                CARB_LOG_ERROR("get_wheel_state: object ID for wheel attachment at path \"%s\" could not be found. "
                    "Make sure the path is correct and that the simulation is running.\n", path);
            }

            py::dict emptyDict;
            return emptyDict;
        },
        docString, py::arg("path"));

    docString = R"(
        Set the rotation speed about the rolling axis of a wheel.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Wheel attachment USD path.
            rotationSpeed: Rotation speed of the wheel in radians per second.
    )";
    dev.def(
        "set_wheel_rotation_speed",
        [](const IPhysx* physX, const char* path, const float rotationSpeed)
        {
            usdparser::ObjectId id = physX->getObjectId(pxr::SdfPath(path), PhysXType::ePTVehicleWheelAttachment);
            if (id != usdparser::kInvalidObjectId)
            {
                physX->setWheelRotationSpeed(&id, 1, &rotationSpeed);
            }
            else
            {
                CARB_LOG_ERROR("set_wheel_rotation_speed: object ID for wheel attachment at path \"%s\" could not be found. "
                    "Make sure the path is correct and that the simulation is running.\n", path);
            }
        },
        docString, py::arg("path"), py::arg("rotationSpeed"));

    docString = R"(
        Set the rotation angle about the rolling axis of a wheel.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Wheel attachment USD path.
            rotationAngle: Rotation angle of the wheel in radians.
    )";
    dev.def(
        "set_wheel_rotation_angle",
        [](const IPhysx* physX, const char* path, const float rotationAngle)
        {
            usdparser::ObjectId id = physX->getObjectId(pxr::SdfPath(path), PhysXType::ePTVehicleWheelAttachment);
            if (id != usdparser::kInvalidObjectId)
            {
                physX->setWheelRotationAngle(&id, 1, &rotationAngle);
            }
            else
            {
                CARB_LOG_ERROR("set_wheel_rotation_angle: object ID for wheel attachment at path \"%s\" could not be found. "
                    "Make sure the path is correct and that the simulation is running.\n", path);
            }
        },
        docString, py::arg("path"), py::arg("rotationAngle"));

    docString = R"(
        Raycast check to detect interactive physics actors

        Note:
            Only produces positive results during simulation

        Args:
            origin:    carb.Float3: World-space origin of raycast
            direction: carb.Float3: Unit-length direction vector of raycast

        Returns:
            True if interactive actor is hit; False otherwise, or if simulation is not running
    )";
    dev.def("is_interactive_actor_raycast", wrapInterfaceFunction(&IPhysx::isInteractiveActorRaycast), docString,
            py::arg("origin"), py::arg("direction"));

    py::enum_<PhysicsInteractionEvent>(m, "PhysicsInteractionEvent", R"(
        Physics interaction event
        )")
        .value("MOUSE_DRAG_BEGAN", PhysicsInteractionEvent::eMouseDragBegan, "Signals that a mouse drag has begun.")
        .value("MOUSE_DRAG_CHANGED", PhysicsInteractionEvent::eMouseDragChanged, "Signals that the mouse is being dragged.")
        .value("MOUSE_DRAG_ENDED", PhysicsInteractionEvent::eMouseDragEnded, "Signals that the mouse drag is being released.")
        .value("MOUSE_LEFT_CLICK", PhysicsInteractionEvent::eMouseLeftClick, "Signals that the mouse left button is clicked.")
        .value("MOUSE_LEFT_DOUBLE_CLICK", PhysicsInteractionEvent::eMouseLeftDoubleClick, "Signals that the mouse left button is being double-clicked.");
    docString = R"(
        Updates actor interaction based on user input and raycast origin and direction

        Note:
            Only provides interaction during simulation

        Args:
            origin:          carb.Float3: World-space origin of interaction
            direction:       carb.Float3: Unit-length direction vector of interaction
            event:           PhysicsInteractionEvent triggered.
    )";
    dev.def("update_interaction", wrapInterfaceFunction(&IPhysx::updateInteraction), docString,
            py::arg("origin"), py::arg("direction"), py::arg("event"));

    docString = R"(
        Update and step a specific scene in the physics simulation. The specific scene specified in scenePath
        is updated and stepped *even if marked as 'Disabled'*.
        If scenePath is empty, it behaves like IPhysx::updateSimulation

        Args:
            scenePath:       uint64_t       Scene USD path use PhysicsSchemaTools::sdfPathToInt
            elapsedStep:     float          Simulation time (seconds).
            currentTime:     float          Current time (seconds), might be used for time sampled transformations to apply.
    )";
    dev.def("update_simulation_scene", wrapInterfaceFunction(&IPhysx::updateSimulationScene), docString,
            py::arg("scene_path"), py::arg("elapsed_step"), py::arg("current_time"));

    docString = R"(
        Update the transformations for a specific scene in the physics simulation. The specific scene specified in scenePath
        has its transformations updated *even if it is marked as 'Disabled'*.
        If scenePath is empty, it behaves like IPhysx::updateTransformations

        scenePath              uint64_t         Scene USD path use PhysicsSchemaTools::sdfPathToInt
        updateToUsd            bool             Update transforms to USD.
        updateVelocitiesToUsd  bool             Update velocities to USD.
    )";
    dev.def("update_transformations_scene", wrapInterfaceFunction(&IPhysx::updateTransformationsScene), docString,
            py::arg("scene_path"), py::arg("update_to_usd"), py::arg("update_velocities_to_usd"));

    PhysXObjectChangedCallbackBindings(dev);

    py::class_<SceneQueryHitObject>(m, "SceneQueryHitObject", R"(
            Scene query hit results structure.
        )")
        .def_property_readonly("collision_encoded",
            [](const SceneQueryHitObject* self)
            {
                const uint32_t p0 = uint32_t(self->collision);
                const uint32_t p1 = uint32_t(self->collision >> 32);
                py::list outList;
                outList.append(p0);
                outList.append(p1);
                return outList;
            }
            , R"(Encoded SdfPath to the collision that was hit. PhysicsSchemaTools.decodeSdfPath will return SdfPath.)")
        .def_property_readonly("collision",
            [](const SceneQueryHitObject* self)
            {
                const pxr::SdfPath path = intToPath(self->collision);
                return path.GetText();
            },
            R"(Path string to the collision that was hit.)")
       .def_property_readonly("rigid_body_encoded",
            [](const SceneQueryHitObject* self)
            {
                const uint32_t p0 = uint32_t(self->rigidBody);
                const uint32_t p1 = uint32_t(self->rigidBody >> 32);
                py::list outList;
                outList.append(p0);
                outList.append(p1);
                return outList;
            }
            , R"(Encoded SdfPath to the rigid body that was hit. PhysicsSchemaTools.decodeSdfPath will return SdfPath.)")
       .def_property_readonly("rigid_body",
            [](const SceneQueryHitObject* self)
            {
                const pxr::SdfPath path = intToPath(self->rigidBody);
                return path.GetText();
            },
            R"(Path string to the rigid body that was hit.)")
        .def_readonly("protoIndex", &SceneQueryHitObject::protoIndex,
            R"(ProtoIndex, filled for pointInstancers otherwise 0xFFFFFFFF.)");

    py::class_<SceneQueryHitLocation, SceneQueryHitObject>(m, "SceneQueryHitLocation", R"(
        Scene query hit location results structure.
        )")
        .def_readonly("position", &SceneQueryHitLocation::position, R"(Hit location position.)")
        .def_readonly("normal", &SceneQueryHitLocation::normal, R"(Hit location normal.)")
        .def_readonly("distance", &SceneQueryHitLocation::distance, R"(Hit location distance.)")
        .def_readonly("face_index", &SceneQueryHitLocation::faceIndex, R"(Hit location face index.)")
        .def_property_readonly(
            "material_encoded",
            [](const SceneQueryHitLocation* self) {
                const uint32_t p0 = uint32_t(self->material);
                const uint32_t p1 = uint32_t(self->material >> 32);
                py::list outList;
                outList.append(p0);
                outList.append(p1);
                return outList;
            },
            R"(Encoded SdfPath to the collider material that was hit. PhysicsSchemaTools.decodeSdfPath will return SdfPath.)")
        .def_property_readonly(
            "material",
            [](const SceneQueryHitLocation* self) {
                const pxr::SdfPath path = intToPath(self->material);
                return path.GetText();
            },
            R"(Path string to the collider material that was hit.)");

    py::class_<RaycastHit, SceneQueryHitLocation>(m, "RaycastHit", R"(
        Raycast hit results structure.
        )");

    py::class_<SweepHit, SceneQueryHitLocation>(m, "SweepHit", R"(
        Sweep hit results structure.
        )");

    py::class_<OverlapHit, SceneQueryHitObject>(m, "OverlapHit", R"(
        Overlap hit results structure.
        )");

    dev = defineInterfaceClass<IPhysxSceneQuery>(m, "PhysXSceneQuery", "acquire_physx_scene_query_interface", "release_physx_scene_query_interface");
    dev.doc() = R"(
        This interface is the access point to the omni.physx extension scene query API.
    )";

    docString = R"(
        Raycast physics scene for the closest collision.

        Args:
            origin: Origin of the raycast.
            dir: Unit direction of the raycast.
            distance: Raycast distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a dictionary with raycast hit::

                'position': float3 -- Hit position.
                'normal': float3 -- Hit normal.
                'distance': float -- Hit distance.
                'faceIndex': int -- Hit mesh face index.
                'collision': string -- Hit collision USD path.
                'rigidBody': string -- Hit rigid body USD path.
                'protoIndex': int -- Rigid body protoIndex - if not point instancer 0xFFFFFFFF
                'material': string -- Hit collider material USD path.
    )";
    dev.def("raycast_closest",
        [](const IPhysxSceneQuery* self, const carb::Float3& origin, const carb::Float3& dir, float distance, bool bothSides) {
        RaycastHit hit;
        const bool retVal = self->raycastClosest(origin, dir, distance, hit, bothSides);
        // Return as dict
        py::dict hitInfo;
        hitInfo["hit"] = retVal;
        if (retVal)
        {
            hitInfo["position"] = hit.position;
            hitInfo["normal"] = hit.normal;
            hitInfo["distance"] = hit.distance;
            hitInfo["faceIndex"] = hit.faceIndex;
            const pxr::SdfPath collisionPath = intToPath(hit.collision);
            hitInfo["collision"] = collisionPath.GetText();
            const pxr::SdfPath rbPath = intToPath(hit.rigidBody);
            hitInfo["rigidBody"] = rbPath.GetText();
            const pxr::SdfPath materialPath = intToPath(hit.material);
            hitInfo["material"] = materialPath.GetText();
            hitInfo["protoIndex"] = hit.protoIndex;
        }
        return hitInfo;
    }, docString,
        py::arg("origin"), py::arg("dir"), py::arg("distance"), py::arg("bothSides") = false);

    docString = R"(
        Raycast physics scene for any collision, reporting only boolean.

        Args:
            origin: Origin of the raycast.
            dir: Unit direction of the raycast.
            distance: Raycast distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a boolean whether raycast hit.
    )";
    dev.def("raycast_any",
        [](const IPhysxSceneQuery* self, const carb::Float3& origin, const carb::Float3& dir, float distance,
           bool bothSides) {
            return self->raycastAny(origin, dir, distance, bothSides);
        },
        docString, py::arg("origin"), py::arg("dir"), py::arg("distance"), py::arg("bothSides") = false);

    docString = R"(
        Raycast physics scene for all collisions.

        Args:
            origin: Origin of the raycast.
            dir: Unit direction of the raycast.
            distance: Raycast distance.
            reportFn: Report function where RaycastHits will be reported, return True to continue traversal, False to stop traversal
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Returns true if hit
    )";
    dev.def("raycast_all",
        [](const IPhysxSceneQuery* self, const carb::Float3& origin, const carb::Float3& dir,
            float distance, std::function<bool(const RaycastHit& hit)> reportFn, bool bothSides) {
        return self->raycastAll(origin, dir, distance, carb::wrapPythonCallback(std::move(reportFn)), bothSides);
    }, docString,
        py::arg("origin"), py::arg("dir"), py::arg("distance"), py::arg("reportFn"), py::arg("bothSides") = false);

    docString = R"(
        Sphere sweep physics scene for the closest collision.

        Args:
            radius: Sphere radius.
            origin: Origin of the sphere cast.
            dir: Unit direction of the sphere cast.
            distance: Sphere cast distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a dictionary with sweep hit:

                'position': float3 - hit position
                'normal': float3 - hit normal
                'distance': float - hit distance
                'faceIndex': int - hit mesh face index
                'collision': string - hit collision USD path
                'rigidBody': string - hit rigid body USD path
                'protoIndex': int -- Rigid body protoIndex - if not point instancer 0xFFFFFFFF
                'material': string -- Hit collider material USD path.
    )";
    dev.def("sweep_sphere_closest",
        [](const IPhysxSceneQuery* self, float radius, const carb::Float3& origin, const carb::Float3& dir, float distance,
            bool bothSides) {
        SweepHit hit;
        const bool retVal = self->sweepSphereClosest(radius, origin, dir, distance, hit, bothSides);
        // Return as dict
        py::dict hitInfo;
        hitInfo["hit"] = retVal;
        if (retVal)
        {
            hitInfo["position"] = hit.position;
            hitInfo["normal"] = hit.normal;
            hitInfo["distance"] = hit.distance;
            hitInfo["faceIndex"] = hit.faceIndex;
            const pxr::SdfPath collisionPath = intToPath(hit.collision);
            hitInfo["collision"] = collisionPath.GetText();
            const pxr::SdfPath rbPath = intToPath(hit.rigidBody);
            hitInfo["rigidBody"] = rbPath.GetText();
            const pxr::SdfPath materialPath = intToPath(hit.material);
            hitInfo["material"] = materialPath.GetText();
            hitInfo["protoIndex"] = hit.protoIndex;
        }
        return hitInfo;
    }, docString,
        py::arg("radius"), py::arg("origin"), py::arg("dir"), py::arg("distance"), py::arg("bothSides") = false);

    docString = R"(
        Box sweep physics scene for the closest collision.

        Args:
            halfExtent: Box half extent.
            pos: Origin of the sweep (barycenter of the box).
            rot: Rotation of the sweep box (quat x, y, z, w)
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a dictionary with raycast hit::

                'position': float3 - hit position
                'normal': float3 - hit normal
                'distance': float - hit distance
                'faceIndex': int - hit mesh face index
                'collision': string - hit collision USD path
                'rigidBody': string - hit rigid body USD path
                'protoIndex': int -- Rigid body protoIndex - if not point instancer 0xFFFFFFFF
                'material': string -- Hit collider material USD path.
    )";
    dev.def("sweep_box_closest",
        [](const IPhysxSceneQuery* self, const carb::Float3& halfExtent, const carb::Float3& pos,
           const carb::Float4& rot, const carb::Float3& dir, float distance,
            bool bothSides) {
        SweepHit hit;
        const bool retVal = self->sweepBoxClosest(halfExtent, pos, rot, dir, distance, hit, bothSides);
        // Return as dict
        py::dict hitInfo;
        hitInfo["hit"] = retVal;
        if (retVal)
        {
            hitInfo["position"] = hit.position;
            hitInfo["normal"] = hit.normal;
            hitInfo["distance"] = hit.distance;
            hitInfo["faceIndex"] = hit.faceIndex;
            const pxr::SdfPath collisionPath = intToPath(hit.collision);
            hitInfo["collision"] = collisionPath.GetText();
            const pxr::SdfPath rbPath = intToPath(hit.rigidBody);
            hitInfo["rigidBody"] = rbPath.GetText();
            const pxr::SdfPath materialPath = intToPath(hit.material);
            hitInfo["material"] = materialPath.GetText();
            hitInfo["protoIndex"] = hit.protoIndex;
        }
        return hitInfo;
    }, docString,
        py::arg("halfExtent"), py::arg("pos"), py::arg("rot"), py::arg("dir"), py::arg("distance"), py::arg("bothSides") = false);

    docString = R"(
        Sweep test of a UsdGeom.Mesh against objects in the physics scene for the closest collision. Sweep test will use convex mesh
        approximation of the input mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            mesh: UsdGeom.Mesh path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a dictionary with sweep hit:

                'position': float3 - hit position
                'normal': float3 - hit normal
                'distance': float - hit distance
                'faceIndex': int - hit mesh face index
                'collision': string - hit collision USD path
                'rigidBody': string - hit rigid body USD path
                'protoIndex': int -- Rigid body protoIndex - if not point instancer 0xFFFFFFFF
                'material': string -- Hit collider material USD path.
    )";
    dev.def("sweep_mesh_closest",
        [](const IPhysxSceneQuery* self, uint32_t ePart0, uint32_t ePart1, const carb::Float3& dir, float distance,
            bool bothSides) {
        const uint64_t part0 = uint64_t(ePart0);
        const uint64_t part1 = uint64_t(ePart1);
        const uint64_t uintPath = part0 + (part1 << 32);
        SweepHit hit;
        const bool retVal = self->sweepMeshClosest(uintPath, dir, distance, hit, bothSides);
        // Return as dict
        py::dict hitInfo;
        hitInfo["hit"] = retVal;
        if (retVal)
        {
            hitInfo["position"] = hit.position;
            hitInfo["normal"] = hit.normal;
            hitInfo["distance"] = hit.distance;
            hitInfo["faceIndex"] = hit.faceIndex;
            const pxr::SdfPath collisionPath = intToPath(hit.collision);
            hitInfo["collision"] = collisionPath.GetText();
            const pxr::SdfPath rbPath = intToPath(hit.rigidBody);
            hitInfo["rigidBody"] = rbPath.GetText();
            const pxr::SdfPath materialPath = intToPath(hit.material);
            hitInfo["material"] = materialPath.GetText();
            hitInfo["protoIndex"] = hit.protoIndex;
        }
        return hitInfo;
    }, docString,
        py::arg("meshPath0"), py::arg("meshPath1"), py::arg("dir"), py::arg("distance"), py::arg("bothSides") = false);

    docString = R"(
        Sweep test of a UsdGeom.GPrim against objects in the physics scene. Sweep test will use convex mesh
        approximation if the input is a mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            gprim: UsdGeom.GPrim path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a dictionary with sweep hit:

                'position': float3 - hit position
                'normal': float3 - hit normal
                'distance': float - hit distance
                'faceIndex': int - hit mesh face index
                'collision': string - hit collision USD path
                'rigidBody': string - hit rigid body USD path
                'protoIndex': int -- Rigid body protoIndex - if not point instancer 0xFFFFFFFF
                'material': string -- Hit collider material USD path.
    )";
    dev.def("sweep_shape_closest",
        [](const IPhysxSceneQuery* self, uint32_t ePart0, uint32_t ePart1, const carb::Float3& dir, float distance,
            bool bothSides) {
        const uint64_t part0 = uint64_t(ePart0);
        const uint64_t part1 = uint64_t(ePart1);
        const uint64_t uintPath = part0 + (part1 << 32);
        SweepHit hit;
        const bool retVal = self->sweepShapeClosest(uintPath, dir, distance, hit, bothSides);
        // Return as dict
        py::dict hitInfo;
        hitInfo["hit"] = retVal;
        if (retVal)
        {
            hitInfo["position"] = hit.position;
            hitInfo["normal"] = hit.normal;
            hitInfo["distance"] = hit.distance;
            hitInfo["faceIndex"] = hit.faceIndex;
            const pxr::SdfPath collisionPath = intToPath(hit.collision);
            hitInfo["collision"] = collisionPath.GetText();
            const pxr::SdfPath rbPath = intToPath(hit.rigidBody);
            hitInfo["rigidBody"] = rbPath.GetText();
            const pxr::SdfPath materialPath = intToPath(hit.material);
            hitInfo["material"] = materialPath.GetText();
            hitInfo["protoIndex"] = hit.protoIndex;
        }
        return hitInfo;
    }, docString,
        py::arg("meshPath0"), py::arg("meshPath1"), py::arg("dir"), py::arg("distance"), py::arg("bothSides") = false);

    docString = R"(
        Sphere sweep physics scene for any collision, reporting only boolean.

        Args:
            radius: Sphere radius.
            origin: Origin of the sphere sweep.
            dir: Unit direction of the sphere sweep.
            distance: Sphere sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a boolean if sphere sweep hit.
    )";
    dev.def("sweep_sphere_any",
        [](const IPhysxSceneQuery* self, float radius, const carb::Float3& origin, const carb::Float3& dir,
           float distance, bool bothSides) {
            return self->sweepSphereAny(radius, origin, dir, distance, bothSides);
        },
        docString, py::arg("radius"), py::arg("origin"), py::arg("dir"), py::arg("distance"),
        py::arg("bothSides") = false);

    docString = R"(
        Box sweep physics scene for any collision, reporting only boolean.

        Args:
            halfExtent: Box half extent.
            pos: Origin of the sweep (barycenter of the box).
            rot: Rotation of the sweep box (quat x, y, z, w)
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a boolean if sweep hit.
    )";
    dev.def("sweep_box_any",
        [](const IPhysxSceneQuery* self, const carb::Float3& halfExtent, const carb::Float3& pos,
           const carb::Float4& rot, const carb::Float3& dir,
           float distance, bool bothSides) {
            return self->sweepBoxAny(halfExtent, pos, rot, dir, distance, bothSides);
        },
        docString, py::arg("halfExtent"), py::arg("pos"), py::arg("rot"), py::arg("dir"), py::arg("distance"),
        py::arg("bothSides") = false);

    docString = R"(
        Sweep test of a UsdGeom.Mesh against objects in the physics scene for any collision, reporting only boolean.
        Sweep test will use convex mesh approximation of the input mesh. The first query will need to cook this
        approximation so if the results are not stored in a local cache, this query might be slow.

        Args:
            mesh: UsdGeom.Mesh path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a boolean if sweep hit.
    )";
    dev.def("sweep_mesh_any",
        [](const IPhysxSceneQuery* self, uint32_t ePart0, uint32_t ePart1, const carb::Float3& dir,
           float distance, bool bothSides) {
            const uint64_t part0 = uint64_t(ePart0);
            const uint64_t part1 = uint64_t(ePart1);
            const uint64_t uintPath = part0 + (part1 << 32);
            return self->sweepMeshAny(uintPath, dir, distance, bothSides);
        },
        docString, py::arg("meshPath0"), py::arg("meshPath1"), py::arg("dir"), py::arg("distance"),
        py::arg("bothSides") = false);

    docString = R"(
        Sweep test of a UsdGeom.GPrim against objects in the physics scene for any collision, reporting only boolean.
        Sweep test will use convex mesh approximation if the input is a mesh. The first query will need to cook this
        approximation so if the results are not stored in a local cache, this query might be slow.

        Args:
            gprim: UsdGeom.GPrim path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a boolean if sweep hit.
    )";
    dev.def("sweep_shape_any",
        [](const IPhysxSceneQuery* self, uint32_t ePart0, uint32_t ePart1, const carb::Float3& dir,
           float distance, bool bothSides) {
            const uint64_t part0 = uint64_t(ePart0);
            const uint64_t part1 = uint64_t(ePart1);
            const uint64_t uintPath = part0 + (part1 << 32);
            return self->sweepShapeAny(uintPath, dir, distance, bothSides);
        },
        docString, py::arg("meshPath0"), py::arg("meshPath1"), py::arg("dir"), py::arg("distance"),
        py::arg("bothSides") = false);

    docString = R"(
        Sphere sweep physics scene for all collisions.

        Args:
            radius: Sphere radius
            origin: Origin of the sweep.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            reportFn: Report function where SweepHits will be reported, return True to continue traversal, False to stop traversal
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Returns true if hit
    )";
    dev.def("sweep_sphere_all",
        [](const IPhysxSceneQuery* self, float radius, const carb::Float3& origin, const carb::Float3& dir,
            float distance, std::function<bool(const SweepHit& hit)> reportFn, bool bothSides) {
        return self->sweepSphereAll(radius, origin, dir, distance, carb::wrapPythonCallback(std::move(reportFn)), bothSides);
    }, docString,
        py::arg("radius"), py::arg("origin"), py::arg("dir"), py::arg("distance"), py::arg("reportFn"), py::arg("bothSides") = false);

    docString = R"(
        Box sweep physics scene for all collisions.

        Args:
            halfExtent: Box half extent.
            pos: Origin of the sweep (barycenter of the box).
            rot: Rotation of the box (quat x, y, z, w)
            dir: Unit direction of the sweep.
            distance: sweep distance.
            reportFn: Report function where SweepHits will be reported, return True to continue traversal, False to stop traversal
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Returns true if hit
    )";
    dev.def("sweep_box_all",
        [](const IPhysxSceneQuery* self, const carb::Float3& halfExtent, const carb::Float3& pos,
           const carb::Float4& rot, const carb::Float3& dir, float distance, std::function<bool(const SweepHit& hit)> reportFn, bool bothSides) {
        return self->sweepBoxAll(halfExtent, pos, rot, dir, distance, carb::wrapPythonCallback(std::move(reportFn)), bothSides);
    }, docString,
        py::arg("halfExtent"), py::arg("pos"), py::arg("rot"), py::arg("dir"), py::arg("distance"), py::arg("reportFn"), py::arg("bothSides") = false);

    docString = R"(
        Sweep test of a UsdGeom.Mesh against objects in the physics scene. Sweep test will use convex mesh
        approximation of the input mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            mesh: UsdGeom.Mesh path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            reportFn: Report function where SweepHits will be reported, return True to continue traversal, False to stop traversal
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Returns true if hit
    )";
    dev.def("sweep_mesh_all",
        [](const IPhysxSceneQuery* self, uint32_t ePart0, uint32_t ePart1, const carb::Float3& dir, float distance, std::function<bool(const SweepHit& hit)> reportFn, bool bothSides) {
            const uint64_t part0 = uint64_t(ePart0);
            const uint64_t part1 = uint64_t(ePart1);
            const uint64_t uintPath = part0 + (part1 << 32);
            return self->sweepMeshAll(uintPath, dir, distance, carb::wrapPythonCallback(std::move(reportFn)), bothSides);
    }, docString,
        py::arg("meshPath0"), py::arg("meshPath1"), py::arg("dir"), py::arg("distance"), py::arg("reportFn"), py::arg("bothSides") = false);

    docString = R"(
        Sweep test of a UsdGeom.GPrim against objects in the physics scene. Sweep test will use convex mesh
        approximation if the input is a mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            gprim: UsdGeom.GPrim path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            reportFn: Report function where SweepHits will be reported, return True to continue traversal, False to stop traversal
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Returns true if hit
    )";
    dev.def("sweep_shape_all",
        [](const IPhysxSceneQuery* self, uint32_t ePart0, uint32_t ePart1, const carb::Float3& dir, float distance, std::function<bool(const SweepHit& hit)> reportFn, bool bothSides) {
            const uint64_t part0 = uint64_t(ePart0);
            const uint64_t part1 = uint64_t(ePart1);
            const uint64_t uintPath = part0 + (part1 << 32);
            return self->sweepShapeAll(uintPath, dir, distance, carb::wrapPythonCallback(std::move(reportFn)), bothSides);
    }, docString,
        py::arg("meshPath0"), py::arg("meshPath1"), py::arg("dir"), py::arg("distance"), py::arg("reportFn"), py::arg("bothSides") = false);

    docString = R"(
        Overlap test of a sphere against objects in the physics scene.

        Args:
            radius: Sphere radius.
            pos: Origin of the sphere overlap.
            reportFn: Report function where SceneQueryHits will be reported, return True to continue traversal, False to stop traversal
            anyHit: Boolean defining whether overlap should report only bool hit (0 no hit, 1 hit found).

        Returns:
            Return number of hits founds
    )";
    dev.def("overlap_sphere",
        [](const IPhysxSceneQuery* self, float radius, const carb::Float3& pos, std::function<bool(const OverlapHit& hit)> reportFn, bool anyHit) {
        return self->overlapSphere(radius, pos, carb::wrapPythonCallback(std::move(reportFn)), anyHit);
    }, docString,
        py::arg("radius"), py::arg("pos"), py::arg("reportFn"), py::arg("anyHit")=false);

    docString = R"(
        Overlap test of a sphere against objects in the physics scene, reports only boolean.

        Args:
            radius: Sphere radius.
            pos: Origin of the sphere overlap.

        Returns:
            Returns True if overlap found.
    )";
    dev.def(
        "overlap_sphere_any",
        [](const IPhysxSceneQuery* self, float radius, const carb::Float3& pos)
        {
            return self->overlapSphereAny(radius, pos);
        },
        docString, py::arg("radius"), py::arg("pos"));

    docString = R"(
        Overlap test of a box against objects in the physics scene.

        Args:
            halfExtent: Box half extent.
            pos: Origin of the box overlap (barycenter of the box).
            rot: Rotation of the box overlap (quat x, y, z, w)
            reportFn: Report function where SceneQueryHits will be reported, return True to continue traversal, False to stop traversal
            anyHit: Boolean defining whether overlap should report only bool hit (0 no hit, 1 hit found).

        Returns:
            Return number of hits founds
    )";
    dev.def(
        "overlap_box",
        [](const IPhysxSceneQuery* self, const carb::Float3& halfExtent, const carb::Float3& pos,
           const carb::Float4& rot, std::function<bool(const OverlapHit& hit)> reportFn, bool anyHit) {
            return self->overlapBox(halfExtent, pos, rot, carb::wrapPythonCallback(std::move(reportFn)), anyHit);
        },
        docString, py::arg("halfExtent"), py::arg("pos"), py::arg("rot"), py::arg("reportFn"), py::arg("anyHit") = false);

    docString = R"(
        Overlap test of a box against objects in the physics scene, reports only boolean

        Args:
            extent: Box extent.
            pos: Origin of the box overlap.
            rot: Rotation of the box overlap (quat x, y, z, w)

        Returns:
            Returns True if overlap found
    )";
    dev.def("overlap_box_any",
        [](const IPhysxSceneQuery* self, const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot) {
        return self->overlapBoxAny(halfExtent, pos, rot);
    }, docString,
        py::arg("halfExtent"), py::arg("pos"), py::arg("rot"));

    docString = R"(
        Overlap test of a UsdGeom.Mesh against objects in the physics scene. Overlap test will use convex mesh
        approximation of the input mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            mesh: UsdGeom.Mesh path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            reportFn: Report function where SceneQueryHits will be reported, return True to continue traversal, False to stop traversal
            anyHit: Boolean defining whether overlap should report only bool hit (0 no hit, 1 hit found).

        Returns:
            Return number of hits founds
    )";
    dev.def("overlap_mesh",
        [](const IPhysxSceneQuery* self, uint32_t ePart0, uint32_t ePart1, std::function<bool(const OverlapHit& hit)> reportFn, bool anyHit)
        {
            const uint64_t part0 = uint64_t(ePart0);
            const uint64_t part1 = uint64_t(ePart1);
            const uint64_t uintPath = part0 + (part1 << 32);

            return self->overlapMesh(uintPath, carb::wrapPythonCallback(std::move(reportFn)), anyHit);
        }
        , docString,
        py::arg("meshPath0"), py::arg("meshPath1"), py::arg("reportFn"), py::arg("anyHit") = false);

    docString = R"(
        Overlap test of a UsdGeom.Mesh against objects in the physics scene. Overlap test will use convex mesh
        approximation of the input mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow. Reports only boolean.

        Args:
            mesh: UsdGeom.Mesh path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.

        Returns:
            Returns True if overlap was found.
    )";
    dev.def(
        "overlap_mesh_any",
        [](const IPhysxSceneQuery* self, uint32_t ePart0, uint32_t ePart1)
        {
            const uint64_t part0 = uint64_t(ePart0);
            const uint64_t part1 = uint64_t(ePart1);
            const uint64_t uintPath = part0 + (part1 << 32);

            return self->overlapMeshAny(uintPath);
        },
        docString, py::arg("meshPath0"), py::arg("meshPath1"));

    docString = R"(
        Overlap test of a UsdGeom.GPrim against objects in the physics scene. Overlap test will use convex mesh
        approximation if the input is a mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            gprim: UsdGeom.GPrim path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            reportFn: Report function where SceneQueryHits will be reported, return True to continue traversal, False to stop traversal
            anyHit: Boolean defining whether overlap should report only bool hit (0 no hit, 1 hit found).

        Returns:
            Return number of hits founds
    )";
    dev.def("overlap_shape",
        [](const IPhysxSceneQuery* self, uint32_t ePart0, uint32_t ePart1, std::function<bool(const OverlapHit& hit)> reportFn, bool anyHit)
        {
            const uint64_t part0 = uint64_t(ePart0);
            const uint64_t part1 = uint64_t(ePart1);
            const uint64_t uintPath = part0 + (part1 << 32);

            return self->overlapShape(uintPath, carb::wrapPythonCallback(std::move(reportFn)), anyHit);
        }
        , docString,
            py::arg("meshPath0"), py::arg("meshPath1"), py::arg("reportFn"), py::arg("anyHit") = false);

    docString = R"(
        Overlap test of a UsdGeom.GPrim against objects in the physics scene. Overlap test will use convex mesh
        approximation if the input is a mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow. Reports only boolean.

        Args:
            gprim: UsdGeom.GPrim path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.

        Returns:
            Returns True if overlap was found.
    )";
    dev.def(
        "overlap_shape_any",
        [](const IPhysxSceneQuery* self, uint32_t ePart0, uint32_t ePart1)
        {
            const uint64_t part0 = uint64_t(ePart0);
            const uint64_t part1 = uint64_t(ePart1);
            const uint64_t uintPath = part0 + (part1 << 32);

            return self->overlapShapeAny(uintPath);
        },
        docString, py::arg("meshPath0"), py::arg("meshPath1"));

//////////////////////////////////////////////////////////////////////////

    dev = defineInterfaceClass<IPhysxUnitTests>(
        m, "PhysXUnitTests", "acquire_physxunittests_interface", "release_physxunittests_interface");
    dev.doc() = R"(
        This interface is the access point to omni.physx test support functions.
    )";

    docString = R"(
        Update physics simulation by one step.

        Args:
            elapsedStep:  Simulation step time (seconds), time elapsed between last step and this step.
            currentTime:  Current time (seconds), might be used for time sampled transformations to apply.
    )";
    dev.def("update", wrapInterfaceFunction(&IPhysxUnitTests::update), docString, py::arg("elapsedStep"), py::arg("currentTime"));

    docString = R"(
        Returns simulation statistics after a simulation step.

        Returns:
            Return a dictionary with statistics::

                'numDynamicRigids': int - Number of dynamic rigids in simulation.
                'numStaticRigids': int - Number of static rigids in simulation.
                'numKinematicBodies': int - Number of kinematic rigids in simulation.
                'numArticulations': int - Number of articulations in simulation.
                'numSphereShapes': int - Number of sphere shapes in simulation.
                'numBoxShapes': int - Number of box shapes in simulation.
                'numCapsuleShapes': int - Number of capsule shapes in simulation.
                'numCylinderShapes': int - Number of cylinder shapes in simulation.
                'numConvexShapes': int - Number of convex shapes in simulation.
                'numTriMeshShapes': int - Number of triangle mesh shapes in simulation.
                'numPlaneShapes': int - Number of plane shapes in simulation.
                'numConeShapes': int - Number of cone shapes in simulation.
    )";
    dev.def("get_physics_stats",
             [](const IPhysxUnitTests* self) {
                 PhysicsStats stats = self->getPhysicsStats();
                 // Return as dict
                 py::dict info;
                 info["numDynamicRigids"] = stats.numDynamicRigids;
                 info["numStaticRigids"] = stats.numStaticRigids;
                 info["numKinematicBodies"] = stats.numKinematicBodies;
                 info["numArticulations"] = stats.numArticulations;
                 info["numSphereShapes"] = stats.numSphereShapes;
                 info["numBoxShapes"] = stats.numBoxShapes;
                 info["numCapsuleShapes"] = stats.numCapsuleShapes;
                 info["numCylinderShapes"] = stats.numCylinderShapes;
                 info["numConvexShapes"] = stats.numConvexShapes;
                 info["numTriMeshShapes"] = stats.numTriMeshShapes;
                 info["numPlaneShapes"] = stats.numPlaneShapes;
                 info["numConeShapes"] = stats.numConeShapes;
                 info["numConstraints"] = stats.numConstraints;

                 return info;
             }, docString);

    dev.def("start_logger_check", wrapInterfaceFunction(&IPhysxUnitTests::startLoggerCheck));
    dev.def("start_logger_check_for_multiple", wrapInterfaceFunction(&IPhysxUnitTests::startLoggerCheckForMultiple));
    dev.def("end_logger_check", wrapInterfaceFunction(&IPhysxUnitTests::endLoggerCheck));
    dev.def("is_cuda_lib_present", wrapInterfaceFunction(&IPhysxUnitTests::isCudaLibPresent));

    docString = R"(
        Returns mass information for a given body.

        Args:
            body_path - USD path to body.

        Returns:
            Return a dictionary with mass information::

                'mass': float - Mass of the body.
                'inertia': float3 - Inertia tensor of body.
                'com': float3 - Center of mass of the body.
    )";
    dev.def("get_mass_information", [](const IPhysxUnitTests* self, const char* body_path) {
            Float3 inertia;
            Float3 com;
            float mass = self->getMassInformation(body_path, inertia, com);
            // Return as dict
            py::dict info;
            info["mass"] = mass;
            info["inertia"] = inertia;
            info["com"] = com;

            return info;
        }, docString, py::arg("body_path"));

    docString = R"(
        Returns material paths for materials found on given collider path.

        Args:
            collider_path - USD path to a collider.

        Returns:
            Return a list of material paths
    )";
    dev.def("get_materials_paths", [](const IPhysxUnitTests* self, const char* collider_path) {
        std::vector<pxr::SdfPath> materialPaths;
        self->getMaterialsPaths(pxr::SdfPath(collider_path), materialPaths);
        // Return as list
        py::list info;
        for (size_t i = 0; i < materialPaths.size(); i++)
        {
            info.append(materialPaths[i].GetText());
        }
        return info;
    }, docString, py::arg("collider_path"));

    dev = defineInterfaceClass<IPhysxVisualization>(
        m, "PhysXVisualization", "acquire_physx_visualization_interface", "release_physx_visualization_interface");
    dev.doc() = R"(
        This interface is the access point to PhysX SDK debug visualization.
    )";

    docString = R"(
        Enable/disable PhysX debug visualization.

        Args:
            enable - Bool if enable or disable.
    )";
    dev.def("enable_visualization", wrapInterfaceFunction(&IPhysxVisualization::enableVisualization), docString, py::arg("enable"));

    docString = R"(
        Set PhysX debug visualization scale.

        Args:
            scale - Float value for scaling debug visualization.
    )";
    dev.def("set_visualization_scale", wrapInterfaceFunction(&IPhysxVisualization::setVisualizationScale), docString, py::arg("scale"));

    docString = R"(
        Toggle individual debug visualization features.

        Args:
            debug_vis - Debug visualization feature string identifier, can be one of the following:
                {'WorldAxes', 'BodyAxes', 'BodyMassAxes', 'BodyLinearVel', 'BodyAngularVel', 'ContactPoint',
                'ContactNormal', 'ContactError', 'ContactImpulse', 'FrictionPoint', 'FrictionNormal',
                'FrictionImpulse', 'ActorAxes', 'CollisionAABBs', 'CollisionShapes', 'CollisionAxes',
                'CollisionCompounds', 'CollisionEdges', 'CollisionStaticPruner', 'CollisionDynamicPruner',
                'JointLocalFrames', 'JointLimits', 'CullBox', 'MBPRegions'}
            enable - Bool to enable/disable the feature.
    )";
    dev.def("set_visualization_parameter", [](const IPhysxVisualization* self, const char* parameter, bool value) {
            PhysXVisualizationParameter visPar = eNone;
            if (strstr(parameter, "WorldAxes"))
            {
                visPar = eWorldAxes;
            }
            else if (strstr(parameter, "BodyAxes"))
            {
                visPar = eBodyAxes;
            }
            else if (strstr(parameter, "BodyMassAxes"))
            {
                visPar = eBodyMassAxes;
            }
            else if (strstr(parameter, "BodyLinearVel"))
            {
                visPar = eBodyLinearVelocity;
            }
            else if (strstr(parameter, "BodyAngularVel"))
            {
                visPar = eBodyAngularVelocity;
            }
            else if (strstr(parameter, "ContactPoint"))
            {
                visPar = eContactPoint;
            }
            else if (strstr(parameter, "ContactNormal"))
            {
                visPar = eContactNormal;
            }
            else if (strstr(parameter, "ContactError"))
            {
                visPar = eContactError;
            }
            else if (strstr(parameter, "ContactImpulse") || strstr(parameter, "ContactForce")) // ContactForce is deprecated
            {
                visPar = eContactImpulse;
            }
            else if (strstr(parameter, "FrictionPoint"))
            {
                visPar = eFrictionPoint;
            }
            else if (strstr(parameter, "FrictionNormal"))
            {
                visPar = eFrictionNormal;
            }
            else if (strstr(parameter, "FrictionImpulse"))
            {
                visPar = eFrictionImpulse;
            }
            else if (strstr(parameter, "ActorAxes"))
            {
                visPar = eActorAxes;
            }
            else if (strstr(parameter, "CollisionAABBs"))
            {
                visPar = eCollisionAABBs;
            }
            else if (strstr(parameter, "CollisionShapes"))
            {
                visPar = eCollisionShapes;
            }
            else if (strstr(parameter, "CollisionAxes"))
            {
                visPar = eCollisionAxes;
            }
            else if (strstr(parameter, "CollisionCompounds"))
            {
                visPar = eCollisionCompounds;
            }
            else if (strstr(parameter, "CollisionFaceNormals"))
            {
                visPar = eCollisionFaceNormals;
            }
            else if (strstr(parameter, "CollisionEdges"))
            {
                visPar = eCollisionEdges;
            }
            else if (strstr(parameter, "CollisionStaticPruner"))
            {
                visPar = eCollisionStaticPruner;
            }
            else if (strstr(parameter, "CollisionDynamicPruner"))
            {
                visPar = eCollisionDynamicPruner;
            }
            else if (strstr(parameter, "JointLocalFrames"))
            {
                visPar = eJointLocalFrames;
            }
            else if (strstr(parameter, "JointLimits"))
            {
                visPar = eJointLimits;
            }
            else if (strstr(parameter, "CullBox"))
            {
                visPar = eCullBox;
            }
            else if (strstr(parameter, "MBPRegions"))
            {
                visPar = eMBPRegions;
            }
            else if (strstr(parameter, "SDFs"))
            {
                visPar = eSDF;
            }

            self->setVisualizationParameter(visPar, value);
        }, docString, py::arg("debug_vis"), py::arg("enable"));

    docString = R"(
    Get number of PhysX debug visualization lines. This serves mostly as a test function.
    )";
    dev.def("get_nb_lines", wrapInterfaceFunction(&IPhysxVisualization::getNbLines), docString);

    //////////////////////////////////////////////////////////////////////////
    // IPhysxSimulation interface
    py::enum_<ContactEventType::Enum>(m, "ContactEventType", R"(
        Contact event type.
        )")
        .value("CONTACT_FOUND", ContactEventType::eCONTACT_FOUND, "Contact found.")
        .value("CONTACT_LOST", ContactEventType::eCONTACT_LOST, "Contact lost.")
        .value("CONTACT_PERSIST", ContactEventType::eCONTACT_PERSIST, "Contact persist.");

    py::class_<ContactEventHeader>(m, "ContactEventHeader", R"(
            Contact event header.
        )")
        .def_readonly("type", &ContactEventHeader::type, R"(Contact event type.)")
        .def_readonly("stage_id", &ContactEventHeader::stageId, R"(Stage id.)")
        .def_readonly("actor0", &ContactEventHeader::actor0, R"(Actor0 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.)")
        .def_readonly("actor1", &ContactEventHeader::actor1, R"(Actor1 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.)")
        .def_readonly("collider0", &ContactEventHeader::collider0, R"(Collider0 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.)")
        .def_readonly("collider1", &ContactEventHeader::collider1, R"(Collider1 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.)")
        .def_readonly("contact_data_offset", &ContactEventHeader::contactDataOffset, R"(Contact data offset.)")
        .def_readonly("num_contact_data", &ContactEventHeader::numContactData, R"(Number of contact data.)")
        .def_readonly("friction_anchors_offset", &ContactEventHeader::frictionAnchorsDataOffset, R"(Friction anchors data offset.)")
        .def_readonly("num_friction_anchors_data", &ContactEventHeader::numfrictionAnchorsData, R"(Number of contact data.)")
        .def_readonly("proto_index0", &ContactEventHeader::protoIndex0, R"(Protoindex0 from a point instancer (0xFFFFFFFF means collider is not part of an instancer).)")
        .def_readonly("proto_index1", &ContactEventHeader::protoIndex1, R"(Protoindex1 from a point instancer (0xFFFFFFFF means collider is not part of an instancer).)");

    py::class_<ContactData>(m, "ContactData", R"(
            Contact data.
        )")
        .def_readonly("position", &ContactData::position, R"(Contact position.)")
        .def_readonly("normal", &ContactData::normal, R"(Contact normal.)")
        .def_readonly("impulse", &ContactData::impulse, R"(Contact impulse)")
        .def_readonly("separation", &ContactData::separation, R"(Contact separation value.)")
        .def_readonly("face_index0", &ContactData::faceIndex0, R"(Face index 0 - non zero only for meshes)")
        .def_readonly("face_index1", &ContactData::faceIndex1, R"(Face index 1 - non zero only for meshes.)")
        .def_readonly("material0", &ContactData::material0, R"(Material0 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.)")
        .def_readonly("material1", &ContactData::material1, R"(Material1 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.)");

    py::class_<FrictionAnchor>(m, "FrictionAnchor", R"(
            Contact data.
        )")
        .def_readonly("position", &FrictionAnchor::position, R"(Contact position.)")
        .def_readonly("impulse", &FrictionAnchor::impulse, R"(Contact impulse)");

    py::bind_vector<ContactEventHeaderVector>(m, "ContactEventHeaderVector", py::module_local(false));
    py::bind_vector<ContactDataVector>(m, "ContactDataVector", py::module_local(false));
    py::bind_vector<FrictionAnchorsDataVector>(m, "FrictionAnchorsDataVector", py::module_local(false));

    dev = defineInterfaceClass<IPhysxSimulation>(m, "IPhysxSimulation", "acquire_physx_simulation_interface", "release_physx_simulation_interface");
    dev.doc() = R"(
        This interface is the access point to the omni.physx simulation control.
    )";

    docString = R"(
        Attach USD stage. This will run the physics parser
        and will populate the PhysX SDK with the corresponding simulation objects.

        Note: previous stage will be detached.

        Args:
            stage_id: USD stageId (can be retrieved from a stagePtr - UsdUtils.StageCache.Get().GetId(stage).ToLongInt())

        Returns:
            Returns true if stage was successfully attached.
    )";
    dev.def("attach_stage", wrapInterfaceFunction(&IPhysxSimulation::attachStage), docString, py::arg("stage_id"));

    docString = R"(
        Detach USD stage, this will remove all objects from the PhysX SDK
    )";
    dev.def("detach_stage", wrapInterfaceFunction(&IPhysxSimulation::detachStage), docString);

    docString = R"(
    Returns the currently attached stage through the IPhysxSimulation interface

    Returns:
        Returns USD stage id.
    )";
    dev.def("get_attached_stage", wrapInterfaceFunction(&IPhysxSimulation::getAttachedStage), docString);

    docString = R"(
        Execute physics asynchronous simulation.

        Args:
            elapsed_time: Simulation time (seconds).
            current_time: Current time (seconds), might be used for time sampled transformations to apply.

    )";
    dev.def("simulate", wrapInterfaceFunction(&IPhysxSimulation::simulate), docString, py::arg("elapsed_time"), py::arg("current_time"), py::call_guard<py::gil_scoped_release>());

    docString = R"(
        Fetch simulation results.
        Writing out simulation results based on physics settings.

        Note This is a blocking call. The function will wait until the simulation is finished.
    )";
    dev.def("fetch_results", wrapInterfaceFunction(&IPhysxSimulation::fetchResults), docString);

    docString = R"(
        Check if simulation finished.

        Returns:
            Returns true if simulation has finished.
    )";
    dev.def("check_results", wrapInterfaceFunction(&IPhysxSimulation::checkResults), docString);

    docString = R"(
            Subscribes to contact report callback function.

            The contact buffer data are available for one simulation step

            Args:
                fn: The callback to be called after simulation step to receive contact reports.

            Returns:
                The subscription holder.
    )";
    dev.def(
        "subscribe_contact_report_events",
        [](IPhysxSimulation* iface, const notify_contact_report_function& fn) {
            notify_contact_report_function* funcCopy = new notify_contact_report_function(fn);
            auto id = gMarshalCallbacks.subscribeContactReportCallback(iface, *funcCopy);
            auto subscription = std::make_shared<Subscription>([=]() {
                gMarshalCallbacks.unsubscribeContactReportCallback(iface, id);
                delete funcCopy;
            });
            return subscription;
        },
        docString, py::arg("fn"));

        docString = R"(
            Subscribes to contact report callback function including friction anchors.

            The contact buffer data are available for one simulation step

            Args:
                fn: The callback to be called after simulation step to receive contact reports.

            Returns:
                The subscription holder.
    )";
    dev.def(
        "subscribe_full_contact_report_events",
        [](IPhysxSimulation* iface, const notify_full_contact_report_function& fn) {
            notify_full_contact_report_function* funcCopy = new notify_full_contact_report_function(fn);
            auto id = gMarshalCallbacks.subscribeContactReportFullCallback(iface, *funcCopy);
            auto subscription = std::make_shared<Subscription>([=]() {
                gMarshalCallbacks.unsubscribeContactReportFullCallback(iface, id);
                delete funcCopy;
            });
            return subscription;
        },
        docString, py::arg("fn"));

    docString = R"(
            Get contact report data for current simulation step directly.

            The contact buffer data are available for one simulation step

            Returns:
                Tuple with contact event vector and contact data vector::

                'contact_headers': vector of contact event headers
                'contact_data': vector of contact data
                'friction_anchors': vector of friction anchors data

    )";
    dev.def(
        "get_contact_report",
        [](IPhysxSimulation* iface) {
            const ContactEventHeaderVector* contactEventHeaderVector = nullptr;
            const ContactDataVector* dataVector = nullptr;
            const FrictionAnchorsDataVector* frictionAnchorsVector = nullptr;

            gMarshalCallbacks.getContactReportTuple(iface, &contactEventHeaderVector, &dataVector);

            return py::make_tuple(*contactEventHeaderVector, *dataVector);
        },
        docString);

    docString = R"(
            Get contact report data for current simulation step directly, including friction anchors.

            The contact buffer data are available for one simulation step

            Returns:
                Tuple with contact event vector and contact data vector::

                'contact_headers': vector of contact event headers
                'contact_data': vector of contact data
                'friction_anchors': vector of friction anchors data

    )";
    dev.def(
        "get_full_contact_report",
        [](IPhysxSimulation* iface) {
            const ContactEventHeaderVector* contactEventHeaderVector = nullptr;
            const ContactDataVector* dataVector = nullptr;
            const FrictionAnchorsDataVector* frictionAnchorsVector = nullptr;

            gMarshalCallbacks.getFullContactReportTuple(iface, &contactEventHeaderVector, &dataVector, &frictionAnchorsVector);

            return py::make_tuple(*contactEventHeaderVector, *dataVector, *frictionAnchorsVector);
        },
        docString);

    docString = R"(
            Execute the physics simulation on a specific scene.

            The PhysX simulation in the scene will simulate the exact elapsedTime passed. No substepping will happen.
            It is the caller's responsibility to provide a reasonable elapsedTime.
            In general it is recommended to use fixed size time steps with a maximum of 1/60 of a second.
            If scenePath is empty, it behaves like IPhysxSimulation::simulate

            scenePath       uint64_t        Scene USD path use PhysicsSchemaTools::sdfPathToInt
            elapsedTime     float           Simulation time (seconds).
            currentTime     float           Current time (seconds), might be used for time sampled transformations to apply.
    )";
    dev.def("simulate_scene", wrapInterfaceFunction(&IPhysxSimulation::simulateScene), docString, py::arg("scene_path"),
            py::arg("elapsed_time"), py::arg("current_time"));

    docString = R"(
            Fetch simulation scene results and writes out simulation results based on physics settings for
            a specific scene. Disabling a scene has no effect on this function.
            If scenePath is empty, it behaves like IPhysxSimulation::fetchResults

            Note: this is a blocking call. The function will wait until the simulation scene is finished.

            scenePath       uint64_t        Scene USD path use PhysicsSchemaTools::sdfPathToInt
    )";
    dev.def("fetch_results_scene", wrapInterfaceFunction(&IPhysxSimulation::fetchResultsScene), docString, py::arg("scene_path"));

    docString = R"(
            Check if a simulation scene is finished. Disabling a scene has no effect on this function.
            If scenePath is empty, it behaves like IPhysxSimulation::checkResults

            Returns True if the simulation scene is finished.

            scenePath       uint64_t        Scene USD path use PhysicsSchemaTools::sdfPathToInt
    )";
    dev.def("check_results_scene", wrapInterfaceFunction(&IPhysxSimulation::checkResultsScene), docString, py::arg("scene_path"));

    docString = R"(
            Flush changes will force physics to process buffered changes

            Changes to physics gets buffered, in some cases flushing changes is required if order is required.

            Example - prim A gets added. Existing prim B has a relationship that gets switched to use A. Currently,
            the relationship change gets processed immediately and fails because prim A only gets added at the
            start of the next sim step.
    )";
    dev.def("flush_changes", wrapInterfaceFunction(&IPhysxSimulation::flushChanges), docString);

    docString = R"(
            Pause fabric change tracking for physics listener.

            Args:
                pause bool Pause or unpause change tracking.
    )";
    dev.def("pause_change_tracking", wrapInterfaceFunction(&IPhysxSimulation::pauseChangeTracking), docString, py::arg("pause"));

    docString = R"(
            Check if fabric change tracking for physics listener is paused or not

            Returns true if paused change tracking.
    )";
    dev.def("is_change_tracking_paused", wrapInterfaceFunction(&IPhysxSimulation::isChangeTrackingPaused), docString);

    docString = R"(
        Applies force at given body with given force position.

        Args:
            stage_id: USD stageId
            body_path: USD path of the body encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            force: Force to apply to the body.
            pos: World position where the force is applied.
            mode: Supporting various modes - Force (default), Impulse, Velocity, Acceleration.
    )";
    dev.def("apply_force_at_pos",
        [](const IPhysxSimulation* self, uint64_t stageId, uint64_t bodyPath, const carb::Float3& force, const carb::Float3& pos,
            const char* mode) {
                std::string smode = mode;
                ForceModeType::Enum fMode = ForceModeType::eFORCE;
                if (smode == "Impulse")
                    fMode = ForceModeType::eIMPULSE;
                else if (smode == "Velocity")
                    fMode = ForceModeType::eVELOCITY_CHANGE;
                else if (smode == "Acceleration")
                    fMode = ForceModeType::eACCELERATION;

                self->addForceAtPos(stageId, bodyPath, force, pos, fMode);
        }, docString,
        py::arg("stage_id"), py::arg("body_path"), py::arg("force"), py::arg("pos"), py::arg("mode") = "Force");

    docString = R"(
        Applies force at given point instancer body with given force position.

        Args:
            stage_id: USD stageId
            point_instancer_path: USD path to a point instancer encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            force: Force to apply to the target.
            pos: World position where the force is applied.
            mode: Supporting various modes - Force (default), Impulse, Velocity, Acceleration.
            protoIndex: If protoIndex is 0xffffffff (default), force will be applied to all instances,
                        otherwise it will only be applied to the instance at this index.
    )";
    dev.def("apply_force_at_pos_instanced",
        [](const IPhysxSimulation* self, uint64_t stageId, uint64_t pointInstancerPath, const carb::Float3& force, const carb::Float3& pos,
            const char* mode, uint32_t protoIndex) {
                std::string smode = mode;
                ForceModeType::Enum fMode = ForceModeType::eFORCE;
                if (smode == "Impulse")
                    fMode = ForceModeType::eIMPULSE;
                else if (smode == "Velocity")
                    fMode = ForceModeType::eVELOCITY_CHANGE;
                else if (smode == "Acceleration")
                    fMode = ForceModeType::eACCELERATION;

                self->addForceAtPosInstanced(stageId, pointInstancerPath, force, pos, fMode, protoIndex);
        }, docString,
        py::arg("stage_id"), py::arg("point_instancer_path"), py::arg("force"), py::arg("pos"), py::arg("mode") = "Force", py::arg("proto_index") = 0xffffffff);

    docString = R"(
        Applies torque at given body at body center of mass.

        Args:
            stage_id: USD stageId
            body_path: USD path of the body encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            torque: Torque to apply to the body.
    )";
    dev.def("apply_torque",
        [](const IPhysxSimulation* self, uint64_t stageId, uint64_t bodyPath, const carb::Float3& torque) {
                self->addTorque(stageId, bodyPath, torque);
        }, docString,
        py::arg("stage_id"), py::arg("body_path"), py::arg("torque"));

    docString = R"(
        Applies torque at given point instancer body at body center of mass.

        Args:
            stage_id: USD stageId
            point_instancer_path: USD path to a point instancer encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            torque: Torque to apply to the body.
            protoIndex: If protoIndex is 0xffffffff (default), torque will be applied to all instances,
                        otherwise it will only be applied to the instance at this index.
    )";
    dev.def("apply_torque_instanced",
        [](const IPhysxSimulation* self, uint64_t stageId, uint64_t pointInstancerPath, const carb::Float3& torque, uint32_t protoIndex) {
                self->addTorqueInstanced(stageId, pointInstancerPath, torque, protoIndex);
        }, docString,
        py::arg("stage_id"), py::arg("body_path"), py::arg("torque"), py::arg("proto_index") = 0xffffffff);

    docString = R"(
        Put body to sleep.

        Args:
            stage_id: USD stageId
            body_path: USD path of the body encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
    )";
    dev.def("put_to_sleep", [](const IPhysxSimulation* self, uint64_t stageId, uint64_t bodyPath) { self->putToSleep(stageId, bodyPath); }, docString,
        py::arg("stage_id"), py::arg("body_path"));

    docString = R"(
        Put point instancer body to sleep.

        Args:
            stage_id: USD stageId
            point_instancer_path: USD path to a point instancer encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            protoIndex: If protoIndex is 0xffffffff (default), all instances will be put to sleep,
                        otherwise it will only be applied to the instance at this index.
    )";
    dev.def("put_to_sleep_instanced", [](const IPhysxSimulation* self, uint64_t stageId, uint64_t pointInstancerPath, uint32_t protoIndex) {
                self->putToSleepInstanced(stageId, pointInstancerPath, protoIndex); }, docString,
        py::arg("stage_id"), py::arg("body_path"), py::arg("proto_index") = 0xffffffff);

    docString = R"(
        Wake up body.

        Args:
            stage_id: USD stageId
            body_path: USD path of the body encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
    )";
    dev.def("wake_up", [](const IPhysxSimulation* self, uint64_t stageId, uint64_t bodyPath) { self->wakeUp(stageId, bodyPath); }, docString,
        py::arg("stage_id"), py::arg("body_path"));

    docString = R"(
        Wake up point instancer body.

        Args:
            stage_id: USD stageId
            point_instancer_path: USD path to a point instancer encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            protoIndex: If protoIndex is 0xffffffff (default), all instances will be awakened,
                        otherwise it will only be applied to the instance at this index.
    )";
    dev.def("wake_up_instanced", [](const IPhysxSimulation* self, uint64_t stageId, uint64_t pointInstancerPath, uint32_t protoIndex) {
                self->wakeUpInstanced(stageId, pointInstancerPath, protoIndex); }, docString,
        py::arg("stage_id"), py::arg("body_path"), py::arg("proto_index") = 0xffffffff);

    docString = R"(
        Is body sleeping.

        Args:
            stage_id: USD stageId
            body_path: USD path of the body encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt

        Returns:
            True if body is asleeep
    )";
    dev.def("is_sleeping", [](const IPhysxSimulation* self, uint64_t stageId, uint64_t bodyPath) { return self->isSleeping(stageId, bodyPath); }, docString,
        py::arg("stage_id"), py::arg("body_path"));

    docString = R"(
        Is point instancer bodypoint instancer body sleeping.

        Args:
            stage_id: USD stageId
            point_instancer_path: USD path to a point instancer encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            protoIndex: Check against the instance at this index.

        Returns:
            True if body is asleeep
    )";
    dev.def("is_sleeping_instanced", [](const IPhysxSimulation* self, uint64_t stageId, uint64_t pointInstancerPath, uint32_t protoIndex) {
                return self->isSleepingInstanced(stageId, pointInstancerPath, protoIndex); }, docString,
        py::arg("stage_id"), py::arg("body_path"), py::arg("proto_index"));

    struct TriggerEventWrappedCallbacks
    {
        std::function<void(const TriggerEventData&)> trigger_report_fn;
        SubscriptionId triggerSubscriptionId;
    };
    struct TriggerEventsMarshalCallbacks
    {
        EventSubscriptionRegistry<TriggerEventWrappedCallbacks> callbacksRegistry;
    };

    static TriggerEventsMarshalCallbacks triggerEventsMarshalCallbacks;

    py::enum_<TriggerEventType::Enum>(m, "TriggerEventType", R"(
        Trigger Event type.
        )")
    .value("TRIGGER_ON_ENTER", TriggerEventType::eTRIGGER_ON_ENTER, "The collider has entered trigger volume")
    .value("TRIGGER_ON_LEAVE", TriggerEventType::eTRIGGER_ON_LEAVE, "The collider has left trigger volume");

    py::class_<TriggerEventData>(m, "TriggerEventData", R"(
        Parameters for trigger event callback.
    )")
    .def_readonly("stage_id", &TriggerEventData::stageId, R"(USD Stage)")
    .def_readonly("subscription_id", &TriggerEventData::subscriptionId, R"(Id of the subscription returned by subscribe_physics_trigger_report_events)")
    .def_readonly("other_collider_prim_id", &TriggerEventData::otherColliderPrimId, R"(USD Path of other prim entering trigger)")
    .def_readonly("trigger_collider_prim_id", &TriggerEventData::triggerColliderPrimId, R"(USD Path of prim representing the trigger)")
    .def_readonly("other_body_prim_id", &TriggerEventData::otherBodyPrimId, R"(USD Path of the body containgint the other collider prim entering trigger)")
    .def_readonly("trigger_body_prim_id", &TriggerEventData::triggerBodyPrimId, R"(USD Path of the body containg the collider prim representing the trigger)")
    .def_readonly("event_type", &TriggerEventData::eventType, R"(Event Type (enter or leave))");

    docString = R"(
        Register for trigger notifications

        Args:
            trigger_report_fn:  function            Report function where enter or leave trigger events will be notified
            stage_id:           uint64_t            [Optional] Stage id to filter triggers from
            prim_id:            uint64_t            [Optional] USD path to filter triggers from (use PhysicsSchemaTools::sdfPathToInt)

        Returns:
            subscription_id     : SubscriptionId    Subscription Id that can be unregistered with unsubscribe_physics_trigger_report_events
    )";
    dev.def(
        "subscribe_physics_trigger_report_events",
        []( const IPhysxSimulation* self,
            std::function<void(const TriggerEventData&)> trigger_report_fn, uint64_t stage_id, uint64_t prim_id) -> SubscriptionId
        {
            SubscriptionId wrapSubscriptionId = triggerEventsMarshalCallbacks.callbacksRegistry.addEvent(TriggerEventWrappedCallbacks());
            TriggerEventWrappedCallbacks& wrappedCallbacks = triggerEventsMarshalCallbacks.callbacksRegistry.map.at(wrapSubscriptionId);
            if(trigger_report_fn)
            {
                wrappedCallbacks.trigger_report_fn = carb::wrapPythonCallback(std::move(trigger_report_fn));
            }

            auto triggerReportFn = [](const TriggerEventData* data, void* userData)
            {
                SubscriptionId wrapSubscriptionId = (SubscriptionId)userData;
                TriggerEventWrappedCallbacks& wrappedCallbacks = triggerEventsMarshalCallbacks.callbacksRegistry.map.at(wrapSubscriptionId);
                if(wrappedCallbacks.trigger_report_fn)
                {
                    // On python we want to pass the 'wrapped' subscription id
                    TriggerEventData dataWithNewSubscriptionId = *data;
                    dataWithNewSubscriptionId.subscriptionId = wrapSubscriptionId;
                    wrappedCallbacks.trigger_report_fn(dataWithNewSubscriptionId);
                }
            };

            wrappedCallbacks.triggerSubscriptionId = self->subscribePhysicsTriggerReportEvents(stage_id, prim_id, triggerReportFn, (void*)wrapSubscriptionId);
            return wrapSubscriptionId;
        },
        docString,  py::arg("trigger_report_fn"), // This is the only non optional argument
                    py::arg("stage_id") = 0,
                    py::arg("prim_id") = 0);

    docString = R"(
        Unregister a subscription for trigger notifications

        Args:
            subscription_id:           SubscriptionId            Subscription Id
    )";
    dev.def("unsubscribe_physics_trigger_report_events",
    []( const IPhysxSimulation* self, SubscriptionId subscriptionId)
    {
        TriggerEventWrappedCallbacks& wrappedCallbacks = triggerEventsMarshalCallbacks.callbacksRegistry.map.at(subscriptionId);
        self->unsubscribePhysicsTriggerReportEvents(wrappedCallbacks.triggerSubscriptionId);
        triggerEventsMarshalCallbacks.callbacksRegistry.removeEvent(subscriptionId);
    }, docString, py::arg("subscriptionId"));

    // IPhysxBenchmarks interface
    py::class_<PhysicsProfileStats>(m, "PhysicsProfileStats", R"(
        Physics profile data.
    )")
        .def_readonly("zone_name", &PhysicsProfileStats::zoneName, R"(Profile zone name)")
        .def_readonly("ms", &PhysicsProfileStats::ms, R"(Time in miliseconds)");

    py::bind_vector<std::vector<PhysicsProfileStats>>(m, "PhysicsProfileStatsVector", py::module_local(false));

    dev = defineInterfaceClass<IPhysxBenchmarks>(m, "IPhysxBenchmarks", "acquire_physx_benchmarks_interface", "release_physx_benchmarks_interface");
    dev.doc() = R"(
        This interface is the access point to the omni.physx benchmarks.
    )";    


    docString = R"(
        Enabled profiling
        Args:
            enable_profile: Do custom physics profiling
    )";
    dev.def("enable_profile", wrapInterfaceFunction(&IPhysxBenchmarks::enableProfile), docString, py::arg("enable_profile"));

    docString = R"(
        Get profiling stats
    )";
    dev.def("get_profile_stats",
        [](const IPhysxBenchmarks* self)
    {
        std::vector<PhysicsProfileStats> stats;
        self->getProfileStats(stats);
        py::dict outDict;
        for (size_t i = 0; i < stats.size(); i++)
        {
            outDict[stats[i].zoneName] = stats[i].ms;
        }
        return outDict;
    }
    , docString);
    docString = R"(
            Subscribes to physics benchmark profile stats.

            Subscription cannot be changed in the onUpdate callback

            If subscribed the profile stats get reset after each call,
            this means get_profile_stats will not work as expected.

            Args:
                fn: The callback to be called on every physics fetchResults.

            Returns:
                The subscription holder.
    )";
    dev.def(
        "subscribe_profile_stats_events",
        [](IPhysxBenchmarks* iface, const std::function<void(const std::vector<PhysicsProfileStats>&)>& fn) {
            return carb::StdFuncUtils<decltype(fn)>::buildSubscription(
                fn, iface->subscribeProfileStatsEvents, iface->unsubscribeProfileStatsEvents);
        },
        docString, py::arg("fn"));


    // IPhysxAttachmentPrivate interface
    dev = defineInterfaceClass<IPhysxAttachmentPrivate>(m, "IPhysxAttachmentPrivate", "acquire_physx_attachment_private_interface", "release_physx_attachment_private_interface");
    dev.doc() = R"(
        This interface is the access point to the omni.physx attachment functionality.
    )";

    docString = R"(
        Setup attachments and filters for prim with PhysxSchema.PhysxAutoDeformableAttachmentAPI,
        by adding prims that implement the needed attachments and filters. The attachment and filter
        prims are not populated with attributes yes, but is done later by calling update_auto_deformable_attachment.

        Args:
            attachment_path: path to primitive with PhysxSchema.PhysxAutoDeformableAttachmentAPI
    )";
    dev.def("setup_auto_deformable_attachment", [](const IPhysxAttachmentPrivate* self, const char* attachmentPath) {
        return self->setupAutoDeformableAttachment(pxr::SdfPath(attachmentPath));
        }, docString, py::arg("attachment_path"));

    docString = R"(
        Update attachments and filters for prim with PhysxSchema.PhysxAutoDeformableAttachmentAPI

        Args:
            attachment_path: path to primitive with PhysxSchema.PhysxAutoDeformableAttachmentAPI
    )";
    dev.def("update_auto_deformable_attachment", [](const IPhysxAttachmentPrivate* self, const char* attachmentPath) {
        return self->updateAutoDeformableAttachment(pxr::SdfPath(attachmentPath));
    }, docString, py::arg("attachment_path"));

    docString = R"(
        Create surface Poisson sampler.

        Args:
            collider_path:       Path to prim with UsdPhysics.CollisionAPI
            sampling_distance:   Distance used for sampling points

        Returns:
            Handle to surface sampler
    )";
    dev.def(
        "create_surface_sampler",
        [](const IPhysxAttachmentPrivate* self, const char* colliderPath, float samplingDistance) {

            pxr::SdfPath colliderSdfPath(colliderPath);
            return self->createSurfaceSampler(colliderSdfPath, samplingDistance);
        },
        docString, py::arg("collider_path"), py::arg("sampling_distance"), py::call_guard<py::gil_scoped_release>());

    docString = R"(
        Release surface Poisson sampler.

        Args:
            surface_sampler:     Handle to surface sampler
    )";
    dev.def(
        "release_surface_sampler",
        [](const IPhysxAttachmentPrivate* self, uint64_t surfaceSampler) {

            self->releaseSurfaceSampler(surfaceSampler);
        },
        docString, py::arg("surface_sampler"));

    docString = R"(
        Add samples to surface sampler.

        Args:
            surface_sampler:    Handle to surface sampler
            points:             Points to add

        This function adds samples to the surface sampler that are then considered for further sampling.
    )";
    dev.def(
        "add_surface_sampler_points",
        [](const IPhysxAttachmentPrivate* self, const uint64_t surfaceSampler, const std::vector<Float3>& points) {

            self->addSurfaceSamplerPoints(surfaceSampler, &points[0], uint32_t(points.size()));
        },
        docString, py::arg("surface_sampler"), py::arg("points"));

    docString = R"(
        Remove samples from surface sampler.

        Args:
            surface_sampler:    Handle to surface sampler
            points:             Points to remove

        This function removes samples from the surface sampler.
    )";
    dev.def(
        "remove_surface_sampler_points",
        [](const IPhysxAttachmentPrivate* self, const uint64_t surfaceSampler, const std::vector<Float3>& points) {
            self->removeSurfaceSamplerPoints(surfaceSampler, &points[0], uint32_t(points.size()));
        },
        docString, py::arg("surface_sampler"), py::arg("points"));

    docString = R"(
        Create new surface samples within specified sphere.

        Args:
            surface_sampler:    Handle to surface sampler
            sphere_center:      Center of sphere used for restricting sampling domain
            sphere_radius:      Radius of sphere used for restricting sampling domain

        Returns:
            Dict mapping 'points' to the resulting sampling points

        New samples are created on the surface for which the samples has been created, and within the sphere specified.
    )";
    dev.def(
        "sample_surface",
        [](const IPhysxAttachmentPrivate* self, const uint64_t surfaceSampler, const Float3& sphereCenter, const float sphereRadius, const float samplingDistance) {

                ResultBuffer<carb::Float3> dstPoints;
                self->sampleSurface(dstPoints.ptr, dstPoints.size, surfaceSampler, sphereCenter, sphereRadius, samplingDistance, ResultBuffer<>::allocate);
                py::dict out;
                out["points"] = static_cast<py::list>(dstPoints);
                return out;
        },
        docString, py::arg("surface_sampler"), py::arg("sphere_center"), py::arg("sphere_radius"), py::arg("sampling_distance"));

    docString = R"(
        Get the samples of surface sampler.

        Args:
            surface_sampler:    Handle to surface sampler

        Returns:
            Dict mapping 'points' to the resulting sampling points
    )";
    dev.def(
        "get_surface_sampler_points",
        [](const IPhysxAttachmentPrivate* self, const uint64_t surfaceSampler) {

            ResultBuffer<carb::Float3> dstPoints;
            self->getSurfaceSamplerPoints(dstPoints.ptr, dstPoints.size, surfaceSampler, ResultBuffer<>::allocate);
            py::dict out;
            out["points"] = static_cast<py::list>(dstPoints);
            return out;
        },
        docString, py::arg("surface_sampler"));

    docString = R"(
        Create tet finder.

        Args:
            points:          Points of the tetrahedral mesh
            indices:         Indices of the tetrahedral mesh

        Returns:
            Handle to tet finder
    )";
    dev.def(
        "create_tet_finder",
        [](const IPhysxAttachmentPrivate* self, const std::vector<carb::Float3>& points, const std::vector<uint32_t>& indices) {

            return self->createTetFinder(&points[0], uint32_t(points.size()), &indices[0], uint32_t(indices.size()));
        },
        docString, py::arg("points"), py::arg("indices"));

    docString = R"(
        Release tet finder.

        Args:
            tet_finder:     Handle to tet finder
    )";
    dev.def(
        "release_tet_finder",
        [](const IPhysxAttachmentPrivate* self, uint64_t tetFinder) {

            self->releaseTetFinder(tetFinder);
        },
        docString, py::arg("tet_finder"));

    docString = R"(
        Map points in euclidean coordiantes to tet mesh coordinates.

        Args:
            tet_finder:    Handle to tet finder
            points:        Points to be mapped

        Returns:
            Dict mapping 'tet_ids' and 'barycentric_coords' to the resulting local tet mesh coordinates. Tet indices might be -1 for points outside of the tetrahedral mesh.
    )";
    dev.def(
        "points_to_tetmesh_local",
        [](const IPhysxAttachmentPrivate* self, const uint64_t tetFinder, std::vector<carb::Float3>& points) {

            std::vector<int32_t> dstTetIds(points.size());
            std::vector<carb::Float4> dstBarycentricCoords(points.size());
            self->pointsToTetMeshLocal(&dstTetIds[0], &dstBarycentricCoords[0], tetFinder, &points[0], uint32_t(points.size()));
            py::dict out;
            out["tet_ids"] = dstTetIds;
            out["barycentric_coords"] = dstBarycentricCoords;
            return out;
        },
        docString, py::arg("tet_finder"), py::arg("points"));

    docString = R"(
        Map tet mesh coordinates to points in euclidean coordinates.

        Args:
            tet_finder:         Handle to tet finder
            tet_ids:            Tetrahedra indices
            barycentric_coords: Barycentric coordinates

        Returns:
            Dict mapping 'points' to the resulting points in euclidean coordinates.
    )";
    dev.def(
        "tetmesh_local_to_points",
        [](const IPhysxAttachmentPrivate* self, const uint64_t tetFinder, std::vector<int32_t>& tetIds, std::vector<carb::Float4>& barycentricCoords) {

            std::vector<carb::Float3> dstPoints(tetIds.size());
            self->tetMeshLocalToPoints(&dstPoints[0], tetFinder, &tetIds[0], &barycentricCoords[0], uint32_t(dstPoints.size()));
            py::dict out;
            out["points"] = dstPoints;
            return out;
        },
        docString, py::arg("tet_finder"), py::arg("tet_ids"), py::arg("barycentric_coords"));

    docString = R"(
        Finds all tetrahedra overlapping with a sphere.

        Args:
            tet_finder:         Handle to tet finder
            center:             Center of sphere
            radius:             Radius of sphere

        Returns:
            Dict mapping 'tet_ids' to the resulting indices of tetrahedra overlapping with the sphere.
    )";
    dev.def(
        "overlap_tetmesh_sphere",
        [](const IPhysxAttachmentPrivate* self, const uint64_t tetFinder, const Float3& center, const float radius) {

            ResultBuffer<int32_t> dstTetIds;
            self->overlapTetMeshSphere(dstTetIds.ptr, dstTetIds.size, tetFinder, center, radius, ResultBuffer<>::allocate);
            py::dict out;
            out["tet_ids"] = static_cast<py::list>(dstTetIds);
            return out;
        },
        docString, py::arg("tet_finder"), py::arg("center"), py::arg("radius"));

    docString = R"(
        Finds all tetrahedra overlapping with a capsule.

        Args:
            tet_finder:         Handle to tet finder
            pos:                Position of capsule
            axis:               Orientation of the capsule
            radius:             Radius of the capsule
            half_height         Half height of the capsule

        Returns:
            Dict mapping 'tet_ids' to the resulting indices of tetrahedra overlapping with the capsule.
    )";
    dev.def(
        "overlap_tetmesh_capsule",
        [](const IPhysxAttachmentPrivate* self, const uint64_t tetFinder, const Float3& pos, const Float3& axis, const float radius, const float halfHeight) {

            ResultBuffer<int32_t> dstTetIds;
            self->overlapTetMeshCapsule(dstTetIds.ptr, dstTetIds.size, tetFinder, pos, axis, radius, halfHeight, ResultBuffer<>::allocate);
            py::dict out;
            out["tet_ids"] = static_cast<py::list>(dstTetIds);
            return out;
        },
        docString, py::arg("tet_finder"), py::arg("pos"), py::arg("axis"), py::arg("radius"), py::arg("half_height"));

    docString = R"(
        Create point finder.

        Args:
            points:         Points of the mesh

        Returns:
            Handle to point finder
    )";
    dev.def(
        "create_point_finder",
        [](const IPhysxAttachmentPrivate* self, const std::vector<carb::Float3>& points) {

            return self->createPointFinder(&points[0], uint32_t(points.size()));
        },
        docString, py::arg("points"));

    docString = R"(
        Release point finder.

        Args:
            point_finder:   Handle to point finder
    )";
    dev.def(
        "release_point_finder",
        [](const IPhysxAttachmentPrivate* self, uint64_t pointFinder) {

            self->releasePointFinder(pointFinder);
        },
        docString, py::arg("point_finder"));

    docString = R"(
        Map points to indices.

        Args:
            point_finder:   Handle to point finder
            points:         Points to be mapped

        Returns:
            Dict mapping 'indices' to the resulting mesh coordinates. Indices might be -1 for points outside of the mesh.
    )";
    dev.def(
        "points_to_indices",
        [](const IPhysxAttachmentPrivate* self, const uint64_t pointFinder, std::vector<carb::Float3>& points) {

            std::vector<int32_t> dstIndices(points.size());
            self->pointsToIndices(&dstIndices[0], pointFinder, &points[0], uint32_t(points.size()));
            py::dict out;
            out["indices"] = dstIndices;
            return out;
        },
        docString, py::arg("point_finder"), py::arg("points"));

    docString = R"(
        Get closest points to the input points on the prim

        Args:
            points: input points
            path:  prim path

        Returns:
            Return a dictionary with closest information::

                'closest_points': float3 - Closest points to the input points on the prim. Only valid when returned distance is strictly positive.
                'dists': float - Square distances between the points and the geom object. 0.0 if the point is inside the object.
    )";
    dev.def(
        "get_closest_points",
        [](const IPhysxAttachmentPrivate* self, const std::vector<carb::Float3>& points, const char* path) {
                pxr::SdfPath rigidPath(path);
                std::vector<carb::Float3> closestPoints(points.size());
                std::vector<float> dists(points.size());
                self->getClosestPoints(&closestPoints[0], &dists[0], &points[0], uint32_t(points.size()), rigidPath);
                py::dict out;
                out["closest_points"] = closestPoints;
                out["dists"] = dists;
                return out;
        },
        docString, py::arg("point"), py::arg("path"));

    dev.def(
        "create_tri_mesh_sampler",
        [](const IPhysxAttachmentPrivate* self, uint64_t surfaceSampler) {

            self->createTriMeshSampler(surfaceSampler);
        },
        docString, py::arg("surface_sampler"));

    dev.def(
        "is_point_inside",
        [](const IPhysxAttachmentPrivate* self, uint64_t surfaceSampler, carb::Float3 point) {

            return self->isPointInside(surfaceSampler, point);
        },
        docString, py::arg("surface_sampler"), py::arg("point"));

    // Debug viz enums to use with debug viz settings
    // PREIST: This approach to expose the enums is a bit more cumbersome than using py::enum_,
    // but this way the constants/enums are ints and work with both settings.set() and settings.set_int().
    // With enum, it only works with set_int() and fails silently with set() if the user
    // forgets to cast the Python enum to int when using with set()
    struct VisualizerMode{};
    py::class_<VisualizerMode>(m, "VisualizerMode", R"(
    Visualization mode for collider, particles, deformables, etc. object types.
    )")
        .def_property_readonly_static("NONE", [](py::object) { return int(omni::physx::ui::VisualizerMode::eNone); })
        .def_property_readonly_static("SELECTED", [](py::object) { return int(omni::physx::ui::VisualizerMode::eSelected); })
        .def_property_readonly_static("ALL", [](py::object) { return int(omni::physx::ui::VisualizerMode::eAll); });

    struct ParticleVisualizationRadiusType{};
    py::class_<ParticleVisualizationRadiusType>(m, "ParticleVisualizationRadiusType", R"(
    Particle radius debug visualization option for use with SETTING_DISPLAY_PARTICLES_RADIUS_TYPE.
    )")
        .def_property_readonly_static("CONTACT_OFFSET", [](py::object) { return int(omni::physx::ui::ParticleRadiusType::eContactOffset); })
        .def_property_readonly_static("REST_OFFSET", [](py::object) { return int(omni::physx::ui::ParticleRadiusType::eRestOffset); })
        .def_property_readonly_static("PARTICLE_CONTACT_OFFSET", [](py::object) { return int(omni::physx::ui::ParticleRadiusType::eParticleContactOffset); })
        .def_property_readonly_static("PARTICLE_REST_OFFSET", [](py::object) { return int(omni::physx::ui::ParticleRadiusType::eParticleRestOffset); })
        .def_property_readonly_static("ANISOTROPY", [](py::object) { return int(omni::physx::ui::ParticleRadiusType::eAnisotropy); })
        .def_property_readonly_static("RENDER_GEOMETRY", [](py::object) { return int(omni::physx::ui::ParticleRadiusType::eRenderGeometry); });

    struct ParticleVisualizationPositionType{};
    py::class_<ParticleVisualizationPositionType>(m, "ParticleVisualizationPositionType", R"(
    Particle position debug visualiztion option for smoothed fluid particles for use with
    SETTING_DISPLAY_PARTICLES_POSITION_TYPE.
    )")
        .def_property_readonly_static("SIM_POSITIONS", [](py::object) { return int(omni::physx::ui::ParticlePositionType::eSimPositions); })
        .def_property_readonly_static("SMOOTHED_POSITIONS", [](py::object) { return int(omni::physx::ui::ParticlePositionType::eSmoothedPositions); });

    py::enum_<PhysxPropertyQueryResult::Enum>(m, "PhysxPropertyQueryResult", R"(
        Query result enumeration.
        )")
        .value("VALID", PhysxPropertyQueryResult::eVALID, "Result is valid")
        .value("ERROR_UNKNOWN_QUERY_MODE", PhysxPropertyQueryResult::eERROR_UNKNOWN_QUERY_MODE, "The requested query mode is unknown")
        .value("ERROR_INVALID_USD_PATH", PhysxPropertyQueryResult::eERROR_INVALID_USD_PATH, "Result invalid because of an invalid USD path")
        .value("ERROR_INVALID_USD_STAGE", PhysxPropertyQueryResult::eERROR_INVALID_USD_STAGE, "Result invalid because of an invalid or expired USD stage")
        .value("ERROR_INVALID_USD_PRIM", PhysxPropertyQueryResult::eERROR_INVALID_USD_PRIM, "Result invalid because of an invalid or deleted USD prim")
        .value("ERROR_PARSING", PhysxPropertyQueryResult::eERROR_PARSING, "Result invalid because parsing USD failed")
        .value("ERROR_TIMEOUT", PhysxPropertyQueryResult::eERROR_TIMEOUT, "Result invalid because async operation exceeds timeout")
        .value("ERROR_RUNTIME", PhysxPropertyQueryResult::eERROR_RUNTIME, "Result invalid because PhysX runtime is in invalid state")
        ;

    py::enum_<PhysxPropertyQueryMode::Enum>(m, "PhysxPropertyQueryMode", R"(
        Query mode.
        )")
        .value("QUERY_RIGID_BODY_WITH_COLLIDERS", PhysxPropertyQueryMode::eQUERY_RIGID_BODY_WITH_COLLIDERS, "Query rigid body and its colliders")
        .value("QUERY_ARTICULATION", PhysxPropertyQueryMode::eQUERY_ARTICULATION, "Query articulation");

    py::enum_<PhysxPropertyQueryRigidBodyType::Enum>(m, "PhysxPropertyQueryRigidBodyResponseType", R"(
        Query result.
        )")
        .value("RIGID_DYNAMIC", PhysxPropertyQueryRigidBodyType::eTYPE_RIGID_DYNAMIC, "Body is a rigid dynamic");

    py::class_<PhysxPropertyQueryRigidBodyResponse>(m, "PhysxPropertyQueryRigidBodyResponse", R"(
            Rigid body query response.
        )")
        .def_readonly("result", &PhysxPropertyQueryRigidBodyResponse::result, R"(Result)")
        .def_readonly("stage_id", &PhysxPropertyQueryRigidBodyResponse::usdStageId, R"(USD Stage)")
        .def_readonly("path_id", &PhysxPropertyQueryRigidBodyResponse::usdPath, R"(USD Path)")
        .def_readonly("type", &PhysxPropertyQueryRigidBodyResponse::type, R"(Type)")
        .def_readonly("mass", &PhysxPropertyQueryRigidBodyResponse::mass, R"(Mass)")
        .def_readonly("inertia", &PhysxPropertyQueryRigidBodyResponse::inertia, R"(Inertia)")
        .def_readonly("center_of_mass", &PhysxPropertyQueryRigidBodyResponse::centerOfMass, R"(Center of Mass)")
        .def_readonly("principal_axes", &PhysxPropertyQueryRigidBodyResponse::principalAxes, R"(Principal Axes Quaternion)");

    py::class_<PhysxPropertyQueryColliderResponse>(m, "PhysxPropertyQueryColliderResponse", R"(
            Collider query response.
        )")
        .def_readonly("result", &PhysxPropertyQueryColliderResponse::result, R"(Result)")
        .def_readonly("stage_id", &PhysxPropertyQueryColliderResponse::usdStageId, R"(USD Stage)")
        .def_readonly("path_id", &PhysxPropertyQueryColliderResponse::usdPath, R"(USD Path)")
        .def_readonly("aabb_local_min", &PhysxPropertyQueryColliderResponse::aabbLocalMin, R"(AABB Min Local Bound)")
        .def_readonly("aabb_local_max", &PhysxPropertyQueryColliderResponse::aabbLocalMax, R"(AABB Max Local Bound)")
        .def_readonly("volume", &PhysxPropertyQueryColliderResponse::volume, R"(Volume of the collider)")
        .def_readonly("local_pos", &PhysxPropertyQueryColliderResponse::localPos, R"(Local position)")
        .def_readonly("local_rot", &PhysxPropertyQueryColliderResponse::localRot, R"(Local rotation)")
        ;

    py::class_<PhysxPropertyQueryArticulationLink>(m, "PhysxPropertyQueryArticulationLink", R"(
            Articulation query response.
        )")
        .def_readonly("rigid_body", &PhysxPropertyQueryArticulationLink::rigidBody, R"(Rigid body name as int)")
        .def_property_readonly(
            "rigid_body_name",
            [](const PhysxPropertyQueryArticulationLink* self) {
                return (intToPath(self->rigidBody)).GetText();
            },
            R"(Rigid Body name as string.)")
        .def_readonly("joint", &PhysxPropertyQueryArticulationLink::joint, R"(Joint name as int)")
        .def_property_readonly(
            "joint_name",
            [](const PhysxPropertyQueryArticulationLink* self) { return (intToPath(self->joint)).GetText(); },
            R"(Joint name as string.)")
        .def_readonly("joint_dof", &PhysxPropertyQueryArticulationLink::jointDof, R"(Joint DOF)");

    py::bind_vector<PhysxPropertyQueryArticulationLinkVector>(m, "PhysxPropertyQueryArticulationLinkVector", py::module_local(false));

    py::class_<PhysxPropertyQueryArticulationResponse>(m, "PhysxPropertyQueryArticulationResponse", R"(
            Articulation query response.
        )")
        .def_readonly("result", &PhysxPropertyQueryArticulationResponse::result, R"(Result)")
        .def_readonly("stage_id", &PhysxPropertyQueryArticulationResponse::usdStageId, R"(USD Stage)")
        .def_readonly("path_id", &PhysxPropertyQueryArticulationResponse::usdPath, R"(USD Path)")
        .def_readonly("links", &PhysxPropertyQueryArticulationResponse::links, R"(Articulation links that belong to the articulation)");        

    dev = defineInterfaceClass<IPhysxPropertyQuery>(m, "IPhysxPropertyQuery", "acquire_physx_property_query_interface", "release_physx_property_query_interface");

    docString = R"(
        Returns information for given prim

        Args:
            stage_id:       uint64_t                Stage id
            prim_id:        uint64_t                USD path (use PhysicsSchemaTools::sdfPathToInt)
            query_mode:     PhysxPropertyQueryMode  Type of query to be made
            timeout_ms:     int64                   Timeout (in milliseconds) for the request. (-1 means wait forever)
            finished_fn:  function                  Report function called when enumeration of all objects is finished
            rigid_body_fn:  function                Report function where rigid body information will be returned in PhysxPropertyQueryRigidBodyResponse object
            collider_fn:    function                Report function where collider information will be returned in PhysxPropertyQueryRigidBodyResponse object
    )";
    dev.def(
        "query_prim",
        []( const IPhysxPropertyQuery* self, uint64_t stage_id, uint64_t prim_id,
            PhysxPropertyQueryMode::Enum query_mode,
            int64_t timeout_ms,
            std::function<void()> finished_fn,
            std::function<void(const PhysxPropertyQueryRigidBodyResponse&)> rigid_body_fn,
            std::function<void(const PhysxPropertyQueryColliderResponse&)> collider_fn,
            std::function<void(const PhysxPropertyQueryArticulationResponse&)> articulation_fn)
        {
            struct WrappedCallbacks
            {
                std::function<void()> finished_fn;
                std::function<void(const PhysxPropertyQueryRigidBodyResponse&)> rigid_body_fn;
                std::function<void(const PhysxPropertyQueryColliderResponse&)> collider_fn;
                std::function<void(const PhysxPropertyQueryArticulationResponse&)> articulation_fn;
            };
            WrappedCallbacks* wrappedCallbacks = new WrappedCallbacks();
            if(finished_fn)
            {
                wrappedCallbacks->finished_fn = carb::wrapPythonCallback(std::move(finished_fn));
            }
            if(rigid_body_fn)
            {
                wrappedCallbacks->rigid_body_fn = carb::wrapPythonCallback(std::move(rigid_body_fn));
            }
            if(collider_fn)
            {
                wrappedCallbacks->collider_fn = carb::wrapPythonCallback(std::move(collider_fn));
            }
            if (articulation_fn)
            {
                wrappedCallbacks->articulation_fn = carb::wrapPythonCallback(std::move(articulation_fn));
            }
            IPhysxPropertyQueryCallback callbacks;
            callbacks.timeoutMs = timeout_ms;
            callbacks.userData = wrappedCallbacks;
            callbacks.queryFinishedCallback = [](void* userData)
            {
                if(static_cast<WrappedCallbacks*>(userData)->finished_fn)
                {
                    static_cast<WrappedCallbacks*>(userData)->finished_fn();
                }
                delete static_cast<WrappedCallbacks*>(userData);
            };
            callbacks.rigidBodyCallback = [](const PhysxPropertyQueryRigidBodyResponse& response, void* userData)
            {
                if(static_cast<WrappedCallbacks*>(userData)->rigid_body_fn)
                {
                    static_cast<WrappedCallbacks*>(userData)->rigid_body_fn(response);
                }
            };
            callbacks.colliderCallback = [](const PhysxPropertyQueryColliderResponse& response, void* userData)
            {
                if(static_cast<WrappedCallbacks*>(userData)->collider_fn)
                {
                    static_cast<WrappedCallbacks*>(userData)->collider_fn(response);
                }
            };
            callbacks.articulationCallback = [](const PhysxPropertyQueryArticulationResponse& response, void* userData) {
                if (static_cast<WrappedCallbacks*>(userData)->articulation_fn)
                {
                    static_cast<WrappedCallbacks*>(userData)->articulation_fn(response);
                }
            };
            self->queryPrim(stage_id, prim_id, query_mode, callbacks);
        },
        docString, py::arg("stage_id"), py::arg("prim_id"),
                    py::arg("query_mode") = PhysxPropertyQueryMode::eQUERY_RIGID_BODY_WITH_COLLIDERS,
                    py::arg("timeout_ms") = -1,
                    py::arg("finished_fn") = std::function<void()>(),
                    py::arg("rigid_body_fn") = std::function<void(const PhysxPropertyQueryRigidBodyResponse&)>(),
                    py::arg("collider_fn") = std::function<void(const PhysxPropertyQueryColliderResponse&)>(),
                    py::arg("articulation_fn") = std::function<void(const PhysxPropertyQueryArticulationResponse&)>());

    PhysXReplicatorBindings(m);
    PhysXStageUpdateBindings(m);
    PhysXCookingBindings(m);
    PhysXCookingPrivateBindings(m);
    PhysXStatisticsBindings(m);
}
} // namespace
