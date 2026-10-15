// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-SERVICES-001
// @covers AC-1 AC-2 AC-3 AC-4

// @implements REQ-ATTACH-RESOLVE-001
// @covers AC-4 AC-6 AC-8

/**
 * @implements REQ-OMNIPVD-TRANSPORT-001
 * @covers AC-3
 * @implements REQ-OMNIPVD-LATE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10
 */

// @implements REQ-SDK-INIT-001
// @covers AC-1 AC-4

#pragma once
#include <carb/tasking/TaskingUtils.h>

#include <PxPhysicsAPI.h>
#include <omnipvd/PxOmniPvd.h>
#include <carb/events/IEvents.h>
#include <carb/settings/ISettings.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/IPhysxSimulation.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include "utils/Pair.h"

#include <mutex>
#include <memory>
#include <unordered_set>

#include "PhysXScene.h"
#include "SceneMultiGPUMode.h"

class CarbPhysXErrorCallback;
class OmniPvdFileWriteStream;
class OmniPvdSocketWriteStream;
class OmniPvdWriteStream;

namespace cookingdataasync
{
class CookingDataAsync;
}

namespace omni
{
namespace physx
{
struct ICookingComputeService;
struct IPhysxFoundation;
typedef void* PhysxCookingAsyncContext;
} // namespace physx
} // namespace omni

namespace omni
{
namespace physx
{
// word3 flags for filtering shader
const uint32_t CONTACT_MODIFY_SURFACE_VELOCITY = 1 << 1;
const uint32_t CONTACT_SOLVE_DISABLE = 1 << 2;

const size_t kDefaultPhysXSceneId = 0;

using PhysXScenesMap = std::unordered_map<const ::physx::PxScene*, PhysXScene*>;

// Class holding and handling PhysX global objects like PxPhysics, PxFoundation
class PhysXSetup : private ::physx::PxOmniPvdEventCallback
{
public:
    PhysXSetup();
    ~PhysXSetup();

    ///////////////////////////////////////////////////////////////////////////////////////
    // Tolerances scales
    // Invalid stage units return an invalid scale and log an error; callers must reject it.
    ::physx::PxTolerancesScale getDefaultTolerances(double metersPerUnit);

    ///////////////////////////////////////////////////////////////////////////////////////
    // Cooking params
    ::physx::PxCookingParams getCookingParams(const ::physx::PxTolerancesScale& tolerances);
    const ::physx::PxCookingParams& getDefaultCookingParams()
    {
        return mDefaultCookingParams;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Pointer acceses
    // Lazily initializes the SDK; returns nullptr when initialization fails.
    ::physx::PxPhysics* getPhysics();
    // Acquire before realization, including empty candidates. Release only after
    // native objects and deferred work are retired, at attachment destruction.
    bool acquireAttachmentResources(usdparser::AttachedStage& attachment);
    ::physx::PxCudaContextManager* acquireAttachmentCudaContext(usdparser::AttachedStage& attachment);
    void releaseAttachmentResources(const usdparser::AttachedStage& attachment);
    bool hasResourceBorrowers() const;
    PhysxSharedSettings getSharedSettingsDefaults() const;
    ::physx::PxDefaultAllocator& getAllocator()
    {
        return mAllocator;
    }
    void reportCudaContextError(const usdparser::AttachedStage& owner, ::physx::PxCudaContextManager* manager);

    ::physx::PxSerializationRegistry* getSerializationRegistry()
    {
        if (!mSerializationRegistry)
        {
            ::physx::PxPhysics* physics = getPhysics();
            if (!physics)
                return nullptr;
            mSerializationRegistry = ::physx::PxSerialization::createSerializationRegistry(*physics);
        }
        return mSerializationRegistry;
    }

    cookingdataasync::CookingDataAsync* getCookingDataAsync()
    {
        return mCookingDataAsync;
    }
    ICookingComputeService* getCookingComputeService();
    IPhysxCookingService* getCookingServiceInterface();
    IPhysxCookingServicePrivate* getCookingServicePrivateInterface();

    ///////////////////////////////////////////////////////////////////////////////////////
    // PxScene handling
    // Exact native-scene ownership lookup; foreign and retired scenes return null.
    PhysXScene* findPhysXScene(const ::physx::PxScene* scene) const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mSceneRegistryMutex);
        const auto it = mPhysXScenes.find(scene);
        return it == mPhysXScenes.end() ? nullptr : it->second;
    }
    void releasePhysXScenes();
    // Native scene retirement only. The caller drains attachment services and releases
    // its records first; shared SDK/CUDA/dispatcher resources remain resident.
    void releasePhysXScenes(usdparser::AttachedStage& attachedStage);
    PhysXScene* createPhysXScene(usdparser::AttachedStage& attachedStage,
                                 size_t sceneId,
                                 double metersPerUnit,
                                 double kilogramsPerUnit,
                                 const usdparser::PhysxSceneDesc& sceneDesc);

    PhysXScenesMap getPhysXScenes() const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mSceneRegistryMutex);
        return mPhysXScenes;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    // Thread count
    uint32_t getThreadCount() const
    {
        return mThreadCount;
    }
    void setThreadCount(uint32_t val)
    {
        mThreadCount = val;
    }
    ///////////////////////////////////////////////////////////////////////////////////////
    // PVD
    // Recording state belongs to the shared PVD transport, independent of owner clocks.
    void notePvdSimulation() { mPvdHasSimulation.store(true); }
    void connectPVD();
    void disconnectPVD();
    ::physx::PxPvd* getPvd()
    {
        return mVisualDebugger;
    }
    ::physx::PxOmniPvd* getOmniPvd()
    {
        return mOmniPvd;
    }
    // Runtime change of PVD settings - CAVEAT: this must NOT be called in a running simulation
    // (no synchronization is provided between post-frame-end pvd flushing and client reconnection)
    void changePVDSettings(bool enableLegacyPVD, bool enableOmniPVD);

    // Called after successful attachment publication. Never replaces an active session.
    void startConfiguredOmniPvdRecording();
    OmniPvdRecordingResult startOmniPvdRecording(const OmniPvdDestination& destination, bool startupFile = false);
    OmniPvdRecordingResult stopOmniPvdRecording();
    bool isOmniPvdRecording() const;

    inline const ::physx::PxVehiclePvdAttributeHandles* getVehiclePvdRegistrationHandles() const
    {
        return mVehiclePvdRegistrationHandles;
    }

    ::physx::PxConvexMesh* getCylinderConvexMesh(omni::physx::usdparser::Axis axis) const;
    ::physx::PxConvexMesh* getConeConvexMesh(omni::physx::usdparser::Axis axis) const;
    ::physx::PxConvexMesh* getVehicleWheelCylinderConvexMesh(omni::physx::usdparser::Axis axis) const;

    ///////////////////////////////////////////////////////////////////////////////////////
    // If any scene is async return true
    bool isAsyncSimEnabled() const
    {
        std::lock_guard<carb::tasking::MutexWrapper> registry(mSceneRegistryMutex);
        for (PhysXScenesMap::const_reference ref : mPhysXScenes)
        {
            if (ref.second->getUpdateType() == usdparser::eAsynchronous)
                return true;
        }
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // create base PhysX classses
    // A.B. this should be optimized, we dont have to create this with every simulation start
    void createPhysics(const ::physx::PxTolerancesScale& tolerances);

    bool hasBeenInitiallyCreated() const
    {
        return mFoundation != nullptr;
    }
    void createPhysics(); // initial create
    void releasePhysics(); // final release, should be called when app is closing

    void resetPhysXErrorCounter();
    void setMaxNumberOfPhysXErrors(uint32_t maxNumberOfPhysXErrors);

private:
    void initializeCookingServiceInterfaces();
    void releaseCookingService();
    void createCookingAsyncContext();
    void releaseCookingAsyncContext();
    void cleanupPhysics(); // called with scene destruction, this should not be required in the end
    void setOmniPVDoutputDirectory(const char* directory);
    void setOmniPVDTimeStampedFileName(const char* fileName);
    void writeOutOmniPVDFile(bool omniPVDWasActive, bool fileWriteStreamClosedOk);
    bool releaseOmniPvdWriteStream();
    bool closeOmniPvdWriteStream();
    void onStartSampling(::physx::PxOmniPvd& omniPvd) override;
    void registerVehiclePvdCallback();
    void unregisterVehiclePvdCallback();
    void releaseVehiclePvdRegistrationHandles();
    void releaseVehiclePvdTelemetry();
    ::physx::PxFoundation* mFoundation{ nullptr };
    ::physx::PxPhysics* mPhysics{ nullptr };
    std::unordered_set<const usdparser::AttachedStage*> mAttachmentBorrowers;
    ::physx::PxSerializationRegistry* mSerializationRegistry{ nullptr };
    carb::settings::ISettings* mISettings{ nullptr };

    ::physx::PxDefaultAllocator mAllocator;
    CarbPhysXErrorCallback* mErrorCallback{ nullptr };

    ::physx::PxTolerancesScale mDefaultScale;
    ::physx::PxCookingParams mDefaultCookingParams{ mDefaultScale };
    cookingdataasync::CookingDataAsync* mCookingDataAsync{ nullptr };
    omni::physx::ICookingComputeService* mCookingComputeService{ nullptr };
    omni::physx::IPhysxCookingServicePrivate mCookingServicePrivate{};
    omni::physx::IPhysxCookingService mCookingService{};
    omni::physx::PhysxCookingAsyncContext mCookingServiceContext{ nullptr };
    bool mPhysicsReleased{ false };
    omni::physx::IPhysxFoundation* mPhysxFoundation{ nullptr };

    ::physx::PxPvd* mVisualDebugger{ nullptr };
    ::physx::PxPvdTransport* mPvdTransport{ nullptr };
    std::atomic<bool> mPvdHasSimulation{ false };
    ::physx::PxOmniPvd* mOmniPvd{ nullptr };
    enum class OmniPvdStreamKind : uint8_t
    {
        eNone,
        eStartupFile,
        eFile,
        eTcp,
    };
    OmniPvdWriteStream* mOmniPvdWriteStream{ nullptr };
    OmniPvdStreamKind mOmniPvdStreamKind{ OmniPvdStreamKind::eNone };
    ::physx::PxVehiclePvdAttributeHandles* mVehiclePvdRegistrationHandles{ nullptr };
    bool mVehiclePvdSnapshotSucceeded{ false };
    bool mWasOmniPVDSimStarted{ false };
    std::string mOmniPVDOutputDirectory;
    std::string mOmniPVDTimeStampedFileName;

    uint32_t mThreadCount{ 8 };

    struct AttachmentCudaResource
    {
        int mode;
        int ordinal;
        uintptr_t externalContext;
        bool synchronous;
        std::shared_ptr<::physx::PxCudaContextManager> manager;
    };
    // PhysX caches GPU kernel wranglers by manager address until PxPhysics dies.
    // Retain compatible managers for that whole SDK generation, including gaps
    // between attachments, so an address cannot alias a stale kernel cache entry.
    std::vector<AttachmentCudaResource> mAttachmentCudaResources;
    mutable carb::tasking::RecursiveMutexWrapper mResourceMutex;
    mutable carb::tasking::MutexWrapper mSceneRegistryMutex;
    bool mReleasingPhysXScenes{ false };
    bool mExtensionsInitialized{ false };
    bool mVehicleSDKInitialized{ false }; // guards against multiple de-init, since scene release can be called
                                          // more than once (TODO: investigate why); also guards multiple init,
                                          // though that hasn't been observed.
    PhysXScenesMap mPhysXScenes;

    mutable ::physx::PxConvexMesh* mCylinderMeshX{ nullptr };
    mutable ::physx::PxConvexMesh* mCylinderMeshY{ nullptr };
    mutable ::physx::PxConvexMesh* mCylinderMeshZ{ nullptr };
    mutable ::physx::PxConvexMesh* mConeMeshX{ nullptr };
    mutable ::physx::PxConvexMesh* mConeMeshY{ nullptr };
    mutable ::physx::PxConvexMesh* mConeMeshZ{ nullptr };
    mutable ::physx::PxConvexMesh* mVehicleWheelCylinderMeshX{ nullptr };
    mutable ::physx::PxConvexMesh* mVehicleWheelCylinderMeshY{ nullptr };
    mutable ::physx::PxConvexMesh* mVehicleWheelCylinderMeshZ{ nullptr };

    // Subscriptions to settings changes
    std::vector<carb::dictionary::SubscriptionId*> mSubscribedSettings;
};

} // namespace physx
} // namespace omni
