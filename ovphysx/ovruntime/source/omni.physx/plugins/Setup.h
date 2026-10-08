// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-OMNIPVD-TRANSPORT-001
 * @covers AC-3
 * @implements REQ-OMNIPVD-LATE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10
 */

#pragma once

#include <PxPhysicsAPI.h>
#include <omnipvd/PxOmniPvd.h>
#include <carb/events/IEvents.h>
#include <carb/settings/ISettings.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/IPhysxSimulation.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include "utils/Pair.h"

#include <mutex>

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

using PhysXScenesMap = std::unordered_map<size_t, PhysXScene*>;
using FilteredPairsSet = std::unordered_map<Pair<uint32_t>, uint32_t, PairHash>;
using CollisionGroupsPairsSet = std::unordered_set<Pair<uint32_t>, PairHash>;

// Class holding and handling PhysX global objects like PxPhysics, PxFoundation
class PhysXSetup : private ::physx::PxOmniPvdEventCallback
{
public:
    PhysXSetup();
    ~PhysXSetup();

    ///////////////////////////////////////////////////////////////////////////////////////
    // Tolerances scales
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
    ::physx::PxPhysics* getPhysics();
    ::physx::PxDefaultAllocator& getAllocator()
    {
        return mAllocator;
    }
    // Borrowed pointer, no reference taken. Only safe on the main thread and only for code that
    // cannot outlive the current manager; anything else must use acquireCudaContextManager().
    ::physx::PxCudaContextManager* getCudaContextManager(size_t id = 0) const;
    ::physx::PxCudaContextManager* getNextCudaContextManager();

    /**
     * Same as getCudaContextManager(), but returns the manager with a reference already taken (or
     * null), under the lock that also guards every release of mCudaContextManagers.
     *
     * This is what other subsystems must use when they keep the pointer past the call. A consumer
     * on another thread has no race-free moment in which to call acquireReference() itself: it
     * borrows the pointer from the getter above, and setupGPU()/clearCudaContextManagers() can
     * destroy the manager on the main thread between that read and the acquire (OMPE-106105).
     *
     * @implements REQ-COOK-CUDACTX-001
     * @covers AC-1
     */
    ::physx::PxCudaContextManager* acquireCudaContextManager(size_t id = 0);

    ::physx::PxCpuDispatcher* getCpuDispatcher() const
    {
        return mDispatcher;
    }
    ::physx::PxSerializationRegistry* getSerializationRegistry()
    {
        if (!mSerializationRegistry)
        {
            mSerializationRegistry = ::physx::PxSerialization::createSerializationRegistry(*getPhysics());
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
    PhysXScene* getPhysXScene(size_t id) const
    {
        if (id == 0)
        {
            return mDefaultScene;
        }
        PhysXScenesMap::const_iterator fit = mPhysXScenes.find(id);
        return (fit != mPhysXScenes.end()) ? fit->second : mDefaultScene;
    }
    void releasePhysXScenes();
    PhysXScene* createPhysXScene(const usdparser::AttachedStage& attachedStage,
                                 size_t sceneId,
                                 double metersPerUnit,
                                 double kilogramsPerUnit,
                                 const usdparser::PhysxSceneDesc& sceneDesc);

    const PhysXScenesMap& getPhysXScenes() const
    {
        return mPhysXScenes;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    const FilteredPairsSet& getFilteredPairs() const
    {
        return mFilteredPairs;
    }
    FilteredPairsSet& getFilteredPairs()
    {
        return mFilteredPairs;
    }
    void addFilteredPair(const Pair<uint32_t>& pair)
    {
        FilteredPairsSet::iterator fit = mFilteredPairs.find(pair);
        if (fit != mFilteredPairs.end())
        {
            fit->second++;
        }
        else
        {
            mFilteredPairs[pair] = 1;
        }
    }
    void removeFilteredPair(const Pair<uint32_t>& pair)
    {
        FilteredPairsSet::iterator fit = mFilteredPairs.find(pair);
        if (fit != mFilteredPairs.end())
        {
            if (fit->second <= 1)
            {
                mFilteredPairs.erase(fit);
            }
            else
            {
                fit->second--;
            }
        }
    }
    const CollisionGroupsPairsSet& getCollisionGroupFilteredPairs() const
    {
        return mCollisionsGroupFilteredPairs;
    }
    CollisionGroupsPairsSet& getCollisionGroupFilteredPairs()
    {
        return mCollisionsGroupFilteredPairs;
    }
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
    // create/release dispatcher
    void createCpuDispatcher(uint32_t numThreads);
    void releaseCpuDispatcher();

    bool isPhysXCpuDispatcher() const
    {
        return mPhysXDispatcher;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // PVD
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

    OmniPvdRecordingResult startOmniPvdRecording(const OmniPvdDestination& destination);
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
        for (PhysXScenesMap::const_reference ref : mPhysXScenes)
        {
            if (ref.second->getUpdateType() == usdparser::eAsynchronous)
                return true;
        }
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // If any scene has not enabled it return false
    bool isReadbackSuppressed() const
    {
        for (PhysXScenesMap::const_reference ref : mPhysXScenes)
        {
            if (!ref.second->isReadbackSuppressed())
                return false;
        }
        return true;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void enableActiveActors(bool val)
    {
        for (PhysXScenesMap::const_reference ref : mPhysXScenes)
        {
            ref.second->getScene()->setFlag(::physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS, val);
        }
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
    void setupGPU();

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
    void clearCudaContextManagers();
    void clearCudaContextManagersAndRefillWithNull(int numToFill);
    // Caller must already hold mCudaContextManagerMutex.
    void clearCudaContextManagersLocked();

    ::physx::PxFoundation* mFoundation{ nullptr };
    ::physx::PxPhysics* mPhysics{ nullptr };
    std::vector<::physx::PxCudaContextManager*> mCudaContextManagers;
    // Guards every mutation of mCudaContextManagers (release, replace, resize) against
    // acquireCudaContextManager(), which cooking calls from UJITSO/carb::tasking worker threads.
    // std::mutex rather than carb::tasking::MutexWrapper on purpose: the critical sections here
    // never yield (a vector access plus an atomic refcount bump, or a release loop), so there is
    // no fiber to reschedule, and this header must not pull in carb tasking.
    mutable std::mutex mCudaContextManagerMutex;
    size_t mNextCudaContextManagerId{ 0 };
    int mSceneMultiGPUMode{ (int)SceneMultiGPUMode::eDisabled };
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

    ::physx::PxCpuDispatcher* mDispatcher{ nullptr };
    uint32_t mThreadCount{ 8 };
    bool mPhysXDispatcher{ false };

    bool mCudaLaunchSynchronous{ false };
    bool mCudaSettingsDeferralWarned{ false };       // sticky-log guard for deferred sync-launch/multiGPU setting change, OMPE-95128
    bool mCudaDeviceCountDeferralWarned{ false };    // sticky-log guard for deferred device-count change, OMPE-95128
    // OMPE-102199: releasePhysXScenes() moves mPhysXScenes aside before destroying the scenes, so the map is
    // empty while live PhysXScenes still cache the PxCudaContextManager. Set for that window so setupGPU()'s
    // hasLiveScenes check keeps deferring manager changes until the last scene is really gone.
    bool mReleasingPhysXScenes{ false };
    bool mExtensionsInitialized{ false };
    bool mVehicleSDKInitialized{ false }; // guards against multiple de-init, since scene release can be called
                                          // more than once (TODO: investigate why); also guards multiple init,
                                          // though that hasn't been observed.
    PhysXScenesMap mPhysXScenes;
    PhysXScene* mDefaultScene; // A.B. should be removed once we support multiple scenes everywhere

    mutable ::physx::PxConvexMesh* mCylinderMeshX{ nullptr };
    mutable ::physx::PxConvexMesh* mCylinderMeshY{ nullptr };
    mutable ::physx::PxConvexMesh* mCylinderMeshZ{ nullptr };
    mutable ::physx::PxConvexMesh* mConeMeshX{ nullptr };
    mutable ::physx::PxConvexMesh* mConeMeshY{ nullptr };
    mutable ::physx::PxConvexMesh* mConeMeshZ{ nullptr };
    mutable ::physx::PxConvexMesh* mVehicleWheelCylinderMeshX{ nullptr };
    mutable ::physx::PxConvexMesh* mVehicleWheelCylinderMeshY{ nullptr };
    mutable ::physx::PxConvexMesh* mVehicleWheelCylinderMeshZ{ nullptr };

    FilteredPairsSet mFilteredPairs;
    CollisionGroupsPairsSet mCollisionsGroupFilteredPairs;
    // Subscriptions to settings changes
    std::vector<carb::dictionary::SubscriptionId*> mSubscribedSettings;
};

} // namespace physx
} // namespace omni
