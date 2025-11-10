// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <PxPhysicsAPI.h>
#include <carb/events/IEvents.h>
#include <carb/settings/ISettings.h>
#include "utils/Pair.h"

#include "PhysXScene.h"
#include "SceneMultiGPUMode.h"

class CarbPhysXErrorCallback;

namespace cookingdataasync
{
class CookingDataAsync;
}

namespace omni
{
namespace physx
{
struct IPhysxCookingService;
struct IPhysxCookingServicePrivate;
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
class PhysXSetup
{
public:
    PhysXSetup();
    ~PhysXSetup();

    ///////////////////////////////////////////////////////////////////////////////////////
    // Tolerances scales
    ::physx::PxTolerancesScale getDefaultTolerances(double metersPerUnit);
    ::physx::PxTolerancesScale getDefaultTolerances(pxr::UsdStageRefPtr stage);

    ///////////////////////////////////////////////////////////////////////////////////////
    // Cooking params
    ::physx::PxCookingParams getCookingParams(const ::physx::PxTolerancesScale& tolerances);
    ::physx::PxCookingParams getStageDefaultCookingParams(pxr::UsdStageRefPtr stage)
    {
        return getCookingParams(getDefaultTolerances(stage));
    }
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
    ::physx::PxCudaContextManager* getCudaContextManager(size_t id = 0) const;
    ::physx::PxCudaContextManager* getNextCudaContextManager();
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

    inline const ::physx::vehicle2::PxVehiclePvdAttributeHandles* getVehiclePvdRegistrationHandles() const
    {
        return mVehiclePvdRegistrationHandles;
    }

    ::physx::PxConvexMesh* getCylinderConvexMesh(enum omni::physx::usdparser::Axis axis) const;
    ::physx::PxConvexMesh* getConeConvexMesh(enum omni::physx::usdparser::Axis axis) const;
    ::physx::PxConvexMesh* getVehicleWheelCylinderConvexMesh(enum omni::physx::usdparser::Axis axis) const;

    ///////////////////////////////////////////////////////////////////////////////////////
    // If any scene is async return true
    bool isAsyncSimEnabled() const
    {
        for (PhysXScenesMap::const_reference ref : mPhysXScenes)
        {
            if (ref.second->getUpdateType() == usdparser::Asynchronous)
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
    void cleanupPhysics(); // called with scene destruction, this should not be required in the end
    void setOmniPVDoutputDirectory(const char* directory);
    void setOmniPVDTimeStampedFileName(const char* fileName);
    void writeOutOmniPVDFile(bool omniPVDWasActive);
    void releaseVehiclePvdRegistrationHandles();
    void clearCudaContextManagers();
    void clearCudaContextManagersAndRefillWithNull(int numToFill);

    ::physx::PxFoundation* mFoundation{ nullptr };
    ::physx::PxPhysics* mPhysics{ nullptr };
    std::vector<::physx::PxCudaContextManager*> mCudaContextManagers;
    size_t mNextCudaContextManagerId{ 0 };
    int mSceneMultiGPUMode{ (int)SceneMultiGPUMode::eDisabled };
    ::physx::PxSerializationRegistry* mSerializationRegistry{ nullptr };
    carb::settings::ISettings* mISettings{ nullptr };

    ::physx::PxDefaultAllocator mAllocator;
    CarbPhysXErrorCallback* mErrorCallback{ nullptr };

    ::physx::PxTolerancesScale mDefaultScale;
    ::physx::PxCookingParams mDefaultCookingParams{ mDefaultScale };
    cookingdataasync::CookingDataAsync* mCookingDataAsync{ nullptr };
    omni::physx::IPhysxCookingServicePrivate* mCookingServicePrivate{ nullptr };
    omni::physx::IPhysxCookingService* mCookingService{ nullptr };
    omni::physx::PhysxCookingAsyncContext mCookingServiceContext{ nullptr };
    omni::physx::IPhysxFoundation* mPhysxFoundation{ nullptr };

    ::physx::PxPvd* mVisualDebugger{ nullptr };
    ::physx::PxPvdTransport* mPvdTransport{ nullptr };
    ::physx::PxOmniPvd* mOmniPvd{ nullptr };
    ::physx::vehicle2::PxVehiclePvdAttributeHandles* mVehiclePvdRegistrationHandles{ nullptr };
    bool mWasOmniPVDSimStarted{ false };
    std::string mOmniPVDOutputDirectory;
    std::string mOmniPVDTimeStampedFileName;

    ::physx::PxCpuDispatcher* mDispatcher{ nullptr };
    uint32_t mThreadCount{ 8 };
    bool mPhysXDispatcher{ false };

    bool mCudaLaunchSynchronous{ false };
    bool mExtensionsInitialized{ false };
    bool mVehicleSDKInitialized{ false }; // for some reason scene release calls can get called multiple times. This
                                          // bool should prevent multiple de-initialization. There is a task to
                                          // investigate this. For now, this bool is even used to guard against multiple
                                          // initialization but did not encounter this yet.
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
