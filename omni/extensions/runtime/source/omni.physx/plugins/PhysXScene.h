// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "internal/InternalScene.h"

#include <atomic>
#include <vector>

#include <carb/tasking/TaskingUtils.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;
}
class PhysXSetup;
class OmniPhysX;
class PhysXScene;
class ContactReport;

class VehicleUpdateTask;

class OmniFilterCallback : public ::physx::PxSimulationFilterCallback
{
    // PT: the old API uses PxU32s for pairIDs. We can just re-route that to the new API and keep supporting both.

    virtual ::physx::PxFilterFlags pairFound(::physx::PxU32 pairID,
                                             ::physx::PxFilterObjectAttributes attributes0,
                                             ::physx::PxFilterData filterData0,
                                             const ::physx::PxActor* a0,
                                             const ::physx::PxShape* s0,
                                             ::physx::PxFilterObjectAttributes attributes1,
                                             ::physx::PxFilterData filterData1,
                                             const ::physx::PxActor* a1,
                                             const ::physx::PxShape* s1,
                                             ::physx::PxPairFlags& pairFlags);

    virtual void pairLost(::physx::PxU32 pairID,
                          ::physx::PxFilterObjectAttributes attributes0,
                          ::physx::PxFilterData filterData0,
                          ::physx::PxFilterObjectAttributes attributes1,
                          ::physx::PxFilterData filterData1,
                          bool objectRemoved)
    {
    }

    virtual bool statusChange(::physx::PxU32& pairID, ::physx::PxPairFlags& pairFlags, ::physx::PxFilterFlags& filterFlags)
    {
        bool ret = false;

        return ret;
    }

    // PT: the new API uses PxU64s for pairIDs

    virtual ::physx::PxFilterFlags pairFound(::physx::PxU64 pairID,
                                             ::physx::PxFilterObjectAttributes attributes0,
                                             ::physx::PxFilterData filterData0,
                                             const ::physx::PxActor* a0,
                                             const ::physx::PxShape* s0,
                                             ::physx::PxFilterObjectAttributes attributes1,
                                             ::physx::PxFilterData filterData1,
                                             const ::physx::PxActor* a1,
                                             const ::physx::PxShape* s1,
                                             ::physx::PxPairFlags& pairFlags);

    virtual void pairLost(::physx::PxU64 pairID,
                          ::physx::PxFilterObjectAttributes attributes0,
                          ::physx::PxFilterData filterData0,
                          ::physx::PxFilterObjectAttributes attributes1,
                          ::physx::PxFilterData filterData1,
                          bool objectRemoved)
    {
    }

    virtual bool statusChange(::physx::PxU64& pairID, ::physx::PxPairFlags& pairFlags, ::physx::PxFilterFlags& filterFlags)
    {
        bool ret = false;

        return ret;
    }

public:
    bool mInvertedCollisionFilter{ false };
    const OmniPhysX* mOmniPhysX{ nullptr };
    ContactReport* mContactReport{ nullptr };
};

class OmniContactReportCallback : public ::physx::PxSimulationEventCallback, public ::physx::PxContactModifyCallback
{
    virtual void onConstraintBreak(::physx::PxConstraintInfo* constraints, ::physx::PxU32 count) final;
    virtual void onWake(::physx::PxActor** actors, ::physx::PxU32 count) final;
    virtual void onSleep(::physx::PxActor** actors, ::physx::PxU32 count) final;
    virtual void onTrigger(::physx::PxTriggerPair* pairs, ::physx::PxU32 count) final;
    virtual void onAdvance(const ::physx::PxRigidBody* const* bodyBuffer,
                           const ::physx::PxTransform* poseBuffer,
                           const ::physx::PxU32 count) final;
    virtual void onContact(const ::physx::PxContactPairHeader& pairHeader,
                           const ::physx::PxContactPair* pairsIn,
                           ::physx::PxU32 countIn) final;
    virtual void onContactModify(::physx::PxContactModifyPair* pairs, ::physx::PxU32 count) final;

    bool isFilteredEvent(const ::physx::PxShape* triggerShape, const ::physx::PxShape* otherShape);

public:
    OmniPhysX* mOmniPhysX{ nullptr };
    PhysXScene* mPhysXScene{ nullptr };
    ContactReport* mContactReport{ nullptr };

    float mStepSize{ 1.0f / 60.0f };
};

class VehicleUpdateSyncTask : public ::physx::PxLightCpuTask
{
private:
    VehicleUpdateSyncTask();

public:
    VehicleUpdateSyncTask(::physx::PxSync& sync) : mSync(sync)
    {
    }

    virtual ~VehicleUpdateSyncTask()
    {
    }

    virtual void run()
    {
    }

    virtual void release()
    {
        // no need to call release() of base class as there is no continuation task
        mSync.set();
    }

    virtual const char* getName() const
    {
        return "PhysXVehicleUpdateSync";
    }

private:
    ::physx::PxSync& mSync;
};

class PhysXStepper : public ::physx::PxLightCpuTask
{
public:
    PhysXStepper();

    virtual ~PhysXStepper();

    float getStepSize() const
    {
        return mStepSize;
    }

    void launch(int nbSubsteps,
                float stepSize,
                PhysXScene* scene,
                ::physx::PxTaskManager* taskManager,
                bool substeppingEnabled,
                bool forceAsync,
                bool lastStep);

    void updateForces();
    void updateQuasistatic();
    void updateVehicles();
    void updateParticles(int step, int numSteps);
    void updateDeformables(int step, int numSteps);

    virtual void run();

    void waitForCompletion(bool sendPostEvents = true);

    bool isComplete() const
    {
        return mComplete;
    }

    virtual const char* getName() const
    {
        return "PhysXStepper";
    }

    static void customizeVehicleSimulationContext(::physx::vehicle2::PxVehiclePhysXSimulationContext&,
                                                  const PhysXActorVehicleBase&,
                                                  PhysXSetup&);

private:
    void createVehicleUpdateTasks();
    void updateQuasistaticActors(const ::physx::PxU32 numActors, ::physx::PxActor** actors);

    carb::tasking::MutexWrapper mCompleteLock;
    carb::tasking::ConditionVariableWrapper mCompleteCV;

    bool mComplete;
    int mNbSubsteps;
    float mStepSize;
    PhysXScene* mPhysXScene;
    bool mUpdateVelocities;
    bool mForceAsync;
    bool mSendStepping;
    bool mLastStep;
    bool mSubsteppingEnabled;
    bool mSkipSimulation;
    ::physx::PxSceneQueryUpdateMode::Enum mSceneQueryUpdateMode;

    // vehicle update
    std::vector<VehicleUpdateTask*> mVehicleUpdateTasks;
    VehicleUpdateSyncTask mVehicleUpdateSyncTask;
    ::physx::PxSync mVehicleSync;
    std::atomic<int32_t> mVehicleProcessingIndex;
};


class PhysXScene
{
public:
    // Get PhysX scene
    ::physx::PxScene* getScene() const
    {
        return mScene;
    }

    // Get internal scene
    internal::InternalScene* getInternalScene() const
    {
        return mInternalScene;
    }

    const pxr::SdfPath& getSceneSdfPath() const
    {
        return mSceneSdfPath;
    }

    // Controller manager
    ::physx::PxControllerManager* getControllerManager() const
    {
        return mControllerManager;
    }

    ::physx::PxMaterial* getDefaultMaterial() const
    {
        return mMaterial;
    }
    void resetDefaultMaterial();        
    const pxr::SdfPath& getDefaultMaterialPath() const
    {
        return mMaterialPath;
    }

    ::physx::PxFEMSoftBodyMaterial* getDefaultFEMSoftBodyMaterialDeprecated() const
    {
        return mSoftBodyMaterialDeprecated;
    }
    const pxr::SdfPath& getDefaultFEMSoftBodyMaterialPathDeprecated() const
    {
        return mSoftBodyMaterialPathDeprecated;
    }

    ::physx::PxDeformableVolumeMaterial* getDefaultVolumeDeformableMaterial() const
    {
        return mVolumeDeformableMaterial;
    }
    const pxr::SdfPath& getDefaultVolumeDeformableMaterialPath() const
    {
        return mVolumeDeformableMaterialPath;
    }

    ::physx::PxDeformableSurfaceMaterial* getDefaultDeformableSurfaceMaterialDeprecated() const
    {
        return mDeformableSurfaceMaterialDeprecated;
    }
    const pxr::SdfPath& getDefaultDeformableSurfaceMaterialPathDeprecated() const
    {
        return mDeformableSurfaceMaterialPathDeprecated;
    }

    ::physx::PxDeformableSurfaceMaterial* getDefaultSurfaceDeformableMaterial() const
    {
        return mSurfaceDeformableMaterial;
    }
    const pxr::SdfPath& getDefaultSurfaceDeformableMaterialPath() const
    {
        return mSurfaceDeformableMaterialPath;
    }

    ::physx::PxPBDMaterial* getDefaultPBDMaterial() const
    {
        return mPBDMaterial;
    }
    const pxr::SdfPath& getDefaultPBDMaterialPath() const
    {
        return mPBDMaterialPath;
    }

    uint32_t getTimeStepsPerSeconds() const
    {
        return mTimeStepsPerSeconds;
    }
    void setTimeStepsPerSeconds(uint32_t val)
    {
        mTimeStepsPerSeconds = val;
    }

    usdparser::SceneUpdateType getUpdateType() const
    {
        return mSceneUpdateType;
    }
    void setUpdateType(usdparser::SceneUpdateType type)
    {
        mSceneUpdateType = type;
    }

    bool isReadbackSuppressed() const
    {
        return mScene && mScene->getFlags().isSet(::physx::PxSceneFlag::eENABLE_DIRECT_GPU_API);
    }

    bool isInvertedCollisionGroupFilter() const
    {
        return mInvertedCollisionGroupFilter;
    }

    OmniContactReportCallback& getContactReportCallback()
    {
        return mContactReportCallback;
    }

    bool isSceneQuerySupported() const
    {
        return mSupportSceneQuery;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Contact report
    const ContactReport* getContactReport() const
    {
        return mContactReport;
    }
    ContactReport* getContactReport()
    {
        return mContactReport;
    }

    void waitForCompletion(bool doPostWork = true)
    {
        mPhysXStepper.waitForCompletion(doPostWork);
    }

    bool isComplete() const
    {
        return mPhysXStepper.isComplete();
    }

    void computeSubstepping(float elapsedSecs, float fixedTimeStep, float timestepsPerSecond);
    void setSubstepping(uint32_t steps, float elapsedSecs)
    {
        mRemaining = 0.0f;
        mCurrentStep = steps;
        mCurrentTimeStep = elapsedSecs;
        if (steps > 1)
        {
            mSubsteppingEnabled = true;
        }
        else
        {
            mSubsteppingEnabled = false;
        }
    }

    void launch(bool forceAsync);
    void step();

    void decreaseCurrentStep()
    {
        CARB_ASSERT(mCurrentStep > 0);
        mCurrentStep--;
    }
    uint32_t getCurrentStep() const
    {
        return mCurrentStep;
    }
    float getCurrentTimeStep() const
    {
        return mCurrentTimeStep;
    }

    void updateMirroredBodies();

    ::physx::PxSceneQueryUpdateMode::Enum getSceneQueryUpdateMode() const
    {
        return mSceneQueryUpdateMode;
    }

    static PhysXScene* createPhysXScene(const usdparser::AttachedStage& attachedStage,
                                        size_t sceneId,
                                        PhysXSetup&,
                                        double metersPerUnit,
                                        double kilogramsPerUnit,
                                        const usdparser::PhysxSceneDesc& sceneDesc);

    bool isNewScene() const
    {
        return mNewScene;
    }

    const usdparser::AttachedStage& getAttachedStage() const
    {
        return mAttachedStage;
    }

    bool isFullGpuPipelineAvailable()
    {
        if (!mScene)
            return false;

        ::physx::PxSceneFlags sceneFlags = mScene->getFlags();
        ::physx::PxBroadPhaseType::Enum bpType = mScene->getBroadPhaseType();
        return (sceneFlags & ::physx::PxSceneFlag::eENABLE_GPU_DYNAMICS && bpType == ::physx::PxBroadPhaseType::eGPU);
    }

private:
    friend class PhysXSetup;
    PhysXScene(const usdparser::AttachedStage& attachedStage);
    ~PhysXScene();

    const usdparser::AttachedStage& mAttachedStage;
    ::physx::PxScene* mScene;
    ::physx::PxControllerManager* mControllerManager;
    internal::InternalScene* mInternalScene;
    uint32_t mTimeStepsPerSeconds;
    usdparser::SceneUpdateType mSceneUpdateType;
    bool mInvertedCollisionGroupFilter;
    bool mSupportSceneQuery;
    pxr::SdfPath mSceneSdfPath;

    // default materials
    ::physx::PxMaterial* mMaterial;
    pxr::SdfPath mMaterialPath;
    ::physx::PxFEMSoftBodyMaterial* mSoftBodyMaterialDeprecated;
    pxr::SdfPath mSoftBodyMaterialPathDeprecated;
    ::physx::PxDeformableSurfaceMaterial* mDeformableSurfaceMaterialDeprecated;
    pxr::SdfPath mDeformableSurfaceMaterialPathDeprecated;
    ::physx::PxDeformableVolumeMaterial* mVolumeDeformableMaterial;
    pxr::SdfPath mVolumeDeformableMaterialPath;
    ::physx::PxDeformableSurfaceMaterial* mSurfaceDeformableMaterial;
    pxr::SdfPath mSurfaceDeformableMaterialPath;
    ::physx::PxPBDMaterial* mPBDMaterial;
    pxr::SdfPath mPBDMaterialPath;

    OmniFilterCallback mFilterCallback;
    OmniContactReportCallback mContactReportCallback;

    ::physx::PxSceneQueryUpdateMode::Enum mSceneQueryUpdateMode;

    // simulation stepping
    float mRemaining;
    bool mSubsteppingEnabled;
    uint32_t mCurrentStep;
    float mCurrentTimeStep;
    PhysXStepper mPhysXStepper;
    bool mNewScene;

    // Contact report
    ContactReport* mContactReport;
};

bool isRigidBodyDynamic(omni::physx::usdparser::ObjectId id);


} // namespace physx
} // namespace omni
