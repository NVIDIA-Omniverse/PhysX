// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS

#include "OmniPhysX.h"
#include "internal/InternalPhysXDatabase.h"
#include "MeshCache.h"
#include "propertiesUpdate/PhysXPropertiesUpdate.h"
#include "PhysXScene.h"
#include "PhysXTools.h"
#include "PhysXUnitTestsIface.h"
#include "PhysXUpdate.h"
#include "usdInterface/UsdInterface.h"
#include "Raycast.h"
#include "Setup.h"
#include "PhysXSceneQuery.h"
#include "PhysXPropertyQuery.h"
#include "PhysXCooking.h"
#include "PhysXSettings.h"
#include "attachment/PhysXAttachmentDeprecated.h"
#include "attachment/PhysXAttachment.h"
#include "attachment/PhysXPoissonSampling.h"
#include "attachment/PhysXTetFinder.h"
#include "attachment/PhysXPointFinder.h"
#include "particles/PhysXParticlePost.h"
#include "particles/FabricParticles.h"
#include "particles/PhysXParticleSampling.h"
#include "PhysXSimulationCallbacks.h"
#include "CookingDataAsync.h"
#include "ContactReport.h"
#include "Trigger.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/Scene.h"
#include "ObjectDataQuery.h"
#include "PhysXPrivate.h"
#include "ScopedNoticeLock.h"

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>
#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxVisualization.h>
#include <private/omni/physx/IPhysxVisualizationPrivate.h>
#include <omni/physx/IPhysxSimulation.h>
#include <private/omni/physx/IPhysxAttachmentPrivate.h>
#include <private/omni/physx/IPhysxParticlesPrivate.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/IPhysxJoint.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/physx/IPhysxCustomJoint.h>
#include <omni/physx/IPhysxCustomGeometry.h>
#include <omni/physx/IPhysxPropertyQuery.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <omni/physx/IPhysxReplicator.h>
#include <omni/physx/IPhysxFoundation.h>
#include <private/omni/physx/IPhysxStageUpdate.h>
#include <omni/physx/IPhysxStatistics.h>
#include <private/omni/physics/schema/IUsdPhysics.h>
#include <private/omni/localcache/ILocalCache.h>
#include <omni/kit/KitUpdateOrder.h>
#include <omni/physx/Version.h>

#include <PxPhysicsAPI.h>
#include <extensions/PxCollectionExt.h>

#include "ChangeRegister.h"
#include "VoxelMap.h"
#include "internal/Internal.h"
#include <common/utilities/MemoryMacros.h>
#include "PhysXDebugVisualization.h"
#include "PhysXUSDProperties.h"

using namespace pxr;
using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace cookingdataasync;

PX_COMPILE_TIME_ASSERT(sizeof(PxVec3)==sizeof(GfVec3f));

const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.plugin", "PhysX", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(kPluginImpl,
    omni::physx::IPhysx,
    omni::physx::IPhysxUnitTests,
    omni::physx::IPhysxBenchmarks,
    omni::physx::IPhysxVisualization,
    omni::physx::IPhysxVisualizationPrivate,
    omni::physx::IPhysxSceneQuery,
    omni::physx::IPhysxCooking,
    omni::physx::IPhysxCookingPrivate,
    omni::physx::IPhysxSimulation,
    omni::physx::IPhysxAttachmentPrivate,
    omni::physx::IPhysxPrivate,
    omni::physx::IPhysxParticlesPrivate,
    omni::physx::usdparser::IPhysxUsdLoad,    
    omni::physx::IPhysxCustomJoint,
    omni::physx::IPhysxJoint,
    omni::physx::IPhysxCustomGeometry,
    omni::physx::IPhysxPropertyQuery,
    omni::physx::IPhysxReplicator,
    omni::physx::IPhysxStageUpdate,
    omni::physx::IPhysxStatistics)

CARB_PLUGIN_IMPL_DEPS(
    carb::settings::ISettings,
    carb::scripting::IScripting,
    omni::convexdecomposition::ConvexDecomposition,
    omni::localcache::ILocalCache,
    carb::tasking::ITasking,
    carb::dictionary::IDictionary,
    omni::physics::schema::IUsdPhysics,
    omni::physx::IPhysxCookingService,
    omni::physx::IPhysxCookingServicePrivate,
    omni::physx::IPhysxFoundation,
    carb::stats::IStats)

CARB_ISTATS_GLOBALS()

void resetPhysX()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omniPhysX.releasePhysXScenes();
}

bool updateMaterialDensity(AttachedStage& attachedStage, ObjectId objectId, const TfToken& propertyName, const UsdTimeCode& timeCode)
{
    PhysXType internalType;
    omni::physx::internal::InternalPhysXDatabase& internalPhysXDatabase = OmniPhysX::getInstance().getInternalPhysXDatabase();
    void* objectRecord = internalPhysXDatabase.getRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    InternalDatabase::Record& materialRec = internalPhysXDatabase.getRecords()[objectId];
    float data;
    if (!getValue<float>(attachedStage, materialRec.mPath, propertyName, timeCode, data))
        return true;

    if (internalType == ePTMaterial)
    {
        getPhysXUsdPhysicsInterface().setDirty(true);
        InternalMaterial* intMat = (InternalMaterial*)materialRec.mInternalPtr;
        intMat->mDensity = data;
        for (size_t i = intMat->mShapeIds.size(); i--;)
        {
            PhysXType shapeType;
            void* shapePtr = internalPhysXDatabase.getRecord(shapeType, intMat->mShapeIds[i]);
            if (shapeType == ePTShape)
            {
                PxShape* shape = (PxShape*)shapePtr;
                if (shape->getActor())
                {
                    internalPhysXDatabase.addDirtyMassActor(size_t(shape->getActor()->userData));
                }
            }
            else if (shapeType == ePTCompoundShape)
            {
                PhysXCompoundShape* compoundShape = (PhysXCompoundShape*)shapePtr;
                if (!compoundShape->getShapes().empty())
                {
                    PxShape* pxShape = (PxShape*)compoundShape->getShapes()[0];
                    if (pxShape->getActor())
                    {
                        internalPhysXDatabase.addDirtyMassActor(size_t(pxShape->getActor()->userData));
                    }
                }
            }
        }
    }
    else if (internalType == ePTDeformableVolumeMaterial || internalType == ePTDeformableSurfaceMaterial)
    {
        InternalDeformableMaterial* intMat = (InternalDeformableMaterial*)materialRec.mInternalPtr;
        intMat->mDensity = data;
        for (ObjectId bodyId : intMat->mDeformableIds)
        {
            attachedStage.getPhysXPhysicsInterface()->updateDeformableBodyMass(attachedStage, bodyId);
        }
    }
    return true;
}

void forceLoadPhysicsFromUSD()
{
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    if (stageId)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        omniPhysX.getPhysXSetup().getPhysics(); // make sure we have physics created
        if (omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) != nullptr)
            getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(SdfPath(omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene)));
        else
            getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(SdfPath());

        UsdLoad::getUsdLoad()->update(0.f);
        AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
        if (attachedStage)
        {
            attachedStage->getPhysXPhysicsInterface()->finishSetup(*attachedStage);
            omniPhysX.getInternalPhysXDatabase().updateDirtyMassActors();
        }
    }
    else
    {
        CARB_LOG_ERROR("No USD stage attached.");
    }
}

void flushChanges()
{
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    UsdLoad::getUsdLoad()->flushChanges();
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (attachedStage)
    {
        attachedStage->getPhysXPhysicsInterface()->finalizeArticulations(*attachedStage);
        OmniPhysX::getInstance().getInternalPhysXDatabase().updateDirtyMassActors();
    }
    OmniPhysX::getInstance().getErrorEventStream()->pump();
}

void pauseChangeTracking(bool pause)
{
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    UsdLoad::getUsdLoad()->pauseChangeTracking(stageId, pause);
}

bool isChangeTrackingPaused()
{
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    return UsdLoad::getUsdLoad()->isChangeTrackingPaused(stageId);
}

void releasePhysicsObjects()
{
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    UsdLoad::getUsdLoad()->releasePhysicsObjects(stageId);
}

ObjectId getObjectId(const SdfPath& path, PhysXType type)
{
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (attachedStage)
        return ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(path, type, OmniPhysX::getInstance().getInternalPhysXDatabase(), *attachedStage));
    else
        return kInvalidObjectId;
}

void* getPhysXPtr(const SdfPath& path, PhysXType type)
{
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (attachedStage)
        return (void*)(getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(path, type, OmniPhysX::getInstance().getInternalPhysXDatabase(), *attachedStage));
    else
        return nullptr;
}

uint32_t getPhysXPtrInstanced(const SdfPath& path, void** data, uint32_t dataSize, PhysXType type)
{
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    uint32_t currentIndex = 0;
    if (attachedStage)
    {
        const usdparser::ObjectIdMap* entries = attachedStage->getObjectIds(path);
        if (entries && !entries->empty())
        {
            auto it = entries->begin();
            while (it != entries->end())
            {
                const usdparser::ObjectId objectId = it->second;
                void* ptr = OmniPhysX::getInstance().getInternalPhysXDatabase().getTypedRecord(type, objectId);
                if (ptr && currentIndex < dataSize)
                {
                    data[currentIndex++] = ptr;
                }
                it++;
            }
        }
    }
    return currentIndex;
}

void* getPhysXPtrFast(ObjectId objectId)
{
    const omni::physx::internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    if (objectId < db.getRecords().size())
    {
        const InternalDatabase::Record& record = db.getRecords()[objectId];
        return record.mPtr;
    }
    return nullptr;
}

void* getInternalPtr(const SdfPath& path, PhysXType type)
{
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (attachedStage)
        return (void*)(getObjectDataOrID<ObjectDataQueryType::eINTERNAL_PTR>(path, type, OmniPhysX::getInstance().getInternalPhysXDatabase(), *attachedStage));
    else
        return nullptr;
}

const void* createD6JointAtPath(const SdfPath& jointPath, const SdfPath& body0,const SdfPath& body1)
{
    omni::physx::internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PxRigidActor* actor0 = (PxRigidActor *)getPhysXPtr(body0, ePTActor);
    if (!actor0)
        actor0 = (PxRigidActor *)getPhysXPtr(body0, ePTLink);

    PxRigidActor* actor1 = (PxRigidActor*)getPhysXPtr(body1, ePTActor);
    if (!actor1)
        actor1 = (PxRigidActor *)getPhysXPtr(body1, ePTLink);

    PxTransform localPose0(PxIdentity);
    PxTransform localPose1(PxIdentity);

    PxJoint* j = PxD6JointCreate(*OmniPhysX::getInstance().getPhysXSetup().getPhysics(), actor0, localPose0, actor1, localPose1);

    if (j)
    {
        InternalJoint* intJoint = ICE_NEW(InternalJoint);
        intJoint->mJointType = eJointD6;
        ObjectId index = db.addRecord(ePTJoint, j, intJoint, jointPath);
        j->userData = (void*)(index);
    }
    return j;
}

void releaseD6Joint(void* jointPtr)
{
    omni::physx::internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    if (jointPtr)
    {
        PxJoint* joint = reinterpret_cast<PxJoint*>(jointPtr);
        size_t index = size_t(joint->userData);
        if (index < db.getRecords().size())
        {
            InternalDatabase::Record& objectRecord = db.getRecords()[index];
            if (objectRecord.mPtr == jointPtr)
            {
                InternalJoint* intJoint = (InternalJoint*)objectRecord.mInternalPtr;
                SAFE_DELETE_SINGLE(intJoint);
                objectRecord.setRemoved();
                joint->release();
            }
        }
    }
}

SubscriptionId subscribeToPhysicsOnStepEvents(bool preStep, int order, OnPhysicsStepEventFn onUpdate, void* userData)
{
    return OmniPhysX::getInstance().addOnStepEventSubscription(onUpdate, userData, preStep, order);
}

void unsubscribeToPhysicsOnStepEvents(SubscriptionId subscriptionId)
{
    OmniPhysX::getInstance().removeOnStepEventSubscription(subscriptionId);
}

SubscriptionId subscribeToPhysicsSimulationEvents(OnPhysicsSimulationEventFn onEvent, void* userData)
{
    return OmniPhysX::getInstance().addStatusEventSubscription(onEvent, userData);
}

void unsubscribeToPhysicsSimulationEvents(SubscriptionId subscriptionId)
{
    OmniPhysX::getInstance().removeStatusEventSubscription(subscriptionId);
}

SdfPath getPhysXObjectUsdPath(ObjectId objectId)
{
    const omni::physx::internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    if (objectId < db.getRecords().size())
    {
        const InternalDatabase::Record& record = db.getRecords()[objectId];
        return record.mPath;
    }
    return SdfPath();
}

// Updates transformations for a specific physX scene. If scenePath is empty, all scenes except the ones marked as
// 'disabled' have their transformations updated. If scenePath is not empty, only that scene has its transformations updated.
void updateTransformationsInternal(const SdfPath& scenePath, bool updateToUSD, bool updateVelocitiesToUsd,  bool outputVelocitiesLocalSpace)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    CARB_PROFILE_ZONE(0, "updateRenderTransforms");
    const bool updateParticlesToUsd = omniPhysX.getISettings()->getAsBool(kSettingUpdateParticlesToUsd);
    const bool updateResidualsToUsd = omniPhysX.getISettings()->getAsBool(kSettingUpdateResidualsToUsd);

    const PhysXScenesMap& physxScenes = omniPhysX.getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;

        // check if this scene must be skipped
        if (scenePath.IsEmpty())
        {
            // update all scenes in the simulation if not specifically disabled
            if (sc->getUpdateType() == Disabled)
                continue;
        }
        else
        {
            // update only the scene that was specifically requested
            if (sc->getSceneSdfPath() != scenePath)
                continue;
        }


        sc->getInternalScene()->updateSimulationOutputs(updateToUSD, updateVelocitiesToUsd, outputVelocitiesLocalSpace, updateParticlesToUsd, updateResidualsToUsd);
    }

    {        
        OmniPhysX::getInstance().getInternalPhysXDatabase().updateSimulationOutputs(updateResidualsToUsd);
    }
}

static void updateTransformations(bool useFaceCache, bool updateToUSD, bool updateVelocitiesToUsd, bool outputVelocitiesLocalSpace)
{
    updateTransformationsInternal(SdfPath() , updateToUSD, updateVelocitiesToUsd, outputVelocitiesLocalSpace);
}

static void updateTransformationsScene(uint64_t scenePath, bool updateToUSD, bool updateVelocitiesToUsd)
{
    updateTransformationsInternal(intToSdfPath(scenePath), updateToUSD, updateVelocitiesToUsd, false /* unused, only left for API compatibility reasons */);
}

static void startSimulation()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omniPhysX.setSimulationStarted(true);
    if(omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) != nullptr)
        getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(SdfPath(omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene)));
    else
        getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(SdfPath());

    getPhysXUsdPhysicsInterface().enableObjectChangeNotifications(false);

    UsdLoad::getUsdLoad()->update(0.0f);

    getPhysXUsdPhysicsInterface().enableObjectChangeNotifications(true);
    // now the initial load is done and notifications should be sent

    const long stageId = OmniPhysX::getInstance().getStageId();
    usdparser::UsdLoad::getUsdLoad()->pauseChangeTracking(stageId, false);

    omniPhysX.sendSimulationEvent(SimulationEvent::eResumed);
    omniPhysX.setSimulationRunning(true);
    omniPhysX.getPhysXSetup().resetPhysXErrorCounter();
}

carb::events::IEventStreamPtr getErrorEventStream()
{
    return OmniPhysX::getInstance().getErrorEventStream();
}

void endSimulation()
{
    OmniPhysX::getInstance().resetSimulation();
}

static long loadTargetStage(const char* path)
{
    long stageId = 0;
    if (path)
    {
        auto stagePtr = UsdStage::Open(path);
        UsdUtilsStageCache::Get().Insert(stagePtr);
        stageId = UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt();
        OmniPhysX::getInstance().physXAttach(stageId, true);
    }
    else
    {
        UsdStageRefPtr stage = OmniPhysX::getInstance().getStage();
        if (stage)
        {
            auto stageErase = stage;
            OmniPhysX::getInstance().physXDetach();
            UsdUtilsStageCache::Get().Erase(stageErase);
        }
    }

    return stageId;
}

static long int createEmptyStage()
{
    UsdStageRefPtr stage = UsdStage::CreateNew("default.usd");
    return UsdUtilsStageCache::Get().Insert(stage).ToLongInt();
}

static void detachStage()
{
    waitForSimulationCompletion(false);
    if (OmniPhysX::getInstance().getStage())
    {
        OmniPhysX::getInstance().physXDetach();
    }
}


static bool loadTargetStage_Id(long stageId)
{
    detachStage();

    if (stageId)
    {
        UsdStageRefPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();

        if (!stage)
            return false;

        omniPhysX.physXAttach(stageId, true);
    }

    return true;
}

bool physxSimulationAttach(long stageId)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if (stageId && omniPhysX.isSimulationAttachedStage() && stageId == omniPhysX.getStageId())
    {
        CARB_LOG_ERROR("Stage %d already attached.", stageId);
        return false;
    }

    omniPhysX.setSimulationRunning(true);

    omniPhysX.getStageUpdate().detachStageUpdate();

    detachStage();

    omniPhysX.setSimulationAttachedStage(true);
    return loadTargetStage_Id(stageId);
}

void physxSimulationDetach()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    detachStage();

    omniPhysX.getStageUpdate().attachStageUpdate();

    omniPhysX.setSimulationRunning(false);
    omniPhysX.setSimulationAttachedStage(false);
}

static long getPhysxSimulationAttachedStage()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if (omniPhysX.isSimulationAttachedStage())
    {
        return omniPhysX.getStageId();
    }
    return 0l;
}

void addCrashreporterMetadata()
{
    carb::crashreporter::addCrashMetadata("lib_physx_buildVersion", PHYSICS_BUILD_VERSION);
    carb::crashreporter::addCrashMetadata("lib_physx_buildRepo", "");
    carb::crashreporter::addCrashMetadata("lib_physx_buildHash", PHYSICS_BUILD_SHA);
    carb::crashreporter::addCrashMetadata("lib_physx_buildBranch", PHYSICS_BUILD_BRANCH);
    carb::crashreporter::addCrashMetadata("lib_physx_buildDate", PHYSICS_BUILD_DATE);
}

CARB_EXPORT void carbOnPluginStartup()
{
    carb::stats::registerStatsInterfaceForClient();
    addCrashreporterMetadata();
    OmniPhysX::createOmniPhysXInstance();
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omniPhysX.onStartup();
}

CARB_EXPORT void carbOnPluginShutdown()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omniPhysX.onShutdown();
}

static void resetSettingsInPreferences()
{
    PhysXSettings::getInstance().resetSettingsInPreferences();
}

static void resetSettingsInStage()
{
    PhysXSettings::getInstance().resetSettingsInStage();
}

static void resetSetting(const char* path)
{
    PhysXSettings::getInstance().resetSettingAtPath(path);
}

static void resetSettings()
{
    PhysXSettings::getInstance().resetAllSettings();
}

static void bmOverwriteGPUSetting(bool enableGPU)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omniPhysX.setGpuPipelineOverride(enableGPU ? 1 : 0);
    omniPhysX.getISettings()->setInt(kSettingOverrideGPU, omniPhysX.getGpuPipelineOverride());
    omniPhysX.releasePhysXScenes();
}

static void overwriteGPUSetting(int val)
{
    {
        if (OmniPhysX::getInstance().hasSimulationStarted()) {
            CARB_LOG_WARN("Physics simulation has started. Not possible to set the GPU parameters.");
            return;
        }
    }
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omniPhysX.setGpuPipelineOverride(val);
    omniPhysX.getISettings()->setInt(kSettingOverrideGPU, val);
    resetPhysX();
}

static int getOverwriteGPUSetting()
{
    return OmniPhysX::getInstance().getGpuPipelineOverride();
}

static void overwriteSolverType(int val)
{
    {
        if (OmniPhysX::getInstance().hasSimulationStarted()) {
            CARB_LOG_WARN("Physics simulation has started. Not possible to set the solver type.");
            return;
        }
    }
    OmniPhysX::getInstance().setSolverTypeOverride(val);
    resetPhysX();
}

static void setThreadCount(uint32_t threadCount)
{
    {
        if (OmniPhysX::getInstance().hasSimulationStarted()) {
            CARB_LOG_WARN("Physics simulation has started. Not possible to set the thread count.");
            return;
        }
    }
    OmniPhysX::getInstance().getPhysXSetup().setThreadCount(threadCount);
    OmniPhysX::getInstance().getPhysXSetup().createCpuDispatcher(threadCount);
    OmniPhysX::getInstance().getISettings()->setInt(kSettingNumThreads, threadCount);
    resetPhysX();
}

static void reconnectPVD()
{
    ::physx::PxPvd* pvd = OmniPhysX::getInstance().getPhysXSetup().getPvd();
    if (pvd)
    {
        pvd->disconnect();

        OmniPhysX::getInstance().getPhysXSetup().connectPVD();
    }
}

bool saveSceneToRepX(const char* filepath)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    bool sceneEmpty = false;
    if (db.getRecords().size() == 1)
    {
        sceneEmpty = true;
    }

    // flush changes or load scene
    forceLoadPhysicsFromUSD();

    const PhysXScenesMap& physxScenes = omniPhysX.getPhysXSetup().getPhysXScenes();
    if (physxScenes.empty())
        return false;

    PxCollection* collectionSdk = ::physx::PxCollectionExt::createCollection(*OmniPhysX::getInstance().getPhysXSetup().getPhysics());
    if (!collectionSdk)
        return false;

    PxSerializationRegistry* sr = PxSerialization::createSerializationRegistry(*OmniPhysX::getInstance().getPhysXSetup().getPhysics());

    std::vector<PxCollection*> sceneCollections;
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PxCollection* collectionScene = PxCollectionExt::createCollection(*ref.second->getScene());
        if (!collectionScene)
            continue;

        collectionSdk->add(*collectionScene);
        sceneCollections.push_back(collectionScene);
    }

    PxSerialization::complete(*collectionSdk, *sr, NULL);

    PxDefaultFileOutputStream theStream(filepath);
    PxSerialization::serializeCollectionToXml(theStream, *collectionSdk, *sr);

    for (size_t i = 0; i < sceneCollections.size(); i++)
    {
        sceneCollections[i]->release();
    }
    collectionSdk->release();

    sr->release();

    // release if scene was empty
    if (sceneEmpty)
        releasePhysicsObjects();

    return true;
}

static void bmSetThreadCount(uint32_t threadCount)
{
    OmniPhysX::getInstance().getPhysXSetup().setThreadCount(threadCount);
    OmniPhysX::getInstance().getPhysXSetup().createCpuDispatcher(threadCount);
    OmniPhysX::getInstance().getISettings()->setInt(kSettingNumThreads, threadCount);
    resetPhysX();
}

static void bmEnablePVDProfile(bool enablePVD)
{
    OmniPhysX::getInstance().setPvdProfileEnabled(enablePVD);
}

static void bmEnableProfile(bool enableProfle)
{
    OmniPhysX::getInstance().setOmniPhysXProfilingEnabled(enableProfle);
}

static void bmGetProfileStats(std::vector<PhysicsProfileStats>& stats)
{
    stats.clear();
    const ProfileStatsVector& profileStats = OmniPhysX::getInstance().getOmniPhysXProfileStats();
    for (size_t i = profileStats.size(); i--;)
    {
        stats.push_back(profileStats[i]);
    }

    OmniPhysX::getInstance().getOmniPhysXProfileStats().clear();
    OmniPhysX::getInstance().getOmniPhysXCrossThreadProfileMap().clear();
}

static SubscriptionId bmSubscribeProfileStatsEvents(ProfileStatsNotificationFn onEvent, void* userData)
{
    return OmniPhysX::getInstance().addProfileStatsSubscription(onEvent, userData);
}

static void bmUnSubscribeProfileStatsEvents(SubscriptionId id)
{
    OmniPhysX::getInstance().removeProfileStatsSubscription(id);
}



// Shared functionality for applying simulation interface functions.
typedef void (*applySimFn)(const InternalDatabase::Record* target, void* data);

static bool applySimulationInterfaceFunctionToPointInstancer(uint64_t stageId, uint64_t path, const applySimFn* applyFunction, void* data, uint32_t protoIndex)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
    {
        CARB_LOG_ERROR("SimulationInterface function could not locate any stage with the specified stage ID.");
        return false;
    }
    const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    const ObjectIdMap* entries = attachedStage->getObjectIds(intToPath(path));
    if (!entries)
    {
        CARB_LOG_ERROR("SimulationInterface function could did not locate any objects at the specified path.");
        return false;
    }
    for (const auto& entry : *entries)
    {
        const InternalDatabase::Record& rec = db.getRecords()[entry.second];
        if(rec.mType == ePTPointInstancer)
        {
            const UsdPrim& prim = attachedStage->getStage()->GetPrimAtPath(rec.mPath);
            const UsdGeomPointInstancer pointInst(prim);

            SdfPathVector prototypes;
            pointInst.GetPrototypesRel().GetTargets(&prototypes);
            if(prototypes.size() == 0)
            {
                CARB_LOG_ERROR("SimulationInterface function applied to PointInstancer without prototypes.");
                return false;
            }
            if(protoIndex != 0xffffffff)
            {
                // If protoIndex is not 0xffffffff, we only need to search the prototype associated with this instance.
                VtArray<int> protoIndices;
                if(!pointInst.GetProtoIndicesAttr().Get(&protoIndices))
                {
                    CARB_LOG_ERROR("SimulationInterface function applied to PointInstancer without valid ProtypeIndices attribute.");
                    return false;
                }
                if(protoIndex >= protoIndices.size())
                {
                    CARB_LOG_ERROR("SimulationInterface function applied to PointInstancer but index was outside range. (%d vs %d)", protoIndex, (int) protoIndices.size());
                    return false;
                }
                // Shrink the prototypes to only contain the prototype for this instance.
                prototypes[0] = prototypes[protoIndices[protoIndex]];
                prototypes.resize(1);
            }
            for (size_t i = 0; i < prototypes.size(); i++)
            {
                // Cycle instances of each relevant prototype. This is necessary as each prototype map their instances to actors.
                const usdparser::ObjectIdMap* instanceEntries = attachedStage->getObjectIds(prototypes[i]);
                if (!instanceEntries)
                {
                    continue;
                }
                for(const auto &instanceEntry : *instanceEntries)
                {
                    if (instanceEntry.first == eBody)
                    {
                        usdparser::ObjectId protoObjectId = instanceEntry.second;
                        PhysXType protoInternalType;
                        const InternalDatabase::Record* objectRecord = db.getFullRecord(protoInternalType, protoObjectId);
                        if (objectRecord && protoInternalType == ePTActor)
                        {
                            const InternalActor* internalActor = (const InternalActor*)objectRecord->mInternalPtr;
                            if (protoIndex == 0xffffffff || internalActor->mInstanceIndex == protoIndex)
                            {
                                (*applyFunction)(objectRecord, data);
                                if(protoIndex != 0xffffffff)
                                {
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        else
        {
            CARB_LOG_ERROR("SimulationInterface instanced function applied to target that was not a point instancer.");
            return false;
        }
    }
    return true;
}

static bool applySimulationInterfaceFunction(uint64_t stageId, uint64_t path, const applySimFn* applyFunction, void* data)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
    {
        CARB_LOG_ERROR("SimulationInterface function could not locate any stage with the specified stage ID.");
        return false;
    }
    const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    const ObjectIdMap* entries = attachedStage->getObjectIds(intToPath(path));
    if (!entries)
    {
        CARB_LOG_ERROR("SimulationInterface function could did not locate any objects at the specified path.");
        return false;
    }
    for (const auto& entry : *entries)
    {
        const InternalDatabase::Record* rec = &db.getRecords()[entry.second];
        (*applyFunction)(rec, data);
    }
    return true;
}

struct addForceAtPosData
{
    const carb::Float3& force;
    const carb::Float3& pos;
    ForceModeType::Enum mode;
};

static void addForceAtPosInternal(const InternalDatabase::Record* target, void* data)
{
    if (target->mType == ePTLink || target->mType == ePTActor)
    {
        addForceAtPosData* forceData = reinterpret_cast<addForceAtPosData*>(data);

        PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
        PxRigidBody* body = actor->is<PxRigidBody>();
        if (body && !(body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) && !(body->getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
        {
            PxRigidBodyExt::addForceAtPos(*body, toPhysX(forceData->force), toPhysX(forceData->pos), (PxForceMode::Enum)forceData->mode);
        }
    }
};

static void addForceAtPosInstanced(uint64_t stageId, uint64_t path, const carb::Float3& force, const carb::Float3& pos, ForceModeType::Enum mode, uint32_t protoIndex)
{
    addForceAtPosData forceData = {force, pos, mode};
    const applySimFn function = addForceAtPosInternal;
    if(!applySimulationInterfaceFunctionToPointInstancer(stageId, path, &function, &forceData, protoIndex))
    {
        CARB_LOG_ERROR("Error executing addForceAtPosInstanced.");
    }
}

static void addForceAtPos(uint64_t stageId, uint64_t path, const carb::Float3& force, const carb::Float3& pos, ForceModeType::Enum mode)
{
    addForceAtPosData forceData = {force, pos, mode};
    const applySimFn function = addForceAtPosInternal;
    if(!applySimulationInterfaceFunction(stageId, path, &function, &forceData))
    {
        CARB_LOG_ERROR("Error executing addForceAtPos.");
    }
}

struct addTorqueData
{
    const carb::Float3& torque;
    ForceModeType::Enum mode;
};

static void addTorqueInsternal(const InternalDatabase::Record* target, void* data)
{
    if (target->mType == ePTLink || target->mType == ePTActor)
    {
        addTorqueData* torqueData = reinterpret_cast<addTorqueData*>(data);

        PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
        PxRigidBody* body = actor->is<PxRigidBody>();
        if (body && !(body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) && !(body->getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
        {
            body->addTorque(toPhysX(torqueData->torque), (PxForceMode::Enum)torqueData->mode);
        }
    };
};

static void addTorqueInstanced(uint64_t stageId, uint64_t path, const carb::Float3& torque, uint32_t protoIndex)
{
    addTorqueData torqueData = {torque, ForceModeType::eFORCE};
    const applySimFn function = addTorqueInsternal;
    if(!applySimulationInterfaceFunctionToPointInstancer(stageId, path, &function, &torqueData, protoIndex))
    {
        CARB_LOG_ERROR("Error executing addForceAtPosInstanced.");
    }
}

static void addTorque(uint64_t stageId, uint64_t path, const carb::Float3& torque)
{
    addTorqueData torqueData = {torque, ForceModeType::eFORCE};
    const applySimFn function = addTorqueInsternal;
    if(!applySimulationInterfaceFunction(stageId, path, &function, &torqueData))
    {
        CARB_LOG_ERROR("Error executing addTorque.");
    }
}

struct getRigidBodyTransformData
{
    carb::Float3& pos;
    carb::Float4& rot;
};

static void getRigidBodyTransformationInternal(const InternalDatabase::Record* target, void* data)
{
    if (target->mType == ePTLink || target->mType == ePTActor)
    {
        getRigidBodyTransformData* transformData = reinterpret_cast<getRigidBodyTransformData*>(data);
        const PxRigidActor* actor = reinterpret_cast<const PxRigidActor*>(target->mPtr);
        const PxTransform transform = actor->getGlobalPose();
        transformData->pos = fromPhysX(transform.p);
        transformData->rot = fromPhysX(transform.q);
    }
};

static bool getRigidBodyTransformationInstanced(uint64_t stageId, uint64_t path, carb::Float3& pos, carb::Float4& rot, uint32_t protoIndex)
{
    getRigidBodyTransformData transformData = {pos, rot};
    const applySimFn function = getRigidBodyTransformationInternal;
    if(!applySimulationInterfaceFunctionToPointInstancer(stageId, path, &function, &transformData, protoIndex))
    {
        CARB_LOG_ERROR("Error executing getRigidBodyTransformationInstanced.");
        return false;
    }
    return true;
}

static bool getRigidBodyTransformation(const SdfPath& path, carb::Float3& pos, carb::Float4& rot)
{
    getRigidBodyTransformData transformData = {pos, rot};
    const applySimFn function = getRigidBodyTransformationInternal;
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if(!applySimulationInterfaceFunction(omniPhysX.getStageId(), sdfPathToInt(SdfPath(path)), &function, &transformData))
    {
        CARB_LOG_ERROR("Error executing getRigidBodyTransformation.");
        return false;
    }
    return true;
}

static void wakeUpInternal(const InternalDatabase::Record* target, void* data)
{
    if (target->mType == ePTActor)
    {
        PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
        PxRigidDynamic* body = actor->is<PxRigidDynamic>();
        if (body && !(body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
        {
            body->wakeUp();
        }
    }
    else if (target->mType == ePTLink)
    {
        PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
        PxArticulationLink* link = actor->is<PxArticulationLink>();
        if (link)
        {
            link->getArticulation().wakeUp();
        }
    }
    else if (target->mType == ePTArticulation)
    {
        PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
        PxArticulationReducedCoordinate* art = actor->is<PxArticulationReducedCoordinate>();
        if (art)
        {
            art->wakeUp();
        }
    }
}

static void wakeUpInstanced(uint64_t stageId, uint64_t path, uint32_t protoIndex)
{
    const applySimFn function = wakeUpInternal;
    if(!applySimulationInterfaceFunctionToPointInstancer(stageId, path, &function, nullptr, protoIndex))
    {
        CARB_LOG_ERROR("Error executing wakeUpInstanced.");
    }
}

static void wakeUp(uint64_t stageId, uint64_t path)
{
    const applySimFn function = wakeUpInternal;
    if(!applySimulationInterfaceFunction(stageId, path, &function, nullptr))
    {
        CARB_LOG_ERROR("Error executing wakeUp.");
    }
}

static void putToSleepInternal(const InternalDatabase::Record* target, void* data)
{
    if (target->mType == ePTActor)
    {
        PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
        PxRigidDynamic* body = actor->is<PxRigidDynamic>();
        if (body)
        {
            body->putToSleep();
        }
    }
    else if (target->mType == ePTLink)
    {
        PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
        PxArticulationLink* link = actor->is<PxArticulationLink>();
        if (link)
        {
            link->getArticulation().putToSleep();
        }
    }
    else if (target->mType == ePTArticulation)
    {
        PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
        PxArticulationReducedCoordinate* art = actor->is<PxArticulationReducedCoordinate>();
        if (art)
        {
            art->putToSleep();
        }
    }
}

static void putToSleepInstanced(uint64_t stageId, uint64_t path, uint32_t protoIndex)
{
    const applySimFn function = putToSleepInternal;
    if(!applySimulationInterfaceFunctionToPointInstancer(stageId, path, &function, nullptr, protoIndex))
    {
        CARB_LOG_ERROR("Error executing putToSleepInstanced.");
    }
}

static void putToSleep(uint64_t stageId, uint64_t path)
{
    const applySimFn function = putToSleepInternal;
    if(!applySimulationInterfaceFunction(stageId, path, &function, nullptr))
    {
        CARB_LOG_ERROR("Error executing putToSleep.");
    }
}

static void isSleepingInternal(const InternalDatabase::Record* target, void* data)
{
    bool* sleeping = reinterpret_cast<bool *>(data);
    if(!*sleeping)
    {
        if (target->mType == ePTActor)
        {
            PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
            PxRigidDynamic* body = actor->is<PxRigidDynamic>();
            if (body)
            {
                *sleeping = body->isSleeping();
            }
        }
        else if (target->mType == ePTLink)
        {
            PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
            PxArticulationLink* link = actor->is<PxArticulationLink>();
            if (link)
            {
                *sleeping = link->getArticulation().isSleeping();
            }
        }
        else if (target->mType == ePTArticulation)
        {
            PxActor* actor = reinterpret_cast<PxActor*>(target->mPtr);
            PxArticulationReducedCoordinate* art = actor->is<PxArticulationReducedCoordinate>();
            if (art)
            {
                *sleeping = art->isSleeping();
            }
        }
    }
}

static bool isSleepingInstanced(uint64_t stageId, uint64_t path, uint32_t protoIndex)
{
    bool sleeping = false;
    const applySimFn function = isSleepingInternal;
    if(!applySimulationInterfaceFunctionToPointInstancer(stageId, path, &function, &sleeping, protoIndex))
    {
        CARB_LOG_ERROR("Error executing isSleepingInstanced.");
    }
    return sleeping;
}

static bool isSleeping(uint64_t stageId, uint64_t path)
{
    bool sleeping = false;
    const applySimFn function = isSleepingInternal;
    if(!applySimulationInterfaceFunction(stageId, path, &function, &sleeping))
    {
        CARB_LOG_ERROR("Error executing isSleeping.");
    }
    return sleeping;
}

static int getWheelIndex(const SdfPath& wheelPath)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    InternalVehicleWheelAttachment* wheelAttachment = static_cast<InternalVehicleWheelAttachment*>(getInternalPtr(wheelPath, ePTVehicleWheelAttachment));
    if (wheelAttachment != nullptr)
    {
        if (wheelAttachment->mVehicle)
            return static_cast<int>(wheelAttachment->mWheelIndex);
        else
            return -1;
    }
    else
        return -1;
}

static float computeVehicleVelocity(const usdparser::ObjectId vehicleId,
    const carb::Float3* direction)
{
    if (vehicleId != kInvalidObjectId)
    {
        const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        void* internalObject = db.getInternalTypedRecord(ePTVehicle, vehicleId);
        if (internalObject)
        {
            const InternalVehicle* internalVehicle = static_cast<const InternalVehicle*>(internalObject);
            carb::Float3 dir;
            if (direction)
                dir = *direction;
            else
            {
                const InternalVehicleContext& vehicleContext = internalVehicle->mInternalScene.getVehicleContext();
                const ::physx::vehicle2::PxVehicleFrame& frame = vehicleContext.getFrame();
                dir = fromPhysX(frame.getLngAxis());
            }

            return internalVehicle->computeVelocity(dir);
        }
        else
        {
            CARB_LOG_ERROR("computeVehicleVelocity: internal vehicle object could not be found for provided object ID.\n");
        }
    }
    else
    {
        CARB_LOG_ERROR("computeVehicleVelocity: invalid object ID provided\n");
    }

    return 0.0f;
}

static bool getVehicleDriveState(const usdparser::ObjectId vehicleControllerId,
    VehicleDriveState& driveState)
{
    if (vehicleControllerId != kInvalidObjectId)
    {
        const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        void* internalObject = db.getInternalTypedRecord(ePTVehicleController, vehicleControllerId);
        if (internalObject)
        {
            const InternalVehicle* internalVehicle = static_cast<const InternalVehicle*>(internalObject);
            internalVehicle->getDriveState(driveState);
            return true;
        }
        else
        {
            CARB_LOG_ERROR("getVehicleDriveState: internal vehicle object could not be found for provided object ID.\n");
        }
    }
    else
    {
        CARB_LOG_ERROR("getVehicleDriveState: invalid object ID provided\n");
    }

    return false;
}

static bool getWheelState(const usdparser::ObjectId* wheelAttachmentIds, const uint32_t wheelAttachmentIdCount,
    VehicleWheelState* states)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    for (uint32_t i = 0; i < wheelAttachmentIdCount; i++)
    {
        const usdparser::ObjectId wheelAttId = wheelAttachmentIds[i];
        if (wheelAttId != kInvalidObjectId)
        {
            void* internalObject = db.getInternalTypedRecord(ePTVehicleWheelAttachment, wheelAttId);
            if (internalObject)
            {
                const InternalVehicleWheelAttachment* internalWheelAtt = static_cast<const InternalVehicleWheelAttachment*>(internalObject);
                internalWheelAtt->getWheelState(states[i]);
            }
            else
            {
                CARB_LOG_ERROR("getWheelState: internal wheel attachment object could not be found for provided object ID.\n");
                return false;
            }
        }
        else
        {
            CARB_LOG_ERROR("getWheelState: invalid object ID provided\n");
        }
    }

    return true;
}

static void setWheelRotationSpeed(const usdparser::ObjectId* wheelAttachmentIds, const uint32_t wheelAttachmentIdCount,
    const float* rotationSpeeds)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    for (uint32_t i = 0; i < wheelAttachmentIdCount; i++)
    {
        const usdparser::ObjectId wheelAttId = wheelAttachmentIds[i];
        if (wheelAttId != kInvalidObjectId)
        {
            void* internalObject = db.getInternalTypedRecord(ePTVehicleWheelAttachment, wheelAttId);
            if (internalObject)
            {
                InternalVehicleWheelAttachment* internalWheelAtt = static_cast<InternalVehicleWheelAttachment*>(internalObject);
                internalWheelAtt->setWheelRotationSpeed(rotationSpeeds[i]);
            }
            else
            {
                CARB_LOG_ERROR("setWheelRotationSpeed: internal wheel attachment object could not be found for provided object ID.\n");
            }
        }
        else
        {
            CARB_LOG_ERROR("setWheelRotationSpeed: invalid object ID provided\n");
        }
    }
}

static void setWheelRotationAngle(const usdparser::ObjectId* wheelAttachmentIds, const uint32_t wheelAttachmentIdCount,
    const float* rotationAngles)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    for (uint32_t i = 0; i < wheelAttachmentIdCount; i++)
    {
        const usdparser::ObjectId wheelAttId = wheelAttachmentIds[i];
        if (wheelAttId != kInvalidObjectId)
        {
            void* internalObject = db.getInternalTypedRecord(ePTVehicleWheelAttachment, wheelAttId);
            if (internalObject)
            {
                InternalVehicleWheelAttachment* internalWheelAtt = static_cast<InternalVehicleWheelAttachment*>(internalObject);
                internalWheelAtt->setWheelRotationAngle(rotationAngles[i]);
            }
            else
            {
                CARB_LOG_ERROR("setWheelRotationAngle: internal wheel attachment object could not be found for provided object ID.\n");
            }
        }
        else
        {
            CARB_LOG_ERROR("setWheelRotationAngle: invalid object ID provided\n");
        }
    }
}

static void getInternalSurfaceDeformableBodyData(const usdparser::ObjectId deformableId, InternalSurfaceDeformableBodyData& data)
{
    InternalSurfaceDeformableBody* internalPtr = omni::physx::getInternalPtr<InternalSurfaceDeformableBody>(ePTDeformableSurface, deformableId);
    if (internalPtr)
    {
        data.bodyPrim = internalPtr->mBodyPrim;
        data.simMeshPrim = internalPtr->mSimMeshPrim;
        data.worldToSimMesh = internalPtr->mWorldToSimMesh;

        data.skinMeshPrims = omni::span<UsdPrim>(internalPtr->mSkinMeshPrims.begin(), internalPtr->mSkinMeshPrims.end());
        data.worldToSkinMeshTransforms = omni::span<GfMatrix4f>(internalPtr->mWorldToSkinMeshTransforms.begin(), internalPtr->mWorldToSkinMeshTransforms.end());
        data.skinMeshRanges = omni::span<carb::Uint2>(internalPtr->mSkinMeshRanges.begin(), internalPtr->mSkinMeshRanges.end());

        data.numSkinMeshVertices = internalPtr->mNumSkinMeshVertices;
        data.numSimMeshVertices = internalPtr->mNumSimMeshVertices;

        data.simMeshPositionInvMassH = internalPtr->mSimMeshPositionInvMassH;

        data.allSkinnedVerticesH = internalPtr->mAllSkinnedVerticesH;
        data.allSkinnedVerticesD = internalPtr->mAllSkinnedVerticesD;
    }
}

static void getInternalVolumeDeformableBodyData(const usdparser::ObjectId deformableId, InternalVolumeDeformableBodyData& data)
{
    InternalVolumeDeformableBody* internalPtr = omni::physx::getInternalPtr<InternalVolumeDeformableBody>(ePTDeformableVolume, deformableId);
    if (internalPtr)
    {
        data.bodyPrim = internalPtr->mBodyPrim;
        data.simMeshPrim = internalPtr->mSimMeshPrim;
        data.worldToSimMesh = internalPtr->mWorldToSimMesh;

        data.skinMeshPrims = omni::span<UsdPrim>(internalPtr->mSkinMeshPrims.begin(), internalPtr->mSkinMeshPrims.end());
        data.worldToSkinMeshTransforms = omni::span<GfMatrix4f>(internalPtr->mWorldToSkinMeshTransforms.begin(), internalPtr->mWorldToSkinMeshTransforms.end());
        data.skinMeshRanges = omni::span<carb::Uint2>(internalPtr->mSkinMeshRanges.begin(), internalPtr->mSkinMeshRanges.end());

        data.numSkinMeshVertices = internalPtr->mNumSkinMeshVertices;
        data.numSimMeshVertices = internalPtr->mNumSimMeshVertices;

        data.collMeshPrim = internalPtr->mCollMeshPrim;
        data.worldToCollMesh = internalPtr->mWorldToCollMesh;
        data.numCollMeshVertices = internalPtr->mNumCollMeshVertices;

        data.simMeshPositionInvMassH = internalPtr->mSimMeshPositionInvMassH;
        data.collMeshPositionInvMassH = internalPtr->mCollMeshPositionInvMassH;

        data.allSkinnedVerticesH = internalPtr->mAllSkinnedVerticesH;
        data.allSkinnedVerticesD = internalPtr->mAllSkinnedVerticesD;
    }
}

carb::events::IEventStreamPtr getSimulationEventStreamV2()
{
    return OmniPhysX::getInstance().getSimulationEventStreamV2();
}

void setSimulationLayer(const char* layerIdentifier)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if (layerIdentifier)
    {
        // If someone set already a layer then we dont override with the anonymous sim layer
        if (omniPhysX.getSimulationLayer() && strstr(layerIdentifier, "PhysicsSimulationLayer"))
            return;

        SdfLayerRefPtr layer = SdfLayer::Find(layerIdentifier);
        omniPhysX.setSimulationLayer(layer);
    }
    else
    {
        omniPhysX.setSimulationLayer(nullptr);
    }
}

bool isRunning()
{
    return OmniPhysX::getInstance().isSimulationRunning();
}

static bool isInteractiveActorRaycast(const carb::Float3* origin, const carb::Float3* direction)
{
    if (!isRunning())
        return false;
    return OmniPhysX::getInstance().getRaycastManager().interactiveActorRaycast(origin, direction);
}

static void updateInteraction(const carb::Float3* origin, const carb::Float3* direction, PhysicsInteractionEvent interactionEvent)
{
    if (!isRunning())
        return;
    OmniPhysX::getInstance().getRaycastManager().handleInteractionEvent(reinterpret_cast<const float*>(origin), reinterpret_cast<const float*>(direction), interactionEvent);
}

void runBackwardsCompatibility(long int stageId)
{
    CARB_LOG_ERROR("runBackwardsCompatibility: deprecated\n");
}

bool checkBackwardsCompatibility(long int stageId)
{
    CARB_LOG_ERROR("checkBackwardsCompatibility: deprecated\n");
    return false;
}

const char* getBackwardsCompatibilityCheckLog()
{
    CARB_LOG_ERROR("getBackwardsCompatibilityCheckLog: deprecated\n");
    return "";
}

SdfPath getCollisionGroupFromCollider(const SdfPath& path)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    const PxShape* shape = static_cast<const PxShape*>(getPhysXPtr(path, ePTShape));
    if (shape != nullptr)
    {
        const uint32_t colGroupId = convertCollisionGroupFromPxFilterData(shape->getQueryFilterData());
        if (colGroupId < db.getRecords().size())
        {
            const InternalDatabase::Record& colGroupRec = db.getRecords()[colGroupId];
            if (colGroupRec.mType == ePTCollisionGroup)
            {
                return colGroupRec.mPath;
            }
        }
    }
    return SdfPath();
}

bool isReadbackSuppressed()
{
    // Get default scene
    // A.B. update for multiple scenes
    PhysXScene* ps = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(0);
    return ps && ps->isReadbackSuppressed();
}

void setVehicleToRestState(const usdparser::ObjectId vehicleId)
{
    if (vehicleId != kInvalidObjectId)
    {
        const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        void* internalObject = db.getInternalTypedRecord(ePTVehicle, vehicleId);
        if (internalObject)
        {
            InternalVehicle* internalVehicle = reinterpret_cast<InternalVehicle*>(internalObject);
            internalVehicle->setToRestState();
        }
        else
        {
            CARB_LOG_ERROR("setVehicleToRestState: provided object ID does not point to a vehicle\n");
        }
    }
    else
    {
        CARB_LOG_ERROR("setVehicleToRestState: invalid object ID provided\n");
    }
}

bool getWheelTransformations(const usdparser::ObjectId vehicleId, const int* wheelIndices,
    const unsigned int wheelIndexCount, const bool addVehicleTransform,
    carb::Float3* positions, carb::Float4* orientations)
{
    if (vehicleId != kInvalidObjectId)
    {
        const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        CARB_ASSERT(vehicleId < db.getRecords().size());
        CARB_ASSERT(db.getRecords()[vehicleId].mType == ePTVehicle);

        void* internalObject = db.getInternalTypedRecord(ePTVehicle, vehicleId);
        if (internalObject)
        {
            const InternalVehicle* internalVehicle = reinterpret_cast<const InternalVehicle*>(internalObject);

            return internalVehicle->getWheelTransformations(wheelIndices, wheelIndexCount,
                addVehicleTransform,
                positions, orientations);
        }
        else
        {
            CARB_LOG_ERROR("getWheelLocalTransformations: provided object ID does not point to a vehicle\n");
        }
    }
    else
    {
        CARB_LOG_ERROR("getWheelLocalTransformations: invalid object ID provided\n");
    }

    return false;
}

SubscriptionId subscribeToObjectChangeNotifications(const IPhysicsObjectChangeCallback& callback)
{
    return getPhysXUsdPhysicsInterface().subscribeToObjectChangeNotifications(callback);
}

void unsubscribeToObjectChangeNotifications(SubscriptionId subscriptionId)
{
    getPhysXUsdPhysicsInterface().unsubscribeToObjectChangeNotifications(subscriptionId);
}

bool isAsyncSimRenderEnabled()
{
    return OmniPhysX::getInstance().getPhysXSetup().isAsyncSimEnabled();
}

void disableResetOnStop(bool disable)
{
    OmniPhysX::getInstance().getCachedSettings().disableResetOnStop = disable;
}

void fillInterface(omni::physx::IPhysx& iface)
{
    iface.getObjectId = getObjectId;
    iface.getPhysXPtr = getPhysXPtr;
    iface.getPhysXPtrFast = getPhysXPtrFast;
    iface.getPhysXObjectUsdPath = getPhysXObjectUsdPath;
    iface.forceLoadPhysicsFromUSD = forceLoadPhysicsFromUSD;
    iface.releasePhysicsObjects = releasePhysicsObjects;
    iface.createD6JointAtPath = createD6JointAtPath;
    iface.releaseD6Joint = releaseD6Joint;
    iface.subscribePhysicsOnStepEvents = subscribeToPhysicsOnStepEvents;
    iface.unsubscribePhysicsOnStepEvents = unsubscribeToPhysicsOnStepEvents;
    iface.setThreadCount = setThreadCount;
    iface.reconnectPVD = reconnectPVD;
    iface.overwriteGPUSetting = overwriteGPUSetting;
    iface.getOverwriteGPUSetting = getOverwriteGPUSetting;
    iface.overwriteSolverType = overwriteSolverType;
    iface.updateSimulation = physXUpdateNonRender;
    iface.updateTransformations = updateTransformations;
    iface.startSimulation = startSimulation;
    iface.resetSimulation = endSimulation;
    iface.getSimulationEventStreamV2 = getSimulationEventStreamV2;
	iface.setVoxelRange = setVoxelRange;
	iface.getWheelIndex = getWheelIndex;
    iface.getErrorEventStream = getErrorEventStream;
    iface.setSimulationLayer = setSimulationLayer;
    iface.subscribePhysicsSimulationEvents = subscribeToPhysicsSimulationEvents;
    iface.unsubscribePhysicsSimulationEvents = unsubscribeToPhysicsSimulationEvents;
    iface.getRigidBodyTransformation = getRigidBodyTransformation;
    iface.runBackwardsCompatibility = runBackwardsCompatibility;
    iface.checkBackwardsCompatibility = checkBackwardsCompatibility;
    iface.getBackwardsCompatibilityCheckLog = getBackwardsCompatibilityCheckLog;
    iface.getCollisionGroupFromCollider = getCollisionGroupFromCollider;
    iface.isRunning = isRunning;
    iface.resetSettingsInPreferences = resetSettingsInPreferences;
    iface.resetSettingsInStage = resetSettingsInStage;
    iface.isReadbackSuppressed = isReadbackSuppressed;
    iface.setVehicleToRestState = setVehicleToRestState;
    iface.getWheelTransformations = getWheelTransformations;
    iface.saveSceneToRepX = saveSceneToRepX;
    iface.subscribeObjectChangeNotifications = subscribeToObjectChangeNotifications;
    iface.unsubscribeObjectChangeNotifications = unsubscribeToObjectChangeNotifications;
    iface.isAsyncSimRenderEnabled = isAsyncSimRenderEnabled;
    iface.resetSetting = resetSetting;
    iface.resetSettings = resetSettings;
    iface.computeVehicleVelocity = computeVehicleVelocity;
    iface.isInteractiveActorRaycast = isInteractiveActorRaycast;
    iface.updateInteraction = updateInteraction;
    iface.updateSimulationScene = physXUpdateSceneNonRender;
    iface.updateTransformationsScene = updateTransformationsScene;
    iface.getVehicleDriveState = getVehicleDriveState;
    iface.getWheelState = getWheelState;
    iface.setWheelRotationSpeed = setWheelRotationSpeed;
    iface.setWheelRotationAngle = setWheelRotationAngle;
    iface.disableResetOnStop = disableResetOnStop;
}

void fillInterface(IPhysxVisualization& iface)
{
    iface.enableVisualization = enableVisualization;
    iface.enableNormalsVisualization = enableNormalsVisualization;
    iface.setVisualizationScale = setVisualizationScale;
    iface.setVisualizationCullingBox = setVisualizationCullingBox;
    iface.setVisualizationParameter = setVisualizationParameter;
    iface.getNbPoints = getNbPoints;
    iface.getPoints = getPoints;
    iface.getNbLines = getNbLines;
    iface.getLines = getLines;
    iface.getNbTriangles = getNbTriangles;
    iface.getTriangles = getTriangles;
    iface.getShapeDebugDraw = getShapeDebugDraw;
    iface.clearDebugVisualizationData = clearDebugVisualizationData;
    iface.getCollisionRepresentation = getCollisionRepresentation;
    iface.releaseCollisionRepresentation = releaseCollisionRepresentation;
    iface.getMeshKey = getMeshKey;
    iface.getDebugDrawCollShapeColor = getDebugDrawCollShapeColor;
}

void fillInterface(IPhysxVisualizationPrivate& iface)
{
    iface.getParticleClothDebugDraw = getParticleClothDebugDrawDeprecated;
    iface.getFEMClothDebugDraw = getFEMClothDebugDrawDeprecated;
}

void fillInterface(IPhysxUnitTests& iface)
{
    iface.update = physXUpdateNonRender;
    iface.getPhysicsStats = getPhysicsStats;
    iface.getMassInformation = getMassInformation;
    iface.startLoggerCheck = startLoggerCheck;
    iface.startLoggerCheckForMultiple = startLoggerCheckForMultiple;
    iface.endLoggerCheck = endLoggerCheck;
    iface.getPhysXPtrInstanced = getPhysXPtrInstanced;
    iface.getMaterialsPaths = getMaterialsPaths;
    iface.updateCooking = updateCooking;
    iface.isCudaLibPresent = isCudaLibPresent;
}

void fillInterface(IPhysxPropertyQuery& iface)
{
    iface.queryPrim = queryPrim;
}

void fillInterface(IPhysxBenchmarks& iface)
{
    iface.update = physXUpdateNonRender;
    iface.updateUsd = physXUpdateUsd;
    iface.createEmptyStage = createEmptyStage;
    iface.loadTargetStage = loadTargetStage;
    iface.loadTargetStage_Id = loadTargetStage_Id;
    iface.overwriteGPUSetting = bmOverwriteGPUSetting;
    iface.setThreadCount = bmSetThreadCount;
    iface.enablePVDProfile = bmEnablePVDProfile;
    iface.enableProfile = bmEnableProfile;
    iface.getProfileStats = bmGetProfileStats;
    iface.subscribeProfileStatsEvents = bmSubscribeProfileStatsEvents;
    iface.unsubscribeProfileStatsEvents = bmUnSubscribeProfileStatsEvents;
}

void fillInterface(IPhysxSimulation& iface)
{
    iface.attachStage = physxSimulationAttach;
    iface.detachStage = physxSimulationDetach;
    iface.setSimulationCallback = physxSetSimulationCallback;
    iface.setSimulationOutputFlags = physxSetSimulationFlags;
    iface.addSimulationOutputFlags = physxAddSimulationFlags;
    iface.removeSimulationOutputFlags = physxRemoveSimulationFlags;
    iface.simulate = physxSimulate;
    iface.fetchResults = physxFetchResults;
    iface.checkResults = physxCheckResults;
    iface.flushChanges = flushChanges;
    iface.pauseChangeTracking = pauseChangeTracking;
    iface.isChangeTrackingPaused = isChangeTrackingPaused;
    iface.subscribePhysicsContactReportEvents = physxSubscribePhysicsContactReportEvents;
    iface.unsubscribePhysicsContactReportEvents = physxUnsubscribePhysicsContactReportEvents;
    iface.subscribePhysicsFullContactReportEvents = physxSubscribePhysicsFullContactReportEvents;
    iface.unsubscribePhysicsFullContactReportEvents = physxUnsubscribePhysicsFullContactReportEvents;
    iface.getContactReport = physxGetContactReport;
    iface.getFullContactReport = physxFullGetContactReport;
    iface.getSimulationTimestamp = physxGetSimulationTimestamp;
    iface.getSimulationStepCount = physxGetSimulationStepCount;
    iface.simulateScene = physxSimulateScene;
    iface.fetchResultsScene = physxFetchResultsScene;
    iface.checkResultsScene = physxCheckResultsScene;
    iface.wakeUp = wakeUp;
    iface.putToSleep = putToSleep;
    iface.isSleeping = isSleeping;
    iface.addForceAtPos = addForceAtPos;
    iface.addTorque = addTorque;
    iface.subscribePhysicsTriggerReportEvents = physxSubscribePhysicsTriggerReportEvents;
    iface.unsubscribePhysicsTriggerReportEvents = physxUnsubscribePhysicsTriggerReportEvents;
    iface.addForceAtPosInstanced = addForceAtPosInstanced;
    iface.addTorqueInstanced = addTorqueInstanced;
    iface.wakeUpInstanced = wakeUpInstanced;
    iface.putToSleepInstanced = putToSleepInstanced;
    iface.isSleepingInstanced = isSleepingInstanced;
    iface.getAttachedStage = getPhysxSimulationAttachedStage;
}

void fillInterface(IPhysxUsdLoad& iface)
{
    iface.releaseDesc = releaseDesc;
    iface.parseCollision = parseCollision;
    iface.parseJoint = parseJoint;
    iface.parseDeformableBody = parseDeformableBodyDeprecated;
    iface.parseDeformableSurface = parseDeformableSurfaceDeprecated;
    iface.parseParticleCloth = parseParticleClothDeprecated;
    iface.parseSpatialTendons = parseSpatialTendons;
    iface.parseFixedTendons = parseFixedTendons;
    iface.createVehicleComponentTracker = createVehicleComponentTracker;
    iface.releaseVehicleComponentTracker = releaseVehicleComponentTracker;
    iface.parseVehicle = parseVehicle;
    iface.parseParticleSystem = parseParticleSystem;
    iface.parseArticulations = parseArticulations;
}

void fillInterface(IPhysxAttachmentPrivate& iface)
{
    iface.computeAttachmentPoints = computeAttachmentPointsDeprecated;
    iface.setupAutoDeformableAttachment = setupAutoDeformableAttachment;
    iface.updateAutoDeformableAttachment = updateAutoDeformableAttachment;

    iface.createSurfaceSampler = omni::sampling::createSurfaceSampler;
    iface.releaseSurfaceSampler = omni::sampling::releaseSurfaceSampler;
    iface.addSurfaceSamplerPoints = omni::sampling::addSurfaceSamplerPoints;
    iface.removeSurfaceSamplerPoints = omni::sampling::removeSurfaceSamplerPoints;
    iface.sampleSurface = omni::sampling::sampleSurface;
    iface.getSurfaceSamplerPoints = omni::sampling::getSurfaceSamplerPoints;

    iface.createTetFinder = omni::tetfinder::createTetFinder;
    iface.releaseTetFinder = omni::tetfinder::releaseTetFinder;
    iface.pointsToTetMeshLocal = omni::tetfinder::pointsToTetMeshLocal;
    iface.tetMeshLocalToPoints = omni::tetfinder::tetMeshLocalToPoints;

    iface.overlapTetMeshSphere = omni::tetfinder::overlapTetMeshSphere;
    iface.overlapTetMeshCapsule = omni::tetfinder::overlapTetMeshCapsule;

    iface.getClosestPoints = omni::tetfinder::getClosestPoints;

    iface.createPointFinder = omni::pointfinder::createPointFinder;
    iface.releasePointFinder = omni::pointfinder::releasePointFinder;
    iface.pointsToIndices = omni::pointfinder::pointsToIndices;

    iface.createTriMeshSampler = omni::sampling::createTriMeshSampler;
    iface.isPointInside = omni::sampling::isPointInside;
}

void fillInterface(IPhysxPrivate& iface)
{
    iface.getPhysXScene = privGetPhysXScene;
    iface.getRigidBodyInstancedData = primGetRigidBodyInstancedData;
    iface.getCudaContextManager = privGetCudaContextManager;
    iface.getInternalSurfaceDeformableBodyData = getInternalSurfaceDeformableBodyData;
    iface.getInternalVolumeDeformableBodyData = getInternalVolumeDeformableBodyData;
    iface.getPhysXPtrInstanced = getPhysXPtrInstanced;
}

void fillInterface(IPhysxParticlesPrivate& iface)
{
    iface.createPostprocess = omni::physx::particles::createPostprocess;
    iface.releasePostprocess = omni::physx::particles::releasePostprocess;
    iface.getPostprocessStages = omni::physx::particles::getPostprocessStages;
    iface.setPostprocessStages = omni::physx::particles::setPostprocessStages;
    iface.addPostprocessParticleSet = omni::physx::particles::addPostprocessParticleSet;
    iface.removePostprocessParticleSet = omni::physx::particles::removePostprocessParticleSet;
    iface.updatePostprocess = omni::physx::particles::updatePostprocess;
    iface.createParticleSampler = omni::physx::particles::createParticleSampler;
    iface.updateParticleSampler = omni::physx::particles::updateParticleSampler;
    iface.removeParticleSampler = omni::physx::particles::removeParticleSampler;
}
