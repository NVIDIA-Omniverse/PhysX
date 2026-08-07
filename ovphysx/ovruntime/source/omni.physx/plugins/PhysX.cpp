// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-3
 * @implements REQ-REPLICATE-001
 * @covers AC-1 AC-2
 */

#include "UsdPCH.h"
#include <common/utilities/SdfPathEncoding.h>

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
#include "attachment/PhysXAttachment.h"
#include "attachment/PhysXPoissonSampling.h"
#include "attachment/PhysXTetFinder.h"
#include "attachment/PhysXPointFinder.h"
#include "PhysXSimulationCallbacks.h"
#include "CookingDataAsync.h"
#include "ContactReport.h"
#include "Trigger.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/Scene.h"
#include "ObjectDataQuery.h"
#include "PhysXPrivate.h"
#include "ScopedNoticeLock.h"

#include <omni/physics/parse/IParseBackend.h>
#include <omni/physics/ovstage/OvstageParseBackend.h>
#include <omni/physics/usd/UsdParseBackend.h>
#include <carb/ClientUtils.h>
#include <carb/Framework.h>
#include <carb/crashreporter/CrashReporterUtils.h>
#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>
#include <common/utilities/OptionalCudaShim.h>
#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxVisualization.h>
#include <omni/physx/IPhysxSimulation.h>
#include <private/omni/physx/IPhysxAttachmentPrivate.h>
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
#include <omni/physx/IOptionalCuda.h>
#include <private/omni/physx/IPhysxStageUpdate.h>
#include <omni/physx/IPhysxStatistics.h>
#include <omni/physx/PhysXRuntime.h>
#include <omni/physics/usd/PrimIterator.h>
#include <pxr/usd/usd/primRange.h>
#include <omni/physics/tensors/TensorApi.h>
#include "tensors/GlobalsAreBad.h"
#include <omni/kit/KitUpdateOrder.h>
#include <omni/physx/Version.h>
#include "PhysXFoundation.h"

#include <PxPhysicsAPI.h>
#include <extensions/PxCollectionExt.h>

#include "ChangeRegister.h"
#include "VoxelMap.h"
#include "internal/Internal.h"
#include <common/utilities/MemoryMacros.h>
#include "PhysXDebugVisualization.h"
#include "PhysXUSDProperties.h"

#include <exception>
#include <limits>

using namespace PXR_NS;
using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace cookingdataasync;

PX_COMPILE_TIME_ASSERT(sizeof(PxVec3)==sizeof(GfVec3f));

// Keep the default channel symbol required by CARB_LOG_* call sites, but do
// not add a static registrar. OvruntimePhysX is linked as an internal runtime
// now, not loaded as a Carbonite plugin module.
OMNI_LOG_DEFINE_CHANNEL(OMNI_LOG_DEFAULT_CHANNEL, "omni.physx", "PhysX runtime")

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
    if (!getValue<float>(attachedStage, materialRec.mKey, propertyName, timeCode, data))
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
    const uint64_t stageId = UsdLoad::getUsdLoad()->getActiveStageId();
    if (stageId)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();

        // If physics objects are already loaded, release them first and notify listeners
        AttachedStage* existingStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
        if (existingStage && !existingStage->getObjectDatabase()->empty())
        {
            UsdLoad::getUsdLoad()->releasePhysicsObjects(stageId);            
        }

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
    const uint64_t stageId = UsdLoad::getUsdLoad()->getActiveStageId();
    UsdLoad::getUsdLoad()->flushChanges();
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (attachedStage)
    {
        attachedStage->getPhysXPhysicsInterface()->finalizeArticulations(*attachedStage);
        OmniPhysX::getInstance().getInternalPhysXDatabase().updateDirtyMassActors();
    }
    OmniPhysX::getInstance().getErrorEventStream()->pump();
}

void releasePhysicsObjects()
{
    const uint64_t stageId = UsdLoad::getUsdLoad()->getActiveStageId();
    UsdLoad::getUsdLoad()->releasePhysicsObjects(stageId);
}

ObjectId getObjectId(const SdfPath& path, PhysXType type)
{
    const uint64_t stageId = UsdLoad::getUsdLoad()->getActiveStageId();
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (attachedStage)
        return ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(path, type, OmniPhysX::getInstance().getInternalPhysXDatabase(), *attachedStage));
    else
        return kInvalidObjectId;
}

void* getPhysXPtr(const SdfPath& path, PhysXType type)
{
    const uint64_t stageId = UsdLoad::getUsdLoad()->getActiveStageId();
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (attachedStage)
        return (void*)(getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(path, type, OmniPhysX::getInstance().getInternalPhysXDatabase(), *attachedStage));
    else
        return nullptr;
}

uint32_t getPhysXPtrInstanced(const SdfPath& path, void** data, uint32_t dataSize, PhysXType type)
{
    const uint64_t stageId = UsdLoad::getUsdLoad()->getActiveStageId();
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
    const uint64_t stageId = UsdLoad::getUsdLoad()->getActiveStageId();
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (attachedStage)
        return (void*)(getObjectDataOrID<ObjectDataQueryType::eINTERNAL_PTR>(path, type, OmniPhysX::getInstance().getInternalPhysXDatabase(), *attachedStage));
    else
        return nullptr;
}

const void* createD6JointAtPath(const SdfPath& jointKey, const SdfPath& body0,const SdfPath& body1)
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

    // No attached stage means there are no records to register against, so skip the add.
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (j && attachedStage)
    {
        InternalJoint* intJoint = ICE_NEW(InternalJoint);
        intJoint->mJointType = eJointD6;
        ObjectId index = db.addRecord(ePTJoint, j, intJoint, attachedStage->keyFor(jointKey));
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
        const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        return attachedStage->pathFor(record.mKey);
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

    const PhysXScenesMap& physxScenes = omniPhysX.getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;

        if (scenePath.IsEmpty())
        {
            if (sc->getUpdateType() == eDisabled)
                continue;
        }
        else
        {
            if (sc->getSceneSdfPath() != scenePath)
                continue;
        }


        sc->getInternalScene()->updateSimulationOutputs(updateToUSD, updateVelocitiesToUsd, outputVelocitiesLocalSpace, updateParticlesToUsd);
    }
}

static void updateTransformations(bool /*useFaceCache*/, bool updateToUSD, bool updateVelocitiesToUsd, bool outputVelocitiesLocalSpace)
{
    updateTransformationsInternal(SdfPath() , updateToUSD, updateVelocitiesToUsd, outputVelocitiesLocalSpace);
}

static void updateTransformationsScene(uint64_t scenePath, bool updateToUSD, bool updateVelocitiesToUsd)
{
    updateTransformationsInternal(omni::physx::intToSdfPath(scenePath), updateToUSD, updateVelocitiesToUsd, false /* unused, only left for API compatibility reasons */);
}

static void startSimulation()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omniPhysX.setSimulationStarted(true);
    if(omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) != nullptr)
        getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(SdfPath(omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene)));
    else
        getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(SdfPath());

    {
        // Suppress initial-population notifications; the scope restores both
        // gates even if update() throws.
        InitialStagePopulationScope populationScope(getPhysXUsdPhysicsInterface());
        UsdLoad::getUsdLoad()->update(0.0f);
    }

    getPhysXUsdPhysicsInterface().enableObjectChangeNotifications(true);
    // now the initial load is done and notifications should be sent

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
        UsdStageRefPtr stage = UsdLoad::getUsdLoad()->getActiveStage();
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
    if (UsdLoad::getUsdLoad()->getActiveAttachedStage())
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

// Shared simulation-attach prologue for both the USD-stage and ovstage entry
// points: mark the session running, take over stage-update ticking, tear down any
// previous attach, and flag the attached-stage state. The only divergence between
// the two public entries is what gets attached afterwards (a USD stage id vs an
// ovstage payload).
static void beginSimulationAttach()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omniPhysX.setSimulationRunning(true);
    omniPhysX.getStageUpdate().detachStageUpdate();
    detachStage();
    omniPhysX.setSimulationAttachedStage(true);
}

bool physxSimulationAttachOvstage(const void* ovstageAttachPayload, uint64_t readOrdinal)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    bool sessionMutationStarted = false;
    const auto rollbackAttach = [&]() noexcept -> bool
    {
        UsdLoad* usdLoad = UsdLoad::getUsdLoad();
        try
        {
            if (usdLoad->getActiveAttachedStage())
                omniPhysX.physXDetach();
        }
        catch (...)
        {
            CARB_LOG_ERROR("Failed to clean up a partial ovstage attachment");
            if (AttachedStage* remainingStage = usdLoad->getActiveAttachedStage())
            {
                try
                {
                    usdLoad->detach(static_cast<uint64_t>(remainingStage->getStageId()));
                }
                catch (...)
                {
                    CARB_LOG_ERROR("Forced cleanup of a partial ovstage attachment failed");
                }
            }
        }

        if (usdLoad->getActiveAttachedStage())
        {
            CARB_LOG_ERROR("Ovstage attach rollback left an active AttachedStage; session flags remain attached");
            return false;
        }

        try
        {
            omniPhysX.getStageUpdate().attachStageUpdate();
        }
        catch (...)
        {
            CARB_LOG_ERROR("Failed to restore stage-update ownership after ovstage attach failure");
        }
        omniPhysX.setSimulationRunning(false);
        omniPhysX.setSimulationAttachedStage(false);
        return true;
    };

    try
    {
        // Resolve and classify the optional compatibility stage before changing
        // simulation-session state. The payload remains caller-owned and unmodified.
        uint64_t candidateStageId = 0;
        const ovstage_api_status_t queryStatus =
            omni::physics::ovstage::queryBackingUsdStageId(ovstageAttachPayload, candidateStageId);

        UsdStageRefPtr backingStage;
        uint64_t effectiveBackingStageId = 0;
        if (queryStatus == OVSTAGE_ERROR_NOT_SUPPORTED)
        {
            // A genuinely backing-less provider uses the supported ovstage-only path.
        }
        else if (queryStatus != OVSTAGE_OK)
        {
            CARB_LOG_ERROR("Could not query the ovstage backing USD stage id (status %u)",
                           static_cast<unsigned int>(queryStatus));
            return false;
        }
        else if (candidateStageId == 0)
        {
            CARB_LOG_ERROR("ovstage returned OVSTAGE_OK with an invalid zero backing USD stage id");
            return false;
        }
        else
        {
            if (candidateStageId > static_cast<uint64_t>(std::numeric_limits<long>::max()))
            {
                CARB_LOG_WARN("ovstage backing USD stage id %llu cannot be represented as a local UsdStageCache id; "
                              "attaching ovstage-only with effective stage id 0; tensor bindings unavailable",
                              static_cast<unsigned long long>(candidateStageId));
            }
            else
            {
                backingStage = UsdUtilsStageCache::Get().Find(
                    UsdStageCache::Id::FromLongInt(static_cast<long>(candidateStageId)));
                if (backingStage)
                {
                    effectiveBackingStageId = candidateStageId;
                }
                else
                {
                    CARB_LOG_WARN("ovstage backing USD stage id %llu is not resident in UsdUtilsStageCache; "
                                  "attaching ovstage-only with effective stage id 0; tensor bindings unavailable",
                                  static_cast<unsigned long long>(candidateStageId));
                }
            }
        }

        sessionMutationStarted = true;
        beginSimulationAttach();
        const bool attached = omniPhysX.physXAttachOvstage(
            ovstageAttachPayload, readOrdinal, backingStage, effectiveBackingStageId);
        if (!attached)
        {
            if (!rollbackAttach())
                CARB_LOG_ERROR("Ovstage attach failed and rollback could not release the caller payload");
            return false;
        }
        return true;
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("ovstage attach failed with an exception: %s", error.what());
    }
    catch (...)
    {
        CARB_LOG_ERROR("ovstage attach failed with an unknown exception");
    }

    if (sessionMutationStarted)
    {
        if (!rollbackAttach())
            CARB_LOG_ERROR("Ovstage attach exception rollback could not release the caller payload");
    }
    return false;
}

bool physxSimulationUpdateFromOvStage(uint64_t fromOrdinal, uint64_t toOrdinal)
{
    // Pull + apply the ovstage change delta for the explicit ordinal range
    // [fromOrdinal, toOrdinal] (ADR-0003 M3). The caller advanced the ordinals.
    return OmniPhysX::getInstance().physXUpdateFromOvStage(fromOrdinal, toOrdinal);
}

namespace
{
// clone() supplies the exact target path for each replicated copy; the replicator asks for the
// clone path per replication index through this rename callback. Keep owning SdfPath instances
// alive in physxSimulationCloneEnvironments while replicate() consumes their encoded path-node
// handles; an integer encoded from a temporary SdfPath becomes dangling as soon as it is destroyed.
uint64_t replicatorCloneRename(uint64_t /*replicatePath*/, uint32_t index, void* userData)
{
    const std::vector<PXR_NS::SdfPath>* targetPaths =
        static_cast<const std::vector<PXR_NS::SdfPath>*>(userData);
    return (targetPaths && index < targetPaths->size()) ? omni::physx::sdfPathToInt((*targetPaths)[index]) : 0;
}
} // namespace

bool physxSimulationCloneEnvironments(const char* sourcePath, const char* const* targetPaths,
                                      uint32_t numTargets, const float* transforms,
                                      const uint32_t* envIds, bool useEnvIds)
{
    // Drives the PhysX SDK replicator. sourcePath + targetPaths are plain strings, so no USD
    // type crosses the ABI. Each copy is named by targetPaths[i] (rename callback) and placed at
    // transforms[i] (a [numTargets*7] pose array, or null to co-locate on the source). envIds
    // optionally carries a logical environment id per target (see IPhysxSimulation.h). Works on
    // both the USD and ovstage attach.
    if (!sourcePath || !targetPaths || numTargets == 0)
        return false;
    const uint64_t stageId = UsdLoad::getUsdLoad()->getActiveStageId();
    if (!stageId)
        return false;

    // Validate the source and every target is a non-root absolute prim path. ovphysx_clone
    // rejects empty/NUL/duplicate/source-equal targets C-first but treats the strings as opaque,
    // so a relative, property, root, or malformed path still reaches here where SdfPath(str) would
    // silently encode a bad key. Reject them at this USD-aware seam before the replicator.
    auto isValidClonePath = [](const char* str, const char* role) -> bool
    {
        if (!str || !str[0])
        {
            CARB_LOG_ERROR("clone: %s is empty.", role);
            return false;
        }
        std::string reason;
        if (!SdfPath::IsValidPathString(str, &reason))
        {
            CARB_LOG_ERROR("clone: %s '%s' is not a valid path: %s", role, str, reason.c_str());
            return false;
        }
        const SdfPath path(str);
        if (!path.IsAbsolutePath() || !path.IsPrimPath())
        {
            CARB_LOG_ERROR("clone: %s '%s' must be a non-root absolute prim path (not relative, a "
                           "property path, or the root).", role, str);
            return false;
        }
        return true;
    };
    if (!isValidClonePath(sourcePath, "source path"))
        return false;
    for (uint32_t k = 0; k < numTargets; ++k)
    {
        if (!isValidClonePath(targetPaths[k], "target path"))
            return false;
    }

    // Mirror the C-level env-id bound here (a direct caller of the interface slot bypasses
    // ovphysx_clone's C-first check): runtime id is envIds[i] + 1 and PhysX supports at most
    // 1<<24 environments, so values >= 0x00FFFFFF silently lose isolation while reporting success.
    if (envIds)
    {
        for (uint32_t k = 0; k < numTargets; ++k)
        {
            if (envIds[k] >= 0x00FFFFFFu)
            {
                CARB_LOG_ERROR("clone: envIds[%u] = %u is out of range; env ids must be < 0x00FFFFFF "
                               "(PhysX supports at most 1<<24 environments; the runtime id is "
                               "envIds[i] + 1).",
                               k, envIds[k]);
                return false;
            }
        }
    }

    // Reject any target already populated with physics from the initial parse: cloning onto it
    // would add duplicate live actors and leave path/tensor lookups ambiguous. Walk each target
    // subtree and reject if any prim resolves to ObjectDb entries (mirrors the source
    // dataAlreadyParsed check in PhysXReplicator::replicate). New paths pass straight through.
    if (AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId))
    {
        const UsdStageWeakPtr stage = attachedStage->getStage();
        if (stage)
        {
            for (uint32_t k = 0; k < numTargets; ++k)
            {
                if (!targetPaths[k])
                    continue;
                const UsdPrim targetPrim = stage->GetPrimAtPath(SdfPath(targetPaths[k]));
                if (!targetPrim)
                    continue; // no USD prim at the target, nothing was parsed there
                const UsdPrimRange range(targetPrim, PXR_NS::UsdTraverseInstanceProxies());
                for (UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
                {
                    const UsdPrim& prim = *iter;
                    if (!prim)
                        continue;
                    const ObjectIdMap* entries = attachedStage->getObjectIds(prim.GetPrimPath());
                    if (entries && !entries->empty())
                    {
                        CARB_LOG_ERROR(
                            "clone: target '%s' is already populated with physics (at '%s'); cloning onto "
                            "a populated target would create duplicate actors. Use an empty target path.",
                            targetPaths[k], prim.GetPrimPath().GetText());
                        return false;
                    }
                }
            }
        }
    }

    // Reject targets that overlap runtime clone records. Successful clones author no USD prim, so
    // the walk above cannot see them: a reused or nested target would stack a second live actor on
    // one path (ObjectIdMap is a multimap, lookups turn ambiguous). Equal, ancestor, and descendant
    // overlaps (against earlier calls and within this call) are rejected; disjoint targets pass.
    if (AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId))
    {
        std::vector<SdfPath> batchTargets;
        batchTargets.reserve(numTargets);
        for (uint32_t k = 0; k < numTargets; ++k)
        {
            const SdfPath target(targetPaths[k]);
            for (const SdfPath& recorded : attachedStage->getRuntimeCloneTargets())
            {
                if (target.HasPrefix(recorded) || recorded.HasPrefix(target))
                {
                    CARB_LOG_ERROR(
                        "clone: target '%s' overlaps the earlier runtime clone target '%s' on this "
                        "attach (same path or ancestor/descendant); cloning there would create "
                        "duplicate live actors. Use a disjoint target path.",
                        targetPaths[k], recorded.GetText());
                    return false;
                }
            }
            for (const SdfPath& sibling : batchTargets)
            {
                if (target.HasPrefix(sibling) || sibling.HasPrefix(target))
                {
                    CARB_LOG_ERROR(
                        "clone: targets '%s' and '%s' overlap within one clone() call (same path or "
                        "ancestor/descendant); each copy needs a disjoint target path.",
                        targetPaths[k], sibling.GetText());
                    return false;
                }
            }
            batchTargets.push_back(target);
        }
    }

    // Own the per-clone paths until the synchronous replicate() and callback have completed.
    std::vector<PXR_NS::SdfPath> cloneTargetPaths;
    cloneTargetPaths.reserve(numTargets);
    for (uint32_t k = 0; k < numTargets; ++k)
        cloneTargetPaths.emplace_back(targetPaths[k]);

    // Register after attach (the source is already parsed). Under the attach-time creation-id mode
    // (kSettingReplicatorEnvIdsOnAttach, set by ovphysx before attaching), the source's bodies hold
    // env id 0, so with useEnvIds the source (env 0) and clones (1..N) are collision-safe even when
    // co-located. useEnvIds stays an optional GPU-broadphase filtering pass-through.
    omni::physx::IPhysxReplicator& replicator = omni::physx::runtime::getPhysxReplicatorInterface();
    omni::physx::IReplicatorCallback cb{};
    cb.hierarchyRenameFn = &replicatorCloneRename;
    cb.userData = &cloneTargetPaths;
    replicator.registerReplicator(stageId, cb);

    // The registration is live only for the synchronous replicate() below. Always drop it on the
    // way out, including when replicate() throws: a lingering entry leaks a PhysXReplicator and
    // hijacks any later re-attach of this stage id (getReplicator() would steer attach down the
    // replicator path instead of a normal parse, breaking reset_stage() reload). unregisterReplicator
    // also clears the replicator-stage flag. The scope guard makes this exception-safe.
    struct ReplicatorUnregisterGuard
    {
        omni::physx::IPhysxReplicator& replicator;
        uint64_t stageId;
        ~ReplicatorUnregisterGuard() { replicator.unregisterReplicator(stageId); }
    } unregisterGuard{ replicator, stageId };

    // Explicit per-clone world poses + optional caller-supplied logical env ids, applied to each
    // copy during the replicate below. Both are set fresh per call (null clears).
    if (PhysXReplicator* concrete = OmniPhysX::getInstance().getReplicator(stageId))
    {
        concrete->setCloneTransforms(transforms, numTargets);
        concrete->setCloneEnvIds(envIds, numTargets);
    }

    const bool cloned = replicator.replicate(
        stageId, omni::physx::sdfPathToInt(PXR_NS::SdfPath(sourcePath)), numTargets, useEnvIds);

    // Record the now-live runtime targets so a later clone() on this attach rejects any
    // overlapping target (see the runtime-overlap check above). Failed replications create no
    // clone records, so their targets stay reusable.
    if (cloned)
    {
        if (AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId))
        {
            for (uint32_t k = 0; k < numTargets; ++k)
                attachedStage->addRuntimeCloneTarget(SdfPath(targetPaths[k]));
        }
    }
    return cloned;
}

bool physxSimulationAttach(long stageId)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if (stageId && omniPhysX.isSimulationAttachedStage() && stageId == UsdLoad::getUsdLoad()->getActiveStageId())
    {
        CARB_LOG_ERROR("Stage %d already attached.", stageId);
        return false;
    }

    beginSimulationAttach();
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
        return UsdLoad::getUsdLoad()->getActiveStageId();
    }
    return 0l;
}

void addCrashreporterMetadata()
{
    carb::crashreporter::addCrashMetadata("lib_physx_buildVersion", PHYSICS_BUILD_VERSION);
    carb::crashreporter::addCrashMetadata("lib_physx_buildRepo", "omniverse/physics");
    carb::crashreporter::addCrashMetadata("lib_physx_buildHash", PHYSICS_BUILD_SHA);
    carb::crashreporter::addCrashMetadata("lib_physx_buildBranch", PHYSICS_BUILD_BRANCH);
    carb::crashreporter::addCrashMetadata("lib_physx_buildDate", PHYSICS_BUILD_DATE);
}

void fillInterface(IPhysxSimulation& iface);
void fillInterface(omni::physx::IPhysx& iface);
void fillInterface(omni::physx::IPhysxPrivate& iface);
void fillInterface(omni::physx::IPhysxJoint& iface);
void fillInterface(omni::physx::IPhysxReplicator& iface);
void fillInterface(omni::physx::IPhysxCustomJoint& iface);
void fillInterface(omni::physx::IPhysxCustomGeometry& iface);
void fillInterface(omni::physx::IPhysxVisualization& iface);
void fillInterface(omni::physx::IPhysxSceneQuery& iface);
void fillInterface(omni::physx::IPhysxPropertyQuery& iface);
void fillInterface(omni::physx::IPhysxAttachmentPrivate& iface);
void fillInterface(omni::physx::IPhysxStageUpdate& iface);
void fillInterface(omni::physx::IPhysxStatistics& iface);
void fillInterface(omni::physx::IPhysxUnitTests& iface);
void fillInterface(omni::physx::IPhysxBenchmarks& iface);
void fillInterface(omni::physx::IPhysxCooking& iface);
void fillInterface(omni::physics::tensors::TensorApi& iface);

namespace omni
{
namespace physx
{
struct PhysXRuntimeInterfaces
{
    IPhysx physx{};
    IPhysxPrivate physxPrivate{};
    IPhysxJoint physxJoint{};
    IPhysxReplicator physxReplicator{};
    IPhysxCustomJoint physxCustomJoint{};
    IPhysxCustomGeometry physxCustomGeometry{};
    IPhysxVisualization physxVisualization{};
    IPhysxSceneQuery physxSceneQuery{};
    IPhysxPropertyQuery physxPropertyQuery{};
    IPhysxAttachmentPrivate physxAttachmentPrivate{};
    IPhysxStageUpdate physxStageUpdate{};
    IPhysxStatistics physxStatistics{};
    IPhysxUnitTests physxUnitTests{};
    IPhysxBenchmarks physxBenchmarks{};
    IPhysxCooking physxCooking{};
    IPhysxFoundation physxFoundation{};
    IOptionalCuda optionalCuda{};
    IPhysxSimulation physxSimulation{};
    omni::physics::tensors::TensorApi tensorApi{};
};

PhysXRuntimeInterfaces& OmniPhysX::getRuntimeInterfaces()
{
    if (!mRuntimeInterfaces)
    {
        mRuntimeInterfaces = new PhysXRuntimeInterfaces();
    }
    return *mRuntimeInterfaces;
}

const PhysXRuntimeInterfaces& OmniPhysX::getRuntimeInterfaces() const
{
    CARB_ASSERT(mRuntimeInterfaces);
    return *mRuntimeInterfaces;
}

void OmniPhysX::releaseRuntimeInterfaces()
{
    delete mRuntimeInterfaces;
    mRuntimeInterfaces = nullptr;
}

namespace runtime
{
namespace
{
OmniPhysX* tryGetRuntimeOwner()
{
    OmniPhysX* omniPhysX = OmniPhysX::getInstanceCheck();
    if (!omniPhysX || !omniPhysX->isPhysxRuntimeStarted())
    {
        return nullptr;
    }
    return omniPhysX;
}

OmniPhysX& getRuntimeOwner()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    CARB_ASSERT(omniPhysX);
    return *omniPhysX;
}

IOptionalCuda* getOptionalCudaForShim()
{
    // The OptionalCuda table is populated before this provider is installed.
    // GPU setup can ask the shim during OmniPhysX::onStartup(), before the
    // wider direct-runtime accessors are marked fully started.
    OmniPhysX* omniPhysX = OmniPhysX::getInstanceCheck();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().optionalCuda : nullptr;
}
} // namespace

OMNI_PHYSX_RUNTIME_API void startup()
{
    OmniPhysX::createOmniPhysXInstance();
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if (omniPhysX.isPhysxRuntimeStarted())
    {
        // startup() is intentionally idempotent. Owners may call it again after
        // Carbonite dependencies are loaded so tensors can finish their own
        // Carbonite-dependent setup.
        omni::physx::tensors::tensorsPluginStartup();
        return;
    }

    PhysXRuntimeInterfaces& interfaces = omniPhysX.getRuntimeInterfaces();
    fillInterface(interfaces.physx);
    fillInterface(interfaces.physxPrivate);
    fillInterface(interfaces.physxJoint);
    fillInterface(interfaces.physxReplicator);
    fillInterface(interfaces.physxCustomJoint);
    fillInterface(interfaces.physxCustomGeometry);
    fillInterface(interfaces.physxVisualization);
    fillInterface(interfaces.physxSceneQuery);
    fillInterface(interfaces.physxPropertyQuery);
    fillInterface(interfaces.physxAttachmentPrivate);
    fillInterface(interfaces.physxStageUpdate);
    fillInterface(interfaces.physxStatistics);
    fillInterface(interfaces.physxUnitTests);
    fillInterface(interfaces.physxBenchmarks);
    fillInterface(interfaces.physxCooking);
    fillInterface(interfaces.physxSimulation);
    omni::physx::foundation::initializeRuntime();
    // Install the default parse backend (USD) for the process (ADR-0005).
    // Exactly one backend is active at a time; tests may override this via
    // omni::physics::parse::setParseBackend() before attaching a stage.
    omni::physics::parse::setParseBackend(omni::physics::usd::makeUsdParseBackend());
    interfaces.physxFoundation = omni::physx::foundation::getInterface();
    interfaces.optionalCuda = omni::physx::foundation::getOptionalCudaInterface();
    fillInterface(interfaces.tensorApi);
    omni::physx::optionalCuda::setProvider(getOptionalCudaForShim);
    addCrashreporterMetadata();
    omniPhysX.onStartup();

    // PhysX tensors initialize through the direct runtime accessors.
    omniPhysX.setPhysxRuntimeStarted(true);
    omni::physx::tensors::tensorsPluginStartup();
}

OMNI_PHYSX_RUNTIME_API void shutdown()
{
    OmniPhysX* omniPhysXPtr = tryGetRuntimeOwner();
    if (!omniPhysXPtr)
    {
        return;
    }

    omni::physx::tensors::tensorsShutdown();
    OmniPhysX& omniPhysX = *omniPhysXPtr;
    omniPhysX.onShutdown();
    omni::physx::foundation::shutdownRuntime();
    omni::physx::optionalCuda::setProvider(nullptr);
    omniPhysX.setPhysxRuntimeStarted(false);
    omniPhysX.releaseRuntimeInterfaces();
}

OMNI_PHYSX_RUNTIME_API IPhysx& getPhysxInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physx;
}

OMNI_PHYSX_RUNTIME_API IPhysx* tryGetPhysxInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physx : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxPrivate& getPhysxPrivateInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxPrivate;
}

OMNI_PHYSX_RUNTIME_API IPhysxPrivate* tryGetPhysxPrivateInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxPrivate : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxJoint& getPhysxJointInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxJoint;
}

OMNI_PHYSX_RUNTIME_API IPhysxJoint* tryGetPhysxJointInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxJoint : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxReplicator& getPhysxReplicatorInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxReplicator;
}

OMNI_PHYSX_RUNTIME_API IPhysxReplicator* tryGetPhysxReplicatorInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxReplicator : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxCustomJoint& getPhysxCustomJointInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxCustomJoint;
}

OMNI_PHYSX_RUNTIME_API IPhysxCustomJoint* tryGetPhysxCustomJointInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxCustomJoint : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxCustomGeometry& getPhysxCustomGeometryInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxCustomGeometry;
}

OMNI_PHYSX_RUNTIME_API IPhysxCustomGeometry* tryGetPhysxCustomGeometryInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxCustomGeometry : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxVisualization& getPhysxVisualizationInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxVisualization;
}

OMNI_PHYSX_RUNTIME_API IPhysxVisualization* tryGetPhysxVisualizationInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxVisualization : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxSceneQuery& getPhysxSceneQueryInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxSceneQuery;
}

OMNI_PHYSX_RUNTIME_API IPhysxSceneQuery* tryGetPhysxSceneQueryInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxSceneQuery : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxPropertyQuery& getPhysxPropertyQueryInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxPropertyQuery;
}

OMNI_PHYSX_RUNTIME_API IPhysxPropertyQuery* tryGetPhysxPropertyQueryInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxPropertyQuery : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxAttachmentPrivate& getPhysxAttachmentPrivateInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxAttachmentPrivate;
}

OMNI_PHYSX_RUNTIME_API IPhysxAttachmentPrivate* tryGetPhysxAttachmentPrivateInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxAttachmentPrivate : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxStageUpdate& getPhysxStageUpdateInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxStageUpdate;
}

OMNI_PHYSX_RUNTIME_API IPhysxStageUpdate* tryGetPhysxStageUpdateInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxStageUpdate : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxStatistics& getPhysxStatisticsInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxStatistics;
}

OMNI_PHYSX_RUNTIME_API IPhysxStatistics* tryGetPhysxStatisticsInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxStatistics : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxUnitTests& getPhysxUnitTestsInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxUnitTests;
}

OMNI_PHYSX_RUNTIME_API IPhysxUnitTests* tryGetPhysxUnitTestsInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxUnitTests : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxBenchmarks& getPhysxBenchmarksInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxBenchmarks;
}

OMNI_PHYSX_RUNTIME_API IPhysxBenchmarks* tryGetPhysxBenchmarksInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxBenchmarks : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxCooking& getPhysxCookingInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxCooking;
}

OMNI_PHYSX_RUNTIME_API IPhysxCooking* tryGetPhysxCookingInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxCooking : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxCookingService& getPhysxCookingServiceInterface()
{
    OmniPhysX& omniPhysX = getRuntimeOwner();
    IPhysxCookingService* iface = omniPhysX.getPhysXSetup().getCookingServiceInterface();
    CARB_ASSERT(iface);
    return *iface;
}

OMNI_PHYSX_RUNTIME_API IPhysxCookingService* tryGetPhysxCookingServiceInterface()
{
    if (!tryGetRuntimeOwner())
    {
        return nullptr;
    }
    return OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
}

OMNI_PHYSX_RUNTIME_API IPhysxCookingServicePrivate& getPhysxCookingServicePrivateInterface()
{
    OmniPhysX& omniPhysX = getRuntimeOwner();
    IPhysxCookingServicePrivate* iface = omniPhysX.getPhysXSetup().getCookingServicePrivateInterface();
    CARB_ASSERT(iface);
    return *iface;
}

OMNI_PHYSX_RUNTIME_API IPhysxCookingServicePrivate* tryGetPhysxCookingServicePrivateInterface()
{
    if (!tryGetRuntimeOwner())
    {
        return nullptr;
    }
    return OmniPhysX::getInstance().getPhysXSetup().getCookingServicePrivateInterface();
}

OMNI_PHYSX_RUNTIME_API IPhysxFoundation& getPhysxFoundationInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxFoundation;
}

OMNI_PHYSX_RUNTIME_API IPhysxFoundation* tryGetPhysxFoundationInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxFoundation : nullptr;
}

OMNI_PHYSX_RUNTIME_API IOptionalCuda& getOptionalCudaInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().optionalCuda;
}

OMNI_PHYSX_RUNTIME_API IOptionalCuda* tryGetOptionalCudaInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().optionalCuda : nullptr;
}

OMNI_PHYSX_RUNTIME_API IPhysxSimulation& getPhysxSimulationInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().physxSimulation;
}

OMNI_PHYSX_RUNTIME_API IPhysxSimulation* tryGetPhysxSimulationInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().physxSimulation : nullptr;
}

OMNI_PHYSX_RUNTIME_API omni::physics::tensors::TensorApi& getTensorApiInterface()
{
    return getRuntimeOwner().getRuntimeInterfaces().tensorApi;
}

OMNI_PHYSX_RUNTIME_API omni::physics::tensors::TensorApi* tryGetTensorApiInterface()
{
    OmniPhysX* omniPhysX = tryGetRuntimeOwner();
    return omniPhysX ? &omniPhysX->getRuntimeInterfaces().tensorApi : nullptr;
}

} // namespace runtime
} // namespace physx
} // namespace omni

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
            SdfPathVector prototypes;
            getRelationshipValue(*attachedStage, rec.mKey, UsdGeomTokens->prototypes, prototypes);
            if(prototypes.size() == 0)
            {
                CARB_LOG_ERROR("SimulationInterface function applied to PointInstancer without prototypes.");
                return false;
            }
            if(protoIndex != 0xffffffff)
            {
                // If protoIndex is not 0xffffffff, we only need to search the prototype associated with this instance.
                VtArray<int> protoIndices;
                if(!getArrayValue(*attachedStage, rec.mKey, UsdGeomTokens->protoIndices, UsdTimeCode::Default(), protoIndices))
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

// Currently unused — kept for potential future use with point instancers
#if 0
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
#endif

static bool getRigidBodyTransformation(const SdfPath& path, carb::Float3& pos, carb::Float4& rot)
{
    getRigidBodyTransformData transformData = {pos, rot};
    const applySimFn function = getRigidBodyTransformationInternal;
    if(!applySimulationInterfaceFunction(UsdLoad::getUsdLoad()->getActiveStageId(), omni::physx::sdfPathToInt(SdfPath(path)), &function, &transformData))
    {
        CARB_LOG_ERROR("Error executing getRigidBodyTransformation.");
        return false;
    }
    return true;
}

static void wakeUpInternal(const InternalDatabase::Record* target, void* /*data*/)
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

static void putToSleepInternal(const InternalDatabase::Record* target, void* /*data*/)
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

static int getWheelIndex(const SdfPath& wheelKey)
{
    InternalVehicleWheelAttachment* wheelAttachment = static_cast<InternalVehicleWheelAttachment*>(getInternalPtr(wheelKey, ePTVehicleWheelAttachment));
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
                const ::physx::PxVehicleFrame& frame = vehicleContext.getFrame();
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
        // Resolve the stored ObjectKeys to SdfPaths at the ABI boundary.
        const usdparser::AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        data.bodyPath = attachedStage ? attachedStage->pathFor(internalPtr->mBodyKey) : SdfPath();
        data.simMeshPath = attachedStage ? attachedStage->pathFor(internalPtr->mSimMeshKey) : SdfPath();
        data.worldToSimMesh = internalPtr->mWorldToSimMesh;

        data.skinMeshPaths.clear();
        data.skinMeshPaths.reserve(internalPtr->mSkinMeshKeys.size());
        for (const omni::physics::parse::ObjectKey skinKey : internalPtr->mSkinMeshKeys)
            data.skinMeshPaths.push_back(attachedStage ? attachedStage->pathFor(skinKey) : SdfPath());
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
        // Resolve the stored ObjectKeys to SdfPaths at the ABI boundary.
        const usdparser::AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        data.bodyPath = attachedStage ? attachedStage->pathFor(internalPtr->mBodyKey) : SdfPath();
        data.simMeshPath = attachedStage ? attachedStage->pathFor(internalPtr->mSimMeshKey) : SdfPath();
        data.worldToSimMesh = internalPtr->mWorldToSimMesh;

        data.skinMeshPaths.clear();
        data.skinMeshPaths.reserve(internalPtr->mSkinMeshKeys.size());
        for (const omni::physics::parse::ObjectKey skinKey : internalPtr->mSkinMeshKeys)
            data.skinMeshPaths.push_back(attachedStage ? attachedStage->pathFor(skinKey) : SdfPath());
        data.worldToSkinMeshTransforms = omni::span<GfMatrix4f>(internalPtr->mWorldToSkinMeshTransforms.begin(), internalPtr->mWorldToSkinMeshTransforms.end());
        data.skinMeshRanges = omni::span<carb::Uint2>(internalPtr->mSkinMeshRanges.begin(), internalPtr->mSkinMeshRanges.end());

        data.numSkinMeshVertices = internalPtr->mNumSkinMeshVertices;
        data.numSimMeshVertices = internalPtr->mNumSimMeshVertices;

        data.collMeshPath = attachedStage ? attachedStage->pathFor(internalPtr->mCollMeshKey) : SdfPath();
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

void runBackwardsCompatibility(long int /*stageId*/)
{
    CARB_LOG_ERROR("runBackwardsCompatibility: deprecated\n");
}

bool checkBackwardsCompatibility(long int /*stageId*/)
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
                const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
                return attachedStage->pathFor(colGroupRec.mKey);
            }
        }
    }
    return SdfPath();
}

bool isReadbackSuppressed()
{
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

void fillInterface(omni::physx::IPhysxFoundation& iface)
{
    // Publish a copy of the static library's canonical function table. The
    // foundation library itself has no Carbonite fillInterface entry point.
    iface = omni::physx::foundation::getInterface();
}

void fillInterface(omni::physx::IOptionalCuda& iface)
{
    // Publish a copy of the static library's canonical function table. The
    // foundation library itself has no Carbonite fillInterface entry point.
    iface = omni::physx::foundation::getOptionalCudaInterface();
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
    iface.setVisualizationParameterValue = setVisualizationParameterValue;
    iface.setVisualizationScopeTokens = setVisualizationScopeTokens;
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

void fillInterface(IPhysxUnitTests& iface)
{
    iface.update = physXUpdateNonRender;
    iface.getPhysicsStats = getPhysicsStats;
    iface.getBatchedContactBufferStats = getBatchedContactBufferStats;
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
    iface.attachOvstage = physxSimulationAttachOvstage;
    iface.updateFromOvStage = physxSimulationUpdateFromOvStage;
    iface.cloneEnvironments = physxSimulationCloneEnvironments;
    iface.detachStage = physxSimulationDetach;
    iface.setSimulationCallback = physxSetSimulationCallback;
    iface.setSimulationOutputFlags = physxSetSimulationFlags;
    iface.addSimulationOutputFlags = physxAddSimulationFlags;
    iface.removeSimulationOutputFlags = physxRemoveSimulationFlags;
    iface.simulate = physxSimulate;
    iface.fetchResults = physxFetchResults;
    iface.checkResults = physxCheckResults;
    iface.flushChanges = flushChanges;
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

// Adapts the 2-param free function to the 1-param CARB_ABI interface.
static bool updateAutoDeformableAttachmentABI(const SdfPath& path)
{
    bool attachmentDataRecomputed = false;
    return updateAutoDeformableAttachment(path, attachmentDataRecomputed);
}

void fillInterface(IPhysxAttachmentPrivate& iface)
{
    iface.setupAutoDeformableAttachment = setupAutoDeformableAttachment;
    iface.updateAutoDeformableAttachment = updateAutoDeformableAttachmentABI;

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
