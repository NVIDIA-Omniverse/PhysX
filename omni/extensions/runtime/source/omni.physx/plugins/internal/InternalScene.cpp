// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/profiler/Profile.h>

#include "InternalScene.h"
#include "InternalParticle.h"
#include "InternalDeformableDeprecated.h"
#include "InternalAttachmentDeprecated.h"
#include "InternalMimicJoint.h"

#include "particles/PhysXParticlePost.h"
#include <deformables/PhysXDeformablePost.h>

#include <PhysXUpdate.h>
#include <PhysXTools.h>
#include <PhysXSimulationCallbacks.h>
#include <CookingDataAsync.h>
#include <usdLoad/FabricSync.h>
#include <usdLoad/LoadUsd.h>
#include <Raycast.h>
#include <PhysXTools.h>
#include <ScopedNoticeLock.h>

#include <common/utilities/MemoryMacros.h>

#if USE_PHYSX_GPU
#include <extensions/PxParticleExt.h>
#endif

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace pxr;
using namespace carb;
using namespace ::physx;

OMNI_LOG_DECLARE_CHANNEL(kRoboticsLogChannel)

static const TfToken gTokTranslate = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeTranslate);
static const TfToken gTokTransform = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeTransform);
static const TfToken gTokOrient = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeOrient);
static const TfToken gTokScale = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeScale);
static const TfToken gTokRotateZYX = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeRotateZYX);
static const TfToken gTokRotateXYZ = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeRotateXYZ);
static const TfToken gTokRotateX = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeRotateX);
static const TfToken gTokRotateY = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeRotateY);
static const TfToken gTokRotateZ = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeRotateZ);

void InternalScene::setVehicleContext(const VehicleContextDesc& contextDesc)
{
    mVehicleContext.init(contextDesc, *mScene);
}

omni::physx::usdparser::ObjectId InternalScene::addVehicle(InternalVehicle& vehicle,
    const uint32_t wheelCount, const UsdPrim& usdPrim, const bool enabled)
{
    const uint32_t oldVehicleCount = static_cast<uint32_t>(mVehicles.size());
    CARB_ASSERT(mVehicles.size() == oldVehicleCount);

    if (enabled)
    {
        if (oldVehicleCount == mEnabledVehicleCount)
        {
            mVehicles.push_back(&vehicle);
            vehicle.mBufferIndex = oldVehicleCount;
        }
        else
        {
            CARB_ASSERT(oldVehicleCount > mEnabledVehicleCount);
            moveVehicleToBack(mEnabledVehicleCount);
            setVehicleAtPosition(mEnabledVehicleCount, vehicle);
        }

        mEnabledVehicleCount++;
    }
    else
    {
        mVehicles.push_back(&vehicle);
        vehicle.mBufferIndex = oldVehicleCount;
    }

    mVehicleActorToVehicle.insert({vehicle.getRigidDynamicActor(), &vehicle});

    ObjectId vehicleObjectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(ePTVehicle, nullptr, &vehicle, usdPrim.GetPrimPath());

    return vehicleObjectId;
}

void InternalScene::removeVehicle(InternalVehicle& vehicle)
{
    PhysXActorVehicleBase* pxVehicle = vehicle.mPhysXVehicle;
    CARB_ASSERT(mVehicleActorToVehicle.find(pxVehicle->getRigidDynamicActorNoCheck()) != mVehicleActorToVehicle.end());
    mVehicleActorToVehicle.erase(pxVehicle->getRigidDynamicActorNoCheck());

    // Remove the vehicle from the list of vehicles.

    CARB_ASSERT(vehicle.mBufferIndex < mVehicles.size());
    CARB_ASSERT(&vehicle == mVehicles[vehicle.mBufferIndex]);

    uint32_t targetIndex = vehicle.mBufferIndex;

    CARB_ASSERT(mVehicles[targetIndex]->mPhysXVehicle == pxVehicle);

    const uint32_t oldVehicleCount = static_cast<uint32_t>(mVehicles.size());
    if (targetIndex < mEnabledVehicleCount)
    {
        const uint32_t sourceIndex = mEnabledVehicleCount - 1;

        if (sourceIndex != targetIndex)
        {
            moveVehicleToPosition(sourceIndex, targetIndex);
            targetIndex = sourceIndex;
        }

        CARB_ASSERT(mEnabledVehicleCount > 0);
        mEnabledVehicleCount--;
    }

    CARB_ASSERT(oldVehicleCount > 0);
    const uint32_t backIndex = oldVehicleCount - 1;
    if (backIndex != targetIndex)
    {
        moveVehicleToPosition(backIndex, targetIndex);
    }

    mVehicles.pop_back();
}

void InternalScene::setVehicleEnabledState(InternalVehicle& vehicle, const bool enabled)
{
    const bool enabledNow = isVehicleEnabled(vehicle);
    if (enabled != enabledNow)
    {
        PhysXActorVehicleBase* pxVehicle = vehicle.mPhysXVehicle;

        if (enabled)
        {
            CARB_ASSERT(mVehicles.size() > mEnabledVehicleCount);
            CARB_ASSERT(!enabledNow);
            CARB_ASSERT(vehicle.mBufferIndex >= mEnabledVehicleCount);

            const uint32_t sourceIndex = mEnabledVehicleCount;
            uint32_t targetIndex = vehicle.mBufferIndex;

            if (sourceIndex != targetIndex)
            {
                moveVehicleToPosition(sourceIndex, targetIndex);
                targetIndex = sourceIndex;

                setVehicleAtPosition(targetIndex, vehicle);
            }

            mEnabledVehicleCount++;
        }
        else
        {
            CARB_ASSERT(mEnabledVehicleCount > 0);
            CARB_ASSERT(enabledNow);
            CARB_ASSERT(vehicle.mBufferIndex < mEnabledVehicleCount);

            const uint32_t sourceIndex = mEnabledVehicleCount - 1;
            uint32_t targetIndex = vehicle.mBufferIndex;
            
            if (sourceIndex != targetIndex)
            {
                moveVehicleToPosition(sourceIndex, targetIndex);
                targetIndex = sourceIndex;

                setVehicleAtPosition(targetIndex, vehicle);
            }

            CARB_ASSERT(mEnabledVehicleCount > 0);
            mEnabledVehicleCount--;

            pxVehicle->setToRestState();
        }
    }
}

void InternalScene::setVehicleAtPosition(const uint32_t index, InternalVehicle& vehicle)
{
    mVehicles[index] = &vehicle;
    vehicle.mBufferIndex = index;
}

void InternalScene::moveVehicleToBack(const uint32_t sourceIndex)
{
    const uint32_t oldVehicleCount = static_cast<uint32_t>(mVehicles.size());
    CARB_ASSERT(mVehicles.size() == oldVehicleCount);
    CARB_ASSERT(sourceIndex < oldVehicleCount);
    CARB_ASSERT(mVehicles[sourceIndex]->mBufferIndex == sourceIndex);

    InternalVehicle* vehicleToMove = mVehicles[sourceIndex];
    mVehicles.push_back(vehicleToMove);
    vehicleToMove->mBufferIndex = oldVehicleCount;
}

void InternalScene::moveVehicleToPosition(const uint32_t sourceIndex, const uint32_t targetIndex)
{
    CARB_ASSERT(sourceIndex < mVehicles.size());
    CARB_ASSERT(targetIndex < mVehicles.size());
    CARB_ASSERT(mVehicles[sourceIndex]->mBufferIndex == sourceIndex);

    InternalVehicle* vehicleToMove = mVehicles[sourceIndex];
    mVehicles[targetIndex] = vehicleToMove;
    vehicleToMove->mBufferIndex = targetIndex;
}

void InternalScene::addAttachmentDeprecated(InternalAttachmentDeprecated& internalAttachment)
{
    mAttachmentsDeprecated.push_back(&internalAttachment);
}

bool InternalScene::removeAttachmentDeprecated(InternalAttachmentDeprecated& internalAttachment)
{
    InternalAttachmentDeprecated* attachment = &internalAttachment;

    auto it = std::find(mAttachmentsDeprecated.begin(), mAttachmentsDeprecated.end(), attachment);
    if (it != mAttachmentsDeprecated.end())
    {
        std::iter_swap(it, --mAttachmentsDeprecated.end());
        mAttachmentsDeprecated.pop_back();

        SAFE_DELETE_SINGLE(attachment);

        return true;
    }

    return false;
}

void InternalScene::removeAttachmentsDeprecated(ObjectId objId)
{
    // Make sure that objId is a valid id
    if (objId == kInvalidObjectId)
        return;

    omni::physx::OmniPhysX& omniPhysX = omni::physx::OmniPhysX::getInstance();
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(omniPhysX.getStageId());

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objId);
    SdfPath path;
    if (objectRecord)
    {
        path = objectRecord->mPath;
    }

    // Loop through all attachments and create a list to remove
    std::vector<InternalAttachmentDeprecated*> attachmentRemoveList;
    for (size_t i = 0; i < mAttachmentsDeprecated.size(); i++)
    {
        for (size_t j = 0; j < 2; j++)
        {
            // Make sure that attachment is not null
            if (mAttachmentsDeprecated[i])
            {
                if ((mAttachmentsDeprecated[i]->mAttachmentActor[j].mObjectId == objId) ||
                    (mAttachmentsDeprecated[i]->mAttachmentActor[j].mShapeId != kInvalidObjectId && mAttachmentsDeprecated[i]->mAttachmentActor[j].mShapeId == objId))
                {
                    attachmentRemoveList.push_back(mAttachmentsDeprecated[i]);
                }
            }
        }
    }

    for (size_t i = 0; i < attachmentRemoveList.size(); i++)
    {
        SdfPath attachmentPath = attachmentRemoveList[i]->mPath;
        if (removeAttachmentDeprecated(*attachmentRemoveList[i]))
        {
            // Add to attachment history
            attachedStage->getAttachmentHistoryMapDeprecated().insert({ path, attachmentPath });
        }
    }
}

uint32_t InternalScene::updateEventAttachmentsDeprecated(ObjectId shapeId, SdfPath shapePath, InternalAttachmentDeprecated::DirtyEventType eventType)
{
    uint32_t numDirtyAttachments = 0;

    for (size_t i = 0; i < mAttachmentsDeprecated.size(); i++)
    {
        if (mAttachmentsDeprecated[i]->hasShape(shapeId) || eventType == InternalAttachmentDeprecated::eShapeAdded)
        {
            InternalAttachmentDeprecated* attachment = mAttachmentsDeprecated[i];

            if (eventType == InternalAttachmentDeprecated::eShapeRemoved)
            {
                attachment->mDirtyEvent = InternalAttachmentDeprecated::eShapeRemoved;
                attachment->mAttachmentShapeRemovedEvent.shapePath = shapePath;
                attachment->mAttachmentShapeRemovedEvent.removeShapeId = shapeId;
                numDirtyAttachments++;
            }
            else if (eventType == InternalAttachmentDeprecated::eShapeAdded)
            {
                if (attachment->mDirtyEvent == InternalAttachmentDeprecated::eShapeRemoved && attachment->mAttachmentShapeRemovedEvent.shapePath == shapePath)
                {
                    // Shape has been added back because it is a shape move event
                    attachment->mDirtyEvent = InternalAttachmentDeprecated::eNone;
                    attachment->mAttachmentActor[1].mShapeId = shapeId;
                    numDirtyAttachments++;

                    PxShape* shape = getPtr<PxShape>(ePTShape, shapeId);
                    attachment->mNewTransformLocal = shape->getLocalPose();
                }
            }
        }
    }

    return numDirtyAttachments;
}

void InternalScene::swapActorRigidAttachmentsDeprecated(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor)
{
    for (size_t i = 0; i < mAttachmentsDeprecated.size(); i++)
    {
        if (mAttachmentsDeprecated[i]->hasActor(sourceActor))
        {
            mAttachmentsDeprecated[i]->swapRigidActor(sourceActor, destActor);
        }
    }
}

void InternalScene::addDeformableAttachment(InternalDeformableAttachment& deformableAttachment)
{
    mDeformableAttachments.push_back(&deformableAttachment);
}

bool InternalScene::removeDeformableAttachment(InternalDeformableAttachment& deformableAttachment)
{
    InternalDeformableAttachment* attachment = &deformableAttachment;

    auto it = std::find(mDeformableAttachments.begin(), mDeformableAttachments.end(), attachment);
    if (it != mDeformableAttachments.end())
    {
        std::iter_swap(it, --mDeformableAttachments.end());
        mDeformableAttachments.pop_back();

        InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        CARB_ASSERT(attachment->mObjectId < db.getRecords().size());
        InternalDatabase::Record& objectRecord = db.getRecords()[attachment->mObjectId];
        objectRecord.setRemoved();

        SAFE_DELETE_SINGLE(attachment);

        return true;
    }

    return false;
}

void InternalScene::removeDeformableAttachments(ObjectId objId)
{
    // Make sure that objId is a valid id
    if (objId == kInvalidObjectId)
        return;

    omni::physx::OmniPhysX& omniPhysX = omni::physx::OmniPhysX::getInstance();
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(omniPhysX.getStageId());

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objId);
    SdfPath path;
    if (objectRecord)
    {
        path = objectRecord->mPath;
    }

    // Loop through all attachments and create a list to remove
    std::vector<InternalDeformableAttachment*> attachmentRemoveList;
    for (size_t i = 0; i < mDeformableAttachments.size(); i++)
    {
        // Make sure that attachment is not null
        if (mDeformableAttachments[i])
        {
            for (size_t j = 0; j < 2; j++)
            {
                if (mDeformableAttachments[i]->mData[j].objId == objId || mDeformableAttachments[i]->mData[j].rootObjId == objId)
                {
                    attachmentRemoveList.push_back(mDeformableAttachments[i]);
                    break;
                }
            }
        }
    }

    for (size_t i = 0; i < attachmentRemoveList.size(); i++)
    {
        SdfPath attachmentPath = attachmentRemoveList[i]->mPath;
        if (removeDeformableAttachment(*attachmentRemoveList[i]))
        {
            // Add to attachment history
            attachedStage->getDeformableAttachmentHistoryMap().insert({ path, attachmentPath });
        }
    }
}

void InternalScene::swapDeformableAttachmentsRigidActor(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor)
{
    for (size_t i = 0; i < mDeformableAttachments.size(); i++)
    {
        if (mDeformableAttachments[i]->hasRigidActor(sourceActor))
        {
            mDeformableAttachments[i]->swapRigidActor(sourceActor, destActor);
        }
    }
}

void InternalScene::addDeformableCollisionFilter(InternalDeformableCollisionFilter& deformableCollisionFilter)
{
    mDeformableCollisionFilters.push_back(&deformableCollisionFilter);
}

bool InternalScene::removeDeformableCollisionFilter(InternalDeformableCollisionFilter& deformableCollisionFilter)
{
    InternalDeformableCollisionFilter* collisionFilter = &deformableCollisionFilter;

    auto it = std::find(mDeformableCollisionFilters.begin(), mDeformableCollisionFilters.end(), collisionFilter);
    if (it != mDeformableCollisionFilters.end())
    {
        std::iter_swap(it, --mDeformableCollisionFilters.end());
        mDeformableCollisionFilters.pop_back();

        InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        CARB_ASSERT(collisionFilter->mObjectId < db.getRecords().size());
        InternalDatabase::Record& objectRecord = db.getRecords()[collisionFilter->mObjectId];
        objectRecord.setRemoved();

        SAFE_DELETE_SINGLE(collisionFilter);

        return true;
    }

    return false;
}

void InternalScene::removeDeformableCollisionFilters(ObjectId objId)
{
    // Make sure that objId is a valid id
    if (objId == kInvalidObjectId)
        return;

    omni::physx::OmniPhysX& omniPhysX = omni::physx::OmniPhysX::getInstance();
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(omniPhysX.getStageId());

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objId);
    SdfPath path;
    if (objectRecord)
    {
        path = objectRecord->mPath;
    }

    // Loop through all collision filters and create a list to remove
    std::vector<InternalDeformableCollisionFilter*> collisionFilterRemoveList;
    for (size_t i = 0; i < mDeformableCollisionFilters.size(); i++)
    {
        // Make sure that collision filter is not null
        if (mDeformableCollisionFilters[i])
        {
            for (size_t j = 0; j < 2; j++)
            {
                if (mDeformableCollisionFilters[i]->mData[j].objId == objId || mDeformableCollisionFilters[i]->mData[j].rootObjId == objId)
                {
                    collisionFilterRemoveList.push_back(mDeformableCollisionFilters[i]);
                    break;
                }
            }
        }
    }

    for (size_t i = 0; i < collisionFilterRemoveList.size(); i++)
    {
        SdfPath collisionFilterPath = collisionFilterRemoveList[i]->mPath;
        if (removeDeformableCollisionFilter(*collisionFilterRemoveList[i]))
        {
            // Add to collision filter history
            attachedStage->getDeformableCollisionFilterHistoryMap().insert({ path, collisionFilterPath });
        }
    }
}

void InternalScene::swapDeformableCollisionFiltersRigidActor(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor)
{
    for (size_t i = 0; i < mDeformableCollisionFilters.size(); i++)
    {
        if (mDeformableCollisionFilters[i]->hasRigidActor(sourceActor))
        {
            mDeformableCollisionFilters[i]->swapRigidActor(sourceActor, destActor);
        }
    }
}

InternalScene::InternalScene(const PhysxSceneDesc& desc, ::physx::PxScene* scene)
    : mEnabledVehicleCount(0),
      mScene(scene), mReportResiduals(false), mVolumeDeformablePostSolveCallback(nullptr), mSurfaceDeformablePostSolveCallback(nullptr)
{
    mSceneDesc = desc;

    PxCudaContextManager* cudaContextManager = scene->getCudaContextManager();
    if (cudaContextManager)
    {
        PxScopedCudaLock _lock(*cudaContextManager);

        // 0x1 means non-blocking. TODO get things right with priorities.
        cudaContextManager->getCudaContext()->streamCreate(&mDeformableCopyStream, 0x1);

        mVolumeDeformablePostSolveCallback  = ICE_NEW(deformables::VolumeDeformablePostSolveCallback)(mDeformableCopyStream, cudaContextManager, mScene);
        mSurfaceDeformablePostSolveCallback = ICE_NEW(deformables::SurfaceDeformablePostSolveCallback)(mDeformableCopyStream, cudaContextManager, mScene);
    }
    else
    {
        mDeformableCopyStream = nullptr;
    }

    mDeformableCopyStreamDirty = false;
}

InternalScene::~InternalScene()
{
    release();
}

void InternalScene::release()
{
    waitForSimulationCompletion(false);

    for (size_t i = 0; i < mAttachmentsDeprecated.size(); ++i)
    {
        SAFE_DELETE_SINGLE(mAttachmentsDeprecated[i]);
    }
    mAttachmentsDeprecated.clear();
    mAttachmentsDeprecated.shrink_to_fit();

    for (size_t i = 0; i < mDeformableAttachments.size(); ++i)
    {
        SAFE_DELETE_SINGLE(mDeformableAttachments[i]);
    }
    mDeformableAttachments.clear();
    mDeformableAttachments.shrink_to_fit();

    for (size_t i = 0; i < mDeformableCollisionFilters.size(); ++i)
    {
        SAFE_DELETE_SINGLE(mDeformableCollisionFilters[i]);
    }
    mDeformableCollisionFilters.clear();
    mDeformableCollisionFilters.shrink_to_fit();

    for (size_t i = 0; i < mParticleSystems.size(); i++)
    {
        SAFE_DELETE_SINGLE(mParticleSystems[i]);
    }
    mParticleSystems.clear();
    mParticleSystems.shrink_to_fit();

    for (size_t i = 0; i < mDeformableBodiesDeprecated.size(); i++)
    {
        InternalDeformableBodyDeprecated* current = mDeformableBodiesDeprecated[i];
        SAFE_DELETE_SINGLE(current);
    }
    mDeformableBodiesDeprecated.clear();
    mDeformableBodiesDeprecated.shrink_to_fit();

    for (size_t i = 0; i < mDeformableSurfacesDeprecated.size(); i++)
    {
        InternalDeformableSurfaceDeprecated* current = mDeformableSurfacesDeprecated[i];
        SAFE_DELETE_SINGLE(current);
    }
    mDeformableSurfacesDeprecated.clear();
    mDeformableSurfacesDeprecated.shrink_to_fit();

    for (size_t i = 0; i < mVolumeDeformableBodies.size(); i++)
    {
        InternalVolumeDeformableBody* current = mVolumeDeformableBodies[i];
        SAFE_DELETE_SINGLE(current);
    }
    mVolumeDeformableBodies.clear();
    mVolumeDeformableBodies.shrink_to_fit();

    for (size_t i = 0; i < mSurfaceDeformableBodies.size(); i++)
    {
        InternalSurfaceDeformableBody* current = mSurfaceDeformableBodies[i];
        SAFE_DELETE_SINGLE(current);
    }
    mSurfaceDeformableBodies.clear();
    mSurfaceDeformableBodies.shrink_to_fit();

    RaycastManager& rayMan = OmniPhysX::getInstance().getRaycastManager();
    const uint32_t nbActors = uint32_t(mActors.size());
    for (uint32_t i = 0; i < nbActors; i++)
    {
        InternalActor* current = mActors[i];
        if (!current->mActor->is<PxArticulationLink>())
        {
            if(current->mActor->is<PxRigidBody>())
            {
                rayMan.clearPicker(current->mActor);
            }
            SAFE_RELEASE(current->mActor)
        }
        for (MirrorActor& mirror : current->mMirrors)
        {
            mirror.release();
        }
        SAFE_RELEASE(current->mMirrorSharedCollection);
        if (current->mMirrorMemory)
            free(current->mMirrorMemory);        
        SAFE_DELETE_ALLOCABLE_SINGLE(current)
    }
    mActors.clear();
    mActors.shrink_to_fit();

    const uint32_t nbArticulations = uint32_t(mArticulations.size());
    for (uint32_t i = 0; i < nbArticulations; i++)
    {
        PxArticulationReducedCoordinate* current = mArticulations[i];
        current->release();
    }
    mArticulations.clear();
    mArticulations.shrink_to_fit();

    mVehicles.clear();
    mVehicleActorToVehicle.clear();

    MimicJointSet::iterator it = mMimicJointSet.begin();
    while (it != mMimicJointSet.end())
    {
        constexpr bool removeFromTrackers = false;

        // the integration code relies on PhysX articulations to release all related mimic joints if the articulation
        // gets released. Thus skipping the PhysX mimic joint object release here.
        constexpr bool releasePhysXObject = false;

        (*it)->release(removeFromTrackers, releasePhysXObject);
        it++;
    }

    PxCudaContextManager* cudaContextManager = mScene->getCudaContextManager();
    if (cudaContextManager)
    {
        PxScopedCudaLock _lock(*cudaContextManager);

        mDeformableCopyStreamDirty = false;
        cudaContextManager->getCudaContext()->streamDestroy(mDeformableCopyStream);
    }

    if (mVolumeDeformablePostSolveCallback)
        PX_DELETE(mVolumeDeformablePostSolveCallback);

    if (mSurfaceDeformablePostSolveCallback)
        PX_DELETE(mSurfaceDeformablePostSolveCallback);
}


struct Transform
{
    GfVec3f position;
    GfQuatf orientation;
    GfVec3f scale;
};

static PXR_NS::GfMatrix4d getGfMatrix4d(const Transform& transform)
{
    PXR_NS::GfMatrix4d mat;
    PXR_NS::GfMatrix4d rotMat;
    PXR_NS::GfMatrix4d scaleMat;

    scaleMat.SetScale(transform.scale);
    rotMat.SetRotate(transform.orientation);
    mat = scaleMat * rotMat;
    mat.SetTranslateOnly(transform.position);

    return mat;
}

bool setPrimXformOpsFast(const InternalActor& actor, const SdfLayerHandle& layer, const GfMatrix4d& mat)
{
    if (layer)
    {        
        SdfAttributeSpecHandle posAttr = layer->GetAttributeAtPath(actor.mXformOpTranslatePath);
        SdfAttributeSpecHandle orAttr = layer->GetAttributeAtPath(actor.mXformOpOrientPath);
        if (posAttr && orAttr)
        {
            const GfTransform tr(mat);
            posAttr->SetDefaultValue(VtValue(GfVec3f(tr.GetTranslation())));
            orAttr->SetDefaultValue(VtValue(GfQuatf(tr.GetRotation().GetQuat())));
            return true;
        }
    }

    return false;
}

bool setPrimXformOpsFast(const InternalActor& actor, const SdfLayerHandle& layer, const Transform& transform)
{
    if (layer)
    {        
        SdfAttributeSpecHandle posAttr = layer->GetAttributeAtPath(actor.mXformOpTranslatePath);
        SdfAttributeSpecHandle orAttr = layer->GetAttributeAtPath(actor.mXformOpOrientPath);
        if (posAttr && orAttr)
        {
            posAttr->SetDefaultValue(VtValue(transform.position));
            orAttr->SetDefaultValue(VtValue(transform.orientation));
            return true;
        }
    }

    return false;
}

void setPrimXformOps(UsdPrim& prim, const GfMatrix4d& mat, bool setScale)
{
    const GfTransform tr(mat);

    UsdGeomXformable primXform(prim);
    
    bool resetXformStack = false;
    bool translateSet = false;
    bool orientSet = false;

    const std::vector<UsdGeomXformOp> xformOps = primXform.GetOrderedXformOps(&resetXformStack);
    for (const UsdGeomXformOp& op : xformOps)
    {
        const TfToken opName = op.GetOpName();
        const UsdGeomXformOp::Precision opPrecision = op.GetPrecision();

        if (opName == gTokTransform)
        {
            op.Set(mat);
            return;
        }
        else if (opName == gTokTranslate && !translateSet)
        {
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(GfVec3f(tr.GetTranslation()));
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfVec3d(tr.GetTranslation()));

            translateSet = true;
        }
        else if (setScale && opName == gTokScale)
        {
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(GfVec3f(tr.GetScale()));
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfVec3d(tr.GetScale()));
        }
        else if (opName == gTokOrient && !orientSet)
        {
            const GfRotation rot = tr.GetRotation();
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(GfQuatf(rot.GetQuat()));
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfQuatd(rot.GetQuat()));
            else if (opPrecision == UsdGeomXformOp::PrecisionHalf)
                op.Set(GfQuath(rot.GetQuat()));

            orientSet = true;
        }
        else if (opName == gTokRotateZYX && !orientSet)
        {
            const GfRotation rot = tr.GetRotation();
            const GfVec3d angles =
                rot.Decompose(GfVec3d::XAxis(), GfVec3d::YAxis(), GfVec3d::ZAxis());
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(GfVec3f(float(angles[0]), float(angles[1]), float(angles[2])));
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfVec3d(angles[0], angles[1], angles[2]));

            orientSet = true;
        }
    }

    // if xformop update failed, fall back to matrix transform
    if (!translateSet || !orientSet)
    {
        primXform.ClearXformOpOrder();
        UsdGeomXformOp xform = primXform.MakeMatrixXform();
        if (xform)
            xform.Set(mat);
    }
}

void setPrimXformOps(UsdPrim& prim, const Transform& transform, bool setScale)
{
    UsdGeomXformable primXform(prim);

    bool translateSet = false;
    bool orientSet = false;
    bool resetXformStack = false;

    const std::vector<UsdGeomXformOp> xformOps = primXform.GetOrderedXformOps(&resetXformStack);
    for (const UsdGeomXformOp& op : xformOps)
    {
        const TfToken opName = op.GetOpName();
        const UsdGeomXformOp::Precision opPrecision = op.GetPrecision();

        if (opName == gTokTransform)
        {
            GfMatrix4d mat = getGfMatrix4d(transform);
            op.Set(mat);
            return;
        }
        else if (opName == gTokTranslate && !translateSet)
        {
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(transform.position);
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfVec3d(transform.position));

            translateSet = true;
        }
        else if (opName == gTokScale && setScale)
        {
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(transform.scale);
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfVec3d(transform.scale));
        }
        else if (opName == gTokOrient && !orientSet)
        {
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(transform.orientation);
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfQuatd(transform.orientation));
            else if (opPrecision == UsdGeomXformOp::PrecisionHalf)
                op.Set(GfQuath(transform.orientation));

            orientSet = true;
        }
        else if (opName == gTokRotateZYX && !orientSet)
        {
            const GfMatrix4d mat = getGfMatrix4d(transform);
            const GfTransform tr(mat);
            const GfRotation rot = tr.GetRotation();
            const GfVec3d angles =
                rot.Decompose(GfVec3d::XAxis(), GfVec3d::YAxis(), GfVec3d::ZAxis());
            if (opPrecision == UsdGeomXformOp::PrecisionFloat)
                op.Set(GfVec3f(float(angles[0]), float(angles[1]), float(angles[2])));
            else if (opPrecision == UsdGeomXformOp::PrecisionDouble)
                op.Set(GfVec3d(angles[0], angles[1], angles[2]));

            orientSet = true;
        }
    }

    // if xformop update failed, fall back to matrix transform
    if (!orientSet || !translateSet)
    {
        const GfMatrix4d mat = getGfMatrix4d(transform);
        UsdGeomXformOp xform = primXform.MakeMatrixXform();
        if (xform)
            xform.Set(mat);
    }
}

void processExtraTransforms(const InternalActor& actor, Transform& transform)
{
    if (actor.mFlags & InternalActorFlag::eHAS_EXTRA_TRANSFORM)
    {
        GfMatrix4d worldPose = getGfMatrix4d(transform);
        if (actor.mFlags & InternalActorFlag::eEXTRA_TRANSFORM_PRE_OP)
        {
            worldPose = worldPose * actor.mExtraTransfInv;
        }
        else
        {
            worldPose = actor.mExtraTransfInv * worldPose;
        }

        transform.position = GfVec3f(worldPose.ExtractTranslation());
        transform.orientation = GfQuatf(worldPose.ExtractRotation().GetQuat());
    }
}

void processExtraTransforms(const InternalActor& actor, GfMatrix4d& transform)
{
    if (actor.mFlags & InternalActorFlag::eHAS_EXTRA_TRANSFORM)
    {
        if (actor.mFlags & InternalActorFlag::eEXTRA_TRANSFORM_PRE_OP)
        {
            transform = transform * actor.mExtraTransfInv;
        }
        else
        {
            transform = actor.mExtraTransfInv * transform;
        }
    }
}

static void writeSingleNonRootTransformToUsd(UsdPrim& prim,
                                             UsdPrim& parentXformPrim,
                                             UsdGeomXformCache& xformCache,
                                             const Transform& transform)
{
    const GfMatrix4d worldPose = getGfMatrix4d(transform);
    const GfMatrix4d parentWorldTransf = xformCache.GetLocalToWorldTransform(parentXformPrim);
    const GfMatrix4d parentWorldTransfInv = parentWorldTransf.GetInverse();
    GfMatrix4d localTransf = worldPose * parentWorldTransfInv;

    setPrimXformOps(prim, localTransf, false);
}

static void writeSingleNonRootTransformToUsd(const InternalActor& actor,
    const SdfLayerHandle& layer,
    UsdPrim& prim,    
    const GfMatrix4d& parentWorldTransfInv,    
    const Transform& transform)
{
    const GfMatrix4d worldPose = getGfMatrix4d(transform);
    GfMatrix4d localTransf = worldPose * parentWorldTransfInv;

    processExtraTransforms(actor, localTransf);
    if (actor.mFlags & InternalActorFlag::eFAST_TRANSFORM)
    {
        if (!setPrimXformOpsFast(actor, layer, localTransf))
        {
            setPrimXformOps(prim, localTransf, false);
        }
    }
    else
    {
        setPrimXformOps(prim, localTransf, false);
    }
}

void writeSingleTransformToUsd(InternalActor& actor, const SdfLayerHandle& layer, UsdGeomXformCache& xformCache, Transform& transform)
{
    if (!actor.mPrim)
        return;

    if (actor.mFlags & InternalActorFlag::eHAS_PARENT_XFORM)
    {
        if (actor.mFlags & InternalActorFlag::ePARENT_XFORM_DIRTY)
        {
            const GfMatrix4d parentWorldTransf = xformCache.GetLocalToWorldTransform(actor.mParentXformPrim);
            actor.mParentWorldTransfInv = parentWorldTransf.GetInverse();
            actor.mFlags &=~InternalActorFlag::ePARENT_XFORM_DIRTY;

            GfVec3f position = GfVec3f(parentWorldTransf.ExtractTranslation());
            GfQuatf orientation = GfQuatf(parentWorldTransf.ExtractRotationQuat());
        }

        writeSingleNonRootTransformToUsd(actor, layer, actor.mPrim, actor.mParentWorldTransfInv, transform);
    }
    else
    {
        processExtraTransforms(actor, transform);
        if (actor.mFlags & InternalActorFlag::eFAST_TRANSFORM)
        {
            if (!setPrimXformOpsFast(actor, layer, transform))
            {
                setPrimXformOps(actor.mPrim, transform, false);
            }
        }
        else
        {
            setPrimXformOps(actor.mPrim, transform, false);
        }
    }
}

void InternalScene::resetStartProperties(bool useUsdUpdate, bool useVelocitiesUSDUpdate, bool outputVelocitiesLocalSpace)
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();

    for (InternalVehicle*& vehicle : mVehicles)
    {
        // note: the vehicle actor is covered by the actor code. Here we deal with the controller properties,
        // wheels and shapes since the vehicle simulation defines the wheel local pose (and shape local pose
        // if PhysX shapes are assigned to wheels)

        if (useUsdUpdate)
            vehicle->restoreInitialProperties();

        const uint32_t wheelTMEntryCount = static_cast<uint32_t>(vehicle->mWheelTransformManagementEntries.size());
        if (wheelTMEntryCount)
        {
            for (uint32_t i = 0; i < wheelTMEntryCount; i++)
            {
                CARB_ASSERT(i < vehicle->mWheelAttachments.size());
                if (vehicle->mWheelAttachments[i])  // the wheel attachment might have been removed
                {
                    InternalVehicle::WheelTransformManagementEntry& wheelTMEntry =
                        vehicle->mWheelTransformManagementEntries[i];

                    const bool hasNonRootShape =
                        wheelTMEntry.shape && (wheelTMEntry.wheelRootPrim != wheelTMEntry.shapePrim);

                    if (useUsdUpdate)
                    {
                        setPrimXformOps(wheelTMEntry.wheelRootPrim, wheelTMEntry.initialTransform, false);

                        if (hasNonRootShape)
                            setPrimXformOps(wheelTMEntry.shapePrim, wheelTMEntry.initialShapeTransform, false);
                    }
                }
            }
        }
    }

    for (size_t particleSystemIndex = 0; particleSystemIndex < mParticleSystems.size(); particleSystemIndex++)
    {
        InternalPbdParticleSystem* particleSystem = mParticleSystems[particleSystemIndex];

        for (size_t particleSetIndex = 0; particleSetIndex < particleSystem->mParticleSets.size(); particleSetIndex++)
        {
            InternalParticleSet* particleSet = particleSystem->mParticleSets[particleSetIndex];

            if (!particleSet->mNumParticles)
                continue;

            // transform particles from world space back to prim local space
            UsdGeomXform xform(particleSet->mPrim);
            GfMatrix4f worldToLocal = GfMatrix4f(xform.ComputeLocalToWorldTransform(UsdTimeCode::Default()).GetInverse());

            // reset start state of particles:
            UsdGeomPointBased pointBased(particleSet->mPrim);
            UsdGeomPointInstancer pointInstancer(particleSet->mPrim);
            {
                UsdAttribute pointAttribute = pointBased ? pointBased.GetPointsAttr() : pointInstancer.GetPositionsAttr();
                VtArray<GfVec3f> outPoints;
                copyBuffer(outPoints, &particleSet->mPositionSaveRestoreBuf[0], uint32_t(particleSet->mPositionSaveRestoreBuf.size()), worldToLocal);
                pointAttribute.Set(outPoints);
            }

            {
                UsdAttribute velocitiesAttribute = pointBased ? pointBased.GetVelocitiesAttr() : pointInstancer.GetVelocitiesAttr();
                VtArray<GfVec3f> outVelocities;
                copyBuffer(outVelocities, &particleSet->mVelocitySaveRestoreBuf[0], uint32_t(particleSet->mVelocitySaveRestoreBuf.size()));
                velocitiesAttribute.Set(outVelocities);
            }

            PhysxSchemaPhysxParticleSetAPI particleSetApi(particleSet->mPrim);
            UsdAttribute simulationPointsAttr = particleSetApi.GetSimulationPointsAttr();
            if (simulationPointsAttr.HasAuthoredValue())
            {
                VtArray<GfVec3f> outSimPositions;
                copyBuffer(outSimPositions, &particleSet->mPositionSaveRestoreBuf[0], uint32_t(particleSet->mPositionSaveRestoreBuf.size()), worldToLocal);
                simulationPointsAttr.Set(outSimPositions);
            }

            particleSet->mNumParticles = uint32_t(particleSet->mPositionSaveRestoreBuf.size());

            if (pointInstancer)
            {
                if (pointInstancer.GetOrientationsAttr().HasValue())
                {
                    pointInstancer.GetOrientationsAttr().Set(VtArray<GfQuath>(particleSet->mNumParticles, GfQuath(1.0f)));
                }

                if (pointInstancer.GetScalesAttr().HasValue())
                {
                    pointInstancer.GetScalesAttr().Set(VtArray<GfVec3f>(particleSet->mNumParticles, GfVec3f(1.0f)));
                }
            }

            if (particleSet->mNumParticles == 0)
            {
                //Workaround for OM-54774
                particleSet->mPrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(true);
                particleSet->mPrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(false);
            }
        }

        for (size_t clothIndex = 0; clothIndex < particleSystem->mCloths.size(); clothIndex++)
        {
            InternalParticleClothDeprecated* cloth = particleSystem->mCloths[clothIndex];

            // write back xformOp
            UsdGeomXformable xformable{ cloth->mPrim };
            cloth->mXformOpStorage.restore(xformable, true);

            // transform particles from world space back to prim local space
            UsdGeomXform xform(cloth->mPrim);

            GfMatrix4f localToWorld = GfMatrix4f(cloth->mInitialLocalToWorld);
            GfMatrix4f worldToLocal = localToWorld.GetInverse();
            GfMatrix4f parentToWorld = GfMatrix4f(xform.ComputeParentToWorldTransform(UsdTimeCode::Default()));
            GfMatrix4f localToParent = localToWorld * parentToWorld.GetInverse();

            VtArray<GfVec3f> tmpPoints;
            VtArray<GfVec3f> tmpVelocities;
            if (cloth->mIsWelded)
            {
                VtArray<GfVec3f> weldedPoints;
                VtArray<GfVec3f> weldedVelocities;
                copyBuffer(weldedPoints, &cloth->mPositionSaveRestoreBuf[0], cloth->mNumParticles, worldToLocal);
                copyBuffer(weldedVelocities, &cloth->mVelocitySaveRestoreBuf[0], cloth->mNumParticles);

                //initialize from usd mesh, as non-referenced vertices are not mapped
                cloth->mGeo.GetPointsAttr().Get(&tmpPoints);
                cloth->mGeo.GetVelocitiesAttr().Get(&tmpVelocities);
                const size_t nbMapped = PxMin(cloth->mVerticesRemapToWeld.size(), PxMin(tmpPoints.size(), tmpVelocities.size()));
                for (size_t i = 0; i < nbMapped; ++i)
                {
                    const uint32_t index = cloth->mVerticesRemapToWeld[i];
                    if (index != 0xffffffff) // non referenced vertices are not simulated
                    {
                        tmpPoints[i] = weldedPoints[index];
                        tmpVelocities[i] = weldedVelocities[index];
                    }
                }
            }
            else
            {
                copyBuffer(tmpPoints, &cloth->mPositionSaveRestoreBuf[0], cloth->mNumParticles, worldToLocal);
                copyBuffer(tmpVelocities, &cloth->mVelocitySaveRestoreBuf[0], cloth->mNumParticles);
            }

            {
                cloth->mGeo.GetPointsAttr().Set(tmpPoints);
                cloth->mGeo.GetVelocitiesAttr().Set(tmpVelocities);
                cloth->mGeo.GetExtentAttr().Set(cloth->mExtentSaveRestoreBuf);
            }
        }
    }

    for (size_t i = 0; i < mDeformableBodiesDeprecated.size(); i++)
    {
        InternalDeformableBodyDeprecated* deformablebody = mDeformableBodiesDeprecated[i];

        // write back xformOp
        UsdGeomXformable xformable{ deformablebody->mPrim };
        deformablebody->mXformOpStorage.restore(xformable, true);

        // transform particles from world space back to prim local space
        GfMatrix4f parentToWorld = GfMatrix4f(UsdGeomXform(deformablebody->mPrim).ComputeParentToWorldTransform(UsdTimeCode::Default()));
        GfMatrix4f worldToSoftbody = (GfMatrix4f(deformablebody->mInitialPrimToParent) * parentToWorld).GetInverse();
        GfMatrix4f softbodyToWorld = worldToSoftbody.GetInverse();

        if (!deformablebody->mIsPartiallyKinematic)
        {
            VtArray<GfVec3f> tmpSkinMeshPositions;
            copyBuffer(tmpSkinMeshPositions, &deformablebody->mInitialCollSkinMeshPositions[deformablebody->mNumCollMeshVertices], deformablebody->mNumSkinMeshVertices, worldToSoftbody);
            UsdGeomMesh mesh(deformablebody->mPrim);
            mesh.GetPointsAttr().Set(tmpSkinMeshPositions);
            mesh.GetExtentAttr().Set(deformablebody->mExtentSaveRestoreBuf);
        }
        VtArray<GfVec3f> tmpCollMeshPositions;
        VtArray<GfVec3f> tmpSimMeshPositions;
        VtArray<GfVec3f> tmpSimMeshVelocities;
        copyBuffer(tmpCollMeshPositions, &deformablebody->mInitialCollSkinMeshPositions[0], deformablebody->mNumCollMeshVertices, worldToSoftbody);
        copyBuffer(tmpSimMeshPositions, &deformablebody->mInitialSimMeshPositions[0], deformablebody->mNumSimMeshVertices, worldToSoftbody);
        copyBuffer(tmpSimMeshVelocities, &deformablebody->mInitialSimMeshVelocities[0], deformablebody->mNumSimMeshVertices);
        PhysxSchemaPhysxDeformableBodyAPI(deformablebody->mPrim).GetCollisionPointsAttr().Set(tmpCollMeshPositions);
        PhysxSchemaPhysxDeformableBodyAPI(deformablebody->mPrim).GetSimulationPointsAttr().Set(tmpSimMeshPositions);
        PhysxSchemaPhysxDeformableAPI(deformablebody->mPrim).GetSimulationVelocitiesAttr().Set(tmpSimMeshVelocities);
    }

    for (size_t i = 0; i < mDeformableSurfacesDeprecated.size(); i++)
    {
        InternalDeformableSurfaceDeprecated* deformableSurface = mDeformableSurfacesDeprecated[i];

        // write back xformOp
        UsdGeomXformable xformable{ deformableSurface->mPrim };
        deformableSurface->mXformOpStorage.restore(xformable, true);

        //workaround for FEM cloths created during simulation
        if (!deformableSurface->mCollMeshPositionSaveRestoreBuf.empty())
        {
            // transform particles from world space back to prim local space
            GfMatrix4f deformableSurfaceToWorld = GfMatrix4f(deformableSurface->mInitialPrimToWorld);
            GfMatrix4f parentToWorld = GfMatrix4f(UsdGeomXform(deformableSurface->mPrim).ComputeParentToWorldTransform(UsdTimeCode::Default()));
            GfMatrix4f deformableSurfaceToParent = deformableSurfaceToWorld * parentToWorld.GetInverse();

            VtArray<GfVec3f> tmpCollMeshPositions;
            VtArray<GfVec3f> tmpCollMeshVelocities;
            copyBuffer(tmpCollMeshPositions, &deformableSurface->mCollMeshPositionSaveRestoreBuf[0], deformableSurface->mNumCollMeshVerticesOrig, deformableSurfaceToWorld.GetInverse(), deformableSurface->mPhysxToUsdVtxMap);
            copyBuffer(tmpCollMeshVelocities, &deformableSurface->mCollMeshVelocitySaveRestoreBuf[0], deformableSurface->mNumCollMeshVerticesOrig, deformableSurface->mPhysxToUsdVtxMap);
            {
                UsdGeomMesh mesh(deformableSurface->mPrim);
                mesh.GetPointsAttr().Set(tmpCollMeshPositions);
                mesh.GetExtentAttr().Set(deformableSurface->mExtentSaveRestoreBuf);
                PhysxSchemaPhysxDeformableAPI(deformableSurface->mPrim).GetSimulationVelocitiesAttr().Set(tmpCollMeshVelocities);
            }
        }
    }

    for (size_t i = 0; i < mVolumeDeformableBodies.size(); i++)
    {
        InternalVolumeDeformableBody* deformableBody = mVolumeDeformableBodies[i];

        if (!deformableBody->mIsKinematic)
        {
            const Float3* srcPtr = deformableBody->mAllSkinMeshPointsSaveRestoreBuf.data();
            const uint32_t srcSize = uint32_t(deformableBody->mAllSkinMeshPointsSaveRestoreBuf.size());

            for (size_t i = 0; i < deformableBody->mSkinMeshPrims.size(); ++i)
            {
                UsdGeomPointBased skin(deformableBody->mSkinMeshPrims[i]);
                if (skin)
                {
                    const Uint2& range = deformableBody->mSkinMeshRanges[i];
                    VtArray<GfVec3f> points;
                    skin.GetPointsAttr().Get(&points);
                    if (points.size() == range.y && srcSize >= range.x + range.y)
                    {
                        std::memcpy(points.data(), srcPtr + range.x, sizeof(GfVec3f) * range.y);
                        skin.GetPointsAttr().Set(points);
                    }
                }
            }
        }

        {
            UsdGeomPointBased simGeom(deformableBody->mSimMeshPrim);
            if (simGeom)
            {
                {
                    const Float3* srcPtr = deformableBody->mSimMeshPointsSaveRestoreBuf.data();
                    const uint32_t srcSize = uint32_t(deformableBody->mSimMeshPointsSaveRestoreBuf.size());
                    VtArray<GfVec3f> points;
                    simGeom.GetPointsAttr().Get(&points);
                    if (points.size() == srcSize)
                    {
                        std::memcpy(points.data(), srcPtr, sizeof(GfVec3f) * points.size());
                        simGeom.GetPointsAttr().Set(points);
                    }
                }

                {
                    const Float3* srcPtr = deformableBody->mSimMeshVelocitiesSaveRestoreBuf.data();
                    const uint32_t srcSize = uint32_t(deformableBody->mSimMeshVelocitiesSaveRestoreBuf.size());
                    VtArray<GfVec3f> velocities;
                    simGeom.GetVelocitiesAttr().Get(&velocities);
                    if (velocities.size() == srcSize)
                    {
                        std::memcpy(velocities.data(), srcPtr, sizeof(GfVec3f) * velocities.size());
                        simGeom.GetVelocitiesAttr().Set(velocities);
                    }
                }
            }

            UsdGeomPointBased collGeom(deformableBody->mCollMeshPrim);
            if (collGeom)
            {
                if (collGeom.GetPrim() != simGeom.GetPrim())
                {
                    const Float3* srcPtr = deformableBody->mCollMeshPointsSaveRestoreBuf.data();
                    const uint32_t srcSize = uint32_t(deformableBody->mCollMeshPointsSaveRestoreBuf.size());
                    VtArray<GfVec3f> points;
                    collGeom.GetPointsAttr().Get(&points);
                    if (points.size() == srcSize)
                    {
                        std::memcpy(points.data(), srcPtr, sizeof(GfVec3f) * points.size());
                        collGeom.GetPointsAttr().Set(points);
                    }
                }

                collGeom.GetExtentAttr().Set(deformableBody->mCollMeshExtentSaveRestoreBuf);
            }
        }
    }

    for (size_t i = 0; i < mSurfaceDeformableBodies.size(); i++)
    {
        InternalSurfaceDeformableBody* deformableBody = mSurfaceDeformableBodies[i];

        if (!deformableBody->mIsKinematic)
        {
            const Float3* srcPtr = deformableBody->mAllSkinMeshPointsSaveRestoreBuf.data();
            const uint32_t srcSize = uint32_t(deformableBody->mAllSkinMeshPointsSaveRestoreBuf.size());

            for (size_t i = 0; i < deformableBody->mSkinMeshPrims.size(); ++i)
            {
                UsdGeomPointBased skin(deformableBody->mSkinMeshPrims[i]);
                if (skin)
                {
                    const Uint2& range = deformableBody->mSkinMeshRanges[i];
                    VtArray<GfVec3f> points;
                    skin.GetPointsAttr().Get(&points);
                    if (points.size() == range.y && srcSize >= range.x + range.y)
                    {
                        std::memcpy(points.data(), srcPtr + range.x, sizeof(GfVec3f) * range.y);
                        skin.GetPointsAttr().Set(points);
                    }
                }
            }
        }

        {
            UsdGeomMesh simMesh(deformableBody->mSimMeshPrim);
            if (simMesh)
            {
                {
                    const Float3* srcPtr = deformableBody->mSimMeshPointsSaveRestoreBuf.data();
                    const uint32_t srcSize = uint32_t(deformableBody->mSimMeshPointsSaveRestoreBuf.size());
                    VtArray<GfVec3f> points;
                    simMesh.GetPointsAttr().Get(&points);
                    if (points.size() == srcSize)
                    {
                        std::memcpy(points.data(), srcPtr, sizeof(GfVec3f) * points.size());
                        simMesh.GetPointsAttr().Set(points);
                    }
                }

                {
                    const Float3* srcPtr = deformableBody->mSimMeshVelocitiesSaveRestoreBuf.data();
                    const uint32_t srcSize = uint32_t(deformableBody->mSimMeshVelocitiesSaveRestoreBuf.size());
                    VtArray<GfVec3f> velocities;
                    simMesh.GetVelocitiesAttr().Get(&velocities);
                    if (velocities.size() == srcSize)
                    {
                        std::memcpy(velocities.data(), srcPtr, sizeof(GfVec3f) * velocities.size());
                        simMesh.GetVelocitiesAttr().Set(velocities);
                    }
                }

                simMesh.GetExtentAttr().Set(deformableBody->mSimMeshExtentSaveRestoreBuf);
            }
        }
    }
}

InternalVehicle* InternalScene::getVehicleBody(const ::physx::PxRigidDynamic& rigidDynamic) const
{
    if (!mVehicles.size())
        return nullptr;

    ActorToVehicleMap::const_iterator iter = mVehicleActorToVehicle.find(&rigidDynamic);
    if (iter != mVehicleActorToVehicle.end())
    {
        return iter->second;
    }
    else
    {
        return nullptr;
    }
}

void InternalScene::updateVehicleOnMassChange(const PxRigidDynamic& rigidDynamic, const float mass,
    const ::physx::PxVec3& massSpaceInertiaTensor, const ::physx::PxTransform& centerOfMassFrame)
{
    InternalVehicle* internalVehicle = getVehicleBody(rigidDynamic);
    if (internalVehicle)
    {
        internalVehicle->updateMassProperties(mass, massSpaceInertiaTensor, centerOfMassFrame);
    }
}

void InternalScene::updateVehicleOnRemovedShape(const PxRigidActor& rigidActor, const PxShape* removedShape)
{
    if (rigidActor.getType() == PxActorType::eRIGID_DYNAMIC)
    {
        const PxRigidDynamic& rigidDynamic = static_cast<const PxRigidDynamic&>(rigidActor);
        InternalVehicle* internalVehicle = getVehicleBody(rigidDynamic);
        if (internalVehicle)
        {
            internalVehicle->updateShapeMappings(removedShape);
        }
    }
}

void InternalScene::updateVehicleTransforms(bool updateToUsd)
{
    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteTransforms = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);
    TransformUpdateNotificationFn transformFn = cb->getTransformationWriteFn();
    void* cbUserData = cb->getUserData();
    const bool notifyTransforms = transformFn && cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eNOTIFY_UPDATE);
    UsdGeomXformCache& xformCache = OmniPhysX::getInstance().getInternalPhysXDatabase().mXformCache;

    const uint32_t enabledVehicleCount = mEnabledVehicleCount;
    for (uint32_t i = 0; i < enabledVehicleCount; i++)
    {
        InternalVehicle* vehicle = mVehicles[i];

        // note: the vehicle actor is covered by the actor code. Here we deal with the wheels and shapes since the
        // vehicle simulation defines the wheel local pose (and shape local pose if PhysX shapes are assigned to wheels)

        PxRigidDynamic* vehicleActor = vehicle->getRigidDynamicActor();
        CARB_ASSERT(vehicleActor);

        if (!vehicleActor->isSleeping())
        {
            const uint32_t wheelTMEntryCount =
                static_cast<uint32_t>(vehicle->mWheelTransformManagementEntries.size());
            if (wheelTMEntryCount)
            {
                PxTransform actor2World = vehicleActor->getGlobalPose();

                if (!actor2World.isValid())
                {
                    CARB_LOG_WARN("Invalid PhysX transform detected for %s.", vehicleActor->getName());
                }
                else
                {
                    PxTransform body2World;
                    body2World = actor2World * vehicleActor->getCMassLocalPose();

                    for (uint32_t j = 0; j < wheelTMEntryCount; j++)
                    {
                        CARB_ASSERT(j < vehicle->mWheelAttachments.size());
                        InternalVehicleWheelAttachment* wheelAtt = vehicle->mWheelAttachments[j];
                        if (wheelAtt)  // the wheel attachment might have been removed
                        {
                            InternalVehicle::WheelTransformManagementEntry& wheelTMEntry =
                                vehicle->mWheelTransformManagementEntries[j];
                            PxTransform wheelGlobalPose;
                            if (wheelTMEntry.shape)
                                wheelGlobalPose = actor2World * wheelTMEntry.shape->getLocalPose();
                            else
                            {
                                const uint32_t wheelIndex = wheelAtt->mWheelIndex;
                                wheelGlobalPose = body2World * vehicle->mPhysXVehicle->getWheelLocalPose(wheelIndex);
                                // note: the local pose is relative to the center of mass frame.
                            }

                            if (notifyTransforms || (vehicle->mFlags & InternalVehicleFlag::eNOTIFY_TRANSFORM &&
                                                     cb->getTransformationWriteFn()))
                            {
                                transformFn(asInt(wheelTMEntry.wheelRootPrim.GetPath()), fromPhysX(wheelGlobalPose.p),
                                            fromPhysX(wheelGlobalPose.q), cbUserData);
                            }

                            if (!wheelGlobalPose.isValid())
                            {
                                CARB_LOG_WARN("Invalid PhysX transform detected for %s on wheel attachment %s.",
                                              vehicleActor->getName(), wheelTMEntry.wheelRootPrim.GetPath().GetText());
                            }
                            else if (!skipWriteTransforms &&
                                     !(vehicle->mFlags & InternalVehicleFlag::eSKIP_UPDATE_TRANSFORM))
                            {
                                Transform fcTransform;
                                fcTransform.position = (GfVec3f&)wheelGlobalPose.p;
                                fcTransform.orientation = (GfQuatf&)wheelGlobalPose.q;
                                fcTransform.scale = (GfVec3f&)wheelTMEntry.scale;

                                if (updateToUsd)
                                    writeSingleNonRootTransformToUsd(wheelTMEntry.wheelRootPrim,
                                                                     wheelTMEntry.wheelRootParentXformPrim, xformCache,
                                                                     fcTransform);

                                if (wheelTMEntry.shape && (wheelTMEntry.wheelRootPrim != wheelTMEntry.shapePrim))
                                {
                                    // the shape position and orientation is set to the same as the wheel root
                                    fcTransform.scale = (GfVec3f&)wheelTMEntry.shapeScale;

                                    if (updateToUsd)
                                    {
                                        Transform fcIdentityTransform;
                                        fcIdentityTransform.position[0] = 0.0f;
                                        fcIdentityTransform.position[1] = 0.0f;
                                        fcIdentityTransform.position[2] = 0.0f;
                                        fcIdentityTransform.orientation.SetImaginary(GfVec3f(0.0f));
                                        fcIdentityTransform.orientation.SetReal(1.0f);
                                        // note: dummy scale as it should be ignored in the subsequent call anyway
                                        fcIdentityTransform.scale[0] = 1.0f;
                                        fcIdentityTransform.scale[1] = 1.0f;
                                        fcIdentityTransform.scale[2] = 1.0f;

                                        setPrimXformOps(wheelTMEntry.shapePrim, fcIdentityTransform, false);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void InternalScene::updateCctTransforms(bool updateToUsd)
{
    UsdGeomXformCache& xformCache = OmniPhysX::getInstance().getInternalPhysXDatabase().mXformCache;
    SdfLayerHandle currentLayer = OmniPhysX::getInstance().getStage()->GetEditTarget().GetLayer();
    CctMap::iterator it = mCctMap.begin();
    while (it != mCctMap.end())
    {
        InternalCct* actor = it->second;

        if (actor && actor->mActor)
        {

            PxRigidDynamic* dyna = static_cast<PxRigidDynamic*>(actor->mActor);

            if (updateToUsd && !dyna->isSleeping())
            {
                const PxTransform transform = dyna->getGlobalPose(); // In

                const PxQuat q = actor->mFixupQ * transform.q;

                if (!transform.isValid())
                {
                    CARB_LOG_WARN("Invalid PhysX transform detected for %s.", dyna->getName());
                }
                else
                {
                    Transform fcTransform;
                    fcTransform.position = (GfVec3f&)transform.p;
                    fcTransform.orientation = (GfQuatf&)q;
                    fcTransform.scale = (GfVec3f&)actor->mScale;

                    writeSingleTransformToUsd(*actor, currentLayer, xformCache, fcTransform);
                }
            }
        }
        it++;
    }
}

// FIXME: outputVelocitiesLocalSpace is currently unused. It comes from the exposed IPhysX API.
void InternalScene::updateRigidBodyTransforms(bool updateToUsd,
    bool updateVelocitiesToUsd,
    bool outputVelocitiesLocalSpace)
{
    // instancer support
    SdfPath currInstancerPath;
    GfMatrix4d currInstancerMatrixInverse;
    PXR_NS::UsdAttribute pos;
    PXR_NS::UsdAttribute orient;
    PXR_NS::UsdAttribute linearVel;
    PXR_NS::UsdAttribute angularVel;

    PXR_NS::VtArray<PXR_NS::GfVec3f> positionValues;
    PXR_NS::VtArray<PXR_NS::GfQuath> orientationValues;
    PXR_NS::VtArray<PXR_NS::GfVec3f> linearVelocityValues;
    PXR_NS::VtArray<PXR_NS::GfVec3f> angularVelocityValues;

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteTransforms = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);
    const bool skipWriteVelocities = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eVELOCITY | GlobalSimulationFlag::eSKIP_WRITE);
    const bool velocitiesInRad = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eVELOCITY | GlobalSimulationFlag::eNOTIFY_IN_RADIANS);
    TransformUpdateNotificationFn transformFn = cb->getTransformationWriteFn();
    VelocityUpdateNotificationFn velocityFn = cb->getVelocityWriteFn();
    void* cbUserData = cb->getUserData();
    const bool notifyTransforms = transformFn && cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eNOTIFY_UPDATE);
    const bool notifyVelocities = velocityFn && cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eVELOCITY | GlobalSimulationFlag::eNOTIFY_UPDATE);

    const bool notifyActorTransforms = cb->checkActorSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION |
        GlobalSimulationFlag::eNOTIFY_UPDATE);
    const bool notifyActorVelocities = cb->checkActorSimulationFlags(GlobalSimulationFlag::eVELOCITY |
        GlobalSimulationFlag::eNOTIFY_UPDATE);

    // skip the update loop if we should skip write, dont have notification callback request as a global
    // setting and if its not set on any actor
    if (!(skipWriteTransforms && skipWriteVelocities && !notifyTransforms && !notifyVelocities &&
        !notifyActorTransforms && !notifyActorVelocities))
    {
        InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        UsdGeomXformCache& xformCache = db.mXformCache;

        SdfLayerHandle currentLayer = OmniPhysX::getInstance().getStage()->GetEditTarget().GetLayer();

        PxU32 nbActors = 0;
        PxActor** activeActors = mScene->getActiveActors(nbActors);

        for (PxU32 i = 0; i < nbActors; i++)
        {
            const PxActor* pxActor = activeActors[i];
            const size_t recordsIndex = (size_t)pxActor->userData;

            if (!pxActor || recordsIndex >= db.getRecords().size())
                continue;

            const InternalDatabase::Record& record = db.getRecords()[recordsIndex];
            const bool isLink = record.mType == ePTLink;
            const bool isActor = record.mType == ePTActor;
            if (!(isActor || isLink))
                continue;

            InternalActor* actor = reinterpret_cast<InternalActor*>(record.mInternalPtr);

            if (actor && actor->mActor)
            {
                if (isLink && updateToUsd)
                {
                    InternalLink* intLink = static_cast<InternalLink*>(actor);
                    if (intLink->hasInboundJointWithStateAPI)
                    {
                        PxArticulationLink* link = static_cast<PxArticulationLink*>(record.mPtr);
                        PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
                        const size_t jointRecordIndex = (size_t)joint->userData;
                        if (jointRecordIndex < db.getRecords().size())
                        {
                            const InternalDatabase::Record& jointRecord = db.getRecords()[jointRecordIndex];
                            if (jointRecord.mType == ePTLinkJoint)
                            {
                                updateJointState(OmniPhysX::getInstance().getStage(), jointRecord, updateVelocitiesToUsd);
                            }
                        }
                    }
                }
                const PxRigidBody* dyna = pxActor->is<PxRigidBody>();
                if (dyna && !(dyna->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
                {
                    // if (actor->mActor->is<PxArticulationLink>() ||
                    //    (actor->mActor->is<PxRigidDynamic>() &&
                    //    !actor->mActor->is<PxRigidDynamic>()->isSleeping()))
                    {
                        bool isPointInstancer = false;
                        const PxTransform transform = dyna->getGlobalPose(); // In

                        if (notifyTransforms ||
                            (actor->mFlags & InternalActorFlag::eNOTIFY_TRANSFORM && cb->getTransformationWriteFn()))
                        {
                            transformFn(asInt(actor->mPrim.GetPrimPath()), fromPhysX(transform.p),
                                fromPhysX(transform.q), cbUserData);
                        }

                        if (!transform.isValid())
                        {
                            CARB_LOG_WARN("Invalid PhysX transform detected for %s.", dyna->getName());
                        }
                        else
                        {                         
                            if (!skipWriteTransforms && !(actor->mFlags & InternalActorFlag::eSKIP_UPDATE_TRANSFORM))
                            {
                                Transform fcTransform;
                                fcTransform.position = (GfVec3f&)transform.p;
                                fcTransform.orientation = (GfQuatf&)transform.q;
                                fcTransform.scale = (GfVec3f&)actor->mScale;

                                if (actor->mInstanceIndex == kInvalidUint32_t)
                                {
                                    if (updateToUsd)
                                    {
                                        // We should ensure that all xformOp setups are consistent during physics scene
                                        // setup. Any rigid bodies setup by PhysicsSchemaTools will have translate and
                                        // rotate ops that we can directly set here. UsdUtils::setLocalTransformMatrix
                                        // could work for prims regardless of their pre-existing xformOps, but with a
                                        // performance cost, and portability issues, we we want omni.physx to be
                                        // independent of any other Kit extensions. Anecdotally, usage of
                                        // UsdUtils::setLocalTransformMatrix seemed to also intermittently give
                                        // incorrect results with the VariousShapesOnTrimesh demo, where a couple of the
                                        // rigid bodies sometimes do not update as expected.
                                        writeSingleTransformToUsd(*actor, currentLayer, xformCache, fcTransform);
                                    }
                                }
                                else
                                {
                                    if (updateToUsd)
                                    {
                                        const PXR_NS::SdfPath& instancerPath = actor->mInstancePrim.GetPrimPath();

                                        if (instancerPath != currInstancerPath)
                                        {
                                            currInstancerMatrixInverse =
                                                xformCache.GetLocalToWorldTransform(actor->mInstancePrim).GetInverse();

                                            if (!currInstancerPath.IsEmpty())
                                            {
                                                if (!pos.Set(positionValues))
                                                {
                                                    CARB_LOG_ERROR(
                                                        "pos.Set failed on instancer: %s", instancerPath.GetText());
                                                }

                                                if (!orient.Set(orientationValues))
                                                {
                                                    CARB_LOG_ERROR(
                                                        "orient.Set failed on instancer: %s", instancerPath.GetText());
                                                }
                                            }

                                            PXR_NS::UsdGeomPointInstancer pointInstancer =
                                                PXR_NS::UsdGeomPointInstancer::Get(
                                                    actor->mInstancePrim.GetStage(), instancerPath);

                                            if (!pointInstancer)
                                            {
                                                CARB_LOG_ERROR(
                                                    "PXR_NS::UsdGeomPointInstancer::Get failed on instancer: %s",
                                                    instancerPath.GetText());
                                                continue;
                                            }

                                            pos = pointInstancer.GetPositionsAttr();

                                            if (!pos)
                                            {
                                                CARB_LOG_ERROR("GetPositionsAttr() failed on instancer: %s",
                                                               instancerPath.GetText());
                                                continue;
                                            }

                                            orient = pointInstancer.GetOrientationsAttr();

                                            if (!orient)
                                            {
                                                CARB_LOG_ERROR("GetOrientationsAttr() failed on instancer: %s",
                                                               instancerPath.GetText());
                                                continue;
                                            }

                                            if (!pos.Get(&positionValues))
                                            {
                                                if (pos.Get(&positionValues, UsdTimeCode::EarliestTime()))
                                                {
                                                    pos.Clear();
                                                    pos.Set(positionValues);
                                                }
                                                else
                                                {
                                                    CARB_LOG_ERROR(
                                                        "pos.Get() failed on instancer: %s", instancerPath.GetText());
                                                    continue;
                                                }
                                            }

                                            if (!orient.Get(&orientationValues))
                                            {
                                                if (orient.Get(&orientationValues, UsdTimeCode::EarliestTime()))
                                                {
                                                    orient.Clear();
                                                    orient.Set(orientationValues);
                                                }
                                                else
                                                {
                                                    CARB_LOG_ERROR(
                                                        "orient.Get() failed on instancer: %s", instancerPath.GetText());
                                                    continue;
                                                }
                                            }

                                            if (updateVelocitiesToUsd)
                                            {
                                                linearVel = pointInstancer.GetVelocitiesAttr();
                                                angularVel = pointInstancer.GetAngularVelocitiesAttr();


                                                if (linearVel)
                                                {
                                                    linearVel.Get(&linearVelocityValues);
                                                }

                                                if (angularVel)
                                                {

                                                    angularVel.Get(&angularVelocityValues);
                                                }
                                            }

                                            currInstancerPath = instancerPath;
                                        }

                                        isPointInstancer = true;

                                        // A.B. optimize this later, we should store the proto inverse matrices
                                        const GfMatrix4d trMatrix(
                                            GfRotation(fcTransform.orientation), GfVec3d(fcTransform.position));
                                        const GfMatrix4d writeMatrix =
                                            actor->mProtoTransformInverse * trMatrix * currInstancerMatrixInverse;

                                        uint32_t idx = actor->mInstanceIndex;
                                        // There might be more actors spawned in the PhysX scene than we had in the
                                        // initial data for the point instancer, this can for example happen if the user
                                        // spawned objects manually
                                        if (positionValues.size() <= idx)
                                            positionValues.resize(idx + 1);
                                        positionValues[idx] = PXR_NS::GfVec3f(writeMatrix.ExtractTranslation());

                                        if (orientationValues.size() <= idx)
                                            orientationValues.resize(idx + 1);
                                        orientationValues[idx] = PXR_NS::GfQuath(writeMatrix.ExtractRotation().GetQuat());
                                    }
                                }
                            }

                            // If updateVelocitiesToUsd write velocity values to the corresponding usd attribute
                            if (updateVelocitiesToUsd && isPointInstancer && updateToUsd) // For point instancer,
                                                                                          // updateToUsd is a
                                                                                          // prerequisite for
                                                                                          // updateVelocitiesToUsd
                            {
                                const PxVec3 linVel = dyna->getLinearVelocity();
                                const PxVec3 angVel = dyna->getAngularVelocity();

                                uint32_t idx = actor->mInstanceIndex;

                                if (actor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES)
                                {
                                    if (linearVel && linearVelocityValues.size() > 0)
                                    {
                                        const PxVec3 linVelTransformed = transform.q.rotateInv(linVel);
                                        const GfVec3f transformedVelocity =
                                            GfCompDiv((const GfVec3f&)linVelTransformed, (const GfVec3f&)actor->mScale);

                                        if (linearVelocityValues.size() <= idx)
                                            linearVelocityValues.resize(idx + 1);

                                        linearVelocityValues[idx] = transformedVelocity;
                                    }

                                    if (angularVel && angularVelocityValues.size() > 0)
                                    {
                                        if (angularVelocityValues.size() <= idx)
                                            angularVelocityValues.resize(idx + 1);

                                        const PxVec3 angularVelRotated = transform.q.rotateInv(angVel);
                                        angularVelocityValues[idx] = radToDeg(
                                            GfVec3f(angularVelRotated.x, angularVelRotated.y, angularVelRotated.z));
                                    }
                                }
                                else
                                {
                                    if (linearVel && linearVelocityValues.size() > 0)
                                    {
                                        if (linearVelocityValues.size() <= idx)
                                            linearVelocityValues.resize(idx + 1);

                                        linearVelocityValues[idx] = GfVec3f(linVel.x, linVel.y, linVel.z);
                                    }

                                    if (angularVel && angularVelocityValues.size() > 0)
                                    {
                                        if (angularVelocityValues.size() <= idx)
                                            angularVelocityValues.resize(idx + 1);

                                        angularVelocityValues[idx] = radToDeg(GfVec3f(angVel.x, angVel.y, angVel.z));
                                    }
                                }
                            }
                            else
                            {
                                if (notifyVelocities ||
                                    (actor->mFlags & InternalActorFlag::eNOTIFY_VELOCITY && velocityFn))
                                {
                                    const PxVec3 linVel = dyna->getLinearVelocity();
                                    const PxVec3 angVel = actor->mFlags & InternalActorFlag::eNOTIFY_VELOCITY_RADIANS ?
                                                              dyna->getAngularVelocity() :
                                                              radToDeg(dyna->getAngularVelocity());

                                    velocityFn(asInt(actor->mPrim.GetPrimPath()), fromPhysX(linVel), fromPhysX(angVel),
                                               cbUserData);
                                }

                                if (updateVelocitiesToUsd && !skipWriteVelocities &&
                                    !(actor->mFlags & InternalActorFlag::eSKIP_UPDATE_VELOCITY))
                                {
                                    SdfAttributeSpecHandle velSdfAttr = currentLayer->GetAttributeAtPath(actor->mVelocityPath);
                                    const PxVec3 linVel = dyna->getLinearVelocity();
                                    GfVec3f transformedVelocity;

                                    if (actor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES)
                                    {
                                        const PxVec3 linVelTransformed = transform.q.rotateInv(linVel);
                                        transformedVelocity = GfCompDiv(
                                            (const GfVec3f&)linVelTransformed, (const GfVec3f&)actor->mScale);
                                    }
                                    else
                                    {
                                        transformedVelocity = GfVec3f(linVel.x, linVel.y, linVel.z);
                                    }

                                    if (velSdfAttr)
                                    {
                                        velSdfAttr->SetDefaultValue(VtValue(transformedVelocity));
                                    }
                                    else if (actor->mPrim)
                                    {
                                        UsdAttribute velAttr =
                                            actor->mPrim.GetAttribute(UsdPhysicsTokens.Get()->physicsVelocity);
                                        velAttr.Set(transformedVelocity);
                                    }

                                    SdfAttributeSpecHandle angSdfVelAttr = currentLayer->GetAttributeAtPath(actor->mAngularVelocityPath);
                                    const PxVec3 angVel = dyna->getAngularVelocity();
                                    GfVec3f transformedAngularVelocity;

                                    if (actor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES)
                                    {
                                        const PxVec3 angularVelRotated = transform.q.rotateInv(angVel);
                                        transformedAngularVelocity =
                                            GfVec3f(angularVelRotated.x, angularVelRotated.y, angularVelRotated.z);
                                    }
                                    else
                                    {
                                        transformedAngularVelocity = GfVec3f(angVel.x, angVel.y, angVel.z);
                                    }

                                    if (angSdfVelAttr)
                                    {
                                        angSdfVelAttr->SetDefaultValue(VtValue(radToDeg(transformedAngularVelocity)));
                                    }
                                    else if (actor->mPrim)
                                    {
                                        UsdAttribute velAngAttr =
                                            actor->mPrim.GetAttribute(UsdPhysicsTokens.Get()->physicsAngularVelocity);
                                        velAngAttr.Set(transformedAngularVelocity);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (!currInstancerPath.IsEmpty())
    {
        if (!pos.Set(positionValues))
        {
            CARB_LOG_ERROR("pos.Set() failed on instancer: %s", currInstancerPath.GetText());
        }

        if (!orient.Set(orientationValues))
        {
            CARB_LOG_ERROR("orient.Set() failed on instancer: %s", currInstancerPath.GetText());
        }

        if (updateVelocitiesToUsd)
        {
            if (linearVel)
            {
                linearVel.Set(linearVelocityValues);
            }

            if (angularVel)
            {
                angularVel.Set(angularVelocityValues);
            }
        }
    }
}

void InternalScene::updateParticleTransforms(bool updateToUsd, bool updateVelocitiesToUsd, bool updateParticlesToUsd)
{
    // 0 is disabled, 1/2 is selected/all
    const bool debugVizEnabled = (OmniPhysX::getInstance().getCachedSettings().visualizationDisplayParticles > 0);
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteTransforms = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);

    for (size_t particleSystemIndex = 0; particleSystemIndex < mParticleSystems.size(); particleSystemIndex++)
    {
        InternalPbdParticleSystem* particleSystem = mParticleSystems[particleSystemIndex];

        uint32_t postFlags = particles::getPostprocessStages(particleSystem->mPath);
        bool particleSystemHasAnisotropy = postFlags & ParticlePostFlag::eAnisotropy;
        bool particleSystemHasSmoothing = postFlags & ParticlePostFlag::eSmoothing;
        bool particleSystemHasIsosurface = postFlags & ParticlePostFlag::eIsosurface;

        // sanity check
        if (!particleSystem->mEnabled || (!updateToUsd && !updateParticlesToUsd))
            continue;

        if (!particleSystem->mParticleDataAvailable)
        {
            continue;
        }
        else
        {
            particleSystem->mParticleDataAvailable = false;
        }
        
        // in case of async sim, we need to sync earlier. See PhysXScene.cpp -> PhysXStepper::run()
        if (!particleSystem->mAsyncSim)
        {
            PxScene* scene = particleSystem->mPS->getScene();
            if (scene)
            {
                scene->fetchResultsParticleSystem();
            }
        }
        particleSystem->mAsyncSim = false;

        PxVec4* anisotropyQ1 = nullptr;
        PxVec4* anisotropyQ2 = nullptr;
        PxVec4* anisotropyQ3 = nullptr;
        if (particleSystemHasAnisotropy)
        {
            particles::getAnisotropy(anisotropyQ1, anisotropyQ2, anisotropyQ3, particleSystem->mPath);
        }

        PxVec4* smoothedPos = nullptr;
        if (particleSystemHasSmoothing)
        {
            smoothedPos = particles::getSmoothedPositions(particleSystem->mPath);
        } 


        // TODO preallocate/resize - this explicitly assumes 0 to clear if there are no diffuse particles
        VtArray<GfVec3f> tmpDiffuseParticlePoints(0);
        VtArray<GfVec3f> tmpDiffuseParticleColors(0);
        PxU32 tmpDiffuseParticleCount = 0;

        /* walk through particle sets */
        for (InternalParticleSet* particleSet : particleSystem->mParticleSets)
        {
            if (!particleSet->mNumParticles || !particleSet->mEnabled || !particleSet->mDownloadDirtyFlags)
                continue;

#if ENABLE_FABRIC_FOR_PARTICLE_SETS
            if (particleSet->mFabric)
                continue;
#endif

            // transform particles from world space back to prim local space
            UsdGeomXform xform(particleSet->mPrim);
            GfMatrix4f worldToLocal = GfMatrix4f(xform.ComputeLocalToWorldTransform(UsdTimeCode::Default()).GetInverse());

            uint32_t flags = particleSet->mDownloadDirtyFlags;

            UsdGeomPointBased pointBased{ particleSet->mPrim };
            UsdGeomPointInstancer pointInstancer{ particleSet->mPrim };

            // AD: we use the raw dirty flags here because particles only work with USD. The logic that decides
            // these flags is in InternalParticleSet::fetchParticles(). In the future, there will likely be a need
            // for updateToUsd based select logic here, and the flags should only be used to signal what data has been
            // DMAd from the GPU.

            if (flags & ParticleDirtyFlags::eVELOCITY)
            {
                UsdAttribute velocityAttr = pointBased ? pointBased.CreateVelocitiesAttr() : pointInstancer.CreateVelocitiesAttr();
                VtArray<GfVec3f> tmpVelocities;
                copyBuffer(tmpVelocities, (const carb::Float4*)particleSet->mVelocities, particleSet->mNumParticles);
                velocityAttr.Set(tmpVelocities);
            }

            if (flags & ParticleDirtyFlags::ePOSITION_INVMASS)
            {
                // means we downloaded positions - so if we also downloaded the smoothed pos, we write the positions into the simPos attribute.
                if (flags & ParticleDirtyFlags::eSMOOTHED_POSITIONS)
                {
                    VtArray<GfVec3f> tmpSimPositions;
                    copyBuffer(tmpSimPositions, particleSet->mPositions, particleSet->mNumParticles, worldToLocal);
                    PhysxSchemaPhysxParticleSetAPI particleSetApi(particleSet->mPrim);
                    particleSetApi.CreateSimulationPointsAttr().Set(tmpSimPositions);
                }
                else
                {
                    UsdAttribute pointsAttr = pointBased ? pointBased.CreatePointsAttr() : pointInstancer.CreatePositionsAttr();
                    VtArray<GfVec3f> tmpPoints;
                    copyBuffer(tmpPoints, particleSet->mPositions, particleSet->mNumParticles, worldToLocal);
                    pointsAttr.Set(tmpPoints);
                }
            }

            if ((flags & ParticleDirtyFlags::eSMOOTHED_POSITIONS) && smoothedPos)
            {
                PxU32 start = particleSet->mParticleBuffer->getFlatListStartIndex();
                UsdAttribute pointsAttr = pointBased ? pointBased.CreatePointsAttr() : pointInstancer.CreatePositionsAttr();
                VtArray<GfVec3f> tmpPoints;
                copyBuffer(tmpPoints, &smoothedPos[start], particleSet->mNumParticles, worldToLocal);
                pointsAttr.Set(tmpPoints);
            }

            if ((flags & ParticleDirtyFlags::eANISOTROPY) && anisotropyQ1 && anisotropyQ2 && anisotropyQ3)
            {
                PxU32 start = particleSet->mParticleBuffer->getFlatListStartIndex();
                if (pointBased)
                {
                    VtArray<GfVec4f> tmpValuesQ1(particleSet->mNumParticles);
                    VtArray<GfVec4f> tmpValuesQ2(particleSet->mNumParticles);
                    VtArray<GfVec4f> tmpValuesQ3(particleSet->mNumParticles);

                    UsdGeomPrimvarsAPI primVarsAPI(particleSet->mPrim);
                    UsdAttribute q1 = primVarsAPI.GetPrimvar(TfToken("anisotropyQ1"));
                    UsdAttribute q2 = primVarsAPI.GetPrimvar(TfToken("anisotropyQ2"));
                    UsdAttribute q3 = primVarsAPI.GetPrimvar(TfToken("anisotropyQ3"));

                    copyBuffer(tmpValuesQ1, &anisotropyQ1[start], particleSet->mNumParticles);
                    copyBuffer(tmpValuesQ2, &anisotropyQ2[start], particleSet->mNumParticles);
                    copyBuffer(tmpValuesQ3, &anisotropyQ3[start], particleSet->mNumParticles);

                    q1.Set(tmpValuesQ1);
                    q2.Set(tmpValuesQ2);
                    q3.Set(tmpValuesQ3);
                }
                else
                {
                    float contactDistanceInv = 1.0f / (particleSystem->mPS->getParticleContactOffset() * 2.0f);
                    VtArray<GfVec3f> tmpScales(particleSet->mNumParticles);
                    VtArray<GfQuath> tmpOrientations(particleSet->mNumParticles);

                    for (PxU32 i = start; i < start + particleSet->mNumParticles; i++)
                    {
                        PxVec4 q1 = anisotropyQ1[i];
                        PxVec4 q2 = anisotropyQ2[i];
                        PxVec4 q3 = anisotropyQ3[i];
                        tmpScales[i - start] = { 4.0f * q1[3] * contactDistanceInv, 4.0f * q2[3] * contactDistanceInv, 4.0f * q3[3] * contactDistanceInv };
                        PxQuat q = PxQuat(PxMat33(q1.getXYZ(), q2.getXYZ(), q3.getXYZ()));
                        tmpOrientations[i - start] = GfQuath(q.w, q.x, q.y, q.z);
                    }

                    pointInstancer.CreateScalesAttr().Set(tmpScales);
                    pointInstancer.CreateOrientationsAttr().Set(tmpOrientations);
                }
            }

            // accumulate diffuse particles
            if (flags & ParticleDirtyFlags::eDIFFUSE_PARTICLES)
            {
                size_t currentSize = tmpDiffuseParticlePoints.size();
                tmpDiffuseParticlePoints.resize(currentSize + particleSet->mNumDiffuseParticles);
                tmpDiffuseParticleColors.resize(currentSize + particleSet->mNumDiffuseParticles);

                for (PxU32 i = 0; i < particleSet->mNumDiffuseParticles; ++i)
                {
                    const PxVec4& p = particleSet->mDiffuseParticlePositions[i];
                    tmpDiffuseParticlePoints[currentSize + i] = GfVec3f(p.x, p.y, p.z);
                    tmpDiffuseParticleColors[currentSize + i] = GfVec3f(1.0f, 1.0f, 1.0f);
                }

                tmpDiffuseParticleCount += particleSet->mNumDiffuseParticles;
            }

            particleSet->mDownloadDirtyFlags = 0;
        }

        if (particleSystem->mDiffuseParticleInstance)
        {
            InternalPointCloud* instance = particleSystem->mDiffuseParticleInstance;
            ScopedLayerEdit scopedSessionLayerEdit(stage, stage->GetSessionLayer());

            instance->mGeo.CreatePointsAttr().Set(tmpDiffuseParticlePoints);
            instance->mGeo.CreateDisplayColorPrimvar().Set(tmpDiffuseParticleColors);
        }

        /* Isosurface */
        if ((updateToUsd || debugVizEnabled) && particleSystemHasIsosurface)
        {
            particles::updateIsosurfaceMesh(particleSystem->mPath);
        }

        // AD: move this up once all the particles are supported by fabric.
        if (skipWriteTransforms) // cloth has fabric, we can skip from here
            continue;

        for (size_t clothIndex = 0; clothIndex < particleSystem->mCloths.size(); clothIndex++)
        {
            InternalParticleClothDeprecated* cloth = particleSystem->mCloths[clothIndex];

            if (!cloth->mEnabled || !cloth->mDownloadDirtyFlags)
                continue;

            // transform particles from world space back to prim local space
            UsdGeomXform xform(cloth->mPrim);
            GfMatrix4f localToWorldOld = GfMatrix4f(xform.ComputeLocalToWorldTransform(UsdTimeCode::Default()));
            GfMatrix4f parentToWorld = GfMatrix4f(xform.ComputeParentToWorldTransform(UsdTimeCode::Default()));

            // AD this assumes 1 volume per cloth - which is fine for now. Consider future-proofing this
#if USE_PHYSX_GPU
            ExtGpu::PxParticleVolumeBufferHelper& volumeBuffers = *cloth->mVolumeBuffer;
            const PxParticleVolume* volumes = volumeBuffers.getParticleVolumes();
            const PxBounds3& bounds = volumes->bound;
            PxVec3 center = bounds.getCenter();
#else
            PxVec3 center = PxVec3(0.0f);
#endif

            GfMatrix4f localToWorldNew = localToWorldOld;
            localToWorldNew.SetTranslateOnly(GfVec3f(center.x, center.y, center.z));

            cloth->mLocalToWorld = GfMatrix4d(localToWorldNew);
            GfMatrix4f worldToLocalNew = localToWorldNew.GetInverse();
            GfMatrix4f localToParent = GfMatrix4f(localToWorldNew * parentToWorld.GetInverse());

            // AD: we use the raw dirty flags here because particles only work with USD. The logic that decides
            // these flags is in InternalParticleClothDeprecated::fetchParticles(). In the future, there will likely be a need
            // for updateToUsd based select logic here, and the flags should only be used to signal what data has been
            // DMAd from the GPU.

            if ((cloth->mDownloadDirtyFlags & ParticleDirtyFlags::ePOSITION_INVMASS) && !cloth->mSkipUpdateTransform)
            {
                setPrimXformOps(cloth->mPrim, GfMatrix4d(localToParent), true);

                VtArray<GfVec3f> tmpPoints;
                if (cloth->mIsWelded)
                {
                    VtArray<GfVec3f> weldedPoints;
                    copyBuffer(weldedPoints, cloth->mPositions, cloth->mNumParticles, worldToLocalNew);

                    //initialize from usd mesh, as non-referenced vertices are not mapped
                    cloth->mGeo.GetPointsAttr().Get(&tmpPoints);
                    const size_t nbMapped = PxMin(cloth->mVerticesRemapToWeld.size(), tmpPoints.size());
                    for (size_t i = 0; i < nbMapped; ++i)
                    {
                        const uint32_t index = cloth->mVerticesRemapToWeld[i];
                        if (index != 0xffffffff) // non referenced vertices are not simulated
                        {
                            tmpPoints[i] = weldedPoints[index];
                        }
                    }
                }
                else
                {
                    copyBuffer(tmpPoints, cloth->mPositions, cloth->mNumParticles, worldToLocalNew);
                }
                cloth->mGeo.GetPointsAttr().Set(tmpPoints);
#if USE_PHYSX_GPU

                GfBBox3d bbox(GfRange3d(toVec3d(bounds.minimum), toVec3d(bounds.maximum)));
                bbox.Transform(GfMatrix4d(worldToLocalNew));
                GfRange3d transformedBounds = bbox.ComputeAlignedBox();

                VtArray<GfVec3f> extent(2);
                extent[0] = GfVec3f(transformedBounds.GetMin());
                extent[1] = GfVec3f(transformedBounds.GetMax());

                cloth->mGeo.CreateExtentAttr().Set(extent);
#endif
            }

            if (cloth->mDownloadDirtyFlags & ParticleDirtyFlags::eVELOCITY)
            {
                VtArray<GfVec3f> tmpVelocities;
                if (cloth->mIsWelded)
                {
                    VtArray<GfVec3f> weldedVelocities;
                    copyBuffer(weldedVelocities, cloth->mVelocities, cloth->mNumParticles);

                    //initialize from usd mesh, as non-referenced vertices are not mapped
                    cloth->mGeo.GetVelocitiesAttr().Get(&tmpVelocities);
                    const size_t nbMapped = PxMin(cloth->mVerticesRemapToWeld.size(), tmpVelocities.size());
                    for (size_t i = 0; i < nbMapped; ++i)
                    {
                        const uint32_t index = cloth->mVerticesRemapToWeld[i];
                        if (index != 0xffffffff) // non referenced vertices are not simulated
                        {
                            tmpVelocities[i] = weldedVelocities[index];
                        }
                    }
                }
                else
                {
                    copyBuffer(tmpVelocities, cloth->mVelocities, cloth->mNumParticles);
                }
                cloth->mGeo.GetVelocitiesAttr().Set(tmpVelocities); 
            }

            cloth->mDownloadDirtyFlags = 0;
        }
    }
}

void InternalScene::updateDeformableTransforms(bool updateToUsd, bool updateVelocitiesToUsd)
{
    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteTransforms = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);
    if (skipWriteTransforms)
        return;

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);

    // TODO adenzler use event
    // TODO be careful about race conditions?
    syncDeformableCopyStream(cudaContextManager);

    // DEPRECATED
    for (size_t i = 0; i < mDeformableBodiesDeprecated.size(); i++)
    {
        InternalDeformableBodyDeprecated* deformableBody = mDeformableBodiesDeprecated[i];

        // update prim transform (using physx bounds center)
        // transform vertices from world space back to prim local space

        GfMatrix4f deformableBodyToWorldOld = GfMatrix4f(UsdGeomMesh(deformableBody->mPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default()));

        const PxBounds3 bounds = deformableBody->mSoftBody->getWorldBounds();
        const PxVec3 center = bounds.getCenter();

        GfMatrix4f deformableBodyToWorldNew = deformableBodyToWorldOld;

        if (!deformableBody->mIsPartiallyKinematic)
        {
            deformableBodyToWorldNew.SetTranslateOnly(GfVec3f(center.x, center.y, center.z));
            deformableBody->mPrimToWorld = GfMatrix4d(deformableBodyToWorldNew);
        }

        if (updateToUsd && !deformableBody->mSkipUpdateTransform)
        {
            GfMatrix4f worldToDeformableBodyNew = deformableBodyToWorldNew.GetInverse();
            GfMatrix4f deformableBodyParentToWorld = GfMatrix4f(UsdGeomXform(deformableBody->mPrim).ComputeParentToWorldTransform(UsdTimeCode::Default()));
            GfMatrix4f deformableBodyToParent = GfMatrix4f(deformableBody->mPrimToWorld) * deformableBodyParentToWorld.GetInverse();
            setPrimXformOps(deformableBody->mPrim, GfMatrix4d(deformableBodyToParent), true);

            const ::physx::PxVec4* src = deformableBody->mCollPositionInvMassH;

            if (!deformableBody->mIsPartiallyKinematic)
            {
                VtArray<GfVec3f> tmpSkinMeshPositions;
                copyBuffer(tmpSkinMeshPositions, src + deformableBody->mNumCollMeshVertices, deformableBody->mNumSkinMeshVertices, worldToDeformableBodyNew);
                UsdGeomMesh mesh(deformableBody->mPrim);
                mesh.GetPointsAttr().Set(tmpSkinMeshPositions);

                GfBBox3d bbox(GfRange3d(toVec3d(bounds.minimum), toVec3d(bounds.maximum)));
                bbox.Transform(GfMatrix4d(worldToDeformableBodyNew));
                GfRange3d transformedBounds = bbox.ComputeAlignedBox();

                VtArray<GfVec3f> extent(2);
                extent[0] = GfVec3f(transformedBounds.GetMin());
                extent[1] = GfVec3f(transformedBounds.GetMax());

                mesh.CreateExtentAttr().Set(extent);
            }
            VtArray<GfVec3f> tmpCollMeshPositions;
            copyBuffer(tmpCollMeshPositions, src, deformableBody->mNumCollMeshVertices, worldToDeformableBodyNew);
            PhysxSchemaPhysxDeformableBodyAPI(deformableBody->mPrim).GetCollisionPointsAttr().Set(tmpCollMeshPositions);

            VtArray<GfVec3f> tmpSimMeshPositions;
            copyBuffer(tmpSimMeshPositions, deformableBody->mSimPositionInvMassH, deformableBody->mNumSimMeshVertices, worldToDeformableBodyNew);
            PhysxSchemaPhysxDeformableBodyAPI(deformableBody->mPrim).GetSimulationPointsAttr().Set(tmpSimMeshPositions);

            if (updateVelocitiesToUsd)
            {
                VtArray<GfVec3f> tmpSimMeshVelocities;
                copyBuffer(tmpSimMeshVelocities, deformableBody->mSimVelocityH, deformableBody->mNumSimMeshVertices);
                PhysxSchemaPhysxDeformableAPI(deformableBody->mPrim).GetSimulationVelocitiesAttr().Set(tmpSimMeshVelocities);
            }
        }
    }

    // DEPRECATED
    for (size_t i = 0; i < mDeformableSurfacesDeprecated.size(); i++)
    {
        InternalDeformableSurfaceDeprecated* deformableSurface = mDeformableSurfacesDeprecated[i];

        // update prim transform (using physx bounds center)
        // transform vertices from world space back to prim local space

        GfMatrix4f deformableSurfaceToWorldOld = GfMatrix4f(UsdGeomMesh(deformableSurface->mPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default()));

        const PxBounds3 bounds = deformableSurface->mDeformableSurface->getWorldBounds();
        const PxVec3 center = bounds.getCenter();

        GfMatrix4f deformableSurfaceToWorldNew = deformableSurfaceToWorldOld;

        deformableSurfaceToWorldNew.SetTranslateOnly(GfVec3f(center.x, center.y, center.z));
        deformableSurface->mPrimToWorld = GfMatrix4d(deformableSurfaceToWorldNew);

        if (updateToUsd && !deformableSurface->mSkipUpdateTransform)
        {
            GfMatrix4f worldToCollisionNew = deformableSurfaceToWorldNew.GetInverse();
            GfMatrix4f deformableSurfaceParentToWorld = GfMatrix4f(UsdGeomXform(deformableSurface->mPrim).ComputeParentToWorldTransform(UsdTimeCode::Default()));
            GfMatrix4f deformableSurfaceToParent = GfMatrix4f(deformableSurface->mPrimToWorld) * deformableSurfaceParentToWorld.GetInverse();
            setPrimXformOps(deformableSurface->mPrim, GfMatrix4d(deformableSurfaceToParent), true);

            VtArray<GfVec3f> tmpCollMeshPositions;
            copyBuffer<::physx::PxVec4>(tmpCollMeshPositions, deformableSurface->mPositionInvMassH, deformableSurface->mNumCollMeshVerticesOrig, worldToCollisionNew, deformableSurface->mPhysxToUsdVtxMap);

            UsdGeomMesh mesh(deformableSurface->mPrim);
            mesh.GetPointsAttr().Set(tmpCollMeshPositions);

            GfBBox3d bbox(GfRange3d(toVec3d(bounds.minimum), toVec3d(bounds.maximum)));
            bbox.Transform(GfMatrix4d(worldToCollisionNew));
            GfRange3d transformedBounds = bbox.ComputeAlignedBox();

            VtArray<GfVec3f> extent(2);
            extent[0] = GfVec3f(transformedBounds.GetMin());
            extent[1] = GfVec3f(transformedBounds.GetMax());

            mesh.CreateExtentAttr().Set(extent);

            if (updateVelocitiesToUsd)
            {
                VtArray<GfVec3f> tmpCollMeshVelocities;
                copyBuffer<::physx::PxVec4>(tmpCollMeshVelocities, deformableSurface->mVelocityH, deformableSurface->mNumCollMeshVerticesOrig, deformableSurface->mPhysxToUsdVtxMap);
                PhysxSchemaPhysxDeformableAPI(deformableSurface->mPrim).CreateSimulationVelocitiesAttr().Set(tmpCollMeshVelocities);
            }
        }
    }

    // Need to synchronize before updating skin meshes
    if (mVolumeDeformablePostSolveCallback)
        mVolumeDeformablePostSolveCallback->synchronize();

    if (mSurfaceDeformablePostSolveCallback)
        mSurfaceDeformablePostSolveCallback->synchronize();

    for (size_t i = 0; i < mVolumeDeformableBodies.size(); i++)
    {
        InternalVolumeDeformableBody* deformableBody = mVolumeDeformableBodies[i];
        if (!updateToUsd)
        {
            continue;
        }

        // transform vertices from world space back to prim local space
        // transforms remain unchanged

        // update skin meshes
        if (mVolumeDeformablePostSolveCallback && deformableBody->mNumSkinMeshVertices > 0)
        {
            const ::physx::PxVec3* srcPtr = deformableBody->mAllSkinnedVerticesH;
            for (size_t i = 0; i < deformableBody->mSkinMeshPrims.size(); ++i)
            {
                UsdGeomPointBased skin(deformableBody->mSkinMeshPrims[i]);
                if (skin)
                {
                    const Uint2& range = deformableBody->mSkinMeshRanges[i];
                    const GfMatrix4f& worldToSkinMesh = deformableBody->mWorldToSkinMeshTransforms[i];
                    VtArray<GfVec3f> points;
                    skin.GetPointsAttr().Get(&points);
                    if (points.size() == range.y)
                    {
                        copyBuffer(points, srcPtr + range.x, range.y, worldToSkinMesh);
                        skin.GetPointsAttr().Set(points);
                    }
                }
            }
        }

        {
            UsdGeomPointBased simGeom(deformableBody->mSimMeshPrim);
            if (simGeom)
            {
                const GfMatrix4f& worldToSimMesh = deformableBody->mWorldToSimMesh;
                VtArray<GfVec3f> points;
                {
                    const ::physx::PxVec4* srcPtr = deformableBody->mSimMeshPositionInvMassH;
                    const uint32_t srcSize = deformableBody->mNumSimMeshVertices;
                    simGeom.GetPointsAttr().Get(&points);
                    if (points.size() == srcSize)
                    {
                        copyBuffer(points, srcPtr, srcSize, worldToSimMesh);
                        simGeom.GetPointsAttr().Set(points);
                    }
                }

                if (updateVelocitiesToUsd)
                {
                    const ::physx::PxVec4* srcPtr = deformableBody->mSimMeshVelocityH;
                    const uint32_t srcSize = deformableBody->mNumSimMeshVertices;
                    if (points.size() == srcSize)
                    {
                        VtArray<GfVec3f> velocities;
                        copyBuffer(velocities, srcPtr, srcSize);
                        simGeom.GetVelocitiesAttr().Set(velocities);
                    }
                }
            }

            UsdGeomPointBased collGeom(deformableBody->mCollMeshPrim);
            if (collGeom)
            {
                const GfMatrix4f& worldToCollMesh = deformableBody->mWorldToCollMesh;
                if (collGeom.GetPrim() != simGeom.GetPrim())
                {
                    const ::physx::PxVec4* srcPtr = deformableBody->mCollMeshPositionInvMassH;
                    const uint32_t srcSize = deformableBody->mNumCollMeshVertices;
                    VtArray<GfVec3f> points;
                    collGeom.GetPointsAttr().Get(&points);
                    if (points.size() == srcSize)
                    {
                        copyBuffer(points, srcPtr, srcSize, worldToCollMesh);
                        collGeom.GetPointsAttr().Set(points);
                    }
                }

                const PxBounds3 worldBounds = deformableBody->mDeformableVolume->getWorldBounds();
                GfBBox3d bbox(GfRange3d(toVec3d(worldBounds.minimum), toVec3d(worldBounds.maximum)));
                bbox.Transform(GfMatrix4d(worldToCollMesh));
                GfRange3d transformedBounds = bbox.ComputeAlignedBox();
                VtArray<GfVec3f> extent(2);
                extent[0] = GfVec3f(transformedBounds.GetMin());
                extent[1] = GfVec3f(transformedBounds.GetMax());
                collGeom.CreateExtentAttr().Set(extent);
            }
        }
    }

    for (size_t i = 0; i < mSurfaceDeformableBodies.size(); i++)
    {
        InternalSurfaceDeformableBody* deformableBody = mSurfaceDeformableBodies[i];
        if (!updateToUsd)
        {
            continue;
        }

        // transform vertices from world space back to prim local space
        // transforms remain unchanged

        // update skin meshes
        if (mSurfaceDeformablePostSolveCallback && deformableBody->mNumSkinMeshVertices > 0)
        {
            const ::physx::PxVec3* srcPtr = deformableBody->mAllSkinnedVerticesH;
            for (size_t i = 0; i < deformableBody->mSkinMeshPrims.size(); ++i)
            {
                UsdGeomPointBased skin(deformableBody->mSkinMeshPrims[i]);
                if (skin)
                {
                    const Uint2& range = deformableBody->mSkinMeshRanges[i];
                    const GfMatrix4f& worldToSkinMesh = deformableBody->mWorldToSkinMeshTransforms[i];
                    VtArray<GfVec3f> points;
                    skin.GetPointsAttr().Get(&points);
                    if (points.size() == range.y)
                    {
                        copyBuffer(points, srcPtr + range.x, range.y, worldToSkinMesh);
                        skin.GetPointsAttr().Set(points);
                    }
                }
            }
        }

        {
            UsdGeomMesh simMesh(deformableBody->mSimMeshPrim);
            if (simMesh)
            {
                const GfMatrix4f& worldToSimMesh = deformableBody->mWorldToSimMesh;
                VtArray<GfVec3f> points;
                {
                    const ::physx::PxVec4* srcPtr = deformableBody->mSimMeshPositionInvMassH;
                    const uint32_t srcSize = deformableBody->mNumSimMeshVertices;
                    
                    simMesh.GetPointsAttr().Get(&points);
                    if (points.size() == srcSize)
                    {
                        copyBuffer(points, srcPtr, uint32_t(points.size()), worldToSimMesh);
                    }
                    simMesh.GetPointsAttr().Set(points);
                }

                if (updateVelocitiesToUsd)
                {
                    const ::physx::PxVec4* srcPtr = deformableBody->mSimMeshVelocityH;
                    const uint32_t srcSize = deformableBody->mNumSimMeshVertices;
                    if (points.size() == srcSize)
                    {
                        VtArray<GfVec3f> velocities;
                        copyBuffer(velocities, srcPtr, srcSize);
                        simMesh.GetVelocitiesAttr().Set(velocities);
                    }
                }

                {
                    const PxBounds3 worldBounds = deformableBody->mDeformableSurface->getWorldBounds();
                    GfBBox3d bbox(GfRange3d(toVec3d(worldBounds.minimum), toVec3d(worldBounds.maximum)));
                    bbox.Transform(GfMatrix4d(worldToSimMesh));
                    GfRange3d transformedBounds = bbox.ComputeAlignedBox();
                    VtArray<GfVec3f> extent(2);
                    extent[0] = GfVec3f(transformedBounds.GetMin());
                    extent[1] = GfVec3f(transformedBounds.GetMax());
                    simMesh.CreateExtentAttr().Set(extent);
                }
            }
        }
    }
}

void InternalScene::updateJointState(UsdStageWeakPtr stage, const InternalDatabase::Record& record, bool updateVelocitiesToUsd)
{
    InternalJoint* intJoint = (InternalJoint*)record.mInternalPtr;
    ::physx::PxArticulationJointReducedCoordinate* joint = (::physx::PxArticulationJointReducedCoordinate*)record.mPtr;

    for (size_t idx = 0; idx < 6; ++idx)
    {
        InternalJoint::InternalJointState& intJointState = intJoint->mJointStates[idx];
        if (!intJointState.enabled)
            continue;
        PhysxSchemaJointStateAPI cachedJointStateAPI =
            intJointState.getCachedJointStateAPI(stage->GetPrimAtPath(record.mPath), intJoint->mJointType);
        UsdAttribute posAttribute = cachedJointStateAPI.GetPositionAttr();
        if (posAttribute)
        {
            const float articulationPos = intJoint->getArticulationJointPosition(joint, intJointState.physxAxis);
            const float positionValue = intJointState.convertToDegrees ? radToDeg(articulationPos) : articulationPos;
            posAttribute.Set(positionValue);
        }
        if (updateVelocitiesToUsd)
        {
            UsdAttribute velAttribute = cachedJointStateAPI.GetVelocityAttr();
            if (velAttribute)
            {
                const float articulationVel = intJoint->getArticulationJointVelocity(joint, intJointState.physxAxis);
                const float velocityValue = intJointState.convertToDegrees ? radToDeg(articulationVel) : articulationVel;
                velAttribute.Set(velocityValue);
            }
        }
    }
}

void InternalScene::updateResiduals(bool updateToUsd)
{
    if (!(mScene->getFlags() & PxSceneFlag::eENABLE_SOLVER_RESIDUAL_REPORTING))
        return;

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteResiduals = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eRESIDUALS | GlobalSimulationFlag::eSKIP_WRITE);
    void* cbUserData = cb->getUserData();
    ResidualUpdateNotificationFn residualFn = cb->getResidualWriteFn();
    const bool notifyResiduals = residualFn && cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eRESIDUALS | GlobalSimulationFlag::eNOTIFY_UPDATE);

    if (notifyResiduals || residualFn || (updateToUsd && !skipWriteResiduals))
    {
        const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        const std::vector<InternalPhysXDatabase::Record>& records = db.getRecords();

        PxArticulationReducedCoordinate* articulation;
        const PxU32 nbArticulations = mScene->getNbArticulations();        

        SdfLayerHandle currentLayer = OmniPhysX::getInstance().getStage()->GetEditTarget().GetLayer();

        for (PxU32 i = 0; i < nbArticulations; ++i)
        {
            mScene->getArticulations(&articulation, 1, i);

            const size_t recordIndex = (size_t)articulation->userData;
            if (recordIndex < db.getRecords().size())
            {
                const InternalDatabase::Record& record = db.getRecords()[recordIndex];
                if (record.mType == ePTArticulation && record.mInternalPtr && ((InternalArticulation*)record.mInternalPtr)->mReportResiduals)
                {
                    const PxResiduals residuals = articulation->getSolverResidual();                    
                    updateResidualsImpl(updateToUsd, skipWriteResiduals, notifyResiduals, residualFn, record.mPath, residuals, currentLayer, cbUserData);
                }
            }
        }

        {
            const size_t recordIndex = (size_t)mScene->userData;
            if (recordIndex < db.getRecords().size())
            {
                const InternalDatabase::Record& record = db.getRecords()[recordIndex];
                if (record.mType == ePTScene  && record.mInternalPtr && ((InternalScene*)record.mInternalPtr)->mReportResiduals)
                {
                    const PxResiduals residuals = mScene->getSolverResidual();
                    updateResidualsImpl(updateToUsd, skipWriteResiduals, notifyResiduals, residualFn, record.mPath, residuals, currentLayer, cbUserData);
                }
            }
        }
    }
}

void InternalScene::updateSimulationOutputs(bool updateToUsd,
                                           bool updateVelocitiesToUsd,
                                           bool outputVelocitiesLocalSpace,
                                           bool updateParticlesToUsd,
                                           bool updateResidualsToUsd)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    ScopedNoticeBlock scopedNoticeBlock;

    if (omniPhysX.getSimulationLayer())
    {
        UsdStageWeakPtr stage = omniPhysX.getStage();
        UsdEditContext editContext(stage, UsdEditTarget(omniPhysX.getSimulationLayer()));

        PXR_NS::SdfChangeBlock changeBlock;

        {
            CARB_PROFILE_ZONE(0, "updateRenderTransforms::USDWrite");
            updateRigidBodyTransforms(updateToUsd, updateVelocitiesToUsd, outputVelocitiesLocalSpace);
            updateResiduals(updateResidualsToUsd);
            updateCctTransforms(updateToUsd);
            updateVehicleTransforms(updateToUsd);
            updateParticleTransforms(updateToUsd, updateVelocitiesToUsd, updateParticlesToUsd);
            updateDeformableTransforms(updateToUsd, updateVelocitiesToUsd);
        }
    }
    else
    {
        PXR_NS::SdfChangeBlock changeBlock;

        {
            CARB_PROFILE_ZONE(0, "updateRenderTransforms::USDWrite");
            updateRigidBodyTransforms(updateToUsd, updateVelocitiesToUsd, outputVelocitiesLocalSpace);
            updateResiduals(updateResidualsToUsd);
            updateCctTransforms(updateToUsd);
            updateVehicleTransforms(updateToUsd);
            updateParticleTransforms(updateToUsd, updateVelocitiesToUsd, updateParticlesToUsd);
            updateDeformableTransforms(updateToUsd, updateVelocitiesToUsd);
        }
    }

    if (updateToUsd || updateVelocitiesToUsd || outputVelocitiesLocalSpace || updateParticlesToUsd || updateResidualsToUsd)
    {
        flushUsdToFabric(omniPhysX.getStage(), false);
    }
}

void InternalScene::addMimicJoint(InternalMimicJoint& mimicJoint)
{
    CARB_ASSERT(mMimicJointSet.find(&mimicJoint) == mMimicJointSet.end());

    mMimicJointSet.insert(&mimicJoint);

    addMimicJointMapEntries(mimicJoint);
}

void InternalScene::removeMimicJoint(InternalMimicJoint& mimicJoint)
{
    CARB_ASSERT(mMimicJointSet.find(&mimicJoint) != mMimicJointSet.end());

    mMimicJointSet.erase(&mimicJoint);

    removeMimicJointMapEntries(mimicJoint);
}

void InternalScene::addMimicJointMapEntries(InternalMimicJoint& mimicJoint)
{
    mJointToMimicJointMap.insert({ mimicJoint.getTargetJoint(), &mimicJoint });
    mJointToMimicJointMap.insert({ mimicJoint.getReferenceJoint(), &mimicJoint });
}

void InternalScene::removeMimicJointMapEntry(const PxArticulationJointReducedCoordinate* pxJoint, InternalMimicJoint* mimicJoint)
{
    std::pair<JointToMimicJointMap::iterator, JointToMimicJointMap::iterator> pairIter = mJointToMimicJointMap.equal_range(pxJoint);

    JointToMimicJointMap::iterator it = pairIter.first;
    while (it != pairIter.second)
    {
        if (it->second == mimicJoint)
        {
            mJointToMimicJointMap.erase(it);
            return;
        }

        it++;
    }
}

void InternalScene::removeMimicJointMapEntries(InternalMimicJoint& mimicJoint)
{
    removeMimicJointMapEntry(mimicJoint.getTargetJoint(), &mimicJoint);
    removeMimicJointMapEntry(mimicJoint.getReferenceJoint(), &mimicJoint);
}

void InternalScene::releasePhysXMimicJoints(const ::physx::PxArticulationJointReducedCoordinate& pxJoint)
{
    std::pair<JointToMimicJointMap::iterator, JointToMimicJointMap::iterator> pairIter = mJointToMimicJointMap.equal_range(&pxJoint);

    JointToMimicJointMap::iterator it = pairIter.first;

    if (it != pairIter.second)
    {
        std::vector<std::pair<const ::physx::PxArticulationJointReducedCoordinate*, InternalMimicJoint*> > otherJointPairList;
        // in theory, a joint can have an arbitrary number of mimic joints (even though the behavior would be
        // arbitrary), thus using an array that can grow and not something of fixed size

        do
        {
            InternalMimicJoint* mimicJoint = it->second;
            const ::physx::PxArticulationJointReducedCoordinate* targetJoint = mimicJoint->getTargetJoint();

            // since both joints of a mimic have an entry in the map, the other joint is temporarily stored
            // such that it can get removed after this loop

            if (&pxJoint == targetJoint)
                otherJointPairList.push_back({ mimicJoint->getReferenceJoint(), mimicJoint });
            else
                otherJointPairList.push_back({ targetJoint, mimicJoint });

            mimicJoint->releasePhysXMimicJoint();  // note: it's safe to call this again even if the PhysX object has been released
                                                   //       already (for example, if the same joint is target and reference but with
                                                   //       different axes)

            it = mJointToMimicJointMap.erase(it);  // erase returns iterator to the element after the erased one
        }
        while (it != pairIter.second);

        for (const std::pair<const ::physx::PxArticulationJointReducedCoordinate*, InternalMimicJoint*>& pair : otherJointPairList)
        {
            removeMimicJointMapEntry(pair.first, pair.second);
        }
    }
}

bool InternalScene::hasMimicJoint(const ::physx::PxArticulationJointReducedCoordinate& pxJoint) const
{
    std::pair<JointToMimicJointMap::const_iterator, JointToMimicJointMap::const_iterator> pairIter = mJointToMimicJointMap.equal_range(&pxJoint);

    return (pairIter.first != pairIter.second);
}

CUstream InternalScene::getDeformableCopyStream()
{
    // Set the dirty flag to ensure that the stream is synchronized in PhysXStepper::run()
    mDeformableCopyStreamDirty = true;
    return mDeformableCopyStream;
}

void InternalScene::syncDeformableCopyStream(PxCudaContextManager* cudaContextManager)
{
    if (mDeformableCopyStream != nullptr && mDeformableCopyStreamDirty == true)
    {
        mDeformableCopyStreamDirty = false;
        cudaContextManager->getCudaContext()->streamSynchronize(mDeformableCopyStream);
    }
}

PxQuat InternalJoint::getLocalPoseFixupQuat() const
{
    PxQuat ret = PxQuat(PxIdentity);

    // no fix required for eX
    if (mAxis == omni::physx::usdparser::Axis::eX)
        return ret;

    const float hRt2 = sqrt(2.0f) / 2.0f;
    if (mAxis == omni::physx::usdparser::Axis::eY)
    {
        // rotate x-axis to y-axis with a +90 deg rot around z
        ret = PxQuat(0.0f, 0.0f, hRt2, hRt2);
        // fixup spherical joint axes to match with cone angle limits:
        if (mJointType == usdparser::eJointSpherical)
            ret *= PxQuat(hRt2, 0.0f, 0.0f, hRt2);
    }
    else if (mAxis == omni::physx::usdparser::Axis::eZ)
    {
        // rotate x-axis to z-axis with a -90 deg rot around y
        ret = PxQuat(0.0f, -hRt2, 0.0f, hRt2);
        // fixup spherical joint axes to match with cone angle limits:
        if (mJointType == usdparser::eJointSpherical)
            ret *= PxQuat(hRt2, 0.0f, 0.0f, hRt2);
    }

    return ret;
}

void InternalJoint::fixupLocalPose(::physx::PxTransform& localPose) const
{
    localPose.q *= getLocalPoseFixupQuat();
}

void InternalJoint::setArticulationJointLimits(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis, float usdLowLimit, float usdHighLimit) const
{
    if (usdLowLimit > usdHighLimit)
        return;

    if (mBody0IsParentLink)
    {
        joint->setLimitParams(axis, PxArticulationLimit(usdLowLimit, usdHighLimit));
    }
    else
    {
        joint->setLimitParams(axis, PxArticulationLimit (-usdHighLimit, -usdLowLimit));
    }
}

void InternalJoint::updateArticulationJointLimitLow(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis, float usdLowLimit) const
{
    PxArticulationLimit limit = joint->getLimitParams(axis);
    if (mBody0IsParentLink)
    {
        if (usdLowLimit <= limit.high)
        {
            limit.low = usdLowLimit;
            joint->setLimitParams(axis, limit);
        }
    }
    else  // if joint body order does not follow articulation hierarchy, need to flip limits
    {
        // FLIP: high = -low and low = -high
        if (limit.low <= -usdLowLimit)
        {
            limit.high = -usdLowLimit;
            joint->setLimitParams(axis, limit);
        }
    }
}

void InternalJoint::updateArticulationJointLimitHigh(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis, float usdHighLimit) const
{

    PxArticulationLimit limit = joint->getLimitParams(axis);
    if (mBody0IsParentLink)
    {
        if (limit.low <= usdHighLimit)
        {
            limit.high = usdHighLimit;
            joint->setLimitParams(axis, limit);
        }
    }
    else  // if joint body order does not follow articulation hierarchy, need to flip limits
    {
        // FLIP: high = -low and low = -high
        if (-usdHighLimit <= limit.high)
        {
            limit.low = -usdHighLimit;
            joint->setLimitParams(axis, limit);
        }
    }
}


void InternalJoint::setArticulationDrivePositionTarget(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis, float positionTarget, const SdfPath jointPath) const
{
    if (positionTarget >= (2.0f * M_PI) || positionTarget <= -(2.0f * M_PI))
    {
        // createObject inside UsdInterface.cpp will only create eREVOLUTE joint if no limits exists
        // so driving to any angle close to 360 will be clamped / wrapped around
        const PxArticulationJointType::Enum type = joint->getJointType();
        if(type == PxArticulationJointType::eREVOLUTE)
        {
            const float targetPosition = radToDeg(positionTarget);
            OMNI_LOG_WARN(
            kRoboticsLogChannel,
            "Physics USD: Drive position target set to %2.f on %s will be wrapped in [-360, 360] range."
            "Consider setting explicit limits to enable use of unwrapped joints",
            targetPosition, jointPath.GetText());
            positionTarget = std::fmod(positionTarget, 2.0f * float(M_PI));  // map to [-360, 360] range here to avoid SDK Np warning
        }
    }
    joint->setDriveTarget(axis, mBody0IsParentLink ? positionTarget : -positionTarget);
}

void InternalJoint::setArticulationDriveVelocityTarget(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis, float velocityTarget) const
{
    joint->setDriveVelocity(axis, mBody0IsParentLink ? velocityTarget : -velocityTarget);
}

void InternalJoint::setArticulationJointPosition(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis, float position) const
{
    joint->setJointPosition(axis, mBody0IsParentLink ? position : -position);
}

void InternalJoint::setArticulationJointVelocity(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis, float velocity) const
{
    joint->setJointVelocity(axis, mBody0IsParentLink ? velocity : -velocity);
}

float InternalJoint::getArticulationJointPosition(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis) const
{
    const float position = joint->getJointPosition(axis);
    return mBody0IsParentLink ? position : -position;
}

float InternalJoint::getArticulationJointVelocity(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis) const
{
    const float velocity = joint->getJointVelocity(axis);
    return mBody0IsParentLink ? velocity : -velocity;
}

PhysxSchemaJointStateAPI InternalJoint::InternalJointState::getCachedJointStateAPI(UsdPrim jointPrim, omni::physx::usdparser::ObjectType jointType)
{
    if (!cachedJointStateAPI)
    {
        TfToken axisToken =  UsdPhysicsTokens->angular;
        switch(jointType)
        {
            case usdparser::eJointRevolute:
            {
                axisToken = UsdPhysicsTokens->angular;
                break;
            }
            case usdparser::eJointPrismatic:
            {
                axisToken = UsdPhysicsTokens->linear;
                break;
            }
            case usdparser::eJointD6:
            {
                axisToken =  UsdPhysicsTokens->rotX;
                switch (physxAxis)
                {
                    case ::physx::PxArticulationAxis::eX: axisToken = UsdPhysicsTokens->transX; break;
                    case ::physx::PxArticulationAxis::eY: axisToken = UsdPhysicsTokens->transY; break;
                    case ::physx::PxArticulationAxis::eZ: axisToken = UsdPhysicsTokens->transZ; break;
                    case ::physx::PxArticulationAxis::eTWIST: axisToken = UsdPhysicsTokens->rotX; break;
                    case ::physx::PxArticulationAxis::eSWING1: axisToken = UsdPhysicsTokens->rotY; break;
                    case ::physx::PxArticulationAxis::eSWING2: axisToken = UsdPhysicsTokens->rotZ; break;
                }
                break;
            }
        }
        cachedJointStateAPI = PhysxSchemaJointStateAPI::Get(jointPrim, axisToken);
    }
    return cachedJointStateAPI;
}
