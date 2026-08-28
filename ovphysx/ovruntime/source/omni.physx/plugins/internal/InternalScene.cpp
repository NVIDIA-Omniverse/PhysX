// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-WRITE-CORE-001
 * @covers AC-2 AC-6
 *
 * @implements REQ-WRITE-TRANSFORM-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-WRITE-DATA-001
 * @covers AC-1 AC-4
 *
 * @implements REQ-WRITE-ARRAY-001
 * @covers AC-1 AC-5
 */

#include "UsdPCH.h"

#include <carb/profiler/Profile.h>

#include "InternalScene.h"
#include "InternalParticle.h"
#include "InternalMimicJoint.h"

#include "particles/PhysXParticlePost.h"
#include <deformables/PhysXDeformablePost.h>

#include <PhysXUpdate.h>
#include <PhysXTools.h>
#include <PhysXSimulationCallbacks.h>
#include <CookingDataAsync.h>
#include <usdLoad/LoadUsd.h>
#include <UsdPhysicsDataWrite.h>
#include <UsdSource.h>
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
using namespace PXR_NS;
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

    // No attached stage means there are no records to register against, so skip the add.
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    ObjectId vehicleObjectId = kInvalidObjectId;
    if (attachedStage)
    {
        vehicleObjectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
            ePTVehicle, nullptr, &vehicle, attachedStage->keyFor(usdPrim.GetPrimPath()));
    }

    return vehicleObjectId;
}

void InternalScene::removeVehicle(InternalVehicle& vehicle)
{
    PhysXActorVehicleBase* pxVehicle = vehicle.mPhysXVehicle;
    CARB_ASSERT(mVehicleActorToVehicle.find(pxVehicle->getRigidDynamicActorNoCheck()) != mVehicleActorToVehicle.end());
    mVehicleActorToVehicle.erase(pxVehicle->getRigidDynamicActorNoCheck());

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
    if (objId == kInvalidObjectId)
        return;

    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objId);
    SdfPath path;
    if (objectRecord)
    {
        path = attachedStage->pathFor(objectRecord->mKey);
    }

    std::vector<InternalDeformableAttachment*> attachmentRemoveList;
    for (size_t i = 0; i < mDeformableAttachments.size(); i++)
    {
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
        SdfPath attachmentPath = attachedStage->pathFor(attachmentRemoveList[i]->mKey);
        if (removeDeformableAttachment(*attachmentRemoveList[i]))
        {
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
    if (objId == kInvalidObjectId)
        return;

    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objId);
    SdfPath path;
    if (objectRecord)
    {
        path = attachedStage->pathFor(objectRecord->mKey);
    }

    std::vector<InternalDeformableCollisionFilter*> collisionFilterRemoveList;
    for (size_t i = 0; i < mDeformableCollisionFilters.size(); i++)
    {
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
        SdfPath collisionFilterPath = attachedStage->pathFor(collisionFilterRemoveList[i]->mKey);
        if (removeDeformableCollisionFilter(*collisionFilterRemoveList[i]))
        {
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
      mScene(scene), mVolumeDeformablePostSolveCallback(nullptr), mSurfaceDeformablePostSolveCallback(nullptr)
{
    mSceneDesc = desc;

    PxCudaContextManager* cudaContextManager = scene->getCudaContextManager();
    if (cudaContextManager && cudaContextManager->getCudaContext())
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

    // Reset the Picker before any actors are freed so applyManipCmd cannot
    // dereference dangling pointers (e.g. a grabbed articulation link).
    OmniPhysX::getInstance().getRaycastManager().clearPickerForScene(mScene);

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
    if (cudaContextManager && cudaContextManager->getCudaContext())
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

static PXR_NS::GfMatrix4d getGfMatrix4dGfTransform(const Transform& transform)
{
    PXR_NS::GfTransform gf(
        transform.position, GfRotation(transform.orientation), GfVec3d(transform.scale), GfVec3d(0.0), GfRotation());

    return gf.GetMatrix();
}

// Writes a value to an SdfAttributeSpec using the type the attribute was
// authored with. Without matching the attribute's precision, USD logs a
// "Coding Error: Type mismatch" each time the spec is written, which floods
// simulation logs for scenes that use double-precision xformOps.
static void setTranslateSpec(const SdfAttributeSpecHandle& attr, const GfVec3d& value)
{
    if (!attr)
        return;
    const SdfValueTypeName& typeName = attr->GetTypeName();
    if (typeName == SdfValueTypeNames->Float3 || typeName == SdfValueTypeNames->Point3f ||
        typeName == SdfValueTypeNames->Vector3f || typeName == SdfValueTypeNames->Normal3f ||
        typeName == SdfValueTypeNames->Color3f)
    {
        attr->SetDefaultValue(VtValue(GfVec3f(value)));
    }
    else
    {
        attr->SetDefaultValue(VtValue(value));
    }
}

static void setOrientSpec(const SdfAttributeSpecHandle& attr, const GfQuatd& value)
{
    if (!attr)
        return;
    const SdfValueTypeName& typeName = attr->GetTypeName();
    if (typeName == SdfValueTypeNames->Quatf)
    {
        attr->SetDefaultValue(VtValue(GfQuatf(value)));
    }
    else if (typeName == SdfValueTypeNames->Quath)
    {
        const GfQuatf qf(value);
        attr->SetDefaultValue(VtValue(GfQuath(
            GfHalf(qf.GetReal()),
            GfVec3h(GfHalf(qf.GetImaginary()[0]), GfHalf(qf.GetImaginary()[1]), GfHalf(qf.GetImaginary()[2])))));
    }
    else
    {
        attr->SetDefaultValue(VtValue(value));
    }
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
        const bool resetXformOpStack = primXform.GetResetXformStack();
        primXform.ClearXformOpOrder();
        UsdGeomXformOp xform = primXform.MakeMatrixXform();
        primXform.SetResetXformStack(resetXformStack);
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
        const bool resetXformOpStack = primXform.GetResetXformStack();
        const GfMatrix4d mat = getGfMatrix4d(transform);
        UsdGeomXformOp xform = primXform.MakeMatrixXform();
        primXform.SetResetXformStack(resetXformStack);
        if (xform)
            xform.Set(mat);
    }
}

static void writeSingleNonRootTransformToUsd(UsdPrim& prim,
                                             const AttachedStage& attachedStage,
                                             omni::physics::parse::ObjectKey parentXformKey,
                                             const Transform& transform)
{
    const GfMatrix4d worldPose = getGfMatrix4d(transform);
    // Parent world transform via the source (per-call at Default; source-side
    // caching can be reintroduced to restore the previous xform-cache reuse).
    const GfMatrix4d parentWorldTransf = getWorldTransform(attachedStage, parentXformKey, UsdTimeCode::Default());
    const GfMatrix4d parentWorldTransfInv = parentWorldTransf.GetInverse();
    GfMatrix4d localTransf = worldPose * parentWorldTransfInv;

    setPrimXformOps(prim, localTransf, false);
}

namespace
{
// Forward declarations: defined in the sink-helper anonymous namespace below,
// but used here by resetStartProperties (which precedes that block).
PXR_NS::UsdPrim usdPrimForWrite(AttachedStage& as, omni::physics::parse::ObjectKey key);
void writeArrayToSink(omni::physics::parse::ObjectKey key, const PXR_NS::TfToken& attr, const void* data,
                      size_t count, omni::physics::parse::DataType type);
void writeMeshPointsToSink(omni::physics::parse::ObjectKey key, const PXR_NS::VtVec3fArray& points);
void writeMeshVelocitiesToSink(omni::physics::parse::ObjectKey key, const PXR_NS::VtVec3fArray& velocities);
void writeMeshExtentToSink(omni::physics::parse::ObjectKey key, const PXR_NS::VtVec3fArray& extent);
bool sourceHasArray(AttachedStage& as, omni::physics::parse::ObjectKey key, const PXR_NS::TfToken& attr,
                    PXR_NS::UsdTimeCode timeCode = PXR_NS::UsdTimeCode::Default());
} // namespace

void InternalScene::resetStartProperties(bool useUsdUpdate, bool useVelocitiesUSDUpdate, bool outputVelocitiesLocalSpace)
{
    // Ovstage source is read-only for ovruntime (see updateSimulationOutputs):
    // with no USD write sink the reset-on-stop write-back must not run.
    {
        AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        if (!as || !as->getDataWrite())
        {
            useUsdUpdate = false;
        }
    }

    UsdStageWeakPtr stage = UsdLoad::getUsdLoad()->getActiveStage();

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

                    if (useUsdUpdate)
                    {
                        AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
                        PXR_NS::UsdPrim wheelRootPrim = usdPrimForWrite(*as, wheelTMEntry.wheelRootKey);
                        PXR_NS::UsdPrim shapePrim = usdPrimForWrite(*as, wheelTMEntry.shapeKey);
                        const bool hasNonRootShape = wheelTMEntry.shape && (wheelRootPrim != shapePrim);

                        setPrimXformOps(wheelRootPrim, wheelTMEntry.initialTransform, false);

                        if (hasNonRootShape)
                            setPrimXformOps(shapePrim, wheelTMEntry.initialShapeTransform, false);
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

            AttachedStage& as = *UsdLoad::getUsdLoad()->getActiveAttachedStage();
            const omni::physics::parse::ObjectKey particleKey = particleSet->mKey;
            const omni::physics::parse::IPhysicsSource* source = as.getSource();
            const bool isPointInstancer =
                source && source->isA(particleKey, schemaTypeToken<UsdGeomPointInstancer>(*source));

            // transform particles from world space back to prim local space
            GfMatrix4f worldToLocal =
                GfMatrix4f(getWorldTransform(as, particleKey, UsdTimeCode::Default()).GetInverse());

            VtArray<GfVec3f> outPoints;
            copyBuffer(outPoints, &particleSet->mPositionSaveRestoreBuf[0],
                       uint32_t(particleSet->mPositionSaveRestoreBuf.size()), worldToLocal);
            writeArrayToSink(particleKey, isPointInstancer ? UsdGeomTokens->positions : UsdGeomTokens->points,
                             outPoints.cdata(), outPoints.size(), omni::physics::parse::DataType::e32Bit);

            VtArray<GfVec3f> outVelocities;
            copyBuffer(outVelocities, &particleSet->mVelocitySaveRestoreBuf[0],
                       uint32_t(particleSet->mVelocitySaveRestoreBuf.size()));
            writeArrayToSink(particleKey, UsdGeomTokens->velocities, outVelocities.cdata(), outVelocities.size(),
                             omni::physics::parse::DataType::e32Bit);

            if (sourceHasArray(as, particleKey, PhysxSchemaTokens->physxParticleSimulationPoints))
            {
                VtArray<GfVec3f> outSimPositions;
                copyBuffer(outSimPositions, &particleSet->mPositionSaveRestoreBuf[0],
                           uint32_t(particleSet->mPositionSaveRestoreBuf.size()), worldToLocal);
                writeArrayToSink(particleKey, PhysxSchemaTokens->physxParticleSimulationPoints,
                                 outSimPositions.cdata(), outSimPositions.size(),
                                 omni::physics::parse::DataType::e32Bit);
            }

            particleSet->mNumParticles = uint32_t(particleSet->mPositionSaveRestoreBuf.size());

            if (isPointInstancer)
            {
                if (sourceHasArray(as, particleKey, UsdGeomTokens->orientations))
                {
                    std::vector<::physx::PxQuat> orientations(particleSet->mNumParticles, ::physx::PxQuat(0.0f, 0.0f, 0.0f, 1.0f));
                    writeArrayToSink(particleKey, UsdGeomTokens->orientations, orientations.data(), orientations.size(),
                                     omni::physics::parse::DataType::e32Bit);
                }

                if (sourceHasArray(as, particleKey, UsdGeomTokens->scales))
                {
                    VtArray<GfVec3f> scales(particleSet->mNumParticles, GfVec3f(1.0f));
                    writeArrayToSink(particleKey, UsdGeomTokens->scales, scales.cdata(), scales.size(),
                                     omni::physics::parse::DataType::e32Bit);
                }
            }

            if (particleSet->mNumParticles == 0)
            {
                if (UsdPrim particlePrim = usdPrimForWrite(as, particleKey))
                {
                    // Workaround for OM-54774: this is a USD renderer hint,
                    // so it remains behind the USD output backend.
                    particlePrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(true);
                    particlePrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(false);
                }
            }
        }
    }

    // Deformable bodies store source-agnostic ObjectKeys; resolve mesh prims via
    // the active stage during this write-back.
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();

    for (size_t i = 0; i < mVolumeDeformableBodies.size(); i++)
    {
        InternalVolumeDeformableBody* deformableBody = mVolumeDeformableBodies[i];

        if (!deformableBody->mIsKinematic)
        {
            const Float3* srcPtr = deformableBody->mAllSkinMeshPointsSaveRestoreBuf.data();
            const uint32_t srcSize = uint32_t(deformableBody->mAllSkinMeshPointsSaveRestoreBuf.size());

            for (size_t i = 0; i < deformableBody->mSkinMeshKeys.size(); ++i)
            {
                const omni::physics::parse::ObjectKey skinKey = deformableBody->mSkinMeshKeys[i];
                const Uint2& range = deformableBody->mSkinMeshRanges[i];
                VtArray<GfVec3f> points;
                getArrayValue<VtVec3fArray>(*attachedStage, skinKey, UsdGeomTokens->points, UsdTimeCode::Default(),
                                            points);
                if (points.size() == range.y && srcSize >= range.x + range.y)
                {
                    std::memcpy(points.data(), srcPtr + range.x, sizeof(GfVec3f) * range.y);
                    writeMeshPointsToSink(skinKey, points);
                }
            }
        }

        {
            const omni::physics::parse::ObjectKey simKey = deformableBody->mSimMeshKey;
            {
                const Float3* srcPtr = deformableBody->mSimMeshPointsSaveRestoreBuf.data();
                const uint32_t srcSize = uint32_t(deformableBody->mSimMeshPointsSaveRestoreBuf.size());
                VtArray<GfVec3f> points;
                getArrayValue<VtVec3fArray>(*attachedStage, simKey, UsdGeomTokens->points, UsdTimeCode::Default(),
                                            points);
                if (points.size() == srcSize)
                {
                    std::memcpy(points.data(), srcPtr, sizeof(GfVec3f) * points.size());
                    writeMeshPointsToSink(simKey, points);
                }
            }

            {
                const Float3* srcPtr = deformableBody->mSimMeshVelocitiesSaveRestoreBuf.data();
                const uint32_t srcSize = uint32_t(deformableBody->mSimMeshVelocitiesSaveRestoreBuf.size());
                VtArray<GfVec3f> velocities;
                getArrayValue<VtVec3fArray>(*attachedStage, simKey, UsdGeomTokens->velocities,
                                            UsdTimeCode::Default(), velocities);
                if (velocities.size() == srcSize)
                {
                    std::memcpy(velocities.data(), srcPtr, sizeof(GfVec3f) * velocities.size());
                    writeMeshVelocitiesToSink(simKey, velocities);
                }
                else if (srcSize == 0) // for velocities, srcSize might be 0, which means no velocities
                {
                    velocities.clear();
                    writeMeshVelocitiesToSink(simKey, velocities);
                }
            }

            const omni::physics::parse::ObjectKey collKey = deformableBody->mCollMeshKey;
            if (collKey != simKey)
            {
                const Float3* srcPtr = deformableBody->mCollMeshPointsSaveRestoreBuf.data();
                const uint32_t srcSize = uint32_t(deformableBody->mCollMeshPointsSaveRestoreBuf.size());
                VtArray<GfVec3f> points;
                getArrayValue<VtVec3fArray>(*attachedStage, collKey, UsdGeomTokens->points, UsdTimeCode::Default(),
                                            points);
                if (points.size() == srcSize)
                {
                    std::memcpy(points.data(), srcPtr, sizeof(GfVec3f) * points.size());
                    writeMeshPointsToSink(collKey, points);
                }
            }

            writeMeshExtentToSink(collKey, deformableBody->mCollMeshExtentSaveRestoreBuf);
        }
    }

    for (size_t i = 0; i < mSurfaceDeformableBodies.size(); i++)
    {
        InternalSurfaceDeformableBody* deformableBody = mSurfaceDeformableBodies[i];

        if (!deformableBody->mIsKinematic)
        {
            const Float3* srcPtr = deformableBody->mAllSkinMeshPointsSaveRestoreBuf.data();
            const uint32_t srcSize = uint32_t(deformableBody->mAllSkinMeshPointsSaveRestoreBuf.size());

            for (size_t i = 0; i < deformableBody->mSkinMeshKeys.size(); ++i)
            {
                const omni::physics::parse::ObjectKey skinKey = deformableBody->mSkinMeshKeys[i];
                const Uint2& range = deformableBody->mSkinMeshRanges[i];
                VtArray<GfVec3f> points;
                getArrayValue<VtVec3fArray>(*attachedStage, skinKey, UsdGeomTokens->points, UsdTimeCode::Default(),
                                            points);
                if (points.size() == range.y && srcSize >= range.x + range.y)
                {
                    std::memcpy(points.data(), srcPtr + range.x, sizeof(GfVec3f) * range.y);
                    writeMeshPointsToSink(skinKey, points);
                }
            }
        }

        {
            const omni::physics::parse::ObjectKey simKey = deformableBody->mSimMeshKey;
            {
                const Float3* srcPtr = deformableBody->mSimMeshPointsSaveRestoreBuf.data();
                const uint32_t srcSize = uint32_t(deformableBody->mSimMeshPointsSaveRestoreBuf.size());
                VtArray<GfVec3f> points;
                getArrayValue<VtVec3fArray>(*attachedStage, simKey, UsdGeomTokens->points, UsdTimeCode::Default(),
                                            points);
                if (points.size() == srcSize)
                {
                    std::memcpy(points.data(), srcPtr, sizeof(GfVec3f) * points.size());
                    writeMeshPointsToSink(simKey, points);
                }
            }

            {
                const Float3* srcPtr = deformableBody->mSimMeshVelocitiesSaveRestoreBuf.data();
                const uint32_t srcSize = uint32_t(deformableBody->mSimMeshVelocitiesSaveRestoreBuf.size());
                VtArray<GfVec3f> velocities;
                getArrayValue<VtVec3fArray>(*attachedStage, simKey, UsdGeomTokens->velocities,
                                            UsdTimeCode::Default(), velocities);
                if (velocities.size() == srcSize)
                {
                    std::memcpy(velocities.data(), srcPtr, sizeof(GfVec3f) * velocities.size());
                    writeMeshVelocitiesToSink(simKey, velocities);
                }
                else if (srcSize == 0) // for velocities, srcSize might be 0, which means no velocities
                {
                    velocities.clear();
                    writeMeshVelocitiesToSink(simKey, velocities);
                }
            }

            writeMeshExtentToSink(simKey, deformableBody->mSimMeshExtentSaveRestoreBuf);
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

    // Wheel/shape prims are stored as source-agnostic keys; resolve to prims via
    // the active stage during this per-frame write-back.
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();

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
                            PXR_NS::UsdPrim wheelRootPrim = usdPrimForWrite(*attachedStage, wheelTMEntry.wheelRootKey);
                            PXR_NS::UsdPrim shapePrim = usdPrimForWrite(*attachedStage, wheelTMEntry.shapeKey);
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
                                transformFn(asInt(wheelRootPrim.GetPath()), fromPhysX(wheelGlobalPose.p),
                                            fromPhysX(wheelGlobalPose.q), cbUserData);
                            }

                            if (!wheelGlobalPose.isValid())
                            {
                                CARB_LOG_WARN("Invalid PhysX transform detected for %s on wheel attachment %s.",
                                              vehicleActor->getName(), wheelRootPrim.GetPath().GetText());
                            }
                            else if (!skipWriteTransforms &&
                                     !(vehicle->mFlags & InternalVehicleFlag::eSKIP_UPDATE_TRANSFORM))
                            {
                                Transform fcTransform;
                                fcTransform.position = (GfVec3f&)wheelGlobalPose.p;
                                fcTransform.orientation = (GfQuatf&)wheelGlobalPose.q;
                                fcTransform.scale = (GfVec3f&)wheelTMEntry.scale;

                                if (updateToUsd)
                                    writeSingleNonRootTransformToUsd(wheelRootPrim, *attachedStage,
                                                                     wheelTMEntry.wheelRootParentXformKey, fcTransform);

                                if (wheelTMEntry.shape && (wheelRootPrim != shapePrim))
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

                                        setPrimXformOps(shapePrim, fcIdentityTransform, false);
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

namespace
{
// Append an actor's WORLD pose to the transform batch. The sink owns the
// world->local conversion (parent frame) and the residual extra-transform, so
// the engine just emits the physics-native world pose -- no USD hierarchy math.
void accumulateSinkTransform(const Transform& fcTransform,
                             omni::physics::parse::ObjectKey key,
                             std::vector<omni::physics::parse::ObjectKey>& keys,
                             std::vector<::physx::PxVec3>& positions,
                             std::vector<::physx::PxQuat>& orientations)
{
    keys.push_back(key);
    const GfVec3f& p = fcTransform.position;
    positions.push_back(::physx::PxVec3{ p[0], p[1], p[2] });
    const GfVec3f img = fcTransform.orientation.GetImaginary();
    orientations.push_back(::physx::PxQuat{ img[0], img[1], img[2], fcTransform.orientation.GetReal() });
}

// Publish an accumulated transform batch through IPhysicsDataWrite in one call.
void flushSinkTransforms(const std::vector<omni::physics::parse::ObjectKey>& keys,
                         const std::vector<::physx::PxVec3>& positions,
                         const std::vector<::physx::PxQuat>& orientations)
{
    if (keys.empty())
        return;
    if (AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage())
    {
        if (omni::physics::parse::IPhysicsDataWrite* dw = as->getDataWrite())
        {
            using omni::physics::parse::DataWriteView;
            using omni::physics::parse::DataType;
            dw->beginWrite();
            // Decomposed world-space columns: positions/orientations as host
            // float vec3/quat(xyzw); a default-constructed (null) scales view
            // leaves scale untouched.
            dw->writeTransforms(
                keys.data(), keys.size(),
                DataWriteView{ positions.data(), positions.size(), 0, -1, DataType::e32Bit },
                DataWriteView{ orientations.data(), orientations.size(), 0, -1, DataType::e32Bit },
                DataWriteView{});
            dw->endWrite();
        }
    }
}

// Publish an accumulated velocity batch (already transformed: local-space and
// deg/s as appropriate) through IPhysicsDataWrite in one call.
void flushSinkVelocities(const std::vector<omni::physics::parse::ObjectKey>& keys,
                         const std::vector<::physx::PxVec3>& linear,
                         const std::vector<::physx::PxVec3>& angular)
{
    if (keys.empty())
        return;
    if (AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage())
    {
        if (omni::physics::parse::IPhysicsDataWrite* dw = as->getDataWrite())
        {
            using omni::physics::parse::DataWriteView;
            using omni::physics::parse::DataType;
            // Velocities are a batched per-object scatter: one writeData per
            // channel, the attribute token interned through the source (the one
            // token vocabulary), one host float-vec3 value per key.
            auto* src = as->getSource();
            const omni::physics::parse::TokenId velTok =
                src->internToken(PXR_NS::UsdPhysicsTokens->physicsVelocity.GetString());
            const omni::physics::parse::TokenId angVelTok =
                src->internToken(PXR_NS::UsdPhysicsTokens->physicsAngularVelocity.GetString());
            dw->beginWrite();
            dw->writeData(keys.data(), keys.size(), velTok,
                          DataWriteView{ linear.data(), linear.size(), 0, -1, DataType::e32Bit });
            dw->writeData(keys.data(), keys.size(), angVelTok,
                          DataWriteView{ angular.data(), angular.size(), 0, -1, DataType::e32Bit });
            dw->endWrite();
        }
    }
}

// Publish a point-instancer's whole-array attributes (positions/orientations,
// and optionally velocities) through the sink. positions/velocities are float
// vec3; orientations are half quats (quath[]), matching the instancer schema.
void flushInstancerArrays(const PXR_NS::SdfPath& instancerPath,
                          const PXR_NS::UsdAttribute& posAttr, const PXR_NS::VtVec3fArray& positions,
                          const PXR_NS::UsdAttribute& orientAttr, const PXR_NS::VtArray<PXR_NS::GfQuath>& orientations,
                          bool writeVel,
                          const PXR_NS::UsdAttribute& linVelAttr, const PXR_NS::VtVec3fArray& linVels,
                          const PXR_NS::UsdAttribute& angVelAttr, const PXR_NS::VtVec3fArray& angVels)
{
    if (instancerPath.IsEmpty())
        return;
    AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!as)
        return;
    omni::physics::parse::IPhysicsDataWrite* dw = as->getDataWrite();
    if (!dw)
        return;

    using omni::physics::parse::DataWriteView;
    using omni::physics::parse::DataType;

    // Single-threaded per-frame path: minting the instancer key here is safe.
    const omni::physics::parse::ObjectKey key = as->keyFor(instancerPath);

    // The sink's quaternion contract is uniform PxQuat order (xyzw); convert the
    // engine's Gf real-first (wxyz) orientation buffer at the boundary. half->float
    // is exact, so the round-trip to the quath destination is lossless.
    std::vector<::physx::PxQuat> orientXYZW;
    if (orientAttr)
    {
        orientXYZW.reserve(orientations.size());
        for (const PXR_NS::GfQuath& q : orientations)
        {
            const PXR_NS::GfVec3h im = q.GetImaginary();
            orientXYZW.push_back(::physx::PxQuat(float(im[0]), float(im[1]), float(im[2]), float(q.GetReal())));
        }
    }

    dw->beginWrite();
    if (posAttr)
        dw->writeArray(key, as->getSource()->internToken(posAttr.GetName().GetString()),
                       DataWriteView{ positions.cdata(), positions.size(), 0, -1, DataType::e32Bit });
    if (orientAttr)
        dw->writeArray(key, as->getSource()->internToken(orientAttr.GetName().GetString()),
                       DataWriteView{ orientXYZW.data(), orientXYZW.size(), 0, -1, DataType::e32Bit });
    if (writeVel)
    {
        if (linVelAttr)
            dw->writeArray(key, as->getSource()->internToken(linVelAttr.GetName().GetString()),
                           DataWriteView{ linVels.cdata(), linVels.size(), 0, -1, DataType::e32Bit });
        if (angVelAttr)
            dw->writeArray(key, as->getSource()->internToken(angVelAttr.GetName().GetString()),
                           DataWriteView{ angVels.cdata(), angVels.size(), 0, -1, DataType::e32Bit });
    }
    dw->endWrite();
}

PXR_NS::UsdPrim usdPrimForWrite(AttachedStage& as, omni::physics::parse::ObjectKey key)
{
    omni::physics::usd::UsdPhysicsDataWrite* dw = omni::physics::usd::asUsdDataWrite(as.getDataWrite());
    return dw ? dw->usdPrimForWrite(key) : PXR_NS::UsdPrim();
}

// Publish one whole-array attribute through the source-agnostic sink. The
// element shape is taken from the destination attribute in the backend; `type`
// gives the source scalar precision.
void writeArrayToSink(omni::physics::parse::ObjectKey key,
                      const PXR_NS::TfToken& attr,
                      const void* data,
                      size_t count,
                      omni::physics::parse::DataType type)
{
    if (!key.valid() || (!data && count != 0))
        return;
    AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!as || !as->getSource())
        return;
    if (omni::physics::parse::IPhysicsDataWrite* dw = as->getDataWrite())
    {
        dw->writeArray(key, as->getSource()->internToken(attr.GetString()),
                       omni::physics::parse::DataWriteView{ data, count, 0, -1, type });
    }
}

bool sourceHasArray(AttachedStage& as,
                    omni::physics::parse::ObjectKey key,
                    const PXR_NS::TfToken& attr,
                    PXR_NS::UsdTimeCode timeCode)
{
    const omni::physics::parse::IPhysicsSource* source = as.getSource();
    if (!source)
        return false;
    const omni::physics::parse::BufferHandle handle =
        source->getArrayAttribute(key, source->internToken(attr.GetString()),
                                  physxtools_detail::toReadTime(timeCode));
    if (!handle.valid())
        return false;
    source->releaseBuffer(handle);
    return true;
}

// Deformable mesh point/velocity write-back helpers (float vec3 on the mesh prim).
void writeMeshPointsToSink(omni::physics::parse::ObjectKey key, const PXR_NS::VtVec3fArray& points)
{
    writeArrayToSink(key, PXR_NS::UsdGeomTokens->points, points.cdata(), points.size(),
                     omni::physics::parse::DataType::e32Bit);
}
void writeMeshVelocitiesToSink(omni::physics::parse::ObjectKey key, const PXR_NS::VtVec3fArray& velocities)
{
    writeArrayToSink(key, PXR_NS::UsdGeomTokens->velocities, velocities.cdata(), velocities.size(),
                     omni::physics::parse::DataType::e32Bit);
}
void writeMeshExtentToSink(omni::physics::parse::ObjectKey key, const PXR_NS::VtVec3fArray& extent)
{
    writeArrayToSink(key, PXR_NS::UsdGeomTokens->extent, extent.cdata(), extent.size(),
                     omni::physics::parse::DataType::e32Bit);
}
} // namespace

void InternalScene::updateCctTransforms(bool updateToUsd)
{
    std::vector<omni::physics::parse::ObjectKey> sinkKeys;
    std::vector<::physx::PxVec3> sinkPositions;
    std::vector<::physx::PxQuat> sinkOrientations;

    CctMap::iterator it = mCctMap.begin();
    while (it != mCctMap.end())
    {
        InternalCct* actor = it->second;

        if (actor && actor->mActor)
        {

            PxRigidDynamic* dyna = static_cast<PxRigidDynamic*>(actor->mActor);

            if (updateToUsd && !dyna->isSleeping())
            {
                const PxTransform transform = dyna->getGlobalPose();

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

                    if (actor->mFlags & InternalActorFlag::eUSE_DATAWRITE_SINK)
                    {
                        accumulateSinkTransform(fcTransform, actor->mKey, sinkKeys, sinkPositions, sinkOrientations);
                    }
                }
            }
        }
        it++;
    }

    flushSinkTransforms(sinkKeys, sinkPositions, sinkOrientations);
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
        AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();

        PxU32 nbActors = 0;
        PxActor** activeActors = mScene->getActiveActors(nbActors);

        // Batch of rigid-body transforms (incl. nested) routed through
        // IPhysicsDataWrite, flushed in one call after the loop. The sink authors
        // ancestor-first, so nested bodies don't need a separate deferred pass.
        std::vector<omni::physics::parse::ObjectKey> sinkKeys;
        std::vector<::physx::PxVec3> sinkPositions;
        std::vector<::physx::PxQuat> sinkOrientations;

        // Velocity batch (values already transformed to local-space / deg-s as
        // needed), flushed through the sink after the loop.
        std::vector<omni::physics::parse::ObjectKey> velKeys;
        std::vector<::physx::PxVec3> velLinear;
        std::vector<::physx::PxVec3> velAngular;

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
                                updateJointState(UsdLoad::getUsdLoad()->getActiveStage(), jointRecord, updateVelocitiesToUsd);
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
                        const PxTransform transform = dyna->getGlobalPose();

                        if (notifyTransforms ||
                            (actor->mFlags & InternalActorFlag::eNOTIFY_TRANSFORM && cb->getTransformationWriteFn()))
                        {
                            transformFn(asInt(attachedStage->pathFor(actor->mKey)), fromPhysX(transform.p),
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
                                    if (updateToUsd && (actor->mFlags & InternalActorFlag::eUSE_DATAWRITE_SINK))
                                    {
                                        // Routed through IPhysicsDataWrite: accumulate the WORLD pose into the
                                        // batch flushed after the loop. The sink converts to local (authoring a
                                        // batch ancestor-first, so nested bodies see fresh parent poses) and
                                        // folds in the residual extra-transform.
                                        accumulateSinkTransform(fcTransform, record.mKey, sinkKeys, sinkPositions,
                                                                sinkOrientations);
                                    }
                                }
                                else
                                {
                                    if (updateToUsd)
                                    {
                                        const PXR_NS::SdfPath instancerPath = attachedStage->pathFor(actor->mInstanceKey);

                                        if (instancerPath != currInstancerPath)
                                        {
                                            // Per-call at Default; recomputed only when the instancer
                                            // changes (gated above), so the per-instancer cost is modest.
                                            currInstancerMatrixInverse =
                                                getWorldTransform(*attachedStage, actor->mInstanceKey, UsdTimeCode::Default())
                                                    .GetInverse();

                                            // Flush the previous instancer's accumulated positions/orientations
                                            // through the sink (velocities are flushed only at the end).
                                            flushInstancerArrays(currInstancerPath, pos, positionValues, orient,
                                                                 orientationValues, /*writeVel*/ false, linearVel,
                                                                 linearVelocityValues, angularVel, angularVelocityValues);

                                            PXR_NS::UsdGeomPointInstancer pointInstancer =
                                                PXR_NS::UsdGeomPointInstancer::Get(
                                                    attachedStage->getStage(), instancerPath);

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

                                    velocityFn(asInt(attachedStage->pathFor(actor->mKey)), fromPhysX(linVel), fromPhysX(angVel),
                                               cbUserData);
                                }

                                if (updateVelocitiesToUsd && !skipWriteVelocities &&
                                    !(actor->mFlags & InternalActorFlag::eSKIP_UPDATE_VELOCITY))
                                {
                                    // Compute the values the sink will author to physicsVelocity /
                                    // physicsAngularVelocity (local-space + scale-divided when requested;
                                    // angular always in deg/s), then accumulate into the velocity batch.
                                    const PxVec3 linVel = dyna->getLinearVelocity();
                                    const PxVec3 angVel = dyna->getAngularVelocity();
                                    GfVec3f outLinear;
                                    GfVec3f outAngular;

                                    if (actor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES)
                                    {
                                        const PxVec3 linVelTransformed = transform.q.rotateInv(linVel);
                                        outLinear = GfCompDiv(
                                            (const GfVec3f&)linVelTransformed, (const GfVec3f&)actor->mScale);
                                        const PxVec3 angularVelRotated = transform.q.rotateInv(angVel);
                                        outAngular = GfVec3f(angularVelRotated.x, angularVelRotated.y, angularVelRotated.z);
                                    }
                                    else
                                    {
                                        outLinear = GfVec3f(linVel.x, linVel.y, linVel.z);
                                        outAngular = GfVec3f(angVel.x, angVel.y, angVel.z);
                                    }
                                    outAngular = radToDeg(outAngular);

                                    velKeys.push_back(record.mKey);
                                    velLinear.push_back(::physx::PxVec3{ outLinear[0], outLinear[1], outLinear[2] });
                                    velAngular.push_back(::physx::PxVec3{ outAngular[0], outAngular[1], outAngular[2] });
                                }
                            }
                        }
                    }
                }
            }
        }

        // Flush the rigid-body transform batch (incl. nested) through the sink in
        // one ancestor-first call.
        flushSinkTransforms(sinkKeys, sinkPositions, sinkOrientations);
        flushSinkVelocities(velKeys, velLinear, velAngular);
    }

    // Flush the last instancer's accumulated arrays (incl. velocities) through the sink.
    flushInstancerArrays(currInstancerPath, pos, positionValues, orient, orientationValues,
                         updateVelocitiesToUsd, linearVel, linearVelocityValues, angularVel, angularVelocityValues);
}

void InternalScene::updateParticleTransforms(bool updateToUsd, bool updateVelocitiesToUsd, bool updateParticlesToUsd)
{
    // 0 is disabled, 1/2 is selected/all
    const bool debugVizEnabled = (OmniPhysX::getInstance().getCachedSettings().visualizationDisplayParticles > 0);
    UsdStageWeakPtr stage = UsdLoad::getUsdLoad()->getActiveStage();

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteTransforms = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);

    for (size_t particleSystemIndex = 0; particleSystemIndex < mParticleSystems.size(); particleSystemIndex++)
    {
        InternalPbdParticleSystem* particleSystem = mParticleSystems[particleSystemIndex];

        uint32_t postFlags = particles::getPostprocessStages(particleSystem->getPath());
        bool particleSystemHasAnisotropy = postFlags & ParticlePostFlag::eAnisotropy;
        bool particleSystemHasSmoothing = postFlags & ParticlePostFlag::eSmoothing;
        bool particleSystemHasIsosurface = postFlags & ParticlePostFlag::eIsosurface;

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
            particles::getAnisotropy(anisotropyQ1, anisotropyQ2, anisotropyQ3, particleSystem->getPath());
        }

        PxVec4* smoothedPos = nullptr;
        if (particleSystemHasSmoothing)
        {
            smoothedPos = particles::getSmoothedPositions(particleSystem->getPath());
        } 


        // TODO preallocate/resize - this explicitly assumes 0 to clear if there are no diffuse particles
        VtArray<GfVec3f> tmpDiffuseParticlePoints(0);
        VtArray<GfVec3f> tmpDiffuseParticleColors(0);
        PxU32 tmpDiffuseParticleCount = 0;

        for (InternalParticleSet* particleSet : particleSystem->mParticleSets)
        {
            if (!particleSet->mNumParticles || !particleSet->mEnabled || !particleSet->mDownloadDirtyFlags)
                continue;

            AttachedStage& as = *UsdLoad::getUsdLoad()->getActiveAttachedStage();
            const omni::physics::parse::ObjectKey particleKey = particleSet->mKey;
            const omni::physics::parse::IPhysicsSource* source = as.getSource();
            const bool isPointInstancer =
                source && source->isA(particleKey, schemaTypeToken<UsdGeomPointInstancer>(*source));

            // transform particles from world space back to prim local space
            GfMatrix4f worldToLocal =
                GfMatrix4f(getWorldTransform(as, particleKey, UsdTimeCode::Default()).GetInverse());

            uint32_t flags = particleSet->mDownloadDirtyFlags;
            const TfToken& pointsToken = isPointInstancer ? UsdGeomTokens->positions : UsdGeomTokens->points;

            if (flags & ParticleDirtyFlags::eVELOCITY)
            {
                VtArray<GfVec3f> tmpVelocities;
                copyBuffer(tmpVelocities, (const carb::Float4*)particleSet->mVelocities, particleSet->mNumParticles);
                writeArrayToSink(particleKey, UsdGeomTokens->velocities, tmpVelocities.cdata(),
                                 tmpVelocities.size(), omni::physics::parse::DataType::e32Bit);
            }

            if (flags & ParticleDirtyFlags::ePOSITION_INVMASS)
            {
                // means we downloaded positions - so if we also downloaded the smoothed pos, we write the positions into the simPos attribute.
                if (flags & ParticleDirtyFlags::eSMOOTHED_POSITIONS)
                {
                    VtArray<GfVec3f> tmpSimPositions;
                    copyBuffer(tmpSimPositions, particleSet->mPositions, particleSet->mNumParticles, worldToLocal);
                    writeArrayToSink(particleKey, PhysxSchemaTokens->physxParticleSimulationPoints,
                                     tmpSimPositions.cdata(), tmpSimPositions.size(),
                                     omni::physics::parse::DataType::e32Bit);
                }
                else
                {
                    VtArray<GfVec3f> tmpPoints;
                    copyBuffer(tmpPoints, particleSet->mPositions, particleSet->mNumParticles, worldToLocal);
                    writeArrayToSink(particleKey, pointsToken, tmpPoints.cdata(), tmpPoints.size(),
                                     omni::physics::parse::DataType::e32Bit);
                }
            }

            if ((flags & ParticleDirtyFlags::eSMOOTHED_POSITIONS) && smoothedPos)
            {
                PxU32 start = particleSet->mParticleBuffer->getFlatListStartIndex();
                VtArray<GfVec3f> tmpPoints;
                copyBuffer(tmpPoints, &smoothedPos[start], particleSet->mNumParticles, worldToLocal);
                writeArrayToSink(particleKey, pointsToken, tmpPoints.cdata(), tmpPoints.size(),
                                 omni::physics::parse::DataType::e32Bit);
            }

            if ((flags & ParticleDirtyFlags::eANISOTROPY) && anisotropyQ1 && anisotropyQ2 && anisotropyQ3)
            {
                PxU32 start = particleSet->mParticleBuffer->getFlatListStartIndex();
                if (!isPointInstancer)
                {
                    if (UsdPrim particlePrim = usdPrimForWrite(as, particleKey))
                    {
                        VtArray<GfVec4f> tmpValuesQ1(particleSet->mNumParticles);
                        VtArray<GfVec4f> tmpValuesQ2(particleSet->mNumParticles);
                        VtArray<GfVec4f> tmpValuesQ3(particleSet->mNumParticles);

                        UsdGeomPrimvarsAPI primVarsAPI(particlePrim);
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
                }
                else
                {
                    float contactDistanceInv = 1.0f / (particleSystem->mPS->getParticleContactOffset() * 2.0f);
                    VtArray<GfVec3f> tmpScales(particleSet->mNumParticles);
                    std::vector<PxQuat> tmpOrientations(particleSet->mNumParticles);

                    for (PxU32 i = start; i < start + particleSet->mNumParticles; i++)
                    {
                        PxVec4 q1 = anisotropyQ1[i];
                        PxVec4 q2 = anisotropyQ2[i];
                        PxVec4 q3 = anisotropyQ3[i];
                        tmpScales[i - start] = { 4.0f * q1[3] * contactDistanceInv, 4.0f * q2[3] * contactDistanceInv, 4.0f * q3[3] * contactDistanceInv };
                        tmpOrientations[i - start] = PxQuat(PxMat33(q1.getXYZ(), q2.getXYZ(), q3.getXYZ()));
                    }

                    writeArrayToSink(particleKey, UsdGeomTokens->scales, tmpScales.cdata(), tmpScales.size(),
                                     omni::physics::parse::DataType::e32Bit);
                    writeArrayToSink(particleKey, UsdGeomTokens->orientations, tmpOrientations.data(),
                                     tmpOrientations.size(), omni::physics::parse::DataType::e32Bit);
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
            particles::updateIsosurfaceMesh(particleSystem->getPath());
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
    if (!cudaContextManager || !cudaContextManager->getCudaContext())
    {
        CARB_LOG_WARN_ONCE("InternalScene::updateDeformableTransforms: CUDA context unavailable, skipping.");
        return;
    }

    // Deformable bodies store source-agnostic ObjectKeys; resolve mesh prims via
    // the active stage during this write-back.
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();

    PxScopedCudaLock _lock(*cudaContextManager);

    // TODO adenzler use event
    // TODO be careful about race conditions?
    syncDeformableCopyStream(cudaContextManager);

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
            for (size_t i = 0; i < deformableBody->mSkinMeshKeys.size(); ++i)
            {
                const omni::physics::parse::ObjectKey skinKey = deformableBody->mSkinMeshKeys[i];
                const Uint2& range = deformableBody->mSkinMeshRanges[i];
                const GfMatrix4f& worldToSkinMesh = deformableBody->mWorldToSkinMeshTransforms[i];
                VtArray<GfVec3f> points;
                getArrayValue<VtVec3fArray>(*attachedStage, skinKey, UsdGeomTokens->points, UsdTimeCode::Default(),
                                            points);
                if (points.size() == range.y)
                {
                    copyBuffer(points, srcPtr + range.x, range.y, worldToSkinMesh);
                    writeMeshPointsToSink(skinKey, points);
                }
            }
        }

        {
            const omni::physics::parse::ObjectKey simKey = deformableBody->mSimMeshKey;
            const GfMatrix4f& worldToSimMesh = deformableBody->mWorldToSimMesh;
            VtArray<GfVec3f> points;
            {
                const ::physx::PxVec4* srcPtr = deformableBody->mSimMeshPositionInvMassH;
                const uint32_t srcSize = deformableBody->mNumSimMeshVertices;
                getArrayValue<VtVec3fArray>(*attachedStage, simKey, UsdGeomTokens->points, UsdTimeCode::Default(),
                                            points);
                if (points.size() == srcSize)
                {
                    copyBuffer(points, srcPtr, srcSize, worldToSimMesh);
                    writeMeshPointsToSink(simKey, points);
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
                    writeMeshVelocitiesToSink(simKey, velocities);
                }
            }

            const omni::physics::parse::ObjectKey collKey = deformableBody->mCollMeshKey;
            const GfMatrix4f& worldToCollMesh = deformableBody->mWorldToCollMesh;
            if (collKey != simKey)
            {
                const ::physx::PxVec4* srcPtr = deformableBody->mCollMeshPositionInvMassH;
                const uint32_t srcSize = deformableBody->mNumCollMeshVertices;
                VtArray<GfVec3f> collPoints;
                getArrayValue<VtVec3fArray>(*attachedStage, collKey, UsdGeomTokens->points, UsdTimeCode::Default(),
                                            collPoints);
                if (collPoints.size() == srcSize)
                {
                    copyBuffer(collPoints, srcPtr, srcSize, worldToCollMesh);
                    writeMeshPointsToSink(collKey, collPoints);
                }
            }

            const PxBounds3 worldBounds = deformableBody->mDeformableVolume->getWorldBounds();
            GfBBox3d bbox(GfRange3d(toVec3d(worldBounds.minimum), toVec3d(worldBounds.maximum)));
            bbox.Transform(GfMatrix4d(worldToCollMesh));
            GfRange3d transformedBounds = bbox.ComputeAlignedBox();
            VtArray<GfVec3f> extent(2);
            extent[0] = GfVec3f(transformedBounds.GetMin());
            extent[1] = GfVec3f(transformedBounds.GetMax());
            writeMeshExtentToSink(collKey, extent);
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
            for (size_t i = 0; i < deformableBody->mSkinMeshKeys.size(); ++i)
            {
                const omni::physics::parse::ObjectKey skinKey = deformableBody->mSkinMeshKeys[i];
                const Uint2& range = deformableBody->mSkinMeshRanges[i];
                const GfMatrix4f& worldToSkinMesh = deformableBody->mWorldToSkinMeshTransforms[i];
                VtArray<GfVec3f> points;
                getArrayValue<VtVec3fArray>(*attachedStage, skinKey, UsdGeomTokens->points, UsdTimeCode::Default(),
                                            points);
                if (points.size() == range.y)
                {
                    copyBuffer(points, srcPtr + range.x, range.y, worldToSkinMesh);
                    writeMeshPointsToSink(skinKey, points);
                }
            }
        }

        {
            const omni::physics::parse::ObjectKey simKey = deformableBody->mSimMeshKey;
            const GfMatrix4f& worldToSimMesh = deformableBody->mWorldToSimMesh;
            VtArray<GfVec3f> points;
            {
                const ::physx::PxVec4* srcPtr = deformableBody->mSimMeshPositionInvMassH;
                const uint32_t srcSize = deformableBody->mNumSimMeshVertices;
                getArrayValue<VtVec3fArray>(*attachedStage, simKey, UsdGeomTokens->points, UsdTimeCode::Default(),
                                            points);
                if (points.size() == srcSize)
                {
                    copyBuffer(points, srcPtr, uint32_t(points.size()), worldToSimMesh);
                    writeMeshPointsToSink(simKey, points);
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
                    writeMeshVelocitiesToSink(simKey, velocities);
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
                writeMeshExtentToSink(simKey, extent);
            }
        }
    }
}

void InternalScene::updateJointState(UsdStageWeakPtr stage, const InternalDatabase::Record& record, bool updateVelocitiesToUsd)
{
    InternalJoint* intJoint = (InternalJoint*)record.mInternalPtr;
    ::physx::PxArticulationJointReducedCoordinate* joint = (::physx::PxArticulationJointReducedCoordinate*)record.mPtr;
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    const SdfPath recordPath = attachedStage->pathFor(record.mKey);

    for (size_t idx = 0; idx < 6; ++idx)
    {
        InternalJoint::InternalJointState& intJointState = intJoint->mJointStates[idx];
        if (!intJointState.enabled)
            continue;
        PhysxSchemaJointStateAPI cachedJointStateAPI =
            intJointState.getCachedJointStateAPI(stage->GetPrimAtPath(recordPath), intJoint->mJointType);
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

void InternalScene::updateSimulationOutputs(bool updateToUsd,
                                           bool updateVelocitiesToUsd,
                                           bool outputVelocitiesLocalSpace,
                                           bool updateParticlesToUsd)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();

    // Under the ovstage backend ovruntime never writes simulation outputs back to
    // the source: the application reads them via the read API (IOvxPhysicsRead.h)
    // and authors ovstage itself. There is no USD write sink (getDataWrite() is
    // null), so the per-frame USD write-back must not run -- it would otherwise
    // resolve to invalid UsdPrims and throw "Used null prim".
    {
        AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        if (!attachedStage || !attachedStage->getDataWrite())
        {
            updateToUsd = false;
            updateVelocitiesToUsd = false;
            updateParticlesToUsd = false;
        }
    }

    ScopedNoticeBlock scopedNoticeBlock;

    if (omniPhysX.getSimulationLayer())
    {
        UsdStageWeakPtr stage = UsdLoad::getUsdLoad()->getActiveStage();
        UsdEditContext editContext(stage, UsdEditTarget(omniPhysX.getSimulationLayer()));

        PXR_NS::SdfChangeBlock changeBlock;

        {
            CARB_PROFILE_ZONE(0, "updateRenderTransforms::USDWrite");
            updateRigidBodyTransforms(updateToUsd, updateVelocitiesToUsd, outputVelocitiesLocalSpace);
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
            updateCctTransforms(updateToUsd);
            updateVehicleTransforms(updateToUsd);
            updateParticleTransforms(updateToUsd, updateVelocitiesToUsd, updateParticlesToUsd);
            updateDeformableTransforms(updateToUsd, updateVelocitiesToUsd);
        }
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
        if (!cudaContextManager || !cudaContextManager->getCudaContext())
        {
            CARB_LOG_WARN_ONCE("InternalScene::syncDeformableCopyStream: CUDA context unavailable, skipping.");
            return;
        }
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


void InternalJoint::setArticulationDrivePositionTarget(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis, float positionTarget, const SdfPath jointKey) const
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
            targetPosition, jointKey.GetText());
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


void InternalScene::debugDraw(omni::physx::OmniRenderBuffer& renderBuffer, uint64_t debugDrawFlags)
{
    if (debugDrawFlags & InternalDebugDrawFlags::eDEBUG_DRAW_SPLINES_SEGMENTS)
    {
        // A.B. TODO buffer the actors
        for (const InternalActor* actor : mActors)
        {
            if (actor->mSplinesCurve)
            {
                const SplinesCurve* spline = actor->mSplinesCurve;
                const PxTransform splineWorldPose = actor->mActor->getGlobalPose() * actor->mSplineLocalSpace;
                spline->draw(renderBuffer, splineWorldPose);
            }
        }
    }
}
