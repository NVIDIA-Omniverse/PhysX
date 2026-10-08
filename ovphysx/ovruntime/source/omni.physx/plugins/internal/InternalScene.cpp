// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-WRITE-CORE-001
 * @covers AC-2 AC-6 AC-9
 *
 * @implements REQ-WRITE-AUTHORING-001
 * @covers AC-2
 *
 * @implements REQ-WRITE-TRANSFORM-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-WRITE-DATA-001
 * @covers AC-1 AC-4
 *
 * @implements REQ-WRITE-ARRAY-001
 * @covers AC-1 AC-5 AC-7
 *
 * @implements REQ-SIM-MULTISCENE-001
 * @covers AC-4
 *
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-9
 *
 * @implements REQ-WRITE-LOCALXFORM-001
 * @covers AC-1 AC-2 AC-4
 *
 * The point-instancer transform write-back needs a backing USD stage for its typed
 * schema object; the gate sits at that read and AFTER `flushInstancerArrays`, so the
 * source-agnostic sink write for the previous instancer is not dropped, and it logs
 * once rather than once per instancer per step.
 *
 * @implements REQ-SIM-PARTICLE-001
 * @covers AC-1 AC-2
 *
 * `updateParticleTransforms` calls `PxScene::fetchResultsParticleSystem()`
 * unconditionally for every enabled, updated particle system; only the
 * subsequent USD/ovstage-authoring work stays gated on `updateToUsd` /
 * `updateParticlesToUsd`. The particle finalize stream is
 * `CU_STREAM_NON_BLOCKING`, so a write-back-sink gate around this call left a
 * stageless (no-sink) attach with no sync point at all, racing any direct GPU
 * readback of particle positions.
 *
 * @implements REQ-SIM-ACTIVEACTOR-001
 * @covers AC-2 AC-3
 *
 * @implements REQ-SIM-DIAGNOSTICS-001
 * @covers AC-2
 *
 * @implements REQ-WRITE-VELOCITYNOTIFY-001
 * @covers AC-1
 *
 * @implements REQ-BUILD-BRIDGE-001
 * @covers AC-5
 */

#include <carb/profiler/Profile.h>

#include <omni/physics/parse/KnownTokens.h>

#include <cstring>

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
#include <Raycast.h>
#include <PhysXTools.h>
#include <ScopedNoticeLock.h>

#include <common/foundation/TransformedExtent.h>
#include <common/utilities/MemoryMacros.h>

#if USE_PHYSX_GPU
#include <extensions/PxParticleExt.h>
#endif

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace carb;
using namespace ::physx;

OMNI_LOG_DECLARE_CHANNEL(kRoboticsLogChannel)

void InternalScene::addActor(InternalActor& actor)
{
    mActors.push_back(&actor);
}

bool InternalScene::removeActor(const InternalActor& actor)
{
    for (size_t i = 0; i < mActors.size(); i++)
    {
        if (mActors[i] == &actor)
        {
            mActors[i] = mActors.back();
            mActors.pop_back();
            return true;
        }
    }
    return false;
}

void MirrorActor::release(bool trackReleasedActor)
{
    if (trackReleasedActor && internalScene)
        internalScene->trackReleasedActiveActor(actor);
    ::physx::PxCollectionExt::releaseObjects(*collection);
    collection->release();
    free(mirrorMemory);
}

void InternalScene::setVehicleContext(const VehicleContextDesc& contextDesc)
{
    mVehicleContext.init(contextDesc, *mScene);
}

omni::physx::usdparser::ObjectId InternalScene::addVehicle(InternalVehicle& vehicle,
    const uint32_t wheelCount, omni::physics::parse::ObjectKey vehicleKey, const bool enabled)
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
    mVehicleSetEpoch++;

    const ObjectId vehicleObjectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
        ePTVehicle, nullptr, &vehicle, vehicleKey);

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
    mVehicleSetEpoch++;
}

void InternalScene::setVehicleEnabledState(InternalVehicle& vehicle, const bool enabled)
{
    const bool enabledNow = isVehicleEnabled(vehicle);
    if (enabled != enabledNow)
    {
        PhysXActorVehicleBase* pxVehicle = vehicle.mPhysXVehicle;
        mVehicleSetEpoch++;

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
    omni::physics::parse::ObjectKey key;
    if (objectRecord)
    {
        key = objectRecord->mKey;
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
        if (removeDeformableAttachment(*attachmentRemoveList[i]))
        {
            attachedStage->getDeformableAttachmentHistoryMap().insert({ key, attachmentRemoveList[i]->mKey });
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
    omni::physics::parse::ObjectKey key;
    if (objectRecord)
    {
        key = objectRecord->mKey;
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
        if (removeDeformableCollisionFilter(*collisionFilterRemoveList[i]))
        {
            attachedStage->getDeformableCollisionFilterHistoryMap().insert({ key, collisionFilterRemoveList[i]->mKey });
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
    : mEnabledVehicleCount(0), mVehicleSetEpoch(1),
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
        // mActor is set right after the InternalActor is registered and is never reset afterwards, so
        // a null here means this entry no longer refers to a live InternalActor. Skip the PxActor
        // work instead of dereferencing it; this is a mitigation, the entry should not be here at all
        // (NVBugs 6504495).
        if (current->mActor && !current->mActor->is<PxArticulationLink>())
        {
            if(current->mActor->is<PxRigidBody>())
            {
                rayMan.clearPicker(current->mActor);
            }
            SAFE_RELEASE(current->mActor)
        }
        for (MirrorActor& mirror : current->mMirrors)
        {
            // All scene tasks have completed and the scenes are being destroyed.
            // Tracking is unnecessary here and another mirror scene may already
            // have been deleted.
            mirror.release(false);
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
    carb::Float3 position;
    carb::Float4 orientation;
    carb::Float3 scale;
};


namespace
{
// Forward declaration: defined in the sink-helper anonymous namespace below,
// but used here by resetStartProperties (which precedes that block).
void writeLocalTransformMatrixToSink(AttachedStage& as, omni::physics::parse::ObjectKey key,
                                     const ::physx::PxMat44d& localMatrix, bool setScale);
void writeArrayToSink(omni::physics::parse::ObjectKey key, omni::physics::parse::TokenId attr, const void* data,
                      size_t count, omni::physics::parse::DataType type);
void writeMeshPointsToSink(omni::physics::parse::ObjectKey key, const carb::Float3* points, size_t count);
void writeMeshVelocitiesToSink(omni::physics::parse::ObjectKey key, const carb::Float3* velocities, size_t count);
void writeMeshExtentToSink(omni::physics::parse::ObjectKey key, const carb::Float3* extent, size_t count);
bool sourceHasArray(AttachedStage& as, omni::physics::parse::ObjectKey key, omni::physics::parse::TokenId attr,
                    omni::physics::parse::ReadTime readTime = omni::physics::parse::ReadTime::defaultTime());
} // namespace

static void writeSingleNonRootTransformToUsd(AttachedStage& attachedStage,
                                             omni::physics::parse::ObjectKey objectKey,
                                             omni::physics::parse::ObjectKey parentXformKey,
                                             const Transform& transform)
{
    // The world->local solve stays in double-precision PhysX math; the local
    // matrix then goes to the write sink, which owns the xform-op authoring.
    const PxMat44d worldPose = makeMatrix(
        PxTransform(toPhysX(transform.position), toPhysXQuat(transform.orientation)), toPhysX(transform.scale));
    // Parent world transform via the source (per-call at Default; source-side
    // caching can be reintroduced to restore the previous xform-cache reuse).
    const PxMat44d parentWorldTransf =
        getWorldTransform(attachedStage, parentXformKey, omni::physics::parse::ReadTime::defaultTime());
    // affineInverse, not inverseRT: the parent frame can carry non-uniform scale.
    const PxMat44d parentWorldTransfInv = affineInverse(parentWorldTransf);
    // Gf `worldPose * parentWorldTransfInv` -- the operands swap in PhysX order.
    const PxMat44d localTransf = parentWorldTransfInv * worldPose;

    // A product of two arbitrary affine transforms: it can carry shear, so it
    // crosses to the sink as a matrix rather than a decomposed pose.
    writeLocalTransformMatrixToSink(attachedStage, objectKey, localTransf, false);
}

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
                        // Object identity, not a UsdPrim resolve: ObjectKey equality is the
                        // backend-agnostic form of "is this the same prim" (a live USD resolve
                        // would agree, since keyFor()/pathFor() are a bijection for a given
                        // stage), so this needs no backing UsdPrim at all.
                        const bool hasNonRootShape = wheelTMEntry.shape && (wheelTMEntry.wheelRootKey != wheelTMEntry.shapeKey);

                        writeLocalTransformMatrixToSink(*as, wheelTMEntry.wheelRootKey,
                                                        wheelTMEntry.initialTransform, false);

                        if (hasNonRootShape)
                            writeLocalTransformMatrixToSink(*as, wheelTMEntry.shapeKey,
                                                            wheelTMEntry.initialShapeTransform, false);
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

            // Interned once per particle set: every writeArrayToSink/sourceHasArray/isA
            // call below routes through the TokenId overload (KnownTokens convention)
            // instead of re-interning a TfToken literal on every hot-path call.
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            const bool isPointInstancer = source && source->isA(particleKey, tok.pointInstancerType);

            // transform particles from world space back to prim local space.
            // The inverse is taken in double precision (the prim transform can carry
            // non-uniform scale, so affineInverse rather than inverseRT) and stays in
            // double: copyBuffer narrows only the transformed result to float.
            const PxMat44d worldToLocal =
                affineInverse(getWorldTransform(as, particleKey, omni::physics::parse::ReadTime::defaultTime()));

            std::vector<carb::Float3> outPoints;
            copyBuffer(outPoints, &particleSet->mPositionSaveRestoreBuf[0],
                       uint32_t(particleSet->mPositionSaveRestoreBuf.size()), worldToLocal);
            writeArrayToSink(particleKey, isPointInstancer ? tok.positions : tok.points,
                             outPoints.data(), outPoints.size(), omni::physics::parse::DataType::e32Bit);

            std::vector<carb::Float3> outVelocities;
            copyBuffer(outVelocities, &particleSet->mVelocitySaveRestoreBuf[0],
                       uint32_t(particleSet->mVelocitySaveRestoreBuf.size()));
            writeArrayToSink(particleKey, tok.velocities, outVelocities.data(), outVelocities.size(),
                             omni::physics::parse::DataType::e32Bit);

            if (sourceHasArray(as, particleKey, tok.physxParticleSimulationPoints))
            {
                std::vector<carb::Float3> outSimPositions;
                copyBuffer(outSimPositions, &particleSet->mPositionSaveRestoreBuf[0],
                           uint32_t(particleSet->mPositionSaveRestoreBuf.size()), worldToLocal);
                writeArrayToSink(particleKey, tok.physxParticleSimulationPoints,
                                 outSimPositions.data(), outSimPositions.size(),
                                 omni::physics::parse::DataType::e32Bit);
            }

            particleSet->mNumParticles = uint32_t(particleSet->mPositionSaveRestoreBuf.size());

            if (isPointInstancer)
            {
                if (sourceHasArray(as, particleKey, tok.orientations))
                {
                    std::vector<::physx::PxQuat> orientations(particleSet->mNumParticles, ::physx::PxQuat(0.0f, 0.0f, 0.0f, 1.0f));
                    writeArrayToSink(particleKey, tok.orientations, orientations.data(), orientations.size(),
                                     omni::physics::parse::DataType::e32Bit);
                }

                if (sourceHasArray(as, particleKey, tok.scales))
                {
                    std::vector<carb::Float3> scales(particleSet->mNumParticles, carb::Float3{ 1.0f, 1.0f, 1.0f });
                    writeArrayToSink(particleKey, tok.scales, scales.data(), scales.size(),
                                     omni::physics::parse::DataType::e32Bit);
                }
            }

            if (particleSet->mNumParticles == 0)
            {
                // Workaround for OM-54774: a Hydra renderer hint; a no-op on a backend
                // without this concept.
                if (omni::physics::parse::IPhysicsDataWrite* dw = as.getDataWrite())
                {
                    dw->writeBoolAttribute(particleKey, "omni:rtx:skip", true);
                    dw->writeBoolAttribute(particleKey, "omni:rtx:skip", false);
                }
            }
        }
    }

    // Deformable bodies store source-agnostic ObjectKeys; resolve mesh prims via
    // the active stage during this write-back.
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    const omni::physics::parse::IPhysicsSource* deformableSource =
        attachedStage ? attachedStage->getSource() : nullptr;
    omni::physics::parse::KnownTokens tok;
    if (deformableSource)
        tok.intern(*deformableSource);

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
                // The authored array is read only for its length: the saved buffer
                // then replaces it wholesale, so it is published straight from the
                // engine's own storage instead of being memcpy'd through a VtArray.
                std::vector<carb::Float3> points;
                getArrayValue(*attachedStage, skinKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);
                if (points.size() == range.y && srcSize >= range.x + range.y)
                {
                    writeMeshPointsToSink(skinKey, srcPtr + range.x, range.y);
                }
            }
        }

        {
            const omni::physics::parse::ObjectKey simKey = deformableBody->mSimMeshKey;
            {
                const Float3* srcPtr = deformableBody->mSimMeshPointsSaveRestoreBuf.data();
                const uint32_t srcSize = uint32_t(deformableBody->mSimMeshPointsSaveRestoreBuf.size());
                std::vector<carb::Float3> points;
                getArrayValue(*attachedStage, simKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);
                if (points.size() == srcSize)
                {
                    writeMeshPointsToSink(simKey, srcPtr, srcSize);
                }
            }

            {
                const Float3* srcPtr = deformableBody->mSimMeshVelocitiesSaveRestoreBuf.data();
                const uint32_t srcSize = uint32_t(deformableBody->mSimMeshVelocitiesSaveRestoreBuf.size());
                std::vector<carb::Float3> velocities;
                getArrayValue(*attachedStage, simKey, tok.velocities, omni::physics::parse::ReadTime::defaultTime(), velocities);
                if (velocities.size() == srcSize)
                {
                    writeMeshVelocitiesToSink(simKey, srcPtr, srcSize);
                }
                else if (srcSize == 0) // for velocities, srcSize might be 0, which means no velocities
                {
                    writeMeshVelocitiesToSink(simKey, nullptr, 0);
                }
            }

            const omni::physics::parse::ObjectKey collKey = deformableBody->mCollMeshKey;
            if (collKey != simKey)
            {
                const Float3* srcPtr = deformableBody->mCollMeshPointsSaveRestoreBuf.data();
                const uint32_t srcSize = uint32_t(deformableBody->mCollMeshPointsSaveRestoreBuf.size());
                std::vector<carb::Float3> points;
                getArrayValue(*attachedStage, collKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);
                if (points.size() == srcSize)
                {
                    writeMeshPointsToSink(collKey, srcPtr, srcSize);
                }
            }

            const std::vector<carb::Float3>& extent = deformableBody->mCollMeshExtentSaveRestoreBuf;
            writeMeshExtentToSink(collKey, extent.data(), extent.size());
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
                // See the volume-deformable equivalent above: the authored array is
                // read for its length only.
                std::vector<carb::Float3> points;
                getArrayValue(*attachedStage, skinKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);
                if (points.size() == range.y && srcSize >= range.x + range.y)
                {
                    writeMeshPointsToSink(skinKey, srcPtr + range.x, range.y);
                }
            }
        }

        {
            const omni::physics::parse::ObjectKey simKey = deformableBody->mSimMeshKey;
            {
                const Float3* srcPtr = deformableBody->mSimMeshPointsSaveRestoreBuf.data();
                const uint32_t srcSize = uint32_t(deformableBody->mSimMeshPointsSaveRestoreBuf.size());
                std::vector<carb::Float3> points;
                getArrayValue(*attachedStage, simKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);
                if (points.size() == srcSize)
                {
                    writeMeshPointsToSink(simKey, srcPtr, srcSize);
                }
            }

            {
                const Float3* srcPtr = deformableBody->mSimMeshVelocitiesSaveRestoreBuf.data();
                const uint32_t srcSize = uint32_t(deformableBody->mSimMeshVelocitiesSaveRestoreBuf.size());
                std::vector<carb::Float3> velocities;
                getArrayValue(*attachedStage, simKey, tok.velocities, omni::physics::parse::ReadTime::defaultTime(), velocities);
                if (velocities.size() == srcSize)
                {
                    writeMeshVelocitiesToSink(simKey, srcPtr, srcSize);
                }
                else if (srcSize == 0) // for velocities, srcSize might be 0, which means no velocities
                {
                    writeMeshVelocitiesToSink(simKey, nullptr, 0);
                }
            }

            const std::vector<carb::Float3>& extent = deformableBody->mSimMeshExtentSaveRestoreBuf;
            writeMeshExtentToSink(simKey, extent.data(), extent.size());
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
                                // TransformUpdateNotificationFn's identifier is the raw
                                // ObjectKey::handle, not an SdfPath-bit encoding (ADR-0018).
                                // See ADR-0021 for the generation-tag staleness contract.
                                transformFn(wheelTMEntry.wheelRootKey.handle, fromPhysX(wheelGlobalPose.p),
                                            fromPhysX(wheelGlobalPose.q), cbUserData);
                            }

                            if (!wheelGlobalPose.isValid())
                            {
                                CARB_LOG_WARN("Invalid PhysX transform detected for %s on wheel attachment %s.",
                                              vehicleActor->getName(), attachedStage->textFor(wheelTMEntry.wheelRootKey));
                            }
                            else if (!skipWriteTransforms &&
                                     !(vehicle->mFlags & InternalVehicleFlag::eSKIP_UPDATE_TRANSFORM))
                            {
                                // USD-authoring staging value: the PhysX pose is converted here, at the
                                // boundary, rather than reinterpret-cast into Gf.
                                Transform fcTransform;
                                fcTransform.position = toFloat3(wheelGlobalPose.p);
                                fcTransform.orientation = toFloat4(wheelGlobalPose.q);
                                fcTransform.scale = wheelTMEntry.scale;

                                if (updateToUsd)
                                    writeSingleNonRootTransformToUsd(*attachedStage, wheelTMEntry.wheelRootKey,
                                                                     wheelTMEntry.wheelRootParentXformKey, fcTransform);

                                // Object identity, not a UsdPrim resolve -- see the matching
                                // comment in resetStartProperties.
                                if (wheelTMEntry.shape && (wheelTMEntry.wheelRootKey != wheelTMEntry.shapeKey))
                                {
                                    // the shape position and orientation is set to the same as the wheel root
                                    fcTransform.scale = wheelTMEntry.shapeScale;

                                    if (updateToUsd)
                                    {
                                        // The shape sits at the wheel root's pose, so its own local
                                        // transform is identity. (Scale is not authored -- the sink
                                        // is called with setScale false -- so identity carries it.)
                                        writeLocalTransformMatrixToSink(*attachedStage, wheelTMEntry.shapeKey,
                                                                        PxMat44d(PxIdentity), false);
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
// Component-wise divide of a body-local velocity by the actor's scale. PxVec3 has no
// component-wise divide operator, and the scale arrives as a carb::Float3.
inline ::physx::PxVec3 divideByScale(const ::physx::PxVec3& v, const carb::Float3& scale)
{
    return ::physx::PxVec3(v.x / scale.x, v.y / scale.y, v.z / scale.z);
}

// Append an actor's WORLD pose to the transform batch. The sink owns the
// world->local conversion (parent frame) and the residual extra-transform, so
// the engine just emits the physics-native world pose -- no USD hierarchy math.
void accumulateSinkTransform(const ::physx::PxTransform& worldPose,
                             omni::physics::parse::ObjectKey key,
                             std::vector<omni::physics::parse::ObjectKey>& keys,
                             std::vector<::physx::PxVec3>& positions,
                             std::vector<::physx::PxQuat>& orientations)
{
    keys.push_back(key);
    positions.push_back(worldPose.p);
    orientations.push_back(worldPose.q);
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
            omni::physics::parse::KnownTokens tok;
            tok.intern(*src);
            const omni::physics::parse::TokenId velTok = tok.physicsVelocity;
            const omni::physics::parse::TokenId angVelTok = tok.physicsAngularVelocity;
            dw->beginWrite();
            dw->writeData(keys.data(), keys.size(), velTok,
                          DataWriteView{ linear.data(), linear.size(), 0, -1, DataType::e32Bit });
            dw->writeData(keys.data(), keys.size(), angVelTok,
                          DataWriteView{ angular.data(), angular.size(), 0, -1, DataType::e32Bit });
            dw->endWrite();
        }
    }
}

// Seed a point-instancer accumulation buffer from the attribute's authored
// array. The buffers are PhysX-typed (the sink takes float vec3 and xyzw
// PxQuat), so the conversion off the authored element type happens once, here,
// on the instancer switch -- and the per-step values then reach the sink at the
// precision the engine computed them at, leaving the narrowing to the quath[]
// destination to the backend that owns the destination.
//
// `out` is left untouched when the attribute has no resolvable value, matching
// the raw UsdAttribute::Get these replaced. The caller promotes a purely
// time-sampled attribute's earliest sample to Default first (via
// IPhysicsDataWrite::promoteEarliestSampleToDefault): the write-back reads/
// writes at Default, so a purely time-sampled attribute needs a Default value
// before this can see it.
//
bool readInstancerArray(AttachedStage& as, omni::physics::parse::ObjectKey key, omni::physics::parse::TokenId attr,
                        std::vector<::physx::PxVec3>& out, bool promoteTimeSamplesToDefault)
{
    if (promoteTimeSamplesToDefault)
    {
        omni::physics::parse::IPhysicsDataWrite* dw = as.getDataWrite();
        if (!dw || !dw->promoteEarliestSampleToDefault(key, attr))
            return false;
    }
    std::vector<carb::Float3> authored;
    if (!getArrayValue(as, key, attr, omni::physics::parse::ReadTime::defaultTime(), authored))
        return false;
    out.resize(authored.size());
    for (size_t i = 0; i < authored.size(); ++i)
        out[i] = ::physx::PxVec3(authored[i].x, authored[i].y, authored[i].z);
    return true;
}

bool readInstancerArray(AttachedStage& as, omni::physics::parse::ObjectKey key, omni::physics::parse::TokenId attr,
                        std::vector<::physx::PxQuat>& out, bool promoteTimeSamplesToDefault)
{
    if (promoteTimeSamplesToDefault)
    {
        omni::physics::parse::IPhysicsDataWrite* dw = as.getDataWrite();
        if (!dw || !dw->promoteEarliestSampleToDefault(key, attr))
            return false;
    }
    std::vector<carb::Float4> authored;
    if (!getArrayValue(as, key, attr, omni::physics::parse::ReadTime::defaultTime(), authored))
        return false;
    out.resize(authored.size());
    for (size_t i = 0; i < authored.size(); ++i)
        // The array-read decode ladder (PhysXTools.h) already produces xyzw
        // (PxQuat order) for a quath source -- see its halfBitsToFloat comment.
        out[i] = ::physx::PxQuat(authored[i].x, authored[i].y, authored[i].z, authored[i].w);
    return true;
}

// Publish a point-instancer's whole-array attributes (positions/orientations,
// and optionally velocities) through the sink. positions/velocities are float
// vec3; orientations are float xyzw quats, which the backend narrows to the
// instancer schema's quath[]. have*/writeVel gate which channels this
// instancer actually had a resolvable value for (see the call site).
void flushInstancerArrays(omni::physics::parse::ObjectKey instancerKey,
                          bool havePositions, const std::vector<::physx::PxVec3>& positions,
                          bool haveOrientations, const std::vector<::physx::PxQuat>& orientations,
                          bool writeVel,
                          bool haveLinearVel, const std::vector<::physx::PxVec3>& linVels,
                          bool haveAngularVel, const std::vector<::physx::PxVec3>& angVels)
{
    if (!instancerKey.valid())
        return;
    AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!as)
        return;
    omni::physics::parse::IPhysicsDataWrite* dw = as->getDataWrite();
    const omni::physics::parse::IPhysicsSource* source = as->getSource();
    if (!dw || !source)
        return;

    using omni::physics::parse::DataWriteView;
    using omni::physics::parse::DataType;

    omni::physics::parse::KnownTokens tok;
    tok.intern(*source);

    dw->beginWrite();
    if (havePositions)
        dw->writeArray(instancerKey, tok.positions,
                       DataWriteView{ positions.data(), positions.size(), 0, -1, DataType::e32Bit });
    if (haveOrientations)
        dw->writeArray(instancerKey, tok.orientations,
                       DataWriteView{ orientations.data(), orientations.size(), 0, -1, DataType::e32Bit });
    if (writeVel)
    {
        if (haveLinearVel)
            dw->writeArray(instancerKey, tok.velocities,
                           DataWriteView{ linVels.data(), linVels.size(), 0, -1, DataType::e32Bit });
        if (haveAngularVel)
            dw->writeArray(instancerKey, tok.angularVelocities,
                           DataWriteView{ angVels.data(), angVels.size(), 0, -1, DataType::e32Bit });
    }
    dw->endWrite();
}

// Author one object's LOCAL pose through the write sink, from a full 4x4.
//
// The attach/reset counterpart to the batched per-step transform output: the
// vehicle wheel writes below are one-shot and already local, so they bypass
// writeTransforms (which takes world poses and resolves the parent frame) and
// go straight to the sink's matrix entry point. A matrix, not a decomposed
// pose, because the caller's `affineInverse(parentWorld) * world` product can
// carry shear a TRS triple cannot represent.
void writeLocalTransformMatrixToSink(AttachedStage& as,
                                     omni::physics::parse::ObjectKey key,
                                     const ::physx::PxMat44d& localMatrix,
                                     bool setScale)
{
    if (omni::physics::parse::IPhysicsDataWrite* dw = as.getDataWrite())
    {
        // Same-layout element copy, not a transpose: PxMat44d and parse::Matrix4d hold
        // the same sixteen row-major doubles for the same transform.
        omni::physics::parse::Matrix4d matrix;
        static_assert(sizeof(matrix.data) == sizeof(localMatrix), "PxMat44d / parse::Matrix4d layout mismatch");
        std::memcpy(matrix.data, &localMatrix, sizeof(matrix.data));
        dw->writeLocalTransformMatrix(key, matrix, setScale);
    }
}

// Publish one whole-array attribute through the source-agnostic sink. The
// element shape is taken from the destination attribute in the backend; `type`
// gives the source scalar precision.
void writeArrayToSink(omni::physics::parse::ObjectKey key,
                      omni::physics::parse::TokenId attr,
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
        dw->writeArray(key, attr, omni::physics::parse::DataWriteView{ data, count, 0, -1, type });
    }
}

bool sourceHasArray(AttachedStage& as,
                    omni::physics::parse::ObjectKey key,
                    omni::physics::parse::TokenId attr,
                    omni::physics::parse::ReadTime readTime)
{
    const omni::physics::parse::IPhysicsSource* source = as.getSource();
    if (!source)
        return false;
    const omni::physics::parse::BufferHandle handle = source->getArrayAttribute(key, attr, readTime);
    if (!handle.valid())
        return false;
    source->releaseBuffer(handle);
    return true;
}

// Deformable mesh point/velocity write-back helpers (float vec3 on the mesh prim).
// They take a raw `carb::Float3` span rather than a `VtArray`: the payload only
// ever crosses the sink as an untyped column, so the buffer type is the engine's
// choice, not USD's. `count == 0` with a null pointer is the sink's "author an
// empty array" reset.
void writeMeshPointsToSink(omni::physics::parse::ObjectKey key, const carb::Float3* points, size_t count)
{
    if (AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage())
        if (const omni::physics::parse::IPhysicsSource* src = as->getSource())
            writeArrayToSink(key, src->internToken("points"), points, count, omni::physics::parse::DataType::e32Bit);
}
void writeMeshVelocitiesToSink(omni::physics::parse::ObjectKey key, const carb::Float3* velocities, size_t count)
{
    if (AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage())
        if (const omni::physics::parse::IPhysicsSource* src = as->getSource())
            writeArrayToSink(key, src->internToken("velocities"), velocities, count, omni::physics::parse::DataType::e32Bit);
}
void writeMeshExtentToSink(omni::physics::parse::ObjectKey key, const carb::Float3* extent, size_t count)
{
    if (AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage())
        if (const omni::physics::parse::IPhysicsSource* src = as->getSource())
            writeArrayToSink(key, src->internToken("extent"), extent, count, omni::physics::parse::DataType::e32Bit);
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
                    if (actor->mFlags & InternalActorFlag::eUSE_DATAWRITE_SINK)
                    {
                        accumulateSinkTransform(PxTransform(transform.p, q), actor->mKey, sinkKeys, sinkPositions,
                                                sinkOrientations);
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
    // instancer support -- current point-instancer identity/state, tracked across the
    // active-actor loop below so the accumulation buffers are only reseeded (and the
    // previous instancer's arrays flushed) on an actual instancer switch.
    omni::physics::parse::ObjectKey currInstancerKey;
    PxMat44d currInstancerMatrixInverse(PxIdentity);
    // "Does this channel have a resolvable value for the CURRENT instancer".
    // haveLinVelAttr/haveAngVelAttr are only reassigned when updateVelocitiesToUsd is true.
    bool havePositions = false;
    bool haveOrientations = false;
    bool haveLinVelAttr = false;
    bool haveAngVelAttr = false;

    // Whole-array accumulation buffers for the point-instancer write-back, in
    // the sink's element types (vec3 float, quat xyzw float). Seeded from the
    // authored arrays on each instancer switch, so instances this scene does not
    // simulate keep their authored values.
    std::vector<::physx::PxVec3> positionValues;
    std::vector<::physx::PxQuat> orientationValues;
    std::vector<::physx::PxVec3> linearVelocityValues;
    std::vector<::physx::PxVec3> angularVelocityValues;

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteTransforms = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);
    const bool skipWriteVelocities = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eVELOCITY | GlobalSimulationFlag::eSKIP_WRITE);
    const bool velocitiesInRad = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eVELOCITY | GlobalSimulationFlag::eNOTIFY_IN_RADIANS);
    TransformUpdateNotificationFn transformFn = cb->getTransformationWriteFn();
    VelocityUpdateNotificationFn velocityFn = cb->getVelocityWriteFn();
    void* cbUserData = cb->getUserData();
    const bool notifyTransforms = transformFn && cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eNOTIFY_UPDATE);
    const bool notifyVelocities = velocityFn && cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eVELOCITY | GlobalSimulationFlag::eNOTIFY_UPDATE);

    // skip the update loop if we should skip write and dont have notification callback request as a global
    // setting
    if (!(skipWriteTransforms && skipWriteVelocities && !notifyTransforms && !notifyVelocities))
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
        const bool hasReleasedActors = hasReleasedActiveActors();

        for (PxU32 i = 0; i < nbActors; i++)
        {
            const PxActor* pxActor = activeActors[i];
            if (!pxActor || (hasReleasedActors && isReleasedActiveActor(pxActor)))
                continue;

            const size_t recordsIndex = (size_t)pxActor->userData;
            if (recordsIndex >= db.getRecords().size())
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
                                updateJointState(attachedStage, jointRecord, updateVelocitiesToUsd);
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
                            // Raw ObjectKey::handle -- see updateVehicleTransforms.
                            transformFn(actor->mKey.handle, fromPhysX(transform.p), fromPhysX(transform.q),
                                cbUserData);
                        }

                        if (!transform.isValid())
                        {
                            CARB_LOG_WARN("Invalid PhysX transform detected for %s.", dyna->getName());
                        }
                        else
                        {                         
                            if (!skipWriteTransforms && !(actor->mFlags & InternalActorFlag::eSKIP_UPDATE_TRANSFORM))
                            {
                                if (actor->mInstanceIndex == kInvalidUint32_t)
                                {
                                    if (updateToUsd && (actor->mFlags & InternalActorFlag::eUSE_DATAWRITE_SINK))
                                    {
                                        // Routed through IPhysicsDataWrite: accumulate the WORLD pose into the
                                        // batch flushed after the loop. The sink converts to local (authoring a
                                        // batch ancestor-first, so nested bodies see fresh parent poses) and
                                        // folds in the residual extra-transform.
                                        accumulateSinkTransform(transform, record.mKey, sinkKeys, sinkPositions,
                                                                sinkOrientations);
                                    }
                                }
                                // Point-instancer write-back. updateToUsd is force-false
                                // whenever there is no write sink (updateSimulationOutputs).
                                else
                                {
                                    if (updateToUsd)
                                    {
                                        if (actor->mInstanceKey != currInstancerKey)
                                        {
                                            // Per-call at Default; recomputed only when the instancer
                                            // changes (gated above), so the per-instancer cost is modest.
                                            // affineInverse, not inverseRT: an instancer can be scaled.
                                            currInstancerMatrixInverse = affineInverse(getWorldTransform(
                                                *attachedStage, actor->mInstanceKey, omni::physics::parse::ReadTime::defaultTime()));

                                            // Flush the previous instancer's accumulated positions/orientations
                                            // through the sink (velocities are flushed only at the end).
                                            flushInstancerArrays(currInstancerKey, havePositions, positionValues,
                                                                 haveOrientations, orientationValues, /*writeVel*/ false,
                                                                 haveLinVelAttr, linearVelocityValues, haveAngVelAttr,
                                                                 angularVelocityValues);

                                            omni::physics::parse::IPhysicsDataWrite* dw = attachedStage->getDataWrite();
                                            const omni::physics::parse::IPhysicsSource* source = attachedStage->getSource();
                                            if (!dw || !source)
                                            {
                                                CARB_LOG_ERROR_ONCE(
                                                    "Point instancer transform write-back requires a backing write "
                                                    "destination, skipping instancer: %s",
                                                    attachedStage->textFor(actor->mInstanceKey));
                                                continue;
                                            }

                                            omni::physics::parse::KnownTokens instTok;
                                            instTok.intern(*source);

                                            if (!source->isA(actor->mInstanceKey, instTok.pointInstancerType))
                                            {
                                                CARB_LOG_ERROR("Point instancer resolution failed on instancer: %s",
                                                               attachedStage->textFor(actor->mInstanceKey));
                                                continue;
                                            }

                                            havePositions = readInstancerArray(*attachedStage, actor->mInstanceKey,
                                                                               instTok.positions, positionValues, true);
                                            if (!havePositions)
                                            {
                                                CARB_LOG_ERROR("positions.Get() failed on instancer: %s",
                                                               attachedStage->textFor(actor->mInstanceKey));
                                                continue;
                                            }

                                            haveOrientations = readInstancerArray(*attachedStage, actor->mInstanceKey,
                                                                                  instTok.orientations, orientationValues, true);
                                            if (!haveOrientations)
                                            {
                                                CARB_LOG_ERROR("orientations.Get() failed on instancer: %s",
                                                               attachedStage->textFor(actor->mInstanceKey));
                                                continue;
                                            }

                                            if (updateVelocitiesToUsd)
                                            {
                                                // UsdGeomPointInstancer's schema always declares
                                                // velocities/angularVelocities once the instancer
                                                // itself resolved (the isA check above).
                                                haveLinVelAttr = true;
                                                haveAngVelAttr = true;

                                                readInstancerArray(*attachedStage, actor->mInstanceKey,
                                                                   instTok.velocities, linearVelocityValues, false);
                                                readInstancerArray(*attachedStage, actor->mInstanceKey,
                                                                   instTok.angularVelocities, angularVelocityValues, false);
                                            }

                                            currInstancerKey = actor->mInstanceKey;
                                        }

                                        isPointInstancer = true;

                                        // A.B. optimize this later, we should store the proto inverse matrices
                                        const PxMat44d trMatrix = makeMatrix(transform);
                                        // Gf `mProtoTransformInverse * trMatrix * currInstancerMatrixInverse`:
                                        // the operands reverse in PhysX's column-vector order.
                                        const PxMat44d writeMatrix =
                                            currInstancerMatrixInverse * trMatrix * actor->mProtoTransformInverse;

                                        uint32_t idx = actor->mInstanceIndex;
                                        // There might be more actors spawned in the PhysX scene than we had in the
                                        // initial data for the point instancer, this can for example happen if the user
                                        // spawned objects manually. The value a grow leaves in the skipped elements is
                                        // observable -- they are authored out with the rest of the array. Zero matches
                                        // what VtArray<GfVec3f> value-initialized to; the orientation buffer used to
                                        // grow with an UNinitialized GfQuath (its default constructor is
                                        // user-provided, so VtArray's `resize(n, value_type())` copied indeterminate
                                        // halves) and gets identity instead.
                                        if (positionValues.size() <= idx)
                                            positionValues.resize(idx + 1, PxVec3(PxZero));
                                        const PxVec3d writePos = writeMatrix.getPosition();
                                        positionValues[idx] =
                                            PxVec3(float(writePos.x), float(writePos.y), float(writePos.z));

                                        if (orientationValues.size() <= idx)
                                            orientationValues.resize(idx + 1, PxQuat(PxIdentity));
                                        // toTransform() discards the scale the proto/instancer inverses can carry
                                        // and returns a normalized rotation, as ExtractRotation().GetQuat() did.
                                        orientationValues[idx] = toTransform(writeMatrix).q;
                                    }
                                }
                            }

                            // If updateVelocitiesToUsd write velocity values to the
                            // corresponding write-sink attribute.
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
                                    if (haveLinVelAttr && linearVelocityValues.size() > 0)
                                    {
                                        const PxVec3 transformedVelocity =
                                            divideByScale(transform.q.rotateInv(linVel), actor->mScale);

                                        if (linearVelocityValues.size() <= idx)
                                            linearVelocityValues.resize(idx + 1, PxVec3(PxZero));

                                        linearVelocityValues[idx] = transformedVelocity;
                                    }

                                    if (haveAngVelAttr && angularVelocityValues.size() > 0)
                                    {
                                        if (angularVelocityValues.size() <= idx)
                                            angularVelocityValues.resize(idx + 1, PxVec3(PxZero));

                                        angularVelocityValues[idx] = radToDeg(transform.q.rotateInv(angVel));
                                    }
                                }
                                else
                                {
                                    if (haveLinVelAttr && linearVelocityValues.size() > 0)
                                    {
                                        if (linearVelocityValues.size() <= idx)
                                            linearVelocityValues.resize(idx + 1, PxVec3(PxZero));

                                        linearVelocityValues[idx] = linVel;
                                    }

                                    if (haveAngVelAttr && angularVelocityValues.size() > 0)
                                    {
                                        if (angularVelocityValues.size() <= idx)
                                            angularVelocityValues.resize(idx + 1, PxVec3(PxZero));

                                        angularVelocityValues[idx] = radToDeg(angVel);
                                    }
                                }
                            }
                            else
                            {
                                if (notifyVelocities ||
                                    (actor->mFlags & InternalActorFlag::eNOTIFY_VELOCITY && velocityFn))
                                {
                                    const PxVec3 linVel = dyna->getLinearVelocity();
                                    const PxVec3 angVel = velocitiesInRad ?
                                                              dyna->getAngularVelocity() :
                                                              radToDeg(dyna->getAngularVelocity());

                                    // Raw ObjectKey::handle, as for transformFn above.
                                    velocityFn(actor->mKey.handle, fromPhysX(linVel), fromPhysX(angVel), cbUserData);
                                }

                                if (updateVelocitiesToUsd && !skipWriteVelocities &&
                                    !(actor->mFlags & InternalActorFlag::eSKIP_UPDATE_VELOCITY))
                                {
                                    // Compute the values the sink will author to physicsVelocity /
                                    // physicsAngularVelocity (local-space + scale-divided when requested;
                                    // angular always in deg/s), then accumulate into the velocity batch.
                                    const PxVec3 linVel = dyna->getLinearVelocity();
                                    const PxVec3 angVel = dyna->getAngularVelocity();
                                    PxVec3 outLinear;
                                    PxVec3 outAngular;

                                    if (actor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES)
                                    {
                                        outLinear = divideByScale(transform.q.rotateInv(linVel), actor->mScale);
                                        outAngular = transform.q.rotateInv(angVel);
                                    }
                                    else
                                    {
                                        outLinear = linVel;
                                        outAngular = angVel;
                                    }
                                    outAngular = radToDeg(outAngular);

                                    velKeys.push_back(record.mKey);
                                    velLinear.push_back(outLinear);
                                    velAngular.push_back(outAngular);
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
    flushInstancerArrays(currInstancerKey, havePositions, positionValues, haveOrientations, orientationValues,
                         updateVelocitiesToUsd, haveLinVelAttr, linearVelocityValues, haveAngVelAttr,
                         angularVelocityValues);
}

void InternalScene::updateParticleTransforms(bool updateToUsd, bool updateVelocitiesToUsd, bool updateParticlesToUsd)
{
    // 0 is disabled, 1/2 is selected/all
    const bool debugVizEnabled = (OmniPhysX::getInstance().getCachedSettings().visualizationDisplayParticles > 0);

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteTransforms = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);

    for (size_t particleSystemIndex = 0; particleSystemIndex < mParticleSystems.size(); particleSystemIndex++)
    {
        InternalPbdParticleSystem* particleSystem = mParticleSystems[particleSystemIndex];

        // The postprocess registry is empty without a Kit viewport, so this returns eNone.
        uint32_t postFlags = particles::getPostprocessStages(particleSystem->mKey);
        bool particleSystemHasAnisotropy = postFlags & ParticlePostFlag::eAnisotropy;
        bool particleSystemHasSmoothing = postFlags & ParticlePostFlag::eSmoothing;
        bool particleSystemHasIsosurface = postFlags & ParticlePostFlag::eIsosurface;

        if (!particleSystem->mEnabled)
            continue;

        if (!particleSystem->mParticleDataAvailable)
        {
            continue;
        }
        else
        {
            particleSystem->mParticleDataAvailable = false;
        }

        // This sync must run whenever particle data is available, regardless of
        // whether there is anything to write back (updateToUsd/updateParticlesToUsd):
        // PxgParticleSystemCore's finalize stream is created CU_STREAM_NON_BLOCKING
        // (PxgParticleSystemCore.cpp), so it is NOT implicitly ordered against other
        // streams by CUDA's legacy-default-stream rules. PxScene::fetchResults() never
        // calls fetchResultsParticleSystem() itself (only OmniPVD sampling does), so
        // skipping this call is the only thing that keeps this stream's postprocess
        // (anisotropy/smoothing/isosurface) and position-finalize kernels synced with
        // any other reader of the particle GPU buffers. Gating it on updateToUsd used
        // to leave a stageless attach (no USD/ovstage data-write sink -- the case every
        // ovphysx/tensor and ovstage-stageless consumer is in) with no sync point at
        // all after simulate()+fetchResults(), racing a direct GPU readback of
        // PxParticleBuffer::getPositionInvMasses() against in-flight finalize-stream
        // work -- intermittently, since the race window widens with more postprocess
        // work queued on that stream.
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

        if (!updateToUsd && !updateParticlesToUsd)
            continue;

        PxVec4* anisotropyQ1 = nullptr;
        PxVec4* anisotropyQ2 = nullptr;
        PxVec4* anisotropyQ3 = nullptr;
        if (particleSystemHasAnisotropy)
        {
            particles::getAnisotropy(anisotropyQ1, anisotropyQ2, anisotropyQ3, particleSystem->mKey);
        }

        PxVec4* smoothedPos = nullptr;
        if (particleSystemHasSmoothing)
        {
            smoothedPos = particles::getSmoothedPositions(particleSystem->mKey);
        }


        // Diffuse-particle foam/spray rendering: Hydra-viewport-only, routed through
        // IPhysicsDataWrite::writeDiffuseParticlePoints below.
        // TODO preallocate/resize - this explicitly assumes 0 to clear if there are no diffuse particles
        std::vector<carb::Float3> tmpDiffuseParticlePoints;
        std::vector<carb::Float3> tmpDiffuseParticleColors;

        for (InternalParticleSet* particleSet : particleSystem->mParticleSets)
        {
            if (!particleSet->mNumParticles || !particleSet->mEnabled || !particleSet->mDownloadDirtyFlags)
                continue;

            AttachedStage& as = *UsdLoad::getUsdLoad()->getActiveAttachedStage();
            const omni::physics::parse::ObjectKey particleKey = particleSet->mKey;
            const omni::physics::parse::IPhysicsSource* source = as.getSource();

            // Interned once per particle set; see the matching comment in
            // resetStartProperties.
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            const bool isPointInstancer = source && source->isA(particleKey, tok.pointInstancerType);

            // transform particles from world space back to prim local space.
            // Inverted and applied in double precision, narrowed only per point --
            // see the matching site in resetStartProperties.
            const PxMat44d worldToLocal =
                affineInverse(getWorldTransform(as, particleKey, omni::physics::parse::ReadTime::defaultTime()));

            uint32_t flags = particleSet->mDownloadDirtyFlags;
            const omni::physics::parse::TokenId pointsToken = isPointInstancer ? tok.positions : tok.points;

            if (flags & ParticleDirtyFlags::eVELOCITY)
            {
                std::vector<carb::Float3> tmpVelocities;
                copyBuffer(tmpVelocities, (const carb::Float4*)particleSet->mVelocities, particleSet->mNumParticles);
                writeArrayToSink(particleKey, tok.velocities, tmpVelocities.data(),
                                 tmpVelocities.size(), omni::physics::parse::DataType::e32Bit);
            }

            if (flags & ParticleDirtyFlags::ePOSITION_INVMASS)
            {
                // means we downloaded positions - so if we also downloaded the smoothed pos, we write the positions into the simPos attribute.
                if (flags & ParticleDirtyFlags::eSMOOTHED_POSITIONS)
                {
                    std::vector<carb::Float3> tmpSimPositions;
                    copyBuffer(tmpSimPositions, particleSet->mPositions, particleSet->mNumParticles, worldToLocal);
                    writeArrayToSink(particleKey, tok.physxParticleSimulationPoints,
                                     tmpSimPositions.data(), tmpSimPositions.size(),
                                     omni::physics::parse::DataType::e32Bit);
                }
                else
                {
                    std::vector<carb::Float3> tmpPoints;
                    copyBuffer(tmpPoints, particleSet->mPositions, particleSet->mNumParticles, worldToLocal);
                    writeArrayToSink(particleKey, pointsToken, tmpPoints.data(), tmpPoints.size(),
                                     omni::physics::parse::DataType::e32Bit);
                }
            }

            if ((flags & ParticleDirtyFlags::eSMOOTHED_POSITIONS) && smoothedPos)
            {
                PxU32 start = particleSet->mParticleBuffer->getFlatListStartIndex();
                std::vector<carb::Float3> tmpPoints;
                copyBuffer(tmpPoints, &smoothedPos[start], particleSet->mNumParticles, worldToLocal);
                writeArrayToSink(particleKey, pointsToken, tmpPoints.data(), tmpPoints.size(),
                                 omni::physics::parse::DataType::e32Bit);
            }

            if ((flags & ParticleDirtyFlags::eANISOTROPY) && anisotropyQ1 && anisotropyQ2 && anisotropyQ3)
            {
                PxU32 start = particleSet->mParticleBuffer->getFlatListStartIndex();
                if (!isPointInstancer)
                {
                    // Anisotropy primvars are authored directly (ADR-0004), declared once
                    // at particle-set setup (UsdInterfaceParticle.cpp).
                    std::vector<carb::Float4> tmpValuesQ1(particleSet->mNumParticles);
                    std::vector<carb::Float4> tmpValuesQ2(particleSet->mNumParticles);
                    std::vector<carb::Float4> tmpValuesQ3(particleSet->mNumParticles);

                    for (PxU32 i = 0; i < particleSet->mNumParticles; ++i)
                    {
                        tmpValuesQ1[i] = toFloat4(anisotropyQ1[start + i]);
                        tmpValuesQ2[i] = toFloat4(anisotropyQ2[start + i]);
                        tmpValuesQ3[i] = toFloat4(anisotropyQ3[start + i]);
                    }

                    if (source)
                    {
                        writeArrayToSink(particleKey, source->internToken("anisotropyQ1"), tmpValuesQ1.data(),
                                         tmpValuesQ1.size(), omni::physics::parse::DataType::e32Bit);
                        writeArrayToSink(particleKey, source->internToken("anisotropyQ2"), tmpValuesQ2.data(),
                                         tmpValuesQ2.size(), omni::physics::parse::DataType::e32Bit);
                        writeArrayToSink(particleKey, source->internToken("anisotropyQ3"), tmpValuesQ3.data(),
                                         tmpValuesQ3.size(), omni::physics::parse::DataType::e32Bit);
                    }
                }
                else
                {
                    float contactDistanceInv = 1.0f / (particleSystem->mPS->getParticleContactOffset() * 2.0f);
                    std::vector<carb::Float3> tmpScales(particleSet->mNumParticles);
                    std::vector<PxQuat> tmpOrientations(particleSet->mNumParticles);

                    for (PxU32 i = start; i < start + particleSet->mNumParticles; i++)
                    {
                        PxVec4 q1 = anisotropyQ1[i];
                        PxVec4 q2 = anisotropyQ2[i];
                        PxVec4 q3 = anisotropyQ3[i];
                        tmpScales[i - start] = { 4.0f * q1[3] * contactDistanceInv, 4.0f * q2[3] * contactDistanceInv, 4.0f * q3[3] * contactDistanceInv };
                        tmpOrientations[i - start] = PxQuat(PxMat33(q1.getXYZ(), q2.getXYZ(), q3.getXYZ()));
                    }

                    writeArrayToSink(particleKey, tok.scales, tmpScales.data(), tmpScales.size(),
                                     omni::physics::parse::DataType::e32Bit);
                    writeArrayToSink(particleKey, tok.orientations, tmpOrientations.data(),
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
                    tmpDiffuseParticlePoints[currentSize + i] = carb::Float3{ p.x, p.y, p.z };
                    tmpDiffuseParticleColors[currentSize + i] = carb::Float3{ 1.0f, 1.0f, 1.0f };
                }
            }

            particleSet->mDownloadDirtyFlags = 0;
        }

        if (AttachedStage* diffuseAttachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage())
        {
            if (omni::physics::parse::IPhysicsDataWrite* dw = diffuseAttachedStage->getDataWrite())
            {
                dw->writeDiffuseParticlePoints(particleSystem->mKey, tmpDiffuseParticlePoints.data(),
                                               tmpDiffuseParticlePoints.size(), tmpDiffuseParticleColors.data(),
                                               tmpDiffuseParticleColors.size());
            }
        }

        /* Isosurface */
        if ((updateToUsd || debugVizEnabled) && particleSystemHasIsosurface)
        {
            particles::updateIsosurfaceMesh(particleSystem->mKey);
        }

    }
}

void InternalScene::updateDeformableTransforms(bool updateToUsd, bool updateVelocitiesToUsd)
{
    if (mVolumeDeformableBodies.empty() && mSurfaceDeformableBodies.empty())
        return;

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteTransforms = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);
    if (skipWriteTransforms)
        return;

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager || !cudaContextManager->getCudaContext())
    {
        CARB_LOG_WARN_ONCE(
            OMNI_LOG_DEFAULT_CHANNEL,
            "InternalScene::updateDeformableTransforms: CUDA context unavailable, skipping.");
        return;
    }

    // Deformable bodies store source-agnostic ObjectKeys; resolve mesh prims via
    // the active stage during this write-back.
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    const omni::physics::parse::IPhysicsSource* deformableSource =
        attachedStage ? attachedStage->getSource() : nullptr;
    omni::physics::parse::KnownTokens tok;
    if (deformableSource)
        tok.intern(*deformableSource);

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
                const PxMat44d& worldToSkinMesh = deformableBody->mWorldToSkinMeshTransforms[i];
                // The authored array supplies the expected length; the transformed
                // points are staged in the engine's own buffer and published as an
                // untyped column.
                std::vector<carb::Float3> points;
                getArrayValue(*attachedStage, skinKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);
                if (points.size() == range.y)
                {
                    std::vector<carb::Float3> outPoints;
                    copyBuffer(outPoints, srcPtr + range.x, range.y, worldToSkinMesh);
                    writeMeshPointsToSink(skinKey, outPoints.data(), outPoints.size());
                }
            }
        }

        {
            const omni::physics::parse::ObjectKey simKey = deformableBody->mSimMeshKey;
            const PxMat44d& worldToSimMesh = deformableBody->mWorldToSimMesh;
            std::vector<carb::Float3> points;
            {
                const ::physx::PxVec4* srcPtr = deformableBody->mSimMeshPositionInvMassH;
                const uint32_t srcSize = deformableBody->mNumSimMeshVertices;
                getArrayValue(*attachedStage, simKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);
                if (points.size() == srcSize)
                {
                    std::vector<carb::Float3> outPoints;
                    copyBuffer(outPoints, srcPtr, srcSize, worldToSimMesh);
                    writeMeshPointsToSink(simKey, outPoints.data(), outPoints.size());
                }
            }

            if (updateVelocitiesToUsd)
            {
                const ::physx::PxVec4* srcPtr = deformableBody->mSimMeshVelocityH;
                const uint32_t srcSize = deformableBody->mNumSimMeshVertices;
                if (points.size() == srcSize)
                {
                    std::vector<carb::Float3> velocities;
                    copyBuffer(velocities, srcPtr, srcSize);
                    writeMeshVelocitiesToSink(simKey, velocities.data(), velocities.size());
                }
            }

            const omni::physics::parse::ObjectKey collKey = deformableBody->mCollMeshKey;
            const PxMat44d& worldToCollMesh = deformableBody->mWorldToCollMesh;
            if (collKey != simKey)
            {
                const ::physx::PxVec4* srcPtr = deformableBody->mCollMeshPositionInvMassH;
                const uint32_t srcSize = deformableBody->mNumCollMeshVertices;
                std::vector<carb::Float3> collPoints;
                getArrayValue(*attachedStage, collKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), collPoints);
                if (collPoints.size() == srcSize)
                {
                    std::vector<carb::Float3> outCollPoints;
                    copyBuffer(outCollPoints, srcPtr, srcSize, worldToCollMesh);
                    writeMeshPointsToSink(collKey, outCollPoints.data(), outCollPoints.size());
                }
            }

            const PxBounds3 worldBounds = deformableBody->mDeformableVolume->getWorldBounds();
            carb::Float3 extent[2];
            computeTransformedExtent(worldBounds, worldToCollMesh, extent);
            writeMeshExtentToSink(collKey, extent, 2);
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
                const PxMat44d& worldToSkinMesh = deformableBody->mWorldToSkinMeshTransforms[i];
                // See the volume-deformable equivalent above.
                std::vector<carb::Float3> points;
                getArrayValue(*attachedStage, skinKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);
                if (points.size() == range.y)
                {
                    std::vector<carb::Float3> outPoints;
                    copyBuffer(outPoints, srcPtr + range.x, range.y, worldToSkinMesh);
                    writeMeshPointsToSink(skinKey, outPoints.data(), outPoints.size());
                }
            }
        }

        {
            const omni::physics::parse::ObjectKey simKey = deformableBody->mSimMeshKey;
            const PxMat44d& worldToSimMesh = deformableBody->mWorldToSimMesh;
            std::vector<carb::Float3> points;
            {
                const ::physx::PxVec4* srcPtr = deformableBody->mSimMeshPositionInvMassH;
                const uint32_t srcSize = deformableBody->mNumSimMeshVertices;
                getArrayValue(*attachedStage, simKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);
                if (points.size() == srcSize)
                {
                    std::vector<carb::Float3> outPoints;
                    copyBuffer(outPoints, srcPtr, uint32_t(points.size()), worldToSimMesh);
                    writeMeshPointsToSink(simKey, outPoints.data(), outPoints.size());
                }
            }

            if (updateVelocitiesToUsd)
            {
                const ::physx::PxVec4* srcPtr = deformableBody->mSimMeshVelocityH;
                const uint32_t srcSize = deformableBody->mNumSimMeshVertices;
                if (points.size() == srcSize)
                {
                    std::vector<carb::Float3> velocities;
                    copyBuffer(velocities, srcPtr, srcSize);
                    writeMeshVelocitiesToSink(simKey, velocities.data(), velocities.size());
                }
            }

            {
                const PxBounds3 worldBounds = deformableBody->mDeformableSurface->getWorldBounds();
                carb::Float3 extent[2];
                computeTransformedExtent(worldBounds, worldToSimMesh, extent);
                writeMeshExtentToSink(simKey, extent, 2);
            }
        }
    }
}

// Declared in InternalScene.h. The definition must be namespace-qualified: a
// "using namespace omni::physx::internal;" in scope would silently define an unrelated
// global-scope function and leave the declared one undefined at link time.
const char* omni::physx::internal::jointStateAxisName(usdparser::ObjectType jointType, ::physx::PxArticulationAxis::Enum physxAxis)
{
    switch (jointType)
    {
        case usdparser::eJointPrismatic:
            return "linear";
        case usdparser::eJointD6:
            switch (physxAxis)
            {
                case ::physx::PxArticulationAxis::eX: return "transX";
                case ::physx::PxArticulationAxis::eY: return "transY";
                case ::physx::PxArticulationAxis::eZ: return "transZ";
                case ::physx::PxArticulationAxis::eSWING1: return "rotY";
                case ::physx::PxArticulationAxis::eSWING2: return "rotZ";
                default: return "rotX"; // eTWIST and any other axis
            }
        case usdparser::eJointRevolute:
        default:
            return "angular";
    }
}

// Publishes PhysxJointStateAPI Position/Velocity through IPhysicsDataWrite; a no-op
// when there is no write sink.
void InternalScene::updateJointState(AttachedStage* attachedStage, const InternalDatabase::Record& record, bool updateVelocitiesToUsd)
{
    omni::physics::parse::IPhysicsDataWrite* dw = attachedStage ? attachedStage->getDataWrite() : nullptr;
    if (!dw)
        return;
    omni::physics::parse::IPhysicsSource* src = attachedStage->getSource();

    InternalJoint* intJoint = (InternalJoint*)record.mInternalPtr;
    ::physx::PxArticulationJointReducedCoordinate* joint = (::physx::PxArticulationJointReducedCoordinate*)record.mPtr;

    for (size_t idx = 0; idx < 6; ++idx)
    {
        InternalJoint::InternalJointState& intJointState = intJoint->mJointStates[idx];
        if (!intJointState.enabled)
            continue;

        const std::string axisName = jointStateAxisName(intJoint->mJointType, intJointState.physxAxis);

        const float articulationPos = intJoint->getArticulationJointPosition(joint, intJointState.physxAxis);
        const float positionValue = intJointState.convertToDegrees ? radToDeg(articulationPos) : articulationPos;
        const omni::physics::parse::TokenId posAttr = src->internToken("state:" + axisName + ":physics:position");
        dw->writeData(&record.mKey, 1, posAttr,
                     omni::physics::parse::DataWriteView{ &positionValue, 1, 0, -1, omni::physics::parse::DataType::e32Bit });

        if (updateVelocitiesToUsd)
        {
            const float articulationVel = intJoint->getArticulationJointVelocity(joint, intJointState.physxAxis);
            const float velocityValue = intJointState.convertToDegrees ? radToDeg(articulationVel) : articulationVel;
            const omni::physics::parse::TokenId velAttr = src->internToken("state:" + axisName + ":physics:velocity");
            dw->writeData(&record.mKey, 1, velAttr,
                         omni::physics::parse::DataWriteView{ &velocityValue, 1, 0, -1, omni::physics::parse::DataType::e32Bit });
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

    // Session-layer edit-context/change-block scaffolding around a full simulation-output
    // flush; a no-op on a backend without this concept. rawLayer() is a non-owning peek:
    // omniPhysX.mSimulationLayer's own reference keeps the layer alive for this call, and
    // beginFrameWrite's USD implementation takes its own ref-counting reference.
    {
        const SimulationLayerHandle simLayer = omniPhysX.getSimulationLayer();
        void* simLayerHandle = simLayer.rawLayer();

        AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        omni::physics::parse::IPhysicsDataWrite* dw = attachedStage ? attachedStage->getDataWrite() : nullptr;

        if (dw)
            dw->beginFrameWrite(simLayerHandle);

        {
            CARB_PROFILE_ZONE(0, "updateRenderTransforms::USDWrite");
            updateRigidBodyTransforms(updateToUsd, updateVelocitiesToUsd, outputVelocitiesLocalSpace);
            updateCctTransforms(updateToUsd);
            updateVehicleTransforms(updateToUsd);
            updateParticleTransforms(updateToUsd, updateVelocitiesToUsd, updateParticlesToUsd);
            updateDeformableTransforms(updateToUsd, updateVelocitiesToUsd);
        }

        if (dw)
            dw->endFrameWrite();
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


void InternalJoint::setArticulationDrivePositionTarget(::physx::PxArticulationJointReducedCoordinate* joint, ::physx::PxArticulationAxis::Enum axis, float positionTarget, omni::physics::parse::ObjectKey jointKey) const
{
    if (positionTarget >= (2.0f * M_PI) || positionTarget <= -(2.0f * M_PI))
    {
        // createObject inside UsdInterface.cpp will only create eREVOLUTE joint if no limits exists
        // so driving to any angle close to 360 will be clamped / wrapped around
        const PxArticulationJointType::Enum type = joint->getJointType();
        if(type == PxArticulationJointType::eREVOLUTE)
        {
            const float targetPosition = radToDeg(positionTarget);
            const AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
            OMNI_LOG_WARN(
            kRoboticsLogChannel,
            "Physics USD: Drive position target set to %2.f on %s will be wrapped in [-360, 360] range."
            "Consider setting explicit limits to enable use of unwrapped joints",
            targetPosition, as ? as->textFor(jointKey) : "");
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
