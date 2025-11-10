// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "InternalAttachmentDeprecated.h"
#include "InternalDeformableDeprecated.h"
#include "InternalParticle.h"
#include "InternalScene.h"
#include <PhysXTools.h>

#include <common/utilities/MemoryMacros.h>

#if USE_PHYSX_GPU
#include "extensions/PxParticleExt.h"
#endif

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace carb;
using namespace ::physx;

extern ObjectId getObjectId(const pxr::SdfPath& path, PhysXType type);

InternalAttachmentDeprecated::InternalAttachmentDeprecated(InternalDeformableBodyDeprecated* deformableBody, ::physx::PxRigidActor* rigidActor, usdparser::ObjectId shapeId)
{
    mAttachmentType = AttachmentType::eDeformableBodyRigidBody;
    mAttachmentActor[0].mObjectInternalPtr = deformableBody;
    mAttachmentActor[0].mObjectId = getObjectId(deformableBody->mPrim.GetPath(), ePTSoftBodyDeprecated);
    mAttachmentActor[1].mRigidActor = rigidActor;
    mAttachmentActor[1].mShapeId = shapeId;
    mAttachmentActor[1].mObjectId = ObjectId(rigidActor->userData);

    if (shapeId != kInvalidObjectId)
    {
        PxShape* shape = getPtr<PxShape>(ePTShape, shapeId);
        mInitialTransformLocal = shape->getLocalPose();
        mNewTransformLocal = mInitialTransformLocal;
    }
}

InternalAttachmentDeprecated::InternalAttachmentDeprecated(InternalParticleClothDeprecated* particleCloth, ::physx::PxRigidActor* rigidActor, usdparser::ObjectId shapeId)
{
    mAttachmentType = AttachmentType::eParticleClothRigidBody;
    mAttachmentActor[0].mObjectInternalPtr = particleCloth;
    mAttachmentActor[0].mObjectId = getObjectId(particleCloth->mPrim.GetPath(), ePTParticleClothDeprecated);
    mAttachmentActor[1].mRigidActor = rigidActor;
    mAttachmentActor[1].mShapeId = shapeId;
    mAttachmentActor[1].mObjectId = ObjectId(rigidActor->userData);

    if (shapeId != kInvalidObjectId)
    {
        PxShape* shape = getPtr<PxShape>(ePTShape, shapeId);
        mInitialTransformLocal = shape->getLocalPose();
        mNewTransformLocal = mInitialTransformLocal;
    }
}

InternalAttachmentDeprecated::InternalAttachmentDeprecated(InternalDeformableSurfaceDeprecated* deformableSurface, ::physx::PxRigidActor* rigidActor, usdparser::ObjectId shapeId)
{
    mAttachmentType = AttachmentType::eDeformableSurfaceRigidBody;
    mAttachmentActor[0].mObjectInternalPtr = deformableSurface;
    mAttachmentActor[0].mObjectId = getObjectId(deformableSurface->mPrim.GetPath(), ePTFEMClothDeprecated);
    mAttachmentActor[1].mRigidActor = rigidActor;
    mAttachmentActor[1].mShapeId = shapeId;
    mAttachmentActor[1].mObjectId = ObjectId(rigidActor->userData);

    if (shapeId != kInvalidObjectId)
    {
        PxShape* shape = getPtr<PxShape>(ePTShape, shapeId);
        mInitialTransformLocal = shape->getLocalPose();
        mNewTransformLocal = mInitialTransformLocal;
    }
}

InternalAttachmentDeprecated::InternalAttachmentDeprecated(InternalDeformableBodyDeprecated* deformableBody0, InternalDeformableBodyDeprecated* deformableBody1)
{
    mAttachmentType = AttachmentType::eDeformableBodyDeformableBody;
    mAttachmentActor[0].mObjectInternalPtr = deformableBody0;
    mAttachmentActor[0].mObjectId = getObjectId(deformableBody0->mPrim.GetPath(), ePTSoftBodyDeprecated);
    mAttachmentActor[1].mObjectInternalPtr = deformableBody1;
    mAttachmentActor[1].mObjectId = getObjectId(deformableBody1->mPrim.GetPath(), ePTSoftBodyDeprecated);
}

InternalAttachmentDeprecated::InternalAttachmentDeprecated(InternalDeformableBodyDeprecated* deformableBody, InternalParticleClothDeprecated* particleCloth)
{
    mAttachmentType = AttachmentType::eDeformableBodyParticleCloth;
    mAttachmentActor[0].mObjectInternalPtr = deformableBody;
    mAttachmentActor[0].mObjectId = getObjectId(deformableBody->mPrim.GetPath(), ePTSoftBodyDeprecated);
    mAttachmentActor[1].mObjectInternalPtr = particleCloth;
    mAttachmentActor[1].mObjectId = getObjectId(particleCloth->mPrim.GetPath(), ePTParticleClothDeprecated);
}

InternalAttachmentDeprecated::InternalAttachmentDeprecated(InternalDeformableBodyDeprecated* deformableBody, InternalDeformableSurfaceDeprecated* deformableSurface)
{
    mAttachmentType = AttachmentType::eDeformableBodyDeformableSurface;
    mAttachmentActor[0].mObjectInternalPtr = deformableBody;
    mAttachmentActor[0].mObjectId = getObjectId(deformableBody->mPrim.GetPath(), ePTSoftBodyDeprecated);
    mAttachmentActor[1].mObjectInternalPtr = deformableSurface;
    mAttachmentActor[1].mObjectId = getObjectId(deformableSurface->mPrim.GetPath(), ePTFEMClothDeprecated);
}

InternalAttachmentDeprecated::InternalAttachmentDeprecated(InternalParticleClothDeprecated* particleCloth)
{
    mAttachmentType = AttachmentType::eParticleClothStatic;
    mAttachmentActor[0].mObjectInternalPtr = particleCloth;
    mAttachmentActor[0].mObjectId = getObjectId(particleCloth->mPrim.GetPath(), ePTParticleClothDeprecated);
}

InternalAttachmentDeprecated::InternalAttachmentDeprecated(InternalDeformableBodyDeprecated* deformableBody)
{
    mAttachmentType = AttachmentType::eDeformableBodyStatic;
    mAttachmentActor[0].mObjectInternalPtr = deformableBody;
    mAttachmentActor[0].mObjectId = getObjectId(deformableBody->mPrim.GetPath(), ePTSoftBodyDeprecated);
}

InternalAttachmentDeprecated::InternalAttachmentDeprecated(InternalDeformableSurfaceDeprecated* deformableSurface)
{
    mAttachmentType = AttachmentType::eDeformableSurfaceStatic;
    mAttachmentActor[0].mObjectInternalPtr = deformableSurface;
    mAttachmentActor[0].mObjectId = getObjectId(deformableSurface->mPrim.GetPath(), ePTFEMClothDeprecated);
}

InternalAttachmentDeprecated::~InternalAttachmentDeprecated()
{
    // Make sure that internal scene is valid
    if (!mInternalScene)
    {
        CARB_LOG_WARN("Deleted attachment %s does not have a valid internal scene pointer. This could potentially result in stale attachment points.\n", mPath.GetText());
        return;
    }

    releaseDeformableRigidBodyAttachment();

    if (mAttachmentType == eDeformableBodyDeformableBody)
    {
        PxSoftBody* softBody = ((InternalDeformableBodyDeprecated*)(mAttachmentActor[0].mObjectInternalPtr))->mSoftBody;

        if (softBody->getScene())
        {
            SAFE_RELEASE(mDeformableAttachment);
            SAFE_RELEASE(mDeformableFilter);
        }
    }

    if (mAttachmentType == eDeformableBodyParticleCloth)
    {
        PxSoftBody* softBody = ((InternalDeformableBodyDeprecated*)(mAttachmentActor[0].mObjectInternalPtr))->mSoftBody;
        InternalParticleClothDeprecated* internalParticleCloth = (InternalParticleClothDeprecated*)(mAttachmentActor[1].mObjectInternalPtr);

        for (uint32_t i = 0; i < mAttachmentActor[0].mTetraFilterIds.size(); ++i)
        {
            unsigned int tetId = mAttachmentActor[0].mTetraFilterIds[i];
            unsigned int vertexId = mAttachmentActor[1].mParticleFilterIds[i];
            if (softBody->getScene())
                softBody->removeParticleFilter(internalParticleCloth->mParentParticleSystem->mPS, internalParticleCloth->mClothBuffer, vertexId, tetId);
        }

        for (uint32_t i = 0; i < mAttachmentActor[0].mTetraAttachmentHandles.size(); ++i)
        {
            unsigned int handle = mAttachmentActor[0].mTetraAttachmentHandles[i];
            if (softBody->getScene())
                softBody->removeParticleAttachment(internalParticleCloth->mParentParticleSystem->mPS, handle);
        }
    }

    if (mAttachmentType == eDeformableBodyDeformableSurface)
    {
        PxSoftBody* softBody = ((InternalDeformableBodyDeprecated*)(mAttachmentActor[0].mObjectInternalPtr))->mSoftBody;

        if (softBody->getScene())
        {
            SAFE_RELEASE(mDeformableAttachment);
            SAFE_RELEASE(mDeformableFilter);
        }
    }

    if (mAttachmentType == eParticleClothStatic)
    {
        InternalParticleClothDeprecated* internalParticleCloth = (InternalParticleClothDeprecated*)(mAttachmentActor[0].mObjectInternalPtr);
        PxRigidActor* rigidActor = mAttachmentActor[0].mRigidActor;

        for (uint32_t i = 0; i < mAttachmentActor[0].mParticleAttachmentIds.size(); ++i)
        {
            unsigned int particleId = mAttachmentActor[0].mParticleAttachmentIds[i];
            if (internalParticleCloth->mParentParticleSystem->mPS->getScene())
                internalParticleCloth->removeRigidAttachment(rigidActor, particleId);
        }
#if USE_PHYSX_GPU
        if (internalParticleCloth->mAttachments)
            internalParticleCloth->mAttachments->copyToDevice();
#endif

        if (rigidActor->getScene())
            rigidActor->getScene()->removeActor(*rigidActor);
        rigidActor->release();
    }

    if (mAttachmentType == eDeformableBodyStatic)
    {
        PxSoftBody* softBody = ((InternalDeformableBodyDeprecated*)(mAttachmentActor[0].mObjectInternalPtr))->mSoftBody;
        if (softBody->getScene())
        {
            SAFE_RELEASE(mDeformableAttachment);
        }

        PxRigidActor* rigidActor = mAttachmentActor[0].mRigidActor;
        if (rigidActor->getScene())
        {
            rigidActor->getScene()->removeActor(*rigidActor);
        }
        SAFE_RELEASE(rigidActor);
    }

    if (mAttachmentType == eDeformableSurfaceStatic)
    {
        PxDeformableSurface* deformableSurface = ((InternalDeformableSurfaceDeprecated*)(mAttachmentActor[0].mObjectInternalPtr))->mDeformableSurface;
        if (deformableSurface->getScene())
        {
            SAFE_RELEASE(mDeformableAttachment);
        }

        PxRigidActor* rigidActor = mAttachmentActor[0].mRigidActor;
        if (rigidActor->getScene())
        {
            rigidActor->getScene()->removeActor(*rigidActor);
        }
        SAFE_RELEASE(rigidActor);
    }
}

bool isValidRigidBody(PxScene* scene, const PxRigidActor* actor)
{
    if (scene)
    {
        const uint32_t numActors = scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC);
        if (numActors)
        {
            std::vector<PxActor*> userBufferActors(numActors);
            scene->getActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC, userBufferActors.data(), (PxU32)userBufferActors.size(), 0);

            for (uint32_t i = 0; i < userBufferActors.size(); i++)
            {
                if (userBufferActors[i]->getType() == PxActorType::eRIGID_STATIC || userBufferActors[i]->getType() == PxActorType::eRIGID_DYNAMIC)
                {
                    PxRigidActor* rigidActor = (PxRigidActor*)userBufferActors[i];
                    if (rigidActor == actor)
                    {
                        return true;
                    }
                }
            }
        }

        // check articulation links (currently articulation links are not supported in multi scenes)
        const uint32_t numArticulations = scene->getNbArticulations();
        if (numArticulations)
        {
            std::vector<PxArticulationReducedCoordinate*> userBufferArticulations(numArticulations);
            scene->getArticulations(userBufferArticulations.data(), (PxU32)userBufferArticulations.size());

            for (uint32_t i = 0; i < userBufferArticulations.size(); i++)
            {
                PxArticulationReducedCoordinate* articulationReducedCoordinate = userBufferArticulations[i];
                const uint32_t numLinks = articulationReducedCoordinate->getNbLinks();

                if (numLinks)
                {
                    std::vector<PxArticulationLink*> userBufferArticulationLinks(numLinks);
                    articulationReducedCoordinate->getLinks(userBufferArticulationLinks.data(), (PxU32)userBufferArticulationLinks.size());

                    for (uint32_t j = 0; j < userBufferArticulationLinks.size(); j++)
                    {
                        PxRigidActor* rigidActor = (PxRigidActor*)userBufferArticulationLinks[j];
                        if (rigidActor == actor)
                        {
                            return true;
                        }
                    }
                }
            }
        }
    }

    return false;
}

bool InternalAttachmentDeprecated::isValidParticleSystem()
{
    InternalPbdParticleSystem* internalParticleSystem = ((InternalParticleClothDeprecated*)(mAttachmentActor[0].mObjectInternalPtr))->mParentParticleSystem;

    for (InternalPbdParticleSystem* internalPS : mInternalScene->mParticleSystems)
    {
        if (internalPS == internalParticleSystem)
        {
            return true;
        }
    }

    return false;
}

void InternalAttachmentDeprecated::releaseDeformableRigidBodyAttachment()
{
    if (mAttachmentType == eDeformableBodyRigidBody)
    {
        PxSoftBody* softBody = ((InternalDeformableBodyDeprecated*)(mAttachmentActor[0].mObjectInternalPtr))->mSoftBody;

        if (isValidRigidBody(softBody->getScene(), mAttachmentActor[1].mRigidActor))
        {
            SAFE_RELEASE(mDeformableAttachment);
            SAFE_RELEASE(mDeformableFilter);
        }
    }

    if (mAttachmentType == eParticleClothRigidBody)
    {
        InternalParticleClothDeprecated* internalParticleCloth = (InternalParticleClothDeprecated*)(mAttachmentActor[0].mObjectInternalPtr);

        if (isValidParticleSystem())
        {
            if (isValidRigidBody(internalParticleCloth->mParentParticleSystem->mPS->getScene(), mAttachmentActor[1].mRigidActor))
            {
                for (uint32_t i = 0; i < mAttachmentActor[0].mParticleFilterIds.size(); ++i)
                {
                    unsigned int particleId = mAttachmentActor[0].mParticleFilterIds[i];
                    internalParticleCloth->removeRigidFilter(mAttachmentActor[1].mRigidActor, particleId);
                }

                for (uint32_t i = 0; i < mAttachmentActor[0].mParticleAttachmentIds.size(); ++i)
                {
                    unsigned int particleId = mAttachmentActor[0].mParticleAttachmentIds[i];
                    internalParticleCloth->removeRigidAttachment(mAttachmentActor[1].mRigidActor, particleId);
                }
            }

#if USE_PHYSX_GPU
            if (internalParticleCloth->mAttachments)
                internalParticleCloth->mAttachments->copyToDevice();
#endif
        }
    }

    if (mAttachmentType == eDeformableSurfaceRigidBody)
    {
        PxDeformableSurface* deformableSurface = ((InternalDeformableSurfaceDeprecated*)(mAttachmentActor[0].mObjectInternalPtr))->mDeformableSurface;

        if (isValidRigidBody(deformableSurface->getScene(), mAttachmentActor[1].mRigidActor))
        {
            SAFE_RELEASE(mDeformableAttachment);
            SAFE_RELEASE(mDeformableFilter);
        }
    }
}

bool InternalAttachmentDeprecated::hasActor(const ::physx::PxRigidActor* rigidActor) const
{
    if (mAttachmentType == eDeformableBodyRigidBody || mAttachmentType == eDeformableSurfaceRigidBody ||
        mAttachmentType == eParticleClothRigidBody)
    {
        if (mAttachmentActor[1].mRigidActor == rigidActor)
            return true;
    }
    return false;
}

bool InternalAttachmentDeprecated::hasShape(const ObjectId shapeId) const
{
    if (mAttachmentType == eDeformableBodyRigidBody || mAttachmentType == eDeformableSurfaceRigidBody ||
        mAttachmentType == eParticleClothRigidBody)
    {
        if (mAttachmentActor[1].mShapeId == shapeId)
            return true;
    }
    return false;
}

void InternalAttachmentDeprecated::swapRigidActor(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor)
{
    if (hasActor(sourceActor))
    {
        // Remove attachments from old rigid actor
        mDirtyEvent = eAttachmentRemove;
        refreshAttachment();

        // Set pending event for new rigid actor
        mAttachmentActor[1].mRigidActor = destActor;
        mDirtyEvent = InternalAttachmentDeprecated::eAttachmentAdd;
    }
}

void InternalAttachmentDeprecated::setRefreshAttachmentEvent(::physx::PxRigidActor* rigidActor)
{
    if (hasActor(rigidActor))
    {
        if (mDirtyEvent == eNone)
        {
            mDirtyEvent = eRefreshAttachment;
        }
    }
}

void InternalAttachmentDeprecated::refreshAttachment()
{
    if (mDirtyEvent & eRefreshAttachment)
    {
        AttachmentActor& actor0 = mAttachmentActor[0];
        AttachmentActor& actor1 = mAttachmentActor[1];
        PxRigidActor* rigidActor = actor1.mRigidActor;

        // Handle changes to shape local pose
        if (!(mInitialTransformLocal == mNewTransformLocal) && (mDirtyEvent & eAttachmentAdd))
        {
            PxTransform transformLocal = mInitialTransformLocal.getInverse() * mNewTransformLocal;

            for (uint32_t i = 0; i < actor1.mRigidActorPositions.size(); ++i)
            {
                PxVec3& pos = (PxVec3&)actor1.mRigidActorPositions[i];
                pos = transformLocal.transform(pos);
            }

            mInitialTransformLocal = mNewTransformLocal;
        }

        if (mAttachmentType == eDeformableBodyRigidBody)
        {
            InternalDeformableBodyDeprecated* deformableBody = (InternalDeformableBodyDeprecated*)(actor0.mObjectInternalPtr);

            if (mDirtyEvent & eAttachmentRemove)
            {
                SAFE_RELEASE(mDeformableAttachment);
                SAFE_RELEASE(mDeformableFilter);
            }

            if (mDirtyEvent & eAttachmentAdd)
            {
                mDeformableAttachment = deformableBody->addRigidAttachments(actor1.mRigidActor, actor0.mDeformableAttachmentIndices, actor0.mDeformableAttachmentCoords, actor1.mDeformableAttachmentCoords);
                mDeformableFilter = deformableBody->addRigidFilters(actor1.mRigidActor, actor0.mDeformableFilterIndices);
            }
        }
        else if (mAttachmentType == eDeformableSurfaceRigidBody)
        {
            InternalDeformableSurfaceDeprecated* deformableSurface = (InternalDeformableSurfaceDeprecated*)(actor0.mObjectInternalPtr);

            if (mDirtyEvent & eAttachmentRemove)
            {
                SAFE_RELEASE(mDeformableAttachment);
                SAFE_RELEASE(mDeformableFilter);
            }

            if (mDirtyEvent & eAttachmentAdd)
            {
                mDeformableAttachment = deformableSurface->addRigidAttachments(actor1.mRigidActor, actor0.mDeformableAttachmentIndices, actor1.mDeformableAttachmentCoords);
                mDeformableFilter = deformableSurface->addRigidFilters(actor1.mRigidActor, actor0.mDeformableFilterIndices);
            }
        }
        else if (mAttachmentType == eParticleClothRigidBody && isValidParticleSystem())
        {
            InternalParticleClothDeprecated* internalParticleCloth = (InternalParticleClothDeprecated*)(actor0.mObjectInternalPtr);

            if (mDirtyEvent & eAttachmentRemove)
            {
                for (uint32_t i = 0; i < actor0.mParticleFilterIds.size(); ++i)
                {
                    unsigned int particleId = actor0.mParticleFilterIds[i];
                    internalParticleCloth->removeRigidFilter(rigidActor, particleId);
                }
                for (uint32_t i = 0; i < actor0.mParticleAttachmentIds.size(); ++i)
                {
                    unsigned int particleId = actor0.mParticleAttachmentIds[i];
                    internalParticleCloth->removeRigidAttachment(rigidActor, particleId);
                }
            }

            if (mDirtyEvent & eAttachmentAdd)
            {
                for (uint32_t i = 0; i < actor0.mParticleFilterIds.size(); ++i)
                {
                    unsigned int particleId = actor0.mParticleFilterIds[i];
                    internalParticleCloth->addRigidFilter(rigidActor, particleId);
                }
                for (uint32_t i = 0; i < actor0.mParticleAttachmentIds.size(); ++i)
                {
                    unsigned int particleId = actor0.mParticleAttachmentIds[i];
                    internalParticleCloth->addRigidAttachment(rigidActor, particleId, (PxVec3&)actor1.mRigidActorPositions[i]);
                }
            }

#if USE_PHYSX_GPU
            if (internalParticleCloth->mAttachments)
                internalParticleCloth->mAttachments->copyToDevice();
#endif
        }
    }

    mDirtyEvent = eNone;
}
