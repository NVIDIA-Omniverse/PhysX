// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "UsdInterface.h"

#include <private/omni/physx/PhysxUsd.h>

#include <usdLoad/LoadUsd.h>

#include <internal/Internal.h>
#include <internal/InternalAttachmentDeprecated.h>
#include <internal/InternalDeformableDeprecated.h>
#include <internal/InternalParticle.h>

#include <Setup.h>
#include <OmniPhysX.h>
#include <PhysXTools.h>
#include <ObjectDataQuery.h>
#include <attachment/PhysXAttachmentDeprecated.h>
#include <attachment/PhysXTetFinder.h>
#include <attachment/PhysXPointFinder.h>
#include <attachment/PhysXTriFinder.h>

#include <common/utilities/MemoryMacros.h>

#if USE_PHYSX_GPU
#include "extensions/PxParticleExt.h"
#endif

using namespace omni::physx::usdparser;
using namespace pxr;
using namespace ::physx;
using namespace omni::physx::internal;
using namespace omni::physx;

namespace
{

void postErrorMessage(const SdfPath& path)
{
    std::string errorStr = "Attachment " + path.GetString() + " was not created! The prims attached must belong to the same scene.";

    PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, errorStr.c_str());
}

template <typename T = int>
struct ResultBuffer
{
    ~ResultBuffer()
    {
        if (ptr)
        {
            ICE_FREE(ptr);
            ptr = nullptr;
        }
        size = 0;
    }

    static void* allocate(size_t numBytes)
    {
        return ICE_ALLOC(numBytes);
    }

    T* ptr = nullptr;
    uint32_t size = 0;
};

bool checkScenes(const PxActor* actor0, const PxActor* actor1)
{
    PxScene* s0 = actor0->getScene();
    PxScene* s1 = actor1->getScene();

    if ((s0 && s1) && (s0 != s1))
    {
        return false;
    }

    return true;
}

bool checkScenes(const PxActor* actor0, const PxActor* actor1, const SdfPath& path)
{
    if (checkScenes(actor0, actor1))
        return true;

    postErrorMessage(path);

    return false;
}

bool matchingRigidBody(const usdparser::AttachedStage& attachedStage, PhysxAttachmentDesc const& attachmentDesc, const PxActor* deformableActor, PxRigidActor*& rigidActor, PxTransform& transformLocal, ObjectId& shapeId)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, attachmentDesc.actor[1].objId);

    if (objectRecord)
    {
        if (internalType == ePTActor || internalType == ePTLink)
        {
            InternalActor* intActor = (InternalActor*)objectRecord->mInternalPtr;

            if (checkScenes(intActor->mActor, deformableActor))
            {
                rigidActor = intActor->mActor;
                transformLocal = PxTransform(PxIdentity);
                shapeId = kInvalidObjectId;
                return true;
            }
            else
            {
                // Loop through mirrored scenes
                for (int i = 0; i < intActor->mMirrors.size(); i++)
                {
                    if (checkScenes(intActor->mMirrors[i].actor, deformableActor))
                    {
                        rigidActor = intActor->mMirrors[i].actor;
                        transformLocal = PxTransform(PxIdentity);
                        shapeId = kInvalidObjectId;
                        return true;
                    }
                }
            }
        }
        else if (internalType == ePTShape)
        {
            PxShape* shape = (PxShape*)objectRecord->mPtr;
            if (shape)
            {
                if (checkScenes(shape->getActor(), deformableActor))
                {
                    rigidActor = shape->getActor();
                    transformLocal = shape->getLocalPose();
                    shapeId = (ObjectId)shape->userData;
                    return true;
                }
                else
                {
                    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, (ObjectId)shape->getActor()->userData);
                    if (objectRecord)
                    {
                        if (internalType == ePTActor)
                        {
                            InternalActor* intActor = (InternalActor*)objectRecord->mInternalPtr;
                            // Loop through mirrored scenes
                            for (int i = 0; i < intActor->mMirrors.size(); i++)
                            {
                                if (checkScenes(intActor->mMirrors[i].actor, deformableActor))
                                {
                                    rigidActor = intActor->mMirrors[i].actor;
                                    transformLocal = shape->getLocalPose();
                                    shapeId = (ObjectId)shape->userData;
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return false;
}

InternalAttachmentDeprecated* createSoftBodyRigidBodyAttachment(const usdparser::AttachedStage& attachedStage, const SdfPath& path, PhysxAttachmentDesc const& attachmentDesc)
{
    const pxr::SdfPath& deformablePath = attachmentDesc.actor[0].path;
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo;

    getDeformableMeshInfoDeprecated(deformablePath, deformableMeshInfo);

    InternalDeformableBodyDeprecated* deformableBody = omni::physx::getInternalPtr<InternalDeformableBodyDeprecated>(omni::physx::ePTSoftBodyDeprecated, attachmentDesc.actor[0].objId);

    PxRigidActor* rigidActor = nullptr;
    PxTransform transformLocal = PxTransform(PxIdentity);
    ObjectId shapeId = kInvalidObjectId;

    // Make sure that both actors belong to the same scene
    if (!matchingRigidBody(attachedStage, attachmentDesc, deformableBody->mSoftBody, rigidActor, transformLocal, shapeId))
    {
        postErrorMessage(path);
        return nullptr;
    }

    InternalAttachmentDeprecated* internalAttachment = ICE_NEW(InternalAttachmentDeprecated)(deformableBody, rigidActor, shapeId);

    const PxTetrahedronMesh* collisionMesh = deformableBody->mSoftBody->getCollisionMesh();
    const uint32_t* collisionMeshIndices = (uint32_t*)collisionMesh->getTetrahedrons();
    uint32_t collisionMeshIndicesSize = collisionMesh->getNbTetrahedrons() * 4;

    UsdPrim prim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(attachmentDesc.actor[1].path);
    GfMatrix4d mat = UsdGeomXformable(prim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::EarliestTime());
    const GfTransform tr(mat);
    const PxVec3 scale = omni::physx::toPhysX(tr.GetScale());

    internalAttachment->mAttachmentActor[0].reserveTetrahedronData(attachmentDesc.actor[0].points.size(), attachmentDesc.actor[0].filterIndices.size());
    internalAttachment->mAttachmentActor[1].reserveRigidActorData(attachmentDesc.actor[0].points.size());

    {
        const carb::Float3* posRest = &deformableMeshInfo.restPositions[0];
        uint32_t posRestSize = uint32_t(deformableMeshInfo.restPositions.size());

        uint64_t tetFinderRestPositions = omni::tetfinder::createTetFinder(posRest, posRestSize, collisionMeshIndices, collisionMeshIndicesSize);

        std::vector<int32_t> tetIds(attachmentDesc.actor[0].points.size());
        std::vector<carb::Float4> tetBary(attachmentDesc.actor[0].points.size());
        omni::tetfinder::pointsToTetMeshLocal(&tetIds[0], &tetBary[0], tetFinderRestPositions, &attachmentDesc.actor[0].points[0], uint32_t(attachmentDesc.actor[0].points.size()));

        for (size_t i = 0; i < attachmentDesc.actor[0].points.size(); i++)
        {
            if (tetIds[i] == -1)
            {
                CARB_LOG_WARN("Invalid attachment point detected for attachment %s. Skipping attachment point [%f, %f, %f].\n",
                    path.GetText(), attachmentDesc.actor[0].points[i].x, attachmentDesc.actor[0].points[i].y, attachmentDesc.actor[0].points[i].z);
                continue;
            }

            PxVec3 actorPos = omni::physx::toPhysX(attachmentDesc.actor[1].points[i]).multiply(scale);
            actorPos = transformLocal.transform(actorPos);

#if 0
            // This step is deferred until finalizeDeformableRigidAttachments()
            PxU32 handle = softBody->mSoftBody->addTetRigidAttachment(rigidActor, tetIds[i], (PxVec4&)tetBary[i], actorPos);
            internalAttachment->mTetraAttachmentHandles.push_back(handle);
#endif

            internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices.push_back(tetIds[i]);
            internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords.push_back(toPhysX(tetBary[i]));
            internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords.push_back(PxVec4(actorPos, 0.0f));
        }

        deformableBody->convertToPhysxAttachmentTets(internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices, internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords);

        omni::tetfinder::releaseTetFinder(tetFinderRestPositions);
    }

    if (attachmentDesc.actor[0].filterType == PhysxAttachmentFilterType::eGEOMETRY)
    {
        const PxU32* map = deformableBody->mSoftBody->getCollisionMesh()->getTetrahedraRemap();
        PxU32 mapSize = deformableBody->mSoftBody->getCollisionMesh()->getNbTetrahedrons();
        PxArray<PxU32> mapInverse;
        mapInverse.resize(mapSize);
        for (PxU32 i = 0; i < mapSize; ++i)
            mapInverse[map[i]] = i;

        for (size_t i = 0; i < attachmentDesc.actor[0].filterIndices.size(); i++)
        {
            PxU32 usdTetId = attachmentDesc.actor[0].filterIndices[i];
            PxU32 tetId = PX_MAX_NB_SOFTBODY_TET;
            if (usdTetId != PxU32(-1))
                tetId = mapInverse[usdTetId];

            if (tetId == -1)
            {
                CARB_LOG_WARN("Invalid filtering index detected for attachment %s. Skipping filtering index [%d].\n", path.GetText(), usdTetId);
                continue;
            }

#if 0
            // This step is deferred until finalizeDeformableRigidAttachments()
            deformableBody->mSoftBody->addTetRigidFilter(rigidActor, tetId);
#endif

            internalAttachment->mAttachmentActor[0].mDeformableFilterIndices.push_back(tetId);
        }
    }
    else
    {
        CARB_ASSERT(false);
    }

    return internalAttachment;
}

InternalAttachmentDeprecated* createParticleClothRigidBodyAttachment(const usdparser::AttachedStage& attachedStage, const SdfPath& path, PhysxAttachmentDesc const& attachmentDesc)
{
    const pxr::SdfPath& clothPath = attachmentDesc.actor[0].path;
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo;

    omni::physx::getDeformableMeshInfoDeprecated(clothPath, deformableMeshInfo);

    InternalParticleClothDeprecated* particleCloth = omni::physx::getInternalPtr<InternalParticleClothDeprecated>(omni::physx::ePTParticleClothDeprecated, attachmentDesc.actor[0].objId);

    PxRigidActor* rigidActor = nullptr;
    PxTransform transformLocal = PxTransform(PxIdentity);
    ObjectId shapeId = kInvalidObjectId;

    // Make sure that both actors belong to the same scene
    if (!matchingRigidBody(attachedStage, attachmentDesc, particleCloth->mParentParticleSystem->mPS, rigidActor, transformLocal, shapeId))
    {
        postErrorMessage(path);
        return nullptr;
    }

    InternalAttachmentDeprecated* internalAttachment = ICE_NEW(InternalAttachmentDeprecated)(particleCloth, rigidActor, shapeId);

    UsdPrim prim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(attachmentDesc.actor[1].path);
    GfMatrix4d mat = UsdGeomXformable(prim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::EarliestTime());
    const GfTransform tr(mat);
    const PxVec3 scale = omni::physx::toPhysX(tr.GetScale());

    internalAttachment->mAttachmentActor[0].reserveVertexData(attachmentDesc.actor[0].points.size(), attachmentDesc.actor[0].filterIndices.size());
    internalAttachment->mAttachmentActor[1].reserveRigidActorData(attachmentDesc.actor[0].points.size());

    uint64_t pointFinder = omni::pointfinder::createPointFinder(&deformableMeshInfo.restPositions[0], uint32_t(deformableMeshInfo.restPositions.size()));
    std::vector<int32_t> attachmentClothIndices(attachmentDesc.actor[0].points.size());
    omni::pointfinder::pointsToIndices(&attachmentClothIndices[0], pointFinder, &attachmentDesc.actor[0].points[0], uint32_t(attachmentClothIndices.size()));
    for (size_t i = 0; i < attachmentDesc.actor[0].points.size(); i++)
    {
        int32_t index = attachmentClothIndices[i];
        if (index >= 0)
        {
            PxVec3 actorPos = omni::physx::toPhysX(attachmentDesc.actor[1].points[i]).multiply(scale);
            actorPos = transformLocal.transform(actorPos);

#if 0
            // This step is deferred until finalizeDeformableRigidAttachments()
            ps->mPS->addRigidAttachment(rigidActor, vertexId, actorPos);
#endif

            internalAttachment->mAttachmentActor[0].mParticleAttachmentIds.push_back(index);
            internalAttachment->mAttachmentActor[1].mRigidActorPositions.push_back((GfVec3f&)actorPos);
        }
    }
    omni::pointfinder::releasePointFinder(pointFinder);

    if (attachmentDesc.actor[0].filterType == PhysxAttachmentFilterType::eVERTICES)
    {
        for (size_t i = 0; i < attachmentDesc.actor[0].filterIndices.size(); i++)
        {
            PxU32 vertexId = attachmentDesc.actor[0].filterIndices[i];
#if 0
            // This step is deferred until finalizeDeformableRigidAttachments()
            ps->mPS->addRigidFilter(rigidActor, vertexId);
#endif

            internalAttachment->mAttachmentActor[0].mParticleFilterIds.push_back(vertexId);
        }
    }
    else
    {
        CARB_ASSERT(false);
    }

    return internalAttachment;
}

InternalAttachmentDeprecated* createDeformableSurfaceRigidBodyAttachment(const usdparser::AttachedStage& attachedStage, const SdfPath& path, PhysxAttachmentDesc const& attachmentDesc)
{
    const pxr::SdfPath& deformableSurfacePath = attachmentDesc.actor[0].path;
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo;

    omni::physx::getDeformableMeshInfoDeprecated(deformableSurfacePath, deformableMeshInfo);

    InternalDeformableSurfaceDeprecated* deformableSurface = omni::physx::getInternalPtr<InternalDeformableSurfaceDeprecated>(omni::physx::ePTFEMClothDeprecated, attachmentDesc.actor[0].objId);

    PxRigidActor* rigidActor = nullptr;
    PxTransform transformLocal = PxTransform(PxIdentity);
    ObjectId shapeId = kInvalidObjectId;

    // Make sure that both actors belong to the same scene
    if (!matchingRigidBody(attachedStage, attachmentDesc, deformableSurface->mDeformableSurface, rigidActor, transformLocal, shapeId))
    {
        postErrorMessage(path);
        return nullptr;
    }

    InternalAttachmentDeprecated* internalAttachment = ICE_NEW(InternalAttachmentDeprecated)(deformableSurface, rigidActor, shapeId);

    UsdPrim prim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(attachmentDesc.actor[1].path);
    GfMatrix4d mat = UsdGeomXformable(prim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::EarliestTime());
    const GfTransform tr(mat);
    const PxVec3 scale = omni::physx::toPhysX(tr.GetScale());

    internalAttachment->mAttachmentActor[0].reserveVertexData(attachmentDesc.actor[0].points.size(), attachmentDesc.actor[0].filterIndices.size());
    internalAttachment->mAttachmentActor[1].reserveRigidActorData(attachmentDesc.actor[0].points.size());

    uint64_t pointFinder = omni::pointfinder::createPointFinder(&deformableMeshInfo.restPositions[0], uint32_t(deformableMeshInfo.restPositions.size()));
    std::vector<int32_t> attachmentClothIndices(attachmentDesc.actor[0].points.size());
    omni::pointfinder::pointsToIndices(&attachmentClothIndices[0], pointFinder, &attachmentDesc.actor[0].points[0], uint32_t(attachmentClothIndices.size()));
    for (size_t i = 0; i < attachmentDesc.actor[0].points.size(); i++)
    {
        int32_t index = attachmentClothIndices[i];
        if (index >= 0)
        {
            PxVec3 actorPos = omni::physx::toPhysX(attachmentDesc.actor[1].points[i]).multiply(scale);
            actorPos = transformLocal.transform(actorPos);

#if 0
            // This step is deferred until finalizeDeformableRigidAttachments()
            PxU32 handle = deformableSurface->mFEMCloth->addRigidAttachment(rigidActor, index, actorPos);
            internalAttachment->mAttachmentActor[0].mParticleAttachmentHandles.push_back(handle);
#endif

            internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices.push_back(index);
            internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords.push_back(PxVec4(actorPos, 0.0f));
        }
    }
    omni::pointfinder::releasePointFinder(pointFinder);

    if (attachmentDesc.actor[0].filterType == PhysxAttachmentFilterType::eVERTICES)
    {
        std::vector<PxU32> vtxIndices;
        std::vector<PxVec4> triBarycentrics;

        for (size_t i = 0; i < attachmentDesc.actor[0].filterIndices.size(); i++)
        {
            PxU32 vertexId = attachmentDesc.actor[0].filterIndices[i];

#if 0
            // This step is deferred until finalizeDeformableRigidAttachments()
            ps->mPS->addRigidFilter(rigidActor, vertexId);
#endif

            vtxIndices.push_back(vertexId);
        }

        deformableSurface->findTrisFromVtxIndices(vtxIndices, internalAttachment->mAttachmentActor[0].mDeformableFilterIndices, triBarycentrics);
    }
    else
    {
        CARB_ASSERT(false);
    }

    return internalAttachment;
}

InternalAttachmentDeprecated* createSoftBodySoftBodyAttachment(const SdfPath& path, PhysxAttachmentDesc const& attachmentDesc)
{
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo0;
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo1;

    omni::physx::getDeformableMeshInfoDeprecated(attachmentDesc.actor[0].path, deformableMeshInfo0);
    omni::physx::getDeformableMeshInfoDeprecated(attachmentDesc.actor[1].path, deformableMeshInfo1);

    InternalDeformableBodyDeprecated* deformableBody0 = omni::physx::getInternalPtr<InternalDeformableBodyDeprecated>(omni::physx::ePTSoftBodyDeprecated, attachmentDesc.actor[0].objId);
    InternalDeformableBodyDeprecated* deformableBody1 = omni::physx::getInternalPtr<InternalDeformableBodyDeprecated>(omni::physx::ePTSoftBodyDeprecated, attachmentDesc.actor[1].objId);

    // Make sure that both actors belong to the same scene
    if (!checkScenes(deformableBody0->mSoftBody, deformableBody1->mSoftBody, path))
        return nullptr;

    InternalAttachmentDeprecated* internalAttachment = ICE_NEW(InternalAttachmentDeprecated)(deformableBody0, deformableBody1);

    //softbody to world for rotating axis into softbody mesh space
    PxTransform pxSoftBody0ToWorld;
    {
        PxVec3 pxSoftBody0ToWorldScale;
        toPhysX(pxSoftBody0ToWorld, pxSoftBody0ToWorldScale, deformableBody0->mPrimToWorld);
    }

    const PxTetrahedronMesh* collisionMesh0 = deformableBody0->mSoftBody->getCollisionMesh();
    uint64_t tetFinderRestPositions0 = omni::tetfinder::createTetFinder(
        &deformableMeshInfo0.restPositions[0], uint32_t(deformableMeshInfo0.restPositions.size()),
        (uint32_t*)collisionMesh0->getTetrahedrons(), collisionMesh0->getNbTetrahedrons()*4);

    const PxTetrahedronMesh* collisionMesh1 = deformableBody1->mSoftBody->getCollisionMesh();
    uint64_t tetFinderRestPositions1 = omni::tetfinder::createTetFinder(
        &deformableMeshInfo1.restPositions[0], uint32_t(deformableMeshInfo1.restPositions.size()),
        (uint32_t*)collisionMesh1->getTetrahedrons(), collisionMesh1->getNbTetrahedrons()*4);

    internalAttachment->mAttachmentActor[0].reserveTetrahedronData(attachmentDesc.actor[0].points.size(), attachmentDesc.actor[0].filterIndices.size());
    internalAttachment->mAttachmentActor[1].reserveTetrahedronData(attachmentDesc.actor[0].points.size(), attachmentDesc.actor[0].filterIndices.size());

    {
        std::vector<int32_t> tetIds0(attachmentDesc.actor[0].points.size());
        std::vector<carb::Float4> bary0(attachmentDesc.actor[0].points.size());
        omni::tetfinder::pointsToTetMeshLocalAll(&tetIds0[0], &bary0[0], tetFinderRestPositions0, &attachmentDesc.actor[0].points[0], uint32_t(attachmentDesc.actor[0].points.size()));

        std::vector<int32_t> tetIds1(attachmentDesc.actor[1].points.size());
        std::vector<carb::Float4> bary1(attachmentDesc.actor[1].points.size());
        omni::tetfinder::pointsToTetMeshLocalAll(&tetIds1[0], &bary1[0], tetFinderRestPositions1, &attachmentDesc.actor[1].points[0], uint32_t(attachmentDesc.actor[1].points.size()));

        CARB_ASSERT(attachmentDesc.actor[0].points.size() == attachmentDesc.actor[1].points.size());
        for (size_t i = 0; i < attachmentDesc.actor[0].points.size(); i++)
        {
            if (tetIds0[i] == -1 || tetIds1[i] == -1)
            {
                CARB_LOG_WARN("Invalid attachment point detected for attachment %s. Skipping attachment point [%f, %f, %f].\n",
                    path.GetText(), attachmentDesc.actor[0].points[i].x, attachmentDesc.actor[0].points[i].y, attachmentDesc.actor[0].points[i].z);
                continue;
            }

            internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices.push_back(tetIds0[i]);
            internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords.push_back(toPhysX(bary0[i]));

            internalAttachment->mAttachmentActor[1].mDeformableAttachmentIndices.push_back(tetIds1[i]);
            internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords.push_back(toPhysX(bary1[i]));
        }

        deformableBody0->convertToPhysxAttachmentTets(internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices, internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords);
        deformableBody1->convertToPhysxAttachmentTets(internalAttachment->mAttachmentActor[1].mDeformableAttachmentIndices, internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords);
        internalAttachment->mDeformableAttachment = deformableBody0->addSoftBodyAttachments(
            deformableBody1->mSoftBody,
            internalAttachment->mAttachmentActor[1].mDeformableAttachmentIndices, internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords,
            internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices, internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords
        );
    }

    if (attachmentDesc.actor[0].filterType == PhysxAttachmentFilterType::eGEOMETRY)
    {
        const PxU32* map0 = deformableBody0->mSoftBody->getCollisionMesh()->getTetrahedraRemap();
        PxU32 mapSize0 = deformableBody0->mSoftBody->getCollisionMesh()->getNbTetrahedrons();
        PxArray<PxU32> mapInverse0;
        mapInverse0.resize(mapSize0);
        for (PxU32 i = 0; i < mapSize0; ++i)
            mapInverse0[map0[i]] = i;

        const PxU32* map1 = deformableBody1->mSoftBody->getCollisionMesh()->getTetrahedraRemap();
        PxU32 mapSize1 = deformableBody1->mSoftBody->getCollisionMesh()->getNbTetrahedrons();
        PxArray<PxU32> mapInverse1;
        mapInverse1.resize(mapSize1);
        for (PxU32 i = 0; i < mapSize1; ++i)
            mapInverse1[map1[i]] = i;

        for (size_t i = 0; i < attachmentDesc.actor[0].filterIndices.size(); i++)
        {
            PxU32 usdTetId0 = attachmentDesc.actor[0].filterIndices[i];
            PxU32 usdTetId1 = attachmentDesc.actor[1].filterIndices[i];

            PxU32 tetId0 = PX_MAX_NB_SOFTBODY_TET;
            if (usdTetId0 != PxU32(-1))
                tetId0 = mapInverse0[usdTetId0];

            PxU32 tetId1 = PX_MAX_NB_SOFTBODY_TET;
            if (usdTetId1 != PxU32(-1))
                tetId1 = mapInverse1[usdTetId1];

            internalAttachment->mAttachmentActor[0].mDeformableFilterIndices.push_back(tetId0);
            internalAttachment->mAttachmentActor[1].mDeformableFilterIndices.push_back(tetId1);
        }

        internalAttachment->mDeformableFilter = deformableBody0->addSoftBodyFilters(
            deformableBody1->mSoftBody,
            internalAttachment->mAttachmentActor[1].mDeformableFilterIndices,
            internalAttachment->mAttachmentActor[0].mDeformableFilterIndices
        );
    }
    else
    {
        CARB_ASSERT(false);
    }

    omni::tetfinder::releaseTetFinder(tetFinderRestPositions0);
    omni::tetfinder::releaseTetFinder(tetFinderRestPositions1);

    return internalAttachment;
}

InternalAttachmentDeprecated* createSoftBodyParticleClothAttachment(const SdfPath& path, PhysxAttachmentDesc const& attachmentDesc)
{
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo0;
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo1;

    omni::physx::getDeformableMeshInfoDeprecated(attachmentDesc.actor[0].path, deformableMeshInfo0);
    omni::physx::getDeformableMeshInfoDeprecated(attachmentDesc.actor[1].path, deformableMeshInfo1);

    InternalDeformableBodyDeprecated* deformableBody = omni::physx::getInternalPtr<InternalDeformableBodyDeprecated>(omni::physx::ePTSoftBodyDeprecated, attachmentDesc.actor[0].objId);
    InternalParticleClothDeprecated* particleCloth = omni::physx::getInternalPtr<InternalParticleClothDeprecated>(omni::physx::ePTParticleClothDeprecated, attachmentDesc.actor[1].objId);
    InternalPbdParticleSystem* particleSystem = particleCloth->mParentParticleSystem;

    if (!particleSystem)
        return nullptr;

    // Make sure that both actors belong to the same scene
    if (!checkScenes(deformableBody->mSoftBody, particleSystem->mPS, path))
        return nullptr;

    InternalAttachmentDeprecated* internalAttachment = ICE_NEW(InternalAttachmentDeprecated)(deformableBody, particleCloth);

    const PxTetrahedronMesh* collisionMesh = deformableBody->mSoftBody->getCollisionMesh();
    uint64_t tetFinderRestPositions = omni::tetfinder::createTetFinder(
        &deformableMeshInfo0.restPositions[0], uint32_t(deformableMeshInfo0.restPositions.size()),
        (uint32_t*)collisionMesh->getTetrahedrons(), collisionMesh->getNbTetrahedrons() * 4);

    uint64_t pointFinder = omni::pointfinder::createPointFinder(&deformableMeshInfo1.restPositions[0], uint32_t(deformableMeshInfo1.restPositions.size()));

    internalAttachment->mAttachmentActor[0].reserveTetrahedronData(attachmentDesc.actor[0].points.size(), attachmentDesc.actor[0].filterIndices.size());
    internalAttachment->mAttachmentActor[1].reserveVertexData(attachmentDesc.actor[0].points.size(), attachmentDesc.actor[0].filterIndices.size());

    {
        std::vector<int32_t> tetIds(attachmentDesc.actor[0].points.size());
        std::vector<carb::Float4> bary(attachmentDesc.actor[0].points.size());
        omni::tetfinder::pointsToTetMeshLocal(&tetIds[0], &bary[0], tetFinderRestPositions, &attachmentDesc.actor[0].points[0], uint32_t(attachmentDesc.actor[0].points.size()));

        std::vector<int32_t> attachmentClothIndices(attachmentDesc.actor[1].points.size());
        omni::pointfinder::pointsToIndices(&attachmentClothIndices[0], pointFinder, &attachmentDesc.actor[1].points[0], uint32_t(attachmentClothIndices.size()));

        CARB_ASSERT(attachmentDesc.actor[0].points.size() == attachmentDesc.actor[1].points.size());
        for (size_t i = 0; i < attachmentDesc.actor[0].points.size(); i++)
        {
            if (tetIds[i] == -1)
            {
                CARB_LOG_WARN("Invalid attachment point detected for attachment %s. Skipping attachment point [%f, %f, %f].\n",
                    path.GetText(), attachmentDesc.actor[0].points[i].x, attachmentDesc.actor[0].points[i].y, attachmentDesc.actor[0].points[i].z);
                continue;
            }

            int32_t index = attachmentClothIndices[i];
            if (index >= 0)
            {
                PxU32 handle = deformableBody->mSoftBody->addParticleAttachment(particleSystem->mPS, particleCloth->mClothBuffer, index, tetIds[i], (PxVec4&)bary[i]);

                internalAttachment->mAttachmentActor[0].mTetraAttachmentHandles.push_back(handle);
                internalAttachment->mAttachmentActor[0].mTetraAttachmentIds.push_back(tetIds[i]);
                internalAttachment->mAttachmentActor[0].mTetraAttachmentBaryCoords.push_back((GfVec4f&)bary);

                internalAttachment->mAttachmentActor[1].mParticleAttachmentIds.push_back(index);
            }
        }
    }

    if (attachmentDesc.actor[0].filterType == PhysxAttachmentFilterType::eGEOMETRY)
    {
        const PxU32* map = deformableBody->mSoftBody->getCollisionMesh()->getTetrahedraRemap();
        PxU32 mapSize = deformableBody->mSoftBody->getCollisionMesh()->getNbTetrahedrons();
        PxArray<PxU32> mapInverse;
        mapInverse.resize(mapSize);
        for (PxU32 i = 0; i < mapSize; ++i)
            mapInverse[map[i]] = i;

        for (size_t i = 0; i < attachmentDesc.actor[0].filterIndices.size(); i++)
        {
            PxU32 usdTetId = attachmentDesc.actor[0].filterIndices[i];
            PxU32 vertexId = attachmentDesc.actor[1].filterIndices[i];

            PxU32 tetId = PX_MAX_NB_SOFTBODY_TET;
            if (usdTetId != PxU32(-1))
                tetId = mapInverse[usdTetId];

            if (tetId == -1)
            {
                CARB_LOG_WARN("Invalid filtering index detected for attachment %s. Skipping filtering index [%d].\n", path.GetText(), usdTetId);
                continue;
            }

            deformableBody->mSoftBody->addParticleFilter(particleSystem->mPS, particleCloth->mClothBuffer, vertexId, tetId);

            internalAttachment->mAttachmentActor[0].mTetraFilterIds.push_back(tetId);
            internalAttachment->mAttachmentActor[1].mParticleFilterIds.push_back(vertexId);
        }
    }
    else
    {
        CARB_ASSERT(false);
    }

    omni::tetfinder::releaseTetFinder(tetFinderRestPositions);
    omni::pointfinder::releasePointFinder(pointFinder);

    return internalAttachment;
}

InternalAttachmentDeprecated* createSoftBodyDeformableSurfaceAttachment(const SdfPath& path, PhysxAttachmentDesc const& attachmentDesc)
{
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo0;
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo1;

    omni::physx::getDeformableMeshInfoDeprecated(attachmentDesc.actor[0].path, deformableMeshInfo0);
    omni::physx::getDeformableMeshInfoDeprecated(attachmentDesc.actor[1].path, deformableMeshInfo1);

    InternalDeformableBodyDeprecated* deformableBody = omni::physx::getInternalPtr<InternalDeformableBodyDeprecated>(omni::physx::ePTSoftBodyDeprecated, attachmentDesc.actor[0].objId);
    InternalDeformableSurfaceDeprecated* deformableSurface = omni::physx::getInternalPtr<InternalDeformableSurfaceDeprecated>(omni::physx::ePTFEMClothDeprecated, attachmentDesc.actor[1].objId);

    // Make sure that both actors belong to the same scene
    if (!checkScenes(deformableBody->mSoftBody, deformableSurface->mDeformableSurface, path))
        return nullptr;

    InternalAttachmentDeprecated* internalAttachment = ICE_NEW(InternalAttachmentDeprecated)(deformableBody, deformableSurface);

    //softbody to world for rotating axis into softbody mesh space
    PxTransform pxSoftBodyToWorld;
    {
        PxVec3 pxSoftBodyToWorldScale;
        toPhysX(pxSoftBodyToWorld, pxSoftBodyToWorldScale, deformableBody->mPrimToWorld);
    }

    const PxTetrahedronMesh* collisionMesh = deformableBody->mSoftBody->getCollisionMesh();
    uint64_t tetFinderRestPositions = omni::tetfinder::createTetFinder(
        &deformableMeshInfo0.restPositions[0], uint32_t(deformableMeshInfo0.restPositions.size()),
        (uint32_t*)collisionMesh->getTetrahedrons(), collisionMesh->getNbTetrahedrons() * 4);

    std::vector<int32_t> tetIds(attachmentDesc.actor[0].points.size());
    std::vector<carb::Float4> tetBary(attachmentDesc.actor[0].points.size());
    omni::tetfinder::pointsToTetMeshLocalAll(&tetIds[0], &tetBary[0], tetFinderRestPositions, &attachmentDesc.actor[0].points[0], uint32_t(attachmentDesc.actor[0].points.size()));

    std::vector<int32_t> attachmentClothIndices(attachmentDesc.actor[1].points.size());
    uint64_t pointFinder = omni::pointfinder::createPointFinder(&deformableMeshInfo1.restPositions[0], uint32_t(deformableMeshInfo1.restPositions.size()));
    omni::pointfinder::pointsToIndices(&attachmentClothIndices[0], pointFinder, &attachmentDesc.actor[1].points[0], uint32_t(attachmentClothIndices.size()));

    const PxTriangleMesh* triMesh = deformableSurface->mTriangleMesh;
    uint64_t triFinderRestPositions = omni::trifinder::createTriFinder(&deformableMeshInfo1.restPositions[0], uint32_t(deformableMeshInfo1.restPositions.size()), (uint32_t*)triMesh->getTriangles(), triMesh->getNbTriangles() * 3);

    {
        std::vector<int32_t> triIds(attachmentDesc.actor[0].points.size());
        std::vector<carb::Float3> triBary(attachmentDesc.actor[0].points.size());
        omni::trifinder::pointsToTriMeshLocal(&triIds[0], &triBary[0], triFinderRestPositions, &attachmentDesc.actor[1].points[0], uint32_t(attachmentDesc.actor[1].points.size()));

        CARB_ASSERT(attachmentDesc.actor[0].points.size() == attachmentDesc.actor[1].points.size());
        for (size_t i = 0; i < attachmentDesc.actor[0].points.size(); i++)
        {
            if (tetIds[i] == -1 || triIds[i] == -1)
            {
                CARB_LOG_WARN("Invalid attachment point detected for attachment %s. Skipping attachment point [%f, %f, %f].\n",
                    path.GetText(), attachmentDesc.actor[0].points[i].x, attachmentDesc.actor[0].points[i].y, attachmentDesc.actor[0].points[i].z);
                continue;
            }

            int32_t index = attachmentClothIndices[i];
            if (index >= 0)
            {
                internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices.push_back(tetIds[i]);
                internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords.push_back(toPhysX(tetBary[i]));

                internalAttachment->mAttachmentActor[1].mDeformableAttachmentIndices.push_back(triIds[i]);
                internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords.push_back(PxVec4(toPhysX(triBary[i]), 0.0f));
            }
        }

        deformableBody->convertToPhysxAttachmentTets(internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices, internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords);
        internalAttachment->mDeformableAttachment = deformableBody->addSoftBodyAttachments(
            deformableSurface->mDeformableSurface,
            internalAttachment->mAttachmentActor[1].mDeformableAttachmentIndices, internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords,
            internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices, internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords
        );
    }

    bool validFilter = attachmentDesc.actor[0].filterType == PhysxAttachmentFilterType::eGEOMETRY &&
                       attachmentDesc.actor[1].filterType == PhysxAttachmentFilterType::eVERTICES &&
                       attachmentDesc.actor[0].filterIndices.size() == attachmentDesc.actor[1].filterIndices.size();
    if (validFilter)
    {
        const PxU32* map = deformableBody->mSoftBody->getCollisionMesh()->getTetrahedraRemap();
        PxU32 mapSize = deformableBody->mSoftBody->getCollisionMesh()->getNbTetrahedrons();
        PxArray<PxU32> mapInverse;
        mapInverse.resize(mapSize);
        for (PxU32 i = 0; i < mapSize; ++i)
            mapInverse[map[i]] = i;

        for (size_t i = 0; i < attachmentDesc.actor[1].filterIndices.size(); i++)
        {
            uint32_t vertId = attachmentDesc.actor[1].filterIndices[i];

            PxU32 usdTetId = attachmentDesc.actor[0].filterIndices[i];
            PxU32 tetId = PX_MAX_NB_SOFTBODY_TET;
            if (usdTetId != PxU32(-1))
                tetId = mapInverse[usdTetId];

            if (tetId == -1)
            {
                CARB_LOG_WARN("Invalid filtering index detected for attachment %s. Skipping filtering index [%d].\n", path.GetText(), usdTetId);
                continue;
            }

            internalAttachment->mAttachmentActor[0].mDeformableFilterIndices.push_back(tetId);
            internalAttachment->mAttachmentActor[1].mDeformableFilterIndices.push_back(vertId);
        }

        std::vector<PxVec4> triBarycentrics;
        deformableSurface->findTrisFromVtxIndices(internalAttachment->mAttachmentActor[1].mDeformableFilterIndices, internalAttachment->mAttachmentActor[1].mDeformableFilterIndices, triBarycentrics);
        internalAttachment->mDeformableFilter = deformableBody->addSoftBodyFilters(
            deformableSurface->mDeformableSurface,
            internalAttachment->mAttachmentActor[1].mDeformableFilterIndices,
            internalAttachment->mAttachmentActor[0].mDeformableFilterIndices
        );
    }
    else
    {
        CARB_LOG_WARN("Invalid attachment collision filtering setup: %s\n", path.GetText());
    }

    omni::tetfinder::releaseTetFinder(tetFinderRestPositions);
    omni::trifinder::releaseTriFinder(triFinderRestPositions);
    omni::pointfinder::releasePointFinder(pointFinder);

    return internalAttachment;
}

InternalAttachmentDeprecated* createParticleClothStaticAttachment(const SdfPath& path, PhysxAttachmentDesc const& attachmentDesc)
{
    const pxr::SdfPath& clothPath = attachmentDesc.actor[0].path;
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo;

    omni::physx::getDeformableMeshInfoDeprecated(clothPath, deformableMeshInfo);

    InternalParticleClothDeprecated* particleCloth = omni::physx::getInternalPtr<InternalParticleClothDeprecated>(omni::physx::ePTParticleClothDeprecated, attachmentDesc.actor[0].objId);
    InternalPbdParticleSystem* particleSystem = particleCloth->mParentParticleSystem;

    InternalAttachmentDeprecated* internalAttachment = ICE_NEW(InternalAttachmentDeprecated)(particleCloth);

    VtArray<GfVec3f> attachmentPoints;
    bool usdGlobalPositions = true;
    if (attachmentDesc.actor[1].points.empty())
        usdGlobalPositions = false;

    uint64_t pointFinder = omni::pointfinder::createPointFinder(&deformableMeshInfo.restPositions[0], uint32_t(deformableMeshInfo.restPositions.size()));
    std::vector<int32_t> attachmentClothIndices(attachmentDesc.actor[0].points.size());
    omni::pointfinder::pointsToIndices(&attachmentClothIndices[0], pointFinder, &attachmentDesc.actor[0].points[0], uint32_t(attachmentClothIndices.size()));

    // Create static actor at origin
    // All offsets are with reference to the origin
    PxRigidStatic* rigidStatic = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createRigidStatic(PxTransform(PxVec3(PxZero)));
    particleSystem->mPS->getScene()->addActor(*rigidStatic);
    internalAttachment->mAttachmentActor[0].mRigidActor = rigidStatic;

    internalAttachment->mAttachmentActor[0].reserveVertexData(attachmentDesc.actor[0].points.size(), 0);

    for (size_t i = 0; i < attachmentDesc.actor[0].points.size(); i++)
    {
        int32_t index = attachmentClothIndices[i];
        if (index >= 0)
        {
            PxVec3 pos;
            if (usdGlobalPositions)
            {
                pos = omni::physx::toPhysX(attachmentDesc.actor[1].points[i]);
            }
            else
            {
                pos = omni::physx::toPhysX(deformableMeshInfo.positions[index]);
                attachmentPoints.push_back((GfVec3f&)pos);
            }

            particleCloth->addRigidAttachment(rigidStatic, index, pos);

            internalAttachment->mAttachmentActor[0].mParticleAttachmentIds.push_back(index);
        }
    }
#if USE_PHYSX_GPU
    if (particleCloth->mAttachments)
        particleCloth->mAttachments->copyToDevice();
#endif

    omni::pointfinder::releasePointFinder(pointFinder);

    if (!usdGlobalPositions)
    {
        // Store the global anchor positions to the stage
        PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(OmniPhysX::getInstance().getStage(), path);
        attachment.GetPointsAttr(attachmentDesc.actor[1].id).Set(attachmentPoints);
    }

    return internalAttachment;
}

InternalAttachmentDeprecated* createSoftBodyStaticAttachment(const SdfPath& path, PhysxAttachmentDesc const& attachmentDesc)
{
    const pxr::SdfPath& softBodyPath = attachmentDesc.actor[0].path;
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo;

    getDeformableMeshInfoDeprecated(softBodyPath, deformableMeshInfo);

    InternalDeformableBodyDeprecated* deformableBody = omni::physx::getInternalPtr<InternalDeformableBodyDeprecated>(omni::physx::ePTSoftBodyDeprecated, attachmentDesc.actor[0].objId);

    InternalAttachmentDeprecated* internalAttachment = ICE_NEW(InternalAttachmentDeprecated)(deformableBody);

    VtArray<GfVec3f> attachmentPoints;
    bool usdGlobalPositions = true;
    if (attachmentDesc.actor[1].points.empty())
        usdGlobalPositions = false;

    const PxTetrahedronMesh* collisionMesh = deformableBody->mSoftBody->getCollisionMesh();
    uint64_t tetFinderRestPositions = omni::tetfinder::createTetFinder(
        &deformableMeshInfo.restPositions[0], uint32_t(deformableMeshInfo.restPositions.size()),
        (uint32_t*)collisionMesh->getTetrahedrons(), collisionMesh->getNbTetrahedrons() * 4);
    uint64_t tetFinderCollisionPositions = omni::tetfinder::createTetFinder(
        &deformableMeshInfo.positions[0], uint32_t(deformableMeshInfo.positions.size()),
        (uint32_t*)collisionMesh->getTetrahedrons(), collisionMesh->getNbTetrahedrons() * 4);

    std::vector<int32_t> tetIds(attachmentDesc.actor[0].points.size());
    std::vector<carb::Float4> bary(attachmentDesc.actor[0].points.size());
    omni::tetfinder::pointsToTetMeshLocal(&tetIds[0], &bary[0], tetFinderRestPositions, &attachmentDesc.actor[0].points[0], uint32_t(attachmentDesc.actor[0].points.size()));

    std::vector<carb::Float3> worldPos(attachmentDesc.actor[0].points.size());
    omni::tetfinder::tetMeshLocalToPoints(&worldPos[0], tetFinderCollisionPositions, &tetIds[0], &bary[0], uint32_t(attachmentDesc.actor[0].points.size()));

    // Create static actor at origin
    // All offsets are with reference to the origin
    PxRigidStatic* rigidStatic = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createRigidStatic(PxTransform(PxVec3(PxZero)));
    deformableBody->mSoftBody->getScene()->addActor(*rigidStatic);
    internalAttachment->mAttachmentActor[0].mRigidActor = rigidStatic;

    internalAttachment->mAttachmentActor[0].reserveTetrahedronData(attachmentDesc.actor[0].points.size(), 0);

    for (size_t i = 0; i < attachmentDesc.actor[0].points.size(); i++)
    {
        if (tetIds[i] == -1)
        {
            CARB_LOG_WARN("Invalid attachment point detected for attachment %s. Skipping attachment point [%f, %f, %f].\n",
                path.GetText(), attachmentDesc.actor[0].points[i].x, attachmentDesc.actor[0].points[i].y, attachmentDesc.actor[0].points[i].z);
            continue;
        }

        PxVec3 pos;
        if (usdGlobalPositions)
        {
            pos = omni::physx::toPhysX(attachmentDesc.actor[1].points[i]);
        }
        else
        {
            pos = omni::physx::toPhysX(worldPos[i]);
            attachmentPoints.push_back((GfVec3f&)pos);
        }

        internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices.push_back(tetIds[i]);
        internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords.push_back((PxVec4&)bary[i]);
        internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords.push_back(PxVec4(pos, 0.0f));
    }

    deformableBody->convertToPhysxAttachmentTets(internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices, internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords);
    internalAttachment->mDeformableAttachment = deformableBody->addRigidAttachments(
        rigidStatic,
        internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices, internalAttachment->mAttachmentActor[0].mDeformableAttachmentCoords,
        internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords
    );

    if (!usdGlobalPositions)
    {
        // Store the global anchor positions to the stage
        PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(OmniPhysX::getInstance().getStage(), path);
        attachment.GetPointsAttr(attachmentDesc.actor[1].id).Set(attachmentPoints);
    }

    return internalAttachment;
}

InternalAttachmentDeprecated* createDeformableSurfaceStaticAttachment(const SdfPath& path, PhysxAttachmentDesc const& attachmentDesc)
{
    const pxr::SdfPath& deformableSurfacePath = attachmentDesc.actor[0].path;
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo;

    omni::physx::getDeformableMeshInfoDeprecated(deformableSurfacePath, deformableMeshInfo);

    InternalDeformableSurfaceDeprecated* deformableSurface = omni::physx::getInternalPtr<InternalDeformableSurfaceDeprecated>(omni::physx::ePTFEMClothDeprecated, attachmentDesc.actor[0].objId);

    InternalAttachmentDeprecated* internalAttachment = ICE_NEW(InternalAttachmentDeprecated)(deformableSurface);

    VtArray<GfVec3f> attachmentPoints;
    bool usdGlobalPositions = true;
    if (attachmentDesc.actor[1].points.empty())
        usdGlobalPositions = false;

    uint64_t pointFinder = omni::pointfinder::createPointFinder(&deformableMeshInfo.restPositions[0], uint32_t(deformableMeshInfo.restPositions.size()));
    std::vector<int32_t> attachmentClothIndices(attachmentDesc.actor[0].points.size());
    omni::pointfinder::pointsToIndices(&attachmentClothIndices[0], pointFinder, &attachmentDesc.actor[0].points[0], uint32_t(attachmentClothIndices.size()));

    // Create static actor at origin
    // All offsets are with reference to the origin
    PxRigidStatic* rigidStatic = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createRigidStatic(PxTransform(PxVec3(PxZero)));
    deformableSurface->mDeformableSurface->getScene()->addActor(*rigidStatic);
    internalAttachment->mAttachmentActor[0].mRigidActor = rigidStatic;

    internalAttachment->mAttachmentActor[0].reserveVertexData(attachmentDesc.actor[0].points.size(), 0);

    for (size_t i = 0; i < attachmentDesc.actor[0].points.size(); i++)
    {
        int32_t index = attachmentClothIndices[i];
        if (index >= 0)
        {
            PxVec3 pos;
            if (usdGlobalPositions)
            {
                pos = omni::physx::toPhysX(attachmentDesc.actor[1].points[i]);
            }
            else
            {
                pos = omni::physx::toPhysX(deformableMeshInfo.positions[index]);
                attachmentPoints.push_back((GfVec3f&)pos);
            }

            internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices.push_back(index);
            internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords.push_back(PxVec4(pos, 0.0f));
        }
    }

    internalAttachment->mDeformableAttachment = deformableSurface->addRigidAttachments(
        rigidStatic,
        internalAttachment->mAttachmentActor[0].mDeformableAttachmentIndices,
        internalAttachment->mAttachmentActor[1].mDeformableAttachmentCoords
    );

    omni::pointfinder::releasePointFinder(pointFinder);

    if (!usdGlobalPositions)
    {
        // Store the global anchor positions to the stage
        PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(OmniPhysX::getInstance().getStage(), path);
        attachment.GetPointsAttr(attachmentDesc.actor[1].id).Set(attachmentPoints);
    }

    return internalAttachment;
}

} // namespace


namespace omni
{
namespace physx
{

void releaseAttachmentsDeprecated(InternalActor* intActor)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    for (MirrorActor& mirror : intActor->mMirrors)
    {
        PxScene* scene = mirror.actor->getScene();
        InternalDatabase::Record& actorRec = db.getRecords()[(ObjectId)scene->userData];
        InternalScene* intScene = (InternalScene*)actorRec.mInternalPtr;
        intScene->removeAttachmentsDeprecated(ObjectId(mirror.actor->userData));
    }
    intActor->mPhysXScene->getInternalScene()->removeAttachmentsDeprecated(ObjectId(intActor->mActor->userData));
}

void updateAttachmentShapeEventsDeprecated(InternalActor* intActor,
                                           ObjectId shapeId,
                                           pxr::SdfPath shapePath,
                                           InternalAttachmentDeprecated::DirtyEventType eventType)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    uint32_t numAttachmentsUpdated = 0;

    const PhysXScenesMap& physxScenes = physxSetup.getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;
        numAttachmentsUpdated += sc->getInternalScene()->updateEventAttachmentsDeprecated(shapeId, shapePath, eventType);
    }

    if (numAttachmentsUpdated > 0 && intActor)
    {
        // Refresh all attachments that has the same rb which the shape is attached to
        std::vector<PxRigidActor*> actorList = { intActor->mActor };
        for (MirrorActor& mirror : intActor->mMirrors)
        {
            actorList.push_back(mirror.actor);
        }

        for (PxRigidActor* actor : actorList)
        {
            const PhysXScenesMap& physxScenes = physxSetup.getPhysXScenes();
            for (PhysXScenesMap::const_reference ref : physxScenes)
            {
                PhysXScene* sc = ref.second;

                std::vector<class InternalAttachmentDeprecated*>& attachementsList =
                    sc->getInternalScene()->mAttachmentsDeprecated;
                for (size_t i = 0; i < attachementsList.size(); ++i)
                {
                    attachementsList[i]->setRefreshAttachmentEvent(actor);
                }
            }
        }
    }
}

ObjectId PhysXUsdPhysicsInterface::createPhysicsAttachmentDeprecated(usdparser::AttachedStage& attachedStage, const SdfPath& path, PhysxAttachmentDesc const& attachmentDesc)
{
    if (attachmentDesc.attachmentEnabled == false)
        return kInvalidObjectId;

    UsdPrim attachmentPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(path);
    if (!attachmentPrim || !attachmentPrim.IsA<PhysxSchemaPhysxPhysicsAttachment>())
    {
        return kInvalidObjectId;
    }

    InternalAttachmentDeprecated* internalAttachment = nullptr;
    void* internalPtr = nullptr;
    PxScene* attachementScene = nullptr;
    if (attachmentDesc.actor[1].objType == eDynamicBody || attachmentDesc.actor[1].objType == eShape || attachmentDesc.actor[1].objType == eArticulationLink)
    {
        // For deformable/cloth-rigid attachments, we add them at the end of the scene creation
        // This is to workaround the issue of attaching to the wrong positions due to changes in CMassLocalPose
        // See finalizeDeformableRigidAttachments()
        if (attachmentDesc.actor[0].objType == eSoftBody)
        {
            internalAttachment = createSoftBodyRigidBodyAttachment(attachedStage, path, attachmentDesc);
        }
        else if (attachmentDesc.actor[0].objType == eParticleCloth)
        {
            internalAttachment = createParticleClothRigidBodyAttachment(attachedStage, path, attachmentDesc);
        }
        else if (attachmentDesc.actor[0].objType == eFEMCloth)
        {
            internalAttachment = createDeformableSurfaceRigidBodyAttachment(attachedStage, path, attachmentDesc);
        }

        if (internalAttachment)
        {
            PxRigidActor* rigidActor = internalAttachment->mAttachmentActor[1].mRigidActor;
            if (rigidActor)
            {
                mAttachmentsToCreateDeprecated.push_back(internalAttachment);
            }
        }
    }
    else if (attachmentDesc.actor[1].objType == ePhysXAttachmentTargetWorld)
    {
        if (attachmentDesc.actor[0].objType == eParticleCloth)
        {
            internalAttachment = createParticleClothStaticAttachment(path, attachmentDesc);
        }
        else if (attachmentDesc.actor[0].objType == eSoftBody)
        {
            internalAttachment = createSoftBodyStaticAttachment(path, attachmentDesc);
        }
        else if (attachmentDesc.actor[0].objType == eFEMCloth)
        {
            internalAttachment = createDeformableSurfaceStaticAttachment(path, attachmentDesc);
        }
    }
    else if (attachmentDesc.actor[0].objType == eSoftBody)
    {
        if (attachmentDesc.actor[1].objType == eSoftBody)
        {
            internalAttachment = createSoftBodySoftBodyAttachment(path, attachmentDesc);
        }
        else if (attachmentDesc.actor[1].objType == eParticleCloth)
        {
            internalAttachment = createSoftBodyParticleClothAttachment(path, attachmentDesc);
        }
        else if (attachmentDesc.actor[1].objType == eFEMCloth)
        {
            internalAttachment = createSoftBodyDeformableSurfaceAttachment(path, attachmentDesc);
        }
    }

    if (internalAttachment)
    {
        if (attachmentDesc.actor[0].objType == eParticleCloth)
        {
            InternalParticleClothDeprecated* particleCloth = getInternalPtr<InternalParticleClothDeprecated>(omni::physx::ePTParticleClothDeprecated, attachmentDesc.actor[0].objId);
            if (particleCloth)
            {
                attachementScene = particleCloth->mParentParticleSystem->mPS->getScene();
            }
        }
        else if (attachmentDesc.actor[0].objType == eSoftBody)
        {
            InternalDeformableBodyDeprecated* deformableBody = getInternalPtr<InternalDeformableBodyDeprecated>(omni::physx::ePTSoftBodyDeprecated, attachmentDesc.actor[0].objId);
            if (deformableBody)
            {
                attachementScene = deformableBody->mSoftBody->getScene();
            }
        }
        else if (attachmentDesc.actor[0].objType == eFEMCloth)
        {
            InternalDeformableSurfaceDeprecated* deformableSurface = getInternalPtr<InternalDeformableSurfaceDeprecated>(omni::physx::ePTFEMClothDeprecated, attachmentDesc.actor[0].objId);
            if (deformableSurface)
            {
                attachementScene = deformableSurface->mDeformableSurface->getScene();
            }
        }
    }

    if (internalAttachment && attachementScene)
    {
        const ObjectId objId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(ePTAttachmentDeprecated, nullptr, internalAttachment, path);
        attachedStage.registerObjectId(SdfPath(path), ePhysxAttachment, objId);

        InternalScene* internalScene = getInternalPtr<InternalScene>(ePTScene, omni::physx::usdparser::ObjectId(attachementScene->userData));
        internalAttachment->mInternalScene = internalScene;
        internalAttachment->mObjectId = objId;
        internalAttachment->mPath = path;
        internalScene->addAttachmentDeprecated(*internalAttachment);

        // check attachment history
        {
            AttachmentHistoryMapDeprecated& history = attachedStage.getAttachmentHistoryMapDeprecated();

            AttachmentHistoryMapDeprecated::const_iterator it = history.begin();
            AttachmentHistoryMapDeprecated::const_iterator itEnd = history.end();
            while (it != itEnd)
            {
                if (it->second == path)
                {
                    it = history.erase(it);
                }
                else
                {
                    it++;
                }
            }
        }

        return objId;
    }

    SAFE_DELETE_SINGLE(internalAttachment);

    return kInvalidObjectId;
}

void PhysXUsdPhysicsInterface::finalizeDeformableRigidAttachmentsDeprecated()
{
    if (mAttachmentsToCreateDeprecated.empty())
        return;

    // For deformable-rigid attachments, we add them at the end of the scene creation
    for (size_t attachmentId = 0; attachmentId < mAttachmentsToCreateDeprecated.size(); attachmentId++)
    {
        InternalAttachmentDeprecated* internalAttachment = mAttachmentsToCreateDeprecated[attachmentId];
        InternalAttachmentDeprecated::AttachmentActor* actor[2] = { &internalAttachment->mAttachmentActor[0], &internalAttachment->mAttachmentActor[1] };

        if (internalAttachment->mAttachmentType == InternalAttachmentDeprecated::eDeformableBodyRigidBody)
        {
            InternalDeformableBodyDeprecated* deformableBody = (InternalDeformableBodyDeprecated*)(actor[0]->mObjectInternalPtr);

            internalAttachment->mDeformableAttachment = deformableBody->addRigidAttachments(actor[1]->mRigidActor, actor[0]->mDeformableAttachmentIndices, actor[0]->mDeformableAttachmentCoords, actor[1]->mDeformableAttachmentCoords);
            internalAttachment->mDeformableFilter = deformableBody->addRigidFilters(actor[1]->mRigidActor, actor[0]->mDeformableFilterIndices);
        }
        else if (internalAttachment->mAttachmentType == InternalAttachmentDeprecated::eParticleClothRigidBody)
        {
            InternalParticleClothDeprecated* internalParticleCloth = (InternalParticleClothDeprecated*)(actor[0]->mObjectInternalPtr);

            for (size_t i = 0; i < actor[0]->mParticleAttachmentIds.size(); i++)
            {
                PxU32 particleId = actor[0]->mParticleAttachmentIds[i];
                internalParticleCloth->addRigidAttachment(actor[1]->mRigidActor, particleId, (PxVec3&)actor[1]->mRigidActorPositions[i]);
            }

            for (size_t i = 0; i < actor[0]->mParticleFilterIds.size(); i++)
            {
                internalParticleCloth->addRigidFilter(actor[1]->mRigidActor, actor[0]->mParticleFilterIds[i]);
            }

#if USE_PHYSX_GPU
            if (internalParticleCloth->mAttachments)
                internalParticleCloth->mAttachments->copyToDevice();
#endif
        }
        else if (internalAttachment->mAttachmentType == InternalAttachmentDeprecated::eDeformableSurfaceRigidBody)
        {
            InternalDeformableSurfaceDeprecated* deformableSurface = (InternalDeformableSurfaceDeprecated*)(actor[0]->mObjectInternalPtr);

            internalAttachment->mDeformableAttachment = deformableSurface->addRigidAttachments(actor[1]->mRigidActor, actor[0]->mDeformableAttachmentIndices, actor[1]->mDeformableAttachmentCoords);
            internalAttachment->mDeformableFilter = deformableSurface->addRigidFilters(actor[1]->mRigidActor, actor[0]->mDeformableFilterIndices);
        }
    }

    mAttachmentsToCreateDeprecated.clear();
}

void PhysXUsdPhysicsInterface::updateDeformableAttachmentsDeprecated()
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();

    std::vector<internal::InternalAttachmentDeprecated*> attachmentsToRemove;

    const PhysXScenesMap& physxScenes = omniPhysX.getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;

        std::vector<InternalAttachmentDeprecated*>& attachements = sc->getInternalScene()->mAttachmentsDeprecated;
        for (size_t i = 0; i < attachements.size(); ++i)
        {
            if (attachements[i]->mDirtyEvent == InternalAttachmentDeprecated::eShapeRemoved)
                attachmentsToRemove.push_back(attachements[i]);
        }
    }

    for (size_t i = 0; i < attachmentsToRemove.size(); i++)
    {
        InternalScene* internalScene = attachmentsToRemove[i]->mInternalScene;

        if (internalScene)
        {
            internalScene->removeAttachmentsDeprecated(attachmentsToRemove[i]->mAttachmentShapeRemovedEvent.removeShapeId);
        }
    }
}

} // namespace physx
} // namespace omni
