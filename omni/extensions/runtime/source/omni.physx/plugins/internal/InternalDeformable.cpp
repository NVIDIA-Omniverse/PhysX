// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "InternalDeformable.h"
#include "InternalDeformableAttachment.h"
#include "InternalScene.h"
#include "InternalTools.h"

#include <PhysXScene.h>
#include <OmniPhysX.h>

#include <carb/settings/ISettings.h>
#include <PhysXSettings.h>

#include "cudamanager/PxCudaContext.h"
#include <common/utilities/MemoryMacros.h>

#include <attachment/PhysXAttachment.h>
#include <attachment/PhysXTetFinder.h>
#include <attachment/PhysXPointFinder.h>
#include <attachment/PhysXTriFinder.h>

#if USE_PHYSX_GPU
#include "extensions/PxDeformableVolumeExt.h"
#include "extensions/PxCudaHelpersExt.h"
#endif

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace pxr;
using namespace carb;
using namespace ::physx;
using namespace physx::Ext;

#ifdef REMOVE_LATER_TODO

PxVec4* InternalDeformableBody::getKinematicTargetsH()
{
    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();

    if (!cudaContextManager)
        return nullptr;

    if (!mKinematicTargetsD)
        mKinematicTargetsD = PX_DEVICE_ALLOC_T(PxVec4, cudaContextManager, mNumSimMeshVertices);

    if (!mKinematicTargetsH)
    {
        mKinematicTargetsH = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, mNumSimMeshVertices);
        const PxVec4 targetInactive = PxConfigureDeformableVolumeKinematicTarget(PxVec4(0.0f), false);
        for (uint32_t i = 0; i < mNumSimMeshVertices; ++i)
        {
            mKinematicTargetsH[i] = targetInactive;
        }
    }

    return mKinematicTargetsH;
}

void InternalDeformableBody::uploadKinematicTargets(PxDeformableVolumeFlags flags)
{
    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();

    if (mKinematicTargetsD && mKinematicTargetsH && cudaContextManager)
    {
        PxScopedCudaLock _lock(*cudaContextManager);

        cudaContextManager->getCudaContext()->memcpyHtoDAsync(reinterpret_cast<CUdeviceptr>(mKinematicTargetsD), mKinematicTargetsH, mNumSimMeshVertices * sizeof(PxVec4), mPhysXScene->getInternalScene()->getDeformableCopyStream()); //TODO async.

        if (!mKinematicTargetsSet)
        {
            mDeformableVolume->setKinematicTargetBufferD(mKinematicTargetsD, flags); 
            mKinematicTargetsSet = true;
        }
    }
}

void InternalDeformableBody::findTetsFromVtxIndices(const std::vector<PxU32>& vtxIds, std::vector<PxU32>& tetIds, std::vector<PxVec4>& tetBarycentrics)
{
    const PxTetrahedronMesh* collisionMesh = mDeformableVolume->getCollisionMesh();
    const carb::Float3* collisionVertices = (const carb::Float3*)collisionMesh->getVertices();
    uint32_t collisionVerticesSize = collisionMesh->getNbVertices();
    const uint32_t* collisionMeshIndices = (uint32_t*)collisionMesh->getTetrahedrons();
    uint32_t collisionMeshIndicesSize = collisionMesh->getNbTetrahedrons() * 4;

    uint64_t tetFinderPositions = omni::tetfinder::createTetFinder(collisionVertices, collisionVerticesSize, collisionMeshIndices, collisionMeshIndicesSize);

    tetIds.resize(vtxIds.size());
    tetBarycentrics.resize(vtxIds.size());
    std::vector<carb::Float3> points(vtxIds.size());

    for (uint32_t i = 0; i < points.size(); i++)
    {
        points[i] = collisionVertices[vtxIds[i]];
    }
    omni::tetfinder::pointsToTetMeshLocal((int32_t*)&tetIds[0], (carb::Float4*)&tetBarycentrics[0], tetFinderPositions, &points[0], uint32_t(points.size()));
}

void InternalDeformableBody::convertToPhysxAttachmentTets(std::vector<PxU32>& tetIds, std::vector<PxVec4>& tetBarycentrics)
{
    for (uint32_t i = 0; i < tetIds.size(); i++)
    {
        PxDeformableVolumeExt::convertCollisionToSimulationTet(*mDeformableVolume, tetIds[i], tetBarycentrics[i], tetIds[i], tetBarycentrics[i]);
    }
}

PxDeformableAttachment* InternalDeformableBody::addRigidAttachments(PxActor* actor, std::vector<PxU32> indices, std::vector<PxVec4> barycentrics, std::vector<PxVec4> coords)
{
    CARB_ASSERT(indices.size() == barycentrics.size());

    PxDeformableAttachmentData desc;

    desc.actor[0] = mDeformableVolume;
    desc.type[0] = PxDeformableAttachmentTargetType::eTETRAHEDRON;
    desc.indices[0].data = indices.data();
    desc.indices[0].count = (PxU32)indices.size();
    desc.coords[0].data = barycentrics.data();
    desc.coords[0].count = (PxU32)barycentrics.size();

    desc.actor[1] = actor;
    desc.type[1] = PxDeformableAttachmentTargetType::eRIGID;
    desc.coords[1].data = coords.data();
    desc.coords[1].count = (PxU32)indices.size();

    return mDeformableVolume->getScene()->getPhysics().createDeformableAttachment(desc);
}

PxDeformableElementFilter* InternalDeformableBody::addRigidFilters(PxActor* actor, std::vector<PxU32> indices)
{
    PxDeformableElementFilterData desc;

    desc.actor[0] = mDeformableVolume;
    PxU32 groupCount0 = (PxU32)indices.size();
    desc.groupElementCounts[0].data = &groupCount0;
    desc.groupElementCounts[0].count = 1;
    desc.groupElementIndices[0].data = indices.data();
    desc.groupElementIndices[0].count = (PxU32)indices.size();

    desc.actor[1] = actor;

    return mDeformableVolume->getScene()->getPhysics().createDeformableElementFilter(desc);
}

PxDeformableAttachment* InternalDeformableBody::addDeformableVolumeAttachments(PxActor* actor1, std::vector<PxU32> indices1, std::vector<PxVec4> barycentrics1, std::vector<PxU32> indices0, std::vector<PxVec4> barycentrics0)
{
    CARB_ASSERT(indices0.size() == barycentrics0.size());
    CARB_ASSERT(indices1.size() == barycentrics1.size());

    PxDeformableAttachmentData desc;

    desc.actor[0] = mDeformableVolume;
    desc.type[0] = PxDeformableAttachmentTargetType::eTETRAHEDRON;
    desc.indices[0].data = indices0.data();
    desc.indices[0].count = (PxU32)indices0.size();
    desc.coords[0].data = barycentrics0.data();
    desc.coords[0].count = (PxU32)barycentrics0.size();

    desc.actor[1] = actor1;
    if (actor1->getConcreteType() == PxConcreteType::eDEFORMABLE_SURFACE)
        desc.type[1] = PxDeformableAttachmentTargetType::eTRIANGLE;
    else if (actor1->getConcreteType() == PxConcreteType::eDEFORMABLE_VOLUME)
        desc.type[1] = PxDeformableAttachmentTargetType::eTETRAHEDRON;
    desc.indices[1].data = indices1.data();
    desc.indices[1].count = (PxU32)indices1.size();
    desc.coords[1].data = barycentrics1.data();
    desc.coords[1].count = (PxU32)barycentrics1.size();

    return mDeformableVolume->getScene()->getPhysics().createDeformableAttachment(desc);
}

PxDeformableElementFilter* InternalDeformableBody::addDeformableVolumeFilters(PxActor* actor1, std::vector<PxU32> indices1, std::vector<PxU32> indices0)
{
    CARB_ASSERT(indices0.size() == indices1.size());

    PxDeformableElementFilterData desc;

    std::vector<PxU32> groupCount(indices0.size());

    for (uint32_t i = 0; i < indices0.size(); i++)
    {
        groupCount[i] = 1;
    }

    desc.actor[0] = mDeformableVolume;
    desc.groupElementCounts[0].data = groupCount.data();
    desc.groupElementCounts[0].count = (PxU32)groupCount.size();
    desc.groupElementIndices[0].data = indices0.data();
    desc.groupElementIndices[0].count = (PxU32)indices0.size();

    desc.actor[1] = actor1;
    desc.groupElementCounts[1].data = groupCount.data();
    desc.groupElementCounts[1].count = (PxU32)groupCount.size();
    desc.groupElementIndices[1].data = indices1.data();
    desc.groupElementIndices[1].count = (PxU32)indices1.size();

    return mDeformableVolume->getScene()->getPhysics().createDeformableElementFilter(desc);
}

void InternalDeformableSurface::findTrisFromVtxIndices(const std::vector<PxU32>& vtxIds, std::vector<PxU32>& triIds, std::vector<PxVec4>& triBarycentrics)
{
    const PxTriangleMesh* triMesh = mTriangleMesh;
    uint64_t triFinderPositions = omni::trifinder::createTriFinder((const carb::Float3*)triMesh->getVertices(), uint32_t(triMesh->getNbVertices()), (uint32_t*)triMesh->getTriangles(), triMesh->getNbTriangles() * 3);

    triIds.resize(vtxIds.size());
    triBarycentrics.resize(vtxIds.size());

    std::vector<carb::Float3> points(vtxIds.size());
    std::vector<carb::Float3> barycentrics(vtxIds.size());

    for (uint32_t i = 0; i < points.size(); i++)
    {
        points[i] = ((const carb::Float3*)triMesh->getVertices())[vtxIds[i]];
    }
    omni::trifinder::pointsToTriMeshLocal((int32_t*)&triIds[0], &barycentrics[0], triFinderPositions, &points[0], uint32_t(points.size()));

    for (uint32_t i = 0; i < barycentrics.size(); i++)
    {
        triBarycentrics[i] = PxVec4(barycentrics[i].x, barycentrics[i].y, barycentrics[i].z, 0.0f);
    }
}

PxDeformableAttachment* InternalDeformableSurface::addRigidAttachments(PxActor* actor, std::vector<PxU32> indices, std::vector<PxVec4> coords)
{
    PxDeformableAttachmentData desc;

    desc.actor[0] = mDeformableSurface;
    desc.type[0] = PxDeformableAttachmentTargetType::eVERTEX;
    desc.indices[0].data = indices.data();
    desc.indices[0].count = (PxU32)indices.size();

    desc.actor[1] = actor;
    desc.type[1] = PxDeformableAttachmentTargetType::eRIGID;
    desc.coords[1].data = coords.data();
    desc.coords[1].count = (PxU32)indices.size();

    return mDeformableSurface->getScene()->getPhysics().createDeformableAttachment(desc);
}

PxDeformableElementFilter* InternalDeformableSurface::addRigidFilters(PxActor* actor, std::vector<PxU32> indices)
{
    PxDeformableElementFilterData desc;

    desc.actor[0] = mDeformableSurface;
    PxU32 groupCount0 = (PxU32)indices.size();
    desc.groupElementCounts[0].data = &groupCount0;
    desc.groupElementCounts[0].count = 1;
    desc.groupElementIndices[0].data = indices.data();
    desc.groupElementIndices[0].count = (PxU32)indices.size();

    desc.actor[1] = actor;

    return mDeformableSurface->getScene()->getPhysics().createDeformableElementFilter(desc);
}

#endif // REMOVE_LATER_TODO

InternalDeformableBody::InternalDeformableBody()
    : mMaterialId(usdparser::kInvalidObjectId)
    , mIsKinematic(false)
    , mBodyMass(0.0f)
    , mWorldToSimMesh(1.0)
    , mNumSkinMeshVertices(0)
    , mNumSimMeshVertices(0)
    , mSimMeshPositionInvMassH(nullptr)
    , mSimMeshVelocityH(nullptr)
    , mPhysXScene(nullptr)
    , mAllSkinnedVerticesH(nullptr)
    , mAllSkinnedVerticesD(nullptr)
{
}

InternalDeformableBody ::~InternalDeformableBody()
{
    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager)
        return;

    if (mSimMeshPositionInvMassH)
        PX_PINNED_HOST_FREE(cudaContextManager, mSimMeshPositionInvMassH);

    if (mSimMeshVelocityH)
        PX_PINNED_HOST_FREE(cudaContextManager, mSimMeshVelocityH);

    if (mAllSkinnedVerticesH)
        PX_PINNED_HOST_FREE(cudaContextManager, mAllSkinnedVerticesH);

    if (mAllSkinnedVerticesD)
        PX_DEVICE_FREE(cudaContextManager, mAllSkinnedVerticesD);
}

InternalVolumeDeformableBody::InternalVolumeDeformableBody()
    : InternalDeformableBody()
    , mWorldToCollMesh(0.0f)
    , mNumCollMeshVertices(0)
    , mCollMeshPositionInvMassH(nullptr)
    , mDeformableVolume(nullptr)
    , mDeformableVolumeMesh(nullptr)
    , mSimMeshTetIndicesD(nullptr)
    , mSkinningEmbeddingInfoD(nullptr)
{
}

InternalVolumeDeformableBody::~InternalVolumeDeformableBody()
{
    // Remove softbody from scene and release
    if (mDeformableVolume)
    {
        if (mDeformableVolume->getScene())
        {
            mDeformableVolume->getScene()->removeActor(*mDeformableVolume, false);
        }
        SAFE_RELEASE(mDeformableVolume);
    }

    SAFE_RELEASE(mDeformableVolumeMesh);

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager)
        return;

    if (mCollMeshPositionInvMassH)
        PX_PINNED_HOST_FREE(cudaContextManager, mCollMeshPositionInvMassH);

    if (mSimMeshTetIndicesD)
        PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, mSimMeshTetIndicesD);

    if (mSkinningEmbeddingInfoD)
        PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, mSkinningEmbeddingInfoD);
}

bool InternalVolumeDeformableBody::findTetsFromVtxIndices(const std::vector<PxU32>& vtxIds, std::vector<PxU32>& tetIds, std::vector<PxVec4>& tetBarycentrics, bool forSimMesh)
{
    const uint32_t* tetMeshIndices = nullptr;
    uint32_t tetMeshNumTets = 0;
    if (forSimMesh)
    {
        const PxTetrahedronMesh* tetMesh = mDeformableVolume->getSimulationMesh();
        tetMeshIndices = (const uint32_t*)tetMesh->getTetrahedrons();
        tetMeshNumTets = tetMesh->getNbTetrahedrons();
    }
    else
    {
        const PxTetrahedronMesh* tetMesh = mDeformableVolume->getCollisionMesh();
        tetMeshIndices = (const uint32_t*)tetMesh->getTetrahedrons();
        tetMeshNumTets = tetMesh->getNbTetrahedrons();
    }
    
    tetIds.resize(vtxIds.size());
    tetBarycentrics.resize(vtxIds.size());

    return tetfinder::tetMeshVtxToTetLocal((uint32_t*)tetIds.data(), (carb::Float4*)tetBarycentrics.data(), vtxIds.data(), uint32_t(vtxIds.size()),
        tetMeshIndices, tetMeshNumTets);
}

bool InternalVolumeDeformableBody::findTetsFromPoints(const std::vector<PxVec3>& points, std::vector<PxU32>& tetIds, std::vector<PxVec4>& tetBarycentrics, bool forSimMesh)
{
    const PxTetrahedronMesh* collisionMesh = mDeformableVolume->getCollisionMesh();
    uint64_t tetFinderPositions = omni::tetfinder::createTetFinder((const carb::Float4*)mCollMeshPositionInvMassH, mNumCollMeshVertices, (uint32_t*)collisionMesh->getTetrahedrons(), collisionMesh->getNbTetrahedrons() * 4);

    tetIds.resize(points.size());
    tetBarycentrics.resize(points.size());

    omni::tetfinder::pointsToTetMeshLocal((int32_t*)&tetIds[0], (carb::Float4*)&tetBarycentrics[0], tetFinderPositions, (carb::Float3*)&points[0], uint32_t(points.size()));

    // check and make sure there are no invalid tet ids
    for (uint32_t i = 0; i < tetIds.size(); i++)
    {
        if (tetIds[i] == -1)
            return false;
    }

    if (forSimMesh)
    {
        convertToPhysxAttachmentTets(tetIds, tetBarycentrics);
    }

    return true;
}

void InternalVolumeDeformableBody::convertToPhysxAttachmentTets(std::vector<PxU32>& tetIds, std::vector<PxVec4>& tetBarycentrics)
{
    for (uint32_t i = 0; i < tetIds.size(); i++)
    {
        PxDeformableVolumeExt::convertCollisionToSimulationTet(*mDeformableVolume, tetIds[i], tetBarycentrics[i], tetIds[i], tetBarycentrics[i]);
    }
}

PxDeformableAttachment* InternalVolumeDeformableBody::addRigidAttachments(PxActor* actor, std::vector<PxU32> indices, std::vector<PxVec4> barycentrics, std::vector<PxVec4> coords)
{
    CARB_ASSERT(indices.size() == barycentrics.size());

    PxDeformableAttachmentData desc;

    desc.actor[0] = mDeformableVolume;
    desc.type[0] = PxDeformableAttachmentTargetType::eTETRAHEDRON;
    desc.indices[0].data = indices.data();
    desc.indices[0].count = (PxU32)indices.size();
    desc.coords[0].data = barycentrics.data();
    desc.coords[0].count = (PxU32)barycentrics.size();

    desc.actor[1] = actor;
    desc.type[1] = PxDeformableAttachmentTargetType::eRIGID;
    desc.coords[1].data = coords.data();
    desc.coords[1].count = (PxU32)indices.size();

    return mDeformableVolume->getScene()->getPhysics().createDeformableAttachment(desc);
}

PxDeformableElementFilter* InternalVolumeDeformableBody::addRigidFilters(PxActor* actor, std::vector<PxU32> indices)
{
    PxDeformableElementFilterData desc;

    desc.actor[0] = mDeformableVolume;
    PxU32 groupCount0 = (PxU32)indices.size();
    desc.groupElementCounts[0].data = &groupCount0;
    desc.groupElementCounts[0].count = 1;
    desc.groupElementIndices[0].data = indices.data();
    desc.groupElementIndices[0].count = (PxU32)indices.size();

    desc.actor[1] = actor;

    return mDeformableVolume->getScene()->getPhysics().createDeformableElementFilter(desc);
}

InternalSurfaceDeformableBody::InternalSurfaceDeformableBody()
    : InternalDeformableBody()
    , mDeformableSurface(nullptr)
    , mTriangleMesh(nullptr)
    , mSimMeshTriIndicesD(nullptr)
    , mSkinningEmbeddingInfoD(nullptr)
    , mNormalVectorsD(nullptr)
{
}

InternalSurfaceDeformableBody::~InternalSurfaceDeformableBody()
{
    // Remove deformable surface from scene and release
    if (mDeformableSurface)
    {
        if (mDeformableSurface->getScene())
        {
            mDeformableSurface->getScene()->removeActor(*mDeformableSurface, false);
        }
        SAFE_RELEASE(mDeformableSurface);
    }
    if (mTriangleMesh)
    {
        SAFE_RELEASE(mTriangleMesh)
    }

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager)
        return;

    if (mSimMeshTriIndicesD)
    {
        PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, mSimMeshTriIndicesD);
    }

    if (mSkinningEmbeddingInfoD)
    {
        PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, mSkinningEmbeddingInfoD);
    }
}

bool InternalSurfaceDeformableBody::findTrisFromVtxIndices(const std::vector<PxU32>& vtxIds, std::vector<PxU32>& triIds, std::vector<PxVec4>& triBarycentrics)
{
    const PxTriangleMesh* triMesh = mTriangleMesh;
    uint64_t triFinderPositions = omni::trifinder::createTriFinder((const carb::Float3*)triMesh->getVertices(), uint32_t(triMesh->getNbVertices()), (uint32_t*)triMesh->getTriangles(), triMesh->getNbTriangles() * 3);

    triIds.resize(vtxIds.size());
    triBarycentrics.resize(vtxIds.size());

    std::vector<carb::Float3> points(vtxIds.size());
    std::vector<carb::Float3> barycentrics(vtxIds.size());

    for (uint32_t i = 0; i < points.size(); i++)
    {
        points[i] = ((const carb::Float3*)triMesh->getVertices())[vtxIds[i]];
    }
    omni::trifinder::pointsToTriMeshLocal((int32_t*)triIds.data(), barycentrics.data(), triFinderPositions, points.data(), uint32_t(points.size()));

    for (uint32_t i = 0; i < barycentrics.size(); i++)
    {
        // check and make sure there are no invalid tri ids
        if (triIds[i] == -1)
            return false;

        triBarycentrics[i] = PxVec4(barycentrics[i].x, barycentrics[i].y, barycentrics[i].z, 0.0f);
    }

    return true;
}

bool InternalSurfaceDeformableBody::findTrisFromPoints(const std::vector<PxVec3>& points, std::vector<PxU32>& triIds, std::vector<PxVec4>& triBarycentrics)
{
    const PxTriangleMesh* triMesh = mTriangleMesh;
    uint64_t triFinderRestPositions = omni::trifinder::createTriFinder((const carb::Float4*)mSimMeshPositionInvMassH, mNumSimMeshVertices, (uint32_t*)triMesh->getTriangles(), triMesh->getNbTriangles() * 3);

    triIds.resize(points.size());
    triBarycentrics.resize(points.size());

    std::vector<carb::Float3> barycentrics(points.size());

    omni::trifinder::pointsToTriMeshLocal((int32_t*)&triIds[0], &barycentrics[0], triFinderRestPositions, (carb::Float3*)&points[0], uint32_t(points.size()));

    for (uint32_t i = 0; i < barycentrics.size(); i++)
    {
        // check and make sure there are no invalid tri ids
        if (triIds[i] == -1)
            return false;

        triBarycentrics[i] = PxVec4(barycentrics[i].x, barycentrics[i].y, barycentrics[i].z, 0.0f);
    }

    return true;
}

PxDeformableAttachment* InternalSurfaceDeformableBody::addRigidAttachments(PxActor* actor, std::vector<PxU32> indices, std::vector<PxVec4> barycentrics, std::vector<PxVec4> coords)
{
    PxDeformableAttachmentData desc;

    desc.actor[0] = mDeformableSurface;
    desc.type[0] = PxDeformableAttachmentTargetType::eTRIANGLE;
    desc.indices[0].data = indices.data();
    desc.indices[0].count = (PxU32)indices.size();
    desc.coords[0].data = barycentrics.data();
    desc.coords[0].count = (PxU32)barycentrics.size();

    desc.actor[1] = actor;
    desc.type[1] = PxDeformableAttachmentTargetType::eRIGID;
    desc.coords[1].data = coords.data();
    desc.coords[1].count = (PxU32)indices.size();

    return mDeformableSurface->getScene()->getPhysics().createDeformableAttachment(desc);
}

PxDeformableElementFilter* InternalSurfaceDeformableBody::addRigidFilters(PxActor* actor, std::vector<PxU32> indices)
{
    PxDeformableElementFilterData desc;

    desc.actor[0] = mDeformableSurface;
    PxU32 groupCount0 = (PxU32)indices.size();
    desc.groupElementCounts[0].data = &groupCount0;
    desc.groupElementCounts[0].count = 1;
    desc.groupElementIndices[0].data = indices.data();
    desc.groupElementIndices[0].count = (PxU32)indices.size();

    desc.actor[1] = actor;

    return mDeformableSurface->getScene()->getPhysics().createDeformableElementFilter(desc);
}
