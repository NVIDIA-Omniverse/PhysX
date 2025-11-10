// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "InternalDeformableDeprecated.h"
#include "InternalAttachmentDeprecated.h"
#include "InternalScene.h"
#include "InternalTools.h"

#include <PhysXScene.h>
#include <OmniPhysX.h>

#include <carb/settings/ISettings.h>
#include <PhysXSettings.h>

#include "cudamanager/PxCudaContext.h"
#include <common/utilities/MemoryMacros.h>

#include <attachment/PhysXAttachmentDeprecated.h>
#include <attachment/PhysXTetFinder.h>
#include <attachment/PhysXPointFinder.h>
#include <attachment/PhysXTriFinder.h>

#include "extensions/PxSoftBodyExt.h"

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace pxr;
using namespace carb;
using namespace ::physx;

InternalDeformableBodyDeprecated::InternalDeformableBodyDeprecated(PhysXScene* ps, pxr::UsdPrim& prim)
    : mSoftBody(nullptr),
    mSoftBodyMesh(nullptr),
    mSimPositionInvMassH(nullptr),
    mSimVelocityH(nullptr),
    mCollPositionInvMassH(nullptr),
    mCollRestPositionH(nullptr),
    mKinematicTargetsH(nullptr),
    mKinematicTargetsD(nullptr),
    mPrim(prim),
    mNumCollMeshVertices(0),
    mNumSimMeshVertices(0),
    mNumSkinMeshVertices(0),
    mNumCollSkinMeshVertices(0),
    mKinematicTargetsSet(false)
{
    // store initial xform ops
    mXformOpStorage.store(pxr::UsdGeomXformable(prim));
    mSkipUpdateTransform = false;
    mPhysXScene = ps;

    const bool updateUSD = OmniPhysX::getInstance().getISettings()->getAsBool(kSettingUpdateToUsd);
    if (updateUSD)
    {
        if (OmniPhysX::getInstance().getSimulationLayer())
        {
            pxr::UsdEditContext editContext(OmniPhysX::getInstance().getStage(), UsdEditTarget(OmniPhysX::getInstance().getSimulationLayer()));
            // if we fail to setup transformations skip the prim for update
            if (!setupTransformOpsAsScaleOrientTranslate(prim))
            {
                CARB_LOG_WARN("Failed to create transformation stack (position, quat, scale) on a deformable body: %s, DeformableBody won't get transformation updates.",
                    prim.GetPrimPath().GetText());
                mSkipUpdateTransform = true;
            }
        }
        else
        {
            // if we fail to setup transformations skip the prim for update
            if (!setupTransformOpsAsScaleOrientTranslate(prim))
            {
                CARB_LOG_WARN("Failed to create transformation stack (position, quat, scale) on a deformable body: %s", prim.GetPrimPath().GetText());
                mSkipUpdateTransform = true;
            }
        }
    }
}

InternalDeformableBodyDeprecated::~InternalDeformableBodyDeprecated()
{
    // Remove softbody from scene
    if (mSoftBody && mSoftBody->getScene())
    {
        mSoftBody->getScene()->removeActor(*mSoftBody, false);
    }

    SAFE_RELEASE(mSoftBody);
    SAFE_RELEASE(mSoftBodyMesh);

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();

    if (!cudaContextManager)
        return;

    if (mSimPositionInvMassH)
        PX_PINNED_HOST_FREE(cudaContextManager, mSimPositionInvMassH);

    if (mSimVelocityH)
        PX_PINNED_HOST_FREE(cudaContextManager, mSimVelocityH);

    if (mCollPositionInvMassH)
        PX_PINNED_HOST_FREE(cudaContextManager, mCollPositionInvMassH);

    if (mCollRestPositionH)
        PX_PINNED_HOST_FREE(cudaContextManager, mCollRestPositionH);

    if (mKinematicTargetsH)
        PX_PINNED_HOST_FREE(cudaContextManager, mKinematicTargetsH);

    if (mKinematicTargetsD)
        PX_DEVICE_FREE(cudaContextManager, mKinematicTargetsD);
}

PxVec4* InternalDeformableBodyDeprecated::getKinematicTargetsH()
{
    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();

    if (!cudaContextManager)
        return nullptr;

    if (!mKinematicTargetsD)
        mKinematicTargetsD = PX_DEVICE_ALLOC_T(PxVec4, cudaContextManager, mNumSimMeshVertices);

    if (!mKinematicTargetsH)
    {
        mKinematicTargetsH = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, mNumSimMeshVertices);
        const PxVec4 targetInactive = PxConfigureSoftBodyKinematicTarget(PxVec4(0.0f), false);
        for (uint32_t i = 0; i < mNumSimMeshVertices; ++i)
        {
            mKinematicTargetsH[i] = targetInactive;
        }
    }

    return mKinematicTargetsH;
}

void InternalDeformableBodyDeprecated::uploadKinematicTargets(PxSoftBodyFlags flags)
{
    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();

    if (mKinematicTargetsD && mKinematicTargetsH && cudaContextManager)
    {
        PxScopedCudaLock _lock(*cudaContextManager);

        cudaContextManager->getCudaContext()->memcpyHtoDAsync(reinterpret_cast<CUdeviceptr>(mKinematicTargetsD), mKinematicTargetsH, mNumSimMeshVertices * sizeof(PxVec4), mPhysXScene->getInternalScene()->getDeformableCopyStream()); //TODO async.

        if (!mKinematicTargetsSet)
        {
            mSoftBody->setKinematicTargetBufferD(mKinematicTargetsD, flags); 
            mKinematicTargetsSet = true;
        }
    }
}

void InternalDeformableBodyDeprecated::findTetsFromPoints(const std::vector<PxVec3>& points, std::vector<PxU32>& tetIds, std::vector<PxVec4>& tetBarycentrics)
{
    const PxTetrahedronMesh* collisionMesh = mSoftBody->getCollisionMesh();
    uint64_t tetFinderRestPositions = omni::tetfinder::createTetFinder((const carb::Float4 *)mCollPositionInvMassH, mNumCollMeshVertices, (uint32_t*)collisionMesh->getTetrahedrons(), collisionMesh->getNbTetrahedrons() * 4);

    tetIds.resize(points.size());
    tetBarycentrics.resize(points.size());

    omni::tetfinder::pointsToTetMeshLocal((int32_t*)&tetIds[0], (carb::Float4*)&tetBarycentrics[0], tetFinderRestPositions, (carb::Float3*)&points[0], uint32_t(points.size()));
}

void InternalDeformableBodyDeprecated::findTetsFromVtxIndices(const std::vector<PxU32>& vtxIds, std::vector<PxU32>& tetIds, std::vector<PxVec4>& tetBarycentrics)
{
    const pxr::SdfPath& deformablePath = mPrim.GetPath();
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo;

    getDeformableMeshInfoDeprecated(deformablePath, deformableMeshInfo);

    const carb::Float3* posRest = &deformableMeshInfo.restPositions[0];
    uint32_t posRestSize = uint32_t(deformableMeshInfo.restPositions.size());

    const PxTetrahedronMesh* collisionMesh = mSoftBody->getCollisionMesh();
    const uint32_t* collisionMeshIndices = (uint32_t*)collisionMesh->getTetrahedrons();
    uint32_t collisionMeshIndicesSize = collisionMesh->getNbTetrahedrons() * 4;

    uint64_t tetFinderRestPositions = omni::tetfinder::createTetFinder(posRest, posRestSize, collisionMeshIndices, collisionMeshIndicesSize);

    tetIds.resize(vtxIds.size());
    tetBarycentrics.resize(vtxIds.size());
    std::vector<carb::Float3> points(vtxIds.size());

    for (uint32_t i = 0; i < points.size(); i++)
    {
        points[i] = posRest[vtxIds[i]];
    }
    omni::tetfinder::pointsToTetMeshLocal((int32_t*)&tetIds[0], (carb::Float4*)&tetBarycentrics[0], tetFinderRestPositions, &points[0], uint32_t(points.size()));
}

void InternalDeformableBodyDeprecated::convertToPhysxAttachmentTets(std::vector<PxU32>& tetIds, std::vector<PxVec4>& tetBarycentrics)
{
    for (uint32_t i = 0; i < tetIds.size(); i++)
    {
        PxSoftBodyExt::convertCollisionToSimulationTet(*mSoftBody, tetIds[i], tetBarycentrics[i], tetIds[i], tetBarycentrics[i]);
    }
}

PxDeformableAttachment* InternalDeformableBodyDeprecated::addRigidAttachments(PxActor* actor, std::vector<PxU32> indices, std::vector<PxVec4> barycentrics, std::vector<PxVec4> coords)
{
    CARB_ASSERT(indices.size() == barycentrics.size());

    PxDeformableAttachmentData desc;

    desc.actor[0] = mSoftBody;
    desc.type[0] = PxDeformableAttachmentTargetType::eTETRAHEDRON;
    desc.indices[0].data = indices.data();
    desc.indices[0].count = (PxU32)indices.size();
    desc.coords[0].data = barycentrics.data();
    desc.coords[0].count = (PxU32)barycentrics.size();

    desc.actor[1] = actor;
    desc.type[1] = PxDeformableAttachmentTargetType::eRIGID;
    desc.coords[1].data = coords.data();
    desc.coords[1].count = (PxU32)indices.size();

    return mSoftBody->getScene()->getPhysics().createDeformableAttachment(desc);
}

PxDeformableElementFilter* InternalDeformableBodyDeprecated::addRigidFilters(PxActor* actor, std::vector<PxU32> indices)
{
    if (indices.size() == 0)
    {
        return nullptr;
    }

    PxDeformableElementFilterData desc;

    desc.actor[0] = mSoftBody;
    PxU32 groupCount0 = (PxU32)indices.size();
    desc.groupElementCounts[0].data = &groupCount0;
    desc.groupElementCounts[0].count = 1;
    desc.groupElementIndices[0].data = indices.data();
    desc.groupElementIndices[0].count = (PxU32)indices.size();

    desc.actor[1] = actor;

    return mSoftBody->getScene()->getPhysics().createDeformableElementFilter(desc);
}

PxDeformableAttachment* InternalDeformableBodyDeprecated::addSoftBodyAttachments(PxActor* actor1, std::vector<PxU32> indices1, std::vector<PxVec4> barycentrics1, std::vector<PxU32> indices0, std::vector<PxVec4> barycentrics0)
{
    CARB_ASSERT(indices0.size() == barycentrics0.size());
    CARB_ASSERT(indices1.size() == barycentrics1.size());

    PxDeformableAttachmentData desc;

    desc.actor[0] = mSoftBody;
    desc.type[0] = PxDeformableAttachmentTargetType::eTETRAHEDRON;
    desc.indices[0].data = indices0.data();
    desc.indices[0].count = (PxU32)indices0.size();
    desc.coords[0].data = barycentrics0.data();
    desc.coords[0].count = (PxU32)barycentrics0.size();

    desc.actor[1] = actor1;
    if (actor1->getConcreteType() == PxConcreteType::eDEFORMABLE_SURFACE)
        desc.type[1] = PxDeformableAttachmentTargetType::eTRIANGLE;
    else if (actor1->getConcreteType() == PxConcreteType::eSOFT_BODY)
        desc.type[1] = PxDeformableAttachmentTargetType::eTETRAHEDRON;
    desc.indices[1].data = indices1.data();
    desc.indices[1].count = (PxU32)indices1.size();
    desc.coords[1].data = barycentrics1.data();
    desc.coords[1].count = (PxU32)barycentrics1.size();

    return mSoftBody->getScene()->getPhysics().createDeformableAttachment(desc);
}

PxDeformableElementFilter* InternalDeformableBodyDeprecated::addSoftBodyFilters(PxActor* actor1, std::vector<PxU32> indices1, std::vector<PxU32> indices0)
{
    CARB_ASSERT(indices0.size() == indices1.size());

    PxDeformableElementFilterData desc;

    std::vector<PxU32> groupCount(indices0.size());

    for (uint32_t i = 0; i < indices0.size(); i++)
    {
        groupCount[i] = 1;
    }

    desc.actor[0] = mSoftBody;
    desc.groupElementCounts[0].data = groupCount.data();
    desc.groupElementCounts[0].count = (PxU32)groupCount.size();
    desc.groupElementIndices[0].data = indices0.data();
    desc.groupElementIndices[0].count = (PxU32)indices0.size();

    desc.actor[1] = actor1;
    desc.groupElementCounts[1].data = groupCount.data();
    desc.groupElementCounts[1].count = (PxU32)groupCount.size();
    desc.groupElementIndices[1].data = indices1.data();
    desc.groupElementIndices[1].count = (PxU32)indices1.size();

    return mSoftBody->getScene()->getPhysics().createDeformableElementFilter(desc);
}

InternalDeformableSurfaceDeprecated::InternalDeformableSurfaceDeprecated(PhysXScene* ps, pxr::UsdPrim& prim)
    : mPositionInvMassH(nullptr),
    mVelocityH(nullptr),
    mRestPositionH(nullptr),
    mPrim(prim),
    mNumCollMeshVerticesWelded(0),
    mNumCollMeshVerticesOrig(0)
{
    // store initial xform ops
    mXformOpStorage.store(pxr::UsdGeomXformable(prim));
    mSkipUpdateTransform = false;
    mPhysXScene = ps;

    const bool updateUSD = OmniPhysX::getInstance().getISettings()->getAsBool(kSettingUpdateToUsd);
    if (updateUSD)
    {
        if (OmniPhysX::getInstance().getSimulationLayer())
        {
            pxr::UsdEditContext editContext(OmniPhysX::getInstance().getStage(), UsdEditTarget(OmniPhysX::getInstance().getSimulationLayer()));
            // if we fail to setup transformations skip the prim for update
            if (!setupTransformOpsAsScaleOrientTranslate(prim))
            {
                CARB_LOG_WARN("Failed to create transformation stack (position, quat, scale) on a deformable surface: %s, DeformableSurface won't get transformation updates.",
                    prim.GetPrimPath().GetText());
                mSkipUpdateTransform = true;
            }
        }
        else
        {
            // if we fail to setup transformations skip the prim for update
            if (!setupTransformOpsAsScaleOrientTranslate(prim))
            {
                CARB_LOG_WARN("Failed to create transformation stack (position, quat, scale) on a deformable surface: %s", prim.GetPrimPath().GetText());
                mSkipUpdateTransform = true;
            }
        }
    }
}

InternalDeformableSurfaceDeprecated::~InternalDeformableSurfaceDeprecated()
{
    // Remove deformable surface from scene
    if (mDeformableSurface->getScene())
        mDeformableSurface->getScene()->removeActor(*mDeformableSurface, false);

    SAFE_RELEASE(mDeformableSurface);
    SAFE_RELEASE(mTriangleMesh);

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();

    if (!cudaContextManager)
        return;

    if (mPositionInvMassH)
        PX_PINNED_HOST_FREE(cudaContextManager, mPositionInvMassH);

    if (mVelocityH)
        PX_PINNED_HOST_FREE(cudaContextManager, mVelocityH);

    if (mRestPositionH)
        PX_PINNED_HOST_FREE(cudaContextManager, mRestPositionH);
}

void InternalDeformableSurfaceDeprecated::buildMap(const std::vector<uint32_t>& usdSimTriVtxIndices)
{
    PxTriangleMesh* triangleMesh = mTriangleMesh;
    const PxU32 numTris = triangleMesh->getNbTriangles();
    const PxU32* remap = triangleMesh->getTrianglesRemap();
    const PxU32* trianglesSource = reinterpret_cast<const PxU32*>(triangleMesh->getTriangles());

    mPhysxToUsdVtxMap.resize(mNumCollMeshVerticesOrig);
    for (PxU32 i = 0; i < mNumCollMeshVerticesOrig; ++i)
        mPhysxToUsdVtxMap[i] = 0;
    for (PxU32 i = 0; i < numTris; ++i)
    {
        const uint32_t* srcTri = &trianglesSource[3 * i];
        PxU32 j = remap[i];
        const uint32_t* destTri = usdSimTriVtxIndices.data() + j*3;
        mPhysxToUsdVtxMap[destTri[0]] = srcTri[0];
        mPhysxToUsdVtxMap[destTri[1]] = srcTri[1];
        mPhysxToUsdVtxMap[destTri[2]] = srcTri[2];
    }
}

void InternalDeformableSurfaceDeprecated::buildInverseMap(const std::vector<uint32_t>& usdSimTriVtxIndices)
{
    PxTriangleMesh* triangleMesh = mTriangleMesh;
    const PxU32 numTris = triangleMesh->getNbTriangles();
    const PxU32* remap = triangleMesh->getTrianglesRemap();
    const PxU32* trianglesSource = reinterpret_cast<const PxU32*>(triangleMesh->getTriangles());

    mUsdToPhysxVtxMap.resize(mNumCollMeshVerticesWelded);
    for (PxU32 i = 0; i < mNumCollMeshVerticesWelded; ++i)
        mUsdToPhysxVtxMap[i] = 0;
    for (PxU32 i = 0; i < numTris; ++i)
    {
        const uint32_t* srcTri = &trianglesSource[3 * i];
        PxU32 j = remap[i];
        const uint32_t* destTri = usdSimTriVtxIndices.data() + j*3;
        mUsdToPhysxVtxMap[srcTri[0]] = destTri[0];
        mUsdToPhysxVtxMap[srcTri[1]] = destTri[1];
        mUsdToPhysxVtxMap[srcTri[2]] = destTri[2];
    }
}

void InternalDeformableSurfaceDeprecated::findTrisFromPoints(const std::vector<PxVec3>& points, std::vector<PxU32>& triIds, std::vector<PxVec4>& triBarycentrics)
{
    const PxTriangleMesh* triMesh = mTriangleMesh;
    uint64_t triFinderRestPositions = omni::trifinder::createTriFinder((const carb::Float4 *)mPositionInvMassH, mNumCollMeshVerticesOrig, (uint32_t*)triMesh->getTriangles(), triMesh->getNbTriangles() * 3);

    triIds.resize(points.size());
    triBarycentrics.resize(points.size());

    std::vector<carb::Float3> barycentrics(points.size());

    omni::trifinder::pointsToTriMeshLocal((int32_t*)&triIds[0], &barycentrics[0], triFinderRestPositions, (carb::Float3*)&points[0], uint32_t(points.size()));

    for (uint32_t i = 0; i < barycentrics.size(); i++)
    {
        triBarycentrics[i] = PxVec4(barycentrics[i].x, barycentrics[i].y, barycentrics[i].z, 0.0f);
    }
}

void InternalDeformableSurfaceDeprecated::findTrisFromVtxIndices(const std::vector<PxU32>& vtxIds, std::vector<PxU32>& triIds, std::vector<PxVec4>& triBarycentrics)
{
    const pxr::SdfPath& deformablePath = mPrim.GetPath();
    omni::physx::DeformableMeshInfoDeprecated deformableMeshInfo;

    omni::physx::getDeformableMeshInfoDeprecated(deformablePath, deformableMeshInfo);

    const PxTriangleMesh* triMesh = mTriangleMesh;
    uint64_t triFinderRestPositions = omni::trifinder::createTriFinder(&deformableMeshInfo.restPositions[0], uint32_t(deformableMeshInfo.restPositions.size()), (uint32_t*)triMesh->getTriangles(), triMesh->getNbTriangles() * 3);

    triIds.resize(vtxIds.size());
    triBarycentrics.resize(vtxIds.size());

    std::vector<carb::Float3> points(vtxIds.size());
    std::vector<carb::Float3> barycentrics(vtxIds.size());

    for (uint32_t i = 0; i < points.size(); i++)
    {
        points[i] = deformableMeshInfo.restPositions[vtxIds[i]];
    }
    omni::trifinder::pointsToTriMeshLocal((int32_t*)&triIds[0], &barycentrics[0], triFinderRestPositions, &points[0], uint32_t(points.size()));

    for (uint32_t i = 0; i < barycentrics.size(); i++)
    {
        triBarycentrics[i] = PxVec4(barycentrics[i].x, barycentrics[i].y, barycentrics[i].z, 0.0f);
    }
}

PxDeformableAttachment* InternalDeformableSurfaceDeprecated::addRigidAttachments(PxActor* actor, std::vector<PxU32> indices, std::vector<PxVec4> coords)
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

PxDeformableAttachment* InternalDeformableSurfaceDeprecated::addRigidAttachments(PxActor* actor, std::vector<PxU32> indices, std::vector<PxVec4> barycentrics, std::vector<PxVec4> coords)
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

PxDeformableElementFilter* InternalDeformableSurfaceDeprecated::addRigidFilters(PxActor* actor, std::vector<PxU32> indices)
{
    if (indices.size() == 0)
    {
        return nullptr;
    }

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
