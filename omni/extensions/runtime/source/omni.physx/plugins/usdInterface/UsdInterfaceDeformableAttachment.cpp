// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "UsdInterface.h"

#include <private/omni/physx/PhysxUsd.h>

#include <usdLoad/LoadUsd.h>

#include <internal/Internal.h>
#include <internal/InternalDeformableAttachment.h>
#include <internal/InternalDeformable.h>

#include <Setup.h>
#include <OmniPhysX.h>
#include <PhysXTools.h>
#include <ObjectDataQuery.h>
#include <attachment/PhysXAttachment.h>
#include <attachment/PhysXTetFinder.h>
#include <attachment/PhysXPointFinder.h>
#include <attachment/PhysXTriFinder.h>

#include <common/utilities/MemoryMacros.h>

using namespace omni::physx::usdparser;
using namespace pxr;
using namespace ::physx;
using namespace omni::physx::internal;
using namespace omni::physx;

extern ObjectId getObjectId(const pxr::SdfPath& path, PhysXType type);

namespace
{

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

} // namespace


namespace omni
{
namespace physx
{

void copyLocalPositionsToPhysx(InternalDeformableAttachment::AttachmentData& attachmentData,
                               const pxr::VtArray<GfVec3f>& localPositions,
                               ::physx::PxVec3 scale)
{
    copyBuffer(attachmentData.coords, localPositions.data(), (unsigned int)localPositions.size(), scale);
    attachmentData.targetType = (attachmentData.actor != nullptr) ?
                                PxDeformableAttachmentTargetType::eRIGID :
                                PxDeformableAttachmentTargetType::eWORLD;
}

void copyVtxToPhysx(InternalDeformableAttachment::AttachmentData& attachmentData, const pxr::VtArray<int>& vtxIndices)
{
    attachmentData.indices.assign(vtxIndices.begin(), vtxIndices.end());
    attachmentData.targetType = PxDeformableAttachmentTargetType::eVERTEX;
}

void convertVtxToPhysx(InternalDeformableAttachment::AttachmentData& attachmentData, const pxr::VtArray<int>& vtxIndices)
{
    if (attachmentData.physxType == ePTDeformableSurface)
    {
        InternalSurfaceDeformableBody* internalPtr = omni::physx::getInternalPtr<InternalSurfaceDeformableBody>(ePTDeformableSurface, attachmentData.objId);

        std::vector<PxU32> vtxIds;
        std::vector<PxU32> triIds;
        std::vector<PxVec4> triBarycentrics;

        vtxIds.assign(vtxIndices.begin(), vtxIndices.end());
        internalPtr->findTrisFromVtxIndices(vtxIds, triIds, triBarycentrics);

        attachmentData.indices.assign(triIds.begin(), triIds.end());
        copyBuffer(attachmentData.coords, triBarycentrics.data(), (unsigned int)triBarycentrics.size());

        attachmentData.targetType = PxDeformableAttachmentTargetType::eTRIANGLE;
    }
    else if (attachmentData.physxType == ePTDeformableVolume)
    {
        InternalVolumeDeformableBody* internalPtr = omni::physx::getInternalPtr<InternalVolumeDeformableBody>(ePTDeformableVolume, attachmentData.objId);

        std::vector<PxU32> vtxIds;
        std::vector<PxU32> tetIds;
        std::vector<PxVec4> tetBarycentrics;

        vtxIds.assign(vtxIndices.begin(), vtxIndices.end());
        const bool forSimMesh = true;
        if (internalPtr->findTetsFromVtxIndices(vtxIds, tetIds, tetBarycentrics, forSimMesh))
        {
            attachmentData.indices.assign(tetIds.begin(), tetIds.end());
            copyBuffer(attachmentData.coords, tetBarycentrics.data(), (unsigned int)tetBarycentrics.size());
        }

        attachmentData.targetType = PxDeformableAttachmentTargetType::eTETRAHEDRON;
    }
}

void convertTriToPhysx(InternalDeformableAttachment::AttachmentData& attachmentData, const pxr::VtArray<int>& triIds, const pxr::VtArray<GfVec3f>& triBarycentrics)
{
    if (attachmentData.physxType == ePTDeformableSurface)
    {
        // PhysX SDK does not support distance attachment for surface deformable. Set w component is 0.
        attachmentData.indices.resize(triIds.size());
        attachmentData.coords.resize(triIds.size());

        for (PxU32 i = 0; i < triIds.size(); i++)
        {
            attachmentData.indices[i] = triIds[i];
            const float w = 0.0f;
            attachmentData.coords[i] = { triBarycentrics[i][0], triBarycentrics[i][1], triBarycentrics[i][2], w };
        }

        attachmentData.targetType = PxDeformableAttachmentTargetType::eTRIANGLE;
    }
}

void convertTetToPhysx(InternalDeformableAttachment::AttachmentData& attachmentData, const pxr::VtArray<int>& tetIds, const pxr::VtArray<GfVec3f>& tetBarycentrics)
{
    if (attachmentData.physxType == ePTDeformableVolume)
    {
        attachmentData.indices.resize(tetIds.size());
        attachmentData.coords.resize(tetIds.size());

        for (PxU32 i = 0; i < tetIds.size(); i++)
        {
            attachmentData.indices[i] = tetIds[i];
            const float w = 1.0f - tetBarycentrics[i][0] - tetBarycentrics[i][1] - tetBarycentrics[i][2];
            attachmentData.coords[i] = { tetBarycentrics[i][0], tetBarycentrics[i][1], tetBarycentrics[i][2], w };
        }

        attachmentData.targetType = PxDeformableAttachmentTargetType::eTETRAHEDRON;
    }
}

ObjectId PhysXUsdPhysicsInterface::createDeformableAttachment(usdparser::AttachedStage& attachedStage, const SdfPath& path, const PhysxDeformableAttachmentDesc& desc)
{
    InternalDeformableAttachment* internalDeformableAttachment = nullptr;
    const UsdPrim attachmentPrim = attachedStage.getStage()->GetPrimAtPath(path);
    if (!attachmentPrim)
    {
        return kInvalidObjectId;
    }

    switch (desc.type)
    {
        case eAttachmentVtxXform:
        {
            if (!attachmentPrim.IsA(UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment)))
            {
                return kInvalidObjectId;
            }

            internalDeformableAttachment = ICE_NEW(InternalDeformableAttachment)(path, desc);
            if (!internalDeformableAttachment->isValid())
            {
                SAFE_DELETE_SINGLE(internalDeformableAttachment);
                return kInvalidObjectId;
            }

            pxr::VtArray<int> vtxIndices;
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0).Get(&vtxIndices);
            copyVtxToPhysx(internalDeformableAttachment->mData[0], vtxIndices);

            pxr::VtArray<GfVec3f> localPositions;
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->localPositionsSrc1).Get(&localPositions);
            copyLocalPositionsToPhysx(internalDeformableAttachment->mData[1], localPositions, internalDeformableAttachment->mScale);

            break;
        }

        case eAttachmentTetXform:
        {
            if (!attachmentPrim.IsA(UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->TetXformAttachment)))
            {
                return kInvalidObjectId;
            }

            internalDeformableAttachment = ICE_NEW(InternalDeformableAttachment)(path, desc);
            if (!internalDeformableAttachment->isValid())
            {
                SAFE_DELETE_SINGLE(internalDeformableAttachment);
                return kInvalidObjectId;
            }

            pxr::VtArray<int> tetIndices;
            pxr::VtArray<GfVec3f> tetCoords;
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->tetIndicesSrc0).Get(&tetIndices);
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->tetCoordsSrc0).Get(&tetCoords);
            convertTetToPhysx(internalDeformableAttachment->mData[0], tetIndices, tetCoords);

            pxr::VtArray<GfVec3f> localPositions;
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->localPositionsSrc1).Get(&localPositions);
            copyLocalPositionsToPhysx(internalDeformableAttachment->mData[1], localPositions, internalDeformableAttachment->mScale);

            break;
        }

        case eAttachmentVtxVtx:
        {
            if (!attachmentPrim.IsA(UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxVtxAttachment)))
            {
                return kInvalidObjectId;
            }

            internalDeformableAttachment = ICE_NEW(InternalDeformableAttachment)(path, desc);
            if (!internalDeformableAttachment->isValid())
            {
                SAFE_DELETE_SINGLE(internalDeformableAttachment);
                return kInvalidObjectId;
            }

            // PhysX SDK does not natively support vtx to vtx attachment so we need to convert them to tri/tet id with barycentrics.
            pxr::VtArray<int> vtxIndices0;
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0).Get(&vtxIndices0);
            convertVtxToPhysx(internalDeformableAttachment->mData[0], vtxIndices0);

            pxr::VtArray<int> vtxIndices1;
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc1).Get(&vtxIndices1);
            convertVtxToPhysx(internalDeformableAttachment->mData[1], vtxIndices1);

            break;
        }

        case eAttachmentVtxTri:
        {
            if (!attachmentPrim.IsA(UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTriAttachment)))
            {
                return kInvalidObjectId;
            }

            internalDeformableAttachment = ICE_NEW(InternalDeformableAttachment)(path, desc);
            if (!internalDeformableAttachment->isValid())
            {
                SAFE_DELETE_SINGLE(internalDeformableAttachment);
                return kInvalidObjectId;
            }

            // PhysX SDK does not natively support vtx for deformable/deformable attachment so we need to convert the vtx deformable to tri/tet id with barycentrics.
            pxr::VtArray<int> vtxIndices;
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0).Get(&vtxIndices);
            convertVtxToPhysx(internalDeformableAttachment->mData[0], vtxIndices);

            pxr::VtArray<int> triIndices;
            pxr::VtArray<GfVec3f> triCoords;
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->triIndicesSrc1).Get(&triIndices);
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->triCoordsSrc1).Get(&triCoords);
            convertTriToPhysx(internalDeformableAttachment->mData[1], triIndices, triCoords);

            break;
        }

        case eAttachmentVtxTet:
        {
            if (!attachmentPrim.IsA(UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment)))
            {
                return kInvalidObjectId;
            }

            internalDeformableAttachment = ICE_NEW(InternalDeformableAttachment)(path, desc);
            if (!internalDeformableAttachment->isValid())
            {
                SAFE_DELETE_SINGLE(internalDeformableAttachment);
                return kInvalidObjectId;
            }

            // PhysX SDK does not natively support vtx for deformable/deformable attachment so we need to convert the vtx deformable to tri/tet id with barycentrics.
            pxr::VtArray<int> vtxIndices;
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0).Get(&vtxIndices);
            convertVtxToPhysx(internalDeformableAttachment->mData[0], vtxIndices);

            pxr::VtArray<int> tetIndices;
            pxr::VtArray<GfVec3f> tetCoords;
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->tetIndicesSrc1).Get(&tetIndices);
            attachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->tetCoordsSrc1).Get(&tetCoords);
            convertTetToPhysx(internalDeformableAttachment->mData[1], tetIndices, tetCoords);

            break;
        }

        default:
        {
            CARB_ASSERT(0);
            return kInvalidObjectId;
        }
    }

    InternalScene* internalScene = internalDeformableAttachment->mInternalScene;
    internalScene->addDeformableAttachment(*internalDeformableAttachment);

    if (desc.enabled)
    {
        internalDeformableAttachment->setCreateAttachmentEvent();
    }

    const ObjectId objId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(ePTDeformableAttachment, nullptr, internalDeformableAttachment, path);
    attachedStage.registerObjectId(path, ObjectType::eDeformableAttachment, objId);

    internalDeformableAttachment->mObjectId = objId;

    // check deformable attachment history
    {
        DeformableAttachmentHistoryMap& history = attachedStage.getDeformableAttachmentHistoryMap();

        DeformableAttachmentHistoryMap::const_iterator it = history.begin();
        DeformableAttachmentHistoryMap::const_iterator itEnd = history.end();
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

namespace
{
    void mapToPhysxFilterIndices(pxr::VtArray<uint32_t>& elemCounts, pxr::VtArray<uint32_t>& elemIndices,
        const InternalDeformableCollisionFilter::CollisionFilterData& filterData, const uint32_t numDeformableElems)
    {
        if (filterData.physxType == ePTDeformableSurface)
        {
            InternalSurfaceDeformableBody* surface = getInternalPtr<InternalSurfaceDeformableBody>(ePTDeformableSurface, (ObjectId)filterData.actor->userData);
            if (surface && numDeformableElems == (uint32_t)surface->mSimToPhysxTriMap.size())
            {
                for (uint32_t i = 0; i < elemIndices.size(); ++i)
                {
                    elemIndices[i] = surface->mSimToPhysxTriMap[elemIndices[i]];
                }
            }
        }
        else if (filterData.physxType == ePTDeformableVolume)
        {
            InternalVolumeDeformableBody* volume = omni::physx::getInternalPtr<InternalVolumeDeformableBody>(ePTDeformableVolume, (ObjectId)filterData.actor->userData);
            if (volume && numDeformableElems == (uint32_t)volume->mCollMeshSurfaceTriToTetMap.size())
            {
                const PxTetrahedronMesh& tetMesh = *volume->mDeformableVolumeMesh->getCollisionMesh();
                const uint32_t* surfaceTris = reinterpret_cast<uint32_t*>(volume->mCollMeshSurfaceTriangles.data());
                const uint32_t numSurfaceTris = uint32_t(volume->mCollMeshSurfaceTriangles.size());
                const std::vector<uint32_t>& surfaceTriToTetMap = volume->mCollMeshSurfaceTriToTetMap;
                const uint32_t numVerts = volume->mNumCollMeshVertices; //careful, don't get vertex count from tetMesh, since it has skinned vertices
                const uint32_t numTets = tetMesh.getNbTetrahedrons();
                const uint32_t* tets = reinterpret_cast<const uint32_t*>(tetMesh.getTetrahedrons());

                std::vector<uint32_t> vtxTetCounts(numVerts, 0);
                std::vector<uint32_t> vtxTetOffsets(numVerts);
                ResultBuffer<uint32_t> vtxTetIndices;
                bool res = tetfinder::tetMeshVtxToTet(vtxTetIndices.ptr, vtxTetIndices.size, vtxTetCounts.data(), vtxTetOffsets.data(),
                    tets, numTets, numVerts, ResultBuffer<uint32_t>::allocate);
                if (!res)
                {
                    return;
                }

                // create set of tets with surface
                std::unordered_set<uint32_t> tetWithSurface;
                {
                    for (uint32_t st = 0; st < numSurfaceTris; ++st)
                    {
                        uint32_t t = surfaceTriToTetMap[st];
                        tetWithSurface.insert(t);
                    }
                }

                // now finally, map surface triangle filter indices to tet indices
                pxr::VtArray<uint32_t> elemIndicesOut;
                {
                    uint32_t totalOffset = 0;
                    for (uint32_t g = 0; g < elemCounts.size(); ++g)
                    {
                        std::unordered_set<uint32_t> groupTets;
                        uint32_t count = elemCounts[g];
                        for (uint32_t i = 0; i < count; ++i)
                        {                             
                            uint32_t st = elemIndices[totalOffset + i];
                            groupTets.insert(surfaceTriToTetMap[st]);
                            const uint32_t* tri = surfaceTris + st * 3;
                            for (uint32_t v = 0; v < 3; ++v)
                            {
                                uint32_t vtxIndex = tri[v];
                                uint32_t offset = vtxTetOffsets[vtxIndex];
                                uint32_t vtxTetCount = vtxTetCounts[vtxIndex];

                                for (uint32_t t = 0; t < vtxTetCount; ++t)
                                {
                                    uint32_t tetIndex = vtxTetIndices.ptr[offset + t];
                                    if (tetWithSurface.count(tetIndex) == 0)
                                    {
                                        groupTets.insert(tetIndex);
                                    }
                                }
                            }
                        }

                        totalOffset += count;
                        for (uint32_t groupTet : groupTets)
                        {
                            elemIndicesOut.push_back(groupTet);
                        }
                        elemCounts[g] = uint32_t(groupTets.size());
                    }
                }
                elemIndices.swap(elemIndicesOut);
            }
        }
    }
}

bool checkGroupElemIndicesAndCounts(const pxr::VtArray<uint32_t>& groupElemCounts, const pxr::VtArray<uint32_t>& groupElemIndices,
    const uint32_t numCollMeshElems, const InternalDeformableCollisionFilter::CollisionFilterData& filterData)
{
    size_t totalCounts = 0;
    for (uint32_t count : groupElemCounts) {
        totalCounts += count;
    }

    if (totalCounts != groupElemIndices.size())
    {
        CARB_LOG_WARN("checkGroupElemIndicesAndCounts(): Sum of groupElemCounts is not equal to groupElemIndices size!");
        return false;
    }

    for (uint32_t index : groupElemIndices)
    {
        if (index >= numCollMeshElems)
        {
            CARB_LOG_WARN("checkGroupElemIndicesAndCounts(): Each group elem index should be smaller than the number of elements of collision mesh!");
            return false;
        }
    }

    return true;
}

uint32_t getNumCollMeshElems(const UsdPrim& collMeshPrim)
{
    uint32_t numCollMeshElems = 0;

    UsdGeomTetMesh collTetMesh(collMeshPrim);
    UsdGeomMesh collTriMesh(collMeshPrim);
    if (collTetMesh)
    {
        VtArray<GfVec3i> collMeshSurfaceTriangles;
        collTetMesh.GetSurfaceFaceVertexIndicesAttr().Get(&collMeshSurfaceTriangles);
        numCollMeshElems = (uint32_t)collMeshSurfaceTriangles.size();
    }
    else if (collTriMesh)
    {
        VtArray<int> faceVertexIndices;
        collTriMesh.GetFaceVertexIndicesAttr().Get(&faceVertexIndices);
        numCollMeshElems = (uint32_t)faceVertexIndices.size() / 3;
    }

    return numCollMeshElems;
}

ObjectId PhysXUsdPhysicsInterface::createDeformableCollisionFilter(usdparser::AttachedStage& attachedStage, const SdfPath& path, const PhysxDeformableCollisionFilterDesc& desc)
{
    const UsdPrim collisionFilterPrim = attachedStage.getStage()->GetPrimAtPath(path);
    if (!collisionFilterPrim.IsA(UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter)))
    {
        return kInvalidObjectId;
    }

    InternalDeformableCollisionFilter* internalDeformableCollisionFilter = ICE_NEW(InternalDeformableCollisionFilter)(path, desc);
    if (!internalDeformableCollisionFilter->isValid())
    {
        SAFE_DELETE_SINGLE(internalDeformableCollisionFilter);
        return kInvalidObjectId;
    }
    
    UsdPrim collMeshPrim0 = attachedStage.getStage()->GetPrimAtPath(desc.src0);
    UsdPrim collMeshPrim1 = attachedStage.getStage()->GetPrimAtPath(desc.src1);
    if (!collMeshPrim0 || !collMeshPrim1)
    {
        return kInvalidObjectId;
    }

    pxr::VtArray<uint32_t> groupElemCounts0;
    pxr::VtArray<uint32_t> groupElemIndices0;
    collisionFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts0).Get(&groupElemCounts0);
    collisionFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices0).Get(&groupElemIndices0);

    uint32_t numCollMeshElems0 = getNumCollMeshElems(collMeshPrim0);
    if (!checkGroupElemIndicesAndCounts(groupElemCounts0, groupElemIndices0, numCollMeshElems0, internalDeformableCollisionFilter->mData[0]))
    {
        return kInvalidObjectId;
    }

    mapToPhysxFilterIndices(groupElemCounts0, groupElemIndices0, internalDeformableCollisionFilter->mData[0], numCollMeshElems0);

    internalDeformableCollisionFilter->mData[0].groupElementCounts.assign(groupElemCounts0.begin(), groupElemCounts0.end());
    internalDeformableCollisionFilter->mData[0].groupElementIndices.assign(groupElemIndices0.begin(), groupElemIndices0.end());

    pxr::VtArray<uint32_t> groupElemCounts1;
    pxr::VtArray<uint32_t> groupElemIndices1;
    collisionFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts1).Get(&groupElemCounts1);
    collisionFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices1).Get(&groupElemIndices1);

    uint32_t numCollMeshElems1 = getNumCollMeshElems(collMeshPrim1);
    if (!checkGroupElemIndicesAndCounts(groupElemCounts1, groupElemIndices1, numCollMeshElems1, internalDeformableCollisionFilter->mData[1]))
    {
        return kInvalidObjectId;
    }

    mapToPhysxFilterIndices(groupElemCounts1, groupElemIndices1, internalDeformableCollisionFilter->mData[1], numCollMeshElems1);

    internalDeformableCollisionFilter->mData[1].groupElementCounts.assign(groupElemCounts1.begin(), groupElemCounts1.end());
    internalDeformableCollisionFilter->mData[1].groupElementIndices.assign(groupElemIndices1.begin(), groupElemIndices1.end());

    InternalScene* internalScene = internalDeformableCollisionFilter->mInternalScene;
    internalScene->addDeformableCollisionFilter(*internalDeformableCollisionFilter);

    if (desc.enabled)
    {
        internalDeformableCollisionFilter->setCreateCollisionFilterEvent();
    }

    const ObjectId objId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(ePTDeformableCollisionFilter, nullptr, internalDeformableCollisionFilter, path);
    attachedStage.registerObjectId(path, ObjectType::eDeformableCollisionFilter, objId);

    internalDeformableCollisionFilter->mObjectId = objId;

    // check deformable collision filter history
    {
        DeformableCollisionFilterHistoryMap& history = attachedStage.getDeformableCollisionFilterHistoryMap();

        DeformableCollisionFilterHistoryMap::const_iterator it = history.begin();
        DeformableCollisionFilterHistoryMap::const_iterator itEnd = history.end();
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

void PhysXUsdPhysicsInterface::processDeformableAttachmentShapeEvents()
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();

    std::vector<internal::InternalDeformableAttachment*> attachmentsToRemove;

    const PhysXScenesMap& physxScenes = omniPhysX.getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;

        std::vector<InternalDeformableAttachment*>& attachmentList = sc->getInternalScene()->mDeformableAttachments;
        for (size_t i = 0; i < attachmentList.size(); ++i)
        {
            if (attachmentList[i]->mDirtyEvent == eShapeRemoved)
                attachmentsToRemove.push_back(attachmentList[i]);
        }
    }

    for (size_t i = 0; i < attachmentsToRemove.size(); i++)
    {
        InternalScene* internalScene = attachmentsToRemove[i]->mInternalScene;

        if (internalScene)
        {
            internalScene->removeDeformableAttachments(attachmentsToRemove[i]->mShapeRemovedEvent.removeShapeId);
        }
    }
}

void PhysXUsdPhysicsInterface::processDeformableCollisionFilterShapeEvents()
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();

    std::vector<internal::InternalDeformableCollisionFilter*> collisionFiltersToRemove;

    const PhysXScenesMap& physxScenes = omniPhysX.getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PhysXScene* sc = ref.second;

        std::vector<InternalDeformableCollisionFilter*>& collisionFilterList = sc->getInternalScene()->mDeformableCollisionFilters;
        for (size_t i = 0; i < collisionFilterList.size(); ++i)
        {
            if (collisionFilterList[i]->mDirtyEvent == eShapeRemoved)
                collisionFiltersToRemove.push_back(collisionFilterList[i]);
        }
    }

    for (size_t i = 0; i < collisionFiltersToRemove.size(); i++)
    {
        InternalScene* internalScene = collisionFiltersToRemove[i]->mInternalScene;

        if (internalScene)
        {
            internalScene->removeDeformableCollisionFilters(collisionFiltersToRemove[i]->mShapeRemovedEvent.removeShapeId);
        }
    }
}

} // namespace physx
} // namespace omni
