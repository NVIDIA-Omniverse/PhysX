// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-24
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 *
 * @implements REQ-SIM-AUTOATTACH-001
 * @covers AC-4
 */

#include <omni/physics/parse/KnownTokens.h>

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
#include <attachment/PhysXTetFinder.h>
#include <attachment/PhysXPointFinder.h>
#include <attachment/PhysXTriFinder.h>

#include <common/utilities/MemoryMacros.h>

using namespace omni::physx::usdparser;
using namespace ::physx;
using namespace omni::physx::internal;
using namespace omni::physx;

extern ObjectId getObjectId(omni::physics::parse::ObjectKey key, PhysXType type);

// createDeformableAttachment/createDeformableCollisionFilter below are ObjectKey-native and
// unconditional: every read routes through IPhysicsSource/getArrayValue/KnownTokens already,
// and the object-database registration goes through AttachedStage::registerObjectId(ObjectKey, ...)
// (AttachedStage.h), so there is no remaining pxr dependency in this file.
namespace omni
{
namespace physx
{

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

void copyLocalPositionsToPhysx(InternalDeformableAttachment::AttachmentData& attachmentData,
                               const std::vector<carb::Float3>& localPositions,
                               ::physx::PxVec3 scale)
{
    copyBuffer(attachmentData.coords, localPositions.data(), (unsigned int)localPositions.size(), scale);
    attachmentData.targetType = (attachmentData.actor != nullptr) ?
                                PxDeformableAttachmentTargetType::eRIGID :
                                PxDeformableAttachmentTargetType::eWORLD;
}

void copyVtxToPhysx(InternalDeformableAttachment::AttachmentData& attachmentData, const std::vector<int32_t>& vtxIndices)
{
    attachmentData.indices.assign(vtxIndices.begin(), vtxIndices.end());
    attachmentData.targetType = PxDeformableAttachmentTargetType::eVERTEX;
}

void convertVtxToPhysx(InternalDeformableAttachment::AttachmentData& attachmentData, const std::vector<int32_t>& vtxIndices)
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

void convertTriToPhysx(InternalDeformableAttachment::AttachmentData& attachmentData, const std::vector<int32_t>& triIds, const std::vector<carb::Float3>& triBarycentrics)
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
            attachmentData.coords[i] = { triBarycentrics[i].x, triBarycentrics[i].y, triBarycentrics[i].z, w };
        }

        attachmentData.targetType = PxDeformableAttachmentTargetType::eTRIANGLE;
    }
}

void convertTetToPhysx(InternalDeformableAttachment::AttachmentData& attachmentData, const std::vector<int32_t>& tetIds, const std::vector<carb::Float3>& tetBarycentrics)
{
    if (attachmentData.physxType == ePTDeformableVolume)
    {
        attachmentData.indices.resize(tetIds.size());
        attachmentData.coords.resize(tetIds.size());

        for (PxU32 i = 0; i < tetIds.size(); i++)
        {
            attachmentData.indices[i] = tetIds[i];
            const float w = 1.0f - tetBarycentrics[i].x - tetBarycentrics[i].y - tetBarycentrics[i].z;
            attachmentData.coords[i] = { tetBarycentrics[i].x, tetBarycentrics[i].y, tetBarycentrics[i].z, w };
        }

        attachmentData.targetType = PxDeformableAttachmentTargetType::eTETRAHEDRON;
    }
}

bool readGeneratedAttachmentArray(const GeneratedDeformableAttachmentData& data,
                                  const omni::physics::parse::KnownTokens& tok,
                                  omni::physics::parse::TokenId attr,
                                  std::vector<int32_t>& out)
{
    if (attr == tok.omniphysicsVtxIndicesSrc0)
    {
        out = data.vtxIndicesSrc0;
        return true;
    }
    if (attr == tok.omniphysicsTetIndicesSrc1)
    {
        out = data.tetIndicesSrc1;
        return true;
    }
    return false;
}

bool readGeneratedAttachmentArray(const GeneratedDeformableAttachmentData& data,
                                  const omni::physics::parse::KnownTokens& tok,
                                  omni::physics::parse::TokenId attr,
                                  std::vector<carb::Float3>& out)
{
    if (attr == tok.omniphysicsTetCoordsSrc1)
    {
        out = data.tetCoordsSrc1;
        return true;
    }
    if (attr == tok.omniphysicsLocalPositionsSrc1)
    {
        out = data.localPositionsSrc1;
        return true;
    }
    return false;
}

bool readGeneratedCollisionFilterArray(const GeneratedDeformableCollisionFilterData& data,
                                       const omni::physics::parse::KnownTokens& tok,
                                       omni::physics::parse::TokenId attr,
                                       std::vector<uint32_t>& out)
{
    if (attr == tok.omniphysicsGroupElemCounts0)
    {
        out = data.groupElemCounts0;
        return true;
    }
    if (attr == tok.omniphysicsGroupElemIndices0)
    {
        out = data.groupElemIndices0;
        return true;
    }
    if (attr == tok.omniphysicsGroupElemCounts1)
    {
        out = data.groupElemCounts1;
        return true;
    }
    if (attr == tok.omniphysicsGroupElemIndices1)
    {
        out = data.groupElemIndices1;
        return true;
    }
    return false;
}

ObjectId PhysXUsdPhysicsInterface::createDeformableAttachment(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey attachmentKey, const PhysxDeformableAttachmentDesc& desc)
{
    InternalDeformableAttachment* internalDeformableAttachment = nullptr;
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    // A generated (in-memory) auto-attachment child has no prim behind its key.
    const GeneratedAutoAttachmentChild* generatedChild = attachedStage.findGeneratedAutoAttachmentChild(attachmentKey);
    if (!src || !(generatedChild || src->exists(attachmentKey)))
    {
        return kInvalidObjectId;
    }
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);

    const GeneratedDeformableAttachmentData* generatedData =
        attachedStage.getGeneratedDeformableAttachmentData(attachmentKey);
    PhysxDeformableAttachmentDesc effectiveDesc = desc;
    if (generatedData)
        effectiveDesc.enabled = generatedData->enabled;

    // Attachment array reads + IsA type checks route through the source rather
    // than reaching into USD via the prim. Auto-generated attachment data can be
    // newer than the scanned source, so prefer the AttachedStage cache when present.
    // TokenId end to end: readGeneratedAttachmentArray now dispatches on TokenId
    // identity against the auto-generated cache, and getArrayValue's ObjectKey +
    // TokenId + ReadTime sibling (PhysXTools.h) replaces the SdfPath/TfToken/
    // UsdTimeCode overload -- ReadTime::defaultTime() is exactly what toReadTime()
    // maps a default-constructed UsdTimeCode to.
    auto readArray = [&](omni::physics::parse::TokenId attr, auto& out)
    {
        if (generatedData && readGeneratedAttachmentArray(*generatedData, tok, attr, out))
            return;
        if (generatedChild)
            return; // nothing authored to fall back to
        getArrayValue(attachedStage, attachmentKey, attr, omni::physics::parse::ReadTime::defaultTime(), out);
    };
    // isA takes a TokenId directly, so this skips the TfToken round trip entirely.
    auto isAttachmentType = [&](omni::physics::parse::TokenId typeToken, ObjectType generatedType)
    {
        return generatedChild ? generatedChild->type == generatedType : src->isA(attachmentKey, typeToken);
    };

    switch (desc.type)
    {
        case eAttachmentVtxXform:
        {
            if (!isAttachmentType(tok.OmniPhysicsVtxXformAttachment, eAttachmentVtxXform))
            {
                return kInvalidObjectId;
            }

            internalDeformableAttachment = ICE_NEW(InternalDeformableAttachment)(attachmentKey, effectiveDesc);
            if (!internalDeformableAttachment->isValid())
            {
                SAFE_DELETE_SINGLE(internalDeformableAttachment);
                return kInvalidObjectId;
            }

            std::vector<int32_t> vtxIndices;
            readArray(tok.omniphysicsVtxIndicesSrc0, vtxIndices);
            copyVtxToPhysx(internalDeformableAttachment->mData[0], vtxIndices);

            std::vector<carb::Float3> localPositions;
            readArray(tok.omniphysicsLocalPositionsSrc1, localPositions);
            copyLocalPositionsToPhysx(internalDeformableAttachment->mData[1], localPositions, internalDeformableAttachment->mScale);

            break;
        }

        case eAttachmentTetXform:
        {
            if (!isAttachmentType(tok.OmniPhysicsTetXformAttachment, eAttachmentTetXform))
            {
                return kInvalidObjectId;
            }

            internalDeformableAttachment = ICE_NEW(InternalDeformableAttachment)(attachmentKey, effectiveDesc);
            if (!internalDeformableAttachment->isValid())
            {
                SAFE_DELETE_SINGLE(internalDeformableAttachment);
                return kInvalidObjectId;
            }

            std::vector<int32_t> tetIndices;
            std::vector<carb::Float3> tetCoords;
            readArray(tok.omniphysicsTetIndicesSrc0, tetIndices);
            readArray(tok.omniphysicsTetCoordsSrc0, tetCoords);
            convertTetToPhysx(internalDeformableAttachment->mData[0], tetIndices, tetCoords);

            std::vector<carb::Float3> localPositions;
            readArray(tok.omniphysicsLocalPositionsSrc1, localPositions);
            copyLocalPositionsToPhysx(internalDeformableAttachment->mData[1], localPositions, internalDeformableAttachment->mScale);

            break;
        }

        case eAttachmentVtxVtx:
        {
            if (!isAttachmentType(tok.OmniPhysicsVtxVtxAttachment, eAttachmentVtxVtx))
            {
                return kInvalidObjectId;
            }

            internalDeformableAttachment = ICE_NEW(InternalDeformableAttachment)(attachmentKey, effectiveDesc);
            if (!internalDeformableAttachment->isValid())
            {
                SAFE_DELETE_SINGLE(internalDeformableAttachment);
                return kInvalidObjectId;
            }

            // PhysX SDK does not natively support vtx to vtx attachment so we need to convert them to tri/tet id with barycentrics.
            std::vector<int32_t> vtxIndices0;
            readArray(tok.omniphysicsVtxIndicesSrc0, vtxIndices0);
            convertVtxToPhysx(internalDeformableAttachment->mData[0], vtxIndices0);

            std::vector<int32_t> vtxIndices1;
            readArray(tok.omniphysicsVtxIndicesSrc1, vtxIndices1);
            convertVtxToPhysx(internalDeformableAttachment->mData[1], vtxIndices1);

            break;
        }

        case eAttachmentVtxTri:
        {
            if (!isAttachmentType(tok.OmniPhysicsVtxTriAttachment, eAttachmentVtxTri))
            {
                return kInvalidObjectId;
            }

            internalDeformableAttachment = ICE_NEW(InternalDeformableAttachment)(attachmentKey, effectiveDesc);
            if (!internalDeformableAttachment->isValid())
            {
                SAFE_DELETE_SINGLE(internalDeformableAttachment);
                return kInvalidObjectId;
            }

            // PhysX SDK does not natively support vtx for deformable/deformable attachment so we need to convert the vtx deformable to tri/tet id with barycentrics.
            std::vector<int32_t> vtxIndices;
            readArray(tok.omniphysicsVtxIndicesSrc0, vtxIndices);
            convertVtxToPhysx(internalDeformableAttachment->mData[0], vtxIndices);

            std::vector<int32_t> triIndices;
            std::vector<carb::Float3> triCoords;
            readArray(tok.omniphysicsTriIndicesSrc1, triIndices);
            readArray(tok.omniphysicsTriCoordsSrc1, triCoords);
            convertTriToPhysx(internalDeformableAttachment->mData[1], triIndices, triCoords);

            break;
        }

        case eAttachmentVtxTet:
        {
            if (!isAttachmentType(tok.OmniPhysicsVtxTetAttachment, eAttachmentVtxTet))
            {
                return kInvalidObjectId;
            }

            internalDeformableAttachment = ICE_NEW(InternalDeformableAttachment)(attachmentKey, effectiveDesc);
            if (!internalDeformableAttachment->isValid())
            {
                SAFE_DELETE_SINGLE(internalDeformableAttachment);
                return kInvalidObjectId;
            }

            // PhysX SDK does not natively support vtx for deformable/deformable attachment so we need to convert the vtx deformable to tri/tet id with barycentrics.
            std::vector<int32_t> vtxIndices;
            readArray(tok.omniphysicsVtxIndicesSrc0, vtxIndices);
            convertVtxToPhysx(internalDeformableAttachment->mData[0], vtxIndices);

            std::vector<int32_t> tetIndices;
            std::vector<carb::Float3> tetCoords;
            readArray(tok.omniphysicsTetIndicesSrc1, tetIndices);
            readArray(tok.omniphysicsTetCoordsSrc1, tetCoords);
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

    if (effectiveDesc.enabled)
    {
        internalDeformableAttachment->setCreateAttachmentEvent();
    }

    const ObjectId objId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(ePTDeformableAttachment, nullptr, internalDeformableAttachment, attachmentKey);
    attachedStage.registerObjectId(attachmentKey, ObjectType::eDeformableAttachment, objId);

    internalDeformableAttachment->mObjectId = objId;

    // check deformable attachment history
    {
        DeformableAttachmentHistoryMap& history = attachedStage.getDeformableAttachmentHistoryMap();

        DeformableAttachmentHistoryMap::const_iterator it = history.begin();
        DeformableAttachmentHistoryMap::const_iterator itEnd = history.end();
        while (it != itEnd)
        {
            if (it->second == attachmentKey)
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
    void mapToPhysxFilterIndices(std::vector<uint32_t>& elemCounts, std::vector<uint32_t>& elemIndices,
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
                std::vector<uint32_t> elemIndicesOut;
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

bool checkGroupElemIndicesAndCounts(const std::vector<uint32_t>& groupElemCounts, const std::vector<uint32_t>& groupElemIndices,
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

uint32_t getNumCollMeshElems(const usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey collMeshKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return 0;

    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);

    // isTetMeshLike, not isA(UsdGeomTetMesh), and checked FIRST: ovstage reports a UsdGeomTetMesh
    // as plain "Mesh" (its populator has no TetMesh mapping), so the concrete-type check fell
    // through to the UsdGeomMesh branch below and returned 0 elements for every tet collision mesh
    // loaded from a non-USD source. See PhysXTools.h::isTetMeshLike.
    if (isTetMeshLike(attachedStage, collMeshKey))
    {
        std::vector<carb::Int3> collMeshSurfaceTriangles;
        getArrayValue(attachedStage, collMeshKey, tok.surfaceFaceVertexIndices,
            omni::physics::parse::ReadTime::defaultTime(), collMeshSurfaceTriangles);
        return static_cast<uint32_t>(collMeshSurfaceTriangles.size());
    }
    if (src->isA(collMeshKey, tok.meshType))
    {
        std::vector<int32_t> faceVertexIndices;
        getArrayValue(attachedStage, collMeshKey, tok.faceVertexIndices,
            omni::physics::parse::ReadTime::defaultTime(), faceVertexIndices);
        return static_cast<uint32_t>(faceVertexIndices.size() / 3);
    }

    return 0;
}

ObjectId PhysXUsdPhysicsInterface::createDeformableCollisionFilter(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey collisionFilterKey, const PhysxDeformableCollisionFilterDesc& desc)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
    {
        return kInvalidObjectId;
    }
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    // A generated (in-memory) auto-attachment child has no prim behind its key.
    const GeneratedAutoAttachmentChild* generatedChild = attachedStage.findGeneratedAutoAttachmentChild(collisionFilterKey);
    const bool isFilterType = generatedChild ? generatedChild->type == eDeformableCollisionFilter
                                             : src->isA(collisionFilterKey, tok.OmniPhysicsElementCollisionFilter);
    if (!isFilterType)
    {
        return kInvalidObjectId;
    }

    const GeneratedDeformableCollisionFilterData* generatedData =
        attachedStage.getGeneratedDeformableCollisionFilterData(collisionFilterKey);
    PhysxDeformableCollisionFilterDesc effectiveDesc = desc;
    if (generatedData)
        effectiveDesc.enabled = generatedData->enabled;

    InternalDeformableCollisionFilter* internalDeformableCollisionFilter = ICE_NEW(InternalDeformableCollisionFilter)(collisionFilterKey, effectiveDesc);
    if (!internalDeformableCollisionFilter->isValid())
    {
        SAFE_DELETE_SINGLE(internalDeformableCollisionFilter);
        return kInvalidObjectId;
    }

    if (!src->exists(effectiveDesc.src0) ||
        !src->exists(effectiveDesc.src1))
    {
        return kInvalidObjectId;
    }

    auto readFilterArray = [&](omni::physics::parse::TokenId attr, auto& out)
    {
        if (generatedData && readGeneratedCollisionFilterArray(*generatedData, tok, attr, out))
            return;
        if (generatedChild)
            return; // nothing authored to fall back to
        getArrayValue(attachedStage, collisionFilterKey, attr, omni::physics::parse::ReadTime::defaultTime(), out);
    };

    std::vector<uint32_t> groupElemCounts0;
    std::vector<uint32_t> groupElemIndices0;
    readFilterArray(tok.omniphysicsGroupElemCounts0, groupElemCounts0);
    readFilterArray(tok.omniphysicsGroupElemIndices0, groupElemIndices0);

    uint32_t numCollMeshElems0 = getNumCollMeshElems(attachedStage, effectiveDesc.src0);
    if (!checkGroupElemIndicesAndCounts(groupElemCounts0, groupElemIndices0, numCollMeshElems0, internalDeformableCollisionFilter->mData[0]))
    {
        return kInvalidObjectId;
    }

    mapToPhysxFilterIndices(groupElemCounts0, groupElemIndices0, internalDeformableCollisionFilter->mData[0], numCollMeshElems0);

    internalDeformableCollisionFilter->mData[0].groupElementCounts.assign(groupElemCounts0.begin(), groupElemCounts0.end());
    internalDeformableCollisionFilter->mData[0].groupElementIndices.assign(groupElemIndices0.begin(), groupElemIndices0.end());

    std::vector<uint32_t> groupElemCounts1;
    std::vector<uint32_t> groupElemIndices1;
    readFilterArray(tok.omniphysicsGroupElemCounts1, groupElemCounts1);
    readFilterArray(tok.omniphysicsGroupElemIndices1, groupElemIndices1);

    uint32_t numCollMeshElems1 = getNumCollMeshElems(attachedStage, effectiveDesc.src1);
    if (!checkGroupElemIndicesAndCounts(groupElemCounts1, groupElemIndices1, numCollMeshElems1, internalDeformableCollisionFilter->mData[1]))
    {
        return kInvalidObjectId;
    }

    mapToPhysxFilterIndices(groupElemCounts1, groupElemIndices1, internalDeformableCollisionFilter->mData[1], numCollMeshElems1);

    internalDeformableCollisionFilter->mData[1].groupElementCounts.assign(groupElemCounts1.begin(), groupElemCounts1.end());
    internalDeformableCollisionFilter->mData[1].groupElementIndices.assign(groupElemIndices1.begin(), groupElemIndices1.end());

    InternalScene* internalScene = internalDeformableCollisionFilter->mInternalScene;
    internalScene->addDeformableCollisionFilter(*internalDeformableCollisionFilter);

    if (effectiveDesc.enabled)
    {
        internalDeformableCollisionFilter->setCreateCollisionFilterEvent();
    }

    const ObjectId objId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(ePTDeformableCollisionFilter, nullptr, internalDeformableCollisionFilter, collisionFilterKey);
    attachedStage.registerObjectId(collisionFilterKey, ObjectType::eDeformableCollisionFilter, objId);

    internalDeformableCollisionFilter->mObjectId = objId;

    // check deformable collision filter history
    {
        DeformableCollisionFilterHistoryMap& history = attachedStage.getDeformableCollisionFilterHistoryMap();

        DeformableCollisionFilterHistoryMap::const_iterator it = history.begin();
        DeformableCollisionFilterHistoryMap::const_iterator itEnd = history.end();
        while (it != itEnd)
        {
            if (it->second == collisionFilterKey)
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
