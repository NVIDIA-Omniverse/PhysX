// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXAttachment.h"
#include "PhysXTetFinder.h"
#include "PhysXTriFinder.h"

#include <usdLoad/LoadUsd.h>

#include <internal/InternalTools.h>
#include <usdInterface/UsdInterface.h>

#include <PhysXScene.h>
#include <PhysXTools.h>
#include <ConeCylinderConvexMesh.h>
#include <CookingDataAsync.h>

using namespace pxr;
using namespace carb;
using namespace ::physx;
using namespace omni::physx::usdparser;
using namespace cookingdataasync;

#if !CARB_PLATFORM_WINDOWS
#define sprintf_s snprintf
#endif

static const TfToken autoDeformableAttachmentInputCrcToken{ "physxAutoDeformableAttachment:inputCrc" };
static const TfToken deformableBodyDataCrcToken("physxDeformableBody:deformableBodyDataCrc");

namespace carb
{

struct Int2_hash
{
    std::size_t operator () (carb::Int2 const &val) const
    {
        return std::hash<int32_t>()(val.x) ^ std::hash<int32_t>()(val.y);
    }
};

bool operator==(const carb::Int2& lhs, const carb::Int2& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

} // namespace carb


namespace omni
{
namespace physx
{
namespace
{

struct PhysxAutoAttachmentDesc
{
    PhysxAutoAttachmentDesc()
    {
        enableDeformableVertexAttachments = false;
        deformableVertexOverlapOffset = 0.0f;
        enableRigidSurfaceAttachments = false;
        rigidSurfaceSamplingDistance = 0.0f;
        enableCollisionFiltering = false;
        collisionFilteringOffset = 0.0f;
        enableDeformableFilteringPairs = false;
    }

    // Auto attachment params
    bool enableDeformableVertexAttachments;
    float deformableVertexOverlapOffset;
    bool enableRigidSurfaceAttachments;
    float rigidSurfaceSamplingDistance;
    bool enableCollisionFiltering;
    float collisionFilteringOffset;
    bool enableDeformableFilteringPairs;
};

struct AttachmentActorType
{
    enum Enum
    {
        eINVALID = 0,
        eSURFACE_DEFORMABLE = 1 << 0,
        eVOLUME_DEFORMABLE = 1 << 1,
        eXFORMABLE = 1 << 2,

        eDEFORMABLE = eSURFACE_DEFORMABLE | eVOLUME_DEFORMABLE
    };
};

struct DeformableMeshInfo
{
    DeformableMeshInfo() : type(AttachmentActorType::eINVALID) { }

    AttachmentActorType::Enum type;
    UsdGeomPointBased simMesh;
    std::vector<uint32_t> simIndices;
    std::vector<carb::Float3> simPositions;

    UsdGeomPointBased collMesh;
    std::vector<uint32_t> collIndices;
    std::vector<uint32_t> collSurfaceTriIndices;
    std::vector<uint32_t> collSurfaceTriToTetMap;
    std::vector<carb::Float3> collPositions;

    omni::physx::usdparser::MeshKey deformableBodyDataCrc;
};

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

struct FilterGroup
{
    uint32_t* indices;
    uint32_t indicesSize;
};

struct FilterGroup_hash
{
    std::size_t operator () (FilterGroup const& val) const
    {
        std::size_t hash = 0;
        for (uint32_t i = 0; i < val.indicesSize; ++i)
        {
            hash = hash ^ std::hash<uint32_t>()(val.indices[i]);
        }
        return hash;
    }
};

bool operator==(const FilterGroup& lhs, const FilterGroup& rhs)
{
    return (lhs.indicesSize == rhs.indicesSize) &&
        (std::memcmp(lhs.indices, rhs.indices, sizeof(uint32_t) * lhs.indicesSize) == 0);
}

void parsePhysxAutoAttachment(PhysxAutoAttachmentDesc& autoAttachmentDesc, const UsdPrim autoAttachmentPrim)
{
    autoAttachmentPrim.GetAttribute(PhysxAdditionAttrTokens->enableDeformableVertexAttachments).Get(&autoAttachmentDesc.enableDeformableVertexAttachments);
    autoAttachmentPrim.GetAttribute(PhysxAdditionAttrTokens->deformableVertexOverlapOffset).Get(&autoAttachmentDesc.deformableVertexOverlapOffset);
    autoAttachmentPrim.GetAttribute(PhysxAdditionAttrTokens->enableRigidSurfaceAttachments).Get(&autoAttachmentDesc.enableRigidSurfaceAttachments);
    autoAttachmentPrim.GetAttribute(PhysxAdditionAttrTokens->rigidSurfaceSamplingDistance).Get(&autoAttachmentDesc.rigidSurfaceSamplingDistance);
    autoAttachmentPrim.GetAttribute(PhysxAdditionAttrTokens->enableCollisionFiltering).Get(&autoAttachmentDesc.enableCollisionFiltering);
    autoAttachmentPrim.GetAttribute(PhysxAdditionAttrTokens->collisionFilteringOffset).Get(&autoAttachmentDesc.collisionFilteringOffset);
    autoAttachmentPrim.GetAttribute(PhysxAdditionAttrTokens->enableDeformableFilteringPairs).Get(&autoAttachmentDesc.enableDeformableFilteringPairs);
}

void parseSingleTargetPathPair(SdfPath(&targetPaths)[2], const UsdPrim prim, const TfToken relName0, const TfToken relName1)
{
    UsdRelationship rel0 = prim.GetRelationship(relName0);
    UsdRelationship rel1 = prim.GetRelationship(relName1);
    if (rel0 && rel1)
    {
        SdfPathVector pathVector;
        SdfPath targetPath;

        rel0.GetTargets(&pathVector);
        targetPath = pathVector.size() == 1 ? pathVector[0] : SdfPath();
        targetPaths[0] = targetPath;

        rel1.GetTargets(&pathVector);
        targetPath = pathVector.size() == 1 ? pathVector[0] : SdfPath();
        targetPaths[1] = targetPath;
    }
}

void parseAttachablePaths(SdfPath(&attachablePaths)[2], const UsdPrim autoAttachmentPrim)
{
    parseSingleTargetPathPair(attachablePaths, autoAttachmentPrim,
        PhysxAdditionAttrTokens->attachable0,
        PhysxAdditionAttrTokens->attachable1);
}

/*
 * mapSurfaceTrisToTets builds a surface triangle to tetrahedron map given a list of surface triangles and the
 * tet mesh. It's more work than extracting the surface itself from scratch, but the order of the given surface triangle
 * list needs to be preserved.
 */
bool mapSurfaceTrisToTets(uint32_t* surfaceTriToTetMap, const uint32_t* surfaceTriIndices, const uint32_t surfaceTriCount,
                          const uint32_t* tetIndices, const uint32_t tetCount)
{
    // We could just build the data structure to build the map given the surface triangles from scratch
    // from the tet mesh, but instead we re-use PxTetrahedronMeshExt::extractTetMeshSurface, which
    // gives the surface triangles and the map from scratch.
    PxArray<PxU32> triToTetMap;
    PxArray<PxU32> triIndices;

    PxTetrahedronMeshExt::extractTetMeshSurface(tetIndices, tetCount, false, triIndices, &triToTetMap, false);

    // Convert PxTetrahedronMeshExt::extractTetMeshSurface triToTetMap output to actual "indices per tet".
    for (uint32_t i = 0; i < triToTetMap.size(); ++i)
    {
        triToTetMap[i] /= 4;
    }

    if (triToTetMap.size() != surfaceTriCount || triToTetMap.size()*3 != triIndices.size())
        return false;

    // Now sort both triangle list according to a triangle key (consisting of its ordered vertex indices)
    // Instead of sorting the triangle lists directly, we sort corresponding index lists.
    const uint32_t triCount = surfaceTriCount;
    const uint32_t* toTriangles = surfaceTriIndices;
    const uint32_t* fromTriangles = triIndices.begin();

    std::vector<uint32_t> toTriOrder(triCount);
    std::vector<uint32_t> fromTriOrder(triCount);

    for (uint32_t i = 0; i < triCount; ++i)
        toTriOrder[i] = i;

    for (uint32_t i = 0; i < triCount; ++i)
        fromTriOrder[i] = i;

    const auto orderVerts = [](carb::Uint3 t) noexcept -> carb::Uint3
    {
        if (t.x > t.y) std::swap(t.x, t.y);
        if (t.y > t.z) std::swap(t.y, t.z);
        if (t.x > t.y) std::swap(t.x, t.y);
        return t;
    };

    const auto cmpTriangles = [orderVerts](const carb::Uint3& a, const carb::Uint3& b) noexcept -> bool
    {
        carb::Uint3 ao = orderVerts(a);
        carb::Uint3 bo = orderVerts(b);
        if (ao.x != bo.x) return ao.x < bo.x;
        if (ao.y != bo.y) return ao.y < bo.y;
        return ao.z < bo.z; // false when keys are equal
    };

    const auto loadTriangle = [](const uint32_t* indices, uint32_t triOffset) -> carb::Uint3
    {
        return { indices[triOffset * 3 + 0], indices[triOffset * 3 + 1], indices[triOffset * 3 + 2] };
    };

    auto fromCmp = [fromTriangles, loadTriangle, cmpTriangles](uint32_t i, uint32_t j) noexcept -> bool
    {
        return cmpTriangles(loadTriangle(fromTriangles, i), loadTriangle(fromTriangles, j));
    };

    auto toCmp = [toTriangles, loadTriangle, cmpTriangles](uint32_t i, uint32_t j) noexcept -> bool
    {
        return cmpTriangles(loadTriangle(toTriangles, i), loadTriangle(toTriangles, j));
    };

    std::sort(fromTriOrder.begin(), fromTriOrder.end(), fromCmp);
    std::sort(toTriOrder.begin(), toTriOrder.end(), toCmp);

    // Now create the output map by remapping triToTetMap using the two orderings.
    for (uint32_t i = 0; i < triCount; ++i)
    {
        uint32_t fromIndex = fromTriOrder[i];
        uint32_t toIndex = toTriOrder[i];

        // Check that both orders are pointing to same triangle
        const carb::Uint3 fromTri = orderVerts(loadTriangle(fromTriangles, fromIndex));
        const carb::Uint3 toTri = orderVerts(loadTriangle(toTriangles, toIndex));
        if (fromTri.x != toTri.x || fromTri.y != toTri.y || fromTri.z != toTri.z)
            return false;

        // Map to output
        uint32_t tetIndex = triToTetMap[fromIndex];
        surfaceTriToTetMap[toIndex] = tetIndex;
    }
    return true;
}

bool parseTetMeshSurface(std::vector<uint32_t>& surfaceTriVtxIndices, std::vector<uint32_t>& surfaceTriToTetMap, const UsdGeomTetMesh tetMesh)
{
    // need to query USD surfaceFaceVertexIndices, for consistency.
    VtArray<GfVec3i> surfaceFaceVertexIndices;
    tetMesh.GetSurfaceFaceVertexIndicesAttr().Get(&surfaceFaceVertexIndices);

    if (surfaceFaceVertexIndices.size() == 0)
    {
        CARB_LOG_WARN("UsdGeomTetMesh is missing surface face vertex indices, %s.", tetMesh.GetPath().GetText());
        return false;
    }

    surfaceTriVtxIndices.resize(surfaceFaceVertexIndices.size() * 3);
    std::memcpy(surfaceTriVtxIndices.data(), surfaceFaceVertexIndices.data(), surfaceTriVtxIndices.size()*sizeof(uint32_t));

    // generate surface to tet map
    VtArray<GfVec4i> tetVertexIndices;
    tetMesh.GetTetVertexIndicesAttr().Get(&tetVertexIndices);

    surfaceTriToTetMap.resize(surfaceFaceVertexIndices.size());
    bool success = mapSurfaceTrisToTets(surfaceTriToTetMap.data(),
                                        (uint32_t*)surfaceFaceVertexIndices.data(),
                                        uint32_t(surfaceFaceVertexIndices.size()),
                                        (uint32_t*)tetVertexIndices.data(),
                                        uint32_t(tetVertexIndices.size()));

    if (!success)
    {
        CARB_LOG_WARN("UsdGeomTetMesh, failed to map surface faces to tets, %s.", tetMesh.GetPath().GetText());
        return false;
    }

    return true;
}

void getColliders(std::vector<UsdPhysicsCollisionAPI>& colliders, const UsdPrim rootPrim)
{
    if (!rootPrim)
    {
        return;
    }

    TfType deformableBodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);

    const UsdPrimRange range(rootPrim, UsdTraverseInstanceProxies());
    for (UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const UsdPrim prim = *iter;
        if (!prim)
            continue;

        if (prim == rootPrim)
        {
            UsdPhysicsCollisionAPI collisionAPI(prim);
            if (collisionAPI)
            {
                colliders.push_back(collisionAPI);
                iter.PruneChildren();
            }
        }
        else
        {
            UsdGeomXformable xformable(prim);
            if (xformable && xformable.GetResetXformStack())
            {
                iter.PruneChildren();
            }
            else if (prim.HasAPI(deformableBodyType))
            {
                iter.PruneChildren();
            }
            else if (prim.HasAPI<UsdPhysicsCollisionAPI>())
            {
                colliders.push_back(UsdPhysicsCollisionAPI(prim));
                iter.PruneChildren();
            }
        }
    }
}

bool getVtxXformAttachment(UsdPrim& attachmentPrim, const UsdPrim autoAttachmentPrim,
    UsdGeomPointBased simMesh, UsdGeomXformable xformable)
{
    TfType vtxXformType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
    auto children = autoAttachmentPrim.GetChildren();
    for (const auto& c : children)
    {
        if (c.IsA(vtxXformType))
        {
            attachmentPrim = c;
            SdfPath srcPaths[2];
            parseSingleTargetPathPair(srcPaths, c, OmniPhysicsDeformableAttrTokens->src0, OmniPhysicsDeformableAttrTokens->src1);
            return srcPaths[0] == simMesh.GetPath() && srcPaths[1] == xformable.GetPath();
        }
    }
    return false;
}

bool getVtxTetAttachment(UsdPrim& attachmentPrim, const UsdPrim autoAttachmentPrim,
    UsdGeomPointBased vtxSimMesh, UsdGeomPointBased tetSimMesh)
{
    TfType vtxTetType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
    auto children = autoAttachmentPrim.GetChildren();
    for (const auto& c : children)
    {
        if (c.IsA(vtxTetType))
        {
            attachmentPrim = c;
            SdfPath srcPaths[2];
            parseSingleTargetPathPair(srcPaths, c, OmniPhysicsDeformableAttrTokens->src0, OmniPhysicsDeformableAttrTokens->src1);
            if (srcPaths[0] == vtxSimMesh.GetPath() && srcPaths[1] == tetSimMesh.GetPath())
            {
                return true;
            }
        }
    }
    return false;
}

bool getElementCollisionFilters(std::vector<UsdPrim>& filterPrims, const UsdPrim autoAttachmentPrim,
    std::vector<UsdPhysicsCollisionAPI>& rigidColliders, UsdPhysicsCollisionAPI deformableCollider)
{
    TfType filterType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);
    auto children = autoAttachmentPrim.GetChildren();
    for (const auto& c : children)
    {
        if (c.IsA(filterType))
        {
            filterPrims.push_back(c);
        }
    }

    if (filterPrims.size() != rigidColliders.size())
    {
        return false;
    }
    for (size_t f = 0; f < filterPrims.size(); ++f)
    {
        UsdPrim filterPrim = filterPrims[f];
        SdfPath srcPaths[2];
        parseSingleTargetPathPair(srcPaths, filterPrim, OmniPhysicsDeformableAttrTokens->src0, OmniPhysicsDeformableAttrTokens->src1);
        if (srcPaths[0] != deformableCollider.GetPath() || srcPaths[1] != rigidColliders[f].GetPath())
        {
            return false;
        }
    }
    return true;
}

bool getElementCollisionFilter(UsdPrim& filterPrim, const UsdPrim autoAttachmentPrim,
    UsdPhysicsCollisionAPI deformableCollider0, UsdPhysicsCollisionAPI deformableCollider1)
{
    TfType filterType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);
    auto children = autoAttachmentPrim.GetChildren();
    for (const auto& c : children)
    {
        if (c.IsA(filterType))
        {
            filterPrim = c;
            break;
        }
    }

    if (filterPrim)
    {
        SdfPath srcPaths[2];
        parseSingleTargetPathPair(srcPaths, filterPrim, OmniPhysicsDeformableAttrTokens->src0, OmniPhysicsDeformableAttrTokens->src1);
        if (srcPaths[0] != deformableCollider0.GetPath() || srcPaths[1] != deformableCollider1.GetPath())
        {
            return false;
        }
    }
    return true;
}

pxr::GfVec3f convertBary(carb::Float4& bary)
{
    return GfVec3f(bary.x, bary.y, bary.z);
}

float getAverageDim(UsdPrim prim, UsdGeomBBoxCache& cacheBBox)
{
    const GfRange3d bounds = cacheBBox.ComputeWorldBound(prim).ComputeAlignedRange();
    const GfVec3d size = bounds.GetSize();
    PxU32 num_edges = 0;
    num_edges += size[0] == 0.0 ? 0 : 1;
    num_edges += size[1] == 0.0 ? 0 : 1;
    num_edges += size[2] == 0.0 ? 0 : 1;
    return (float)(size[0] + size[1] + size[2]) / std::max(num_edges, 1u);
}

float getAverageDim(const PxGeometry& geom, const PxTransform& geomPose)
{
    float avg_dim = PX_MAX_REAL;
    if (geom.getType() != PxGeometryType::ePLANE)
    {
        PxBounds3 bounds;
        PxGeometryQuery::computeGeomBounds(bounds, geom, geomPose, 0.0f, 1.01f);

        PxVec3 dimensions = bounds.getDimensions();
        PxU32 num_edges = 0;
        num_edges += dimensions.x == 0.0 ? 0 : 1;
        num_edges += dimensions.y == 0.0 ? 0 : 1;
        num_edges += dimensions.z == 0.0 ? 0 : 1;
        avg_dim = (dimensions.x + dimensions.y + dimensions.z) / std::max(num_edges, 1u);
    }
    return avg_dim;
}

carb::Float4 computeDistancePointBarycentric(const carb::Float3* positions, const uint32_t* indices, const uint32_t srcTetIndex,
    const carb::Float3& srcPoint, const carb::Float3& srcPointDistanceDir)
{
    PxVec3 tp[4];
    for (uint32_t i = 0; i < 4; ++i)
    {
        uint32_t v = indices[srcTetIndex * 4 + i];
        tp[i] = omni::physx::toPhysX(positions[v]);
    }
    PxVec4 physxBary;
    PxComputeBarycentric(tp[0], tp[1], tp[2], tp[3], omni::physx::toPhysX(srcPoint) + omni::physx::toPhysX(srcPointDistanceDir), physxBary);
    return *reinterpret_cast<carb::Float4*>(&physxBary);
}

/**
* Sort indices in filter groups, and removes duplicates to make the groups deterministically comparable.
*/
void sortFilterGroups(VtArray<uint32_t>& dstCounts, VtArray<uint32_t>& dstIndices,
    VtArray<uint32_t>& srcCounts, VtArray<uint32_t>& srcIndices)
{
    dstCounts.clear();
    dstIndices.clear();
    dstCounts.reserve(srcCounts.size());
    dstIndices.reserve(srcIndices.size());

    uint32_t srcIndexOffset = 0;
    uint32_t dstIndexOffset = 0;
    for (uint32_t g = 0; g < uint32_t(srcCounts.size()); ++g)
    {
        const uint32_t srcCount = srcCounts[g];
        for (uint32_t i = 0; i < srcCount; ++i)
        {
            dstIndices.push_back(srcIndices[srcIndexOffset + i]);
        }

        uint32_t* dstGroupBegin = dstIndices.data() + dstIndexOffset;
        uint32_t* dstGroupEnd = dstGroupBegin + srcCount;
        std::sort(dstGroupBegin, dstGroupEnd);
        dstGroupEnd = std::unique(dstGroupBegin, dstGroupEnd);
        uint32_t dstCount(dstGroupEnd - dstGroupBegin);

        srcIndexOffset += srcCount;
        dstIndexOffset += dstCount;
        dstIndices.resize(dstIndexOffset);
        dstCounts.push_back(dstCount);
    }
}

/**
* Merge groups in A. Groups in B are adjusted accordingly.
* Assumes group indices are pre-sorted
*/
void compressFilterGroupsSingleSided(
    VtArray<uint32_t>& dstCountsA, VtArray<uint32_t>& dstIndicesA,
    VtArray<uint32_t>& dstCountsB, VtArray<uint32_t>& dstIndicesB,
    const VtArray<uint32_t>& srcCountsA, const VtArray<uint32_t>& srcIndicesA,
    const VtArray<uint32_t>& srcCountsB, const VtArray<uint32_t>& srcIndicesB)
{
    if (srcCountsA.size() == 0 || srcCountsA.size() != srcCountsB.size())
    {
        dstCountsA.assign(srcCountsA.begin(), srcCountsA.end());
        dstIndicesA.assign(srcIndicesA.begin(), srcIndicesA.end());
        dstCountsB.assign(srcCountsB.begin(), srcCountsB.end());
        dstIndicesB.assign(srcIndicesB.begin(), srcIndicesB.end());
        return;
    }

    std::unordered_set<uint32_t> mergedGroups;

        
    std::unordered_map<FilterGroup, uint32_t, FilterGroup_hash> groupToOffset;

    //we assume


    uint32_t srcIndexOffsetA = 0;
    uint32_t srcIndexOffsetB = 0;

    std::unordered_set<uint32_t> srcGroupA;
    std::unordered_set<uint32_t> srcGroupB;
    for (uint32_t i = 0; i < srcCountsA[0]; ++i)
    {
        srcGroupA.insert(srcIndicesA[srcIndexOffsetA + i]);
    }
    for (uint32_t i = 0; i < srcCountsB[0]; ++i)
    {
        srcGroupB.insert(srcIndicesB[srcIndexOffsetB + i]);
    }
    srcIndexOffsetA += srcCountsA[0];
    srcIndexOffsetB += srcCountsB[0];

    std::unordered_set<uint32_t> srcGroupCurrentA;
    std::unordered_set<uint32_t> srcGroupAggregateB;
    srcGroupCurrentA.swap(srcGroupA);
    srcGroupAggregateB.swap(srcGroupB);

    for (size_t g = 1; g < srcCountsA.size(); ++g)
    {
        srcGroupA.clear();
        srcGroupB.clear();
        for (uint32_t i = 0; i < srcCountsA[g]; ++i)
        {
            srcGroupA.insert(srcIndicesA[srcIndexOffsetA + i]);
        }
        for (uint32_t i = 0; i < srcCountsB[g]; ++i)
        {
            srcGroupB.insert(srcIndicesB[srcIndexOffsetB + i]);
        }
        srcIndexOffsetA += srcCountsA[g];
        srcIndexOffsetB += srcCountsB[g];

        const bool isMismatch = (srcGroupCurrentA != srcGroupA);
        if (isMismatch)
        {
            //flush previous group 
            dstCountsA.push_back(uint32_t(srcGroupCurrentA.size()));
            for (uint32_t index : srcGroupCurrentA)
            {
                dstIndicesA.push_back(index);
            }
            dstCountsB.push_back(uint32_t(srcGroupAggregateB.size()));
            for (uint32_t index : srcGroupAggregateB)
            {
                dstIndicesB.push_back(index);
            }
            srcGroupCurrentA.swap(srcGroupA);
            srcGroupAggregateB.clear();
        }
        srcGroupAggregateB.insert(srcGroupB.begin(), srcGroupB.end());
    }

    {
        //flush last group
        dstCountsA.push_back(uint32_t(srcGroupCurrentA.size()));
        for (uint32_t index : srcGroupCurrentA)
        {
            dstIndicesA.push_back(index);
        }
        dstCountsB.push_back(uint32_t(srcGroupAggregateB.size()));
        for (uint32_t index : srcGroupAggregateB)
        {
            dstIndicesB.push_back(index);
        }
    }
}

void compressFilterGroups(
    VtArray<uint32_t>& filterGroupCountsA, VtArray<uint32_t>& filterGroupIndicesA,
    VtArray<uint32_t>& filterGroupCountsB, VtArray<uint32_t>& filterGroupIndicesB)
{
    if (filterGroupCountsA.size() == 0 || filterGroupCountsA.size() != filterGroupCountsB.size())
    {
        filterGroupCountsA.clear();
        filterGroupIndicesA.clear();
        filterGroupCountsB.clear();
        filterGroupIndicesB.clear();
        return;
    }

    VtArray<uint32_t> tmpFilterGroupCounts[2];
    VtArray<uint32_t> tmpFilterGroupIndices[2];
    sortFilterGroups(tmpFilterGroupCounts[0], tmpFilterGroupIndices[0],
        filterGroupCountsA, filterGroupIndicesA);

    sortFilterGroups(tmpFilterGroupCounts[1], tmpFilterGroupIndices[1],
        filterGroupCountsB, filterGroupIndicesB);

    filterGroupCountsA.clear();
    filterGroupIndicesA.clear();
    filterGroupCountsB.clear();
    filterGroupIndicesB.clear();

    compressFilterGroupsSingleSided(
        filterGroupCountsA, filterGroupIndicesA,
        filterGroupCountsB, filterGroupIndicesB,
        tmpFilterGroupCounts[0], tmpFilterGroupIndices[0],
        tmpFilterGroupCounts[1], tmpFilterGroupIndices[1]);

    tmpFilterGroupCounts[0].clear();
    tmpFilterGroupIndices[0].clear();
    tmpFilterGroupCounts[1].clear();
    tmpFilterGroupIndices[1].clear();

    compressFilterGroupsSingleSided(
        tmpFilterGroupCounts[1], tmpFilterGroupIndices[1],
        tmpFilterGroupCounts[0], tmpFilterGroupIndices[0],
        filterGroupCountsB, filterGroupIndicesB,
        filterGroupCountsA, filterGroupIndicesA);

    filterGroupCountsA.swap(tmpFilterGroupCounts[0]);
    filterGroupCountsB.swap(tmpFilterGroupCounts[1]);
    filterGroupIndicesA.swap(tmpFilterGroupIndices[0]);
    filterGroupIndicesB.swap(tmpFilterGroupIndices[1]);
}

void convertTetGroupsToSurfaceTriGroups(VtArray<uint32_t>& triGroupCounts, VtArray<uint32_t>& triGroupIndices,
        const std::vector<uint32_t>& tetVtxIndices, const std::vector<uint32_t>& surfaceTriToTetMap,
        const VtArray<uint32_t>& tetGroupCounts, const VtArray<uint32_t>& tetGroupIndices)
{
    uint32_t numTets = uint32_t(tetVtxIndices.size() / 4);
    uint32_t numSurfaceTris = uint32_t(surfaceTriToTetMap.size());

    ResultBuffer<uint32_t> tetToSurfaceTriIndices;
    std::vector<uint32_t> tetToSurfaceTriCounts;
    std::vector<uint32_t> tetToSurfaceTriOffsets;

    {
        tetToSurfaceTriCounts.resize(numTets);
        tetToSurfaceTriOffsets.resize(numTets);

        bool noError = omni::tetfinder::tetMeshTetToSurfaceTri(tetToSurfaceTriIndices.ptr, tetToSurfaceTriIndices.size,
            tetToSurfaceTriCounts.data(), tetToSurfaceTriOffsets.data(), numTets,
            surfaceTriToTetMap.data(), numSurfaceTris, ResultBuffer<uint32_t>::allocate);

        if (!noError)
        {
            return;
        }
    }

    uint32_t tetFilterOffset = 0;
    for (uint32_t g = 0; g < tetGroupCounts.size(); ++g)
    {
        uint32_t groupTetCount = tetGroupCounts[g];
        uint32_t dstTriCount = 0;
        for (uint32_t i = 0; i < groupTetCount; ++i)
        {
            uint32_t tetIndex = tetGroupIndices[tetFilterOffset + i];
            uint32_t triCount = tetToSurfaceTriCounts[tetIndex];
            uint32_t triOffset = tetToSurfaceTriOffsets[tetIndex];
            for (uint32_t t = 0; t < triCount; ++t)
            {
                uint32_t triIndex = tetToSurfaceTriIndices.ptr[triOffset + t];
                triGroupIndices.push_back(triIndex);
            }
            dstTriCount += triCount;
        }
        triGroupCounts.push_back(dstTriCount);
        tetFilterOffset += groupTetCount;
    }
}

void convertVtxGroupsToTriGroups(VtArray<uint32_t>& triGroupCounts, VtArray<uint32_t>& triGroupIndices,
    const std::vector<carb::Float3>& points, const std::vector<uint32_t>& triVtxIndices,
    const VtArray<uint32_t>& vtxGroupCounts, const VtArray<uint32_t>& vtxGroupIndices)
{
    std::vector<uint32_t> vtxTriCounts(vtxGroupIndices.size());
    ResultBuffer<uint32_t> vtxTriIndices;

    uint64_t triFinderColl = omni::trifinder::createTriFinder(
        points.data(), uint32_t(points.size()),
        triVtxIndices.data(), uint32_t(triVtxIndices.size()));

    omni::trifinder::getAdjacency(vtxTriCounts.data(), vtxTriIndices.ptr, vtxTriIndices.size, triFinderColl,
        vtxGroupIndices.data(), uint32_t(vtxGroupIndices.size()), ResultBuffer<uint32_t>::allocate);

    omni::trifinder::releaseTriFinder(triFinderColl);

    uint32_t vtxGroupIndicesOffset = 0;
    uint32_t vtxTriIndicesOffset = 0;
    for (uint32_t g = 0; g < vtxGroupCounts.size(); ++g)
    {
        uint32_t vtxGroupCount = vtxGroupCounts[g];
        uint32_t triGroupCount = 0;
        for (uint32_t i = 0; i < vtxGroupCount; ++i)
        {
            uint32_t adjIndex = vtxGroupIndicesOffset + i;
            uint32_t vtxIndex = vtxGroupIndices[adjIndex];
            uint32_t triCount = vtxTriCounts[adjIndex];
            for (uint32_t t = 0; t < triCount; ++t)
            {
                uint32_t triIndex = vtxTriIndices.ptr[vtxTriIndicesOffset + t];
                triGroupIndices.push_back(triIndex);
            }
            vtxTriIndicesOffset += triCount;
            triGroupCount += triCount;
        }
        vtxGroupIndicesOffset += vtxGroupCount;
        triGroupCounts.push_back(triGroupCount);
    }
}

void addPairsToFilterGroups(
    VtArray<uint32_t>& groupCountsA, VtArray<uint32_t>& groupIndicesA,
    VtArray<uint32_t>& groupCountsB, VtArray<uint32_t>& groupIndicesB,
    const carb::Int2* pairsAB, const uint32_t pairsABsize,
    const uint32_t* mapIndicesA, const uint32_t* mapIndicesB)
{
    for (uint32_t p = 0; p < pairsABsize; ++p)
    {
        const carb::Int2& pair = pairsAB[p];
        groupCountsA.push_back(1);
        groupCountsB.push_back(1);
        const uint32_t a = mapIndicesA ? mapIndicesA[pair.x] : pair.x;
        const uint32_t b = mapIndicesB ? mapIndicesB[pair.y] : pair.y;
        groupIndicesA.push_back(a);
        groupIndicesB.push_back(b);
    }
}

void addPairsToFilterGroups(
    VtArray<uint32_t>& groupCountsA, VtArray<uint32_t>& groupIndicesA,
    VtArray<uint32_t>& groupCountsB, VtArray<uint32_t>& groupIndicesB,
    const int32_t* indicesA, const uint32_t indicesAsize,
    const int32_t* indicesB, const uint32_t indicesBsize,
    const uint32_t* mapIndicesA, const uint32_t* mapIndicesB)
{
    //TODO OMPE-22590, for now add bidirectional, but in theory it should work to just add the indices with smaller count. 
    //if (indicesAsize <= indicesBsize)
    {
        if (indicesAsize > 0)
        {
            groupCountsA.push_back(indicesAsize);
            groupCountsB.push_back(0);
            for (uint32_t i = 0; i < indicesAsize; ++i)
            {
                const uint32_t index = indicesA[i];
                const uint32_t a = mapIndicesA ? mapIndicesA[index] : index;
                groupIndicesA.push_back(a);
            }
        }
    }
    //else
    {
        if (indicesBsize > 0)
        {
            groupCountsB.push_back(indicesBsize);
            groupCountsA.push_back(0);
            for (uint32_t i = 0; i < indicesBsize; ++i)
            {
                const uint32_t index = indicesB[i];
                const uint32_t b = mapIndicesB ? mapIndicesB[index] : index;
                groupIndicesB.push_back(b);
            }
        }
    }
}

void computeVtxTetAttachments(
    VtArray<int32_t>& attachmentVtxIndices,
    VtArray<int32_t>& attachmentTetIndices,
    VtArray<GfVec3f>& attachmentTetCoords,
    const std::vector<carb::Float3>& srcPoints,
    const std::vector<uint32_t>& srcPointIndices,
    const std::vector<carb::Float3>& dstTetMeshPoints,
    const std::vector<uint32_t>& dstTetMeshIndices,
    const uint64_t tetFinder,
    const float vertexOverlapOffset)
{
    std::vector<int32_t> srcPointTetIds(srcPoints.size());
    std::vector<carb::Float4> srcPointTetBary(srcPoints.size());
    std::vector<carb::Float3> srcPointDistanceDirs(srcPoints.size());
    bool isSuccess = omni::tetfinder::pointsToTetMeshLocalClosest(
        srcPointTetIds.data(), srcPointTetBary.data(), srcPointDistanceDirs.data(),
        tetFinder, srcPoints.data(), uint32_t(srcPoints.size()));

    if (isSuccess)
    {
        for (PxU32 i = 0; i < srcPoints.size(); i++)
        {
            PxVec3 srcPointDistanceDir = omni::physx::toPhysX(srcPointDistanceDirs[i]);
            float separationSq = srcPointDistanceDir.magnitudeSquared();

            if (separationSq <= vertexOverlapOffset * vertexOverlapOffset)
            {
                attachmentVtxIndices.push_back(srcPointIndices[i]);
                attachmentTetIndices.push_back(srcPointTetIds[i]);
                carb::Float4 bary;
                if (separationSq == 0.0f)
                {
                    bary = srcPointTetBary[i];
                }
                else
                {
                    bary = computeDistancePointBarycentric(dstTetMeshPoints.data(), dstTetMeshIndices.data(),
                        srcPointTetIds[i], srcPoints[i], srcPointDistanceDirs[i]);
                }
                attachmentTetCoords.push_back(convertBary(bary));
            }
        }
    }
}

void checkNonUniformScale(const GfVec3d& scale, const SdfPath& primPath)
{
    const double tolerance = 1e-4;
    if (abs(scale[0] - scale[1]) > tolerance || abs(scale[0] - scale[2]) > tolerance || abs(scale[2] - scale[1]) > tolerance)
    {
        CARB_LOG_WARN("Non-uniform scale may result in a non matching attachment shape representation: %s", primPath.GetText());
    }
}

struct MaskShapes
{
    std::vector<PxGeometryHolder> geometries;
    std::vector<PxTransform> transforms;
};

/*
 * Cull input points to union of attachment shapes, returning the dense list of intersecting points and their original indices.
 * Returns a copy of the original points, in case there are no shapes.
 */
void cullPointsToMaskShapes(std::vector<carb::Float3>& culledPoints, std::vector<uint32_t>& culledPointIndices,
    const MaskShapes& maskShapes, const float shapeOffset, const carb::Float3* points, const uint32_t pointsSize)
{
    if (!points || pointsSize == 0)
    {
        return;
    }

    if (maskShapes.geometries.empty())
    {
        culledPoints.assign(points, points + pointsSize);
        culledPointIndices.resize(pointsSize);
        for (uint32_t i = 0; i < pointsSize; ++i)
        {
            culledPointIndices[i] = i;
        }
        return;
    }

    for (uint32_t i = 0; i < pointsSize; ++i)
    {
        const PxVec3 pos = toPhysX(points[i]);
        for (uint32_t s = 0; s < maskShapes.geometries.size(); ++s)
        {
            PxReal distance = PxGeometryQuery::pointDistance(pos, maskShapes.geometries[s].any(), maskShapes.transforms[s]);
            if (distance <= shapeOffset)
            {
                culledPointIndices.push_back(i);
                break;
            }
        }
    }

    culledPoints.resize(culledPointIndices.size());
    for (uint32_t i = 0; i < culledPoints.size(); ++i)
    {
        culledPoints[i] = points[culledPointIndices[i]];
    }
}

/*
 * Cull tets to the union of the attachment shapes, returning a new tetfinder for the intersecting tets, and a list mapping back to the original tet ids.
 */
uint64_t cullTetsToMaskShapes(std::vector<uint32_t>& culledTetIds, const MaskShapes& maskShapes, const float shapeOffset, const uint64_t tetFinder)
{
    if (maskShapes.geometries.empty())
    {
        return 0;
    }

    std::unordered_set<int32_t> uniqueTetIds;
    for (uint32_t s = 0; s < maskShapes.geometries.size(); ++s)
    {
        ResultBuffer<int32_t> tetIds;
        tetfinder::overlapTetMeshGeom(tetIds.ptr, tetIds.size, tetFinder, maskShapes.geometries[s].any(), maskShapes.transforms[s], shapeOffset, ResultBuffer<>::allocate);
        uniqueTetIds.insert(tetIds.ptr, tetIds.ptr + tetIds.size);
    }

    culledTetIds.reserve(uniqueTetIds.size());
    for (int32_t tetId : uniqueTetIds)
    {
        culledTetIds.push_back(tetId);
    }

    uint32_t indicesSize = 0;
    const uint32_t* indices = tetfinder::getIndices(indicesSize, tetFinder);
    std::vector<uint32_t> culledTetVertIndices(culledTetIds.size() * 4);
    for (uint32_t i = 0; i < culledTetIds.size(); ++i)
    {
        culledTetVertIndices[i * 4 + 0] = indices[culledTetIds[i] * 4 + 0];
        culledTetVertIndices[i * 4 + 1] = indices[culledTetIds[i] * 4 + 1];
        culledTetVertIndices[i * 4 + 2] = indices[culledTetIds[i] * 4 + 2];
        culledTetVertIndices[i * 4 + 3] = indices[culledTetIds[i] * 4 + 3];
    }

    uint32_t pointsSize, pointsByteStride;
    const carb::Float3* points = tetfinder::getPoints(pointsSize, pointsByteStride, tetFinder);
    return tetfinder::createTetFinder(points, pointsSize, pointsByteStride, culledTetVertIndices.data(), uint32_t(culledTetVertIndices.size()));
}

/*
 * Cull tris to the union of the attachment shapes, returning a new trifinder for the intersecting tris, and a list mapping back to the original tri ids.
 */
uint64_t cullTrisToMaskShapes(std::vector<uint32_t>& culledTriIds, const MaskShapes& maskShapes, const float shapeOffset, const uint64_t triFinder)
{
    if (maskShapes.geometries.empty())
    {
        return 0;
    }

    std::unordered_set<int32_t> uniqueTriIds;
    for (uint32_t s = 0; s < maskShapes.geometries.size(); ++s)
    {
        ResultBuffer<int32_t> triIds;
        trifinder::overlapTriMeshGeom(triIds.ptr, triIds.size, triFinder, maskShapes.geometries[s].any(), maskShapes.transforms[s], shapeOffset, ResultBuffer<>::allocate);
        uniqueTriIds.insert(triIds.ptr, triIds.ptr + triIds.size);
    }

    culledTriIds.reserve(uniqueTriIds.size());
    for (int32_t triId : uniqueTriIds)
    {
        culledTriIds.push_back(triId);
    }

    uint32_t indicesSize = 0;
    const uint32_t* indices = trifinder::getIndices(indicesSize, triFinder);
    std::vector<uint32_t> culledTriVertIndices(culledTriIds.size() * 3);
    for (uint32_t i = 0; i < culledTriIds.size(); ++i)
    {
        culledTriVertIndices[i * 3 + 0] = indices[culledTriIds[i] * 3 + 0];
        culledTriVertIndices[i * 3 + 1] = indices[culledTriIds[i] * 3 + 1];
        culledTriVertIndices[i * 3 + 2] = indices[culledTriIds[i] * 3 + 2];
    }

    uint32_t pointsSize, pointsByteStride;
    const carb::Float3* points = trifinder::getPoints(pointsSize, pointsByteStride, triFinder);
    return trifinder::createTriFinder(points, pointsSize, culledTriVertIndices.data(), uint32_t(culledTriVertIndices.size()));
}

struct UserDataInfo
{
    VtArray<int32_t>& attachmentVtxIndicesDeformable;
    VtArray<GfVec3f>& attachmentVtxPointsXformable;
    VtArray<uint32_t>& filterTriIndicesDeformable;
    const DeformableMeshInfo& deformableMeshInfo;
    const GfMatrix4d& worldToRigid;
    const PhysxAutoAttachmentDesc& desc;
    const MaskShapes& maskShapes;
};

} // namespace

void updateDeformableRigidColliderAttachments(const PxGeometry& geom, const PxTransform& geomPose, void* userData)
{
    const UserDataInfo* info = (const UserDataInfo*)userData;

    VtArray<int32_t>& attachmentVtxIndicesDeformable = info->attachmentVtxIndicesDeformable;
    VtArray<GfVec3f>& attachmentVtxPointsXformable = info->attachmentVtxPointsXformable;
    VtArray<uint32_t>& filterGroupIndices = info->filterTriIndicesDeformable;

    const DeformableMeshInfo& deformableMeshInfo = info->deformableMeshInfo;
    const GfMatrix4d& worldToRigid = info->worldToRigid;
    const PhysxAutoAttachmentDesc& desc = info->desc;

    UsdGeomBBoxCache cacheBBox(UsdTimeCode::Default(), { UsdGeomTokens->default_, UsdGeomTokens->proxy, UsdGeomTokens->guide });

    // Use the minimum average dimension
    float avg_dim0 = getAverageDim(deformableMeshInfo.collMesh.GetPrim(), cacheBBox);
    float avg_dim1 = getAverageDim(geom, geomPose);

    float avg_dim = PxMin(avg_dim0, avg_dim1);
    float default_rad = avg_dim * 0.05f;

    // Apply default heuristics
    float rigidSurfaceSamplingDistance = desc.rigidSurfaceSamplingDistance;
    if (!isfinite(rigidSurfaceSamplingDistance))
        rigidSurfaceSamplingDistance = default_rad;

    float collisionFilteringOffset = desc.collisionFilteringOffset;
    if (!isfinite(collisionFilteringOffset))
        collisionFilteringOffset = default_rad * 2;

    PxSphereGeometry defaultVertexAttachmentSphere(desc.deformableVertexOverlapOffset);
    PxSphereGeometry defaultFilteringSphere(collisionFilteringOffset);

    bool isTriangleMesh = geom.getType() == PxGeometryType::eTRIANGLEMESH;

    PxTriangleMeshPoissonSampler* triMeshSampler = nullptr;
    std::vector<PxU32> triangleIndices;
    std::vector<PxVec3> triangleVertices;

    if (isTriangleMesh)
    {
        const PxTriangleMeshGeometry& triMesh = *(static_cast<const PxTriangleMeshGeometry *>(&geom));

        if (triMesh.triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES)
        {
            const PxU16* ptr16 = reinterpret_cast<const PxU16*>(triMesh.triangleMesh->getTriangles());
            const PxU32 nbTriIndices = triMesh.triangleMesh->getNbTriangles() * 3;

            triangleIndices.reserve(nbTriIndices);
            for (uint32_t i = 0; i < nbTriIndices; ++i)
            {
                triangleIndices.push_back(ptr16[i]);
            }
        }
        const PxU32* trianglePtr = triMesh.triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES ? &triangleIndices[0] : reinterpret_cast<const PxU32*>(triMesh.triangleMesh->getTriangles());

        triangleVertices.reserve(triMesh.triangleMesh->getNbVertices());
        for (uint32_t i = 0; i < triMesh.triangleMesh->getNbVertices(); ++i)
        {
            PxVec3 vertex = triMesh.triangleMesh->getVertices()[i];
            vertex = vertex.multiply(triMesh.scale.scale);
            vertex = geomPose.transform(vertex);

            triangleVertices.push_back(vertex);
        }

        triMeshSampler = PxCreateTriangleMeshSampler(trianglePtr, triMesh.triangleMesh->getNbTriangles(), &triangleVertices[0], triMesh.triangleMesh->getNbVertices(), 30);
    }

#if 0
    // Surface sampling on rigid body
    if (enableRigidSurfaceAttachments && deformableMeshInfo.type == AttachmentActorType::eVOLUME_DEFORMABLE && rigidSurfaceSamplingDistance > 0.0f)
    {
        PxArray<PxVec3> samples;

        if (geom.getType() == PxGeometryType::ePLANE)
        {
            PxBounds3 worldBounds = PxBounds3(toPhysX(sb_bounds.GetMin()), toPhysX(sb_bounds.GetMax()));
            worldBounds.fattenSafe(default_rad);
            PxQuat quat = PxQuat(PxIdentity);
            PxSamplingExt::poissonSample(geom, geomPos, worldBounds, rigidSurfaceSamplingDistance, samples, 0.0f, &worldBounds, &quat);
        }
        else
        {
            PxBounds3 bounds;
            PxGeometryQuery::computeGeomBounds(bounds, geom, geomPos, 0.0f, 1.01f);

            PxSamplingExt::poissonSample(geom, geomPos, bounds, rigidSurfaceSamplingDistance, samples);
        }

        std::vector<carb::Float3> culledSamples;
        std::vector<uint32_t> culledSampleIndices;
        cullPointsToMaskShapes(culledSamples, culledSampleIndices, info->maskShapes, 0.0f,
            reinterpret_cast<carb::Float3*>(samples.begin()), samples.size());

        std::vector<int32_t> tetIds(culledSamples.size());
        std::vector<carb::Float4> tetBary(culledSamples.size());
        omni::tetfinder::pointsToTetMeshLocal(&tetIds[0], &tetBary[0], tetFinderCollisionPositions, culledSamples.data(), uint32_t(culledSamples.size()));
        std::vector<carb::Float3> localSamples(culledSamples.size());
        omni::tetfinder::tetMeshLocalToPoints(&localSamples[0], tetFinderRestPositions, &tetIds[0], &tetBary[0], uint32_t(localSamples.size()));

        for (uint32_t i = 0; i < culledSamples.size(); ++i)
        {
            if (tetIds[i] >= 0)
            {
                const carb::Float3& localSample = localSamples[i];
                attachmentPointsDeformable.push_back(GfVec3f(localSample.x, localSample.y, localSample.z));

                const PxVec3 actorPos = transform.transformInv(toPhysX(culledSamples[i])).multiply(invScale);
                attachmentPointsRigidBody.push_back(GfVec3f(actorPos.x, actorPos.y, actorPos.z));
            }
        }
    }
#endif

    // Vertex overlaps
    if (desc.enableDeformableVertexAttachments)
    {
        std::vector<carb::Float3> culledVertices;
        std::vector<uint32_t> culledVertexIndices;
        cullPointsToMaskShapes(culledVertices, culledVertexIndices, info->maskShapes, 0.0f,
            deformableMeshInfo.simPositions.data(), uint32_t(deformableMeshInfo.simPositions.size()));

        for (PxU32 i = 0; i < culledVertices.size(); i++)
        {
            const uint32_t vertexIndex = culledVertexIndices[i];
            const PxVec3 vertexPos = toPhysX(culledVertices[i]);

            bool attachmentHit = PxGeometryQuery::overlap(geom, geomPose, defaultVertexAttachmentSphere, PxTransform(vertexPos));

            if (!attachmentHit && isTriangleMesh && triMeshSampler)
                attachmentHit = triMeshSampler->isPointInTriangleMesh(vertexPos);

            if (!attachmentHit)
                continue;

            attachmentVtxIndicesDeformable.push_back(int32_t(vertexIndex));

            GfVec3f rigidPos = worldToRigid.Transform(GfVec3f(vertexPos.x, vertexPos.y, vertexPos.z));
            attachmentVtxPointsXformable.push_back(rigidPos);
        }
    }

    // Filtering
    if (desc.enableCollisionFiltering)
    {
        std::vector<carb::Float3> culledVertices;
        std::vector<uint32_t> culledVertexIndices;
        cullPointsToMaskShapes(culledVertices, culledVertexIndices, info->maskShapes, collisionFilteringOffset,
            deformableMeshInfo.collPositions.data(), uint32_t(deformableMeshInfo.collPositions.size()));

        VtArray<uint32_t> vtxGroupIndices;
        VtArray<uint32_t> vtxGroupCounts;
        for (PxU32 i = 0; i < culledVertices.size(); i++)
        {
            const PxVec3 particlePos = toPhysX(culledVertices[i]);

            bool filterHit = PxGeometryQuery::overlap(geom, geomPose, defaultFilteringSphere, PxTransform(particlePos));

            if (!filterHit && isTriangleMesh && triMeshSampler)
                filterHit = triMeshSampler->isPointInTriangleMesh(particlePos);

            if (!filterHit)
                continue;

            vtxGroupIndices.push_back(culledVertexIndices[i]);
        }
        vtxGroupCounts.push_back(uint32_t(vtxGroupIndices.size()));

        const std::vector<carb::Float3>* points;
        const std::vector<uint32_t>* triVtxIndices;
        if (deformableMeshInfo.type == AttachmentActorType::eVOLUME_DEFORMABLE)
        {
            points = &deformableMeshInfo.collPositions;
            triVtxIndices = &deformableMeshInfo.collSurfaceTriIndices;
        }
        else
        {
            CARB_ASSERT(deformableMeshInfo.type == AttachmentActorType::eSURFACE_DEFORMABLE);
            points = &deformableMeshInfo.collPositions;
            triVtxIndices = &deformableMeshInfo.collIndices;
        }

        VtArray<uint32_t> filterGroupCounts;
        convertVtxGroupsToTriGroups(filterGroupCounts, filterGroupIndices, *points, *triVtxIndices,
            vtxGroupCounts, vtxGroupIndices);

        CARB_ASSERT(filterGroupCounts.size() == 1);
        VtArray<uint32_t> filterGroupCountsRigid = { 0 };
        VtArray<uint32_t> filterGroupIndicesRigid;
        compressFilterGroups(filterGroupCounts, filterGroupIndices, filterGroupCountsRigid, filterGroupIndicesRigid);
    }

    PX_DELETE(triMeshSampler);
}

bool parseMaskShape(PxGeometryHolder& geometryHolder, PxTransform& transform, UsdPrim usdPrim)
{
    UsdTimeCode timeCode = UsdTimeCode::Default();

    UsdGeomSphere geomSphere(usdPrim);
    UsdGeomCapsule geomCapsule(usdPrim);
    UsdGeomCube geomCube(usdPrim);

    GfMatrix4d shapeMat = UsdGeomXformable(usdPrim).ComputeLocalToWorldTransform(timeCode);
    const GfTransform tr(shapeMat);
    const GfVec3d shapePos = tr.GetTranslation();
    const GfQuatd shapeRot = tr.GetRotation().GetQuat();
    const GfVec3d shapeScale = tr.GetScale();

    transform = PxTransform(toPhysX(shapePos), toPhysX(shapeRot));

    if (geomSphere)
    {
        float radius = 1.0f;

        {
            // as we dont support scale in physics and scale can be non uniform
            // we pick the largest scale value as the sphere radius base
            checkNonUniformScale(shapeScale, usdPrim.GetPrimPath());
            radius = fmaxf(fmaxf(fabsf(float(shapeScale[1])), fabsf(float(shapeScale[0]))), fabsf(float(shapeScale[2])));
        }

        // Get shape parameters
        {
            double radiusAttr;
            geomSphere.GetRadiusAttr().Get(&radiusAttr);
            radius *= (float)radiusAttr;
        }

        geometryHolder = PxSphereGeometry(fabsf(radius));
        return true;
    }

    if (geomCapsule)
    {
        float radius = 1.0f;
        float halfHeight = 1.0f;
        TfToken axis = UsdPhysicsTokens.Get()->x;

        // Get shape parameters
        {
            double radiusAttr;
            geomCapsule.GetRadiusAttr().Get(&radiusAttr);
            double heightAttr;
            geomCapsule.GetHeightAttr().Get(&heightAttr);
            radius = (float)radiusAttr;
            halfHeight = (float)heightAttr * 0.5f;

            if (geomCapsule.GetAxisAttr())
            {
                geomCapsule.GetAxisAttr().Get(&axis);
            }
        }

        {
            // scale the radius and height based on the given axis token
            checkNonUniformScale(shapeScale, usdPrim.GetPrimPath());
            if (axis == UsdPhysicsTokens.Get()->x)
            {
                halfHeight *= float(shapeScale[0]);
                radius *= fmaxf(fabsf(float(shapeScale[1])), fabsf(float(shapeScale[2])));
            }
            else if (axis == UsdPhysicsTokens.Get()->y)
            {
                halfHeight *= float(shapeScale[1]);
                radius *= fmaxf(fabsf(float(shapeScale[0])), fabsf(float(shapeScale[2])));
            }
            else
            {
                halfHeight *= float(shapeScale[2]);
                radius *= fmaxf(fabsf(float(shapeScale[1])), fabsf(float(shapeScale[0])));
            }
        }

        geometryHolder = PxCapsuleGeometry(radius, halfHeight);

        const float hRt2 = sqrt(2.0f) / 2.0f;
        PxQuat fixupQ(PxIdentity);

        if (axis == UsdPhysicsTokens.Get()->y)
        {
            fixupQ = PxQuat(hRt2, -hRt2, 0.0f, 0.0f);
        }
        else if (axis == UsdPhysicsTokens.Get()->z)
        {
            fixupQ = PxQuat(hRt2, 0.0f, -hRt2, 0.0f);
        }

        transform.q = transform.q * fixupQ;
        return true;
    }

    if (geomCube)
    {
        GfVec3f halfExtents;

        {
            // scale is taken, its a part of the cube size, as the physics does not support scale
            halfExtents = GfVec3f(shapeScale);
        }

        // Get shape parameters
        {
            double sizeAttr;
            geomCube.GetSizeAttr().Get(&sizeAttr);
            sizeAttr = abs(sizeAttr) * 0.5f; // convert cube edge length to half extend
            halfExtents *= (float)sizeAttr;
        }

        geometryHolder = PxBoxGeometry(toPhysX(halfExtents));
        return true;
    }

    return false;
}

void processRigidShapeGeometry(const UsdPrim& rigidBodyPrim, const usdparser::PhysxShapeDesc* desc, getGeometryInfoCallback callbackFn, void* userData)
{
    UsdPrim usdPrim = rigidBodyPrim;
    PxTransform transform = toPhysX(desc->localPos, desc->localRot);
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    CookingDataAsync* cookingDataAsync = physxSetup.getCookingDataAsync();
    CARB_ASSERT(cookingDataAsync);

    switch (desc->type)
    {
    case eSphereShape:
    {
        SpherePhysxShapeDesc* sphereDesc = (SpherePhysxShapeDesc*)desc;
        PxSphereGeometry sphereGeom(sphereDesc->radius);
        callbackFn(sphereGeom, transform, userData);
    }
    break;
    case eBoxShape:
    {
        BoxPhysxShapeDesc* boxDesc = (BoxPhysxShapeDesc*)desc;
        PxBoxGeometry boxGeom((const PxVec3&)boxDesc->halfExtents);
        callbackFn(boxGeom, transform, userData);
    }
    break;
    case eCapsuleShape:
    {
        CapsulePhysxShapeDesc* capsuleDesc = (CapsulePhysxShapeDesc*)desc;

        const float hRt2 = sqrt(2.0f) / 2.0f;
        PxQuat fixupQ(PxIdentity);
        if (capsuleDesc->axis == eY)
        {
            fixupQ = PxQuat(hRt2, -hRt2, 0.0f, 0.0f);
        }
        else if (capsuleDesc->axis == eZ)
        {
            fixupQ = PxQuat(hRt2, 0.0f, -hRt2, 0.0f);
        }
        transform.q = transform.q * fixupQ;

        PxCapsuleGeometry capsuleGeom(capsuleDesc->radius, capsuleDesc->halfHeight);
        callbackFn(capsuleGeom, transform, userData);
    }
    break;
    case ePlaneShape:
    {
        PlanePhysxShapeDesc* planeDesc = (PlanePhysxShapeDesc*)desc;

        const float hRt2 = sqrt(2.0f) / 2.0f;
        PxQuat fixupQ(PxIdentity);
        if (planeDesc->axis == eY)
        {
            fixupQ = PxQuat(hRt2, hRt2, 0.0f, 0.0f);
        }
        else if (planeDesc->axis == eZ)
        {
            fixupQ = PxQuat(hRt2, 0.0f, hRt2, 0.0f);
        }
        transform.q = transform.q * fixupQ;

        PxPlaneGeometry planeGeom;
        callbackFn(planeGeom, transform, userData);
    }
    break;
    case eCylinderShape:
    {
        CylinderPhysxShapeDesc* cylinderDesc = (CylinderPhysxShapeDesc*)desc;
        PxConvexMesh* convexMesh = physxSetup.getCylinderConvexMesh(cylinderDesc->axis);
        if (convexMesh)
        {
            const PxVec3 scale = getConeOrCylinderScale(cylinderDesc->halfHeight, cylinderDesc->radius, cylinderDesc->axis);
            PxConvexMeshGeometry convexMeshGeom(convexMesh, scale);
            callbackFn(convexMeshGeom, transform, userData);
        }
    }
    break;
    case eConeShape:
    {
        ConePhysxShapeDesc* coneDesc = (ConePhysxShapeDesc*)desc;
        PxConvexMesh* convexMesh = physxSetup.getConeConvexMesh(coneDesc->axis);
        if (convexMesh)
        {
            const PxVec3 scale = getConeOrCylinderScale(coneDesc->halfHeight, coneDesc->radius, coneDesc->axis);
            PxConvexMeshGeometry convexMeshGeom(convexMesh, scale);
            callbackFn(convexMeshGeom, transform, userData);
        }
    }
    break;
    case eConvexMeshShape:
    {
        ConvexMeshPhysxShapeDesc* convexDesc = (ConvexMeshPhysxShapeDesc*)desc;
        PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(*convexDesc, usdPrim, false);
        if (convexMesh)
        {
            PxConvexMeshGeometry convexMeshGeom(convexMesh, toPhysX(convexDesc->meshScale));
            callbackFn(convexMeshGeom, transform, userData);
        }
    }
    break;
    case eConvexMeshDecompositionShape:
    {
        ConvexMeshDecompositionPhysxShapeDesc* convexDecompositionDesc = (ConvexMeshDecompositionPhysxShapeDesc*)desc;
        std::vector<PxConvexMesh*> convexMeshes = cookingDataAsync->getConvexMeshDecomposition(*convexDecompositionDesc, usdPrim, false);
        if (!convexMeshes.empty())
        {
            const PxVec3 scale(fabsf(convexDecompositionDesc->meshScale.x), fabsf(convexDecompositionDesc->meshScale.y), fabsf(convexDecompositionDesc->meshScale.z));
            for (size_t i = 0; i < convexMeshes.size(); i++)
            {
                PxConvexMeshGeometry convexMeshGeom(convexMeshes[i], scale);
                callbackFn(convexMeshGeom, transform, userData);
            }
        }
    }
    break;
    case eTriangleMeshShape:
    {
        TriangleMeshPhysxShapeDesc* meshDesc = (TriangleMeshPhysxShapeDesc*)desc;
        PxTriangleMesh* triMesh = cookingDataAsync->getTriangleMesh(*meshDesc, usdPrim, false);
        if (triMesh)
        {
            PxTriangleMeshGeometry triangleMeshGeom(triMesh, toPhysX(meshDesc->meshScale));
            callbackFn(triangleMeshGeom, transform, userData);
        }
    }
    break;
    }
}

void updateDeformableVolumeSurfaceAttachments(UsdPrim vtxTetAttachmentPrim, UsdPrim filterPrim,
    const DeformableMeshInfo& volumeMeshInfo, const uint32_t volumeSlot,
    const DeformableMeshInfo& surfaceMeshInfo, const uint32_t surfaceSlot,
    const PhysxAutoAttachmentDesc& desc, const MaskShapes& maskShapes)
{
    UsdGeomBBoxCache cacheBBox(UsdTimeCode::Default(), { UsdGeomTokens->default_, UsdGeomTokens->proxy, UsdGeomTokens->guide });
    // Use the minimum average dimension
    float avg_dimV = getAverageDim(volumeMeshInfo.collMesh.GetPrim(), cacheBBox);
    float avg_dimS = getAverageDim(surfaceMeshInfo.collMesh.GetPrim(), cacheBBox);
    float avg_dim = PxMin(avg_dimV, avg_dimS);
    float default_rad = avg_dim * 0.05f;

    // Apply default heuristics
    float collisionFilteringOffset = desc.collisionFilteringOffset;
    if (!isfinite(collisionFilteringOffset))
        collisionFilteringOffset = default_rad * 2;

    VtArray<int32_t> attachmentVtxIndices;
    VtArray<GfVec3f> attachmentTetCoords;
    VtArray<int32_t> attachmentTetIndices;
    VtArray<uint32_t> filterGroupCounts[2];
    VtArray<uint32_t> filterGroupIndices[2];

    uint64_t tetFinderSim = omni::tetfinder::createTetFinder(
        &volumeMeshInfo.simPositions[0], uint32_t(volumeMeshInfo.simPositions.size()),
        &volumeMeshInfo.simIndices[0], uint32_t(volumeMeshInfo.simIndices.size()));

    // Vertex overlaps
    if (desc.enableDeformableVertexAttachments)
    {
        //cull surface points
        std::vector<carb::Float3> srcPoints;
        std::vector<uint32_t> srcPointIndices;
        cullPointsToMaskShapes(srcPoints, srcPointIndices, maskShapes, 0.0f,
            &surfaceMeshInfo.simPositions[0], uint32_t(surfaceMeshInfo.simPositions.size()));

        //compute attachment points based on overlaps with tetmesh
        computeVtxTetAttachments(attachmentVtxIndices, attachmentTetIndices, attachmentTetCoords,
            srcPoints, srcPointIndices, volumeMeshInfo.simPositions, volumeMeshInfo.simIndices,
            tetFinderSim, desc.deformableVertexOverlapOffset);
    }

    // Filtering
    if (desc.enableCollisionFiltering)
    {
        uint64_t tetFinderColl = tetFinderSim;
        if (volumeMeshInfo.simMesh.GetPrim() != volumeMeshInfo.collMesh.GetPrim())
        {
            tetFinderColl = omni::tetfinder::createTetFinder(
                volumeMeshInfo.collPositions.data(), uint32_t(volumeMeshInfo.collPositions.size()),
                volumeMeshInfo.collIndices.data(), uint32_t(volumeMeshInfo.collIndices.size()));
        }

        VtArray<uint32_t> tetFilterGroupCounts;
        VtArray<uint32_t> tetFilterGroupIndices;
        VtArray<uint32_t> triFilterGroupCounts;
        VtArray<uint32_t> triFilterGroupIndices;

        if (maskShapes.geometries.empty())
        {
            //bi-directional overlap tet-mesh tri-mesh overlap
            if (desc.enableDeformableFilteringPairs)
            {
                ResultBuffer<carb::Int2> tetTriIdPairs;
                omni::tetfinder::overlapTetMeshTriMeshPairs(tetTriIdPairs.ptr, tetTriIdPairs.size,
                    tetFinderColl, surfaceMeshInfo.collPositions.data(),
                    surfaceMeshInfo.collIndices.data(), uint32_t(surfaceMeshInfo.collIndices.size()),
                    collisionFilteringOffset, ResultBuffer<>::allocate);

                addPairsToFilterGroups(tetFilterGroupCounts, tetFilterGroupIndices, triFilterGroupCounts, triFilterGroupIndices,
                    tetTriIdPairs.ptr, tetTriIdPairs.size, nullptr, nullptr);
            }
            else
            {
                ResultBuffer<int32_t> tetIds;
                ResultBuffer<int32_t> triIds;
                omni::tetfinder::overlapTetMeshTriMeshAny(tetIds.ptr, tetIds.size, triIds.ptr, triIds.size,
                    tetFinderColl, surfaceMeshInfo.collPositions.data(),
                    surfaceMeshInfo.collIndices.data(), uint32_t(surfaceMeshInfo.collIndices.size()),
                    collisionFilteringOffset, ResultBuffer<>::allocate);

                addPairsToFilterGroups(tetFilterGroupCounts, tetFilterGroupIndices, triFilterGroupCounts, triFilterGroupIndices,
                    tetIds.ptr, tetIds.size, triIds.ptr, triIds.size, nullptr, nullptr);
            }
        }
        else
        {
            //we need to cull tet mesh and tri mesh separately and test against other full mesh to catch all pairs
            {
                //cull src tets
                std::vector<uint32_t> culledSrcTetIds;
                uint64_t culledSrcTetFinderColl = cullTetsToMaskShapes(culledSrcTetIds, maskShapes, collisionFilteringOffset, tetFinderColl);

                if (desc.enableDeformableFilteringPairs)
                {
                    ResultBuffer<carb::Int2> tetTriIdPairs;
                    omni::tetfinder::overlapTetMeshTriMeshPairs(tetTriIdPairs.ptr, tetTriIdPairs.size,
                        culledSrcTetFinderColl, surfaceMeshInfo.collPositions.data(),
                        surfaceMeshInfo.collIndices.data(), uint32_t(surfaceMeshInfo.collIndices.size()),
                        collisionFilteringOffset, ResultBuffer<>::allocate);

                    addPairsToFilterGroups(tetFilterGroupCounts, tetFilterGroupIndices, triFilterGroupCounts, triFilterGroupIndices,
                        tetTriIdPairs.ptr, tetTriIdPairs.size, culledSrcTetIds.data(), nullptr);
                }
                else
                {
                    ResultBuffer<int32_t> tetIds;
                    ResultBuffer<int32_t> triIds;
                    omni::tetfinder::overlapTetMeshTriMeshAny(tetIds.ptr, tetIds.size, triIds.ptr, triIds.size,
                        culledSrcTetFinderColl, surfaceMeshInfo.collPositions.data(),
                        surfaceMeshInfo.collIndices.data(), uint32_t(surfaceMeshInfo.collIndices.size()),
                        collisionFilteringOffset, ResultBuffer<>::allocate);

                    addPairsToFilterGroups(tetFilterGroupCounts, tetFilterGroupIndices, triFilterGroupCounts, triFilterGroupIndices,
                        tetIds.ptr, tetIds.size, triIds.ptr, triIds.size, culledSrcTetIds.data(), nullptr);
                }
                tetfinder::releaseTetFinder(culledSrcTetFinderColl);
            }

            {
                uint64_t triFinderColl = trifinder::createTriFinder(surfaceMeshInfo.collPositions.data(), uint32_t(surfaceMeshInfo.collPositions.size()),
                    surfaceMeshInfo.collIndices.data(), uint32_t(surfaceMeshInfo.collIndices.size()));

                //cull src tris
                std::vector<uint32_t> culledSrcTriIds;
                uint64_t culledSrcTriFinderColl = cullTrisToMaskShapes(culledSrcTriIds, maskShapes, collisionFilteringOffset, triFinderColl);
                uint32_t culledSrcTriIndicesSize;
                const uint32_t* culledSrcTriIndices = trifinder::getIndices(culledSrcTriIndicesSize, culledSrcTriFinderColl);

                if (desc.enableDeformableFilteringPairs)
                {
                    ResultBuffer<carb::Int2> tetTriIdPairs;
                    omni::tetfinder::overlapTetMeshTriMeshPairs(tetTriIdPairs.ptr, tetTriIdPairs.size,
                        tetFinderColl, surfaceMeshInfo.collPositions.data(), culledSrcTriIndices, culledSrcTriIndicesSize,
                        collisionFilteringOffset, ResultBuffer<>::allocate);

                    addPairsToFilterGroups(tetFilterGroupCounts, tetFilterGroupIndices, triFilterGroupCounts, triFilterGroupIndices,
                        tetTriIdPairs.ptr, tetTriIdPairs.size, nullptr, culledSrcTriIds.data());
                }
                else
                {
                    ResultBuffer<int32_t> tetIds;
                    ResultBuffer<int32_t> triIds;
                    omni::tetfinder::overlapTetMeshTriMeshAny(tetIds.ptr, tetIds.size, triIds.ptr, triIds.size,
                        tetFinderColl, surfaceMeshInfo.collPositions.data(), culledSrcTriIndices, culledSrcTriIndicesSize,
                        collisionFilteringOffset, ResultBuffer<>::allocate);

                    addPairsToFilterGroups(tetFilterGroupCounts, tetFilterGroupIndices, triFilterGroupCounts, triFilterGroupIndices,
                        tetIds.ptr, tetIds.size, triIds.ptr, triIds.size, nullptr, culledSrcTriIds.data());
                }
                trifinder::releaseTriFinder(triFinderColl);
                trifinder::releaseTriFinder(culledSrcTriFinderColl);
            }
        }

        convertTetGroupsToSurfaceTriGroups(filterGroupCounts[0], filterGroupIndices[0],
            volumeMeshInfo.collIndices, volumeMeshInfo.collSurfaceTriToTetMap,
            tetFilterGroupCounts, tetFilterGroupIndices);

        filterGroupCounts[1].swap(triFilterGroupCounts);
        filterGroupIndices[1].swap(triFilterGroupIndices);

        compressFilterGroups(filterGroupCounts[0], filterGroupIndices[0], filterGroupCounts[1], filterGroupIndices[1]);

        if (tetFinderColl != tetFinderSim)
        {
            omni::tetfinder::releaseTetFinder(tetFinderColl);
        }
    }

    omni::tetfinder::releaseTetFinder(tetFinderSim);

    vtxTetAttachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0).Set(attachmentVtxIndices);
    vtxTetAttachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->tetIndicesSrc1).Set(attachmentTetIndices);
    vtxTetAttachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->tetCoordsSrc1).Set(attachmentTetCoords);
    vtxTetAttachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->attachmentEnabled).Set(desc.enableDeformableVertexAttachments);

    if (filterGroupCounts[volumeSlot].size() > 0)
    {
        filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts0).Set(filterGroupCounts[volumeSlot]);
        filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts1).Set(filterGroupCounts[surfaceSlot]);
        filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices0).Set(filterGroupIndices[volumeSlot]);
        filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices1).Set(filterGroupIndices[surfaceSlot]);
        filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->filterEnabled).Set(desc.enableCollisionFiltering);
    }
}

void updateDeformableVolumeVolumeAttachments(UsdPrim (&vtxTetAttachmentPrims)[2], UsdPrim filterPrim,
    const DeformableMeshInfo(&deformableMeshInfo)[2], const uint32_t(&slots)[2], const PhysxAutoAttachmentDesc& desc, const MaskShapes& maskShapes)
{
    UsdGeomBBoxCache cacheBBox(UsdTimeCode::Default(), { UsdGeomTokens->default_, UsdGeomTokens->proxy, UsdGeomTokens->guide});

    // Use the minimum average dimension
    float avg_dim0 = getAverageDim(deformableMeshInfo[0].collMesh.GetPrim(), cacheBBox);
    float avg_dim1 = getAverageDim(deformableMeshInfo[1].collMesh.GetPrim(), cacheBBox);
    float avg_dim = PxMin(avg_dim0, avg_dim1);
    float default_rad = avg_dim * 0.05f;

    // Apply default heuristics
    float collisionFilteringOffset = desc.collisionFilteringOffset;
    if (!isfinite(collisionFilteringOffset))
        collisionFilteringOffset = default_rad * 2;

    VtArray<int32_t> attachmentVtxIndices[2];
    VtArray<GfVec3f> attachmentTetCoords[2];
    VtArray<int32_t> attachmentTetIndices[2];
    VtArray<uint32_t> filterGroupCounts[2];
    VtArray<uint32_t> filterGroupIndices[2];

    uint64_t tetFinderSim[2] = { 0, 0 };
    for (uint32_t s = 0; s < 2; ++s)
    {
        tetFinderSim[s] = omni::tetfinder::createTetFinder(
            &deformableMeshInfo[s].simPositions[0], uint32_t(deformableMeshInfo[s].simPositions.size()),
            &deformableMeshInfo[s].simIndices[0], uint32_t(deformableMeshInfo[s].simIndices.size()));
    }

    // Vertex overlaps
    if (desc.enableDeformableVertexAttachments)
    {
        for (uint32_t s = 0; s < 2; ++s)
        {
            const DeformableMeshInfo& srcMeshInfo = deformableMeshInfo[s];
            const DeformableMeshInfo& dstMeshInfo = deformableMeshInfo[1-s];
            uint64_t dstTetFinderSim = tetFinderSim[1-s];

            //cull src points
            std::vector<carb::Float3> srcPoints;
            std::vector<uint32_t> srcPointIndices;
            cullPointsToMaskShapes(srcPoints, srcPointIndices, maskShapes, 0.0f,
                &srcMeshInfo.simPositions[0], uint32_t(srcMeshInfo.simPositions.size()));

            //compute attachment points based on overlaps with tetmesh
            computeVtxTetAttachments(attachmentVtxIndices[s], attachmentTetIndices[s], attachmentTetCoords[s],
                srcPoints, srcPointIndices, dstMeshInfo.simPositions, dstMeshInfo.simIndices,
                dstTetFinderSim, desc.deformableVertexOverlapOffset);
        }
    }

    // Filtering
    if (desc.enableCollisionFiltering)
    {
        uint64_t tetFinderColl[2] = { tetFinderSim[0], tetFinderSim[1] };
        for (uint32_t s = 0; s < 2; ++s)
        {
            if (deformableMeshInfo[s].simMesh.GetPrim() != deformableMeshInfo[s].collMesh.GetPrim())
            {
                tetFinderColl[s] = omni::tetfinder::createTetFinder(
                    &deformableMeshInfo[s].collPositions[0], uint32_t(deformableMeshInfo[s].collPositions.size()),
                    &deformableMeshInfo[s].collIndices[0], uint32_t(deformableMeshInfo[s].collIndices.size()));
            }
        }

        VtArray<uint32_t> tetFilterGroupCounts[2];
        VtArray<uint32_t> tetFilterGroupIndices[2];

        if (maskShapes.geometries.empty())
        {
            //bi-directional overlap tet-mesh tet-mesh overlap
            if (desc.enableDeformableFilteringPairs)
            {
                ResultBuffer<carb::Int2> tetIdPairs;
                omni::tetfinder::overlapTetMeshTetMeshPairs(tetIdPairs.ptr, tetIdPairs.size,
                    tetFinderColl[0], tetFinderColl[1], collisionFilteringOffset, ResultBuffer<>::allocate);

                addPairsToFilterGroups(tetFilterGroupCounts[0], tetFilterGroupIndices[0], tetFilterGroupCounts[1], tetFilterGroupIndices[1],
                    tetIdPairs.ptr, tetIdPairs.size, nullptr, nullptr);
            }
            else
            {
                ResultBuffer<int32_t> tetIds[2];
                omni::tetfinder::overlapTetMeshTetMeshAny(tetIds[0].ptr, tetIds[0].size, tetIds[1].ptr, tetIds[1].size,
                    tetFinderColl[0], tetFinderColl[1], collisionFilteringOffset, ResultBuffer<>::allocate);

                addPairsToFilterGroups(tetFilterGroupCounts[0], tetFilterGroupIndices[0], tetFilterGroupCounts[1], tetFilterGroupIndices[1],
                    tetIds[0].ptr, tetIds[0].size, tetIds[1].ptr, tetIds[1].size, nullptr, nullptr);
            }
        }
        else
        {
            //we need to cull each tet mesh separately and test against other full tet mesh to catch all pairs
            for (uint32_t s = 0; s < 2; ++s)
            {
                const DeformableMeshInfo& srcMeshInfo = deformableMeshInfo[s];
                const DeformableMeshInfo& dstMeshInfo = deformableMeshInfo[1-s];
                uint64_t srcTetFinderColl = tetFinderColl[s];
                uint64_t dstTetFinderColl = tetFinderColl[1-s];

                //cull src tets
                std::vector<uint32_t> culledSrcTetIds;
                uint64_t culledSrcTetFinderColl = cullTetsToMaskShapes(culledSrcTetIds, maskShapes, collisionFilteringOffset, srcTetFinderColl);

                if (desc.enableDeformableFilteringPairs)
                {
                    ResultBuffer<carb::Int2> tetIdPairs;
                    omni::tetfinder::overlapTetMeshTetMeshPairs(tetIdPairs.ptr, tetIdPairs.size,
                        culledSrcTetFinderColl, dstTetFinderColl, collisionFilteringOffset, ResultBuffer<>::allocate);

                    addPairsToFilterGroups(tetFilterGroupCounts[s], tetFilterGroupIndices[s], tetFilterGroupCounts[1-s], tetFilterGroupIndices[1-s],
                        tetIdPairs.ptr, tetIdPairs.size, culledSrcTetIds.data(), nullptr);
                }
                else
                {
                    ResultBuffer<int32_t> tetIds[2];
                    omni::tetfinder::overlapTetMeshTetMeshAny(tetIds[s].ptr, tetIds[s].size, tetIds[1-s].ptr, tetIds[1-s].size,
                        culledSrcTetFinderColl, dstTetFinderColl, collisionFilteringOffset, ResultBuffer<>::allocate);

                    addPairsToFilterGroups(tetFilterGroupCounts[s], tetFilterGroupIndices[s], tetFilterGroupCounts[1 - s], tetFilterGroupIndices[1 - s],
                        tetIds[0].ptr, tetIds[0].size, tetIds[1].ptr, tetIds[1].size, culledSrcTetIds.data(), nullptr);
                }
                tetfinder::releaseTetFinder(culledSrcTetFinderColl);
            }
        }

        for (uint32_t s = 0; s < 2; ++s)
        {
            convertTetGroupsToSurfaceTriGroups(filterGroupCounts[s], filterGroupIndices[s],
                deformableMeshInfo[s].collIndices, deformableMeshInfo[s].collSurfaceTriToTetMap,
                tetFilterGroupCounts[s], tetFilterGroupIndices[s]);
        }

        compressFilterGroups(filterGroupCounts[0], filterGroupIndices[0], filterGroupCounts[1], filterGroupIndices[1]);

        for (uint32_t s = 0; s < 2; ++s)
        {
            if (tetFinderColl[s] != tetFinderSim[s])
            {
                omni::tetfinder::releaseTetFinder(tetFinderColl[s]);
            }
        }
    }

    for (uint32_t s = 0; s < 2; ++s)
    {
        omni::tetfinder::releaseTetFinder(tetFinderSim[s]);

        vtxTetAttachmentPrims[s].GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0).Set(attachmentVtxIndices[s]);
        vtxTetAttachmentPrims[s].GetAttribute(OmniPhysicsDeformableAttrTokens->tetIndicesSrc1).Set(attachmentTetIndices[s]);
        vtxTetAttachmentPrims[s].GetAttribute(OmniPhysicsDeformableAttrTokens->tetCoordsSrc1).Set(attachmentTetCoords[s]);
        vtxTetAttachmentPrims[s].GetAttribute(OmniPhysicsDeformableAttrTokens->attachmentEnabled).Set(desc.enableDeformableVertexAttachments);
    }

    if (filterGroupCounts[slots[0]].size() > 0)
    {
        filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts0).Set(filterGroupCounts[slots[0]]);
        filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts1).Set(filterGroupCounts[slots[1]]);
        filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices0).Set(filterGroupIndices[slots[0]]);
        filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices1).Set(filterGroupIndices[slots[1]]);
        filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->filterEnabled).Set(desc.enableCollisionFiltering);
    }
}

void updateDeformableXformableAttachments(
    UsdPrim vtxXformAttachmentPrim,
    std::vector<UsdPrim>& filterPrims,
    const DeformableMeshInfo& deformableMeshInfo,
    const uint32_t deformableSlot,
    const UsdGeomXformable rigidRoot, 
    const std::vector<UsdPhysicsCollisionAPI>& rigidColliders,
    const PhysxAutoAttachmentDesc& desc, const MaskShapes& maskShapes)
{
    //we need to define all attachment local positions relative to
    //the same frame, because we don't want to have an attachment per
    //collider. we assume all colliders move in the same frame if they move.
    const GfMatrix4d rigidToWorld(rigidRoot.ComputeLocalToWorldTransform(UsdTimeCode::Default()));
    GfMatrix4d worldToRigid = rigidToWorld.GetInverse();

    VtArray<int32_t> attachmentVtxIndicesDeformable;
    VtArray<GfVec3f> attachmentVtxPointsXformable;
    VtArray<uint32_t> filterTriIndicesDeformable;

    if (rigidColliders.size() == 0)
    {
        if (desc.enableDeformableVertexAttachments)
        {
            std::vector<carb::Float3> culledVertices;
            std::vector<uint32_t> culledVertexIndices;
            cullPointsToMaskShapes(culledVertices, culledVertexIndices, maskShapes, 0.0f,
                deformableMeshInfo.simPositions.data(), uint32_t(deformableMeshInfo.simPositions.size()));

            for (PxU32 i = 0; i < culledVertices.size(); i++)
            {
                const uint32_t vertexIndex = culledVertexIndices[i];
                const PxVec3 vertexPos = toPhysX(culledVertices[i]);

                attachmentVtxIndicesDeformable.push_back(int32_t(vertexIndex));

                GfVec3f rigidPos = worldToRigid.Transform(GfVec3f(vertexPos.x, vertexPos.y, vertexPos.z));
                attachmentVtxPointsXformable.push_back(rigidPos);
            }
        }
    }
    else
    {
        for (size_t i = 0; i < rigidColliders.size(); ++i)
        {
            UsdPhysicsCollisionAPI rigidCollider = rigidColliders[i];

            filterTriIndicesDeformable.clear();

            UserDataInfo userData
            {
                attachmentVtxIndicesDeformable, attachmentVtxPointsXformable, filterTriIndicesDeformable,
                deformableMeshInfo, worldToRigid, desc, maskShapes
            };

            const GfMatrix4d mat = UsdGeomXformable(rigidCollider.GetPrim()).ComputeLocalToWorldTransform(UsdTimeCode::Default());
            const GfTransform tr(mat);

            if (!(rigidCollider.GetPrim().IsA<UsdGeomGprim>()))
                continue;

            const uint64_t stageId = UsdUtilsStageCache::Get().GetId(rigidCollider.GetPrim().GetStage()).ToLongInt();
            usdparser::PhysxShapeDesc* shapeDesc = usdparser::parseCollision(stageId, rigidCollider.GetPath(), rigidCollider.GetPath());
            if (!shapeDesc)
                continue;

            const GfVec3d pos = tr.GetTranslation();
            const GfQuatd rot = tr.GetRotation().GetQuat();
            const GfVec3d scale = tr.GetScale();

            shapeDesc->localPos = { float(pos[0]), float(pos[1]), float(pos[2]) };
            shapeDesc->localRot = { float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]), float(rot.GetImaginary()[2]), float(rot.GetReal()) };
            shapeDesc->localScale = { float(scale[0]), float(scale[1]), float(scale[2]) };

            processRigidShapeGeometry(rigidCollider.GetPrim(), shapeDesc, updateDeformableRigidColliderAttachments, &userData);

            UsdPrim filterPrim = filterPrims[i];
            if (filterTriIndicesDeformable.size() > 0)
            {
                if (deformableSlot == 0)
                {
                    filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts0).Set(VtArray<uint32_t>({ uint32_t(filterTriIndicesDeformable.size()) }));
                    filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices0).Set(filterTriIndicesDeformable);
                    filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts1).Set(VtArray<uint32_t>({}));
                    filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices1).Set(VtArray<uint32_t>({}));
                }
                else
                {
                    filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts0).Set(VtArray<uint32_t>({}));
                    filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices0).Set(VtArray<uint32_t>({}));
                    filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemCounts1).Set(VtArray<uint32_t>({ uint32_t(filterTriIndicesDeformable.size()) }));
                    filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->groupElemIndices1).Set(filterTriIndicesDeformable);
                }
                filterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->filterEnabled).Set(desc.enableCollisionFiltering);
            }
        }
    }

    vtxXformAttachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->vtxIndicesSrc0).Set(attachmentVtxIndicesDeformable);
    vtxXformAttachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->localPositionsSrc1).Set(attachmentVtxPointsXformable);
    vtxXformAttachmentPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->attachmentEnabled).Set(desc.enableDeformableVertexAttachments);
}

bool getDeformableMeshInfo(DeformableMeshInfo& deformableMeshInfo, UsdPrim deformableBodyPrim, const PhysxDeformableBodyDesc& deformableDesc)
{
    UsdGeomPointBased simMesh = UsdGeomPointBased::Get(deformableBodyPrim.GetStage(), deformableDesc.simMeshPath);
    if (!simMesh)
    {
        return false;
    }

    const GfMatrix4d simToWorld(simMesh.ComputeLocalToWorldTransform(UsdTimeCode::Default()));

    VtArray<GfVec3f> simMeshPoints;
    simMesh.GetPointsAttr().Get(&simMeshPoints);

    std::vector<uint32_t> simIndices;
    if (deformableDesc.type == ObjectType::eVolumeDeformableBody)
    {
        UsdGeomTetMesh simTetMesh(simMesh);
        if (!simTetMesh)
        {
            return false;
        }

        VtArray<GfVec4i> tmpIndices;
        simTetMesh.GetTetVertexIndicesAttr().Get(&tmpIndices);

        PX_COMPILE_TIME_ASSERT(sizeof(GfVec4i) == sizeof(uint32_t) * 4);
        simIndices.resize(tmpIndices.size() * 4);
        std::memcpy(simIndices.data(), tmpIndices.data(), simIndices.size() * sizeof(uint32_t));

        deformableMeshInfo.type = AttachmentActorType::eVOLUME_DEFORMABLE;
    }
    else if (deformableDesc.type == ObjectType::eSurfaceDeformableBody)
    {
        UsdGeomMesh simTriMesh(simMesh);
        if (!simTriMesh)
        {
            return false;
        }

        VtArray<int32_t> tmpIndices;
        simTriMesh.GetFaceVertexIndicesAttr().Get(&tmpIndices);
        simIndices.resize(tmpIndices.size());
        std::memcpy(simIndices.data(), tmpIndices.data(), simIndices.size() * sizeof(uint32_t));

        deformableMeshInfo.type = AttachmentActorType::eSURFACE_DEFORMABLE;
    }

    deformableMeshInfo.simPositions.resize(simMeshPoints.size());
    for (size_t i = 0; i < deformableMeshInfo.simPositions.size(); ++i)
    {
        GfVec3f position = simToWorld.Transform(simMeshPoints[i]);
        deformableMeshInfo.simPositions[i] = { position[0], position[1], position[2] };
    }

    deformableMeshInfo.simIndices.swap(simIndices);
    deformableMeshInfo.simMesh = simMesh;

    if (deformableDesc.type == ObjectType::eVolumeDeformableBody)
    {
        // could take some shortcuts here, if collision mesh equals simulation mesh.
        // however, since for filtering we use tet mesh surface triangles, as opposed to tets we
        // treat the collision mesh separately anyways.
        UsdPrim collMeshPrim = deformableBodyPrim.GetStage()->GetPrimAtPath(deformableDesc.collisionMeshPath);
        UsdGeomTetMesh collTetMesh(collMeshPrim);
        if (!collTetMesh)
        {
            return false;
        }

        const GfMatrix4d collToWorld(UsdGeomXformable(collMeshPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default()));

        VtArray<GfVec3f> collMeshPoints;
        UsdGeomPointBased(collMeshPrim).GetPointsAttr().Get(&collMeshPoints);

        VtArray<GfVec4i> vtxTetIndices;
        collTetMesh.GetTetVertexIndicesAttr().Get(&vtxTetIndices);
        std::vector<uint32_t> collIndices(vtxTetIndices.size()*4);
        std::memcpy(collIndices.data(), vtxTetIndices.data(), sizeof(uint32_t) * collIndices.size());

        std::vector<uint32_t> collSurfaceTriToTetMap;
        std::vector<uint32_t> collSurfaceTriIndices;
        {
            bool hasSurface = parseTetMeshSurface(collSurfaceTriIndices, collSurfaceTriToTetMap, collTetMesh);
            if (!hasSurface)
                return false;
        }

        deformableMeshInfo.collPositions.resize(collMeshPoints.size());
        for (size_t i = 0; i < deformableMeshInfo.collPositions.size(); ++i)
        {
            GfVec3f position = collToWorld.Transform(collMeshPoints[i]);
            deformableMeshInfo.collPositions[i] = { position[0], position[1], position[2] };
        }

        deformableMeshInfo.collIndices.swap(collIndices);
        deformableMeshInfo.collSurfaceTriIndices.swap(collSurfaceTriIndices);
        deformableMeshInfo.collSurfaceTriToTetMap.swap(collSurfaceTriToTetMap);
        deformableMeshInfo.collMesh = UsdGeomPointBased(collMeshPrim);
    }
    else if (deformableDesc.type == ObjectType::eSurfaceDeformableBody)
    {
        deformableMeshInfo.collPositions.assign(deformableMeshInfo.simPositions.begin(), deformableMeshInfo.simPositions.end());
        deformableMeshInfo.collIndices.assign(deformableMeshInfo.simIndices.begin(), deformableMeshInfo.simIndices.end());
        deformableMeshInfo.collMesh = simMesh;
    }
    else
    {
        return false;
    }

    deformableMeshInfo.deformableBodyDataCrc = loadMeshKey(deformableBodyPrim, deformableBodyDataCrcToken);

    return true;
}

omni::physx::usdparser::MeshKey calculateAutoAttachmentCRC(const PhysxAutoAttachmentDesc& autoAttachmentDesc, const MaskShapes& maskShapes)
{
    omni::physx::usdparser::MeshKey meshKey;
    meshKey.setMiscData((const uint8_t*)&autoAttachmentDesc.enableDeformableVertexAttachments, sizeof(autoAttachmentDesc.enableDeformableVertexAttachments));
    meshKey.setMiscData((const uint8_t*)&autoAttachmentDesc.deformableVertexOverlapOffset, sizeof(autoAttachmentDesc.deformableVertexOverlapOffset));
    meshKey.setMiscData((const uint8_t*)&autoAttachmentDesc.enableRigidSurfaceAttachments, sizeof(autoAttachmentDesc.enableRigidSurfaceAttachments));
    meshKey.setMiscData((const uint8_t*)&autoAttachmentDesc.rigidSurfaceSamplingDistance, sizeof(autoAttachmentDesc.rigidSurfaceSamplingDistance));
    meshKey.setMiscData((const uint8_t*)&autoAttachmentDesc.enableCollisionFiltering, sizeof(autoAttachmentDesc.enableCollisionFiltering));
    meshKey.setMiscData((const uint8_t*)&autoAttachmentDesc.collisionFilteringOffset, sizeof(autoAttachmentDesc.collisionFilteringOffset));
    meshKey.setMiscData((const uint8_t*)&autoAttachmentDesc.enableDeformableFilteringPairs, sizeof(autoAttachmentDesc.enableDeformableFilteringPairs));

    for (size_t i = 0; i < maskShapes.geometries.size(); ++i)
    {
        const PxTransform& transform = maskShapes.transforms[i];
        const PxGeometryHolder& geoHolder = maskShapes.geometries[i];
        const PxGeometryType::Enum type = geoHolder.getType();
        if (type == PxGeometryType::eSPHERE)
        {
            const PxSphereGeometry& sphereGeo = geoHolder.sphere();
            meshKey.setMiscData((const uint8_t*)&sphereGeo.radius, sizeof(sphereGeo.radius));
        }
        else if (type == PxGeometryType::eCAPSULE)
        {
            const PxCapsuleGeometry& capsuleGeo = geoHolder.capsule();
            meshKey.setMiscData((const uint8_t*)&capsuleGeo.radius, sizeof(capsuleGeo.radius));
            meshKey.setMiscData((const uint8_t*)&capsuleGeo.halfHeight, sizeof(capsuleGeo.halfHeight));
        }
        else if (type == PxGeometryType::eBOX)
        {
            const PxBoxGeometry& boxGeo = geoHolder.box();
            meshKey.setMiscData((const uint8_t*)&boxGeo.halfExtents, sizeof(boxGeo.halfExtents));
        }
        meshKey.setMiscData((const uint8_t*)&transform, sizeof(transform));
    }

    return meshKey;
}

struct DeformableDescRef
{
    ~DeformableDescRef() { *this = nullptr; }
    DeformableDescRef& operator=(PhysxDeformableBodyDesc* inDesc)
    {
        if (desc != inDesc)
        {
            ICE_FREE(desc);
            desc = inDesc;
        }
        return *this;
    }
    PhysxDeformableBodyDesc* operator->() { return desc; }
    bool operator!() const { return desc == nullptr; }
    PhysxDeformableBodyDesc* desc = nullptr;
};

bool parseAttachables(AttachmentActorType::Enum(&types)[2], UsdPrim(&attachablePrims)[2], DeformableDescRef(&deformableDescs)[2],
    uint32_t(&attachableSlots)[2],
    uint32_t& numDeformables,
    uint32_t& numXformables,
    std::vector<UsdPhysicsCollisionAPI>& rigidColliders,
    const UsdPrim autoAttachmentPrim,
    CookingDataAsync& cookingDataAsync)
{
    UsdStageWeakPtr stage = autoAttachmentPrim.GetStage();

    SdfPath attachablePath[2];
    parseAttachablePaths(attachablePath, autoAttachmentPrim);

    for (uint32_t t = 0; t < 2; ++t)
    {
        if (!attachablePath[t].IsEmpty())
        {
            TfType dbType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
            attachablePrims[t] = stage->GetPrimAtPath(attachablePath[t]);
            if (!attachablePrims[t].IsValid())
            {
                return false;
            }
            else if (attachablePrims[t].HasAPI(dbType))
            {
                PhysxDeformableBodyDesc* desc = cookingDataAsync.parseDeformableBody(attachablePrims[t]);
                if (!desc)
                {
                    return false;
                }

                if (desc->type == ObjectType::eSurfaceDeformableBody)
                {
                    types[t] = AttachmentActorType::eSURFACE_DEFORMABLE;
                }
                else if (desc->type == ObjectType::eVolumeDeformableBody)
                {
                    types[t] = AttachmentActorType::eVOLUME_DEFORMABLE;
                }
                deformableDescs[t] = desc;
            }
            else if (attachablePrims[t].IsA<UsdGeomXformable>())
            {
                getColliders(rigidColliders, attachablePrims[t]);
                types[t] = AttachmentActorType::eXFORMABLE;
            }
        }
        else
        {
            return false;
        }
    }

    // Make sure that
    // * the first actor is always a deformable
    // * the first actor is always a volume deformable if present
    for (uint32_t t = 0; t < 2; ++t)
    {
        if (types[0] == AttachmentActorType::eVOLUME_DEFORMABLE)
        {
            if (types[1] == AttachmentActorType::eXFORMABLE)
            {
                numDeformables = 1;
                numXformables = 1;
                break;
            }
            else if (types[1] & AttachmentActorType::eDEFORMABLE)
            {
                numDeformables = 2;
                break;
            }
        }
        else if (types[0] & AttachmentActorType::eDEFORMABLE)
        {
            if (types[1] == AttachmentActorType::eXFORMABLE)
            {
                numDeformables = 1;
                numXformables = 1;
                break;
            }
        }

        std::swap(attachablePrims[0], attachablePrims[1]);
        std::swap(deformableDescs[0].desc, deformableDescs[1].desc);
        std::swap(attachableSlots[0], attachableSlots[1]);
        std::swap(types[0], types[1]);
    }

    return true;
}

void disableAttachmentsAndFilters(const UsdPrim autoAttachmentPrim)
{
    UsdPrimSiblingRange children = autoAttachmentPrim.GetChildren();
    for (UsdPrim child : children)
    {
        if (child.IsA(UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->Attachment)))
        {
            child.GetAttribute(OmniPhysicsDeformableAttrTokens->attachmentEnabled).Set(false);
        }
        else if (child.IsA(UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter)))
        {
            child.GetAttribute(OmniPhysicsDeformableAttrTokens->filterEnabled).Set(false);
        }
    }
}

void removeAttachmentsAndFilters(UsdPrim autoAttachmentPrim)
{
    UsdStageWeakPtr stage = autoAttachmentPrim.GetStage();
    if (!stage)
    {
        return;
    }

    std::vector<SdfPath> toRemove;
    UsdPrimSiblingRange children = autoAttachmentPrim.GetChildren();
    for (UsdPrim child : children)
    {
        if (child.IsA(UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->Attachment)) ||
            child.IsA(UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter)))
        {
            toRemove.push_back(child.GetPath());
        }
    }
    for (SdfPath path : toRemove)
    {
        stage->RemovePrim(path);
    }
}

/**
* Initial setup of auto attachment. Creates all necessary sub prims depending on the attachable types.
* Pre-existing sub prims just get removed.
*/
bool setupAutoDeformableAttachment(const SdfPath& autoAttachmentPath)
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    UsdPrim autoAttachmentPrim = stage->GetPrimAtPath(autoAttachmentPath);
    if (!autoAttachmentPrim)
        return false;

    TfType autoAttachmentType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableAttachmentAPI);
    if (!autoAttachmentPrim.HasAPI(autoAttachmentType))
        return false;

    CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (!cookingDataAsync)
    {
        CARB_LOG_WARN("setupAutoDeformableAttachment: failed - Cooking not available");
        return false;
    }

    SdfPath attachablePaths[2];
    parseAttachablePaths(attachablePaths, autoAttachmentPrim);

    AttachmentActorType::Enum types[2] = { AttachmentActorType::eINVALID, AttachmentActorType::eINVALID };
    UsdPrim attachablePrims[2];
    DeformableDescRef deformableDescs[2] = { nullptr, nullptr };
    uint32_t attachableSlots[2] = { 0, 1 };
    uint32_t numDeformables = 0;
    uint32_t numXformables = 0;
    std::vector<UsdPhysicsCollisionAPI> rigidColliders;

    if (!parseAttachables(types, attachablePrims, deformableDescs, attachableSlots, numDeformables,
        numXformables, rigidColliders, autoAttachmentPrim, *cookingDataAsync))
    {
        CARB_LOG_WARN("setupAutoDeformableAttachment: parsing attachables failed");
        return false;
    }

    // Early out because there is no valid attachment combination (at least 1 volume or surface deformable)
    if (numDeformables == 0)
        return false;

    removeAttachmentsAndFilters(autoAttachmentPrim);

    // Add all possibly needed primitives, they can always be disabled, if not needed. This is according to the strategy
    // to never create primitives asynchronously. We are adding the targets already, because parsing might already
    // happen before authoring gets to generate the data.
    if (numXformables == 1)
    {
        UsdGeomXformable xformable(attachablePrims[1]);
        UsdGeomPointBased simMesh = UsdGeomPointBased::Get(stage, deformableDescs[0]->simMeshPath);
        UsdGeomPointBased collMesh = UsdGeomPointBased::Get(stage, deformableDescs[0]->collisionMeshPath);

        SdfPath vtxXformPath = autoAttachmentPath.AppendElementString("vtx_xform_attachment");
        UsdPrim vtxXformPrim = stage->DefinePrim(vtxXformPath, OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
        vtxXformPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->attachmentEnabled).Set(false);
        // TODO extend UsdPhysics with TriXform and TetXform attachments ?
        vtxXformPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src0).SetTargets({ simMesh.GetPath() });
        vtxXformPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src1).SetTargets({ xformable.GetPath() });

        for (uint32_t c = 0; c < uint32_t(rigidColliders.size()); ++c)
        {
            UsdPhysicsCollisionAPI rigidCollider = rigidColliders[c];
            char primName[64]; sprintf_s(primName, 64, "element_filter_%d", c);
            SdfPath elementFilterPath = autoAttachmentPath.AppendElementString(primName);
            UsdPrim elementFilterPrim = stage->DefinePrim(elementFilterPath, OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);
            elementFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->filterEnabled).Set(false);
            elementFilterPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src0).SetTargets({ attachableSlots[0] == 0 ? collMesh.GetPath() : rigidCollider.GetPath() });
            elementFilterPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src1).SetTargets({ attachableSlots[0] == 0 ? rigidCollider.GetPath() : collMesh.GetPath() });
        }
    }
    else if (numDeformables == 2)
    {
        UsdGeomPointBased simMeshes[2] = {
            UsdGeomPointBased::Get(stage, deformableDescs[0]->simMeshPath),
            UsdGeomPointBased::Get(stage, deformableDescs[1]->simMeshPath) };

        UsdGeomPointBased collMeshes[2] = {
            UsdGeomPointBased::Get(stage, deformableDescs[0]->collisionMeshPath),
            UsdGeomPointBased::Get(stage, deformableDescs[1]->collisionMeshPath) };

        // Create vtxVtx for snapping vertex to vertex attachments
        SdfPath vtxVtxPath = autoAttachmentPath.AppendElementString("vtx_vtx_attachment");
        UsdPrim vtxVtxPrim = stage->DefinePrim(vtxVtxPath, OmniPhysicsDeformableTypeTokens->VtxVtxAttachment);
        vtxVtxPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->attachmentEnabled).Set(false);
        vtxVtxPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src0).SetTargets({ simMeshes[attachableSlots[0]].GetPath() });
        vtxVtxPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src1).SetTargets({ simMeshes[attachableSlots[1]].GetPath() });

        for (uint32_t t = 0; t < 2; ++t)
        {
            uint32_t other = 1 - t;
            if (types[t] == AttachmentActorType::eSURFACE_DEFORMABLE)
            {
                // Create vtxTri for snapping vertex to triangle attachments, we don't support triangle offsets yet
                char primName[64]; sprintf_s(primName, 64, "vtx%d_tri%d_attachment", attachableSlots[other], attachableSlots[t]);
                SdfPath vtxTriPath = autoAttachmentPath.AppendElementString(primName);
                UsdPrim vtxTriPrim = stage->DefinePrim(vtxTriPath, OmniPhysicsDeformableTypeTokens->VtxTriAttachment);
                vtxTriPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->attachmentEnabled).Set(false);
                vtxTriPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src0).SetTargets({ simMeshes[other].GetPath() });
                vtxTriPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src1).SetTargets({ simMeshes[t].GetPath() });
            }
            else if (types[t] == AttachmentActorType::eVOLUME_DEFORMABLE)
            {
                // Not supporting surface tri attachments yet
                char primName[64]; sprintf_s(primName, 64, "vtx%d_tet%d_attachment", attachableSlots[other], attachableSlots[t]);
                SdfPath vtxTetPath = autoAttachmentPath.AppendElementString(primName);
                UsdPrim vtxTetPrim = stage->DefinePrim(vtxTetPath, OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
                vtxTetPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->attachmentEnabled).Set(false);
                vtxTetPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src0).SetTargets({ simMeshes[other].GetPath() });
                vtxTetPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src1).SetTargets({ simMeshes[t].GetPath() });
            }
        }

        SdfPath elementFilterPath = autoAttachmentPath.AppendElementString("element_filter");
        UsdPrim elementFilterPrim = stage->DefinePrim(elementFilterPath, OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);
        elementFilterPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->filterEnabled).Set(false);
        elementFilterPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src0).SetTargets({ collMeshes[attachableSlots[0]].GetPath() });
        elementFilterPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->src1).SetTargets({ collMeshes[attachableSlots[1]].GetPath() });
    }

    return true;
}

bool updateAutoDeformableAttachment(const SdfPath& autoAttachmentPath)
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    UsdPrim autoAttachmentPrim = stage->GetPrimAtPath(autoAttachmentPath);
    if (!autoAttachmentPrim)
        return false;

    TfType autoAttachmentType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableAttachmentAPI);
    if (!autoAttachmentPrim.HasAPI(autoAttachmentType))
        return false;

    PhysxAutoAttachmentDesc autoAttachmentDesc;
    parsePhysxAutoAttachment(autoAttachmentDesc, autoAttachmentPrim);

    CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (!cookingDataAsync)
    {
        CARB_LOG_WARN("updateAutoDeformableAttachment: failed - Cooking not available");
        return false;
    }

    AttachmentActorType::Enum types[2] = { AttachmentActorType::eINVALID, AttachmentActorType::eINVALID };
    UsdPrim attachablePrims[2];
    DeformableDescRef deformableDescs[2] = { nullptr, nullptr };
    uint32_t attachableSlots[2] = { 0, 1 };
    uint32_t numDeformables = 0;
    uint32_t numXformables = 0;
    std::vector<UsdPhysicsCollisionAPI> rigidColliders;

    if (!parseAttachables(types, attachablePrims, deformableDescs, attachableSlots, numDeformables,
        numXformables, rigidColliders, autoAttachmentPrim, *cookingDataAsync))
    {
        CARB_LOG_WARN("updateAutoDeformableAttachment: parsing attachables failed");
        return false;
    }

    // Early out because there is no valid attachment combination (at least 1 volume or surface deformable)
    if (numDeformables == 0)
        return false;

    UsdRelationship maskShapesRel = autoAttachmentPrim.GetRelationship(PhysxAdditionAttrTokens->maskShapes);
    MaskShapes maskShapes;
    if (maskShapesRel)
    {
        SdfPathVector targets;
        maskShapesRel.GetTargets(&targets);

        for (uint32_t i = 0; i < targets.size(); ++i)
        {
            UsdPrim maskShapePrim = stage->GetPrimAtPath(targets[i]);
            if (maskShapePrim)
            {
                PxGeometryHolder holder;
                PxTransform transform;
                bool success = parseMaskShape(holder, transform, maskShapePrim);
                if (success)
                {
                    maskShapes.geometries.push_back(holder);
                    maskShapes.transforms.push_back(transform);
                }
            }
        }
    }

    // compute hash of auto attachment computation inputs, for which we care to recompute if different:
    // auto parameters
    // deformable mesh keys
    omni::physx::usdparser::MeshKey inputCrc = calculateAutoAttachmentCRC(autoAttachmentDesc, maskShapes);

    DeformableMeshInfo deformableMeshInfo[2];
    for (uint32_t d = 0; d < numDeformables; ++d)
    {
        const UsdPrim prim = attachablePrims[d];
        AttachmentActorType::Enum type = types[d];
        PhysxDeformableBodyDesc* desc = deformableDescs[d].desc;
        if ((type & AttachmentActorType::eDEFORMABLE) == 0 || desc == nullptr)
        {
            return false;
        }

        if (desc->hasAutoAPI)
        {
            if (type == AttachmentActorType::eSURFACE_DEFORMABLE)
            {
                cookingDataAsync->cookSurfaceDeformableBody(*(PhysxSurfaceDeformableBodyDesc*)desc, prim, false);
            }
            else if (type == AttachmentActorType::eVOLUME_DEFORMABLE)
            {
                cookingDataAsync->cookVolumeDeformableBody(*(PhysxVolumeDeformableBodyDesc*)desc, prim, false);
            }
        }

        if (!getDeformableMeshInfo(deformableMeshInfo[d], prim, *desc))
            return false;

        inputCrc.setMeshKey(deformableMeshInfo[d].deformableBodyDataCrc);
    }

    // load attachment mesh key
    omni::physx::usdparser::MeshKey usdInputCrc = loadMeshKey(autoAttachmentPrim, autoDeformableAttachmentInputCrcToken);
    if (usdInputCrc == inputCrc)
    {
        // nothing to compute
        return true;
    }
    storeMeshKey(autoAttachmentPrim, autoDeformableAttachmentInputCrcToken, inputCrc);

    disableAttachmentsAndFilters(autoAttachmentPrim);

    if (numXformables == 1)
    {
        DeformableMeshInfo& deformableInfo = deformableMeshInfo[0];
        UsdGeomXformable xformable(attachablePrims[1]);
        UsdPrim vtxXformPrim;
        std::vector<UsdPrim> filtersPrims;
        bool validAttachment = getVtxXformAttachment(vtxXformPrim, autoAttachmentPrim, deformableInfo.simMesh, xformable);
        bool validFilters = getElementCollisionFilters(filtersPrims, autoAttachmentPrim, rigidColliders, UsdPhysicsCollisionAPI(deformableInfo.collMesh));

        if (validAttachment && validFilters)
        {
            updateDeformableXformableAttachments(vtxXformPrim, filtersPrims, deformableInfo, attachableSlots[0], xformable, rigidColliders,
                autoAttachmentDesc, maskShapes);
        }
        else
        {
            PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, "Deformable attachment targets changed, please refresh or re-create attachment!");
        }
    }
    else if (numDeformables == 2)
    {
        if (types[0] == AttachmentActorType::eVOLUME_DEFORMABLE)
        {
            if (types[1] == AttachmentActorType::eVOLUME_DEFORMABLE)
            {
                UsdPrim vtxTetPrims[2];
                UsdPrim filterPrim;
                bool validAttachment0 = getVtxTetAttachment(vtxTetPrims[0], autoAttachmentPrim, deformableMeshInfo[0].simMesh, deformableMeshInfo[1].simMesh);
                bool validAttachment1 = getVtxTetAttachment(vtxTetPrims[1], autoAttachmentPrim, deformableMeshInfo[1].simMesh, deformableMeshInfo[0].simMesh);
                bool validFilter = getElementCollisionFilter(filterPrim, autoAttachmentPrim,
                    UsdPhysicsCollisionAPI(deformableMeshInfo[0].collMesh),
                    UsdPhysicsCollisionAPI(deformableMeshInfo[1].collMesh));

                if (validAttachment0 && validAttachment1 && validFilter)
                {
                    updateDeformableVolumeVolumeAttachments(vtxTetPrims, filterPrim, deformableMeshInfo, attachableSlots, autoAttachmentDesc, maskShapes);
                }
                else
                {
                    PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, "Deformable attachment targets changed, please refresh or re-create attachment!");
                }
            }
            else if (types[1] == AttachmentActorType::eSURFACE_DEFORMABLE)
            {
                UsdPrim vtxTetPrim;
                UsdPrim filterPrim;
                bool validAttachment = getVtxTetAttachment(vtxTetPrim, autoAttachmentPrim, deformableMeshInfo[1].simMesh, deformableMeshInfo[0].simMesh);
                bool validFilter = getElementCollisionFilter(filterPrim, autoAttachmentPrim,
                    UsdPhysicsCollisionAPI(deformableMeshInfo[attachableSlots[0]].collMesh),
                    UsdPhysicsCollisionAPI(deformableMeshInfo[attachableSlots[1]].collMesh));

                if (validAttachment && validFilter)
                {
                    updateDeformableVolumeSurfaceAttachments(vtxTetPrim, filterPrim,
                        deformableMeshInfo[0], attachableSlots[0],
                        deformableMeshInfo[1], attachableSlots[1],
                        autoAttachmentDesc, maskShapes);
                }
                else
                {
                    PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, "Deformable attachment targets changed, please refresh or re-create attachment!");
                }
            }
        }
        else if (types[0] == AttachmentActorType::eSURFACE_DEFORMABLE)
        {
            if (types[1] == AttachmentActorType::eSURFACE_DEFORMABLE)
            {
                //no support yet
                //updateDeformableSurfaceSurfaceAttachments
            }
        }
    }

    return true;
}

} // namespace physx
} // namespace omni
