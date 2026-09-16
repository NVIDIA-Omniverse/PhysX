// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-24
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-23 AC-27 AC-30
 *
 * @implements REQ-WRITE-AUTHORING-001
 * @covers AC-5
 *
 * @implements REQ-SIM-AUTOATTACH-001
 * @covers AC-1 AC-3 AC-5
 */

// Auto-attachment sub prims exist in one of two forms (ADR-0028):
//  * authored USD prims, created through usdBridge/AttachmentAuthoringBridge.h when the attach
//    has a live USD stage (canAuthor()); the stage's change notices then create the objects;
//  * an in-memory GeneratedAutoAttachmentLayout on the AttachedStage otherwise (ovstage), with
//    the runtime objects created directly from it. Nothing is pushed back into the source.
//
// The payloads are authored through AttachedStage::getAuthoringDataWrite()
// (REQ-WRITE-AUTHORING-001), never getDataWrite(): the ovstage sink is null by design, and the
// authoring accessor falls back to a sink over the resident backing USD stage that resolves
// ObjectKeys/TokenIds through the active (ovstage) source. That fallback is what keeps the
// create gate -- canAuthor(), which asks about the live stage -- and the publish gate
// agreeing, so an ovstage attach with a backing stage cannot end up with created-but-empty
// attachment/filter prims. Without any stage the payload lives in the generated-data cache only.

#include "PhysXAttachment.h"
#include "PhysXTetFinder.h"
#include "PhysXTriFinder.h"

#include <omni/physics/parse/KnownTokens.h>

#include <common/foundation/MatrixTools.h>

#include <usdLoad/LoadUsd.h>
#include <usdLoad/IceDescriptorAllocator.h>
#include <usdLoad/ScannedShapeCookingDispatch.h>

#include <omni/physics/parse/ScanBackend.h>
#include <omni/physics/parse/ScannedStage.h>

#include <usdInterface/UsdInterface.h>

#include <PhysXScene.h>
#include <PhysXTools.h>
#include <ConeCylinderConvexMesh.h>
#include <CookingDataAsync.h>

// Every toPhysX()/toPhysXQuat() math-conversion call in this file (as opposed to the
// GfVec3f/VtArray USD write-sink helpers above) takes a carb::Float2/3/4, never a Gf type --
// despite this TU also pulling in PhysXTools.h, which brings the Gf-typed TypeCast.h overloads
// too. Include the pxr-free half directly (same swap SplinesCurve.h made) so this file's own
// math conversions do not rely on TypeCast.h/PhysXTools.h happening to supply them.
#include <common/foundation/CarbPhysXCast.h>

#include <usdBridge/AttachmentAuthoringBridge.h>

using namespace ::physx;
using namespace omni::physx::usdparser;
using namespace cookingdataasync;

#if !CARB_PLATFORM_WINDOWS
#define sprintf_s snprintf
#endif

// Plain names, interned to a TokenId per-call via IPhysicsSource::internToken (see
// loadMeshKey/storeMeshKey below) rather than materialized as a TfToken once here --
// these two CRC attributes are not part of KnownTokens' standard vocabulary, and this
// keeps loadMeshKey/storeMeshKey pxr-free.
static constexpr std::string_view autoDeformableAttachmentInputCrcToken{ "physxAutoDeformableAttachment:inputCrc" };
static constexpr std::string_view deformableBodyDataCrcToken{ "physxDeformableBody:deformableBodyDataCrc" };

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
    omni::physics::parse::ObjectKey simMeshKey;
    std::vector<uint32_t> simIndices;
    std::vector<carb::Float3> simPositions;

    omni::physics::parse::ObjectKey collMeshKey;
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

void parsePhysxAutoAttachment(usdparser::AttachedStage& attachedStage,
                              PhysxAutoAttachmentDesc& autoAttachmentDesc,
                              omni::physics::parse::ObjectKey autoAttachmentKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    const auto read = [&](omni::physics::parse::TokenId attr, auto& out) {
        src->getAttribute(autoAttachmentKey, attr, out);
    };
    read(tok.physxAutoDeformableAttachmentEnableDeformableVertexAttachments, autoAttachmentDesc.enableDeformableVertexAttachments);
    read(tok.physxAutoDeformableAttachmentDeformableVertexOverlapOffset, autoAttachmentDesc.deformableVertexOverlapOffset);
    read(tok.physxAutoDeformableAttachmentEnableRigidSurfaceAttachments, autoAttachmentDesc.enableRigidSurfaceAttachments);
    read(tok.physxAutoDeformableAttachmentRigidSurfaceSamplingDistance, autoAttachmentDesc.rigidSurfaceSamplingDistance);
    read(tok.physxAutoDeformableAttachmentEnableCollisionFiltering, autoAttachmentDesc.enableCollisionFiltering);
    read(tok.physxAutoDeformableAttachmentCollisionFilteringOffset, autoAttachmentDesc.collisionFilteringOffset);
    read(tok.physxAutoDeformableAttachmentEnableDeformableFilteringPairs, autoAttachmentDesc.enableDeformableFilteringPairs);
}

void parseSingleTargetKeyPair(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey(&targetKeys)[2],
                              omni::physics::parse::ObjectKey key,
                              omni::physics::parse::TokenId rel0, omni::physics::parse::TokenId rel1)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    // Both targets are resolved only when both relationships are present;
    // a single-valued target resolves to its key, anything else to invalid.
    if (src->hasRelationship(key, rel0) && src->hasRelationship(key, rel1))
    {
        const auto resolveOne = [&](omni::physics::parse::TokenId rel) -> omni::physics::parse::ObjectKey {
            std::vector<omni::physics::parse::ObjectKey> targets;
            src->getRelationshipTargets(key, rel, targets);
            return targets.size() == 1 ? targets[0] : omni::physics::parse::ObjectKey{};
        };
        targetKeys[0] = resolveOne(rel0);
        targetKeys[1] = resolveOne(rel1);
    }
}

void parseAttachableKeys(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey(&attachableKeys)[2],
                         omni::physics::parse::ObjectKey autoAttachmentKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    parseSingleTargetKeyPair(attachedStage, attachableKeys, autoAttachmentKey,
        tok.physxAutoDeformableAttachmentAttachable0,
        tok.physxAutoDeformableAttachmentAttachable1);
}

omni::physx::usdparser::MeshKey loadMeshKey(usdparser::AttachedStage& attachedStage,
                                            omni::physics::parse::ObjectKey key,
                                            std::string_view crcTokenName)
{
    omni::physx::usdparser::MeshKey meshKey;
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return meshKey;
    const omni::physics::parse::TokenId crcToken = src->internToken(crcTokenName);
    std::vector<uint8_t> bytes;
    if (internal::getArrayValue(attachedStage, key, crcToken, omni::physics::parse::ReadTime::defaultTime(), bytes) &&
        bytes.size() == sizeof(meshKey))
    {
        std::memcpy(&meshKey, bytes.data(), sizeof(meshKey));
    }
    return meshKey;
}

// Writes the input CRC through IPhysicsDataWrite::writeByteArrayAttribute on the *authoring*
// sink (REQ-WRITE-AUTHORING-001), the same sink the generated payloads below go to: the CRC
// is the cache key for those payloads, so storing it anywhere they are not stored would
// declare a payload fresh that was never written. A null authoring sink (no backing stage at
// all) is a no-op, so the next updateAutoDeformableAttachment call always recomputes
// (inputCrc never matches a stored value), which is correct, just not cached.
void storeMeshKey(usdparser::AttachedStage& attachedStage,
                  omni::physics::parse::ObjectKey key,
                  std::string_view crcTokenName,
                  const omni::physx::usdparser::MeshKey& meshKey)
{
    if (omni::physics::parse::IPhysicsDataWrite* write = attachedStage.getAuthoringDataWrite())
    {
        write->writeByteArrayAttribute(
            key, crcTokenName, reinterpret_cast<const uint8_t*>(&meshKey), sizeof(meshKey));
    }
}

// Publishes the generated attachment point payloads through
// IPhysicsDataWrite::writeVtx*Attachment/writeElementCollisionFilter on the *authoring* sink
// (REQ-WRITE-AUTHORING-001), so an ovstage attach with a resident backing stage still authors
// into it -- the attachment sub-prims were created on that same stage. Only a build with no
// backing stage at all no-ops here; the generated data stays cached on the AttachedStage.
void publishVtxTetAttachment(usdparser::AttachedStage& attachedStage,
                             omni::physics::parse::ObjectKey attachmentKey,
                             const std::vector<int32_t>& vtxIndicesSrc0,
                             const std::vector<int32_t>& tetIndicesSrc1,
                             const std::vector<carb::Float3>& tetCoordsSrc1,
                             bool enabled)
{
    GeneratedDeformableAttachmentData data;
    data.kind = GeneratedDeformableAttachmentData::Kind::eVtxTet;
    data.enabled = enabled;
    data.vtxIndicesSrc0 = vtxIndicesSrc0;
    data.tetIndicesSrc1 = tetIndicesSrc1;
    data.tetCoordsSrc1 = tetCoordsSrc1;
    attachedStage.setGeneratedDeformableAttachmentData(attachmentKey, data);

    if (omni::physics::parse::IPhysicsDataWrite* write = attachedStage.getAuthoringDataWrite())
        write->writeVtxTetAttachment(attachmentKey, vtxIndicesSrc0, tetIndicesSrc1, tetCoordsSrc1, enabled);
}

void publishVtxXformAttachment(usdparser::AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey attachmentKey,
                               const std::vector<int32_t>& vtxIndicesSrc0,
                               const std::vector<carb::Float3>& localPositionsSrc1,
                               bool enabled)
{
    GeneratedDeformableAttachmentData data;
    data.kind = GeneratedDeformableAttachmentData::Kind::eVtxXform;
    data.enabled = enabled;
    data.vtxIndicesSrc0 = vtxIndicesSrc0;
    data.localPositionsSrc1 = localPositionsSrc1;
    attachedStage.setGeneratedDeformableAttachmentData(attachmentKey, data);

    if (omni::physics::parse::IPhysicsDataWrite* write = attachedStage.getAuthoringDataWrite())
        write->writeVtxXformAttachment(attachmentKey, vtxIndicesSrc0, localPositionsSrc1, enabled);
}

void publishElementCollisionFilter(usdparser::AttachedStage& attachedStage,
                                   omni::physics::parse::ObjectKey filterKey,
                                   const std::vector<uint32_t>& groupElemCounts0,
                                   const std::vector<uint32_t>& groupElemIndices0,
                                   const std::vector<uint32_t>& groupElemCounts1,
                                   const std::vector<uint32_t>& groupElemIndices1,
                                   bool enabled)
{
    GeneratedDeformableCollisionFilterData data;
    data.enabled = enabled;
    data.groupElemCounts0 = groupElemCounts0;
    data.groupElemIndices0 = groupElemIndices0;
    data.groupElemCounts1 = groupElemCounts1;
    data.groupElemIndices1 = groupElemIndices1;
    attachedStage.setGeneratedDeformableCollisionFilterData(filterKey, data);

    if (omni::physics::parse::IPhysicsDataWrite* write = attachedStage.getAuthoringDataWrite())
        write->writeElementCollisionFilter(
            filterKey, groupElemCounts0, groupElemIndices0, groupElemCounts1, groupElemIndices1, enabled);
}

PhysxShapeDesc* prepareScannedShapeForAttachment(usdparser::AttachedStage& attachedStage,
                                                 omni::physics::parse::ScannedStage& scanned,
                                                 std::string_view shapePathText)
{
    const omni::physics::parse::IPhysicsSource& scanSrc = scanned.source();
    PhysxShapeDesc* scanDesc = nullptr;
    for (auto& shape : scanned.shapes)
    {
        if (scanSrc.sourceKeyToString(shape->primKey) == shapePathText ||
            scanSrc.sourceKeyToString(shape->sourceGprim) == shapePathText)
        {
            scanDesc = shape.get();
            break;
        }
    }
    if (!scanDesc)
        return nullptr;

    usdparser::scan::dispatchScannedShapeCooking(attachedStage, scanned, scanDesc);

    if (scanDesc->rigidBody.valid())
        scanDesc->rigidBody = attachedStage.keyFor(scanSrc.sourceKeyToString(scanDesc->rigidBody));
    if (scanDesc->sourceGprim.valid())
        scanDesc->sourceGprim = attachedStage.keyFor(scanSrc.sourceKeyToString(scanDesc->sourceGprim));
    if (scanDesc->type == eConvexMeshShape)
    {
        auto* d = static_cast<ConvexMeshPhysxShapeDesc*>(scanDesc);
        if (d->meshPrimKey.valid())
            d->meshPrimKey = attachedStage.keyFor(scanSrc.sourceKeyToString(d->meshPrimKey));
    }
    else if (scanDesc->type == eTriangleMeshShape ||
             scanDesc->type == eConvexMeshDecompositionShape ||
             scanDesc->type == eSpherePointsShape)
    {
        auto* d = static_cast<TriangleMeshPhysxShapeDesc*>(scanDesc);
        if (d->meshPrimKey.valid())
            d->meshPrimKey = attachedStage.keyFor(scanSrc.sourceKeyToString(d->meshPrimKey));
    }
    return scanDesc;
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

bool parseTetMeshSurface(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey tetMeshKey,
                         std::vector<uint32_t>& surfaceTriVtxIndices, std::vector<uint32_t>& surfaceTriToTetMap)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);

    // need to query USD surfaceFaceVertexIndices, for consistency.
    std::vector<carb::Int3> surfaceFaceVertexIndices;
    internal::getArrayValue(attachedStage, tetMeshKey, tok.surfaceFaceVertexIndices,
                            omni::physics::parse::ReadTime::defaultTime(), surfaceFaceVertexIndices);

    if (surfaceFaceVertexIndices.size() == 0)
    {
        CARB_LOG_WARN("UsdGeomTetMesh is missing surface face vertex indices, %s.", attachedStage.textFor(tetMeshKey));
        return false;
    }

    surfaceTriVtxIndices.resize(surfaceFaceVertexIndices.size() * 3);
    std::memcpy(surfaceTriVtxIndices.data(), surfaceFaceVertexIndices.data(), surfaceTriVtxIndices.size()*sizeof(uint32_t));

    // generate surface to tet map
    std::vector<carb::Int4> tetVertexIndices;
    internal::getArrayValue(attachedStage, tetMeshKey, tok.tetVertexIndices,
                            omni::physics::parse::ReadTime::defaultTime(), tetVertexIndices);

    surfaceTriToTetMap.resize(surfaceFaceVertexIndices.size());
    bool success = mapSurfaceTrisToTets(surfaceTriToTetMap.data(),
                                        (uint32_t*)surfaceFaceVertexIndices.data(),
                                        uint32_t(surfaceFaceVertexIndices.size()),
                                        (uint32_t*)tetVertexIndices.data(),
                                        uint32_t(tetVertexIndices.size()));

    if (!success)
    {
        CARB_LOG_WARN("UsdGeomTetMesh, failed to map surface faces to tets, %s.", attachedStage.textFor(tetMeshKey));
        return false;
    }

    return true;
}

void getColliders(usdparser::AttachedStage& attachedStage, std::vector<omni::physics::parse::ObjectKey>& colliders, omni::physics::parse::ObjectKey rootKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(rootKey))
        return;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    const omni::physics::parse::TokenId deformableBodyTok = tok.omniphysicsDeformableBodyAPI;
    const omni::physics::parse::TokenId collisionTok = tok.physicsCollisionAPI;

    // Active subtree, descending through instance proxies (matches the
    // legacy UsdTraverseInstanceProxies walk). Returning true prunes.
    src->forEachDescendantPruned(rootKey,
        [&](omni::physics::parse::ObjectKey key) -> bool
        {
            if (key == rootKey)
            {
                if (src->hasSchema(key, collisionTok))
                {
                    colliders.push_back(key);
                    return true;
                }
                return false; // descend into children
            }
            // Prune at a reset-xform-stack boundary (new transform space) or
            // at a nested deformable body; collect collider subtrees.
            omni::physics::parse::Matrix4d m{};
            bool resetsXformStack = false;
            src->getLocalTransform(key, omni::physics::parse::ReadTime::defaultTime(), m, resetsXformStack);
            if (resetsXformStack)
                return true;
            if (src->hasSchema(key, deformableBodyTok))
                return true;
            if (src->hasSchema(key, collisionTok))
            {
                colliders.push_back(key);
                return true;
            }
            return false;
        },
        omni::physics::parse::DescendantScope::eActiveInstanced);
}

// Visits the Attachment / ElementCollisionFilter children of `type` under an auto-attachment
// prim with (childKey, src0, src1). An in-memory layout (attach that cannot author sub-prims)
// takes precedence over the source's children, which is what lets the update path below run
// unchanged on both.
template <typename Fn>
void forEachAutoAttachmentChild(usdparser::AttachedStage& attachedStage,
                                omni::physics::parse::ObjectKey autoAttachmentKey,
                                omni::physics::parse::ObjectType type,
                                omni::physics::parse::TokenId typeToken,
                                Fn&& fn)
{
    if (const GeneratedAutoAttachmentLayout* layout = attachedStage.getGeneratedAutoAttachmentLayout(autoAttachmentKey))
    {
        for (const GeneratedAutoAttachmentChild& child : layout->children)
        {
            if (child.type == type)
                fn(child.key, child.src0, child.src1);
        }
        return;
    }

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    src->forEachChild(autoAttachmentKey,
        [&](omni::physics::parse::ObjectKey childKey)
        {
            if (!src->isA(childKey, typeToken))
                return;
            omni::physics::parse::ObjectKey srcKeys[2];
            parseSingleTargetKeyPair(attachedStage, srcKeys, childKey, tok.omniphysicsSrc0, tok.omniphysicsSrc1);
            fn(childKey, srcKeys[0], srcKeys[1]);
        });
}

bool getVtxXformAttachment(usdparser::AttachedStage& attachedStage,
    omni::physics::parse::ObjectKey& attachmentKey,
    omni::physics::parse::ObjectKey autoAttachmentKey,
    omni::physics::parse::ObjectKey simMeshKey, omni::physics::parse::ObjectKey xformableKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    // Only the FIRST VtxXformAttachment child is considered (match or not).
    bool done = false, result = false;
    forEachAutoAttachmentChild(attachedStage, autoAttachmentKey, eAttachmentVtxXform, tok.OmniPhysicsVtxXformAttachment,
        [&](omni::physics::parse::ObjectKey childKey, omni::physics::parse::ObjectKey src0, omni::physics::parse::ObjectKey src1)
        {
            if (done)
                return;
            done = true;
            attachmentKey = childKey;
            result = src0 == simMeshKey && src1 == xformableKey;
        });
    return result;
}

bool getVtxTetAttachment(usdparser::AttachedStage& attachedStage,
    omni::physics::parse::ObjectKey& attachmentKey,
    omni::physics::parse::ObjectKey autoAttachmentKey,
    omni::physics::parse::ObjectKey vtxSimMeshKey, omni::physics::parse::ObjectKey tetSimMeshKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    // All VtxTetAttachment children are scanned, updating attachmentKey
    // each time; returns true on the first whose targets match.
    bool result = false;
    forEachAutoAttachmentChild(attachedStage, autoAttachmentKey, eAttachmentVtxTet, tok.OmniPhysicsVtxTetAttachment,
        [&](omni::physics::parse::ObjectKey childKey, omni::physics::parse::ObjectKey src0, omni::physics::parse::ObjectKey src1)
        {
            if (result)
                return;
            attachmentKey = childKey;
            if (src0 == vtxSimMeshKey && src1 == tetSimMeshKey)
                result = true;
        });
    return result;
}

bool getElementCollisionFilters(usdparser::AttachedStage& attachedStage,
    std::vector<omni::physics::parse::ObjectKey>& filterKeys,
    omni::physics::parse::ObjectKey autoAttachmentKey,
    std::vector<omni::physics::parse::ObjectKey>& rigidColliders, omni::physics::parse::ObjectKey deformableColliderKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    std::vector<omni::physics::parse::ObjectKey> srcKeys0, srcKeys1;
    forEachAutoAttachmentChild(attachedStage, autoAttachmentKey, eDeformableCollisionFilter, tok.OmniPhysicsElementCollisionFilter,
        [&](omni::physics::parse::ObjectKey childKey, omni::physics::parse::ObjectKey src0, omni::physics::parse::ObjectKey src1)
        {
            filterKeys.push_back(childKey);
            srcKeys0.push_back(src0);
            srcKeys1.push_back(src1);
        });

    if (filterKeys.size() != rigidColliders.size())
    {
        return false;
    }
    for (size_t f = 0; f < filterKeys.size(); ++f)
    {
        if (srcKeys0[f] != deformableColliderKey || srcKeys1[f] != rigidColliders[f])
        {
            return false;
        }
    }
    return true;
}

bool getElementCollisionFilter(usdparser::AttachedStage& attachedStage,
    omni::physics::parse::ObjectKey& filterKey,
    omni::physics::parse::ObjectKey autoAttachmentKey,
    omni::physics::parse::ObjectKey deformableCollider0Key, omni::physics::parse::ObjectKey deformableCollider1Key)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    // Only the FIRST filter child is considered; no child at all is not a mismatch.
    bool done = false, result = true;
    forEachAutoAttachmentChild(attachedStage, autoAttachmentKey, eDeformableCollisionFilter, tok.OmniPhysicsElementCollisionFilter,
        [&](omni::physics::parse::ObjectKey childKey, omni::physics::parse::ObjectKey src0, omni::physics::parse::ObjectKey src1)
        {
            if (done)
                return;
            done = true;
            filterKey = childKey;
            result = src0 == deformableCollider0Key && src1 == deformableCollider1Key;
        });
    return result;
}

carb::Float3 convertBary(carb::Float4& bary)
{
    return carb::Float3{ bary.x, bary.y, bary.z };
}

// Average non-degenerate edge length of the world-space AABB of a point set.
// `collPositions` already holds the collision mesh's points in world space
// (see getDeformableMeshInfo), so the AABB of those points is the mesh's
// world bound for a leaf boundable -- exactly what the attachment math consumes.
float getAverageDim(const std::vector<carb::Float3>& worldPositions)
{
    if (worldPositions.empty())
        return 0.0f;
    carb::Float3 lo = worldPositions[0];
    carb::Float3 hi = worldPositions[0];
    for (const carb::Float3& p : worldPositions)
    {
        lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y); lo.z = std::min(lo.z, p.z);
        hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y); hi.z = std::max(hi.z, p.z);
    }
    const double size[3] = { double(hi.x) - lo.x, double(hi.y) - lo.y, double(hi.z) - lo.z };
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
void sortFilterGroups(std::vector<uint32_t>& dstCounts, std::vector<uint32_t>& dstIndices,
    std::vector<uint32_t>& srcCounts, std::vector<uint32_t>& srcIndices)
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
    std::vector<uint32_t>& dstCountsA, std::vector<uint32_t>& dstIndicesA,
    std::vector<uint32_t>& dstCountsB, std::vector<uint32_t>& dstIndicesB,
    const std::vector<uint32_t>& srcCountsA, const std::vector<uint32_t>& srcIndicesA,
    const std::vector<uint32_t>& srcCountsB, const std::vector<uint32_t>& srcIndicesB)
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
    std::vector<uint32_t>& filterGroupCountsA, std::vector<uint32_t>& filterGroupIndicesA,
    std::vector<uint32_t>& filterGroupCountsB, std::vector<uint32_t>& filterGroupIndicesB)
{
    if (filterGroupCountsA.size() == 0 || filterGroupCountsA.size() != filterGroupCountsB.size())
    {
        filterGroupCountsA.clear();
        filterGroupIndicesA.clear();
        filterGroupCountsB.clear();
        filterGroupIndicesB.clear();
        return;
    }

    std::vector<uint32_t> tmpFilterGroupCounts[2];
    std::vector<uint32_t> tmpFilterGroupIndices[2];
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

void convertTetGroupsToSurfaceTriGroups(std::vector<uint32_t>& triGroupCounts, std::vector<uint32_t>& triGroupIndices,
        const std::vector<uint32_t>& tetVtxIndices, const std::vector<uint32_t>& surfaceTriToTetMap,
        const std::vector<uint32_t>& tetGroupCounts, const std::vector<uint32_t>& tetGroupIndices)
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

void convertVtxGroupsToTriGroups(std::vector<uint32_t>& triGroupCounts, std::vector<uint32_t>& triGroupIndices,
    const std::vector<carb::Float3>& points, const std::vector<uint32_t>& triVtxIndices,
    const std::vector<uint32_t>& vtxGroupCounts, const std::vector<uint32_t>& vtxGroupIndices)
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
    std::vector<uint32_t>& groupCountsA, std::vector<uint32_t>& groupIndicesA,
    std::vector<uint32_t>& groupCountsB, std::vector<uint32_t>& groupIndicesB,
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
    std::vector<uint32_t>& groupCountsA, std::vector<uint32_t>& groupIndicesA,
    std::vector<uint32_t>& groupCountsB, std::vector<uint32_t>& groupIndicesB,
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
    std::vector<int32_t>& attachmentVtxIndices,
    std::vector<int32_t>& attachmentTetIndices,
    std::vector<carb::Float3>& attachmentTetCoords,
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

void checkNonUniformScale(const PxVec3& scale, const char* primPathText)
{
    const float tolerance = 1e-4f;
    if (fabsf(scale[0] - scale[1]) > tolerance || fabsf(scale[0] - scale[2]) > tolerance ||
        fabsf(scale[2] - scale[1]) > tolerance)
    {
        CARB_LOG_WARN("Non-uniform scale may result in a non matching attachment shape representation: %s", primPathText);
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
    std::vector<int32_t>& attachmentVtxIndicesDeformable;
    std::vector<carb::Float3>& attachmentVtxPointsXformable;
    std::vector<uint32_t>& filterTriIndicesDeformable;
    const DeformableMeshInfo& deformableMeshInfo;
    const PxMat44d& worldToRigid;
    const PhysxAutoAttachmentDesc& desc;
    const MaskShapes& maskShapes;
};

} // namespace

void updateDeformableRigidColliderAttachments(const PxGeometry& geom, const PxTransform& geomPose, void* userData)
{
    const UserDataInfo* info = (const UserDataInfo*)userData;

    std::vector<int32_t>& attachmentVtxIndicesDeformable = info->attachmentVtxIndicesDeformable;
    std::vector<carb::Float3>& attachmentVtxPointsXformable = info->attachmentVtxPointsXformable;
    std::vector<uint32_t>& filterGroupIndices = info->filterTriIndicesDeformable;

    const DeformableMeshInfo& deformableMeshInfo = info->deformableMeshInfo;
    const PxMat44d& worldToRigid = info->worldToRigid;
    const PhysxAutoAttachmentDesc& desc = info->desc;

    // Use the minimum average dimension
    float avg_dim0 = getAverageDim(deformableMeshInfo.collPositions);
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
                attachmentPointsDeformable.push_back(carb::Float3{ localSample.x, localSample.y, localSample.z });

                const PxVec3 actorPos = transform.transformInv(toPhysX(culledSamples[i])).multiply(invScale);
                attachmentPointsRigidBody.push_back(carb::Float3{ actorPos.x, actorPos.y, actorPos.z });
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

            const PxVec3d rigidPos = worldToRigid.transform(PxVec3d(vertexPos.x, vertexPos.y, vertexPos.z));
            attachmentVtxPointsXformable.push_back(carb::Float3{ float(rigidPos.x), float(rigidPos.y), float(rigidPos.z) });
        }
    }

    // Filtering
    if (desc.enableCollisionFiltering)
    {
        std::vector<carb::Float3> culledVertices;
        std::vector<uint32_t> culledVertexIndices;
        cullPointsToMaskShapes(culledVertices, culledVertexIndices, info->maskShapes, collisionFilteringOffset,
            deformableMeshInfo.collPositions.data(), uint32_t(deformableMeshInfo.collPositions.size()));

        std::vector<uint32_t> vtxGroupIndices;
        std::vector<uint32_t> vtxGroupCounts;
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

        std::vector<uint32_t> filterGroupCounts;
        convertVtxGroupsToTriGroups(filterGroupCounts, filterGroupIndices, *points, *triVtxIndices,
            vtxGroupCounts, vtxGroupIndices);

        CARB_ASSERT(filterGroupCounts.size() == 1);
        std::vector<uint32_t> filterGroupCountsRigid = { 0 };
        std::vector<uint32_t> filterGroupIndicesRigid;
        compressFilterGroups(filterGroupCounts, filterGroupIndices, filterGroupCountsRigid, filterGroupIndicesRigid);
    }

    PX_DELETE(triMeshSampler);
}

bool parseMaskShape(usdparser::AttachedStage& attachedStage, PxGeometryHolder& geometryHolder, PxTransform& transform, omni::physics::parse::ObjectKey key)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    const bool isSphere  = src->isA(key, tok.sphereType);
    const bool isCapsule = src->isA(key, tok.capsuleType);
    const bool isCube    = src->isA(key, tok.cubeType);

    const PxMat44d shapeMat = internal::getWorldTransform(attachedStage, key, omni::physics::parse::ReadTime::defaultTime());
    // Bit-exact Gf decomposition, not the PhysX-native decomposeMatrix: this
    // feeds calculateAutoAttachmentCRC (transforms/radius/halfHeight hashed
    // byte-for-byte into physxAutoDeformableAttachment:inputCrc), and a
    // sheared mask-shape ancestor makes the two decompositions disagree (see
    // MatrixTools.h) -- switching decompositions here would invalidate every
    // persisted attachment CRC and regenerate different attachment data on
    // sheared inputs. This keeps the pre-port CRC and attachment data intact.
    const omni::physx::gfmath::PivotTransform shapeXf = omni::physx::gfmath::decomposeWithPivot(shapeMat);
    const PxQuatd shapeRotD = omni::physx::gfmath::getQuat(shapeXf.rotation);
    transform = PxTransform(PxVec3(float(shapeXf.translation.x), float(shapeXf.translation.y), float(shapeXf.translation.z)),
                            PxQuat(float(shapeRotD.x), float(shapeRotD.y), float(shapeRotD.z), float(shapeRotD.w)));
    PxVec3 shapeScale(float(shapeXf.scale.x), float(shapeXf.scale.y), float(shapeXf.scale.z));

    if (isSphere)
    {
        float radius = 1.0f;

        {
            // as we dont support scale in physics and scale can be non uniform
            // we pick the largest scale value as the sphere radius base
            checkNonUniformScale(shapeScale, attachedStage.textFor(key));
            radius = fmaxf(fmaxf(fabsf(shapeScale[1]), fabsf(shapeScale[0])), fabsf(shapeScale[2]));
        }

        {
            double radiusAttr = 1.0;
            internal::getValue<double>(attachedStage, key, tok.radius, omni::physics::parse::ReadTime::defaultTime(), radiusAttr);
            radius *= (float)radiusAttr;
        }

        geometryHolder = PxSphereGeometry(fabsf(radius));
        return true;
    }

    if (isCapsule)
    {
        float radius = 1.0f;
        float halfHeight = 1.0f;
        omni::physics::parse::TokenId axis = tok.x;

        {
            double radiusAttr = 0.5;
            internal::getValue<double>(attachedStage, key, tok.radius, omni::physics::parse::ReadTime::defaultTime(), radiusAttr);
            double heightAttr = 2.0;
            internal::getValue<double>(attachedStage, key, tok.height, omni::physics::parse::ReadTime::defaultTime(), heightAttr);
            radius = (float)radiusAttr;
            halfHeight = (float)heightAttr * 0.5f;

            internal::getValue(attachedStage, key, tok.axis, omni::physics::parse::ReadTime::defaultTime(), axis);
        }

        {
            // scale the radius and height based on the given axis token
            checkNonUniformScale(shapeScale, attachedStage.textFor(key));
            if (axis == tok.x)
            {
                halfHeight *= shapeScale[0];
                radius *= fmaxf(fabsf(shapeScale[1]), fabsf(shapeScale[2]));
            }
            else if (axis == tok.y)
            {
                halfHeight *= shapeScale[1];
                radius *= fmaxf(fabsf(shapeScale[0]), fabsf(shapeScale[2]));
            }
            else
            {
                halfHeight *= shapeScale[2];
                radius *= fmaxf(fabsf(shapeScale[1]), fabsf(shapeScale[0]));
            }
        }

        geometryHolder = PxCapsuleGeometry(radius, halfHeight);

        const float hRt2 = sqrt(2.0f) / 2.0f;
        PxQuat fixupQ(PxIdentity);

        if (axis == tok.y)
        {
            fixupQ = PxQuat(hRt2, -hRt2, 0.0f, 0.0f);
        }
        else if (axis == tok.z)
        {
            fixupQ = PxQuat(hRt2, 0.0f, -hRt2, 0.0f);
        }

        transform.q = transform.q * fixupQ;
        return true;
    }

    if (isCube)
    {
        PxVec3 halfExtents;

        {
            // scale is taken, its a part of the cube size, as the physics does not support scale
            halfExtents = shapeScale;
        }

        {
            double sizeAttr = 2.0;
            internal::getValue<double>(attachedStage, key, tok.size, omni::physics::parse::ReadTime::defaultTime(), sizeAttr);
            sizeAttr = abs(sizeAttr) * 0.5f; // convert cube edge length to half extend
            halfExtents *= (float)sizeAttr;
        }

        geometryHolder = PxBoxGeometry(halfExtents);
        return true;
    }

    return false;
}

void processRigidShapeGeometry(usdparser::AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey rigidColliderKey,
                               const usdparser::PhysxShapeDesc* desc,
                               getGeometryInfoCallback callbackFn,
                               void* userData)
{
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
        PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(*convexDesc, rigidColliderKey, attachedStage, false);
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
        std::vector<PxConvexMesh*> convexMeshes = cookingDataAsync->getConvexMeshDecomposition(*convexDecompositionDesc, rigidColliderKey, attachedStage, false);
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
        PxTriangleMesh* triMesh = cookingDataAsync->getTriangleMesh(*meshDesc, rigidColliderKey, attachedStage, false);
        if (triMesh)
        {
            PxTriangleMeshGeometry triangleMeshGeom(triMesh, toPhysX(meshDesc->meshScale));
            callbackFn(triangleMeshGeom, transform, userData);
        }
    }
    break;
    }
}

void updateDeformableVolumeSurfaceAttachments(usdparser::AttachedStage& attachedStage,
    omni::physics::parse::ObjectKey vtxTetAttachmentKey,
    omni::physics::parse::ObjectKey filterKey,
    const DeformableMeshInfo& volumeMeshInfo, const uint32_t volumeSlot,
    const DeformableMeshInfo& surfaceMeshInfo, const uint32_t surfaceSlot,
    const PhysxAutoAttachmentDesc& desc, const MaskShapes& maskShapes)
{
    // Use the minimum average dimension
    float avg_dimV = getAverageDim(volumeMeshInfo.collPositions);
    float avg_dimS = getAverageDim(surfaceMeshInfo.collPositions);
    float avg_dim = PxMin(avg_dimV, avg_dimS);
    float default_rad = avg_dim * 0.05f;

    // Apply default heuristics
    float collisionFilteringOffset = desc.collisionFilteringOffset;
    if (!isfinite(collisionFilteringOffset))
        collisionFilteringOffset = default_rad * 2;

    std::vector<int32_t> attachmentVtxIndices;
    std::vector<carb::Float3> attachmentTetCoords;
    std::vector<int32_t> attachmentTetIndices;
    std::vector<uint32_t> filterGroupCounts[2];
    std::vector<uint32_t> filterGroupIndices[2];

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
        if (volumeMeshInfo.simMeshKey != volumeMeshInfo.collMeshKey)
        {
            tetFinderColl = omni::tetfinder::createTetFinder(
                volumeMeshInfo.collPositions.data(), uint32_t(volumeMeshInfo.collPositions.size()),
                volumeMeshInfo.collIndices.data(), uint32_t(volumeMeshInfo.collIndices.size()));
        }

        std::vector<uint32_t> tetFilterGroupCounts;
        std::vector<uint32_t> tetFilterGroupIndices;
        std::vector<uint32_t> triFilterGroupCounts;
        std::vector<uint32_t> triFilterGroupIndices;

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

    publishVtxTetAttachment(attachedStage, vtxTetAttachmentKey,
        attachmentVtxIndices, attachmentTetIndices, attachmentTetCoords, desc.enableDeformableVertexAttachments);

    if (filterKey.valid() && filterGroupCounts[volumeSlot].size() > 0)
    {
        publishElementCollisionFilter(attachedStage, filterKey,
            filterGroupCounts[volumeSlot], filterGroupIndices[volumeSlot],
            filterGroupCounts[surfaceSlot], filterGroupIndices[surfaceSlot],
            desc.enableCollisionFiltering);
    }
}

void updateDeformableVolumeVolumeAttachments(usdparser::AttachedStage& attachedStage,
    omni::physics::parse::ObjectKey (&vtxTetAttachmentKeys)[2],
    omni::physics::parse::ObjectKey filterKey,
    const DeformableMeshInfo(&deformableMeshInfo)[2], const uint32_t(&slots)[2], const PhysxAutoAttachmentDesc& desc, const MaskShapes& maskShapes)
{
    // Use the minimum average dimension
    float avg_dim0 = getAverageDim(deformableMeshInfo[0].collPositions);
    float avg_dim1 = getAverageDim(deformableMeshInfo[1].collPositions);
    float avg_dim = PxMin(avg_dim0, avg_dim1);
    float default_rad = avg_dim * 0.05f;

    // Apply default heuristics
    float collisionFilteringOffset = desc.collisionFilteringOffset;
    if (!isfinite(collisionFilteringOffset))
        collisionFilteringOffset = default_rad * 2;

    std::vector<int32_t> attachmentVtxIndices[2];
    std::vector<carb::Float3> attachmentTetCoords[2];
    std::vector<int32_t> attachmentTetIndices[2];
    std::vector<uint32_t> filterGroupCounts[2];
    std::vector<uint32_t> filterGroupIndices[2];

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
            if (deformableMeshInfo[s].simMeshKey != deformableMeshInfo[s].collMeshKey)
            {
                tetFinderColl[s] = omni::tetfinder::createTetFinder(
                    &deformableMeshInfo[s].collPositions[0], uint32_t(deformableMeshInfo[s].collPositions.size()),
                    &deformableMeshInfo[s].collIndices[0], uint32_t(deformableMeshInfo[s].collIndices.size()));
            }
        }

        std::vector<uint32_t> tetFilterGroupCounts[2];
        std::vector<uint32_t> tetFilterGroupIndices[2];

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

        publishVtxTetAttachment(attachedStage, vtxTetAttachmentKeys[s],
            attachmentVtxIndices[s], attachmentTetIndices[s], attachmentTetCoords[s], desc.enableDeformableVertexAttachments);
    }

    if (filterKey.valid() && filterGroupCounts[slots[0]].size() > 0)
    {
        publishElementCollisionFilter(attachedStage, filterKey,
            filterGroupCounts[slots[0]], filterGroupIndices[slots[0]],
            filterGroupCounts[slots[1]], filterGroupIndices[slots[1]],
            desc.enableCollisionFiltering);
    }
}

void updateDeformableXformableAttachments(
    usdparser::AttachedStage& attachedStage,
    omni::physics::parse::ObjectKey vtxXformAttachmentKey,
    std::vector<omni::physics::parse::ObjectKey>& filterKeys,
    const DeformableMeshInfo& deformableMeshInfo,
    const uint32_t deformableSlot,
    omni::physics::parse::ObjectKey rigidRootKey,
    const std::vector<omni::physics::parse::ObjectKey>& rigidColliders,
    const PhysxAutoAttachmentDesc& desc, const MaskShapes& maskShapes)
{
    //we need to define all attachment local positions relative to
    //the same frame, because we don't want to have an attachment per
    //collider. we assume all colliders move in the same frame if they move.
    const PxMat44d rigidToWorld(internal::getWorldTransform(attachedStage, rigidRootKey, omni::physics::parse::ReadTime::defaultTime()));
    const PxMat44d worldToRigid = omni::physx::affineInverse(rigidToWorld);

    std::vector<int32_t> attachmentVtxIndicesDeformable;
    std::vector<carb::Float3> attachmentVtxPointsXformable;
    std::vector<uint32_t> filterTriIndicesDeformable;

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

                const PxVec3d rigidPos = worldToRigid.transform(PxVec3d(vertexPos.x, vertexPos.y, vertexPos.z));
                attachmentVtxPointsXformable.push_back(carb::Float3{ float(rigidPos.x), float(rigidPos.y), float(rigidPos.z) });
            }
        }
    }
    else
    {
        for (size_t i = 0; i < rigidColliders.size(); ++i)
        {
            const omni::physics::parse::ObjectKey rigidColliderKey = rigidColliders[i];

            filterTriIndicesDeformable.clear();

            UserDataInfo userData
            {
                attachmentVtxIndicesDeformable, attachmentVtxPointsXformable, filterTriIndicesDeformable,
                deformableMeshInfo, worldToRigid, desc, maskShapes
            };

            const PxMat44d mat = internal::getWorldTransform(attachedStage, rigidColliderKey, omni::physics::parse::ReadTime::defaultTime());
            // Bit-exact Gf decomposition, not the PhysX-native decomposeMatrix:
            // this feeds shapeDesc->localPos/localRot/localScale below, which
            // drives rigid-surface attachment sampling in
            // processRigidShapeGeometry. Not hashed into a CRC, but a sheared
            // rigid-collider world transform would otherwise move the sampled
            // attachment points silently -- keep it numerically unchanged from
            // the pre-port behaviour, same as the CRC-hashed decomposition above.
            const omni::physx::gfmath::PivotTransform poseXf = omni::physx::gfmath::decomposeWithPivot(mat);
            const PxQuatd poseRotD = omni::physx::gfmath::getQuat(poseXf.rotation);
            const PxTransform pose(PxVec3(float(poseXf.translation.x), float(poseXf.translation.y), float(poseXf.translation.z)),
                                   PxQuat(float(poseRotD.x), float(poseRotD.y), float(poseRotD.z), float(poseRotD.w)));
            const PxVec3 scale(float(poseXf.scale.x), float(poseXf.scale.y), float(poseXf.scale.z));

            const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (src)
                tok.intern(*src);
            if (!(src && src->isA(rigidColliderKey, tok.gprimType)))
                continue;

            const std::string rigidColliderPathText(attachedStage.textViewFor(rigidColliderKey));
            static const std::vector<std::string> kNoExclude;
            omni::physics::parse::ScanOptions scanOptions;
            scanOptions.descendantScope = omni::physics::parse::DescendantScope::eActive;
            const std::vector<std::string> scanRoots{ rigidColliderPathText };
            omni::physics::parse::ScannedStage scanned = omni::physics::parse::scanStage(
                attachedStage.attachTarget(), scanRoots, kNoExclude, scanOptions,
                omni::physx::usdparser::iceDescriptorAllocator());
            usdparser::PhysxShapeDesc* shapeDesc = prepareScannedShapeForAttachment(attachedStage, scanned, rigidColliderPathText);
            if (!shapeDesc)
                continue;

            shapeDesc->localPos = { pose.p.x, pose.p.y, pose.p.z };
            shapeDesc->localRot = { pose.q.x, pose.q.y, pose.q.z, pose.q.w };
            shapeDesc->localScale = { scale.x, scale.y, scale.z };

            processRigidShapeGeometry(attachedStage, rigidColliderKey, shapeDesc, updateDeformableRigidColliderAttachments, &userData);

            const omni::physics::parse::ObjectKey filterKey = filterKeys[i];
            if (filterKey.valid() && filterTriIndicesDeformable.size() > 0)
            {
                const std::vector<uint32_t> filterCount({ uint32_t(filterTriIndicesDeformable.size()) });
                const std::vector<uint32_t> empty;
                if (deformableSlot == 0)
                    publishElementCollisionFilter(attachedStage, filterKey,
                        filterCount, filterTriIndicesDeformable, empty, empty, desc.enableCollisionFiltering);
                else
                    publishElementCollisionFilter(attachedStage, filterKey,
                        empty, empty, filterCount, filterTriIndicesDeformable, desc.enableCollisionFiltering);
            }
        }
    }

    publishVtxXformAttachment(attachedStage, vtxXformAttachmentKey,
        attachmentVtxIndicesDeformable, attachmentVtxPointsXformable, desc.enableDeformableVertexAttachments);
}

bool getDeformableMeshInfo(usdparser::AttachedStage& attachedStage,
                           DeformableMeshInfo& deformableMeshInfo,
                           omni::physics::parse::ObjectKey deformableBodyKey,
                           const PhysxDeformableBodyDesc& deformableDesc)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    // deformableDesc.simMeshKey/collisionMeshKey are ObjectKeys (ADR-0019 increment 7): used
    // directly below wherever a key is accepted, including DeformableMeshInfo's own
    // ObjectKey-typed simMeshKey/collMeshKey fields and parseTetMeshSurface (both retyped off
    // SdfPath in the USD-removal pass that added this comment's follow-up).
    const omni::physics::parse::ObjectKey simMeshKey = deformableDesc.simMeshKey;
    if (!src->isA(simMeshKey, tok.pointBasedType))
    {
        return false;
    }

    const PxMat44d simToWorld(internal::getWorldTransform(attachedStage, simMeshKey, omni::physics::parse::ReadTime::defaultTime()));

    std::vector<carb::Float3> simMeshPoints;
    internal::getArrayValue(attachedStage, simMeshKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), simMeshPoints);

    std::vector<uint32_t> simIndices;
    if (deformableDesc.type == ObjectType::eVolumeDeformableBody)
    {
        // isTetMeshLike, not isA(UsdGeomTetMesh): ovstage reports a UsdGeomTetMesh as plain "Mesh"
        // (its populator has no TetMesh mapping), so the concrete-type gate made
        // getDeformableMeshInfo() return false and silently stopped volume auto-attachment /
        // filter computation for every non-USD source. See PhysXTools.h::isTetMeshLike.
        if (!internal::isTetMeshLike(attachedStage, simMeshKey))
        {
            return false;
        }

        std::vector<carb::Int4> tmpIndices;
        internal::getArrayValue(attachedStage, simMeshKey, tok.tetVertexIndices, omni::physics::parse::ReadTime::defaultTime(), tmpIndices);

        PX_COMPILE_TIME_ASSERT(sizeof(carb::Int4) == sizeof(uint32_t) * 4);
        simIndices.resize(tmpIndices.size() * 4);
        std::memcpy(simIndices.data(), tmpIndices.data(), simIndices.size() * sizeof(uint32_t));

        deformableMeshInfo.type = AttachmentActorType::eVOLUME_DEFORMABLE;
    }
    else if (deformableDesc.type == ObjectType::eSurfaceDeformableBody)
    {
        if (!src->isA(simMeshKey, tok.meshType))
        {
            return false;
        }

        std::vector<int32_t> tmpIndices;
        internal::getArrayValue(attachedStage, simMeshKey, tok.faceVertexIndices, omni::physics::parse::ReadTime::defaultTime(), tmpIndices);
        simIndices.resize(tmpIndices.size());
        std::memcpy(simIndices.data(), tmpIndices.data(), simIndices.size() * sizeof(uint32_t));

        deformableMeshInfo.type = AttachmentActorType::eSURFACE_DEFORMABLE;
    }

    deformableMeshInfo.simPositions.resize(simMeshPoints.size());
    for (size_t i = 0; i < deformableMeshInfo.simPositions.size(); ++i)
    {
        const carb::Float3& p = simMeshPoints[i];
        const PxVec3d position = simToWorld.transform(PxVec3d(p.x, p.y, p.z));
        deformableMeshInfo.simPositions[i] = { float(position.x), float(position.y), float(position.z) };
    }

    deformableMeshInfo.simIndices.swap(simIndices);
    deformableMeshInfo.simMeshKey = simMeshKey;

    if (deformableDesc.type == ObjectType::eVolumeDeformableBody)
    {
        // could take some shortcuts here, if collision mesh equals simulation mesh.
        // however, since for filtering we use tet mesh surface triangles, as opposed to tets we
        // treat the collision mesh separately anyways.
        // isTetMeshLike, not isA(UsdGeomTetMesh) — same reason as the sim-mesh gate above.
        const omni::physics::parse::ObjectKey collisionMeshKey = deformableDesc.collisionMeshKey;
        if (!internal::isTetMeshLike(attachedStage, collisionMeshKey))
        {
            return false;
        }

        const PxMat44d collToWorld(internal::getWorldTransform(attachedStage, collisionMeshKey, omni::physics::parse::ReadTime::defaultTime()));

        std::vector<carb::Float3> collMeshPoints;
        internal::getArrayValue(attachedStage, collisionMeshKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), collMeshPoints);

        std::vector<carb::Int4> vtxTetIndices;
        internal::getArrayValue(attachedStage, collisionMeshKey, tok.tetVertexIndices, omni::physics::parse::ReadTime::defaultTime(), vtxTetIndices);
        std::vector<uint32_t> collIndices(vtxTetIndices.size()*4);
        std::memcpy(collIndices.data(), vtxTetIndices.data(), sizeof(uint32_t) * collIndices.size());

        std::vector<uint32_t> collSurfaceTriToTetMap;
        std::vector<uint32_t> collSurfaceTriIndices;
        {
            bool hasSurface = parseTetMeshSurface(attachedStage, collisionMeshKey, collSurfaceTriIndices, collSurfaceTriToTetMap);
            if (!hasSurface)
                return false;
        }

        deformableMeshInfo.collPositions.resize(collMeshPoints.size());
        for (size_t i = 0; i < deformableMeshInfo.collPositions.size(); ++i)
        {
            const carb::Float3& p = collMeshPoints[i];
            const PxVec3d position = collToWorld.transform(PxVec3d(p.x, p.y, p.z));
            deformableMeshInfo.collPositions[i] = { float(position.x), float(position.y), float(position.z) };
        }

        deformableMeshInfo.collIndices.swap(collIndices);
        deformableMeshInfo.collSurfaceTriIndices.swap(collSurfaceTriIndices);
        deformableMeshInfo.collSurfaceTriToTetMap.swap(collSurfaceTriToTetMap);
        deformableMeshInfo.collMeshKey = collisionMeshKey;
    }
    else if (deformableDesc.type == ObjectType::eSurfaceDeformableBody)
    {
        deformableMeshInfo.collPositions.assign(deformableMeshInfo.simPositions.begin(), deformableMeshInfo.simPositions.end());
        deformableMeshInfo.collIndices.assign(deformableMeshInfo.simIndices.begin(), deformableMeshInfo.simIndices.end());
        deformableMeshInfo.collMeshKey = simMeshKey;
    }
    else
    {
        return false;
    }

    deformableMeshInfo.deformableBodyDataCrc = loadMeshKey(attachedStage, deformableBodyKey, deformableBodyDataCrcToken);

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

bool parseAttachables(usdparser::AttachedStage& attachedStage,
    AttachmentActorType::Enum(&types)[2],
    omni::physics::parse::ObjectKey(&attachableKeys)[2],
    DeformableDescRef(&deformableDescs)[2],
    uint32_t(&attachableSlots)[2],
    uint32_t& numDeformables,
    uint32_t& numXformables,
    std::vector<omni::physics::parse::ObjectKey>& rigidColliders,
    omni::physics::parse::ObjectKey autoAttachmentKey,
    CookingDataAsync& cookingDataAsync)
{
    parseAttachableKeys(attachedStage, attachableKeys, autoAttachmentKey);

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    omni::physics::parse::KnownTokens tok;
    if (src)
        tok.intern(*src);
    const omni::physics::parse::TokenId deformableBodyTok =
        src ? tok.omniphysicsDeformableBodyAPI : omni::physics::parse::TokenId{};
    for (uint32_t t = 0; t < 2; ++t)
    {
        if (attachableKeys[t].valid())
        {
            if (!(src && src->exists(attachableKeys[t])))
            {
                return false;
            }
            else if (src->hasSchema(attachableKeys[t], deformableBodyTok))
            {
                PhysxDeformableBodyDesc* desc = cookingDataAsync.parseDeformableBody(attachableKeys[t], attachedStage);
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
            else if (src->isA(attachableKeys[t], tok.xformableType))
            {
                getColliders(attachedStage, rigidColliders, attachableKeys[t]);
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

        std::swap(attachableKeys[0], attachableKeys[1]);
        std::swap(deformableDescs[0].desc, deformableDescs[1].desc);
        std::swap(attachableSlots[0], attachableSlots[1]);
        std::swap(types[0], types[1]);
    }

    return true;
}

namespace
{

// Everything setupAutoDeformableAttachment / the in-memory layout need to know about one
// auto-attachment prim's attachables.
struct AutoAttachmentInputs
{
    AttachmentActorType::Enum types[2] = { AttachmentActorType::eINVALID, AttachmentActorType::eINVALID };
    omni::physics::parse::ObjectKey attachableKeys[2];
    DeformableDescRef deformableDescs[2] = { nullptr, nullptr };
    uint32_t attachableSlots[2] = { 0, 1 };
    uint32_t numDeformables = 0;
    uint32_t numXformables = 0;
    std::vector<omni::physics::parse::ObjectKey> rigidColliders;
};

// Schema gate + attachable classification. False when the prim is not an auto attachment or the
// attachables form no supported combination (at least one volume or surface deformable).
bool parseAutoAttachmentInputs(usdparser::AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey autoAttachmentKey,
                               AutoAttachmentInputs& in,
                               const char* caller)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    if (!src->hasSchema(autoAttachmentKey, tok.PhysxAutoDeformableAttachmentAPI))
        return false;

    CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (!cookingDataAsync)
    {
        CARB_LOG_WARN("%s: failed - Cooking not available", caller);
        return false;
    }

    if (!parseAttachables(attachedStage, in.types, in.attachableKeys, in.deformableDescs, in.attachableSlots,
        in.numDeformables, in.numXformables, in.rigidColliders, autoAttachmentKey, *cookingDataAsync))
    {
        CARB_LOG_WARN("%s: parsing attachables failed", caller);
        return false;
    }

    return in.numDeformables != 0;
}

omni::physics::parse::ObjectType toObjectType(attachmentauthoring::AttachmentPrimKind kind)
{
    switch (kind)
    {
    case attachmentauthoring::AttachmentPrimKind::eVtxVtx:
        return eAttachmentVtxVtx;
    case attachmentauthoring::AttachmentPrimKind::eVtxTri:
        return eAttachmentVtxTri;
    case attachmentauthoring::AttachmentPrimKind::eVtxTet:
        return eAttachmentVtxTet;
    case attachmentauthoring::AttachmentPrimKind::eVtxXform:
    default:
        return eAttachmentVtxXform;
    }
}

// Receives the sub-prim set an auto attachment needs, realized either as authored USD prims
// or as in-memory generated children.
struct AutoAttachmentSubPrimSink
{
    virtual ~AutoAttachmentSubPrimSink() = default;
    virtual void attachment(const char* name, attachmentauthoring::AttachmentPrimKind kind,
                            omni::physics::parse::ObjectKey body0, omni::physics::parse::ObjectKey body1) = 0;
    virtual void filter(const char* name, omni::physics::parse::ObjectKey body0, omni::physics::parse::ObjectKey body1) = 0;
};

struct AuthoredSubPrimSink : AutoAttachmentSubPrimSink
{
    usdparser::AttachedStage& stage;
    omni::physics::parse::ObjectKey autoKey;

    AuthoredSubPrimSink(usdparser::AttachedStage& s, omni::physics::parse::ObjectKey k) : stage(s), autoKey(k) {}

    void attachment(const char* name, attachmentauthoring::AttachmentPrimKind kind,
                    omni::physics::parse::ObjectKey body0, omni::physics::parse::ObjectKey body1) override
    {
        attachmentauthoring::defineAttachmentPrim(stage, autoKey, name, kind, body0, body1);
    }
    void filter(const char* name, omni::physics::parse::ObjectKey body0, omni::physics::parse::ObjectKey body1) override
    {
        attachmentauthoring::defineElementCollisionFilterPrim(stage, autoKey, name, body0, body1);
    }
};

struct GeneratedSubPrimSink : AutoAttachmentSubPrimSink
{
    usdparser::AttachedStage& stage;
    std::string autoPathText;
    GeneratedAutoAttachmentLayout layout;

    GeneratedSubPrimSink(usdparser::AttachedStage& s, omni::physics::parse::ObjectKey autoKey)
        : stage(s), autoPathText(s.textViewFor(autoKey)) {}

    // Same child paths the USD arm authors, minted without a prim behind them.
    omni::physics::parse::ObjectKey childKey(const char* name) const
    {
        return stage.keyFor(autoPathText + "/" + name);
    }
    void attachment(const char* name, attachmentauthoring::AttachmentPrimKind kind,
                    omni::physics::parse::ObjectKey body0, omni::physics::parse::ObjectKey body1) override
    {
        layout.children.push_back({ childKey(name), toObjectType(kind), body0, body1 });
    }
    void filter(const char* name, omni::physics::parse::ObjectKey body0, omni::physics::parse::ObjectKey body1) override
    {
        layout.children.push_back({ childKey(name), eDeformableCollisionFilter, body0, body1 });
    }
};

// Add all possibly needed primitives, they can always be disabled, if not needed. This is according to the strategy
// to never create primitives asynchronously. We are adding the targets already, because parsing might already
// happen before authoring gets to generate the data.
void defineAutoAttachmentSubPrims(AutoAttachmentSubPrimSink& sink, AutoAttachmentInputs& in)
{
    if (in.numXformables == 1)
    {
        const omni::physics::parse::ObjectKey simMeshKey = in.deformableDescs[0]->simMeshKey;
        const omni::physics::parse::ObjectKey collMeshKey = in.deformableDescs[0]->collisionMeshKey;
        const omni::physics::parse::ObjectKey xformableKey = in.attachableKeys[1];

        // TODO extend UsdPhysics with TriXform and TetXform attachments ?
        sink.attachment("vtx_xform_attachment", attachmentauthoring::AttachmentPrimKind::eVtxXform, simMeshKey, xformableKey);

        for (uint32_t c = 0; c < uint32_t(in.rigidColliders.size()); ++c)
        {
            const omni::physics::parse::ObjectKey rigidColliderKey = in.rigidColliders[c];
            char primName[64]; sprintf_s(primName, 64, "element_filter_%d", c);
            sink.filter(primName,
                in.attachableSlots[0] == 0 ? collMeshKey : rigidColliderKey,
                in.attachableSlots[0] == 0 ? rigidColliderKey : collMeshKey);
        }
    }
    else if (in.numDeformables == 2)
    {
        const omni::physics::parse::ObjectKey simMeshKeys[2] = { in.deformableDescs[0]->simMeshKey, in.deformableDescs[1]->simMeshKey };
        const omni::physics::parse::ObjectKey collMeshKeys[2] = { in.deformableDescs[0]->collisionMeshKey, in.deformableDescs[1]->collisionMeshKey };

        // Create vtxVtx for snapping vertex to vertex attachments
        sink.attachment("vtx_vtx_attachment", attachmentauthoring::AttachmentPrimKind::eVtxVtx,
            simMeshKeys[in.attachableSlots[0]], simMeshKeys[in.attachableSlots[1]]);

        for (uint32_t t = 0; t < 2; ++t)
        {
            uint32_t other = 1 - t;
            if (in.types[t] == AttachmentActorType::eSURFACE_DEFORMABLE)
            {
                // Create vtxTri for snapping vertex to triangle attachments, we don't support triangle offsets yet
                char primName[64]; sprintf_s(primName, 64, "vtx%d_tri%d_attachment", in.attachableSlots[other], in.attachableSlots[t]);
                sink.attachment(primName, attachmentauthoring::AttachmentPrimKind::eVtxTri, simMeshKeys[other], simMeshKeys[t]);
            }
            else if (in.types[t] == AttachmentActorType::eVOLUME_DEFORMABLE)
            {
                // Not supporting surface tri attachments yet
                char primName[64]; sprintf_s(primName, 64, "vtx%d_tet%d_attachment", in.attachableSlots[other], in.attachableSlots[t]);
                sink.attachment(primName, attachmentauthoring::AttachmentPrimKind::eVtxTet, simMeshKeys[other], simMeshKeys[t]);
            }
        }

        sink.filter("element_filter", collMeshKeys[in.attachableSlots[0]], collMeshKeys[in.attachableSlots[1]]);
    }
}

// False, with nothing stored, when a child key fails to mint: the source's mintKeyForPath
// is existence-dependent, so no layout could be wired up without corrupting the reverse
// index (every child would share the invalid key).
bool buildGeneratedLayout(usdparser::AttachedStage& attachedStage,
                          omni::physics::parse::ObjectKey autoAttachmentKey,
                          AutoAttachmentInputs& in)
{
    GeneratedSubPrimSink sink(attachedStage, autoAttachmentKey);
    defineAutoAttachmentSubPrims(sink, in);
    for (const GeneratedAutoAttachmentChild& child : sink.layout.children)
    {
        if (!child.key.valid())
        {
            CARB_LOG_WARN("setupAutoDeformableAttachment: %s: the source cannot mint keys for unauthored sub prims, "
                          "no in-memory attachment generated", attachedStage.textFor(autoAttachmentKey));
            return false;
        }
    }
    attachedStage.setGeneratedAutoAttachmentLayout(autoAttachmentKey, std::move(sink.layout));
    return true;
}

// Number of sub prims defineAutoAttachmentSubPrims produces for the prim; 0 when it is not a
// usable auto attachment.
struct CountingSubPrimSink : AutoAttachmentSubPrimSink
{
    uint32_t count = 0;
    void attachment(const char*, attachmentauthoring::AttachmentPrimKind, omni::physics::parse::ObjectKey,
                    omni::physics::parse::ObjectKey) override
    {
        ++count;
    }
    void filter(const char*, omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey) override
    {
        ++count;
    }
};

// Releases the runtime objects of the current in-memory layout, the analog of
// removeAttachmentsAndFilters for authored sub-prims (mirrors PrimUpdate.cpp's handleRemovedPrim).
void releaseGeneratedAutoAttachmentObjects(usdparser::AttachedStage& attachedStage,
                                           omni::physics::parse::ObjectKey autoAttachmentKey)
{
    const GeneratedAutoAttachmentLayout* layout = attachedStage.getGeneratedAutoAttachmentLayout(autoAttachmentKey);
    if (!layout)
        return;
    ObjectDb* objectDb = attachedStage.getObjectDatabase();
    PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
    for (const GeneratedAutoAttachmentChild& child : layout->children)
    {
        const ObjectIdMap* entries = objectDb->getEntries(child.key);
        if (!entries || entries->empty())
            continue;
        for (const auto& entry : *entries)
            physInt->releaseObject(attachedStage, child.key, entry.second);
        objectDb->removeEntries(child.key);
    }
}

// Creates the runtime objects of the in-memory layout, attachments before filters like the load path.
void createGeneratedAutoAttachmentObjects(usdparser::AttachedStage& attachedStage,
                                          omni::physics::parse::ObjectKey autoAttachmentKey)
{
    const GeneratedAutoAttachmentLayout* layout = attachedStage.getGeneratedAutoAttachmentLayout(autoAttachmentKey);
    if (!layout)
        return;
    ObjectDb* objectDb = attachedStage.getObjectDatabase();
    PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
    for (const bool filters : { false, true })
    {
        for (const GeneratedAutoAttachmentChild& child : layout->children)
        {
            const bool isFilter = child.type == eDeformableCollisionFilter;
            if (isFilter != filters)
                continue;
            PhysxObjectDesc* desc = isFilter ? static_cast<PhysxObjectDesc*>(makeGeneratedDeformableCollisionFilterDesc(child))
                                             : static_cast<PhysxObjectDesc*>(makeGeneratedDeformableAttachmentDesc(child));
            const ObjectId id = physInt->createObject(attachedStage, child.key, *desc);
            if (id != kInvalidObjectId)
            {
                objectDb->findOrCreateEntry(child.key, attachedStage.textViewFor(child.key), desc->type, id);
            }
            else
            {
                // No prim is left behind to inspect on this path, so the log is the only trace.
                CARB_LOG_WARN("setupAutoDeformableAttachment: failed to create the generated %s %s",
                    isFilter ? "collision filter" : "attachment", attachedStage.textFor(child.key));
            }
            ICE_FREE(desc);
        }
    }
}

} // namespace

PhysxDeformableAttachmentDesc* makeGeneratedDeformableAttachmentDesc(const GeneratedAutoAttachmentChild& child)
{
    auto* desc = ICE_PLACEMENT_NEW(PhysxDeformableAttachmentDesc)();
    desc->type = child.type;
    desc->primKey = child.key;
    // Like the authored sub-prim, disabled until the generated payload says otherwise.
    desc->enabled = false;
    desc->src0 = child.src0;
    desc->src1 = child.src1;
    return desc;
}

PhysxDeformableCollisionFilterDesc* makeGeneratedDeformableCollisionFilterDesc(const GeneratedAutoAttachmentChild& child)
{
    auto* desc = ICE_PLACEMENT_NEW(PhysxDeformableCollisionFilterDesc)();
    desc->primKey = child.key;
    desc->enabled = false;
    desc->src0 = child.src0;
    desc->src1 = child.src1;
    return desc;
}

bool buildGeneratedAutoDeformableAttachmentLayout(usdparser::AttachedStage& attachedStage,
                                                  omni::physics::parse::ObjectKey autoAttachmentKey)
{
    AutoAttachmentInputs in;
    if (!parseAutoAttachmentInputs(attachedStage, autoAttachmentKey, in, "buildGeneratedAutoDeformableAttachmentLayout"))
        return false;
    return buildGeneratedLayout(attachedStage, autoAttachmentKey, in);
}

uint32_t expectedAutoDeformableAttachmentSubPrimCount(usdparser::AttachedStage& attachedStage,
                                                      omni::physics::parse::ObjectKey autoAttachmentKey)
{
    AutoAttachmentInputs in;
    if (!parseAutoAttachmentInputs(attachedStage, autoAttachmentKey, in, "expectedAutoDeformableAttachmentSubPrimCount"))
        return 0;
    CountingSubPrimSink sink;
    defineAutoAttachmentSubPrims(sink, in);
    return sink.count;
}

/**
* Initial setup of auto attachment. Creates all necessary sub prims depending on the attachable types.
* Pre-existing sub prims just get removed.
*
* With a live USD stage the sub prims are authored into it (Kit flow; the stage's change
* notices then create the runtime objects). Without one -- an ovstage attach -- nothing is
* pushed back to the source: the sub prims become an in-memory GeneratedAutoAttachmentLayout
* on the AttachedStage, the payload is generated into its generated-data cache, and the runtime
* objects are created here directly.
*/
bool setupAutoDeformableAttachment(omni::physics::parse::ObjectKey autoAttachmentKey)
{
    usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!attachedStage)
        return false;

    AutoAttachmentInputs in;
    if (!parseAutoAttachmentInputs(*attachedStage, autoAttachmentKey, in, "setupAutoDeformableAttachment"))
        return false;

    // Whatever the previous setup produced for this prim goes first: the in-memory layout's
    // objects and the layout itself, then (authoring branch) the authored sub prims. Clearing
    // the layout on the authoring branch matters because the getters and the CRC store prefer
    // a layout whenever one exists; a stale one would shadow the freshly authored prims.
    releaseGeneratedAutoAttachmentObjects(*attachedStage, autoAttachmentKey);
    attachedStage->clearGeneratedAutoAttachmentLayout(autoAttachmentKey);

    if (!attachmentauthoring::canAuthor(*attachedStage))
    {
        if (!buildGeneratedLayout(*attachedStage, autoAttachmentKey, in))
            return false;
        bool attachmentDataRecomputed = false;
        if (!updateAutoDeformableAttachment(autoAttachmentKey, attachmentDataRecomputed))
        {
            // No objects: a registered attachment with an empty payload would simulate as
            // nothing while the caller was told setup succeeded.
            CARB_LOG_WARN("setupAutoDeformableAttachment: generating attachment data failed for %s",
                attachedStage->textFor(autoAttachmentKey));
            attachedStage->clearGeneratedAutoAttachmentLayout(autoAttachmentKey);
            return false;
        }
        createGeneratedAutoAttachmentObjects(*attachedStage, autoAttachmentKey);
        return true;
    }

    attachmentauthoring::removeAttachmentsAndFilters(*attachedStage, autoAttachmentKey);

    AuthoredSubPrimSink sink(*attachedStage, autoAttachmentKey);
    defineAutoAttachmentSubPrims(sink, in);
    return true;
}

bool updateAutoDeformableAttachment(omni::physics::parse::ObjectKey autoAttachmentKey, bool& attachmentDataRecomputed)
{
    attachmentDataRecomputed = false;

    usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!attachedStage)
        return false;

    const omni::physics::parse::IPhysicsSource* src = attachedStage->getSource();
    if (!src)
        return false;
    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    if (!src->hasSchema(autoAttachmentKey, tok.PhysxAutoDeformableAttachmentAPI))
        return false;

    PhysxAutoAttachmentDesc autoAttachmentDesc;
    parsePhysxAutoAttachment(*attachedStage, autoAttachmentDesc, autoAttachmentKey);

    CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (!cookingDataAsync)
    {
        CARB_LOG_WARN("updateAutoDeformableAttachment: failed - Cooking not available");
        return false;
    }

    AttachmentActorType::Enum types[2] = { AttachmentActorType::eINVALID, AttachmentActorType::eINVALID };
    omni::physics::parse::ObjectKey attachableKeys[2];
    DeformableDescRef deformableDescs[2] = { nullptr, nullptr };
    uint32_t attachableSlots[2] = { 0, 1 };
    uint32_t numDeformables = 0;
    uint32_t numXformables = 0;
    std::vector<omni::physics::parse::ObjectKey> rigidColliders;

    if (!parseAttachables(*attachedStage, types, attachableKeys, deformableDescs, attachableSlots, numDeformables,
        numXformables, rigidColliders, autoAttachmentKey, *cookingDataAsync))
    {
        CARB_LOG_WARN("updateAutoDeformableAttachment: parsing attachables failed");
        return false;
    }

    // Early out because there is no valid attachment combination (at least 1 volume or surface deformable)
    if (numDeformables == 0)
        return false;

    MaskShapes maskShapes;
    {
        std::vector<omni::physics::parse::ObjectKey> targets;
        internal::getRelationshipValue(*attachedStage, autoAttachmentKey, tok.physxAutoDeformableAttachmentMaskShapes, targets);

        for (uint32_t i = 0; i < targets.size(); ++i)
        {
            const omni::physics::parse::ObjectKey maskShapeKey = targets[i];
            if (src->exists(maskShapeKey))
            {
                PxGeometryHolder holder;
                PxTransform transform;
                bool success = parseMaskShape(*attachedStage, holder, transform, maskShapeKey);
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
        const omni::physics::parse::ObjectKey primKey = attachableKeys[d];
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
                cookingDataAsync->cookSurfaceDeformableBody(*(PhysxSurfaceDeformableBodyDesc*)desc, primKey, *attachedStage, false);
            }
            else if (type == AttachmentActorType::eVOLUME_DEFORMABLE)
            {
                cookingDataAsync->cookVolumeDeformableBody(*(PhysxVolumeDeformableBodyDesc*)desc, primKey, *attachedStage, false);
            }
        }

        if (!getDeformableMeshInfo(*attachedStage, deformableMeshInfo[d], primKey, *desc))
            return false;

        inputCrc.setMeshKey(deformableMeshInfo[d].deformableBodyDataCrc);
    }

    // Cache key of the last generated payload: kept on the in-memory layout when the attach
    // cannot author sub-prims, otherwise as the inputCrc attribute on the prim.
    GeneratedAutoAttachmentLayout* layout = attachedStage->getGeneratedAutoAttachmentLayout(autoAttachmentKey);
    omni::physx::usdparser::MeshKey storedInputCrc;
    if (layout)
    {
        if (layout->inputCrc.size() == sizeof(storedInputCrc))
            std::memcpy(&storedInputCrc, layout->inputCrc.data(), sizeof(storedInputCrc));
    }
    else
    {
        storedInputCrc = loadMeshKey(*attachedStage, autoAttachmentKey, autoDeformableAttachmentInputCrcToken);
    }
    if (storedInputCrc == inputCrc)
    {
        // nothing to compute
        return true;
    }
    if (layout)
    {
        const uint8_t* crcBytes = reinterpret_cast<const uint8_t*>(&inputCrc);
        layout->inputCrc.assign(crcBytes, crcBytes + sizeof(inputCrc));
    }
    else
    {
        storeMeshKey(*attachedStage, autoAttachmentKey, autoDeformableAttachmentInputCrcToken, inputCrc);
    }
    attachmentDataRecomputed = true;

    attachedStage->clearGeneratedDeformableAttachmentDataUnderPath(autoAttachmentKey);
    // USD-stage mirror of the reset above: disables the previous run's Attachment /
    // ElementCollisionFilter prims on the live stage. No-op without one (in-memory layout).
    attachmentauthoring::disableAttachmentsAndFilters(*attachedStage, autoAttachmentKey);

    if (numXformables == 1)
    {
        DeformableMeshInfo& deformableInfo = deformableMeshInfo[0];
        const omni::physics::parse::ObjectKey xformableKey = attachableKeys[1];
        omni::physics::parse::ObjectKey vtxXformKey;
        std::vector<omni::physics::parse::ObjectKey> filterKeys;
        bool validAttachment = getVtxXformAttachment(*attachedStage, vtxXformKey, autoAttachmentKey,
            deformableInfo.simMeshKey, xformableKey);
        bool validFilters = getElementCollisionFilters(*attachedStage, filterKeys, autoAttachmentKey,
            rigidColliders, deformableInfo.collMeshKey);

        if (validAttachment && validFilters)
        {
            updateDeformableXformableAttachments(*attachedStage, vtxXformKey, filterKeys, deformableInfo,
                attachableSlots[0], xformableKey, rigidColliders, autoAttachmentDesc, maskShapes);
        }
        else
        {
            PhysXUsdPhysicsInterface::reportLoadError(
                usdparser::ErrorCode::eError,
                "Deformable attachment targets changed, please refresh or re-create attachment!");
        }
    }
    else if (numDeformables == 2)
    {
        if (types[0] == AttachmentActorType::eVOLUME_DEFORMABLE)
        {
            if (types[1] == AttachmentActorType::eVOLUME_DEFORMABLE)
            {
                omni::physics::parse::ObjectKey vtxTetKeys[2];
                omni::physics::parse::ObjectKey filterKey;
                bool validAttachment0 = getVtxTetAttachment(*attachedStage, vtxTetKeys[0], autoAttachmentKey,
                    deformableMeshInfo[0].simMeshKey, deformableMeshInfo[1].simMeshKey);
                bool validAttachment1 = getVtxTetAttachment(*attachedStage, vtxTetKeys[1], autoAttachmentKey,
                    deformableMeshInfo[1].simMeshKey, deformableMeshInfo[0].simMeshKey);
                bool validFilter = getElementCollisionFilter(*attachedStage, filterKey, autoAttachmentKey,
                    deformableMeshInfo[0].collMeshKey,
                    deformableMeshInfo[1].collMeshKey);

                if (validAttachment0 && validAttachment1 && validFilter)
                {
                    updateDeformableVolumeVolumeAttachments(*attachedStage, vtxTetKeys, filterKey,
                        deformableMeshInfo, attachableSlots, autoAttachmentDesc, maskShapes);
                }
                else
                {
                    PhysXUsdPhysicsInterface::reportLoadError(
                        usdparser::ErrorCode::eError,
                        "Deformable attachment targets changed, please refresh or re-create attachment!");
                }
            }
            else if (types[1] == AttachmentActorType::eSURFACE_DEFORMABLE)
            {
                omni::physics::parse::ObjectKey vtxTetKey;
                omni::physics::parse::ObjectKey filterKey;
                bool validAttachment = getVtxTetAttachment(*attachedStage, vtxTetKey, autoAttachmentKey,
                    deformableMeshInfo[1].simMeshKey, deformableMeshInfo[0].simMeshKey);
                bool validFilter = getElementCollisionFilter(*attachedStage, filterKey, autoAttachmentKey,
                    deformableMeshInfo[attachableSlots[0]].collMeshKey,
                    deformableMeshInfo[attachableSlots[1]].collMeshKey);

                if (validAttachment && validFilter)
                {
                    updateDeformableVolumeSurfaceAttachments(*attachedStage, vtxTetKey, filterKey,
                        deformableMeshInfo[0], attachableSlots[0],
                        deformableMeshInfo[1], attachableSlots[1],
                        autoAttachmentDesc, maskShapes);
                }
                else
                {
                    PhysXUsdPhysicsInterface::reportLoadError(
                        usdparser::ErrorCode::eError,
                        "Deformable attachment targets changed, please refresh or re-create attachment!");
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
