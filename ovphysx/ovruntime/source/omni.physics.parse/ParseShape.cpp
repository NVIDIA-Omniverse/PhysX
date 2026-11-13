// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-PARSE-SHAPE-001
 * @covers AC-3 AC-4
 *
 * @implements REQ-PARSE-SHAPE-006
 * @covers AC-2
 *
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-1
 */

// Simple-shape parsers — per-type construction plus the shared
// fillPhysxShapeDesc overlay. Mesh shapes and their cooking-service
// invocation live in the consumer-side walker, not here.

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedShape.h>
#include <omni/physics/parse/KnownTokens.h>

#include <common/foundation/MatrixTools.h>

#include <carb/logging/Log.h>

#include <algorithm>
#include <string>

namespace omni::physics::parse
{

namespace
{

// Common shape fill: applies the shared transform/scale/body fields,
// the PhysxCollisionAPI extension overlay, and resolves simulationOwners.
// Returns false when `simulationOwners` was non-empty but no owner
// resolved to a scene ObjectId — the shape is rejected in that case.
bool fillCommonShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info, PhysxShapeDesc& desc)
{
    const PreparedShapeCommon input = prepareShapeCommon(ctx, key, info);
    desc.sceneIds.resize(input.sceneIds.size());
    return fillShapeCommon(input, desc);
}

// Reads `physxConvexGeometry:margin` (Cylinder/Cone shapes). Gated on
// HasAuthoredValue and clamped to [0, +inf).
void readConvexGeometryMargin(ParseContext& ctx, ObjectKey key, float& margin)
{
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();

    if (!src.hasAuthoredAttribute(key, tok.physxConvexGeometryMargin))
        return;
    float m;
    if (!src.getAttribute(key, tok.physxConvexGeometryMargin, m))
        return;
    margin = std::max(0.0f, m);
}

} // namespace

PreparedShapeCommon prepareShapeCommon(ParseContext& ctx, ObjectKey key, const ShapeInfo& info)
{
    PreparedShapeCommon input;
    input.units = ctx.units();
    input.info = info;
    input.collision = prepareCollisionExt(ctx, key);
    input.sceneIds.reserve(info.simulationOwners.size());
    for (const ObjectKey owner : info.simulationOwners)
    {
        const ObjectId id = ctx.objects().findEntry(owner, eScene);
        if (id != kInvalidObjectId)
            input.sceneIds.push_back(id);
    }
    input.ownersResolved = info.simulationOwners.empty() || !input.sceneIds.empty();
    input.prepared = true;
    return input;
}

bool fillShapeCommon(const PreparedShapeCommon& input, PhysxShapeDesc& desc) noexcept
{
    if (!input.prepared || !input.ownersResolved || desc.sceneIds.size() != input.sceneIds.size())
        return false;
    setToDefault(desc, input.units);

    desc.collisionEnabled = input.info.collisionEnabled;
    desc.collisionGroup   = input.info.collisionGroup;
    desc.rigidBody        = input.info.rigidBody;
    desc.localPos         = input.info.localPos;
    desc.localRot         = input.info.localRot;
    desc.localScale       = input.info.localScale;
    desc.sourceGprim      = input.info.sourceGprim;

    // Overlay PhysX extension fields + Newton contactMargin/contactGap
    // fallbacks via parseCollisionExt (already covers torsionalPatchRadius,
    // minTorsionalPatchRadius, contactOffset, restOffset, isTrigger,
    // isTriggerUsdOutput, plus the sentinel/range/cross-validation logic).
    CollisionExtFields fields;
    fields.contactOffset          = desc.contactOffset;
    fields.restOffset             = desc.restOffset;
    fields.torsionalPatchRadius   = desc.torsionalPatchRadius;
    fields.minTorsionalPatchRadius = desc.minTorsionalPatchRadius;
    fields.isTrigger              = desc.isTrigger;
    fields.isTriggerUsdOutput     = desc.isTriggerUsdOutput;
    if (!fillCollisionExt(input.collision, fields))
        return false;
    desc.contactOffset            = fields.contactOffset;
    desc.restOffset               = fields.restOffset;
    desc.torsionalPatchRadius     = fields.torsionalPatchRadius;
    desc.minTorsionalPatchRadius  = fields.minTorsionalPatchRadius;
    desc.isTrigger                = fields.isTrigger;
    desc.isTriggerUsdOutput       = fields.isTriggerUsdOutput;

    std::copy(input.sceneIds.begin(), input.sceneIds.end(), desc.sceneIds.begin());
    return true;
}

void setToDefault(PhysxShapeDesc& desc, const SourceUnits& units)
{
    // Units-aware overlay only. Units-agnostic fields (restOffset,
    // torsionalPatchRadius*, isTrigger*) are default-initialised in the
    // descriptor body in Descriptors.h.
    if (desc.type == ePlaneShape)
    {
        const float tolerancesLength = 1.0f / units.metersPerUnit;
        desc.contactOffset = 0.02f * tolerancesLength;
    }
    // For non-plane shapes the descriptor's in-class default leaves
    // contactOffset at the -1 sentinel ("recompute in the PhysX layer").
}

DescPtr<SpherePhysxShapeDesc> parseSphereShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info, float radius)
{
    DescPtr<SpherePhysxShapeDesc> d = allocateDesc<SpherePhysxShapeDesc>(ctx.descriptorAllocator(), radius);
    if (!fillCommonShape(ctx, key, info, *d))
        return {};
    return d;
}

DescPtr<BoxPhysxShapeDesc> parseBoxShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info, carb::Float3 halfExtents)
{
    DescPtr<BoxPhysxShapeDesc> d = allocateDesc<BoxPhysxShapeDesc>(ctx.descriptorAllocator(), halfExtents);
    if (!fillCommonShape(ctx, key, info, *d))
        return {};
    return d;
}

DescPtr<CapsulePhysxShapeDesc> parseCapsuleShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info,
                                                 float radius, float halfHeight, Axis axis)
{
    DescPtr<CapsulePhysxShapeDesc> d =
        allocateDesc<CapsulePhysxShapeDesc>(ctx.descriptorAllocator(), radius, halfHeight, axis);
    if (!fillCommonShape(ctx, key, info, *d))
        return {};
    return d;
}

DescPtr<CylinderPhysxShapeDesc> parseCylinderShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info,
                                                   float radius, float halfHeight, Axis axis)
{
    DescPtr<CylinderPhysxShapeDesc> d =
        allocateDesc<CylinderPhysxShapeDesc>(ctx.descriptorAllocator(), radius, halfHeight, axis);
    if (!fillCommonShape(ctx, key, info, *d))
        return {};
    readConvexGeometryMargin(ctx, key, d->margin);
    return d;
}

DescPtr<ConePhysxShapeDesc> parseConeShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info,
                                            float radius, float halfHeight, Axis axis)
{
    DescPtr<ConePhysxShapeDesc> d =
        allocateDesc<ConePhysxShapeDesc>(ctx.descriptorAllocator(), radius, halfHeight, axis);
    if (!fillCommonShape(ctx, key, info, *d))
        return {};
    readConvexGeometryMargin(ctx, key, d->margin);
    return d;
}

DescPtr<PlanePhysxShapeDesc> parsePlaneShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info, Axis axis)
{
    DescPtr<PlanePhysxShapeDesc> d = allocateDesc<PlanePhysxShapeDesc>(ctx.descriptorAllocator());
    d->axis = axis;
    if (!fillCommonShape(ctx, key, info, *d))
        return {};
    return d;
}

DescPtr<SpherePointsPhysxShapeDesc> parseSpherePointsShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info,
                                                           std::vector<SpherePhysxPoint> spheres)
{
    DescPtr<SpherePointsPhysxShapeDesc> d = allocateDesc<SpherePointsPhysxShapeDesc>(ctx.descriptorAllocator());
    d->spheres = std::move(spheres);
    if (!fillCommonShape(ctx, key, info, *d))
        return {};
    return d;
}

DescPtr<CustomPhysxShapeDesc> parseCustomShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info,
                                                size_t customGeometryTokenHash)
{
    DescPtr<CustomPhysxShapeDesc> d = allocateDesc<CustomPhysxShapeDesc>(ctx.descriptorAllocator());
    d->customGeometryTokenHash = customGeometryTokenHash;
    if (!fillCommonShape(ctx, key, info, *d))
        return {};
    return d;
}

MeshApproximation parseMeshApproximation(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();

    //   - When PhysicsMeshCollisionAPI is not applied, default to "none".
    //   - When applied, read `physics:approximation` (schema fallback is
    //     also "none"), then map to the enum.
    if (!src.hasSchema(key, tok.physicsMeshCollisionAPI))
        return MeshApproximation::eNone;

    TokenId approx;
    if (!src.getAttribute(key, tok.physicsApproximation, approx))
        return MeshApproximation::eNone;

    if      (approx == tok.approximationNone)               return MeshApproximation::eNone;
    else if (approx == tok.approximationConvexHull)         return MeshApproximation::eConvexHull;
    else if (approx == tok.approximationBoundingSphere)     return MeshApproximation::eBoundingSphere;
    else if (approx == tok.approximationBoundingCube)       return MeshApproximation::eBoundingCube;
    else if (approx == tok.approximationMeshSimplification) return MeshApproximation::eMeshSimplification;
    else if (approx == tok.approximationConvexDecomposition)return MeshApproximation::eConvexDecomposition;
    else if (approx == tok.approximationSphereFill)         return MeshApproximation::eSphereFill;
    else if (approx == tok.approximationSdf)                return MeshApproximation::eSdf;

    // Unknown token — return eNone as a safe fallback. The consumer's
    // routing site logs the error and rejects the shape.
    return MeshApproximation::eNone;
}

MeshGeometry parseMeshGeometry(ParseContext& ctx, ObjectKey key)
{
    return ctx.source().getMeshAttributes(key);
}

namespace
{

// A member's topology has to be self-consistent before its faces are reversed and
// its indices offset: every face has at least three vertices, the face counts sum to
// the index count, every index addresses one of its points and every hole one of its
// faces. USD does not enforce any of this on authoring, and the triangulator only
// advances its cursor for faces of three or more vertices.
bool mergedMemberTopologyIsConsistent(BufferSpan<carb::Float3> points, BufferSpan<int32_t> indices,
                                      BufferSpan<int32_t> faceCounts, BufferSpan<int32_t> holes)
{
    size_t indexTotal = 0;
    for (size_t face = 0; face < faceCounts.count; ++face)
    {
        if (faceCounts[face] < 3)
            return false;
        indexTotal += size_t(faceCounts[face]);
    }
    if (indexTotal != indices.count)
        return false;
    for (size_t i = 0; i < indices.count; ++i)
        if (indices[i] < 0 || size_t(indices[i]) >= points.count)
            return false;
    for (size_t i = 0; i < holes.count; ++i)
        if (holes[i] < 0 || size_t(holes[i]) >= faceCounts.count)
            return false;
    return true;
}

// Releases the buffers a getMeshAttributes read registered on the source once the
// member has been copied or rejected, so a rescan on a long-lived source does not
// accumulate them.
struct MeshGeometryRelease
{
    const IPhysicsSource& src;
    const MeshGeometry& geometry;
    ~MeshGeometryRelease()
    {
        for (const BufferHandle handle : { geometry.points, geometry.indices, geometry.faceCounts, geometry.holes, geometry.faceMaterials })
            if (handle.valid())
                src.releaseBuffer(handle);
    }
};

} // namespace

size_t buildMergedMesh(ParseContext& ctx, ObjectKey colliderKey, MergeMeshDesc& out)
{
    out.points.clear();
    out.indices.clear();
    out.faces.clear();
    out.holes.clear();
    const IPhysicsSource& src = ctx.source();

    std::vector<ObjectKey> members;
    src.resolveCollection(colliderKey, src.internToken("collisionmeshes"), members);
    // IPhysicsSource::resolveCollection promises no order: the USD source returns the
    // sorted path set, the ovstage source the include targets in authored order expanded
    // depth first. The merged layout must not depend on which source built it.
    std::sort(members.begin(), members.end(), [&src](ObjectKey a, ObjectKey b)
    { return src.sourceKeyToString(a) < src.sourceKeyToString(b); });

    const TokenId orientationToken = src.internToken("orientation");
    const TokenId leftHandedToken = src.internToken("leftHanded");
    Matrix4d colliderWorld;
    src.getLocalToWorldTransform(colliderKey, colliderWorld);
    // Gf's `member * inverse(collider)` reads `inverse(collider) * member` in PhysX operand order.
    const ::physx::PxMat44d colliderInverse = omni::physx::affineInverse(::physx::PxMat44d(colliderWorld.data));

    size_t merged = 0;
    for (const ObjectKey member : members)
    {
        // Face materials are not merged; do not have the source derive them.
        const MeshGeometry geometry = src.getMeshAttributes(member, /*includeFaceMaterials=*/false);
        const MeshGeometryRelease release{ src, geometry };
        const BufferSpan<carb::Float3> points = ctx.getBuffer<carb::Float3>(geometry.points);
        const BufferSpan<int32_t> indices = ctx.getBuffer<int32_t>(geometry.indices);
        const BufferSpan<int32_t> faceCounts = ctx.getBuffer<int32_t>(geometry.faceCounts);
        const BufferSpan<int32_t> holes = ctx.getBuffer<int32_t>(geometry.holes);
        if (points.empty() || indices.empty() || faceCounts.empty())
            continue; // not a mesh, or a mesh whose geometry the source did not populate
        if (!mergedMemberTopologyIsConsistent(points, indices, faceCounts, holes))
        {
            const std::string memberPath(src.sourceKeyToString(member));
            CARB_LOG_WARN("PhysxMeshMergeCollisionAPI: skipping collisionmeshes member %s, its face counts, "
                          "indices, points and holes are inconsistent", memberPath.c_str());
            continue;
        }
        TokenId orientation;
        const bool leftHanded = src.getAttribute(member, orientationToken, orientation) && orientation == leftHandedToken;

        Matrix4d memberWorld;
        src.getLocalToWorldTransform(member, memberWorld);
        const ::physx::PxMat44d relative = colliderInverse * ::physx::PxMat44d(memberWorld.data);
        // A reflecting member transform is baked into the points, where PhysX's negative
        // meshScale handling cannot see it, so it reverses the winding like a left-handed
        // authoring does.
        const ::physx::PxVec3d basis0 = relative.column0.getXYZ();
        const ::physx::PxVec3d basis1 = relative.column1.getXYZ();
        const ::physx::PxVec3d basis2 = relative.column2.getXYZ();
        const bool reflects = basis0.dot(basis1.cross(basis2)) < 0.0;
        const bool reverse = leftHanded != reflects;

        // No per-member reserve: an exact-size reserve would reallocate on every member
        // and copy everything merged so far, so the vectors grow geometrically instead.
        const size_t pointsOffset = out.points.size();
        const size_t indicesOffset = out.indices.size();
        const size_t facesOffset = out.faces.size();
        for (size_t i = 0; i < points.count; ++i)
        {
            const ::physx::PxVec3d p = relative.transform(::physx::PxVec3d(points[i].x, points[i].y, points[i].z));
            out.points.push_back({ float(p.x), float(p.y), float(p.z) });
        }
        out.faces.insert(out.faces.end(), faceCounts.data, faceCounts.data + faceCounts.count);
        const int32_t base = int32_t(pointsOffset);
        for (size_t i = 0; i < indices.count; ++i)
            out.indices.push_back(indices[i] + base);
        if (reverse)
        {
            // Keep each face's first vertex and reverse the rest: the triangulator fans
            // from the first vertex, so this flips the winding without moving the fan
            // anchor, which is exactly what cooking a left-handed mesh on its own does.
            auto faceBegin = out.indices.begin() + ptrdiff_t(indicesOffset);
            for (size_t face = 0; face < faceCounts.count; ++face)
            {
                const ptrdiff_t count = ptrdiff_t(faceCounts[face]);
                std::reverse(faceBegin + 1, faceBegin + count);
                faceBegin += count;
            }
        }
        for (size_t i = 0; i < holes.count; ++i)
            out.holes.push_back(holes[i] + int32_t(facesOffset));
        ++merged;
    }
    if (merged == 0)
    {
        const std::string colliderPath(src.sourceKeyToString(colliderKey));
        CARB_LOG_WARN("PhysxMeshMergeCollisionAPI on %s: no collisionmeshes member supplied mesh geometry, "
                      "the collider emits no shape", colliderPath.c_str());
    }
    return merged;
}

void scaleShapeDescByInstance(PhysxShapeDesc& desc, carb::Float3 instanceScale)
{
    // Per-shape-type geometry-field scaling. Capsule/Cylinder/Cone use
    // the long-axis scale for halfHeight and the max of the perpendicular
    // components for radius — preserves an upright shape under
    // non-uniform scale. localPos / localScale are NOT touched here —
    // the caller sets them from the instance proxy's world transform.
    const float absX = std::fabs(instanceScale.x);
    const float absY = std::fabs(instanceScale.y);
    const float absZ = std::fabs(instanceScale.z);
    const float maxAll = std::max(absX, std::max(absY, absZ));

    auto compMult = [](const carb::Float3& a, const carb::Float3& b) {
        return carb::Float3{ a.x * b.x, a.y * b.y, a.z * b.z };
    };

    // Bounding shapes are fitted later from this per-instance working buffer.
    // Scaling just radius/halfExtents is insufficient: fitting overwrites them.
    if (desc.type == eBoundingSphereShape || desc.type == eBoundingBoxShape)
    {
        MergeMeshPhysxShapeDesc& d = static_cast<MergeMeshPhysxShapeDesc&>(desc);
        if (d.mergedMesh)
            for (carb::Float3& point : d.mergedMesh->points)
                point = compMult(point, instanceScale);
    }

    switch (desc.type)
    {
    case eSphereShape:
    {
        SpherePhysxShapeDesc& d = static_cast<SpherePhysxShapeDesc&>(desc);
        d.radius *= maxAll;
        break;
    }
    case eBoxShape:
    {
        BoxPhysxShapeDesc& d = static_cast<BoxPhysxShapeDesc&>(desc);
        d.halfExtents = compMult(d.halfExtents, instanceScale);
        break;
    }
    case eCapsuleShape:
    {
        CapsulePhysxShapeDesc& d = static_cast<CapsulePhysxShapeDesc&>(desc);
        if (d.axis == Axis::eX) { d.halfHeight *= instanceScale.x; d.radius *= std::max(absY, absZ); }
        else if (d.axis == Axis::eY) { d.halfHeight *= instanceScale.y; d.radius *= std::max(absX, absZ); }
        else { d.halfHeight *= instanceScale.z; d.radius *= std::max(absX, absY); }
        break;
    }
    case eCylinderShape:
    {
        CylinderPhysxShapeDesc& d = static_cast<CylinderPhysxShapeDesc&>(desc);
        if (d.axis == Axis::eX) { d.halfHeight *= instanceScale.x; d.radius *= std::max(absY, absZ); }
        else if (d.axis == Axis::eY) { d.halfHeight *= instanceScale.y; d.radius *= std::max(absX, absZ); }
        else { d.halfHeight *= instanceScale.z; d.radius *= std::max(absX, absY); }
        break;
    }
    case eConeShape:
    {
        ConePhysxShapeDesc& d = static_cast<ConePhysxShapeDesc&>(desc);
        if (d.axis == Axis::eX) { d.halfHeight *= instanceScale.x; d.radius *= std::max(absY, absZ); }
        else if (d.axis == Axis::eY) { d.halfHeight *= instanceScale.y; d.radius *= std::max(absX, absZ); }
        else { d.halfHeight *= instanceScale.z; d.radius *= std::max(absX, absY); }
        break;
    }
    case eConvexMeshShape:
    {
        ConvexMeshPhysxShapeDesc& d = static_cast<ConvexMeshPhysxShapeDesc&>(desc);
        d.meshScale = compMult(d.meshScale, instanceScale);
        d.convexCookingParams.signScale = scaleToSignScale(d.meshScale);
        break;
    }
    case eConvexMeshDecompositionShape:
    {
        ConvexMeshDecompositionPhysxShapeDesc& d = static_cast<ConvexMeshDecompositionPhysxShapeDesc&>(desc);
        d.meshScale = compMult(d.meshScale, instanceScale);
        d.convexDecompositionCookingParams.signScale = scaleToSignScale(d.meshScale);
        break;
    }
    case eTriangleMeshShape:
    {
        TriangleMeshPhysxShapeDesc& d = static_cast<TriangleMeshPhysxShapeDesc&>(desc);
        d.meshScale = compMult(d.meshScale, instanceScale);
        break;
    }
    case eSpherePointsShape:
    {
        SpherePointsPhysxShapeDesc& d = static_cast<SpherePointsPhysxShapeDesc&>(desc);
        d.meshScale = compMult(d.meshScale, instanceScale);
        d.sphereFillCookingParams.signScale = scaleToSignScale(d.meshScale);
        break;
    }
    case eBoundingSphereShape:
    {
        BoundingSpherePhysxShapeDesc& d = static_cast<BoundingSpherePhysxShapeDesc&>(desc);
        d.positionOffset = compMult(d.positionOffset, instanceScale);
        d.radius *= maxAll;
        break;
    }
    case eBoundingBoxShape:
    {
        BoundingBoxPhysxShapeDesc& d = static_cast<BoundingBoxPhysxShapeDesc&>(desc);
        d.positionOffset = compMult(d.positionOffset, instanceScale);
        d.halfExtents = compMult(d.halfExtents, instanceScale);
        break;
    }
    case ePlaneShape:
    case eCustomShape:
    default:
        break;
    }
}

void scaleMeshPoints(ParseContext& ctx, BufferHandle points,
                     carb::Float3 meshScale, std::vector<carb::Float3>& out)
{
    out.clear();
    if (!points.valid())
        return;

    BufferSpan<carb::Float3> in = ctx.getBuffer<carb::Float3>(points);
    if (in.empty())
        return;

    out.reserve(in.count);
    for (size_t i = 0; i < in.count; ++i)
    {
        const carb::Float3& p = in[i];
        out.push_back({ p.x * meshScale.x, p.y * meshScale.y, p.z * meshScale.z });
    }
}

DescPtr<BoundingSpherePhysxShapeDesc> makeBoundingSphereShape(
    ParseContext& ctx, ObjectKey key, const ShapeInfo& info,
    carb::Float3 sphereCenter, float radius)
{
    DescPtr<BoundingSpherePhysxShapeDesc> d = allocateDesc<BoundingSpherePhysxShapeDesc>(ctx.descriptorAllocator());
    d->positionOffset = sphereCenter;
    d->radius = radius;
    if (!fillCommonShape(ctx, key, info, *d))
        return {};
    return d;
}

DescPtr<BoundingBoxPhysxShapeDesc> makeBoundingBoxShape(
    ParseContext& ctx, ObjectKey key, const ShapeInfo& info,
    carb::Float3 halfExtents, carb::Float3 offsetPos, carb::Float4 offsetRot)
{
    DescPtr<BoundingBoxPhysxShapeDesc> d = allocateDesc<BoundingBoxPhysxShapeDesc>(ctx.descriptorAllocator());
    d->halfExtents = halfExtents;
    d->positionOffset = offsetPos;
    d->rotationOffset = offsetRot;
    if (!fillCommonShape(ctx, key, info, *d))
        return {};
    return d;
}

} // namespace omni::physics::parse
