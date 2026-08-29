// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-SHAPE-001
 * @covers AC-3 AC-4
 *
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-1
 */

// Simple-shape parsers — per-type construction plus the shared
// fillPhysxShapeDesc overlay. Mesh shapes and their cooking-service
// invocation live in the consumer-side walker, not here.

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/KnownTokens.h>

#include <algorithm>

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
    setToDefault(desc, ctx.units());

    desc.collisionEnabled = info.collisionEnabled;
    desc.collisionGroup   = info.collisionGroup;
    desc.rigidBody        = info.rigidBody;
    desc.localPos         = info.localPos;
    desc.localRot         = info.localRot;
    desc.localScale       = info.localScale;
    desc.sourceGprim      = info.sourceGprim;

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
    parseCollisionExt(ctx, key, fields);
    desc.contactOffset            = fields.contactOffset;
    desc.restOffset               = fields.restOffset;
    desc.torsionalPatchRadius     = fields.torsionalPatchRadius;
    desc.minTorsionalPatchRadius  = fields.minTorsionalPatchRadius;
    desc.isTrigger                = fields.isTrigger;
    desc.isTriggerUsdOutput       = fields.isTriggerUsdOutput;

    // Resolve simulationOwners → sceneIds (scene category).
    for (ObjectKey owner : info.simulationOwners)
    {
        ObjectId id = ctx.objects().findEntry(owner, eScene);
        if (id != kInvalidObjectId)
            desc.sceneIds.push_back(id);
    }
    if (!info.simulationOwners.empty() && desc.sceneIds.empty())
        return false;

    return true;
}

// Reads `physxConvexGeometry:margin` (Cylinder/Cone shapes). Gated on
// HasAuthoredValue and clamped to [0, +inf).
void readConvexGeometryMargin(ParseContext& ctx, ObjectKey key, float& margin)
{
    IPhysicsSource& src = ctx.source();
    KnownTokens tok;
    tok.intern(src);

    if (!src.hasAuthoredAttribute(key, tok.physxConvexGeometryMargin))
        return;
    float m;
    if (!src.getAttribute(key, tok.physxConvexGeometryMargin, m))
        return;
    margin = std::max(0.0f, m);
}

} // namespace

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
    KnownTokens tok;
    tok.intern(src);

    //   - When PhysicsMeshCollisionAPI is not applied, default to "none".
    //   - When applied, read `physics:approximation` (schema fallback is
    //     also "none"), then map to the enum.
    if (!src.hasSchema(key, tok.usdPhysicsMeshCollisionAPI))
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
