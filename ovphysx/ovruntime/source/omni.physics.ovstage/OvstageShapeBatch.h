// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BATCH-001
 * @covers AC-1 AC-2 AC-3
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3 AC-5
 */
#pragma once
#include <omni/physics/parse/DescriptorBatchAllocator.h>
#include <omni/physics/parse/PreparedShape.h>
#include <variant>
#include <cmath>
#include <stdexcept>

namespace omni::physics::ovstage
{
using namespace omni::physics::parse;

using PreparedShapeCooking = std::variant<std::monostate, PreparedConvexHullCookingExt,
    PreparedConvexDecompositionCookingExt, PreparedSphereFillCookingExt,
    PreparedTriangleMeshCookingExt, PreparedTriangleMeshSimplificationCookingExt>;

struct PreparedOvstageShape
{
    ObjectType type = eUndefined;
    ObjectKey collider, geometry;
    Matrix4d world, relative, bodyWorld;
    bool relativePose = false;
    PreparedShapeCommon common;
    carb::Float3 geometryScale{ 1.0f, 1.0f, 1.0f };
    double size = 1.0, radius = 1.0, height = 2.0;
    Axis axis = eX;
    size_t customHash = 0;
    PreparedAttribute<float> convexMargin;
    bool doubleSided = false;
    bool sdf = false;
    PreparedShapeCooking cooking;
    PreparedSdfMeshCookingExt sdfCooking;
    BufferSpan<carb::Float3> points;
    MergeMeshDesc* mergedMesh = nullptr;
    std::vector<ObjectKey> materials, filteredCollisions, simulationOwners;
};

struct ShapeColumnTokens
{
    TokenId enabled, size, radius, height, axis, doubleSided, axisX, axisY;
    std::array<TokenId, 5> geometryAttributes() const { return {size, radius, height, axis, doubleSided}; }
};

template <typename Reader>
void readOvstageShapeAttributes(const Reader& collider, const Reader& geometry,
                               const KnownTokens& tok, const ShapeColumnTokens& fields,
                               PreparedOvstageShape& input)
{
    collider.getAttribute(input.collider, fields.enabled, input.common.info.collisionEnabled);
    readCollisionExtAttributes(collider, input.collider, tok, input.common.collision);
    if (input.type == eBoxShape) geometry.getAttribute(input.geometry, fields.size, input.size);
    if (input.type == eSphereShape || input.type == eCapsuleShape || input.type == eCylinderShape || input.type == eConeShape)
        geometry.getAttribute(input.geometry, fields.radius, input.radius);
    if (input.type == eCapsuleShape || input.type == eCylinderShape || input.type == eConeShape)
        geometry.getAttribute(input.geometry, fields.height, input.height);
    if (input.type == ePlaneShape || input.type == eCapsuleShape || input.type == eCylinderShape || input.type == eConeShape)
    {
        TokenId axis;
        geometry.getAttribute(input.geometry, fields.axis, axis);
        input.axis = axis == fields.axisX ? eX : axis == fields.axisY ? eY : eZ;
    }
    if (input.type == eCylinderShape || input.type == eConeShape)
        input.convexMargin = prepareAttribute<float>(collider, input.collider, tok.physxConvexGeometryMargin, true);
    geometry.getAttribute(input.geometry, fields.doubleSided, input.doubleSided);
    std::visit([&](auto& cooking)
    {
        using T = std::decay_t<decltype(cooking)>;
        if constexpr (std::is_same_v<T, PreparedConvexHullCookingExt>)
            readConvexHullCookingExtAttributes(collider, input.collider, tok, cooking);
        else if constexpr (std::is_same_v<T, PreparedConvexDecompositionCookingExt>)
            readConvexDecompositionCookingExtAttributes(collider, input.collider, tok, cooking);
        else if constexpr (std::is_same_v<T, PreparedSphereFillCookingExt>)
            readSphereFillCookingExtAttributes(collider, input.collider, tok, cooking);
        else if constexpr (std::is_same_v<T, PreparedTriangleMeshCookingExt>)
            readTriangleMeshCookingExtAttributes(collider, input.collider, tok, cooking);
        else if constexpr (std::is_same_v<T, PreparedTriangleMeshSimplificationCookingExt>)
            readTriangleMeshSimplificationCookingExtAttributes(collider, input.collider, tok, cooking);
    }, input.cooking);
    if (input.sdf) readSdfMeshCookingExtAttributes(collider, input.collider, tok, input.sdfCooking);
}

template <typename T>
void constructShapeType(DescriptorBatchAllocator& pool, ObjectType type,
                        const std::vector<PreparedOvstageShape>& inputs,
                        std::vector<DescPtr<PhysxShapeDesc>>& outputs)
{
    const size_t count = size_t(std::count_if(inputs.begin(), inputs.end(),
        [=](const PreparedOvstageShape& input) { return input.type == type; }));
    if (!count) return;
    std::vector<DescPtr<PhysxShapeDesc>> batch = allocateDescriptorBatch<T, PhysxShapeDesc>(pool, count);
    size_t slot = 0;
    for (size_t index = 0; index < inputs.size(); ++index)
        if (inputs[index].type == type)
            outputs[index] = std::move(batch[slot++]);
}

inline void constructShapeBatches(DescriptorBatchAllocator& pool, std::vector<PreparedOvstageShape>& inputs,
                                  std::vector<DescPtr<PhysxShapeDesc>>& outputs,
                                  std::vector<DescPtr<MergeMeshDesc>>& meshes)
{
    outputs.resize(inputs.size());
    constructShapeType<SpherePhysxShapeDesc>(pool, eSphereShape, inputs, outputs);
    constructShapeType<BoxPhysxShapeDesc>(pool, eBoxShape, inputs, outputs);
    constructShapeType<CapsulePhysxShapeDesc>(pool, eCapsuleShape, inputs, outputs);
    constructShapeType<CylinderPhysxShapeDesc>(pool, eCylinderShape, inputs, outputs);
    constructShapeType<ConePhysxShapeDesc>(pool, eConeShape, inputs, outputs);
    constructShapeType<PlanePhysxShapeDesc>(pool, ePlaneShape, inputs, outputs);
    constructShapeType<CustomPhysxShapeDesc>(pool, eCustomShape, inputs, outputs);
    constructShapeType<ConvexMeshPhysxShapeDesc>(pool, eConvexMeshShape, inputs, outputs);
    constructShapeType<ConvexMeshDecompositionPhysxShapeDesc>(pool, eConvexMeshDecompositionShape, inputs, outputs);
    constructShapeType<TriangleMeshPhysxShapeDesc>(pool, eTriangleMeshShape, inputs, outputs);
    constructShapeType<SpherePointsPhysxShapeDesc>(pool, eSpherePointsShape, inputs, outputs);
    constructShapeType<BoundingSpherePhysxShapeDesc>(pool, eBoundingSphereShape, inputs, outputs);
    constructShapeType<BoundingBoxPhysxShapeDesc>(pool, eBoundingBoxShape, inputs, outputs);
    const size_t mergedCount = size_t(std::count_if(inputs.begin(), inputs.end(), [](const PreparedOvstageShape& input)
    { return input.type == eBoundingSphereShape || input.type == eBoundingBoxShape; }));
    meshes = allocateDescriptorBatch<MergeMeshDesc>(pool, mergedCount);
    size_t mesh = 0;
    for (size_t index = 0; index < inputs.size(); ++index)
    {
        PreparedOvstageShape& input = inputs[index];
        if (!outputs[index]) throw std::logic_error("unclassified shape batch slot");
        PhysxShapeDesc& output = *outputs[index];
        output.sceneIds.resize(input.common.sceneIds.size());
        output.sourceMaterials.resize(input.materials.size());
        output.sourceFilteredCollisions.resize(input.filteredCollisions.size());
        output.sourceSimulationOwners.resize(input.simulationOwners.size());
        if (input.type == eBoundingSphereShape || input.type == eBoundingBoxShape)
        {
            input.mergedMesh = meshes[mesh++].get();
            input.mergedMesh->points.resize(input.points.count);
        }
    }
}

template <typename T>
void fillRoundGeometry(const PreparedOvstageShape& input, T& output) noexcept
{
    const carb::Float3 scale = input.geometryScale;
    float radialScale, axialScale;
    if (input.axis == eX) { axialScale = scale.x; radialScale = std::max(std::fabs(scale.y), std::fabs(scale.z)); }
    else if (input.axis == eY) { axialScale = scale.y; radialScale = std::max(std::fabs(scale.x), std::fabs(scale.z)); }
    else { axialScale = scale.z; radialScale = std::max(std::fabs(scale.x), std::fabs(scale.y)); }
    output.radius = std::fabs(static_cast<float>(input.radius) * radialScale);
    output.halfHeight = std::fabs(static_cast<float>(input.height) * 0.5f * axialScale);
    output.axis = input.axis;
}

inline bool fillOvstageShape(const PreparedOvstageShape& input, PhysxShapeDesc& output) noexcept
{
    if (output.type != input.type || output.sourceMaterials.size() != input.materials.size() ||
        output.sourceFilteredCollisions.size() != input.filteredCollisions.size() ||
        output.sourceSimulationOwners.size() != input.simulationOwners.size() ||
        !fillShapeCommon(input.common, output)) return false;
    output.primKey = input.collider;
    std::copy(input.materials.begin(), input.materials.end(), output.sourceMaterials.begin());
    std::copy(input.filteredCollisions.begin(), input.filteredCollisions.end(), output.sourceFilteredCollisions.begin());
    std::copy(input.simulationOwners.begin(), input.simulationOwners.end(), output.sourceSimulationOwners.begin());
    if (input.points.count && !input.points.data) return false;
    switch (input.type)
    {
    case eSphereShape:
        static_cast<SpherePhysxShapeDesc&>(output).radius = static_cast<float>(std::abs(input.radius)) *
            std::max(std::max(std::fabs(input.geometryScale.x), std::fabs(input.geometryScale.y)), std::fabs(input.geometryScale.z));
        break;
    case eBoxShape:
    {
        const float half = static_cast<float>(std::abs(input.size)) * 0.5f;
        static_cast<BoxPhysxShapeDesc&>(output).halfExtents =
            { half * std::fabs(input.geometryScale.x), half * std::fabs(input.geometryScale.y), half * std::fabs(input.geometryScale.z) };
        break;
    }
    case eCapsuleShape:
        fillRoundGeometry(input, static_cast<CapsulePhysxShapeDesc&>(output));
        break;
    case eCylinderShape:
    {
        CylinderPhysxShapeDesc& shape = static_cast<CylinderPhysxShapeDesc&>(output);
        fillRoundGeometry(input, shape);
        if (input.convexMargin.valid) shape.margin = std::max(0.0f, input.convexMargin.value);
        break;
    }
    case eConeShape:
    {
        ConePhysxShapeDesc& shape = static_cast<ConePhysxShapeDesc&>(output);
        fillRoundGeometry(input, shape);
        if (input.convexMargin.valid) shape.margin = std::max(0.0f, input.convexMargin.value);
        break;
    }
    case ePlaneShape:
        static_cast<PlanePhysxShapeDesc&>(output).axis = input.axis;
        break;
    case eCustomShape:
        static_cast<CustomPhysxShapeDesc&>(output).customGeometryTokenHash = input.customHash;
        break;
    case eConvexMeshShape:
    {
        ConvexMeshPhysxShapeDesc& shape = static_cast<ConvexMeshPhysxShapeDesc&>(output);
        shape.meshScale = input.geometryScale;
        shape.meshPrimKey = input.common.info.sourceGprim;
        shape.convexCookingParams.signScale = scaleToSignScale(input.geometryScale);
        const PreparedConvexHullCookingExt* cooking = std::get_if<PreparedConvexHullCookingExt>(&input.cooking);
        if (!cooking || !fillConvexHullCookingExt(*cooking, shape.convexCookingParams)) return false;
        break;
    }
    case eTriangleMeshShape:
    case eConvexMeshDecompositionShape:
    case eSpherePointsShape:
    {
        TriangleMeshPhysxShapeDesc& shape = static_cast<TriangleMeshPhysxShapeDesc&>(output);
        shape.meshScale = input.geometryScale;
        shape.meshPrimKey = input.common.info.sourceGprim;
        shape.doubleSided = input.doubleSided;
        if (!input.sdf) shape.sdfMeshCookingParams.sdfResolution = 0;
        if (input.type == eConvexMeshDecompositionShape)
        {
            ConvexMeshDecompositionPhysxShapeDesc& decomposition = static_cast<ConvexMeshDecompositionPhysxShapeDesc&>(output);
            decomposition.convexDecompositionCookingParams.signScale = scaleToSignScale(input.geometryScale);
            const PreparedConvexDecompositionCookingExt* cooking = std::get_if<PreparedConvexDecompositionCookingExt>(&input.cooking);
            if (!cooking || !fillConvexDecompositionCookingExt(*cooking, decomposition.convexDecompositionCookingParams)) return false;
        }
        else if (input.type == eSpherePointsShape)
        {
            SpherePointsPhysxShapeDesc& spheres = static_cast<SpherePointsPhysxShapeDesc&>(output);
            spheres.sphereFillCookingParams.signScale = scaleToSignScale(input.geometryScale);
            const PreparedSphereFillCookingExt* cooking = std::get_if<PreparedSphereFillCookingExt>(&input.cooking);
            if (!cooking || !fillSphereFillCookingExt(*cooking, spheres.sphereFillCookingParams)) return false;
        }
        else if (const PreparedTriangleMeshCookingExt* cooking = std::get_if<PreparedTriangleMeshCookingExt>(&input.cooking))
        {
            if (!fillTriangleMeshCookingExt(*cooking, shape.triangleMeshCookingParams)) return false;
        }
        else if (const PreparedTriangleMeshSimplificationCookingExt* cooking = std::get_if<PreparedTriangleMeshSimplificationCookingExt>(&input.cooking))
        {
            shape.triangleMeshCookingParams.mode = omni::physx::TriangleMeshMode::eQUADRIC_SIMPLIFICATION;
            if (!fillTriangleMeshSimplificationCookingExt(*cooking, shape.triangleMeshCookingParams)) return false;
        }
        else return false;
        // An absent/zero SDF resolution preserves the existing descriptor defaults.
        if (input.sdf) fillSdfMeshCookingExt(input.sdfCooking, shape.sdfMeshCookingParams);
        break;
    }
    case eBoundingSphereShape:
    case eBoundingBoxShape:
    {
        if (!input.mergedMesh || input.mergedMesh->points.size() != input.points.count) return false;
        for (size_t index = 0; index < input.points.count; ++index)
        {
            const carb::Float3& point = input.points[index];
            input.mergedMesh->points[index] = { point.x * input.geometryScale.x,
                point.y * input.geometryScale.y, point.z * input.geometryScale.z };
        }
        static_cast<MergeMeshPhysxShapeDesc&>(output).mergedMesh = input.mergedMesh;
        break;
    }
    default: return false;
    }
    return true;
}

} // namespace omni::physics::ovstage
