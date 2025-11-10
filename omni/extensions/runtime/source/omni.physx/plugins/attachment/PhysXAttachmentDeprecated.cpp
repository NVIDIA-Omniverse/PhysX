// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXAttachmentDeprecated.h"
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

static const TfToken physxDeformableBodyTetMeshCrcToken{ "physxDeformable:tetMeshCrc" };
static const TfToken physxDeformableSurfaceInputCrcToken{ "physxDeformableSurface:clothDataInputCrc" };
static const TfToken physxParticleClothInputCrcToken{ "physxParticle:clothDataInputCrc" };
static const TfToken physxAttachmentInputCrcToken{ "physxAttachment:inputCrc" };
static const TfToken physxAttachmentDistanceAxesToken{ "physxAttachment:distanceAxes" };
static const TfToken physxAutoAttachmentMaskShapesToken{ "physxAutoAttachment:maskShapes" };

namespace carb
{

struct Int2_hash
{
    std::size_t operator()(carb::Int2 const& val) const
    {
        return std::hash<int32_t>()(val.x) ^ std::hash<int32_t>()(val.y);
    }
};

extern bool operator==(const carb::Int2& lhs, const carb::Int2& rhs);

} // namespace carb

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
        std::vector<uint32_t> culledTetVertIndices(culledTetIds.size()*4);
        for (uint32_t i = 0; i < culledTetIds.size(); ++i)
        {
            culledTetVertIndices[i*4 + 0] = indices[culledTetIds[i]*4 + 0];
            culledTetVertIndices[i*4 + 1] = indices[culledTetIds[i]*4 + 1];
            culledTetVertIndices[i*4 + 2] = indices[culledTetIds[i]*4 + 2];
            culledTetVertIndices[i*4 + 3] = indices[culledTetIds[i]*4 + 3];
        }

        uint32_t pointsSize, pointsByteStride;
        const carb::Float3* points = tetfinder::getPoints(pointsSize, pointsByteStride, tetFinder);
        return tetfinder::createTetFinder(points, pointsSize, pointsByteStride, culledTetVertIndices.data(), uint32_t(culledTetVertIndices.size()));
    }

} // namespace

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

struct UserDataInfo
{
    const UsdPrim& deformablePrim;
    const DeformableMeshInfoDeprecated& deformableMeshInfo;
    const UsdPrim& attachmentPrim;
    const PxTransform& rigidActorTransform;
    const PxVec3& rigidActorScale;
    const uint32_t actorSlot[2];
    const PhysxAutoAttachmentDesc& autoAttachmentDesc;
    const MaskShapes& maskShapes;
};

void updateDeformableRigidAttachments(const PxGeometry& geom, const PxTransform& geomPos, const PxVec3& scale, const PxTransform* shapeTransform, void* userData)
{
    const UserDataInfo* info = (const UserDataInfo*)userData;
    const UsdPrim& deformablePrim = info->deformablePrim;
    const DeformableMeshInfoDeprecated& deformableMeshInfo = info->deformableMeshInfo;
    const UsdPrim& attachmentPrim = info->attachmentPrim;
    const PxTransform& transform = info->rigidActorTransform;
    const PxVec3& invScale = PxVec3(1.0f / info->rigidActorScale.x, 1.0f / info->rigidActorScale.y, 1.0f / info->rigidActorScale.z);
    const uint32_t actorSlot[2] = { info->actorSlot[0], info->actorSlot[1] };

    UsdGeomBBoxCache cacheBBox(UsdTimeCode::EarliestTime(), { UsdGeomTokens->default_, UsdGeomTokens->proxy});

    const GfRange3d sb_bounds = cacheBBox.ComputeWorldBound(deformablePrim).ComputeAlignedRange();
    const GfVec3d sb_size = sb_bounds.GetSize();
    PxU32 num_edges0 = 0;
    num_edges0 += sb_size[0] == 0.0 ? 0 : 1;
    num_edges0 += sb_size[1] == 0.0 ? 0 : 1;
    num_edges0 += sb_size[2] == 0.0 ? 0 : 1;
    float avg_dim0 = (float)(sb_size[0] + sb_size[1] + sb_size[2]) / num_edges0;

    float avg_dim1 = PX_MAX_REAL;
    if (geom.getType() != PxGeometryType::ePLANE)
    {
        PxBounds3 bounds;
        PxGeometryQuery::computeGeomBounds(bounds, geom, geomPos, 0.0f, 1.01f);

        PxVec3 dimensions = bounds.getDimensions();
        PxU32 num_edges1 = 0;
        num_edges1 += dimensions.x == 0.0 ? 0 : 1;
        num_edges1 += dimensions.y == 0.0 ? 0 : 1;
        num_edges1 += dimensions.z == 0.0 ? 0 : 1;
        avg_dim1 = (dimensions.x + dimensions.y + dimensions.z) / num_edges1;
    }

    // Use the minimum average dimension
    float avg_dim = PxMin(avg_dim0, avg_dim1);
    float default_rad = avg_dim * 0.05f;

#if 0
    const PxVec3 invScaleShape = PxVec3(1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z);
#endif

    VtArray<GfVec3f> attachmentPointsDeformable;
    VtArray<GfVec3f> attachmentPointsRigidBody;
    VtArray<uint32_t> filterIds;

    PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(attachmentPrim.GetStage(), attachmentPrim.GetPath());
    attachment.GetPointsAttr(actorSlot[0]).Get(&attachmentPointsDeformable);
    attachment.GetCollisionFilterIndicesAttr(actorSlot[0]).Get(&filterIds);
    attachment.GetPointsAttr(actorSlot[1]).Get(&attachmentPointsRigidBody);

    // Get auto attachment attributes
    bool enableDeformableVertexAttachments = info->autoAttachmentDesc.enableDeformableVertexAttachments;
    float deformableVertexOverlapOffset = info->autoAttachmentDesc.deformableVertexOverlapOffset;
    bool enableRigidSurfaceAttachments = info->autoAttachmentDesc.enableRigidSurfaceAttachments;
    float rigidSurfaceSamplingDistance = info->autoAttachmentDesc.rigidSurfaceSamplingDistance;
    bool enableCollisionFiltering = info->autoAttachmentDesc.enableCollisionFiltering;
    float collisionFilteringOffset = info->autoAttachmentDesc.collisionFilteringOffset;

    // Apply default heuristics
    if (!isfinite(rigidSurfaceSamplingDistance))
        rigidSurfaceSamplingDistance = default_rad;

    if (!isfinite(collisionFilteringOffset))
        collisionFilteringOffset = default_rad * 2;

    PxSphereGeometry defaultVertexAttachmentSphere(deformableVertexOverlapOffset);
    PxSphereGeometry defaultFilteringSphere(collisionFilteringOffset);

#if 0
    PxTransform transform;
    if (shapeTransform)
        transform = *shapeTransform;
    else
        transform = geomPos;
#endif

    bool isTriangleMesh = geom.getType() == PxGeometryType::eTRIANGLEMESH;

    uint64_t tetFinderCollisionPositions = 0;
    uint64_t tetFinderRestPositions = 0;

    if (deformableMeshInfo.type == AttachmentActorTypeDeprecated::eDEFORMABLE_BODY)
    {
        tetFinderCollisionPositions = omni::tetfinder::createTetFinder(
            &deformableMeshInfo.positions[0], uint32_t(deformableMeshInfo.positions.size()),
            &deformableMeshInfo.indices[0], uint32_t(deformableMeshInfo.indices.size()));

        tetFinderRestPositions = omni::tetfinder::createTetFinder(
            &deformableMeshInfo.restPositions[0], uint32_t(deformableMeshInfo.restPositions.size()),
            &deformableMeshInfo.indices[0], uint32_t(deformableMeshInfo.indices.size()));
    }

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
            vertex = geomPos.transform(vertex);

            triangleVertices.push_back(vertex);
        }

        triMeshSampler = PxCreateTriangleMeshSampler(trianglePtr, triMesh.triangleMesh->getNbTriangles(), &triangleVertices[0], triMesh.triangleMesh->getNbVertices(), 30);
    }

    // Surface sampling on rigid body
    if (enableRigidSurfaceAttachments && deformableMeshInfo.type == AttachmentActorTypeDeprecated::eDEFORMABLE_BODY && rigidSurfaceSamplingDistance > 0.0f)
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

    // Vertex overlaps
    if (enableDeformableVertexAttachments)
    {
        std::vector<carb::Float3> culledVertices;
        std::vector<uint32_t> culledVertexIndices;
        cullPointsToMaskShapes(culledVertices, culledVertexIndices, info->maskShapes, 0.0f,
            deformableMeshInfo.positions.data(), uint32_t(deformableMeshInfo.positions.size()));

        for (PxU32 i = 0; i < culledVertices.size(); i++)
        {
            const uint32_t particleIndex = culledVertexIndices[i];
            const PxVec3 particlePos = toPhysX(culledVertices[i]);

            bool attachmentHit = PxGeometryQuery::overlap(geom, geomPos, defaultVertexAttachmentSphere, PxTransform(particlePos));

            if (!attachmentHit && isTriangleMesh && triMeshSampler)
                attachmentHit = triMeshSampler->isPointInTriangleMesh(particlePos);

            if (!attachmentHit)
                continue;

            PxVec3 localPos = toPhysX(deformableMeshInfo.restPositions[particleIndex]);
            attachmentPointsDeformable.push_back(GfVec3f(localPos.x, localPos.y, localPos.z));

            const PxVec3 actorPos = transform.transformInv(particlePos).multiply(invScale);
            attachmentPointsRigidBody.push_back(GfVec3f(actorPos.x, actorPos.y, actorPos.z));
        }
    }

    // Filtering
    if (enableCollisionFiltering)
    {
        if (deformableMeshInfo.type == AttachmentActorTypeDeprecated::eDEFORMABLE_BODY)
        {
            std::vector<uint32_t> filterList;
            filterList.assign(filterIds.begin(), filterIds.end());

            if (info->maskShapes.geometries.empty())
            {
                ResultBuffer<int32_t> tetIds;
                omni::tetfinder::overlapTetMeshGeom(tetIds.ptr, tetIds.size, tetFinderCollisionPositions, geom, geomPos, collisionFilteringOffset, ResultBuffer<>::allocate, triMeshSampler);

                filterList.insert(filterList.end(), reinterpret_cast<uint32_t*>(tetIds.ptr), reinterpret_cast<uint32_t*>(tetIds.ptr) + tetIds.size);
            }
            else
            {
                std::vector<uint32_t> culledTetIds;
                uint64_t culledTetFinder = cullTetsToMaskShapes(culledTetIds, info->maskShapes, collisionFilteringOffset, tetFinderCollisionPositions);

                ResultBuffer<int32_t> tetIds;
                omni::tetfinder::overlapTetMeshGeom(tetIds.ptr, tetIds.size, culledTetFinder, geom, geomPos, collisionFilteringOffset, ResultBuffer<>::allocate, triMeshSampler);
                tetfinder::releaseTetFinder(culledTetFinder);

                filterList.reserve(filterList.size() + tetIds.size);
                for (uint32_t i = 0; i < tetIds.size; ++i)
                {
                    filterList.push_back(culledTetIds[tetIds.ptr[i]]);
                }
            }

            filterIds.assign(filterList.begin(), filterList.end());
        }
        else
        {
            std::vector<carb::Float3> culledVertices;
            std::vector<uint32_t> culledVertexIndices;
            cullPointsToMaskShapes(culledVertices, culledVertexIndices, info->maskShapes, collisionFilteringOffset,
                deformableMeshInfo.positions.data(), uint32_t(deformableMeshInfo.positions.size()));

            for (PxU32 i = 0; i < culledVertices.size(); i++)
            {
                const PxVec3 particlePos = toPhysX(culledVertices[i]);

                bool filterHit = PxGeometryQuery::overlap(geom, geomPos, defaultFilteringSphere, PxTransform(particlePos));

                if (!filterHit && isTriangleMesh && triMeshSampler)
                    filterHit = triMeshSampler->isPointInTriangleMesh(particlePos);

                if (!filterHit)
                    continue;

                filterIds.push_back(culledVertexIndices[i]);
            }
        }
    }

#if 0
    // Hack filtering to include all
    filterIds.clear();

    for (PxU32 i = 0; i < PxU32(deformableMeshInfo.indices.size() / 4); i++)
    {
        filterIds.push_back(i);
    }
#endif

    PX_DELETE(triMeshSampler);

    omni::tetfinder::releaseTetFinder(tetFinderCollisionPositions);
    omni::tetfinder::releaseTetFinder(tetFinderRestPositions);

    attachment.GetPointsAttr(actorSlot[0]).Set(attachmentPointsDeformable);
    attachment.GetCollisionFilterIndicesAttr(actorSlot[0]).Set(filterIds);

    TfToken filterType = (deformableMeshInfo.type == AttachmentActorTypeDeprecated::eDEFORMABLE_BODY) ? PhysxSchemaTokens.Get()->Geometry : PhysxSchemaTokens.Get()->Vertices;
    attachment.GetFilterTypeAttr(actorSlot[0]).Set(filterType);
    attachment.GetPointsAttr(actorSlot[1]).Set(attachmentPointsRigidBody);
}

bool parseMaskShapeDeprecated(PxGeometryHolder& geometryHolder, PxTransform& transform, UsdPrim usdPrim)
{
    UsdTimeCode timeCode = UsdTimeCode::EarliestTime();

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

void parseRigidBodyShapesDeprecated(const UsdPrim& rigidBodyPrim, const usdparser::PhysxShapeDesc* desc, getGeometryInfoCallbackDeprecated callbackFn, void* userData)
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
        callbackFn(sphereGeom, transform, toPhysX(desc->localScale), nullptr, userData);
    }
    break;
    case eBoxShape:
    {
        BoxPhysxShapeDesc* boxDesc = (BoxPhysxShapeDesc*)desc;
        PxBoxGeometry boxGeom((const PxVec3&)boxDesc->halfExtents);
        callbackFn(boxGeom, transform, toPhysX(desc->localScale), nullptr, userData);
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
        PxTransform capsuleTransform = transform;
        transform.q = transform.q * fixupQ;

        PxCapsuleGeometry capsuleGeom(capsuleDesc->radius, capsuleDesc->halfHeight);
        callbackFn(capsuleGeom, transform, toPhysX(desc->localScale), &capsuleTransform, userData);
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
        PxTransform planeTransform = transform;
        transform.q = transform.q * fixupQ;

        PxPlaneGeometry planeGeom;
        callbackFn(planeGeom, transform, toPhysX(desc->localScale), &planeTransform, userData);
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
            callbackFn(convexMeshGeom, transform, toPhysX(desc->localScale), nullptr, userData);
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
            callbackFn(convexMeshGeom, transform, toPhysX(desc->localScale), nullptr, userData);
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
            callbackFn(convexMeshGeom, transform, toPhysX(desc->localScale), nullptr, userData);
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
                callbackFn(convexMeshGeom, transform, toPhysX(desc->localScale), nullptr, userData);
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
            callbackFn(triangleMeshGeom, transform, toPhysX(desc->localScale), nullptr, userData);
        }
    }
    break;
    }
}

void updateDeformableDeformableAttachments(
    const UsdPrim& deformablePrim0, const DeformableMeshInfoDeprecated& deformableMeshInfo0,
    const UsdPrim& deformablePrim1, const DeformableMeshInfoDeprecated& deformableMeshInfo1,
    const UsdPrim& attachmentPrim, const uint32_t actorSlot[2],
    const PhysxAutoAttachmentDesc& autoAttachmentDesc, const MaskShapes& maskShapes)
{
    CARB_ASSERT(deformableMeshInfo0.type == AttachmentActorTypeDeprecated::eDEFORMABLE_BODY);
    UsdGeomBBoxCache cacheBBox(UsdTimeCode::EarliestTime(), { UsdGeomTokens->default_, UsdGeomTokens->proxy});

    const GfRange3d sb_bounds0 = cacheBBox.ComputeWorldBound(deformablePrim0).ComputeAlignedRange();
    const GfVec3d sb_size0 = sb_bounds0.GetSize();
    PxU32 num_edges0 = 0;
    num_edges0 += sb_size0[0] == 0.0 ? 0 : 1;
    num_edges0 += sb_size0[1] == 0.0 ? 0 : 1;
    num_edges0 += sb_size0[2] == 0.0 ? 0 : 1;
    float avg_dim0 = (float)(sb_size0[0] + sb_size0[1] + sb_size0[2]) / num_edges0;

    const GfRange3d sb_bounds1 = cacheBBox.ComputeWorldBound(deformablePrim1).ComputeAlignedRange();
    const GfVec3d sb_size1 = sb_bounds1.GetSize();
    PxU32 num_edges1 = 0;
    num_edges1 += sb_size1[0] == 0.0 ? 0 : 1;
    num_edges1 += sb_size1[1] == 0.0 ? 0 : 1;
    num_edges1 += sb_size1[2] == 0.0 ? 0 : 1;
    float avg_dim1 = (float)(sb_size1[0] + sb_size1[1] + sb_size1[2]) / num_edges1;

    // Use the minimum average dimension
    float avg_dim = PxMin(avg_dim0, avg_dim1);
    float default_rad = avg_dim * 0.05f;

    VtArray<GfVec3f> attachmentPointsDeformable[2];
    VtArray<uint32_t> filterIdsDeformable[2];

    // Anchor points and distances
    VtArray<GfVec3f> attachmentDistanceAxes;

    const DeformableMeshInfoDeprecated* deformableMeshInfo[2] = { &deformableMeshInfo0, &deformableMeshInfo1 };
    uint64_t tetFinderCollision[2] = {0, 0};
    uint64_t tetFinderRest[2] = {0, 0};

    for (uint32_t s = 0; s < 2; ++s)
    {
        if (deformableMeshInfo[s]->type == AttachmentActorTypeDeprecated::eDEFORMABLE_BODY)
        {
            tetFinderCollision[s] = omni::tetfinder::createTetFinder(
                &deformableMeshInfo[s]->positions[0], uint32_t(deformableMeshInfo[s]->positions.size()),
                &deformableMeshInfo[s]->indices[0], uint32_t(deformableMeshInfo[s]->indices.size()));

            tetFinderRest[s] = omni::tetfinder::createTetFinder(
                &deformableMeshInfo[s]->restPositions[0], uint32_t(deformableMeshInfo[s]->restPositions.size()),
                &deformableMeshInfo[s]->indices[0], uint32_t(deformableMeshInfo[s]->indices.size()));
        }
    }

    // Get auto attachment attributes
    bool enableDeformableVertexAttachments = autoAttachmentDesc.enableDeformableVertexAttachments;
    float deformableVertexOverlapOffset = autoAttachmentDesc.deformableVertexOverlapOffset;
    bool enableRigidSurfaceAttachments = autoAttachmentDesc.enableRigidSurfaceAttachments;
    float rigidSurfaceSamplingDistance = autoAttachmentDesc.rigidSurfaceSamplingDistance;
    bool enableCollisionFiltering = autoAttachmentDesc.enableCollisionFiltering;
    float collisionFilteringOffset = autoAttachmentDesc.collisionFilteringOffset;
    bool enableDeformableFilteringPairs = autoAttachmentDesc.enableDeformableFilteringPairs;

    // Apply default heuristics
    if (!isfinite(rigidSurfaceSamplingDistance))
        rigidSurfaceSamplingDistance = default_rad;

    if (!isfinite(collisionFilteringOffset))
        collisionFilteringOffset = default_rad * 2;

    // Vertex overlaps
    if (enableDeformableVertexAttachments)
    {
        for (uint32_t s = 0; s < 2; ++s)
        {
            const DeformableMeshInfoDeprecated& srcMeshInfo = *deformableMeshInfo[s];
            const DeformableMeshInfoDeprecated& dstMeshInfo = *deformableMeshInfo[1-s];

            if (dstMeshInfo.type != AttachmentActorTypeDeprecated::eDEFORMABLE_BODY)
            {
                continue;
            }

            uint64_t dstTetFinderCollision = tetFinderCollision[1-s];
            uint64_t dstTetFinderRest = tetFinderRest[1-s];

            //cull src points
            std::vector<carb::Float3> srcPoints;
            std::vector<uint32_t> srcPointIndices;
            cullPointsToMaskShapes(srcPoints, srcPointIndices, maskShapes, 0.0f,
                &srcMeshInfo.positions[0], uint32_t(srcMeshInfo.positions.size()));

            std::vector<int32_t> srcPointTetIds(srcPoints.size());
            std::vector<carb::Float4> srcPointTetBary(srcPoints.size());
            std::vector<carb::Float3> srcPointDistanceDirs(srcPoints.size());
            bool isSuccess = omni::tetfinder::pointsToTetMeshLocalClosest(srcPointTetIds.data(), srcPointTetBary.data(), srcPointDistanceDirs.data(), dstTetFinderCollision, srcPoints.data(), uint32_t(srcPoints.size()));
            if (isSuccess)
            {
                std::vector<carb::Float3> srcPointsInDstRest(srcPoints.size());
                omni::tetfinder::tetMeshLocalToPoints(srcPointsInDstRest.data(), dstTetFinderRest, srcPointTetIds.data(), srcPointTetBary.data(), uint32_t(srcPointsInDstRest.size()));

                for (PxU32 i = 0; i < srcPoints.size(); i++)
                {
                    PxVec3 srcPointDistanceDir = toPhysX(srcPointDistanceDirs[i]);
                    float separationSq = srcPointDistanceDir.magnitudeSquared();

                    if (separationSq <= deformableVertexOverlapOffset * deformableVertexOverlapOffset)
                    {
                        const carb::Float3& srcPointRest = srcMeshInfo.restPositions[srcPointIndices[i]];
                        const carb::Float3& srcPointInDstRest = srcPointsInDstRest[i];

                        if (separationSq == 0.0f)
                        {
                            attachmentPointsDeformable[s].push_back(GfVec3f(srcPointRest.x, srcPointRest.y, srcPointRest.z));
                            attachmentPointsDeformable[1 - s].push_back(GfVec3f(srcPointInDstRest.x, srcPointInDstRest.y, srcPointInDstRest.z));
                        }
                        else
                        {
                            if (s == 0)
                            {
                                attachmentPointsDeformable[1 - s].push_back(GfVec3f(srcPointInDstRest.x, srcPointInDstRest.y, srcPointInDstRest.z));

                                PxVec3 distancePoint = toPhysX(srcPointRest) - srcPointDistanceDir;
                                attachmentPointsDeformable[s].push_back(GfVec3f(distancePoint.x, distancePoint.y, distancePoint.z));
                            }
                            else
                            {
                                attachmentPointsDeformable[s].push_back(GfVec3f(srcPointRest.x, srcPointRest.y, srcPointRest.z));

                                PxVec3 distancePoint = toPhysX(srcPointInDstRest) + srcPointDistanceDir;
                                attachmentPointsDeformable[1 - s].push_back(GfVec3f(distancePoint.x, distancePoint.y, distancePoint.z));
                            }
                        }
                    }
                }
            }
        }
    }

    // Filtering
    if (enableCollisionFiltering)
    {
        if (deformableMeshInfo1.type == AttachmentActorTypeDeprecated::eDEFORMABLE_BODY)
        {
            if (maskShapes.geometries.empty())
            {
                //bi-directional overlap tet-mesh tet-mesh overlap
                ResultBuffer<carb::Int2> tetIdPairs;
                omni::tetfinder::overlapTetMeshTetMeshDeprecated(tetIdPairs.ptr, tetIdPairs.size, tetFinderCollision[0], tetFinderCollision[1],
                                                                 collisionFilteringOffset, enableDeformableFilteringPairs, ResultBuffer<>::allocate);

                filterIdsDeformable[0].reserve(tetIdPairs.size);
                filterIdsDeformable[1].reserve(tetIdPairs.size);
                for (PxU32 t = 0; t < tetIdPairs.size; ++t)
                {
                    filterIdsDeformable[0].push_back(tetIdPairs.ptr[t].x);
                    filterIdsDeformable[1].push_back(tetIdPairs.ptr[t].y);
                }
            }
            else
            {
                //we need to cull each tet mesh separately and test against other full tet mesh,
                //then combine the resulting filter pairs
                std::unordered_set<carb::Int2, Int2_hash> uniquePairs;
                for (uint32_t s = 0; s < 2; ++s)
                {
                    const DeformableMeshInfoDeprecated& srcMeshInfo = *deformableMeshInfo[s];
                    const DeformableMeshInfoDeprecated& dstMeshInfo = *deformableMeshInfo[1-s];
                    uint64_t srcTetFinderCollision = tetFinderCollision[s];
                    uint64_t dstTetFinderCollision = tetFinderCollision[1-s];

                    //cull src tets
                    std::vector<uint32_t> culledSrcTetIds;
                    uint64_t culledSrcTetFinderCollision = cullTetsToMaskShapes(culledSrcTetIds, maskShapes, collisionFilteringOffset, srcTetFinderCollision);

                    ResultBuffer<carb::Int2> tetIdPairs;
                    omni::tetfinder::overlapTetMeshTetMeshDeprecated(tetIdPairs.ptr, tetIdPairs.size, culledSrcTetFinderCollision, dstTetFinderCollision,
                                                                     collisionFilteringOffset, enableDeformableFilteringPairs, ResultBuffer<>::allocate);

                    uniquePairs.reserve(uniquePairs.size() + tetIdPairs.size);
                    for (uint32_t t = 0; t < tetIdPairs.size; ++t)
                    {
                        carb::Int2 pair = tetIdPairs.ptr[t];
                        if (pair.x != -1)
                        {
                            pair.x = culledSrcTetIds[pair.x]; //convert tetId from culled mesh back to original
                        }
                        if (s == 1)
                        {
                            std::swap(pair.x, pair.y); //adjust for output deformable ordering
                        }
                        uniquePairs.insert(pair);
                    }

                    tetfinder::releaseTetFinder(culledSrcTetFinderCollision);
                }

                filterIdsDeformable[0].reserve(uniquePairs.size());
                filterIdsDeformable[1].reserve(uniquePairs.size());
                for (const carb::Int2& pair : uniquePairs)
                {
                    filterIdsDeformable[0].push_back(pair.x);
                    filterIdsDeformable[1].push_back(pair.y);
                }
            }
        }
        else
        {
            const DeformableMeshInfoDeprecated& srcMeshInfo = *deformableMeshInfo[1];
            const DeformableMeshInfoDeprecated& dstMeshInfo = *deformableMeshInfo[0];
            const uint64_t dstTetFinder = tetFinderCollision[0];

            //cull src points
            std::vector<carb::Float3> srcPoints;
            std::vector<uint32_t> srcPointIndices;
            cullPointsToMaskShapes(srcPoints, srcPointIndices, maskShapes, collisionFilteringOffset,
                srcMeshInfo.positions.data(), uint32_t(srcMeshInfo.positions.size()));

            if (enableDeformableFilteringPairs)
            {
                for (PxU32 i = 0; i < srcPoints.size(); i++)
                {
                    ResultBuffer<int32_t> srcPointTetIds;
                    omni::tetfinder::overlapTetMeshSphere(srcPointTetIds.ptr, srcPointTetIds.size, dstTetFinder, srcPoints[i], collisionFilteringOffset, ResultBuffer<>::allocate);

                    for (uint32_t t = 0; t < srcPointTetIds.size; ++t)
                    {
                        filterIdsDeformable[0].push_back(srcPointTetIds.ptr[t]);
                        filterIdsDeformable[1].push_back(srcPointIndices[i]);
                    }
                }
            }
            else
            {
                std::vector<int32_t> srcPointTetIds(srcPoints.size());
                std::vector<carb::Float4> srcPointTetBarys(srcPoints.size());
                std::vector<carb::Float3> srcPointDistanceDirs(srcPoints.size());
                bool isSuccess = omni::tetfinder::pointsToTetMeshLocalClosest(srcPointTetIds.data(), srcPointTetBarys.data(), srcPointDistanceDirs.data(), dstTetFinder, srcPoints.data(), uint32_t(srcPoints.size()));
                if (isSuccess)
                {
                    for (PxU32 i = 0; i < srcPoints.size(); i++)
                    {
                        PxVec3 srcPointDistanceDir = toPhysX(srcPointDistanceDirs[i]);
                        float separationSq = srcPointDistanceDir.magnitudeSquared();

                        if (separationSq <= collisionFilteringOffset * collisionFilteringOffset)
                        {
                            filterIdsDeformable[0].push_back(uint32_t(-1));
                            filterIdsDeformable[1].push_back(srcPointIndices[i]);
                        }
                    }
                }
            }
        }
    }

    omni::tetfinder::releaseTetFinder(tetFinderCollision[0]);
    omni::tetfinder::releaseTetFinder(tetFinderRest[0]);
    omni::tetfinder::releaseTetFinder(tetFinderCollision[1]);
    omni::tetfinder::releaseTetFinder(tetFinderRest[1]);

    PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(attachmentPrim.GetStage(), attachmentPrim.GetPath());
    attachment.GetPointsAttr(actorSlot[0]).Set(attachmentPointsDeformable[0]);
    attachment.GetCollisionFilterIndicesAttr(actorSlot[0]).Set(filterIdsDeformable[0]);

    attachment.GetFilterTypeAttr(actorSlot[0]).Set(PhysxSchemaTokens.Get()->Geometry);
    attachment.GetPointsAttr(actorSlot[1]).Set(attachmentPointsDeformable[1]);
    attachment.GetCollisionFilterIndicesAttr(actorSlot[1]).Set(filterIdsDeformable[1]);

    TfToken filterType1 = (deformableMeshInfo1.type == AttachmentActorTypeDeprecated::eDEFORMABLE_BODY) ? PhysxSchemaTokens.Get()->Geometry : PhysxSchemaTokens.Get()->Vertices;
    attachment.GetFilterTypeAttr(actorSlot[1]).Set(filterType1);

    bool hasAttachmentAtDistance = false;
    for (uint32_t i = 0; i < uint32_t(attachmentDistanceAxes.size()); ++i)
    {
        if (attachmentDistanceAxes[i] != GfVec3f(0.0f))
        {
            hasAttachmentAtDistance = true;
            break;
        }
    }
    if (hasAttachmentAtDistance)
    {
        // Save attachment distance axes only if distance based constraints are used
        attachmentPrim.CreateAttribute(physxAttachmentDistanceAxesToken, SdfValueTypeNames->Point3fArray).Set(attachmentDistanceAxes);
    }
    else
    {
        // Remove the attribute if it already exists
        ((UsdPrim)attachmentPrim).RemoveProperty(physxAttachmentDistanceAxesToken);
    }
}

void updateDeformableWorldAttachments(const UsdPrim deformablePrim, const DeformableMeshInfoDeprecated& deformableMeshInfo, UsdPrim attachmentPrim, const uint32_t actorSlot,
    const PhysxAutoAttachmentDesc& autoAttachmentDesc, const MaskShapes& maskShapes)
{
    CARB_ASSERT(deformableMeshInfo.type & AttachmentActorTypeDeprecated::eDEFORMABLE);

    VtArray<GfVec3f> attachmentPointsDeformable;
    VtArray<GfVec3f> attachmentPointsWorld;

    // Get auto attachment attributes
    bool enableDeformableVertexAttachments = autoAttachmentDesc.enableDeformableVertexAttachments;
    float deformableVertexOverlapOffset = autoAttachmentDesc.deformableVertexOverlapOffset;
    bool enableRigidSurfaceAttachments = autoAttachmentDesc.enableRigidSurfaceAttachments;
    float rigidSurfaceSamplingDistance = autoAttachmentDesc.rigidSurfaceSamplingDistance;
    bool enableCollisionFiltering = autoAttachmentDesc.enableCollisionFiltering;
    float collisionFilteringOffset = autoAttachmentDesc.collisionFilteringOffset;
    bool enableDeformableFilteringPairs = autoAttachmentDesc.enableDeformableFilteringPairs;

    // Vertex overlaps
    if (enableDeformableVertexAttachments)
    {
        //cull src points
        std::vector<carb::Float3> srcPoints;
        std::vector<uint32_t> srcPointIndices;
        cullPointsToMaskShapes(srcPoints, srcPointIndices, maskShapes, 0.0f,
            &deformableMeshInfo.positions[0], uint32_t(deformableMeshInfo.positions.size()));

        for (PxU32 i = 0; i < srcPoints.size(); i++)
        {
            const carb::Float3& srcPoint = srcPoints[i];
            const carb::Float3& srcPointRest = deformableMeshInfo.restPositions[srcPointIndices[i]];
            attachmentPointsDeformable.push_back(GfVec3f(srcPointRest.x, srcPointRest.y, srcPointRest.z));
            attachmentPointsWorld.push_back(GfVec3f(srcPoint.x, srcPoint.y, srcPoint.z));
        }
    }

    PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment(attachmentPrim);
    attachment.GetPointsAttr(actorSlot).Set(attachmentPointsDeformable);
    attachment.GetPointsAttr(1-actorSlot).Set(attachmentPointsWorld);
    attachment.GetCollisionFilterIndicesAttr(actorSlot).Clear();
    attachment.GetFilterTypeAttr(actorSlot).Clear();

    // Remove the attribute if it already exists
    attachmentPrim.RemoveProperty(physxAttachmentDistanceAxesToken);
}

bool getDeformableMeshInfoDeprecated(const SdfPath& deformablePath, DeformableMeshInfoDeprecated& deformableMeshInfo)
{
	UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    const uint64_t stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    UsdPrim deformablePrim = stage->GetPrimAtPath(deformablePath);    
    if (!deformablePrim)
    {
        return false;
    }
    
    CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if(!cookingDataAsync)
    {
        CARB_LOG_WARN("Deformable getDeformableMeshInfoDeprecated failed - Cooking not available");
        return false;
    }

    UsdGeomMesh usdGeomMesh(deformablePrim);

    if (deformablePrim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
    {
        // make sure simulation mesh is written by the async cooker
        cookingDataAsync->cookDeformableBodyTetMeshDeprecated(const_cast<UsdPrim&>(deformablePrim), false);
        SoftBodyDesc* softBodyDesc = parseDeformableBodyDeprecated(stageId, deformablePath);

        if (softBodyDesc)
        {
            deformableMeshInfo.type = AttachmentActorTypeDeprecated::eDEFORMABLE_BODY;

            const GfMatrix4d softbodyToWorld(usdGeomMesh.ComputeLocalToWorldTransform(UsdTimeCode::EarliestTime()));
            PxTransform softbodyVerticesToWorld;
            PxVec3 softbodyVerticesToWorldScale;
            toPhysX(softbodyVerticesToWorld, softbodyVerticesToWorldScale, softbodyToWorld);

            deformableMeshInfo.positions.resize(softBodyDesc->collisionPoints.size());
            for (size_t i = 0; i < softBodyDesc->collisionPoints.size(); ++i)
            {
                PxVec3 position = toPhysX(softBodyDesc->collisionPoints[i]).multiply(softbodyVerticesToWorldScale);
                position = softbodyVerticesToWorld.transform(position);
                deformableMeshInfo.positions[i] = { position.x, position.y, position.z };
            }

            deformableMeshInfo.restPositions.swap(softBodyDesc->collisionRestPoints);
            deformableMeshInfo.indices.swap(softBodyDesc->collisionIndices);

            // read crc
            deformableMeshInfo.meshCrc = loadMeshKey(deformablePrim, physxDeformableBodyTetMeshCrcToken);

            // store simulation owner
            deformableMeshInfo.scenePath = softBodyDesc->scenePath;

            ICE_FREE(softBodyDesc);

            return true;
        }
    }
    else if (deformablePrim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
    {
        FEMClothDesc* femClothDesc = parseDeformableSurfaceDeprecated(stageId, deformablePath);

        if (femClothDesc)
        {
            deformableMeshInfo.type = AttachmentActorTypeDeprecated::eDEFORMABLE_SURFACE;

            const GfMatrix4d clothToWorld(usdGeomMesh.ComputeLocalToWorldTransform(UsdTimeCode::EarliestTime()));

            deformableMeshInfo.positions.resize(femClothDesc->points.size());

            for (size_t i = 0; i < femClothDesc->points.size(); ++i)
            {
                GfVec3f position = { femClothDesc->points[i].x, femClothDesc->points[i].y, femClothDesc->points[i].z };
                position = clothToWorld.Transform(position);
                deformableMeshInfo.positions[i] = *(Float3*)position.data();
            }

            deformableMeshInfo.restPositions.swap(femClothDesc->restPoints);
            deformableMeshInfo.indices.swap(femClothDesc->simulationIndices);

            // read crc
            deformableMeshInfo.meshCrc = loadMeshKey(deformablePrim, physxDeformableSurfaceInputCrcToken);

            // store simulation owner
            deformableMeshInfo.scenePath = femClothDesc->scenePath;

            ICE_FREE(femClothDesc);

            return true;
        }
    }
    else if (deformablePrim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
    {
        // make sure particle cloth cooked data is written by the async cooker
        cookingDataAsync->cookParticleClothDeprecated(deformablePrim, false);
        ParticleClothDesc* particleClothDesc = parseParticleClothDeprecated(stageId, deformablePath);

        if (particleClothDesc)
        {
            deformableMeshInfo.type = AttachmentActorTypeDeprecated::ePARTICLE_CLOTH;

            const GfMatrix4d clothToWorld(usdGeomMesh.ComputeLocalToWorldTransform(UsdTimeCode::EarliestTime()));

            deformableMeshInfo.positions.resize(particleClothDesc->points.size());

            for (size_t i = 0; i < particleClothDesc->points.size(); ++i)
            {
                GfVec3f position = { particleClothDesc->points[i].x, particleClothDesc->points[i].y, particleClothDesc->points[i].z };
                position = clothToWorld.Transform(position);
                deformableMeshInfo.positions[i] = *(Float3*)position.data();
            }

            deformableMeshInfo.restPositions.swap(particleClothDesc->restPoints);
            deformableMeshInfo.indices.swap(particleClothDesc->triangleIndices);

            // read crc
            deformableMeshInfo.meshCrc = loadMeshKey(deformablePrim, physxParticleClothInputCrcToken);

            // store simulation owner
            deformableMeshInfo.scenePath = particleClothDesc->scenePath;

            ICE_FREE(particleClothDesc);

            return true;
        }
    }

    return false;
}

bool parsePhysxAutoAttachment(PhysxAutoAttachmentDesc& autoAttachmentDesc, const UsdStageWeakPtr stage, const UsdPrim& usdPrim)
{
    PhysxSchemaPhysxAutoAttachmentAPI autoAttachment = PhysxSchemaPhysxAutoAttachmentAPI::Get(stage, usdPrim.GetPath());
    if (!autoAttachment)
    {
        return false;
    }
    autoAttachment.GetEnableDeformableVertexAttachmentsAttr().Get(&autoAttachmentDesc.enableDeformableVertexAttachments);
    autoAttachment.GetDeformableVertexOverlapOffsetAttr().Get(&autoAttachmentDesc.deformableVertexOverlapOffset);
    autoAttachment.GetEnableRigidSurfaceAttachmentsAttr().Get(&autoAttachmentDesc.enableRigidSurfaceAttachments);
    autoAttachment.GetRigidSurfaceSamplingDistanceAttr().Get(&autoAttachmentDesc.rigidSurfaceSamplingDistance);
    autoAttachment.GetEnableCollisionFilteringAttr().Get(&autoAttachmentDesc.enableCollisionFiltering);
    autoAttachment.GetCollisionFilteringOffsetAttr().Get(&autoAttachmentDesc.collisionFilteringOffset);
    autoAttachment.GetEnableDeformableFilteringPairsAttr().Get(&autoAttachmentDesc.enableDeformableFilteringPairs);
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

bool computeAttachmentPointsDeprecated(const SdfPath& attachmentPath)
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    const uint64_t stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(stage, attachmentPath);
    if (!attachment)
        return false;

    PhysxAutoAttachmentDesc autoAttachmentDesc;
    if (!parsePhysxAutoAttachment(autoAttachmentDesc, stage, attachment.GetPrim()))
        return false;

    SdfPath actorPath[2];
    UsdPrim actorPrim[2];
    AttachmentActorTypeDeprecated::Enum type[2] = { AttachmentActorTypeDeprecated::eWORLD, AttachmentActorTypeDeprecated::eWORLD };
    SdfPathVector targets;
    for (uint32_t t = 0; t < 2; ++t)
    {
        attachment.GetActorRel(t).GetTargets(&targets);
        if (targets.size())
        {
            actorPath[t] = targets[0];
            actorPrim[t] = stage->GetPrimAtPath(actorPath[t]);
            if (!actorPrim[t].IsValid())
            {
                return false;
            }
            else if (actorPrim[t].HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
            {
                type[t] = AttachmentActorTypeDeprecated::eDEFORMABLE_BODY;
            }
            else if (actorPrim[t].HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
            {
                type[t] = AttachmentActorTypeDeprecated::eDEFORMABLE_SURFACE;
            }
            else if (actorPrim[t].HasAPI<PhysxSchemaPhysxParticleClothAPI>())
            {
                type[t] = AttachmentActorTypeDeprecated::ePARTICLE_CLOTH;
            }
            else if (actorPrim[t].HasAPI<UsdPhysicsRigidBodyAPI>() || actorPrim[t].HasAPI<UsdPhysicsCollisionAPI>())
            {
                type[t] = AttachmentActorTypeDeprecated::eRIGID;
            }
        }
        else
        {
            actorPath[t] = SdfPath();
            type[t] = AttachmentActorTypeDeprecated::eWORLD;
        }
    }

    DeformableMeshInfoDeprecated deformableMeshInfo[2];
    uint32_t actorSlot[2] = { 0, 1 };
    uint32_t numDeformables = 0;
    uint32_t numRigids = 0;
    uint32_t numWorld = 0;

    // Make sure that
    // * the first actor is always a deformable (body, surface or cloth)
    // * the first actor is always a deformable body if present
    for (uint32_t t = 0; t < 2; ++t)
    {
        if (type[0] == AttachmentActorTypeDeprecated::eDEFORMABLE_BODY)
        {
            if (type[1] == AttachmentActorTypeDeprecated::eWORLD)
            {
                numDeformables = 1;
                numWorld = 1;
                break;
            }
            else if (type[1] == AttachmentActorTypeDeprecated::eRIGID)
            {
                numDeformables = 1;
                numRigids = 1;
                break;
            }
            else if (type[1] & AttachmentActorTypeDeprecated::eDEFORMABLE)
            {
                numDeformables = 2;
                break;
            }
        }
        else if (type[0] & AttachmentActorTypeDeprecated::eDEFORMABLE)
        {
            if (type[1] == AttachmentActorTypeDeprecated::eWORLD)
            {
                numDeformables = 1;
                numWorld = 1;
                break;
            }
            else if (type[1] == AttachmentActorTypeDeprecated::eRIGID)
            {
                numDeformables = 1;
                numRigids = 1;
                break;
            }
        }

        std::swap(actorPrim[0], actorPrim[1]);
        std::swap(actorSlot[0], actorSlot[1]);
        std::swap(type[0], type[1]);
    }

    // Early out because there is no valid attachment combination (at least 1 deformable (body, surface, particle cloth)
    if (numDeformables == 0)
        return false;

    UsdRelationship maskShapesRel = attachment.GetPrim().GetRelationship(physxAutoAttachmentMaskShapesToken);
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
                bool success = parseMaskShapeDeprecated(holder, transform, maskShapePrim);
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
    for (uint32_t d = 0; d < numDeformables; ++d)
    {
        const UsdPrim prim = actorPrim[d];
        if (!getDeformableMeshInfoDeprecated(prim.GetPath(), deformableMeshInfo[d]))
            return false;   // Something went wrong with getDeformableMeshInfoDeprected, return false

        if (type[d] & AttachmentActorTypeDeprecated::eDEFORMABLE)
        {
            inputCrc.setMeshKey(deformableMeshInfo[d].meshCrc);
        }
    }

    // load attachment mesh key
    omni::physx::usdparser::MeshKey usdInputCrc = loadMeshKey(attachment.GetPrim(), physxAttachmentInputCrcToken);
    if (usdInputCrc == inputCrc)
    {
        // nothing to compute
        return true;
    }
    storeMeshKey(attachment.GetPrim(), physxAttachmentInputCrcToken, inputCrc);

    SdfPathVector simulationOwners;
    if (numWorld == 1)
    {
        updateDeformableWorldAttachments(actorPrim[0], deformableMeshInfo[0], attachment.GetPrim(), actorSlot[0],
            autoAttachmentDesc, maskShapes);
    }
    else if (numRigids == 1)
    {
        const UsdPrim deformablePrim = actorPrim[0];
        const UsdPrim rigidPrim = actorPrim[1];

        VtArray<GfVec3f> emptyPoints;
        VtArray<uint32_t> emptyIndices;
        attachment.GetPointsAttr(0).Set(emptyPoints);
        attachment.GetCollisionFilterIndicesAttr(0).Set(emptyIndices);
        attachment.GetPointsAttr(1).Set(emptyPoints);
        attachment.GetCollisionFilterIndicesAttr(1).Set(emptyIndices);

        GfMatrix4d rigidActorMat = UsdGeomXformable(rigidPrim).ComputeLocalToWorldTransform(UsdTimeCode::EarliestTime());
        PxTransform rigidActorTransform = toPhysX(rigidActorMat);
        const GfTransform tr(rigidActorMat);
        const PxVec3 rigidActorScale = toPhysX(tr.GetScale());

        const UsdPrimRange range(rigidPrim, UsdTraverseInstanceProxies());

        for (UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const UsdPrim& prim = *iter;
            if (!prim)
                continue;

            const bool hasCollisionApi = prim.HasAPI<UsdPhysicsCollisionAPI>();

            if (hasCollisionApi)
            {
                UserDataInfo userData =
                {
                    deformablePrim, deformableMeshInfo[0], attachment.GetPrim(), rigidActorTransform, rigidActorScale,
                    {actorSlot[0], actorSlot[1]}, autoAttachmentDesc, maskShapes
                };

                const GfMatrix4d mat = UsdGeomXformable(prim).ComputeLocalToWorldTransform(UsdTimeCode::EarliestTime());
                const GfTransform tr(mat);

                if (hasCollisionApi)
                {
                    static TfToken oldConvexPrim("ConvexMesh");
                    if (!(prim.IsA<UsdGeomGprim>() || prim.GetTypeName() == oldConvexPrim))
                        continue;

                    usdparser::PhysxShapeDesc* shapeDesc = usdparser::parseCollision(stageId, prim.GetPrimPath(), prim.GetPrimPath());
                    if (!shapeDesc)
                        continue;

                    const GfVec3d pos = tr.GetTranslation();
                    const GfQuatd rot = tr.GetRotation().GetQuat();
                    const GfVec3d scale = tr.GetScale();

                    shapeDesc->localPos = { float(pos[0]), float(pos[1]), float(pos[2]) };
                    shapeDesc->localRot = { float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]), float(rot.GetImaginary()[2]), float(rot.GetReal()) };
                    shapeDesc->localScale = { float(scale[0]), float(scale[1]), float(scale[2]) };

                    parseRigidBodyShapesDeprecated(prim, shapeDesc, updateDeformableRigidAttachments, &userData);

                    iter.PruneChildren();
                }
            }
        }
    }
    else if (numDeformables == 2)
    {
        updateDeformableDeformableAttachments(actorPrim[0], deformableMeshInfo[0], actorPrim[1], deformableMeshInfo[1],
            attachment.GetPrim(), actorSlot, autoAttachmentDesc, maskShapes);
    }

    return true;
}

bool computeAttachmentPointsDeprecated(const SdfPath& deformablePath, const SdfPath& rigidBodyPath, const SdfPath& attachmentPath)
{
    CARB_LOG_WARN("Deprecated function, will be replaced by new deformable attachment feature.");

    return computeAttachmentPointsDeprecated(attachmentPath);
}

} // namespace physx
} // namespace omni
