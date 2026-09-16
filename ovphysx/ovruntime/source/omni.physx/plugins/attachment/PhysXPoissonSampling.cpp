// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-20 AC-21
 *
 * @implements REQ-PARSE-CORE-006
 * @covers AC-8
 */

#include "PhysXPoissonSampling.h"
#include "PhysXAttachment.h"

#include <usdLoad/IceDescriptorAllocator.h>
#include <usdLoad/LoadUsd.h>
#include <usdLoad/ScannedShapeCookingDispatch.h>

#include <omni/physics/parse/ScanBackend.h>
#include <omni/physics/parse/ScannedStage.h>

#include <PhysXScene.h>
#include <PhysXTools.h>
#include <Setup.h>

using namespace carb;
using namespace ::physx;
using namespace omni::physx;
using namespace omni::physx::usdparser;

namespace omni
{
namespace sampling
{

struct SurfaceSampler
{
    PxPoissonSampler* sampler;

    PxSphereGeometry sphere;
    PxBoxGeometry box;
    PxCapsuleGeometry capsule;
    PxPlaneGeometry plane;
    PxConvexMeshGeometry convexMesh;
    PxTriangleMeshGeometry triangleMesh;

    omni::physics::parse::ObjectKey colliderKey;
    float samplingDistance;

    PxPhysics* physicsPtr;
    PxArray<PxVec3> samples;

    PxGeometryType::Enum geomType;
    PxTransform geomPos;

    // optional tri mesh sampler
    std::vector<PxU32> triangleIndices;
    std::vector<PxVec3> triangleVertices;
    PxTriangleMeshPoissonSampler* triMeshSampler = nullptr;
};

std::vector<SurfaceSampler*> gSurfaceSamplerTable;

struct UserDataInfo
{
    SurfaceSampler* surfaceSampler;
    float samplingDistance;
};

// Re-scans just the collider prim to cook a standalone PhysxShapeDesc for it (the sampler
// needs the raw geometry, not an already-tracked simulation object).
// Returns an owning DescPtr: the descriptor is moved out of the temporary ScannedStage,
// so the caller's unique_ptr is what frees it.
omni::physics::parse::DescPtr<PhysxShapeDesc> parseSurfaceSamplerShape(
    AttachedStage& attachedStage, omni::physics::parse::ObjectKey colliderKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(colliderKey))
        return {};

    const std::string colliderPathText(attachedStage.textViewFor(colliderKey));
    const std::vector<std::string> scanRoots{ colliderPathText };
    static const std::vector<std::string> kNoExclude;
    omni::physics::parse::ScannedStage scanned = omni::physics::parse::scanStage(
        attachedStage.attachTarget(), scanRoots, kNoExclude, omni::physics::parse::ScanOptions{},
        iceDescriptorAllocator());
    const omni::physics::parse::IPhysicsSource& scanSrc = scanned.source();

    for (auto& shape : scanned.shapes)
    {
        PhysxShapeDesc* desc = shape.get();
        if (scanSrc.sourceKeyToString(desc->primKey) != colliderPathText &&
            (!desc->sourceGprim.valid() || scanSrc.sourceKeyToString(desc->sourceGprim) != colliderPathText))
        {
            continue;
        }

        usdparser::scan::dispatchScannedShapeCooking(attachedStage, scanned, desc);

        if (desc->rigidBody.valid())
            desc->rigidBody = attachedStage.keyFor(scanSrc.sourceKeyToString(desc->rigidBody));
        if (desc->sourceGprim.valid())
            desc->sourceGprim = attachedStage.keyFor(scanSrc.sourceKeyToString(desc->sourceGprim));
        if (desc->type == eConvexMeshShape)
        {
            auto* d = static_cast<ConvexMeshPhysxShapeDesc*>(desc);
            if (d->meshPrimKey.valid())
                d->meshPrimKey = attachedStage.keyFor(scanSrc.sourceKeyToString(d->meshPrimKey));
        }
        else if (desc->type == eTriangleMeshShape ||
                 desc->type == eConvexMeshDecompositionShape ||
                 desc->type == eSpherePointsShape)
        {
            auto* d = static_cast<TriangleMeshPhysxShapeDesc*>(desc);
            if (d->meshPrimKey.valid())
                d->meshPrimKey = attachedStage.keyFor(scanSrc.sourceKeyToString(d->meshPrimKey));
        }
        return std::move(shape);
    }

    return {};
}

void createSamplerCallback(const ::physx::PxGeometry& geom, const ::physx::PxTransform& geomPos, void* userData)
{
    UserDataInfo* info = (UserDataInfo*)userData;

    PxBounds3 bounds;
    PxGeometryQuery::computeGeomBounds(bounds, geom, geomPos, 0.0f, 1.01f);

    switch (geom.getType())
    {
        case PxGeometryType::eSPHERE:
        {
            const PxSphereGeometry& shapeGeom = static_cast<const PxSphereGeometry&>(geom);
            info->surfaceSampler->sphere = shapeGeom;
            info->surfaceSampler->sampler = PxCreateShapeSampler(info->surfaceSampler->sphere, geomPos, bounds, info->samplingDistance);
            break;
        }
        case PxGeometryType::ePLANE:
        {
            const PxPlaneGeometry& shapeGeom = static_cast<const PxPlaneGeometry&>(geom);
            info->surfaceSampler->plane = shapeGeom;
            info->surfaceSampler->sampler = PxCreateShapeSampler(info->surfaceSampler->plane, geomPos, bounds, info->samplingDistance);
            break;
        }
        case PxGeometryType::eCAPSULE:
        {
            const PxCapsuleGeometry& shapeGeom = static_cast<const PxCapsuleGeometry&>(geom);
            info->surfaceSampler->capsule = shapeGeom;
            info->surfaceSampler->sampler = PxCreateShapeSampler(info->surfaceSampler->capsule, geomPos, bounds, info->samplingDistance);
            break;
        }
        case PxGeometryType::eBOX:
        {
            const PxBoxGeometry& shapeGeom = static_cast<const PxBoxGeometry&>(geom);
            info->surfaceSampler->box = shapeGeom;
            info->surfaceSampler->sampler = PxCreateShapeSampler(info->surfaceSampler->box, geomPos, bounds, info->samplingDistance);
            break;
        }
        case PxGeometryType::eCONVEXMESH:
        {
            const PxConvexMeshGeometry& shapeGeom = static_cast<const PxConvexMeshGeometry&>(geom);
            info->surfaceSampler->convexMesh = shapeGeom;
            info->surfaceSampler->sampler = PxCreateShapeSampler(info->surfaceSampler->convexMesh, geomPos, bounds, info->samplingDistance);
            break;
        }
        case PxGeometryType::eTRIANGLEMESH:
        {
            const PxTriangleMeshGeometry& shapeGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
            info->surfaceSampler->triangleMesh = shapeGeom;
            info->surfaceSampler->sampler = PxCreateShapeSampler(info->surfaceSampler->triangleMesh, geomPos, bounds, info->samplingDistance);
            break;
        }
        default:
        {
            CARB_ASSERT(false);
            break;
        }
    }

    info->surfaceSampler->geomType = geom.getType();
    info->surfaceSampler->geomPos = geomPos;
}

uint64_t createSurfaceSampler(omni::physics::parse::ObjectKey colliderKey, float samplingDistance)
{
    omni::physx::OmniPhysX& omniPhysX = omni::physx::OmniPhysX::getInstance();
    usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!attachedStage)
        return 0;

    const omni::physics::parse::DescPtr<PhysxShapeDesc> shapeOwner =
        parseSurfaceSamplerShape(*attachedStage, colliderKey);
    PhysxShapeDesc* shapeDesc = shapeOwner.get();
    if (!shapeDesc)
        return 0;

    const PxMat44d mat = omni::physx::internal::getWorldTransform(*attachedStage, colliderKey, omni::physics::parse::ReadTime::defaultTime());
    PxTransform pose;
    PxVec3 scale;
    omni::physx::decomposeMatrix(pose, scale, mat);

    shapeDesc->localPos = { pose.p.x, pose.p.y, pose.p.z };
    shapeDesc->localRot = { pose.q.x, pose.q.y, pose.q.z, pose.q.w };
    shapeDesc->localScale = { scale.x, scale.y, scale.z };

    SurfaceSampler* surfaceSampler = ICE_PLACEMENT_NEW(SurfaceSampler)();
    surfaceSampler->colliderKey = colliderKey;
    surfaceSampler->samplingDistance = samplingDistance;

    UserDataInfo userData = { surfaceSampler, samplingDistance };

    processRigidShapeGeometry(*attachedStage, colliderKey, shapeDesc, createSamplerCallback, &userData);

    surfaceSampler->physicsPtr = omniPhysX.getPhysXSetup().getPhysics();
    gSurfaceSamplerTable.push_back(surfaceSampler);

    return gSurfaceSamplerTable.size() - 1;
}

void releaseSurfaceSampler(const uint64_t surfaceSampler)
{
    SurfaceSampler* s = gSurfaceSamplerTable[surfaceSampler];

    PX_DELETE(s->triMeshSampler);
    PX_DELETE(s->sampler);
    // For consistency, should have a corresponding PxFreeShapeSampler or release() method

    ICE_FREE(s);

    gSurfaceSamplerTable[surfaceSampler] = nullptr;
}

void notifyPhysXSceneRelease()
{
    if (gSurfaceSamplerTable.size() == 0)
        return;

    // Garbage collection
    size_t count;
    for (count = 0; count < gSurfaceSamplerTable.size(); count++)
    {
        if (gSurfaceSamplerTable[count])
            break;
    }
    if (count == gSurfaceSamplerTable.size())
    {
        // All the surface samplers are null, set the table to size 0
        gSurfaceSamplerTable.clear();
        return;
    }

    for (size_t i = 0; i < gSurfaceSamplerTable.size(); i++)
    {
        SurfaceSampler* s = gSurfaceSamplerTable[i];
        if (s && s->sampler)
        {
            s->samples = s->sampler->getSamples();
            PX_DELETE(s->sampler);

            s->sampler = nullptr;
            s->physicsPtr = nullptr;
        }
    }
}

SurfaceSampler* getSurfaceSampler(const uint64_t surfaceSampler)
{
    SurfaceSampler* s = gSurfaceSamplerTable[surfaceSampler];
    CARB_ASSERT(s);

    omni::physx::OmniPhysX& omniPhysX = omni::physx::OmniPhysX::getInstance();

    if (s->physicsPtr != omniPhysX.getPhysXSetup().getPhysics())
    {
        // PhysX pointer has changed. Recreate sampler with new PhysX pointer.
        if (s->sampler)
        {
            s->samples = s->sampler->getSamples();
            PX_DELETE(s->sampler);
        }

        usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
        if (!attachedStage)
        {
            CARB_ASSERT(0);
            return nullptr;
        }

        const omni::physics::parse::DescPtr<PhysxShapeDesc> shapeOwner =
            parseSurfaceSamplerShape(*attachedStage, s->colliderKey);
        PhysxShapeDesc* shapeDesc = shapeOwner.get();
        if (!shapeDesc)
        {
            CARB_ASSERT(0);
            return nullptr;
        }

        const PxMat44d mat = omni::physx::internal::getWorldTransform(*attachedStage, s->colliderKey, omni::physics::parse::ReadTime::defaultTime());
        PxTransform pose;
        PxVec3 scale;
        omni::physx::decomposeMatrix(pose, scale, mat);

        shapeDesc->localPos = { pose.p.x, pose.p.y, pose.p.z };
        shapeDesc->localRot = { pose.q.x, pose.q.y, pose.q.z, pose.q.w };
        shapeDesc->localScale = { scale.x, scale.y, scale.z };

        UserDataInfo userData = { s, s->samplingDistance };

        processRigidShapeGeometry(*attachedStage, s->colliderKey, shapeDesc, createSamplerCallback, &userData);

        s->sampler->addSamples(s->samples);
        s->physicsPtr = OmniPhysX::getInstance().getPhysXSetup().getPhysics();

        if (s->triMeshSampler)
        {
            // recreate tri mesh sampler if it exists
            createTriMeshSampler(surfaceSampler);
        }
    }

    return s;
}

void addSurfaceSamplerPoints(const uint64_t surfaceSampler, const carb::Float3* points, const uint32_t pointsSize)
{
    SurfaceSampler* s = getSurfaceSampler(surfaceSampler);
    PxPoissonSampler* sampler = s->sampler;

    PxArray<PxVec3> samples;
    samples.assign((const PxVec3*)points, (const PxVec3*)points + pointsSize);

    sampler->addSamples(samples);
}

void removeSurfaceSamplerPoints(const uint64_t surfaceSampler, const carb::Float3* points, const uint32_t pointsSize)
{
    SurfaceSampler* s = getSurfaceSampler(surfaceSampler);
    PxPoissonSampler* sampler = s->sampler;

    PxArray<PxVec3> targetPoints;
    targetPoints.assign((const PxVec3*)points, (const PxVec3*)points + pointsSize);

    sampler->removeSamples(targetPoints);
}

void sampleSurface(carb::Float3*& points, uint32_t& pointsSize, const uint64_t surfaceSampler, const Float3& sphereCenter, const float sphereRadius, const float samplingDistance, void* (*allocateBytes)(size_t))
{
    SurfaceSampler* s = getSurfaceSampler(surfaceSampler);

    if (s->samplingDistance != samplingDistance)
    {
        s->samplingDistance = samplingDistance;
        s->physicsPtr = nullptr;
        s = getSurfaceSampler(surfaceSampler);
    }

    PxPoissonSampler* sampler = s->sampler;

    sampler->addSamplesInSphere(toPhysX(sphereCenter), sphereRadius);
    const PxArray<PxVec3>& samples = sampler->getSamples();

    pointsSize = samples.size();
    points = (carb::Float3*)allocateBytes(sizeof(carb::Float3) * pointsSize);
    memcpy(points, &samples[0], sizeof(carb::Float3) * pointsSize);
}

void getSurfaceSamplerPoints(carb::Float3*& points, uint32_t& pointsSize, const uint64_t surfaceSampler, void* (*allocateBytes)(size_t))
{
    SurfaceSampler* s = getSurfaceSampler(surfaceSampler);
    PxPoissonSampler* sampler = s->sampler;
    const PxArray<PxVec3>& samples = sampler->getSamples();

    pointsSize = samples.size();
    points = (carb::Float3*)allocateBytes(sizeof(carb::Float3) * pointsSize);
    memcpy(points, &samples[0], sizeof(carb::Float3) * pointsSize);
}

void createTriMeshSampler(const uint64_t surfaceSampler)
{
    SurfaceSampler* s = getSurfaceSampler(surfaceSampler);
    bool isTriangleMesh = s->geomType == PxGeometryType::eTRIANGLEMESH;

    if (isTriangleMesh)
    {
        std::vector<PxU32>& triangleIndices = s->triangleIndices;
        std::vector<PxVec3>& triangleVertices = s->triangleVertices;

        triangleIndices.clear();
        triangleVertices.clear();
        PX_DELETE(s->triMeshSampler);

        const PxTriangleMeshGeometry& triMesh = s->triangleMesh;

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
            vertex = s->geomPos.transform(vertex);

            triangleVertices.push_back(vertex);
        }

        s->triMeshSampler = PxCreateTriangleMeshSampler(trianglePtr, triMesh.triangleMesh->getNbTriangles(), &triangleVertices[0], triMesh.triangleMesh->getNbVertices(), 30);
    }
}

bool isPointInside(const uint64_t surfaceSampler, const carb::Float3 point)
{
    SurfaceSampler* s = getSurfaceSampler(surfaceSampler);

    if (s->triMeshSampler)
        return s->triMeshSampler->isPointInTriangleMesh(toPhysX(point));

    return false;
}

// TODO, implement and move somewhere else
struct UserDataInfoEx
{
    int32_t* overlapIndices;
    uint32_t& overlapIndicesSize;
    const carb::Float3* points;
    const uint32_t pointsSize;
    const float overlapOffset;
};

void findOverlap(const PxGeometry& geom, const PxTransform& geomPos, const PxVec3& scale, void* userData)
{
    UserDataInfoEx* info = (UserDataInfoEx*)userData;
    int32_t* overlapIndices = info->overlapIndices;
    uint32_t& overlapIndicesSize = info->overlapIndicesSize;
    const carb::Float3* points = info->points;
    const uint32_t pointsSize = info->pointsSize;
    const float overlapOffset = info->overlapOffset;

    PxSphereGeometry overlapSphere(overlapOffset);

    overlapIndicesSize = 0;
    for (PxU32 i = 0; i < pointsSize; i++)
    {
        const PxVec3 particlePos = toPhysX(points[i]);

        bool attachmentHit = PxGeometryQuery::overlap(geom, geomPos, overlapSphere, PxTransform(particlePos));
        if (!attachmentHit)
            continue;

        overlapIndices[overlapIndicesSize++] = i;
    }
}

} // namespace sampling
} // namespace omni
