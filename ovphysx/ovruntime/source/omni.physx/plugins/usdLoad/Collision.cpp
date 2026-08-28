// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-COL-001
 * @covers AC-4
 *
 * @implements REQ-PARSE-COL-002
 * @covers AC-5
 *
 * @implements REQ-PARSE-COL-003
 * @covers AC-4 AC-5
 *
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-3 AC-4 AC-5
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-3
 */

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>
#include <omni/physics/usd/PrimIterator.h>
#include <private/omni/physx/CustomGeometryHash.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/UsdMaterialParsing.h>
#include <carb/profiler/Profile.h>

#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>


#include <utils/Profile.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include <PhysXTools.h>
#include "Mass.h"
#include "PhysicsBody.h"

#include <OmniPhysX.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include "UsdSource.h"
#include "Collision.h"
#include "Material.h"
#include "CollisionGroup.h"
#include "AttributeHelpers.h"
#include "NewtonCompat.h"

#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <common/utilities/Utilities.h>
#include <omni/physx/IPhysxCookingService.h>

using namespace PXR_NS;
using namespace carb;
using namespace carb::tasking;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

static TfToken oldConvexPrim("ConvexMesh");
static TfToken obsoleteCustomGeometryAttribute("physxCollisionCustomGeometry");
using MeshKeyMap = std::unordered_map< SdfPath, omni::physx::usdparser::MeshKey,SdfPath::Hash >;


class MeshKeyCache
{
public:

    void setMeshKey(const SdfPath &name,const MeshKey &key)
    {
        mMeshKeys[name] = key;
    }

    bool getMeshKey(const SdfPath &name,MeshKey &key) const
    {
        bool ret = false;

        MeshKeyMap::const_iterator found = mMeshKeys.find(name);
        if ( found != mMeshKeys.end() )
        {
            ret = true;
            key = (*found).second;
        }

        return ret;
    }

    bool clearMeshKey(const SdfPath &name)
    {
        bool ret = false;

        MeshKeyMap::iterator found = mMeshKeys.find(name);
        if (found != mMeshKeys.end())
        {
            ret = true;
            mMeshKeys.erase(found);
        }
        return ret;
    }

    void reset(void)
    {
        mMeshKeys.clear();
    }

private:
    MeshKeyMap  mMeshKeys;
};

MeshKeyCache gMeshKeyCache;


// compute bounding sphere for a given vertex array
// get the furthers points along each axis first and create a sphere based on these two initial points (min/max)
// traverse the vertices and if a vertex does not belong to the sphere, construct a new center and radius including this
// vertex and continue
BoundingSpherePhysxShapeDesc* computeBoundingSphereShape(const std::vector<carb::Float3>& pointsValue)
{
    Float3 sphereCenter;
    float radius;

    const bool success = PhysXUsdPhysicsInterface::createBoundingSphere(
        pointsValue.data(), pointsValue.size(), sphereCenter, radius);
    if (!success)
    {
        CARB_LOG_ERROR("Failed to create OBB for input point clouds!");
        return nullptr;
    }

    BoundingSpherePhysxShapeDesc* sphereDesc = ICE_PLACEMENT_NEW(BoundingSpherePhysxShapeDesc)();
    sphereDesc->radius = radius;
    sphereDesc->positionOffset = sphereCenter;

    return sphereDesc;
}

// compute OBB around given points
BoundingBoxPhysxShapeDesc* computeBoundingBoxShape(const std::vector<carb::Float3>& pointsValue)
{
    Float3 halfExtent;
    Float3 offsetPos;
    Float4 offsetRot;

    const bool success = PhysXUsdPhysicsInterface::createOBB(
        pointsValue.data(), pointsValue.size(), halfExtent, offsetPos, offsetRot);
    if (!success)
    {
        CARB_LOG_ERROR("Failed to create OBB for input point clouds!");
        return nullptr;
    }

    BoundingBoxPhysxShapeDesc* boxDesc = ICE_PLACEMENT_NEW(BoundingBoxPhysxShapeDesc)();

    boxDesc->rotationOffset = offsetRot;
    boxDesc->positionOffset = offsetPos;
    boxDesc->halfExtents = halfExtent;

    return boxDesc;
}

// Helper method to initialize the maximum number of convex hull vertices and add it to the MeshKey CRC
void initMaxHullVertices(const UsdAttribute& attr, uint32_t &maxHullVertices)
{
    int _maxHullVertices;
    if (attr.Get(&_maxHullVertices))
    {
        maxHullVertices = uint32_t(_maxHullVertices);
    }    
}

// Helper method to initialize the maximum number of convex hulls and add it to the MeshKey CRC
void initMaxConvexHulls(const UsdAttribute& attr, uint32_t &maxConvexHulls)
{
    int _maxConvexHulls;
    if (attr.Get(&_maxConvexHulls))
    {
        maxConvexHulls = uint32_t(_maxConvexHulls);
    }    
}

// Helper method to initialize the maximum number of spheres and add it to the MeshKey CRC
void initMaxSpheres(const UsdAttribute& attr, uint32_t &maxSpheres)
{
    int _maxSpheres;
    if (attr.Get(&_maxSpheres))
    {
        maxSpheres = uint32_t(_maxSpheres);
    }    
}

void initSeedCount(const UsdAttribute& attr, uint32_t &seedCount)
{
    int _seedCount;
    if (attr.Get(&_seedCount))
    {
        seedCount = uint32_t(_seedCount);
    }    
}

void initFillMode(const UsdAttribute& attr, SphereFillMode::Enum&fillMode)
{
    static TfToken flood("flood");
    static TfToken raycast("raycast");
    static TfToken surface("surface");
    TfToken _fillMode;
    if (attr.Get(&_fillMode))
    {
        if ( _fillMode == flood )
        {
            fillMode = SphereFillMode::eFLOOD;
        }
        else if ( _fillMode == raycast )
        {
            fillMode = SphereFillMode::eRAYCAST;
        }
        else if ( _fillMode == surface )
        {
            fillMode = SphereFillMode::eSURFACE;
        }
    }    
}

// Helper method to initialize the voxel resolution and add it to the MeshKey CRC
void initVoxelResolution(const UsdAttribute& attr, uint32_t &voxelResolution)
{
    int _voxelResolution;
    if (attr.Get(&_voxelResolution))
    {
        voxelResolution = uint32_t(_voxelResolution);
    }    
}

// Helper method to initialize the mesh simplification metric value and add it to the MeshKey CRC
void initUseShrinkwrap(const UsdAttribute& attr, bool &useShrinkWrap)
{
    attr.Get(&useShrinkWrap);    
}

// Helper method to initialize the volume error percentage threshold and add it to the MeshKey CRC
void initErrorPercentage(const UsdAttribute& attr, float &errorPercentage)
{
    attr.Get(&errorPercentage);    
}

// Helper method to initialize the minimum collision thickness value and add it to the MeshKey CRC
void initMinThickness(const UsdAttribute& attr, float &minThickness)
{
    attr.Get(&minThickness);    
}

// Helper method to initialize the mesh simplification metric value and add it to the MeshKey CRC
void initSimplificationMetric(const UsdAttribute& attr, float &simplificationMetric)
{
    attr.Get(&simplificationMetric);    
}

// Helper method to initialize the mesh weld tolerance value and add it to the MeshKey CRC
void initWeldToleranceMetric(const UsdAttribute& attr, float &weldTolerance)
{
    attr.Get(&weldTolerance);
    if (isnan(weldTolerance))
    {
        weldTolerance = -FLT_MAX;
    }    
}

void fillCookingRequest(omni::physx::PhysxCookingComputeRequest& request, const UsdPrim& prim, UsdTimeCode time)
{
    // If we have the meshKey in cache, we use it, saving omni.physx.cooking from recomputing it
    gMeshKeyCache.getMeshKey(prim.GetPath(), request.meshKey);
    request.primStageId = UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();
    request.primId = asInt(prim.GetPath());
    request.primTimeCode = time.GetValue();
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kExecuteCookingOnGPU, false);
    request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;
}

bool isCollisionShape(const UsdStageWeakPtr stage, const UsdPrim& prim)
{
    return prim.HasAPI<UsdPhysicsCollisionAPI>();
}

PhysxShapeDesc* scaleShapeDesc(const PhysxShapeDesc& inDesc, const GfVec3f& scale)
{
    PhysxShapeDesc* desc = nullptr;

    switch (inDesc.type)
    {
    case eSphereShape:
    {
        desc = ICE_PLACEMENT_NEW(SpherePhysxShapeDesc)();
        SpherePhysxShapeDesc& sphereDesc = (SpherePhysxShapeDesc&)*desc;
        sphereDesc = (const SpherePhysxShapeDesc&)inDesc;
        const float radiusScale = fmaxf(fmaxf(fabsf(float(scale[1])), fabsf(float(scale[0]))), fabsf(float(scale[2])));
        sphereDesc.radius = sphereDesc.radius * radiusScale;
    }
    break;
    case eBoxShape:
    {
        desc = ICE_PLACEMENT_NEW(BoxPhysxShapeDesc)();
        BoxPhysxShapeDesc& boxDesc = (BoxPhysxShapeDesc&)*desc;
        boxDesc = (const BoxPhysxShapeDesc&)inDesc;

        (GfVec3f&)boxDesc.halfExtents = GfCompMult((const GfVec3f&)boxDesc.halfExtents, scale);
    }
    break;
    case eCapsuleShape:
    {
        desc = ICE_PLACEMENT_NEW(CapsulePhysxShapeDesc)();
        CapsulePhysxShapeDesc& capsuleDesc = (CapsulePhysxShapeDesc&)*desc;
        capsuleDesc = (const CapsulePhysxShapeDesc&)inDesc;

        if (capsuleDesc.axis == Axis::eX)
        {
            capsuleDesc.halfHeight *= scale[0];
            capsuleDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[2]));
        }
        else if (capsuleDesc.axis == Axis::eY)
        {
            capsuleDesc.halfHeight *= scale[1];
            capsuleDesc.radius *= fmaxf(fabsf(scale[0]), fabsf(scale[2]));
        }
        else
        {
            capsuleDesc.halfHeight *= scale[2];
            capsuleDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[0]));
        }
    }
    break;
    case eCylinderShape:
    {
        desc = ICE_PLACEMENT_NEW(CylinderPhysxShapeDesc)();
        CylinderPhysxShapeDesc& cylinderDesc = (CylinderPhysxShapeDesc&)*desc;
        cylinderDesc = (const CylinderPhysxShapeDesc&)inDesc;

        if (cylinderDesc.axis == Axis::eX)
        {
            cylinderDesc.halfHeight *= scale[0];
            cylinderDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[2]));
        }
        else if (cylinderDesc.axis == Axis::eY)
        {
            cylinderDesc.halfHeight *= scale[1];
            cylinderDesc.radius *= fmaxf(fabsf(scale[0]), fabsf(scale[2]));
        }
        else
        {
            cylinderDesc.halfHeight *= scale[2];
            cylinderDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[0]));
        }
    }
    break;
    case eConeShape:
    {
        desc = ICE_PLACEMENT_NEW(ConePhysxShapeDesc)();
        ConePhysxShapeDesc& coneDesc = (ConePhysxShapeDesc&)*desc;
        coneDesc = (const ConePhysxShapeDesc&)inDesc;

        if (coneDesc.axis == Axis::eX)
        {
            coneDesc.halfHeight *= scale[0];
            coneDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[2]));
        }
        else if (coneDesc.axis == Axis::eY)
        {
            coneDesc.halfHeight *= scale[1];
            coneDesc.radius *= fmaxf(fabsf(scale[0]), fabsf(scale[2]));
        }
        else
        {
            coneDesc.halfHeight *= scale[2];
            coneDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[0]));
        }
    }
    break;
    case eConvexMeshShape:
    {
        desc = ICE_PLACEMENT_NEW(ConvexMeshPhysxShapeDesc)();
        ConvexMeshPhysxShapeDesc& convexDesc = (ConvexMeshPhysxShapeDesc&)*desc;
        convexDesc = (const ConvexMeshPhysxShapeDesc&)inDesc;

        (GfVec3f&)convexDesc.meshScale = GfCompMult((const GfVec3f&)convexDesc.meshScale, scale);
        convexDesc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(convexDesc.meshScale);
    }
    break;
    case eConvexMeshDecompositionShape:
    {
        desc = ICE_PLACEMENT_NEW(ConvexMeshDecompositionPhysxShapeDesc)();
        ConvexMeshDecompositionPhysxShapeDesc& convexDecDesc = (ConvexMeshDecompositionPhysxShapeDesc&)*desc;
        convexDecDesc = (const ConvexMeshDecompositionPhysxShapeDesc&)inDesc;

        (GfVec3f&)convexDecDesc.meshScale = GfCompMult((const GfVec3f&)convexDecDesc.meshScale, scale);
        convexDecDesc.convexDecompositionCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(convexDecDesc.meshScale);
    }
    break;
    case eTriangleMeshShape:
    {
        desc = ICE_PLACEMENT_NEW(TriangleMeshPhysxShapeDesc)();
        TriangleMeshPhysxShapeDesc& meshDesc = (TriangleMeshPhysxShapeDesc&)*desc;
        meshDesc = (const TriangleMeshPhysxShapeDesc&)inDesc;

        (GfVec3f&)meshDesc.meshScale = GfCompMult((const GfVec3f&)meshDesc.meshScale, scale);
    }
    break;
    case eBoundingSphereShape:
    {
        desc = ICE_PLACEMENT_NEW(BoundingSpherePhysxShapeDesc)();
        BoundingSpherePhysxShapeDesc& bsDesc = (BoundingSpherePhysxShapeDesc&)*desc;
        bsDesc = (const BoundingSpherePhysxShapeDesc&)inDesc;

        (GfVec3f&)bsDesc.positionOffset = GfCompMult((const GfVec3f&)bsDesc.positionOffset, scale);
        const float radiusScale = fmaxf(fmaxf(fabsf(float(scale[1])), fabsf(float(scale[0]))), fabsf(float(scale[2])));
        bsDesc.radius = bsDesc.radius * radiusScale;
    }
    break;
    case eBoundingBoxShape:
    {
        desc = ICE_PLACEMENT_NEW(BoundingBoxPhysxShapeDesc)();
        BoundingBoxPhysxShapeDesc& bbDesc = (BoundingBoxPhysxShapeDesc&)*desc;
        bbDesc = (const BoundingBoxPhysxShapeDesc&)inDesc;

        (GfVec3f&)bbDesc.positionOffset = GfCompMult((const GfVec3f&)bbDesc.positionOffset, scale);
        (GfVec3f&)bbDesc.halfExtents = GfCompMult((const GfVec3f&)bbDesc.halfExtents, scale);
    }
    break;
    case ePlaneShape:
    {
        desc = ICE_PLACEMENT_NEW(PlanePhysxShapeDesc)();
        PlanePhysxShapeDesc& bbDesc = (PlanePhysxShapeDesc&)*desc;
    }
    break;
    default:
        break;
    }

    if (desc)
    {
        (GfVec3f&)desc->localPos = GfCompMult((const GfVec3f&)desc->localPos, scale);
        (GfVec3f&)desc->localScale = GfCompMult((const GfVec3f&)desc->localScale, scale);
    }

    return desc;
}

void finalizeShape(AttachedStage& attachedStage, PhysxShapeDesc* desc, const SdfPathVector& materials)
{
    for (const SdfPath& materialKey : materials)
    {
        desc->materials.push_back(getMaterial(attachedStage, materialKey));
    }

}

PhysxRigidBodyDesc* createShape(AttachedStage& attachedStage, const SdfPath& path, PhysxShapeDesc* shapeDesc, const ObjectInstance* objectInstance, ObjectId* instancedShapeId)
{
    const bool hadNoRigidBody = !shapeDesc->rigidBody.valid();

    // If we use shape for instanced create, we should not search for existing bodies
    const ObjectId bodyId = instancedShapeId ? kInvalidObjectId : getRigidBody(attachedStage, path, *shapeDesc);
    PhysxRigidBodyDesc* bodyDesc = nullptr;
    if (!shapeDesc->rigidBody.valid())
    {
        bodyDesc = createStaticBody();
        bodyDesc->position = shapeDesc->localPos;
        bodyDesc->rotation = shapeDesc->localRot;
        bodyDesc->scale = shapeDesc->localScale;
        bodyDesc->sceneIds = shapeDesc->sceneIds;

        if (shapeDesc->sourceGprim != attachedStage.keyFor(path))
        {
            ((StaticPhysxRigidBodyDesc*)bodyDesc)->sourceGPrimKey = shapeDesc->sourceGprim;
        }

        shapeDesc->localPos = { 0.0f, 0.0f , 0.0f };
        shapeDesc->localRot = { 0.0f, 0.0f , 0.0f, 1.0f };
        shapeDesc->localScale = { 1.0f, 1.0f , 1.0f };
    }
    else if (hadNoRigidBody)
    {
        // Need to re-calculate the shape TM relative to the rigid body
        GfVec3f localPos;
        GfVec3f localScale;
        GfQuatf localRot;
        getCollisionShapeLocalTransform(attachedStage, attachedStage.keyFor(path), shapeDesc->rigidBody,
            localPos, localRot, localScale);
        GfVec3ToFloat3(localPos, shapeDesc->localPos);
        GfQuatToFloat4(localRot, shapeDesc->localRot);
        GfVec3ToFloat3(localScale, shapeDesc->localScale);
    }

    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createShape(path, *shapeDesc, bodyId, objectInstance);
    if (id != kInvalidObjectId)
        attachedStage.getObjectDatabase()->findOrCreateEntry(path, eShape, id);

    if (instancedShapeId)
    {
        *instancedShapeId = id;
    }

    if (bodyId != kInvalidObjectId)
    {
        attachedStage.bufferRequestRigidBodyMassUpdate(attachedStage.pathFor(shapeDesc->rigidBody));
    }

    if (bodyDesc && id != kInvalidObjectId)
    {
        bodyDesc->shapes.push_back(id);
    }

    return bodyDesc;
}

void releaseShapeDesc(PhysxShapeDesc* desc)
{
    ICE_FREE(desc);
}

SourceMeshGeometryScope::~SourceMeshGeometryScope()
{
    if (!src)
        return;
    if (geom.points.valid())        src->releaseBuffer(geom.points);
    if (geom.indices.valid())       src->releaseBuffer(geom.indices);
    if (geom.faceCounts.valid())    src->releaseBuffer(geom.faceCounts);
    if (geom.holes.valid())         src->releaseBuffer(geom.holes);
    if (geom.faceMaterials.valid()) src->releaseBuffer(geom.faceMaterials);
}

bool fillCookingMeshViewFromSource(omni::physx::PhysxCookingComputeRequest& request,
                                   SourceMeshGeometryScope& scope,
                                   const AttachedStage& attachedStage,
                                   omni::physics::parse::ObjectKey meshKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    scope.src = src;
    scope.geom = src->getMeshAttributes(meshKey);
    const omni::physics::parse::MeshGeometry& g = scope.geom;
    if (!g.points.valid() || !g.indices.valid() || !g.faceCounts.valid())
        return false;  // not enough geometry — caller keeps the prim-id path

    size_t pointsBytes = 0, indicesBytes = 0, facesBytes = 0, holesBytes = 0;
    const carb::Float3* points = static_cast<const carb::Float3*>(src->resolveBuffer(g.points, pointsBytes));
    const int32_t* indices = static_cast<const int32_t*>(src->resolveBuffer(g.indices, indicesBytes));
    const int32_t* faces = static_cast<const int32_t*>(src->resolveBuffer(g.faceCounts, facesBytes));
    const int32_t* holes = g.holes.valid() ? static_cast<const int32_t*>(src->resolveBuffer(g.holes, holesBytes)) : nullptr;
    if (!points || !indices || !faces)
        return false;

    // The resolved buffers stay valid until `scope` releases them (after the
    // synchronous submission below copies the view in setupTaskFromRequest).
    request.dataInputMode = omni::physx::PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_MESH_VIEW;
    request.primMeshView.points = { points, pointsBytes / sizeof(carb::Float3) };
    request.primMeshView.indices = { indices, indicesBytes / sizeof(int32_t) };
    request.primMeshView.faces = { faces, facesBytes / sizeof(int32_t) };
    request.primMeshView.holeIndices = { holes, holes ? holesBytes / sizeof(int32_t) : size_t(0) };
    // Matches the service's isRightHandedOrientation (orientation != leftHanded).
    request.primMeshView.rightHandedOrientation = !g.leftHanded;

    // Per-face physics-material indices for multi-material triangle/SDF meshes
    // (empty for single-material meshes — the service then treats it as one
    // material, matching the prim-id path).
    if (g.faceMaterials.valid())
    {
        size_t faceMaterialsBytes = 0;
        const uint16_t* faceMaterials = static_cast<const uint16_t*>(src->resolveBuffer(g.faceMaterials, faceMaterialsBytes));
        if (faceMaterials)
            request.primMeshView.faceMaterials = { faceMaterials, faceMaterialsBytes / sizeof(uint16_t) };
    }
    return true;
}


static bool fillCookingRequestFromSourceMesh(omni::physx::PhysxCookingComputeRequest& request,
                                             SourceMeshGeometryScope& geomScope,
                                             AttachedStage* attachedStage,
                                             omni::physics::parse::ObjectKey meshKey,
                                             UsdTimeCode time,
                                             SdfPath& meshSdfPath)
{
    if (!attachedStage || !meshKey.valid())
        return false;

    const omni::physics::parse::IPhysicsSource* src = attachedStage->getSource();
    if (!src || !src->exists(meshKey) || !internal::isAType<UsdGeomMesh>(*src, meshKey))
        return false;

    meshSdfPath = attachedStage->pathFor(meshKey);
    if (meshSdfPath.IsEmpty())
        return false;

    gMeshKeyCache.getMeshKey(meshSdfPath, request.meshKey);
    request.primStageId = uint64_t(attachedStage->getStageId());
    request.primId = asInt(meshSdfPath);
    request.primTimeCode = time.GetValue();
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kExecuteCookingOnGPU, false);
    request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;

    return fillCookingMeshViewFromSource(request, geomScope, *attachedStage, meshKey);
}

bool fillConvexMeshDesc(AttachedStage* attachedStage, const UsdGeomMesh& mesh, omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc, const omni::physx::ConvexMeshCookingParams& cookingParams)
{
    UsdTimeCode time = UsdTimeCode::Default();
    const SdfPath meshSdfPath = mesh.GetPath();
    desc.meshPrimKey = attachedStage ? attachedStage->keyFor(meshSdfPath) : omni::physics::parse::ObjectKey{};

    Float3 scale = { 1.0f, 1.0f, 1.0f };
    GfMatrix4d worldXf = attachedStage ? internal::getWorldTransform(*attachedStage, desc.meshPrimKey, time)
                                       : mesh.ComputeLocalToWorldTransform(time);
    const GfTransform tr(worldXf);
    const GfVec3d sc = tr.GetScale();
    GfVec3ToFloat3(sc, scale);
    desc.meshScale = scale;
    desc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);

    desc.convexCookingParams = cookingParams;
    omni::physx::PhysxCookingComputeRequest request;
    fillCookingRequest(request, mesh.GetPrim(), time);
    // Provide the mesh geometry through IPhysicsSource so the cooking service
    // computes the CRC over the source geometry rather than reading USD; same
    // geometry feeds the cook (getConvexMesh), keeping the cache CRC consistent.
    // The scope must outlive the synchronous submission below.
    SourceMeshGeometryScope geomScope;
    if (attachedStage)
        fillCookingMeshViewFromSource(request, geomScope, *attachedStage, desc.meshPrimKey);
    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshSdfPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return false;
    cookingService->requestConvexMeshCookedData(nullptr, request, desc.convexCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillConvexMeshDesc(AttachedStage* attachedStage, omni::physics::parse::ObjectKey meshKey, omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc, const omni::physx::ConvexMeshCookingParams& cookingParams)
{
    if (!attachedStage || !meshKey.valid())
        return false;

    const UsdTimeCode time = UsdTimeCode::Default();
    const SdfPath meshSdfPath = attachedStage->pathFor(meshKey);
    if (meshSdfPath.IsEmpty())
        return false;

    desc.meshPrimKey = meshKey;

    Float3 scale = { 1.0f, 1.0f, 1.0f };
    const GfMatrix4d worldXf = internal::getWorldTransform(*attachedStage, desc.meshPrimKey, time);
    const GfTransform tr(worldXf);
    const GfVec3d sc = tr.GetScale();
    GfVec3ToFloat3(sc, scale);
    desc.meshScale = scale;
    desc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);

    desc.convexCookingParams = cookingParams;
    omni::physx::PhysxCookingComputeRequest request;
    gMeshKeyCache.getMeshKey(meshSdfPath, request.meshKey);
    request.primStageId = uint64_t(attachedStage->getStageId());
    request.primId = asInt(meshSdfPath);
    request.primTimeCode = time.GetValue();
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kExecuteCookingOnGPU, false);
    request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;

    SourceMeshGeometryScope geomScope;
    if (!fillCookingMeshViewFromSource(request, geomScope, *attachedStage, desc.meshPrimKey))
        return false;

    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshSdfPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    cookingService->requestConvexMeshCookedData(nullptr, request, desc.convexCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillConvexDecompositionDesc(AttachedStage* attachedStage, const UsdGeomMesh& mesh, omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc, const omni::physx::ConvexDecompositionCookingParams& cookingParams)
{
    UsdTimeCode time = UsdTimeCode::Default();
    const SdfPath meshSdfPath = mesh.GetPath();
    desc.meshPrimKey = attachedStage ? attachedStage->keyFor(meshSdfPath) : omni::physics::parse::ObjectKey{};
    desc.sdfMeshCookingParams.sdfResolution = 0;

    Float3 scale = { 1.0f, 1.0f, 1.0f };
    GfMatrix4d worldXf = attachedStage ? internal::getWorldTransform(*attachedStage, desc.meshPrimKey, time)
                                       : mesh.ComputeLocalToWorldTransform(time);
    const GfTransform tr(worldXf);
    const GfVec3d sc = tr.GetScale();
    GfVec3ToFloat3(sc, scale);
    desc.meshScale = scale;
    desc.convexDecompositionCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);

    desc.convexDecompositionCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    fillCookingRequest(request, mesh.GetPrim(), time);
    // Feed the cooking service mesh geometry via IPhysicsSource so it does not
    // read USD; same geometry as the cook keeps the cache CRC consistent.
    // (Convex decomposition produces convex hulls — no per-face materials.)
    SourceMeshGeometryScope geomScope;
    if (attachedStage)
        fillCookingMeshViewFromSource(request, geomScope, *attachedStage, desc.meshPrimKey);
    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshSdfPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return false;
    cookingService->requestConvexMeshDecompositionCookedData(nullptr, request, desc.convexDecompositionCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillSphereFillDesc(AttachedStage* attachedStage, const UsdGeomMesh& mesh, omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc, const omni::physx::SphereFillCookingParams& cookingParams)
{
    UsdTimeCode time = UsdTimeCode::Default();
    const SdfPath meshSdfPath = mesh.GetPath();
    desc.meshPrimKey = attachedStage ? attachedStage->keyFor(meshSdfPath) : omni::physics::parse::ObjectKey{};
    desc.sdfMeshCookingParams.sdfResolution = 0;

    Float3 scale = { 1.0f, 1.0f, 1.0f };
    GfMatrix4d worldXf = attachedStage ? internal::getWorldTransform(*attachedStage, desc.meshPrimKey, time)
                                       : mesh.ComputeLocalToWorldTransform(time);
    const GfTransform tr(worldXf);
    const GfVec3d sc = tr.GetScale();
    GfVec3ToFloat3(sc, scale);
    desc.meshScale = scale;
    desc.sphereFillCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);
    desc.sphereFillCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    fillCookingRequest(request, mesh.GetPrim(), time);
    // Feed the cooking service mesh geometry via IPhysicsSource so it does not
    // read USD; same geometry as the cook keeps the cache CRC consistent.
    // (Sphere fill produces spheres — no per-face materials.)
    SourceMeshGeometryScope geomScope;
    if (attachedStage)
        fillCookingMeshViewFromSource(request, geomScope, *attachedStage, desc.meshPrimKey);
    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshSdfPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return false;
    cookingService->requestSphereFillCookedData(nullptr, request, desc.sphereFillCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillTriangleMeshDesc(AttachedStage* attachedStage, const UsdGeomMesh& mesh, omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc, const omni::physx::TriangleMeshCookingParams& cookingParams)
{
    UsdTimeCode time = UsdTimeCode::Default();
    const SdfPath meshSdfPath = mesh.GetPath();
    desc.meshPrimKey = attachedStage ? attachedStage->keyFor(meshSdfPath) : omni::physics::parse::ObjectKey{};

    desc.sdfMeshCookingParams.sdfResolution = 0;
    desc.triangleMeshCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    fillCookingRequest(request, mesh.GetPrim(), time);
    // Feed the cooking service mesh geometry (incl. per-face materials) via
    // IPhysicsSource so it does not read USD; same geometry as the cook keeps
    // the cache CRC consistent.
    SourceMeshGeometryScope geomScope;
    if (attachedStage)
        fillCookingMeshViewFromSource(request, geomScope, *attachedStage, desc.meshPrimKey);
    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshSdfPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return false;
    cookingService->requestTriangleMeshCookedData(nullptr, request, desc.triangleMeshCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillSdfTriangleMeshDesc(AttachedStage* attachedStage, const UsdGeomMesh& mesh, omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc, const omni::physx::SdfMeshCookingParams& cookingParams)
{
    UsdTimeCode time = UsdTimeCode::Default();
    const SdfPath meshSdfPath = mesh.GetPath();
    desc.meshPrimKey = attachedStage ? attachedStage->keyFor(meshSdfPath) : omni::physics::parse::ObjectKey{};

    desc.sdfMeshCookingParams = cookingParams;
    omni::physx::PhysxCookingComputeRequest request;
    fillCookingRequest(request, mesh.GetPrim(), time);
    // Feed the cooking service mesh geometry (incl. per-face materials) via
    // IPhysicsSource so it does not read USD; same geometry as the cook keeps
    // the cache CRC consistent.
    SourceMeshGeometryScope geomScope;
    if (attachedStage)
        fillCookingMeshViewFromSource(request, geomScope, *attachedStage, desc.meshPrimKey);
    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshSdfPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return false;
    cookingService->requestSdfMeshCookedData(nullptr, request, desc.triangleMeshCookingParams, desc.sdfMeshCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillConvexDecompositionDesc(AttachedStage* attachedStage, omni::physics::parse::ObjectKey meshKey, omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc, const omni::physx::ConvexDecompositionCookingParams& cookingParams)
{
    if (!attachedStage || !meshKey.valid())
        return false;

    const UsdTimeCode time = UsdTimeCode::Default();
    SdfPath meshSdfPath;
    desc.meshPrimKey = meshKey;
    desc.sdfMeshCookingParams.sdfResolution = 0;

    Float3 scale = { 1.0f, 1.0f, 1.0f };
    const GfMatrix4d worldXf = internal::getWorldTransform(*attachedStage, desc.meshPrimKey, time);
    const GfTransform tr(worldXf);
    const GfVec3d sc = tr.GetScale();
    GfVec3ToFloat3(sc, scale);
    desc.meshScale = scale;
    desc.convexDecompositionCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);

    desc.convexDecompositionCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    SourceMeshGeometryScope geomScope;
    if (!fillCookingRequestFromSourceMesh(request, geomScope, attachedStage, meshKey, time, meshSdfPath))
        return false;

    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshSdfPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return false;
    cookingService->requestConvexMeshDecompositionCookedData(nullptr, request, desc.convexDecompositionCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillSphereFillDesc(AttachedStage* attachedStage, omni::physics::parse::ObjectKey meshKey, omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc, const omni::physx::SphereFillCookingParams& cookingParams)
{
    if (!attachedStage || !meshKey.valid())
        return false;

    const UsdTimeCode time = UsdTimeCode::Default();
    SdfPath meshSdfPath;
    desc.meshPrimKey = meshKey;
    desc.sdfMeshCookingParams.sdfResolution = 0;

    Float3 scale = { 1.0f, 1.0f, 1.0f };
    const GfMatrix4d worldXf = internal::getWorldTransform(*attachedStage, desc.meshPrimKey, time);
    const GfTransform tr(worldXf);
    const GfVec3d sc = tr.GetScale();
    GfVec3ToFloat3(sc, scale);
    desc.meshScale = scale;
    desc.sphereFillCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);
    desc.sphereFillCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    SourceMeshGeometryScope geomScope;
    if (!fillCookingRequestFromSourceMesh(request, geomScope, attachedStage, meshKey, time, meshSdfPath))
        return false;

    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshSdfPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return false;
    cookingService->requestSphereFillCookedData(nullptr, request, desc.sphereFillCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillTriangleMeshDesc(AttachedStage* attachedStage, omni::physics::parse::ObjectKey meshKey, omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc, const omni::physx::TriangleMeshCookingParams& cookingParams)
{
    if (!attachedStage || !meshKey.valid())
        return false;

    const UsdTimeCode time = UsdTimeCode::Default();
    SdfPath meshSdfPath;
    desc.meshPrimKey = meshKey;
    desc.sdfMeshCookingParams.sdfResolution = 0;
    desc.triangleMeshCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    SourceMeshGeometryScope geomScope;
    if (!fillCookingRequestFromSourceMesh(request, geomScope, attachedStage, meshKey, time, meshSdfPath))
        return false;

    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshSdfPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return false;
    cookingService->requestTriangleMeshCookedData(nullptr, request, desc.triangleMeshCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillSdfTriangleMeshDesc(AttachedStage* attachedStage, omni::physics::parse::ObjectKey meshKey, omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc, const omni::physx::SdfMeshCookingParams& cookingParams)
{
    if (!attachedStage || !meshKey.valid())
        return false;

    const UsdTimeCode time = UsdTimeCode::Default();
    SdfPath meshSdfPath;
    desc.meshPrimKey = meshKey;
    desc.sdfMeshCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    SourceMeshGeometryScope geomScope;
    if (!fillCookingRequestFromSourceMesh(request, geomScope, attachedStage, meshKey, time, meshSdfPath))
        return false;

    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshSdfPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    if (!cookingService)
        return false;
    cookingService->requestSdfMeshCookedData(nullptr, request, desc.triangleMeshCookingParams, desc.sdfMeshCookingParams);
    return meshCRCComputedSuccessfully;
}

void notifyStageReset(void)
{
    gMeshKeyCache.reset();
}

void invalidateMeshKeyCache(const SdfPath& path)
{
    gMeshKeyCache.clearMeshKey(path);
}

} // namespace usdparser
} // namespace physx
} // namespace omni
