// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-SHAPE-005
 * @covers AC-3
 *
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
 *
 * @implements REQ-COOK-SOURCE-001
 * @covers AC-1 AC-3
 */

#include <omni/physics/parse/KnownTokens.h>

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>
#include <private/omni/physx/CustomGeometryHash.h>
#include <common/foundation/Allocator.h>
#include <carb/profiler/Profile.h>

#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include <PhysXTools.h>
#include "Mass.h"
#include "PhysicsBody.h"

#include <OmniPhysX.h>
#include <omni/physx/IPhysxSettings.h>
// pxr-free half of TypeCast.h: this file only ever names the carb <-> PhysX
// overloads (toPhysX/fromPhysX/toFloat3/toFloat4), never a Gf type.
#include <common/foundation/CarbPhysXCast.h>
#include "Collision.h"
#include "Material.h"
#include "CollisionGroup.h"
#include "AttributeHelpers.h"

#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <omni/physx/IPhysxCookingService.h>

using namespace carb;
using namespace carb::tasking;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

// ObjectKey-keyed rather than SdfPath-keyed: every caller (below) already has
// the mesh's ObjectKey directly and previously materialized a PXR_NS::SdfPath
// purely to key this cache, mirroring the CctMap retype in internal/
// InternalScene.h (same ObjectKey-opaque-identity pattern, ADR-0019).
using MeshKeyMap = std::unordered_map<omni::physics::parse::ObjectKey, omni::physx::usdparser::MeshKey,
                                      omni::physics::parse::ObjectKey::Hash>;


class MeshKeyCache
{
public:

    void setMeshKey(omni::physics::parse::ObjectKey name, const MeshKey &key)
    {
        mMeshKeys[name] = key;
    }

    bool getMeshKey(omni::physics::parse::ObjectKey name, MeshKey &key) const
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

    bool clearMeshKey(omni::physics::parse::ObjectKey name)
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

PhysxShapeDesc* scaleShapeDesc(const PhysxShapeDesc& inDesc, const carb::Float3& scale)
{
    PhysxShapeDesc* desc = nullptr;

    // The incoming scale is source-neutral; everything below is internal math on
    // descriptor fields, so convert once here (ADR-0001 section 8).
    const ::physx::PxVec3 s = toPhysX(scale);

    switch (inDesc.type)
    {
    case eSphereShape:
    {
        desc = ICE_PLACEMENT_NEW(SpherePhysxShapeDesc)();
        SpherePhysxShapeDesc& sphereDesc = (SpherePhysxShapeDesc&)*desc;
        sphereDesc = (const SpherePhysxShapeDesc&)inDesc;
        const float radiusScale = fmaxf(fmaxf(fabsf(s.y), fabsf(s.x)), fabsf(s.z));
        sphereDesc.radius = sphereDesc.radius * radiusScale;
    }
    break;
    case eBoxShape:
    {
        desc = ICE_PLACEMENT_NEW(BoxPhysxShapeDesc)();
        BoxPhysxShapeDesc& boxDesc = (BoxPhysxShapeDesc&)*desc;
        boxDesc = (const BoxPhysxShapeDesc&)inDesc;

        boxDesc.halfExtents = fromPhysX(toPhysX(boxDesc.halfExtents).multiply(s));
    }
    break;
    case eCapsuleShape:
    {
        desc = ICE_PLACEMENT_NEW(CapsulePhysxShapeDesc)();
        CapsulePhysxShapeDesc& capsuleDesc = (CapsulePhysxShapeDesc&)*desc;
        capsuleDesc = (const CapsulePhysxShapeDesc&)inDesc;

        if (capsuleDesc.axis == Axis::eX)
        {
            capsuleDesc.halfHeight *= s.x;
            capsuleDesc.radius *= fmaxf(fabsf(s.y), fabsf(s.z));
        }
        else if (capsuleDesc.axis == Axis::eY)
        {
            capsuleDesc.halfHeight *= s.y;
            capsuleDesc.radius *= fmaxf(fabsf(s.x), fabsf(s.z));
        }
        else
        {
            capsuleDesc.halfHeight *= s.z;
            capsuleDesc.radius *= fmaxf(fabsf(s.y), fabsf(s.x));
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
            cylinderDesc.halfHeight *= s.x;
            cylinderDesc.radius *= fmaxf(fabsf(s.y), fabsf(s.z));
        }
        else if (cylinderDesc.axis == Axis::eY)
        {
            cylinderDesc.halfHeight *= s.y;
            cylinderDesc.radius *= fmaxf(fabsf(s.x), fabsf(s.z));
        }
        else
        {
            cylinderDesc.halfHeight *= s.z;
            cylinderDesc.radius *= fmaxf(fabsf(s.y), fabsf(s.x));
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
            coneDesc.halfHeight *= s.x;
            coneDesc.radius *= fmaxf(fabsf(s.y), fabsf(s.z));
        }
        else if (coneDesc.axis == Axis::eY)
        {
            coneDesc.halfHeight *= s.y;
            coneDesc.radius *= fmaxf(fabsf(s.x), fabsf(s.z));
        }
        else
        {
            coneDesc.halfHeight *= s.z;
            coneDesc.radius *= fmaxf(fabsf(s.y), fabsf(s.x));
        }
    }
    break;
    case eConvexMeshShape:
    {
        desc = ICE_PLACEMENT_NEW(ConvexMeshPhysxShapeDesc)();
        ConvexMeshPhysxShapeDesc& convexDesc = (ConvexMeshPhysxShapeDesc&)*desc;
        convexDesc = (const ConvexMeshPhysxShapeDesc&)inDesc;

        convexDesc.meshScale = fromPhysX(toPhysX(convexDesc.meshScale).multiply(s));
        convexDesc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(convexDesc.meshScale);
    }
    break;
    case eConvexMeshDecompositionShape:
    {
        desc = ICE_PLACEMENT_NEW(ConvexMeshDecompositionPhysxShapeDesc)();
        ConvexMeshDecompositionPhysxShapeDesc& convexDecDesc = (ConvexMeshDecompositionPhysxShapeDesc&)*desc;
        convexDecDesc = (const ConvexMeshDecompositionPhysxShapeDesc&)inDesc;

        convexDecDesc.meshScale = fromPhysX(toPhysX(convexDecDesc.meshScale).multiply(s));
        convexDecDesc.convexDecompositionCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(convexDecDesc.meshScale);
    }
    break;
    case eTriangleMeshShape:
    {
        desc = ICE_PLACEMENT_NEW(TriangleMeshPhysxShapeDesc)();
        TriangleMeshPhysxShapeDesc& meshDesc = (TriangleMeshPhysxShapeDesc&)*desc;
        meshDesc = (const TriangleMeshPhysxShapeDesc&)inDesc;

        meshDesc.meshScale = fromPhysX(toPhysX(meshDesc.meshScale).multiply(s));
    }
    break;
    case eBoundingSphereShape:
    {
        desc = ICE_PLACEMENT_NEW(BoundingSpherePhysxShapeDesc)();
        BoundingSpherePhysxShapeDesc& bsDesc = (BoundingSpherePhysxShapeDesc&)*desc;
        bsDesc = (const BoundingSpherePhysxShapeDesc&)inDesc;

        bsDesc.positionOffset = fromPhysX(toPhysX(bsDesc.positionOffset).multiply(s));
        const float radiusScale = fmaxf(fmaxf(fabsf(s.y), fabsf(s.x)), fabsf(s.z));
        bsDesc.radius = bsDesc.radius * radiusScale;
    }
    break;
    case eBoundingBoxShape:
    {
        desc = ICE_PLACEMENT_NEW(BoundingBoxPhysxShapeDesc)();
        BoundingBoxPhysxShapeDesc& bbDesc = (BoundingBoxPhysxShapeDesc&)*desc;
        bbDesc = (const BoundingBoxPhysxShapeDesc&)inDesc;

        bbDesc.positionOffset = fromPhysX(toPhysX(bbDesc.positionOffset).multiply(s));
        bbDesc.halfExtents = fromPhysX(toPhysX(bbDesc.halfExtents).multiply(s));
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
        desc->localPos = fromPhysX(toPhysX(desc->localPos).multiply(s));
        desc->localScale = fromPhysX(toPhysX(desc->localScale).multiply(s));
    }

    return desc;
}

void finalizeShape(AttachedStage& attachedStage, PhysxShapeDesc* desc, const std::vector<omni::physics::parse::ObjectKey>& materials)
{
    for (const omni::physics::parse::ObjectKey materialKey : materials)
    {
        desc->materials.push_back(getMaterial(attachedStage, materialKey));
    }

}

PhysxRigidBodyDesc* createShape(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, PhysxShapeDesc* shapeDesc, const ObjectInstance* objectInstance, ObjectId* instancedShapeId)
{
    const bool hadNoRigidBody = !shapeDesc->rigidBody.valid();

    // If we use shape for instanced create, we should not search for existing bodies.
    // getRigidBody is ObjectKey-native (ADR-0019).
    const ObjectId bodyId = instancedShapeId ? kInvalidObjectId : getRigidBody(attachedStage, key, *shapeDesc);
    PhysxRigidBodyDesc* bodyDesc = nullptr;
    if (!shapeDesc->rigidBody.valid())
    {
        bodyDesc = createStaticBody();
        bodyDesc->position = shapeDesc->localPos;
        bodyDesc->rotation = shapeDesc->localRot;
        bodyDesc->scale = shapeDesc->localScale;
        bodyDesc->sceneIds = shapeDesc->sceneIds;

        if (shapeDesc->sourceGprim != key)
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
        getCollisionShapeLocalTransform(attachedStage, key, shapeDesc->rigidBody,
            shapeDesc->localPos, shapeDesc->localRot, shapeDesc->localScale);
    }

    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createShape(key, *shapeDesc, bodyId, objectInstance);
    if (id != kInvalidObjectId)
    {
        // findOrCreateEntry(ObjectKey, pathText, ...) so this also feeds
        // PrimHierarchyStorage (and, under a USD-backed source, mPathMap) --
        // the bare ObjectKey overload only touches the Key-side maps. Mirrors
        // the createBodies/createDeformableAttachments/
        // createDeformableCollisionFilters fix in LoadStage.cpp. This one specific
        // gap (a shape registered ObjectKey-only, so SdfPath-keyed lookups can no
        // longer see it) was root-caused to a real regression the first time this
        // function was retyped: PrimUpdate.cpp's handleRemovedPrim/live-property-
        // update dispatch resolve shapes via getEntries(SdfPath)/removeEntries(SdfPath)
        // (mPathMap), so a shape missing from mPathMap silently drops out of prim-
        // removal cleanup (observed as a SIGSEGV via a dangling trigger-state
        // reference, TestContactsAndTriggers.cpp "Overlapping body removal clears
        // trigger state relationship") and out of live contactOffset/restOffset
        // property-update dispatch (observed as getContactOffset()/getRestOffset()
        // value mismatches). Do not drop this without re-verifying both symptoms.
        attachedStage.getObjectDatabase()->findOrCreateEntry(key, attachedStage.textViewFor(key), eShape, id);
    }

    if (instancedShapeId)
    {
        *instancedShapeId = id;
    }

    if (bodyId != kInvalidObjectId)
    {
        attachedStage.bufferRequestRigidBodyMassUpdate(shapeDesc->rigidBody);
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
                                   omni::physics::parse::ObjectKey meshKey,
                                   bool includeFaceMaterials)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    scope.src = src;
    scope.geom = src->getMeshAttributes(meshKey, includeFaceMaterials);
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
    // Every request is mesh-view mode now (eINPUT_MODE_FROM_PRIM_ID removed); the struct
    // default for metersPerUnit is 1.0 unless it is set here. Every cooking tolerance
    // derived from PxTolerancesScale would otherwise be wrong by 1/metersPerUnit on a
    // stage that is not authored in metres (100x on a centimetre stage). Read from the
    // source so it holds with or without a backing UsdStage.
    request.primMeshMetersPerUnit = double(attachedStage.getSourceUnits().metersPerUnit);
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


// PhysxCookingComputeRequest's primId is correlation/logging-only (IPhysxCookingService.h's
// own doc comment); it carries the legacy asInt(ObjectKey) encoding (key.handle), same as
// CookingDataAsync.cpp's/ContactReport.cpp's keyToLegacyPathInt.
static uint64_t keyToLegacyPathInt(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    (void)attachedStage;
    return key.handle;
}

static bool fillCookingRequestFromSourceMesh(omni::physx::PhysxCookingComputeRequest& request,
                                             SourceMeshGeometryScope& geomScope,
                                             AttachedStage* attachedStage,
                                             omni::physics::parse::ObjectKey meshKey,
                                             omni::physics::parse::ReadTime time)
{
    if (!attachedStage || !meshKey.valid())
        return false;

    const omni::physics::parse::IPhysicsSource* src = attachedStage->getSource();
    if (!src || !src->exists(meshKey))
        return false;
    if (!src->isA(meshKey, attachedStage->getKnownTokens().meshType))
        return false;

    // ObjectKey-keyed (see MeshKeyMap above) -- every caller already has meshKey.
    gMeshKeyCache.getMeshKey(meshKey, request.meshKey);
    // primStageId/primId are correlation keys only (REQ-COOK-SOURCE-001 AC-1); attachHandle is
    // the attach this request belongs to, and is what any completion-side attach lookup
    // resolves (ADR-0016 Decision 6).
    request.primStageId = uint64_t(attachedStage->getStageId());
    request.attachHandle = attachedStage->getAttachHandle();
    request.primId = keyToLegacyPathInt(*attachedStage, meshKey);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kExecuteCookingOnGPU, false);
    request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;

    return fillCookingMeshViewFromSource(request, geomScope, *attachedStage, meshKey);
}

bool fillConvexMeshDesc(AttachedStage* attachedStage, omni::physics::parse::ObjectKey meshKey, omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc, const omni::physx::ConvexMeshCookingParams& cookingParams)
{
    if (!attachedStage || !meshKey.valid())
        return false;

    // Existence gate, backend-agnostic: equivalent to the once-USD-only
    // pathFor(meshKey).IsEmpty() check (see fillCookingRequestFromSourceMesh above,
    // which already relies on this same src->exists() form).
    {
        const omni::physics::parse::IPhysicsSource* src = attachedStage->getSource();
        if (!src || !src->exists(meshKey))
            return false;
    }

    const omni::physics::parse::ReadTime time = omni::physics::parse::ReadTime::defaultTime();
    desc.meshPrimKey = meshKey;

    const ::physx::PxMat44d worldXf = internal::getWorldTransform(*attachedStage, desc.meshPrimKey, time);
    // Signed per-axis scale, matching the GfTransform::GetScale() this replaced --
    // scaleToSignScale() below depends on the sign for mirrored prims.
    desc.meshScale = toFloat3(getScale(worldXf));
    desc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);

    desc.convexCookingParams = cookingParams;
    omni::physx::PhysxCookingComputeRequest request;
    gMeshKeyCache.getMeshKey(meshKey, request.meshKey);
    // primStageId/primId are correlation keys only (REQ-COOK-SOURCE-001 AC-1); attachHandle is
    // the attach identity (ADR-0016 Decision 6).
    request.primStageId = uint64_t(attachedStage->getStageId());
    request.attachHandle = attachedStage->getAttachHandle();
    request.primId = keyToLegacyPathInt(*attachedStage, meshKey);
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
        gMeshKeyCache.setMeshKey(meshKey, desc.meshKey);
    };
    IPhysxCookingService* cookingService = OmniPhysX::getInstance().getPhysXSetup().getCookingServiceInterface();
    cookingService->requestConvexMeshCookedData(nullptr, request, desc.convexCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillConvexDecompositionDesc(AttachedStage* attachedStage, omni::physics::parse::ObjectKey meshKey, omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc, const omni::physx::ConvexDecompositionCookingParams& cookingParams)
{
    if (!attachedStage || !meshKey.valid())
        return false;

    const omni::physics::parse::ReadTime time = omni::physics::parse::ReadTime::defaultTime();
    desc.meshPrimKey = meshKey;
    desc.sdfMeshCookingParams.sdfResolution = 0;

    const ::physx::PxMat44d worldXf = internal::getWorldTransform(*attachedStage, desc.meshPrimKey, time);
    // Signed per-axis scale, matching the GfTransform::GetScale() this replaced --
    // scaleToSignScale() below depends on the sign for mirrored prims.
    desc.meshScale = toFloat3(getScale(worldXf));
    desc.convexDecompositionCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);

    desc.convexDecompositionCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    SourceMeshGeometryScope geomScope;
    if (!fillCookingRequestFromSourceMesh(request, geomScope, attachedStage, meshKey, time))
        return false;

    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshKey, desc.meshKey);
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

    const omni::physics::parse::ReadTime time = omni::physics::parse::ReadTime::defaultTime();
    desc.meshPrimKey = meshKey;
    desc.sdfMeshCookingParams.sdfResolution = 0;

    const ::physx::PxMat44d worldXf = internal::getWorldTransform(*attachedStage, desc.meshPrimKey, time);
    // Signed per-axis scale, matching the GfTransform::GetScale() this replaced --
    // scaleToSignScale() below depends on the sign for mirrored prims.
    desc.meshScale = toFloat3(getScale(worldXf));
    desc.sphereFillCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);
    desc.sphereFillCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    SourceMeshGeometryScope geomScope;
    if (!fillCookingRequestFromSourceMesh(request, geomScope, attachedStage, meshKey, time))
        return false;

    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshKey, desc.meshKey);
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

    const omni::physics::parse::ReadTime time = omni::physics::parse::ReadTime::defaultTime();
    desc.meshPrimKey = meshKey;
    desc.sdfMeshCookingParams.sdfResolution = 0;
    desc.triangleMeshCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    SourceMeshGeometryScope geomScope;
    if (!fillCookingRequestFromSourceMesh(request, geomScope, attachedStage, meshKey, time))
        return false;

    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshKey, desc.meshKey);
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

    const omni::physics::parse::ReadTime time = omni::physics::parse::ReadTime::defaultTime();
    desc.meshPrimKey = meshKey;
    desc.sdfMeshCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    SourceMeshGeometryScope geomScope;
    if (!fillCookingRequestFromSourceMesh(request, geomScope, attachedStage, meshKey, time))
        return false;

    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(meshKey, desc.meshKey);
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

void invalidateMeshKeyCache(omni::physics::parse::ObjectKey key)
{
    gMeshKeyCache.clearMeshKey(key);
}

} // namespace usdparser
} // namespace physx
} // namespace omni
