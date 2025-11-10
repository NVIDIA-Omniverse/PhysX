// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXSceneQuery.h"
#include "internal/InternalScene.h"
#include "PhysXTools.h"
#include "PhysXScene.h"
#include "ObjectDataQuery.h"
#include "ConeCylinderConvexMesh.h"

#include "usdLoad/Collision.h"
#include "CookingDataAsync.h"
#include <omni/physx/IPhysx.h>

extern void* getPhysXPtr(const pxr::SdfPath& path, omni::physx::PhysXType type);

using namespace ::physx;
using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace cookingdataasync;
using namespace pxr;
using namespace carb;

namespace omni
{
namespace physx
{

static PxBoxGeometry usdToPxBoxGeometry(const UsdPrim& prim, const pxr::GfVec3d& scale)
{
    CARB_ASSERT(prim.IsA<UsdGeomCube>());
    UsdGeomCube shape(prim);
    double sizeAttr;
    shape.GetSizeAttr().Get(&sizeAttr);
    sizeAttr = abs(sizeAttr) * 0.5f; // convert cube edge length to half extend
    return PxBoxGeometry(toPhysX(scale * sizeAttr));
}

static PxSphereGeometry usdToPxSphereGeometry(const UsdPrim& prim, const pxr::GfVec3d& scale)
{
    CARB_ASSERT(prim.IsA<UsdGeomSphere>());
    // as we dont support scale in physics and scale can be non uniform
    // we pick the largest scale value as the sphere radius base
    const double tolerance = 1e-4;
    if (abs(scale[0] - scale[1]) > tolerance || abs(scale[0] - scale[2]) > tolerance || abs(scale[2] - scale[1]) > tolerance)
    {
        CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", prim.GetPath().GetText());
    }    

    double radius = std::max(std::max(abs(scale[1]), abs(scale[0])), abs(scale[2]));
    UsdGeomSphere shape(prim);
    double radiusAttr;
    shape.GetRadiusAttr().Get(&radiusAttr);
    radius *= radiusAttr;
    return PxSphereGeometry((float)radius);
}

static PxCapsuleGeometry usdToPxCapsuleGeometry(const UsdPrim& prim, const pxr::GfVec3d& scale)
{
    CARB_ASSERT(prim.IsA<UsdGeomCapsule>());
    // as we dont support scale in physics and scale can be non uniform
    // we pick the largest scale value as the sphere radius base

    UsdGeomCapsule shape(prim);
    double radius;
    shape.GetRadiusAttr().Get(&radius);
    double height;
    shape.GetHeightAttr().Get(&height);

    TfToken capAxis;
    shape.GetAxisAttr().Get(&capAxis);
    const double tolerance = 1e-4;
    // as we dont support scale in physics and scale can be non uniform
    // we pick the largest scale value as the sphere radius base
    if (capAxis == UsdPhysicsTokens.Get()->x)
    {
        height *= scale[0];
        radius *= std::max(abs(scale[1]), abs(scale[2]));
        if (abs(scale[2] - scale[1]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", prim.GetPath().GetText());
        }    
    }
    else if (capAxis == UsdPhysicsTokens.Get()->y)
    {
        height *= scale[1];
        radius *= std::max(abs(scale[0]), abs(scale[2]));
        if (abs(scale[2] - scale[0]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", prim.GetPath().GetText());
        }    
    }
    else
    {
        height *= scale[2];
        radius *= std::max(abs(scale[1]), abs(scale[0]));
        if (abs(scale[1] - scale[0]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", prim.GetPath().GetText());
        }
    }
    return PxCapsuleGeometry((float)radius, (float)(height * 0.5));
}


static PxConvexMeshGeometry usdCylinderOrConeToPxConvexMeshGeometry(const UsdPrim& prim, const pxr::GfVec3d& scale)
{
    CARB_ASSERT(prim.IsA<UsdGeomCylinder>() || prim.IsA<UsdGeomCone>());
    double radius;
    double height;
    TfToken capAxis;

    if(prim.IsA<UsdGeomCylinder>())
    {
        UsdGeomCylinder shape(prim);
        shape.GetRadiusAttr().Get(&radius);
        shape.GetHeightAttr().Get(&height);
        shape.GetAxisAttr().Get(&capAxis);
    }
    else
    {
        UsdGeomCone shape(prim);
        shape.GetRadiusAttr().Get(&radius);
        shape.GetHeightAttr().Get(&height);
        shape.GetAxisAttr().Get(&capAxis);
    }

    usdparser::Axis axis = eZ;
    const double tolerance = 1e-4;
    // as we dont support scale in physics and scale can be non uniform
    // we pick the largest scale value as the sphere radius base
    if (capAxis == UsdPhysicsTokens.Get()->x)
    {
        height *= scale[0];
        radius *= std::max(abs(scale[1]), abs(scale[2]));
        if (abs(scale[2] - scale[1]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", prim.GetPath().GetText());
        }
        axis = eX;
    }
    else if (capAxis == UsdPhysicsTokens.Get()->y)
    {
        height *= scale[1];
        radius *= std::max(abs(scale[0]), abs(scale[2]));
        if (abs(scale[2] - scale[0]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", prim.GetPath().GetText());
        }
        axis = eY;
    }
    else
    {
        height *= scale[2];
        radius *= std::max(abs(scale[1]), abs(scale[0]));
        if (abs(scale[1] - scale[0]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", prim.GetPath().GetText());
        }
    }

    const PxVec3 mesh_scale = getConeOrCylinderScale((float) height * 0.5, radius, axis);
    PxMeshScale meshScale = PxMeshScale(mesh_scale);
    PxConvexMeshGeometryFlags flags = PxConvexMeshGeometryFlag::eTIGHT_BOUNDS;
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    if(prim.IsA<UsdGeomCylinder>())
    {
        return PxConvexMeshGeometry(physxSetup.getCylinderConvexMesh(axis), meshScale);
    }
    else
    {
        return PxConvexMeshGeometry(physxSetup.getConeConvexMesh(axis), meshScale);
    }
}

static PxConvexMeshGeometry usdToPxConvexMeshGeometry(UsdPrim& prim, const pxr::GfVec3d& scale)
{
    CARB_ASSERT(prim.IsA<UsdGeomMesh>());
    usdparser::ConvexMeshPhysxShapeDesc convexMeshDesc;
    if (fillConvexMeshDesc(UsdGeomMesh(prim), convexMeshDesc, convexMeshDesc.convexCookingParams))
    {
        Float3 mesh_scale = { 1.0f, 1.0f, 1.0f };
        GfVec3ToFloat3(scale, mesh_scale);
        convexMeshDesc.meshScale = mesh_scale;
        convexMeshDesc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(mesh_scale);
        PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
        CookingDataAsync* cookingDataAsync = physxSetup.getCookingDataAsync();

        if (!cookingDataAsync)
        {
            CARB_LOG_ERROR("omni::physx::usdToPxConvexMeshGeometry: Failed to cook convex mesh data for prim: %s - cooking service unavailable", prim.GetPath().GetText());
            return PxConvexMeshGeometry();
        }

        PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(convexMeshDesc, prim, false);

        if (convexMesh)
        {
            const PxMeshScale meshScale(PxVec3(mesh_scale.x, mesh_scale.y, mesh_scale.z).abs());
            return PxConvexMeshGeometry(convexMesh, meshScale);
        }
        else
        {
            CARB_LOG_ERROR("omni::physx::usdToPxConvexMeshGeometry: Failed to cook convex mesh data for prim: %s", prim.GetPath().GetText());
            return PxConvexMeshGeometry();
        }
    }
    else
    {
        CARB_LOG_ERROR("omni::physx::usdToPxConvexMeshGeometry: Provided mesh is not a valid mesh: %s", prim.GetPath().GetText());
        return PxConvexMeshGeometry();
    }
}


using geometrySceneQueryFn = std::function<bool(const PxGeometry& geom, const PxTransform& transform)>;

static bool doShapeSceneQuery(uint64_t rPrimPath, geometrySceneQueryFn queryFn)
{
    const SdfPath primPath = intToPath(rPrimPath);
    UsdPrim prim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(primPath);
    if(!prim.IsValid())
    {
        CARB_LOG_ERROR("omni::physx::doShapeSceneQuery: Provided prim is invalid: %s", prim.GetPath().GetText());
        return false;
    }
    if (!prim.IsA<UsdGeomGprim>())
    {
        CARB_LOG_ERROR("omni::physx::doShapeSceneQuery: Provided prim is not a UsdGeomGPrim: %s", prim.GetPath().GetText());
        return false;
    }
    const pxr::GfTransform tr(UsdGeomXform(prim).ComputeLocalToWorldTransform(UsdTimeCode::Default()));
    const pxr::GfVec3d sc = tr.GetScale();
    PxTransform transform(toPhysX(tr.GetTranslation()), toPhysX(tr.GetRotation().GetQuat()));

    if (prim.IsA<UsdGeomCube>())
    {
        return queryFn(usdToPxBoxGeometry(prim, sc), transform);
    }
    else if (prim.IsA<UsdGeomSphere>())
    {
        return queryFn(usdToPxSphereGeometry(prim, sc), transform);
    }
    else if (prim.IsA<UsdGeomCapsule>())
    {
        const UsdGeomCapsule shape(prim);
        TfToken capAxis;
        shape.GetAxisAttr().Get(&capAxis);
        const PxQuat fixupQ = fixupCapsuleQuat(capAxis);
        PxQuat q = toPhysX(tr.GetRotation().GetQuat());
        q = q * fixupQ;
        transform.q = q;
        return queryFn(usdToPxCapsuleGeometry(prim, sc), transform);
    } 
    PxConvexMeshGeometry geom;
    if (prim.IsA<UsdGeomCylinder>() || prim.IsA<UsdGeomCone>())
    {
        geom = usdCylinderOrConeToPxConvexMeshGeometry(prim, sc);
    }
    else if (prim.IsA<UsdGeomMesh>())
    {
        geom = usdToPxConvexMeshGeometry(prim, sc);
    }
    if(geom.isValid())
    {
        return queryFn(geom, transform);
    }
    CARB_LOG_ERROR("omni::physx::doShapeSceneQuery: Provided prim is not a valid UsdGeom type (supported types are: box, sphere, capsule, convex): %s", prim.GetPath().GetText());
    return false;
}

static bool doSphereSceneQuery(float radius, const carb::Float3& pos, geometrySceneQueryFn queryFn)
{
    const PxSphereGeometry sphereGeom(radius);
    const PxTransform transform(toPhysX(pos));

    return queryFn(sphereGeom, transform);
}

static bool doBoxSceneQuery(const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot, geometrySceneQueryFn queryFn)
{
    const PxBoxGeometry boxGeom(toPhysX(halfExtent));
    const PxTransform transform(toPhysX(pos), toPhysXQuat(rot));

    return queryFn(boxGeom, transform);
}

static bool doMeshSceneQuery(uint64_t meshPath, geometrySceneQueryFn queryFn)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    CookingDataAsync* cookingDataAsync = physxSetup.getCookingDataAsync();

    if (cookingDataAsync== nullptr || physxSetup.getPhysXScenes().empty())
    {
        return false;
    }

    const SdfPath primPath = intToPath(meshPath);
    UsdPrim prim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(primPath);

    if (!prim || !prim.IsA<UsdGeomMesh>())
    {
        return false;
    }
    usdparser::ConvexMeshPhysxShapeDesc convexMeshDesc;

    if (!fillConvexMeshDesc(UsdGeomMesh(prim), convexMeshDesc, convexMeshDesc.convexCookingParams))
    {
        return false;
    }
    Float3 scale = { 1.0f, 1.0f, 1.0f };

    UsdGeomXformCache xfCache;
    const pxr::GfTransform tr(xfCache.GetLocalToWorldTransform(prim));
    const pxr::GfVec3d sc = tr.GetScale();
    GfVec3ToFloat3(sc, scale);
    convexMeshDesc.meshScale = scale;
    convexMeshDesc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(scale);

    PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(convexMeshDesc, prim, false);
    if(!convexMesh)
    {
        return false;
    }

    const PxMeshScale meshScale(PxVec3(scale.x, scale.y, scale.z).abs());
    const PxTransform transform(toPhysX(tr.GetTranslation()), toPhysX(tr.GetRotation().GetQuat()));

    return queryFn(PxConvexMeshGeometry(convexMesh, meshScale), transform);
}

bool raycastClosest(const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHit& outHit, bool bothSides)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (!physxSetup.getPhysXScenes().empty())
    {
        const PxVec3 pos = toPhysX(origin);
        PxVec3 dir = toPhysX(unitDir);
        dir.normalize();

        PxRaycastHit hit;
        PxHitFlags hitFlags = PxHitFlag::eDEFAULT;
        if (bothSides)
            hitFlags |= PxHitFlag::eMESH_BOTH_SIDES;
        hit.distance = FLT_MAX;

        bool ret = false;
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            PxRaycastHit localHit;
            const PxScene* scene = ref.second->getScene();
            if (!scene)
                continue;
            const bool localRet = PxSceneQueryExt::raycastSingle(*scene, pos, dir, distance, hitFlags, localHit);
            if (localRet && localHit.distance < hit.distance)
            {
                hit = localHit;
                ret = localRet;
            }
        }
        
        if (ret)
        {
            outHit.distance = hit.distance;
            outHit.normal = (const Float3&)hit.normal;
            outHit.position = (const Float3&)hit.position;

            // resolve face index from PhysX index to USD face index
            FaceIndexResolve indexResolve(hit.shape);
            outHit.faceIndex = indexResolve.resolveFaceIndex(hit.faceIndex);

            const ObjectId shapeIndex = (ObjectId)hit.shape->userData;
            const ObjectId bodyIndex = (ObjectId)hit.actor->userData;
            outHit.collision = shapeIndex < db.getRecords().size() ? asInt(db.getRecords()[shapeIndex].mPath) : 0;
            if (bodyIndex < db.getRecords().size())
            {
                const InternalDatabase::Record& record = db.getRecords()[bodyIndex];
                outHit.rigidBody = asInt(record.mPath);
                if (record.mType == ePTActor)
                {
                    outHit.protoIndex = ((InternalActor*)record.mInternalPtr)->mInstanceIndex;
                }                
            }
            else
            {
                outHit.rigidBody = 0;
                outHit.protoIndex = 0xFFFFFFFF;
            }
            PxBaseMaterial* baseMaterial = hit.shape->getMaterialFromInternalFaceIndex(hit.faceIndex);
            PX_ASSERT(!baseMaterial || baseMaterial->getConcreteType() == PxConcreteType::eMATERIAL);
            const PxMaterial* material = (baseMaterial != nullptr) ? baseMaterial->is<PxMaterial>() : nullptr;
            if (material && material->userData)
            {
                const size_t materialIndex = (size_t)material->userData;
                outHit.material = materialIndex < db.getRecords().size() ? asInt(db.getRecords()[materialIndex].mPath) : 0;
            }
            else
            {
                outHit.material = 0;
            }
        }
        return ret;
    }
    return false;
}

bool raycastAny(const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (!physxSetup.getPhysXScenes().empty())
    {
        const PxVec3 pos = toPhysX(origin);
        PxVec3 dir = toPhysX(unitDir);
        dir.normalize();
        
        PxHitFlags hitFlags = PxHitFlag::eDEFAULT | PxHitFlag::eANY_HIT;
        if (bothSides)
            hitFlags |= PxHitFlag::eMESH_BOTH_SIDES;       

        PxSceneQueryFilterData fdAny;
        fdAny.flags |= PxQueryFlag::eANY_HIT;
        PxRaycastBuffer buf;
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {            
            const PxScene* scene = ref.second->getScene();
            if (!scene)
                continue;
            scene->raycast(pos, dir, distance, buf, hitFlags, fdAny);
            if(buf.hasBlock)
                return true;
        }
    }
    return false;
}

template <typename T, typename QueryHit, typename PxQueryHit>
bool reportLocationHit(const PxQueryHit& hit, T reportFn, const InternalPhysXDatabase& db)
{
    QueryHit queryHit;
    const ObjectId shapeIndex = (ObjectId)hit.shape->userData;
    const ObjectId bodyIndex = (ObjectId)hit.actor->userData;
    queryHit.collision = shapeIndex < db.getRecords().size() ? asInt(db.getRecords()[shapeIndex].mPath) : 0;
    if (bodyIndex < db.getRecords().size())
    {
        const InternalDatabase::Record& record = db.getRecords()[bodyIndex];
        queryHit.rigidBody = asInt(record.mPath);
        if (record.mType == ePTActor)
        {
            queryHit.protoIndex = ((InternalActor*)record.mInternalPtr)->mInstanceIndex;
        }
    }
    else
    {
        queryHit.rigidBody = 0;
        queryHit.protoIndex = 0xFFFFFFFF;
    }

    queryHit.normal = fromPhysX(hit.normal);
    queryHit.position = fromPhysX(hit.position);
    queryHit.distance = hit.distance;
    // resolve face index from PhysX index to USD face index
    FaceIndexResolve indexResolve(hit.shape);
    queryHit.faceIndex = indexResolve.resolveFaceIndex(hit.faceIndex);

    PxBaseMaterial* baseMaterial = hit.shape->getMaterialFromInternalFaceIndex(hit.faceIndex);
    PX_ASSERT(!baseMaterial || baseMaterial->getConcreteType() == PxConcreteType::eMATERIAL);
    const PxMaterial* material = (baseMaterial != nullptr) ? baseMaterial->is<PxMaterial>() : nullptr;

    if (material && material->userData)
    {
        const size_t materialIndex = (size_t)material->userData;
        queryHit.material = materialIndex < db.getRecords().size() ? asInt(db.getRecords()[materialIndex].mPath) : 0;
    }
    else
    {
        queryHit.material = 0;
    }

    return reportFn(queryHit);
}

struct RaycastCallback : PxRaycastCallback
{
    RaycastCallback(PxRaycastHit* buf, PxU32 maxNb, const RaycastHitReportFn& reportFn, const InternalPhysXDatabase& inDb)
        : PxRaycastCallback(buf, maxNb), sceneQueryReportFn(reportFn), numHits(0), db(inDb)
    {
    }

    virtual PxAgain processTouches(const PxRaycastHit* hits, PxU32 count)
    {
        numHits += count;
        if (sceneQueryReportFn)
        {
            for (PxU32 i = 0; i < count; i++)
            {
                if(!reportLocationHit<RaycastHitReportFn, RaycastHit, PxRaycastHit>(hits[i], sceneQueryReportFn, db))
                    return false;
            }
        }        
        return true;
    }

    const InternalPhysXDatabase& db;
    const RaycastHitReportFn& sceneQueryReportFn;
    uint32_t numHits;
};

bool raycastAll(const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHitReportFn reportFn, bool bothSides)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (!physxSetup.getPhysXScenes().empty())
    {
        const PxVec3 pos = toPhysX(origin);
        PxVec3 dir = toPhysX(unitDir);
        dir.normalize();
        
        PxHitFlags hitFlags = PxHitFlag::eDEFAULT;
        if (bothSides)
            hitFlags |= PxHitFlag::eMESH_BOTH_SIDES;
        
        const PxU32 hitBufferSize = 256;
        PxRaycastHit hitBuffer[hitBufferSize];
        RaycastCallback cb(hitBuffer, hitBufferSize, reportFn, db);
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            const PxScene* scene = ref.second->getScene();
            if (!scene)
                continue;
            scene->raycast(pos, dir, distance, cb, hitFlags);
        }
        return cb.numHits ? true : false;
    }
    return false;
}

struct SceneQueryFilterCallbackIgnorePrim : PxQueryFilterCallback
{
    SceneQueryFilterCallbackIgnorePrim() {};

    SceneQueryFilterCallbackIgnorePrim(uint64_t primPath, bool bMultiple=false) : bMultiple(bMultiple)
    {
        const SdfPath& path = intToPath(primPath);
        shapePrim = (PxShape *)(getPhysXPtr(path, ePTShape));
    }

    virtual ::physx::PxQueryHitType::Enum preFilter(const ::physx::PxFilterData& filterData0,
                                            const ::physx::PxShape* shape,
                                            const ::physx::PxRigidActor* actor,
                                            ::physx::PxHitFlags&) override
    {
        if(shapePrim && shape == shapePrim)
        {
            return ::physx::PxQueryHitType::eNONE;
        }
        return (bMultiple ? ::physx::PxQueryHitType::eTOUCH : ::physx::PxQueryHitType::eBLOCK);
    }
    
    virtual ::physx::PxQueryHitType::Enum postFilter(const ::physx::PxFilterData&, const ::physx::PxQueryHit&,
                                                    const ::physx::PxShape* shape, const ::physx::PxRigidActor* actor) 
    {
        if(shapePrim && shape == shapePrim)
        {
            return ::physx::PxQueryHitType::eNONE;
        }
        return (bMultiple ? ::physx::PxQueryHitType::eTOUCH : ::physx::PxQueryHitType::eBLOCK);
    }

    const ::physx::PxShape* shapePrim;
    bool bMultiple;
};

static bool sweepInternalClosest(const PxGeometry& geom, const PxTransform& transform, const carb::Float3& unitDir, float distance, SweepHit& outHit, bool bothSides, uint64_t primSelf=0)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (!physxSetup.getPhysXScenes().empty())
    {
        PxVec3 dir(unitDir.x, unitDir.y, unitDir.z);
        dir.normalize();

        PxSweepHit hit;
        hit.distance = FLT_MAX;
        PxHitFlags hitFlags = PxHitFlag::eDEFAULT;
        if (bothSides)
            hitFlags |= PxHitFlag::eMESH_BOTH_SIDES;
        bool ret = false;
        SceneQueryFilterCallbackIgnorePrim filter;
        PxSceneQueryFilterData filterData;
        if(primSelf)
        {
            filter = SceneQueryFilterCallbackIgnorePrim(primSelf);
            filterData.flags |= PxQueryFlag::ePREFILTER;
        }
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            PxSweepHit localHit;
            const PxScene* scene = ref.second->getScene();
            if (!scene)
                continue;
            const bool localRet = PxSceneQueryExt::sweepSingle(*scene, geom, transform, dir, distance, hitFlags, localHit, filterData, (primSelf ? &filter : nullptr));
            if (localRet && localHit.distance < hit.distance)
            {
                ret = true;
                hit = localHit;
            }
        }

        if (ret)
        {
            outHit.distance = hit.distance;
            outHit.normal = (const Float3&)hit.normal;
            outHit.position = (const Float3&)hit.position;

            const ObjectId shapeIndex = (ObjectId)hit.shape->userData;
            const ObjectId bodyIndex = (ObjectId)hit.actor->userData;
            outHit.collision = shapeIndex < db.getRecords().size() ? asInt(db.getRecords()[shapeIndex].mPath) : 0;
            if (bodyIndex < db.getRecords().size())
            {
                const InternalDatabase::Record& record = db.getRecords()[bodyIndex];
                outHit.rigidBody = asInt(record.mPath);
                if (record.mType == ePTActor)
                {
                    outHit.protoIndex = ((InternalActor*)record.mInternalPtr)->mInstanceIndex;
                }
            }
            else
            {
                outHit.rigidBody = 0;
                outHit.protoIndex = 0xFFFFFFFF;
            }

            // resolve face index from PhysX index to USD face index
            FaceIndexResolve indexResolve(hit.shape);
            outHit.faceIndex = indexResolve.resolveFaceIndex(hit.faceIndex);

            PxBaseMaterial* baseMaterial = hit.shape->getMaterialFromInternalFaceIndex(hit.faceIndex);
            PX_ASSERT(!baseMaterial || baseMaterial->getConcreteType() == PxConcreteType::eMATERIAL);
            const PxMaterial* material = (baseMaterial != nullptr) ? baseMaterial->is<PxMaterial>() : nullptr;

            if (material && material->userData)
            {
                const size_t materialIndex = (size_t)material->userData;
                outHit.material = materialIndex < db.getRecords().size() ? asInt(db.getRecords()[materialIndex].mPath) : 0;
            }
            else
            {
                outHit.material = 0;
            }

        }
        return ret;
    }
    return false;
}

bool sweepSphereClosest(float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, SweepHit& outHit, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepInternalClosest(geom, transform, unitDir, distance, outHit, bothSides);
    };

    return doSphereSceneQuery(radius, origin, sweepFn);
}

bool sweepBoxClosest(const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot, const carb::Float3& unitDir, float distance, SweepHit& outHit, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepInternalClosest(geom, transform, unitDir, distance, outHit, bothSides);
    };
    
    return doBoxSceneQuery(halfExtent, pos, rot, sweepFn);
}

bool sweepMeshClosest(uint64_t meshPath, const carb::Float3& unitDir, float distance, SweepHit& outHit, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepInternalClosest(geom, transform, unitDir, distance, outHit, bothSides, meshPath);
    };

    return doMeshSceneQuery(meshPath, sweepFn);
}

bool sweepShapeClosest(uint64_t rPrimPath, const carb::Float3& unitDir, float distance, SweepHit& outHit, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepInternalClosest(geom, transform, unitDir, distance, outHit, bothSides, rPrimPath);
    };

    return doShapeSceneQuery(rPrimPath, sweepFn);
}

static bool sweepAnyInternal(const PxGeometry& geom, const PxTransform& transform,
                        const carb::Float3& unitDir,
                        float distance,
                        bool bothSides, uint64_t primSelf=0)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (!physxSetup.getPhysXScenes().empty())
    {
        PxVec3 dir(unitDir.x, unitDir.y, unitDir.z);
        dir.normalize();
        
        PxHitFlags hitFlags = PxHitFlag::eDEFAULT | PxHitFlag::eANY_HIT;
        if (bothSides)
            hitFlags |= PxHitFlag::eMESH_BOTH_SIDES;

        PxSceneQueryFilterData fdAny;
        fdAny.flags |= PxQueryFlag::eANY_HIT;
        SceneQueryFilterCallbackIgnorePrim filter;
        if(primSelf)
        {
            filter = SceneQueryFilterCallbackIgnorePrim(primSelf);
            fdAny.flags |= PxQueryFlag::ePREFILTER;
        }        
        PxSweepBuffer buf;
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            const PxScene* scene = ref.second->getScene();
            if (!scene)
                continue;
            scene->sweep(geom, transform, dir, distance, buf, hitFlags, fdAny, (primSelf ? &filter : nullptr));
            if (buf.hasBlock)
            {
                return true;
            }
        }
        return false;
    }
    return false;
}

bool sweepSphereAny(float radius,
                        const carb::Float3& origin,
                        const carb::Float3& unitDir,
                        float distance,
                        bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepAnyInternal(geom, transform, unitDir, distance, bothSides);
    };

    return doSphereSceneQuery(radius, origin, sweepFn);
}

bool sweepBoxAny(const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot, const carb::Float3& unitDir, float distance, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepAnyInternal(geom, transform, unitDir, distance, bothSides);
    };

    return doBoxSceneQuery(halfExtent, pos, rot, sweepFn);
}

bool sweepMeshAny(uint64_t meshPath, const carb::Float3& unitDir, float distance, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepAnyInternal(geom, transform, unitDir, distance, bothSides, meshPath);
    };

    return doMeshSceneQuery(meshPath, sweepFn);
}

bool sweepShapeAny(uint64_t rPrimPath, const carb::Float3& unitDir, float distance, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepAnyInternal(geom, transform, unitDir, distance, bothSides, rPrimPath);
    };

    return doShapeSceneQuery(rPrimPath, sweepFn);
}

struct SweepCallback : PxSweepCallback
{
    SweepCallback(PxSweepHit* buf, PxU32 maxNb, const SweepHitReportFn& reportFn, const InternalPhysXDatabase& inDb)
        : PxSweepCallback(buf, maxNb), sceneQueryReportFn(reportFn), numHits(0), db(inDb)
    {
    }

    virtual PxAgain processTouches(const PxSweepHit* hits, PxU32 count)
    {
        numHits += count;
        if (sceneQueryReportFn)
        {
            for (PxU32 i = 0; i < count; i++)
            {
                if (!reportLocationHit<SweepHitReportFn, SweepHit, PxSweepHit>(hits[i], sceneQueryReportFn, db))
                    return false;
            }
        }        
        return true;
    }

    const InternalPhysXDatabase& db;
    const SweepHitReportFn& sceneQueryReportFn;
    uint32_t numHits;
};


bool sweepAllInternal(const PxGeometry& geom, const PxTransform& transform, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides, uint64_t primSelf=0)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (!physxSetup.getPhysXScenes().empty())
    {
        PxVec3 dir = toPhysX(unitDir);
        dir.normalize();

        PxHitFlags hitFlags = PxHitFlag::eDEFAULT;
        if (bothSides)
            hitFlags |= PxHitFlag::eMESH_BOTH_SIDES;

        const PxU32 hitBufferSize = 256;
        PxSweepHit hitBuffer[hitBufferSize];
        SweepCallback cb(hitBuffer, hitBufferSize, reportFn, db);
        SceneQueryFilterCallbackIgnorePrim filter;
        PxSceneQueryFilterData filterData;
        if(primSelf)
        {
            filter = SceneQueryFilterCallbackIgnorePrim(primSelf, true);
            filterData.flags |= PxQueryFlag::ePREFILTER;
        }
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            const PxScene* scene = ref.second->getScene();
            if (!scene)
                continue;
            scene->sweep(geom, transform, dir, distance, cb, hitFlags, filterData, (primSelf? &filter : nullptr));
        }
        return cb.numHits ? true : false;
    }
    return false;
}

bool sweepSphereAll(float radius, const carb::Float3& origin, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepAllInternal(geom, transform, unitDir, distance, reportFn, bothSides);
    };

    return doSphereSceneQuery(radius, origin, sweepFn);
}

bool sweepBoxAll(const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepAllInternal(geom, transform, unitDir, distance, reportFn, bothSides);
    };

    return doBoxSceneQuery(halfExtent, pos, rot, sweepFn);
}

bool sweepMeshAll(uint64_t meshPath, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepAllInternal(geom, transform, unitDir, distance, reportFn, bothSides, meshPath);
    };

    return doMeshSceneQuery(meshPath, sweepFn);
}

bool sweepShapeAll(uint64_t rPrimPath, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepAllInternal(geom, transform, unitDir, distance, reportFn, bothSides, rPrimPath);
    };

    return doShapeSceneQuery(rPrimPath, sweepFn);
}

static bool reportOverlapHit(const PxOverlapHit& hit, const OverlapHitReportFn& reportFn, const InternalPhysXDatabase& db)
{
    OverlapHit queryHit;
    const ObjectId shapeIndex = (ObjectId)hit.shape->userData;
    const ObjectId bodyIndex = (ObjectId)hit.actor->userData;

    const size_t size = db.getRecords().size();
    queryHit.collision = shapeIndex < size ? asInt(db.getRecords()[shapeIndex].mPath) : 0;
    if (bodyIndex < db.getRecords().size())
    {
        const InternalDatabase::Record& record = db.getRecords()[bodyIndex];
        queryHit.rigidBody = asInt(record.mPath);
        if (record.mType == ePTActor)
        {
            queryHit.protoIndex = ((InternalActor*)record.mInternalPtr)->mInstanceIndex;
        }
    }
    else
    {
        queryHit.rigidBody = 0;
        queryHit.protoIndex = 0xFFFFFFFF;
    }
    return reportFn(queryHit);
}

struct OverlapCallback : PxOverlapCallback
{
    OverlapCallback(PxOverlapHit* buf, PxU32 maxNb, const OverlapHitReportFn& reportFn, const InternalPhysXDatabase& inDb)
        : PxOverlapCallback(buf, maxNb), sceneQueryReportFn(reportFn), numHits(0), db(inDb)
    {
    }    

    virtual PxAgain processTouches(const PxOverlapHit* hits, PxU32 count)
    {
        numHits += count;
        if (sceneQueryReportFn)
        {
            for (PxU32 i = 0; i < count; i++)
            {
                if (!reportOverlapHit(hits[i], sceneQueryReportFn, db))
                    return false;
            }
        }        
        return true;
    }

    const InternalPhysXDatabase& db;
    const OverlapHitReportFn& sceneQueryReportFn;
    uint32_t numHits;
};

static uint32_t overlapInternal(const PxGeometry& geometry, const PxTransform& transform, const OverlapHitReportFn& reportFn, bool anyHit)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (physxSetup.getPhysXScenes().empty())
    {
        return 0;
    }

    uint32_t numHits = 0;
    PxQueryFilterData queryFilterData;
    if (anyHit)
    {
        queryFilterData.flags |= (PxQueryFlag::eANY_HIT | PxQueryFlag::eNO_BLOCK);
        PxOverlapBuffer buf;
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            const PxScene* scene = ref.second->getScene();
            if (!scene)
                continue;
            scene->overlap(geometry, transform, buf, queryFilterData);
            if (buf.hasBlock)
                return 1;
        }        
    }
    else
    {
        queryFilterData.flags |= PxQueryFlag::eNO_BLOCK;
        const PxU32 hitBufferSize = 256;
        PxOverlapHit hitBuffer[hitBufferSize];
        OverlapCallback cb(hitBuffer, hitBufferSize, reportFn, db);
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            const PxScene* scene = ref.second->getScene();
            if (!scene)
                continue;
            scene->overlap(geometry, transform, cb, queryFilterData);
        }
        numHits = cb.numHits;
    }

    return numHits;
}

uint32_t overlapSphere(float radius, const carb::Float3& pos, OverlapHitReportFn reportFn, bool anyHit)
{
    uint32_t result = 0;    
    geometrySceneQueryFn overlapFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        result = overlapInternal(geom, transform, reportFn, anyHit);
        return (result > 0);
    };

    if(!doSphereSceneQuery(radius, pos, overlapFn))
    {
        return 0;
    };

    return result;
}

bool overlapSphereAny(float radius, const carb::Float3& pos)
{
    return overlapSphere(radius, pos, nullptr, true);
}

uint32_t overlapBox(const carb::Float3& halfExtent, const carb::Float3& pos, const carb::Float4& rot, OverlapHitReportFn reportFn, bool anyHit)
{
    uint32_t result = 0;
    geometrySceneQueryFn overlapFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        result = overlapInternal(geom, transform, reportFn, anyHit);
        return (result > 0);
    };

    if(!doBoxSceneQuery(halfExtent, pos, rot, overlapFn))
    {
        return 0;
    };

    return result;
}

bool overlapBoxAny(const carb::Float3& halfExtent,
                    const carb::Float3& pos,
                    const carb::Float4& rot)
{
    return overlapBox(halfExtent, pos, rot, nullptr, true);
}

uint32_t overlapMesh(uint64_t meshPath, OverlapHitReportFn reportFn, bool anyHit)
{
    uint32_t result = 0;
    geometrySceneQueryFn overlapFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        result = overlapInternal(geom, transform, reportFn, anyHit);
        return (result > 0);
    };

    if(!doMeshSceneQuery(meshPath, overlapFn))
    {
        return 0;
    };

    return result;
}

bool overlapMeshAny(uint64_t meshPath)
{
    return overlapMesh(meshPath, nullptr, true);
}

uint32_t overlapShape(uint64_t rPrimPath, OverlapHitReportFn reportFn, bool anyHit)
{
    uint32_t result = 0;
    geometrySceneQueryFn overlapFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        result = overlapInternal(geom, transform, reportFn, anyHit);
        return (result > 0);
    };

    if(!doShapeSceneQuery(rPrimPath, overlapFn))
    {
        return 0;
    };

    return result;
}

bool overlapShapeAny(uint64_t rPrimPath)
{
    return overlapShape(rPrimPath, nullptr, true);
}

void reportCollisionShape(const SdfPath& path, const PxRigidActor& actor, const PxShape& shape,
    ICollisionShapeQueryCallback& reportCallback, const InternalPhysXDatabase& db)
{
    const PxTransform globalPose = actor.getGlobalPose() * shape.getLocalPose();
    const PxGeometry& geom = shape.getGeometry();
    switch (geom.getType())
    {
    case PxGeometryType::eSPHERE:
    {
        const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);
        if (reportCallback.sphereShapeReportFn)
        {
            reportCallback.sphereShapeReportFn(sdfPathToInt(path), fromPhysX(globalPose.p), fromPhysX(globalPose.q), sphereGeom.radius, reportCallback.userData);
        }
    }
    break;
    case PxGeometryType::eBOX:
    {
        const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);
        if (reportCallback.boxShapeReportFn)
        {
            reportCallback.boxShapeReportFn(sdfPathToInt(path), fromPhysX(globalPose.p), fromPhysX(globalPose.q), fromPhysX(boxGeom.halfExtents), reportCallback.userData);
        }
    }
    break;
    case PxGeometryType::eCAPSULE:
    {
        const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);
        CollisionShapeAxis::Enum axis;
        PxTransform updatedGlobalPose = globalPose;
        if ((size_t)shape.userData < db.getRecords().size())
        {
            const InternalDatabase::Record& record = db.getRecords()[(size_t)shape.userData];
            InternalShape* internalShape = (InternalShape *)record.mInternalPtr;
            axis = (CollisionShapeAxis::Enum)internalShape->mAxis;
            const PxQuat fixupQ = fixupCapsuleQuat((usdparser::Axis)internalShape->mAxis);
            PxTransform localPose = shape.getLocalPose();
            PxQuat q = localPose.q;
            localPose.q = q * fixupQ.getConjugate();
            updatedGlobalPose = actor.getGlobalPose() * localPose;
        }

        if (reportCallback.capsuleShapeReportFn)
        {
            reportCallback.capsuleShapeReportFn(sdfPathToInt(path), fromPhysX(updatedGlobalPose.p), fromPhysX(updatedGlobalPose.q), axis, capsuleGeom.radius, capsuleGeom.halfHeight * 2.0f, reportCallback.userData);
        }
    }
    break;
    case PxGeometryType::eTRIANGLEMESH:
    {
        const PxTriangleMeshGeometry& triMeshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
        if (reportCallback.triangleMeshShapeReportFn)
        {
            reportCallback.triangleMeshShapeReportFn(sdfPathToInt(path), fromPhysX(globalPose.p), fromPhysX(globalPose.q), fromPhysX(triMeshGeom.scale.scale),
                triMeshGeom.triangleMesh->getNbVertices(), (const carb::Float3*)triMeshGeom.triangleMesh->getVertices(),
                triMeshGeom.triangleMesh->getNbTriangles(), (const uint32_t*)triMeshGeom.triangleMesh->getTriangles(), reportCallback.userData);
        }
    }
    break;
    case PxGeometryType::eCONVEXMESH:
    {
        const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

        const PxConvexMeshGeometry& convexMeshGeom = static_cast<const PxConvexMeshGeometry&>(geom);
        if (convexMeshGeom.convexMesh == physxSetup.getCylinderConvexMesh(Axis::eX) ||
            convexMeshGeom.convexMesh == physxSetup.getCylinderConvexMesh(Axis::eY) ||
            convexMeshGeom.convexMesh == physxSetup.getCylinderConvexMesh(Axis::eZ))
        {
            CollisionShapeAxis::Enum axis;
            if ((size_t)shape.userData < db.getRecords().size())
            {
                const InternalDatabase::Record& record = db.getRecords()[(size_t)shape.userData];
                InternalShape* internalShape = (InternalShape*)record.mInternalPtr;
                axis = (CollisionShapeAxis::Enum)internalShape->mAxis;
            }
            const PxVec3& meshScale = convexMeshGeom.scale.scale;
            float radius;
            float halfHeight;
            getConeOrCylinderSize(meshScale, (usdparser::Axis)axis, halfHeight, radius);
            if (reportCallback.cylinderShapeReportFn)
            {
                reportCallback.cylinderShapeReportFn(sdfPathToInt(path), fromPhysX(globalPose.p), fromPhysX(globalPose.q), axis, radius, halfHeight * 2.0f, reportCallback.userData);
            }
        }
        else if (convexMeshGeom.convexMesh == physxSetup.getConeConvexMesh(Axis::eX) ||
            convexMeshGeom.convexMesh == physxSetup.getConeConvexMesh(Axis::eY) ||
            convexMeshGeom.convexMesh == physxSetup.getConeConvexMesh(Axis::eZ))
        {
            CollisionShapeAxis::Enum axis;
            if ((size_t)shape.userData < db.getRecords().size())
            {
                const InternalDatabase::Record& record = db.getRecords()[(size_t)shape.userData];
                InternalShape* internalShape = (InternalShape*)record.mInternalPtr;
                axis = (CollisionShapeAxis::Enum)internalShape->mAxis;
            }
            const PxVec3& meshScale = convexMeshGeom.scale.scale;
            float radius;
            float halfHeight;
            getConeOrCylinderSize(meshScale, (usdparser::Axis)axis, halfHeight, radius);
            if (reportCallback.coneShapeReportFn)
            {
                reportCallback.coneShapeReportFn(sdfPathToInt(path), fromPhysX(globalPose.p), fromPhysX(globalPose.q), axis, radius, halfHeight * 2.0f, reportCallback.userData);
            }
        }
        else
        {
            if (reportCallback.convexMeshShapeReportFn)
            {
                std::vector<ConvexMeshPolygon> polygons;
                polygons.resize(convexMeshGeom.convexMesh->getNbPolygons());
                for (uint32_t i = 0; i < convexMeshGeom.convexMesh->getNbPolygons(); i++)
                {
                    PxHullPolygon pxHull;
                    convexMeshGeom.convexMesh->getPolygonData(i, pxHull);
                    ConvexMeshPolygon& polygon = polygons[i];
                    polygon.indexBase = pxHull.mIndexBase;
                    polygon.numVerts = pxHull.mNbVerts;
                    for (int j = 0; j < 4; j++)
                    {
                        polygon.plane[j] = pxHull.mPlane[j];
                    }                    
                }                
                reportCallback.convexMeshShapeReportFn(sdfPathToInt(path), fromPhysX(globalPose.p), fromPhysX(globalPose.q), fromPhysX(convexMeshGeom.scale.scale),
                    convexMeshGeom.convexMesh->getNbVertices(), (const carb::Float3*)convexMeshGeom.convexMesh->getVertices(),
                    convexMeshGeom.convexMesh->getIndexBuffer(), convexMeshGeom.convexMesh->getNbPolygons(), polygons.data(), reportCallback.userData);
            }
        }
    }
    break;
    case PxGeometryType::eCONVEXCORE:
    {
        const PxConvexCoreGeometry& convexGeom = static_cast<const PxConvexCoreGeometry&>(geom);
        PxConvexCore::Type type = convexGeom.getCoreType();
        if (type == PxConvexCore::eCYLINDER && reportCallback.cylinderShapeReportFn)
        {
            const PxConvexCore::Cylinder& c = convexGeom.getCore<PxConvexCore::Cylinder>();
            CollisionShapeAxis::Enum axis;
            PxTransform updatedGlobalPose = globalPose;
            if ((size_t)shape.userData < db.getRecords().size())
            {
                const InternalDatabase::Record& record = db.getRecords()[(size_t)shape.userData];
                InternalShape* internalShape = (InternalShape*)record.mInternalPtr;
                axis = (CollisionShapeAxis::Enum)internalShape->mAxis;
                const PxQuat fixupQ = fixupConeAndCylinderQuat((usdparser::Axis)internalShape->mAxis);
                PxTransform localPose = shape.getLocalPose();
                localPose.q = localPose.q * fixupQ.getConjugate();
                updatedGlobalPose = actor.getGlobalPose() * localPose;
            }

            reportCallback.cylinderShapeReportFn(sdfPathToInt(path), fromPhysX(updatedGlobalPose.p),
                fromPhysX(updatedGlobalPose.q), axis, c.radius + convexGeom.getMargin(),
                c.height + convexGeom.getMargin() * 2.0f, reportCallback.userData);
        }
        else if (type == PxConvexCore::eCONE && reportCallback.coneShapeReportFn)
        {
            const PxConvexCore::Cone& c = convexGeom.getCore<PxConvexCore::Cone>();
            CollisionShapeAxis::Enum axis;
            PxTransform updatedGlobalPose = globalPose;
            if ((size_t)shape.userData < db.getRecords().size())
            {
                const InternalDatabase::Record& record = db.getRecords()[(size_t)shape.userData];
                InternalShape* internalShape = (InternalShape*)record.mInternalPtr;
                axis = (CollisionShapeAxis::Enum)internalShape->mAxis;
                const PxQuat fixupQ = fixupConeAndCylinderQuat((usdparser::Axis)internalShape->mAxis);
                PxTransform localPose = shape.getLocalPose();
                localPose.q = localPose.q * fixupQ.getConjugate();
                updatedGlobalPose = actor.getGlobalPose() * localPose;
            }

            reportCallback.coneShapeReportFn(sdfPathToInt(path), fromPhysX(updatedGlobalPose.p),
                fromPhysX(updatedGlobalPose.q), axis, c.radius + convexGeom.getMargin(),
                c.height + convexGeom.getMargin() * 2.0f, reportCallback.userData);
        }
    }
    break;
    default:
        break;
    }
}

uint32_t reportCollisionShapes(uint64_t path, ICollisionShapeQueryCallback& reportCallback)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    uint32_t numShapes = 0;
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    if (physxSetup.getPhysXScenes().empty() || !stage)
        return numShapes;

    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
        return numShapes;

    const SdfPath sdfPath = intToPath(path);
    const UsdPrim topPrim = stage->GetPrimAtPath(sdfPath);
    if (!topPrim)
        return numShapes;
    
    pxr::UsdPrimRange primRange = pxr::UsdPrimRange::AllPrims(topPrim);
    for (pxr::UsdPrimRange::const_iterator iter = primRange.begin(); iter != primRange.end(); ++iter)
    {
        const pxr::UsdPrim& prim = *iter;
        if (!prim)
            continue;

        const SdfPath primPath = prim.GetPrimPath();
        const usdparser::ObjectIdMap* entries = attachedStage->getObjectIds(primPath);
        if (entries && !entries->empty())
        {
            ObjectIdMap::const_iterator it = entries->begin();
            while (it != entries->end())
            {
                usdparser::ObjectId objectId = it->second;
                const PxShape* shape = reinterpret_cast<const PxShape*>(db.getTypedRecord(ePTShape, objectId));
                if (shape)
                {
                    numShapes++;
                    if (shape->getActor())
                    {
                        reportCollisionShape(primPath, *shape->getActor(), *shape, reportCallback, db);
                    }
                    else
                    {
                        // shared shapes, need to check parents for the actor
                        UsdPrim parent = prim;
                        bool actorFound = false;
                        while (!actorFound && parent && !parent.IsPseudoRoot())
                        {
                            const usdparser::ObjectIdMap* actorEntries = attachedStage->getObjectIds(parent.GetPrimPath());
                            if (actorEntries && !actorEntries->empty())
                            {
                                ObjectIdMap::const_iterator itActor = actorEntries->begin();                                
                                while (itActor != actorEntries->end())
                                {
                                    usdparser::ObjectId actorObjectId = itActor->second;
                                    const PxRigidActor* actor = reinterpret_cast<const PxRigidActor*>(db.getTypedRecord(ePTActor, actorObjectId));
                                    if (actor)
                                    {
                                        actorFound = true;
                                        reportCollisionShape(primPath, *actor, *shape, reportCallback, db);
                                    }
                                    itActor++;
                                }
                            }
                            parent = parent.GetParent();
                        }
                    }
                }
                else
                {                    
                    const PhysXCompoundShape* compoundShape = reinterpret_cast<const PhysXCompoundShape*>(db.getTypedRecord(ePTCompoundShape, objectId));
                    if (compoundShape)
                    {
                        for (size_t i = 0; i < compoundShape->getShapes().size(); i++)
                        {
                            numShapes++;
                            PxShape* shape = reinterpret_cast<PxShape*>(compoundShape->getShapes()[i]);
                            if (shape->getActor())
                            {                                
                                reportCollisionShape(primPath, *shape->getActor(), *shape, reportCallback, db);
                            }
                            else
                            {
                                // shared shapes, need to check parents for the actor
                                UsdPrim parent = prim;
                                bool actorFound = false;
                                while (!actorFound && parent && !parent.IsPseudoRoot())
                                {
                                    const usdparser::ObjectIdMap* actorEntries = attachedStage->getObjectIds(parent.GetPrimPath());
                                    if (actorEntries && !actorEntries->empty())
                                    {
                                        ObjectIdMap::const_iterator itActor = actorEntries->begin();                                        
                                        while (itActor != actorEntries->end())
                                        {
                                            usdparser::ObjectId actorObjectId = itActor->second;
                                            const PxRigidActor* actor = reinterpret_cast<const PxRigidActor*>(db.getTypedRecord(ePTActor, actorObjectId));
                                            if (actor)
                                            {
                                                actorFound = true;
                                                reportCollisionShape(primPath, *actor, *shape, reportCallback, db);
                                            }
                                            itActor++;
                                        }
                                    }
                                    parent = parent.GetParent();
                                }
                            }                            
                        }
                    }
                }
                it++;
            }
        }


    }
    return numShapes;
}

}
}


void fillInterface(omni::physx::IPhysxSceneQuery& iface)
{
    iface.raycastClosest = raycastClosest;
    iface.sweepSphereClosest = sweepSphereClosest;
    iface.sweepBoxClosest = sweepBoxClosest;
    iface.sweepMeshClosest = sweepMeshClosest;
    iface.sweepShapeClosest = sweepShapeClosest;
    iface.raycastAll = raycastAll;
    iface.sweepSphereAll = sweepSphereAll;
    iface.sweepBoxAll = sweepBoxAll;
    iface.sweepMeshAll = sweepMeshAll;
    iface.sweepShapeAll = sweepShapeAll;
    iface.raycastAny = raycastAny;
    iface.sweepSphereAny = sweepSphereAny;
    iface.sweepBoxAny = sweepBoxAny;
    iface.sweepMeshAny = sweepMeshAny;
    iface.sweepShapeAny = sweepShapeAny;
    iface.overlapSphere = overlapSphere;
    iface.overlapBox = overlapBox;
    iface.overlapMesh = overlapMesh;
    iface.overlapBoxAny = overlapBoxAny;
    iface.overlapSphereAny = overlapSphereAny;
    iface.overlapMeshAny = overlapMeshAny;
    iface.reportCollisionShapes = reportCollisionShapes;
    iface.overlapShape = overlapShape;
    iface.overlapShapeAny = overlapShapeAny;
}
