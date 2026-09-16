// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1
 *
 * @implements REQ-SIM-SCENEQUERY-001
 * @covers AC-3
 *
 * @implements REQ-MATH-001
 * @covers AC-13
 */

#include <omni/physics/parse/KnownTokens.h>

#include "PhysXSceneQuery.h"
#include "internal/InternalScene.h"
#include "PhysXTools.h"
#include "PhysXScene.h"
#include "ObjectDataQuery.h"
#include "ConeCylinderConvexMesh.h"

#include "usdLoad/Collision.h"
#include "CookingDataAsync.h"
#include <omni/physx/IPhysx.h>
#include <omni/physx/PhysxTokens.h>

extern void* getPhysXPtr(omni::physics::parse::ObjectKey key, omni::physx::PhysXType type);

using namespace ::physx;
using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace cookingdataasync;
using namespace carb;

namespace omni
{
namespace physx
{

// `tok` must already be interned by the caller (AttachedStage::getKnownTokens(),
// cached once per attach) -- these helpers run once per doShapeSceneQuery() call,
// so re-interning the ~659-field vocabulary here would double the cost of every
// single query (REQ-SIM-SCENEQUERY-001).
static PxBoxGeometry usdToPxBoxGeometry(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, const PxVec3& scale, const omni::physics::parse::KnownTokens& tok)
{
    double sizeAttr = 0.0;
    if (attachedStage.getSource())
    {
        getValue(attachedStage, key, tok.size, omni::physics::parse::ReadTime::defaultTime(), sizeAttr);
    }
    sizeAttr = abs(sizeAttr) * 0.5f; // convert cube edge length to half extend
    return PxBoxGeometry(scale * float(sizeAttr));
}

static PxSphereGeometry usdToPxSphereGeometry(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, const PxVec3& scale, const omni::physics::parse::KnownTokens& tok)
{
    // Scale is unsupported/non-uniform-unsafe here; use the largest component as the radius base.
    const float tolerance = 1e-4f;
    if (PxAbs(scale[0] - scale[1]) > tolerance || PxAbs(scale[0] - scale[2]) > tolerance || PxAbs(scale[2] - scale[1]) > tolerance)
    {
        CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", attachedStage.textFor(key));
    }

    double radius = std::max(std::max(PxAbs(scale[1]), PxAbs(scale[0])), PxAbs(scale[2]));
    double radiusAttr = 0.0;
    if (attachedStage.getSource())
    {
        getValue(attachedStage, key, tok.radius, omni::physics::parse::ReadTime::defaultTime(), radiusAttr);
    }
    radius *= radiusAttr;
    return PxSphereGeometry((float)radius);
}

static PxCapsuleGeometry usdToPxCapsuleGeometry(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, const PxVec3& scale, const omni::physics::parse::KnownTokens& tok)
{
    double radius = 0.0;
    getValue(attachedStage, key, tok.radius, omni::physics::parse::ReadTime::defaultTime(), radius);
    double height = 0.0;
    getValue(attachedStage, key, tok.height, omni::physics::parse::ReadTime::defaultTime(), height);

    // capAxis read via the TokenId+ReadTime getValue overload (PhysXTools.h):
    // it resolves an int-encoded token column (ovstage) correctly for
    // T=TokenId, so no TfToken/UsdTimeCode is needed here.
    omni::physics::parse::TokenId capAxis{};
    getValue(attachedStage, key, tok.axis, omni::physics::parse::ReadTime::defaultTime(), capAxis);
    const float tolerance = 1e-4f;
    // Scale is unsupported/non-uniform-unsafe here; use the largest component as the radius base.
    if (capAxis == tok.x)
    {
        height *= scale[0];
        radius *= std::max(PxAbs(scale[1]), PxAbs(scale[2]));
        if (PxAbs(scale[2] - scale[1]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", attachedStage.textFor(key));
        }
    }
    else if (capAxis == tok.y)
    {
        height *= scale[1];
        radius *= std::max(PxAbs(scale[0]), PxAbs(scale[2]));
        if (PxAbs(scale[2] - scale[0]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", attachedStage.textFor(key));
        }    
    }
    else
    {
        height *= scale[2];
        radius *= std::max(PxAbs(scale[1]), PxAbs(scale[0]));
        if (PxAbs(scale[1] - scale[0]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", attachedStage.textFor(key));
        }
    }
    return PxCapsuleGeometry((float)radius, (float)(height * 0.5));
}


static PxConvexMeshGeometry usdCylinderOrConeToPxConvexMeshGeometry(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, bool isCylinder, const PxVec3& scale, const omni::physics::parse::KnownTokens& tok)
{
    // Cylinder and Cone share the same radius/height/axis attribute names
    // (UsdGeomCylinder and UsdGeomCone both derive from UsdGeomGprim), so the
    // source-routed read is identical for both types.
    double radius = 0.0;
    double height = 0.0;
    omni::physics::parse::TokenId capAxis{};
    getValue(attachedStage, key, tok.radius, omni::physics::parse::ReadTime::defaultTime(), radius);
    getValue(attachedStage, key, tok.height, omni::physics::parse::ReadTime::defaultTime(), height);
    // capAxis read via the TokenId+ReadTime getValue overload: see the comment
    // in usdToPxCapsuleGeometry above.
    getValue(attachedStage, key, tok.axis, omni::physics::parse::ReadTime::defaultTime(), capAxis);

    usdparser::Axis axis = eZ;
    const float tolerance = 1e-4f;
    // Scale is unsupported/non-uniform-unsafe here; use the largest component as the radius base.
    if (capAxis == tok.x)
    {
        height *= scale[0];
        radius *= std::max(PxAbs(scale[1]), PxAbs(scale[2]));
        if (PxAbs(scale[2] - scale[1]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", attachedStage.textFor(key));
        }
        axis = eX;
    }
    else if (capAxis == tok.y)
    {
        height *= scale[1];
        radius *= std::max(PxAbs(scale[0]), PxAbs(scale[2]));
        if (PxAbs(scale[2] - scale[0]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", attachedStage.textFor(key));
        }
        axis = eY;
    }
    else
    {
        height *= scale[2];
        radius *= std::max(PxAbs(scale[1]), PxAbs(scale[0]));
        if (PxAbs(scale[1] - scale[0]) > tolerance)
        {
            CARB_LOG_WARN("Non-uniform scale may result in a non matching collision representation on prim: %s", attachedStage.textFor(key));
        }
    }

    const PxVec3 mesh_scale = getConeOrCylinderScale((float) height * 0.5, radius, axis);
    PxMeshScale meshScale = PxMeshScale(mesh_scale);
    PxConvexMeshGeometryFlags flags = PxConvexMeshGeometryFlag::eTIGHT_BOUNDS;
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    if (isCylinder)
    {
        return PxConvexMeshGeometry(physxSetup.getCylinderConvexMesh(axis), meshScale);
    }
    else
    {
        return PxConvexMeshGeometry(physxSetup.getConeConvexMesh(axis), meshScale);
    }
}

static PxConvexMeshGeometry usdToPxConvexMeshGeometry(omni::physics::parse::ObjectKey key, const PxVec3& scale)
{
    usdparser::ConvexMeshPhysxShapeDesc convexMeshDesc;
    usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    const char* primText = attachedStage ? attachedStage->textFor(key) : "";
    if (fillConvexMeshDesc(attachedStage, key, convexMeshDesc, convexMeshDesc.convexCookingParams))
    {
        const Float3 mesh_scale = toFloat3(scale);
        convexMeshDesc.meshScale = mesh_scale;
        convexMeshDesc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(mesh_scale);
        PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
        CookingDataAsync* cookingDataAsync = physxSetup.getCookingDataAsync();

        if (!cookingDataAsync || !attachedStage)
        {
            CARB_LOG_ERROR("omni::physx::usdToPxConvexMeshGeometry: Failed to cook convex mesh data for prim: %s - cooking service unavailable", primText);
            return PxConvexMeshGeometry();
        }

        PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(convexMeshDesc, key, *attachedStage, false);

        if (convexMesh)
        {
            const PxMeshScale meshScale(scale.abs());
            return PxConvexMeshGeometry(convexMesh, meshScale);
        }
        else
        {
            CARB_LOG_ERROR("omni::physx::usdToPxConvexMeshGeometry: Failed to cook convex mesh data for prim: %s", primText);
            return PxConvexMeshGeometry();
        }
    }
    else
    {
        CARB_LOG_ERROR("omni::physx::usdToPxConvexMeshGeometry: Provided mesh is not a valid mesh: %s", primText);
        return PxConvexMeshGeometry();
    }
}


// Decomposes the prim's world matrix at the USD boundary and hands back PhysX
// math only: a rigid PxTransform plus the (possibly non-uniform) scale.
static void computeWorldTransform(const AttachedStage& attachedStage,
                                  omni::physics::parse::ObjectKey key,
                                  PxTransform& transform,
                                  PxVec3& scale)
{
    // NOTE: `scale` stays signed here (a mirrored prim reports negative
    // components), matching the previous GfTransform::GetScale() behaviour.
    // usdToPxBoxGeometry() relies on that; the other usdToPx*Geometry helpers
    // take PxAbs themselves.
    //
    // `m` is an arbitrary gprim's local-to-world matrix, so it can carry shear
    // (a non-uniform ancestor scale above a rotation). decomposeMatrix's polar
    // factor disagrees with GfTransform on sheared input (see MatrixTools.h),
    // and the actual collider for the same prim is still built off
    // GfTransform via UsdSource::getLocalToWorldRotationAndScale, so this must
    // stay on gfmath::decomposeWithPivot to match it -- same convention as
    // CollisionShapeTransform.h::decomposeCollisionShapeLocalTransform.
    const PxMat44d m = getWorldTransform(attachedStage, key, omni::physics::parse::ReadTime::defaultTime());
    const gfmath::PivotTransform pt = gfmath::decomposeWithPivot(m);
    const PxQuatd q = gfmath::getQuat(pt.rotation);
    transform = PxTransform(PxVec3(float(pt.translation.x), float(pt.translation.y), float(pt.translation.z)),
                            PxQuat(float(q.x), float(q.y), float(q.z), float(q.w)));
    scale = PxVec3(float(pt.scale.x), float(pt.scale.y), float(pt.scale.z));
}

using geometrySceneQueryFn = std::function<bool(const PxGeometry& geom, const PxTransform& transform)>;

static bool doShapeSceneQuery(omni::physics::parse::ObjectKey key, geometrySceneQueryFn queryFn)
{
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    if (physxSetup.getPhysXScenes().empty())
    {
        CARB_LOG_WARN_ONCE("Physx scene queries can only be performed during simulation.");
        return false;
    }

    const AttachedStage* attachedStagePtr = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!attachedStagePtr)
        return false;
    const AttachedStage& attachedStage = *attachedStagePtr;
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(key))
    {
        CARB_LOG_ERROR("omni::physx::doShapeSceneQuery: Provided prim is invalid: %s", attachedStage.textFor(key));
        return false;
    }
    // Interned once per attach and cached on AttachedStage (REQ-SIM-SCENEQUERY-001)
    // -- not re-interned here, and threaded by const reference into every helper
    // below instead of each one interning its own copy.
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    const bool isMeshMerge = src->hasSchema(key, tok.physxMeshMergeCollisionAPI);
    if (!src->isA(key, tok.gprimType) && !isMeshMerge)
    {
        CARB_LOG_ERROR("omni::physx::doShapeSceneQuery: Provided prim is not a UsdGeomGPrim: %s", attachedStage.textFor(key));
        return false;
    }

    PxTransform transform;
    PxVec3 sc;
    computeWorldTransform(attachedStage, key, transform, sc);

    const bool isCylinder = src->isA(key, tok.cylinderType);
    const bool isCone = src->isA(key, tok.coneType);
    if (src->isA(key, tok.cubeType))
    {
        return queryFn(usdToPxBoxGeometry(attachedStage, key, sc, tok), transform);
    }
    else if (src->isA(key, tok.sphereType))
    {
        return queryFn(usdToPxSphereGeometry(attachedStage, key, sc, tok), transform);
    }
    else if (src->isA(key, tok.capsuleType))
    {
        // capAxis read via the TokenId+ReadTime getValue overload, then bridged to the
        // pxr-free usdparser::Axis enum so fixupCapsuleQuat's Axis overload (already used
        // elsewhere in this file) can be used instead of its TfToken overload.
        omni::physics::parse::TokenId capAxis{};
        getValue(attachedStage, key, tok.axis, omni::physics::parse::ReadTime::defaultTime(), capAxis);
        usdparser::Axis axisEnum = usdparser::eX;
        if (capAxis == tok.z)
            axisEnum = usdparser::eZ;
        else if (capAxis == tok.y)
            axisEnum = usdparser::eY;
        const PxQuat fixupQ = fixupCapsuleQuat(axisEnum);
        transform.q = transform.q * fixupQ;
        return queryFn(usdToPxCapsuleGeometry(attachedStage, key, sc, tok), transform);
    }
    PxConvexMeshGeometry geom;
    if (isCylinder || isCone)
    {
        geom = usdCylinderOrConeToPxConvexMeshGeometry(attachedStage, key, isCylinder, sc, tok);
    }
    else if (src->isA(key, tok.meshType))
    {
        // Convex cooking is keyed; the residual cooking-input USD read is
        // encapsulated in fillConvexMeshDesc (tracked separately).
        geom = usdToPxConvexMeshGeometry(key, sc);
    }
    else if (isMeshMerge)
    {
        // for meshmerge the convex must already exist, lets find it
        PxShape* shapePrim = (PxShape*)(getPhysXPtr(key, ePTShape));
        if (!shapePrim || shapePrim->getGeometry().getType() != PxGeometryType::eCONVEXMESH)
        {
            CARB_LOG_WARN(
                "Mesh merge collision overlap works only with convexHull approximation, please switch the approximation for the prim: %s",
                attachedStage.textFor(key));
            return false;
        }

        geom = static_cast<const PxConvexMeshGeometry&>(shapePrim->getGeometry());
    }
    if(geom.isValid())
    {
        return queryFn(geom, transform);
    }
    CARB_LOG_ERROR("omni::physx::doShapeSceneQuery: Provided prim is not a valid UsdGeom type (supported types are: box, sphere, capsule, convex): %s", attachedStage.textFor(key));
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

static bool doMeshSceneQuery(omni::physics::parse::ObjectKey key, geometrySceneQueryFn queryFn)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    if (physxSetup.getPhysXScenes().empty())
    {
        CARB_LOG_WARN_ONCE("Physx scene queries can only be performed during simulation.");
        return false;
    }

    CookingDataAsync* cookingDataAsync = physxSetup.getCookingDataAsync();
    if (cookingDataAsync== nullptr)
    {
        return false;
    }

    usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!attachedStage)
    {
        return false;
    }
    const omni::physics::parse::IPhysicsSource* src = attachedStage->getSource();
    if (!src || !src->exists(key))
    {
        return false;
    }
    // Cached on AttachedStage, interned once per attach -- see doShapeSceneQuery's
    // comment above (REQ-SIM-SCENEQUERY-001); avoids rebuilding the whole batch on
    // every mesh-query call.
    const omni::physics::parse::KnownTokens& tok = attachedStage->getKnownTokens();
    bool meshMergeUsed = false;
    if (!src->isA(key, tok.meshType))
    {
        if (!src->hasSchema(key, tok.physxMeshMergeCollisionAPI))
        {
            return false;
        }
        else
        {
            meshMergeUsed = true;
        }
    }

    PxTransform transform;
    PxVec3 sc;
    computeWorldTransform(*attachedStage, key, transform, sc);
    const Float3 scale = toFloat3(sc);

    PxConvexMesh* convexMesh = nullptr;
    if (meshMergeUsed)
    {
        // for meshmerge the convex must already exist, lets find it
        PxShape* shapePrim = (PxShape*)(getPhysXPtr(key, ePTShape));
        if (!shapePrim || shapePrim->getGeometry().getType() != PxGeometryType::eCONVEXMESH)
        {
            CARB_LOG_WARN(
                "Mesh merge collision overlap works only with convexHull approximation, please switch the approximation for the prim: %s",
                attachedStage->textFor(key));
            return false;
        }

        const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(shapePrim->getGeometry());
        convexMesh = convexGeom.convexMesh;
    }
    else
    {
        usdparser::ConvexMeshPhysxShapeDesc convexMeshDesc;

        if (!fillConvexMeshDesc(attachedStage, key, convexMeshDesc, convexMeshDesc.convexCookingParams))
        {
            return false;
        }

        convexMeshDesc.meshScale = scale;
        convexMeshDesc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(scale);

        convexMesh = cookingDataAsync->getConvexMesh(convexMeshDesc, key, *attachedStage, false);
    }


    if(!convexMesh)
    {
        return false;
    }

    const PxMeshScale meshScale(sc.abs());

    return queryFn(PxConvexMeshGeometry(convexMesh, meshScale), transform);
}

bool raycastClosest(const carb::Float3& origin, const carb::Float3& unitDir, float distance, RaycastHit& outHit, bool bothSides)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (physxSetup.getPhysXScenes().empty())
    {
        CARB_LOG_WARN_ONCE("Physx scene queries can only be performed during simulation.");
        return false;
    }

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
        outHit.collision = shapeIndex < db.getRecords().size() ? db.getRecords()[shapeIndex].mKey : omni::physics::parse::ObjectKey{};
        if (bodyIndex < db.getRecords().size())
        {
            const InternalDatabase::Record& record = db.getRecords()[bodyIndex];
            outHit.rigidBody = record.mKey;
            if (record.mType == ePTActor)
            {
                outHit.protoIndex = ((InternalActor*)record.mInternalPtr)->mInstanceIndex;
            }                
        }
        else
        {
            outHit.rigidBody = omni::physics::parse::ObjectKey{};
            outHit.protoIndex = 0xFFFFFFFF;
        }
        PxBaseMaterial* baseMaterial = hit.shape->getMaterialFromInternalFaceIndex(hit.faceIndex);
        PX_ASSERT(!baseMaterial || baseMaterial->getConcreteType() == PxConcreteType::eMATERIAL);
        const PxMaterial* material = (baseMaterial != nullptr) ? baseMaterial->is<PxMaterial>() : nullptr;
        if (material && material->userData)
        {
            const size_t materialIndex = (size_t)material->userData;
            outHit.material = materialIndex < db.getRecords().size() ? db.getRecords()[materialIndex].mKey : omni::physics::parse::ObjectKey{};
        }
        else
        {
            outHit.material = omni::physics::parse::ObjectKey{};
        }
    }
    return ret;
}

bool raycastAny(const carb::Float3& origin, const carb::Float3& unitDir, float distance, bool bothSides)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (physxSetup.getPhysXScenes().empty())
    {
        CARB_LOG_WARN_ONCE("Physx scene queries can only be performed during simulation.");
        return false;
    }

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
    return false;
}

template <typename T, typename QueryHit, typename PxQueryHit>
bool reportLocationHit(const PxQueryHit& hit, T reportFn, const InternalPhysXDatabase& db)
{
    QueryHit queryHit;
    const ObjectId shapeIndex = (ObjectId)hit.shape->userData;
    const ObjectId bodyIndex = (ObjectId)hit.actor->userData;
    queryHit.collision = shapeIndex < db.getRecords().size() ? db.getRecords()[shapeIndex].mKey : omni::physics::parse::ObjectKey{};
    if (bodyIndex < db.getRecords().size())
    {
        const InternalDatabase::Record& record = db.getRecords()[bodyIndex];
        queryHit.rigidBody = record.mKey;
        if (record.mType == ePTActor)
        {
            queryHit.protoIndex = ((InternalActor*)record.mInternalPtr)->mInstanceIndex;
        }
    }
    else
    {
        queryHit.rigidBody = omni::physics::parse::ObjectKey{};
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
        queryHit.material = materialIndex < db.getRecords().size() ? db.getRecords()[materialIndex].mKey : omni::physics::parse::ObjectKey{};
    }
    else
    {
        queryHit.material = omni::physics::parse::ObjectKey{};
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

    if (physxSetup.getPhysXScenes().empty())
    {
        CARB_LOG_WARN_ONCE("Physx scene queries can only be performed during simulation.");
        return false;
    }

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

struct SceneQueryFilterCallbackIgnorePrim : PxQueryFilterCallback
{
    SceneQueryFilterCallbackIgnorePrim() {};

    SceneQueryFilterCallbackIgnorePrim(omni::physics::parse::ObjectKey primKey, bool bMultiple=false) : bMultiple(bMultiple)
    {
        shapePrim = (PxShape *)(getPhysXPtr(primKey, ePTShape));
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

static bool sweepInternalClosest(const PxGeometry& geom, const PxTransform& transform, const carb::Float3& unitDir, float distance, SweepHit& outHit, bool bothSides, omni::physics::parse::ObjectKey primSelf = {})
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (physxSetup.getPhysXScenes().empty())
    {
        CARB_LOG_WARN_ONCE("Physx scene queries can only be performed during simulation.");
        return false;
    }

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
    if (primSelf.valid())
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
        const bool localRet = PxSceneQueryExt::sweepSingle(*scene, geom, transform, dir, distance, hitFlags, localHit, filterData, (primSelf.valid() ? &filter : nullptr));
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
        outHit.collision = shapeIndex < db.getRecords().size() ? db.getRecords()[shapeIndex].mKey : omni::physics::parse::ObjectKey{};
        if (bodyIndex < db.getRecords().size())
        {
            const InternalDatabase::Record& record = db.getRecords()[bodyIndex];
            outHit.rigidBody = record.mKey;
            if (record.mType == ePTActor)
            {
                outHit.protoIndex = ((InternalActor*)record.mInternalPtr)->mInstanceIndex;
            }
        }
        else
        {
            outHit.rigidBody = omni::physics::parse::ObjectKey{};
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
            outHit.material = materialIndex < db.getRecords().size() ? db.getRecords()[materialIndex].mKey : omni::physics::parse::ObjectKey{};
        }
        else
        {
            outHit.material = omni::physics::parse::ObjectKey{};
        }

    }
    return ret;
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

bool sweepMeshClosest(omni::physics::parse::ObjectKey meshPrimKey, const carb::Float3& unitDir, float distance, SweepHit& outHit, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepInternalClosest(geom, transform, unitDir, distance, outHit, bothSides, meshPrimKey);
    };

    return doMeshSceneQuery(meshPrimKey, sweepFn);
}

bool sweepShapeClosest(omni::physics::parse::ObjectKey rPrimPath, const carb::Float3& unitDir, float distance, SweepHit& outHit, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepInternalClosest(geom, transform, unitDir, distance, outHit, bothSides, rPrimPath);
    };

    return doShapeSceneQuery(rPrimPath, sweepFn);
}

static bool sweepAnyInternal(const PxGeometry& geom, const PxTransform& transform,
                        const carb::Float3& unitDir,
                        float distance,
                        bool bothSides, omni::physics::parse::ObjectKey primSelf = {})
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (physxSetup.getPhysXScenes().empty())
    {
        CARB_LOG_WARN_ONCE("Physx scene queries can only be performed during simulation.");
        return false;
    }

    PxVec3 dir(unitDir.x, unitDir.y, unitDir.z);
    dir.normalize();
    
    PxHitFlags hitFlags = PxHitFlag::eDEFAULT | PxHitFlag::eANY_HIT;
    if (bothSides)
        hitFlags |= PxHitFlag::eMESH_BOTH_SIDES;

    PxSceneQueryFilterData fdAny;
    fdAny.flags |= PxQueryFlag::eANY_HIT;
    SceneQueryFilterCallbackIgnorePrim filter;
    if (primSelf.valid())
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
        scene->sweep(geom, transform, dir, distance, buf, hitFlags, fdAny, (primSelf.valid() ? &filter : nullptr));
        if (buf.hasBlock)
        {
            return true;
        }
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

bool sweepMeshAny(omni::physics::parse::ObjectKey meshPrimKey, const carb::Float3& unitDir, float distance, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepAnyInternal(geom, transform, unitDir, distance, bothSides, meshPrimKey);
    };

    return doMeshSceneQuery(meshPrimKey, sweepFn);
}

bool sweepShapeAny(omni::physics::parse::ObjectKey rPrimPath, const carb::Float3& unitDir, float distance, bool bothSides)
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


bool sweepAllInternal(const PxGeometry& geom, const PxTransform& transform, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides, omni::physics::parse::ObjectKey primSelf = {})
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (physxSetup.getPhysXScenes().empty())
    {
        CARB_LOG_WARN_ONCE("Physx scene queries can only be performed during simulation.");
        return false;
    }

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
    if (primSelf.valid())
    {
        filter = SceneQueryFilterCallbackIgnorePrim(primSelf, true);
        filterData.flags |= PxQueryFlag::ePREFILTER;
    }
    for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
    {
        const PxScene* scene = ref.second->getScene();
        if (!scene)
            continue;
        scene->sweep(geom, transform, dir, distance, cb, hitFlags, filterData, (primSelf.valid() ? &filter : nullptr));
    }
    return cb.numHits ? true : false;
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

bool sweepMeshAll(omni::physics::parse::ObjectKey meshPrimKey, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides)
{
    geometrySceneQueryFn sweepFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        return sweepAllInternal(geom, transform, unitDir, distance, reportFn, bothSides, meshPrimKey);
    };

    return doMeshSceneQuery(meshPrimKey, sweepFn);
}

bool sweepShapeAll(omni::physics::parse::ObjectKey rPrimPath, const carb::Float3& unitDir, float distance, SweepHitReportFn reportFn, bool bothSides)
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
    queryHit.collision = shapeIndex < size ? db.getRecords()[shapeIndex].mKey : omni::physics::parse::ObjectKey{};
    if (bodyIndex < db.getRecords().size())
    {
        const InternalDatabase::Record& record = db.getRecords()[bodyIndex];
        queryHit.rigidBody = record.mKey;
        if (record.mType == ePTActor)
        {
            queryHit.protoIndex = ((InternalActor*)record.mInternalPtr)->mInstanceIndex;
        }
    }
    else
    {
        queryHit.rigidBody = omni::physics::parse::ObjectKey{};
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
        CARB_LOG_WARN_ONCE("Physx scene queries can only be performed during simulation.");
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

uint32_t overlapMesh(omni::physics::parse::ObjectKey meshPrimKey, OverlapHitReportFn reportFn, bool anyHit)
{
    uint32_t result = 0;
    geometrySceneQueryFn overlapFn = [&](const PxGeometry &geom, const PxTransform &transform) {
        result = overlapInternal(geom, transform, reportFn, anyHit);
        return (result > 0);
    };

    if(!doMeshSceneQuery(meshPrimKey, overlapFn))
    {
        return 0;
    };

    return result;
}

bool overlapMeshAny(omni::physics::parse::ObjectKey meshPrimKey)
{
    return overlapMesh(meshPrimKey, nullptr, true);
}

uint32_t overlapShape(omni::physics::parse::ObjectKey rPrimPath, OverlapHitReportFn reportFn, bool anyHit)
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

bool overlapShapeAny(omni::physics::parse::ObjectKey rPrimPath)
{
    return overlapShape(rPrimPath, nullptr, true);
}

void reportCollisionShape(omni::physics::parse::ObjectKey key, const PxRigidActor& actor, const PxShape& shape,
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
            reportCallback.sphereShapeReportFn(key, fromPhysX(globalPose.p), fromPhysX(globalPose.q), sphereGeom.radius, reportCallback.userData);
        }
    }
    break;
    case PxGeometryType::eBOX:
    {
        const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);
        if (reportCallback.boxShapeReportFn)
        {
            reportCallback.boxShapeReportFn(key, fromPhysX(globalPose.p), fromPhysX(globalPose.q), fromPhysX(boxGeom.halfExtents), reportCallback.userData);
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
            reportCallback.capsuleShapeReportFn(key, fromPhysX(updatedGlobalPose.p), fromPhysX(updatedGlobalPose.q), axis, capsuleGeom.radius, capsuleGeom.halfHeight * 2.0f, reportCallback.userData);
        }
    }
    break;
    case PxGeometryType::eTRIANGLEMESH:
    {
        const PxTriangleMeshGeometry& triMeshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
        if (reportCallback.triangleMeshShapeReportFn)
        {
            reportCallback.triangleMeshShapeReportFn(key, fromPhysX(globalPose.p), fromPhysX(globalPose.q), fromPhysX(triMeshGeom.scale.scale),
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
                reportCallback.cylinderShapeReportFn(key, fromPhysX(globalPose.p), fromPhysX(globalPose.q), axis, radius, halfHeight * 2.0f, reportCallback.userData);
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
                reportCallback.coneShapeReportFn(key, fromPhysX(globalPose.p), fromPhysX(globalPose.q), axis, radius, halfHeight * 2.0f, reportCallback.userData);
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
                reportCallback.convexMeshShapeReportFn(key, fromPhysX(globalPose.p), fromPhysX(globalPose.q), fromPhysX(convexMeshGeom.scale.scale),
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

            reportCallback.cylinderShapeReportFn(key, fromPhysX(updatedGlobalPose.p),
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

            reportCallback.coneShapeReportFn(key, fromPhysX(updatedGlobalPose.p),
                fromPhysX(updatedGlobalPose.q), axis, c.radius + convexGeom.getMargin(),
                c.height + convexGeom.getMargin() * 2.0f, reportCallback.userData);
        }
    }
    break;
    default:
        break;
    }
}

// Shared shapes have no actor of their own; walk `shapeKey` and its ancestors
// (root-exclusive, via the source) for the owning rigid actor and report the
// shape against it. Replicates the legacy prim.GetParent() walk: reports the
// shape against every actor found at the first ancestor that has one, then
// stops ascending. Returns whether any actor was found.
static bool reportSharedShapeActors(const AttachedStage& attachedStage,
                                    const omni::physics::parse::IPhysicsSource& source,
                                    omni::physics::parse::ObjectKey shapeKey,
                                    const PxShape& shape,
                                    ICollisionShapeQueryCallback& reportCallback,
                                    const InternalPhysXDatabase& db)
{
    bool actorFound = false;
    for (omni::physics::parse::ObjectKey parentKey = shapeKey; !actorFound && parentKey.valid();
         parentKey = source.getParent(parentKey))
    {
        // ObjectKey-native root check (ADR-0019): getParent() returns getRootKey() for a
        // top-level object, so the walk needs an explicit stop there -- matches the legacy
        // SdfPath::IsAbsoluteRootPath() check this replaces (see IPhysicsSource::getParent's
        // doc: "the invalid sentinel for the root itself").
        if (parentKey == source.getRootKey())
            break;
        const usdparser::ObjectIdMap* actorEntries = attachedStage.getObjectIds(parentKey);
        if (actorEntries && !actorEntries->empty())
        {
            for (auto itActor = actorEntries->begin(); itActor != actorEntries->end(); ++itActor)
            {
                const PxRigidActor* actor =
                    reinterpret_cast<const PxRigidActor*>(db.getTypedRecord(ePTActor, itActor->second));
                if (actor)
                {
                    actorFound = true;
                    reportCollisionShape(shapeKey, *actor, shape, reportCallback, db);
                }
            }
        }
    }
    return actorFound;
}

uint32_t reportCollisionShapes(omni::physics::parse::ObjectKey rootKey, ICollisionShapeQueryCallback& reportCallback)
{
    const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    uint32_t numShapes = 0;
    if (physxSetup.getPhysXScenes().empty())
        return numShapes;

    AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!attachedStage)
        return numShapes;

    const omni::physics::parse::IPhysicsSource* source = attachedStage->getSource();
    if (!source)
        return numShapes;

    if (!rootKey.valid())
        return numShapes;
    // Do not gate on source->exists(rootKey): the query root is frequently a
    // hierarchy-only prim (e.g. an Xform/scope like "/World") that carries no
    // physics data and is therefore absent from a data-plane source (ovstage),
    // yet still anchors a valid subtree. forEachDescendant resolves children via
    // the hierarchy (incl. the backing-USD fallback) and yields nothing for a
    // genuinely empty/bogus root, so the descendant loop is self-limiting.

    // Collect every object in the subtree (root inclusive) up front so the loop
    // below keeps a simple control flow.
    std::vector<omni::physics::parse::ObjectKey> descendants;
    source->forEachDescendant(rootKey,
                              [&descendants](omni::physics::parse::ObjectKey k) { descendants.push_back(k); });

    for (const omni::physics::parse::ObjectKey primObjKey : descendants)
    {
        // ObjectKey-native resolvability check (ADR-0019), replacing the legacy
        // pathFor(primObjKey).IsEmpty() gate: forEachDescendant only yields keys the source
        // itself enumerated, but guard against an unresolvable one the same way the SdfPath
        // form did.
        if (!primObjKey.valid())
            continue;

        const usdparser::ObjectIdMap* entries = attachedStage->getObjectIds(primObjKey);
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
                        reportCollisionShape(primObjKey, *shape->getActor(), *shape, reportCallback, db);
                    }
                    else
                    {
                        // shared shape: find the owning actor up the hierarchy.
                        reportSharedShapeActors(*attachedStage, *source, primObjKey, *shape, reportCallback, db);
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
                                reportCollisionShape(primObjKey, *shape->getActor(), *shape, reportCallback, db);
                            }
                            else
                            {
                                // shared shape: find the owning actor up the hierarchy.
                                reportSharedShapeActors(*attachedStage, *source, primObjKey, *shape, reportCallback, db);
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
