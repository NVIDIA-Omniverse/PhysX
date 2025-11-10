// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "Raycast.h"
#include <omni/physx/IPhysxSettings.h>
#include <PhysXTools.h>

#include <common/utilities/MemoryMacros.h>

#if USE_PHYSX_GPU
#include "extensions/PxParticleExt.h"
#endif

using namespace physx;
using namespace carb;
using namespace omni::physx;

const float kPickDistance=1000000.0f; // distance to raycast to do object 'picking'

const float kMouseGrabForceMultiplier = 50.0f;
const float kMouseGrabForceAngularDamping = 0.025f;
const float kMouseGrabForcePointDamping = 0.02f;
const float kMouseGrabForceGravityCompensation = 0.1f; // Helps moving objects close to the screen, especially in high gravity/scale scenes.

const float kMouseGrabJointMaxLinearAcceleration = 3000.0f;
const float kMouseGrabJointMaxAngularAcceleration = 3.14f; // Assuming rotation of 180 degrees per second
const float kMouseGrabJointMaxForceJointedMultiplier=0.3f;
const float kMouseGrabJointSpringDamperTau=0.16f; // Time delay within which the d6 error converges, lower => snappier, less stable

const float kDeformableTargetShapeRadius = 10.0f;


#define GU_CULLING_EPSILON_RAY_TRIANGLE FLT_EPSILON* FLT_EPSILON

static bool intersectRayTriangle(const PxVec3& orig,
                                 const PxVec3& dir,
                                 const PxVec3& vert0,
                                 const PxVec3& vert1,
                                 const PxVec3& vert2,
                                 PxReal& at,
                                 PxReal& au,
                                 PxReal& av)
{
    // Find vectors for two edges sharing vert0
    const PxVec3 edge1 = vert1 - vert0;
    const PxVec3 edge2 = vert2 - vert0;

    // Begin calculating determinant - also used to calculate U parameter
    const PxVec3 pvec = dir.cross(edge2); // error ~ |v2-v0|

    // If determinant is near zero, ray lies in plane of triangle
    const PxReal det = edge1.dot(pvec); // error ~ |v2-v0|*|v1-v0|

    // the non-culling branch
    if (PxAbs(det) < GU_CULLING_EPSILON_RAY_TRIANGLE)
        return false;

    const PxReal inv_det = 1.0f / det;

    // Calculate distance from vert0 to ray origin
    const PxVec3 tvec = orig - vert0; // error ~ |orig-v0|

    // Calculate U parameter and test bounds
    const PxReal u = tvec.dot(pvec) * inv_det;
    if (u < 0.0f || u > 1.0f)
        return false;

    // prepare to test V parameter
    const PxVec3 qvec = tvec.cross(edge1);

    // Calculate V parameter and test bounds
    const PxReal v = dir.dot(qvec) * inv_det;
    if (v < 0.0f || (u + v) > 1.0f)
        return false;

    // Calculate t, ray intersects triangle
    const PxReal t = edge2.dot(qvec) * inv_det;

    at = t;
    au = u;
    av = v;
    return true;
}

RaycastManager::RaycastManager()
{
    picker = Picker();
}

RaycastManager::~RaycastManager()
{
}

void RaycastManager::clearCommandBuffer()
{
    mManipCmdBuffer.clear();
}


::physx::PxQueryHitType::Enum omni::physx::raycastFilterExcludeInvisible::preFilter(const ::physx::PxFilterData& filterData0,
                                                         const ::physx::PxShape* shape,
                                                         const ::physx::PxRigidActor* actor,
                                                         ::physx::PxHitFlags& flag)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const usdparser::ObjectId shapeIndex = (usdparser::ObjectId)shape->userData;

    PhysXType type;
    const internal::InternalDatabase::Record* objectRecord = db.getFullRecord(type, shapeIndex);

    if (objectRecord &&
        (type == ePTShape || type == ePTCompoundShape) &&
        pxr::UsdGeomImageable(mStage->GetPrimAtPath(objectRecord->mPath)).ComputeVisibility() == pxr::UsdGeomTokens->invisible)
    {
        return PxQueryHitType::eNONE;
    }
    return PxQueryHitType::eBLOCK;
}

static bool raycastSingle(const PxVec3& orig,
                          const PxVec3& dir,
                          const PxReal distance,
                          PickType::Enum& outType,
                          PxRigidActor*& outRigidActor,
                          PxVec3& outPosition,
                          PxReal& outDistance,
                          DeformableId& outDeformableId,
                          PhysXScene*& outPhysXScene)
{
    PxRaycastHit hit;

    PxReal dist = distance;
    PxVec3 hitPoint(0.f);
    PickType::Enum hitType = PickType::eNONE;
    PxRigidActor* rigidActor = nullptr;
    PhysXScene* physXScene = nullptr;
    DeformableId deformableId;

    const PhysXScenesMap& physXScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    pxr::UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const omni::physx::raycastFilterExcludeInvisible filter = raycastFilterExcludeInvisible(stage);
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();

    for (PhysXScenesMap::const_reference ref : physXScenes)
    {
        PxScene* scene = ref.second->getScene();
        if (!scene)
            continue;


        //can't skip if PxSceneQueryExt::raycastSingle returns false, as particles and deformables need to be checked too
        if (PxSceneQueryExt::raycastSingle(
                *scene, orig, dir, dist, PxSceneQueryFlag::ePOSITION, hit,
                PxSceneQueryFilterData(PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC | PxQueryFlag::ePREFILTER),
                (settings->getAsBool(kSettingMouseGrabIgnoreInvisible) ? (PxQueryFilterCallback*)&filter : NULL)))
        {
            hitType = PickType::eRIGID;
            dist = hit.distance;
            hitPoint = hit.position;
            rigidActor = hit.actor;
            physXScene = ref.second;
        }

        const internal::InternalScene* intScene = ref.second->getInternalScene();
        
 #if USE_PHYSX_GPU
        for (size_t i = 0; i < intScene->mParticleSystems.size(); ++i)
        {
            internal::InternalPbdParticleSystem* system = intScene->mParticleSystems[i];
            const size_t nbCloths = system->mCloths.size();

            for (size_t j = 0; j < nbCloths; ++j)
            {
                internal::InternalParticleClothDeprecated* cloth = system->mCloths[j];

                if ((pxr::UsdGeomImageable(cloth->mPrim).ComputeVisibility() == pxr::UsdGeomTokens->invisible))
                    continue;

                ExtGpu::PxParticleVolumeBufferHelper& volumeBuffers = *cloth->mVolumeBuffer;
                const PxParticleVolume* volumes = volumeBuffers.getParticleVolumes();
                const ExtGpu::PxParticleVolumeMesh* volumeMeshes = volumeBuffers.getParticleVolumeMeshes();
                const PxU32 numVolumes = volumeBuffers.getNumVolumes();
                const PxU32* triangles = volumeBuffers.getTriangles();

                PxVec4* positions = cloth->mPositions;
                for (size_t k = 0; k < numVolumes; ++k)
                {
                    if (volumes[k].numParticles == 0)
                        continue;

                    const PxBounds3& bounds = volumes[k].bound;
                    PxBoxGeometry geom(bounds.getExtents());
                    PxTransform pose(bounds.getCenter());
                    PxU32 numHits = PxGeometryQuery::raycast(orig, dir, geom, pose, dist, PxSceneQueryFlag::ePOSITION, 1, &hit);

                    if (numHits > 0)
                    {
                        const PxU32 startIndex = volumeMeshes[k].startIndex;
                        const PxU32 triangleCount = volumeMeshes[k].count;

                        for (PxU32 t = 0; t < triangleCount; ++t)
                        {
                            const PxU32 ind = (startIndex + t) * 3;
                            {
                                PxVec3 v0 = cloth->mPositions[triangles[ind]].getXYZ();
                                PxVec3 v1 = cloth->mPositions[triangles[ind + 1]].getXYZ();
                                PxVec3 v2 = cloth->mPositions[triangles[ind + 2]].getXYZ();

                                PxReal at, uv, uw;
                                if (intersectRayTriangle(orig, dir, v0, v1, v2, at, uv, uw))
                                {
                                    if (at < dist)
                                    {
                                        hitType = PickType::ePARTICLE_CLOTH_DEPRECATED;
                                        dist = at;
                                        hitPoint = orig + dir * at;
                                        physXScene = ref.second;
                                        deformableId.object = (int)i;
                                        deformableId.subObject = (int)j;
                                        deformableId.bary = PxVec4(0.0f);

                                        // Pick closes point
                                        PxReal dist0 = (hitPoint - v0).magnitudeSquared();
                                        PxReal dist1 = (hitPoint - v1).magnitudeSquared();
                                        PxReal dist2 = (hitPoint - v2).magnitudeSquared();

                                        if (dist0 < dist1 && dist0 < dist2)
                                            deformableId.element = triangles[ind + 0];
                                        else if (dist1 < dist2)
                                            deformableId.element = triangles[ind + 1];
                                        else
                                            deformableId.element = triangles[ind + 2];
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
#endif

        for (size_t i = 0; i < intScene->mDeformableBodiesDeprecated.size(); ++i)
        {
            internal::InternalDeformableBodyDeprecated* deformableBody = intScene->mDeformableBodiesDeprecated[i];

            if ((pxr::UsdGeomImageable(deformableBody->mPrim).ComputeVisibility() == pxr::UsdGeomTokens->invisible))
                continue;

            const PxBounds3 bounds = deformableBody->mSoftBody->getWorldBounds();
            PxBoxGeometry geom(bounds.getExtents());
            PxTransform pose(bounds.getCenter());
            PxU32 numHits = PxGeometryQuery::raycast(orig, dir, geom, pose, dist, PxSceneQueryFlag::ePOSITION, 1, &hit);

            if (numHits > 0)
            {
                const PxVec4* positionsInvMass = deformableBody->mCollPositionInvMassH;

                const PxU32 nbSurfaceTriangles = PxU32(deformableBody->mCollMeshSurfaceTriangles.size());
                for (PxU32 t = 0; t < nbSurfaceTriangles; ++t)
                {
                    carb::Uint3 vindices = deformableBody->mCollMeshSurfaceTriangles[t];
                    PxVec3 v0 = positionsInvMass[vindices.x].getXYZ();
                    PxVec3 v1 = positionsInvMass[vindices.y].getXYZ();
                    PxVec3 v2 = positionsInvMass[vindices.z].getXYZ();

                    PxReal at, uv, uw;
                    if (intersectRayTriangle(orig, dir, v0, v1, v2, at, uv, uw))
                    {
                        if (at < dist)
                        {
                            hitType = PickType::eDEFORMABLE_BODY_DEPRECATED;
                            dist = at;
                            hitPoint = orig + dir * at;
                            physXScene = ref.second;
                            deformableId.object = (int)i;
                            deformableId.subObject = -1;
                            // hit point will need to be converted to sim mesh tet later.
                            // let's store the collision mesh surface triangle info for completeness
                            deformableId.element = t;
                            PxComputeBarycentric(v0, v1, v2, hitPoint, deformableId.bary);
                        }
                    }
                }
            }
        }

        for (size_t i = 0; i < intScene->mDeformableSurfacesDeprecated.size(); ++i)
        {
            internal::InternalDeformableSurfaceDeprecated* deformableSurface = intScene->mDeformableSurfacesDeprecated[i];

            if ((pxr::UsdGeomImageable(deformableSurface->mPrim).ComputeVisibility() == pxr::UsdGeomTokens->invisible))
                continue;

            const PxBounds3 bounds = deformableSurface->mDeformableSurface->getWorldBounds();
            PxBoxGeometry geom(bounds.getExtents());
            PxTransform pose(bounds.getCenter());
            PxU32 numHits = PxGeometryQuery::raycast(orig, dir, geom, pose, dist, PxSceneQueryFlag::ePOSITION, 1, &hit);

            if (numHits > 0)
            {
                const PxVec4* positionsInvMass = deformableSurface->mPositionInvMassH;
                const PxU32 nbTriangles = deformableSurface->mTriangleMesh->getNbTriangles();
                const PxU32* triangles = reinterpret_cast<const PxU32*>(deformableSurface->mTriangleMesh->getTriangles());
                for (PxU32 t = 0; t < nbTriangles; ++t)
                {
                    const PxU32* vindices = triangles + t*3;
                    const PxVec3 v0 = positionsInvMass[vindices[0]].getXYZ();
                    const PxVec3 v1 = positionsInvMass[vindices[1]].getXYZ();
                    const PxVec3 v2 = positionsInvMass[vindices[2]].getXYZ();

                    PxReal at, uv, uw;
                    if (intersectRayTriangle(orig, dir, v0, v1, v2, at, uv, uw))
                    {
                        if (at < dist)
                        {
                            hitType = PickType::eDEFORMABLE_SURFACE_DEPRECATED;
                            dist = at;
                            hitPoint = orig + dir * at;
                            physXScene = ref.second;
                            deformableId.object = (int)(i);
                            deformableId.subObject = -1;
                            deformableId.element = t;
                            PxComputeBarycentric(v0, v1, v2, hitPoint, deformableId.bary);
                        }
                    }
                }
            }
        }

        for (size_t i = 0; i < intScene->mVolumeDeformableBodies.size(); ++i)
        {
            internal::InternalVolumeDeformableBody* body = intScene->mVolumeDeformableBodies[i];

            if ((pxr::UsdGeomImageable(body->mBodyPrim).ComputeVisibility() == pxr::UsdGeomTokens->invisible))
                continue;

            const PxBounds3 bounds = body->mDeformableVolume->getWorldBounds();
            PxBoxGeometry geom(bounds.getExtents());
            PxTransform pose(bounds.getCenter());
            PxU32 numHits = PxGeometryQuery::raycast(orig, dir, geom, pose, dist, PxSceneQueryFlag::ePOSITION, 1, &hit);

            if (numHits > 0)
            {
                const PxVec4* positionsInvMass = body->mCollMeshPositionInvMassH;

                const PxU32 nbSurfaceTriangles = PxU32(body->mCollMeshSurfaceTriangles.size());
                for (PxU32 t = 0; t < nbSurfaceTriangles; ++t)
                {
                    carb::Uint3 vindices = body->mCollMeshSurfaceTriangles[t];
                    PxVec3 v0 = positionsInvMass[vindices.x].getXYZ();
                    PxVec3 v1 = positionsInvMass[vindices.y].getXYZ();
                    PxVec3 v2 = positionsInvMass[vindices.z].getXYZ();

                    PxReal at, uv, uw;
                    if (intersectRayTriangle(orig, dir, v0, v1, v2, at, uv, uw))
                    {
                        if (at < dist)
                        {
                            hitType = PickType::eVOLUME_DEFORMABLE;
                            dist = at;
                            hitPoint = orig + dir * at;
                            physXScene = ref.second;
                            deformableId.object = (int)i;
                            deformableId.subObject = -1;
                            // hit point will need to be converted to sim mesh tet later.
                            // let's store the collision mesh surface triangle info for completeness
                            deformableId.element = t;
                            PxComputeBarycentric(v0, v1, v2, hitPoint, deformableId.bary);
                        }
                    }
                }
            }
        }

        for (size_t i = 0; i < intScene->mSurfaceDeformableBodies.size(); ++i)
        {
            internal::InternalSurfaceDeformableBody* body = intScene->mSurfaceDeformableBodies[i];

            if ((pxr::UsdGeomImageable(body->mBodyPrim).ComputeVisibility() == pxr::UsdGeomTokens->invisible))
                continue;

            const PxBounds3 bounds = body->mDeformableSurface->getWorldBounds();
            PxBoxGeometry geom(bounds.getExtents());
            PxTransform pose(bounds.getCenter());
            PxU32 numHits = PxGeometryQuery::raycast(orig, dir, geom, pose, dist, PxSceneQueryFlag::ePOSITION, 1, &hit);

            if (numHits > 0)
            {
                const PxVec4* positionsInvMass = body->mSimMeshPositionInvMassH;
                const PxU32 nbTriangles = body->mTriangleMesh->getNbTriangles();
                const PxU32* triangles = (PxU32*)body->mTriangleMesh->getTriangles();

                for (PxU32 t = 0; t < nbTriangles; ++t)
                {
                    const carb::Uint3& vindices = *reinterpret_cast<const carb::Uint3*>(triangles + t * 3);
                    PxVec3 v0 = positionsInvMass[vindices.x].getXYZ();
                    PxVec3 v1 = positionsInvMass[vindices.y].getXYZ();
                    PxVec3 v2 = positionsInvMass[vindices.z].getXYZ();

                    PxReal at, uv, uw;
                    if (intersectRayTriangle(orig, dir, v0, v1, v2, at, uv, uw))
                    {
                        if (at < dist)
                        {
                            hitType = PickType::eSURFACE_DEFORMABLE;
                            dist = at;
                            hitPoint = orig + dir * at;
                            physXScene = ref.second;
                            deformableId.object = (int)(i);
                            deformableId.subObject = -1;
                            deformableId.element = t;
                            PxComputeBarycentric(v0, v1, v2, hitPoint, deformableId.bary);
                        }
                    }
                }
            }
        }
    }

    outPosition = hitPoint;
    outDistance = dist;
    outPhysXScene = physXScene;
    outType = hitType;
    outRigidActor = (hitType == PickType::eRIGID) ? rigidActor : nullptr;
    outDeformableId = (hitType == PickType::eRIGID) ? DeformableId() : deformableId;
    return hitType != PickType::eNONE;
}

static int findOverlappingParticles(const PxVec3& orig,
                                    const PxReal radius,
                                    const PxVec4* const pos_invMass,
                                    const PxU32 nbParticles,
                                    const int ignoreIndex,
                                    std::vector<unsigned int>& particles)
{
    int nbFound = 0;
    const PxReal sqRadius = radius * radius;
    for (PxU32 i = 0; i < nbParticles; ++i)
    {
        const PxVec3 pos = pos_invMass[i].getXYZ();
        if ((pos - orig).magnitudeSquared() < sqRadius && (int)i != ignoreIndex)
        {
            particles.push_back(i);
            nbFound++;
        }
    }
    return nbFound;
}


static void drawGrabbedPointVisualization(PxVec3 grabbedPosition, PxVec3 grabTargetPosition)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    carb::events::IEventPtr eventPtr = omniPhysX.createSimulationEventV2(
        ePointGrabbed, std::make_pair("grabbed_position", (const carb::Float3&)grabbedPosition),
        std::make_pair("grab_target_position", (const carb::Float3&)grabTargetPosition));
    omniPhysX.getSimulationEventStreamV2()->push(eventPtr.get());
}

static void removePointVisualization()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    carb::events::IEventPtr eventPtr = omniPhysX.createSimulationEventV2(ePointReleased);
    omniPhysX.getSimulationEventStreamV2()->push(eventPtr.get());
}

static void drawPushedPointVisualization(PxVec3 pushedPosition)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    carb::events::IEventPtr eventPtr = omniPhysX.createSimulationEventV2(
        ePointPushed, std::make_pair("pushed_position", (const carb::Float3&)pushedPosition));
    omniPhysX.getSimulationEventStreamV2()->push(eventPtr.get());
}

static PxRigidDynamic* createDeformableAttachmentActor(PxScene& scene, PxMaterial& material, PxVec3 pos, PxReal radius)
{
    PxRigidDynamic* actor = scene.getPhysics().createRigidDynamic(PxTransform(pos));
    if (actor)
    {
        scene.addActor(*actor);
        PxSphereGeometry geo = PxSphereGeometry(radius);
        CARB_ASSERT(geo.isValid());
        PxShape* shape = scene.getPhysics().createShape(geo, material, true, PxShapeFlag::eVISUALIZATION);
        actor->attachShape(*shape);
        shape->release();
        actor->setMass(0.f);
        actor->setMassSpaceInertiaTensor(PxVec3(0.f));
        actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    }
    return actor;
}

static bool getDeformableElementPosition(PxVec3& elementPos, const internal::InternalScene& intScene, PickType::Enum type, const DeformableId& id)
{
    if (type == PickType::eDEFORMABLE_BODY_DEPRECATED)
    {
        if (id.object < intScene.mDeformableBodiesDeprecated.size())
        {
            internal::InternalDeformableBodyDeprecated* deformableBody = intScene.mDeformableBodiesDeprecated[id.object];
            if (uint32_t(id.element) < deformableBody->mCollMeshSurfaceTriangles.size())
            {
                carb::Uint3 vindices = deformableBody->mCollMeshSurfaceTriangles[id.element];
                const PxVec3& v0 = deformableBody->mCollPositionInvMassH[vindices.x].getXYZ();
                const PxVec3& v1 = deformableBody->mCollPositionInvMassH[vindices.y].getXYZ();
                const PxVec3& v2 = deformableBody->mCollPositionInvMassH[vindices.z].getXYZ();
                elementPos = v0 * id.bary.x + v1 * id.bary.y + v2 * id.bary.z;
                return true;
            }
        }
    }
    else if (type == PickType::eDEFORMABLE_SURFACE_DEPRECATED)
    {
        if (id.object < intScene.mDeformableSurfacesDeprecated.size())
        {
            internal::InternalDeformableSurfaceDeprecated* deformableSurface = intScene.mDeformableSurfacesDeprecated[id.object];
            const PxU32 nbTriangles = deformableSurface->mTriangleMesh->getNbTriangles();
            const PxU32* triangles = reinterpret_cast<const PxU32*>(deformableSurface->mTriangleMesh->getTriangles());
            if (uint32_t(id.element) < nbTriangles)
            {
                const PxU32* vindices = triangles + id.element*3;
                const PxVec3& v0 = deformableSurface->mPositionInvMassH[vindices[0]].getXYZ();
                const PxVec3& v1 = deformableSurface->mPositionInvMassH[vindices[1]].getXYZ();
                const PxVec3& v2 = deformableSurface->mPositionInvMassH[vindices[2]].getXYZ();
                elementPos = v0 * id.bary.x + v1 * id.bary.y + v2 * id.bary.z;
                return true;
            }
        }
    }
    else if (type == PickType::ePARTICLE_CLOTH_DEPRECATED)
    {
        if (id.object < intScene.mParticleSystems.size())
        {
            internal::InternalPbdParticleSystem* system = intScene.mParticleSystems[id.object];
            if (id.subObject >= 0 && id.subObject < system->mCloths.size())
            {
                internal::InternalParticleClothDeprecated* cloth = system->mCloths[id.subObject];
                if (uint32_t(id.element) < cloth->mNumParticles)
                {
                    elementPos = cloth->mPositions[id.element].getXYZ();
                    return true;
                }
            }
        }
    }
    else if (type == PickType::eVOLUME_DEFORMABLE)
    {
        if (id.object < intScene.mVolumeDeformableBodies.size())
        {
            internal::InternalVolumeDeformableBody* body = intScene.mVolumeDeformableBodies[id.object];
            if (uint32_t(id.element) < body->mCollMeshSurfaceTriangles.size())
            {
                carb::Uint3 vindices = body->mCollMeshSurfaceTriangles[id.element];
                const PxVec3& v0 = body->mCollMeshPositionInvMassH[vindices.x].getXYZ();
                const PxVec3& v1 = body->mCollMeshPositionInvMassH[vindices.y].getXYZ();
                const PxVec3& v2 = body->mCollMeshPositionInvMassH[vindices.z].getXYZ();
                elementPos = v0 * id.bary.x + v1 * id.bary.y + v2 * id.bary.z;
                return true;
            }
        }
    }
    else if (type == PickType::eSURFACE_DEFORMABLE)
    {
        if (id.object < intScene.mDeformableSurfacesDeprecated.size())
        {
            internal::InternalSurfaceDeformableBody* body = intScene.mSurfaceDeformableBodies[id.object];
            const PxU32 nbTriangles = body->mTriangleMesh->getNbTriangles();
            const PxU32* triangles = reinterpret_cast<const PxU32*>(body->mTriangleMesh->getTriangles());
            if (uint32_t(id.element) < nbTriangles)
            {
                const PxU32* vindices = triangles + id.element * 3;
                const PxVec3& v0 = body->mSimMeshPositionInvMassH[vindices[0]].getXYZ();
                const PxVec3& v1 = body->mSimMeshPositionInvMassH[vindices[1]].getXYZ();
                const PxVec3& v2 = body->mSimMeshPositionInvMassH[vindices[2]].getXYZ();
                elementPos = v0 * id.bary.x + v1 * id.bary.y + v2 * id.bary.z;
                return true;
            }
        }
    }
    return false;
}

static void startPickingDeformable(const PickType::Enum type, const DeformableId& deformableId, const PxReal distance, const PxVec3& hitPoint)
{
    if (type == PickType::eNONE || type == PickType::eRIGID)
        return;

    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    PxPhysics* physics = omniPhysX.getPhysXSetup().getPhysics();
    Picker& picker = omniPhysX.getRaycastManager().getPicker();
    internal::InternalScene* intScene = picker.physXScene ? picker.physXScene->getInternalScene() : nullptr;
    if (!intScene)
        return;

    PxMaterial* defaultMat = picker.physXScene->getDefaultMaterial();
    PxScene* scene = intScene->getScene();
    if (!defaultMat || !scene)
        return;

    if (!picker.targetActor)
    {
        picker.targetActor = createDeformableAttachmentActor(*scene, *defaultMat, hitPoint, kDeformableTargetShapeRadius);
    }

    if (!picker.targetActor)
        return;

    picker.type = type;
    picker.hitDeformableId = deformableId;

    picker.targetActor->setGlobalPose(PxTransform(hitPoint));
    std::vector<PxVec4> rigidLocalCoords;
    rigidLocalCoords.push_back(PxVec4(0.0f));

    if (type == PickType::eDEFORMABLE_BODY_DEPRECATED && deformableId.object < intScene->mDeformableBodiesDeprecated.size())
    {
        internal::InternalDeformableBodyDeprecated* deformableBody = intScene->mDeformableBodiesDeprecated[deformableId.object];
        CARB_ASSERT(scene == deformableBody->mSoftBody->getScene());

        std::vector<PxU32> tetIndices;
        std::vector<PxVec4> tetBarycentrics;
        std::vector<PxVec3> positions;
        positions.push_back(hitPoint);
        deformableBody->findTetsFromPoints(positions, tetIndices, tetBarycentrics);

        // sanity check that it is a valid tri index
        if (tetIndices[0] != -1)
        {
            std::vector<PxU32> filterIndices;
            filterIndices.assign(tetIndices.begin(), tetIndices.end());

            deformableBody->convertToPhysxAttachmentTets(tetIndices, tetBarycentrics);
            PxDeformableAttachment* attachment =  deformableBody->addRigidAttachments(picker.targetActor, tetIndices, tetBarycentrics, rigidLocalCoords);
            picker.deformableAttachment = attachment;

            PxDeformableElementFilter* filter = deformableBody->addRigidFilters(picker.targetActor, filterIndices);
            picker.deformableFilter = filter;
        }
    }
    else if (type == PickType::eDEFORMABLE_SURFACE_DEPRECATED && deformableId.object < intScene->mDeformableSurfacesDeprecated.size())
    {
        internal::InternalDeformableSurfaceDeprecated* deformableSurface = intScene->mDeformableSurfacesDeprecated[deformableId.object];
        CARB_ASSERT(scene == deformableSurface->mDeformableSurface->getScene());

        std::vector<PxU32> triIndices;
        triIndices.push_back(deformableId.element);

        std::vector<PxVec4> triBarycentrics;
        triBarycentrics.push_back(deformableId.bary);

        // sanity check that it is a valid tri index
        if (triIndices[0] != -1)
        {
            PxDeformableAttachment* attachment = deformableSurface->addRigidAttachments(picker.targetActor, triIndices, triBarycentrics, rigidLocalCoords);
            picker.deformableAttachment = attachment;

            PxDeformableElementFilter* filter = deformableSurface->addRigidFilters(picker.targetActor, triIndices);
            picker.deformableFilter = filter;
        }
    }
    else if (type == PickType::ePARTICLE_CLOTH_DEPRECATED && deformableId.object < intScene->mParticleSystems.size())
    {
        internal::InternalPbdParticleSystem* system = intScene->mParticleSystems[deformableId.object];
        CARB_ASSERT(scene == system->mPS->getScene());

        system->mCloths[deformableId.subObject]->addRigidAttachment(picker.targetActor, deformableId.element, rigidLocalCoords[0].getXYZ());
#if USE_PHYSX_GPU
        if (system->mCloths[deformableId.subObject]->mAttachments)
            system->mCloths[deformableId.subObject]->mAttachments->copyToDevice();
#endif
    }
    else if (type == PickType::eVOLUME_DEFORMABLE && deformableId.object < intScene->mVolumeDeformableBodies.size())
    {
        internal::InternalVolumeDeformableBody* body = intScene->mVolumeDeformableBodies[deformableId.object];
        CARB_ASSERT(scene == body->mDeformableVolume->getScene());

        std::vector<PxU32> tetIndices;
        std::vector<PxVec4> tetBarycentrics;
        std::vector<PxVec3> positions;
        positions.push_back(hitPoint);
        body->findTetsFromPoints(positions, tetIndices, tetBarycentrics, false);

        // sanity check that it is a valid tri index
        if (tetIndices[0] != -1)
        {
            std::vector<PxU32> filterIndices;
            filterIndices.assign(tetIndices.begin(), tetIndices.end());

            body->convertToPhysxAttachmentTets(tetIndices, tetBarycentrics);
            PxDeformableAttachment* attachment = body->addRigidAttachments(picker.targetActor, tetIndices, tetBarycentrics, rigidLocalCoords);
            picker.deformableAttachment = attachment;

            PxDeformableElementFilter* filter = body->addRigidFilters(picker.targetActor, filterIndices);
            picker.deformableFilter = filter;
        }
    }
    else if (type == PickType::eSURFACE_DEFORMABLE && deformableId.object < intScene->mSurfaceDeformableBodies.size())
    {
        internal::InternalSurfaceDeformableBody* body = intScene->mSurfaceDeformableBodies[deformableId.object];
        CARB_ASSERT(scene == body->mDeformableSurface->getScene());

        std::vector<PxU32> triIndices;
        triIndices.push_back(deformableId.element);

        std::vector<PxVec4> triBarycentrics;
        triBarycentrics.push_back(deformableId.bary);

        // sanity check that it is a valid tri index
        if (triIndices[0] != -1)
        {
            PxDeformableAttachment* attachment = body->addRigidAttachments(picker.targetActor, triIndices, triBarycentrics, rigidLocalCoords);
            picker.deformableAttachment = attachment;

            PxDeformableElementFilter* filter = body->addRigidFilters(picker.targetActor, triIndices);
            picker.deformableFilter = filter;
        }
    }
}

static void updatePickingDeformable(const PxVec3& target)
{
    Picker& picker = OmniPhysX::getInstance().getRaycastManager().getPicker();
    picker.targetActor->setKinematicTarget(PxTransform(target));

    const DeformableId& id = picker.hitDeformableId;
    internal::InternalScene* intScene = picker.physXScene ? picker.physXScene->getInternalScene() : nullptr;
    if (!intScene)
        return;

    PxVec3 currentPos;
    if (!getDeformableElementPosition(currentPos, *intScene, picker.type, id))
    {
        currentPos = target;
    }

    drawGrabbedPointVisualization(currentPos, target);
}

static void releasePickingDeformable()
{
    Picker& picker = OmniPhysX::getInstance().getRaycastManager().getPicker();
    if (picker.type == PickType::eNONE || picker.type == PickType::eRIGID)
        return;

    internal::InternalScene* intScene = picker.physXScene ? picker.physXScene->getInternalScene() : nullptr;
    if (!intScene)
        return;

    if (picker.type == PickType::ePARTICLE_CLOTH_DEPRECATED)
    {
        internal::InternalPbdParticleSystem* system = intScene->mParticleSystems[picker.hitDeformableId.object];
        CARB_ASSERT(system->mPS->getScene() == intScene->getScene());
        if (picker.targetActor && picker.hitDeformableId.element >= 0)
        {
            system->mCloths[picker.hitDeformableId.subObject]->removeRigidAttachment(
                picker.targetActor, picker.hitDeformableId.element);
        }

#if USE_PHYSX_GPU
        if (system->mCloths[picker.hitDeformableId.subObject]->mAttachments)
            system->mCloths[picker.hitDeformableId.subObject]->mAttachments->copyToDevice();
#endif
    }

    if (picker.deformableAttachment)
    {
        SAFE_RELEASE(picker.deformableAttachment);
    }
    if (picker.deformableFilter)
    {
        SAFE_RELEASE(picker.deformableFilter);
    }

    picker.hitDeformableId = DeformableId();

    if (picker.targetActor)
    {
        if (intScene->getScene())
        {
            intScene->getScene()->removeActor(*picker.targetActor);
        }
        SAFE_RELEASE(picker.targetActor);
    }

    removePointVisualization();
}

static void applyManipCmd(const ManipCmd& cmd, float delta_time)
{
    static float manipDist = 0.0f;

    static PxVec3 grabbingLocalPoint = PxVec3(0.0f);
    static float grabbingDistance;
    static float grabbingSavedAngularDamping;

    Picker& picker = OmniPhysX::getInstance().getRaycastManager().getPicker();

    PickType::Enum type = PickType::eNONE;
    PxVec3 position;
    PxReal distance = 0.0f;
    PxRigidActor* rigidActor = nullptr;
    DeformableId deformableId;
    PhysXScene* physXScene = nullptr;
    bool isRaycastSingle = false;
    switch (cmd.type)
    {
    case ManipCmd::ePush:
        if(cmd.state==ManipCmd::eStartOrUpdate)
        {
            isRaycastSingle = raycastSingle(cmd.orig, cmd.dir, kPickDistance, type, rigidActor, position, distance, deformableId, physXScene);
            if (isRaycastSingle)
            {
                if (type == PickType::eRIGID)
                {
                    PxRigidBody* rb = rigidActor->is<PxRigidBody>();
                    if (rb && !(rb->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) && !(rb->getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
                    {
                        // important to use impulse, because unlike forces, impulses are not retained by the actor if the retain accelerations flag is set (rb or link)
                        const PxVec3 impulse = cmd.dir * cmd.acceleration * rb->getMass() * delta_time;
                        PxRigidBodyExt::addForceAtPos(*rb, impulse, position, PxForceMode::eIMPULSE);

                        drawPushedPointVisualization(position);
                    }
                }
            }
        }
        else if(cmd.state==ManipCmd::eRelease)
        {
            removePointVisualization();
        }
        break;
    case ManipCmd::eGrabWithForce:
        if(cmd.state==ManipCmd::eStartOrUpdate)
        {
            if (picker.hitRigidActor && !(picker.hitRigidActor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) && !(picker.hitRigidActor->getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
            {
                const PxTransform worldTransform = picker.hitRigidActor->getGlobalPose();
                const PxVec3 newWorldPt = worldTransform.transform(grabbingLocalPoint);
                const PxVec3 dragPoint = cmd.orig + cmd.dir * grabbingDistance;

                float grabbingForce = kMouseGrabForceMultiplier;
                {
                    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
                    grabbingForce *= settings->getAsFloat(kSettingMousePickingForce);
                }

                PxVec3 action = (dragPoint - newWorldPt);

                // Apply some of opposite current velocity to force.
                // This is similar to linear and angular dampening but better adjusts to grabbed point offset.
                const PxVec3 velocity = PxRigidBodyExt::getVelocityAtPos(*picker.hitRigidActor, newWorldPt);
                action -= velocity * kMouseGrabForcePointDamping;

                PxRigidBodyExt::addForceAtPos(
                    *picker.hitRigidActor, grabbingForce * picker.hitRigidActor->getMass() * action * delta_time,
                                              newWorldPt, PxForceMode::eIMPULSE);

                if(!(picker.hitRigidActor->getActorFlags() & ::physx::PxActorFlag::eDISABLE_GRAVITY))
                {
                    // Compensate for some of the scene gravity to assist in lifting the object when it is close to the screen / gravity is high.
                    const ::physx::PxScene* scene = picker.physXScene->getScene();
                    const PxVec3 gravity = scene->getGravity();
                    float fGravityMagnitude = gravity.magnitude();
                    if(fGravityMagnitude > 0.0f)
                    {
                        PxVec3 gravityCompensation = -gravity *
                             std::max(0.0f, (fGravityMagnitude - kMouseGrabForceGravityCompensation * grabbingForce * grabbingDistance)) / fGravityMagnitude;

                        picker.hitRigidActor->addForce(gravityCompensation * delta_time, PxForceMode::eVELOCITY_CHANGE);
                    }
                }
                drawGrabbedPointVisualization(newWorldPt, dragPoint);
            }
            else
            {
                if (picker.type == PickType::eNONE)
                {
                    if (raycastSingle(cmd.orig, cmd.dir, kPickDistance, type, rigidActor, position, distance, deformableId, physXScene))
                    {
                        picker.physXScene = physXScene;
                        if (type == PickType::eRIGID)
                        {
                            picker.hitRigidActor = rigidActor->is<PxRigidBody>();
                            if(picker.hitRigidActor)
                            {
                                picker.type = PickType::eRIGID;
                                const PxTransform worldTransform = picker.hitRigidActor->getGlobalPose();

                                float grabbingForce = kMouseGrabForceMultiplier;
                                {
                                    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
                                    grabbingForce *= settings->getAsFloat(kSettingMousePickingForce);
                                }

                                grabbingDistance = distance;
                                grabbingLocalPoint = worldTransform.transformInv(position);
                                grabbingSavedAngularDamping = picker.hitRigidActor->getAngularDamping();

                                picker.hitRigidActor->setAngularDamping(fmaxf(kMouseGrabForceAngularDamping * grabbingForce, grabbingSavedAngularDamping));
                                drawGrabbedPointVisualization(position, position);
                            }
                        }
                        else
                        {
                            startPickingDeformable(type, deformableId, distance, position);
                            manipDist = distance;

                            if(picker.targetActor != nullptr)
                            {
                                const PxTransform worldTransform = picker.targetActor->getGlobalPose();
                                grabbingLocalPoint = worldTransform.transformInv(position);
                                drawGrabbedPointVisualization(position, position);
                            }
                        }
                    }
                }
                else
                {
                    const PxVec3 target = cmd.orig + manipDist * cmd.dir;
                    if (picker.type == PickType::eRIGID)
                    {
                        if (picker.hitRigidActor)
                        {
                            const PxTransform worldTransform = picker.hitRigidActor->getGlobalPose();
                            const PxVec3 newWorldPt = worldTransform.transform(grabbingLocalPoint);
                            const PxVec3 dragPoint = cmd.orig + cmd.dir * grabbingDistance;
                            drawGrabbedPointVisualization(newWorldPt, dragPoint);
                        }
                    }
                    else if (picker.type != PickType::eNONE)
                    {
                        updatePickingDeformable(target);
                    }
                }
            }
        }
        else if(cmd.state==ManipCmd::eRelease)
        {
            if (picker.type == PickType::eRIGID)
            {
                if (picker.hitRigidActor)
                {
                    picker.hitRigidActor->setAngularDamping(grabbingSavedAngularDamping);

                    PxRigidDynamic* rd = picker.hitRigidActor->is<PxRigidDynamic>();
                    if (rd && !(rd->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
                    {
                        rd->wakeUp();
                    }
                    picker.hitRigidActor = nullptr;
                }
            }
            else if (picker.type != PickType::eNONE)
            {
                releasePickingDeformable();
            }
            picker.physXScene = nullptr;
            picker.type = PickType::eNONE;
            removePointVisualization();
        }
        break;

    case ManipCmd::eGrabWithJoint:
        switch (cmd.state)
        {
        case ManipCmd::eStartOrUpdate:
            if (picker.type == PickType::eNONE && picker.targetActor == nullptr)
            {
                isRaycastSingle = raycastSingle(cmd.orig, cmd.dir, kPickDistance, type, rigidActor, position, distance,
                                                deformableId, physXScene);
                if (isRaycastSingle)
                {
                    picker.physXScene = physXScene;
                    if (type == PickType::eRIGID)
                    {
                        picker.hitRigidActor = rigidActor->is<PxRigidBody>();
                        if (picker.hitRigidActor != nullptr)
                        {
                            picker.type = PickType::eRIGID;
                            manipDist = distance;
                            PxTransform dragPose(position);
                            
                            const PxTransform worldTransform = picker.hitRigidActor->getGlobalPose();
                            grabbingLocalPoint = worldTransform.transformInv(position);

                            // Create joint actor.
                            {
                                PxPhysics* physics = OmniPhysX::getInstance().getPhysXSetup().getPhysics();
                                PxRigidDynamic* rd = physics->createRigidDynamic(PxTransform(position));

                                ::physx::PxScene* scene = picker.physXScene->getScene();
                                if (rd && scene)
                                {
                                    scene->addActor(*rd);

                                    const PxMaterial* material = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene((size_t)scene->userData)->getDefaultMaterial();
                                    PxSphereGeometry geo = PxSphereGeometry(1.0f);
                                    CARB_ASSERT(geo.isValid());
                                    PxShape* shape = physics->createShape(geo, *material, true, PxShapeFlag::eVISUALIZATION);
                                    rd->attachShape(*shape);
                                    shape->release();

                                    rd->setMass(0.f);
                                    rd->setMassSpaceInertiaTensor(PxVec3(0.f));
                                    rd->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
                                    rd->setRigidDynamicLockFlag(physx::PxRigidDynamicLockFlag::eLOCK_LINEAR_X, false);
                                    rd->setRigidDynamicLockFlag(physx::PxRigidDynamicLockFlag::eLOCK_LINEAR_Y, false);
                                    rd->setRigidDynamicLockFlag(physx::PxRigidDynamicLockFlag::eLOCK_LINEAR_Z, false);
                                    rd->setRigidDynamicLockFlag(physx::PxRigidDynamicLockFlag::eLOCK_ANGULAR_X, false);
                                    rd->setRigidDynamicLockFlag(physx::PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y, false);
                                    rd->setRigidDynamicLockFlag(physx::PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z, false);

                                    picker.targetActor = rd;

                                    PxD6Joint* d6 = PxD6JointCreate(*OmniPhysX::getInstance().getPhysXSetup().getPhysics(), rd, PxTransform(PxVec3(0.0f,0.0f,0.0f)), picker.hitRigidActor, PxTransform(grabbingLocalPoint));

                                    d6->setAngularDriveConfig(PxD6AngularDriveConfig::eSWING_TWIST);

                                    d6->setMotion(::physx::PxD6Axis::eX, ::physx::PxD6Motion::eFREE);
                                    d6->setMotion(::physx::PxD6Axis::eY, ::physx::PxD6Motion::eFREE);
                                    d6->setMotion(::physx::PxD6Axis::eZ, ::physx::PxD6Motion::eFREE);
                                    d6->setMotion(::physx::PxD6Axis::eSWING1, ::physx::PxD6Motion::eFREE);
                                    d6->setMotion(::physx::PxD6Axis::eSWING2, ::physx::PxD6Motion::eFREE);
                                    d6->setMotion(::physx::PxD6Axis::eTWIST, ::physx::PxD6Motion::eFREE);

                                    //spring damper
                                    const float m = picker.hitRigidActor->getMass();
                                    float damping = 2.0f * m / kMouseGrabJointSpringDamperTau;
                                    float stiffness = damping * damping / (4 * m);

                                    //max force
                                    const ::physx::PxU32 nbConstraints = picker.hitRigidActor->getNbConstraints();
                                    const float kJointed = nbConstraints > 1 ? kMouseGrabJointMaxForceJointedMultiplier : 1.0f;
                                    float maxForce = kJointed * m * kMouseGrabJointMaxLinearAcceleration;

                                    ::physx::PxD6JointDrive linearDrive = physx::PxD6JointDrive(stiffness, damping, maxForce);
                                    d6->setDrive(::physx::PxD6Drive::eX, linearDrive);
                                    d6->setDrive(::physx::PxD6Drive::eY, linearDrive);
                                    d6->setDrive(::physx::PxD6Drive::eZ, linearDrive);

                                    //spring damper
                                    const float inertia = picker.hitRigidActor->getMassSpaceInertiaTensor().magnitude();
                                    //max force
                                    maxForce = kJointed * inertia * kMouseGrabJointMaxAngularAcceleration;

                                    physx::PxD6JointDrive angularDrive = physx::PxD6JointDrive(stiffness, damping, maxForce);
                                    d6->setDrive(::physx::PxD6Drive::eTWIST, angularDrive);
                                    d6->setDrive(::physx::PxD6Drive::eSWING1, angularDrive);
                                    d6->setDrive(::physx::PxD6Drive::eSWING2, angularDrive);

                                    picker.joint = d6;
                                }

                            }
                            drawGrabbedPointVisualization(position, position);
                        }
                    }
                    else
                    {
                        startPickingDeformable(type, deformableId, distance, position);
                        manipDist = distance; 
                        if (picker.targetActor != nullptr)
                        {
                            drawGrabbedPointVisualization(position, position);
                        }
                    }
                }
            }
            else
            {
                CARB_ASSERT(picker.targetActor);
                const PxVec3 target = cmd.orig + manipDist * cmd.dir;
                if(picker.type == PickType::eRIGID)
                {
                    PxTransform dragPose(target);

                    picker.targetActor->setKinematicTarget(dragPose);
                    //Set global pose too. This forces kinematic velocity to 0.0 which prevents overshooting with our
                    //critical damping approach.
                    picker.targetActor->setGlobalPose(dragPose);

                    const PxTransform worldTransform = picker.hitRigidActor->getGlobalPose();
                    const PxVec3 newWorldPt = worldTransform.transform(grabbingLocalPoint);
                    drawGrabbedPointVisualization(newWorldPt, target);
                }
                else if (picker.type != PickType::eNONE)
                {
                    updatePickingDeformable(target);
                }
            }
            break;
        case ManipCmd::eRelease:
        default:
            if (picker.type == PickType::eRIGID)
            {
                if (picker.hitRigidActor != nullptr)
                {
                    PxRigidDynamic* rd = picker.hitRigidActor->is<PxRigidDynamic>();
                    if (rd && !(rd->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
                    {
                        rd->wakeUp();
                    }
                    picker.hitRigidActor = nullptr;
                }
                if (picker.targetActor != nullptr)
                {
                    CARB_ASSERT(picker.physXScene->getScene());
                    picker.physXScene->getScene()->removeActor(*picker.targetActor);
                    SAFE_RELEASE(picker.targetActor);
                }
                SAFE_RELEASE(picker.joint);
            }
            else if (picker.type != PickType::eNONE)
            {
                releasePickingDeformable();
            }
            picker.type = PickType::eNONE;
            removePointVisualization();
            break;
        }
        break;
    case ManipCmd::eManipCmdTypeCount:
        break;
    }
}

void RaycastManager::clearPicker(const ::physx::PxRigidActor* rb)
{
    if (picker.hitRigidActor == rb)
    {
        picker.hitRigidActor = nullptr;
        picker.physXScene = nullptr;
        picker.type = PickType::eNONE;
    }
}

void RaycastManager::onUpdateRaycasts(float delta_time) const
{
    // Lock buffer and apply forces
    mManipCmdBuffer.read(
        [](const ManipCmd* cmd, size_t size, void* userData) {
            for (const ManipCmd* stop = cmd + size; cmd < stop; ++cmd)
            {
                applyManipCmd(*cmd, *(static_cast<float*>(userData)));
            }
        },
        &delta_time);

    // Remove any relevant commands from the command buffer depending on execution type.
    mManipCmdBuffer.access([](std::vector<ManipCmd>& list, void* userData) {
        // If we've encountered a terminal command, also erase any leading commands. Handle this by iterating in reverse.
        bool erase = false;
        for(typename std::vector<ManipCmd>::reverse_iterator data_ptr = list.rbegin(); data_ptr != list.rend();)
        {
            if(!erase && (&(*data_ptr))->exec == ManipCmd::eTerminal)
            {
                // From here on, all commands (including this) should be erased.
                erase = true;
            }

            // Check if we should delete this entry.
            if(erase || (&(*data_ptr))->exec == ManipCmd::eOnce)
            {
                // To erase, we need a forward iterator, which is always the next reverse iterator.
                data_ptr =  std::make_reverse_iterator(
                                list.erase(std::next(data_ptr).base())
                                );
            }
            else
            {
                // Note that when using next with reverse iterators, we actually move backwards in the list.
                data_ptr = std::next(data_ptr);
            }
        }}, nullptr );
}

void RaycastManager::push(const float* orig, const float* dir, float acceleration)
{
    ManipCmd cmd;
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    cmd.type = ManipCmd::ePush;
    cmd.acceleration = settings->getAsFloat(kSettingMousePush);
    if (orig && dir)
    {
        cmd.orig = PxVec3(orig[0], orig[1], orig[2]);
        cmd.dir = PxVec3(dir[0], dir[1], dir[2]);
        cmd.state = ManipCmd::eStartOrUpdate;
        cmd.exec = ManipCmd::eRepeating;
    }
    else
    {
        cmd.state = ManipCmd::eRelease;
        cmd.exec = ManipCmd::eTerminal;
    }
    mManipCmdBuffer.push(cmd);
}

void RaycastManager::grab(const float* orig, const float* dir)
{
    ManipCmd cmd;
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    cmd.type = settings->getAsBool(kSettingMouseGrabWithForce) ? ManipCmd::eGrabWithForce : ManipCmd::eGrabWithJoint;
    cmd.acceleration = 0.0f; // Unused
    if (orig && dir)
    {
        cmd.orig = PxVec3(orig[0], orig[1], orig[2]);
        cmd.dir = PxVec3(dir[0], dir[1], dir[2]);
        cmd.state = ManipCmd::eStartOrUpdate;
        cmd.exec = ManipCmd::eRepeating;
    }
    else
    {
        cmd.state = ManipCmd::eRelease;
        cmd.exec = ManipCmd::eTerminal;
    }
    mManipCmdBuffer.push(cmd);
}

void RaycastManager::handleInteractionEvent(const float* orig, const float* dir, PhysicsInteractionEvent interactionEvent)
{
    // Remove any trailing repeating commands.
    mManipCmdBuffer.access([](std::vector<ManipCmd>& list, void* userData) {
        for(typename std::vector<ManipCmd>::reverse_iterator data_ptr = list.rbegin(); data_ptr != list.rend();)
        {
            if(data_ptr->exec == ManipCmd::eTerminal)
            {
                // This command will make sure that any repeating commands prior in the buffer will be removed too so we simply let this take care of it.
                break;
            }

            // Check if we should delete this entry.
            if(data_ptr->exec == ManipCmd::eRepeating)
            {
                // To erase, we need a forward iterator, which is always the next reverse iterator.
                data_ptr =  std::make_reverse_iterator(
                                list.erase(std::next(data_ptr).base())
                                );
            }
            else
            {
                // Note that when using next with reverse iterators, we actually move backwards in the list.
                data_ptr = std::next(data_ptr);
            }
        }}, nullptr );

    static bool grabInProgress = false;
    static bool pushInProgress = false;
    static PhysicsInteractionEvent lastEvent = PhysicsInteractionEvent::eNone;

    switch(interactionEvent)
    {
    case PhysicsInteractionEvent::eMouseLeftDoubleClick:
    case PhysicsInteractionEvent::eMouseDragBegan:
    {
        carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
        if (settings->getAsBool(kSettingMouseInteractionEnabled))
        {
            // Whether the drag began event or double click is first is not guaranteed,
            // so we use push both when this or last event was double click.
            if (!settings->getAsBool(kSettingMouseGrab) || interactionEvent == PhysicsInteractionEvent::eMouseLeftDoubleClick
                || lastEvent == PhysicsInteractionEvent::eMouseLeftDoubleClick)
            {
                const float acceleration = settings->getAsFloat(kSettingMousePush);
                pushInProgress = true;
                grabInProgress = false;
                push(orig, dir, acceleration);
            }
            else
            {
                grabInProgress = true;
                grab(orig, dir); // Start grab
            }
        }
        break;
    }
    case PhysicsInteractionEvent::eMouseDragEnded:
    {
        if (grabInProgress)
        {
            grabInProgress = false;
            grab(nullptr, nullptr); // Stop grab
        }
        if (pushInProgress)
        {
            pushInProgress = false;
            push(nullptr, nullptr, 0.0f); // Stop push
        }
        break;
    }
    case PhysicsInteractionEvent::eMouseDragChanged:
    {
        if (grabInProgress)
        {
            grab(orig, dir); // Update grab
        }
        if (pushInProgress)
        {
            carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
            const float acceleration = settings->getAsFloat(kSettingMousePush);
            push(orig, dir, acceleration); // Update push
        }
        break;
    }

    }
    lastEvent = interactionEvent;
}

void omni::physx::handleRaycast(const float* orig, const float* dir, bool input)
{
    static bool lastInput = false;
    const bool mouseClicked = input && !lastInput;

    PhysicsInteractionEvent interactionEvent;
    if(mouseClicked)
    {
        interactionEvent = PhysicsInteractionEvent::eMouseDragBegan;
    }
    else if(input)
    {
        interactionEvent = PhysicsInteractionEvent::eMouseDragChanged;
    }
    else
    {
        interactionEvent = PhysicsInteractionEvent::eMouseDragEnded;
    }
    OmniPhysX::getInstance().getRaycastManager().handleInteractionEvent(orig, dir, interactionEvent);
    lastInput = input;
}

bool RaycastManager::interactiveActorRaycast(const carb::Float3* origin, const carb::Float3* direction)
{
    PxVec3 position(0.0f);
    PxReal distance(0.0f);
    DeformableId deformableId;
    PxRigidActor* rigidActor = nullptr;
    PhysXScene* physXScene = nullptr;
    PickType::Enum type;
    bool hit = raycastSingle(toPhysX(*origin), toPhysX(*direction), kPickDistance, type, rigidActor, position, distance, deformableId, physXScene);

    if (hit)
    {
        bool hitRigidBodyOrLink = false;
        if (rigidActor)
        {
            PxRigidBody* body = rigidActor->is<PxRigidBody>();
            if (body)
            {
                hitRigidBodyOrLink = true;
                if (body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
                {
                    // cannot pick kinematic
                    hitRigidBodyOrLink = false;
                }
            }
        }
        const bool hitDeformable = deformableId.object >= 0;
        return hitRigidBodyOrLink || hitDeformable;
    }
    return false;
}
