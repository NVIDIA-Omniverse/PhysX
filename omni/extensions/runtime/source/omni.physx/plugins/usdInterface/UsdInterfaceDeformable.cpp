// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include "UsdInterface.h"

#include <private/omni/physx/PhysxUsd.h>

#include <usdLoad/LoadUsd.h>
#include <usdLoad/Mass.h>

#include <internal/InternalScene.h>
#include <internal/InternalDeformable.h>
#include <internal/InternalTools.h>

#include <PhysXTools.h>
#include <CookingDataAsync.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <deformables/PhysXDeformablePost.h>

#include <common/utilities/MemoryMacros.h>

#include "extensions/PxDeformableVolumeExt.h"
#include "extensions/PxDeformableSurfaceExt.h"
#include "extensions/PxDeformableSkinningExt.h"
#include "extensions/PxCudaHelpersExt.h"

using namespace pxr;
using namespace ::physx;
using namespace physx::Ext;
using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace cookingdataasync;

namespace
{

carb::Float3 sub(const carb::Float3& a, const carb::Float3& b)
{
    return carb::Float3{ a.x - b.x, a.y - b.y, a.z - b.z };
}

float length(const carb::Float3& d)
{
    return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

float area(const carb::Float3& a, const carb::Float3& b, const carb::Float3& c)
{
    carb::Float3 ab = sub(b, a);
    carb::Float3 ac = sub(c, a);
    carb::Float3 cross{ ab.y * ac.z - ab.z * ac.y, ab.z * ac.x - ab.x * ac.z, ab.x * ac.y - ab.y * ac.x };
    return 0.5f * length(cross);
}

bool setVolumeDeformableMass(PxDeformableVolume& deformableVolume, PxVec4* simMeshPositionInvMassH,
    const float bodyMass, const float* materialDensity)
{
    float mass;
    float density;
    float defaultDensitySi = 1000.0f; //Mass.cpp: parseCollisionShapeForMass

    density = materialDensity ? *materialDensity : 0.0f;
    mass = bodyMass;
    if (mass <= 0.0f)
    {
        // set default if neither deformable mass nor material density are specified:
        if (density <= 0.0f)
        {
            density = getScaledDensity(OmniPhysX::getInstance().getStage(), defaultDensitySi);
        }
    }

    const float maxInvMass = 50.f;
    const float lowerBoundMass = 1e-16f; // mass cannot be set to 0, but user-defined values are >= 0, so clamp it
                                          // reasonably close to zero
    if (mass > 0.0f)
    {
        PxDeformableVolumeExt::setMass(
            deformableVolume, fmaxf(mass, lowerBoundMass), maxInvMass, simMeshPositionInvMassH);
    }
    else
    {
        PxDeformableVolumeExt::updateMass(deformableVolume, density, maxInvMass, simMeshPositionInvMassH);
    }
    return true;
}

bool setSurfaceDeformableMass(PxDeformableSurface& deformableSurface, PxVec4* simMeshPositionInvMassH,
    const float bodyMass, const float* materialDensity, const float materialThickness)
{
    float mass;
    float density;
    float defaultDensitySi = 100.0f; // choose lower density than volume deformables or rigid bodies

    PxShape* shape = deformableSurface.getShape();
    if (!shape || shape->getGeometry().getType() != PxGeometryType::eTRIANGLEMESH)
    {
        return false;
    }
    const PxTriangleMeshGeometry& triangleMeshGeom = static_cast<const PxTriangleMeshGeometry&>(shape->getGeometry());
    uint32_t numSimMeshVertices = triangleMeshGeom.triangleMesh->getNbVertices();

    density = materialDensity ? *materialDensity : 0.0f;
    mass = bodyMass;
    if (mass <= 0.0f)
    {
        // set default if neither deformable mass nor material density are specified:
        if (density <= 0.0f)
        {
            density = getScaledDensity(OmniPhysX::getInstance().getStage(), defaultDensitySi);
        }
    }

    const float maxInvMass = 50.f;
    const float lowerBoundMass = 1e-16f; // mass cannot be set to 0, but user-defined values are >= 0, so clamp it
                                          // reasonably close to zero

    float vertexMassInv;
    if (mass > 0.0f)
    {
        vertexMassInv = numSimMeshVertices / mass;
    }
    else
    {
        // using material thickness
        float surfaceThickness = fmaxf(materialThickness, FLT_EPSILON); 

        //TODO fix up if PxTriangleMesh becomes scaleable...
        float totalArea = 0.0f;
        const carb::Float3* simMeshVertices = reinterpret_cast<const carb::Float3*>(triangleMeshGeom.triangleMesh->getVertices());
        const uint32_t* simMeshIndices = reinterpret_cast<const PxU32*>(triangleMeshGeom.triangleMesh->getTriangles());
        const uint32_t numSimMeshTriangles = triangleMeshGeom.triangleMesh->getNbTriangles();

        for (uint32_t i = 0; i < numSimMeshTriangles; ++i)
        {
            const uint32_t vtx0 = simMeshIndices[3*i];
            const uint32_t vtx1 = simMeshIndices[3*i + 1];
            const uint32_t vtx2 = simMeshIndices[3*i + 2];
            totalArea += area(simMeshVertices[vtx0], simMeshVertices[vtx1], simMeshVertices[vtx2]);
        }

        vertexMassInv = numSimMeshVertices / (totalArea * density * surfaceThickness);
    }

    for (uint32_t i = 0; i < numSimMeshVertices; ++i)
    {
        PxVec4& posInvMass = simMeshPositionInvMassH[i];
        posInvMass.w = vertexMassInv;
    }
    return true;
}

void deriveCollisionOffsets(float& outRestOffset, float& outContactOffset,
    const omni::physx::PhysXScene& scene, const PxGeometry& geometry, const float geometryScale,
    const float restOffset, const float contactOffset, const pxr::SdfPath reportPath)
{
    // TODO unify with UsdInterface.cpp, createShape()
    outRestOffset = restOffset;
    outContactOffset = contactOffset;

    if (contactOffset >= 0.0f)
    {
        if (contactOffset <= restOffset)
        {
            CARB_LOG_ERROR("Collision contact offset must be positive and greater then restOffset, prim: %s", reportPath.GetText());
            outContactOffset = contactOffset + restOffset + 1e-3f;
        }
    }
    else
    {
        PxBounds3 aabbLocalBounds;
        PxGeometryQuery::computeGeomBounds(aabbLocalBounds, geometry, PxTransform(PxIdentity));
        const PxVec3 extents = aabbLocalBounds.getDimensions() * geometryScale;

        const PxReal g = scene.getScene()->getGravity().magnitude();
        const PxReal dt = 1.0f / scene.getTimeStepsPerSeconds();
        //Make sure the lower bound is not exacly zero in case of zero gravity
        const PxReal dynamicLowerThreshold = 2.0f * dt * dt * PxMax(g, 1.0f);

        const PxReal minContactOffset = extents.minElement() * 0.02f;
        outContactOffset = fmaxf(dynamicLowerThreshold, minContactOffset);

        if (isfinite(restOffset) && restOffset > 0.0f)
        {
            outContactOffset += restOffset;
        }
    }

    if (!isfinite(restOffset))
    {
        outRestOffset = 0.0f;
    }
    else
    {
        if (restOffset > outContactOffset)
        {
            CARB_LOG_ERROR("Collision rest offset must be lesser then contact offset, prim: %s", reportPath.GetText());
            outRestOffset = 0.0f;
        }
    }
}

float deriveSelfCollisionFilterDistance(const float usdSelfCollisionFilterDistance, const float restOffset, const float contactOffset,
    float metersPerUnit, PhysXType type)
{
    const float selfCollisionFilterDistanceEps = 1.0e-5f * metersPerUnit;
    float minSelfCollisionFilterDistance = usdSelfCollisionFilterDistance;
    if (type == PhysXType::ePTDeformableVolume)
    {
        minSelfCollisionFilterDistance = 2.5f * contactOffset + selfCollisionFilterDistanceEps;
    }
    else if (type == PhysXType::ePTDeformableSurface)
    {
        //for PxFEMCloth we use the restOffset, but PxDeformableVolume uses contact offset
        minSelfCollisionFilterDistance = 2.0f * restOffset + selfCollisionFilterDistanceEps;
    }
    return fmaxf(usdSelfCollisionFilterDistance, minSelfCollisionFilterDistance);
}

size_t collectSkinMeshes(std::vector<UsdPrim>& skinMeshPrims,
                         std::vector<carb::Uint2>& skinMeshRanges,
                         std::vector<GfMatrix4f>& worldToSkinMeshTransforms,
                         const UsdStageWeakPtr stage,
                         const SdfPathVector& skinGeomPaths)
{
    size_t numAllSkinMeshPoints = 0;

    // histogram, transforms + references
    for (SdfPath skinGeomPath : skinGeomPaths)
    {
        UsdGeomPointBased skinGeom = UsdGeomPointBased::Get(stage, skinGeomPath);
        if (skinGeom)
        {
            GfMatrix4d skinGeomToWorld = skinGeom.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
            VtArray<GfVec3f> points;
            skinGeom.GetPointsAttr().Get(&points);
            if (points.size())
            {
                carb::Uint2 range = { uint32_t(numAllSkinMeshPoints), uint32_t(points.size()) };
                numAllSkinMeshPoints += uint32_t(points.size());

                skinMeshPrims.push_back(skinGeom.GetPrim());
                skinMeshRanges.push_back(range);
                worldToSkinMeshTransforms.push_back(GfMatrix4f(skinGeomToWorld.GetInverse()));
            }
        }
    }
    return numAllSkinMeshPoints;
}

void parseSkinBindPointsWorld(VtArray<GfVec3f>& allSkinMeshBindPointsWorld,
                              const UsdStageWeakPtr stage,
                              const SdfPathVector& skinGeomPaths,
                              const TfTokenVector& skinGeomBindPoseTokens,
                              const size_t numAllSkinMeshPoints)
{
    allSkinMeshBindPointsWorld.resize(numAllSkinMeshPoints);
    TfType dpType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);

    size_t offsetSkinMeshPoints = 0;
    for (size_t i = 0; i < skinGeomPaths.size(); ++i)
    {
        SdfPath skinGeomPath = skinGeomPaths[i];
        TfToken skinGeomBindPoseToken = skinGeomBindPoseTokens[i];
        UsdGeomPointBased skinGeom = UsdGeomPointBased::Get(stage, skinGeomPath);

        if (skinGeom)
        {
            VtArray<GfVec3f> skinMeshBindPointsLocal;
            bool hasBindPoseAPI = !skinGeomBindPoseToken.IsEmpty() &&
                                  skinGeom.GetPrim().HasAPI(dpType, skinGeomBindPoseToken);
            if (hasBindPoseAPI)
            {
                TfToken pointAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                    OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points, skinGeomBindPoseToken);

                skinGeom.GetPrim().GetAttribute(pointAttrName).Get(&skinMeshBindPointsLocal);
            }
            else
            {
                // When there is no bind pose, use points instead
                skinGeom.GetPointsAttr().Get(&skinMeshBindPointsLocal);
            }

            const GfMatrix4d skinGeomToWorld = skinGeom.ComputeLocalToWorldTransform(UsdTimeCode::Default());
            // transform to world space and add to allSkinMeshBindPointsWorld
            if (offsetSkinMeshPoints + skinMeshBindPointsLocal.size() <= allSkinMeshBindPointsWorld.size())
            {
                for (const GfVec3f& point : skinMeshBindPointsLocal)
                {
                    allSkinMeshBindPointsWorld[offsetSkinMeshPoints++] = skinGeomToWorld.Transform(point);
                }
            }
        }
    }
}

void parseSkinMeshPoints(VtArray<GfVec3f>& allSkinMeshPoints,
                         const std::vector<UsdPrim>& skinMeshPrims,
                         const size_t numAllSkinMeshPoints)
{
    allSkinMeshPoints.resize(numAllSkinMeshPoints);
    size_t offsetSkinPoints = 0;
    for (size_t i = 0; i < skinMeshPrims.size(); ++i)
    {
        UsdGeomPointBased skinMesh(skinMeshPrims[i]);
        if (skinMesh)
        {
            pxr::VtArray<pxr::GfVec3f> points;
            skinMesh.GetPointsAttr().Get(&points);

            if (offsetSkinPoints + points.size() <= allSkinMeshPoints.size())
            {
                for (const pxr::GfVec3f& point : points)
                {
                    allSkinMeshPoints[offsetSkinPoints++] = point;
                }
            }
        }
    }
}

void parseSimBindPoints(VtArray<GfVec3f>& simMeshBindPoints, const UsdPrim simMeshPrim, TfToken simMeshBindPoseToken,
                        const VtArray<GfVec3f>& simMeshPoints)
{
    TfType dpType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
    bool hasBindPoseAPI = !simMeshBindPoseToken.IsEmpty() && simMeshPrim.HasAPI(dpType, simMeshBindPoseToken);
    if (hasBindPoseAPI)
    {
        TfToken multipleApplyTemplate_points = OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points;
        TfToken pointAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(multipleApplyTemplate_points, simMeshBindPoseToken);
        simMeshPrim.GetAttribute(pointAttrName).Get(&simMeshBindPoints);
    }
    if (simMeshBindPoints.size() != simMeshPoints.size())
    {
        simMeshBindPoints = simMeshPoints;
    }
}

} // namespace

namespace omni
{
namespace physx
{

extern bool checkScenes();
extern bool isRenderable(const UsdPrim& prim);

bool PhysXUsdPhysicsInterface::updateDeformableBodyMass(const usdparser::AttachedStage& attachedStage, ObjectId objectId)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType type;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(type, objectId);
    if (!objectRecord || !objectRecord->mPtr || !objectRecord->mInternalPtr ||
        !(type == PhysXType::ePTDeformableSurface || type == PhysXType::ePTDeformableVolume))
        return false;

    InternalDeformableBody* intDeformableBody = (InternalDeformableBody*)objectRecord->mInternalPtr;

    float* matDensityPtr = nullptr;
    PxDeformableSurfaceMaterial* deformableSurfaceMaterial = nullptr;
    {
        PhysXType matType;
        InternalDeformableMaterial* intMat = nullptr;
        const InternalDatabase::Record* matRecord = db.getFullRecord(matType, intDeformableBody->mMaterialId);
        if (matRecord)
        {
            bool isConsistentMat = false;
            if (matType == PhysXType::ePTDeformableSurfaceMaterial)
            {
                isConsistentMat = (type == PhysXType::ePTDeformableSurface);
                deformableSurfaceMaterial = (PxDeformableSurfaceMaterial*)matRecord->mPtr;
            }
            else if (matType == PhysXType::ePTDeformableVolumeMaterial)
            {
                isConsistentMat = (type == PhysXType::ePTDeformableVolume);
            }

            if (!isConsistentMat)
            {
                CARB_LOG_ERROR("updateDeformableBodyMass: found inconsistent material type for %s: %s",
                    intDeformableBody->mBodyPrim.GetPath().GetText(),
                    matRecord->mPath.GetText());
                return false;
            }

            intMat = (InternalDeformableMaterial*)matRecord->mInternalPtr;
            if (intMat)
            {
                matDensityPtr = &intMat->mDensity;
            }
        }
    }

    PxDeformableBody* deformableBody = (PxDeformableBody*)objectRecord->mPtr;
    if (type == PhysXType::ePTDeformableSurface)
    {
        if (deformableSurfaceMaterial)
        {
            PxDeformableSurface* deformableSurface = (PxDeformableSurface*)deformableBody;
            if (!setSurfaceDeformableMass(*deformableSurface, intDeformableBody->mSimMeshPositionInvMassH,
                                          intDeformableBody->mBodyMass, matDensityPtr,
                                          deformableSurfaceMaterial->getThickness()))
            {
                CARB_LOG_ERROR("updateDeformableBodyMass: %s failed to update surface deformable mass.",
                               intDeformableBody->mBodyPrim.GetPath().GetText());
            }

            PxDeformableSurfaceDataFlags flags = PxDeformableSurfaceDataFlag::ePOSITION_INVMASS;
            PxDeformableSurfaceExt::copyToDevice(
                *deformableSurface, flags, intDeformableBody->mNumSimMeshVertices,
                intDeformableBody->mSimMeshPositionInvMassH, nullptr, nullptr,
                intDeformableBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
        }
    }
    else if (type == PhysXType::ePTDeformableVolume)
    {
        PxDeformableVolume* deformableVolume = (PxDeformableVolume*)deformableBody;
        if (!setVolumeDeformableMass(*deformableVolume, intDeformableBody->mSimMeshPositionInvMassH,
            intDeformableBody->mBodyMass, matDensityPtr))
        {
            CARB_LOG_ERROR("updateDeformableBodyMass: %s failed to update volume deformable mass.",
                intDeformableBody->mBodyPrim.GetPath().GetText());
        }

        PxDeformableVolumeDataFlags flags = PxDeformableVolumeDataFlag::eSIM_POSITION_INVMASS;
        PxDeformableVolumeExt::copyToDevice(*deformableVolume, flags, intDeformableBody->mSimMeshPositionInvMassH, nullptr,
            ((InternalVolumeDeformableBody*)intDeformableBody)->mCollMeshPositionInvMassH, nullptr,
            intDeformableBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
    }

    // wakeup actor
    const PxScene* scene = deformableBody->getScene();
    if (scene)
    {
        deformableBody->setWakeCounter(scene->getWakeCounterResetValue());
    }

    return true;
}

bool PhysXUsdPhysicsInterface::updateDeformableBodyPositions(const usdparser::AttachedStage& attachedStage, const ObjectId objectId)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);

    if (!objectFullRecord || !objectFullRecord->mPtr)
    {
        return true;
    }

    UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(objectFullRecord->mPath);

    GfMatrix4d transform = UsdGeomXform(prim).ComputeLocalToWorldTransform(UsdTimeCode::Default());

    VtArray<GfVec3f> points;
    UsdGeomPointBased simMesh(prim);
    simMesh.GetPointsAttr().Get(&points);

    InternalDeformableBody* internalDeformableBody = (InternalDeformableBody*)objectFullRecord->mInternalPtr;
    if (internalDeformableBody)
    {
        if (points.size() != internalDeformableBody->mNumSimMeshVertices)
        {
            CARB_LOG_WARN("Size of points of %s has changed - skipping update.", prim.GetPath().GetText());
            return true;
        }

        PxVec4* simPositionInvMass = internalDeformableBody->mSimMeshPositionInvMassH;
        for (unsigned int i = 0; i < internalDeformableBody->mNumSimMeshVertices; ++i)
        {
            GfVec3d localPoint = GfVec3d(points[i][0], points[i][1], points[i][2]);
            GfVec3d worldPoint = transform.Transform(localPoint);

            simPositionInvMass[i] = PxVec4((float)worldPoint[0], (float)worldPoint[1], (float)worldPoint[2], simPositionInvMass[i].w);
        }
    }

    if (internalType == ePTDeformableSurface)
    {
        PxDeformableSurface* deformableSurface = (PxDeformableSurface*)objectFullRecord->mPtr;     

        if (deformableSurface &&
            internalDeformableBody &&
            prim.HasAPI(UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI)))
        {
            PxDeformableSurfaceDataFlags flags = PxDeformableSurfaceDataFlags(0);
            flags.raise(PxDeformableSurfaceDataFlag::ePOSITION_INVMASS);

            PxDeformableSurfaceExt::copyToDevice(*deformableSurface, flags, internalDeformableBody->mNumSimMeshVertices,
                internalDeformableBody->mSimMeshPositionInvMassH, internalDeformableBody->mSimMeshVelocityH, nullptr,
                internalDeformableBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
        }
    }
    else if (internalType == ePTDeformableVolume)
    {
        PxDeformableVolume* deformableVolume = (PxDeformableVolume*)objectFullRecord->mPtr;
        InternalVolumeDeformableBody* internalVolumeDeformableBody = (InternalVolumeDeformableBody*)internalDeformableBody;

        if (deformableVolume &&
            internalVolumeDeformableBody &&
            prim.HasAPI(UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI)))
        {
            PxDeformableVolumeDataFlags flags = PxDeformableVolumeDataFlags(0);
            flags.raise(PxDeformableVolumeDataFlag::eSIM_POSITION_INVMASS);

            PxDeformableVolumeExt::copyToDevice(*deformableVolume, flags, internalVolumeDeformableBody->mSimMeshPositionInvMassH,
                internalVolumeDeformableBody->mSimMeshVelocityH, internalVolumeDeformableBody->mCollMeshPositionInvMassH,
                nullptr, internalVolumeDeformableBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
        }
    }

    return true;
}

bool PhysXUsdPhysicsInterface::updateDeformableBodyVelocities(const usdparser::AttachedStage& attachedStage, const ObjectId objectId)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);

    if (!objectFullRecord || !objectFullRecord->mPtr)
    {
        return true;
    }

    UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(objectFullRecord->mPath);

    VtArray<GfVec3f> velocities;
    UsdGeomPointBased simMesh(prim);
    simMesh.GetVelocitiesAttr().Get(&velocities);

    InternalDeformableBody* internalDeformableBody = (InternalDeformableBody*)objectFullRecord->mInternalPtr;
    if (internalDeformableBody)
    {
        if (velocities.size() != internalDeformableBody->mNumSimMeshVertices)
        {
            CARB_LOG_WARN("Size of velocities of %s has changed - skipping update.", prim.GetPath().GetText());
            return true;
        }

        PxVec4* simVelocity = internalDeformableBody->mSimMeshVelocityH;
        for (unsigned int i = 0; i < internalDeformableBody->mNumSimMeshVertices; ++i)
        {
            simVelocity[i] = PxVec4(velocities[i][0], velocities[i][1], velocities[i][2], simVelocity[i].w);
        }
    }

    if (internalType == ePTDeformableSurface)
    {
        PxDeformableSurface* deformableSurface = (PxDeformableSurface*)objectFullRecord->mPtr;

        if (deformableSurface &&
            internalDeformableBody &&
            prim.HasAPI(UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI)))
        {
            PxDeformableSurfaceDataFlags flags = PxDeformableSurfaceDataFlags(0);
            flags.raise(PxDeformableSurfaceDataFlag::eVELOCITY);

            PxDeformableSurfaceExt::copyToDevice(*deformableSurface, flags, internalDeformableBody->mNumSimMeshVertices,
                internalDeformableBody->mSimMeshPositionInvMassH, internalDeformableBody->mSimMeshVelocityH, nullptr,
                internalDeformableBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
        }
    }
    else if (internalType == ePTDeformableVolume)
    {
        PxDeformableVolume* deformableVolume = (PxDeformableVolume*)objectFullRecord->mPtr;
        InternalVolumeDeformableBody* internalVolumeDeformableBody = (InternalVolumeDeformableBody*)internalDeformableBody;

        if (deformableVolume &&
            internalVolumeDeformableBody &&
            prim.HasAPI(UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI)))
        {
            PxDeformableVolumeDataFlags flags = PxDeformableVolumeDataFlags(0);
            flags.raise(PxDeformableVolumeDataFlag::eSIM_VELOCITY);

            PxDeformableVolumeExt::copyToDevice(*deformableVolume, flags, internalVolumeDeformableBody->mSimMeshPositionInvMassH,
                internalVolumeDeformableBody->mSimMeshVelocityH, internalVolumeDeformableBody->mCollMeshPositionInvMassH,
                nullptr, internalVolumeDeformableBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
        }
    }

    return true;
}

bool PhysXUsdPhysicsInterface::updateDeformableRestOffset(const usdparser::AttachedStage& attachedStage, const ObjectId objectId, float value)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType type;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(type, objectId);
    if (!objectRecord || !objectRecord->mPtr || !objectRecord->mInternalPtr ||
        !(type == PhysXType::ePTDeformableSurface || type == PhysXType::ePTDeformableVolume))
        return false;

    PxDeformableBody* deformableBody = (PxDeformableBody*)objectRecord->mPtr;
    PxShape* shape = deformableBody->getShape();
    if (!shape)
    {
        return false;
    }
    float restOffset = shape->getRestOffset();
    float contactOffset = shape->getContactOffset();

    if (isfinite(value) && value < contactOffset)
    {
        shape->setRestOffset(value);
        restOffset = value;
    }
    else
    {
        CARB_LOG_ERROR("Collision rest offset must be lesser then contact offset, prim: %s", objectRecord->mPath.GetText());
        return true;
    }

    float metersPerUnit = float(pxr::UsdGeomGetStageMetersPerUnit(attachedStage.getStage()));
    float selfCollFilterDist = deformableBody->getSelfCollisionFilterDistance();
    selfCollFilterDist = deriveSelfCollisionFilterDistance(selfCollFilterDist, restOffset, contactOffset, metersPerUnit, type);
    deformableBody->setSelfCollisionFilterDistance(selfCollFilterDist);

    //wakeup actor
    PxScene* scene = deformableBody->getScene();
    if (scene)
    {
        deformableBody->setWakeCounter(scene->getWakeCounterResetValue());
    }
    return true;
}

bool PhysXUsdPhysicsInterface::updateDeformableContactOffset(const usdparser::AttachedStage& attachedStage, const ObjectId objectId, float value)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType type;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(type, objectId);
    if (!objectRecord || !objectRecord->mPtr || !objectRecord->mInternalPtr ||
        !(type == PhysXType::ePTDeformableSurface || type == PhysXType::ePTDeformableVolume))
        return false;

    PxDeformableBody* deformableBody = (PxDeformableSurface*)objectRecord->mPtr;
    PxShape* shape = deformableBody->getShape();
    if (!shape)
    {
        return false;
    }
    float restOffset = shape->getRestOffset();
    float contactOffset = shape->getContactOffset();

    if (value >= 0.0f && value > shape->getRestOffset())
    {
        shape->setContactOffset(value);
        contactOffset = value;
    }
    else
    {
        CARB_LOG_ERROR("Collision contact offset must be positive and greater then restOffset, prim: %s", objectRecord->mPath.GetText());
        return true;
    }

    float metersPerUnit = float(pxr::UsdGeomGetStageMetersPerUnit(attachedStage.getStage()));
    float selfCollFilterDist = deformableBody->getSelfCollisionFilterDistance();
    selfCollFilterDist = deriveSelfCollisionFilterDistance(selfCollFilterDist, restOffset, contactOffset, metersPerUnit, type);
    deformableBody->setSelfCollisionFilterDistance(selfCollFilterDist);

    // wakeup actor
    PxScene* scene = deformableBody->getScene();
    if (scene)
    {
        deformableBody->setWakeCounter(scene->getWakeCounterResetValue());
    }
    return true;
}

bool PhysXUsdPhysicsInterface::updateDeformableSelfCollisionFilterDistance(const usdparser::AttachedStage& attachedStage, const ObjectId objectId, float value)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType type;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(type, objectId);
    if (!objectRecord || !objectRecord->mPtr || !objectRecord->mInternalPtr ||
        !(type == PhysXType::ePTDeformableSurface || type == PhysXType::ePTDeformableVolume))
        return false;

    PxDeformableBody* deformableBody = (PxDeformableBody*)objectRecord->mPtr;
    PxShape* shape = deformableBody->getShape();
    if (!shape)
    {
        return false;
    }
    float restOffset = shape->getRestOffset();
    float contactOffset = shape->getContactOffset();
    float metersPerUnit = float(pxr::UsdGeomGetStageMetersPerUnit(attachedStage.getStage()));
    float selfCollisionFilterDistance = deriveSelfCollisionFilterDistance(value, restOffset, contactOffset, metersPerUnit, type);
    deformableBody->setSelfCollisionFilterDistance(selfCollisionFilterDistance);

    // wakeup actor
    PxScene* scene = deformableBody->getScene();
    if (scene)
    {
        deformableBody->setWakeCounter(scene->getWakeCounterResetValue());
    }
    return true;
}

ObjectId getDeformableMaterialId(const PhysXScene& scene, const PhysxDeformableBodyDesc& deformableDesc)
{
    ObjectId materialId = deformableDesc.simMeshMaterial;

    if (materialId == kInvalidObjectId)
    {
        // Scene might have a default material with InternalDeformableMaterial wrapper and valid ObjectId
        PxFEMMaterial* material = nullptr;
        if (deformableDesc.type == eVolumeDeformableBody)
        {
            material = scene.getDefaultVolumeDeformableMaterial();
        }
        else if (deformableDesc.type == eSurfaceDeformableBody)
        {
            material = scene.getDefaultSurfaceDeformableMaterial();
        }

        if (material && material->userData)
        {
            materialId = (size_t)material->userData;
        }
    }
    return materialId;
}

PxDeformableVolumeMaterial* getVolumeDeformableBodyMaterial(InternalDeformableMaterial*& internalMaterial,
    const PhysXScene& scene, ObjectId materialId)
{
    internalMaterial = getInternalPtr<InternalDeformableMaterial>(PhysXType::ePTDeformableVolumeMaterial, materialId);
    PxDeformableVolumeMaterial* material = getPtr<PxDeformableVolumeMaterial>(PhysXType::ePTDeformableVolumeMaterial, materialId);
    if (!material)
    {
        // Default material migth not have InternalDeformableMaterial wrapper and valid ObjectId
        material = scene.getDefaultVolumeDeformableMaterial();
    }
    return material;
}

PxDeformableSurfaceMaterial* getSurfaceDeformableBodyMaterial(InternalDeformableMaterial*& internalMaterial,
    const PhysXScene& scene, ObjectId materialId)
{
    internalMaterial = getInternalPtr<InternalDeformableMaterial>(PhysXType::ePTDeformableSurfaceMaterial, materialId);
    PxDeformableSurfaceMaterial* material = getPtr<PxDeformableSurfaceMaterial>(PhysXType::ePTDeformableSurfaceMaterial, materialId);
    if (!material)
    {
        // Default material migth not have InternalDeformableMaterial wrapper and valid ObjectId
        material = scene.getDefaultSurfaceDeformableMaterial();
    }
    return material;
}

ObjectId PhysXUsdPhysicsInterface::createVolumeDeformableBody(usdparser::AttachedStage& attachedStage,
    const SdfPath& path, PhysxVolumeDeformableBodyDesc const& desc)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PhysXScene* scene = physxSetup.getPhysXScene(desc.sceneId);
    if (!scene || !scene->isFullGpuPipelineAvailable())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            ErrorCode::eError,
            "Deformable Body feature is only supported on GPU. Please enable GPU dynamics flag in Property/Scene of physics scene!");
        return kInvalidObjectId;
    }

    if (!checkScenes())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            ErrorCode::eError, "No physics scene created, please add physics scene into stage!");
        return kInvalidObjectId;
    }

    if (desc.kinematicBody)
    {
        CARB_LOG_WARN("Currently kinematic deformable bodies are not supported.");
        return kInvalidObjectId;
    }

    PxPhysics* physics = physxSetup.getPhysics();
    PxCudaContextManager* cudaContextManager = scene->getScene()->getCudaContextManager();
    if (!cudaContextManager)
    {
        CARB_LOG_WARN("PhysX Deformable Body creation failed due to missing PxCudaContextManager.");
        return kInvalidObjectId;
    }

    CookingDataAsync* cookingDataAsync = physxSetup.getCookingDataAsync();
    if (!cookingDataAsync)
    {
        return kInvalidObjectId;
    }

    UsdPrim bodyPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(path);
    UsdPrim simMeshPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(desc.simMeshPath);
    UsdPrim collMeshPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(desc.collisionMeshPath);
    if (!bodyPrim || !simMeshPrim.IsA<UsdGeomTetMesh>() || !collMeshPrim.IsA<UsdGeomTetMesh>())
    {
        CARB_LOG_WARN("PhysX Deformable Body creation failed due to invalid deformable prims.");
        return kInvalidObjectId;
    }

    GfMatrix4d simMeshToWorld = pxr::UsdGeomMesh(simMeshPrim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
    GfMatrix4d worldToSimMesh = simMeshToWorld.GetInverse();

    GfMatrix4d worldToCollMesh = worldToSimMesh;
    if (collMeshPrim != simMeshPrim)
    {
        GfMatrix4d collMeshToWorld = pxr::UsdGeomMesh(collMeshPrim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
        worldToCollMesh = collMeshToWorld.GetInverse();
    }

    // sync tet mesh generation
    if (desc.hasAutoAPI)
    {
        cookingDataAsync->cookVolumeDeformableBody(desc, bodyPrim, false);
    }

    // get cooked deformable volume mesh data (in 'simMesh' space)
    PxDefaultMemoryOutputStream outData;
    bool deformableVolumeMeshCooked =
        cookingDataAsync->cookDeformableVolumeMesh(outData, desc, bodyPrim, false);

    if (!deformableVolumeMeshCooked)
    {
        CARB_LOG_WARN("Failed to cook PxDeformableVolumeMesh! Prim(%s)\n", path.GetText());
        return kInvalidObjectId;
    }

    // surface triangles are mandatory on the collision mesh, because we need to be able to deterministically
    // reference surface triangles for attachments, also useful for picking.
    VtArray<GfVec3i> collMeshSurfaceTriangles;
    UsdGeomTetMesh(collMeshPrim).GetSurfaceFaceVertexIndicesAttr().Get(&collMeshSurfaceTriangles);
    if (collMeshSurfaceTriangles.size() == 0)
    {
        return kInvalidObjectId;
    }

    std::vector<uint32_t> collMeshSurfaceTriToTetMap;
    PxDefaultMemoryInputData inData(outData.getData(), outData.getSize());
    PxDeformableVolumeMesh* deformableVolumeMesh = cookingDataAsync->createDeformableVolumeMesh(collMeshSurfaceTriToTetMap, inData);
    if (!deformableVolumeMesh || collMeshSurfaceTriangles.size() != collMeshSurfaceTriToTetMap.size())
    {
        CARB_LOG_WARN("Failed to create PxDeformableVolumeMesh from cooked data! Prim(%s)\n", path.GetText());
        return kInvalidObjectId;
    }

    // create internal deformable body, and store/copy some data
    InternalVolumeDeformableBody* internalBody = ICE_NEW(InternalVolumeDeformableBody)();
    if (!internalBody)
    {
        return kInvalidObjectId;
    }

    internalBody->mPhysXScene = scene;
    if (!desc.bodyEnabled)
    {
        ObjectId objectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
            ePTDeformableVolume, nullptr, internalBody, bodyPrim.GetPrimPath());

        scene->getInternalScene()->mVolumeDeformableBodies.push_back(internalBody);
        return objectId;
    }

    internalBody->mBodyPrim = bodyPrim;
    internalBody->mSimMeshPrim = simMeshPrim;
    internalBody->mCollMeshPrim = collMeshPrim;
    internalBody->mWorldToSimMesh = GfMatrix4f(worldToSimMesh);
    internalBody->mWorldToCollMesh = GfMatrix4f(worldToCollMesh);
    internalBody->mIsKinematic = desc.kinematicBody;
    internalBody->mBodyMass = desc.mass;

    // parse mesh data
    VtArray<GfVec3f> simMeshPoints;
    VtArray<GfVec3f> simMeshBindPoints;
    VtArray<GfVec3f> simMeshVelocities;
    VtArray<GfVec3f> collMeshPoints;
    VtArray<GfVec3f> allSkinMeshPoints;
    VtArray<GfVec3f> allSkinMeshBindPointsWorld;
    size_t numAllSkinMeshPoints;

    {
        UsdGeomPointBased simGeom(simMeshPrim);
        simGeom.GetPointsAttr().Get(&simMeshPoints);
        simGeom.GetVelocitiesAttr().Get(&simMeshVelocities);
        if (simMeshPrim != collMeshPrim)
        {
            UsdGeomPointBased collGeom(collMeshPrim);
            collGeom.GetPointsAttr().Get(&collMeshPoints);
        }
    }

    parseSimBindPoints(simMeshBindPoints, simMeshPrim, desc.simMeshBindPoseToken, simMeshPoints);

    pxr::GfMatrix4d cookingToWorld;
    pxr::GfMatrix4d simMeshToCooking;
    double cookingToWorldScale;
    cookingDataAsync->computeDeformableCookingTransform(&simMeshToCooking, &cookingToWorld, &cookingToWorldScale,
                                                        simMeshToWorld, simMeshBindPoints);
    {
        //simMeshToCooking output excludes world scale, so adding it here
        GfMatrix4d s;
        s.SetScale(cookingToWorldScale);
        simMeshToCooking = simMeshToCooking * s;
    }

    numAllSkinMeshPoints = collectSkinMeshes(internalBody->mSkinMeshPrims, internalBody->mSkinMeshRanges,
                                             internalBody->mWorldToSkinMeshTransforms, bodyPrim.GetStage(), desc.skinGeomPaths);

    parseSkinMeshPoints(allSkinMeshPoints, internalBody->mSkinMeshPrims, numAllSkinMeshPoints);

    parseSkinBindPointsWorld(allSkinMeshBindPointsWorld, bodyPrim.GetStage(), desc.skinGeomPaths,
                             desc.skinGeomBindPoseTokens, numAllSkinMeshPoints);

    PxTetrahedronMesh* collisionTetMesh = deformableVolumeMesh->getCollisionMesh();
    PxTetrahedronMesh* simulationTetMesh = deformableVolumeMesh->getSimulationMesh();
    CARB_ASSERT(collisionTetMesh);
    CARB_ASSERT(simulationTetMesh);

    PxTransform pxCookingToWorld;
    PxVec3 pxCookingToWorldScaleDir; //should be identity scale
    toPhysX(pxCookingToWorld, pxCookingToWorldScaleDir, cookingToWorld);
    PxReal pxCookingToWorldScale = PxReal(cookingToWorldScale);

    internalBody->mNumSimMeshVertices = simulationTetMesh->getNbVertices();
    internalBody->mNumCollMeshVertices = collisionTetMesh->getNbVertices();
    internalBody->mNumSkinMeshVertices = uint32_t(numAllSkinMeshPoints);

    internalBody->mCollMeshSurfaceTriangles.resize(collMeshSurfaceTriangles.size());
    std::memcpy(internalBody->mCollMeshSurfaceTriangles.data(), collMeshSurfaceTriangles.data(),
                internalBody->mCollMeshSurfaceTriangles.size() * sizeof(carb::Uint3));

    for (const GfVec3i& tri : collMeshSurfaceTriangles)
    {
        for (uint32_t i = 0; i < 3; ++i)
        {
            if (tri[i] >= (int)internalBody->mNumCollMeshVertices)
            {
                CARB_LOG_WARN("createVolumeDeformableBody(): Each surface face vertex index of collision mesh should be smaller than mNumCollMeshVertices!");
                ICE_FREE(internalBody);
                return kInvalidObjectId;
            }
        }
    }

    internalBody->mCollMeshSurfaceTriToTetMap.swap(collMeshSurfaceTriToTetMap);

    // Get assigned material
    ObjectId materialId = getDeformableMaterialId(*scene, desc);
    InternalDeformableMaterial* internalMaterial;
    PxDeformableVolumeMaterial* material = getVolumeDeformableBodyMaterial(internalMaterial, *scene, materialId);
    if (!material)
    {
        CARB_LOG_WARN("Failed to aquire material for volume deformable! Prim(%s)\n", path.GetText());
        ICE_FREE(internalBody);
        return kInvalidObjectId;
    }
    internalBody->mMaterialId = materialId;

    // Create physx deformable volume
    PxDeformableVolume* deformableVolume = physics->createDeformableVolume(*cudaContextManager);
    if (!deformableVolume)
    {
        CARB_LOG_WARN("Failed to create PxDeformableVolume! Prim(%s)\n", path.GetText());
        ICE_FREE(internalBody);
        return kInvalidObjectId;
    }

    PxShapeFlags shapeFlags = PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;

    if (OmniPhysX::getInstance().isDebugVisualizationEnabled())
        shapeFlags |= PxShapeFlag::eVISUALIZATION;

    PxTetrahedronMeshGeometry geometry(collisionTetMesh);
    PxShape* shape = physxSetup.getPhysics()->createShape(geometry, &material, 1, true, shapeFlags);
    if (shape)
    {
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
        PxFilterData fd;
        convertCollisionGroupToPxFilterData(collisionGroup, fd);
        shape->setSimulationFilterData(fd);

        PxReal restOffset;
        PxReal contactOffset;
        deriveCollisionOffsets(restOffset, contactOffset,
            *scene, geometry, pxCookingToWorldScale, desc.restOffset, desc.contactOffset, path);
        shape->setContactOffset(contactOffset);
        shape->setRestOffset(restOffset);

        deformableVolume->attachShape(*shape);
        shape->release();
    }

    deformableVolume->attachSimulationMesh(*simulationTetMesh, *deformableVolumeMesh->getDeformableVolumeAuxData());
    scene->getScene()->addActor(*deformableVolume);

    PxVec4* collMeshRestPositionH = nullptr;
    PxDeformableVolumeExt::allocateAndInitializeHostMirror(
        *deformableVolume, cudaContextManager, internalBody->mSimMeshPositionInvMassH,
        internalBody->mSimMeshVelocityH, internalBody->mCollMeshPositionInvMassH,
        collMeshRestPositionH);

    {
        float metersPerUnit = float(pxr::UsdGeomGetStageMetersPerUnit(attachedStage.getStage()));
        float contactOffset = deformableVolume->getShape()->getContactOffset();
        float restOffset = deformableVolume->getShape()->getRestOffset();
        float selfCollisionFilterDistance = deriveSelfCollisionFilterDistance(desc.selfCollisionFilterDistance,
            restOffset, contactOffset, metersPerUnit, PhysXType::ePTDeformableVolume);
        deformableVolume->setSelfCollisionFilterDistance(selfCollisionFilterDistance);
        deformableVolume->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, !desc.selfCollision);
    }
    {
        const bool disableSleeping = OmniPhysX::getInstance().getCachedSettings().disableSleeping;
        deformableVolume->setSleepThreshold(disableSleeping ? 0.0f : desc.sleepThreshold);
        deformableVolume->setSettlingThreshold(disableSleeping ? 0.0f : desc.settlingThreshold);
        deformableVolume->setSettlingDamping(desc.settlingDamping);
    }
    deformableVolume->setDeformableBodyFlag(PxDeformableBodyFlag::eKINEMATIC, desc.kinematicBody);
    deformableVolume->setDeformableBodyFlag(PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD, desc.enableSpeculativeCCD);
    deformableVolume->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, desc.disableGravity);

    deformableVolume->setLinearDamping(desc.linearDamping);
    deformableVolume->setSolverIterationCounts(scene->getInternalScene()->clampPosIterationCount(desc.solverPositionIterationCount));

    // startsAsleep
    if (desc.startsAsleep)
    {
        // velocities are initialized to zero below
        // TODO implement putToSleep in SDK
        deformableVolume->setWakeCounter(0.0f);
    }

    const float maxLinearVelocity = fminf(fmaxf(desc.maxLinearVelocity, 0.0f), SQRT_FLT_MAX);
    deformableVolume->setMaxLinearVelocity(maxLinearVelocity);

    deformableVolume->setMaxDepenetrationVelocity(desc.maxDepenetrationVelocity);

    PxDeformableVolumeExt::transform(*deformableVolume, pxCookingToWorld, pxCookingToWorldScale,
        internalBody->mSimMeshPositionInvMassH, internalBody->mSimMeshVelocityH,
        internalBody->mCollMeshPositionInvMassH,
        collMeshRestPositionH);

    if (!setVolumeDeformableMass(*deformableVolume, internalBody->mSimMeshPositionInvMassH,
        internalBody->mBodyMass, internalMaterial ? &internalMaterial->mDensity : nullptr))
    {
        CARB_LOG_WARN("Failed to aquire mass/density for volume deformable body! Prim(%s)\n", path.GetText());
        ICE_FREE(internalBody);
        PX_PINNED_HOST_FREE(cudaContextManager, collMeshRestPositionH);
        return kInvalidObjectId;
    }

    //set inv mass of collision mesh vertices to 1.0
    {
        PxVec4* positionInvMass = internalBody->mCollMeshPositionInvMassH;
        const PxReal invMass = 1.0f; // does not do anything except it would lock if == 0.0f
        for (PxU32 i = 0; i < internalBody->mNumCollMeshVertices; ++i)
        {
            positionInvMass[i].w = invMass;
        }
    }

    for (PxU32 i = 0; i < internalBody->mNumSimMeshVertices; ++i)
    {
        GfVec3f simPosition = simMeshToWorld.Transform(simMeshPoints[i]);
        internalBody->mSimMeshPositionInvMassH[i] = PxVec4(toPhysX(simPosition), internalBody->mSimMeshPositionInvMassH[i].w);
    }

    //TODO fix. Velocities should be applied before PxDeformableVolumeExt::transform, and treated correctly according to
    //local transform or world transform depending on configuration
    if (!desc.startsAsleep && simMeshVelocities.size() > 0 &&
        simMeshVelocities.size() == internalBody->mNumSimMeshVertices)
    {
        copyBuffer(internalBody->mSimMeshVelocityH, reinterpret_cast<const carb::Float3*>(simMeshVelocities.data()),
            internalBody->mNumSimMeshVertices);
    }
    else
    {
        for (uint32_t i = 0; i < internalBody->mNumSimMeshVertices; ++i)
        {
            internalBody->mSimMeshVelocityH[i].x = 0.0f;
            internalBody->mSimMeshVelocityH[i].y = 0.0f;
            internalBody->mSimMeshVelocityH[i].z = 0.0f;
        }
    }

    PxDeformableVolumeExt::copyToDevice(*deformableVolume, PxDeformableVolumeDataFlag::eALL,
        internalBody->mSimMeshPositionInvMassH, internalBody->mSimMeshVelocityH,
        internalBody->mCollMeshPositionInvMassH, collMeshRestPositionH,
        internalBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());

    internalBody->mDeformableVolume = deformableVolume;
    internalBody->mDeformableVolumeMesh = deformableVolumeMesh;

    // Store initial properties for reset of deformables created during simulation
    copyBuffer(internalBody->mSimMeshPointsSaveRestoreBuf, simMeshPoints.data(), uint32_t(simMeshPoints.size()));
    copyBuffer(internalBody->mSimMeshVelocitiesSaveRestoreBuf, simMeshVelocities.data(), uint32_t(simMeshVelocities.size()));
    copyBuffer(internalBody->mCollMeshPointsSaveRestoreBuf, collMeshPoints.data(), uint32_t(collMeshPoints.size()));
    copyBuffer(internalBody->mAllSkinMeshPointsSaveRestoreBuf, allSkinMeshPoints.data(), uint32_t(allSkinMeshPoints.size()));

    UsdGeomBoundable::ComputeExtentFromPlugins(UsdGeomBoundable(collMeshPrim), UsdTimeCode::Default(),
        &internalBody->mCollMeshExtentSaveRestoreBuf);

    ObjectId objectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
        ePTDeformableVolume, deformableVolume, internalBody, bodyPrim.GetPrimPath());

    deformableVolume->userData = (void*)objectId;

    if (internalBody->mSimMeshPrim != internalBody->mBodyPrim)
    {
        //registering sub objects that need change handling 
        ObjectId simulationId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
            ePTDeformableVolume, deformableVolume, internalBody, desc.simMeshPath);
        attachedStage.getObjectDatabase()->findOrCreateEntry(desc.simMeshPath, eVolumeDeformableBody, simulationId);
    }
    if (internalBody->mCollMeshPrim != internalBody->mSimMeshPrim)
    {
        //registering sub objects that need change handling 
        ObjectId collisionId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
            ePTDeformableVolume, deformableVolume, internalBody, desc.collisionMeshPath);
        attachedStage.getObjectDatabase()->findOrCreateEntry(desc.collisionMeshPath, eVolumeDeformableBody, collisionId);
    }

    if (internalMaterial)
    {
        internalMaterial->addDeformableId(objectId);
    }

    // TODO check with Alain whether this is reasonable
    PxScopedCudaLock _lock(*cudaContextManager);
    PxCudaContext* cudaCtx = cudaContextManager->getCudaContext();
    cudaCtx->streamSynchronize(internalBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
    PX_PINNED_HOST_FREE(cudaContextManager, collMeshRestPositionH);

    // Skinning mesh setup
    if (numAllSkinMeshPoints > 0)
    {
        PxTetrahedronMesh& simulationMesh = *deformableVolume->getSimulationMesh();
        PxU32 nbTetrahedra = simulationMesh.getNbTetrahedrons();
        bool uses16bit = simulationMesh.getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;
        const PxU32 numTetIndices = 4 * nbTetrahedra;

        internalBody->mSimMeshTetIndicesD = PxCudaHelpersExt::allocDeviceBuffer<PxU32>(*cudaContextManager, numTetIndices);

        internalBody->mAllSkinnedVerticesH = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec3>(*cudaContextManager, numAllSkinMeshPoints);
        internalBody->mAllSkinnedVerticesD = PxCudaHelpersExt::allocDeviceBuffer<PxVec3>(*cudaContextManager, numAllSkinMeshPoints);
        if (internalBody->mAllSkinnedVerticesH && internalBody->mAllSkinnedVerticesD)
        {
            std::memcpy(internalBody->mAllSkinnedVerticesH, allSkinMeshBindPointsWorld.data(), sizeof(PxVec3)*numAllSkinMeshPoints);
            PxCudaHelpersExt::copyHToD(*cudaContextManager, internalBody->mAllSkinnedVerticesD, internalBody->mAllSkinnedVerticesH, numAllSkinMeshPoints);
        }

        PxTetrahedronMeshEmbeddingInfo* skinningEmbeddingInfoH = PxCudaHelpersExt::allocPinnedHostBuffer<PxTetrahedronMeshEmbeddingInfo>(*cudaContextManager, numAllSkinMeshPoints);
        internalBody->mSkinningEmbeddingInfoD = PxCudaHelpersExt::allocDeviceBuffer<PxTetrahedronMeshEmbeddingInfo>(*cudaContextManager, numAllSkinMeshPoints);

        PxArray<PxU32> simMeshTetIndices(numTetIndices);
        if (uses16bit)
        {
            const PxU16* tetIndices = reinterpret_cast<const PxU16*>(simulationMesh.getTetrahedrons());
            for (PxU32 i = 0; i < simMeshTetIndices.size(); ++i)
                simMeshTetIndices[i] = tetIndices[i];
        }
        else
        {
            const PxU32* tetIndices = reinterpret_cast<const PxU32*>(simulationMesh.getTetrahedrons());
            for (PxU32 i = 0; i < simMeshTetIndices.size(); ++i)
                simMeshTetIndices[i] = tetIndices[i];
        }

        // For setting up skinning data, we use the bind pose of the sim mesh and all skin meshes
        // in the cooking space with world scale to preserve distances.
        PxArray<PxVec3> guideVertices((uint32_t)simMeshBindPoints.size());
        for (uint32_t i = 0; i < guideVertices.size(); ++i)
        {
            const GfVec3f point = simMeshToCooking.Transform(simMeshBindPoints[i]);
            guideVertices[i] = PxVec3(point[0], point[1], point[2]);
        }

        GfMatrix4d worldToCooking = cookingToWorld.GetInverse();
        PxArray<PxVec3> embeddedVertices((uint32_t)allSkinMeshBindPointsWorld.size());
        for (uint32_t i = 0; i < embeddedVertices.size(); ++i)
        {
            const GfVec3f point = worldToCooking.Transform(allSkinMeshBindPointsWorld[i]);
            embeddedVertices[i] = PxVec3(point[0], point[1], point[2]);
        }

        PxDeformableSkinningExt::initializeInterpolatedVertices(
            skinningEmbeddingInfoH, guideVertices.begin(), simMeshTetIndices.begin(), nbTetrahedra,
            embeddedVertices.begin(), (PxU32)embeddedVertices.size());

        PxCudaHelpersExt::copyHToD(*cudaContextManager, internalBody->mSimMeshTetIndicesD, simMeshTetIndices.begin(), numTetIndices);
        PxCudaHelpersExt::copyHToD(*cudaContextManager, internalBody->mAllSkinnedVerticesD, internalBody->mAllSkinnedVerticesH, numAllSkinMeshPoints);
        PxCudaHelpersExt::copyHToD(*cudaContextManager, internalBody->mSkinningEmbeddingInfoD, skinningEmbeddingInfoH, numAllSkinMeshPoints);

        if (skinningEmbeddingInfoH)
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, skinningEmbeddingInfoH);

        // Add skinning data after the volume deformable body is created
        deformables::VolumeDeformableSkinningData skinningData;
        skinningData.mDeformableVolume = deformableVolume;
        skinningData.mGuideTetrahedraD = internalBody->mSimMeshTetIndicesD;
        skinningData.mSkinningEmbeddingInfoD = internalBody->mSkinningEmbeddingInfoD;
        skinningData.mAllSkinnedVerticesD = internalBody->mAllSkinnedVerticesD;
        skinningData.mNumSkinnedVertices = internalBody->mNumSkinMeshVertices;
        scene->getInternalScene()->mVolumeDeformablePostSolveCallback->addVolumeDeformableSkinningData(skinningData);
    }

    // add internal volume deformable body to internal scene
    scene->getInternalScene()->mVolumeDeformableBodies.push_back(internalBody);
    return objectId;
}

ObjectId PhysXUsdPhysicsInterface::createSurfaceDeformableBody(usdparser::AttachedStage& attachedStage,
    const SdfPath& path, PhysxSurfaceDeformableBodyDesc const& desc)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PhysXScene* scene = physxSetup.getPhysXScene(desc.sceneId);
    if (!scene || !scene->isFullGpuPipelineAvailable())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            ErrorCode::eError,
            "Deformable Body feature is only supported on GPU. Please enable GPU dynamics flag in Property/Scene of physics scene!");
        return kInvalidObjectId;
    }

    if (!checkScenes())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            ErrorCode::eError, "No physics scene created, please add physics scene into stage!");
        return kInvalidObjectId;
    }

    if (desc.kinematicBody)
    {
        CARB_LOG_WARN("Currently kinematic deformable bodies are not supported.");
        return kInvalidObjectId;
    }

    PxPhysics* physics = physxSetup.getPhysics();
    PxCudaContextManager* cudaContextManager = scene->getScene()->getCudaContextManager();
    if (!cudaContextManager)
    {
        CARB_LOG_WARN("PhysX Deformable Body creation failed due to missing PxCudaContextManager.");
        return kInvalidObjectId;
    }

    CookingDataAsync* cookingDataAsync = physxSetup.getCookingDataAsync();
    if (!cookingDataAsync)
    {
        return kInvalidObjectId;
    }

    UsdPrim bodyPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(path);
    UsdPrim simMeshPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(desc.simMeshPath);
    if (desc.collisionMeshPath != desc.simMeshPath)
    {
        CARB_LOG_WARN("No support for PhysX surface deformables with separate collision meshes. %s",
            desc.collisionMeshPath.GetText());
        return kInvalidObjectId;
    }

    UsdGeomMesh simMesh(simMeshPrim);
    if (!bodyPrim || !simMesh)
    {
        CARB_LOG_WARN("PhysX Deformable Body creation failed due to invalid deformable prims.");
        return kInvalidObjectId;
    }

    TfType simType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
    if (!simMeshPrim.HasAPI(simType))
    {
        CARB_LOG_WARN("PhysX Deformable Body creation failed due to missing SurfaceDeformableSimAPI on simulation mesh, %s.",
                      simMeshPrim.GetPath().GetText());
        return kInvalidObjectId;
    }

    GfMatrix4d simMeshToWorld = pxr::UsdGeomMesh(simMeshPrim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
    GfMatrix4d worldToSimMesh = simMeshToWorld.GetInverse();

    // sync mesh generation
    if (desc.hasAutoAPI)
    {
        cookingDataAsync->cookSurfaceDeformableBody(desc, bodyPrim, false);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: load cooked triangle mesh
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // create internal deformable surface, and store/copy some data
    InternalSurfaceDeformableBody* internalBody = ICE_NEW(InternalSurfaceDeformableBody)();
    if (!internalBody)
    {
        return kInvalidObjectId;
    }

    internalBody->mPhysXScene = scene;
    if (!desc.bodyEnabled)
    {
        ObjectId objectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
            ePTDeformableSurface, nullptr, internalBody, bodyPrim.GetPrimPath());
        scene->getInternalScene()->mSurfaceDeformableBodies.push_back(internalBody);
        return objectId;
    }

    internalBody->mBodyPrim = bodyPrim;
    internalBody->mSimMeshPrim = simMeshPrim;
    internalBody->mWorldToSimMesh = GfMatrix4f(worldToSimMesh);
    internalBody->mIsKinematic = desc.kinematicBody;
    internalBody->mBodyMass = desc.mass;

    // parse mesh data
    VtArray<GfVec3f> simMeshPoints;
    VtArray<GfVec3f> simMeshBindPoints;
    VtArray<GfVec3f> simMeshVelocities;
    VtArray<int32_t> simMeshIndices;
    VtArray<GfVec3f> simMeshRestShapePoints;
    VtArray<GfVec3i> simMeshRestTriVtxIndices;

    VtArray<GfVec3f> allSkinMeshPoints;
    VtArray<GfVec3f> allSkinMeshBindPointsWorld;
    size_t numAllSkinMeshPoints;

    {
        UsdGeomPointBased simGeom(simMeshPrim);
        simGeom.GetPointsAttr().Get(&simMeshPoints);
        simGeom.GetVelocitiesAttr().Get(&simMeshVelocities);
    }

    parseSimBindPoints(simMeshBindPoints, simMeshPrim, desc.simMeshBindPoseToken, simMeshPoints);

    simMesh.GetFaceVertexIndicesAttr().Get(&simMeshIndices);

    // Read rest shape
    {
        UsdAttribute restShapePointsAttr = simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints);
        restShapePointsAttr.Get(&simMeshRestShapePoints);
        UsdAttribute restTriVtxIndicesAttr = simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTriVtxIndices);
        restTriVtxIndicesAttr.Get(&simMeshRestTriVtxIndices);

        bool mismatch = simMeshRestShapePoints.size() != simMeshPoints.size() ||
                        simMeshRestTriVtxIndices.size() * 3 != simMeshIndices.size() ||
                        std::memcmp(simMeshRestTriVtxIndices.data(), simMeshIndices.data(),
                                    sizeof(int32_t) * simMeshIndices.size()) != 0;

        if (mismatch)
        {
            CARB_LOG_WARN("Surface deformable body creation failed. Mismatch between OmniPhysicsSurfaceDeformableSimAPI "
                          "rest shape attributes and UsdGeomMesh topology detected, %s",
                          simMeshPrim.GetPath().GetText());
            ICE_FREE(internalBody);
            return kInvalidObjectId;
        }
    }

    pxr::GfMatrix4d cookingToWorld;
    pxr::GfMatrix4d simMeshToCooking;
    double cookingToWorldScale;
    cookingDataAsync->computeDeformableCookingTransform(&simMeshToCooking, &cookingToWorld, &cookingToWorldScale,
                                                        simMeshToWorld, simMeshPoints);
    {
        // simMeshToCooking output excludes world scale, so adding it here
        GfMatrix4d s;
        s.SetScale(cookingToWorldScale);
        simMeshToCooking = simMeshToCooking * s;
    }

    numAllSkinMeshPoints = collectSkinMeshes(internalBody->mSkinMeshPrims, internalBody->mSkinMeshRanges,
                                             internalBody->mWorldToSkinMeshTransforms, bodyPrim.GetStage(), desc.skinGeomPaths);

    parseSkinMeshPoints(allSkinMeshPoints, internalBody->mSkinMeshPrims, numAllSkinMeshPoints);

    parseSkinBindPointsWorld(allSkinMeshBindPointsWorld, bodyPrim.GetStage(), desc.skinGeomPaths,
                             desc.skinGeomBindPoseTokens, numAllSkinMeshPoints);

    // Get assigned material
    ObjectId materialId = getDeformableMaterialId(*scene, desc);
    InternalDeformableMaterial* internalMaterial;
    PxDeformableSurfaceMaterial* material = getSurfaceDeformableBodyMaterial(internalMaterial, *scene, materialId);
    if (!material)
    {
        CARB_LOG_WARN("Failed to aquire material for surface deformable body! Prim(%s)\n", path.GetText());
        ICE_FREE(internalBody);
        return kInvalidObjectId;
    }
    internalBody->mMaterialId = materialId;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: implement mesh cooking and transform for PxDeformableSurface
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Still cooking in cooking space to be consistent with volume deformables, except scaled to world size!
    PxDeformableSurface* deformableSurface = nullptr;
    PxTriangleMesh* triangleMesh = nullptr;
    {
        PxCookingParams ckParams = OmniPhysX::getInstance().getPhysXSetup().getDefaultCookingParams();
        ckParams.buildTriangleAdjacencies = false;
        ckParams.buildGPUData = true;
        ckParams.midphaseDesc = PxMeshMidPhase::eBVH34;
        ckParams.meshPreprocessParams |= PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES;
        ckParams.meshPreprocessParams &= ~(PxMeshPreprocessingFlags)(PxMeshPreprocessingFlag::eENABLE_VERT_MAPPING);
        ckParams.meshPreprocessParams &= ~(PxMeshPreprocessingFlags)(PxMeshPreprocessingFlag::eWELD_VERTICES);
        ckParams.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;
        ckParams.meshWeldTolerance = 0.0f;

        // transform to cooking space and scaling to world size
        std::vector<pxr::GfVec3f> positions;
        positions.resize(simMeshRestShapePoints.size());
        for (PxU32 i = 0; i < positions.size(); ++i)
        {
            const pxr::GfVec3f& simRestPos = simMeshRestShapePoints[i];
            positions[i] = simMeshToCooking.Transform(simRestPos);
        }

        // cook triangle mesh
        PxTriangleMeshDesc meshDesc;
        meshDesc.points.count = PxU32(positions.size());
        meshDesc.triangles.count = PxU32(simMeshIndices.size() / 3);
        meshDesc.points.stride = sizeof(float) * 3;
        meshDesc.triangles.stride = sizeof(int) * 3;
        meshDesc.points.data = positions.data();
        meshDesc.triangles.data = simMeshIndices.data();

        // validation, we already clean the mesh during sim mesh generation, we require
        // the simulation mesh in USD to be "clean" in terms of PhysX PxTriangleMesh standards.
        if (!PxValidateTriangleMesh(ckParams, meshDesc))
        {
            CARB_LOG_WARN("PxValidateTriangleMesh for PxDeformableSurface failed, %s", simMeshPrim.GetPath().GetText());
            ICE_FREE(internalBody);
            return kInvalidObjectId;
        }

        // cooking without cleaning
        PxDefaultMemoryOutputStream writeBuffer;
        bool status = PxCookTriangleMesh(ckParams, meshDesc, writeBuffer);
        if (!status)
        {
            CARB_LOG_WARN("PxCookTriangleMesh for PxDeformableSurface failed, %s", simMeshPrim.GetPath().GetText());
            ICE_FREE(internalBody);
            return kInvalidObjectId;
        }

        PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
        triangleMesh = physxSetup.getPhysics()->createTriangleMesh(readBuffer);
        if (!triangleMesh)
        {
            CARB_LOG_WARN("PxTriangleMesh creation failed: %s\n", path.GetText());
            ICE_FREE(internalBody);
            return kInvalidObjectId;
        }

        if (triangleMesh->getNbVertices() != uint32_t(simMeshPoints.size()))
        {
            CARB_LOG_WARN("PxTriangleMesh vertices don't align with simulation mesh vertices: %s\n", path.GetText());
            ICE_FREE(internalBody);
            triangleMesh->release();
            return kInvalidObjectId;
        }

        deformableSurface = physxSetup.getPhysics()->createDeformableSurface(*cudaContextManager);
        if (!deformableSurface)
        {
            CARB_LOG_WARN("Failed to create PxDeformableSurface: %s\n", path.GetText());
            ICE_FREE(internalBody);
            triangleMesh->release();
            return kInvalidObjectId;
        }

        PxShapeFlags shapeFlags = PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;

        if (OmniPhysX::getInstance().isDebugVisualizationEnabled())
            shapeFlags |= PxShapeFlag::eVISUALIZATION;

        PxDeformableSurfaceMaterial* materialPtr = material;
        PxTriangleMeshGeometry geometry(triangleMesh);
        PxShape* shape = physxSetup.getPhysics()->createShape(geometry, &materialPtr, 1, true, shapeFlags);
        if (shape)
        {
            const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
            PxFilterData fd;
            convertCollisionGroupToPxFilterData(collisionGroup, fd);
            shape->setSimulationFilterData(fd);

            //special PxFEMCloth overrides, for PxDeformableVolume we are not doing that.
            //TODO: consider removing or generalizing
            PxReal restOffset = desc.restOffset;
            PxReal contactOffset = desc.contactOffset;

            if (desc.restOffset <= 0.0f)
                restOffset = 0.5f * material->getThickness();

            if (desc.contactOffset <= 0.0f)
                contactOffset = 1.2f * restOffset;

            deriveCollisionOffsets(restOffset, contactOffset,
                *scene, geometry, 1.0f, restOffset, contactOffset, path);
            shape->setContactOffset(contactOffset);
            shape->setRestOffset(restOffset);

            deformableSurface->attachShape(*shape);
            shape->release();
        }

        scene->getScene()->addActor(*deformableSurface);
    }

    {
        float metersPerUnit = float(pxr::UsdGeomGetStageMetersPerUnit(attachedStage.getStage()));
        float contactOffset = deformableSurface->getShape()->getContactOffset();
        float restOffset = deformableSurface->getShape()->getRestOffset();
        float selfCollisionFilterDistance = deriveSelfCollisionFilterDistance(desc.selfCollisionFilterDistance,
            restOffset, contactOffset, metersPerUnit, PhysXType::ePTDeformableSurface);
        deformableSurface->setSelfCollisionFilterDistance(selfCollisionFilterDistance);
        deformableSurface->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, !desc.selfCollision);
    }
    {
        const bool disableSleeping = OmniPhysX::getInstance().getCachedSettings().disableSleeping;
        deformableSurface->setSleepThreshold(disableSleeping ? 0.0f : desc.sleepThreshold);
        deformableSurface->setSettlingThreshold(disableSleeping ? 0.0f : desc.settlingThreshold);
        deformableSurface->setSettlingDamping(desc.settlingDamping);
    }

    deformableSurface->setDeformableBodyFlag(PxDeformableBodyFlag::eKINEMATIC, desc.kinematicBody);
    deformableSurface->setDeformableBodyFlag(PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD, desc.enableSpeculativeCCD);
    deformableSurface->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, desc.disableGravity);

    deformableSurface->setLinearDamping(desc.linearDamping);
    deformableSurface->setSolverIterationCounts(scene->getInternalScene()->clampPosIterationCount(desc.solverPositionIterationCount));

    // bending
    bool enableFlattening = (desc.restBendAnglesDefault == OmniPhysicsDeformableAttrTokens->flatDefault);
    deformableSurface->setDeformableSurfaceFlag(PxDeformableSurfaceFlag::eENABLE_FLATTENING, enableFlattening);

    // collision substepping
    deformableSurface->setNbCollisionPairUpdatesPerTimestep(desc.collisionPairUpdateFrequency);
    deformableSurface->setNbCollisionSubsteps(desc.collisionIterationMultiplier);

    // velocity clamping
    deformableSurface->setMaxVelocity(desc.maxLinearVelocity);
    deformableSurface->setMaxDepenetrationVelocity(desc.maxDepenetrationVelocity);

    //startsAsleep
    if (desc.startsAsleep)
    {
        //velocities are initialized to zero below
        //TODO implement putToSleep in SDK
        deformableSurface->setWakeCounter(0.0f);
    }

    internalBody->mNumSimMeshVertices = uint32_t(simMeshPoints.size());
    internalBody->mNumSkinMeshVertices = uint32_t(numAllSkinMeshPoints);

    internalBody->mSimMeshPositionInvMassH = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, triangleMesh->getNbVertices());
    internalBody->mSimMeshVelocityH = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, triangleMesh->getNbVertices());
    PxVec4* simMeshRestPositionH = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, triangleMesh->getNbVertices());

    bool zeroVelocities = desc.startsAsleep || simMeshVelocities.empty() ||
                          simMeshVelocities.size() != internalBody->mNumSimMeshVertices;
    for (uint32_t i = 0; i < internalBody->mNumSimMeshVertices; ++i)
    {
        internalBody->mSimMeshPositionInvMassH[i] = PxVec4(toPhysX(simMeshToWorld.Transform(simMeshPoints[i])), 0.0f);
        simMeshRestPositionH[i] = PxVec4(toPhysX(simMeshToWorld.Transform(simMeshRestShapePoints[i])), 0.0f);
        //TODO add rotation if feature enabled
        PxVec4 vel = zeroVelocities ? PxVec4(0.0f) : PxVec4(toPhysX(simMeshVelocities[i]), 0.0f);
        internalBody->mSimMeshVelocityH[i] = vel;
    }

    //TODO: make PxTriangleMesh for PxFEMCloth reusable
    //PxDeformableVolumeExt::transform(*deformableVolume, pxCookingToWorld, pxCookingToWorldScale,
    //    internalBody->mSimMeshPositionInvMassH, internalBody->mSimMeshVelocityH,
    //    internalBody->mCollMeshPositionInvMassH,
    //    collMeshRestPositionH);

    if (!setSurfaceDeformableMass(*deformableSurface, internalBody->mSimMeshPositionInvMassH,
        internalBody->mBodyMass, internalMaterial ? &internalMaterial->mDensity : nullptr, material->getThickness()))
    {
        CARB_LOG_WARN("Failed to aquire mass/density for surface deformable body! Prim(%s)\n", path.GetText());
        ICE_FREE(internalBody);
        deformableSurface->release();
        triangleMesh->release();
        PX_PINNED_HOST_FREE(cudaContextManager, simMeshRestPositionH);
        return kInvalidObjectId;
    }

    //TODO: Why does PxFEMClothExt::copyToDevice take internalBody->mNumSimMeshVertices, but PxDeformableVolumeExt doesn't
    //it seems redundant with passing deformableSurface, but maybe better to make checks possible against deformableSurface instance?
    //maybe this should also be fixed for setSurfaceDeformableMass/setVolumeDeformableMass?
    PxDeformableSurfaceExt::copyToDevice(*deformableSurface, PxDeformableSurfaceDataFlag::eALL,
        internalBody->mNumSimMeshVertices, internalBody->mSimMeshPositionInvMassH, internalBody->mSimMeshVelocityH, simMeshRestPositionH,
        internalBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());

    internalBody->mTriangleMesh = triangleMesh;
    internalBody->mDeformableSurface = deformableSurface;

    // compute and store map to get physx triangle indexing from sim triangle ordering
    // (vertices are in same order)
    {
        internalBody->mSimToPhysxTriMap.resize(triangleMesh->getNbTriangles());
        const uint32_t* triRemap = triangleMesh->getTrianglesRemap();
        for (uint32_t i = 0; i < triangleMesh->getNbTriangles(); ++i)
        {
            internalBody->mSimToPhysxTriMap[triRemap[i]] = i;
        }
    }

    // Store initial properties for reset of deformables created during simulation
    copyBuffer(internalBody->mSimMeshPointsSaveRestoreBuf, simMeshPoints.data(), uint32_t(simMeshPoints.size()));
    copyBuffer(internalBody->mSimMeshVelocitiesSaveRestoreBuf, simMeshVelocities.data(), uint32_t(simMeshVelocities.size()));
    copyBuffer(internalBody->mAllSkinMeshPointsSaveRestoreBuf, allSkinMeshPoints.data(), uint32_t(allSkinMeshPoints.size()));

    // not a typo, we use the simMeshPrim for bounds. TODO check for all types what the purpose is of the USD bounds and on
    // which prims they should be set and updated.
    UsdGeomBoundable::ComputeExtentFromPlugins(UsdGeomBoundable(simMeshPrim), UsdTimeCode::Default(),
        &internalBody->mSimMeshExtentSaveRestoreBuf);

    ObjectId objectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
        ePTDeformableSurface, deformableSurface, internalBody, path);

    deformableSurface->userData = (void*)objectId;

    if (internalBody->mSimMeshPrim != internalBody->mBodyPrim)
    {
        //registering sub objects that need change handling 
        ObjectId simulationId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
            ePTDeformableSurface, deformableSurface, internalBody, desc.simMeshPath);
        attachedStage.getObjectDatabase()->findOrCreateEntry(desc.simMeshPath, eSurfaceDeformableBody, simulationId);
    }

    if (internalMaterial)
    {
        internalMaterial->addDeformableId(objectId);
    }

    // TODO check with Alain whether this is reasonable
    PxScopedCudaLock _lock(*cudaContextManager);
    PxCudaContext* cudaCtx = cudaContextManager->getCudaContext();
    cudaCtx->streamSynchronize(internalBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
    PX_PINNED_HOST_FREE(cudaContextManager, simMeshRestPositionH);

    if (numAllSkinMeshPoints > 0)
    {
        // Skinning mesh setup
        const PxTriangleMeshGeometry& triangleMeshGeom =
            static_cast<const PxTriangleMeshGeometry&>(deformableSurface->getShape()->getGeometry());
        const PxU32 nbTriangles = triangleMeshGeom.triangleMesh->getNbTriangles();
        const PxU32 numTriIndices = 3 * nbTriangles;

        bool uses16bit = triangleMeshGeom.triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;

        internalBody->mSimMeshTriIndicesD = PxCudaHelpersExt::allocDeviceBuffer<PxU32>(*cudaContextManager, numTriIndices);

        internalBody->mAllSkinnedVerticesH = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec3>(*cudaContextManager, numAllSkinMeshPoints);
        internalBody->mAllSkinnedVerticesD = PxCudaHelpersExt::allocDeviceBuffer<PxVec3>(*cudaContextManager, numAllSkinMeshPoints);
        if (internalBody->mAllSkinnedVerticesH && internalBody->mAllSkinnedVerticesD)
        {
            std::memcpy(internalBody->mAllSkinnedVerticesH, allSkinMeshBindPointsWorld.data(), sizeof(PxVec3) * numAllSkinMeshPoints);
            PxCudaHelpersExt::copyHToD(*cudaContextManager, internalBody->mAllSkinnedVerticesD, internalBody->mAllSkinnedVerticesH, numAllSkinMeshPoints);
        }

        PxTriangleMeshEmbeddingInfo* skinningEmbeddingInfoH = PxCudaHelpersExt::allocPinnedHostBuffer<PxTriangleMeshEmbeddingInfo>(*cudaContextManager, numAllSkinMeshPoints);
        internalBody->mSkinningEmbeddingInfoD = PxCudaHelpersExt::allocDeviceBuffer<PxTriangleMeshEmbeddingInfo>(*cudaContextManager, numAllSkinMeshPoints);
        internalBody->mNormalVectorsD = PxCudaHelpersExt::allocDeviceBuffer<PxVec3>(*cudaContextManager, triangleMeshGeom.triangleMesh->getNbVertices());

        PxArray<PxU32> simMeshTriIndices(numTriIndices);
        if (uses16bit)
        {
            const PxU16* triangleIndices = reinterpret_cast<const PxU16*>(triangleMeshGeom.triangleMesh->getTriangles());
            for (PxU32 i = 0; i < simMeshTriIndices.size(); ++i)
                simMeshTriIndices[i] = triangleIndices[i];
        }
        else
        {
            const PxU32* triangleIndices = reinterpret_cast<const PxU32*>(triangleMeshGeom.triangleMesh->getTriangles());
            for (PxU32 i = 0; i < simMeshTriIndices.size(); ++i)
                simMeshTriIndices[i] = triangleIndices[i];
        }

        // For setting up skinning data, we use the bind pose of the sim mesh and all skin meshes
        // in the cooking space with world scale to preserve distances.
        PxArray<PxVec3> guideVertices((uint32_t)simMeshBindPoints.size());
        for (uint32_t i = 0; i < guideVertices.size(); ++i)
        {
            const GfVec3f point = simMeshToCooking.Transform(simMeshBindPoints[i]);
            guideVertices[i] = PxVec3(point[0], point[1], point[2]);
        }

        GfMatrix4d worldToCooking = cookingToWorld.GetInverse();
        PxArray<PxVec3> embeddedVertices((uint32_t)allSkinMeshBindPointsWorld.size());
        for (uint32_t i = 0; i < embeddedVertices.size(); ++i)
        {
            const GfVec3f point = worldToCooking.Transform(allSkinMeshBindPointsWorld[i]);
            embeddedVertices[i] = PxVec3(point[0], point[1], point[2]);
        }

        PxDeformableSkinningExt::initializeInterpolatedVertices(
            skinningEmbeddingInfoH, guideVertices.begin(), nullptr, simMeshTriIndices.begin(), nbTriangles,
            embeddedVertices.begin(), (PxU32)embeddedVertices.size());

        PxCudaHelpersExt::copyHToD(*cudaContextManager, internalBody->mSimMeshTriIndicesD, simMeshTriIndices.begin(), numTriIndices);
        PxCudaHelpersExt::copyHToD(*cudaContextManager, internalBody->mAllSkinnedVerticesD, internalBody->mAllSkinnedVerticesH, numAllSkinMeshPoints);
        PxCudaHelpersExt::copyHToD(*cudaContextManager, internalBody->mSkinningEmbeddingInfoD, skinningEmbeddingInfoH, numAllSkinMeshPoints);

        if (skinningEmbeddingInfoH)
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, skinningEmbeddingInfoH);

        PxShape* surfaceShape = deformableSurface->getShape();
        const PxReal halfThickness = surfaceShape->getRestOffset();

        // Add skinning data after the surface deformable body is created
        deformables::SurfaceDeformableSkinningData skinningData;
        skinningData.mDeformableSurface = deformableSurface;
        skinningData.mGuideTrianglesD = internalBody->mSimMeshTriIndicesD;
        skinningData.mNumGuideTriangles = nbTriangles;
        skinningData.mGuideNormalsD = internalBody->mNormalVectorsD;
        skinningData.mNumGuideVertices = triangleMeshGeom.triangleMesh->getNbVertices();
        skinningData.mSkinningEmbeddingInfoD = internalBody->mSkinningEmbeddingInfoD;
        skinningData.mSkinnedVerticesD = internalBody->mAllSkinnedVerticesD;
        skinningData.mNumSkinnedVertices = internalBody->mNumSkinMeshVertices;
        skinningData.mHalfThickness = halfThickness;

        scene->getInternalScene()->mSurfaceDeformablePostSolveCallback->addSurfaceDeformableSkinningData(skinningData);
    }

    // add internal surface deformable body to internal scene
    scene->getInternalScene()->mSurfaceDeformableBodies.push_back(internalBody);
    return objectId;
}


} // namespace physx
} // namespace omni
