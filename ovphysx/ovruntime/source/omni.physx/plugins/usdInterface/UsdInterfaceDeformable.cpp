// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-24
 *
 * @implements REQ-COOK-SOURCE-001
 * @covers AC-5 AC-6
 *
 * @implements REQ-MATH-001
 * @covers AC-9
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 */

#include <omni/physics/parse/KnownTokens.h>

#include "UsdInterface.h"

#include <private/omni/physx/PhysxUsd.h>

#include <usdLoad/LoadUsd.h>
#include <usdLoad/Mass.h>

#include <internal/InternalScene.h>
#include <internal/InternalDeformable.h>

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

void computePointExtent(const std::vector<carb::Float3>& points, std::vector<carb::Float3>& extent)
{
    extent.clear();
    if (points.empty())
        return;

    const carb::Float3* p3 = points.data();
    carb::Float3 minPoint = p3[0];
    carb::Float3 maxPoint = p3[0];
    for (size_t i = 0; i < points.size(); ++i)
    {
        minPoint.x = std::min(minPoint.x, p3[i].x);
        minPoint.y = std::min(minPoint.y, p3[i].y);
        minPoint.z = std::min(minPoint.z, p3[i].z);
        maxPoint.x = std::max(maxPoint.x, p3[i].x);
        maxPoint.y = std::max(maxPoint.y, p3[i].y);
        maxPoint.z = std::max(maxPoint.z, p3[i].z);
    }
    extent = { minPoint, maxPoint };
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
            density = getScaledDensity(usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage()->getSourceUnits(), defaultDensitySi);
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
            density = getScaledDensity(usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage()->getSourceUnits(), defaultDensitySi);
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

// reportPath is resolved text (AttachedStage::textFor), used only for diagnostic logging --
// unconditional, no pxr dependency.
void deriveCollisionOffsets(float& outRestOffset, float& outContactOffset,
    const omni::physx::PhysXScene& scene, const PxGeometry& geometry, const float geometryScale,
    const float restOffset, const float contactOffset, const char* reportPath)
{
    // TODO unify with UsdInterface.cpp, createShape()
    outRestOffset = restOffset;
    outContactOffset = contactOffset;

    if (contactOffset >= 0.0f)
    {
        if (contactOffset <= restOffset)
        {
            CARB_LOG_ERROR("Collision contact offset must be positive and greater then restOffset, prim: %s", reportPath);
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
            CARB_LOG_ERROR("Collision rest offset must be lesser then contact offset, prim: %s", reportPath);
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

// Local-to-world transform of `key` via the physics source (no direct USD prim
// access). Bind/creation transforms are static, so they read at the default
// time code (see PhysXTools.h getWorldTransform).
PxMat44d sourceLocalToWorld(const usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    return getWorldTransform(attachedStage, key, omni::physics::parse::ReadTime::defaultTime());
}

// Source-routed single-apply HasAPI check (no direct USD prim access).
// `schemaToken` is the already-interned applied-API-schema name (e.g.
// tok.OmniPhysicsSurfaceDeformableSimAPI) -- the same TokenId convention
// IPhysicsSource::hasSchema is queried with everywhere else in the runtime.
bool sourceHasAPI(const usdparser::AttachedStage& attachedStage,
                  omni::physics::parse::ObjectKey key,
                  omni::physics::parse::TokenId schemaToken)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    return src && src->hasSchema(key, schemaToken);
}

// Source-agnostic equivalent of pxr's UsdSchemaRegistry::MakeMultipleApplyNameInstance:
// substitutes the __INSTANCE_NAME__ placeholder in a multi-apply attribute-name
// template (e.g. tok.deformablePose_MultipleApplyTemplate_OmniphysicsPoints) with the
// given instance name, and interns the result. Returns an invalid TokenId when `src`
// is null, matching the null-source behavior of the pxr-based path it replaces.
omni::physics::parse::TokenId makeMultiApplyAttributeToken(const omni::physics::parse::IPhysicsSource* src,
                                                            omni::physics::parse::TokenId nameTemplate,
                                                            omni::physics::parse::TokenId instanceName)
{
    if (!src)
        return omni::physics::parse::TokenId{};
    static constexpr char kInstanceNamePlaceholder[] = "__INSTANCE_NAME__";
    std::string result(src->tokenToString(nameTemplate));
    const size_t pos = result.find(kInstanceNamePlaceholder);
    if (pos != std::string::npos)
        result.replace(pos, sizeof(kInstanceNamePlaceholder) - 1, src->tokenToString(instanceName));
    return src->internToken(result);
}

size_t collectSkinMeshes(const usdparser::AttachedStage& attachedStage,
                         std::vector<omni::physics::parse::ObjectKey>& skinMeshKeys,
                         std::vector<carb::Uint2>& skinMeshRanges,
                         std::vector<::physx::PxMat44d>& worldToSkinMeshTransforms,
                         const std::vector<omni::physics::parse::ObjectKey>& skinGeomKeys)
{
    size_t numAllSkinMeshPoints = 0;

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    omni::physics::parse::KnownTokens tok;
    if (src)
        tok.intern(*src);

    // histogram, transforms + references
    for (const omni::physics::parse::ObjectKey key : skinGeomKeys)
    {
        std::vector<carb::Float3> points;
        getArrayValue(attachedStage, key, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);
        if (points.size())
        {
            carb::Uint2 range = { uint32_t(numAllSkinMeshPoints), uint32_t(points.size()) };
            numAllSkinMeshPoints += uint32_t(points.size());

            const PxMat44d skinGeomToWorld = sourceLocalToWorld(attachedStage, key);
            skinMeshKeys.push_back(key);
            skinMeshRanges.push_back(range);
            worldToSkinMeshTransforms.push_back(affineInverse(skinGeomToWorld));
        }
    }
    return numAllSkinMeshPoints;
}

void parseSkinBindPointsWorld(const usdparser::AttachedStage& attachedStage,
                              std::vector<carb::Float3>& allSkinMeshBindPointsWorld,
                              const std::vector<omni::physics::parse::ObjectKey>& skinGeomKeys,
                              const std::vector<omni::physics::parse::TokenId>& skinGeomBindPoseTokens,
                              const size_t numAllSkinMeshPoints)
{
    allSkinMeshBindPointsWorld.resize(numAllSkinMeshPoints);

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    omni::physics::parse::KnownTokens tok;
    if (src)
        tok.intern(*src);

    size_t offsetSkinMeshPoints = 0;
    for (size_t i = 0; i < skinGeomKeys.size(); ++i)
    {
        const omni::physics::parse::ObjectKey key = skinGeomKeys[i];
        const omni::physics::parse::TokenId skinGeomBindPoseToken = skinGeomBindPoseTokens[i];

        std::vector<carb::Float3> skinMeshBindPointsLocal;
        // A valid bind-pose token guarantees the OmniPhysicsDeformablePoseAPI
        // instance is applied (the parser only emits it then), so reading the
        // instance's points attribute is sufficient — no HasAPI re-check needed.
        if (skinGeomBindPoseToken.valid())
        {
            const omni::physics::parse::TokenId pointAttrName = makeMultiApplyAttributeToken(
                src, tok.deformablePose_MultipleApplyTemplate_OmniphysicsPoints, skinGeomBindPoseToken);
            getArrayValue(attachedStage, key, pointAttrName, omni::physics::parse::ReadTime::defaultTime(), skinMeshBindPointsLocal);
        }
        else
        {
            // When there is no bind pose, use points instead
            getArrayValue(attachedStage, key, tok.points, omni::physics::parse::ReadTime::defaultTime(), skinMeshBindPointsLocal);
        }

        const PxMat44d skinGeomToWorld = sourceLocalToWorld(attachedStage, key);
        if (offsetSkinMeshPoints + skinMeshBindPointsLocal.size() <= allSkinMeshBindPointsWorld.size())
        {
            for (const carb::Float3& point : skinMeshBindPointsLocal)
            {
                const PxVec3d world = skinGeomToWorld.transform(toPhysXd(point));
                allSkinMeshBindPointsWorld[offsetSkinMeshPoints++] =
                    carb::Float3{ float(world.x), float(world.y), float(world.z) };
            }
        }
    }
}

void parseSkinMeshPoints(const usdparser::AttachedStage& attachedStage,
                         std::vector<carb::Float3>& allSkinMeshPoints,
                         const std::vector<omni::physics::parse::ObjectKey>& skinMeshKeys,
                         const size_t numAllSkinMeshPoints)
{
    allSkinMeshPoints.resize(numAllSkinMeshPoints);

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    omni::physics::parse::KnownTokens tok;
    if (src)
        tok.intern(*src);

    size_t offsetSkinPoints = 0;
    for (const omni::physics::parse::ObjectKey key : skinMeshKeys)
    {
        std::vector<carb::Float3> points;
        getArrayValue(attachedStage, key, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);

        if (offsetSkinPoints + points.size() <= allSkinMeshPoints.size())
        {
            for (const carb::Float3& point : points)
            {
                allSkinMeshPoints[offsetSkinPoints++] = point;
            }
        }
    }
}

void parseSimBindPoints(const usdparser::AttachedStage& attachedStage,
                        std::vector<carb::Float3>& simMeshBindPoints,
                        omni::physics::parse::ObjectKey simMeshKey,
                        omni::physics::parse::TokenId simMeshBindPoseTokenId,
                        const std::vector<carb::Float3>& simMeshPoints)
{
    // A valid bind-pose token guarantees the OmniPhysicsDeformablePoseAPI
    // instance is applied (parser invariant), so reading its points attribute
    // needs no HasAPI re-check.
    if (simMeshBindPoseTokenId.valid())
    {
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        omni::physics::parse::KnownTokens tok;
        if (src)
            tok.intern(*src);

        const omni::physics::parse::TokenId pointAttrName = makeMultiApplyAttributeToken(
            src, tok.deformablePose_MultipleApplyTemplate_OmniphysicsPoints, simMeshBindPoseTokenId);
        getArrayValue(attachedStage, simMeshKey, pointAttrName, omni::physics::parse::ReadTime::defaultTime(), simMeshBindPoints);
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
                    attachedStage.textFor(intDeformableBody->mBodyKey),
                    attachedStage.textFor(matRecord->mKey));
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
                               attachedStage.textFor(intDeformableBody->mBodyKey));
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
                attachedStage.textFor(intDeformableBody->mBodyKey));
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

bool PhysXUsdPhysicsInterface::updateDeformableBodyPositions(usdparser::AttachedStage& attachedStage, const ObjectId objectId)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);

    if (!objectFullRecord || !objectFullRecord->mPtr)
    {
        return true;
    }

    const PxMat44d transform = sourceLocalToWorld(attachedStage, objectFullRecord->mKey);

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    omni::physics::parse::KnownTokens tok;
    if (src)
        tok.intern(*src);

    // This dispatch only fires in response to a live authored edit on `points` (see
    // omni::physx::updateDeformableBody), so any cooked-geometry carrier entry for
    // this key is a stale bind pose the edit has already superseded. Drop it before
    // reading so a sink-less (ovstage) attach falls through to the fresh value
    // instead of serving the cook's bind pose forever (ADR-0022).
    attachedStage.clearCookedArray(objectFullRecord->mKey, tok.points);

    std::vector<carb::Float3> points;
    getArrayValue(attachedStage, objectFullRecord->mKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), points);

    InternalDeformableBody* internalDeformableBody = (InternalDeformableBody*)objectFullRecord->mInternalPtr;
    if (internalDeformableBody)
    {
        if (points.size() != internalDeformableBody->mNumSimMeshVertices)
        {
            CARB_LOG_WARN("Size of points of %s has changed - skipping update.", attachedStage.textFor(objectFullRecord->mKey));
            return true;
        }

        PxVec4* simPositionInvMass = internalDeformableBody->mSimMeshPositionInvMassH;
        for (unsigned int i = 0; i < internalDeformableBody->mNumSimMeshVertices; ++i)
        {
            const PxVec3d worldPoint = transform.transform(toPhysXd(points[i]));

            simPositionInvMass[i] = PxVec4(float(worldPoint.x), float(worldPoint.y), float(worldPoint.z), simPositionInvMass[i].w);
        }
    }

    if (internalType == ePTDeformableSurface)
    {
        PxDeformableSurface* deformableSurface = (PxDeformableSurface*)objectFullRecord->mPtr;     

        if (deformableSurface &&
            internalDeformableBody &&
            sourceHasAPI(attachedStage, objectFullRecord->mKey, tok.OmniPhysicsSurfaceDeformableSimAPI))
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
            sourceHasAPI(attachedStage, objectFullRecord->mKey, tok.OmniPhysicsVolumeDeformableSimAPI))
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

bool PhysXUsdPhysicsInterface::updateDeformableBodyVelocities(usdparser::AttachedStage& attachedStage, const ObjectId objectId)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);

    if (!objectFullRecord || !objectFullRecord->mPtr)
    {
        return true;
    }

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    omni::physics::parse::KnownTokens tok;
    if (src)
        tok.intern(*src);

    // See the position-update twin above: a live edit already landed, so any
    // carrier entry for `velocities` on this key is stale and must not shadow it.
    attachedStage.clearCookedArray(objectFullRecord->mKey, tok.velocities);

    std::vector<carb::Float3> velocities;
    getArrayValue(attachedStage, objectFullRecord->mKey, tok.velocities, omni::physics::parse::ReadTime::defaultTime(), velocities);

    InternalDeformableBody* internalDeformableBody = (InternalDeformableBody*)objectFullRecord->mInternalPtr;
    if (internalDeformableBody)
    {
        if (velocities.size() != internalDeformableBody->mNumSimMeshVertices)
        {
            CARB_LOG_WARN("Size of velocities of %s has changed - skipping update.", attachedStage.textFor(objectFullRecord->mKey));
            return true;
        }

        PxVec4* simVelocity = internalDeformableBody->mSimMeshVelocityH;
        for (unsigned int i = 0; i < internalDeformableBody->mNumSimMeshVertices; ++i)
        {
            simVelocity[i] = PxVec4(toPhysX(velocities[i]), simVelocity[i].w);
        }
    }

    if (internalType == ePTDeformableSurface)
    {
        PxDeformableSurface* deformableSurface = (PxDeformableSurface*)objectFullRecord->mPtr;

        if (deformableSurface &&
            internalDeformableBody &&
            sourceHasAPI(attachedStage, objectFullRecord->mKey, tok.OmniPhysicsSurfaceDeformableSimAPI))
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
            sourceHasAPI(attachedStage, objectFullRecord->mKey, tok.OmniPhysicsVolumeDeformableSimAPI))
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
        CARB_LOG_ERROR("Collision rest offset must be lesser then contact offset, prim: %s", attachedStage.textFor(objectRecord->mKey));
        return true;
    }

    float metersPerUnit = attachedStage.getSource()->getSourceUnits().metersPerUnit;
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
        CARB_LOG_ERROR("Collision contact offset must be positive and greater then restOffset, prim: %s", attachedStage.textFor(objectRecord->mKey));
        return true;
    }

    float metersPerUnit = attachedStage.getSource()->getSourceUnits().metersPerUnit;
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
    float metersPerUnit = attachedStage.getSource()->getSourceUnits().metersPerUnit;
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
        PxDeformableMaterial* material = nullptr;
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
    omni::physics::parse::ObjectKey bodyKey, PhysxVolumeDeformableBodyDesc const& desc)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PhysXScene* scene = physxSetup.getPhysXScene(desc.sceneId);
    if (!scene || !scene->isFullGpuPipelineAvailable())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            usdparser::ErrorCode::eError,
            "Deformable Body feature is only supported on GPU. Please enable GPU dynamics flag in Property/Scene of physics scene!");
        return kInvalidObjectId;
    }

    if (!checkScenes())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            usdparser::ErrorCode::eError, "No physics scene created, please add physics scene into stage!");
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

    const omni::physics::parse::ObjectKey simMeshKey = desc.simMeshKey;
    const omni::physics::parse::ObjectKey collMeshKey = desc.collisionMeshKey;
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    // isTetMeshLike, not isA(UsdGeomTetMesh): ovstage reports a UsdGeomTetMesh as plain "Mesh"
    // (its populator has no TetMesh mapping), so the concrete-type check rejected every volume
    // deformable loaded from a non-USD source. See PhysXTools.h::isTetMeshLike.
    if (!src || !src->exists(bodyKey) || !isTetMeshLike(attachedStage, simMeshKey) ||
        !isTetMeshLike(attachedStage, collMeshKey))
    {
        CARB_LOG_WARN("PhysX Deformable Body creation failed due to invalid deformable prims.");
        return kInvalidObjectId;
    }

    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);

    const PxMat44d simMeshToWorld = sourceLocalToWorld(attachedStage, simMeshKey);
    const PxMat44d worldToSimMesh = affineInverse(simMeshToWorld);

    PxMat44d worldToCollMesh = worldToSimMesh;
    if (desc.collisionMeshKey != desc.simMeshKey)
    {
        const PxMat44d collMeshToWorld = sourceLocalToWorld(attachedStage, collMeshKey);
        worldToCollMesh = affineInverse(collMeshToWorld);
    }

    // sync tet mesh generation
    if (desc.hasAutoAPI)
    {
        cookingDataAsync->cookVolumeDeformableBody(desc, bodyKey, attachedStage, false);
    }

    // get cooked deformable volume mesh data (in 'simMesh' space)
    PxDefaultMemoryOutputStream outData;
    bool deformableVolumeMeshCooked =
        cookingDataAsync->cookDeformableVolumeMesh(outData, desc, bodyKey, attachedStage, false);

    if (!deformableVolumeMeshCooked)
    {
        CARB_LOG_WARN("Failed to cook PxDeformableVolumeMesh! Prim(%s)\n", attachedStage.textFor(bodyKey));
        return kInvalidObjectId;
    }

    // surface triangles are mandatory on the collision mesh, because we need to be able to deterministically
    // reference surface triangles for attachments, also useful for picking.
    std::vector<carb::Int3> collMeshSurfaceTriangles;
    getArrayValue(attachedStage, collMeshKey, tok.surfaceFaceVertexIndices,
                  omni::physics::parse::ReadTime::defaultTime(), collMeshSurfaceTriangles);
    if (collMeshSurfaceTriangles.size() == 0)
    {
        CARB_LOG_WARN("PhysX Deformable Body creation failed, collision mesh %s has no surfaceFaceVertexIndices. "
                      "For an auto-cooked body these are produced by the cook and read back from the scene "
                      "description, so an empty array means the cooked data was never published. Prim(%s)\n",
                      attachedStage.textFor(collMeshKey), attachedStage.textFor(bodyKey));
        return kInvalidObjectId;
    }

    std::vector<uint32_t> collMeshSurfaceTriToTetMap;
    PxDefaultMemoryInputData inData(outData.getData(), outData.getSize());
    PxDeformableVolumeMesh* deformableVolumeMesh = cookingDataAsync->createDeformableVolumeMesh(collMeshSurfaceTriToTetMap, inData);
    if (!deformableVolumeMesh || collMeshSurfaceTriangles.size() != collMeshSurfaceTriToTetMap.size())
    {
        CARB_LOG_WARN("Failed to create PxDeformableVolumeMesh from cooked data! Prim(%s)\n", attachedStage.textFor(bodyKey));
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
            ePTDeformableVolume, nullptr, internalBody, bodyKey);

        scene->getInternalScene()->mVolumeDeformableBodies.push_back(internalBody);
        return objectId;
    }

    internalBody->mBodyKey = bodyKey;
    internalBody->mSimMeshKey = simMeshKey;
    internalBody->mCollMeshKey = collMeshKey;
    // Kept in double: mWorldTo* are PxMat44d, so the world inverses are stored
    // without the float narrowing the Gf-typed fields used to force.
    internalBody->mWorldToSimMesh = worldToSimMesh;
    internalBody->mWorldToCollMesh = worldToCollMesh;
    internalBody->mIsKinematic = desc.kinematicBody;
    internalBody->mBodyMass = desc.mass;

    // parse mesh data
    std::vector<carb::Float3> simMeshPoints;
    std::vector<carb::Float3> simMeshBindPoints;
    std::vector<carb::Float3> simMeshVelocities;
    std::vector<carb::Float3> collMeshPoints;
    std::vector<carb::Float3> allSkinMeshPoints;
    std::vector<carb::Float3> allSkinMeshBindPointsWorld;
    size_t numAllSkinMeshPoints;

    {
        getArrayValue(attachedStage, simMeshKey, tok.points,
                      omni::physics::parse::ReadTime::defaultTime(), simMeshPoints);
        getArrayValue(attachedStage, simMeshKey, tok.velocities,
                      omni::physics::parse::ReadTime::defaultTime(), simMeshVelocities);
        if (desc.simMeshKey != desc.collisionMeshKey)
        {
            getArrayValue(attachedStage, collMeshKey, tok.points,
                          omni::physics::parse::ReadTime::defaultTime(), collMeshPoints);
        }
    }

    parseSimBindPoints(attachedStage, simMeshBindPoints, internalBody->mSimMeshKey, desc.simMeshBindPoseToken, simMeshPoints);

    PxMat44d cookingToWorld;
    PxMat44d simMeshToCooking;
    double cookingToWorldScale;
    cookingDataAsync->computeDeformableCookingTransform(&simMeshToCooking, &cookingToWorld, &cookingToWorldScale,
                                                        simMeshToWorld,
                                                        simMeshBindPoints.data(),
                                                        simMeshBindPoints.size());
    {
        // simMeshToCooking output excludes world scale, so adding it here.
        // Gf's `simMeshToCooking * s` on the shared sixteen doubles, spelled
        // through gfmath so the accumulation order does not move.
        simMeshToCooking = gfmath::multiply(simMeshToCooking, gfmath::setScale(cookingToWorldScale));
    }

    // collectSkinMeshes fills the body's skin-mesh ObjectKeys (plus the
    // ranges/transforms it collects 1:1) directly from the source, so the body
    // holds no UsdPrim. parseSkinMeshPoints reads points back through the keys.
    numAllSkinMeshPoints = collectSkinMeshes(attachedStage, internalBody->mSkinMeshKeys, internalBody->mSkinMeshRanges,
                                             internalBody->mWorldToSkinMeshTransforms, desc.skinGeomPaths);

    parseSkinMeshPoints(attachedStage, allSkinMeshPoints, internalBody->mSkinMeshKeys, numAllSkinMeshPoints);

    parseSkinBindPointsWorld(attachedStage, allSkinMeshBindPointsWorld, desc.skinGeomPaths,
                             desc.skinGeomBindPoseTokens, numAllSkinMeshPoints);

    PxTetrahedronMesh* collisionTetMesh = deformableVolumeMesh->getCollisionMesh();
    PxTetrahedronMesh* simulationTetMesh = deformableVolumeMesh->getSimulationMesh();
    CARB_ASSERT(collisionTetMesh);
    CARB_ASSERT(simulationTetMesh);

    PxTransform pxCookingToWorld;
    PxVec3 pxCookingToWorldScaleDir; //should be identity scale
    // The PhysX-semantics decomposition, not a gfmath one, and deliberately so:
    // cookingToWorld is rigid by construction and the result is narrowed to
    // float here, so this is not a cache input and does not need exact bits.
    // (It replaces toPhysX(PxTransform&, PxVec3&, GfMatrix4d), which factored
    // through GfTransform; on a rigid matrix the two agree to ~4e-8.)
    decomposeMatrix(pxCookingToWorld, pxCookingToWorldScaleDir, cookingToWorld);
    PxReal pxCookingToWorldScale = PxReal(cookingToWorldScale);

    internalBody->mNumSimMeshVertices = simulationTetMesh->getNbVertices();
    internalBody->mNumCollMeshVertices = collisionTetMesh->getNbVertices();
    internalBody->mNumSkinMeshVertices = uint32_t(numAllSkinMeshPoints);

    internalBody->mCollMeshSurfaceTriangles.resize(collMeshSurfaceTriangles.size());
    std::memcpy(internalBody->mCollMeshSurfaceTriangles.data(), collMeshSurfaceTriangles.data(),
                internalBody->mCollMeshSurfaceTriangles.size() * sizeof(carb::Uint3));

    for (const carb::Int3& tri : collMeshSurfaceTriangles)
    {
        if (tri.x >= (int)internalBody->mNumCollMeshVertices || tri.y >= (int)internalBody->mNumCollMeshVertices ||
            tri.z >= (int)internalBody->mNumCollMeshVertices)
        {
            CARB_LOG_WARN("createVolumeDeformableBody(): Each surface face vertex index of collision mesh should be smaller than mNumCollMeshVertices!");
            ICE_FREE(internalBody);
            return kInvalidObjectId;
        }
    }

    internalBody->mCollMeshSurfaceTriToTetMap.swap(collMeshSurfaceTriToTetMap);

    // Get assigned material
    ObjectId materialId = getDeformableMaterialId(*scene, desc);
    InternalDeformableMaterial* internalMaterial;
    PxDeformableVolumeMaterial* material = getVolumeDeformableBodyMaterial(internalMaterial, *scene, materialId);
    if (!material)
    {
        CARB_LOG_WARN("Failed to aquire material for volume deformable! Prim(%s)\n", attachedStage.textFor(bodyKey));
        ICE_FREE(internalBody);
        return kInvalidObjectId;
    }
    internalBody->mMaterialId = materialId;

    // Create physx deformable volume
    PxDeformableVolume* deformableVolume = physics->createDeformableVolume(*cudaContextManager);
    if (!deformableVolume)
    {
        CARB_LOG_WARN("Failed to create PxDeformableVolume! Prim(%s)\n", attachedStage.textFor(bodyKey));
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
            *scene, geometry, pxCookingToWorldScale, desc.restOffset, desc.contactOffset,
            attachedStage.textFor(bodyKey));
        shape->setContactOffset(contactOffset);
        shape->setRestOffset(restOffset);

        deformableVolume->attachShape(*shape);
        shape->release();
    }

    deformableVolume->attachSimulationMesh(*simulationTetMesh, *deformableVolumeMesh->getDeformableVolumeAuxData());
    if (!scene->getScene()->addActor(*deformableVolume))
    {
        CARB_LOG_WARN("Failed to add PxDeformableVolume to scene: %s\n", attachedStage.textFor(bodyKey));
        ICE_FREE(internalBody);
        deformableVolume->release();
        return kInvalidObjectId;
    }

    PxVec4* collMeshRestPositionH = nullptr;
    PxDeformableVolumeExt::allocateAndInitializeHostMirror(
        *deformableVolume, cudaContextManager, internalBody->mSimMeshPositionInvMassH,
        internalBody->mSimMeshVelocityH, internalBody->mCollMeshPositionInvMassH,
        collMeshRestPositionH);

    {
        float metersPerUnit = attachedStage.getSource()->getSourceUnits().metersPerUnit;
        float contactOffset = deformableVolume->getShape()->getContactOffset();
        float restOffset = deformableVolume->getShape()->getRestOffset();
        float selfCollisionFilterDistance = deriveSelfCollisionFilterDistance(desc.selfCollisionFilterDistance,
            restOffset, contactOffset, metersPerUnit, PhysXType::ePTDeformableVolume);
        deformableVolume->setSelfCollisionFilterDistance(selfCollisionFilterDistance);
        deformableVolume->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, !desc.selfCollision);
    }
    {
        deformableVolume->setSleepThreshold(desc.sleepThreshold);
        deformableVolume->setSettlingThreshold(desc.settlingThreshold);
        deformableVolume->setSettlingDamping(desc.settlingDamping);
    }
    deformableVolume->setDeformableBodyFlag(PxDeformableBodyFlag::eKINEMATIC, desc.kinematicBody);
    deformableVolume->setDeformableBodyFlag(PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD, desc.enableSpeculativeCCD);
    deformableVolume->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, desc.disableGravity);

    deformableVolume->setLinearDamping(desc.linearDamping);
    deformableVolume->setSolverIterationCounts(scene->getInternalScene()->clampPosIterationCount(desc.solverPositionIterationCount));

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
        CARB_LOG_WARN("Failed to aquire mass/density for volume deformable body! Prim(%s)\n", attachedStage.textFor(bodyKey));
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
        const carb::Float3& p = simMeshPoints[i];
        const PxVec3d simPosition = simMeshToWorld.transform(toPhysXd(p));
        internalBody->mSimMeshPositionInvMassH[i] = PxVec4(float(simPosition.x), float(simPosition.y),
                                                          float(simPosition.z), internalBody->mSimMeshPositionInvMassH[i].w);
    }

    //TODO fix. Velocities should be applied before PxDeformableVolumeExt::transform, and treated correctly according to
    //local transform or world transform depending on configuration
    if (!desc.startsAsleep && simMeshVelocities.size() > 0 &&
        simMeshVelocities.size() == internalBody->mNumSimMeshVertices)
    {
        copyBuffer(internalBody->mSimMeshVelocityH, simMeshVelocities.data(),
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

    computePointExtent(collMeshPoints.empty() ? simMeshPoints : collMeshPoints,
                       internalBody->mCollMeshExtentSaveRestoreBuf);

    ObjectId objectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
        ePTDeformableVolume, deformableVolume, internalBody, bodyKey);

    deformableVolume->userData = (void*)objectId;
    if (mExposePrimNames)
        deformableVolume->setName(attachedStage.textFor(bodyKey));

    if (internalBody->mSimMeshKey != internalBody->mBodyKey)
    {
        //registering sub objects that need change handling 
        ObjectId simulationId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
            ePTDeformableVolume, deformableVolume, internalBody, simMeshKey);
        attachedStage.getObjectDatabase()->findOrCreateEntry(
            simMeshKey, attachedStage.textFor(simMeshKey), eVolumeDeformableBody, simulationId);
    }
    if (internalBody->mCollMeshKey != internalBody->mSimMeshKey)
    {
        //registering sub objects that need change handling
        ObjectId collisionId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
            ePTDeformableVolume, deformableVolume, internalBody, collMeshKey);
        attachedStage.getObjectDatabase()->findOrCreateEntry(
            collMeshKey, attachedStage.textFor(collMeshKey), eVolumeDeformableBody, collisionId);
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
            guideVertices[i] = toPhysXf(gfmath::transformPoint(simMeshToCooking, toPhysXd(simMeshBindPoints[i])));
        }

        const PxMat44d worldToCooking = gfmath::inverse(cookingToWorld);
        PxArray<PxVec3> embeddedVertices((uint32_t)allSkinMeshBindPointsWorld.size());
        for (uint32_t i = 0; i < embeddedVertices.size(); ++i)
        {
            embeddedVertices[i] =
                toPhysXf(gfmath::transformPoint(worldToCooking, toPhysXd(allSkinMeshBindPointsWorld[i])));
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

    scene->getInternalScene()->mVolumeDeformableBodies.push_back(internalBody);
    return objectId;
}

ObjectId PhysXUsdPhysicsInterface::createSurfaceDeformableBody(usdparser::AttachedStage& attachedStage,
    omni::physics::parse::ObjectKey bodyKey, PhysxSurfaceDeformableBodyDesc const& desc)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PhysXScene* scene = physxSetup.getPhysXScene(desc.sceneId);
    if (!scene || !scene->isFullGpuPipelineAvailable())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            usdparser::ErrorCode::eError,
            "Deformable Body feature is only supported on GPU. Please enable GPU dynamics flag in Property/Scene of physics scene!");
        return kInvalidObjectId;
    }

    if (!checkScenes())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            usdparser::ErrorCode::eError, "No physics scene created, please add physics scene into stage!");
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

    const omni::physics::parse::ObjectKey simMeshKey = desc.simMeshKey;
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (desc.collisionMeshKey != desc.simMeshKey)
    {
        CARB_LOG_WARN("No support for PhysX surface deformables with separate collision meshes. %s",
            attachedStage.textFor(desc.collisionMeshKey));
        return kInvalidObjectId;
    }

    if (!src || !src->exists(bodyKey))
    {
        CARB_LOG_WARN("PhysX Deformable Body creation failed due to invalid deformable prims.");
        return kInvalidObjectId;
    }

    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);

    if (!src->isA(simMeshKey, tok.meshType))
    {
        CARB_LOG_WARN("PhysX Deformable Body creation failed due to invalid deformable prims.");
        return kInvalidObjectId;
    }

    if (!sourceHasAPI(attachedStage, simMeshKey, tok.OmniPhysicsSurfaceDeformableSimAPI))
    {
        CARB_LOG_WARN("PhysX Deformable Body creation failed due to missing SurfaceDeformableSimAPI on simulation mesh, %s.",
                      attachedStage.textFor(simMeshKey));
        return kInvalidObjectId;
    }

    const PxMat44d simMeshToWorld = sourceLocalToWorld(attachedStage, simMeshKey);
    const PxMat44d worldToSimMesh = affineInverse(simMeshToWorld);

    // sync mesh generation
    if (desc.hasAutoAPI)
    {
        cookingDataAsync->cookSurfaceDeformableBody(desc, bodyKey, attachedStage, false);
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
            ePTDeformableSurface, nullptr, internalBody, bodyKey);
        scene->getInternalScene()->mSurfaceDeformableBodies.push_back(internalBody);
        return objectId;
    }

    internalBody->mBodyKey = bodyKey;
    internalBody->mSimMeshKey = simMeshKey;
    // Kept in double -- see the volume-deformable equivalent.
    internalBody->mWorldToSimMesh = worldToSimMesh;
    internalBody->mIsKinematic = desc.kinematicBody;
    internalBody->mBodyMass = desc.mass;

    // parse mesh data
    std::vector<carb::Float3> simMeshPoints;
    std::vector<carb::Float3> simMeshBindPoints;
    std::vector<carb::Float3> simMeshVelocities;
    std::vector<int32_t> simMeshIndices;
    std::vector<carb::Float3> simMeshRestShapePoints;
    std::vector<carb::Int3> simMeshRestTriVtxIndices;

    std::vector<carb::Float3> allSkinMeshPoints;
    std::vector<carb::Float3> allSkinMeshBindPointsWorld;
    size_t numAllSkinMeshPoints;

    {
        getArrayValue(attachedStage, simMeshKey, tok.points,
                      omni::physics::parse::ReadTime::defaultTime(), simMeshPoints);
        getArrayValue(attachedStage, simMeshKey, tok.velocities,
                      omni::physics::parse::ReadTime::defaultTime(), simMeshVelocities);
    }

    parseSimBindPoints(attachedStage, simMeshBindPoints, internalBody->mSimMeshKey, desc.simMeshBindPoseToken, simMeshPoints);

    getArrayValue(attachedStage, simMeshKey, tok.faceVertexIndices,
                  omni::physics::parse::ReadTime::defaultTime(), simMeshIndices);

    // Read rest shape
    {
        getArrayValue(attachedStage, simMeshKey, tok.omniphysicsRestShapePoints,
                      omni::physics::parse::ReadTime::defaultTime(), simMeshRestShapePoints);
        getArrayValue(attachedStage, simMeshKey, tok.omniphysicsRestTriVtxIndices,
                      omni::physics::parse::ReadTime::defaultTime(), simMeshRestTriVtxIndices);

        bool mismatch = simMeshRestShapePoints.size() != simMeshPoints.size() ||
                        simMeshRestTriVtxIndices.size() * 3 != simMeshIndices.size() ||
                        std::memcmp(simMeshRestTriVtxIndices.data(), simMeshIndices.data(),
                                    sizeof(int32_t) * simMeshIndices.size()) != 0;

        if (mismatch)
        {
            CARB_LOG_WARN("Surface deformable body creation failed. Mismatch between OmniPhysicsSurfaceDeformableSimAPI "
                          "rest shape attributes and UsdGeomMesh topology detected, %s",
                          attachedStage.textFor(simMeshKey));
            ICE_FREE(internalBody);
            return kInvalidObjectId;
        }
    }

    PxMat44d cookingToWorld;
    PxMat44d simMeshToCooking;
    double cookingToWorldScale;
    cookingDataAsync->computeDeformableCookingTransform(&simMeshToCooking, &cookingToWorld, &cookingToWorldScale,
                                                        simMeshToWorld,
                                                        simMeshBindPoints.data(),
                                                        simMeshBindPoints.size());
    {
        // simMeshToCooking output excludes world scale, so adding it here.
        // Gf's `simMeshToCooking * s` on the shared sixteen doubles, spelled
        // through gfmath so the accumulation order does not move.
        simMeshToCooking = gfmath::multiply(simMeshToCooking, gfmath::setScale(cookingToWorldScale));
    }

    // collectSkinMeshes fills the body's skin-mesh ObjectKeys (plus the
    // ranges/transforms it collects 1:1) directly from the source, so the body
    // holds no UsdPrim. parseSkinMeshPoints reads points back through the keys.
    numAllSkinMeshPoints = collectSkinMeshes(attachedStage, internalBody->mSkinMeshKeys, internalBody->mSkinMeshRanges,
                                             internalBody->mWorldToSkinMeshTransforms, desc.skinGeomPaths);

    parseSkinMeshPoints(attachedStage, allSkinMeshPoints, internalBody->mSkinMeshKeys, numAllSkinMeshPoints);

    parseSkinBindPointsWorld(attachedStage, allSkinMeshBindPointsWorld, desc.skinGeomPaths,
                             desc.skinGeomBindPoseTokens, numAllSkinMeshPoints);

    // Get assigned material
    ObjectId materialId = getDeformableMaterialId(*scene, desc);
    InternalDeformableMaterial* internalMaterial;
    PxDeformableSurfaceMaterial* material = getSurfaceDeformableBodyMaterial(internalMaterial, *scene, materialId);
    if (!material)
    {
        CARB_LOG_WARN("Failed to aquire material for surface deformable body! Prim(%s)\n", attachedStage.textFor(bodyKey));
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
        std::vector<PxVec3> positions;
        positions.resize(simMeshRestShapePoints.size());
        for (PxU32 i = 0; i < positions.size(); ++i)
        {
            const carb::Float3& simRestPos = simMeshRestShapePoints[i];
            positions[i] = toPhysXf(gfmath::transformPoint(simMeshToCooking, toPhysXd(simRestPos)));
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
            CARB_LOG_WARN("PxValidateTriangleMesh for PxDeformableSurface failed, %s", attachedStage.textFor(simMeshKey));
            ICE_FREE(internalBody);
            return kInvalidObjectId;
        }

        // cooking without cleaning
        PxDefaultMemoryOutputStream writeBuffer;
        bool status = PxCookTriangleMesh(ckParams, meshDesc, writeBuffer);
        if (!status)
        {
            CARB_LOG_WARN("PxCookTriangleMesh for PxDeformableSurface failed, %s", attachedStage.textFor(simMeshKey));
            ICE_FREE(internalBody);
            return kInvalidObjectId;
        }

        PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
        triangleMesh = physxSetup.getPhysics()->createTriangleMesh(readBuffer);
        if (!triangleMesh)
        {
            CARB_LOG_WARN("PxTriangleMesh creation failed: %s\n", attachedStage.textFor(bodyKey));
            ICE_FREE(internalBody);
            return kInvalidObjectId;
        }

        if (triangleMesh->getNbVertices() != uint32_t(simMeshPoints.size()))
        {
            CARB_LOG_WARN("PxTriangleMesh vertices don't align with simulation mesh vertices: %s\n", attachedStage.textFor(bodyKey));
            ICE_FREE(internalBody);
            triangleMesh->release();
            return kInvalidObjectId;
        }

        deformableSurface = physxSetup.getPhysics()->createDeformableSurface(*cudaContextManager);
        if (!deformableSurface)
        {
            CARB_LOG_WARN("Failed to create PxDeformableSurface: %s\n", attachedStage.textFor(bodyKey));
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
                *scene, geometry, 1.0f, restOffset, contactOffset, attachedStage.textFor(bodyKey));
            shape->setContactOffset(contactOffset);
            shape->setRestOffset(restOffset);

            deformableSurface->attachShape(*shape);
            shape->release();
        }

        if (!scene->getScene()->addActor(*deformableSurface))
        {
            CARB_LOG_WARN("Failed to add PxDeformableSurface to scene: %s\n", attachedStage.textFor(bodyKey));
            ICE_FREE(internalBody);
            deformableSurface->release();  // also releases attached shape and triangleMesh
            return kInvalidObjectId;
        }
    }

    {
        float metersPerUnit = attachedStage.getSource()->getSourceUnits().metersPerUnit;
        float contactOffset = deformableSurface->getShape()->getContactOffset();
        float restOffset = deformableSurface->getShape()->getRestOffset();
        float selfCollisionFilterDistance = deriveSelfCollisionFilterDistance(desc.selfCollisionFilterDistance,
            restOffset, contactOffset, metersPerUnit, PhysXType::ePTDeformableSurface);
        deformableSurface->setSelfCollisionFilterDistance(selfCollisionFilterDistance);
        deformableSurface->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, !desc.selfCollision);
    }
    {
        deformableSurface->setSleepThreshold(desc.sleepThreshold);
        deformableSurface->setSettlingThreshold(desc.settlingThreshold);
        deformableSurface->setSettlingDamping(desc.settlingDamping);
    }

    deformableSurface->setDeformableBodyFlag(PxDeformableBodyFlag::eKINEMATIC, desc.kinematicBody);
    deformableSurface->setDeformableBodyFlag(PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD, desc.enableSpeculativeCCD);
    deformableSurface->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, desc.disableGravity);

    deformableSurface->setLinearDamping(desc.linearDamping);
    deformableSurface->setSolverIterationCounts(scene->getInternalScene()->clampPosIterationCount(desc.solverPositionIterationCount));

    // bending
    bool enableFlattening = src && (desc.restBendAnglesDefault == tok.flatDefault);
    deformableSurface->setDeformableSurfaceFlag(PxDeformableSurfaceFlag::eENABLE_FLATTENING, enableFlattening);

    // collision substepping
    deformableSurface->setNbCollisionPairUpdatesPerTimestep(desc.collisionPairUpdateFrequency);
    deformableSurface->setNbCollisionSubsteps(desc.collisionIterationMultiplier);

    // velocity clamping
    deformableSurface->setMaxLinearVelocity(desc.maxLinearVelocity);
    deformableSurface->setMaxDepenetrationVelocity(desc.maxDepenetrationVelocity);

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
        const carb::Float3& p = simMeshPoints[i];
        const carb::Float3& r = simMeshRestShapePoints[i];
        const PxVec3d worldPos = simMeshToWorld.transform(toPhysXd(p));
        const PxVec3d worldRest = simMeshToWorld.transform(toPhysXd(r));
        internalBody->mSimMeshPositionInvMassH[i] =
            PxVec4(float(worldPos.x), float(worldPos.y), float(worldPos.z), 0.0f);
        simMeshRestPositionH[i] = PxVec4(float(worldRest.x), float(worldRest.y), float(worldRest.z), 0.0f);
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
        CARB_LOG_WARN("Failed to aquire mass/density for surface deformable body! Prim(%s)\n", attachedStage.textFor(bodyKey));
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

    // not a typo, we use the sim mesh points for bounds. TODO check for all types what the purpose is of these bounds and on
    // which prims they should be set and updated.
    computePointExtent(simMeshPoints, internalBody->mSimMeshExtentSaveRestoreBuf);

    ObjectId objectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
        ePTDeformableSurface, deformableSurface, internalBody, bodyKey);

    deformableSurface->userData = (void*)objectId;
    if (mExposePrimNames)
        deformableSurface->setName(attachedStage.textFor(bodyKey));

    if (internalBody->mSimMeshKey != internalBody->mBodyKey)
    {
        //registering sub objects that need change handling
        ObjectId simulationId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
            ePTDeformableSurface, deformableSurface, internalBody, simMeshKey);
        attachedStage.getObjectDatabase()->findOrCreateEntry(
            simMeshKey, attachedStage.textFor(simMeshKey), eSurfaceDeformableBody, simulationId);
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
            guideVertices[i] = toPhysXf(gfmath::transformPoint(simMeshToCooking, toPhysXd(simMeshBindPoints[i])));
        }

        const PxMat44d worldToCooking = gfmath::inverse(cookingToWorld);
        PxArray<PxVec3> embeddedVertices((uint32_t)allSkinMeshBindPointsWorld.size());
        for (uint32_t i = 0; i < embeddedVertices.size(); ++i)
        {
            embeddedVertices[i] =
                toPhysXf(gfmath::transformPoint(worldToCooking, toPhysXd(allSkinMeshBindPointsWorld[i])));
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

    scene->getInternalScene()->mSurfaceDeformableBodies.push_back(internalBody);
    return objectId;
}


} // namespace physx
} // namespace omni
