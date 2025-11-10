// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include "UsdInterface.h"

#include <private/omni/physx/PhysxUsd.h>

#include <usdLoad/LoadUsd.h>
#include <usdLoad/Mass.h>

#include <internal/InternalScene.h>
#include <internal/InternalDeformableDeprecated.h>
#include <internal/InternalTools.h>

#include <PhysXTools.h>
#include <CookingDataAsync.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <common/utilities/MemoryMacros.h>

#include "extensions/PxSoftBodyExt.h"
#include "extensions/PxDeformableSurfaceExt.h"

using namespace pxr;
using namespace ::physx;
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

PxDeformableSurface* createPhysxDeformableSurface(
    FEMClothDesc const& femClothDesc,
    PxFEMParameters& params,
    PxDeformableSurfaceMaterial*& deformableSurfaceMaterial,
    PxTriangleMesh*& triangleMesh,
    const GfMatrix4d& collisionToWorld,
    const PhysXScene* defaultScene)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxCudaContextManager* cudaContextManager = physxSetup.getCudaContextManager();
    if (!cudaContextManager)
    {
        return nullptr;
    }

    PxCookingParams ckParams = OmniPhysX::getInstance().getPhysXSetup().getDefaultCookingParams();
    ckParams.buildTriangleAdjacencies = false;
    ckParams.buildGPUData = true;
    ckParams.midphaseDesc = PxMeshMidPhase::eBVH34;
    ckParams.meshPreprocessParams |= PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES;
    ckParams.meshPreprocessParams |= PxMeshPreprocessingFlag::eENABLE_VERT_MAPPING;
    ckParams.meshPreprocessParams |= PxMeshPreprocessingFlag::eWELD_VERTICES;
    ckParams.meshWeldTolerance = 0.001f;

    // transform mesh
    std::vector<GfVec3f> restPositions;

    restPositions.resize(femClothDesc.restPoints.size());
    for (PxU32 i = 0; i < femClothDesc.restPoints.size(); ++i)
    {
        GfVec3f restPos = { femClothDesc.restPoints[i].x, femClothDesc.restPoints[i].y,
                                 femClothDesc.restPoints[i].z };
        restPositions[i] = collisionToWorld.Transform(restPos);
    }

    // cook triangle mesh
    PxTriangleMeshDesc meshDesc;

    meshDesc.points.count = (PxU32)femClothDesc.restPoints.size();
    meshDesc.triangles.count = (PxU32)femClothDesc.simulationIndices.size() / 3;
    meshDesc.points.stride = sizeof(float) * 3;
    meshDesc.triangles.stride = sizeof(int) * 3;
    meshDesc.points.data = restPositions.data(); // femClothDesc.collisionMeshRestPositions.data();
    meshDesc.triangles.data = femClothDesc.simulationIndices.data();

    PxDefaultMemoryOutputStream writeBuffer;
    bool status = PxCookTriangleMesh(ckParams, meshDesc, writeBuffer);
    PX_ASSERT(status);

    PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
    triangleMesh = physxSetup.getPhysics()->createTriangleMesh(readBuffer);

    PxDeformableSurface* deformableSurface = nullptr;

    if (triangleMesh)
    {
        deformableSurface = physxSetup.getPhysics()->createDeformableSurface(*cudaContextManager);
        if (deformableSurface)
        {
            PxShapeFlags shapeFlags =
                PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;

            PxDeformableSurfaceMaterial* materialPtr = deformableSurfaceMaterial;
            PxTriangleMeshGeometry geometry(triangleMesh);
            PxShape* shape = physxSetup.getPhysics()->createShape(geometry, &materialPtr, 1, true, shapeFlags);
            if (shape)
            {
                deformableSurface->attachShape(*shape);
            }
            shape->release();

            defaultScene->getScene()->addActor(*deformableSurface);
        }
    }

    // PxFEMParameters: same setting as softBody
    params.velocityDamping = femClothDesc.velocityDamping;
    params.settlingThreshold = femClothDesc.settlingThreshold;
    const bool disableSleeping = OmniPhysX::getInstance().getCachedSettings().disableSleeping;
    params.sleepThreshold = disableSleeping ? 0.0f : femClothDesc.sleepThreshold;
    params.sleepDamping = femClothDesc.sleepDamping;

    return deformableSurface;
}

void getDeformableMassDensity(const UsdPrim& prim, const DeformableDesc* desc, const float materialDensity, float& mass, float& density)
{
    float defaultDensity = 1000.0f;
    if (prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
    {
        CARB_ASSERT(desc->type == eFEMCloth);
        defaultDensity = 100.0f;
    }

    density = materialDensity;
    mass = desc->mass;
    if (desc->density > 0.0f)
    {
        density = desc->density;
    }

    if (mass <= 0.0f && density <= 0.0f)
    {
        // set default if no mass specified and both massAPI and material density are invalid/not provided:
        density = getScaledDensity(OmniPhysX::getInstance().getStage(), defaultDensity);
    }
}

} // namespace

namespace omni
{
namespace physx
{

extern bool checkScenes();
extern bool isRenderable(const UsdPrim& prim);

bool PhysXUsdPhysicsInterface::updateDeformableBodyMassDeprecated(const SdfPath& path, ObjectId objectId, const SoftBodyDesc& softBodyDesc)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);
    if (!objectFullRecord || !objectFullRecord->mPtr)
        return true;

    if (internalType == ePTSoftBodyDeprecated)
    {
        PxSoftBody* softBody = (PxSoftBody*)objectFullRecord->mPtr;
        InternalDeformableBodyDeprecated* internalDeformableBody = (InternalDeformableBodyDeprecated*)objectFullRecord->mInternalPtr;
        if (softBody && internalDeformableBody)
        {
            UsdPrim usdPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(path);

            InternalDeformableMaterial* intMat = getInternalPtr<InternalDeformableMaterial>(
                ePTSoftBodyMaterialDeprecated, internalDeformableBody->mMaterialId);

            const PxReal maxInvMass = 50.f;
            const PxReal lowerBoundMass = 1e-16f; // mass cannot be set to 0, but user-defined values are >= 0, so clamp it reasonably close to zero

            float materialDensity = intMat ? intMat->mDensity : 0.0f;
            float density;
            float mass;
            getDeformableMassDensity(usdPrim, &softBodyDesc, materialDensity, mass, density);

            if (mass > 0.0f)
            {
                PxSoftBodyExt::setMass(*softBody, fmaxf(mass, lowerBoundMass), maxInvMass, internalDeformableBody->mSimPositionInvMassH);
            }
            else if (intMat)
            {
                PxSoftBodyExt::updateMass(*softBody, density, maxInvMass, internalDeformableBody->mSimPositionInvMassH);
            }
            else
            {
                CARB_LOG_ERROR("updateDeformableBodyMass: %s is missing an deformable material.", path.GetString().c_str());
            }

            PxTetrahedronMesh* simulationTetMesh = internalDeformableBody->mSoftBodyMesh->getSimulationMesh();

            std::vector<float> invMassScale;
            invMassScale.resize(internalDeformableBody->mNumSimMeshVertices, 1.0f);

            //remap softBodyDesc.invMassScale to sim vertices if collisionVertexToSimulationTetIndices available
            if (!softBodyDesc.collisionVertexToSimulationTetIndices.empty() && !softBodyDesc.invMassScale.empty())
            {
                const uint32_t* simTetIndices = reinterpret_cast<const uint32_t*>(simulationTetMesh->getTetrahedrons());
                uint32_t numSimTets = simulationTetMesh->getNbTetrahedrons();

                for (PxU32 i = 0; i < internalDeformableBody->mNumCollMeshVertices; ++i)
                {
                    uint32_t simTetId = softBodyDesc.collisionVertexToSimulationTetIndices[i];
                    if (simTetId < numSimTets)
                    {
                        const uint32_t* idxPtr = simTetIndices + simTetId*4;
                        for (uint32_t j = 0; j < 4; j++)
                            invMassScale[idxPtr[j]] = softBodyDesc.invMassScale[i];
                    }
                }
            }

            PxVec4* simPositionInvMass = internalDeformableBody->mSimPositionInvMassH;
            for (unsigned int i = 0; i < internalDeformableBody->mNumSimMeshVertices; ++i)
            {
                simPositionInvMass[i] = PxVec4(simPositionInvMass[i].getXYZ(), simPositionInvMass[i].w * invMassScale[i]);
            }

            PxSoftBodyDataFlags flags = PxSoftBodyDataFlags(0);
            flags.raise(PxSoftBodyDataFlag::eSIM_POSITION_INVMASS);
            flags.raise(PxSoftBodyDataFlag::ePOSITION_INVMASS);

            PxSoftBodyExt::copyToDevice(*softBody, flags,  internalDeformableBody->mSimPositionInvMassH,
                internalDeformableBody->mSimVelocityH, internalDeformableBody->mCollPositionInvMassH,
                internalDeformableBody->mCollRestPositionH, internalDeformableBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
        }
    }

    return true;
}

bool PhysXUsdPhysicsInterface::updateDeformableBodyPositionsDeprecated(const usdparser::AttachedStage& attachedStage, const ObjectId objectId)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);

    if (!objectFullRecord || !objectFullRecord->mPtr)
    {
        return true;
    }

    if (internalType == ePTSoftBodyDeprecated)
    {
        UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(objectFullRecord->mPath);

        PxSoftBody* softBody = (PxSoftBody*)objectFullRecord->mPtr;
        InternalDeformableBodyDeprecated* internalDeformableBody = (InternalDeformableBodyDeprecated*)objectFullRecord->mInternalPtr;

        if (softBody &&
            internalDeformableBody &&
            prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
        {
            GfMatrix4d transform = UsdGeomXform(prim).ComputeLocalToWorldTransform(UsdTimeCode::Default());

            VtArray<GfVec3f> points;
            PhysxSchemaPhysxDeformableBodyAPI deformableBody(prim);
            deformableBody.GetSimulationPointsAttr().Get(&points);

            if (points.size() != internalDeformableBody->mNumSimMeshVertices)
            {
                CARB_LOG_WARN("Size of physxDeformable:simulationPoints of %s has changed - skipping update.", prim.GetPath().GetText());
                return true;
            }

            PxVec4* simPositionInvMass = internalDeformableBody->mSimPositionInvMassH;
            for (unsigned int i = 0; i < internalDeformableBody->mNumSimMeshVertices; ++i)
            {
                GfVec3d localPoint = GfVec3d(points[i][0], points[i][1], points[i][2]);
                GfVec3d worldPoint = transform.Transform(localPoint);

                simPositionInvMass[i] = PxVec4((float)worldPoint[0], (float)worldPoint[1], (float)worldPoint[2], simPositionInvMass[i].w);
            }

            PxSoftBodyDataFlags flags = PxSoftBodyDataFlags(0);
            flags.raise(PxSoftBodyDataFlag::eSIM_POSITION_INVMASS);
            flags.raise(PxSoftBodyDataFlag::ePOSITION_INVMASS);

            PxSoftBodyExt::copyToDevice(*softBody, flags, internalDeformableBody->mSimPositionInvMassH,
                internalDeformableBody->mSimVelocityH, internalDeformableBody->mCollPositionInvMassH,
                internalDeformableBody->mCollRestPositionH, internalDeformableBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
        }
    }

    return true;
}

bool PhysXUsdPhysicsInterface::updateDeformableBodyVelocitiesDeprecated(const usdparser::AttachedStage& attachedStage, const ObjectId objectId)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);

    if (!objectFullRecord || !objectFullRecord->mPtr)
    {
        return true;
    }

    if (internalType == ePTSoftBodyDeprecated)
    {
        UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(objectFullRecord->mPath);

        PxSoftBody* softBody = (PxSoftBody*)objectFullRecord->mPtr;
        InternalDeformableBodyDeprecated* internalDeformableBody = (InternalDeformableBodyDeprecated*)objectFullRecord->mInternalPtr;

        if (softBody &&
            internalDeformableBody &&
            prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
        {
            VtArray<GfVec3f> velocities;
            PhysxSchemaPhysxDeformableAPI deformable(prim);
            deformable.GetSimulationVelocitiesAttr().Get(&velocities);

            if (velocities.size() != internalDeformableBody->mNumSimMeshVertices)
            {
                CARB_LOG_WARN("Size of physxDeformable:simulationVelocities of %s has changed - skipping update.", prim.GetPath().GetText());
                return true;
            }

            PxVec4* simVelocity = internalDeformableBody->mSimVelocityH;
            for (unsigned int i = 0; i < internalDeformableBody->mNumSimMeshVertices; ++i)
            {
                simVelocity[i] = PxVec4(velocities[i][0], velocities[i][1], velocities[i][2], simVelocity[i].w);
            }

            PxSoftBodyDataFlags flags = PxSoftBodyDataFlags(0);
            flags.raise(PxSoftBodyDataFlag::eSIM_VELOCITY);

            PxSoftBodyExt::copyToDevice(*softBody, flags, internalDeformableBody->mSimPositionInvMassH,
                internalDeformableBody->mSimVelocityH, internalDeformableBody->mCollPositionInvMassH,
                internalDeformableBody->mCollRestPositionH, internalDeformableBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());
        }
    }

    return true;
}

ObjectId getDeformableMaterialIdDeprecated(const PhysXScene& scene, const DeformableDesc& deformableDesc)
{
    ObjectId materialId = kInvalidObjectId;
    if (deformableDesc.materials.size() == 1 && deformableDesc.materials[0] != kInvalidObjectId)
    {
        materialId = deformableDesc.materials[0];
    }
    else
    {
        // Scene might have a default material with InternalDeformableMaterial wrapper and valid ObjectId
        PxFEMMaterial* material = nullptr;
        if (deformableDesc.type == eSoftBody)
        {
            material = scene.getDefaultFEMSoftBodyMaterialDeprecated();
        }
        else if (deformableDesc.type == eFEMCloth)
        {
            material = scene.getDefaultDeformableSurfaceMaterialDeprecated();
        }

        if (material && material->userData)
        {
            materialId = (size_t)material->userData;
        }
    }
    return materialId;
}

PxFEMSoftBodyMaterial* getDeformableBodyMaterialDeprecated(InternalDeformableMaterial** internalMaterial,
    const PhysXScene& scene, ObjectId materialId)
{
    if (internalMaterial)
    {
        *internalMaterial = getInternalPtr<InternalDeformableMaterial>(ePTSoftBodyMaterialDeprecated, materialId);
    }
    PxFEMSoftBodyMaterial* material = getPtr<PxFEMSoftBodyMaterial>(ePTSoftBodyMaterialDeprecated, materialId);
    if (!material)
    {
        // Default material migth not have InternalDeformableMaterial wrapper and valid ObjectId
        material = scene.getDefaultFEMSoftBodyMaterialDeprecated();
    }
    return material;
}

PxDeformableSurfaceMaterial* getDeformableSurfaceMaterialDeprecated(InternalDeformableMaterial** internalMaterial,
    const PhysXScene& scene, ObjectId materialId)
{
    if (internalMaterial)
    {
        *internalMaterial = getInternalPtr<InternalDeformableMaterial>(ePTFEMClothMaterialDeprecated, materialId);
    }
    PxDeformableSurfaceMaterial* material = getPtr<PxDeformableSurfaceMaterial>(ePTFEMClothMaterialDeprecated, materialId);
    if (!material)
    {
        // Default material migth not have InternalDeformableMaterial wrapper and valid ObjectId
        material = scene.getDefaultDeformableSurfaceMaterialDeprecated();
    }
    return material;
}

ObjectId PhysXUsdPhysicsInterface::createDeformableBodyDeprecated(const usdparser::AttachedStage& attachedStage, const SdfPath& path, SoftBodyDesc const& softBodyDesc)
{
    if (!checkScenes())
    {
        PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, "No physics scene created, please add physics scene into stage!");
        return kInvalidObjectId;
    }

    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxPhysics* physics = physxSetup.getPhysics();

    const ObjectId sceneId = attachedStage.getObjectDatabase()->findEntry(softBodyDesc.scenePath, eScene);
    PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);
    if (!physxScene || !physxScene->isFullGpuPipelineAvailable())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            ErrorCode::eError,
            "Deformable Body feature is only supported on GPU. Please enable GPU dynamics flag in Property/Scene of physics scene!");
        return kInvalidObjectId;
    }

    PxCudaContextManager* cudaContextManager = physxScene->getScene()->getCudaContextManager();
    if (!cudaContextManager)
    {
        CARB_LOG_ERROR("PhysX Deformable Body creation failed due to missing PxCudaContextManager.");
        return kInvalidObjectId;
    }

    if (softBodyDesc.deformableEnabled == false)
        return kInvalidObjectId;

    CookingDataAsync* cookingDataAsync = physxSetup.getCookingDataAsync();
    if (!cookingDataAsync)
        return kInvalidObjectId;

    // Get assigned material
    ObjectId materialId = getDeformableMaterialIdDeprecated(*physxScene, softBodyDesc);
    InternalDeformableMaterial* internalMaterial;
    PxFEMSoftBodyMaterial* material = getDeformableBodyMaterialDeprecated(&internalMaterial, *physxScene, materialId);
    if (!material)
    {
        return kInvalidObjectId;
    }

    float metersPerUnit = float(UsdGeomGetStageMetersPerUnit(attachedStage.getStage()));
    UsdPrim usdPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(path);
    bool shouldRender = isRenderable(usdPrim);

    // create internal deformable body, and store/copy some data

    // sanitize transforms to default scale->orient->translate
    InternalDeformableBodyDeprecated* internalDeformableBody = ICE_NEW(InternalDeformableBodyDeprecated)(physxScene, usdPrim);
    const GfMatrix4d softBodyToWorld(UsdGeomMesh(usdPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default()));

    // get cooked softbody mesh data (in 'softbody' space)
    PxDefaultMemoryOutputStream softBodyMeshDataOut;
    bool softBodyMeshCooked = cookingDataAsync->cookSoftBodyMeshDeprecated(softBodyMeshDataOut, softBodyDesc, usdPrim, false);
    PxSoftBodyMesh* softBodyMesh = nullptr;
    if (softBodyMeshCooked)
    {
        PxDefaultMemoryInputData softBodyMeshDataIn(softBodyMeshDataOut.getData(), softBodyMeshDataOut.getSize());
        softBodyMesh = cookingDataAsync->createSoftBodyMeshDeprecated(internalDeformableBody->mCollMeshSurfaceTriangles, softBodyMeshDataIn);
        if (!softBodyMesh)
        {
            CARB_LOG_ERROR("Failed to create soft body mesh from cooked data! Prim: %s\n", path.GetText());
        }
    }

    if (softBodyMesh)
    {
        // Store immediately - if there is an error it will be released with InternalDeformableBodyDeprecated
        internalDeformableBody->mSoftBodyMesh = softBodyMesh;

        PxTetrahedronMesh* collisionTetMesh = softBodyMesh->getCollisionMesh();
        PxTetrahedronMesh* simulationTetMesh = softBodyMesh->getSimulationMesh();
        CARB_ASSERT(collisionTetMesh);
        CARB_ASSERT(simulationTetMesh);
        if (collisionTetMesh->getNbTetrahedrons() > PX_MAX_NB_SOFTBODY_TET || simulationTetMesh->getNbTetrahedrons() > PX_MAX_NB_SOFTBODY_TET)
        {
            CARB_LOG_ERROR("Deformable body creation failed due to tetrahedral mesh exceeding maximum number of supported tets (%d). Try to adjust the simulation mesh resolution. Prim: %s\n",
                PX_MAX_NB_SOFTBODY_TET, path.GetText());
            SAFE_DELETE_SINGLE(internalDeformableBody);
            return kInvalidObjectId;
        }

        // copy the collision mesh data to buffer used for PxSoftBodyData::ePOSITION_INVMASS
        PxTransform pxSoftBodyToWorld;
        PxVec3 pxSoftBodyToWorldScale;
        toPhysX(pxSoftBodyToWorld, pxSoftBodyToWorldScale, softBodyToWorld);
        PxReal pxSoftBodyToWorldScaleNorm = pxSoftBodyToWorldScale.magnitude();

        const PxVec3* collisionVertices = (PxVec3*)&softBodyDesc.collisionPoints[0];
        const PxVec3* skinVertices = (PxVec3*)&softBodyDesc.points[0];

        PxBounds3 bounds = PxBounds3::empty();
        for (size_t i = 0; i < softBodyDesc.points.size(); ++i)
        {
            bounds.include(skinVertices[i]);
        }
        PxVec3 boundsSize = bounds.maximum - bounds.minimum;
        float meshSize = PxMax(boundsSize.x, PxMax(boundsSize.y, boundsSize.z));
        pxSoftBodyToWorldScaleNorm *= meshSize;

        // compute local to world and extract scale
        internalDeformableBody->mPrimToWorld = softBodyToWorld;

        //@sschirm, todo make this cleaner
        internalDeformableBody->mNumSkinMeshVertices = PxU32(softBodyDesc.points.size());
        internalDeformableBody->mNumCollSkinMeshVertices = collisionTetMesh->getNbVertices();
        internalDeformableBody->mNumCollMeshVertices = internalDeformableBody->mNumCollSkinMeshVertices - (softBodyDesc.kinematicBody ? 0 : internalDeformableBody->mNumSkinMeshVertices);
        internalDeformableBody->mNumSimMeshVertices = simulationTetMesh->getNbVertices();

        // Assign material
        internalDeformableBody->mMaterialId = materialId;

        // Create soft body
        PxSoftBody* softBody = physics->createSoftBody(*cudaContextManager);
        if (softBody)
        {
            // Store immediately - if there is an error it will be released with InternalDeformableBodyDeprecated
            internalDeformableBody->mSoftBody = softBody;

            softBody->setSoftBodyFlag(PxSoftBodyFlag::eDISABLE_SELF_COLLISION, !softBodyDesc.selfCollision);
            softBody->setSoftBodyFlag(PxSoftBodyFlag::ePARTIALLY_KINEMATIC, softBodyDesc.kinematicBody);

            internalDeformableBody->mIsPartiallyKinematic = softBodyDesc.kinematicBody;
            if (softBodyDesc.kinematicBody)
            {
                //TODO this is not okay, need to talk to twidmer
                material->setMaterialModel(PxFEMSoftBodyMaterialModel::eNEO_HOOKEAN);
            }

            PxShapeFlags shapeFlags = PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;

            if (OmniPhysX::getInstance().isDebugVisualizationEnabled())
                shapeFlags |= PxShapeFlag::eVISUALIZATION;

            PxTetrahedronMeshGeometry geometry(collisionTetMesh);
            PxShape* shape = physxSetup.getPhysics()->createShape(geometry, &material, 1, true, shapeFlags);
            if (shape)
            {
                const uint32_t collisionGroup = convertToCollisionGroup(softBodyDesc.collisionGroup);
                PxFilterData fd;
                convertCollisionGroupToPxFilterData(collisionGroup, fd);
                shape->setSimulationFilterData(fd);

                float restOffset = softBodyDesc.collisionRestOffset;
                if (isinf(restOffset))
                {
                    restOffset = 0.02f / metersPerUnit;
                }

                // TODO preist: Double-check if unit conversion is required here
                if (softBodyDesc.collisionContactOffset >= 0.0f)
                {
                    PxReal contactOffset = softBodyDesc.collisionContactOffset;
                    if (contactOffset < restOffset)
                        contactOffset += restOffset;
                    shape->setContactOffset(contactOffset);
                }
                else
                {
                    // Contact offset calculation is referenced from createShape() implementation
#if 1               // Comment out PxGeometryQuery::getWorldBounds() until the next PhysX SDK update that has the bounds calculation implemented for tetrahedrons (OM-24582)
                    PxBounds3 bounds;
                    PxGeometryQuery::computeGeomBounds(bounds, geometry, PxTransform(PxIdentity));
                    const PxVec3 extents = bounds.getDimensions().multiply(pxSoftBodyToWorldScale);
#else
                    // Let the tolerance determine the contact offset
                    const PxVec3 extents = PxVec3(PxZERO::PxZero);
#endif

                    const PxReal g = physxScene->getScene()->getGravity().magnitude();
                    const PxReal dt = 1.0f / physxScene->getTimeStepsPerSeconds();
                    const PxReal dynamicLowerThreshold = 2.0f * dt * dt * PxMax(g, 1.0f); //Make sure the lower bound is not exacly zero in case of zero gravity

                    const PxReal minContactOffset = extents.minElement() * 0.02f;
                    float contactOffset = fmaxf(dynamicLowerThreshold, minContactOffset);

                    if (restOffset > 0.0f)
                        contactOffset += restOffset;

                    shape->setContactOffset(contactOffset);
                }
                shape->setRestOffset(restOffset);
                if (!softBody->attachShape(*shape))
                {
                    CARB_LOG_WARN("PhysX Deformable Body creation failed.");
                    shape->release();
                    SAFE_DELETE_SINGLE(internalDeformableBody);
                    return kInvalidObjectId;
                }
                shape->release();
            }

            if (!softBody->attachSimulationMesh(*simulationTetMesh, *softBodyMesh->getSoftBodyAuxData()))
            {
                CARB_LOG_WARN("PhysX Deformable Body creation failed.");
                SAFE_DELETE_SINGLE(internalDeformableBody);
                return kInvalidObjectId;
            }
            physxScene->getScene()->addActor(*softBody);
        }

        PxSoftBodyExt::allocateAndInitializeHostMirror(*softBody, cudaContextManager, internalDeformableBody->mSimPositionInvMassH,
            internalDeformableBody->mSimVelocityH, internalDeformableBody->mCollPositionInvMassH, internalDeformableBody->mCollRestPositionH);

        PxFEMParameters params;
        params.velocityDamping = softBodyDesc.velocityDamping;
        params.settlingThreshold = softBodyDesc.settlingThreshold;
        const bool disableSleeping = OmniPhysX::getInstance().getCachedSettings().disableSleeping;
        params.sleepThreshold = disableSleeping ? 0.0f : softBodyDesc.sleepThreshold;
        params.sleepDamping = softBodyDesc.sleepDamping;

        float restOffset = softBody->getShape()->getRestOffset();

        float selfCollisionFilterDistance = softBodyDesc.selfCollisionFilterDistance;
        if (selfCollisionFilterDistance < 0.f)
        {
            selfCollisionFilterDistance = 2.5f * restOffset;
        }

        float selfCollisionFilterDistanceEps = 1.0e-5f / metersPerUnit;
        float minSelfCollisionFilterDistance = 2.0f * restOffset + selfCollisionFilterDistanceEps;
        params.selfCollisionFilterDistance = fmax(selfCollisionFilterDistance, minSelfCollisionFilterDistance);

        softBody->setParameter(params);
        softBody->setSoftBodyFlag(PxSoftBodyFlag::eENABLE_CCD, softBodyDesc.enableCCD);
        softBody->setSolverIterationCounts(physxScene->getInternalScene()->clampPosIterationCount(softBodyDesc.solverPositionIterations), 0);

        PxSoftBodyExt::transform(*softBody, pxSoftBodyToWorld, pxSoftBodyToWorldScaleNorm, internalDeformableBody->mSimPositionInvMassH,
            internalDeformableBody->mSimVelocityH, internalDeformableBody->mCollPositionInvMassH, internalDeformableBody->mCollRestPositionH);
 

        const PxReal maxInvMass = 50.f;
        const PxReal lowerBoundMass = 1e-16f; // mass cannot be set to 0, but user-defined values are >= 0, so clamp it reasonably close to zero

        float materialDensity = internalMaterial ? internalMaterial->mDensity : 0.0f;
        float density;
        float mass;
        getDeformableMassDensity(usdPrim, &softBodyDesc, materialDensity, mass, density);
        if (mass > 0.0f)
        {
            PxSoftBodyExt::setMass(*softBody, fmaxf(mass, lowerBoundMass), maxInvMass, internalDeformableBody->mSimPositionInvMassH);
        }
        else
        {
            PxSoftBodyExt::updateMass(*softBody, density, maxInvMass, internalDeformableBody->mSimPositionInvMassH);
        }

        PxVec4* positionInvMass = internalDeformableBody->mCollPositionInvMassH;
        const PxReal invMass = 1.0f; // does not do anything except it would lock if == 0.0f

        std::vector<float> invMassScale;
        invMassScale.resize(internalDeformableBody->mNumSimMeshVertices, 1.0f);

        //remap softBodyDesc.invMassScale to sim vertices if collisionVertexToSimulationTetIndices available
        if (!softBodyDesc.collisionVertexToSimulationTetIndices.empty() && !softBodyDesc.invMassScale.empty())
        {
            const uint32_t* simTetIndices = reinterpret_cast<const uint32_t*>(simulationTetMesh->getTetrahedrons());
            uint32_t numSimTets = simulationTetMesh->getNbTetrahedrons();

            for (PxU32 i = 0; i < internalDeformableBody->mNumCollMeshVertices; ++i)
            {
                uint32_t simTetId = softBodyDesc.collisionVertexToSimulationTetIndices[i];
                if (simTetId < numSimTets)
                {
                    const uint32_t* idxPtr = simTetIndices + simTetId*4;
                    for (uint32_t j = 0; j < 4; j++)
                        invMassScale[idxPtr[j]] = softBodyDesc.invMassScale[i];
                }
            }
        }

        for (PxU32 i = 0; i < internalDeformableBody->mNumCollMeshVertices; ++i)
        {
            PxVec3 position = collisionVertices[i].multiply(pxSoftBodyToWorldScale);
            position = pxSoftBodyToWorld.transform(position);
            positionInvMass[i] = PxVec4(position, invMass);
        }

        if (!internalDeformableBody->mIsPartiallyKinematic)
        {
            for (PxU32 i = 0; i < internalDeformableBody->mNumSkinMeshVertices; ++i)
            {
                PxVec3 position = skinVertices[i].multiply(pxSoftBodyToWorldScale);
                position = pxSoftBodyToWorld.transform(position);
                positionInvMass[i + internalDeformableBody->mNumCollMeshVertices] = PxVec4(position, invMass);
            }
        }

        PxVec4* simPositionInvMass = internalDeformableBody->mSimPositionInvMassH;
        for (PxU32 i = 0; i < internalDeformableBody->mNumSimMeshVertices; ++i)
        {
            PxVec3 simPosition = toPhysX(softBodyDesc.simulationPoints[i]).multiply(pxSoftBodyToWorldScale);
            simPosition = pxSoftBodyToWorld.transform(simPosition);
            simPositionInvMass[i] = PxVec4(simPosition, simPositionInvMass[i].w * invMassScale[i]);
        }

        if (softBodyDesc.simulationVelocities.size() > 0)
        {
            CARB_ASSERT(softBodyDesc.simulationVelocities.size() == internalDeformableBody->mNumSimMeshVertices);
            copyBuffer(internalDeformableBody->mSimVelocityH, &softBodyDesc.simulationVelocities[0], internalDeformableBody->mNumSimMeshVertices);
        }
        else
        {
            PxVec4* simVelocityInvMassCPU = internalDeformableBody->mSimVelocityH;
            for (uint32_t i = 0; i < internalDeformableBody->mNumSimMeshVertices; ++i)
            {
                simVelocityInvMassCPU[i].x = 0.0f;
                simVelocityInvMassCPU[i].y = 0.0f;
                simVelocityInvMassCPU[i].z = 0.0f;
            }
        }
        PxSoftBodyExt::copyToDevice(*softBody, PxSoftBodyDataFlag::eALL, internalDeformableBody->mSimPositionInvMassH,
            internalDeformableBody->mSimVelocityH, internalDeformableBody->mCollPositionInvMassH,
            internalDeformableBody->mCollRestPositionH, internalDeformableBody->mPhysXScene->getInternalScene()->getDeformableCopyStream());

        // Store initial properties for reset of deformables created during simulation
        bool resetsXformStack;
        UsdGeomXform(usdPrim).GetLocalTransformation(&internalDeformableBody->mInitialPrimToParent, &resetsXformStack, UsdTimeCode::Default());
        copyBuffer(internalDeformableBody->mInitialCollSkinMeshPositions, internalDeformableBody->mCollPositionInvMassH, internalDeformableBody->mNumCollSkinMeshVertices);
        copyBuffer(internalDeformableBody->mInitialSimMeshPositions, internalDeformableBody->mSimPositionInvMassH, internalDeformableBody->mNumSimMeshVertices);
        copyBuffer(internalDeformableBody->mInitialSimMeshVelocities, internalDeformableBody->mSimVelocityH, internalDeformableBody->mNumSimMeshVertices);

        UsdGeomBoundable::ComputeExtentFromPlugins(UsdGeomMesh(internalDeformableBody->mPrim), UsdTimeCode::Default(), &internalDeformableBody->mExtentSaveRestoreBuf);

        ObjectId objectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(ePTSoftBodyDeprecated, softBody, internalDeformableBody, usdPrim.GetPrimPath());
        if (internalMaterial)
        {
            internalMaterial->addDeformableId(objectId);
        }

        if (softBodyDesc.kinematicBody)
        {
            if (softBodyDesc.collisionVertexToSkinTriVertexIndices.size() > 0)
            {
                internalDeformableBody->mCollisionVertexToSkinTriVertexIndices.assign(
                    softBodyDesc.collisionVertexToSkinTriVertexIndices.begin(), softBodyDesc.collisionVertexToSkinTriVertexIndices.end());

                internalDeformableBody->mCollisionVertexToSkinTriBarycentrics.assign(
                    softBodyDesc.collisionVertexToSkinTriBarycentrics.begin(), softBodyDesc.collisionVertexToSkinTriBarycentrics.end());
            }

            //update kinematic targets once if not registered as an animated kinematic soft body
            if (attachedStage.getAnimatedKinematicDeformableBodies().count(usdPrim.GetPath()) == 0)
            {
                updateKinematicVertexTargetsFromSimDeprecated(usdPrim.GetPath(), objectId,
                    reinterpret_cast<carb::Float4*>(internalDeformableBody->mSimPositionInvMassH), internalDeformableBody->mNumSimMeshVertices);
            }
        }

        //add internal soft body to internal scene
        physxScene->getInternalScene()->mDeformableBodiesDeprecated.push_back(internalDeformableBody);

        return objectId;
    }
    else
    {
        SAFE_DELETE_SINGLE(internalDeformableBody);
    }

    return kInvalidObjectId;
}

bool PhysXUsdPhysicsInterface::updateDeformableSurfaceMassDeprecated(const SdfPath& path, ObjectId objectId, const usdparser::FEMClothDesc& clothDesc)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);

    if (!objectFullRecord || !objectFullRecord->mPtr)
    {
        return true;
    }

    if (internalType == ePTFEMClothDeprecated)
    {
        UsdPrim prim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(path);

        PxDeformableSurface* deformableSurface = (PxDeformableSurface*)objectFullRecord->mPtr;
        InternalDeformableSurfaceDeprecated* internalDeformableSurface = (InternalDeformableSurfaceDeprecated*)objectFullRecord->mInternalPtr;

        if (deformableSurface &&
            internalDeformableSurface &&
            prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
        {
            InternalDeformableMaterial* intMat;
            PxDeformableSurfaceMaterial* deformableSurfaceMaterial = getDeformableSurfaceMaterialDeprecated(
                &intMat, *internalDeformableSurface->mPhysXScene, internalDeformableSurface->mMaterialId);

            // cloth mass
            float vertexMassInv = 0.0f;

            float materialDensity = intMat ? intMat->mDensity : 0.0f;
            float density;
            float mass;
            getDeformableMassDensity(prim, &clothDesc, materialDensity, mass, density);

            if (mass > 0.0f)
            {
                vertexMassInv = internalDeformableSurface->mNumCollMeshVerticesWelded / mass;
            }
            else if (deformableSurfaceMaterial)
            {
                // use material thickness vs. keep it consistent with particle cloth.
                float clothThickness = std::max(deformableSurfaceMaterial->getThickness(), FLT_EPSILON);

                float totalArea = 0.0f;
                const std::vector<carb::Float3>& positions = clothDesc.restPoints;
                const std::vector<uint32_t>& triangleIndices = clothDesc.simulationIndices;

                for (int i = 0; i < triangleIndices.size(); i += 3)
                {
                    const uint32_t triIndex0 = triangleIndices[i];
                    const uint32_t triIndex1 = triangleIndices[i + 1];
                    const uint32_t triIndex2 = triangleIndices[i + 2];
                    totalArea += area(positions[triIndex0], positions[triIndex1], positions[triIndex2]);
                }

                vertexMassInv = internalDeformableSurface->mNumCollMeshVerticesWelded / (totalArea * density * clothThickness);
            }
            else
            {
                CARB_LOG_ERROR("updateDeformableSurfaceMass: %s is missing an deformable material.", path.GetString().c_str());
            }

            if (vertexMassInv > 0.0f)
            {
                PxVec4* simPositionInvMass = internalDeformableSurface->mPositionInvMassH;
                PxVec4* simVelocityInvMass = internalDeformableSurface->mVelocityH;

                for (unsigned int i = 0; i < internalDeformableSurface->mNumCollMeshVerticesWelded; ++i)
                {
                    simPositionInvMass[i] = PxVec4(simPositionInvMass[i].getXYZ(), vertexMassInv);
                    simVelocityInvMass[i] = PxVec4(simVelocityInvMass[i].getXYZ(), vertexMassInv);
                }

                PxDeformableSurfaceDataFlags flags = PxDeformableSurfaceDataFlags(0);
                flags.raise(PxDeformableSurfaceDataFlag::ePOSITION_INVMASS);
                flags.raise(PxDeformableSurfaceDataFlag::eVELOCITY);

                PxDeformableSurfaceExt::copyToDevice(*deformableSurface, flags, internalDeformableSurface->mNumCollMeshVerticesWelded,
                    internalDeformableSurface->mPositionInvMassH, internalDeformableSurface->mVelocityH, internalDeformableSurface->mRestPositionH,
                    internalDeformableSurface->mPhysXScene->getInternalScene()->getDeformableCopyStream());
            }
        }
    }

    return true;
}

bool PhysXUsdPhysicsInterface::updateDeformableSurfacePositionsDeprecated(const usdparser::AttachedStage& attachedStage, const ObjectId objectId)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);

    if (!objectFullRecord || !objectFullRecord->mPtr)
    {
        return true;
    }

    if (internalType == ePTFEMClothDeprecated)
    {
        UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(objectFullRecord->mPath);

        PxDeformableSurface* deformableSurface = (PxDeformableSurface*)objectFullRecord->mPtr;
        InternalDeformableSurfaceDeprecated* internalDeformableSurface = (InternalDeformableSurfaceDeprecated*)objectFullRecord->mInternalPtr;

        if (deformableSurface &&
            internalDeformableSurface &&
            prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
        {
            GfMatrix4d transform = UsdGeomXform(prim).ComputeLocalToWorldTransform(UsdTimeCode::Default());

            VtArray<GfVec3f> points;
            UsdGeomPointBased pointBased(prim);
            pointBased.GetPointsAttr().Get(&points);
            if (points.size() != internalDeformableSurface->mNumCollMeshVerticesOrig)
            {
                CARB_LOG_WARN("Size of points of %s has changed - skipping update.", prim.GetPath().GetText());
                return true;
            }

            PxVec4* simPositionInvMass = internalDeformableSurface->mPositionInvMassH;
            CARB_ASSERT(internalDeformableSurface->mNumCollMeshVerticesWelded = uint32_t(internalDeformableSurface->mUsdToPhysxVtxMap.size()));
            for (uint32_t i = 0; i < internalDeformableSurface->mNumCollMeshVerticesWelded; ++i)
            {
                uint32_t origIndex = internalDeformableSurface->mUsdToPhysxVtxMap[i];
                GfVec3d localPoint = GfVec3d(points[origIndex][0], points[origIndex][1], points[origIndex][2]);
                GfVec3d worldPoint = transform.Transform(localPoint);

                simPositionInvMass[i] = PxVec4((float)worldPoint[0], (float)worldPoint[1], (float)worldPoint[2], simPositionInvMass[i].w);
            }

            PxDeformableSurfaceDataFlags flags = PxDeformableSurfaceDataFlags(0);
            flags.raise(PxDeformableSurfaceDataFlag::ePOSITION_INVMASS);

            PxDeformableSurfaceExt::copyToDevice(*deformableSurface, flags, internalDeformableSurface->mNumCollMeshVerticesWelded,
                internalDeformableSurface->mPositionInvMassH, internalDeformableSurface->mVelocityH, internalDeformableSurface->mRestPositionH,
                internalDeformableSurface->mPhysXScene->getInternalScene()->getDeformableCopyStream());
        }
    }

    return true;
}

bool PhysXUsdPhysicsInterface::updateDeformableSurfaceVelocitiesDeprecated(const usdparser::AttachedStage& attachedStage, const ObjectId objectId)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);

    if (!objectFullRecord || !objectFullRecord->mPtr)
    {
        return true;
    }

    if (internalType == ePTFEMClothDeprecated)
    {
        UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(objectFullRecord->mPath);

        PxDeformableSurface* deformableSurface = (PxDeformableSurface*)objectFullRecord->mPtr;
        InternalDeformableSurfaceDeprecated* internalDeformableSurface = (InternalDeformableSurfaceDeprecated*)objectFullRecord->mInternalPtr;

        if (deformableSurface &&
            internalDeformableSurface &&
            prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
        {
            VtArray<GfVec3f> velocities;
            PhysxSchemaPhysxDeformableAPI deformable(prim);
            deformable.GetSimulationVelocitiesAttr().Get(&velocities);
            if (velocities.size() != internalDeformableSurface->mNumCollMeshVerticesWelded)
            {
                CARB_LOG_WARN("Size of physxDeformable:simulationVelocities of %s has changed - skipping update.", prim.GetPath().GetText());
                return true;
            }

            PxVec4* simVelocity = internalDeformableSurface->mVelocityH;

            for (unsigned int i = 0; i < internalDeformableSurface->mNumCollMeshVerticesWelded; ++i)
            {
                simVelocity[i] = PxVec4(velocities[i][0], velocities[i][1], velocities[i][2], velocities[i][3]);
            }

            PxDeformableSurfaceDataFlags flags = PxDeformableSurfaceDataFlags(0);
            flags.raise(PxDeformableSurfaceDataFlag::eVELOCITY);

            PxDeformableSurfaceExt::copyToDevice(*deformableSurface, flags, internalDeformableSurface->mNumCollMeshVerticesWelded,
                internalDeformableSurface->mPositionInvMassH, internalDeformableSurface->mVelocityH, internalDeformableSurface->mRestPositionH,
                internalDeformableSurface->mPhysXScene->getInternalScene()->getDeformableCopyStream());
        }
    }

    return true;
}

static PxVec3 toPhysX(const carb::Float3& p, const GfMatrix4d& m)
{
    GfVec3f pos = { p.x, p.y, p.z };
    GfVec3f position = m.Transform(pos);
    return toPhysX(position);
}

ObjectId PhysXUsdPhysicsInterface::createDeformableSurfaceDeprecated(const usdparser::AttachedStage& attachedStage, const SdfPath& path, FEMClothDesc const& femClothDesc)
{
    if (!checkScenes())
    {
        PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, "No physics scene created, please add physics scene into stage!");
        return kInvalidObjectId;
    }

    const float metersPerUnit = float(UsdGeomGetStageMetersPerUnit(OmniPhysX::getInstance().getStage()));

    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxPhysics* physics = physxSetup.getPhysics();

    const ObjectId sceneId = attachedStage.getObjectDatabase()->findEntry(femClothDesc.scenePath, eScene);
    PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);
    if (!physxScene || !physxScene->isFullGpuPipelineAvailable())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            ErrorCode::eError,
            "Deformable Surface feature is only supported on GPU. Please enable GPU dynamics flag in Property/Scene of physics scene!");
        return kInvalidObjectId;
    }

    PxCudaContextManager* cudaContextManager = physxScene->getScene()->getCudaContextManager();
    if (!cudaContextManager)
    {
        CARB_LOG_ERROR("PhysX Deformable Surface creation failed due to missing PxCudaContextManager.");
        return kInvalidObjectId;
    }

    if (femClothDesc.deformableEnabled == false)
    {
        return kInvalidObjectId;
    }

    // Get assigned material
    ObjectId materialId = getDeformableMaterialIdDeprecated(*physxScene, femClothDesc);
    InternalDeformableMaterial* internalMaterial;
    PxDeformableSurfaceMaterial* material = getDeformableSurfaceMaterialDeprecated(&internalMaterial, *physxScene, materialId);
    if (!material)
    {
        return kInvalidObjectId;
    }

    UsdPrim usdPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(path);

    bool shouldRender = isRenderable(usdPrim);

    // Create internal deformable surface, and store/copy some data
    InternalDeformableSurfaceDeprecated* internalDeformableSurface = ICE_NEW(InternalDeformableSurfaceDeprecated)(physxScene, usdPrim);

    // Move to after creating internal object, because constructor sanitizes transform
    const GfMatrix4d femClothToWorld(UsdGeomXform(usdPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default()));

    // Compute local to world and extract scale
    internalDeformableSurface->mPrimToWorld = femClothToWorld;
    internalDeformableSurface->mNumCollMeshVerticesOrig = (unsigned int)femClothDesc.points.size();

    // Assign material id
    internalDeformableSurface->mMaterialId = materialId;

    // Create deformable surface
    PxFEMParameters femParams = internalDeformableSurface->mFEMParameters;
    PxTriangleMesh* triangleMesh;
    PxDeformableSurface* deformableSurface = createPhysxDeformableSurface(
        femClothDesc, femParams, material, triangleMesh, femClothToWorld, physxScene);
    if (!deformableSurface)
    {
        ICE_FREE(internalDeformableSurface);
        return kInvalidObjectId;
    }

    internalDeformableSurface->mNumCollMeshVerticesWelded = triangleMesh->getNbVertices();

    // Compute the triangle mesh area when rest offset or mass is unset or invalid.
    float totalArea = 0.0f;
    float materialDensity = internalMaterial ? internalMaterial->mDensity : 0.0f;
    float density, mass;
    getDeformableMassDensity(usdPrim, &femClothDesc, materialDensity, mass, density);

    float restOffset = femClothDesc.collisionRestOffset;

    // Compute the total area of the triangle mesh.
    if (mass <= 0.0f || restOffset <= 0.0f)
    {
        const std::vector<carb::Float3>& positions = femClothDesc.restPoints;
        const std::vector<uint32_t>& triangleIndices = femClothDesc.simulationIndices;

        for (int i = 0; i < triangleIndices.size(); i += 3)
        {
            const uint32_t triIndex0 = triangleIndices[i];
            const uint32_t triIndex1 = triangleIndices[i + 1];
            const uint32_t triIndex2 = triangleIndices[i + 2];
            totalArea += area(positions[triIndex0], positions[triIndex1], positions[triIndex2]);
        }
    }

    // If the rest offset is not set, use the estimated edge length.
    if (restOffset <= 0.0f)
    {
        const uint32_t numTriangles = static_cast<uint32_t>(femClothDesc.simulationIndices.size()) / 3;
        const float estimatedEdgeLength = numTriangles ? std::sqrt(totalArea / static_cast<float>(numTriangles)) : 0.0f;
        restOffset = PxMax(0.5f * estimatedEdgeLength, 0.01f / metersPerUnit);
    }

    // Inflate contact offset if it is less than the rest offset.
    float contactOffset = femClothDesc.collisionContactOffset;
    if (contactOffset < restOffset)
        contactOffset = 1.2f * restOffset;

    // Update selfCollisionFilterDistance if invalid.
    float selfCollisionFilterDistance = femClothDesc.selfCollisionFilterDistance;
    if (selfCollisionFilterDistance <= 0.0f)
    {
        femParams.selfCollisionFilterDistance = 2.5f * restOffset;
    }
    else
    {
        femParams.selfCollisionFilterDistance = selfCollisionFilterDistance;
    }

    PxShape* shape = deformableSurface->getShape();
    const uint32_t collisionGroup = convertToCollisionGroup(femClothDesc.collisionGroup);
    PxFilterData fd;
    convertCollisionGroupToPxFilterData(collisionGroup, fd);
    shape->setSimulationFilterData(fd);

    shape->setContactOffset(contactOffset);
    shape->setRestOffset(restOffset);

    physxScene->getInternalScene()->mDeformableSurfacesDeprecated.push_back(internalDeformableSurface);

    // Update mass.
    float vertexMassInv;
    if (mass > 0.0f)
    {
        vertexMassInv = internalDeformableSurface->mNumCollMeshVerticesWelded / mass;
    }
    else
    {
        float clothThickness = std::max(material->getThickness(), FLT_EPSILON);
        vertexMassInv = internalDeformableSurface->mNumCollMeshVerticesWelded / (totalArea * density * clothThickness);
    }

    PxVec4* positionInvMass = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, internalDeformableSurface->mNumCollMeshVerticesWelded);
    PxVec4* velocity = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, internalDeformableSurface->mNumCollMeshVerticesWelded);
    PxVec4* restPosition = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, internalDeformableSurface->mNumCollMeshVerticesWelded);

    const PxU32 numTris = triangleMesh->getNbTriangles();
    const PxU32* remap = triangleMesh->getTrianglesRemap();
    const PxU32* trianglesSource = reinterpret_cast<const PxU32*>(triangleMesh->getTriangles());

    internalDeformableSurface->mTriangleMesh = triangleMesh;
    internalDeformableSurface->buildMap(femClothDesc.simulationIndices);
    internalDeformableSurface->buildInverseMap(femClothDesc.simulationIndices);

    for (size_t i = 0; i < internalDeformableSurface->mUsdToPhysxVtxMap.size(); ++i)
    {
        PxU32 j = internalDeformableSurface->mUsdToPhysxVtxMap[i];
        restPosition[i] = PxVec4(toPhysX(femClothDesc.restPoints[j], femClothToWorld), vertexMassInv);
        positionInvMass[i] = PxVec4(toPhysX(femClothDesc.points[j], femClothToWorld), vertexMassInv);
        if (PxU32(femClothDesc.simulationVelocities.size()) == internalDeformableSurface->mNumCollMeshVerticesOrig)
        {
            //No transform needed here for the direction of the velocity?
            velocity[i] = PxVec4(toPhysX(femClothDesc.simulationVelocities[j]), vertexMassInv);
        }
        else
        {
            velocity[i] = PxVec4(0.0f, 0.0f, 0.0f, vertexMassInv);
        }
    }

    deformableSurface->setParameter(femParams);
    deformableSurface->setSolverIterationCounts(physxScene->getInternalScene()->clampPosIterationCount(femClothDesc.solverPositionIterations));
    deformableSurface->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, !femClothDesc.selfCollision);
    deformableSurface->setDeformableSurfaceFlag(PxDeformableSurfaceFlag::eENABLE_FLATTENING, femClothDesc.flatteningEnabled);

    deformableSurface->setNbCollisionPairUpdatesPerTimestep(femClothDesc.collisionPairUpdateFrequency);
    deformableSurface->setNbCollisionSubsteps(femClothDesc.collisionIterationMultiplier);

    float maxVelocity = femClothDesc.maxVelocity;
    if (isinf(maxVelocity))
        maxVelocity = PX_MAX_REAL;

    deformableSurface->setMaxVelocity(maxVelocity);
    deformableSurface->setMaxDepenetrationVelocity(femClothDesc.maxDepenetrationVelocity);

    PxDeformableSurfaceExt::copyToDevice(*deformableSurface, PxDeformableSurfaceDataFlag::eALL, internalDeformableSurface->mNumCollMeshVerticesWelded, positionInvMass, velocity,
        restPosition, internalDeformableSurface->mPhysXScene->getInternalScene()->getDeformableCopyStream());

    // Store them in internal deformable surface so we can use them when updating the graphics
    internalDeformableSurface->mPositionInvMassH = positionInvMass;
    internalDeformableSurface->mVelocityH = velocity;
    internalDeformableSurface->mRestPositionH = restPosition;
    internalDeformableSurface->mDeformableSurface = deformableSurface;

    internalDeformableSurface->mSolverPositionIterations = femClothDesc.solverPositionIterations;

    // Store start transformations
    if (internalDeformableSurface->mCollMeshPositionSaveRestoreBuf.size() < internalDeformableSurface->mNumCollMeshVerticesWelded)
    {
        internalDeformableSurface->mCollMeshPositionSaveRestoreBuf.resize(internalDeformableSurface->mNumCollMeshVerticesWelded);
        internalDeformableSurface->mCollMeshVelocitySaveRestoreBuf.resize(internalDeformableSurface->mNumCollMeshVerticesWelded);
    }
    copyBuffer(internalDeformableSurface->mCollMeshPositionSaveRestoreBuf, internalDeformableSurface->mPositionInvMassH, internalDeformableSurface->mNumCollMeshVerticesWelded);
    copyBuffer(internalDeformableSurface->mCollMeshVelocitySaveRestoreBuf, internalDeformableSurface->mVelocityH, internalDeformableSurface->mNumCollMeshVerticesWelded);
    internalDeformableSurface->mInitialPrimToWorld = UsdGeomXform(internalDeformableSurface->mPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());

    UsdGeomBoundable::ComputeExtentFromPlugins(UsdGeomMesh(internalDeformableSurface->mPrim), UsdTimeCode::Default(), &internalDeformableSurface->mExtentSaveRestoreBuf);

    ObjectId objectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(ePTFEMClothDeprecated, deformableSurface, internalDeformableSurface, usdPrim.GetPrimPath());
    if (internalMaterial)
    {
        internalMaterial->addDeformableId(objectId);
    }
    return objectId;
}

static ::physx::PxVec3 evaluateBarycentric(const carb::Uint3& triangle, const carb::Float3& bary, const carb::Float3* vertices)
{
    const ::physx::PxVec3& v0 = reinterpret_cast<const ::physx::PxVec3&>(vertices[triangle.x]);
    const ::physx::PxVec3& v1 = reinterpret_cast<const ::physx::PxVec3&>(vertices[triangle.y]);
    const ::physx::PxVec3& v2 = reinterpret_cast<const ::physx::PxVec3&>(vertices[triangle.z]);
    return bary.x * v0 + bary.y * v1 + bary.z * v2;
}

bool PhysXUsdPhysicsInterface::updateKinematicVertexTargetsFromSkinDeprecated(const SdfPath& path,
                                                                              ObjectId objectId,
    const carb::Float3* skinPoints, const size_t skinPointsSize, const GfMatrix4d& transformTimeSampled)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType;
    const InternalDatabase::Record* fullRecord = db.getFullRecord(internalType, objectId);
    if (!fullRecord)
        return true;
    void* objectRecord = fullRecord->mPtr;
    if (internalType == ePTSoftBodyDeprecated)
    {
        PxSoftBody* softBody = (PxSoftBody*)objectRecord;
        InternalDatabase::Record& actorRec = db.getRecords()[objectId];
        InternalDeformableBodyDeprecated* internalDeformableBody = (InternalDeformableBodyDeprecated*)fullRecord->mInternalPtr;

        bool isKinematic = (softBody->getSoftBodyFlag() & PxSoftBodyFlag::ePARTIALLY_KINEMATIC);
        bool isValidUpdate = (skinPointsSize == internalDeformableBody->mNumSkinMeshVertices);
        if (isKinematic && isValidUpdate)
        {
            const std::vector<carb::Uint3>& collisionVertexToSkinTriVertexIndices = internalDeformableBody->mCollisionVertexToSkinTriVertexIndices;
            const std::vector<carb::Float3>& collisionVertexToSkinTriBarycentrics = internalDeformableBody->mCollisionVertexToSkinTriBarycentrics;

            PxVec4* kinematicTarget = internalDeformableBody->getKinematicTargetsH();
            if (kinematicTarget)
            {
                if (collisionVertexToSkinTriBarycentrics.size() > 0)
                {
                    for (size_t i = 0; i < collisionVertexToSkinTriBarycentrics.size(); ++i)
                    {
                        PxVec3 p = evaluateBarycentric(collisionVertexToSkinTriVertexIndices[i],
                                                       collisionVertexToSkinTriBarycentrics[i], skinPoints);
                        GfVec3f s = transformTimeSampled.Transform(GfVec3f(p.x, p.y, p.z));
                        kinematicTarget[i] = PxConfigureSoftBodyKinematicTarget(toPhysX(s), true);
                    }
                    internalDeformableBody->uploadKinematicTargets(softBody->getSoftBodyFlag());
                }
                else
                {
                    //Collision mesh is not simplified
                    for (size_t i = 0; i < skinPointsSize; ++i)
                    {
                        const GfVec3f& source = reinterpret_cast<const GfVec3f&>(skinPoints[i]);
                        GfVec3f s = transformTimeSampled.Transform(source);
                        kinematicTarget[i] = PxConfigureSoftBodyKinematicTarget(toPhysX(s), true);
                    }
                    internalDeformableBody->uploadKinematicTargets(softBody->getSoftBodyFlag());
                }
            }
        }
        else
        {
            internalDeformableBody->mSoftBody->setKinematicTargetBufferD(nullptr, PxSoftBodyFlags(0));
            internalDeformableBody->mKinematicTargetsSet = false;

            if (!isKinematic)
            {
                CARB_LOG_ERROR("Deformable body is not configured as kinematic but received kinematic update: %s", path.GetText());
            }
            if (!isValidUpdate)
            {
                CARB_LOG_WARN("Size of points of %s has changed - skipping kinematic update.", path.GetText());
            }

        }
    }
    return true;
}

bool PhysXUsdPhysicsInterface::updateKinematicVertexTargetsFromSimDeprecated(const SdfPath& path, ObjectId objectId,
    const carb::Float4* simPoints, const size_t simPointsSize)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType;
    const InternalDatabase::Record* fullRecord = db.getFullRecord(internalType, objectId);
    if (!fullRecord)
        return true;
    void* objectRecord = fullRecord->mPtr;
    if (internalType == ePTSoftBodyDeprecated)
    {
        PxSoftBody* softBody = (PxSoftBody*)objectRecord;
        InternalDatabase::Record& actorRec = db.getRecords()[objectId];
        InternalDeformableBodyDeprecated* internalDeformableBody = (InternalDeformableBodyDeprecated*)fullRecord->mInternalPtr;

        bool isKinematic = (softBody->getSoftBodyFlag() & PxSoftBodyFlag::ePARTIALLY_KINEMATIC);
        bool isValidUpdate = (simPointsSize == internalDeformableBody->mNumSimMeshVertices);
        if (isKinematic && isValidUpdate)
        {
            PxVec4* kinematicTarget = internalDeformableBody->getKinematicTargetsH();
            if (kinematicTarget)
            {
                for (size_t i = 0; i < simPointsSize; ++i)
                {
                    kinematicTarget[i] = PxConfigureSoftBodyKinematicTarget(toPhysX(simPoints[i]), true);
                }
                internalDeformableBody->uploadKinematicTargets(softBody->getSoftBodyFlag());
            }
        }
        else
        {
            internalDeformableBody->mSoftBody->setKinematicTargetBufferD(nullptr, PxSoftBodyFlags(0));
            internalDeformableBody->mKinematicTargetsSet = false;

            if (!isKinematic)
            {
                CARB_LOG_ERROR(
                    "Deformable body is not configured as kinematic but received kinematic update: %s", path.GetText());
            }
            if (!isValidUpdate)
            {
                CARB_LOG_WARN("Size of simulation mesh points of %s has changed - skipping kinematic update.", path.GetText());
            }
        }
    }
    return true;
}

} // namespace physx
} // namespace omni
