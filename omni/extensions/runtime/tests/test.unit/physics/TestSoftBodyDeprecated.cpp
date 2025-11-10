// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>
#include <physxSchema/tetrahedralMesh.h>

#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

namespace
{

template <typename T = int>
struct ResultBuffer
{
    ~ResultBuffer()
    {
        if (ptr)
        {
            std::free(ptr);
            ptr = nullptr;
        }
        size = 0;
    }

    static void* allocate(size_t numBytes)
    {
        return std::malloc(numBytes);
    }

    T* ptr = nullptr;
    uint32_t size = 0;
};

pxr::VtArray<int> ExtractTriangulatedFaces(pxr::UsdGeomMesh const& usdMesh)
{
    // indices and faces converted to triangles
    pxr::VtArray<int> indices;
    usdMesh.GetFaceVertexIndicesAttr().Get(&indices);

    pxr::VtArray<int> faces;
    usdMesh.GetFaceVertexCountsAttr().Get(&faces);

    pxr::VtArray<int> triangles;
    if (indices.empty() || faces.empty())
        return triangles;

    triangles.reserve(1024);

    uint32_t indicesOffset = 0;

    uint32_t numIndices = uint32_t(indices.size());
    uint32_t numFaces = uint32_t(faces.size());
    for (uint32_t i = 0; i < numFaces; i++)
    {
        const uint32_t faceCount = faces[i];
        CARB_ASSERT(faceCount >= 3);
        CARB_ASSERT(indicesOffset < numIndices);
        const uint32_t startIndex = indices[indicesOffset];
        for (uint32_t faceIndex = 0; faceIndex < faceCount - 2; faceIndex++)
        {
            uint32_t index1 = indicesOffset + faceIndex + 1;
            uint32_t index2 = indicesOffset + faceIndex + 2;
            CARB_ASSERT(index1 < numIndices);
            CARB_ASSERT(index2 < numIndices);

            triangles.push_back(startIndex);
            triangles.push_back(indices[index1]);
            triangles.push_back(indices[index2]);
        }
        indicesOffset += faceCount;
    }

    return triangles;
}

} // namespace

bool createConformingTetrahedralMesh(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking,
                                     const VtArray<GfVec3f>& srcTriMeshPoints, const VtArray<int>& srcTriMeshIndices,
                                     VtArray<GfVec3f>& dstTetMeshPoints, VtArray<int>& dstTetMeshIndices)
{
    ResultBuffer<carb::Float3> dstTetPointsR;
    ResultBuffer<uint32_t> dstTetIndicesR;
    bool success = physxCooking.computeConformingTetrahedralMesh(
        dstTetPointsR.ptr, dstTetPointsR.size, dstTetIndicesR.ptr, dstTetIndicesR.size,
        (carb::Float3*)srcTriMeshPoints.data(), uint32_t(srcTriMeshPoints.size()),
        (uint32_t*)srcTriMeshIndices.data(), uint32_t(srcTriMeshIndices.size()),
        ResultBuffer<>::allocate);
    if (success)
    {
        dstTetMeshPoints.assign((GfVec3f*)dstTetPointsR.ptr, (GfVec3f*)dstTetPointsR.ptr + dstTetPointsR.size);
        dstTetMeshIndices.assign((int*)dstTetIndicesR.ptr, (int*)dstTetIndicesR.ptr + dstTetIndicesR.size);
    }
    return success;
}

bool createConformingTetrahedralMeshPrim(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking,
                                         const SdfPath& srcMeshPath, const SdfPath& destTetMeshPath)
{
    UsdGeomMesh geomMesh = UsdGeomMesh::Get(stage, srcMeshPath);
    VtArray<GfVec3f> srcPoints;
    geomMesh.GetPointsAttr().Get(&srcPoints);
    VtArray<int> srcIndices = ExtractTriangulatedFaces(geomMesh);

    VtArray<GfVec3f> dstPoints;
    VtArray<int> dstIndices;
    bool result = createConformingTetrahedralMesh(stage, physxCooking, srcPoints, srcIndices, dstPoints, dstIndices);
    PhysxSchemaTetrahedralMesh tetMesh = PhysxSchemaTetrahedralMesh::Define(stage, destTetMeshPath);
    tetMesh.CreatePointsAttr().Set(dstPoints);
    tetMesh.CreateIndicesAttr().Set(dstIndices);
    return result;
}

bool createVoxelTetrahedralMesh(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking,
                                const VtArray<GfVec3f>& srcTetMeshPoints, const VtArray<int>& srcTetMeshIndices,
                                VtArray<GfVec3f>& dstTetMeshPoints, VtArray<int>& dstTetMeshIndices)
{
    ResultBuffer<carb::Float3> dstTetPointsR;
    ResultBuffer<uint32_t> dstTetIndicesR;
    ResultBuffer<int32_t> dstEmbeddingR;
    bool success = physxCooking.computeVoxelTetrahedralMesh(
        dstTetPointsR.ptr, dstTetPointsR.size, dstTetIndicesR.ptr, dstTetIndicesR.size,
        dstEmbeddingR.ptr, dstEmbeddingR.size,
        (carb::Float3*)srcTetMeshPoints.data(), uint32_t(srcTetMeshPoints.size()),
        (uint32_t*)srcTetMeshIndices.data(), uint32_t(srcTetMeshIndices.size()),
        { 1.0f, 1.0f, 1.0f }, 10, ResultBuffer<>::allocate);
    if (success)
    {
        dstTetMeshPoints.assign((GfVec3f*)dstTetPointsR.ptr, (GfVec3f*)dstTetPointsR.ptr + dstTetPointsR.size);
        dstTetMeshIndices.assign((int*)dstTetIndicesR.ptr, (int*)dstTetIndicesR.ptr + dstTetIndicesR.size);
    }
    return success;
}

bool createVoxelTetrahedralMeshPrim(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking,
                                    const SdfPath& srcTetMeshPath, const SdfPath& destTetMeshPath)
{
    PhysxSchemaTetrahedralMesh srcTetMesh = PhysxSchemaTetrahedralMesh::Get(stage, srcTetMeshPath);
    VtArray<GfVec3f> srcPoints;
    VtArray<int> srcIndices;
    srcTetMesh.GetPointsAttr().Get(&srcPoints);
    srcTetMesh.GetIndicesAttr().Get(&srcIndices);

    VtArray<GfVec3f> dstPoints;
    VtArray<int> dstIndices;
    bool result = createVoxelTetrahedralMesh(stage, physxCooking, srcPoints, srcIndices, dstPoints, dstIndices);
    PhysxSchemaTetrahedralMesh tetMesh = PhysxSchemaTetrahedralMesh::Define(stage, destTetMeshPath);
    tetMesh.CreatePointsAttr().Set(dstPoints);
    tetMesh.CreateIndicesAttr().Set(dstIndices);
    return result;
}

PhysxSchemaPhysxDeformableBodyAPI createDefaultDeformableBody(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking, SdfPath path)
{
    PhysxSchemaPhysxDeformableBodyAPI deformableBody;
    UsdGeomMesh boxMesh = createMeshBox(stage, path, { 50, 50, 50 });

    VtArray<GfVec3f> srcMeshPoints;
    VtArray<int> srcMeshIndices;
    boxMesh.GetPointsAttr().Get(&srcMeshPoints);
    srcMeshIndices = ExtractTriangulatedFaces(boxMesh);

    VtArray<GfVec3f> conformingPoints;
    VtArray<int> conformingIndices;
    if (!createConformingTetrahedralMesh(stage, physxCooking, srcMeshPoints, srcMeshIndices, conformingPoints, conformingIndices))
    {
        return deformableBody;
    }

    VtArray<GfVec3f> voxelPoints;
    VtArray<int> voxelIndices;
    if (!createVoxelTetrahedralMesh(stage, physxCooking, conformingPoints, conformingIndices, voxelPoints, voxelIndices))
    {
        return deformableBody;
    }

    deformableBody = PhysxSchemaPhysxDeformableBodyAPI::Apply(boxMesh.GetPrim());

    PhysxSchemaPhysxDeformableAPI(deformableBody).CreateRestPointsAttr().Set(srcMeshPoints);
    deformableBody.CreateCollisionPointsAttr().Set(conformingPoints);
    deformableBody.CreateCollisionIndicesAttr().Set(conformingIndices);
    deformableBody.CreateCollisionRestPointsAttr().Set(conformingPoints);
    deformableBody.CreateSimulationPointsAttr().Set(voxelPoints);
    PhysxSchemaPhysxDeformableAPI(deformableBody).CreateSimulationIndicesAttr().Set(voxelIndices);
    deformableBody.CreateSimulationRestPointsAttr().Set(voxelPoints);

    static const TfToken simulationHexahedralResolutionToken("physxDeformable:simulationHexahedralResolution");
    deformableBody.GetPrim().CreateAttribute(simulationHexahedralResolutionToken, SdfValueTypeNames->UInt, true).Set(10u);

    return deformableBody;
}

PhysxSchemaPhysxDeformableSurfaceAPI createDefaultDeformableSurface(UsdStageRefPtr& stage, SdfPath path)
{
    // Create plane
    UsdGeomMesh planeMesh = createMeshSquare(stage, path, 50.0f, 50.0f);

    PhysxSchemaPhysxDeformableSurfaceAPI deformableSurface = PhysxSchemaPhysxDeformableSurfaceAPI::Apply(planeMesh.GetPrim());

    VtArray<GfVec3f> meshPoints;
    VtArray<int> meshIndices;
    pxr::VtIntArray faceVertexIndices({ 0, 1, 2, 1, 2, 3 });

    planeMesh.CreatePointsAttr().Get(&meshPoints);
    planeMesh.CreateFaceVertexIndicesAttr().Get(&meshIndices);

    PhysxSchemaPhysxDeformableAPI(deformableSurface).CreateRestPointsAttr().Set(meshPoints);
    PhysxSchemaPhysxDeformableAPI(deformableSurface).CreateSimulationIndicesAttr().Set(faceVertexIndices);
    return deformableSurface;
}

//-----------------------------------------------------------------------------
// Init test
TEST_CASE("Soft Body Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=clow][priority=mandatory]")
{
    // constants for setup
    const GfVec3f gravityDirection(0.0f, 0.0f, -1.0f); // use z-up
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const float kilogramsPerStageUnit = 1.0f;
    const float gravityMagnitude = 10.0f / metersPerStageUnit;
    const float density = 1000.f * metersPerStageUnit * metersPerStageUnit * metersPerStageUnit / kilogramsPerStageUnit;

    // setup common to all subcases
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxCooking* physxCooking = physicsTests.acquirePhysxCookingInterface();
    REQUIRE(physxCooking);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    scene.GetGravityMagnitudeAttr().Set(gravityMagnitude);
    scene.GetGravityDirectionAttr().Set(gravityDirection);

    SUBCASE("USD API Regression Test")
    {
        // GetAttribute(...).Get(...) is not well documented and there might be no guarantee that the output variable is not modified when Get() fails because the attribute is not present
        // Check that Get() performs as expected
        // The same regression test is implemented as Python unit test test_regression_usd_api(self)

        static TfToken tokenBool("regressionTestBool");
        static TfToken tokenBoolBad("regressionTestBoolBad");

        bool checkBool = true;
        defaultPrim.CreateAttribute(tokenBool, SdfValueTypeNames->Bool).Set(checkBool);

        checkBool = false;
        defaultPrim.GetAttribute(tokenBool).Get(&checkBool);
        CHECK_EQ(checkBool, true);

        // Check both bool states (true and false)
        checkBool = false;
        defaultPrim.GetAttribute(tokenBoolBad).Get(&checkBool);
        CHECK_EQ(checkBool, false);

        checkBool = true;
        defaultPrim.GetAttribute(tokenBoolBad).Get(&checkBool);
        CHECK_EQ(checkBool, true);

        static TfToken tokenFloat("regressionTestFloat");
        static TfToken tokenFloatBad("regressionTestFloatBad");

        float value = PX_MAX_F32;
        defaultPrim.CreateAttribute(tokenFloat, SdfValueTypeNames->Float).Set(value);

        value = 0.0f;
        defaultPrim.GetAttribute(tokenFloat).Get(&value);
        CHECK_EQ(value, PX_MAX_F32);

        // Check 0.0f, -PX_MAX_F32 and PX_MAX_F32
        value = 0.0f;
        defaultPrim.GetAttribute(tokenFloatBad).Get(&value);
        CHECK_EQ(value, 0.0f);

        value = -PX_MAX_F32;
        defaultPrim.GetAttribute(tokenFloatBad).Get(&value);
        CHECK_EQ(value, -PX_MAX_F32);

        value = PX_MAX_F32;
        defaultPrim.GetAttribute(tokenFloatBad).Get(&value);
        CHECK_EQ(value, PX_MAX_F32);
    }

    SUBCASE("Conforming and Voxel TetrahedralMeshes")
    {
        const SdfPath boxMeshPath = defaultPrimPath.AppendChild(TfToken("boxMesh"));
        const SdfPath boxTetMeshConformingPath = defaultPrimPath.AppendChild(TfToken("boxTetMeshConforming"));
        const SdfPath boxTetMeshVoxelPath = defaultPrimPath.AppendChild(TfToken("boxTetMeshVoxel"));

        // Create box mesh
        createMeshBox(stage, boxMeshPath, { 50, 50, 50 });

        REQUIRE(createConformingTetrahedralMeshPrim(stage, *physxCooking, boxMeshPath, boxTetMeshConformingPath));
        REQUIRE(createVoxelTetrahedralMeshPrim(stage, *physxCooking, boxTetMeshConformingPath, boxTetMeshVoxelPath));
    }

#if USE_PHYSX_GPU
    SUBCASE("Deformable Body Material")
    {
        const SdfPath physicsMaterialPath = defaultPrimPath.AppendChild(TfToken("PhysicsMaterial"));

        UsdShadeMaterial material = UsdShadeMaterial::Define(stage, physicsMaterialPath);

        PhysxSchemaPhysxDeformableBodyMaterialAPI materialAPI = PhysxSchemaPhysxDeformableBodyMaterialAPI::Apply(material.GetPrim());

        materialAPI.GetYoungsModulusAttr().Set(5000.0f);
        materialAPI.GetPoissonsRatioAttr().Set(0.045f);
        materialAPI.GetDynamicFrictionAttr().Set(0.025f);
        materialAPI.GetElasticityDampingAttr().Set(0.0005f);
        materialAPI.GetDampingScaleAttr().Set(1.0f);

        // parse
        physxSim->attachStage(stageId);

        // sanity check
        PxFEMSoftBodyMaterial* femMaterial = getPhysxBaseDerivedFromPathChecked<PxFEMSoftBodyMaterial>(physicsMaterialPath, PhysXType::ePTSoftBodyMaterialDeprecated);

        // Check user material
        CHECK_EQ(5000.0f, femMaterial->getYoungsModulus());
        CHECK_EQ(0.045f, femMaterial->getPoissons());
        CHECK_EQ(0.025f, femMaterial->getDynamicFriction());
        CHECK_EQ(0.0005f, femMaterial->getDamping());
        CHECK_EQ(1.0f, femMaterial->getDampingScale());

        // update params:
        materialAPI.GetYoungsModulusAttr().Set(15000.0f);
        materialAPI.GetPoissonsRatioAttr().Set(0.145f);
        materialAPI.GetDynamicFrictionAttr().Set(0.125f);
        materialAPI.GetElasticityDampingAttr().Set(0.0015f);
        materialAPI.GetDampingScaleAttr().Set(0.5f);

        // step
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        CHECK_EQ(15000.0f, femMaterial->getYoungsModulus());
        CHECK_EQ(0.145f, femMaterial->getPoissons());
        CHECK_EQ(0.125f, femMaterial->getDynamicFriction());
        CHECK_EQ(0.0015f, femMaterial->getDamping());
        CHECK(femMaterial->getDampingScale() > 0.4999f);
        CHECK(femMaterial->getDampingScale() < 0.5001f);
    }

    SUBCASE("Material Precedence")
    {
        //for (bool useBody : { false, true })
        bool useBody = false;
        {
            // Create deformable
            const SdfPath deformablePath = defaultPrimPath.AppendChild(TfToken("deformable"));
            PhysxSchemaPhysxDeformableAPI deformable;
            if (useBody)
            {
                deformable = PhysxSchemaPhysxDeformableAPI(createDefaultDeformableBody(stage, *physxCooking, deformablePath));
            }
            else
            {
                deformable = PhysxSchemaPhysxDeformableAPI(createDefaultDeformableSurface(stage, deformablePath));
            }
            CHECK(deformable);

            // Create four deformable materials
            SdfPath deformableMaterialPath[4];
            UsdShadeMaterial shadeMaterial[4];
            for (uint32_t i = 0; i < 4; ++i)
            {
                std::string name(std::string("deformableMaterial_") + std::to_string(i));
                deformableMaterialPath[i] = defaultPrimPath.AppendChild(TfToken(name.c_str()));
                shadeMaterial[i] = UsdShadeMaterial::Define(stage, deformableMaterialPath[i]);
                if (useBody)
                {
                    PhysxSchemaPhysxDeformableBodyMaterialAPI::Apply(shadeMaterial[i].GetPrim());
                }
                else
                {
                    PhysxSchemaPhysxDeformableSurfaceMaterialAPI::Apply(shadeMaterial[i].GetPrim());
                }
            }

            // The following SUBCASE structure makes sure we are going through assinging materials
            // at ever increasing precedence.
            uint32_t caseCounter = 0;
            SUBCASE("Default Material"){}
            SUBCASE("Scene Material Binding")
            {
                UsdShadeMaterialBindingAPI sceneBindAPI = UsdShadeMaterialBindingAPI::Apply(scene.GetPrim());
                sceneBindAPI.Bind(shadeMaterial[0], UsdShadeTokens->fallbackStrength, TfToken("physics"));
                caseCounter++;
                SUBCASE("No Further Bindings"){}
                SUBCASE("Scene Physics Material Binding")
                {
                    UsdRelationship sceneRel =
                        scene.GetPrim().CreateRelationship(TfToken("material:binding:physics"), false);
                    sceneRel.SetTargets(SdfPathVector({ deformableMaterialPath[1] }));
                    caseCounter++;
                    SUBCASE("No Further Bindings"){}
                    SUBCASE("Deformable Material Binding")
                    {
                        UsdShadeMaterialBindingAPI deformableBindAPI = UsdShadeMaterialBindingAPI::Apply(deformable.GetPrim());
                        deformableBindAPI.Bind(shadeMaterial[2], UsdShadeTokens->fallbackStrength, TfToken("physics"));
                        caseCounter++;
                        SUBCASE("No Further Bindings"){}
                        SUBCASE("Deformable Physics Material Binding")
                        {
                            UsdRelationship deformableRel = deformable.GetPrim().CreateRelationship(TfToken("material:binding:physics"), false);
                            deformableRel.SetTargets({ deformableMaterialPath[3] });
                            caseCounter++;
                        }
                    }
                }
            }

            physxSim->attachStage(stageId);

            // Materials should be there
            PxBase* baseMaterialPtrs[4];
            for (uint32_t i = 0; i < 4; ++i)
            {
                PhysXType materialType = useBody ? ePTSoftBodyMaterialDeprecated : ePTFEMClothMaterialDeprecated;
                baseMaterialPtrs[i] = reinterpret_cast<PxBase*>(physx->getPhysXPtr(deformableMaterialPath[i], materialType));
                CHECK(baseMaterialPtrs[i] != nullptr);
                bool isRightType = useBody ? baseMaterialPtrs[i]->is<PxFEMSoftBodyMaterial>() != nullptr :
                                             baseMaterialPtrs[i]->is<PxDeformableSurfaceMaterial>() != nullptr;
                CHECK(isRightType);
            }

            // Deformable should be there
            PhysXType deformableType = useBody ? ePTSoftBodyDeprecated : ePTFEMClothDeprecated;
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(deformablePath, deformableType));
            CHECK(basePtr != nullptr);
            bool isRightType = useBody ? basePtr->is<PxSoftBody>() != nullptr : basePtr->is<PxDeformableSurface>() != nullptr;
            CHECK(isRightType);

            // Check material bindings
            PxFEMMaterial* deformableMaterial = nullptr;
            if (useBody)
            {
                PxFEMSoftBodyMaterial* femSoftBodyMaterial;
                PxShape* shape = basePtr->is<PxSoftBody>()->getShape();
                REQUIRE(shape->getNbMaterials() == 1);
                PxU32 numMaterials = shape->getSoftBodyMaterials(&femSoftBodyMaterial, 1);
                REQUIRE(numMaterials == 1);
                deformableMaterial = femSoftBodyMaterial;
            }
            else
            {
                PxDeformableSurfaceMaterial* deformableSurfaceMaterial;
                PxShape* shape = basePtr->is<PxDeformableSurface>()->getShape();
                REQUIRE(shape->getNbMaterials() == 1);
                REQUIRE(shape->getDeformableSurfaceMaterials(&deformableSurfaceMaterial, 1) == 1);
                deformableMaterial = deformableSurfaceMaterial;
            }

            if (caseCounter > 0)
            {
                REQUIRE(deformableMaterial == baseMaterialPtrs[caseCounter - 1]);
            }
        }
    }

    SUBCASE("Material Density")
    {
        // Create base physics and deformable body materials
        const SdfPath basePhysicsMaterialPath = defaultPrimPath.AppendChild(TfToken("basePhysicsMaterial"));
        UsdShadeMaterial basePhysicsMaterial = UsdShadeMaterial::Define(stage, basePhysicsMaterialPath);
        UsdPhysicsMaterialAPI::Apply(basePhysicsMaterial.GetPrim());

        UsdShadeMaterial baseDeformableBodyMaterial = UsdShadeMaterial::Define(stage, basePhysicsMaterialPath);
        PhysxSchemaPhysxDeformableBodyMaterialAPI materialAPI = PhysxSchemaPhysxDeformableBodyMaterialAPI::Apply(baseDeformableBodyMaterial.GetPrim());

        SUBCASE("DefaultScene")
        {
            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(scene.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));
        }

        // Create deformable body
        const SdfPath deformableBodyPath = defaultPrimPath.AppendChild(TfToken("boxMesh"));
        PhysxSchemaPhysxDeformableBodyAPI deformableBody = createDefaultDeformableBody(stage, *physxCooking, deformableBodyPath);
        CHECK(deformableBody);

        SUBCASE("BoxPrim")
        {
            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(deformableBody.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));
        }

        materialAPI.CreateDensityAttr().Set(1000.0f);

        physxSim->attachStage(stageId);

        // material should be there
        PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTSoftBodyMaterialDeprecated));
        CHECK(baseMaterialPtr != nullptr);
        CHECK(baseMaterialPtr->is<PxFEMSoftBodyMaterial>());

        // deformable body should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(deformableBodyPath, ePTSoftBodyDeprecated));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxSoftBody>());

        PxSoftBody* softBody = basePtr->is<PxSoftBody>();
        PxShape* shape = softBody->getShape();
        REQUIRE(shape->getNbMaterials() == 1);
        PxFEMSoftBodyMaterial* material = nullptr;
        shape->getSoftBodyMaterials(&material, 1);
        CHECK(material->userData != nullptr);
        CHECK(material == baseMaterialPtr->is<PxFEMSoftBodyMaterial>());

        // Workaround for forcing a sync to get softBody->getSimPositionInvMassBufferD updated.
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        PxCudaContextManager* cudaContextManager = softBody->getCudaContextManager();
        PxScopedCudaLock _lock(*cudaContextManager);

        PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

        // only allocate one PxVec4 to get the mass.
        PxVec4* positionInvGM = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, 1);
        cudaContext->memcpyDtoH(positionInvGM, reinterpret_cast<CUdeviceptr>(softBody->getSimPositionInvMassBufferD()), sizeof(PxVec4));

        float initialMass = 1.0f / positionInvGM[0].w;

        // Change material density
        materialAPI.CreateDensityAttr().Set(100.0f);

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // Check mass
        cudaContext->memcpyDtoH(positionInvGM, reinterpret_cast<CUdeviceptr>(softBody->getSimPositionInvMassBufferD()), sizeof(PxVec4));
        float finalMass = 1.0f / positionInvGM[0].w;

        CHECK_LT(finalMass, initialMass);

        // Change mass instead of density
        UsdPhysicsMassAPI massApi = UsdPhysicsMassAPI::Apply(deformableBody.GetPrim());
        massApi.CreateMassAttr().Set(1000.0f);

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // Check mass
        cudaContext->memcpyDtoH(positionInvGM, reinterpret_cast<CUdeviceptr>(softBody->getSimPositionInvMassBufferD()), sizeof(PxVec4));
        float mass = 1.0f / positionInvGM[0].w;
        CHECK_LT(mass, finalMass);

        PX_PINNED_HOST_FREE(cudaContextManager, positionInvGM);
    }

    SUBCASE("Deformable Body")
    {
        // Create deformable body
        const SdfPath deformableBodyPath = defaultPrimPath.AppendChild(TfToken("boxMesh"));
        PhysxSchemaPhysxDeformableBodyAPI deformableBody = createDefaultDeformableBody(stage, *physxCooking, deformableBodyPath);
        CHECK(deformableBody);

        // parse
        physxSim->attachStage(stageId);

        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);

        // sanity check
        PxU32 nbSoftBodies = pxScene->getNbSoftBodies();
        CHECK_EQ(1u, nbSoftBodies);

        PxSoftBody* softBody = getPhysxBaseDerivedFromPathChecked<PxSoftBody>(deformableBodyPath, PhysXType::ePTSoftBodyDeprecated);

        // Check global FEM parameters
        PxFEMParameters params = softBody->getParameter();
        CHECK_EQ(0.005f, params.velocityDamping);
        CHECK_EQ(10.0f, params.settlingThreshold);
        CHECK_EQ(5.0f, params.sleepThreshold);
        CHECK_EQ(10.0f, params.sleepDamping);

        // Check simulation
        PxBounds3 bounds = softBody->getWorldBounds();
        PxVec3 startPosition = bounds.getCenter();

        // start simulating to get up-to-date PhysX values
        for (PxU32 u = 0; u < 10; u++)
        {
            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();
        }

        bounds = softBody->getWorldBounds();
        PxVec3 endPosition = bounds.getCenter();

        CHECK_LT(endPosition.z, startPosition.z);

        // Update the velocities to test USD writes.
        pxr::VtArray<pxr::GfVec3f> velocities;

        for (PxU32 u = 0; u < 10; u++)
        {
            PhysxSchemaPhysxDeformableAPI(deformableBody).GetSimulationVelocitiesAttr().Get(&velocities);
            
            for (unsigned int i = 0; i < velocities.size(); ++i)
            {
                velocities[i][2] += 100.0f;
            }

            PhysxSchemaPhysxDeformableAPI(deformableBody).GetSimulationVelocitiesAttr().Set(velocities);

            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();
        }

        bounds = softBody->getWorldBounds();
        endPosition = bounds.getCenter();

        CHECK_GT(endPosition.z, startPosition.z);
    }

    SUBCASE("Deformable Surface")
    {
        const SdfPath deformableSurfacePath = defaultPrimPath.AppendChild(TfToken("deformableSurface"));
        PhysxSchemaPhysxDeformableSurfaceAPI deformableSurfaceAPI = createDefaultDeformableSurface(stage, deformableSurfacePath);

        // parse
        physxSim->attachStage(stageId);

        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);

        // sanity check
        PxU32 nbDeformableSurfaces = pxScene->getNbDeformableSurfaces();
        CHECK_EQ(1u, nbDeformableSurfaces);

        PxDeformableSurface* deformableSurface = getPhysxBaseDerivedFromPathChecked<PxDeformableSurface>(deformableSurfacePath, PhysXType::ePTFEMClothDeprecated);

        // Check global FEM parameters
        PxFEMParameters params = deformableSurface->getParameter();
        CHECK_EQ(0.005f, params.velocityDamping);
        CHECK_EQ(10.0f, params.settlingThreshold);
        CHECK_EQ(5.0f, params.sleepThreshold);
        CHECK_EQ(10.0f, params.sleepDamping);

        // Check simulation
        PxBounds3 bounds = deformableSurface->getWorldBounds();
        PxVec3 startPosition = bounds.getCenter();

        // start simulating to get up-to-date PhysX values
        for (PxU32 u = 0; u < 10; u++)
        {
            physxSim->simulate(0.017f, 0.0f);
        }
        physxSim->fetchResults();

        bounds = deformableSurface->getWorldBounds();
        PxVec3 endPosition = bounds.getCenter();

        CHECK_LT(endPosition.z, startPosition.z);

        // Update the velocities to test USD writes.
        pxr::VtArray<pxr::GfVec3f> velocities;

        for (PxU32 u = 0; u < 10; u++)
        {
            PhysxSchemaPhysxDeformableAPI(deformableSurfaceAPI).GetSimulationVelocitiesAttr().Get(&velocities);

            for (unsigned int i = 0; i < velocities.size(); ++i)
            {
                velocities[i][2] += 100.0f;
            }

            PhysxSchemaPhysxDeformableAPI(deformableSurfaceAPI).GetSimulationVelocitiesAttr().Set(velocities);

            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();
        }

        bounds = deformableSurface->getWorldBounds();
        endPosition = bounds.getCenter();

        CHECK_GT(endPosition.z, startPosition.z);
    }

#endif // USE_PHYSX_GPU

#if 0
    // debugging
    stage->Export("SoftBodyDebug.usda");
#endif

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
