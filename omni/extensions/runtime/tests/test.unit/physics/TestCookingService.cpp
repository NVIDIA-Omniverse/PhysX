// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <chrono>
#include <thread>

#include "../common/TestHelpers.h"
#include "../../common/Tools.h" // asInt

#include <omni/physx/IPhysxCookingService.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <private/omni/physx/IPhysxTests.h>

#include <omni/fabric/SimStageWithHistory.h>
#include <omni/fabric/FabricUSD.h>

#include <carb/settings/ISettings.h>
#include <carb/tasking/ITasking.h>
#include <omni/physx/IPhysxSettings.h>

#include "PhysicsTools.h"

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

static void resetLocalMeshCaches(IPhysxCookingServicePrivate* physxCookingPrivate)
{
    // Reset old cache and LocalDataStore using the cooking interface
    physxCookingPrivate->resetLocalMeshCacheContents();

    // "Reset" the OmniHub cache using settings to regenerate dev key so that it won't use its previous cache contents
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    if (settings)
    {
        settings->setInt64(kSettingUjitsoCookingDevKey, -1ll);
    }
}

// This test is checking all permutations of the following parameters
struct CookingServiceTestParameters
{
    bool asynchronous = false;
    enum InputType
    {
        eInputUSD,
        eInputFabric,
        eInputMeshView
    };
    InputType inputType = eInputUSD;
    bool computeGPUCookingData = true;
    bool enableLoadFromLocalCache = false;

    bool forceTriangulation = false;

    // These are SDF Specific, and not accounted as permutations for
    bool enableGPUCooking = false;
    omni::physx::PhysxCookingAsyncContext context = nullptr;

    omni::physx::IPhysxCookingService* physxCooking = nullptr;
    omni::physx::IPhysxCookingServicePrivate* physxCookingPrivate = nullptr;
    PxDefaultAllocator pxAllocator;
    PxDefaultErrorCallback pxErrorCallback;
    PxFoundation* pxFoundation = nullptr;
    PxPhysics* pxPhysics = nullptr;
    UsdStageRefPtr stage;
    long int primStageId = 0;
    UsdGeomMesh usdMesh;
    SdfPath meshPath;
    SdfPath defaultPrimPath;

    void createShared()
    {
        PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

        physxCooking = physicsTests.acquirePhysxCookingServiceInterface();
        physxCookingPrivate = physicsTests.acquirePhysxCookingServicePrivateInterface();

        REQUIRE(physxCooking);
        REQUIRE(physxCookingPrivate);
        // Always start with empty cache
        physxCookingPrivate->setLocalMeshCacheEnabled(true);
        resetLocalMeshCaches(physxCookingPrivate);
        pxFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, pxAllocator, pxErrorCallback);
        stage = UsdStage::CreateInMemory();

        if (inputType != CookingServiceTestParameters::eInputMeshView)
        {
            pxr::UsdUtilsStageCache::Get().Insert(stage);
            primStageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
        }

        const float metersPerStageUnit = 0.01f; // work in default centimeters
        const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
        pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
        const PxTolerancesScale tolerancesScale(float(1.0 / metersPerUnit), float(10.0 / metersPerUnit));
        pxPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *pxFoundation, tolerancesScale);
    }

    void createInstanceSpecific(const std::string& name, omni::physx::PhysxCookingAsyncContext cookingContext)
    {
        context = cookingContext;
        defaultPrimPath = SdfPath("/World");
        UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
        stage->SetDefaultPrim(defaultPrim);

        meshPath = SdfPath(name);
        usdMesh = createMeshCapsule(stage, meshPath, 1.0, 1.0);

        if (inputType == CookingServiceTestParameters::eInputFabric)
        {
            auto iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
            auto iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();
            auto iFabricUsd = carb::getCachedInterface<omni::fabric::IFabricUsd>();
            iSimStageWithHistory->getOrCreate(primStageId, 1, { 1, 30 }, omni::fabric::GpuComputeType::eNone);
            iStageReaderWriter->create(primStageId, 0);
            auto stageReaderWriterId = iStageReaderWriter->get(primStageId);
            auto fabricId = iStageReaderWriter->getFabricId(stageReaderWriterId);
            std::set<omni::fabric::TokenC> filter = {};
            iFabricUsd->prefetchPrimToFabric(
                fabricId, meshPath, usdMesh.GetPrim(), filter, false, true, pxr::UsdTimeCode::Default());
            stage->RemovePrim(meshPath);
        }
    }

    void destroyInstanceSpecific()
    {
        if (context)
        {
            physxCooking->destroyAsyncContext(context);
        }
        context = nullptr;
    }

    void destroyShared()
    {
        if (inputType == CookingServiceTestParameters::eInputFabric)
        {
            auto iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();
            iSimStageWithHistory->release(primStageId);
        }
        else if (inputType != CookingServiceTestParameters::eInputMeshView)
        {
            pxr::UsdUtilsStageCache::Get().Erase(stage);
        }

        PX_RELEASE(pxPhysics);
        PX_RELEASE(pxFoundation)
    }
};

PxDefaultMemoryInputData& fromCookedData(PxDefaultMemoryInputData& inputData, const PhysxCookedDataSpan& cookedData)
{
    inputData = PxDefaultMemoryInputData(
        const_cast<PxU8*>(reinterpret_cast<const PxU8*>(cookedData.data)), static_cast<PxU32>(cookedData.sizeInBytes));
    return inputData;
}

void testCookingServiceParametricImplementation(const PhysxCookingDataType::Enum dataType,
                                                const CookingServiceTestParameters& testParameters,
                                                uint32_t threadIdx = 0, // For multitrheaded since we can't clear cache
                                                bool runTwice = true // Test for cache hits
)
{
    carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>();
    omni::physx::IPhysxCookingService* physxCooking = testParameters.physxCooking;
    omni::physx::IPhysxCookingServicePrivate* physxCookingPrivate = testParameters.physxCookingPrivate;
    REQUIRE(physxCooking);
    REQUIRE(physxCookingPrivate);
    // setup basic stage
    UsdStageRefPtr stage = testParameters.stage;
    const SdfPath meshPath = testParameters.meshPath;
    const SdfPath defaultPrimPath = testParameters.defaultPrimPath;
    UsdGeomMesh usdMesh = testParameters.usdMesh;

    // Setup request
    PhysxCookingComputeRequest request;
    using Options = omni::physx::PhysxCookingComputeRequest::Options;
    request.options.setFlag(Options::kComputeGPUCookingData, true);
    request.options.setFlag(Options::kExecuteCookingOnGPU, testParameters.enableGPUCooking);
    request.options.setFlag(Options::kComputeAsynchronously, testParameters.asynchronous);
    request.mode = PhysxCookingComputeRequest::eMODE_REQUEST_COOKED_DATA;

    if (dataType == PhysxCookingDataType::eTRIANGLE_MESH)
    {
        switch (testParameters.inputType)
        {
        case CookingServiceTestParameters::eInputFabric:
            request.triangulation.needsMaxMaterialIndex = false; // OM-96865: Multi-materials with Fabric not supported
            break;
        case CookingServiceTestParameters::eInputMeshView:
            request.triangulation.needsMaxMaterialIndex = true; // Retrieves result.triangulationMaxMaterialIndex
            break;
        case CookingServiceTestParameters::eInputUSD: {
            request.triangulation.needsMaxMaterialIndex = true; // Retrieves result.triangulationMaxMaterialIndex
            // Create physics materials
            const SdfPath physicsMaterialPath0 = defaultPrimPath.AppendChild(TfToken("physicsMaterial0"));
            UsdShadeMaterial physicsMaterial0 = UsdShadeMaterial::Define(stage, physicsMaterialPath0);

            const SdfPath physicsMaterialPath1 = defaultPrimPath.AppendChild(TfToken("physicsMaterial1"));
            UsdShadeMaterial physicsMaterial1 = UsdShadeMaterial::Define(stage, physicsMaterialPath1);
            UsdPhysicsMaterialAPI::Apply(physicsMaterial0.GetPrim());
            UsdPhysicsMaterialAPI::Apply(physicsMaterial1.GetPrim());
            // Setup materials on mesh to trigger result.triangulationMaxMaterialIndex == 2.
            // Default material is assigned to faces not here included and it has index == 2.
            {
                const SdfPath subsetPath = meshPath.AppendChild(TfToken("subset0"));
                UsdGeomSubset subset = UsdGeomSubset::Define(stage, subsetPath);
                subset.CreateElementTypeAttr().Set(TfToken("face"));
                subset.CreateFamilyNameAttr().Set(UsdShadeTokens->materialBind);
                VtArray<int> indices = { 0 };
                subset.CreateIndicesAttr().Set(indices);
                UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(subset.GetPrim());
                bindingAPI.Bind(physicsMaterial0, UsdShadeTokens->weakerThanDescendants);
            }
            {
                const SdfPath subsetPath = meshPath.AppendChild(TfToken("subset1"));
                UsdGeomSubset subset = UsdGeomSubset::Define(stage, subsetPath);
                subset.CreateElementTypeAttr().Set(TfToken("face"));
                subset.CreateFamilyNameAttr().Set(UsdShadeTokens->materialBind);
                VtArray<int> indices = { 1 };
                subset.CreateIndicesAttr().Set(indices);
                UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(subset.GetPrim());
                bindingAPI.Bind(physicsMaterial1, UsdShadeTokens->weakerThanDescendants);
            }
        }
        break;
        }
    }
    pxr::VtArray<pxr::GfVec3f> usdPoints;
    pxr::VtArray<int> usdIndices;
    pxr::VtArray<int> usdFaces;
    pxr::VtArray<int> usdHoles;
    pxr::VtArray<int> viewMeshMaterials;

    if (testParameters.inputType == CookingServiceTestParameters::eInputMeshView)
    {
        request.dataInputMode = PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_MESH_VIEW;
        usdMesh.GetPointsAttr().Get(&usdPoints);
        usdMesh.GetFaceVertexIndicesAttr().Get(&usdIndices);
        usdMesh.GetFaceVertexCountsAttr().Get(&usdFaces);
        usdMesh.GetHoleIndicesAttr().Get(&usdHoles);
        request.primMeshView.points = { reinterpret_cast<const carb::Float3*>(&usdPoints[0]), usdPoints.size() };
        request.primMeshView.indices = { usdIndices.data(), usdIndices.size() };
        request.primMeshView.faces = { usdFaces.data(), usdFaces.size() };
        request.primMeshView.holeIndices = { usdHoles.data(), usdHoles.size() };
        if (request.triangulation.needsMaxMaterialIndex)
        {
            // Setup materials on mesh to trigger result.triangulationMaxMaterialIndex == 2.
            // Here Default material must be assigned explicitly, as it computed by getMaxMaterialIndex that
            // just loops the facematerials array, finding the max.
            // This is replicating what's done in USD at TriangulateUSDPrim::TriangulateUSDPrim::fillFaceMaterials
            // where faces that don't have a Subset assigned will get the default material index set.
            for (size_t idx = 0; idx < usdFaces.size(); ++idx)
            {
                viewMeshMaterials.push_back(2); // 2 is the default material
            }
            viewMeshMaterials[1] = 1;
            viewMeshMaterials[4] = 0;
            request.primMeshView.faceMaterials = { reinterpret_cast<const uint16_t*>(&viewMeshMaterials[0]),
                                                   viewMeshMaterials.size() };
        }
        request.primMeshMetersPerUnit = 1.0f;
        request.primMeshText = { meshPath.GetText(), strlen(meshPath.GetText()) };
    }
    else
    {
        pxr::UsdUtilsStageCache::Get().Insert(stage);
        request.dataInputMode = PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID;
        request.primId = asInt(meshPath);
        request.primStageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    }

    // Setup caches usage
    request.options.setFlag(Options::kLoadCookedDataFromCache, testParameters.enableLoadFromLocalCache);
    request.options.setFlag(Options::kSaveCookedDataToCache, testParameters.enableLoadFromLocalCache);

    // Determine if we're using ujitso
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    const bool usingUjitso = settings->getAsBool(omni::physx::kSettingUjitsoCollisionCooking);

    // Force triangulation build if requested
    if (testParameters.forceTriangulation)
    {
        request.triangulation.needsVertices = true;
    }

    // Setup callback
    bool callbackHasBeenCalled = false;
    int numRun = 0;
    request.onFinished = [&](const PhysxCookingComputeResult& result) {
        callbackHasBeenCalled = true;
        // If we're getting a cache it
        if (request.options.hasFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously))
        {
            // When the request is asynchronous, we may get a synchronous result anyway if the
            // data was found in the USD or local cache
            if (usingUjitso)
            {
                const bool isNonUjitsoCacheHit =
                    result.resultSource != PhysxCookingComputeResult::eRESULT_CACHE_MISS &&
                    result.resultSource != PhysxCookingComputeResult::eRESULT_CACHE_HIT_UJITSO;
                CHECK(result.isSynchronousResult == isNonUjitsoCacheHit);
            }
            else
            {
                const bool isCacheHit = result.resultSource != PhysxCookingComputeResult::eRESULT_CACHE_MISS;
                CHECK(result.isSynchronousResult == isCacheHit);
            }
        }
        else
        {
            // When request is synchronous, we're going to get  isSynchronousResult == true all the time
            CHECK(result.isSynchronousResult);
        }
        CHECK(result.result == PhysxCookingResult::eVALID);

        // Cache meshkey for next request
        request.meshKey = result.meshKey;

        PxDefaultMemoryInputData inputData(nullptr, 0);

        PxPhysics* pxPhysics = testParameters.pxPhysics;
        // Create the runtime object
        CHECK(result.request->dataType == dataType);
        switch (dataType)
        {
        case PhysxCookingDataType::eCONVEX_MESH: {
            CHECK(result.cookedDataNumElements == 1);
            if (result.cookedDataNumElements == 1)
            {
                PxConvexMesh* pxMesh = pxPhysics->createConvexMesh(fromCookedData(inputData, result.cookedData[0]));
                CHECK(pxMesh != nullptr);
                pxMesh->release();
            }
            break;
        }
        case PhysxCookingDataType::eSDF_TRIANGLE_MESH:
        case PhysxCookingDataType::eTRIANGLE_MESH: {
            CHECK(result.cookedDataNumElements == 1);
            if (request.triangulation.needsMaxMaterialIndex)
            {
                CHECK(result.triangulationMaxMaterialIndex == 2);
            }
            if (result.cookedDataNumElements == 1)
            {
                PxTriangleMesh* pxMesh = pxPhysics->createTriangleMesh(fromCookedData(inputData, result.cookedData[0]));
                CHECK(pxMesh != nullptr);
                pxMesh->release();
            }
            break;
        }
        case PhysxCookingDataType::eCONVEX_DECOMPOSITION: {
            CHECK(result.cookedDataNumElements >= 1);
            if (result.cookedDataNumElements >= 1)
            {
                for (uint64_t idx = 0; idx < result.cookedDataNumElements; idx++)
                {
                    PxConvexMesh* pxMesh = pxPhysics->createConvexMesh(fromCookedData(inputData, result.cookedData[idx]));
                    CHECK(pxMesh != nullptr);
                    pxMesh->release();
                }
            }
            break;
        }
        case PhysxCookingDataType::eSPHERE_FILL: {
            CHECK(result.cookedDataNumElements == 1);
            if (result.cookedDataNumElements == 1)
            {
                uint32_t sphereCount = 0;
                // This is not really safe if you're not sure about the alignment of the raw bytes
                // and it will crash on corrupted data
                const uint32_t* data = reinterpret_cast<const uint32_t*>(result.cookedData[0].data);
                sphereCount = *data++;
                CHECK(sphereCount > 0);
                // format is [sphereCount] and then a list of [positionx][positiony][positionz][radius]...
                const size_t expectedSize = sizeof(uint32_t) + sphereCount * sizeof(carb::Float4);
                CHECK(result.cookedData[0].sizeInBytes == expectedSize);
            }
            break;
        }
        default:
            CHECK(false);
            break;
        }

        if (numRun == 0 || (!testParameters.enableLoadFromLocalCache))
        {
            // During first run or with disabled cache, cooking data will be recomputed from scratch
            CHECK(result.resultSource == PhysxCookingComputeResult::eRESULT_CACHE_MISS);
        }
        else
        {
            // Here we are in second run and we expect local or ujitso cache hit
            if (usingUjitso)
            {
                CHECK(result.resultSource == PhysxCookingComputeResult::eRESULT_CACHE_HIT_UJITSO);
            }
            else
            {
                CHECK(result.resultSource == PhysxCookingComputeResult::eRESULT_CACHE_HIT_LOCAL);
            }
        }
    };
    auto ctx = testParameters.context;
    // Issue cooking task
    // We do two runs to test:
    // - request.meshKey
    // - reading from caches (if they're available)
    for (numRun = 0; numRun < (runTwice ? 2 : 1); ++numRun)
    {
        callbackHasBeenCalled = false;
        if (numRun > 0)
        {
            // After first run we will supply the optional mesh key
            CHECK(request.meshKey != omni::physx::usdparser::MeshKey());
        }
        switch (dataType)
        {
        case PhysxCookingDataType::eCONVEX_MESH: {
            ConvexMeshCookingParams params;
            if (threadIdx)
            {
                params.minThickness *= 1.0f + 0.00001f * threadIdx;
            }
            physxCooking->requestConvexMeshCookedData(ctx, request, params);
            break;
        }
        case PhysxCookingDataType::eSDF_TRIANGLE_MESH: {
            TriangleMeshCookingParams params;
            SdfMeshCookingParams sdfParams;
            if (threadIdx)
            {
                params.simplificationMetric *= 1.0f + 0.00001f * threadIdx;
            }
            CHECK(sdfParams.sdfResolution > 0); // sanity check
            physxCooking->requestSdfMeshCookedData(ctx, request, params, sdfParams);
            break;
        }
        case PhysxCookingDataType::eTRIANGLE_MESH: {
            TriangleMeshCookingParams params;
            if (threadIdx)
            {
                params.simplificationMetric *= 1.0f + 0.00001f * threadIdx;
            }
            physxCooking->requestTriangleMeshCookedData(ctx, request, params);
            break;
        }
        case PhysxCookingDataType::eCONVEX_DECOMPOSITION: {
            ConvexDecompositionCookingParams params;
            if (threadIdx)
            {
                params.minThickness *= 1.0f + 0.00001f * threadIdx;
            }
            physxCooking->requestConvexMeshDecompositionCookedData(ctx, request, params);
            break;
        }
        case PhysxCookingDataType::eSPHERE_FILL: {
            SphereFillCookingParams params;
            physxCooking->requestSphereFillCookedData(ctx, request, params);
            if (threadIdx)
            {
                params.voxelResolution += threadIdx;
            }
            break;
        }
        default:
            CHECK(false);
            break;
        }
        // Check results
        if (!testParameters.asynchronous)
        {
            // In synchronous mode the callback should be called immediately
            // In Asynchronous mode it will be called immediately only during a USD/Local cache hit
            CHECK(callbackHasBeenCalled);
        }

        // If task is asynchronous, let's call pump and wait for it to be done
        if (testParameters.asynchronous)
        {
            const int maxNumSteps = 30000;
            int numSteps = 0;
            while (!callbackHasBeenCalled && numSteps++ < maxNumSteps)
            {
                physxCooking->pumpAsyncContext(ctx);
                tasking->sleep_for(std::chrono::milliseconds(10));
            }
            CHECK(callbackHasBeenCalled);
        }
    }
}

void testCookingServiceParametric(const PhysxCookingDataType::Enum dataType, CookingServiceTestParameters& testParameters)
{
    testParameters.createShared();
    testParameters.createInstanceSpecific("/World/mesh", testParameters.context);
    testCookingServiceParametricImplementation(dataType, testParameters);
    testParameters.destroyInstanceSpecific();
    testParameters.destroyShared();
}

void testAllApproximationTypes(CookingServiceTestParameters& testParameters)
{
    SUBCASE("Convex Mesh")
    {
        testCookingServiceParametric(PhysxCookingDataType::eCONVEX_MESH, testParameters);
    }
    SUBCASE("Triangle Mesh")
    {
        testCookingServiceParametric(PhysxCookingDataType::eTRIANGLE_MESH, testParameters);
    }
    if (testParameters.computeGPUCookingData)
    {
        SUBCASE("SDF CPU Cooking")
        {
            testParameters.enableGPUCooking = false;
            testCookingServiceParametric(PhysxCookingDataType::eSDF_TRIANGLE_MESH, testParameters);
        }
#if USE_PHYSX_GPU
        SUBCASE("SDF GPU Cooking")
        {
            testParameters.enableGPUCooking = true;
            testCookingServiceParametric(PhysxCookingDataType::eSDF_TRIANGLE_MESH, testParameters);
        }
#endif
    }
#if CARB_DEBUG
    // S.C. Not running Sphere and decomposition tests in debug build
#else
    SUBCASE("Sphere Fill")
    {
        testCookingServiceParametric(PhysxCookingDataType::eSPHERE_FILL, testParameters);
    }
    SUBCASE("Convex Decomposition")
    {
        testCookingServiceParametric(PhysxCookingDataType::eCONVEX_DECOMPOSITION, testParameters);
    }
#endif
}

void testComputeGPUCookingData(CookingServiceTestParameters& testParameters)
{
#if USE_PHYSX_GPU
    SUBCASE("gpuCookingData")
    {
        testParameters.computeGPUCookingData = true;
        testAllApproximationTypes(testParameters);
    }
#endif
    SUBCASE("cpuCookingData")
    {
        testParameters.computeGPUCookingData = false;
        testAllApproximationTypes(testParameters);
    }
}

void testLoadFromCaches(CookingServiceTestParameters& testParameters)
{
    SUBCASE("cacheDisabled")
    {
        testParameters.enableLoadFromLocalCache = false;
        testComputeGPUCookingData(testParameters);
    }
    SUBCASE("cacheLocal")
    {
        testParameters.enableLoadFromLocalCache = true;
        testComputeGPUCookingData(testParameters);
    }
}
void testPrimIdOrMeshView(CookingServiceTestParameters& testParameters)
{
    SUBCASE("inputUSD")
    {
        testParameters.inputType = CookingServiceTestParameters::eInputUSD;
        testLoadFromCaches(testParameters);
    }
    SUBCASE("inputFabric")
    {
        testParameters.inputType = CookingServiceTestParameters::eInputFabric;
        testLoadFromCaches(testParameters);
    }
    SUBCASE("inputMeshView")
    {
        testParameters.inputType = CookingServiceTestParameters::eInputMeshView;
        testLoadFromCaches(testParameters);
    }
}

void testSynchronousAsynchronous(CookingServiceTestParameters& testParameters)
{
    SUBCASE("synchronous")
    {
        testParameters.asynchronous = false;
        testPrimIdOrMeshView(testParameters);
    }
    SUBCASE("asynchronous")
    {
        testParameters.asynchronous = true;
        testPrimIdOrMeshView(testParameters);
    }
}

void testCookingServiceCooking()
{
    CookingServiceTestParameters testParameters;
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::physx::IPhysxCookingService* physxCooking = physicsTests.acquirePhysxCookingServiceInterface();
    REQUIRE(physxCooking);
    PhysxCookingAsyncContextParameters asyncParams;
    asyncParams.contextName = { "Test", strlen("Test") };
    testParameters.context = physxCooking->createAsyncContext(asyncParams);
    REQUIRE(testParameters.context);
    testSynchronousAsynchronous(testParameters);

    // the tests should release the context
    REQUIRE(testParameters.context == nullptr);
    // but just to be safe, release it here if it hasn't been already
    if (testParameters.context != nullptr)
    {
        physxCooking->destroyAsyncContext(testParameters.context);
    }
}

static void testMultiThreadedParametric(const bool asynchronous, const bool caching)
{
    // On this test we are creating a swarm of threads to test multithreaded sync cooking
    // Setup and teardown operations that are not thread safe (like releasing mesh cache)
    // are done in CookingServiceTestParameters::createShared / destroyShared
    // This is for example how omni.physx.graph is using the cooking system (as of today, may change in future).

    static const uint32_t numTasks = 8;
    carb::tasking::Future<> futures[numTasks];
    carb::tasking::ITasking* carbTasking = carb::getCachedInterface<carb::tasking::ITasking>();

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    auto physxCooking = physicsTests.acquirePhysxCookingServiceInterface();
    auto physxCookingPrivate = physicsTests.acquirePhysxCookingServicePrivateInterface();
    // Always start with empty cache (we can't do this on single thread)
    physxCookingPrivate->setLocalMeshCacheEnabled(true);
    resetLocalMeshCaches(physxCookingPrivate);

    // Setup a shared stage, PxPhysics and PxFoundation
    CookingServiceTestParameters sharedParameters;
    sharedParameters.createShared();

    // Create thread specific meshes. We copy the sharedParameters and as all threads
    // will share the same stage, we should create the prims in main thread here
    CookingServiceTestParameters threadParameters[numTasks];
    for (uint32_t idx = 0; idx < numTasks; ++idx)
    {
        // CookingServiceTestParameters is not RAII (by design), so we can safely copy it
        // and destruction of its content is done in CookingServiceTestParameters::destroyShared
        threadParameters[idx] = sharedParameters;
        omni::physx::PhysxCookingAsyncContext cookingAsyncContext = nullptr;
        if (asynchronous)
        {
            std::stringstream contexSS;
            contexSS << "Thread" << idx;
            PhysxCookingAsyncContextParameters asyncParams;
            auto name = contexSS.str();
            asyncParams.contextName = { name.c_str(), name.size() };
            // Context is destroyed in destroyInstanceSpecific
            cookingAsyncContext = physxCooking->createAsyncContext(asyncParams);
            REQUIRE(cookingAsyncContext);
        }
        threadParameters[idx].asynchronous = asynchronous; // Force SYNC or ASYNC cooking
        threadParameters[idx].inputType = CookingServiceTestParameters::eInputMeshView; // Do not touch stage cache
        threadParameters[idx].enableLoadFromLocalCache = caching; // Allow use of local cache
        std::stringstream ss;
        ss << "/World/mesh" << idx;
        threadParameters[idx].createInstanceSpecific(ss.str(), cookingAsyncContext);
    }

    // Create the swarm of threads / tasks
    for (uint32_t idx = 0; idx < numTasks; ++idx)
    {
        CookingServiceTestParameters& parameters = threadParameters[idx];
        futures[idx] = carbTasking->addTask(carb::tasking::Priority::eDefault, {}, [parameters, idx]() mutable {
            // Setup multithread safe parameters
            parameters.computeGPUCookingData = false; // To speedup the test for all non SDF cases
            parameters.enableGPUCooking = false; // To speedup the test
            // We cannot call testCookingServiceParametric as we can't safely clear cache from random threads
#if CARB_DEBUG
            // BRG Not running Sphere and decomposition tests in debug build
            const auto selectedTest = idx % 3;
#else
            const auto selectedTest = idx % 5;
#endif
            switch (selectedTest)
            {
            case 0:
                testCookingServiceParametricImplementation(PhysxCookingDataType::eCONVEX_MESH, parameters, idx);
                break;
            case 1:
                testCookingServiceParametricImplementation(PhysxCookingDataType::eTRIANGLE_MESH, parameters, idx);
                break;
            case 2:
#if USE_PHYSX_GPU
                parameters.enableGPUCooking = true;
#else
                parameters.enableGPUCooking = false;
#endif
                parameters.computeGPUCookingData = true;
                testCookingServiceParametricImplementation(PhysxCookingDataType::eSDF_TRIANGLE_MESH, parameters, idx);
                break;
            case 3:
                testCookingServiceParametricImplementation(PhysxCookingDataType::eSPHERE_FILL, parameters, idx);
                break;
            case 4:
                testCookingServiceParametricImplementation(PhysxCookingDataType::eCONVEX_DECOMPOSITION, parameters, idx);
                break;
            }
        });
    }

    // Wait for all tasks to be done
    for (uint32_t idx = 0; idx < numTasks; idx++)
    {
        if (futures[idx].valid())
            futures[idx].wait();
    }

    // Teardown
    for (uint32_t idx = 0; idx < numTasks; idx++)
    {
        threadParameters[idx].destroyInstanceSpecific();
    }
    sharedParameters.destroyShared();
}

void testMultiThreadedSyncAsync(const bool caching)
{
    SUBCASE("synchronous")
    {
        testMultiThreadedParametric(false, caching); // asynchronous == false
    }
    SUBCASE("asynchronous")
    {
        testMultiThreadedParametric(true, caching); // asynchronous == true
    }
}

void testMultiThreaded()
{
    SUBCASE("nonCaching")
    {
        testMultiThreadedSyncAsync(false); // caching == false
    }
    SUBCASE("caching")
    {
        testMultiThreadedSyncAsync(true); // caching == true
    }
}

void testCookingServiceContextAndCancellations()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);
    const SdfPath meshPath = SdfPath("/World/mesh");
    UsdGeomMesh usdMesh = createMeshCapsule(stage, meshPath, 1.0, 1.0);
    const SdfPath meshPath2 = SdfPath("/World/mesh2");
    UsdGeomMesh usdMesh2 = createMeshCapsule(stage, meshPath2, 1.0, 1.0);

    pxr::UsdUtilsStageCache::Get().Insert(stage);

    // We create five requests that will all end up having the same CRC.
    // The first two requests are created in the same context and they will share the same handle.
    // Both of their callbacks will be called when cancelling the first callback.
    // The third request lives on another context and will just proceed to completion with success.
    // The forth request will be cancelled but it will not call callback as we explicitly request it during cancelTask.
    // The fifth request is being waited on before running its pumpAsync and will get its callback called.

    PhysxCookingComputeRequest request1;
    using Options = omni::physx::PhysxCookingComputeRequest::Options;
    request1.options.setFlag(Options::kComputeGPUCookingData, false);
    request1.options.setFlag(Options::kComputeAsynchronously, true);
    // Disable all caching
    request1.options.setFlag(Options::kLoadCookedDataFromCache, false);
    request1.options.setFlag(Options::kSaveCookedDataToCache, false);
    request1.mode = PhysxCookingComputeRequest::eMODE_REQUEST_COOKED_DATA;
    request1.dataInputMode = PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID;
    request1.primId = asInt(meshPath);
    request1.primStageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    PhysxCookingComputeRequest request2 = request1;
    request2.primId = asInt(meshPath2); // We point at another prim that will have the same CRC
    PhysxCookingComputeRequest request3 = request1;
    PhysxCookingComputeRequest request4 = request1;
    PhysxCookingComputeRequest request5 = request1;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::physx::IPhysxCookingService* physxCooking = physicsTests.acquirePhysxCookingServiceInterface();
    omni::physx::IPhysxCookingServicePrivate* physxCookingPrivate =
        physicsTests.acquirePhysxCookingServicePrivateInterface();
    REQUIRE(physxCooking);
    REQUIRE(physxCookingPrivate);
    // Always start with empty cache
    physxCookingPrivate->setLocalMeshCacheEnabled(true);
    resetLocalMeshCaches(physxCookingPrivate);

    PhysxCookingAsyncContextParameters asyncParams1;
    asyncParams1.contextName = { "Context1", strlen("Context1") };
    PhysxCookingAsyncContext asyncContext1 = physxCooking->createAsyncContext(asyncParams1);
    REQUIRE(asyncContext1);
    PhysxCookingAsyncContextParameters asyncParams2;
    asyncParams2.contextName = { "Context2", strlen("Context2") };
    PhysxCookingAsyncContext asyncContext2 = physxCooking->createAsyncContext(asyncParams2);
    REQUIRE(asyncContext2);
    PhysxCookingAsyncContextParameters asyncParams3;
    asyncParams3.contextName = { "Context3", strlen("Context3") };
    PhysxCookingAsyncContext asyncContext3 = physxCooking->createAsyncContext(asyncParams3);
    REQUIRE(asyncContext3);
    PhysxCookingAsyncContextParameters asyncParams4;
    asyncParams4.contextName = { "Context4", strlen("Context4") };
    PhysxCookingAsyncContext asyncContext4 = physxCooking->createAsyncContext(asyncParams4);
    REQUIRE(asyncContext4);

    bool result1GotCancelled = false;
    request1.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        result1GotCancelled = result.result == omni::physx::PhysxCookingResult::eERROR_CANCELED;
    };

    bool result2GotCancelled = false;
    request2.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        result2GotCancelled = result.result == omni::physx::PhysxCookingResult::eERROR_CANCELED;
    };
    bool result3GotValid = false;
    request3.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        result3GotValid = result.result == omni::physx::PhysxCookingResult::eVALID;
    };
    bool result4GotCalled = false;
    request4.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) { result4GotCalled = true; };
    bool result5GotValid = false;
    request5.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        result5GotValid = result.result == omni::physx::PhysxCookingResult::eVALID;
    };
    ConvexMeshCookingParams params;
    auto request1Handle = physxCooking->requestConvexMeshCookedData(asyncContext1, request1, params);
    auto request2Handle = physxCooking->requestConvexMeshCookedData(asyncContext1, request2, params);
    auto request3Handle = physxCooking->requestConvexMeshCookedData(asyncContext2, request3, params);
    auto request4Handle = physxCooking->requestConvexMeshCookedData(asyncContext3, request4, params);
    auto request5Handle = physxCooking->requestConvexMeshCookedData(asyncContext4, request5, params);

    CHECK(request1Handle == request2Handle); // even if prim2 points to the same prim, it will return the same task
    CHECK(request3Handle != request1Handle);
    CHECK(request4Handle != request3Handle);
    CHECK(request4Handle != request1Handle);
    CHECK(request5Handle != request4Handle);
    CHECK(request5Handle != request3Handle);
    CHECK(request5Handle != request1Handle);

    bool request5Cooked = physxCooking->waitForTaskToFinish(request5Handle, -1);
    CHECK(request5Cooked);
    physxCooking->cancelTask(request1Handle, true);
    physxCooking->cancelTask(request4Handle, false); // with false it will not call callback

    CHECK(physxCooking->pumpAsyncContext(nullptr) == 0);

    while (physxCooking->pumpAsyncContext(asyncContext1) != 0)
    {
    }
    while (physxCooking->pumpAsyncContext(asyncContext2) != 0)
    {
    }
    while (physxCooking->pumpAsyncContext(asyncContext3) != 0)
    {
    }
    while (physxCooking->pumpAsyncContext(asyncContext4) != 0)
    {
    }

    CHECK(result1GotCancelled);
    CHECK(result2GotCancelled);
    CHECK(result3GotValid);
    CHECK(!result4GotCalled);
    CHECK(result5GotValid);

    physxCooking->destroyAsyncContext(asyncContext1);
    physxCooking->destroyAsyncContext(asyncContext2);
    physxCooking->destroyAsyncContext(asyncContext3);
    physxCooking->destroyAsyncContext(asyncContext4);
    pxr::UsdUtilsStageCache::Get().Erase(stage);
}

void testCookingEmptyMesh()
{
    // We create an empty mesh and expect cooking to return eERROR_INVALID_PRIM error
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);
    const SdfPath meshPath = SdfPath("/World/mesh");
    UsdGeomMesh usdMesh = createMeshCapsule(stage, meshPath, 1.0, 1.0);
    usdMesh.GetPointsAttr().Clear(); // Clear all points, creating an empty mesh
    // We could also clear FaceVertexCounts
    // usdMesh.GetFaceVertexCountsAttr().Clear(); // Clear all faces, creating an empty mesh
    pxr::UsdUtilsStageCache::Get().Insert(stage);

    PhysxCookingComputeRequest request;
    using Options = omni::physx::PhysxCookingComputeRequest::Options;
    request.options.setFlag(Options::kComputeGPUCookingData, false);
    request.options.setFlag(Options::kComputeAsynchronously, false);
    request.mode = PhysxCookingComputeRequest::eMODE_REQUEST_COOKED_DATA;
    request.dataInputMode = PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID;
    request.primId = asInt(meshPath);
    request.primStageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::physx::IPhysxCookingService* physxCooking = physicsTests.acquirePhysxCookingServiceInterface();
    omni::physx::IPhysxCookingServicePrivate* physxCookingPrivate =
        physicsTests.acquirePhysxCookingServicePrivateInterface();
    REQUIRE(physxCooking);
    REQUIRE(physxCookingPrivate);
    // Always start with empty cache
    physxCookingPrivate->setLocalMeshCacheEnabled(true);
    resetLocalMeshCaches(physxCookingPrivate);

    bool gotCorrectResult = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        gotCorrectResult = result.result == omni::physx::PhysxCookingResult::eERROR_INVALID_PRIM;
        gotCorrectResult &= result.requestSource == omni::physx::PhysxCookingComputeResult::eREQUEST_SOURCE_USD;
    };

    ConvexMeshCookingParams params;
    auto requestHandle = physxCooking->requestConvexMeshCookedData(nullptr, request, params);
    CHECK(gotCorrectResult);

    // Test Mesh View
    pxr::VtArray<pxr::GfVec3f> usdPoints;
    pxr::VtArray<int> usdIndices;
    pxr::VtArray<int> usdFaces;
    pxr::VtArray<int> usdHoles;
    pxr::VtArray<int> viewMeshMaterials;


    request.dataInputMode = PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_MESH_VIEW;
    usdMesh.GetPointsAttr().Get(&usdPoints);
    usdMesh.GetFaceVertexIndicesAttr().Get(&usdIndices);
    usdMesh.GetFaceVertexCountsAttr().Get(&usdFaces);
    usdMesh.GetHoleIndicesAttr().Get(&usdHoles);
    request.primMeshView.points = { reinterpret_cast<const carb::Float3*>(&usdPoints[0]), usdPoints.size() };
    request.primMeshView.indices = { usdIndices.data(), usdIndices.size() };
    request.primMeshView.faces = { usdFaces.data(), usdFaces.size() };
    request.primMeshView.holeIndices = { usdHoles.data(), usdHoles.size() };

    gotCorrectResult = false;
    resetLocalMeshCaches(physxCookingPrivate);
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        gotCorrectResult = result.result == omni::physx::PhysxCookingResult::eERROR_INVALID_PRIM;
        gotCorrectResult &= result.requestSource == omni::physx::PhysxCookingComputeResult::eREQUEST_SOURCE_MESHVIEW;
    };

    requestHandle = physxCooking->requestConvexMeshCookedData(nullptr, request, params);
    CHECK(gotCorrectResult);

#if 0 
    // OM-117937: Disabling since on f158b1cc (107.0) Fabric has changed behaviour on prefetch.
    // Now if an attribute has not authored value and no default it's not prefetched.
    // For this reason the test below is failing, as it will not be able to discern a missing prim from a 
    // prim that exists in fabric with all of the required attributes (points, vertices, faces) but with them not
    // being a proper mesh as all these arrays are zero sized.

    // Test Fabric
    request.dataInputMode = PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID;
    auto iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    auto iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();
    auto iFabricUsd = carb::getCachedInterface<omni::fabric::IFabricUsd>();
    iSimStageWithHistory->getOrCreate(request.primStageId, 1, { 1, 30 }, omni::fabric::GpuComputeType::eNone);
    iStageReaderWriter->create(request.primStageId, 0);
    auto stageReaderWriterId = iStageReaderWriter->get(request.primStageId);
    auto fabricId = iStageReaderWriter->getFabricId(stageReaderWriterId);
    std::set<omni::fabric::TokenC> filter = {};
    iFabricUsd->prefetchPrimToFabric(
        fabricId, meshPath, usdMesh.GetPrim(), filter, false, true, pxr::UsdTimeCode::Default());
    stage->RemovePrim(meshPath);

    gotCorrectResult = false;
    resetLocalMeshCaches(physxCookingPrivate);
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        gotCorrectResult = result.result == omni::physx::PhysxCookingResult::eERROR_INVALID_PRIM;
        gotCorrectResult &= result.requestSource == omni::physx::PhysxCookingComputeResult::eREQUEST_SOURCE_FABRIC;
    };
    requestHandle = physxCooking->requestConvexMeshCookedData(nullptr, request, params);
    CHECK(gotCorrectResult);
#endif
    pxr::UsdUtilsStageCache::Get().Erase(stage);
}

struct ThreadingHelper
{
    ThreadingHelper(carb::tasking::ITasking* tasking,
                    PhysicsTest& physicsTests,
                    size_t initialParamCount = 8,
                    size_t resizeGrowth = 4)
        : m_tasking(tasking), m_resizeGrowth(resizeGrowth), m_nextParamsNum(0)
    {
        m_physxCooking = physicsTests.acquirePhysxCookingServiceInterface();

        // Setup a shared stage, PxPhysics and PxFoundation
        m_sharedParameters.createShared();
        resizeParams(initialParamCount);
    }

    ~ThreadingHelper()
    {
        // Teardown
        for (CookingServiceTestParameters& params : m_threadParameters)
        {
            params.destroyInstanceSpecific();
        }
        m_sharedParameters.destroyShared();
    }

    carb::tasking::Future<> createTask(PhysxCookingDataType::Enum type, bool asynchronous)
    {
        if (m_nextParamsNum >= m_threadParameters.size())
        {
            resizeParams(m_nextParamsNum + m_resizeGrowth);
        }

        CookingServiceTestParameters& params = m_threadParameters[m_nextParamsNum];
        params.asynchronous = asynchronous;
        params.forceTriangulation = true;

        const uint32_t idx = (uint32_t)m_nextParamsNum++;

        return m_tasking->addTask(carb::tasking::Priority::eDefault, {}, [type, params, idx]() mutable {
            testCookingServiceParametricImplementation(type, params, idx, false);
        });
    }

private:
    void resizeParams(size_t newSize)
    {
        const size_t oldSize = m_threadParameters.size();
        if (newSize > oldSize)
        {
            m_threadParameters.resize(newSize);
            for (size_t i = oldSize; i < newSize; ++i)
            {
                createThreadParams(m_threadParameters[i], i);
            }
        }
    }

    void createThreadParams(CookingServiceTestParameters& threadParameters, size_t idx)
    {
        // CookingServiceTestParameters is not RAII (by design), so we can safely copy it
        // and destruction of its content is done in CookingServiceTestParameters::destroyShared
        threadParameters = m_sharedParameters;
        PhysxCookingAsyncContext cookingAsyncContext = nullptr;

        std::stringstream contexSS;
        contexSS << "Thread" << idx;
        PhysxCookingAsyncContextParameters asyncParams;
        auto name = contexSS.str();
        asyncParams.contextName = { name.c_str(), name.size() };
        // Context is destroyed in destroyInstanceSpecific
        cookingAsyncContext = m_physxCooking->createAsyncContext(asyncParams);
        REQUIRE(cookingAsyncContext);

        threadParameters.asynchronous = true; // Force async cooking
        threadParameters.inputType = CookingServiceTestParameters::eInputMeshView; // Do not touch stage cache
        threadParameters.enableLoadFromLocalCache = true; // Allow use of local cache
        std::stringstream ss;
        ss << "/World/mesh" << idx;
        threadParameters.createInstanceSpecific(ss.str(), cookingAsyncContext);

        // Setup multithread safe parameters
        threadParameters.computeGPUCookingData = false; // To speedup the test for all non SDF cases
        threadParameters.enableGPUCooking = false; // To speedup the test

#if USE_PHYSX_GPU
        threadParameters.enableGPUCooking = true;
#else
        threadParameters.enableGPUCooking = false;
#endif
    }

    carb::tasking::ITasking* m_tasking;
    IPhysxCookingService* m_physxCooking;
    CookingServiceTestParameters m_sharedParameters;
    std::vector<CookingServiceTestParameters> m_threadParameters;
    size_t m_resizeGrowth;
    size_t m_nextParamsNum;
};

void testCookingCacheChange()
{
    carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>();

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysxCookingServicePrivate* physxCookingPrivate = physicsTests.acquirePhysxCookingServicePrivateInterface();
    // Always start with empty cache (we can't do this on single thread)
    physxCookingPrivate->setLocalMeshCacheEnabled(true);
    resetLocalMeshCaches(physxCookingPrivate);

    ThreadingHelper helper(tasking, physicsTests);
    typedef carb::tasking::Future<> Task;

    /**************************************************************************************************************
     * 1) Make sure a cache change with no processes running executes quickly
     **************************************************************************************************************/

    // Set local cache size
    uint32_t newCacheSize = 100;
    physxCookingPrivate->setLocalMeshCacheSize(newCacheSize);

    // It should quickly take effect
    tasking->sleep_for(std::chrono::milliseconds(10));
    CHECK(physxCookingPrivate->getLocalMeshCacheSize() == newCacheSize);

    uint32_t oldCacheSize = newCacheSize;


    /**************************************************************************************************************
     * 2) Run several long-running tasks and request a cache change.  Make sure the change does not happen
     * quickly.
     **************************************************************************************************************/

    // Container of our long-running tasks
    std::vector<Task> tasks;

    // Create a few long-running tasks
    tasks.push_back(helper.createTask(PhysxCookingDataType::eSDF_TRIANGLE_MESH, true));
    tasks.push_back(helper.createTask(PhysxCookingDataType::eSDF_TRIANGLE_MESH, true));
    tasks.push_back(helper.createTask(PhysxCookingDataType::eSDF_TRIANGLE_MESH, true));

    // Wait for tasks to start up
    // Note, in debug it takes at least 3ms for the first task to start
    tasking->sleep_for(std::chrono::milliseconds(10));

    // Now request a cache size change
    physxCookingPrivate->setLocalMeshCacheSize(++newCacheSize);

    // Before the cooking finishes, the cache size should not have changed
    CHECK(physxCookingPrivate->getLocalMeshCacheSize() == oldCacheSize);


    /**************************************************************************************************************
     * 3) Wait for the long-running tasks to finish.  While waiting, make sure the cache change does not take
     * effect until all the tasks are complete.
     **************************************************************************************************************/

    // (cache size changed) => (task count == 0)
    bool cacheSizeChangedImpliesTasksDone = true; // Check this condition after tasks complete

    // Now wait for all tasks to complete
    constexpr int maxIter = 6000; // Wait this many iterations max
    for (int count = maxIter; tasks.size() && count--;)
    {
        const uint32_t currentCacheSize = physxCookingPrivate->getLocalMeshCacheSize();
        const bool cacheSizeChanged = currentCacheSize != oldCacheSize;
        for (auto it = tasks.begin(); it != tasks.end();)
        {
            Task& task = *it;
            // Give the task plenty of time to end so we know that resources will have been returned
            if (!task.valid() || task.wait_for(std::chrono::milliseconds(10)))
                it = tasks.erase(it);
            else
                ++it;
        }

        if (cacheSizeChanged && tasks.size())
            cacheSizeChangedImpliesTasksDone = false;
    }

    CHECK(tasks.empty()); // If not empty then the tasks didn't complete in the maxIter iterations

    // Make sure the cache change did not take effect before the tasks completed
    CHECK(cacheSizeChangedImpliesTasksDone);

    // Check to see if the cache size has updated
    CHECK(physxCookingPrivate->getLocalMeshCacheSize() == newCacheSize);

    oldCacheSize = newCacheSize;


    /**************************************************************************************************************
     * 4) Run a long task and in the meanwhile make several cache change requests.  Once the task completes
     * the last change should take effect.
     **************************************************************************************************************/

    // Create a new long-running task and hammer the requests
    Task task = helper.createTask(PhysxCookingDataType::eSDF_TRIANGLE_MESH, true);
    tasking->sleep_for(std::chrono::milliseconds(10));
    physxCookingPrivate->setLocalMeshCacheSize(++newCacheSize);
    physxCookingPrivate->setLocalMeshCacheSize(++newCacheSize);
    physxCookingPrivate->setLocalMeshCacheSize(++newCacheSize);

    // The request should not have taken yet
    CHECK(physxCookingPrivate->getLocalMeshCacheSize() == oldCacheSize);

    cacheSizeChangedImpliesTasksDone = true;
    bool cacheSizeChangedToLastValue = true;

    // Wait for it to complete
    bool taskDone = false;
    for (int count = maxIter; !taskDone && count--;)
    {
        const uint32_t currentCacheSize = physxCookingPrivate->getLocalMeshCacheSize();
        const bool cacheSizeChanged = currentCacheSize != oldCacheSize;
        taskDone = !task.valid() || task.wait_for(std::chrono::milliseconds(10));
        if (cacheSizeChanged && !taskDone)
        {
            cacheSizeChangedImpliesTasksDone = false;
            if (currentCacheSize != newCacheSize)
                cacheSizeChangedToLastValue = false;
        }
    }

    CHECK(taskDone); // If false then the tasks didn't complete in the maxIter iterations

    // Now the param should be the last set one
    if (physxCookingPrivate->getLocalMeshCacheSize() != newCacheSize)
        cacheSizeChangedToLastValue = false;

    CHECK(cacheSizeChangedImpliesTasksDone);
    CHECK(cacheSizeChangedToLastValue);

    oldCacheSize = newCacheSize;


    /**************************************************************************************************************
     * 5.1) Run a long task and in the meanwhile make a cache change request followed by a fast task.  The
     * long task should always finish first.
     *
     * 5.2) Same as (5.1) except now we follow the fast task creation with another request which has parameters
     * equal to that for the current resources, so nothing needs to be done.  Now the fast task should finish first.
     *
     * 5.3) Repeat (5.1) except we make the second task synchronous, to make sure the infinite timeout doesn't cause
     * a freeze-up.
     *
     * 5.4) Repeat (5.2) except we make the second task synchronous.
     **************************************************************************************************************/

    for (int run : { 1, 2, 3, 4 })
    {
        // Create a new long-running task
        Task task0 = helper.createTask(PhysxCookingDataType::eSDF_TRIANGLE_MESH, true);
        tasking->sleep_for(std::chrono::milliseconds(10));

        // Request a cache change
        physxCookingPrivate->setLocalMeshCacheSize(++newCacheSize);
        tasking->sleep_for(std::chrono::milliseconds(10));

        // On even runs, revert the cache change immediately
        if (!(run % 2))
            physxCookingPrivate->setLocalMeshCacheSize(--newCacheSize);

        // Create a fast task
        const bool async = run <= 2; // The first two runs are async, 2nd two sync
        Task task1 = helper.createTask(PhysxCookingDataType::eCONVEX_MESH, async);

        // Wait until they complete; make sure the order is correct
        int taskFinishOrder = 0; // 1 => task0 before task1, -1 => task1 before task0, 0 => unknown
        bool tasksDone = false;
        for (int count = maxIter; !tasksDone && count--;)
        {
            const bool task0Done = !task0.valid() || task0.wait_for(std::chrono::milliseconds(10));
            const bool task1Done = !task1.valid() || task1.wait_for(std::chrono::milliseconds(10));
            tasksDone = task0Done && task1Done;
            if (!taskFinishOrder && task0Done != task1Done)
                taskFinishOrder = (int)task0Done - (int)task1Done;
        }

        CHECK(tasksDone); // If false then the tasks didn't complete in the maxIter iterations

        CHECK(physxCookingPrivate->getLocalMeshCacheSize() == newCacheSize);

        switch (run)
        {
        case 1:
            CHECK(taskFinishOrder > 0);
            break;
        case 2:
            CHECK(taskFinishOrder < 0);
            break;
        case 3:
            CHECK(taskFinishOrder > 0);
            break;
        case 4:
            CHECK(taskFinishOrder < 0);
            break;
        }
    }

    oldCacheSize = newCacheSize; // Putting this here in case more tests follow
}

void testAll()
{
    SUBCASE("Empty Mesh")
    {
        testCookingEmptyMesh();
    }
    SUBCASE("context/cancellations")
    {
        testCookingServiceContextAndCancellations();
    }
    SUBCASE("cooking")
    {
        testCookingServiceCooking();
    }
    SUBCASE("multithreaded")
    {
        testMultiThreaded();
    }
}

TEST_CASE("Cooking Service",
          "[omniphysics]"
          "[component=OmniPhysics][owner=scristiano][priority=mandatory]")
{
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();

    // Save off current ujitso settings
    const bool currentUjitsoSetting = settings->getAsBool(kSettingUjitsoCollisionCooking);
    const int64_t currentDevKeySetting = settings->getAsInt64(kSettingUjitsoCookingDevKey);
    const int32_t currentMaxProcessSetting = settings->getAsInt(kSettingUjitsoCookingMaxProcessCount);

    SUBCASE("noUjitso")
    {
        settings->setBool(kSettingUjitsoCollisionCooking, false);
        testAll();
    }
    SUBCASE("withUjitso")
    {
        settings->setBool(kSettingUjitsoCollisionCooking, true);
        settings->setInt(kSettingUjitsoCookingMaxProcessCount, 16);
        testAll();
    }

    // Restore ujitso settings
    settings->setBool(kSettingUjitsoCollisionCooking, currentUjitsoSetting);
    settings->setInt64(kSettingUjitsoCookingDevKey, currentDevKeySetting);
    settings->setInt(kSettingUjitsoCookingMaxProcessCount, currentMaxProcessSetting);
}
