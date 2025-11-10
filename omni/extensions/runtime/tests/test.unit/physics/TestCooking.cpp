// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <chrono>
#include <thread>

#include "../common/TestHelpers.h"

#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxTests.h>

#include "PhysicsTools.h"

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Init test
TEST_CASE("Convex Cooking",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    omni::physx::IPhysxCooking* physxCooking = physicsTests.acquirePhysxCookingInterface();
    REQUIRE(physxCooking);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    
    const SdfPath meshPath = SdfPath("/World/mesh");
    UsdGeomMesh usdMesh = createMeshBox(stage, meshPath);

    physxSim->attachStage(stageId);

    SECTION("Create Convex Mesh")
    {
        ConvexMeshData meshData;
        const bool retVal = physxCooking->createConvexMesh(meshPath, 64, meshData);
        REQUIRE(retVal);

        CHECK(meshData.numPolygons == 6);
        CHECK(meshData.numVertices == 8);
    }

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}


TEST_CASE("Pre-Cooking",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    omni::physx::IPhysxCooking* physxCooking = physicsTests.acquirePhysxCookingInterface();
    REQUIRE(physxCooking);
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    omni::physx::IPhysxUnitTests* physxTests = physicsTests.acquirePhysxUnitTestInterface();
    REQUIRE(physxTests);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    const long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    float halfSize = 100.0f;
    GfVec3f position(200.0f);
    GfVec3f scale(1.0f, 2.0f, 3.0f);
    const SdfPath shapePath = defaultPrimPath.AppendChild(TfToken("shape"));
    UsdGeomMesh shape = createConcaveMesh(stage, shapePath, halfSize);

    physxSim->attachStage(stageId);

    struct CookedData
    {
        uint64_t stageId{ 0ull };
        uint64_t cookedPrim { 0ull };
    };

    CookedData cookedData;

    IPhysxCookingCallback cb = { (void*)(&cookedData), nullptr};
    cb.cookingFinishedCallback = [](uint64_t stageIdIn, uint64_t primPathIn, PhysxCookingResult::Enum result, void* userData)
    {
        CHECK(result == PhysxCookingResult::eVALID);
        CookedData* cd = (CookedData*)userData;
        cd->cookedPrim = primPathIn;
        cd->stageId = stageIdIn;
    };

    physxCooking->releaseLocalMeshCache();

    SECTION("Convex Mesh - synchronous")
    {
        PxPhysics* pxPhysics = reinterpret_cast<PxPhysics*>(physx->getPhysXPtr(pxr::SdfPath(), omni::physx::ePTPhysics));
        REQUIRE(pxPhysics);

        // check we have initial hulls created
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);

        ConvexMeshCookingParams params;
        const bool retVal = physxCooking->precookMesh(stageId, sdfPathToInt(shapePath), params, nullptr);
        CHECK(retVal);
        // precook, we should have one more hull
        CHECK(pxPhysics->getNbConvexMeshes() == 10);
        CHECK(pxPhysics->getNbShapes() == 0);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexHull);

        physxSim->simulate(1.0f/60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        // create the object in physics, no new hull should be created, just shape
        CHECK(pxPhysics->getNbConvexMeshes() == 10);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);
        CHECK(pxPhysics->getNbShapes() == 1);
    }

    SECTION("Convex Mesh - asynchronous")
    {
        PxPhysics* pxPhysics = reinterpret_cast<PxPhysics*>(physx->getPhysXPtr(pxr::SdfPath(), omni::physx::ePTPhysics));
        REQUIRE(pxPhysics);

        // check we have initial hulls created
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);

        ConvexMeshCookingParams params;
        const bool retVal = physxCooking->precookMesh(stageId, sdfPathToInt(shapePath), params, &cb);

        const uint64_t cookingTimeout = 1000ull;
        const uint64_t sleepStep = 1ull;
        uint64_t time = 0ull;
        while (time < cookingTimeout && cookedData.cookedPrim == 0ull)
        {
            physxTests->updateCooking();

            std::this_thread::sleep_for(std::chrono::milliseconds(sleepStep));
            time += sleepStep;
        }
        
        CHECK(cookedData.cookedPrim == sdfPathToInt(shapePath));
        CHECK(cookedData.stageId == stageId);
        // precook, we should have one more hull
        CHECK(pxPhysics->getNbConvexMeshes() == 10);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);
        CHECK(pxPhysics->getNbShapes() == 0);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexHull);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        // create the object in physics, no new hull should be created, just shape
        CHECK(pxPhysics->getNbConvexMeshes() == 10);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);
        CHECK(pxPhysics->getNbShapes() == 1);
    }

#if CARB_DEBUG
    // A.B. Convex decomposition cooking is very slow, we should not run it in debug

#else
    SECTION("Convex Decomposition - synchronous")
    {
        PxPhysics* pxPhysics = reinterpret_cast<PxPhysics*>(physx->getPhysXPtr(pxr::SdfPath(), omni::physx::ePTPhysics));
        REQUIRE(pxPhysics);

        // check we have initial meshes created
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);

        ConvexDecompositionCookingParams params;
        const bool retVal = physxCooking->precookMesh(stageId, sdfPathToInt(shapePath), params, nullptr);
        CHECK(retVal);
        // precook, we should have more convex meshes
        CHECK(pxPhysics->getNbConvexMeshes() > 9);
        const PxU32 numConvexMeshes = pxPhysics->getNbConvexMeshes();
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);
        CHECK(pxPhysics->getNbShapes() == 0);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexDecomposition);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        // create the object in physics, no new hull should be created, just shape
        CHECK(pxPhysics->getNbConvexMeshes() == numConvexMeshes);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);
        CHECK(pxPhysics->getNbShapes() > 0);
    }

    SECTION("Convex Decomposition - asynchronous")
    {
        PxPhysics* pxPhysics = reinterpret_cast<PxPhysics*>(physx->getPhysXPtr(pxr::SdfPath(), omni::physx::ePTPhysics));
        REQUIRE(pxPhysics);

        // check we have initial meshes created
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);

        ConvexDecompositionCookingParams params;
        const bool retVal = physxCooking->precookMesh(stageId, sdfPathToInt(shapePath), params, &cb);

        const uint64_t cookingTimeout = 50000ull;
        const uint64_t sleepStep = 1ull;
        uint64_t time = 0ull;
        while (time < cookingTimeout && cookedData.cookedPrim == 0ull)
        {
            physxTests->updateCooking();

            std::this_thread::sleep_for(std::chrono::milliseconds(sleepStep));
            time += sleepStep;
        }

        CHECK(cookedData.cookedPrim == sdfPathToInt(shapePath));
        CHECK(cookedData.stageId == stageId);
        // precook, we should have more convex meshes
        CHECK(pxPhysics->getNbConvexMeshes() > 9);
        const PxU32 numConvexMeshes = pxPhysics->getNbConvexMeshes();
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);
        CHECK(pxPhysics->getNbShapes() == 0);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexDecomposition);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        // create the object in physics, no new hull should be created, just shape
        CHECK(pxPhysics->getNbConvexMeshes() == numConvexMeshes);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);
        CHECK(pxPhysics->getNbShapes() > 0);
    }
#endif

    // We cant really test this for validity
    //SECTION("Sphere Fill - synchronous")
    //{
    //    PxPhysics* pxPhysics = reinterpret_cast<PxPhysics*>(physx->getPhysXPtr(pxr::SdfPath(), omni::physx::ePTPhysics));
    //    REQUIRE(pxPhysics);

    //    // check we have initial meshes created
    //    CHECK(pxPhysics->getNbConvexMeshes() == 9);
    //    CHECK(pxPhysics->getNbTriangleMeshes() == 0);

    //    SphereFillCookingParams params;
    //    const bool retVal = physxCooking->precookMesh(stageId, sdfPathToInt(shapePath), params, nullptr);
    //    CHECK(retVal);
    //    // precook, we should have more convex meshes
    //    CHECK(pxPhysics->getNbConvexMeshes() == 9);        
    //    CHECK(pxPhysics->getNbTriangleMeshes() == 0);
    //    CHECK(pxPhysics->getNbShapes() == 0);

    //    UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
    //    UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
    //    meshAPI.CreateApproximationAttr().Set(TfToken("sphereFill"));

    //    physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
    //    physxSim->fetchResults();

    //    // create the object in physics, no new hull should be created, just shape
    //    CHECK(pxPhysics->getNbConvexMeshes() == 9);
    //    CHECK(pxPhysics->getNbTriangleMeshes() == 0);
    //    CHECK(pxPhysics->getNbShapes() > 0);
    //}

    SECTION("Triangle Mesh - synchronous")
    {
        PxPhysics* pxPhysics = reinterpret_cast<PxPhysics*>(physx->getPhysXPtr(pxr::SdfPath(), omni::physx::ePTPhysics));
        REQUIRE(pxPhysics);

        // check we have initial meshes created
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);

        TriangleMeshCookingParams params;
        const bool retVal = physxCooking->precookMesh(stageId, sdfPathToInt(shapePath), params, nullptr);
        CHECK(retVal);
        // precook, we should have one more trimesh
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 1);
        CHECK(pxPhysics->getNbShapes() == 0);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->none);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        // create the object in physics, no new hull should be created, just shape
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 1);
        CHECK(pxPhysics->getNbShapes() == 1);
    }

    SECTION("Triangle Mesh - asynchronous")
    {
        PxPhysics* pxPhysics = reinterpret_cast<PxPhysics*>(physx->getPhysXPtr(pxr::SdfPath(), omni::physx::ePTPhysics));
        REQUIRE(pxPhysics);

        // check we have initial meshes created
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);

        TriangleMeshCookingParams params;
        const bool retVal = physxCooking->precookMesh(stageId, sdfPathToInt(shapePath), params, &cb);

        const uint64_t cookingTimeout = 1000ull;
        const uint64_t sleepStep = 1ull;
        uint64_t time = 0ull;
        while (time < cookingTimeout && cookedData.cookedPrim == 0ull)
        {
            physxTests->updateCooking();

            std::this_thread::sleep_for(std::chrono::milliseconds(sleepStep));
            time += sleepStep;
        }

        CHECK(cookedData.cookedPrim == sdfPathToInt(shapePath));
        CHECK(cookedData.stageId == stageId);
        // precook, we should have one more trimesh
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 1);
        CHECK(pxPhysics->getNbShapes() == 0);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->none);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        // create the object in physics, no new hull should be created, just shape
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 1);
        CHECK(pxPhysics->getNbShapes() == 1);
    }

#if CARB_DEBUG
    // A.B. SDF cooking is very slow, we should not run it in debug

#else
#if USE_PHYSX_GPU
    SECTION("Sdf Triangle Mesh - synchronous")
    {
        PxPhysics* pxPhysics = reinterpret_cast<PxPhysics*>(physx->getPhysXPtr(pxr::SdfPath(), omni::physx::ePTPhysics));
        REQUIRE(pxPhysics);

        // check we have initial meshes created
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);

        SdfMeshCookingParams params;
        const bool retVal = physxCooking->precookMesh(stageId, sdfPathToInt(shapePath), params, nullptr);
        CHECK(retVal);
        // precook, we should have one more trimesh
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 1);
        CHECK(pxPhysics->getNbShapes() == 0);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        PhysxSchemaPhysxSDFMeshCollisionAPI sdfMeshAPI = PhysxSchemaPhysxSDFMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(PhysxSchemaTokens->sdf);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        // create the object in physics, no new hull should be created, just shape
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 1);
        CHECK(pxPhysics->getNbShapes() == 1);
    }

    SECTION("Sdf Triangle Mesh - asynchronous")
    {
        PxPhysics* pxPhysics = reinterpret_cast<PxPhysics*>(physx->getPhysXPtr(pxr::SdfPath(), omni::physx::ePTPhysics));
        REQUIRE(pxPhysics);

        // check we have initial meshes created
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 0);

        SdfMeshCookingParams params;
        const bool retVal = physxCooking->precookMesh(stageId, sdfPathToInt(shapePath), params, &cb);

        const uint64_t cookingTimeout = 2000ull;
        const uint64_t sleepStep = 1ull;
        uint64_t time = 0ull;
        while (time < cookingTimeout && cookedData.cookedPrim == 0ull)
        {
            physxTests->updateCooking();

            std::this_thread::sleep_for(std::chrono::milliseconds(sleepStep));
            time += sleepStep;
        }

        CHECK(cookedData.cookedPrim == sdfPathToInt(shapePath));
        CHECK(cookedData.stageId == stageId);
        // precook, we should have one more trimesh
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 1);
        CHECK(pxPhysics->getNbShapes() == 0);

        UsdPhysicsCollisionAPI::Apply(shape.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(shape.GetPrim());
        PhysxSchemaPhysxSDFMeshCollisionAPI sdfMeshAPI = PhysxSchemaPhysxSDFMeshCollisionAPI::Apply(shape.GetPrim());
        meshAPI.CreateApproximationAttr().Set(PhysxSchemaTokens->sdf);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        // create the object in physics, no new hull should be created, just shape
        CHECK(pxPhysics->getNbConvexMeshes() == 9);
        CHECK(pxPhysics->getNbTriangleMeshes() == 1);
        CHECK(pxPhysics->getNbShapes() == 1);
    }
#endif
#endif

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
