// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <chrono>
#include <carb/tasking/ITasking.h>

#include "../common/TestHelpers.h"
#include "../../common/Tools.h"

#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxTests.h>

#include "PhysicsTools.h"

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Init test
TEST_CASE("Collision Representation",
          "[omniphysics]"
          "[component=OmniPhysics][owner=scristiano][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    omni::physx::IPhysxCooking* physxCooking = physicsTests.acquirePhysxCookingInterface();
    REQUIRE(physxCooking);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    omni::physx::IPhysxUnitTests* physxTests = physicsTests.acquirePhysxUnitTestInterface();
    REQUIRE(physxTests);
    carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>();
    REQUIRE(tasking);

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

    physxCooking->releaseLocalMeshCache();

    PhysxCollisionRepresentationRequest request;
    request.options.setFlag(PhysxCollisionRepresentationRequest::Options::kComputeAsynchronously, true);
    request.stageId = stageId;
    request.collisionPrimId = asInt(meshPath);

    int numResultCalled = 0;
    SECTION("Missing Collision API")
    {
        auto onResult = [&](PhysxCollisionRepresentationResult::Enum result,
                            const PhysxCollisionRepresentationConvexResult& data) {
            // Failure because the specific prim doesn't have Collision API
            REQUIRE(result == PhysxCollisionRepresentationResult::eRESULT_ERROR_INVALID_PARSING);
            numResultCalled++;
        };
        physxCooking->requestConvexCollisionRepresentation(request, onResult);
        // Validation errors are always reported synchronously, no need to call updateCooking
        REQUIRE(numResultCalled == 1);
    }

    SECTION("Unsupported Collision API")
    {
        UsdPhysicsCollisionAPI::Apply(usdMesh.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(usdMesh.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->boundingSphere);
        auto onResult = [&](PhysxCollisionRepresentationResult::Enum result,
                            const PhysxCollisionRepresentationConvexResult& data) {
            // Failure because bounding sphere is not a supported approximation
            REQUIRE(result == PhysxCollisionRepresentationResult::eRESULT_ERROR_UNSUPPORTED_APPROXIMATION);
            numResultCalled++;
        };
        physxCooking->requestConvexCollisionRepresentation(request, onResult);
        // Validation errors are always reported synchronously, no need to call updateCooking
        REQUIRE(numResultCalled == 1);
    }

    SECTION("Convex Hull")
    {
        UsdPhysicsCollisionAPI::Apply(usdMesh.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(usdMesh.GetPrim());
        pxr::TfToken cookingToken = UsdPhysicsTokens->convexHull;
        meshAPI.CreateApproximationAttr().Set(cookingToken);
        auto onResult = [&](PhysxCollisionRepresentationResult::Enum result,
                            const PhysxCollisionRepresentationConvexResult& data) {
            REQUIRE(result == PhysxCollisionRepresentationResult::eRESULT_VALID);
            REQUIRE(data.convexes.size() == 1);
            REQUIRE(data.convexes[0].numPolygons == 6);
            REQUIRE(data.convexes[0].numVertices == 8);
            REQUIRE(data.convexes[0].indices != nullptr);
            REQUIRE(data.convexes[0].vertices != nullptr);
            numResultCalled++;
        };
        physxCooking->requestConvexCollisionRepresentation(request, onResult);
        REQUIRE(numResultCalled == 0);
        physxTests->updateCooking(); // Dispatch the task
        tasking->sleep_for(std::chrono::milliseconds(100)); // Give it time to convex hull
        physxTests->updateCooking(); // Retrieve result
        REQUIRE(numResultCalled == 1); // Ensure callback is called only once

        SECTION("MeshCache")
        {
            // As we've just created the mesh, this call now will retrieve data from the in-memory MeshCache
            numResultCalled = 0;
            physxCooking->requestConvexCollisionRepresentation(request, onResult);
            // As we're hitting MeshCache, the result is immediately reported without need to wait for cooking
            REQUIRE(numResultCalled == 1);
        }

        SECTION("Cancellation")
        {
            physxCooking->releaseLocalMeshCache(); // Make sure that Cooking cache is empty
            physxSim->detachStage(); // Make sure that MeshCache is empty
            stage->RemovePrim(meshPath); // Remove the prim  (otherwise attachStage will cook and place it in MeshCache)
            physxSim->attachStage(stageId);
            // Recreate the prim
            usdMesh = createMeshBox(stage, meshPath);
            UsdPhysicsCollisionAPI::Apply(usdMesh.GetPrim());
            UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(usdMesh.GetPrim());
            meshAPI.CreateApproximationAttr().Set(cookingToken);

            numResultCalled = 0;
            PhysxCollisionRepresentationTask task = physxCooking->requestConvexCollisionRepresentation(request, onResult);
            REQUIRE(numResultCalled == 0);
            physxTests->updateCooking(); // Dispatch the task
            physxCooking->cancelCollisionRepresentationTask(task, false); // Cancel it without invoking callback
            tasking->sleep_for(std::chrono::milliseconds(100)); // Give it time
            physxTests->updateCooking(); // Retrieve result
            REQUIRE(numResultCalled == 0); // Taks was never delivered, as it was cancelled
        }

        SECTION("Synchronous")
        {
            // Let's ask for a synchronous collision representation
            request.options.setFlag(PhysxCollisionRepresentationRequest::Options::kComputeAsynchronously, false);
            physxCooking->releaseLocalMeshCache(); // Make sure that Cooking cache is empty
            physxSim->detachStage(); // Make sure that MeshCache is empty
            stage->RemovePrim(meshPath); // Remove the prim  (otherwise attachStage will cook and place it in MeshCache)
            physxSim->attachStage(stageId);
            // Recreate the prim
            usdMesh = createMeshBox(stage, meshPath);
            UsdPhysicsCollisionAPI::Apply(usdMesh.GetPrim());
            UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(usdMesh.GetPrim());
            meshAPI.CreateApproximationAttr().Set(cookingToken);
            numResultCalled = 0;
            physxCooking->requestConvexCollisionRepresentation(request, onResult);
            // Result is available immediately without needing to call updateCooking()
            REQUIRE(numResultCalled == 1);
        }
    }

    SECTION("Convex Decomposition")
    {
        UsdPhysicsCollisionAPI::Apply(usdMesh.GetPrim());
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(usdMesh.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexDecomposition);
        auto onResult = [&](PhysxCollisionRepresentationResult::Enum result,
                            const PhysxCollisionRepresentationConvexResult& data) {
            REQUIRE(result == PhysxCollisionRepresentationResult::eRESULT_VALID);
            REQUIRE(data.convexes.size() >= 1);
            REQUIRE(data.convexes[0].numPolygons == 6);
            REQUIRE(data.convexes[0].numVertices == 8);
            REQUIRE(data.convexes[0].numIndices == 24);
            REQUIRE(data.convexes[0].indices != nullptr);
            REQUIRE(data.convexes[0].vertices != nullptr);
            numResultCalled++;
        };
        physxCooking->requestConvexCollisionRepresentation(request, onResult);
        REQUIRE(numResultCalled == 0);
        physxTests->updateCooking(); // Dispatch the task
#if CARB_DEBUG
        // In Debug this can take much longer than release, and guessing a good timeout here is not really important
        while(numResultCalled == 0)
        {
            tasking->sleep_for(std::chrono::milliseconds(500)); // Give it time to convex decomp
            physxTests->updateCooking(); // Retrieve result
        }
#else
        tasking->sleep_for(std::chrono::milliseconds(500)); // Give it time to convex decomp
#endif
        physxTests->updateCooking(); // Retrieve result
        REQUIRE(numResultCalled == 1); // Ensure callback is called only once
    }
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
