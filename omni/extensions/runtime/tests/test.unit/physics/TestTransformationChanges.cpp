// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/PhysicsChangeTemplate.h"

#include "PhysicsTools.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>

#include <common/foundation/TypeCast.h>

#include <usdrt/hierarchy/IFabricHierarchy.h>

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Transformation changes tests
TEST_CASE_TEMPLATE("Transformation Changes Tests", T, USDChange, FabricChange)
{
    T changeTemplate;

    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    ScopedPopulationActivation scopedPopulation;

    // Require the hierarchy to be available, otherwise the xforms won't update correctly for the FabricChange.
    auto iHierarchyMaker = omni::core::createType<usdrt::hierarchy::IFabricHierarchy>();
    REQUIRE(iHierarchyMaker);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body with a box collider
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath = SdfPath("/World/box");
    UsdPrim boxPrim = stage->GetPrimAtPath(boxPath);

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // Set change specific parameters on the prim such as the matrix attributes for fabric.
    changeTemplate.initPrim(boxPath);

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Transformation Change")
    {        
        GfVec3f pos = getPhysicsPrimPos(stage, boxPath);
        // check initial pos
        compare(pos, GfVec3f(0.0f), epsilon);

        // get the dynamic actor
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());

        PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
        PxTransform tr = actor->getGlobalPose();
        compare(tr.p, PxVec3(0.0f, 0.0f, 0.0f), epsilon);

        // set new pos
        GfVec3f newPos(10.0f);
        changeTemplate.setTransformation(boxPath, newPos, GfQuatf(1.0f));
        changeTemplate.broadcastChanges();
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        tr = actor->getGlobalPose();
        compare(tr.p, toPhysX(newPos), epsilon);
    }

   
    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

