// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <usdrt/scenegraph/usd/usd/stage.h>
#include <usdrt/population/IUtils.h>

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Transformation changes tests
TEST_CASE_TEMPLATE("Transformation Changes Tests", T, USDChange, FabricChange)
{
    T changeTemplate;

    FabricChange fabricChangeTemplate;

    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

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

    const SdfPath xformPath = defaultPrimPath.AppendChild(TfToken("xform"));
    UsdGeomXform xform = UsdGeomXform::Define(stage, xformPath);
    xform.AddTranslateOp(UsdGeomXformOp::PrecisionFloat);
    xform.AddOrientOp();

    // create rigid body with a box collider
    addRigidBox(stage, "/World/xform/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath = SdfPath("/World/xform/box");
    UsdPrim boxPrim = stage->GetPrimAtPath(boxPath);

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());
    // Enable population utils to emulate FSD behavior
    fabricChangeTemplate.init(stageId, physicsTests.getApp()->getFramework());
    auto populationUtils = omni::core::createType<usdrt::population::IUtils>();
    const bool usdNoticeHandling = populationUtils->getEnableUsdNoticeHandling(fabricChangeTemplate.mStageId);
    populationUtils->setEnableUsdNoticeHandling(
        fabricChangeTemplate.mStageId,
        fabricChangeTemplate.iStageReaderWriter->getFabricId(fabricChangeTemplate.mSrwId), true);

    // Fill the stage in progress with USD values
    {
        populationUtils->populateFromUsd(
            fabricChangeTemplate.mSrwId, stageId,
            omni::fabric::convertToPathType<omni::fabric::Path>(
                fabricChangeTemplate.iStageReaderWriter->getFabricId(fabricChangeTemplate.mSrwId),
                PXR_NS::SdfPath::AbsoluteRootPath()),
            nullptr, 0.0);
    }

    SUBCASE("Direct Transformation Change")
    {        
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

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
        physxSim->flushChanges();        

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        REQUIRE(basePtr == newBasePtr);
        tr = actor->getGlobalPose();
        compare(tr.p, toPhysX(newPos), epsilon);
    }

    SUBCASE("Parent Transformation Change")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

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
        changeTemplate.setTransformation(xformPath, newPos, GfQuatf(1.0f));
        physxSim->flushChanges();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        REQUIRE(basePtr == newBasePtr);
        tr = actor->getGlobalPose();
        compare(tr.p, toPhysX(newPos), epsilon);
    }

    SUBCASE("Parent Transformation XformOpReset Change")
    {
        // add xformOpReset on the rigid body
        UsdGeomXform xformBox(boxPrim);
        xformBox.SetResetXformStack(true);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

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

        // set new pos of the parent, it should not effect the rigid body
        GfVec3f newPos(10.0f);
        changeTemplate.setTransformation(xformPath, newPos, GfQuatf(1.0f));
        physxSim->flushChanges();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        REQUIRE(basePtr == newBasePtr);
        tr = actor->getGlobalPose();
        compare(tr.p, PxVec3(0.0f, 0.0f, 0.0f), epsilon);

        // run simulation and make sure velocity is not reset if we change the transformation
        for (size_t i = 0; i < 10; i++)
        {
            physxSim->simulate(1.0f / 60.0f, 0.0f);
            physxSim->fetchResults();
        }

        // set new pos of the parent, it should not effect the rigid body
        newPos = GfVec3f(20.0f);
        changeTemplate.setTransformation(xformPath, newPos, GfQuatf(1.0f));
        physxSim->flushChanges();

        const PxVec3 velocity = actor->getLinearVelocity();
        CHECK(velocity.z < -1.0f);
    }

    populationUtils->setEnableUsdNoticeHandling(
    fabricChangeTemplate.mStageId,
    fabricChangeTemplate.iStageReaderWriter->getFabricId(fabricChangeTemplate.mSrwId), usdNoticeHandling);

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
