// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/Tools.h"

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>

#include "PhysicsTools.h"

using namespace pxr;
using namespace omni::physx;

struct TransformData
{
    pxr::GfVec3f    pos;
    pxr::GfQuatf    rot;
    uint64_t        path;
};

void transformNotificationFn(uint64_t sdfPath, const carb::Float3& pos, const carb::Float4& rot, void* userData)
{
    TransformData* data = (TransformData*) userData;

    data->pos = (const pxr::GfVec3f&)(pos);
    data->rot = (const pxr::GfQuatf&)(rot);
    data->path = sdfPath;
}

struct VelocityData
{
    pxr::GfVec3f    linVelocity;
    pxr::GfVec3f    angVelocity;
    uint64_t        path;
};

void velocityNotificationFn(uint64_t sdfPath, const carb::Float3& linVelocity, const carb::Float3& angVelocity, void* userData)
{
    VelocityData* data = (VelocityData*)userData;

    data->linVelocity = (const pxr::GfVec3f&)(linVelocity);
    data->angVelocity = (const pxr::GfVec3f&)(angVelocity);
    data->path = sdfPath;
}

//-----------------------------------------------------------------------------
// Init test
TEST_CASE("IPhysxSimulation interface test",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    const float toleranceEpsilon = 1e-4f;

    SECTION("Module init test")
    {
        omni::physx::IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
        CHECK(physxSim);
    }

    SECTION("Simulation")
    {
        omni::physx::IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
        CHECK(physxSim);

        UsdStageRefPtr stage = UsdStage::CreateInMemory();
        UsdPrim defaultPrim = stage->DefinePrim(SdfPath("/root"));

        pxr::UsdUtilsStageCache::Get().Insert(stage);
        long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

        UsdPhysicsScene::Define(stage, SdfPath("/root/physicsScene"));

        addRigidBox(stage, "/root/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
            GfVec3f(0.7f), 0.001f);

        const SdfPath boxPath = SdfPath("/root/box");

        physxSim->attachStage(stageId);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f pos = getPhysicsPrimPos(stage, boxPath);
        CHECK_LT(pos[1], -100.0f);        

        pxr::UsdUtilsStageCache::Get().Erase(stage);
        stage = nullptr;
    }

}

TEST_CASE("IPhysxSimulation transformation data",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    const float toleranceEpsilon = 1e-4f;
    
    omni::physx::IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    CHECK(physxSim);

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdPrim defaultPrim = stage->DefinePrim(SdfPath("/root"));

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    UsdPhysicsScene::Define(stage, SdfPath("/root/physicsScene"));

    addRigidBox(stage, "/root/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath = SdfPath("/root/box");
    const uint64_t boxPathInt = asInt(boxPath);

    setPhysicsPrimPos(stage, boxPath, GfVec3f(0.0f));

    // regular simulation
    SECTION("Regular Simulation")
    {
        physxSim->attachStage(stageId);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f pos = getPhysicsPrimPos(stage, boxPath);
        CHECK_LT(pos[1], -100.0f);
    }

   
    // disabled sim write globally
    SECTION("Disabled Write Globally")
    {
        physxSim->attachStage(stageId);

        physxSim->setSimulationOutputFlags(SimulationOutputType::eTRANSFORMATION, SimulationOutputFlag::eSKIP_WRITE, nullptr, 0);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f pos = getPhysicsPrimPos(stage, boxPath);
        CHECK_LT(fabsf(pos[1]), toleranceEpsilon);

        physxSim->removeSimulationOutputFlags(SimulationOutputType::eTRANSFORMATION, SimulationOutputFlag::eSKIP_WRITE, nullptr, 0);
    }

    // disabled sim write per SdfPath
    SECTION("Disabled Write Per SdfPath")
    {
        physxSim->attachStage(stageId);


        physxSim->setSimulationOutputFlags(SimulationOutputType::eTRANSFORMATION, SimulationOutputFlag::eSKIP_WRITE, &boxPathInt, 1);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f pos = getPhysicsPrimPos(stage, boxPath);
        CHECK_LT(fabsf(pos[1]), toleranceEpsilon);
    }

    // enabled sim notification globally
    SECTION("Enabled Notification Globally")
    {
        physxSim->attachStage(stageId);

        TransformData data;

        ISimulationCallback cb;
        cb.transformationWriteFn = transformNotificationFn;
        cb.userData = &data;

        physxSim->setSimulationCallback(cb);
        physxSim->setSimulationOutputFlags(SimulationOutputType::eTRANSFORMATION, SimulationOutputFlag::eNOTIFY_UPDATE, nullptr, 0);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f pos = getPhysicsPrimPos(stage, boxPath);
        CHECK_LT(fabsf(pos[0] - data.pos[0]), toleranceEpsilon);
        CHECK_LT(fabsf(pos[1] - data.pos[1]), toleranceEpsilon);
        CHECK_LT(fabsf(pos[2] - data.pos[2]), toleranceEpsilon);
        CHECK_EQ(data.path, boxPathInt);
    }

    // enabled sim notification per path
    SECTION("Enabled Notification Per Path")
    {
        physxSim->attachStage(stageId);

        TransformData data;

        ISimulationCallback cb;
        cb.transformationWriteFn = transformNotificationFn;
        cb.userData = &data;

        physxSim->setSimulationCallback(cb);
        physxSim->setSimulationOutputFlags(SimulationOutputType::eTRANSFORMATION, SimulationOutputFlag::eNOTIFY_UPDATE, &boxPathInt, 1);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f pos = getPhysicsPrimPos(stage, boxPath);
        CHECK_LT(fabsf(pos[0] - data.pos[0]), toleranceEpsilon);
        CHECK_LT(fabsf(pos[1] - data.pos[1]), toleranceEpsilon);
        CHECK_LT(fabsf(pos[2] - data.pos[2]), toleranceEpsilon);
        CHECK_EQ(data.path, boxPathInt);
    }

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
    
}

TEST_CASE("IPhysxSimulation velocity data",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    const float toleranceEpsilon = 1e-4f;

    omni::physx::IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    CHECK(physxSim);

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdPrim defaultPrim = stage->DefinePrim(SdfPath("/root"));

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    UsdPhysicsScene::Define(stage, SdfPath("/root/physicsScene"));

    addRigidBox(stage, "/root/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath = SdfPath("/root/box");
    const uint64_t boxPathInt = asInt(boxPath);

    UsdPhysicsRigidBodyAPI rbAPI = UsdPhysicsRigidBodyAPI::Get(stage, boxPath);
    REQUIRE(rbAPI);
    const GfVec3f initialAngularVel(0.0f, 0.0f, 180.0f);
    const GfVec3f initialLinearVel(0.0f, 10.0f, 0.0f);
    rbAPI.GetAngularVelocityAttr().Set(initialAngularVel);
    rbAPI.GetVelocityAttr().Set(initialLinearVel);

    // regular simulation
    SECTION("Regular simulation")
    {
        physxSim->attachStage(stageId);

        for (int u = 0; u < 10; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f linVelocity;
        rbAPI.GetVelocityAttr().Get(&linVelocity);
        CHECK_LT(linVelocity[1], -150.0f);

        GfVec3f angVelocity;
        rbAPI.GetAngularVelocityAttr().Get(&angVelocity);
        CHECK_LT(fabsf(angVelocity[0]), toleranceEpsilon);
        CHECK_LT(fabsf(angVelocity[1]), toleranceEpsilon);
        CHECK_GT(angVelocity[2], 150.0f);
    }

    // disabled sim write globally
    SECTION("Disabled sim write globally")
    {
        physxSim->attachStage(stageId);

        physxSim->setSimulationOutputFlags(SimulationOutputType::eVELOCITY, SimulationOutputFlag::eSKIP_WRITE, nullptr, 0);

        for (int u = 0; u < 10; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f linVelocity;
        rbAPI.GetVelocityAttr().Get(&linVelocity);
        compare(linVelocity, initialLinearVel, toleranceEpsilon);

        GfVec3f angVelocity;
        rbAPI.GetAngularVelocityAttr().Get(&angVelocity);
        compare(angVelocity, initialAngularVel, toleranceEpsilon);

        physxSim->removeSimulationOutputFlags(SimulationOutputType::eVELOCITY, SimulationOutputFlag::eSKIP_WRITE, nullptr, 0);
    }

    // disabled sim write per SdfPath
    SECTION("Disabled sim write per SdfPath")
    {
        physxSim->attachStage(stageId);

        physxSim->setSimulationOutputFlags(SimulationOutputType::eVELOCITY, SimulationOutputFlag::eSKIP_WRITE, &boxPathInt, 1);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f linVelocity;
        rbAPI.GetVelocityAttr().Get(&linVelocity);
        compare(linVelocity, initialLinearVel, toleranceEpsilon);

        GfVec3f angVelocity;
        rbAPI.GetAngularVelocityAttr().Get(&angVelocity);
        compare(angVelocity, initialAngularVel, toleranceEpsilon);
    }

    // enabled sim notification globally
    SECTION("Enabled notification globally")
    {
        physxSim->attachStage(stageId);

        VelocityData data;

        ISimulationCallback cb;
        cb.velocityWriteFn = velocityNotificationFn;
        cb.userData = &data;

        physxSim->setSimulationCallback(cb);
        physxSim->setSimulationOutputFlags(SimulationOutputType::eVELOCITY, SimulationOutputFlag::eNOTIFY_UPDATE, nullptr, 0);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f linVelocity;
        rbAPI.GetVelocityAttr().Get(&linVelocity);
        compare(linVelocity, data.linVelocity, toleranceEpsilon);

        GfVec3f angVelocity;
        rbAPI.GetAngularVelocityAttr().Get(&angVelocity);
        compare(angVelocity, data.angVelocity, toleranceEpsilon);

        CHECK_EQ(data.path, boxPathInt);
    }

    // enabled sim notification per path
    SECTION("Enabled notification per path")
    {
        physxSim->attachStage(stageId);

        VelocityData data;

        ISimulationCallback cb;
        cb.velocityWriteFn = velocityNotificationFn;
        cb.userData = &data;

        physxSim->setSimulationCallback(cb);
        physxSim->setSimulationOutputFlags(SimulationOutputType::eVELOCITY, SimulationOutputFlag::eNOTIFY_UPDATE, &boxPathInt, 1);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f linVelocity;
        rbAPI.GetVelocityAttr().Get(&linVelocity);
        compare(linVelocity, data.linVelocity, toleranceEpsilon);

        GfVec3f angVelocity;
        rbAPI.GetAngularVelocityAttr().Get(&angVelocity);
        compare(angVelocity, data.angVelocity, toleranceEpsilon);

        CHECK_EQ(data.path, boxPathInt);
    }

    // enabled sim notification per path, radians
    SECTION("Enabled notification per path, data in radians")
    {
        physxSim->attachStage(stageId);

        VelocityData data;

        ISimulationCallback cb;
        cb.velocityWriteFn = velocityNotificationFn;
        cb.userData = &data;

        physxSim->setSimulationCallback(cb);
        physxSim->setSimulationOutputFlags(SimulationOutputType::eVELOCITY, SimulationOutputFlag::eNOTIFY_UPDATE | SimulationOutputFlag::eNOTIFY_IN_RADIANS, &boxPathInt, 1);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        physxSim->detachStage();

        GfVec3f linVelocity;
        rbAPI.GetVelocityAttr().Get(&linVelocity);
        compare(linVelocity, data.linVelocity, toleranceEpsilon);

        GfVec3f angVelocity;
        rbAPI.GetAngularVelocityAttr().Get(&angVelocity);
        compare(angVelocity, radToDeg(data.angVelocity), toleranceEpsilon);

        CHECK_EQ(data.path, boxPathInt);
    }

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;

}
