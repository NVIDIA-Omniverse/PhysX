// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../../common/PhysicsChangeTemplate.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>


class ObjectChangeNotifier
{
public:
    struct Entry
    {
        Entry(pxr::SdfPath inPath, omni::physx::usdparser::ObjectId inObjectId, omni::physx::PhysXType inType)
            : path(inPath)
            , objectId(inObjectId)
            , type(inType)
        {
        }

        pxr::SdfPath path;
        omni::physx::usdparser::ObjectId objectId;
        omni::physx::PhysXType type;
    };

    ObjectChangeNotifier()
    {
        clear();
    }

    static void onCreation(const pxr::SdfPath& sdfPath, omni::physx::usdparser::ObjectId objectId,
        omni::physx::PhysXType type, void* userData)
    {
        ObjectChangeNotifier& ocn = *reinterpret_cast<ObjectChangeNotifier*>(userData);
        ocn.mCreationEntries.push_back(Entry(sdfPath, objectId, type));
    }

    static void onDestruction(const pxr::SdfPath& sdfPath, omni::physx::usdparser::ObjectId objectId,
        omni::physx::PhysXType type, void* userData)
    {
        ObjectChangeNotifier& ocn = *reinterpret_cast<ObjectChangeNotifier*>(userData);
        ocn.mDestructionEntries.push_back(Entry(sdfPath, objectId, type));
    }

    static void onTotalDestruction(void* userData)
    {
        ObjectChangeNotifier& ocn = *reinterpret_cast<ObjectChangeNotifier*>(userData);
        ocn.mTotalDestructionCount++;
    }

    void clear()
    {
        mCreationEntries.clear();
        mDestructionEntries.clear();
        mTotalDestructionCount = 0;
    }


    std::vector<Entry> mCreationEntries;
    std::vector<Entry> mDestructionEntries;
    uint32_t mTotalDestructionCount;
};


TEST_CASE("Object Change Notifications Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=msauter][priority=mandatory]")
{
    const float timeStep = 1.0f / 60.0f;

    // setup basic stage
    pxr::UsdStageRefPtr stage = pxr::UsdStage::CreateInMemory();
    const pxr::SdfPath defaultPrimPath("/World");
    pxr::UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::physx::IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    omni::physx::IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    ObjectChangeNotifier objChangeNotifier;
    omni::physx::IPhysicsObjectChangeCallback callback;
    callback.objectCreationNotifyFn = ObjectChangeNotifier::onCreation;
    callback.objectDestructionNotifyFn = ObjectChangeNotifier::onDestruction;
    callback.allObjectsDestructionNotifyFn = ObjectChangeNotifier::onTotalDestruction;
    callback.userData = &objChangeNotifier;
    omni::physx::SubscriptionId subscriptionId = physx->subscribeObjectChangeNotifications(callback);

    const pxr::SdfPath physicsScenePath = defaultPrimPath.AppendChild(pxr::TfToken("physicsScene"));
    pxr::UsdPhysicsScene scene = pxr::UsdPhysicsScene::Define(stage, physicsScenePath);

    const pxr::SdfPath box0Path = defaultPrimPath.AppendChild(pxr::TfToken("box0"));
    addRigidBox(stage, box0Path.GetText(), pxr::GfVec3f(100.f), pxr::GfVec3f(0.0f), pxr::GfQuatf(1.0f),
        pxr::GfVec3f(0.7f), 0.001f);

    physxSim->attachStage(stageId);

    //
    // there should be no notifications at initial stage traversal
    //

    REQUIRE_EQ(objChangeNotifier.mCreationEntries.size(), 0);
    REQUIRE_EQ(objChangeNotifier.mDestructionEntries.size(), 0);
    objChangeNotifier.clear();

    float currentTime = 0.0f;
    physxSim->simulate(timeStep, currentTime);
    physxSim->fetchResults();
    currentTime += timeStep;

    //
    // add box while simulation is running. Notification is expected
    //
    const pxr::SdfPath box1Path = defaultPrimPath.AppendChild(pxr::TfToken("box1"));
    addRigidBox(stage, box1Path.GetText(), pxr::GfVec3f(100.f), pxr::GfVec3f(10.0f), pxr::GfQuatf(1.0f),
        pxr::GfVec3f(0.7f), 0.001f);

    physxSim->simulate(timeStep, currentTime);
    physxSim->fetchResults();
    currentTime += timeStep;

    REQUIRE_EQ(objChangeNotifier.mCreationEntries.size(), 2);
    REQUIRE_EQ(objChangeNotifier.mDestructionEntries.size(), 0);
    bool foundActor = false;
    bool foundShape = false;
    for (const ObjectChangeNotifier::Entry& entry : objChangeNotifier.mCreationEntries)
    {
        if (entry.type == omni::physx::ePTActor)
        {
            REQUIRE(!foundActor);
            foundActor = true;

            REQUIRE_EQ(entry.path, box1Path);
            REQUIRE_EQ(entry.objectId, physx->getObjectId(box1Path, omni::physx::ePTActor));
        }
        else if (entry.type == omni::physx::ePTShape)
        {
            REQUIRE(!foundShape);
            foundShape = true;

            REQUIRE_EQ(entry.path, box1Path);
            REQUIRE_EQ(entry.objectId, physx->getObjectId(box1Path, omni::physx::ePTShape));
        }
    }
    REQUIRE(foundActor);
    REQUIRE(foundShape);
    objChangeNotifier.clear();

    //
    // removing rigid body API schema should send notifications as
    // the actor and shape will get receated
    //
    pxr::UsdPrim box1Prim = stage->GetPrimAtPath(box1Path);
    box1Prim.RemoveAPI<pxr::UsdPhysicsRigidBodyAPI>();

    physxSim->simulate(timeStep, currentTime);
    physxSim->fetchResults();
    currentTime += timeStep;

    REQUIRE_EQ(objChangeNotifier.mCreationEntries.size(), 2);
    REQUIRE_EQ(objChangeNotifier.mDestructionEntries.size(), 2);
    REQUIRE_EQ(objChangeNotifier.mTotalDestructionCount, 0);
    objChangeNotifier.clear();

    //
    // removing collider API schema should send notifications
    //
    box1Prim.RemoveAPI<pxr::UsdPhysicsCollisionAPI>();

    physxSim->simulate(timeStep, currentTime);
    physxSim->fetchResults();
    currentTime += timeStep;

    REQUIRE_EQ(objChangeNotifier.mCreationEntries.size(), 0);
    REQUIRE_EQ(objChangeNotifier.mDestructionEntries.size(), 2);
    REQUIRE_EQ(objChangeNotifier.mTotalDestructionCount, 0);
    objChangeNotifier.clear();

    //
    // re-adding rigid body API should send notifications
    //
    pxr::UsdPhysicsRigidBodyAPI::Apply(box1Prim);

    physxSim->simulate(timeStep, currentTime);
    physxSim->fetchResults();
    currentTime += timeStep;

    REQUIRE_EQ(objChangeNotifier.mCreationEntries.size(), 1);
    REQUIRE_EQ(objChangeNotifier.mDestructionEntries.size(), 0);
    REQUIRE_EQ(objChangeNotifier.mTotalDestructionCount, 0);
    objChangeNotifier.clear();

    //
    // removing rigid body prim should send notifications
    //
    stage->RemovePrim(box1Path);

    physxSim->simulate(timeStep, currentTime);
    physxSim->fetchResults();
    currentTime += timeStep;

    REQUIRE_EQ(objChangeNotifier.mCreationEntries.size(), 0);
    REQUIRE_EQ(objChangeNotifier.mDestructionEntries.size(), 1);
    REQUIRE_EQ(objChangeNotifier.mTotalDestructionCount, 0);
    objChangeNotifier.clear();

    //
    // add box while simulation is running but after unsubscribing from notifications.
    // No notification is expected.
    //
    physx->unsubscribeObjectChangeNotifications(subscriptionId);

    const pxr::SdfPath box2Path = defaultPrimPath.AppendChild(pxr::TfToken("box2"));
    addRigidBox(stage, box2Path.GetText(), pxr::GfVec3f(100.f), pxr::GfVec3f(20.0f), pxr::GfQuatf(1.0f),
        pxr::GfVec3f(0.7f), 0.001f);

    physxSim->simulate(timeStep, currentTime);
    physxSim->fetchResults();
    currentTime += timeStep;

    REQUIRE_EQ(objChangeNotifier.mCreationEntries.size(), 0);
    REQUIRE_EQ(objChangeNotifier.mDestructionEntries.size(), 0);
    REQUIRE_EQ(objChangeNotifier.mTotalDestructionCount, 0);
    objChangeNotifier.clear();

    subscriptionId = physx->subscribeObjectChangeNotifications(callback);

    physxSim->simulate(timeStep, currentTime);
    physxSim->fetchResults();
    currentTime += timeStep;

    //
    // releasing all objects should trigger the corresponding notification.
    //
    physx->releasePhysicsObjects();

    physxSim->simulate(timeStep, currentTime);
    physxSim->fetchResults();
    currentTime += timeStep;

    REQUIRE_EQ(objChangeNotifier.mCreationEntries.size(), 5);
    // because after releasePhysicsObjects, all objects get created again in the next step.
    // Apparently it is more of a debug feature and not really meant to be called during sim.

    REQUIRE_EQ(objChangeNotifier.mDestructionEntries.size(), 0);
    REQUIRE_EQ(objChangeNotifier.mTotalDestructionCount, 1);
    objChangeNotifier.clear();

    //
    // add another object before stopping the simulation
    //
    const pxr::SdfPath box3Path = defaultPrimPath.AppendChild(pxr::TfToken("box3"));
    addRigidBox(stage, box3Path.GetText(), pxr::GfVec3f(100.f), pxr::GfVec3f(30.0f), pxr::GfQuatf(1.0f),
        pxr::GfVec3f(0.7f), 0.001f);

    physxSim->simulate(timeStep, currentTime);
    physxSim->fetchResults();
    currentTime += timeStep;

    REQUIRE_EQ(objChangeNotifier.mCreationEntries.size(), 2);
    REQUIRE_EQ(objChangeNotifier.mDestructionEntries.size(), 0);
    REQUIRE_EQ(objChangeNotifier.mTotalDestructionCount, 0);
    objChangeNotifier.clear();

    physxSim->detachStage();

    //
    // detaching should not trigger any notification
    //
    REQUIRE_EQ(objChangeNotifier.mCreationEntries.size(), 0);
    REQUIRE_EQ(objChangeNotifier.mDestructionEntries.size(), 0);
    REQUIRE_EQ(objChangeNotifier.mTotalDestructionCount, 0);

    physx->unsubscribeObjectChangeNotifications(subscriptionId);

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
