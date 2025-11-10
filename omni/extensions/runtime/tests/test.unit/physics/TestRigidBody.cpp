// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/PhysicsChangeTemplate.h"

#include "PhysicsTools.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/PhysxTokens.h>

#include <common/foundation/TypeCast.h>

#include "pxr/usd/sdf/payload.h"
#include "pxr/usd/usd/payloads.h"

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Rigidbody tests
TEST_CASE_TEMPLATE("RigidBody Tests", T, USDChange, FabricChange)
{
    T changeTemplate;

    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    REQUIRE(pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit)));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath = SdfPath("/World/box");
    UsdPrim boxPrim = stage->GetPrimAtPath(boxPath);

    const TfToken forceToken(gWorldForceTokenString);
    UsdAttribute attr = boxPrim.CreateAttribute(forceToken, SdfValueTypeNames->Vector3f);
    REQUIRE(attr);
    attr.Set(GfVec3f(0.0f));

    // get rigid body API's thats going to be tested
    PhysxSchemaPhysxRigidBodyAPI physxRbApi = PhysxSchemaPhysxRigidBodyAPI::Apply(boxPrim);

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);    

    SUBCASE("RigidBody Enabled")
    {
        omni::physx::IPhysicsObjectChangeCallback callback;
        struct ObjectNotficationInfo
        {
            bool create;
            pxr::SdfPath path;
            usdparser::ObjectId objectId;
            PhysXType type;
        };
        callback.objectCreationNotifyFn = [](const pxr::SdfPath& sdfPath, usdparser::ObjectId objectId, PhysXType type, void* userData)
        {
            std::vector<ObjectNotficationInfo>* objectInfo = (std::vector<ObjectNotficationInfo>*)userData;
            objectInfo->push_back({ true, sdfPath, objectId, type });
        };
        callback.objectDestructionNotifyFn = [](const pxr::SdfPath& sdfPath, usdparser::ObjectId objectId, PhysXType type, void* userData)
        {
            std::vector<ObjectNotficationInfo>* objectInfo = (std::vector<ObjectNotficationInfo>*)userData;
            objectInfo->push_back({ false, sdfPath, objectId, type });
        };
        std::vector<ObjectNotficationInfo> objectInfo;
        callback.userData = &objectInfo;
        omni::physx::SubscriptionId subscriptionObjId = physx->subscribeObjectChangeNotifications(callback);

        const TfToken& changeToken = UsdPhysicsTokens->physicsRigidBodyEnabled;

        bool enabled = changeTemplate.template getAttributeValue<bool>(boxPath, changeToken);        
        // check default value
        CHECK(enabled == true);

        // if enabled dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());

        // disable rigid body API -> static body should be created
        // A.B. reparse changes dont work from fabric, as we read from USD the new parsed data
        changeTemplate.setAttributeValue(boxPath, changeToken, false);
        changeTemplate.broadcastChanges();
        UsdPhysicsRigidBodyAPI usdPhysicsRigidBodyAPI = UsdPhysicsRigidBodyAPI::Get(stage, boxPath);
        usdPhysicsRigidBodyAPI.GetRigidBodyEnabledAttr().Set(false);

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxRigidStatic>());

        REQUIRE(objectInfo.size() >= 2);
        CHECK(objectInfo[0].create == false);
        CHECK(objectInfo[1].create == true);
        for (size_t i = 0; i < 2; i++)
        {
            CHECK(objectInfo[i].path == boxPath);
            CHECK(objectInfo[i].objectId == (usdparser::ObjectId)basePtr->is<PxRigidStatic>()->userData);
            CHECK(objectInfo[i].type == ePTActor);
        }

        physx->unsubscribeObjectChangeNotifications(subscriptionObjId);
    }

    SUBCASE("RigidBody Kinematics")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsKinematicEnabled;

        bool enabled = changeTemplate.template getAttributeValue<bool>(boxPath, changeToken);        
        // check default value
        CHECK(enabled == false);

        // if enabled dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) == false);

        // enable kinematics 
        changeTemplate.setAttributeValue(boxPath, changeToken, true);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);        
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) == true);
    }

    SUBCASE("RigidBody Starts Asleep")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsStartsAsleep;

        bool enabled = changeTemplate.template getAttributeValue<bool>(boxPath, changeToken);
        // check default value
        CHECK(enabled == false);

        // if enabled dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(basePtr->is<PxRigidDynamic>()->isSleeping() == false);

        // Common post-test actions
        physxSim->detachStage();

        // enable starts asleep 
        changeTemplate.setAttributeValue(boxPath, changeToken, true);
        changeTemplate.broadcastChanges();
        UsdPhysicsRigidBodyAPI usdPhysicsRigidBodyAPI = UsdPhysicsRigidBodyAPI::Get(stage, boxPath);
        usdPhysicsRigidBodyAPI.GetStartsAsleepAttr().Set(true);

        physxSim->attachStage(stageId);
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();
        
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(basePtr->is<PxRigidDynamic>()->isSleeping() == true);
    }


    SUBCASE("RigidBody Linear velocity")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsVelocity;

        GfVec3f velocity = changeTemplate.template getAttributeValue<GfVec3f>(boxPath, changeToken);        
        // check default value
        compare(velocity, GfVec3f(0.0f), epsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        compare(basePtr->is<PxRigidDynamic>()->getLinearVelocity(), PxVec3(PxZero), epsilon);

        // enable velocity
        velocity = GfVec3f(50.0f, 20.0f, 10.0f);
        changeTemplate.setAttributeValue(boxPath, changeToken, velocity);
        changeTemplate.broadcastChanges();

        physxSim->flushChanges();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        compare(basePtr->is<PxRigidDynamic>()->getLinearVelocity(), omni::physx::toPhysX(velocity), epsilon);
    }

    SUBCASE("RigidBody Angular velocity")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsAngularVelocity;

        GfVec3f velocity = changeTemplate.template getAttributeValue<GfVec3f>(boxPath, changeToken);        
        // check default value
        compare(velocity, GfVec3f(0.0f), epsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        compare(basePtr->is<PxRigidDynamic>()->getAngularVelocity(), PxVec3(PxZero), epsilon);

        // enable velocity
        velocity = GfVec3f(50.0f, 20.0f, 10.0f);
        changeTemplate.setAttributeValue(boxPath, changeToken, velocity);
        changeTemplate.broadcastChanges();

        physxSim->flushChanges();        

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        velocity = degToRad(velocity);
        compare(basePtr->is<PxRigidDynamic>()->getAngularVelocity(), omni::physx::toPhysX(velocity), epsilon);
    }

    SUBCASE("RigidBody World Force")
    {
        const TfToken changeToken = forceToken;

        GfVec3f force = changeTemplate.template getAttributeValue<GfVec3f>(boxPath, changeToken);
        // check default value
        compare(force, GfVec3f(0.0f), epsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        PxVec3 initPos = basePtr->is<PxRigidDynamic>()->getGlobalPose().p;

        // set force
        force = GfVec3f(0.0f, 20000000.0f, 0.0f);
        changeTemplate.setAttributeValue(boxPath, changeToken, force);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.02f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK(basePtr->is<PxRigidDynamic>());
        PxVec3 newPos = basePtr->is<PxRigidDynamic>()->getGlobalPose().p;
        CHECK(newPos.y > initPos.y);
    }

    SUBCASE("RigidBody World Force - Disabled Simulation")
    {
        const TfToken changeToken = forceToken;

        GfVec3f force = changeTemplate.template getAttributeValue<GfVec3f>(boxPath, changeToken);
        // check default value
        compare(force, GfVec3f(0.0f), epsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        PxVec3 initPos = basePtr->is<PxRigidDynamic>()->getGlobalPose().p;
        basePtr->is<PxRigidDynamic>()->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, true);

        // set force
        force = GfVec3f(0.0f, 20000000.0f, 0.0f);
        changeTemplate.setAttributeValue(boxPath, changeToken, force);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.02f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK(basePtr->is<PxRigidDynamic>());
        PxVec3 newPos = basePtr->is<PxRigidDynamic>()->getGlobalPose().p;
        CHECK(fabsf(newPos.y - initPos.y) < epsilon);
    }

    SUBCASE("RigidBody Linear Damping")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyLinearDamping;

        float damping = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);
        // check default value
        CHECK(fabsf(damping) < epsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getLinearDamping() - damping) < epsilon);

        damping = 1.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, damping);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);        
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getLinearDamping() - damping) < epsilon);
    }

    SUBCASE("RigidBody Angular Damping")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyAngularDamping;

        float damping = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);        
        // check default value
        CHECK(fabsf(damping - 0.05f) < epsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getAngularDamping() - damping) < epsilon);

        damping = 1.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, damping);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getAngularDamping() - damping) < epsilon);
    }

    SUBCASE("RigidBody Max Linear Velocity")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyMaxLinearVelocity;

        float maxVel = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);        
        // check default value
        CHECK(maxVel > 1e10f);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(basePtr->is<PxRigidDynamic>()->getMaxLinearVelocity() > 1e10f);

        maxVel = 10.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, maxVel);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getMaxLinearVelocity() - maxVel) < epsilon);
    }

    SUBCASE("RigidBody Max Angular Velocity")
    {
        // we get quite some imprecision from the schema radDeg conversions, so we need bigger tolerance
        const float localEpsilon = epsilon * 100.0f;
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyMaxAngularVelocity;

        float maxVel = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);        
        // check default value
        CHECK(fabsf(maxVel - radToDeg(100.0f)) < localEpsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(fabsf(radToDeg(basePtr->is<PxRigidDynamic>()->getMaxAngularVelocity()) - maxVel) < localEpsilon);

        maxVel = 10.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, maxVel);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(radToDeg(basePtr->is<PxRigidDynamic>()->getMaxAngularVelocity()) - maxVel) < localEpsilon);
    }

    SUBCASE("RigidBody Sleep Threshold")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodySleepThreshold;

        float value = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);        
        // check default value
        CHECK(fabsf(value - 0.00005f) < epsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        // scaled by tolerances speed * tolerances speed
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getSleepThreshold() - value * 1000.0f * 1000.0f) < epsilon);

        value = 10.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getSleepThreshold() - value * 1000.0f * 1000.0f) < epsilon);
    }

    SUBCASE("RigidBody Stabilization Threshold")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyStabilizationThreshold;

        float value = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);        
        // check default value
        CHECK(fabsf(value - 0.00001f) < epsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        // scaled by tolerances speed * tolerances speed
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getStabilizationThreshold() - value * 1000.0f * 1000.0f) < epsilon);

        value = 10.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getStabilizationThreshold() - value * 1000.0f * 1000.0f) < epsilon);
    }

    SUBCASE("RigidBody Max Depenetration Velocity")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyMaxDepenetrationVelocity;

        float value = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);        
        // check default value
        CHECK(fabsf(value - 3.0f) < epsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getMaxDepenetrationVelocity() * metersPerStageUnit - value) < epsilon);

        value = 10.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getMaxDepenetrationVelocity() - value) < epsilon);
    }

    SUBCASE("RigidBody Contact Slop Offset")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyContactSlopCoefficient;

        float value = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);
        // check default value
        CHECK(fabsf(value - 0.0f) < epsilon);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getContactSlopCoefficient() - value) < epsilon);

        // redo with nonzero parsing value:
        physxSim->detachStage();
        value = 5.0;
        physxRbApi.CreateContactSlopCoefficientAttr().Set(value);
        physxSim->attachStage(stageId);

        // dynamic rigid body should be created
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getContactSlopCoefficient() - value) < epsilon);

        value = 10.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getContactSlopCoefficient() - value) < epsilon);
    }

    SUBCASE("RigidBody Max Contact Impulse")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyMaxContactImpulse;

        float value = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);
        // check default value
        CHECK(isinf(value));

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(basePtr->is<PxRigidDynamic>()->getMaxContactImpulse() > 1e20f);

        value = 10.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(basePtr->is<PxRigidDynamic>()->getMaxContactImpulse() - value) < epsilon);
    }

    SUBCASE("RigidBody Solver Position Iteration Count")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodySolverPositionIterationCount;

        int value = changeTemplate.template getAttributeValue<int>(boxPath, changeToken);
        // check default value
        CHECK(value == 16);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        PxU32 numPosCounts = 0;
        PxU32 numVelCounts = 0;
        basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);
        CHECK(numPosCounts == value);

        value = 10;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        numPosCounts = 0;
        numVelCounts = 0;
        basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);
        CHECK(numPosCounts == value);
    }

    SUBCASE("RigidBody Solver Position Iteration Count Clamp")
    {
        physxSim->detachStage();
        PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());
        physxSceneAPI.CreateMinPositionIterationCountAttr().Set(32u);
        physxSim->attachStage(stageId);

        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodySolverPositionIterationCount;

        {
            // dynamic rigid body should be created
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxU32 numPosCounts = 0;
            PxU32 numVelCounts = 0;
            basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);
            CHECK(numPosCounts == 32);

            int value = 10;
            changeTemplate.setAttributeValue(boxPath, changeToken, value);
            changeTemplate.broadcastChanges();

            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();

            // same ptr should be returned - no reparsing
            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
            CHECK(basePtr == newBasePtr);
            numPosCounts = 0;
            numVelCounts = 0;
            basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);
            CHECK(numPosCounts == 32);
        }

        physxSim->detachStage();
        physxSceneAPI.CreateMinPositionIterationCountAttr().Set(2u);
        physxSceneAPI.CreateMaxPositionIterationCountAttr().Set(8u);
        physxSim->attachStage(stageId);

        {
            // dynamic rigid body should be created
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxU32 numPosCounts = 0;
            PxU32 numVelCounts = 0;

            int value = 10;
            changeTemplate.setAttributeValue(boxPath, changeToken, value);
            changeTemplate.broadcastChanges();

            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();

            // same ptr should be returned - no reparsing
            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
            CHECK(basePtr == newBasePtr);
            numPosCounts = 0;
            numVelCounts = 0;
            basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);
            CHECK(numPosCounts == 8);
        }

    }

    SUBCASE("RigidBody Solver Velocity Iteration Count")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodySolverVelocityIterationCount;

        int value = changeTemplate.template getAttributeValue<int>(boxPath, changeToken);        
        // check default value
        CHECK(value == 1);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        PxU32 numPosCounts = 0;
        PxU32 numVelCounts = 0;
        basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);
        CHECK(numVelCounts == value);

        value = 10;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        numPosCounts = 0;
        numVelCounts = 0;
        basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);
        CHECK(numVelCounts == value);
    }

    SUBCASE("RigidBody Solver Velocity Iteration Count Clamp")
    {
        physxSim->detachStage();
        PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());
        physxSceneAPI.CreateMinVelocityIterationCountAttr().Set(32u);
        physxSim->attachStage(stageId);

        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodySolverVelocityIterationCount;

        {
            // dynamic rigid body should be created
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxU32 numPosCounts = 0;
            PxU32 numVelCounts = 0;
            basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);
            CHECK(numVelCounts == 32);

            int value = 10;
            changeTemplate.setAttributeValue(boxPath, changeToken, value);
            changeTemplate.broadcastChanges();

            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();

            // same ptr should be returned - no reparsing
            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
            CHECK(basePtr == newBasePtr);
            numPosCounts = 0;
            numVelCounts = 0;
            basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);
            CHECK(numVelCounts == 32);
        }

        physxSim->detachStage();
        physxSceneAPI.CreateMinVelocityIterationCountAttr().Set(2u);
        physxSceneAPI.CreateMaxVelocityIterationCountAttr().Set(8u);
        physxSim->attachStage(stageId);

        {
            // dynamic rigid body should be created
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxU32 numPosCounts = 0;
            PxU32 numVelCounts = 0;

            int value = 10;
            changeTemplate.setAttributeValue(boxPath, changeToken, value);
            changeTemplate.broadcastChanges();

            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();

            // same ptr should be returned - no reparsing
            PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
            CHECK(basePtr == newBasePtr);
            numPosCounts = 0;
            numVelCounts = 0;
            basePtr->is<PxRigidDynamic>()->getSolverIterationCounts(numPosCounts, numVelCounts);
            CHECK(numVelCounts == 8);
        }
    }

    SUBCASE("RigidBody Enable CCD")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyEnableCCD;

        bool value = changeTemplate.template getAttributeValue<bool>(boxPath, changeToken);        
        // check default value
        CHECK(value == false);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());        
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eENABLE_CCD) == value);

        value = true;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eENABLE_CCD) == value);
    }

    SUBCASE("RigidBody Enable Speculative CCD")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyEnableSpeculativeCCD;

        bool value = changeTemplate.template getAttributeValue<bool>(boxPath, changeToken);        
        // check default value
        CHECK(value == false);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD) == value);

        value = true;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD) == value);
    }

    SUBCASE("RigidBody Retain Accelerations")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyRetainAccelerations;

        bool value = changeTemplate.template getAttributeValue<bool>(boxPath, changeToken);
        // check default value
        CHECK(value == false);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eRETAIN_ACCELERATIONS) == value);

        value = true;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eRETAIN_ACCELERATIONS) == value);
    }

    SUBCASE("RigidBody Enable Gyroscopic Forces")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyEnableGyroscopicForces;

        bool value = changeTemplate.template getAttributeValue<bool>(boxPath, changeToken);
        // check default value
        CHECK(value == true);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES) == value);

        value = false;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES) == value);
    }

    SUBCASE("RigidBody Disable Gravity")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyDisableGravity;

        bool value = changeTemplate.template getAttributeValue<bool>(boxPath, changeToken);        
        // check default value
        CHECK(value == false);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK((basePtr->is<PxRigidDynamic>()->getActorFlags() & PxActorFlag::eDISABLE_GRAVITY) == value);

        value = true;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK((basePtr->is<PxRigidDynamic>()->getActorFlags() & PxActorFlag::eDISABLE_GRAVITY) == value);
    }

    SUBCASE("RigidBody Lock Pos Axis")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyLockedPosAxis;

        int value = changeTemplate.template getAttributeValue<int>(boxPath, changeToken);        
        // check default value
        CHECK(value == 0);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(basePtr->is<PxRigidDynamic>()->getRigidDynamicLockFlags() == PxRigidDynamicLockFlags(0));

        value = 7;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_LINEAR_X) == true);
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y) == true);
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z) == true);
    }

    SUBCASE("RigidBody Lock Rot Axis")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxRigidBodyLockedRotAxis;

        int value = changeTemplate.template getAttributeValue<int>(boxPath, changeToken);        
        // check default value
        CHECK(value == 0);

        // dynamic rigid body should be created
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK(basePtr->is<PxRigidDynamic>()->getRigidDynamicLockFlags() == PxRigidDynamicLockFlags(0));

        value = 7;
        changeTemplate.setAttributeValue(boxPath, changeToken, value);
        changeTemplate.broadcastChanges();

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(basePtr == newBasePtr);
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X) == true);
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y) == true);
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z) == true);
    }

    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("RigidBody PointInstancer",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    const float toleranceEpsilon = 1e-4f;

    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxUnitTests* physxUT = physicsTests.acquirePhysxUnitTestInterface();
    REQUIRE(physxUT);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxPrivate* physxPriv = physicsTests.acquirePhysxPrivateInterface();
    REQUIRE(physxPriv);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    REQUIRE(pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit)));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    const SdfPath geomPointInstancerPath = defaultPrimPath.AppendChild(TfToken("pointInstancer"));
    const SdfPath boxActorPath = geomPointInstancerPath.AppendChild(TfToken("boxActor"));    

    UsdGeomCube cubeGeom = UsdGeomCube::Define(stage, boxActorPath);

    UsdPhysicsCollisionAPI::Apply(cubeGeom.GetPrim());
    UsdPhysicsRigidBodyAPI::Apply(cubeGeom.GetPrim());

    const uint32_t numIndices = 2;
    VtArray<int> meshIndices = { 0, 0 };    
    VtArray<GfVec3f> positions = { GfVec3f(-125.0, 0.0, 500.0), GfVec3f(-125.0, 0.0, 500.0) };
    VtArray<GfQuath> orientations = { GfQuath(1.0), GfQuath(1.0) };
    VtArray<GfVec3f> linearVelocities = { GfVec3f(0.0), GfVec3f(0.0, 0.0, 0.0) };
    VtArray<GfVec3f> angularVelocities = { GfVec3f(0.0, 10.0, 0.0), GfVec3f(0.0) };
   
    UsdGeomPointInstancer shapeList = UsdGeomPointInstancer::Define(stage, geomPointInstancerPath);
    shapeList.GetPrototypesRel().AddTarget(boxActorPath);

    shapeList.GetProtoIndicesAttr().Set(meshIndices);
    shapeList.GetPositionsAttr().Set(positions);
    shapeList.GetOrientationsAttr().Set(orientations);
    shapeList.GetVelocitiesAttr().Set(linearVelocities);
    shapeList.GetAngularVelocitiesAttr().Set(angularVelocities);

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Positions")
    {
        positions.clear();
        shapeList.GetPositionsAttr().Get(&positions);
        // check default value
        CHECK(positions.size() == size_t(numIndices));

        // dynamic rigid body should be created
        std::vector<void*> ptrs;
        ptrs.resize(size_t(numIndices));
        uint32_t numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTActor);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            PxTransform tr = actor->getGlobalPose();
            compare(tr.p, omni::physx::toPhysX(positions[i]), toleranceEpsilon);
        }

        // update positions
        for (uint32_t i = 0; i < numIndices; i++)
        {
            positions[i][0] = i * 500.0f;
        }
        shapeList.GetPositionsAttr().Set(positions);

        physxSim->flushChanges();        

        numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTActor);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            PxTransform tr = actor->getGlobalPose();
            compare(tr.p, omni::physx::toPhysX(positions[i]), toleranceEpsilon);
        }
    }

    SUBCASE("Orientations")
    {
        const float quatEpsilon = 0.01f;

        orientations.clear();
        shapeList.GetOrientationsAttr().Get(&orientations);
        // check default value
        CHECK(orientations.size() == size_t(numIndices));

        // dynamic rigid body should be created
        std::vector<void*> ptrs;
        ptrs.resize(size_t(numIndices));
        uint32_t numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTActor);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            PxTransform tr = actor->getGlobalPose();
            compare(tr.q, toPhysX(orientations[i]), quatEpsilon);
        }

        // update orientations
        for (uint32_t i = 0; i < numIndices; i++)
        {
            orientations[i] = GfQuath(0.8660254f * i, 0.0f, 0.0f, 0.5f).GetNormalized();
        }
        shapeList.GetOrientationsAttr().Set(orientations);

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTActor);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            PxTransform tr = actor->getGlobalPose();
            compare(tr.q, toPhysX(orientations[i]), quatEpsilon);
        }
    }

    SUBCASE("Linear Velocities")
    {
        linearVelocities.clear();
        shapeList.GetVelocitiesAttr().Get(&linearVelocities);
        // check default value
        CHECK(linearVelocities.size() == size_t(numIndices));

        // dynamic rigid body should be created
        std::vector<void*> ptrs;
        ptrs.resize(size_t(numIndices));
        uint32_t numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTActor);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();            
            compare(actor->getLinearVelocity(), omni::physx::toPhysX(linearVelocities[i]), toleranceEpsilon);
        }

        // update velocities
        for (uint32_t i = 0; i < numIndices; i++)
        {
            linearVelocities[i][0] = i * 500.0f + 200.0f;
        }
        shapeList.GetVelocitiesAttr().Set(linearVelocities);

        physxSim->flushChanges();

        numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTActor);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();            
            compare(actor->getLinearVelocity(), omni::physx::toPhysX(linearVelocities[i]), toleranceEpsilon);
        }
    }

    SUBCASE("Angular Velocities")
    {
        angularVelocities.clear();
        shapeList.GetAngularVelocitiesAttr().Get(&angularVelocities);
        // check default value
        CHECK(angularVelocities.size() == size_t(numIndices));

        // dynamic rigid body should be created
        std::vector<void*> ptrs;
        ptrs.resize(size_t(numIndices));
        uint32_t numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTActor);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            compare(actor->getAngularVelocity(), degToRad(angularVelocities[i]), toleranceEpsilon);
        }

        // update velocities
        for (uint32_t i = 0; i < numIndices; i++)
        {
            angularVelocities[i][0] = i * 500.0f + 200.0f;
        }
        shapeList.GetAngularVelocitiesAttr().Set(angularVelocities);

        physxSim->flushChanges();

        numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTActor);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            compare(actor->getAngularVelocity(), degToRad(angularVelocities[i]), toleranceEpsilon);
        }
    }

    SUBCASE("PointInstancer Data")
    {
        PxScene* scene = reinterpret_cast<PxScene*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        CHECK(scene != nullptr);

        PxU32 numActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
        REQUIRE(numActors == numIndices);

        std::vector<PxActor*> actors;
        std::vector<usdparser::ObjectId> ids;
        std::vector<InstancedData> instancedData;
        actors.resize(numIndices);
        ids.resize(numIndices);
        instancedData.resize(numIndices);
        scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, actors.data(), numIndices);
        for (PxU32 i = 0; i < numActors; i++)
        {
            ids[i] = (usdparser::ObjectId)actors[i]->userData;
        }
        physxPriv->getRigidBodyInstancedData(ids.data(), numIndices, instancedData.data());
        for (PxU32 i = 0; i < numActors; i++)
        {
            const InstancedData& data = instancedData[i];
            const SdfPath instancerPath = intToSdfPath(data.instancerPath);
            CHECK(instancerPath == shapeList.GetPath());
            CHECK(data.instanceIndex == i);
        }


    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("RigidBody Enable",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    REQUIRE(pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit)));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body - start as static
    addRigidBox(stage, "/World/box0", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath box0Path = SdfPath("/World/box0");
    UsdPrim box0Prim = stage->GetPrimAtPath(box0Path);

    UsdPhysicsRigidBodyAPI usdPhysicsRigidBodyAPI = UsdPhysicsRigidBodyAPI::Get(stage, box0Path);
    usdPhysicsRigidBodyAPI.GetRigidBodyEnabledAttr().Set(false);

    // create rigid body -starts as dynamic
    addRigidBox(stage, "/World/box1", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath box1Path = SdfPath("/World/box1");
    UsdPrim box1Prim = stage->GetPrimAtPath(box1Path);

    usdPhysicsRigidBodyAPI = UsdPhysicsRigidBodyAPI::Get(stage, box1Path);
    usdPhysicsRigidBodyAPI.GetRigidBodyEnabledAttr().Set(true);

    // create a joint between
    const SdfPath jointPath = SdfPath("/World/fixedJoint");
    UsdPhysicsFixedJoint joint = UsdPhysicsFixedJoint::Define(stage, jointPath);
    joint.GetBody0Rel().AddTarget(box0Path);
    joint.GetBody1Rel().AddTarget(box1Path);

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Parse Test")
    {
        PxBase* base0Ptr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box0Path, ePTActor));
        CHECK(base0Ptr != nullptr);
        CHECK(base0Ptr->is<PxRigidStatic>());

        PxBase* base1Ptr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box1Path, ePTActor));
        CHECK(base1Ptr != nullptr);
        CHECK(base1Ptr->is<PxRigidDynamic>());

        PxBase* jointPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(jointPtr != nullptr);
        CHECK(jointPtr->is<PxD6Joint>());

        PxD6Joint* fixedJoint = jointPtr->is<PxD6Joint>();
        PxRigidActor* actor0 = nullptr;
        PxRigidActor* actor1 = nullptr;

        fixedJoint->getActors(actor0, actor1);
        CHECK(actor0 == base0Ptr);
        CHECK(actor1 == base1Ptr);
    }

    SUBCASE("RigidBody Enable")
    {
        usdPhysicsRigidBodyAPI = UsdPhysicsRigidBodyAPI::Get(stage, box0Path);
        usdPhysicsRigidBodyAPI.GetRigidBodyEnabledAttr().Set(true);

        physxSim->simulate(0.02f, 0.0f);
        physxSim->fetchResults();

        PxBase* base0Ptr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box0Path, ePTActor));
        CHECK(base0Ptr != nullptr);
        CHECK(base0Ptr->is<PxRigidDynamic>());

        PxBase* base1Ptr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box1Path, ePTActor));
        CHECK(base1Ptr != nullptr);
        CHECK(base1Ptr->is<PxRigidDynamic>());

        PxBase* jointPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(jointPtr != nullptr);
        CHECK(jointPtr->is<PxD6Joint>());

        PxD6Joint* fixedJoint = jointPtr->is<PxD6Joint>();
        PxRigidActor* actor0 = nullptr;
        PxRigidActor* actor1 = nullptr;

        fixedJoint->getActors(actor0, actor1);
        CHECK(actor0 == base0Ptr);
        CHECK(actor1 == base1Ptr);
    }

    SUBCASE("RigidBody Enable Body0 Disable Body1")
    {
        usdPhysicsRigidBodyAPI = UsdPhysicsRigidBodyAPI::Get(stage, box0Path);
        usdPhysicsRigidBodyAPI.GetRigidBodyEnabledAttr().Set(true);

        usdPhysicsRigidBodyAPI = UsdPhysicsRigidBodyAPI::Get(stage, box1Path);
        usdPhysicsRigidBodyAPI.GetRigidBodyEnabledAttr().Set(false);

        physxSim->simulate(0.02f, 0.0f);
        physxSim->fetchResults();

        PxBase* base0Ptr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box0Path, ePTActor));
        CHECK(base0Ptr != nullptr);
        CHECK(base0Ptr->is<PxRigidDynamic>());

        PxBase* base1Ptr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box1Path, ePTActor));
        CHECK(base1Ptr != nullptr);
        CHECK(base1Ptr->is<PxRigidStatic>());

        PxBase* jointPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(jointPtr != nullptr);
        CHECK(jointPtr->is<PxD6Joint>());

        PxD6Joint* fixedJoint = jointPtr->is<PxD6Joint>();
        PxRigidActor* actor0 = nullptr;
        PxRigidActor* actor1 = nullptr;

        fixedJoint->getActors(actor0, actor1);
        CHECK(actor0 == base0Ptr);
        CHECK(actor1 == base1Ptr);
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("RigidBody Sleeping",
          "[omniphysics]"
          "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    REQUIRE(pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit)));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body
    addRigidBox(stage, "/World/box0", GfVec3f(100.f), GfVec3f(0.0f, 0.0f, 200.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    // create ground plane
    addGroundPlane(stage, "/World/plane");

    const SdfPath box0Path = SdfPath("/World/box0");
    UsdPrim box0Prim = stage->GetPrimAtPath(box0Path);

    carb::settings::ISettings* settings =
        physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();

    SUBCASE("Sleeping On")
    {        
        settings->setBool(kSettingDisableSleeping, false);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box0Path, ePTActor));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxRigidDynamic>());
        CHECK(basePtr->is<PxRigidDynamic>()->isSleeping() == false);

        for (int i = 0; i < 100; i++)
        {
            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();
        }

        CHECK(basePtr->is<PxRigidDynamic>()->isSleeping() == true);
    }

    SUBCASE("Sleeping Off")
    {
        settings->setBool(kSettingDisableSleeping, true);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box0Path, ePTActor));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxRigidDynamic>());
        CHECK(basePtr->is<PxRigidDynamic>()->isSleeping() == false);

        for (int i = 0; i < 100; i++)
        {
            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();
        }

        CHECK(basePtr->is<PxRigidDynamic>()->isSleeping() == false);
    }

    // set back default
    settings->setBool(kSettingDisableSleeping, false);

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("RigidBody Simulation Owner",
          "[omniphysics]"
          "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    REQUIRE(pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit)));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScene0Path = defaultPrimPath.AppendChild(TfToken("physicsScene0"));
    UsdPhysicsScene::Define(stage, physicsScene0Path);
    const SdfPath physicsScene1Path = defaultPrimPath.AppendChild(TfToken("physicsScene1"));
    UsdPhysicsScene::Define(stage, physicsScene1Path);
    const SdfPath physicsScene2Path = defaultPrimPath.AppendChild(TfToken("physicsScene2"));
    UsdPhysicsScene::Define(stage, physicsScene2Path);
    const SdfPath physicsScene3Path = defaultPrimPath.AppendChild(TfToken("physicsScene3"));
    UsdPhysicsScene::Define(stage, physicsScene3Path);
    const SdfPath physicsScene4Path = defaultPrimPath.AppendChild(TfToken("physicsScene4"));
    UsdPhysicsScene::Define(stage, physicsScene4Path);

    // create rigid body
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f, 0.0f, 200.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    // create ground plane
    addGroundPlane(stage, "/World/plane");

    const SdfPath boxPath = SdfPath("/World/box");

    SUBCASE("Simulation Owner Changes")
    {        
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        PxBase* boxActorPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        CHECK(boxActorPtr != nullptr);
        REQUIRE(boxActorPtr->is<PxRigidDynamic>());

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        PxBase* scenePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene0Path, ePTScene));
        REQUIRE(scenePtr);
        PxScene* scene0 = (PxScene*)scenePtr;

        scenePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene1Path, ePTScene));
        REQUIRE(scenePtr);
        PxScene* scene1 = (PxScene*)scenePtr;

        scenePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene2Path, ePTScene));
        REQUIRE(scenePtr);
        PxScene* scene2 = (PxScene*)scenePtr;

        scenePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene3Path, ePTScene));
        REQUIRE(scenePtr);
        PxScene* scene3 = (PxScene*)scenePtr;

        scenePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene4Path, ePTScene));
        REQUIRE(scenePtr);
        PxScene* scene4 = (PxScene*)scenePtr;

        CHECK(scene0->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
        CHECK(scene1->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
        CHECK(scene2->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
        CHECK(scene3->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
        CHECK(scene4->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);

        UsdPhysicsRigidBodyAPI rboAPI = UsdPhysicsRigidBodyAPI::Get(stage, boxPath);
        rboAPI.GetSimulationOwnerRel().AddTarget(physicsScene1Path);

        {
            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();

            CHECK(scene0->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            REQUIRE(scene1->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            CHECK(scene2->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            CHECK(scene3->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            CHECK(scene4->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            PxActor* actor = nullptr;
            scene1->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            CHECK(actor == boxActorPtr);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK(!(actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));
        }

        rboAPI.GetSimulationOwnerRel().AddTarget(physicsScene2Path);
        {
            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();

            CHECK(scene0->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            REQUIRE(scene1->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            REQUIRE(scene2->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            CHECK(scene3->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            CHECK(scene4->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            PxActor* actor = nullptr;
            scene1->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            CHECK(actor == boxActorPtr);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK(!(actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));

            scene2->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK((actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));
        }
        
        rboAPI.GetSimulationOwnerRel().SetTargets({ physicsScene4Path, physicsScene1Path, physicsScene2Path,
            physicsScene3Path, physicsScene0Path });
        {
            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();

            REQUIRE(scene0->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            REQUIRE(scene1->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            REQUIRE(scene2->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            REQUIRE(scene3->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            REQUIRE(scene4->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            PxActor* actor = nullptr;
            scene4->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            CHECK(actor == boxActorPtr);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK(!(actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));

            scene0->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK((actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));

            scene1->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK((actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));

            scene2->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK((actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));

            scene3->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK((actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));
        }

        rboAPI.GetSimulationOwnerRel().SetTargets({ physicsScene1Path, physicsScene2Path });
        {
            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();

            CHECK(scene0->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            REQUIRE(scene1->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            REQUIRE(scene2->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            CHECK(scene3->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            CHECK(scene4->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            PxActor* actor = nullptr;
            scene1->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            CHECK(actor == boxActorPtr);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK(!(actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));

            scene2->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK((actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));
        }

        rboAPI.GetSimulationOwnerRel().ClearTargets(false);
        {
            physxSim->simulate(0.017f, 0.0f);
            physxSim->fetchResults();

            REQUIRE(scene0->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 1);
            CHECK(scene1->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            CHECK(scene2->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            CHECK(scene3->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            CHECK(scene4->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            PxActor* actor = nullptr;
            scene0->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1);
            CHECK(actor == boxActorPtr);
            REQUIRE(actor->is<PxRigidDynamic>());
            CHECK(!(actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC));
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}


TEST_CASE("RigidBody PointInstancer Scale",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    const float toleranceEpsilon = 1e-4f;

    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxUnitTests* physxUT = physicsTests.acquirePhysxUnitTestInterface();
    REQUIRE(physxUT);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    REQUIRE(pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit)));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    const SdfPath geomPointInstancerPath = defaultPrimPath.AppendChild(TfToken("pointInstancer"));
    const SdfPath boxActorPath = geomPointInstancerPath.AppendChild(TfToken("boxActor"));

    UsdGeomCube cubeGeom = UsdGeomCube::Define(stage, boxActorPath);
    cubeGeom.CreateSizeAttr().Set(1.0);

    const uint32_t numIndices = 2;
    VtArray<int> meshIndices = { 0, 0 };
    VtArray<GfVec3f> positions = { GfVec3f(-125.0, 0.0, 500.0), GfVec3f(-125.0, 0.0, 500.0) };
    VtArray<GfVec3f> scales = { GfVec3f(1.0, 2.0, 3.0), GfVec3f(4.0, 5.0, 6.0) };
    VtArray<GfQuath> orientations = { GfQuath(1.0), GfQuath(1.0) };
    VtArray<GfVec3f> linearVelocities = { GfVec3f(0.0), GfVec3f(0.0, 0.0, 0.0) };
    VtArray<GfVec3f> angularVelocities = { GfVec3f(0.0, 10.0, 0.0), GfVec3f(0.0) };

    UsdGeomPointInstancer shapeList = UsdGeomPointInstancer::Define(stage, geomPointInstancerPath);
    shapeList.GetPrototypesRel().AddTarget(boxActorPath);
    
    shapeList.GetProtoIndicesAttr().Set(meshIndices);
    shapeList.GetPositionsAttr().Set(positions);
    shapeList.GetOrientationsAttr().Set(orientations);
    shapeList.GetScalesAttr().Set(scales);
    shapeList.GetVelocitiesAttr().Set(linearVelocities);
    shapeList.GetAngularVelocitiesAttr().Set(angularVelocities);

    SUBCASE("Dynamic RigidBody Prototype")
    {
        UsdPhysicsCollisionAPI::Apply(cubeGeom.GetPrim());
        UsdPhysicsRigidBodyAPI::Apply(cubeGeom.GetPrim());

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        SUBCASE("Scales")
        {
            // dynamic rigid body should be created
            std::vector<void*> ptrs;
            ptrs.resize(size_t(numIndices));
            uint32_t numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTActor);
            CHECK(numPtrs == numIndices);
            for (uint32_t i = 0; i < numIndices; i++)
            {
                PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
                REQUIRE(basePtr != nullptr);
                REQUIRE(basePtr->is<PxRigidDynamic>());
                PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
                REQUIRE(actor->getNbShapes() == 1);
                PxShape* shape = nullptr;
                actor->getShapes(&shape, 1);
                REQUIRE(shape);
                const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(shape->getGeometry());
                const PxVec3 extent = boxGeom.halfExtents * 2.0f;
                compare(extent, scales[i], toleranceEpsilon);
            }
        }
    }

    SUBCASE("Static RigidBody Prototype")
    {
        UsdPhysicsCollisionAPI::Apply(cubeGeom.GetPrim());

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        SUBCASE("Scales")
        {
            // dynamic rigid body should be created
            std::vector<void*> ptrs;
            ptrs.resize(size_t(numIndices));
            uint32_t numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTActor);
            CHECK(numPtrs == numIndices);
            for (uint32_t i = 0; i < numIndices; i++)
            {
                PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
                REQUIRE(basePtr != nullptr);
                REQUIRE(basePtr->is<PxRigidStatic>());
                PxRigidStatic* actor = basePtr->is<PxRigidStatic>();
                REQUIRE(actor->getNbShapes() == 1);
                PxShape* shape = nullptr;
                actor->getShapes(&shape, 1);
                REQUIRE(shape);
                const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(shape->getGeometry());
                const PxVec3 extent = boxGeom.halfExtents * 2.0f;
                compare(extent, scales[i], toleranceEpsilon);
            }
        }
    }

    SUBCASE("Dynamic RigidBody Instancer")
    {
        UsdPhysicsCollisionAPI::Apply(cubeGeom.GetPrim());
        UsdPhysicsRigidBodyAPI::Apply(shapeList.GetPrim());

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        SUBCASE("Scales")
        {
            // dynamic rigid body should be created
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(geomPointInstancerPath, ePTActor));
            REQUIRE(basePtr);
            REQUIRE(basePtr->is<PxRigidDynamic>());
            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            REQUIRE(actor->getNbShapes() == 2);
            for (int i = 0; i < 2; i++)
            {
                PxShape* shape = nullptr;
                actor->getShapes(&shape, 1, i);
                REQUIRE(shape);
                const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(shape->getGeometry());
                const PxVec3 extent = boxGeom.halfExtents * 2.0f;
                compare(extent, scales[i], toleranceEpsilon);
            }
        }
    }

    SUBCASE("RevoluteJointLimitAt180Deg")
    {
        std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "RevoluteJoint180DegLimit.usd";

        UsdStageRefPtr stage = UsdStage::Open(usdFileName);
        REQUIRE(stage);

        pxr::UsdUtilsStageCache::Get().Insert(stage);
        long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

        physxSim->attachStage(stageId);


        const SdfPath box1Path = defaultPrimPath.AppendChild(TfToken("box1"));
        
        // run the sim
        for (int i = 0; i < 100; ++i)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();

            PxRigidDynamic* box0 = getPhysxBaseDerivedFromPathChecked<PxRigidDynamic>(box1Path, PhysXType::ePTActor);
            PxTransform transform = box0->getGlobalPose();

            REQUIRE(transform.p.y > -0.1f);

            //printf("p.y: %f\n", transform.p.y);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

// PT: this used to crash in the GPU island manager. The repro removes an articulation stored in the stage as
// a payload, and then re-adds the payload immediately. We didn't pinpoint what exactly triggers the crash, this
// is the minimal repro we could come up with (down from a larger USD scene and more ops, the initial repro also
// involved creating fixed joints at runtime, etc).
TEST_CASE("OM_115801_Repro",
    "[omniphysics]"
    "[component=OmniPhysics][owner=pterdiman][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxUnitTests* physxUT = physicsTests.acquirePhysxUnitTestInterface();
    REQUIRE(physxUT);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "franka_shadow_repro.usd";

    UsdStageRefPtr stage = UsdStage::Open(usdFileName);
    REQUIRE(stage);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    physxSim->attachStage(stageId);

    const SdfPath attach_path = SdfPath("/World/shadow_hand");

    UsdPrim prim = stage->GetPrimAtPath(attach_path);

    physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
    physxSim->fetchResults();

    const SdfPrimSpecHandleVector primStack = prim.GetPrimStack();
    if(primStack.size())
    {
        const SdfPrimSpec& ps = primStack[0].GetSpec();
        if(ps.HasPayloads())
        {
            const SdfPayloadsProxy rp = ps.GetPayloadList();
            const uint32_t nbItems = uint32_t(rp.GetAddedOrExplicitItems().size());
            for(uint32_t j=0;j<nbItems;j++)
            {
                const SdfPayload pl = rp.GetAddedOrExplicitItems()[j];

                UsdPayloads payloads = prim.GetPayloads();

                payloads.RemovePayload(pl);
                payloads.AddPayload(pl);
            }
        }
    }

    physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
    physxSim->fetchResults();

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

////Test passes if it does not crash
//TEST_CASE("DeformableSurfaceCrash",
//    "[omniphysics]"
//    "[component=OmniPhysics][owner=twidmer][priority=mandatory]")
//{
//    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
//    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
//    REQUIRE(physxSim);
//
//    std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "CustomGeoVsFemCloth.usd";
//
//    UsdStageRefPtr stage = UsdStage::Open(usdFileName);
//    REQUIRE(stage);
//
//    pxr::UsdUtilsStageCache::Get().Insert(stage);
//    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
//
//    physxSim->attachStage(stageId);
//
//    // run the sim
//    for (int i = 0; i < 20; ++i)
//    {
//        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
//        physxSim->fetchResults();
//        printf("iteration %i\n", i);
//    }
//}
